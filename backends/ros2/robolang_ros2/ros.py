"""ROS 2 / MoveIt / Gazebo operations. The graph is checked before constructing a planner or action client."""
from __future__ import annotations

import subprocess
import time

from .executor import BackendFailure


class ROSOperations:
    def __init__(self):
        import rclpy
        from rosgraph_msgs.msg import Clock

        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("robolang_ros2_guard", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.clock_values = []
        self.node.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.planner = None
        self.arm = None
        self.gripper = None
        self.model = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.node.destroy_node()
        self.rclpy.shutdown()

    def _on_clock(self, msg):
        value = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if not self.clock_values or value > self.clock_values[-1]:
            self.clock_values.append(value)

    def _until(self, predicate, seconds: float):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        return predicate()

    @staticmethod
    def _command(args: list[str], timeout: int = 20) -> str:
        try:
            done = subprocess.run(args, check=True, capture_output=True, text=True, timeout=timeout)
            return done.stdout
        except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired) as exc:
            raise BackendFailure("S501", f"simulation graph check failed: {args[0]}: {exc}") from exc

    def guard(self, model: str) -> None:
        if self.node.get_parameter("use_sim_time").value is not True:
            raise BackendFailure("S501", "executor is not using simulation time")
        if not self._until(lambda: len(self.clock_values) >= 5, 30):
            raise BackendFailure("S501", "no advancing Gazebo /clock")
        listing = self._command(["gz", "model", "--list"])
        if f"- {model}" not in listing:
            raise BackendFailure("S501", f"declared Gazebo model {model!r} is absent")
        hardware = self._command(["ros2", "control", "list_hardware_components", "-c", "/controller_manager"])
        if "gz_ros2_control/GazeboSimSystem" not in hardware:
            raise BackendFailure("S501", "controller manager does not report GazeboSimSystem hardware")
        self.model = model
        # MoveItPy can contact controllers while loading. Construct it only after every graph check passes.
        try:
            from moveit.planning import MoveItPy
            self.planner = MoveItPy(node_name="robolang_moveit")
            self.arm = self.planner.get_planning_component("ur_manipulator")
        except Exception as exc:
            raise BackendFailure("S502", f"MoveIt did not start: {exc}") from exc

    def now_ms(self) -> int:
        return self.clock_values[-1] // 1_000_000

    def move(self, pose: dict) -> None:
        from geometry_msgs.msg import PoseStamped

        target = PoseStamped()
        target.header.frame_id = pose["frame"]
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = pose["position"]
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = pose["orientation"]
        try:
            self.arm.set_start_state_to_current_state()
            if not self.arm.set_goal_state(pose_stamped_msg=target, pose_link="tool0"):
                raise BackendFailure("S502", "MoveIt rejected the pose goal")
            plan = self.arm.plan()
            if not plan:
                raise BackendFailure("S502", "MoveIt found no motion plan")
            status = self.planner.execute(plan.trajectory, controllers=["joint_trajectory_controller"])
            if status.status != "SUCCEEDED":
                code = "S504" if status.status == "TIMED_OUT" else "S503"
                raise BackendFailure(code, f"MoveIt execution ended {status.status}")
        except BackendFailure:
            raise
        except Exception as exc:
            raise BackendFailure("S503", f"MoveIt motion failed: {exc}") from exc

    def grip(self, closed: bool) -> None:
        from control_msgs.action import GripperCommand
        from rclpy.action import ActionClient

        if self.gripper is None:
            self.gripper = ActionClient(self.node, GripperCommand, "/gripper_controller/gripper_cmd")
        if not self._until(self.gripper.server_is_ready, 20):
            raise BackendFailure("S505", "gripper controller unavailable")
        goal = GripperCommand.Goal()
        goal.command.position = 0.8 if closed else 0.0
        goal.command.max_effort = 40.0
        future = self.gripper.send_goal_async(goal)
        if not self._until(future.done, 10):
            raise BackendFailure("S504", "gripper goal timed out before acceptance")
        handle = future.result()
        if handle is None or not handle.accepted:
            raise BackendFailure("S505", "gripper goal rejected")
        result = handle.get_result_async()
        if not self._until(result.done, 20):
            handle.cancel_goal_async()
            raise BackendFailure("S504", "gripper action timed out")
        if not result.result().result.reached_goal:
            raise BackendFailure("S505", "gripper did not reach commanded position")

    def attach(self, model: str) -> None:
        self._joint(model, "attach")

    def detach(self, model: str) -> None:
        self._joint(model, "detach")

    def _joint(self, model: str, action: str) -> None:
        if model != "box" or self.model != "ur":
            raise BackendFailure("S506", "this harness declares only the ur/box detachable joint")
        topic = f"/model/{self.model}/detachable_joint/{action}"
        try:
            subprocess.run(["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", ""],
                           check=True, capture_output=True, text=True, timeout=10)
        except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired) as exc:
            raise BackendFailure("S506", f"Gazebo {action} command failed: {exc}") from exc

    def wait_ms(self, duration: int) -> None:
        target = self.now_ms() + duration
        if not self._until(lambda: self.now_ms() >= target, max(10, duration / 1000 * 3)):
            raise BackendFailure("S504", "simulation clock stopped during wait")

    def versions(self) -> dict[str, str]:
        packages = {"ros": "lyrical"}
        for package in ("ros-lyrical-moveit", "ros-lyrical-gz-ros2-control", "ros-lyrical-ur-simulation-gz"):
            try:
                result = subprocess.run(["dpkg-query", "-W", "-f=${Version}", package], capture_output=True,
                                        text=True, check=True, timeout=3)
                packages[package] = result.stdout.strip()
            except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
                packages[package] = "unknown"
        return packages
