"""ROS 2 / MoveIt / Gazebo operations. The graph is checked before constructing a planner or action client."""
from __future__ import annotations

import re
import subprocess
import time
import xml.etree.ElementTree as ET

from . import observe
from .executor import BackendFailure

FRAMES = 10  # camera frames fused per observation (10 Hz camera: one simulated second)
SETTLE_MS = 500  # let a released object come to rest before the first fused frame


class ROSOperations:
    def __init__(self, camera: dict):
        import rclpy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from rclpy.qos import DurabilityPolicy, QoSProfile

        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("robolang_ros2_guard", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.clock_values = []
        self.node.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.joints = {}
        self.joint_stamp_ms = None
        self.node.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        from tf2_ros import Buffer, TransformListener
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self.node, spin_thread=False)
        self.frames = None
        self.description = None
        description_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, "/robot_description", self._on_description, description_qos)
        self.planner = None
        self.arm = None
        self.gripper = None
        self.model = None
        self.camera = camera

    def _on_clock(self, msg):
        value = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if not self.clock_values or value > self.clock_values[-1]:
            self.clock_values.append(value)

    def _on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))
        self.joint_stamp_ms = (msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec) // 1_000_000

    def _on_image(self, msg):
        if self.frames is not None:
            self.frames.append(msg)

    def _on_description(self, msg):
        self.description = msg.data

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
        if not re.search(rf"^\s*- {re.escape(model)}\s*$", listing, re.MULTILINE):
            raise BackendFailure("S501", f"declared Gazebo model {model!r} is absent")
        hardware = self._command(["ros2", "control", "list_hardware_components", "-c", "/controller_manager"])
        # Lyrical's ros2controlcli prints an empty plugin name even for active Gazebo systems.
        # Compare the actual component names with the graph's published URDF declarations instead.
        if not self._until(lambda: self.description is not None, 10):
            raise BackendFailure("S501", "no published robot description")
        if not _gazebo_hardware_matches(hardware, self.description, model):
            raise BackendFailure("S501", "controller hardware must match GazeboSimSystem declarations")
        required_topics = {f"/model/{model}/detachable_joint/{suffix}"
                           for suffix in ("attach", "detach", "state")}
        available_topics = set(self._command(["gz", "topic", "-l"]).splitlines())
        if not required_topics <= available_topics:
            raise BackendFailure("S501", "declared Gazebo detachable joint is unavailable")
        # The plugin may start attached, and a one-shot transport publication can be lost.
        # Force a known detached state before any motion. The first detach may be a no-op;
        # the attach and final detach must each be confirmed by a state transition.
        for _ in range(3):
            self._command(["gz", "topic", "-t", f"/model/{model}/detachable_joint/detach",
                           "-m", "gz.msgs.Empty", "-p", ""])
            time.sleep(0.2)
        try:
            gazebo_joint_command(model, "attach")
            gazebo_joint_command(model, "detach")
        except BackendFailure as exc:
            raise BackendFailure("S501", f"could not initialize detached simulation joint: {exc.message}") from exc
        self.model = model
        # MoveItPy can contact controllers while loading. Construct it only after every graph check passes.
        try:
            from control_msgs.action import FollowJointTrajectory
            from moveit.planning import MoveItPy
            from rclpy.action import ActionClient
            parameters = moveit_parameters(self.description)
            self.planner = MoveItPy(node_name="robolang_moveit", config_dict=parameters)
            self.arm = self.planner.get_planning_component("ur_manipulator")
            arm_server = ActionClient(self.node, FollowJointTrajectory,
                                      "/joint_trajectory_controller/follow_joint_trajectory")
            if not self._until(arm_server.server_is_ready, 20):
                raise BackendFailure("S503", "arm trajectory controller unavailable")
            # MoveIt's own action client is created separately and does not wait for DDS discovery.
            # Give that client time to discover the already-active controller before first execution.
            time.sleep(2)
        except BackendFailure:
            raise
        except Exception as exc:
            raise BackendFailure("S502", f"MoveIt did not start: {exc}") from exc

    def now_ms(self) -> int:
        before = self.clock_values[-1]
        self._until(lambda: self.clock_values[-1] > before, 1)
        return self.clock_values[-1] // 1_000_000

    def move(self, pose: dict) -> None:
        from geometry_msgs.msg import PoseStamped

        target = PoseStamped()
        target.header.frame_id = pose["frame"]
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = map(float, pose["position"])
        (target.pose.orientation.x, target.pose.orientation.y,
         target.pose.orientation.z, target.pose.orientation.w) = map(float, pose["orientation"])
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
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from rclpy.duration import Duration
        from trajectory_msgs.msg import JointTrajectoryPoint

        if self.gripper is None:
            self.gripper = ActionClient(self.node, FollowJointTrajectory,
                                        "/gripper_trajectory_controller/follow_joint_trajectory")
        if not self._until(self.gripper.server_is_ready, 20):
            raise BackendFailure("S505", "gripper controller unavailable")
        target = 0.8 if closed else 0.0
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["robotiq_85_left_knuckle_joint"]
        goal.trajectory.points = [JointTrajectoryPoint(positions=[target],
                                                       time_from_start=Duration(seconds=3).to_msg())]
        future = self.gripper.send_goal_async(goal)
        if not self._until(future.done, 10):
            raise BackendFailure("S504", "gripper goal timed out before acceptance")
        handle = future.result()
        if handle is None or not handle.accepted:
            raise BackendFailure("S505", "gripper goal rejected")
        result = handle.get_result_async()
        if not self._until(result.done, 20):
            handle.cancel_goal_async()
            raise BackendFailure("S504", "gripper trajectory timed out")
        if result.result().result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise BackendFailure("S505", "gripper trajectory failed")
        joint = "robotiq_85_left_knuckle_joint"
        if not self._until(lambda: joint in self.joints and abs(self.joints[joint] - target) < 0.05, 3):
            raise BackendFailure("S505", "gripper joint state did not reach target")

    def attach(self, model: str) -> None:
        self._joint(model, "attach")

    def detach(self, model: str) -> None:
        self._joint(model, "detach")

    def _joint(self, model: str, action: str) -> None:
        if model != "box" or self.model != "ur":
            raise BackendFailure("S506", "this harness declares only the ur/box detachable joint")
        gazebo_joint_command(self.model, action)

    def wait_ms(self, duration: int) -> None:
        target = self.now_ms() + duration
        if not self._until(lambda: self.now_ms() >= target, max(10, duration / 1000 * 3)):
            raise BackendFailure("S504", "simulation clock stopped during wait")

    def observe(self, marker_ids: list[int]) -> observe.Snapshot:
        """Fuse FRAMES overhead camera frames, the latest tool0 transform and the gripper joint state."""
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from sensor_msgs.msg import Image

        self.frames = []
        start = self.now_ms()
        subscription = self.node.create_subscription(Image, "/overhead_camera/image", self._on_image,
                                                     qos_profile_sensor_data)
        stamp = lambda m: (m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec) // 1_000_000  # noqa: E731
        try:
            self._until(lambda: sum(stamp(m) >= start + SETTLE_MS for m in self.frames) >= FRAMES, 60)
            frames = [m for m in self.frames if stamp(m) >= start + SETTLE_MS][:FRAMES]
        finally:
            self.node.destroy_subscription(subscription)
            self.frames = None
        found = {i: ([], []) for i in marker_ids}
        for message in frames:
            for marker, point in detect_markers(message, self.camera).items():
                if marker in found:
                    found[marker][0].append(observe.camera_to_base(point, self.camera))
                    found[marker][1].append(stamp(message))
        markers = {}
        for marker, (positions, stamps) in found.items():
            reading = observe.fuse(positions, stamps, FRAMES, "overhead_camera", "aruco_fiducial")
            if reading is not None:
                markers[marker] = reading
        tool = None
        try:
            t = self.tf.lookup_transform("base_link", "tool0", Time())
            tool = observe.Reading("tf", "tf_tool0_from_joint_states",
                                   (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
                                   (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
                                    t.transform.rotation.w), 1.0, stamp(t))
        except Exception:  # no transform yet: robot_at and holding stay unknown
            pass
        joint = "robotiq_85_left_knuckle_joint"
        gripper = None
        if joint in self.joints and self.joint_stamp_ms is not None:
            gripper = observe.Reading("joint_states", "gripper_joint_state", (self.joints[joint],), None, 1.0,
                                      self.joint_stamp_ms)
        return observe.Snapshot(self.now_ms(), markers, tool, gripper)

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


def detect_markers(message, camera: dict) -> dict[int, tuple[float, float, float]]:
    """ArUco 4x4_50 marker centres in the camera's optical frame, from one RGB frame."""
    import cv2
    import numpy as np

    if message.width != camera["width"] or message.height != camera["height"] or message.encoding != "rgb8":
        return {}
    image = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(message.height, message.width, 3)
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return {}
    focal, cx, cy = observe.intrinsics(camera)
    matrix = np.array([[focal, 0, cx], [0, focal, cy], [0, 0, 1]], dtype=np.float64)
    half = camera["marker_length"] / 2
    square = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    result = {}
    for marker, corner in zip(ids.flatten(), corners):
        ok, _, tvec = cv2.solvePnP(square, corner.reshape(4, 2).astype(np.float64), matrix, None,
                                   flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok and int(marker) not in result:
            result[int(marker)] = tuple(float(v) for v in tvec.flatten())
    return result


def gazebo_joint_command(model: str, action: str) -> None:
    """Publish until Gazebo confirms a state transition; transport discovery can lose one-shot sends."""
    if action not in ("attach", "detach"):
        raise ValueError(action)
    topic = f"/model/{model}/detachable_joint/{action}"
    state_topic = f"/model/{model}/detachable_joint/state"
    expected = "attached" if action == "attach" else "detached"
    try:
        subscriber = subprocess.Popen(["gz", "topic", "-e", "-t", state_topic, "-n", "1"],
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    except OSError as exc:
        raise BackendFailure("S506", f"Gazebo state subscription failed: {exc}") from exc
    try:
        time.sleep(0.5)
        for _ in range(5):
            subprocess.run(["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", ""],
                           check=True, capture_output=True, text=True, timeout=10)
            try:
                output, errors = subscriber.communicate(timeout=2)
            except subprocess.TimeoutExpired:
                continue
            if f'data: "{expected}"' in output:
                return
            raise BackendFailure("S506", f"Gazebo {action} reported an unexpected state: {output or errors}")
        raise BackendFailure("S506", f"Gazebo {action} did not confirm {expected} after five requests")
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired) as exc:
        raise BackendFailure("S506", f"Gazebo {action} command failed: {exc}") from exc
    finally:
        if subscriber.poll() is None:
            subscriber.terminate()
            try:
                subscriber.communicate(timeout=2)
            except subprocess.TimeoutExpired:
                subscriber.kill()
                subscriber.communicate()


def moveit_parameters(description: str) -> dict:
    """Use the live URDF and allow only the Robotiq's internal self contacts."""
    from pathlib import Path
    from moveit_configs_utils import MoveItConfigsBuilder

    config = (MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
              .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur"})
              .to_moveit_configs())
    parameters = config.to_dict()
    parameters["robot_description"] = description
    semantic = ET.fromstring(parameters["robot_description_semantic"])
    links = sorted(link.attrib["name"] for link in ET.fromstring(description).findall("link")
                   if link.attrib["name"].startswith("robotiq_85_"))
    for index, first in enumerate(links):
        for second in links[index + 1:]:
            ET.SubElement(semantic, "disable_collisions", link1=first, link2=second, reason="Adjacent")
    parameters["robot_description_semantic"] = ET.tostring(semantic, encoding="unicode")
    parameters["planning_pipelines"] = {"pipeline_names": ["ompl"]}
    parameters["plan_request_params"] = {"planning_attempts": 4, "planning_pipeline": "ompl",
                                         "planning_time": 15.0}
    parameters["use_sim_time"] = True
    return parameters


def _gazebo_hardware_matches(hardware: str, description: str, model: str) -> bool:
    """Require each active controller component to match a Gazebo system in the live URDF."""
    try:
        root = ET.fromstring(description)
        controls = root.findall("ros2_control")
        declared = {control.attrib["name"]: control.findtext("hardware/plugin") for control in controls}
    except (ET.ParseError, KeyError):
        return False
    blocks = re.findall(r"Hardware Component \d+\n(.*?)(?=Hardware Component \d+\n|\Z)",
                        hardware, re.DOTALL)
    names = [re.search(r"^\s*name:\s*(\S+)", block, re.MULTILINE) for block in blocks]
    observed = [match.group(1) for match in names if match]
    return (root.attrib.get("name") == model and len(controls) == len(declared) and
            len(observed) == len(blocks) and len(observed) == len(declared) and
            set(observed) == set(declared) and
            all("label=active" in block for block in blocks) and
            all(plugin == "gz_ros2_control/GazeboSimSystem" for plugin in declared.values()))
