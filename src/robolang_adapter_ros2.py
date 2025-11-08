# src/robolang_adapter_ros2.py

from __future__ import annotations
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand
from std_msgs.msg import String

# Optional custom services – adapt to your own definitions
# If you don't have them yet, you can comment these out and stub the methods.
try:
    from my_msgs.srv import RegionStatus, SensorStatus
except ImportError:
    RegionStatus = None
    SensorStatus = None


class RobotAdapterROS2(Node):
    """
    ROS2-based adapter for RoboLang.

    This replaces the simulated adapter. It maps:

      move_to_pose -> FollowJointTrajectory
      set_gripper  -> GripperCommand
      inspect      -> SensorStatus / your own service
      communicate  -> std_msgs/String publish
      wait_for     -> time.sleep() + rclpy.spin_once()

    You can extend this with your specific robot's APIs.
    """

    def __init__(
        self,
        robot_name: str,
        move_action_name: str = "/arm_controller/follow_joint_trajectory",
        gripper_action_name: str = "/gripper_controller/gripper_action",
        comm_topic_prefix: str = "/robot_comm",
        region_service_name: Optional[str] = "/region_clear",
        sensor_service_name: Optional[str] = "/sensor_status",
    ):
        super().__init__(f"{robot_name}_adapter")
        self.robot_name = robot_name

        # Action clients
        self._move_client = ActionClient(self, FollowJointTrajectory, move_action_name)
        self._gripper_client = ActionClient(self, GripperCommand, gripper_action_name)

        # Services (optional)
        self._region_client = (
            self.create_client(RegionStatus, region_service_name)
            if RegionStatus and region_service_name
            else None
        )
        self._sensor_client = (
            self.create_client(SensorStatus, sensor_service_name)
            if SensorStatus and sensor_service_name
            else None
        )

        # Communication publisher
        self._comm_pub = self.create_publisher(String, comm_topic_prefix, 10)

        self.get_logger().info("RobotAdapterROS2 initialized.")

    # ---------- helpers ----------

    def wait_for_servers(self, timeout_sec: float = 5.0):
        self.get_logger().info("Waiting for ROS2 action/service servers...")

        if not self._move_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("Move action server not available!")
        if not self._gripper_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("Gripper action server not available!")

        if self._region_client and not self._region_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn("RegionStatus service not available (optional).")
        if self._sensor_client and not self._sensor_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().warn("SensorStatus service not available (optional).")

        self.get_logger().info("Servers ready (or skipped).")

    def _send_goal_and_wait(self, client: ActionClient, goal_msg):
        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Action goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f"Action result: {result}")
        return True

    # ---------- RoboLang primitives ----------

    def move_to_pose(
        self,
        joint_names: List[str],
        joint_positions: List[float],
        duration: float = 2.0,
    ):
        """
        Map RoboLang 'move' to FollowJointTrajectory.

        In a real system, you would probably compute joint targets from a pose
        using MoveIt2. Here we assume you already have joint values.
        """
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        traj.points.append(point)
        goal.trajectory = traj

        self.get_logger().info(f"[MOVE] {self.robot_name} → {joint_positions}")
        return self._send_goal_and_wait(self._move_client, goal)

    def set_gripper(self, position: float, max_effort: float = 50.0):
        """
        position: abstract 0.0 (open) to 1.0 (closed) scale.
        Map this to your gripper command units as appropriate.
        """
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort

        self.get_logger().info(f"[GRIPPER] {self.robot_name} → pos={position}")
        return self._send_goal_and_wait(self._gripper_client, goal)

    def inspect(self, target_id: str, sensor_name: str) -> bool:
        """
        Example mapping to a SensorStatus service.
        You can replace this with your own inspection pipeline.
        """
        if not self._sensor_client:
            self.get_logger().warn("inspect() called but SensorStatus client is not configured.")
            return False

        req = SensorStatus.Request()
        req.sensor_name = sensor_name
        req.target_id = target_id

        self.get_logger().info(f"[INSPECT] {target_id} via {sensor_name}")
        future = self._sensor_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            self.get_logger().error("SensorStatus call failed (no result).")
            return False

        self.get_logger().info(f"SensorStatus ok={res.ok}")
        return res.ok

    def wait_for(self, seconds: float):
        """
        Simple blocking wait integrated with rclpy spin.
        """
        self.get_logger().info(f"[WAIT] {seconds:.2f}s")
        end_time = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_until_region_clear(self, region_id: str, poll_interval: float = 0.5):
        """
        Example implementation that polls a RegionStatus service.
        """
        if not self._region_client:
            self.get_logger().warn("wait_until_region_clear() called but RegionStatus client is not configured.")
            return True

        self.get_logger().info(f"[WAIT-REGION] '{region_id}'")

        while rclpy.ok():
            req = RegionStatus.Request()
            req.region_id = region_id
            future = self._region_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            if res and getattr(res, "clear", False):
                self.get_logger().info(f"Region '{region_id}' is clear.")
                return True

            self.get_logger().info(f"Region '{region_id}' not clear, retrying...")
            rclpy.spin_once(self, timeout_sec=poll_interval)

    def communicate(self, channel: str, message: str):
        """
        Publish a String on a communication bus topic.
        """
        msg = String()
        msg.data = f"{self.robot_name}@{channel}: {message}"
        self.get_logger().info(f"[COMM] {msg.data}")
        self._comm_pub.publish(msg)

    # ---------- "precondition" helpers ----------

    def check_region_clear(self, region_id: str) -> bool:
        if not self._region_client:
            self.get_logger().warn("check_region_clear() without RegionStatus client; assuming clear.")
            return True

        req = RegionStatus.Request()
        req.region_id = region_id
        future = self._region_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            self.get_logger().error("RegionStatus call failed (no result).")
            return False

        self.get_logger().info(f"[CHECK] region_clear({region_id})={res.clear}")
        return res.clear

    def check_sensor_ok(self, sensor_name: str) -> bool:
        if not self._sensor_client:
            self.get_logger().warn("check_sensor_ok() without SensorStatus client; assuming OK.")
            return True

        req = SensorStatus.Request()
        req.sensor_name = sensor_name
        future = self._sensor_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            self.get_logger().error("SensorStatus call failed (no result).")
            return False

        self.get_logger().info(f"[CHECK] sensor_ok({sensor_name})={res.ok}")
        return res.ok
