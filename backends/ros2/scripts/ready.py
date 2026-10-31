"""Harness setup: move the arm from its spawn pose to a raised ready pose above the table, before the executor runs.

The phase 1 smoke test did the same. Joint values checked with UR5e forward kinematics: tool0 at about
(-0.13, 0.39, 0.67) m pointing down, clear of the table and the boxes. Exit 1 if the controller does not reach it.
"""
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
          "wrist_3_joint"]
READY = [1.5708, -1.7, 1.2, -1.0708, -1.5708, 0.0]

rclpy.init()
node = rclpy.create_node("h3_ready", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
joints = {}
node.create_subscription(JointState, "/joint_states", lambda m: joints.update(zip(m.name, m.position)), 10)
client = ActionClient(node, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")


def spin_until(condition, seconds):
    deadline = time.monotonic() + seconds
    while not condition() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return condition()


if not spin_until(lambda: client.server_is_ready() and len(joints) >= 6, 120):
    sys.exit("no trajectory controller or joint states")
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = JOINTS
goal.trajectory.points = [JointTrajectoryPoint(positions=READY, time_from_start=Duration(seconds=6).to_msg())]
future = client.send_goal_async(goal)
spin_until(future.done, 30)
handle = future.result()
if handle is None or not handle.accepted:
    sys.exit("ready goal rejected")
result = handle.get_result_async()
spin_until(result.done, 120)
spin_until(lambda: False, 1.0)  # settle
error = max(abs(joints[j] - q) for j, q in zip(JOINTS, READY))
print(f"ready: max joint error {error:.4f} rad")
sys.exit(0 if result.done() and result.result().result.error_code == 0 and error < 0.02 else 1)
