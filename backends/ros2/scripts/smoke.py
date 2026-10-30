"""H3 phase 1 smoke test: one arm motion and one camera frame, on simulation time. Run inside the container.

Exit code: 0 when the arm reached the target; 1 otherwise. The camera result is reported in the JSON either way
(decision 0029 section 5: software rendering is measured here, and a failure there is a documented fallback).
"""
import argparse
import json
import sys
import time

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TARGET = [0.5, -1.2, 1.2, -1.57, -1.57, 0.0]
ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"


class Smoke(Node):
    def __init__(self):
        super().__init__("h3_smoke", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.t0 = time.monotonic()
        self.marks = {}
        self.clock_msgs = 0
        self.joints = {}
        self.image = None
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(Image, "/overhead_camera/image", self._on_image, qos_profile_sensor_data)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)

    def mark(self, name):
        self.marks.setdefault(name, round(time.monotonic() - self.t0, 2))

    def _on_clock(self, _):
        self.clock_msgs += 1
        self.mark("first_clock_s")

    def _on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))
        self.mark("first_joint_states_s")

    def _on_image(self, msg):
        if self.image is None:
            self.image = msg
            self.mark("first_camera_frame_s")

    def spin_until(self, cond, timeout):
        end = time.monotonic() + timeout
        while not cond() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return cond()


def check_camera(msg, png_path):
    import cv2

    img = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    channels = len(img) // (msg.width * msg.height) if msg.width * msg.height else 0
    result = {"width": msg.width, "height": msg.height, "encoding": msg.encoding, "bytes": len(img)}
    if channels not in (1, 3, 4):
        return dict(result, non_empty=False, reason="unexpected data length")
    img = img.reshape(msg.height, msg.width, channels)
    result.update(mean=float(img.mean()), std=float(img.std()))
    # Empty means no data or one flat colour (a black frame is what a broken software renderer returns).
    result["non_empty"] = bool(img.size and img.std() > 2.0)
    if channels == 1:
        bgr = img
    else:
        bgr = np.ascontiguousarray(img[:, :, :3][:, :, ::-1])  # RGB(A) -> BGR
    cv2.imwrite(png_path, bgr)
    try:
        detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        _, ids, _ = detector.detectMarkers(bgr)
        result["aruco_ids_detected"] = [] if ids is None else [int(i) for i in ids.flatten()]
    except Exception as exc:  # detection is informational in phase 1
        result["aruco_error"] = repr(exc)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/out")
    ap.add_argument("--tolerance", type=float, default=0.05, help="rad, per joint")
    ap.add_argument("--startup-timeout", type=float, default=240.0)
    args = ap.parse_args()

    rclpy.init()
    node = Smoke()
    result = {"motion": {"ok": False}, "camera": {"non_empty": False}}

    def finish(code):
        result["timings"] = node.marks
        result["sim_time_only"] = True
        with open(f"{args.out}/smoke.json", "w") as f:
            json.dump(result, f, indent=2)
        print(json.dumps(result, indent=2))
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(code)

    # 1. Simulation clock: the whole graph must run on /clock (decision 0029 X7).
    if not node.spin_until(lambda: node.clock_msgs > 5, args.startup_timeout):
        result["motion"]["error"] = "no /clock from Gazebo"
        finish(1)
    node.mark("gazebo_clock_s")
    # 2. Controller and joint states.
    if not node.spin_until(lambda: node.client.server_is_ready() and len(node.joints) >= 6, args.startup_timeout):
        result["motion"]["error"] = "trajectory controller or joint_states not available"
        finish(1)
    node.mark("controllers_ready_s")

    # 3. One trajectory.
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    when = rclpy.duration.Duration(seconds=6).to_msg()
    goal.trajectory.points = [JointTrajectoryPoint(positions=TARGET, time_from_start=when)]
    t_send = time.monotonic()
    sim_send = node.get_clock().now()
    fut = node.client.send_goal_async(goal)
    node.spin_until(fut.done, 30)
    handle = fut.result()
    if handle is None or not handle.accepted:
        result["motion"]["error"] = "goal rejected or no answer"
        finish(1)
    res_fut = handle.get_result_async()
    node.spin_until(res_fut.done, 120)
    if not res_fut.done():
        result["motion"]["error"] = "trajectory did not finish within 120 s wall time"
        finish(1)
    code = res_fut.result().result.error_code
    node.spin_until(lambda: False, 1.0)  # let /joint_states settle
    errors = [abs(node.joints.get(j, 1e9) - t) for j, t in zip(JOINTS, TARGET)]
    result["motion"] = {
        "ok": code == FollowJointTrajectory.Result.SUCCESSFUL and max(errors) <= args.tolerance,
        "error_code": code,
        "max_joint_error_rad": max(errors),
        "tolerance_rad": args.tolerance,
        "wall_s": round(time.monotonic() - t_send, 2),
        "sim_s": round((node.get_clock().now() - sim_send).nanoseconds / 1e9, 2),
        "final": {j: round(node.joints.get(j, float("nan")), 4) for j in JOINTS},
    }

    # 4. Camera (already subscribed since the start; give it time if the renderer is slow).
    if node.spin_until(lambda: node.image is not None, 120):
        result["camera"] = check_camera(node.image, f"{args.out}/camera.png")
    else:
        result["camera"] = {"non_empty": False, "reason": "no frame within 120 s"}
    finish(0 if result["motion"]["ok"] else 1)


if __name__ == "__main__":
    main()
