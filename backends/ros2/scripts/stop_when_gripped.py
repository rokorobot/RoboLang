"""Test harness only: write "cancel" to PATH once the simulated gripper has closed (decision 0038 P7 stop case).

Usage: stop_when_gripped.py PATH. Reads /joint_states; exits after writing.
"""
import os
import sys
import time

import rclpy
from sensor_msgs.msg import JointState

CLOSED_RAD = 0.75  # as the oracle

rclpy.init()
node = rclpy.create_node("h3_stop_when_gripped")
state = {}
node.create_subscription(JointState, "/joint_states", lambda m: state.update(zip(m.name, m.position)), 10)
deadline = time.monotonic() + 600
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
    if state.get("robotiq_85_left_knuckle_joint", 0.0) >= CLOSED_RAD:
        with open(sys.argv[1], "w", encoding="utf-8") as stop:
            stop.write("cancel\n")
        print(f"cancel written at knuckle {state['robotiq_85_left_knuckle_joint']:.3f}", flush=True)
        break
os._exit(0)  # rclpy teardown can crash; nothing is left to do
