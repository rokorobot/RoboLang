"""
robolang_adapter.py

Simple simulated adapter for RoboLang v1.

This version does NOT require ROS2 – it just prints actions.
Later you can swap this out for the real ROS2-based RobotAdapter.
"""

from __future__ import annotations
import time
from dataclasses import dataclass


@dataclass
class RobotState:
    name: str
    last_pose: list[float] | None = None
    holding: str | None = None


class RobotAdapter:
    """
    Minimal "simulated" adapter for RoboLang.

    It implements the same methods you would later map to ROS2:
    - move_to_pose
    - set_gripper
    - inspect
    - wait_for
    - wait_until_region_clear
    - communicate
    - check_region_clear
    - check_sensor_ok
    """

    def __init__(self, robot_name: str = "robot_arm_1"):
        self.state = RobotState(name=robot_name)

    # -------- core primitives --------

    def move_to_pose(self, joint_names: list[str] | None, joint_positions: list[float], duration: float = 2.0):
        """Simulate a move by printing the target pose."""
        self.state.last_pose = joint_positions
        print(f"[MOVE] {self.state.name} → joints={joint_positions} (duration={duration}s)")

    def set_gripper(self, position: float, max_effort: float = 50.0):
        """
        position: 0.0 = open, 1.0 = closed.
        In simulation we just log it.
        """
        state = "CLOSE" if position > 0.5 else "OPEN"
        print(f"[GRIPPER] {self.state.name} → {state} (position={position}, max_effort={max_effort})")

    def inspect(self, target_id: str, sensor_name: str) -> bool:
        """Simulate inspection."""
        print(f"[INSPECT] {self.state.name} → target={target_id} via sensor={sensor_name}")
        return True

    def wait_for(self, seconds: float):
        """Block for the given amount of time."""
        print(f"[WAIT] {self.state.name} → {seconds:.2f}s")
        time.sleep(min(seconds, 2.0))  # cap for demo

    def wait_until_region_clear(self, region_id: str, poll_interval: float = 0.5):
        """Simulate waiting for region to clear."""
        print(f"[WAIT-REGION] {self.state.name} → waiting until region '{region_id}' is clear...")
        # In simulation we just pretend it's instantly clear
        time.sleep(poll_interval)
        print(f"[WAIT-REGION] region '{region_id}' is now clear (simulated).")

    def communicate(self, channel: str, message: str):
        """Simulate sending a message on a communication channel."""
        print(f"[COMM] {self.state.name}@{channel} → {message}")

    # -------- precondition-style helpers --------

    def check_region_clear(self, region_id: str) -> bool:
        print(f"[CHECK] region_clear({region_id}) -> True (simulated)")
        return True

    def check_sensor_ok(self, sensor_name: str) -> bool:
        print(f"[CHECK] sensor_ok({sensor_name}) -> True (simulated)")
        return True
