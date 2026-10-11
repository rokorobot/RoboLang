"""Shared fixtures for the RoboLang v1 baseline tests."""

from __future__ import annotations
from pathlib import Path

import pytest

from robolang_runtime import RoboLangRuntime

REPO_ROOT = Path(__file__).resolve().parent.parent
PICK_AND_PLACE = REPO_ROOT / "examples" / "pick_and_place.rob"


class RecordingAdapter:
    """
    Adapter double that records every call instead of printing.

    The precondition helpers return configurable values so tests can show
    whether the runtime consults them at all.
    """

    def __init__(self, region_clear: bool = True, sensor_ok: bool = True):
        self.calls: list[tuple] = []
        self._region_clear = region_clear
        self._sensor_ok = sensor_ok

    # -------- core primitives --------

    def move_to_pose(self, joint_names, joint_positions, duration=2.0):
        self.calls.append(("move_to_pose", list(joint_names or []), list(joint_positions), duration))

    def set_gripper(self, position, max_effort=50.0):
        self.calls.append(("set_gripper", position))

    def inspect(self, target_id, sensor_name):
        self.calls.append(("inspect", target_id, sensor_name))
        return True

    def wait_for(self, seconds):
        self.calls.append(("wait_for", seconds))

    def wait_until_region_clear(self, region_id, poll_interval=0.5):
        self.calls.append(("wait_until_region_clear", region_id))

    def communicate(self, channel, message):
        self.calls.append(("communicate", channel, message))

    # -------- precondition-style helpers --------

    def check_region_clear(self, region_id):
        self.calls.append(("check_region_clear", region_id))
        return self._region_clear

    def check_sensor_ok(self, sensor_name):
        self.calls.append(("check_sensor_ok", sensor_name))
        return self._sensor_ok

    # -------- test helpers --------

    def names(self) -> list[str]:
        return [call[0] for call in self.calls]

    def named(self, name: str) -> list[tuple]:
        return [call for call in self.calls if call[0] == name]


@pytest.fixture
def adapter() -> RecordingAdapter:
    return RecordingAdapter()


@pytest.fixture
def runtime(adapter: RecordingAdapter) -> RoboLangRuntime:
    return RoboLangRuntime(adapter)


def task(plan: str, pre: str = "robot_ready r;", safety: str = "@safety max_speed 0.5;") -> str:
    """Wrap plan statements in a minimal v1 task."""
    return (
        "task t(robot r, object box, location src, location dst, region cell) {\n"
        f"    {safety}\n"
        f"    pre {{\n        {pre}\n    }}\n"
        f"    plan {{\n        {plan}\n    }}\n"
        "}\n"
    )
