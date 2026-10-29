"""Smoke tests for the simulated adapter (legacy/robolang_adapter.py)."""

from __future__ import annotations

import robolang_adapter
from robolang_adapter import RobotAdapter


def test_move_records_pose_and_prints(capsys):
    adapter = RobotAdapter("arm")
    adapter.move_to_pose(joint_names=[], joint_positions=[0.1, 0.2], duration=1.5)

    assert adapter.state.last_pose == [0.1, 0.2]
    assert capsys.readouterr().out == "[MOVE] arm → joints=[0.1, 0.2] (duration=1.5s)\n"


def test_gripper_reports_open_and_close(capsys):
    adapter = RobotAdapter("arm")
    adapter.set_gripper(position=1.0)
    adapter.set_gripper(position=0.0)

    out = capsys.readouterr().out.splitlines()
    assert out == [
        "[GRIPPER] arm → CLOSE (position=1.0, max_effort=50.0)",
        "[GRIPPER] arm → OPEN (position=0.0, max_effort=50.0)",
    ]


def test_inspect_prints_and_returns_true(capsys):
    adapter = RobotAdapter("arm")

    assert adapter.inspect(target_id="box", sensor_name="camera_top") is True
    assert capsys.readouterr().out == "[INSPECT] arm → target=box via sensor=camera_top\n"


def test_communicate_prints_channel_and_message(capsys):
    RobotAdapter("arm").communicate("fleet", "TASK_COMPLETE")

    assert capsys.readouterr().out == "[COMM] arm@fleet → TASK_COMPLETE\n"


def test_wait_for_sleep_is_capped_at_two_seconds(monkeypatch, capsys):
    slept = []
    monkeypatch.setattr(robolang_adapter.time, "sleep", slept.append)

    RobotAdapter("arm").wait_for(30)

    assert slept == [2.0]
    assert capsys.readouterr().out == "[WAIT] arm → 30.00s\n"


def test_wait_until_region_clear_returns_after_one_poll(monkeypatch, capsys):
    slept = []
    monkeypatch.setattr(robolang_adapter.time, "sleep", slept.append)

    RobotAdapter("arm").wait_until_region_clear("cell", poll_interval=0.25)

    assert slept == [0.25]
    assert "region 'cell' is now clear (simulated)." in capsys.readouterr().out


def test_default_robot_name():
    assert RobotAdapter().state.name == "robot_arm_1"
