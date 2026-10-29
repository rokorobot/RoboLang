"""Smoke tests for the v1 line-based interpreter (legacy/robolang_runtime.py)."""

from __future__ import annotations
import os
import subprocess
import sys

from conftest import PICK_AND_PLACE, REPO_ROOT, task

DEMO_JOINTS = [0.0, -1.0, 1.0, 0.0, 1.0, 0.0]

# Mirrors the "Expected output" section of docs/legacy-v1.md.
EXPECTED_DEMO_OUTPUT = """\
[RUNTIME] executing: move r to src;
[MOVE] robot_arm_1 → joints=[0.0, -1.0, 1.0, 0.0, 1.0, 0.0] (duration=2.0s)
[RUNTIME] executing: grasp r box;
[GRIPPER] robot_arm_1 → CLOSE (position=1.0, max_effort=50.0)
[RUNTIME] executing: move r to dst;
[MOVE] robot_arm_1 → joints=[0.0, -1.0, 1.0, 0.0, 1.0, 0.0] (duration=2.0s)
[RUNTIME] executing: place r box at dst;
[GRIPPER] robot_arm_1 → OPEN (position=0.0, max_effort=50.0)
[RUNTIME] executing: communicate r to "fleet" with "TASK_COMPLETE";
[COMM] robot_arm_1@fleet → TASK_COMPLETE
"""


def run_cli(*args: str, encoding: str | None = None) -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env.pop("PYTHONUTF8", None)
    env.pop("PYTHONIOENCODING", None)
    if encoding:
        env["PYTHONIOENCODING"] = encoding
    return subprocess.run(
        [sys.executable, "legacy/robolang_runtime.py", *args],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        timeout=60,
    )


# -------- in-process --------

def test_pick_and_place_example_call_sequence(runtime, adapter):
    runtime.execute_file(PICK_AND_PLACE)

    assert adapter.calls == [
        ("move_to_pose", [], DEMO_JOINTS, 2.0),
        ("set_gripper", 1.0),
        ("move_to_pose", [], DEMO_JOINTS, 2.0),
        ("set_gripper", 0.0),
        ("communicate", "fleet", "TASK_COMPLETE"),
    ]


def test_stdlib_pick_and_place_runs_like_the_example(runtime, adapter):
    runtime.execute_file(REPO_ROOT / "legacy" / "robolang_std_v1.rob")

    assert adapter.names() == ["move_to_pose", "set_gripper", "move_to_pose", "set_gripper", "communicate"]


def test_nothing_outside_plan_block_is_executed(runtime, adapter):
    runtime.execute("move r to src;\ngrasp r box;\n")

    assert adapter.calls == []


def test_comments_and_blank_lines_are_skipped(runtime, adapter):
    runtime.execute(task("// move r to src;\n\n        grasp r box;"))

    assert adapter.calls == [("set_gripper", 1.0)]


def test_inspect_uses_quoted_sensor_or_default(runtime, adapter):
    runtime.execute(task('inspect r box with "camera_side";\n        inspect r box;'))

    assert adapter.calls == [
        ("inspect", "object", "camera_side"),
        ("inspect", "object", "camera_top"),
    ]


def test_wait_for_whole_seconds(runtime, adapter):
    runtime.execute(task("wait for 5s;"))

    assert adapter.calls == [("wait_for", 5.0)]


def test_empty_file_executes_nothing(runtime, adapter):
    runtime.execute("")

    assert adapter.calls == []


# -------- command line --------

def test_demo_cli_matches_readme_output():
    result = run_cli("examples/pick_and_place.rob")

    assert result.returncode == 0, result.stderr.decode(errors="replace")
    assert result.stdout.decode("utf-8").replace("\r\n", "\n") == EXPECTED_DEMO_OUTPUT


def test_demo_cli_survives_non_unicode_console():
    """Regression: piped output on Windows (cp1252) crashed with UnicodeEncodeError on '→'."""
    for encoding in ("cp1252", "ascii"):
        result = run_cli("examples/pick_and_place.rob", encoding=encoding)

        assert result.returncode == 0, result.stderr.decode(errors="replace")
        assert result.stdout.decode("utf-8").replace("\r\n", "\n") == EXPECTED_DEMO_OUTPUT


def test_cli_without_arguments_prints_usage_and_fails():
    result = run_cli()

    assert result.returncode == 1
    assert b"Usage:" in result.stdout
