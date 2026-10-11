"""
Regression tests pinning down the known limitations of the v1 interpreter.

Each limitation has two tests:

  test_current_*   asserts what the runtime does TODAY. These must keep passing
                   until the behavior is changed on purpose.
  test_intended_*  asserts what the runtime SHOULD do. They are marked
                   xfail(strict=True): they fail today, and the suite goes red
                   the moment one starts passing, so the fix has to update the
                   matching test_current_* in the same change.

The intended behavior follows docs/DEVELOPMENT_PLAN.md (v1.1 / v1.2). None of
it is implemented here.
"""

from __future__ import annotations

import pytest

from conftest import RecordingAdapter, task
from robolang_runtime import RoboLangRuntime

planned = pytest.mark.xfail(strict=True, reason="planned behavior, not implemented in v1")


# -------- preconditions are ignored --------

def test_current_preconditions_are_never_evaluated():
    adapter = RecordingAdapter(region_clear=False)
    RoboLangRuntime(adapter).execute(task("move r to src;", pre="region_clear cell;"))

    assert adapter.named("check_region_clear") == []
    assert adapter.names() == ["move_to_pose"]


@planned
def test_intended_failed_precondition_blocks_motion():
    adapter = RecordingAdapter(region_clear=False)
    try:
        RoboLangRuntime(adapter).execute(task("move r to src;", pre="region_clear cell;"))
    except Exception:
        pass

    assert adapter.named("move_to_pose") == []


# -------- @safety annotations are ignored --------

@pytest.mark.parametrize("safety", ["@safety max_speed 0.5;", "@safety max_speed 99;", ""])
def test_current_safety_annotation_has_no_effect(safety):
    adapter = RecordingAdapter()
    RoboLangRuntime(adapter).execute(task("move r to src;", safety=safety))

    assert adapter.calls == [("move_to_pose", [], [0.0, -1.0, 1.0, 0.0, 1.0, 0.0], 2.0)]


def test_current_invalid_safety_annotation_is_accepted():
    adapter = RecordingAdapter()
    RoboLangRuntime(adapter).execute(task("move r to src;", safety="@safety max_speed -1;"))

    assert adapter.names() == ["move_to_pose"]


@planned
def test_intended_invalid_safety_annotation_is_rejected_before_dispatch():
    adapter = RecordingAdapter()
    with pytest.raises(Exception):
        RoboLangRuntime(adapter).execute(task("move r to src;", safety="@safety max_speed -1;"))

    assert adapter.calls == []


# -------- move destinations are not resolved --------

def test_current_move_ignores_destination():
    adapter = RecordingAdapter()
    RoboLangRuntime(adapter).execute(task("move r to src;\n        move r to dst;"))

    first, second = adapter.named("move_to_pose")
    assert first == second == ("move_to_pose", [], [0.0, -1.0, 1.0, 0.0, 1.0, 0.0], 2.0)


@planned
def test_intended_different_destinations_produce_different_commands():
    adapter = RecordingAdapter()
    RoboLangRuntime(adapter).execute(task("move r to src;\n        move r to dst;"))

    first, second = adapter.named("move_to_pose")
    assert first != second


def test_current_undeclared_destination_is_accepted():
    adapter = RecordingAdapter()
    RoboLangRuntime(adapter).execute(task("move r to nowhere;"))

    assert adapter.names() == ["move_to_pose"]


@planned
def test_intended_undeclared_destination_is_rejected_before_dispatch():
    adapter = RecordingAdapter()
    with pytest.raises(Exception):
        RoboLangRuntime(adapter).execute(task("move r to nowhere;"))

    assert adapter.calls == []


# -------- decimal wait durations are truncated --------

def test_current_decimal_wait_is_truncated_to_whole_seconds():
    adapter = RecordingAdapter()
    RoboLangRuntime(adapter).execute(task("wait for 2.5s;"))

    assert adapter.calls == [("wait_for", 2.0)]


@planned
def test_intended_decimal_wait_is_parsed():
    adapter = RecordingAdapter()
    RoboLangRuntime(adapter).execute(task("wait for 2.5s;"))

    assert adapter.calls == [("wait_for", 2.5)]


# -------- unknown instructions are ignored --------

def test_current_unknown_instruction_is_ignored_and_execution_continues(capsys):
    adapter = RecordingAdapter()
    RoboLangRuntime(adapter).execute(task("fly r to moon;\n        grasp r box;"))

    assert adapter.calls == [("set_gripper", 1.0)]
    assert "[RUNTIME] (ignored line): fly r to moon;" in capsys.readouterr().out


@planned
def test_intended_unknown_instruction_is_rejected_before_dispatch():
    adapter = RecordingAdapter()
    with pytest.raises(Exception):
        RoboLangRuntime(adapter).execute(task("grasp r box;\n        fly r to moon;"))

    assert adapter.calls == []


def test_current_instructions_are_matched_by_prefix_only():
    adapter = RecordingAdapter()
    RoboLangRuntime(adapter).execute(task("movement r;\n        grasping;"))

    assert adapter.names() == ["move_to_pose", "set_gripper"]


def test_current_wait_until_is_ignored():
    adapter = RecordingAdapter()
    RoboLangRuntime(adapter).execute(task("wait until region_clear cell;"))

    assert adapter.calls == []


# -------- block structure is not parsed --------

def test_current_plan_block_ends_at_first_closing_brace():
    adapter = RecordingAdapter()
    RoboLangRuntime(adapter).execute(
        task("move r to src;\n        {\n        grasp r box;\n        }\n        place r box at dst;")
    )

    # '{' is ignored, 'grasp' runs, the inner '}' ends the plan, 'place' is dropped.
    assert adapter.names() == ["move_to_pose", "set_gripper"]


def test_current_every_plan_block_in_a_file_is_executed():
    adapter = RecordingAdapter()
    RoboLangRuntime(adapter).execute(task("grasp r box;") + task("place r box at dst;"))

    assert adapter.calls == [("set_gripper", 1.0), ("set_gripper", 0.0)]


# -------- simulated checks always pass --------

def test_current_simulated_adapter_checks_always_report_safe(capsys):
    from robolang_adapter import RobotAdapter

    adapter = RobotAdapter("arm")

    assert adapter.check_region_clear("cell") is True
    assert adapter.check_sensor_ok("camera_top") is True
