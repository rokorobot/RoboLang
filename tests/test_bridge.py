"""The shared .rob -> Core 0.1 mapping (src/robolang/bridge.py)."""

from __future__ import annotations
from decimal import Decimal

import pytest

from robolang.bridge import NotExportable, Triple, core_id, core_time, goal_atoms, goal_triples
from robolang.ir import compile_source

TASK = """robolang 1.3;
task t(robot r, object box, location dst) {
    goal { object_at box dst; robot_at r dst; holding r box; }
    plan { move r to dst; }
}
"""
BINDINGS = {"r": "arm1", "box": "box1", "dst": "table"}


def task_of(source: str):
    return compile_source(source).ir.tasks[0]


def test_core_ids_by_prefix():
    assert core_id("object", "box1") == "rl:thing:box1"
    assert core_id("location", "table") == "rl:place:table"
    assert core_id("robot", "arm1") == "rl:robot:arm1"


def test_core_time_counts_from_the_epoch_in_milliseconds():
    assert core_time(0) == "2000-01-01T00:00:00.000Z"
    assert core_time(Decimal("61500.9")) == "2000-01-01T00:01:01.500Z"


def test_outcome_atoms_map_to_triples_in_goal_order():
    assert goal_triples(task_of(TASK), BINDINGS) == (
        Triple("rl:thing:box1", "located_in", "rl:place:table"),
        Triple("rl:robot:arm1", "located_in", "rl:place:table"),
        Triple("rl:thing:box1", "carried_by", "rl:robot:arm1"),
    )


def test_explicit_and_is_the_same_conjunction_as_separate_goal_entries():
    separate = task_of(TASK)
    joined = task_of(TASK.replace(
        "object_at box dst; robot_at r dst; holding r box;",
        "object_at box dst and robot_at r dst and holding r box;",
    ))
    assert goal_atoms(joined) == goal_atoms(separate)
    assert goal_triples(joined, BINDINGS) == goal_triples(separate, BINDINGS)


@pytest.mark.parametrize("condition, kind", [
    ("robot_ready r", "robot_ready"),
    ("not object_at box dst", "not"),
    ("object_at box dst or robot_at r dst", "or"),
])
def test_other_goal_conditions_are_not_exportable(condition, kind):
    source = TASK.replace("holding r box;", f"{condition};")
    with pytest.raises(NotExportable) as raised:
        goal_triples(task_of(source), BINDINGS)
    assert (raised.value.index, raised.value.kind) == (3, kind)


def test_a_task_without_a_goal_is_not_exportable():
    with pytest.raises(NotExportable):
        goal_triples(task_of("task t(robot r, location l) { plan { move r to l; } }"), {"r": "a", "l": "b"})
