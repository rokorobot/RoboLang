"""
The robot's own charge in a world (RoboLang 2.4, decision 0048; spec/simulation-v1.0.md section 22).

A robot has a charge in `%` (world file 1.9: `charge`, a value and an optional uncertainty), or none, which is an
unknown charge. The charge changes only by the scheduled `charge_events` of the world: there is no drain and no charging
model, and `do r charge charger` changes no reading (S4, open point 7). A reading of an unknown charge fails the step
(S239), which fails closed. This module holds what depends only on the task and the world: the checks S012 and S013 that
run before the run reaches `AUTHORIZED` and in `validate --world` (open point 3), and the text of a reading.
"""

from __future__ import annotations
from decimal import Decimal

from .. import ir
from ..measured import compare_intervals, reading_interval
from .actions import ActionProblem
from .values import duration_ms_of
from .world import FACT_CHARGE, Charge, World

NO_CHARGE = "S239"  # a reading of a charge the robot does not have: the step fails (fails closed)
UNKNOWN_CHARGE = "S012"  # a reading of a robot that never has a charge, with no fallback: fails before the run
NEVER_MET = "S013"  # a wait that no charge the world ever gives can meet, with no fallback: fails before the run


def _ends_in_fail(handler: tuple[ir.Step, ...] | None) -> bool:
    """No handler, or one that always ends in `fail`: there is no fallback for the step (the rule of E329)."""
    return handler is None or not handler or isinstance(handler[-1], ir.Fail)


def _required(condition: ir.Condition):
    """The comparisons a condition requires: the condition itself or the members of an `and` (not under `or`/`not`)."""
    if isinstance(condition, ir.And):
        for inner in condition.conditions:
            yield from _required(inner)
    elif isinstance(condition, ir.Compare):
        yield condition


def _values(world: World, robot: str, until_ms: Decimal) -> list[Charge]:
    """Every charge the world gives `robot` up to `until_ms`: the one it starts with, then each scheduled event."""
    start = world.robots[robot].charge
    given = [] if start is None else [start]
    return given + [e.value for e in world.events if e.fact == FACT_CHARGE and e.target == robot
                    and e.at_ms <= until_ms]


def charge_problems(task: ir.Task, world: World, bound: dict[str, str],
                    elapsed_ms: Decimal = Decimal(0)) -> list[ActionProblem]:
    """
    S012 and S013 (decision 0048 S4, open point 3): a step of the plan's own sequence that cannot work in this world and
    has no fallback fails before the run.

    - S012: a `measure r charge`, or a `wait until robot_charge r ...`, of a robot that has no charge and is given none
      by any event. Its reading is unknown at every look.
    - S013: a `wait until robot_charge r OP T` (alone or as a member of an `and`) that none of the charges the world
      gives the robot, up to the end of the wait, can ever meet.

    "No fallback" is no failure handler, or one that always ends in `fail`. Only a step that every run reaches is
    checked (one in the plan's own sequence, not in an `if` branch or a handler), as K3 does for the clock, and only in
    the first run of a mission: a later run starts from a world that the runs before it have changed. The latest a wait
    can start is after every earlier `wait for` and every earlier `wait until` ran to its timeout.
    """
    if elapsed_ms:
        return []
    problems: list[ActionProblem] = []
    latest = Decimal(0)  # the latest simulated time the next step can start
    for index, (label, step) in enumerate(ir.walk(task.steps), start=1):
        if "." in label:
            continue
        if isinstance(step, ir.ReadCharge) and _ends_in_fail(step.orelse):
            robot = bound.get(step.robot)
            if robot is not None and not _values(world, robot, Decimal("Infinity")):
                problems.append(ActionProblem(
                    UNKNOWN_CHARGE, f"'measure {step.robot} charge': robot '{robot}' has no charge in this world and "
                                    "no scheduled event gives it one, so the reading is unknown and the step fails "
                                    "closed", robot, index, label))
        elif isinstance(step, ir.WaitFor):
            latest += duration_ms_of(step.duration)
        elif isinstance(step, ir.WaitUntil):
            end = latest + duration_ms_of(step.timeout)
            if _ends_in_fail(step.orelse):
                problems += _wait_problems(step, world, bound, index, label, end)
            latest = end
    return problems


def _wait_problems(step: ir.WaitUntil, world: World, bound: dict[str, str], index: int, label: str,
                   end_ms: Decimal) -> list[ActionProblem]:
    problems: list[ActionProblem] = []
    for atom in _required(step.condition):
        if not isinstance(atom.left, ir.LiveCharge) or atom.left.robot not in bound \
                or not isinstance(atom.right, ir.QuantityLit):
            continue
        robot = bound[atom.left.robot]
        given = _values(world, robot, end_ms)
        text = f"'wait until robot_charge {atom.left.robot} {atom.op} {ir.decimal_text(atom.right.value)} %'"
        if not _values(world, robot, Decimal("Infinity")):
            problems.append(ActionProblem(
                UNKNOWN_CHARGE, f"{text}: robot '{robot}' has no charge in this world and no scheduled event gives "
                                "it one, so the wait never sees a value and times out", robot, index, label))
            continue
        limit = reading_interval(atom.right.value, Decimal(0), atom.right.unit)
        if not any(compare_intervals(atom.op, reading_interval(c.value, c.uncertainty, "%"), limit) is True
                   for c in given):
            problems.append(ActionProblem(
                NEVER_MET, f"{text}: no charge that this world gives robot '{robot}' by the end of the wait meets it "
                           "(the charge changes only by the scheduled charge events), so the wait would time out",
                robot, index, label))
    return problems
