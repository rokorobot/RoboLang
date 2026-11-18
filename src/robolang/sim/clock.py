"""
The clock of a world (RoboLang 1.9, decision 0042; spec/simulation-v1.0.md section 17).

`time` is the world's `clock.start` plus the simulated time, on the calendar day of the start only (T4); `elapsed` is
the simulated time since the run reached `RUNNING` (K6). Readings are exact. This module holds what depends only on the
task, the world and the bindings: the check K3 (a wait for a time that has already passed) that runs before the run
reaches `AUTHORIZED` and in `validate --world`, and the text of a reading (T8).
"""

from __future__ import annotations
from decimal import Decimal

from .. import ir
from .actions import ActionProblem
from .world import World

DAY_S = 86400  # one day only (T4)


def clock_text(kind: str, value: Decimal) -> str:
    """T8, K8: a `time` as `HH:MM:SS` (a fraction of a second is cut off), an `elapsed` as seconds with the unit."""
    if kind == "time":
        whole = int(value)
        return f"{whole // 3600:02d}:{whole // 60 % 60:02d}:{whole % 60:02d}"
    return f"{ir.decimal_text(value)} s"


def _seconds(text: str) -> int:
    hours, minutes, seconds = (int(part) for part in text.split(":"))
    return hours * 3600 + minutes * 60 + seconds


def _required(condition: ir.Condition):
    """The comparisons a condition requires: the condition itself or the members of an `and` (not under `or`/`not`)."""
    if isinstance(condition, ir.And):
        for inner in condition.conditions:
            yield from _required(inner)
    elif isinstance(condition, ir.Compare):
        yield condition


def clock_problems(task: ir.Task, world: World, bound: dict[str, str],
                   elapsed_ms: Decimal = Decimal(0)) -> list[ActionProblem]:
    """
    K3: a `wait until clock_time >= T` (or `> T`) whose T is before the clock's start fails before the run: "at 09:00"
    must never silently mean "now". Only a wait that every run reaches is checked, that is, one in the plan's own
    sequence: a wait in an `if` branch or a failure handler may not run, and a time test around it is exactly how "if it
    is already after 9, go now" is written. A world with no clock is not checked here: the `time` reading fails the step
    (T7). `elapsed_ms` is the time already gone when the run starts: a run of a mission (RoboLang 2.0) starts after the
    runs before it, and its start time is the clock's start plus that.
    """
    if world.clock is None:
        return []
    now = world.clock.start_s + elapsed_ms / 1000
    problems = []
    for index, (label, step) in enumerate(ir.walk(task.steps), start=1):
        if not isinstance(step, ir.WaitUntil) or "." in label:
            continue
        for atom in _required(step.condition):
            if not (isinstance(atom.left, ir.LiveClock) and atom.left.reading == "time" and atom.op in (">=", ">")
                    and isinstance(atom.right, ir.TimeLit)):
                continue
            if _seconds(atom.right.value) < now:
                start = clock_text("time", Decimal(int(now)))
                problems.append(ActionProblem(
                    "S010", f"'wait until clock_time {atom.op} {atom.right.value}': the clock "
                            f"{'is at' if elapsed_ms else 'starts at'} {start} on "
                            f"{world.clock.date}, so {atom.right.value} has already passed and the wait would not "
                            "wait; to act at once when it is already past that time, read it with 'measure' and "
                            "use 'if'",
                    None, index, label))
    return problems
