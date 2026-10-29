"""
Authoring-time world checks (`robolang validate --world`, spec/simulation-v1.0.md section 11).

The failures a run would certainly hit that depend only on the task, the world's robot profiles and sensors, and the
bindings, never on the state during the run. A 1.3 plan is a straight line, so a plan with one of these can never
succeed in this world. A branching plan (RoboLang 1.4) is checked path by path as straight lines (decision 0027 B6):
a step of a branch is an error on every path through that branch. Nothing is executed and no state is read.
"""

from __future__ import annotations
from dataclasses import dataclass

from .. import ir
from .executor import _sensor_id
from .world import World


@dataclass(frozen=True)
class Finding:
    """
    A certain run failure. `where` is ("safety", i) or ("steps", i): the task's i-th limit or top-level plan step,
    from 0. `label` is the step's path label (ir.walk) when it is inside a branch, else "".
    """

    code: str
    message: str
    where: tuple[str, int]
    label: str = ""


def _speed(value, unit: str) -> str:
    return f"{ir.decimal_text(value)} {unit}"


def check_task(task: ir.Task, world: World) -> list[Finding]:
    """Every certain failure of `task` against `world` under the world's bindings (load_world applied --bind)."""
    bound = dict(world.task.bindings)
    robots = [bound[p.name] for p in task.parameters if p.type == "robot"]
    findings = []
    for index, limit in enumerate(task.safety):  # section 4.3, but every limit, not only the first that fails
        unit = limit.limit.unit
        if unit is None:
            continue
        for robot in robots:
            speeds = {s.unit: s.value for s in world.robots[robot].max_speed}
            if unit not in speeds:
                declared = ", ".join(sorted(speeds)) or "none"
                findings.append(Finding("S002", f"max_speed is in {unit}, but robot '{robot}' declares no maximum "
                                                f"in {unit} (its units: {declared})", ("safety", index)))
                break
            if limit.limit.value > speeds[unit]:
                findings.append(Finding("S001", f"max_speed {_speed(limit.limit.value, unit)} is above the maximum "
                                                f"of robot '{robot}' ({_speed(speeds[unit], unit)})",
                                        ("safety", index)))
                break
    for label, step in ir.walk(task.steps):  # section 4.5; every step of every branch, each reported once
        where = ("steps", int(label.split(".")[0]) - 1)
        problem = _step_problem(task, world, bound, step)
        if problem:
            code, message = problem
            nested = "." in label
            on = _branch_of(label)
            findings.append(Finding(code, f"{message} (on path {on})" if nested else message, where,
                                    label if nested else ""))
    return findings


def _branch_of(label: str) -> str:
    """The innermost branch a step is in, as a path label: "3.else.2.then.1" -> "3.else.2.then"."""
    return label.rsplit(".", 1)[0]


def _step_problem(task: ir.Task, world: World, bound: dict[str, str], step: ir.Step):
    """(code, message) if `step` certainly fails in this world, else None."""
    if isinstance(step, ir.Grasp) and not world.robots[bound[step.robot]].gripper:
        return "S211", f"robot '{bound[step.robot]}' has no gripper"
    if isinstance(step, ir.Inspect):
        robot, sensor_id = bound[step.robot], _sensor_id(task, bound, step.sensor)
        sensor = world.sensors.get(sensor_id)
        if sensor is None:
            return "S231", f"sensor '{sensor_id}' is not declared in the world"
        if sensor.robot != robot:
            mounted = f"mounted on '{sensor.robot}'" if sensor.robot else "not mounted on any robot"
            return "S231", f"sensor '{sensor_id}' is {mounted}, not on robot '{robot}'"
    return None
