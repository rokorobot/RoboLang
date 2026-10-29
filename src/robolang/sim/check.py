"""
Authoring-time world checks (`robolang validate --world`, spec/simulation-v1.0.md section 11).

The failures a run would certainly hit that depend only on the task, the world's robot profiles and sensors, and the
bindings, never on the state during the run. A 1.3 plan is a straight line, so a plan with one of these can never
succeed in this world. Nothing is executed and no state is read.
"""

from __future__ import annotations
from dataclasses import dataclass

from .. import ir
from .executor import _sensor_id
from .world import World


@dataclass(frozen=True)
class Finding:
    """A certain run failure. `where` is ("safety", i) or ("steps", i): the task's i-th limit or plan step, from 0."""

    code: str
    message: str
    where: tuple[str, int]


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
    # A branching task (RoboLang 1.4) has no straight line to check: its step checks come with its paths.
    for index, step in enumerate([] if task.branching else task.steps):  # section 4.5
        if isinstance(step, ir.Grasp) and not world.robots[bound[step.robot]].gripper:
            findings.append(Finding("S211", f"robot '{bound[step.robot]}' has no gripper", ("steps", index)))
        elif isinstance(step, ir.Inspect):
            robot, sensor_id = bound[step.robot], _sensor_id(task, bound, step.sensor)
            sensor = world.sensors.get(sensor_id)
            if sensor is None:
                findings.append(Finding("S231", f"sensor '{sensor_id}' is not declared in the world",
                                        ("steps", index)))
            elif sensor.robot != robot:
                mounted = f"mounted on '{sensor.robot}'" if sensor.robot else "not mounted on any robot"
                findings.append(Finding("S231", f"sensor '{sensor_id}' is {mounted}, not on robot '{robot}'",
                                        ("steps", index)))
    return findings
