"""
Region limits against a world (RoboLang 1.7, decision 0039; spec/simulation-v1.0.md section 15).

The checks of `@safety no_go_region R;` and `@safety in_region R;` that depend only on the task, the world's regions and
robot profiles, and the bindings: whether a robot can honour the limit (P6, C6), whether a target lies where the limit
forbids (P5), and whether a robot starts where it forbids (C7). They run before the run reaches `AUTHORIZED` and in
`validate --world`. Nothing is enforced: no route is modelled, so a limit that passes these checks is recorded as
`checked_against_world_not_enforced`, never as enforced.
"""

from __future__ import annotations
from dataclasses import dataclass

from .. import ir
from .world import World

CHECKED = "checked_against_world_not_enforced"


@dataclass(frozen=True)
class RegionProblem:
    code: str  # S003, S004 or S005
    message: str
    robot: str  # the robot at fault (an entity ID)


def _violates(limit: ir.RegionLimit, world: World, region: str, location: str) -> bool:
    """True when `location` is where the limit forbids a robot to be. A region that lists no locations is unknown."""
    contains = world.regions[region].contains
    if contains is None:
        return False
    return location in contains if limit.kind == "no_go_region" else location not in contains


def _forbidden(limit: ir.RegionLimit, region: str) -> str:
    return f"inside {limit.kind} '{region}'" if limit.kind == "no_go_region" else f"outside in_region '{region}'"


def _goal_atoms(task: ir.Task):
    """The `robot_at` atoms that stand alone in a goal list (the task's or an `if` branch's): a robot must end there."""
    lists = [task.goal or ()]
    for _, step in ir.walk(task.steps):
        if isinstance(step, ir.If):
            lists += [step.goal_then or (), step.goal_else or ()]
    return [c for goals in lists for c in goals if isinstance(c, ir.RobotAt)]


def limit_problems(task: ir.Task, world: World, bound: dict[str, str], limit: ir.RegionLimit) -> list[RegionProblem]:
    """Every problem of one region limit, in the order of section 15.3: S003, S004, S005."""
    region = bound[limit.region]
    steps = [step for _, step in ir.walk(task.steps)]
    movers = []  # the robots that move on any path, handlers included (C6), in parameter order
    for p in task.parameters:
        if p.type == "robot" and any(isinstance(s, ir.Move) and s.robot == p.name for s in steps):
            movers.append(bound[p.name])
    problems = []
    for robot in movers:
        if limit.kind not in world.robots[robot].honours:
            declared = ", ".join(world.robots[robot].honours) or "none"
            problems.append(RegionProblem("S003", f"robot '{robot}' moves in this task but does not declare that it "
                                                  f"honours {limit.kind} (its 'honours': {declared})", robot))
    for step in steps:
        if getattr(step, "target", None) not in bound or getattr(step, "robot", None) not in bound:
            continue  # RoboLang 2.3: a robot or place a `choose` binds is checked when the step runs (decision 0045)
        # RoboLang 1.8 (decision 0040 C13): a `do` on a location counts as a target, as `move` and `place` do
        counts = isinstance(step, (ir.Move, ir.Place)) or (
            isinstance(step, ir.Do) and world.kind_of(bound[step.target]) == "location")
        if counts and _violates(limit, world, region, bound[step.target]):
            problems.append(RegionProblem("S004", f"a {step.KIND} step has '{bound[step.target]}' as its target, which "
                                                  f"is {_forbidden(limit, region)}", bound[step.robot]))
    for atom in _goal_atoms(task):
        if atom.location not in bound or atom.robot not in bound:
            continue
        if _violates(limit, world, region, bound[atom.location]):
            problems.append(RegionProblem("S004", f"the goal puts robot '{bound[atom.robot]}' at "
                                                  f"'{bound[atom.location]}', {_forbidden(limit, region)}",
                                          bound[atom.robot]))
    for robot in movers:
        start = world.robots[robot].location
        if _violates(limit, world, region, start):
            problems.append(RegionProblem("S005", f"robot '{robot}' starts at '{start}', {_forbidden(limit, region)}",
                                          robot))
    return problems
