"""
Declared actions against a world (RoboLang 1.8, decision 0040; spec/simulation-v1.0.md section 16).

The checks of `do` and of a goal `in_state` that depend only on the task, the world's robot profiles and states, and the
bindings: the robot declares the action (A6), its target is of the declared kind (A6), the value is of the declared
kind (A6, A5), and a required goal state is one that some declared action produces or the world already holds (C7).
They run before the run reaches `AUTHORIZED` and in `validate --world`. A declared action is a request to the robot's
own controller (A9): nothing here says what an action does physically.
"""

from __future__ import annotations
from dataclasses import dataclass

from .. import ir
from ..tokens import KIND_UNITS
from .world import Action, World


@dataclass(frozen=True)
class ActionProblem:
    code: str  # S006, S007, S008 or S009
    message: str
    robot: str | None  # the robot at fault (an entity ID); None for S009
    step: int | None  # the 1-based index of the `do` step in ir.walk order; None for S009
    label: str = ""  # the path label of the step (ir.walk), "" for S009
    goal: int | None = None  # S009: the 0-based index of the goal condition
    precondition: int | None = None  # S011: the 1-based number of the `pre` condition, when the problem is there


def entity_state(world: World, entity: str) -> str | None:
    """The state label the world gives an object or a location, or None."""
    obj = world.objects.get(entity)
    return obj.state if obj is not None else world.location_states.get(entity)


def _value_text(value: ir.QuantityLit | ir.TextLit) -> str:
    return f"{ir.decimal_text(value.value)} {value.unit}" if isinstance(value, ir.QuantityLit) else f'"{value.value}"'


def value_problem(action: Action, value: ir.QuantityLit | ir.TextLit | None) -> str | None:
    """Why `value` does not fit what `action` declares it takes (A5, A6), or None."""
    if action.value is None:
        given = None if value is None else _value_text(value)
        return None if value is None else f"action '{action.name}' takes no value, and the step gives {given}"
    units = KIND_UNITS[action.value]
    wanted = "a text" if action.value == "label" else f"a {action.value} (in {', '.join(units)})"
    if value is None:
        return f"action '{action.name}' takes a value, {wanted}, and the step gives none"
    if isinstance(value, ir.TextLit) == (action.value == "label") and (
            action.value == "label" or value.unit in units):
        return None
    return f"action '{action.name}' takes {wanted}, and the step gives {_value_text(value)}"


def _step_problem(world: World, bound: dict[str, str], step: ir.Do) -> tuple[str, str] | None:
    robot, target = bound[step.robot], bound[step.target]
    action = world.robots[robot].action(step.action)
    if action is None:
        declared = ", ".join(a.name for a in world.robots[robot].actions) or "none"
        return "S006", f"robot '{robot}' does not declare the action '{step.action}' (its actions: {declared})"
    kind = world.kind_of(target)
    if action.target != kind:
        return "S007", (f"action '{step.action}' of robot '{robot}' acts on {_a(action.target)}, "
                        f"but '{target}' is {_a(kind)}")
    problem = value_problem(action, step.value)
    return None if problem is None else ("S008", f"robot '{robot}': {problem}")


def _a(word: str) -> str:
    return f"an {word}" if word[0] in "aeiou" else f"a {word}"


def _required(conditions: tuple[ir.Condition, ...]):
    """The `in_state` atoms a goal list requires on every path: the ones directly in it or in an `and`."""
    for i, condition in enumerate(conditions):
        stack = [condition]
        while stack:
            node = stack.pop(0)
            if isinstance(node, ir.And):
                stack = list(node.conditions) + stack
            elif isinstance(node, ir.InState):
                yield i, node


def _goal_problems(task: ir.Task, world: World, bound: dict[str, str]) -> list[ActionProblem]:
    robots = [bound[p.name] for p in task.parameters if p.type == "robot"]
    problems = []
    for i, atom in _required(task.goal or ()):
        entity = bound[atom.target]
        kind = world.kind_of(entity)
        if entity_state(world, entity) == atom.label:
            continue  # the world already holds it
        if any(a.result == atom.label and a.target == kind for r in robots for a in world.robots[r].actions):
            continue
        held = entity_state(world, entity)
        now = f"the world holds '{held}'" if held is not None else "the world gives it no state"
        problems.append(ActionProblem(
            "S009", f"no declared action of the task's robots leaves '{entity}' in state '{atom.label}', and {now}; "
                    "the goal can never hold", None, None, goal=i))
    return problems


def action_problems(task: ir.Task, world: World, bound: dict[str, str]) -> list[ActionProblem]:
    """Every problem of the task's `do` steps (in step order) and then of its required goal states."""
    problems = []
    for index, (label, step) in enumerate(ir.walk(task.steps), start=1):
        if isinstance(step, ir.Do):
            found = _step_problem(world, bound, step)
            if found is not None:
                problems.append(ActionProblem(found[0], found[1], bound[step.robot], index, label))
    return problems + _goal_problems(task, world, bound)
