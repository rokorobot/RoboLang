"""
Making the choices of a task before it is sent (RoboLang 2.3, decision 0045 C8; spec/simulation-v1.0.md section 21.5).

Core 0.2 is unchanged: a robot only ever receives a goal with every ID fixed. `export-goal` and `delegate` therefore
accept a task with `choose` only if every `choose` is at the top level of the plan with nothing but other `choose`
statements before it (so before the first action) and uses world facts only (which E330 already guarantees). The
planner makes the choices from the world model as it is at the start, records them, and sends the task with the
choices filled in: each chosen name becomes a parameter bound to the entity chosen, and the `choose` steps are gone.
Any other task with `choose` is refused (X609).
"""

from __future__ import annotations
import dataclasses
from dataclasses import dataclass

from .. import ir
from .choice import Choice, choice_problems, choose
from .executor import _evaluate, _State
from .world import World


class GroundError(Exception):
    """The choices cannot be made by the planner. `code` is X609."""

    def __init__(self, message: str):
        super().__init__(message)
        self.code = "X609"
        self.message = message


@dataclass(frozen=True)
class Grounded:
    task: ir.Task
    world: World
    choices: tuple[Choice, ...]  # empty for a task with no `choose`


def ground(task: ir.Task, world: World) -> Grounded:
    """`task` with its choices made against `world` as it starts, or GroundError (X609); no `choose`: as it was."""
    chooses = [(label, step) for label, step in ir.walk(task.steps) if isinstance(step, ir.Choose)]
    if not chooses:
        return Grounded(task, world, ())
    leading = 0
    while leading < len(task.steps) and isinstance(task.steps[leading], ir.Choose):
        leading += 1
    late = [label for label, step in chooses if "." in label or int(label) > leading]
    if late:
        raise GroundError(
            f"the 'choose' at step {late[0]} does not come before the first action: a robot is sent a goal with every "
            "ID fixed, so the planner makes the choices, and only for a 'choose' that has nothing but other choices "
            "before it, at the top level of the plan (decision 0045 C8). A choice that uses only world facts may "
            "come first.")
    bound = dict(world.task.bindings)
    state = _State(world)
    state.params = frozenset(bound)
    problems = choice_problems(task, world, bound)
    if problems:
        raise GroundError(f"{problems[0].message}; the planner cannot choose (S273)")
    made = []
    for index, (label, step) in enumerate(chooses, start=1):
        choice = choose(state, bound, step, label, index, lambda b, c: _evaluate(task, state, b, c))
        made.append(choice)
        if choice.chosen is None:
            raise GroundError(f"the choice '{step.name}' (step {label}) leaves no candidate in the world as it starts: "
                              "there is no goal to send (the 'else' would fail the task)")
        bound[step.name] = choice.chosen
    chosen = [(c.name, c.type, c.chosen) for c in made]
    fixed = dataclasses.replace(
        task, parameters=(*task.parameters, *(ir.Parameter(name, kind) for name, kind, _ in chosen)),
        steps=tuple(task.steps[leading:]))
    bindings = (*world.task.bindings, *((name, entity) for name, _, entity in chosen))
    script = dataclasses.replace(world.task, bindings=bindings)
    return Grounded(fixed, dataclasses.replace(world, task=script), tuple(made))
