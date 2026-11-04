"""
Paths through a branching plan (RoboLang 1.4, spec/robolang-v1.4.md section 4).

A path is one choice of `then` or `else` at every `if` it reaches. Only the rule "every path has a
goal condition or ends in `fail`" (E309) needs them, and it is decided here without listing the
paths: a path is summarised by whether it has a goal yet, and one example label per summary is kept.
"""

from __future__ import annotations
from typing import Callable

# parts(step) -> None (a plain step), "fail", or (then_steps, then_has_goal, else_steps, else_has_goal[, sides]):
# an `if`, or in 1.5 a step with a failure handler, whose sides are named ("ok", "failed") instead of ("then", "else")
Parts = Callable[[object], "None | str | tuple"]


def _join(label: str, side: str) -> str:
    return f"{label}/{side}" if label else side


def _run(steps, states: dict[bool, str], parts: Parts) -> dict[bool, str]:
    """The live (not failed) paths after `steps`: has-goal -> an example label, starting from `states`."""
    for step in steps:
        part = parts(step)
        if part is None:
            continue
        if part == "fail":
            return {}
        then_steps, then_goal, else_steps, else_goal, *named = part
        first, second = named[0] if named else ("then", "else")
        after: dict[bool, str] = {}
        for has_goal, label in states.items():
            for side, branch, goal in ((first, then_steps, then_goal), (second, else_steps, else_goal)):
                for key, found in _run(branch, {has_goal or goal: _join(label, side)}, parts).items():
                    after.setdefault(key, found)
        states = after
    return states


def goalless_path(steps, task_has_goal: bool, parts: Parts) -> str | None:
    """Label of a path that ends with no goal condition and no `fail` ("" for a plan with no `if`), or None."""
    return _run(steps, {task_has_goal: ""}, parts).get(False)


# kinds(step) -> None (no effect on holding, no handler), "fail", ("if", then_steps, else_steps, test),
# or (kind, robot parameter, object parameter, handler steps or None) where kind is "grasp", "place" or "step"
# (any other step with a 1.5 failure handler; robot and object are then None). `test` describes an `if` condition
# that the holder facts can contradict: ("holding", robot, object, positive), ("object_at", object, positive), or None.


def _possible(facts: frozenset, test, side: bool) -> bool:
    """False when the known facts make the `then` (side True) or `else` branch of an `if` impossible to take."""
    if test is None:
        return True
    required = test[-1] if side else not test[-1]  # what the branch needs the atom to be
    if test[0] == "holding":
        return required or (test[1], test[2]) not in {(f[0], f[1]) for f in facts}
    return not (required and any(f[1] == test[1] for f in facts))  # object_at: a held object is at no location
Kinds = Callable[[object], "None | str | tuple"]


def double_grasps(steps, kinds: Kinds) -> list[tuple[object, object, str, str]]:
    """
    Grasps that fail on every world where their path is taken (decision 0032, E315): a `grasp` of an object
    that a robot parameter still holds, or by a robot parameter that still holds an object, since that robot's
    `grasp` and before any `place`. A `place` ends what is known: any two parameters may be bound to one
    entity, so after it only "the placing robot holds nothing" is certain, and that is never an error. A path
    stops at its first such grasp, as the run would. Returns (grasp, earlier grasp, holder robot, held object)
    once per grasp, in the order found.
    """
    found: list[tuple[object, object, str, str]] = []
    grasps: list[object] = []  # every grasp in the order visited; a fact names one by its index

    def run(steps, states: set[frozenset]) -> set[frozenset]:
        # a state is the facts (robot, object, grasp index) known on one or more paths; each step is visited once
        for step in steps:
            part = kinds(step)
            if part is None:
                continue
            if part == "fail":
                return set()
            if part[0] == "if":
                test = part[3] if len(part) > 3 else None
                then = run(part[1], {f for f in states if _possible(f, test, True)})
                states = then | run(part[2], {f for f in states if _possible(f, test, False)})
                continue
            kind, robot, obj, handler = part
            before = states
            if kind == "place":
                states = {frozenset()} if states else set()
            elif kind == "grasp":
                after, clashes = set(), []
                for facts in states:
                    clash = next((f for f in facts if f[0] == robot), None) or next((f for f in facts if f[1] == obj),
                                                                                    None)
                    if clash is None:
                        after.add(facts | {(robot, obj, len(grasps))})
                    else:
                        clashes.append(clash)
                if clashes:  # the earliest grasp, so the report does not depend on set order
                    holder, held, earlier = min(clashes, key=lambda f: f[2])
                    found.append((step, grasps[earlier], holder, held))
                grasps.append(step)
                states = after
            if handler is not None:  # the step failed and had no effect, then its handler ran (decision 0033 F8)
                states = states | run(handler, before)
        return states

    run(steps, {frozenset()})
    return found
