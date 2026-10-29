"""
Paths through a branching plan (RoboLang 1.4, spec/robolang-v1.4.md section 4).

A path is one choice of `then` or `else` at every `if` it reaches. Only the rule "every path has a
goal condition or ends in `fail`" (E309) needs them, and it is decided here without listing the
paths: a path is summarised by whether it has a goal yet, and one example label per summary is kept.
"""

from __future__ import annotations
from typing import Callable

# parts(step) -> None (a plain step), "fail", or (then_steps, then_has_goal, else_steps, else_has_goal)
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
        then_steps, then_goal, else_steps, else_goal = part
        after: dict[bool, str] = {}
        for has_goal, label in states.items():
            for side, branch, goal in (("then", then_steps, then_goal), ("else", else_steps, else_goal)):
                for key, found in _run(branch, {has_goal or goal: _join(label, side)}, parts).items():
                    after.setdefault(key, found)
        states = after
    return states


def goalless_path(steps, task_has_goal: bool, parts: Parts) -> str | None:
    """Label of a path that ends with no goal condition and no `fail` ("" for a plan with no `if`), or None."""
    return _run(steps, {task_has_goal: ""}, parts).get(False)
