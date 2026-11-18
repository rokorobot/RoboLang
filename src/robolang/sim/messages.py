"""
Incoming messages (RoboLang 2.1, decision 0043; spec/simulation-v1.0.md section 19).

A message counts for a condition once it has arrived at or after the start of the run (C4), and stays counted for the
rest of the run (S2, C7). Its text is compared exactly after trimming spaces at both ends of both texts (S3, C11), and
it counts only if its sender is one the world declares for its channel (S4): any other message is recorded as ignored.
A world with no `channels` has no message service, so the condition is unknown there (S5, C1). This module holds the
checks that depend only on the task and the world (the channel the task names is declared: S011, run before the run
reaches `AUTHORIZED` and in `validate --world`) and the record of what arrived.
"""

from __future__ import annotations
from collections.abc import Iterator
from dataclasses import dataclass
from decimal import Decimal

from .. import ir
from .actions import ActionProblem
from .world import FACT_MESSAGE, Event, World


@dataclass(frozen=True)
class ReceivedMessage:
    """A message that arrived during a run (or a mission): `accepted` when its sender is allowed on its channel."""

    t_ms: Decimal
    channel: str
    text: str
    sender: str
    accepted: bool


def same_text(wanted: str, arrived: str) -> bool:
    """S3, C11: exact and case-sensitive, after trimming spaces (U+0020 only) at both ends of both texts."""
    return wanted.strip(" ") == arrived.strip(" ")


def message_events(world: World) -> tuple[Event, ...]:
    """The world's messages, in time order (the order of `World.events`)."""
    return tuple(e for e in world.events if e.fact == FACT_MESSAGE)


def received(world_messages: tuple[Event, ...], world: World, start_ms: Decimal, now_ms: Decimal
             ) -> tuple[ReceivedMessage, ...]:
    """Every message that arrived from `start_ms` to `now_ms`, both included, with whether it was accepted."""
    return tuple(
        ReceivedMessage(e.at_ms, e.target, e.value.text, e.value.sender, e.value.sender in world.channels[e.target])
        for e in world_messages if start_ms <= e.at_ms <= now_ms
    )


def atoms(condition: ir.Condition) -> Iterator[ir.MessageReceived]:
    """The `message_received` atoms of a condition, in order."""
    stack = [condition]
    while stack:
        node = stack.pop()
        if isinstance(node, (ir.And, ir.Or)):
            stack.extend(reversed(node.conditions))
        elif isinstance(node, ir.Not):
            stack.append(node.condition)
        elif isinstance(node, ir.MessageReceived):
            yield node


def has_message(task: ir.Task) -> bool:
    """True when a condition of the task (a `pre`, an `if`, a `wait until`) is a `message_received`."""
    return any(True for _ in _conditions(task))


def _conditions(task: ir.Task) -> Iterator[tuple[int | None, int | None, str, ir.MessageReceived]]:
    """(precondition number, step number, label, atom) of every message condition of a task; numbers from 1."""
    for number, condition in enumerate(task.preconditions, start=1):
        for atom in atoms(condition):
            yield number, None, "", atom
    for index, (label, step) in enumerate(ir.walk(task.steps), start=1):
        if isinstance(step, (ir.If, ir.WaitUntil)):
            for atom in atoms(step.condition):
                yield None, index, label, atom


def message_problems(task: ir.Task, world: World) -> list[ActionProblem]:
    """
    S011 (C1, S4): in a world that declares channels, a task that names a channel the world does not declare fails
    before the run. A world with no channels is not checked here: it has no message service, and the condition is
    unknown at run time (S5). Every message condition counts, whether or not a run reaches it: the channel is a fact
    of the task and the world, not of the path.
    """
    if not world.channels:
        return []
    declared = ", ".join(f"'{c}'" for c in sorted(world.channels))
    return [
        ActionProblem("S011", f"'message_received \"{atom.channel}\"': the world declares no channel '{atom.channel}' "
                              f"(channels: {declared})", None, step, label, precondition=number)
        for number, step, label, atom in _conditions(task) if atom.channel not in world.channels
    ]


def undeclared_in(condition: ir.Condition | None, world: World) -> list[str]:
    """The channels a condition names that a world with channels does not declare (a mission's `until`)."""
    if condition is None or not world.channels:
        return []
    return [a.channel for a in atoms(condition) if a.channel not in world.channels]
