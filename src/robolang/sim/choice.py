"""
Choosing at run time (RoboLang 2.3, decision 0045; spec/simulation-v1.0.md section 21).

`choose TYPE NAME from CANDIDATES [where C] [nearest to X | farthest from X] else { ... }` binds a name to one entity.
The candidates are a list, `objects at L` (the objects that are at L when the choice is made, in the order the world
lists them) or `robots` (every robot, in the world's order). `where` keeps the candidates for which a world fact
holds; an unknown value excludes a candidate (it fails closed). The order is the straight-line distance between the
positions of two locations (world file 1.8): a robot or an object is at the position of its location, and a candidate
with no position is unknown and left out. Ties, and a choice with no order, go to the first candidate in the order
written or listed, so a choice is deterministic and can be reproduced. Nothing here reads a value: a choice uses world
facts only (C3, C9).
"""

from __future__ import annotations
from collections.abc import Callable
from dataclasses import dataclass
from decimal import Decimal

from .. import ir
from .values import Truth
from .world import World

NO_CANDIDATE = "S272"  # a choice left no candidate: its `else` runs (a handled step failure, decision 0045 C6)
NO_POSITION = "S273"  # the place an order measures from has no position in the world: the run fails before it starts
WHERE_FALSE, WHERE_UNKNOWN, UNPOSITIONED = "where_false", "where_unknown", "no_position"  # why a candidate is out


@dataclass(frozen=True)
class Choice:
    """One choice that was reached (C7): what it chose from, what was left out and why, the order and the result."""

    step: str  # path label of the `choose` step
    index: int  # its 1-based index in ir.walk order
    type: str
    name: str
    candidates: tuple[str, ...]  # in the order written or listed, each entity once
    excluded: tuple[tuple[str, str], ...]  # (entity, why): where_false, where_unknown or no_position
    order: ir.Order | None
    reference: str | None  # the entity the order measures from
    distances: tuple[tuple[str, Decimal], ...]  # (entity, squared distance in m2) of the candidates that were ordered
    kept: tuple[str, ...]  # the candidates left, best first
    chosen: str | None  # None when no candidate was left
    t_ms: Decimal


@dataclass(frozen=True)
class ChoiceProblem:
    """A choice that certainly fails before the run starts (S273). The fields of `actions.ActionProblem` it shares."""

    code: str
    message: str
    robot: str | None
    step: int
    label: str
    goal: int | None = None
    precondition: int | None = None


def entity_location(state, entity: str) -> str | None:
    """The location a robot, an object or a location is at now; None when it is unknown (or an object is absent)."""
    world: World = state.world
    if entity in world.locations:
        return entity
    if entity in world.robots:
        return state.location[entity]
    if entity in world.objects:
        if state.present[entity] is not True:
            return None
        holder = state.held_by[entity]
        return state.location[holder] if holder is not None else state.at[entity]
    return None


def position_of(state, entity: str) -> tuple[Decimal, Decimal] | None:
    place = entity_location(state, entity)
    return None if place is None else state.world.positions.get(place)


def squared_distance(a: tuple[Decimal, Decimal], b: tuple[Decimal, Decimal]) -> Decimal:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2  # exact: nothing is rounded


def candidates_of(state, bound: dict[str, str], step: ir.Choose) -> tuple[str, ...]:
    """The candidate entities, in the order written (a list) or the world lists them, each once."""
    source = step.source
    if isinstance(source, ir.ChooseList):
        found = [bound[name] for name in source.candidates]
    elif isinstance(source, ir.ObjectsAt):
        place = bound[source.location]
        found = [o for o in state.world.objects
                 if state.present[o] is True and state.held_by[o] is None and state.at[o] == place]
    else:
        found = list(state.world.robots)
    return tuple(dict.fromkeys(found))


def choose(state, bound: dict[str, str], step: ir.Choose, label: str, index: int,
           evaluate: Callable[[dict[str, str], ir.Condition], Truth]) -> Choice:
    """
    Make one choice from the facts of `state` now. `evaluate(bound, condition)` is the executor's condition evaluator.
    The entity is returned in `Choice.chosen`; the caller binds `step.name` to it.
    """
    candidates = candidates_of(state, bound, step)
    excluded: list[tuple[str, str]] = []
    kept = []
    for entity in candidates:
        if step.where is not None:
            value = evaluate({**bound, step.name: entity}, step.where)
            if value is not Truth.TRUE:
                excluded.append((entity, WHERE_UNKNOWN if value is Truth.UNKNOWN else WHERE_FALSE))
                continue
        kept.append(entity)
    reference = None
    distances: list[tuple[str, Decimal]] = []
    if step.order is not None:
        reference = bound[step.order.reference]
        origin = position_of(state, reference)
        ranked = []
        for entity in kept:
            where = None if origin is None else position_of(state, entity)
            if where is None:  # a candidate or a reference point with no position is unknown: left out (C5)
                excluded.append((entity, UNPOSITIONED))
            else:
                ranked.append((entity, squared_distance(origin, where)))
        distances = ranked
        # sorted is stable, so a tie keeps the order written; farthest negates the distance, not the order
        ranked = sorted(ranked, key=lambda pair: pair[1] if step.order.mode == "nearest" else -pair[1])
        kept = [entity for entity, _ in ranked]
    return Choice(label, index, step.type, step.name, candidates, tuple(excluded), step.order, reference,
                  tuple(distances), tuple(kept), kept[0] if kept else None, state.clock)


def choice_problems(task: ir.Task, world: World, bound: dict[str, str]) -> list[ChoiceProblem]:
    """
    S273 (decision 0045 C5, open point 7): a choice with an order whose reference point is a location with no position
    in the world. Positions never change, so every candidate would be left out whatever the run does: the task is
    infeasible in this world and fails before it starts, as `delegate` needs to know. A robot or an object has a place
    that depends on the run; it is checked when the choice is made.
    """
    problems = []
    for index, (label, step) in enumerate(ir.walk(task.steps), start=1):
        if isinstance(step, ir.Choose) and step.order is not None:
            reference = bound[step.order.reference]
            if reference in world.locations and reference not in world.positions:
                problems.append(ChoiceProblem(
                    NO_POSITION, f"the place '{reference}' that '{step.order.mode}' measures from has no position in "
                                 "the world file (world file 1.8 'position'): every candidate would be left out",
                    None, index, label))
    return problems
