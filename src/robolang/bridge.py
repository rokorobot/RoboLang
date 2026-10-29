"""
The mapping from RoboLang 1.3 tasks to RoboLang Core 0.1 (docs/plans/goals-and-core-bridge.md).

Shared by the Core-shaped outcome of a simulated run and by `export-goal`, so that both give the
same identifiers and the same triples for one goal.

- A world entity ID becomes a Core identifier by prefix: object `box1` -> `rl:thing:box1`,
  location `table` -> `rl:place:table`, robot `arm1` -> `rl:robot:arm1` (plan decision Q5).
- A goal is exportable when it is a conjunction of outcome atoms: `object_at` and `robot_at`
  -> `located_in`, `holding` -> `carried_by` (Core 0.1 section 5.2). Top-level goal entries
  and explicit `and` operands have the same meaning and are flattened in source order. Any
  other condition (`or`, `not`, `robot_ready`, ...) is valid locally but has no Core form
  (plan decision Q3). Flattened outcome atom i is Core success criterion i.
- Simulated time t (ms from the start of the run) becomes a Core time counted from SIM_EPOCH.
"""

from __future__ import annotations
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from decimal import ROUND_FLOOR, Decimal

from . import ir
from .core import time_ms

CORE_KIND = {"robot": "robot", "object": "thing", "location": "place", "region": "region", "sensor": "sensor"}
SIM_EPOCH = datetime(2000, 1, 1, tzinfo=timezone.utc)  # Core time of simulated t = 0


def core_id(param_type: str, entity_id: str) -> str:
    """The Core identifier of a world entity bound to a parameter of `param_type`."""
    return f"rl:{CORE_KIND[param_type]}:{entity_id}"


def core_time(t_ms: Decimal | int) -> str:
    """Simulated time as a Core 0.1 time (RFC 3339, UTC, millisecond precision, rounded down)."""
    ms = int(Decimal(t_ms).to_integral_value(rounding=ROUND_FLOOR))
    t = SIM_EPOCH + timedelta(milliseconds=ms)
    return t.strftime("%Y-%m-%dT%H:%M:%S.") + f"{t.microsecond // 1000:03d}Z"


def core_time_after(time: str, delta_ms: int) -> str:
    """A Core time `delta_ms` after the Core time `time`, in the millisecond form of `core_time`."""
    total = time_ms(time) + delta_ms
    t = datetime.fromtimestamp(total // 1000, tz=timezone.utc)
    return t.strftime("%Y-%m-%dT%H:%M:%S.") + f"{total % 1000:03d}Z"


@dataclass(frozen=True)
class Triple:
    subject: str
    property: str  # located_in or carried_by
    value: str

    def as_dict(self) -> dict:
        return {"subject": self.subject, "property": self.property, "value": self.value}


class NotExportable(ValueError):
    """The goal has a condition without a Core 0.1 form. `index` counts goal conditions from 1."""

    def __init__(self, index: int, kind: str):
        super().__init__(f"goal condition {index} ('{kind}') has no RoboLang Core 0.1 form")
        self.index = index
        self.kind = kind


def triple_of(condition: ir.Condition, bindings: dict[str, str]) -> Triple | None:
    """The Core triple an outcome atom states, with parameters replaced by bound IDs; None for other conditions."""
    if isinstance(condition, ir.ObjectAt):
        return Triple(core_id("object", bindings[condition.obj]), "located_in",
                      core_id("location", bindings[condition.location]))
    if isinstance(condition, ir.RobotAt):
        return Triple(core_id("robot", bindings[condition.robot]), "located_in",
                      core_id("location", bindings[condition.location]))
    if isinstance(condition, ir.Holding):
        return Triple(core_id("object", bindings[condition.obj]), "carried_by",
                      core_id("robot", bindings[condition.robot]))
    return None


def goal_atoms(task: ir.Task) -> tuple[ir.Condition, ...]:
    """The goal's outcome atoms in semantic order, flattening conjunctions; reject every other form."""
    if task.goal is None:
        raise NotExportable(0, "no goal")

    def flatten(condition: ir.Condition):
        if isinstance(condition, ir.And):
            for child in condition.conditions:
                yield from flatten(child)
        else:
            yield condition

    atoms = tuple(atom for condition in task.goal for atom in flatten(condition))
    for index, condition in enumerate(atoms, start=1):
        if not isinstance(condition, (ir.ObjectAt, ir.RobotAt, ir.Holding)):
            raise NotExportable(index, condition.KIND)
    return atoms


def goal_triples(task: ir.Task, bindings: dict[str, str]) -> tuple[Triple, ...]:
    """One triple per flattened goal atom, in order. Raise NotExportable without a Core form."""
    triples = []
    for condition in goal_atoms(task):
        triple = triple_of(condition, bindings)
        assert triple is not None  # goal_atoms admits only the three mapped outcome atom types
        triples.append(triple)
    return tuple(triples)
