"""
The peer's side of `delegate` (spec/delegate-v0.2.md, decision record 0022): pure helpers.

Finding the custody point in a plan, the criteria and the observation of the custody exchange, and the
lease option. Nothing here sends a message; `robolang.delegate` does.
"""

from __future__ import annotations
import re
from dataclasses import dataclass
from decimal import Decimal

from . import bridge, ir
from .core import Key, sign_sim
from .export_goal import DEFAULT_MAX_AGE, DEFAULT_MIN_CONFIDENCE, simulation_key
from .sim.outcome import OBSERVATION_VALID_FOR

DEFAULT_LEASE_MS = 60_000
MAX_LEASE_MS = 86_400_000  # a lease is never open-ended (RFC-001 11.1); a day is the longest this command offers
_LEASE = re.compile(r"([0-9]+(?:\.[0-9]+)?)(ms|s|min)")
_UNIT_MS = {"ms": Decimal(1), "s": Decimal(1000), "min": Decimal(60_000)}


@dataclass(frozen=True)
class CustodyPoint:
    step: int  # the plan step (from 1) where the peer grasps the thing
    thing: str  # the object's world ID
    place: str  # the location where the robot placed it: the handoff place


def lease_ms(text: str) -> int:
    """`--lease` as whole milliseconds: a number and `ms`, `s` or `min`, between 1 ms and a day. Raises ValueError."""
    match = _LEASE.fullmatch(text)
    value = None if match is None else Decimal(match.group(1)) * _UNIT_MS[match.group(2)]
    if value is None or value != value.to_integral_value() or not 1 <= value <= MAX_LEASE_MS:
        raise ValueError(f"invalid lease '{text}': use a duration such as 60s, 500ms or 2min, from 1ms to 24h")
    return int(value)


def custody_point(task: ir.Task, bindings: dict[str, str], robot: str, peer: str) -> CustodyPoint | None:
    """The first step where `peer` grasps an object that `robot` placed earlier in the plan, or None."""
    placed: dict[str, str] = {}
    for index, step in enumerate(task.steps, start=1):
        if isinstance(step, ir.Place) and bindings[step.robot] == robot:
            placed[bindings[step.obj]] = bindings[step.target]
        elif isinstance(step, ir.Grasp) and bindings[step.robot] == peer and bindings[step.obj] in placed:
            thing = bindings[step.obj]
            return CustodyPoint(index, thing, placed[thing])
    return None


def handoff_criteria(point: CustodyPoint, peer: str) -> list[dict]:
    """What the peer's confirming observation must meet: the thing is located_in the handoff place, seen by the peer."""
    return [{
        "kind": "observed",
        **bridge.Triple(bridge.core_id("object", point.thing), "located_in",
                        bridge.core_id("location", point.place)).as_dict(),
        "min_confidence": DEFAULT_MIN_CONFIDENCE,
        "max_age": dict(DEFAULT_MAX_AGE),
        "accepted_sources": [peer],
    }]


def peer_observation(identifier: str, triple: bridge.Triple, observed_at: str, peer: str) -> dict:
    """The peer's own signed observation of the thing (simulator ground truth, signed with the peer's key)."""
    key_ref = f"{peer}#k1"
    return sign_sim({
        "kind": "observation",
        "id": identifier,
        **triple.as_dict(),
        "observed_at": observed_at,
        "valid_for": dict(OBSERVATION_VALID_FOR),
        "confidence": "1",
        "source": {"participant": peer, "sensor": "world", "method": "simulation_ground_truth"},
    }, key_ref, simulation_key(key_ref))


def trust_keys(planner: str, robot: str, peer: str | None, sim_key_ref: str, sim_key: Key) -> dict[str, dict[str, Key]]:
    """The keys each participant verifies with: its counterparts' only (0013), the planner also the simulator's."""
    def key(participant: str) -> dict[str, Key]:
        ref = f"{participant}#k1"
        return {ref: simulation_key(ref)}

    trust = {planner: key(robot) | {sim_key_ref: sim_key}, robot: key(planner)}
    if peer is not None:
        trust[robot] |= key(peer)
        trust[peer] = key(robot)
    return trust
