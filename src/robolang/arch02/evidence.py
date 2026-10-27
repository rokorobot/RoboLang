"""
Evaluating success criteria against observations (spec/reference-simulation-v0.1.md 6.3;
RFC-001 11.3).

An observation counts for a criterion only if its source is accepted, its content matches,
its confidence is high enough, and it is within both its own `valid_for` and the criterion's
`max_age` at evaluation time. Ages are taken in the safe direction: the clock tolerance is
added, so an observation is treated as older (RFC-001 11.4).
"""

from __future__ import annotations
from dataclasses import dataclass
from decimal import Decimal

from ..core import time_ms
from .scenario import ms

MET, STALE, MISSING, CONTRADICTED = "met", "stale", "missing", "contradicted"


@dataclass(frozen=True)
class Evaluation:
    result: str  # met, stale, missing or contradicted
    evidence: tuple[str, ...]  # observation IDs that meet the criterion

    @property
    def reason(self) -> str | None:
        return {STALE: "evidence_stale", MISSING: "evidence_missing"}.get(self.result)


def evaluate(criterion: dict, observations: list[dict], now_ms: int, tolerance_ms: int,
             domain_members: frozenset[str]) -> Evaluation:
    sources = set(criterion["accepted_sources"])
    accepted = [
        o for o in observations
        if o["source"]["participant"] in sources
        or ("any_participant_in_domain" in sources and o["source"]["participant"] in domain_members)
    ]
    relevant = [o for o in accepted if (o["subject"], o["property"]) == (criterion["subject"], criterion["property"])]
    if not relevant:
        return Evaluation(MISSING, ())
    fresh_enough = []
    for o in relevant:
        age = now_ms - time_ms(o["observed_at"]) + tolerance_ms
        if age <= ms(criterion["max_age"]) and age <= ms(o["valid_for"]):
            fresh_enough.append(o)
    if not fresh_enough:
        return Evaluation(STALE, ())
    meeting = [
        o for o in fresh_enough
        if o["value"] == criterion["value"] and Decimal(o["confidence"]) >= Decimal(criterion["min_confidence"])
    ]
    if meeting:
        return Evaluation(MET, tuple(o["id"] for o in meeting))
    return Evaluation(CONTRADICTED, ())
