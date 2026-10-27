"""
The symbolic world and its sensors (spec/reference-simulation-v0.1.md section 4).

Facts are Core triples keyed by (subject, property). A thing that is `carried_by` a robot is
`located_in` that robot's place; the world derives it. The world changes only through action
effects, and every action starts with a motion event (decision R8).
"""

from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class Fact:
    subject: str
    property: str
    value: str


class World:
    def __init__(self, initial_facts: list[dict]):
        self._facts: dict[tuple[str, str], str] = {}
        for fact in initial_facts:
            self._facts[(fact["subject"], fact["property"])] = fact["value"]

    def get(self, subject: str, prop: str) -> str | None:
        if prop == "located_in" and (subject, "carried_by") in self._facts:
            return self._facts.get((self._facts[(subject, "carried_by")], "located_in"))
        return self._facts.get((subject, prop))

    def set(self, subject: str, prop: str, value: str) -> None:
        self._facts[(subject, prop)] = value
        if prop == "carried_by":
            self._facts.pop((subject, "located_in"), None)  # derived from the carrier from now on

    def carried_by(self, robot: str) -> list[str]:
        return sorted(s for (s, p), v in self._facts.items() if p == "carried_by" and v == robot)

    def facts(self) -> tuple[Fact, ...]:
        """All facts, derived locations included, sorted."""
        out = {(s, p): v for (s, p), v in self._facts.items()}
        for (subject, prop), _ in list(self._facts.items()):
            if prop == "carried_by":
                location = self.get(subject, "located_in")
                if location is not None:
                    out[(subject, "located_in")] = location
        return tuple(Fact(s, p, v) for (s, p), v in sorted(out.items()))


@dataclass
class Sensor:
    participant: str
    name: str
    scope: str  # "places" or "own_deck"
    places: tuple[str, ...]
    valid_for: dict
    confidence: str
    method: str
    lag_ms: int = 0
    unavailable_from_ms: int | None = None

    def available(self, now_ms: int) -> bool:
        return self.unavailable_from_ms is None or now_ms < self.unavailable_from_ms

    def sees(self, world: World, thing: str) -> bool:
        if self.scope == "own_deck":
            return world.get(thing, "carried_by") == self.participant
        return world.get(thing, "located_in") in self.places
