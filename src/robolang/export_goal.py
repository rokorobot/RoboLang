"""
`export-goal`: a RoboLang 1.3 task's goal as a signed Core 0.1 `propose_goal` message
(spec/export-goal-v0.1.md, decision record 0019).

The goal triples and identifiers come from `bridge`; this module adds the envelope, the success
criteria, the constraints, the things and the simulation signature. Nothing is sent: the message is
built and returned. SIMULATION ONLY: the signature is `sim-hmac-sha256` under a published key.
"""

from __future__ import annotations
import hashlib
import re
from dataclasses import dataclass

from . import bridge, ir
from .core import SIM_HMAC, Key, canonical_bytes, sign_sim
from .sim import World

DEFAULT_MIN_CONFIDENCE = "0.9"
DEFAULT_MAX_AGE = {"value": "10", "unit": "s"}
ACCEPTED_SOURCES = ["any_participant_in_domain"]
MESSAGE_LIFETIME_MS = 30_000
SIM_KEY_LABEL = b"robolang export-goal simulation key 1:"
PARTICIPANT = re.compile(r"rl:[a-z][a-z_]*:[A-Za-z0-9][A-Za-z0-9._:@-]{0,127}")  # an identifier without '#'


def simulation_key(key_ref: str) -> Key:
    """
    The fixed key of a sender, `<participant>#k1`. Its secret is the SHA-256 of a published label
    and the key reference, so anyone can compute it: it shows that a check happens, never who signed.
    """
    participant = key_ref.partition("#")[0]
    return Key(participant, SIM_HMAC, hashlib.sha256(SIM_KEY_LABEL + key_ref.encode("utf-8")).digest())


class ExportError(Exception):
    """The goal cannot be exported. `code` is X601 (no Core form) or X602 (a thing lacks class or mass)."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class Exported:
    message: dict
    warnings: tuple[str, ...]


def valid_participant(text: str) -> bool:
    return PARTICIPANT.fullmatch(text) is not None


def default_recipient(task: ir.Task, bindings: dict[str, str]) -> str | None:
    """The Core ID of the only robot parameter's robot; None when the task has none or several."""
    robots = [p.name for p in task.parameters if p.type == "robot"]
    return bridge.core_id("robot", bindings[robots[0]]) if len(robots) == 1 else None


def _criterion(triple: bridge.Triple, verify: ir.Verify | None) -> dict:
    confidence = verify.min_confidence if verify else None
    max_age = verify.max_age if verify else None
    return {
        "kind": "observed",
        **triple.as_dict(),
        "min_confidence": DEFAULT_MIN_CONFIDENCE if confidence is None else ir.decimal_text(confidence),
        "max_age": dict(DEFAULT_MAX_AGE) if max_age is None else
        {"value": ir.decimal_text(max_age.value), "unit": max_age.unit},
        "accepted_sources": list(ACCEPTED_SOURCES),
    }


def _constraints(task: ir.Task) -> tuple[list[dict], list[str]]:
    """`max_speed` limits in m/s become constraints; the others have no Core form and give a warning."""
    constraints, warnings = [], []
    for limit in task.safety:
        if limit.limit.unit == "m/s":
            speed = {"value": ir.decimal_text(limit.limit.value), "unit": "m/s"}
            constraints.append({"kind": "max_speed", "limit": speed})
        else:
            shown = f"{ir.decimal_text(limit.limit.value)} {limit.limit.unit or '(no unit)'}"
            warnings.append(f"skipped {limit.kind} {shown}: Core 0.1 has only m/s limits")
    return constraints, warnings


def _things(task: ir.Task, world: World, bindings: dict[str, str]) -> list[dict]:
    things: dict[str, dict] = {}
    for parameter in task.parameters:
        if parameter.type != "object":
            continue
        entity = world.objects[bindings[parameter.name]]
        missing = [name for name, value in (("class", entity.object_class), ("mass", entity.mass)) if value is None]
        if missing:
            raise ExportError(
                "X602",
                f"object '{entity.id}' (parameter '{parameter.name}') has no {' and no '.join(missing)} in the world "
                "file; a Core thing needs both (world_schema_version 1.1)",
            )
        thing_id = bridge.core_id("object", entity.id)
        things[thing_id] = {
            "kind": "thing",
            "id": thing_id,
            "class": entity.object_class,
            "mass": {
                "value": ir.decimal_text(entity.mass.value),
                "unit": entity.mass.unit,
                "uncertainty": ir.decimal_text(entity.mass.uncertainty),
            },
        }
    return list(things.values())


def _hash(*parts: object) -> str:
    return hashlib.sha256(canonical_bytes(list(parts))).hexdigest()


CORE_VERSIONS = ("0.1", "0.2")


def _goal_01(task: ir.Task, bindings: dict[str, str]) -> tuple[bridge.Triple, ...]:
    """The triples of a Core 0.1 goal; a goal that needs two or more alternatives says to use --core 0.2 (C8)."""
    try:
        return bridge.goal_triples(task, bindings)
    except bridge.PathsDiffer:
        raise  # its message already says to use --core 0.2
    except bridge.NotExportable as error:
        try:
            count = len(bridge.goal_alternatives(task, bindings))
        except bridge.NotExportable:
            raise error from None
        raise error if count < 2 else bridge.NeedsCore02(error, count) from None


def export_goal(task: ir.Task, world: World, *, sender: str, recipient: str | None = None,
                sent_at: str | None = None, core: str = "0.1") -> Exported:
    """
    The signed `propose_goal` message for `task` bound as in `world.task`. `recipient` defaults to the
    task's only robot. `core` is the message version: "0.1" (default) or "0.2", whose goal carries one
    alternative per path (spec/core-v0.2.md; stable order: depth-first, `then` before `else`).
    Raises ExportError, or bridge.NotExportable for a goal without a Core form for that version.
    """
    if core not in CORE_VERSIONS:
        raise ValueError(f"unknown Core version '{core}': use one of {', '.join(CORE_VERSIONS)}")
    bindings = dict(world.task.bindings)
    alternatives = bridge.alternative_triples(task, bindings) if core == "0.2" else ()
    triples = () if core == "0.2" else _goal_01(task, bindings)
    if recipient is None:
        recipient = default_recipient(task, bindings)
    if recipient is None:
        raise ValueError("the task does not have exactly one robot parameter: name the recipient")
    sent_at = sent_at or bridge.core_time(0)
    constraints, warnings = _constraints(task)
    goal = {
        "kind": "goal",
        "id": f"rl:goal:{task.name}",
        "requested_by": sender,
        **({"alternatives": [{"desired_state": [t.as_dict() for t in alt],
                              "success_criteria": [_criterion(t, task.verify) for t in alt]} for alt in alternatives]}
           if core == "0.2" else
           {"desired_state": [t.as_dict() for t in triples],
            "success_criteria": [_criterion(t, task.verify) for t in triples]}),
        "constraints": constraints,
        "priority": "normal",
    }
    payload = {"goal": goal, "things": _things(task, world, bindings)}
    digest = _hash(sender, recipient, payload)
    message = {
        "schema": "robolang.core.message",
        "schema_version": core,
        "message_id": f"m-{digest[:32]}",
        "conversation_id": f"rl:conv:{digest[32:48]}",
        "sequence": 1,
        "in_reply_to": None,
        "profile": "a2r",
        "type": "propose_goal",
        "sender": sender,
        "recipient": recipient,
        "sent_at": sent_at,
        "expires_at": bridge.core_time_after(sent_at, MESSAGE_LIFETIME_MS),
        "payload": payload,
    }
    key_ref = f"{sender}#k1"
    return Exported(sign_sim(message, key_ref, simulation_key(key_ref)), tuple(warnings))
