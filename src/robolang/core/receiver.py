"""
What a Core 0.1 receiver checks about one message or descriptor (spec/core-v0.1.md section 8).

`check_message` applies the stages in the specified order and stops at the first failure:
form (C101-C107), signature (C201-C204), freshness and recipient (C301-C303, C307), meaning
(C401, C402, C404). Checks that need conversation state - duplicates, ordering, C306, C403 -
belong to the participant that holds that state (ARCH-2), not here.

A message that passes is well-formed, attributable and fresh. It is never permission to act:
whether a robot acts is decided by its local policy (RFC-001 13.2).
"""

from __future__ import annotations
import calendar
import re
from dataclasses import dataclass, field
from typing import Mapping

from .reader import ReadError, read_document
from .schema import DESCRIPTOR_SCHEMA, MESSAGE_SCHEMA, load_schema, validate
from .signatures import SIM_HMAC, Key, verify_sim

MESSAGE = ("robolang.core.message", "0.1")
DESCRIPTOR = ("robolang.core.capability-descriptor", "0.1")
RESERVED_TYPES = frozenset(
    "discover observe subscribe negotiate progress cancel advertise renew release share_observation".split()
)
_TIME = re.compile(r"([0-9]{4})-([0-9]{2})-([0-9]{2})T([0-9]{2}):([0-9]{2}):([0-9]{2})(?:\.([0-9]{1,3}))?Z")


@dataclass(frozen=True)
class Receipt:
    """`accepted`, or `rejected` with a Core code, its stage and the JSON Pointer it concerns."""

    status: str
    code: str | None = None
    stage: str | None = None
    pointer: str = ""

    @property
    def accepted(self) -> bool:
        return self.status == "accepted"


ACCEPTED = Receipt("accepted")


def _reject(code: str, stage: str, pointer: str = "") -> Receipt:
    return Receipt("rejected", code, stage, pointer)


@dataclass(frozen=True)
class Context:
    """The receiver's situation. Times are milliseconds since the Unix epoch, UTC."""

    receiver: str
    now_ms: int
    clock_tolerance_ms: int
    max_message_lifetime_ms: int
    keys: Mapping[str, Key]
    authorities: Mapping[str, str] = field(default_factory=dict)
    mode: str = "simulation"


def time_ms(text: str) -> int:
    """Milliseconds since the epoch of a Core time. Raises ValueError for an impossible date."""
    match = _TIME.fullmatch(text)
    if match is None:
        raise ValueError(f"not a Core time: {text!r}")
    year, month, day, hour, minute, second = (int(g) for g in match.groups()[:6])
    if not (1 <= month <= 12 and 1 <= day <= calendar.monthrange(year, month)[1]
            and hour < 24 and minute < 60 and second < 60):
        raise ValueError(f"not a valid date and time: {text!r}")
    millis = int((match.group(7) or "0").ljust(3, "0"))
    return calendar.timegm((year, month, day, hour, minute, second)) * 1000 + millis


# -------- stage 1: form --------

def _type_profiles() -> dict[str, frozenset[str]]:
    """Each defined type and the profiles it allows, read from the schema's `allOf` rules."""
    out = {}
    for rule in load_schema(MESSAGE_SCHEMA)["allOf"]:
        name = rule["if"]["properties"]["type"]["const"]
        profile = rule["then"]["properties"]["profile"]
        out[name] = frozenset(profile["enum"] if "enum" in profile else [profile["const"]])
    return out


def _times(document) -> list[tuple[str, str]]:
    """Every Core time in the document, with its JSON Pointer."""
    found = []

    def walk(node, pointer):
        if isinstance(node, dict):
            for key, value in node.items():
                walk(value, f"{pointer}/{key}")
        elif isinstance(node, list):
            for i, value in enumerate(node):
                walk(value, f"{pointer}/{i}")
        elif isinstance(node, str) and _TIME.fullmatch(node):
            found.append((pointer, node))

    walk(document, "")
    return found


def _form(document, kind: tuple[str, str], schema_name: str) -> Receipt | None:
    if not isinstance(document, dict) or (document.get("schema"), document.get("schema_version")) != kind:
        return _reject("C102", "form", "/schema_version" if isinstance(document, dict) else "")
    if kind == MESSAGE:
        mtype = document.get("type")
        profiles = _type_profiles()
        if mtype in RESERVED_TYPES:
            return _reject("C106", "form", "/type")
        if mtype not in profiles:
            return _reject("C105", "form", "/type")
        if "signature" not in document:
            return _reject("C201", "signature", "/signature")
        if document.get("profile") not in profiles[mtype]:
            return _reject("C107", "form", "/profile")
        outcome = document.get("payload", {}).get("outcome") if isinstance(document.get("payload"), dict) else None
        if mtype == "outcome" and isinstance(outcome, dict) and outcome.get("status") == "succeeded" \
                and outcome.get("verification") is None:
            return _reject("C402", "meaning", "/payload/outcome/verification")
    errors = validate(document, schema_name)
    if errors:
        unknown = [e for e in errors if e.keyword == "additionalProperties"]
        first = (unknown or errors)[0]
        return _reject("C103" if unknown else "C104", "form", first.pointer)
    for pointer, text in _times(document):
        try:
            time_ms(text)
        except ValueError:
            return _reject("C104", "form", pointer)
    for pointer, lease in _leases(document):
        if time_ms(lease["expires_at"]) <= time_ms(lease["granted_at"]):
            return _reject("C104", "form", f"{pointer}/expires_at")
    return None


def _leases(document) -> list[tuple[str, dict]]:
    """Every lease object (Core 5.7) in the document, with its JSON Pointer."""
    found = []

    def walk(node, pointer):
        if isinstance(node, dict):
            if pointer.endswith("/lease"):
                found.append((pointer, node))
            for key, value in node.items():
                walk(value, f"{pointer}/{key}")
        elif isinstance(node, list):
            for i, value in enumerate(node):
                walk(value, f"{pointer}/{i}")

    walk(document, "")
    return found


# -------- stage 2: signature --------

def _signed_objects(document, signer_field: str) -> list[tuple[dict, str, str]]:
    """The document and every observation it carries, with each one's expected signer."""
    objects = [(document, document[signer_field], "")]
    payload = document.get("payload", {})
    for name in ("observations", "evidence"):
        for i, observation in enumerate(payload.get(name, [])):
            objects.append((observation, observation["source"]["participant"], f"/payload/{name}/{i}"))
    return objects


def _signature(document, signer_field: str, context: Context) -> Receipt | None:
    for obj, signer, pointer in _signed_objects(document, signer_field):
        signature = obj["signature"]
        if signature["alg"] != SIM_HMAC or context.mode != "simulation":
            return _reject("C204", "signature", f"{pointer}/signature/alg")
        key = context.keys.get(signature["key"])
        if key is None or key.participant != signer or key.alg != SIM_HMAC:
            return _reject("C203", "signature", f"{pointer}/signature/key")
        if not verify_sim(obj, key):
            return _reject("C202", "signature", f"{pointer}/signature/value")
    return None


# -------- stage 3: freshness and recipient --------

def _freshness(document, context: Context) -> Receipt | None:
    sent, expires = time_ms(document["sent_at"]), time_ms(document["expires_at"])
    tolerance = context.clock_tolerance_ms
    if expires <= sent or expires - sent > context.max_message_lifetime_ms:
        return _reject("C302", "freshness", "/expires_at")
    if context.now_ms >= expires - tolerance:
        return _reject("C301", "freshness", "/expires_at")
    if sent > context.now_ms + tolerance:
        return _reject("C303", "freshness", "/sent_at")
    if document["recipient"] != context.receiver:
        return _reject("C307", "freshness", "/recipient")
    return None


# -------- stage 5: meaning (stateless part) --------

def _meaning(document, context: Context) -> Receipt | None:
    mtype, payload = document["type"], document["payload"]
    if payload.get("mode") == "physical":
        return _reject("C404", "meaning", "/payload/mode")
    if mtype == "authorization_decision" and context.authorities.get(document["recipient"]) != document["sender"]:
        return _reject("C401", "meaning", "/sender")
    if mtype == "outcome":
        outcome = payload["outcome"]
        verification = outcome["verification"]
        if outcome["status"] == "succeeded" and not all(r["met"] for r in verification["criteria_results"]):
            return _reject("C402", "meaning", "/payload/outcome/verification")
    return None


def check_message(data: bytes, context: Context, meaning: bool = True) -> Receipt:
    """
    The receipt for one message under `context`, applying Core 0.1 section 8 in order.
    Duplicate and order handling (8.4) and state-dependent rules are not applied here: a
    receiver with conversation state passes `meaning=False`, handles duplicates and order
    (stage 4), and only then applies `check_meaning` (stage 5).
    """
    try:
        document = read_document(data)
    except ReadError:
        return _reject("C101", "form")
    return (
        _form(document, MESSAGE, MESSAGE_SCHEMA)
        or _signature(document, "sender", context)
        or _freshness(document, context)
        or (_meaning(document, context) if meaning else None)
        or ACCEPTED
    )


def check_meaning(document: dict, context: Context) -> Receipt:
    """The stateless stage-5 codes (C401, C402 for an unmet criterion, C404) of a message past stage 4."""
    return _meaning(document, context) or ACCEPTED


def check_descriptor(data: bytes, context: Context) -> Receipt:
    """The receipt for a capability descriptor: form, signature and the rules of Core 6."""
    try:
        document = read_document(data)
    except ReadError:
        return _reject("C101", "form")
    rejection = _form(document, DESCRIPTOR, DESCRIPTOR_SCHEMA) or _signature(document, "robot", context)
    if rejection:
        return rejection
    kinds = [c["kind"] for c in document["capabilities"]] + document["unsupported"]
    if len(kinds) != len(set(kinds)):
        return _reject("C104", "form", "/capabilities")
    if time_ms(document["valid_until"]) <= time_ms(document["issued_at"]):
        return _reject("C104", "form", "/valid_until")
    return ACCEPTED


def descriptor_is_current(document: dict, now_ms: int, clock_tolerance_ms: int) -> bool:
    """A descriptor past `valid_until` is treated as absent (Core 6), judged in the safe direction."""
    return now_ms < time_ms(document["valid_until"]) - clock_tolerance_ms
