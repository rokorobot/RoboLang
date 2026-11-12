"""
Physical-run authorization documents (decision 0038 step 2; spec/authorization-v0.1.md).

Three signed documents form a chain: a cell configuration signed by the owner, a task approval
signed by a reviewer, and a run grant signed by an operator. `check_chain` reads all three
strictly and checks every link. Even a valid chain ends in `A731`: physical mode is not enabled
in this release, and nothing here executes anything. Needs the extra `robolang[ed25519]`.
"""

from __future__ import annotations
import base64
import binascii
import hashlib
import json
import re
from collections.abc import Mapping
from dataclasses import dataclass

from .core.canonical import canonical_bytes
from .core.receiver import time_ms
from .core.signatures import ED25519, Key, ed25519_available, verify_ed25519
from .ir import TaskIR, to_dict

VERSION = "0.1"
CELL = "robolang.cell_configuration"
APPROVAL = "robolang.task_approval"
GRANT = "robolang.run_grant"
SAFE_STATE = {"position": "hold", "gripper": "unchanged"}  # the only safe state in 0.1 (0038 P8)
NOT_ENABLED = "A731"

_ID = re.compile(r"rl:[a-z][a-z_]*:[A-Za-z0-9][A-Za-z0-9._:@-]{0,127}")
_KEY_REF = re.compile(_ID.pattern + r"#[A-Za-z0-9._-]{1,64}")
_SHA256 = re.compile(r"[0-9a-f]{64}")
_TEXT = re.compile(r"[ -~]{1,128}")  # printable ASCII

_FIELDS = {
    CELL: {
        "kind": "kind", "version": "version", "id": "id", "robot": "id", "cell": "id",
        "safety_case": {"version": "text", "sha256": "sha256"},
        "commissioning": {"record": "text", "sha256": "sha256"},
        "keys": "keys", "authorities": "ids", "reviewers": "ids",
        "max_grant_lease_s": "seconds", "safe_state": "safe_state",
        "valid_from": "time", "valid_until": "time", "signature": "signature",
    },
    APPROVAL: {
        "kind": "kind", "version": "version", "id": "id", "ir_sha256": "sha256", "task": "text",
        "language_version": "text", "robot": "id", "cell": "id", "author": "text", "reviewer": "id",
        "approved_at": "time", "signature": "signature",
    },
    GRANT: {
        "kind": "kind", "version": "version", "id": "id", "approval_sha256": "sha256", "robot": "id",
        "cell": "id", "operator": "id", "mode": "mode", "granted_at": "time", "expires_at": "time",
        "signature": "signature",
    },
}


class AuthorizationError(Exception):
    """The first broken link. `document` is "cell", "approval" or "grant"; `pointer` a JSON Pointer in it."""

    def __init__(self, code: str, document: str, pointer: str, message: str):
        super().__init__(f"{code} in {document} at {pointer or '(document)'}: {message}")
        self.code = code
        self.document = document
        self.pointer = pointer
        self.message = message


@dataclass(frozen=True)
class Chain:
    """A chain whose every link holds. It still does not authorize a run (`A731`)."""

    cell: dict
    approval: dict
    grant: dict
    approval_sha256: str


def ir_sha256(ir: TaskIR) -> str:
    """The hash a task approval names: SHA-256 of the Task IR in canonical JSON (RFC 8785)."""
    return hashlib.sha256(canonical_bytes(to_dict(ir))).hexdigest()


def document_sha256(document: dict) -> str:
    """SHA-256 of a whole document, signature included, in canonical JSON."""
    return hashlib.sha256(canonical_bytes(document)).hexdigest()


def public_key(text: str) -> bytes:
    """A raw 32-byte ed25519 public key from its unpadded base64url text, or ValueError."""
    if not isinstance(text, str) or len(text) != 43:
        raise ValueError("a public key is 43 base64url characters")
    try:
        raw = base64.urlsafe_b64decode(text + "=")
    except (binascii.Error, ValueError) as exc:
        raise ValueError("not base64url") from exc
    if base64.urlsafe_b64encode(raw).rstrip(b"=").decode() != text:
        raise ValueError("not the canonical encoding")
    return raw


# -------- reading --------

def read(data: bytes | dict, kind: str, name: str) -> dict:
    """Parse and check the form of one document (`A701`, `A702`)."""
    if isinstance(data, dict):
        doc = data
    else:
        try:
            doc = json.loads(data.decode("utf-8"), object_pairs_hook=_no_duplicates, parse_float=_no_float,
                             parse_constant=_no_float)
        except (UnicodeDecodeError, ValueError) as exc:
            raise AuthorizationError("A701", name, "", f"not valid JSON: {exc}") from None
    if not isinstance(doc, dict):
        raise AuthorizationError("A701", name, "", "must be a JSON object")
    if doc.get("kind") != kind or doc.get("version") != VERSION:
        raise AuthorizationError("A701", name, "", f"expected kind {kind!r} and version {VERSION!r}")
    _object(doc, _FIELDS[kind], "", name)
    return doc


def _no_duplicates(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise ValueError(f"duplicate key {key!r}")
        out[key] = value
    return out


def _no_float(text):
    raise ValueError(f"number {text} is not allowed")


def _object(value, fields: dict, pointer: str, name: str):
    if not isinstance(value, dict):
        raise AuthorizationError("A702", name, pointer, "must be an object")
    missing = [f for f in fields if f not in value]
    if missing:
        raise AuthorizationError("A702", name, pointer, "missing " + ", ".join(f"'{f}'" for f in missing))
    for key in value:
        if key not in fields:
            raise AuthorizationError("A702", name, f"{pointer}/{key}", f"unknown field '{key}'")
    for key, form in fields.items():
        p = f"{pointer}/{key}"
        if isinstance(form, dict):
            _object(value[key], form, p, name)
        elif not _FORMS[form](value[key]):
            raise AuthorizationError("A702", name, p, f"not a valid {form.replace('_', ' ')}")


def _is_time(v) -> bool:
    try:
        time_ms(v)
    except (TypeError, ValueError):
        return False
    return True


def _is_keys(v) -> bool:
    if not isinstance(v, dict) or not v:
        return False
    for ref, entry in v.items():
        if not _KEY_REF.fullmatch(ref) or not isinstance(entry, dict) or set(entry) != {"alg", "public_key"}:
            return False
        if entry["alg"] != ED25519:
            return False
        try:
            public_key(entry["public_key"])
        except ValueError:
            return False
    return True


def _owner_prefix(ref: str) -> str:
    return ref.split("#", 1)[0]


_FORMS = {
    "kind": lambda v: True,  # checked in `read`
    "version": lambda v: True,
    "id": lambda v: isinstance(v, str) and _ID.fullmatch(v) is not None,
    "ids": lambda v: isinstance(v, list) and len(v) > 0 and len(set(v)) == len(v)
    and all(isinstance(i, str) and _ID.fullmatch(i) for i in v),
    "text": lambda v: isinstance(v, str) and _TEXT.fullmatch(v) is not None,
    "sha256": lambda v: isinstance(v, str) and _SHA256.fullmatch(v) is not None,
    "seconds": lambda v: type(v) is int and 1 <= v <= 86400,
    "time": _is_time,
    "keys": _is_keys,
    "safe_state": lambda v: v == SAFE_STATE,
    "mode": lambda v: v == "physical",
    "signature": lambda v: isinstance(v, dict) and set(v) == {"alg", "key", "value"}
    and all(isinstance(x, str) for x in v.values()) and _KEY_REF.fullmatch(v["key"]) is not None,
}


# -------- signatures --------

def _signed_by(doc: dict, signer: str, keys: Mapping[str, Key], name: str):
    signature = doc["signature"]
    if signature["alg"] != ED25519:
        raise AuthorizationError("A711", name, "/signature/alg", "only ed25519 is accepted")
    key = keys.get(signature["key"])
    if key is None or key.participant != signer or key.alg != ED25519:
        raise AuthorizationError("A712", name, "/signature/key", f"not a listed key of {signer}")
    if not verify_ed25519(doc, key):
        raise AuthorizationError("A713", name, "/signature/value", "the signature does not verify")


def cell_keys(cell: dict) -> dict[str, Key]:
    return {ref: Key(_owner_prefix(ref), ED25519, public_key=public_key(entry["public_key"]))
            for ref, entry in cell["keys"].items()}


# -------- the chain --------

def _current(start: str, end: str, now_ms: int, tolerance_ms: int) -> bool:
    """Current even if this clock is off by up to the tolerance in either direction (the safe side)."""
    return time_ms(start) + tolerance_ms <= now_ms < time_ms(end) - tolerance_ms


def check_chain(ir: TaskIR, cell_data, approval_data, grant_data, owner_keys: Mapping[str, Key], now_ms: int,
                clock_tolerance_ms: int = 0) -> Chain:
    """Check every link, in order: cell, approval, grant. Raises AuthorizationError for the first broken one.

    `owner_keys` is the trust root installed on the robot out of band: the owner's ed25519 keys.
    Returns the Chain when every link holds; the caller must still refuse to run (`A731`).
    """
    if not ed25519_available():
        raise AuthorizationError("A711", "cell", "/signature/alg", "ed25519 needs pip install 'robolang[ed25519]'")
    cell = read(cell_data, CELL, "cell")
    if not owner_keys or any(k.alg != ED25519 for k in owner_keys.values()):
        raise AuthorizationError("A712", "cell", "/signature/key", "no owner ed25519 key is installed")
    owner = _owner_prefix(cell["signature"]["key"])
    _signed_by(cell, owner, owner_keys, "cell")
    if not _current(cell["valid_from"], cell["valid_until"], now_ms, clock_tolerance_ms):
        raise AuthorizationError("A721", "cell", "/valid_until", "the cell configuration is not current")
    keys = cell_keys(cell)

    approval = read(approval_data, APPROVAL, "approval")
    _signed_by(approval, approval["reviewer"], keys, "approval")
    if approval["task"] not in {task.name for task in ir.tasks}:
        raise AuthorizationError("A722", "approval", "/task", "the IR has no task of that name")
    checks = (("ir_sha256", ir_sha256(ir)), ("language_version", ir.language_version), ("robot", cell["robot"]),
              ("cell", cell["cell"]))
    for field, expected in checks:
        if approval[field] != expected:
            raise AuthorizationError("A722", "approval", f"/{field}", f"does not match: expected {expected}")
    if approval["reviewer"] not in cell["reviewers"]:
        raise AuthorizationError("A723", "approval", "/reviewer", "not a reviewer of this cell")
    if approval["reviewer"] == approval["author"]:
        raise AuthorizationError("A723", "approval", "/reviewer", "the author cannot approve its own task")

    grant = read(grant_data, GRANT, "grant")
    _signed_by(grant, grant["operator"], keys, "grant")
    approval_hash = document_sha256(approval)
    for field, expected in (("approval_sha256", approval_hash), ("robot", cell["robot"]), ("cell", cell["cell"])):
        if grant[field] != expected:
            raise AuthorizationError("A724", "grant", f"/{field}", f"does not match: expected {expected}")
    if grant["operator"] not in cell["authorities"]:
        raise AuthorizationError("A725", "grant", "/operator", "not an authority of this cell")
    granted, expires = time_ms(grant["granted_at"]), time_ms(grant["expires_at"])
    if expires <= granted or expires - granted > cell["max_grant_lease_s"] * 1000:
        raise AuthorizationError("A726", "grant", "/expires_at", "the lease is empty or longer than the cell allows")
    if not _current(grant["granted_at"], grant["expires_at"], now_ms, clock_tolerance_ms):
        raise AuthorizationError("A726", "grant", "/expires_at", "the grant is not current")
    return Chain(cell, approval, grant, approval_hash)


def refuse_physical(chain: Chain) -> AuthorizationError:
    """What every caller does with a valid chain in this release."""
    del chain
    return AuthorizationError(NOT_ENABLED, "grant", "/mode", "physical mode is not enabled in this release")
