"""
Signatures for Core 0.1 (spec/core-v0.1.md section 7).

Only `sim-hmac-sha256` is implemented. It is SIMULATION ONLY: the verifier holds the same
secret as the signer, so a valid signature shows that a check happened, never who signed.
`ed25519` is defined by the specification but not implemented here (decision R5).
"""

from __future__ import annotations
import hashlib
import hmac
from dataclasses import dataclass

from .canonical import canonical_bytes

SIM_HMAC = "sim-hmac-sha256"
ED25519 = "ed25519"


@dataclass(frozen=True)
class Key:
    """A configured key: who owns it, its algorithm and, for sim-hmac-sha256, its secret."""

    participant: str
    alg: str
    secret: bytes


def signed_bytes(document: dict) -> bytes:
    """The bytes a signature covers: the object without its `signature` field, in JCS form."""
    return canonical_bytes({k: v for k, v in document.items() if k != "signature"})


def sim_hmac(document: dict, secret: bytes) -> str:
    return hmac.new(secret, signed_bytes(document), hashlib.sha256).hexdigest()


def sign_sim(document: dict, key_ref: str, key: Key) -> dict:
    """A copy of `document` signed with sim-hmac-sha256 (for fixtures and the simulation)."""
    if key.alg != SIM_HMAC:
        raise ValueError(f"cannot sign with {key.alg}")
    body = {k: v for k, v in document.items() if k != "signature"}
    return {**body, "signature": {"alg": SIM_HMAC, "key": key_ref, "value": sim_hmac(body, key.secret)}}


def verify_sim(document: dict, key: Key) -> bool:
    expected = sim_hmac(document, key.secret)
    return hmac.compare_digest(str(document["signature"]["value"]), expected)
