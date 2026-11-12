"""
Signatures for Core 0.1 (spec/core-v0.1.md section 7).

`sim-hmac-sha256` is SIMULATION ONLY: the verifier holds the same secret as the signer, so a
valid signature shows that a check happened, never who signed.

`ed25519` (RFC 8032; `value` is the unpadded base64url signature) needs the optional extra
`robolang[ed25519]`, which installs `cryptography` (decision 0038; the core wheel stays free of
runtime dependencies, decision 0007 R5). A receiver accepts it only when its context lists it.
"""

from __future__ import annotations
import base64
import binascii
import hashlib
import hmac
from dataclasses import dataclass

from .canonical import canonical_bytes

SIM_HMAC = "sim-hmac-sha256"
ED25519 = "ed25519"


@dataclass(frozen=True)
class Key:
    """A configured key: owner, algorithm, and its sim-hmac-sha256 secret or raw 32-byte ed25519 public key."""

    participant: str
    alg: str
    secret: bytes = b""
    public_key: bytes = b""


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


# -------- ed25519 (optional extra) --------

def ed25519_available() -> bool:
    """True when the optional `cryptography` package is installed."""
    try:
        from cryptography.hazmat.primitives.asymmetric import ed25519  # noqa: F401
    except ImportError:
        return False
    return True


def _ed25519():
    try:
        from cryptography.hazmat.primitives.asymmetric import ed25519
    except ImportError as exc:
        raise RuntimeError("ed25519 needs the optional extra: pip install 'robolang[ed25519]'") from exc
    return ed25519


def _b64url(data: bytes) -> str:
    return base64.urlsafe_b64encode(data).rstrip(b"=").decode("ascii")


def generate_ed25519() -> tuple[bytes, bytes]:
    """A new key pair as raw bytes: (32-byte private seed, 32-byte public key)."""
    private = _ed25519().Ed25519PrivateKey.generate()
    from cryptography.hazmat.primitives import serialization as ser
    seed = private.private_bytes(ser.Encoding.Raw, ser.PrivateFormat.Raw, ser.NoEncryption())
    return seed, private.public_key().public_bytes(ser.Encoding.Raw, ser.PublicFormat.Raw)


def sign_ed25519(document: dict, key_ref: str, private_seed: bytes) -> dict:
    """A copy of `document` signed with ed25519; `private_seed` is the raw 32-byte private key."""
    body = {k: v for k, v in document.items() if k != "signature"}
    signer = _ed25519().Ed25519PrivateKey.from_private_bytes(private_seed)
    return {**body, "signature": {"alg": ED25519, "key": key_ref, "value": _b64url(signer.sign(signed_bytes(body)))}}


def verify_ed25519(document: dict, key: Key) -> bool:
    """False for a wrong, malformed or non-canonical signature value, or a malformed public key."""
    from cryptography.exceptions import InvalidSignature
    value = document["signature"]["value"]
    if not isinstance(value, str) or len(value) != 86 or "=" in value:
        return False
    try:
        raw = base64.urlsafe_b64decode(value + "==")
        if _b64url(raw) != value:  # one encoding only: no stray bits in the last character
            return False
        _ed25519().Ed25519PublicKey.from_public_bytes(key.public_key).verify(raw, signed_bytes(document))
    except (InvalidSignature, ValueError, binascii.Error):
        return False
    return True
