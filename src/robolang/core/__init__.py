"""
RoboLang Core 0.1 document checks (ARCH-1; spec/core-v0.1.md).

Strict JSON reading, the Core 0.1 schemas, RFC 8785 canonical form, `sim-hmac-sha256`
signatures and the per-message receiver checks with their stable codes.

Nothing here sends, receives or executes anything. It imports no network, adapter or
ROS 2 module and nothing from the RoboLang 1.x parser, IR or `robolang.sim` (decision R13).
`sim-hmac-sha256` is simulation only: it is never an identity proof.
"""

from __future__ import annotations

from .canonical import canonical_bytes
from .reader import ReadError, read_document
from .receiver import (
    ACCEPTED,
    DESCRIPTOR,
    MESSAGE,
    RESERVED_TYPES,
    Context,
    Receipt,
    check_descriptor,
    check_message,
    descriptor_is_current,
    time_ms,
)
from .schema import DESCRIPTOR_SCHEMA, MESSAGE_SCHEMA, SchemaError, load_schema, validate
from .signatures import ED25519, SIM_HMAC, Key, sign_sim, signed_bytes, verify_sim

__all__ = [
    "ACCEPTED",
    "DESCRIPTOR",
    "DESCRIPTOR_SCHEMA",
    "ED25519",
    "MESSAGE",
    "MESSAGE_SCHEMA",
    "RESERVED_TYPES",
    "SIM_HMAC",
    "Context",
    "Key",
    "ReadError",
    "Receipt",
    "SchemaError",
    "canonical_bytes",
    "check_descriptor",
    "check_message",
    "descriptor_is_current",
    "load_schema",
    "read_document",
    "sign_sim",
    "signed_bytes",
    "time_ms",
    "validate",
    "verify_sim",
]
