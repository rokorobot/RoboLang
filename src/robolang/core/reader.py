"""
Strict JSON reading for Core 0.1 (spec/core-v0.1.md section 2.3).

UTF-8 without a byte order mark, no duplicate keys, integers only. A JSON Schema validator
cannot enforce the integer rule (its `integer` type accepts 1.0), so it is enforced here.
"""

from __future__ import annotations
import json


class ReadError(ValueError):
    """
    The bytes are not a Core JSON document (C101).

    `reason` says which rule broke, for readers that report it more finely (the Task IR reader):
    "encoding", "json", "duplicate_key", "number" or "depth". It is not part of the message.
    """

    def __init__(self, message: str, reason: str = "json"):
        super().__init__(message)
        self.reason = reason


def _strict_object(pairs):
    seen = set()
    for key, _ in pairs:
        if key in seen:
            raise ReadError(f"duplicate key {key!r}", "duplicate_key")
        seen.add(key)
    return dict(pairs)


def _no_float(text):
    raise ReadError(f"non-integer number {text}", "number")


def _no_constant(text):
    raise ReadError(f"{text} is not JSON")


def read_document(data: bytes):
    """Parse `data` as a Core JSON document. Raises ReadError for anything C101 covers."""
    if not isinstance(data, (bytes, bytearray)):
        raise TypeError("read_document needs bytes")
    if data.startswith(b"\xef\xbb\xbf"):
        raise ReadError("byte order mark", "encoding")
    try:
        text = bytes(data).decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ReadError(f"not UTF-8: {exc.reason}", "encoding") from None
    try:
        return json.loads(
            text, object_pairs_hook=_strict_object, parse_float=_no_float, parse_constant=_no_constant
        )
    except ReadError:
        raise
    except RecursionError:
        raise ReadError("nested too deeply", "depth") from None
    except ValueError as exc:
        raise ReadError(f"not JSON: {exc}") from None
