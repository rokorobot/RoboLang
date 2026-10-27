"""
JSON Canonicalization Scheme, RFC 8785 (spec/core-v0.1.md section 7).

Object keys are sorted by their UTF-16 code units, there is no whitespace, strings use the
shortest JSON escapes, and numbers use the ECMAScript number-to-string algorithm. Core
documents contain integers only, but the general number rule is implemented so that the
RFC's own test vectors apply.
"""

from __future__ import annotations
import math
from decimal import Decimal

_ESCAPES = {'"': '\\"', "\\": "\\\\", "\b": "\\b", "\f": "\\f", "\n": "\\n", "\r": "\\r", "\t": "\\t"}


def _string(value: str) -> str:
    out = ['"']
    for ch in value:
        if ch in _ESCAPES:
            out.append(_ESCAPES[ch])
        elif ord(ch) < 0x20:
            out.append(f"\\u{ord(ch):04x}")
        else:
            out.append(ch)
    out.append('"')
    return "".join(out)


def _number(value: int | float) -> str:
    if isinstance(value, int):
        return str(value)
    if not math.isfinite(value):
        raise ValueError("JCS has no representation for NaN or Infinity")
    if value == 0:
        return "0"  # also -0
    sign = "-" if value < 0 else ""
    # Shortest round-trip digits, as ECMAScript requires; Python's repr produces them.
    digits_tuple = Decimal(repr(abs(value))).normalize().as_tuple()
    digits = "".join(map(str, digits_tuple.digits))
    k = len(digits)
    n = digits_tuple.exponent + k  # position of the decimal point
    if k <= n <= 21:
        text = digits + "0" * (n - k)
    elif 0 < n <= 21:
        text = digits[:n] + "." + digits[n:]
    elif -6 < n <= 0:
        text = "0." + "0" * (-n) + digits
    else:
        exponent = n - 1
        mantissa = digits[0] + ("." + digits[1:] if k > 1 else "")
        text = f"{mantissa}e{'+' if exponent > 0 else '-'}{abs(exponent)}"
    return sign + text


def _utf16_key(key: str) -> bytes:
    return key.encode("utf-16-be")


def _serialize(value, out: list[str]) -> None:
    if value is None:
        out.append("null")
    elif value is True:
        out.append("true")
    elif value is False:
        out.append("false")
    elif isinstance(value, (int, float)):
        out.append(_number(value))
    elif isinstance(value, str):
        out.append(_string(value))
    elif isinstance(value, (list, tuple)):
        out.append("[")
        for i, item in enumerate(value):
            if i:
                out.append(",")
            _serialize(item, out)
        out.append("]")
    elif isinstance(value, dict):
        out.append("{")
        for i, key in enumerate(sorted(value, key=_utf16_key)):
            if i:
                out.append(",")
            out.append(_string(key))
            out.append(":")
            _serialize(value[key], out)
        out.append("}")
    else:
        raise TypeError(f"not a JSON value: {type(value).__name__}")


def canonical_bytes(value) -> bytes:
    """The RFC 8785 canonical UTF-8 bytes of a JSON value."""
    out: list[str] = []
    _serialize(value, out)
    return "".join(out).encode("utf-8")
