"""
Exact values for the simulation (spec/simulation-v1.0.md sections 3.7 and 4.2).

Three-valued truth, decimal parsing and duration arithmetic. Decimal only, never float:
2.5 s is exactly 2500 ms.
"""

from __future__ import annotations
import re
from dataclasses import dataclass
from decimal import Decimal
from enum import Enum

from .. import ir

_DECIMAL = re.compile(r"[0-9]+(\.[0-9]+)?")

DURATION_UNITS = {"ms": 1, "s": 1000, "min": 60000}
SPEED_UNITS = ("m/s", "rad/s")


class Truth(Enum):
    TRUE = "true"
    FALSE = "false"
    UNKNOWN = "unknown"

    @staticmethod
    def of(value: bool | None) -> Truth:
        """None is unknown. An unknown fact is never treated as true."""
        if value is None:
            return Truth.UNKNOWN
        return Truth.TRUE if value else Truth.FALSE


def parse_decimal(text: str) -> Decimal:
    """Digits with an optional fractional part: no sign, no exponent, no spaces."""
    if not isinstance(text, str) or _DECIMAL.fullmatch(text) is None:
        raise ValueError(f"not a decimal: {text!r}")
    return Decimal(text)


def duration_ms(value: Decimal, unit: str) -> Decimal:
    """A duration in milliseconds, exactly."""
    if unit not in DURATION_UNITS:
        raise ValueError(f"not a duration unit: {unit!r}")
    return value * DURATION_UNITS[unit]


def duration_ms_of(q: ir.Quantity) -> Decimal:
    return duration_ms(q.value, q.unit)


def ms_text(value: Decimal) -> str:
    """Canonical text of a millisecond count: "2500", "0.5", "0"."""
    return ir.decimal_text(value)


@dataclass(frozen=True)
class Speed:
    value: Decimal
    unit: str
