"""Tests for the exact simulation values (src/robolang/sim/values.py)."""

from __future__ import annotations
from decimal import Decimal

import pytest

from robolang.ir import Quantity
from robolang.sim.values import (
    DURATION_UNITS,
    SPEED_UNITS,
    Speed,
    Truth,
    duration_ms,
    duration_ms_of,
    ms_text,
    parse_decimal,
)


def test_truth_of():
    assert Truth.of(True) is Truth.TRUE
    assert Truth.of(False) is Truth.FALSE
    assert Truth.of(None) is Truth.UNKNOWN
    assert [t.value for t in Truth] == ["true", "false", "unknown"]


@pytest.mark.parametrize("text", ["0", "2.5", "007"])
def test_parse_decimal_accepts(text):
    assert parse_decimal(text) == Decimal(text)
    assert type(parse_decimal(text)) is Decimal


@pytest.mark.parametrize("text", ["-1", "+1", "1e3", "", "1.", ".5", " 1", "1 ", "1,5", "NaN"])
def test_parse_decimal_rejects(text):
    with pytest.raises(ValueError):
        parse_decimal(text)


def test_units():
    assert DURATION_UNITS == {"ms": 1, "s": 1000, "min": 60000}
    assert SPEED_UNITS == ("m/s", "rad/s")
    assert Speed(Decimal("1"), "m/s").unit == "m/s"


def test_duration_is_exact():
    assert duration_ms(Decimal("2.5"), "s") == 2500
    assert duration_ms(Decimal("0.5"), "ms") == Decimal("0.5")
    assert duration_ms(Decimal("1"), "min") == 60000
    assert duration_ms(Decimal("2500"), "ms") == duration_ms(Decimal("2.5"), "s")
    assert duration_ms(Decimal("0.1"), "s") == Decimal("100")
    assert type(duration_ms(Decimal("2.5"), "s")) is Decimal


def test_duration_rejects_other_units():
    with pytest.raises(ValueError):
        duration_ms(Decimal("1"), "m/s")


def test_duration_of_quantity():
    result = duration_ms_of(Quantity(Decimal("2.5"), "s"))
    assert result == 2500 and type(result) is Decimal


def test_ms_text_is_canonical():
    assert ms_text(duration_ms(Decimal("2.5"), "s")) == "2500"
    assert ms_text(Decimal("0.5")) == "0.5"
    assert ms_text(Decimal("0")) == "0"
    assert ms_text(Decimal("0.0")) == "0"
    assert ms_text(Decimal("1E+3")) == "1000"
