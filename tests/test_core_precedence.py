"""
INTEROP-2f, F2-F5: the multi-fault precedence fixtures of tests/fixtures/core.

A fixture that is merely rejected proves nothing about precedence. For each one this checks that:

1. the file is exactly what tests/core_precedence.py builds (reproducible);
2. `robolang.core` gives the stated code and stage (also checked by test_core.py, A-T16);
3. every competing fault is really present: with the faults before it removed, the receiver gives
   that fault's code, so the stated code wins over another applicable failure;
4. the order follows Core 8.1 (stage 1 ranks; stage 2 C204, C203, C202 and the message before its
   observations; stage 3 C302, C301, C303, C307).
"""

from __future__ import annotations
import json
from pathlib import Path

import pytest

from core_precedence import CONTEXT, FAULTS, FIXTURES, KEYS, build, case
from robolang.core import Context, check_message, time_ms

CORE = Path(__file__).parent / "fixtures" / "core"
CASES = {c["file"]: c for c in json.loads((CORE / "cases.json").read_text(encoding="utf-8"))["cases"]}
# Core 8.1, stage 1 ranks; within rank 7, C103 before C104 (0011 N13).
RANK = {"C102": 2, "C105": 3, "C106": 3, "C201": 4, "C107": 5, "C402": 6, "C103": 7, "C104": 7.5}
STAGE_2 = ["C204", "C203", "C202"]
STAGE_3 = ["C302", "C301", "C303", "C307"]


def ms(quantity: dict) -> int:
    return int(quantity["value"]) * {"ms": 1, "s": 1000, "min": 60000}[quantity["unit"]]


def receipt(document: dict, receiver: str | None):
    context = Context(receiver or document["recipient"], time_ms(CONTEXT["now"]), ms(CONTEXT["clock_tolerance"]),
                      ms(CONTEXT["max_message_lifetime"]), KEYS, CONTEXT["authorities"], "simulation")
    return check_message(json.dumps(document).encode("utf-8"), context)


@pytest.mark.parametrize("stem", FIXTURES)
def test_fixture_file_and_case_are_reproducible(stem):
    on_disk = json.loads((CORE / "invalid" / f"{stem}.json").read_text(encoding="utf-8"))
    assert on_disk == build(stem)
    assert CASES[f"invalid/{stem}.json"] == case(stem)


@pytest.mark.parametrize("stem", FIXTURES)
def test_stated_code_wins_over_each_competing_fault(stem):
    _, stage, faults, receiver = FIXTURES[stem]
    first = receipt(build(stem), receiver)
    assert (first.code, first.stage) == (FAULTS[faults[0]][0], stage)
    for i in range(1, len(faults)):
        later = receipt(build(stem, faults[i:]), receiver)
        assert later.code == FAULTS[faults[i]][0], (stem, faults[i:], later)


@pytest.mark.parametrize("stem", FIXTURES)
def test_order_is_the_order_core_8_1_states(stem):
    codes = [FAULTS[name][0] for name in FIXTURES[stem][2]]
    if all(c in RANK for c in codes):
        ranks = [RANK[c] for c in codes]
        assert ranks == sorted(ranks) and len(set(ranks)) == len(ranks)
    elif "observation" in stem:
        assert FIXTURES[stem][2][0] in ("bad_value", "other_key")  # the message's own signature first
    elif all(c in STAGE_2 for c in codes):
        assert [STAGE_2.index(c) for c in codes] == sorted(STAGE_2.index(c) for c in codes)
    else:
        assert all(c in STAGE_3 for c in codes)
        assert [STAGE_3.index(c) for c in codes] == sorted(STAGE_3.index(c) for c in codes)


def test_competing_message_and_observation_codes_differ():
    # The observation's fault has a different code from the message's, so the receipt shows which
    # object was checked first; an implementation checking observations first would report it.
    for stem in ("c202_message_before_c203_observation", "c203_message_before_c204_observation"):
        codes = [FAULTS[name][0] for name in FIXTURES[stem][2]]
        assert len(set(codes)) == 2
        assert receipt(build(stem), None).pointer == "/signature/" + {"C202": "value", "C203": "key"}[codes[0]]
