"""
Golden run records for the reference simulation (follow-up F1 of the RL-ARCH-02 gate 1 evidence).

Every case's run record must hash to the value pinned in tests/fixtures/arch02/golden_records.json.
CI runs this on Ubuntu and Windows with Python 3.11 and 3.13, so a pass on every job shows that
the records are byte-identical across platforms (charter MC8), not only deterministic on each.

To regenerate after an intended change to the simulation or the record:

    ROBOLANG_UPDATE_GOLDEN=1 python -m pytest tests/test_arch02_golden.py
"""

from __future__ import annotations
import hashlib
import json
import os
from pathlib import Path

import pytest

from robolang.arch02 import simulate_case, to_json

FIXTURES = Path(__file__).parent / "fixtures" / "arch02"
GOLDEN = FIXTURES / "golden_records.json"
UPDATE = os.environ.get("ROBOLANG_UPDATE_GOLDEN") == "1"
SCENARIO = (FIXTURES / "scenario.json").read_bytes()
CASE_FILES = ("cases.json", "sequencing_cases.json")
CASES = [
    (name, case)
    for name in CASE_FILES
    for case in json.loads((FIXTURES / name).read_text(encoding="utf-8"))["cases"]
]


def record_digest(case: dict) -> tuple[str, str]:
    record = simulate_case(SCENARIO, case)
    return record["run_id"], hashlib.sha256(to_json(record).encode("ascii")).hexdigest()


def current() -> dict:
    return {
        "fixture_set": "robolang.arch02.golden",
        "spec": "spec/reference-simulation-v0.1.md",
        "hash": "SHA-256 of the record's JSON text (to_json, ASCII, LF)",
        "records": {
            f"{name}#{case['id']}": dict(zip(("run_id", "record_sha256"), record_digest(case)))
            for name, case in CASES
        },
    }


if UPDATE:
    GOLDEN.write_bytes((json.dumps(current(), indent=2, ensure_ascii=True) + "\n").encode("ascii"))

PINNED = json.loads(GOLDEN.read_text(encoding="utf-8"))["records"]


def test_every_case_is_pinned():
    assert sorted(PINNED) == sorted(f"{name}#{case['id']}" for name, case in CASES)
    assert len(PINNED) == 24


@pytest.mark.parametrize(("name", "case"), CASES, ids=[f"{n}#{c['id']}" for n, c in CASES])
def test_record_matches_its_golden_hash(name, case):
    run_id, digest = record_digest(case)
    pinned = PINNED[f"{name}#{case['id']}"]
    assert run_id == pinned["run_id"]
    assert digest == pinned["record_sha256"], "the run record differs from the pinned golden record"
