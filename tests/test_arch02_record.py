"""
Tests for the reference simulation's run record, run ID and `robolang arch02` command (ARCH-3;
spec/reference-simulation-v0.1.md sections 9 and 12).

A-T15: every case gives a byte-identical record and the same run ID when run twice; a different
seed changes only the run ID and the recorded seed. A-T18 (part): every record says
`"mode": "simulation"`. The record shape and the run-ID derivation are checked against the
specification, with the run ID recomputed here independently.
"""

from __future__ import annotations
import hashlib
import io
import json
import os
from pathlib import Path

import pytest

from robolang.arch02 import simulate_case, to_json
from robolang.cli import main

ROOT = Path(__file__).parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "arch02"
SCENARIO_PATH = FIXTURES / "scenario.json"
SCENARIO = SCENARIO_PATH.read_bytes()
CASES = json.loads((FIXTURES / "cases.json").read_text(encoding="utf-8"))["cases"]
SEQ_CASES = json.loads((FIXTURES / "sequencing_cases.json").read_text(encoding="utf-8"))["cases"]
ALL = [pytest.param(case, id=case["id"]) for case in CASES + SEQ_CASES]

TOP_KEYS = ["run_record_schema", "run_record_schema_version", "mode", "run_id", "inputs", "messages", "events",
            "final", "designated_custody", "final_world", "end_t_ms"]
MESSAGE_KEYS = ["t_sent_ms", "t_delivered_ms", "fate", "document", "receipts"]
RECEIPT_KEYS = ["t_ms", "receiver", "kind", "receipt", "code"]
COMMITMENT_STATES = {None, "active", "fulfilled", "lapsed", "failed"}  # Core 5.8


def expected_run_id(case: str, seed: str) -> str:
    """Section 9.1, recomputed without the package."""
    data = SCENARIO[3:] if SCENARIO.startswith(b"\xef\xbb\xbf") else SCENARIO
    digest = hashlib.sha256(data.replace(b"\r\n", b"\n")).hexdigest()
    preimage = f"robolang-arch02-run/0.1\n{digest}\n{case}\n{seed}\n".encode()
    return "arch02-" + hashlib.sha256(preimage).hexdigest()


@pytest.mark.parametrize("case", ALL)
def test_record_has_the_specified_shape(case):
    record = simulate_case(SCENARIO, case)
    assert list(record) == TOP_KEYS
    assert (record["run_record_schema"], record["run_record_schema_version"]) == ("robolang.arch02.run", "0.1")
    assert record["mode"] == "simulation"
    assert record["run_id"] == expected_run_id(case["id"], case["seed"])
    assert list(record["inputs"]) == ["scenario_sha256", "case", "seed", "core_version"]
    assert record["inputs"]["core_version"] == "0.1"
    for message in record["messages"]:
        assert list(message) == MESSAGE_KEYS
        assert message["fate"] in ("delivered", "dropped", "injected")
        for receipt in message["receipts"]:
            assert list(receipt) == RECEIPT_KEYS
    for participant in record["final"]["participants"].values():
        assert participant["commitment_state"] in COMMITMENT_STATES
    assert record["end_t_ms"] >= max(m["t_sent_ms"] for m in record["messages"])


@pytest.mark.parametrize("case", ALL)
def test_record_is_byte_identical_when_run_twice(case):
    first = to_json(simulate_case(SCENARIO, case))
    assert first == to_json(simulate_case(SCENARIO, case))
    assert first.endswith("}\n") and "\r" not in first
    first.encode("ascii")


def test_seed_changes_only_the_run_id_and_the_seed():
    happy = next(case for case in CASES if case["id"] == "happy_path")
    a, b = simulate_case(SCENARIO, happy), simulate_case(SCENARIO, happy, seed="1")
    assert a["run_id"] != b["run_id"] == expected_run_id("happy_path", "1")
    for record in (a, b):
        record.pop("run_id")
        record["inputs"].pop("seed")
    assert a == b


def test_held_message_has_both_receipts_in_the_record():
    case = next(case for case in SEQ_CASES if case["id"] == "seq_gap_filled")
    record = simulate_case(SCENARIO, case)
    accept = next(m for m in record["messages"] if m["document"]["type"] == "accept")
    assert [(r["t_ms"], r["kind"], r["receipt"]) for r in accept["receipts"]] == [
        (600, "original", "held"), (1200, "released", "accepted"),
    ]


def test_lost_message_has_no_delivery_time():
    case = next(case for case in CASES if case["id"] == "v07_release_ack_lost")
    record = simulate_case(SCENARIO, case)
    lost = [m for m in record["messages"] if m["fate"] == "dropped"]
    assert len(lost) == 1 and lost[0]["t_delivered_ms"] is None and lost[0]["receipts"] == []


# -------- the command --------

def arch02(*args: str) -> tuple[int, str, str]:
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(ROOT)
    try:
        code = main(["arch02", *args], out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


BASE = ["--scenario", "tests/fixtures/arch02/scenario.json", "--cases", "tests/fixtures/arch02/cases.json"]


def test_command_prints_the_record_and_a_simulation_only_summary():
    code, out, err = arch02(*BASE, "--case", "happy_path")
    record = json.loads(out)
    assert code == 0
    assert out == to_json(simulate_case(SCENARIO, CASES[0]))
    assert err.strip() == f"simulation only: run {record['run_id']}: planner-1 recorded succeeded"


def test_command_runs_a_sequencing_case():
    code, out, _ = arch02("--scenario", "tests/fixtures/arch02/scenario.json",
                          "--cases", "tests/fixtures/arch02/sequencing_cases.json", "--case", "seq_gap_abandoned")
    assert code == 0 and json.loads(out)["final"]["planner_status"] == "succeeded"


@pytest.mark.parametrize(
    ("args", "code"),
    [
        (["--case", "no_such_case"], 2),
        (["--case", "happy_path", "--seed", "not a seed"], 2),
        (["--scenario", "missing.json", "--cases", "tests/fixtures/arch02/cases.json", "--case", "happy_path"], 2),
        (["--scenario", "tests/fixtures/arch02/cases.json", "--cases", "tests/fixtures/arch02/cases.json",
          "--case", "happy_path"], 3),
        (["--scenario", "tests/fixtures/arch02/scenario.json", "--cases", "tests/fixtures/arch02/scenario.json",
          "--case", "happy_path"], 3),
    ],
)
def test_command_errors(args, code):
    full = args if args[0] == "--scenario" else [*BASE, *args]
    result, out, err = arch02(*full)
    assert result == code and out == "" and err
