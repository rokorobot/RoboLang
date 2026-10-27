"""
Tests for robolang.arch02, the reference simulation (ARCH-2; spec/reference-simulation-v0.1.md).

A-T1 to A-T14: every case of tests/fixtures/arch02/cases.json gives exactly its expected
results. A-T15 (in memory): every run is deterministic. A-T18 (part): the package loads no
network or adapter module and imports nothing from the language or `robolang.sim`. Also the
conversation-state rules (C306, C403) and the happy path's timeline against the spec table.
The JSON run record and the run ID are ARCH-3.
"""

from __future__ import annotations
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

from robolang.arch02 import RunResult, load_scenario, run_case, time_text

ROOT = Path(__file__).parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "arch02"
SCENARIO = load_scenario((FIXTURES / "scenario.json").read_bytes())
CASES = json.loads((FIXTURES / "cases.json").read_text(encoding="utf-8"))["cases"]
SPEC = (ROOT / "spec" / "reference-simulation-v0.1.md").read_text(encoding="utf-8")
P, A, M, H = "rl:agent:planner-1", "rl:robot:arm-02", "rl:robot:amr-07", "rl:human:shift-lead-3"
C42 = "rl:thing:container-c42"
RESULTS: dict[str, RunResult] = {}


def result(case: dict) -> RunResult:
    if case["id"] not in RESULTS:
        RESULTS[case["id"]] = run_case(SCENARIO, case)
    return RESULTS[case["id"]]


def case_named(name: str) -> dict:
    return next(case for case in CASES if case["id"] == name)


# -------- what each expectation field means (spec section 11) --------

def observed(r: RunResult, field: str, expect: dict):
    sent = [s.document for s in r.sent]
    if field == "planner_status":
        return r.planner_status
    if field == "arm02_goal_state":
        return dict(r.goal_states).get(A)
    if field == "message_types":
        return [[d["type"], d["sender"], d["recipient"]] for d in sent]
    if field == "rejections":
        rejected = [d for d in r.deliveries if d.receipt == "rejected"]
        return [[r.sent[d.message].document["type"], d.receiver, d.code] for d in rejected]
    if field == "duplicates":
        return [[r.sent[d.message].document["type"], d.receiver] for d in r.deliveries if d.receipt == "duplicate"]
    if field == "motion":
        return {p: sum(e.kind == "motion" and e.participant == p for e in r.events) for p in expect["motion"]}
    if field == "custody":
        local = dict(r.custody)
        return {k: dict(r.designated_custody)[C42] if k == "designated" else local.get(k) for k in expect["custody"]}
    if field == "reasons":
        last = [d for d in sent if d["recipient"] == P and d["type"] in ("outcome", "reject")][-1]
        return [x["code"] for x in last["payload"].get("outcome", last["payload"])["reasons"]]
    if field == "escalated":
        return sorted({e.participant for e in r.events if e.kind == "escalated"})
    if field in ("messages_present", "messages_absent"):
        pairs = {(d["type"], d["sender"]) for d in sent}
        return [list(pair) for pair in map(tuple, expect[field]) if (pair in pairs) == (field == "messages_present")]
    if field == "final_world":
        facts = {(f.subject, f.property, f.value) for f in r.final_world}
        return [x for x in expect[field] if (x["subject"], x["property"], x["value"]) in facts]
    raise KeyError(field)


EXPECTATIONS = [
    pytest.param(case, field, id=f"{case['id']}:{field}") for case in CASES for field in case["expect"]
]


@pytest.mark.parametrize(("case", "field"), EXPECTATIONS)
def test_case_meets_its_expectation(case, field):
    expected = case["expect"][field]
    if field == "escalated":
        expected = sorted(expected)
    assert observed(result(case), field, case["expect"]) == expected


def test_every_case_is_run():
    assert {case["id"] for case in CASES} == {
        "happy_path", "happy_path_seed_1", "v01_declines", "v02_duplicate_commit", "v03_replayed_commit",
        "v04_assistant_silent", "v05_stale_confirmation", "v06_partition_before_release", "v07_release_ack_lost",
        "v08_arrived_without_evidence", "v09_stale_reverification", "v10_requester_authorizes",
        "v11_payload_beyond_standing_authorization", "v12a_unsigned_commit", "v12b_wrongly_signed_commit",
        "v13_no_grasp",
    }


# -------- A-T1: the happy path in detail --------

def test_happy_path_follows_the_specification_timeline():
    section = SPEC[SPEC.index("### 6.1 Happy path"):SPEC.index("Final state:")]
    rows = [line.split("|")[1:-1] for line in section.splitlines() if re.match(r"\| \d+ \|", line)]
    r = result(case_named("happy_path"))
    assert [(time_text(s.t_sent_ms)[11:23], s.document["type"]) for s in r.sent] == [
        (row[2].strip(), row[4].strip().strip("`")) for row in rows
    ]


def test_happy_path_outcome_is_verified_by_evidence():
    r = result(case_named("happy_path"))
    final = r.sent[-1].document["payload"]
    verification = final["outcome"]["verification"]
    assert final["outcome"]["status"] == "succeeded"
    assert verification["method"] == "criteria_evaluated" and verification["verified_by"] == A
    assert [o["source"]["participant"] for o in final["evidence"]] == [M]
    assert verification["criteria_results"][0]["evidence"] == [o["id"] for o in final["evidence"]]
    sub = r.sent[-2].document["payload"]["outcome"]
    assert sub["verification"]["method"] == "self_reported"  # amr-07's only evidence is its own


def test_every_delivered_message_passes_the_core_checks():
    r = result(case_named("happy_path"))
    assert all(d.receipt == "accepted" for d in r.deliveries)
    assert len(r.deliveries) == 20


# -------- safeguards named in the RL-ARCH-02a review --------

def test_v11_denied_authorization_means_no_motion_despite_capacity():
    r = result(case_named("v11_payload_beyond_standing_authorization"))
    assert not [e for e in r.events if e.kind == "motion" and e.participant == M]
    assert [e.get("decision") for e in r.events if e.kind == "authorization" and e.participant == M] == ["denied"]
    capacity = SCENARIO["descriptors"][M]["capabilities"][0]
    assert capacity["kind"] == "transport_payload" and capacity["max_mass"]["value"] == "40"  # 35 kg would fit


def test_no_motion_happens_without_an_authorization_first():
    for case in CASES:
        r = result(case)
        for participant in (A, M):
            kinds = [e.kind for e in r.events if e.participant == participant and e.kind in ("authorization", "motion")]
            if "motion" in kinds:
                assert kinds.index("motion") > kinds.index("authorization"), (case["id"], participant)


# -------- A-T15: determinism (in memory; the serialized record is ARCH-3) --------

@pytest.mark.parametrize("case", CASES, ids=lambda case: case["id"])
def test_run_is_deterministic(case):
    assert run_case(SCENARIO, case) == result(case)


def test_seed_changes_nothing_but_the_seed():
    a, b = result(case_named("happy_path")), result(case_named("happy_path_seed_1"))
    assert (a.seed, b.seed) == ("0", "1")
    assert (a.sent, a.deliveries, a.events, a.final_world) == (b.sent, b.deliveries, b.events, b.final_world)


# -------- conversation-state rules --------

def custom(faults: list[dict], name: str = "custom") -> RunResult:
    return run_case(SCENARIO, {"id": name, "seed": "0", "faults": faults})


def test_release_naming_another_confirmation_is_c403():
    r = custom([{"kind": "inject", "at": "2026-10-19T10:14:51.150Z", "message": {
        "type": "custody_released", "profile": "r2r", "sender": A, "recipient": M,
        "payload": {"thing": C42, "confirmation": "m-not-the-confirmation"}}}])
    assert ["custody_released", M, "C403"] in [
        [r.sent[d.message].document["type"], d.receiver, d.code] for d in r.deliveries if d.receipt == "rejected"
    ]


def test_message_with_no_transition_is_c306():
    r = custom([{"kind": "inject", "at": "2026-10-19T10:14:40.000Z", "message": {
        "type": "custody_query", "profile": "r2r", "sender": M, "recipient": A, "payload": {"thing": C42}}}])
    assert [d.code for d in r.deliveries if d.receipt == "rejected"] == ["C306"]
    assert r.planner_status == "succeeded"


def test_message_overtaken_by_a_later_one_is_rejected_as_out_of_order():
    # arm-02's first assessment is delayed past its later `accept`: processing it after would go backwards.
    delay = {"kind": "delay", "match": {"type": "assessment", "sender": A, "n": 1}, "by": {"value": "1", "unit": "s"}}
    r = custom([delay])
    assert [[r.sent[d.message].document["type"], d.code] for d in r.deliveries if d.receipt == "rejected"] == [
        ["assessment", "C306"]
    ]


# -------- A-T18: isolation (part) --------

FORBIDDEN_LOADED = ("socket", "ssl", "http", "urllib.request", "rclpy", "robolang_adapter", "robolang_runtime")


def test_importing_robolang_arch02_loads_no_network_or_adapter_module():
    probe = (
        "import sys, robolang.arch02; "
        f"print([m for m in sys.modules if any(m == f or m.startswith(f + '.') for f in {FORBIDDEN_LOADED!r})])"
    )
    result = subprocess.run([sys.executable, "-c", probe], capture_output=True, text=True, check=True,
                            env={**os.environ, "PYTHONPATH": str(ROOT / "src")})
    assert result.stdout.strip() == "[]"


def test_arch02_imports_only_robolang_core_from_the_package():
    for path in (ROOT / "src" / "robolang" / "arch02").glob("*.py"):
        text = path.read_text(encoding="utf-8")
        imports = re.findall(r"^from \.\.(\w+)", text, re.MULTILINE)
        assert set(imports) <= {"core"}, (path.name, imports)
        for module in ("socket", "http", "urllib", "rclpy", "robolang_adapter", "robolang_runtime"):
            assert f"import {module}" not in text, (path.name, module)
