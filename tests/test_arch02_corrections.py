"""
Regression tests for INTEROP-2d: the reference corrections D1-D7 (decision records 0011 section 4
and 0012; docs/interop/spec-implementation-discrepancies.md section 1).

Each test builds a case that the 24 golden cases do not reach, and checks the behavior the
specifications now state: reference simulation 6.2-6.4 (D1-D5) and Core 8.1 and 8.4 (D6, D7).
"""

from __future__ import annotations
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from robolang.arch02 import RunResult, load_scenario, run_case, time_text
from robolang.arch02.engine import build_observation
from robolang.arch02.participants import Amr
from robolang.arch02.scenario import keys_of
from robolang.core import Context, check_meaning, check_message, time_ms

ROOT = Path(__file__).parent.parent
SCENARIO = load_scenario((ROOT / "tests" / "fixtures" / "arch02" / "scenario.json").read_bytes())
P, A, M = "rl:agent:planner-1", "rl:robot:arm-02", "rl:robot:amr-07"
C42, H1, S3 = "rl:thing:container-c42", "rl:place:handoff-h1", "rl:place:station-s3"
DAY = "2026-10-19T"


def run(*faults: dict) -> RunResult:
    return run_case(SCENARIO, {"id": "custom", "seed": "0", "faults": list(faults)})


def override(path: str, value) -> dict:
    return {"kind": "override", "path": path, "value": value}


def leases(offer_end: str, reservation_end: str) -> tuple[dict, dict]:
    return (override("/parameters/offer_lease_end", DAY + offer_end),
            override("/parameters/reservation_end", DAY + reservation_end))


def sent(r: RunResult, mtype: str, sender: str) -> list[dict]:
    return [s.document for s in r.sent if s.document["type"] == mtype and s.document["sender"] == sender]


def receipts(r: RunResult, message_id: str) -> list[tuple[str, str, str | None]]:
    return [(d.kind, d.receipt, d.code) for d in r.deliveries if r.sent[d.message].document["message_id"] == message_id]


def events(r: RunResult, participant: str, kind: str) -> list:
    return [e for e in r.events if e.participant == participant and e.kind == kind]


# -------- D1: a contradicted criterion gives `failed` with empty reasons (spec 6.3) --------

def test_d1_amr07_contradicted_outcome_has_empty_reasons():
    # deck_camera reports 0.95; a criterion asking 0.99 is contradicted (right value, too low a confidence).
    r = run(override("/goal/success_criteria/0/min_confidence", "0.99"))
    outcome = sent(r, "outcome", M)[-1]["payload"]["outcome"]
    assert (outcome["status"], outcome["reasons"], outcome["verification"]) == ("failed", [], None)
    assert all(d.receipt == "accepted" for d in r.deliveries if r.sent[d.message].document["type"] == "outcome")


def test_d1_arm02_contradicted_outcome_has_empty_reasons():
    # amr-07 claims success with evidence arm-02 finds contradicted (confidence 0.5 < 0.9).
    at = DAY + "10:15:00.000Z"
    evidence = build_observation(M, 99, C42, "located_in", S3, time_ms(at), "deck_camera", {"value": "5", "unit": "s"},
                                 "0.5", "camera_detection", keys_of(SCENARIO)[f"{M}#k1"])
    verification = {"verified_by": M, "method": "self_reported", "evaluated_at": at,
                    "criteria_results": [{"criterion": 0, "met": True, "evidence": [evidence["id"]]}]}
    r = run({"kind": "inject", "at": at, "message": {
        "type": "outcome", "profile": "r2r", "sender": M, "recipient": A, "payload": {
            "outcome": {"kind": "outcome", "goal": SCENARIO["goal"]["id"] + "-sub-1", "status": "succeeded",
                        "verification": verification, "reasons": [], "reported_by": M, "reported_at": at},
            "evidence": [evidence]}}})
    outcome = sent(r, "outcome", A)[0]["payload"]["outcome"]
    assert (outcome["status"], outcome["reasons"]) == ("failed", [])
    assert r.planner_status == "failed"


# -------- D2: a custody_query before any sub-goal is rejected C306 (spec 6.4) --------

def test_d2_custody_query_before_request_assistance_is_c306():
    r = run({"kind": "inject", "at": DAY + "10:14:00.000Z", "message": {
        "type": "custody_query", "profile": "r2r", "sender": A, "recipient": M, "payload": {"thing": C42}}})
    query = sent(r, "custody_query", A)[0]
    assert receipts(r, query["message_id"]) == [("original", "rejected", "C306")]
    assert not sent(r, "custody_state", M)  # amr-07 never replies to a message it rejects
    assert r.planner_status == "succeeded"  # the stream moved on; the scenario runs as usual


def test_d2_custody_query_for_another_thing_is_still_c306():
    r = run({"kind": "inject", "at": DAY + "10:14:40.000Z", "message": {
        "type": "custody_query", "profile": "r2r", "sender": A, "recipient": M, "payload": {"thing": "rl:thing:x"}}})
    assert receipts(r, sent(r, "custody_query", A)[0]["message_id"]) == [("original", "rejected", "C306")]


# -------- D3: an action in progress at lease expiry (spec 6.2 rule 3, 6.4; 0012 P1) --------

@pytest.mark.parametrize("offer_end", ["10:15:30.000Z", "10:15:51.200Z"], ids=["during_move", "at_arrival"])
def test_d3_final_move_completes_but_amr07_takes_no_follow_up_step(offer_end):
    # The final move runs 10:14:51.200-10:15:51.200. "at_arrival": the lease ends at the arrival instant.
    r = run(*leases(offer_end, "10:15:00.000Z"))
    expiry = time_ms(DAY + offer_end)
    assert [e.t_ms for e in events(r, M, "lease_expired")] == [expiry]
    # The move completes in the world, and its motion and effect events stand.
    assert [(time_text(e.t_ms)[11:23], e.get("to")) for e in events(r, M, "effect")][-1] == ("10:15:51.200", S3)
    # amr-07: one outcome only (the expiry's), no sensing and no state change after the expiry.
    assert [o["payload"]["outcome"]["status"] for o in sent(r, "outcome", M)] == ["unknown"]
    assert not [e for e in r.events if e.participant == M and e.t_ms > expiry and e.kind in ("observation", "state")]
    assert dict(r.commitments)[M] == "lapsed"
    assert dict(r.goal_states)[M] == "UNKNOWN"


def test_d3_first_move_completes_but_amr07_sends_no_ready_for_handover():
    # The move to the hand-off place runs 10:14:00.600-10:14:30.600; the lease ends during it.
    r = run(*leases("10:14:20.000Z", "10:14:10.000Z"))
    assert [e.get("to") for e in events(r, M, "effect")] == [H1]
    assert not sent(r, "coordinate", M)
    assert [o["payload"]["outcome"]["status"] for o in sent(r, "outcome", M)] == ["failed"]
    assert dict(r.commitments)[M] == "lapsed"


def test_d3_lease_ending_after_arrival_does_not_change_the_happy_path():
    r = run(*leases("10:15:51.201Z", "10:15:00.000Z"))
    assert [o["payload"]["outcome"]["status"] for o in sent(r, "outcome", M)] == ["succeeded"]
    assert dict(r.commitments)[M] == "fulfilled"


# -------- D4: capability_unsupported versus capability_insufficient (spec 6.4; Core 5.11) --------

DESCRIPTOR = f"/descriptors/{M}"
SUB_GOAL = dict(SCENARIO["goal"], id=SCENARIO["goal"]["id"] + "-sub-1", requested_by=A)


def request_assistance() -> dict:
    """arm-02 applies the same descriptor test before it asks (and would not ask), so the
    request is delivered to amr-07 directly."""
    return {"kind": "inject", "at": DAY + "10:14:00.000Z", "message": {
        "type": "request_assistance", "profile": "r2r", "sender": A, "recipient": M,
        "payload": {"goal": SUB_GOAL, "things": SCENARIO["things"], "parent_goal": SCENARIO["goal"]["id"]}}}


@pytest.mark.parametrize("fault, code", [
    (override(f"{DESCRIPTOR}/valid_until", "2026-10-19T10:00:00Z"), "capability_unsupported"),
    (override(f"{DESCRIPTOR}/unsupported", ["grasp", "transport_payload"]), "capability_unsupported"),
    (override(f"{DESCRIPTOR}/capabilities/1/kind", "inspect_only"), "capability_unsupported"),  # no dock_at
    (override(f"{DESCRIPTOR}/signature/key", f"{A}#k1"), "capability_unsupported"),  # not validly signed
    (override(f"{DESCRIPTOR}/capabilities/0/max_mass/value", "24.9"), "capability_insufficient"),
    (override(f"{DESCRIPTOR}/capabilities/1/places", ["rl:place:dock-d1", H1]), "capability_insufficient"),
], ids=["not_current", "listed_unsupported", "kind_absent", "wrong_key", "mass", "place"])
def test_d4_capability_decline_code(fault, code):
    r = run(fault, request_assistance())
    assert [d["payload"]["reasons"] for d in sent(r, "decline", M)] == [[{"code": code}]]
    assert not events(r, M, "motion")


def test_d4_descriptor_expiring_between_the_two_checks_is_unsupported_for_amr07():
    # Current for arm-02 at 10:14:00.100, not for amr-07 at 10:14:00.200 (safe direction, 500 ms).
    r = run(override(f"{DESCRIPTOR}/valid_until", "2026-10-19T10:14:00.650Z"))
    assert [d["payload"]["reasons"] for d in sent(r, "decline", M)] == [[{"code": "capability_unsupported"}]]


def test_d4_mass_at_the_limit_is_accepted():
    r = run(override(f"{DESCRIPTOR}/capabilities/0/max_mass/value", "25"))  # C42 is 25 kg; limits are inclusive
    assert not sent(r, "decline", M) and r.planner_status == "succeeded"


# -------- D5: all four conditions before every move (spec 6.2 rule 2, 6.4) --------

@pytest.mark.parametrize("reservation_end, moves", [
    ("10:14:51.700Z", False),  # 51.200 < 51.700 - 0.500 is false: refused (safe direction)
    ("10:14:51.701Z", True),
], ids=["at_tolerance_boundary", "just_inside"])
def test_d5_reservation_must_be_unexpired_in_the_safe_direction(reservation_end, moves):
    r = run(override("/parameters/reservation_end", DAY + reservation_end))
    to_s3 = [e for e in events(r, M, "motion") if e.get("to") == S3]
    refused = [e for e in r.events if e.participant == M and e.get("record") == "motion_refused"]
    assert bool(to_s3) is moves and bool(refused) is not moves
    if not moves:
        assert refused[0].get("to") == S3 and time_text(refused[0].t_ms)[11:23] == "10:14:51.200"


def test_d5_standing_authorization_must_still_be_valid_at_the_move():
    # Valid when amr-07 authorizes (10:14:00.300) and at the first move, expired at the final one.
    r = run(override(f"/standing_authorizations/{M}/lease/expires_at", DAY + "10:14:51.000Z"))
    assert [e.get("to") for e in events(r, M, "motion")] == [H1]
    assert [e.get("to") for e in r.events if e.participant == M and e.get("record") == "motion_refused"] == [S3]
    assert dict(r.custody)[M] == "HOLDING" and dict(r.goal_states)[M] == "UNKNOWN"  # then the lease expiry decides


def amr(now: int, **state) -> SimpleNamespace:
    """A stand-in amr-07 with every pre-move condition met at `now`, then `state` applied."""
    fields = dict(
        sim=SimpleNamespace(now=now, tolerance_ms=500), authorization=now + 10_000, goal_state="RUNNING",
        commitment_state="active", commitment={"lease": {"expires_at": time_text(now + 10_000)}},
        reservation_until=now + 10_000, _standing_authorization=lambda: "rl:sa:x",
    )
    fields.update(state)
    return SimpleNamespace(**fields)


NOW = time_ms(DAY + "10:14:51.200Z")


@pytest.mark.parametrize("state, moves", [
    ({}, True),
    ({"authorization": None}, False),  # never established
    ({"authorization": NOW + 500}, False),  # decision lease at the tolerance boundary
    ({"authorization": NOW + 501}, True),
    ({"authorization": "standing", "_standing_authorization": lambda: None}, False),
    ({"authorization": "standing"}, True),
    ({"goal_state": "AUTHORIZED"}, False),
    ({"commitment_state": "lapsed"}, False),
    ({"commitment": {"lease": {"expires_at": time_text(NOW + 500)}}}, False),  # active, but in the tolerance window
    ({"commitment": {"lease": {"expires_at": time_text(NOW + 501)}}}, True),
    ({"reservation_until": None}, False),  # no granted reservation
    ({"reservation_until": NOW + 500}, False),
], ids=["all_met", "no_authorization", "decision_at_boundary", "decision_inside", "standing_lapsed",
        "standing_valid", "goal_not_running", "commitment_lapsed", "commitment_lease_at_boundary",
        "commitment_lease_inside", "no_reservation", "reservation_at_boundary"])
def test_d5_each_pre_move_condition(state, moves):
    assert Amr._may_move(amr(NOW, **state)) is moves


# -------- D6: a conflicting sequence while one is held (Core 8.4; 0012 P2) --------

DELAY_ASSESSMENT = {"kind": "delay", "match": {"type": "assessment", "sender": A, "n": 1},
                    "by": {"value": "1", "unit": "s"}}
# With the assessment delayed, arm-02's `accept` (sequence 2) is held by planner-1 from 10:14:00.600
# to 10:14:01.200 (sequencing case seq_gap_filled).


def inject_to_planner(sender: str, sequence: int, message_id: str, at: str = "10:14:00.700Z") -> dict:
    return {"kind": "inject", "at": DAY + at, "message": {
        "type": "accept", "profile": "a2r", "sender": sender, "recipient": P, "sequence": sequence,
        "message_id": message_id, "payload": {"goal": SCENARIO["goal"]["id"]}}}


def test_d6_conflicting_sequence_is_c306_and_the_held_message_is_undisturbed():
    r = run(DELAY_ASSESSMENT, inject_to_planner(A, 2, "m-conflicting-accept"))
    accept = next(d for d in sent(r, "accept", A) if d["message_id"] != "m-conflicting-accept")
    assert receipts(r, "m-conflicting-accept") == [("original", "rejected", "C306")]
    # The original keeps its place and gets exactly one terminal (release) receipt.
    assert receipts(r, accept["message_id"]) == [("original", "held", None), ("released", "accepted", None)]
    planner = [(time_text(e.t_ms)[11:23], e.get("record")) for e in events(r, P, "state")]
    assert planner[:2] == [("10:14:01.200", "assessment"), ("10:14:01.200", "accepted")]
    assert r.planner_status == "succeeded"


def test_d6_same_id_while_held_is_still_a_duplicate():
    r = run(DELAY_ASSESSMENT, {"kind": "duplicate", "match": {"type": "accept", "sender": A, "n": 1},
                               "after": {"value": "100", "unit": "ms"}})
    accept = sent(r, "accept", A)[0]
    assert receipts(r, accept["message_id"]) == [
        ("original", "held", None), ("duplicate", "duplicate", None), ("released", "accepted", None)]


def test_d6_the_rule_does_not_cross_streams():
    # The same sequence from another sender is its own stream: held there, not rejected.
    r = run(DELAY_ASSESSMENT, inject_to_planner(M, 2, "m-other-stream"))
    assert receipts(r, "m-other-stream")[0] == ("original", "held", None)
    accept = sent(r, "accept", A)[0]
    assert receipts(r, accept["message_id"]) == [("original", "held", None), ("released", "accepted", None)]


def test_d6_conflict_after_release_is_stale():
    r = run(DELAY_ASSESSMENT, inject_to_planner(A, 2, "m-late-conflict", at="10:14:02.000Z"))
    assert receipts(r, "m-late-conflict") == [("original", "rejected", "C306")]


# -------- D7: stage 5 after duplicate and order handling (Core 8.1) --------

def foreign_decision(sequence: int | None = None, at: str = "10:14:10.000Z") -> dict:
    """An authorization_decision to amr-07 from planner-1, which is not its authority: C401 at stage 5."""
    message = {"type": "authorization_decision", "profile": "a2r", "sender": P, "recipient": M, "payload": {
        "goal": SCENARIO["goal"]["id"] + "-sub-1", "decision": "granted", "mode": "simulation", "constraints": [],
        "lease": {"granted_at": DAY + at, "expires_at": DAY + "11:00:00.000Z", "renewal": 0}}}
    if sequence is not None:
        message["sequence"] = sequence
    return {"kind": "inject", "at": DAY + at, "message": message}


def test_d7_stage_5_rejection_after_a_sequence_gap():
    # Sequence 2 with 1 missing: held first (stage 4), and rejected C401 only when it is released.
    r = run(foreign_decision(sequence=2))
    decision = sent(r, "authorization_decision", P)[0]
    assert receipts(r, decision["message_id"]) == [("original", "held", None), ("released", "rejected", "C401")]
    released = next(d for d in r.deliveries if d.kind == "released" and d.code == "C401")
    assert time_text(released.t_ms)[11:23] == "10:14:15.100"  # max_hold (5 s) after delivery
    assert r.planner_status == "succeeded"


def test_d7_copy_of_a_stage_5_rejected_message_is_judged_again():
    # Rejected at stage 5, the message is not remembered: its copy is not a duplicate, and its
    # stream has passed it, so it is stale (Core 8.4, 0011 N13).
    r = run(foreign_decision(), {"kind": "duplicate", "match": {"type": "authorization_decision", "sender": P, "n": 1},
                                 "after": {"value": "1", "unit": "s"}})
    decision = sent(r, "authorization_decision", P)[0]
    assert receipts(r, decision["message_id"]) == [("original", "rejected", "C401"), ("duplicate", "rejected", "C306")]


def test_d7_stateless_check_still_gives_the_stage_5_code():
    # A conformance check of a single message, with no conversation state, gives the same code (Core 8.1).
    r = run(foreign_decision())
    document = sent(r, "authorization_decision", P)[0]
    context = Context(M, time_ms(DAY + "10:14:10.100Z"), 500, 60_000, keys_of(SCENARIO), SCENARIO["authorities"])
    data = json.dumps(document).encode("utf-8")
    assert check_message(data, context).code == "C401"
    assert check_message(data, context, meaning=False).accepted
    assert check_meaning(document, context).code == "C401"
