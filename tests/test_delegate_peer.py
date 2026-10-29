"""Tests for `delegate` with a peer robot: commitment, lease, custody (spec/delegate-v0.2.md, decision record 0022)."""

from __future__ import annotations
import copy
import functools
import hashlib
import json

import pytest

from robolang import cli
from robolang.bridge import core_time
from robolang.core import Context, check_message, sign_sim, time_ms
from robolang.delegate import SIM_KEY, SIM_KEY_REF, delegate
from robolang.delegate_peer import CustodyPoint, custody_point, lease_ms, trust_keys
from robolang.export_goal import simulation_key
from robolang.ir import compile_source
from test_delegate import (
    ARM, BASE, BINDINGS, LIBRARY, PLANNER, REPO_ROOT, TASK, event, record_of, robot_world, run,
    types, write,
)

AMR = "rl:robot:amr1"
HANDOVER = (REPO_ROOT / "examples" / "handover.rob").read_text(encoding="utf-8")
FULL = ["propose_goal", "assessment", "request_assistance", "offer_assistance", "commit", "accept",
        "transfer_custody", "custody_confirmed", "custody_released", "acknowledge", "outcome"]
SPEC = (REPO_ROOT / "spec" / "delegate-v0.2.md").read_text(encoding="utf-8")


def events(record: dict, kind: str) -> list[dict]:
    return [e for e in record["events"] if e["event"] == kind]


def custody(record: dict, actor: str) -> list[str]:
    return [e["state"] for e in events(record, "custody") if e["actor"] == actor]


def intercept_with(monkeypatch, change, mtype: str, resign: str | None = None) -> None:
    """Change the first message of type `mtype` in flight; sign it again as `resign` (a participant) if given."""
    done = []

    def intercept(message):
        if message["type"] == mtype and not done:
            done.append(True)
            message = copy.deepcopy(message)
            change(message)
            if resign:
                message = sign_sim(message, f"{resign}#k1", simulation_key(f"{resign}#k1"))
        return message

    monkeypatch.setattr(cli, "delegate", functools.partial(delegate, intercept=intercept))


def slow_plan(tmp_path, seconds: int) -> str:
    """The handover plan with a wait before the arm's last step, so the custody point comes later."""
    text = HANDOVER.replace("        communicate arm", f"        wait for {seconds}s;\n        communicate arm")
    return write(tmp_path, "slow.rob", text)


# -------- the happy path --------

def test_handover_command_succeeds_with_the_full_exchange():
    code, out, err = run(*BASE, "--plans", "examples/handover.rob")
    record = json.loads(out)
    assert code == 0, err
    assert err.strip() == "simulation only: delegate rl:goal:handover → rl:robot:arm1: succeeded (verified)"
    assert types(record) == FULL
    assert all(m["receipt"] == {"status": "accepted", "code": None} for m in record["messages"])
    assert record["planner_verdict"]["status"] == "succeeded" and record["planner_verdict"]["reasons"] == []
    assert record["version"] == "0.2" and record["peer"] == AMR
    assert record["robot_run"]["state"] == "SUCCEEDED" and record["robot_run"]["cancelled_before_step"] is None


def test_messages_name_the_right_participants_and_profiles():
    record, _ = record_of(*BASE, "--plans", "examples/handover.rob")
    route = [(m["document"]["sender"], m["document"]["recipient"], m["document"]["profile"])
             for m in record["messages"]]
    a2r, r2r = "a2r", "r2r"
    assert route == [
        (PLANNER, ARM, a2r), (ARM, PLANNER, a2r), (ARM, AMR, r2r), (AMR, ARM, r2r), (ARM, AMR, r2r),
        (ARM, PLANNER, a2r), (ARM, AMR, r2r), (AMR, ARM, r2r), (ARM, AMR, r2r), (AMR, ARM, r2r), (ARM, PLANNER, a2r)]
    messages = {m["document"]["type"]: m["document"] for m in record["messages"]}
    assessment = messages["assessment"]["payload"]
    assert assessment["result"] == "feasible_with_assistance" and assessment["reasons"] == [
        {"code": "assistance_required"}]
    request = messages["request_assistance"]["payload"]
    assert request["parent_goal"] == "rl:goal:handover" and request["goal"]["id"] == "rl:goal:handover-sub-1"
    assert request["goal"]["requested_by"] == ARM
    assert messages["offer_assistance"]["in_reply_to"] == messages["request_assistance"]["message_id"]


def test_commit_carries_the_offered_lease_and_a_60_second_default():
    record, _ = record_of(*BASE, "--plans", "examples/handover.rob")
    messages = {m["document"]["type"]: m["document"] for m in record["messages"]}
    lease = messages["offer_assistance"]["payload"]["lease"]
    assert time_ms(lease["expires_at"]) - time_ms(lease["granted_at"]) == 60_000 and lease["renewal"] == 0
    commitment = messages["commit"]["payload"]["commitment"]
    assert (commitment["debtor"], commitment["creditor"], commitment["state"]) == (AMR, ARM, "active")
    assert commitment["lease"] == lease and commitment["obligation"] == {"goal": "rl:goal:handover-sub-1"}
    assert messages["transfer_custody"]["payload"]["commitment"] == commitment["id"]
    assert record["commitment"] == {**commitment, "state": "fulfilled"}


def test_custody_transfer_follows_the_rules():
    record, _ = record_of(*BASE, "--plans", "examples/handover.rob")
    messages = {m["document"]["type"]: m["document"] for m in record["messages"]}
    transfer = messages["transfer_custody"]["payload"]
    assert transfer["thing"] == "rl:thing:box1"
    criterion = transfer["criteria"][0]
    assert (criterion["subject"], criterion["property"], criterion["value"]) == (
        "rl:thing:box1", "located_in", "rl:place:handoff")
    observation = messages["custody_confirmed"]["payload"]["observations"][0]
    assert observation["source"]["participant"] == AMR and observation["signature"]["key"] == AMR + "#k1"
    assert observation["value"] == "rl:place:handoff"
    released = messages["custody_released"]
    assert released["payload"]["confirmation"] == messages["custody_confirmed"]["message_id"]
    acknowledge = messages["acknowledge"]
    assert acknowledge["payload"] == {"acknowledged": released["message_id"], "accepted": True}
    # the plan continues only after the exchange: the outcome comes after the acknowledgement
    times = [m["t_ms"] for m in record["messages"]]
    assert times == sorted(times) and times[-1] > times[-2]


def test_custody_and_commitment_states_are_events():
    record, _ = record_of(*BASE, "--plans", "examples/handover.rob")
    assert custody(record, ARM) == ["OFFERED", "RELEASE_SENT", "RELEASED"]
    assert custody(record, AMR) == ["CONFIRMATION_SENT", "HOLDING"]
    assert [e["state"] for e in events(record, "commitment")] == ["active", "fulfilled"]
    assert event(record, "custody_point")["step"] == 8
    assert {e["actor"] for e in events(record, "authorization")} == {ARM, AMR}
    assert all(e["basis"] == "standing_authorization" and e["mode"] == "simulation"
               for e in events(record, "authorization"))
    committed = event(record, "custody_committed")
    release = next(m for m in record["messages"] if m["document"]["type"] == "custody_released")
    assert committed["release"] == release["document"]["message_id"]


def test_the_final_record_is_a_normal_run_of_the_whole_plan():
    record, _ = record_of(*BASE, "--plans", "examples/handover.rob")
    run_record = record["robot_run"]
    assert [s["status"] for s in run_record["steps"]] == ["succeeded"] * 10
    assert run_record["goal"] and all(g["result"] == "true" for g in run_record["goal"])


def test_sequences_and_scoped_trust():
    record, _ = record_of(*BASE, "--plans", "examples/handover.rob")
    streams = {}
    for entry in record["messages"]:
        m = entry["document"]
        streams.setdefault((m["sender"], m["recipient"]), []).append(m["sequence"])
    assert streams == {(PLANNER, ARM): [1], (ARM, PLANNER): [1, 2, 3], (ARM, AMR): [1, 2, 3, 4], (AMR, ARM): [1, 2, 3]}
    for entry in record["messages"]:
        message = entry["document"]
        keys = trust_keys(PLANNER, ARM, AMR, SIM_KEY_REF, SIM_KEY)[message["recipient"]]
        context = Context(receiver=message["recipient"], now_ms=time_ms(core_time(entry["t_ms"])),
                          clock_tolerance_ms=500, max_message_lifetime_ms=60_000, keys=keys)
        assert check_message(json.dumps(message).encode(), context).accepted, message["type"]


def test_the_peer_trusts_only_the_robot():
    trust = trust_keys(PLANNER, ARM, AMR, SIM_KEY_REF, SIM_KEY)
    assert set(trust[AMR]) == {ARM + "#k1"}
    assert set(trust[ARM]) == {PLANNER + "#k1", AMR + "#k1"} and set(trust[PLANNER]) == {ARM + "#k1", SIM_KEY_REF}
    message = {"schema": "robolang.core.message", "schema_version": "0.1", "message_id": "m-x",
               "conversation_id": "rl:conv:test-1", "sequence": 1, "in_reply_to": None, "profile": "r2r",
               "type": "decline",
               "sender": PLANNER, "recipient": AMR, "sent_at": core_time(0), "expires_at": core_time(1000),
               "payload": {"goal": "rl:goal:g", "reasons": []}}
    signed = sign_sim(message, PLANNER + "#k1", simulation_key(PLANNER + "#k1"))
    context = Context(receiver=AMR, now_ms=time_ms(core_time(10)), clock_tolerance_ms=500,
                      max_message_lifetime_ms=60_000, keys=trust[AMR])
    receipt = check_message(json.dumps(signed).encode(), context)
    assert (receipt.code, receipt.stage) == ("C203", "signature")  # a key outside the peer's context


def test_record_is_deterministic():
    assert run(*BASE, "--plans", "examples/handover.rob") == run(*BASE, "--plans", "examples/handover.rob")


# -------- single-robot delegations are unchanged --------

def test_single_robot_record_is_byte_identical_to_before_peers():
    code, out, _ = run(*LIBRARY)
    assert code == 0
    # sha256 of the stdout of this command before decision 0022
    digest = "3dfc87bbaa2822a05a9cdd78f3bca45da678deddfb595235d67e9bd9aaca1137"
    assert hashlib.sha256(out.encode()).hexdigest() == digest
    record = json.loads(out)
    assert record["version"] == "0.1" and "peer" not in record and "commitment" not in record


# -------- the peer declines --------

def test_a_peer_that_is_not_ready_declines_and_the_robot_rejects_the_goal(tmp_path):
    def not_ready(document):
        document["robots"]["amr1"]["ready"] = False

    world = robot_world(tmp_path, {}, not_ready)
    code, out, err = run(*BASE, "--plans", "examples/handover.rob", "--robot-world", world)
    record = json.loads(out)
    assert code == 7
    assert types(record) == ["propose_goal", "assessment", "request_assistance", "decline", "assessment", "reject"]
    assert all(m["receipt"]["status"] == "accepted" for m in record["messages"])
    decline = record["messages"][3]["document"]
    assert decline["sender"] == AMR and decline["payload"]["reasons"] == [{"code": "participant_busy"}]
    assert record["messages"][4]["document"]["payload"] == {
        "goal": "rl:goal:handover", "result": "infeasible", "reasons": [{"code": "assistance_unavailable"}]}
    assert record["messages"][5]["document"]["payload"]["reasons"] == [{"code": "assistance_unavailable"}]
    assert record["planner_verdict"] == {
        "status": "rejected", "criteria": [], "reasons": [{"code": "assistance_unavailable"}]}
    assert record["robot_run"] is None and record["commitment"] is None and record["peer"] == AMR
    assert "rejected (assistance_unavailable)" in err


# -------- the lease --------

def test_a_lease_that_ends_before_the_custody_point_lapses_the_commitment():
    code, out, err = run(*BASE, "--plans", "examples/handover.rob", "--lease", "1s")
    record = json.loads(out)
    assert code == 4
    assert types(record) == ["propose_goal", "assessment", "request_assistance", "offer_assistance", "commit",
                             "accept", "outcome"]
    assert "transfer_custody" not in types(record)
    outcome = record["messages"][-1]["document"]["payload"]["outcome"]
    assert outcome["status"] == "failed" and outcome["reasons"] == [{"code": "lease_expired"}]
    assert outcome["verification"] is None
    # the run was cancelled before the peer's step; it is still a normal run record
    assert record["robot_run"]["state"] == "CANCELLED" and record["robot_run"]["cancelled_before_step"] == 8
    assert record["commitment"]["state"] == "lapsed"
    assert [e["state"] for e in events(record, "commitment")] == ["active", "lapsed"]
    assert event(record, "lease_expired")["commitment"] == record["commitment"]["id"]
    assert record["planner_verdict"]["status"] == "failed" and record["planner_verdict"]["reasons"] == []
    assert custody(record, ARM) == [] and "failed (verified)" in err


def test_a_wait_longer_than_the_lease_lapses_it(tmp_path):
    code, out, _ = run(*BASE, "--plans", slow_plan(tmp_path, 70))
    record = json.loads(out)
    assert code == 4 and "transfer_custody" not in types(record)
    assert record["messages"][-1]["document"]["payload"]["outcome"]["reasons"] == [{"code": "lease_expired"}]
    assert record["commitment"]["state"] == "lapsed" and record["robot_run"]["cancelled_before_step"] == 9
    expired = event(record, "lease_expired")["t_ms"]
    assert expired == 150 + 60_000 and record["messages"][-1]["t_ms"] >= expired


def test_a_longer_lease_covers_the_longer_wait(tmp_path):
    record, code = record_of(*BASE, "--plans", slow_plan(tmp_path, 70), "--lease", "2min")
    assert code == 0 and types(record) == FULL
    lease = record["commitment"]["lease"]
    assert time_ms(lease["expires_at"]) - time_ms(lease["granted_at"]) == 120_000


def test_the_exchange_must_finish_inside_the_lease():
    # custody point at 5300 ms; the exchange ends 200 ms later; the lease ends at 150 ms + lease, less 500 ms
    holds, _ = record_of(*BASE, "--plans", "examples/handover.rob", "--lease", "5851ms")
    assert types(holds) == FULL
    lapses, _ = record_of(*BASE, "--plans", "examples/handover.rob", "--lease", "5850ms")
    assert "transfer_custody" not in types(lapses) and lapses["commitment"]["state"] == "lapsed"


@pytest.mark.parametrize("text, expected", [("60s", 60_000), ("500ms", 500), ("2min", 120_000), ("0.5s", 500)])
def test_lease_durations(text, expected):
    assert lease_ms(text) == expected


@pytest.mark.parametrize("text", ["", "60", "0s", "-1s", "1.5ms", "25h", "1441min", "1 s", "abc", "1d"])
def test_invalid_leases_are_usage_errors(text):
    with pytest.raises(ValueError):
        lease_ms(text)
    code, out, err = run(*BASE, "--plans", "examples/handover.rob", f"--lease={text}")
    assert code == 2 and out == "" and "invalid lease" in err


# -------- the custody confirmation does not match --------

def test_a_confirmation_of_the_thing_elsewhere_is_not_released(monkeypatch):
    def elsewhere(message):
        observation = message["payload"]["observations"][0]
        observation["value"] = "rl:place:shelf"
        message["payload"]["observations"][0] = sign_sim(observation, AMR + "#k1", simulation_key(AMR + "#k1"))

    intercept_with(monkeypatch, elsewhere, "custody_confirmed", resign=AMR)
    code, out, err = run(*BASE, "--plans", "examples/handover.rob")
    record = json.loads(out)
    assert code == 4
    assert types(record) == FULL[:8] + ["outcome"]  # no custody_released, no acknowledge
    assert all(m["receipt"]["status"] == "accepted" for m in record["messages"])
    outcome = record["messages"][-1]["document"]["payload"]["outcome"]
    assert outcome["status"] == "failed" and outcome["reasons"] == [{"code": "custody_unknown"}]
    rejected = event(record, "confirmation_not_accepted")
    assert rejected["reason"] == "criterion_not_met"
    assert custody(record, ARM) == ["OFFERED", "HOLDING"] and custody(record, AMR) == ["CONFIRMATION_SENT"]
    assert record["robot_run"]["state"] == "CANCELLED" and record["robot_run"]["cancelled_before_step"] == 8
    assert record["commitment"]["state"] == "failed"
    assert record["planner_verdict"]["status"] == "failed"


def test_a_confirmation_that_is_not_the_peers_own_is_not_released(monkeypatch):
    def borrowed(message):  # an observation by a participant the robot trusts, but not by the peer
        observation = message["payload"]["observations"][0]
        observation["source"]["participant"] = PLANNER
        message["payload"]["observations"][0] = sign_sim(observation, PLANNER + "#k1", simulation_key(PLANNER + "#k1"))

    intercept_with(monkeypatch, borrowed, "custody_confirmed", resign=AMR)
    code, out, _ = run(*BASE, "--plans", "examples/handover.rob")
    record = json.loads(out)
    assert code == 4 and "custody_released" not in types(record)
    assert event(record, "confirmation_not_accepted")["reason"] == "evidence_missing"


def test_a_stale_confirmation_is_not_released(monkeypatch):
    def stale(message):
        observation = message["payload"]["observations"][0]
        observation["valid_for"] = {"value": "1", "unit": "ms"}  # gone before the robot looks
        message["payload"]["observations"][0] = sign_sim(observation, AMR + "#k1", simulation_key(AMR + "#k1"))

    intercept_with(monkeypatch, stale, "custody_confirmed", resign=AMR)
    code, out, _ = run(*BASE, "--plans", "examples/handover.rob")
    record = json.loads(out)
    assert code == 4 and "custody_released" not in types(record)
    assert event(record, "confirmation_not_accepted")["reason"] == "evidence_stale"


# -------- tampering --------

def test_a_tampered_offer_is_rejected_by_its_signature(monkeypatch):
    def longer(message):
        message["payload"]["lease"]["expires_at"] = core_time(3_600_000)

    intercept_with(monkeypatch, longer, "offer_assistance")
    code, out, err = run(*BASE, "--plans", "examples/handover.rob")
    record = json.loads(out)
    assert code == 7
    last = record["messages"][-1]
    assert last["document"]["type"] == "offer_assistance" and last["receipt"] == {"status": "rejected", "code": "C202"}
    reason = record["planner_verdict"]["reasons"][0]
    assert (reason["code"], reason["receiver"], reason["message"]) == ("C202", ARM, "offer_assistance")
    assert record["robot_run"] is None and record["planner_verdict"]["status"] == "unknown"
    assert "unknown (C202)" in err


@pytest.mark.parametrize("mtype, receiver", [
    ("request_assistance", AMR), ("commit", AMR), ("transfer_custody", AMR), ("custody_released", AMR),
    ("custody_confirmed", ARM), ("acknowledge", ARM)])
def test_any_tampered_message_of_the_exchange_is_rejected(monkeypatch, mtype, receiver):
    def retarget(message):
        message["in_reply_to"] = "m-tampered"

    intercept_with(monkeypatch, retarget, mtype)
    code, out, _ = run(*BASE, "--plans", "examples/handover.rob")
    record = json.loads(out)
    assert code == 7
    assert record["messages"][-1]["receipt"] == {"status": "rejected", "code": "C202"}
    assert record["planner_verdict"]["reasons"][0]["receiver"] == receiver


def test_a_release_naming_another_confirmation_is_rejected_c403(monkeypatch):
    def other(message):
        message["payload"]["confirmation"] = "m-0123456789abcdef0123456789abcdef"

    intercept_with(monkeypatch, other, "custody_released", resign=ARM)
    code, out, _ = run(*BASE, "--plans", "examples/handover.rob")
    record = json.loads(out)
    assert code == 7
    assert record["messages"][-1]["receipt"] == {"status": "rejected", "code": "C403"}
    assert record["planner_verdict"]["reasons"][0]["receiver"] == AMR
    assert "acknowledge" not in types(record) and custody(record, AMR) == ["CONFIRMATION_SENT"]


# -------- other plans --------

def test_a_peer_without_a_custody_point_has_a_commitment_and_no_custody_exchange(tmp_path):
    text = ("robolang 1.3;\ntask pair(robot r, robot p, object box, location src, location dst) {\n"
            "    goal { object_at box dst; }\n"
            "    plan { move r to src; grasp r box; move r to dst; place r box at dst; move p to dst; }\n}\n")
    plan = write(tmp_path, "pair.rob", text)
    world = robot_world(tmp_path, {"pair": {"bindings": {**BINDINGS, "p": "amr1"}}})
    record, code = record_of(*BASE, "--plans", plan, "--robot-world", world)
    assert code == 0 and types(record) == [
        "propose_goal", "assessment", "request_assistance", "offer_assistance", "commit", "accept", "outcome"]
    assert record["commitment"]["state"] == "fulfilled" and events(record, "custody") == []


def test_a_peer_grasping_what_the_robot_did_not_place_is_no_custody_point():
    source = TASK.format(name="t", goal="object_at box dst;", plan="move r to src; grasp r box; place r box at dst;")
    task = compile_source(source, "t.rob").ir.tasks[0]
    # the arm only grasps: nothing was placed, so the AMR grasping later is not a custody point
    assert custody_point(task, {"r": "arm1", "box": "box1", "src": "shelf", "dst": "table"}, "arm1", "amr1") is None
    handover = compile_source(HANDOVER, "handover.rob").ir.tasks[0]
    bindings = {"arm": "arm1", "mobile": "amr1", "box": "box1", "src": "shelf", "handoff": "handoff",
                "dst": "table", "zone": "handoff_zone"}
    assert custody_point(handover, bindings, "arm1", "amr1") == CustodyPoint(8, "box1", "handoff")
    assert custody_point(handover, bindings, "amr1", "arm1") is None


def test_a_run_that_fails_before_the_custody_point_is_the_final_run(tmp_path):
    def broken(document):
        document["tasks"]["handover"]["faults"] = [{"step": 2}]

    world = robot_world(tmp_path, {}, broken)
    record, code = record_of(*BASE, "--plans", "examples/handover.rob", "--robot-world", world)
    assert code == 4 and "transfer_custody" not in types(record)
    assert record["robot_run"]["state"] == "FAILED" and record["robot_run"]["failure"]["code"] == "S251"
    assert record["commitment"]["state"] == "failed" and record["planner_verdict"]["status"] == "failed"


def test_a_scripted_cancel_before_the_custody_point_ends_the_run_without_an_exchange(tmp_path):
    def cancel(document):
        document["tasks"]["handover"]["cancel"] = {"before_step": 3}

    record, code = record_of(*BASE, "--plans", "examples/handover.rob",
                             "--robot-world", robot_world(tmp_path, {}, cancel))
    assert code == 4 and "transfer_custody" not in types(record)
    assert record["robot_run"]["cancelled_before_step"] == 3
    assert record["planner_verdict"]["status"] == "cancelled" and record["commitment"]["state"] == "failed"


def test_a_fault_after_the_exchange_fails_the_run_but_the_custody_is_released(tmp_path):
    def broken(document):
        document["tasks"]["handover"]["faults"] = [{"step": 9}]

    record, code = record_of(*BASE, "--plans", "examples/handover.rob",
                             "--robot-world", robot_world(tmp_path, {}, broken))
    assert code == 4 and types(record) == FULL
    assert record["robot_run"]["failure"]["code"] == "S251" and record["commitment"]["state"] == "failed"
    assert custody(record, ARM) == ["OFFERED", "RELEASE_SENT", "RELEASED"]


def test_a_task_that_binds_three_robots_is_not_served(tmp_path):
    text = ("robolang 1.3;\ntask trio(robot r, robot p, robot q, object box, location src, location dst) {\n"
            "    goal { object_at box dst; }\n"
            "    plan { move r to src; grasp r box; move r to dst; place r box at dst; }\n}\n")
    plan = write(tmp_path, "trio.rob", text)

    def third(document):
        document["robots"]["amr2"] = dict(document["robots"]["amr1"])

    world = robot_world(tmp_path, {"trio": {"bindings": {**BINDINGS, "p": "amr1", "q": "amr2"}}}, third)
    record, code = record_of(*BASE, "--plans", plan, "--robot-world", world)
    assert code == 7 and types(record) == ["propose_goal", "reject"]
    assert event(record, "plans_considered")["plans"][0]["result"] == "robots_unsupported"


# -------- the spec --------

def test_spec_documents_the_exchange_and_the_failures():
    for text in ("request_assistance", "offer_assistance", "transfer_custody", "custody_confirmed", "custody_released",
                 "lease_expired", "custody_unknown", "assistance_unavailable", "participant_busy", "--lease", "C403",
                 "0.2", "robots_unsupported"):
        assert text in SPEC


# -------- the lease holds for every act of the peer, not only the custody exchange --------

def wait_after_custody(tmp_path, seconds: int) -> str:
    """The handover plan with a wait after the peer takes the box, before the peer's next step (step 10)."""
    text = HANDOVER.replace("        move mobile to dst;", f"        wait for {seconds}s;\n        move mobile to dst;")
    return write(tmp_path, "late.rob", text)


def test_a_peer_step_after_the_lease_is_not_run(tmp_path):
    code, out, _ = run(*BASE, "--plans", wait_after_custody(tmp_path, 70))
    record = json.loads(out)
    assert code == 4
    assert types(record) == FULL  # the custody exchange itself was inside the lease
    assert record["robot_run"]["cancelled_before_step"] == 10  # `move mobile to dst` would start too late
    assert record["commitment"]["state"] == "lapsed"
    outcome = record["messages"][-1]["document"]["payload"]["outcome"]
    assert outcome["status"] == "failed" and outcome["reasons"] == [{"code": "lease_expired"}]
    assert record["planner_verdict"]["status"] == "failed"


def test_a_lease_that_covers_the_peers_last_step_succeeds(tmp_path):
    record, code = record_of(*BASE, "--plans", wait_after_custody(tmp_path, 70), "--lease", "2min")
    assert code == 0 and record["commitment"]["state"] == "fulfilled"
