"""Tests for `robolang delegate` (spec/delegate-v0.1.md, decision record 0020)."""

from __future__ import annotations
import copy
import functools
import io
import json
import os
from pathlib import Path

import pytest

from robolang import cli
from robolang.cli import main
from robolang.bridge import core_time
from robolang.core import Context, check_message, sign_sim, time_ms
from robolang.delegate import DelegateError, delegate, verify_outcome
from robolang.export_goal import simulation_key
from robolang.ir import compile_source
from robolang.sim import load_world
from robolang.sim.outcome import SIM_KEY, SIM_KEY_REF

REPO_ROOT = Path(__file__).resolve().parent.parent
WORLD = "examples/world.json"
PLANNER, ARM = "rl:agent:planner-1", "rl:robot:arm1"
SPEC = (REPO_ROOT / "spec" / "delegate-v0.1.md").read_text(encoding="utf-8")
BASE = ["examples/handover.rob", "--world", WORLD, "--from", PLANNER, "--to", ARM]
LIBRARY = [*BASE, "--plans", "examples/plans"]

TASK = """robolang 1.3;
task {name}(robot r, object box, location src, location dst) {{
    goal {{ {goal} }}
    plan {{ {plan} }}
}}
"""
BINDINGS = {"r": "arm1", "box": "box1", "src": "shelf", "dst": "table"}


def run(*argv: str):
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(REPO_ROOT)
    try:
        code = main(["delegate", *argv], out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


def record_of(*argv: str) -> tuple[dict, int]:
    code, out, err = run(*argv)
    assert out, err
    return json.loads(out), code


def write(tmp_path, name: str, text: str) -> str:
    path = tmp_path / name
    path.write_text(text, encoding="utf-8")
    return str(path)


def plan_file(tmp_path, name: str, goal: str, plan: str) -> str:
    return write(tmp_path, f"{name}.rob", TASK.format(name=name, goal=goal, plan=plan))


def robot_world(tmp_path, tasks: dict, edit=None) -> str:
    """A copy of examples/world.json with extra task entries, and `edit(document)` applied."""
    document = json.loads((REPO_ROOT / WORLD).read_text(encoding="utf-8"))
    document["tasks"].update(tasks)
    if edit:
        edit(document)
    return write(tmp_path, "robot_world.json", json.dumps(document))


def context(entry: dict) -> Context:
    message = entry["document"]
    ref = message["signature"]["key"]
    keys = {ref: simulation_key(ref), SIM_KEY_REF: SIM_KEY}
    return Context(receiver=message["recipient"], now_ms=time_ms(core_time(entry["t_ms"])), clock_tolerance_ms=500,
                   max_message_lifetime_ms=60_000, keys=keys)


def types(record: dict) -> list[str]:
    return [m["document"]["type"] for m in record["messages"]]


def event(record: dict, kind: str) -> dict:
    return next(e for e in record["events"] if e["event"] == kind)


# -------- the happy path --------

def test_handover_goal_is_served_by_the_robots_own_plan():
    code, out, err = run(*LIBRARY)
    record = json.loads(out)
    assert code == 0, err
    assert err.strip() == "simulation only: delegate rl:goal:handover → rl:robot:arm1: succeeded (verified)"
    assert record["delegate_record_schema"] == "robolang.delegate" and record["version"] == "0.1"
    assert record["mode"] == "simulation" and record["goal"] == "rl:goal:handover"
    assert (record["planner"], record["robot"]) == (PLANNER, ARM)
    assert types(record) == ["propose_goal", "assessment", "accept", "outcome"]
    assert all(m["receipt"] == {"status": "accepted", "code": None} for m in record["messages"])
    # the robot ran its OWN task, not the AI's
    matched = event(record, "plan_matched")
    assert matched["task"] == "relocate_box" and matched["file"] == "examples/plans/relocate_box.rob"
    assert record["robot_run"]["inputs"]["task"] == "relocate_box" and record["robot_run"]["state"] == "SUCCEEDED"
    assert event(record, "authorization")["mode"] == "simulation"
    assert [p["result"] for p in event(record, "plans_considered")["plans"]] == ["goal_differs", "matched"]
    verdict = record["planner_verdict"]
    assert verdict["status"] == "succeeded" and verdict["reasons"] == []
    evidence = "rl:obs:" + record["robot_run"]["run_id"][4:20] + "-0"
    assert verdict["criteria"] == [{"criterion": 0, "met": True, "evidence": [evidence], "reason": None}]


def test_outcome_names_the_proposed_goal_and_the_robot_reports_it():
    record, _ = record_of(*LIBRARY)
    outcome = record["messages"][3]["document"]["payload"]["outcome"]
    assert outcome["goal"] == "rl:goal:handover"  # not rl:goal:relocate_box
    assert outcome["reported_by"] == ARM and outcome["verification"]["verified_by"] == "rl:sim:world"
    # the run record keeps the run's own view of its goal
    assert record["robot_run"]["core_outcome"]["outcome"]["goal"] == "rl:goal:relocate_box"


def test_sequences_count_per_sender_and_recipient():
    record, _ = record_of(*LIBRARY)
    streams = {}
    for entry in record["messages"]:
        m = entry["document"]
        streams.setdefault((m["sender"], m["recipient"]), []).append(m["sequence"])
    assert streams == {(PLANNER, ARM): [1], (ARM, PLANNER): [1, 2, 3]}
    assert len({m["document"]["conversation_id"] for m in record["messages"]}) == 1


def test_every_message_passes_check_message():
    record, _ = record_of(*LIBRARY)
    for entry in record["messages"]:
        receipt = check_message(json.dumps(entry["document"]).encode(), context(entry))
        assert receipt.accepted, (entry["document"]["type"], receipt)
    times = [time_ms(m["document"]["sent_at"]) for m in record["messages"]]
    assert times == sorted(times)


def test_default_library_is_the_file_itself():
    record, code = record_of(*BASE)
    assert code == 0 and event(record, "plan_matched")["task"] == "handover"


def test_record_is_deterministic():
    first, second = run(*LIBRARY), run(*LIBRARY)
    assert first == second and first[1].endswith("\n")


# -------- the robot says no --------

def test_no_matching_plan_is_rejected():
    code, out, err = run(*BASE, "--plans", "examples/plans/park_robot.rob")
    record = json.loads(out)
    assert code == 7
    assert types(record) == ["propose_goal", "reject"]
    reject = record["messages"][1]["document"]["payload"]
    assert reject["goal"] == "rl:goal:handover" and reject["reasons"] == [{"code": "capability_unsupported"}]
    assert record["planner_verdict"] == {
        "status": "rejected", "criteria": [], "reasons": [{"code": "capability_unsupported"}]}
    assert record["robot_run"] is None
    assert "rejected (capability_unsupported)" in err


def test_robot_not_bound_in_the_matching_plan_is_rejected():
    code, out, _ = run("examples/handover.rob", "--world", WORLD, "--from", PLANNER, "--to", "rl:robot:amr1",
                       "--plans", "examples/plans")
    record = json.loads(out)
    assert code == 7 and types(record) == ["propose_goal", "reject"]
    assert [p["result"] for p in event(record, "plans_considered")["plans"]] == ["goal_differs", "robot_not_bound"]


def test_task_without_bindings_in_the_robot_world_is_skipped(tmp_path):
    plan = plan_file(tmp_path, "unbound", "object_at box dst;", "move r to dst;")
    code, out, _ = run(*BASE, "--plans", plan)
    assert code == 7
    assert event(json.loads(out), "plans_considered")["plans"][0]["result"] == "world_E506"


# -------- the run does not succeed --------

def test_failed_precondition_gives_a_failed_outcome(tmp_path):
    def not_ready(document):
        document["robots"]["arm1"]["ready"] = False

    record, code = record_of(*LIBRARY, "--robot-world", robot_world(tmp_path, {}, not_ready))
    assert code == 4
    assert record["robot_run"]["failure"]["code"] == "S101"
    outcome = record["messages"][3]["document"]["payload"]["outcome"]
    assert outcome["status"] == "failed" and outcome["verification"] is None
    assert record["planner_verdict"]["status"] == "failed"
    assert record["planner_verdict"]["criteria"][0]["reason"] == "evidence_missing"
    assert record["planner_verdict"]["reasons"] == []  # the claim and the evidence agree


def test_goal_not_met_is_failed_with_criteria_not_met(tmp_path):
    plan = plan_file(tmp_path, "lazy", "object_at box dst;",
                     "move r to src; grasp r box; move r to src; place r box at src;")
    world = robot_world(tmp_path, {"lazy": {"bindings": BINDINGS}})
    record, code = record_of(*BASE, "--plans", plan, "--robot-world", world)
    assert code == 4
    assert record["robot_run"]["failure"]["code"] == "S301"
    verdict = record["planner_verdict"]
    assert verdict["status"] == "failed" and verdict["reasons"] == []
    assert verdict["criteria"][0]["met"] is False and verdict["criteria"][0]["reason"] == "criterion_not_met"
    observed = record["messages"][3]["document"]["payload"]["evidence"][0]
    assert observed["value"] == "rl:place:shelf"  # the evidence shows where the box really is


def test_unknown_goal_is_unknown(tmp_path):
    plan = plan_file(tmp_path, "idle", "object_at box dst;", "wait for 1s;")

    def unknown_box(document):
        del document["objects"]["box1"]["present"], document["objects"]["box1"]["location"]

    world = robot_world(tmp_path, {"idle": {"bindings": BINDINGS}}, unknown_box)
    record, code = record_of(*BASE, "--plans", plan, "--robot-world", world)
    assert code == 4
    assert record["robot_run"]["failure"]["code"] == "S302"
    outcome = record["messages"][3]["document"]["payload"]["outcome"]
    assert outcome["status"] == "unknown" and outcome["reasons"] == [{"code": "evidence_missing"}]
    assert record["planner_verdict"]["status"] == "unknown"


def test_cancelled_run_is_cancelled(tmp_path):
    world = robot_world(tmp_path, {"relocate_box": {"bindings": BINDINGS, "cancel": {"before_step": 2}}})
    record, code = record_of(*LIBRARY, "--robot-world", world)
    assert code == 4 and record["planner_verdict"]["status"] == "cancelled"


# -------- the planner does not trust blindly --------

def tamper(change, resign=False):
    """A test seam: change the outcome message in flight, and sign it again as the robot if `resign`."""
    def intercept(message):
        if message["type"] == "outcome":
            message = copy.deepcopy(message)
            change(message)
            if resign:
                message = sign_sim(message, message["signature"]["key"], simulation_key(message["signature"]["key"]))
        return message
    return intercept


def test_tampered_evidence_is_rejected_by_the_planner(monkeypatch):
    def flip(message):
        message["payload"]["evidence"][0]["value"] = "rl:place:shelf"

    # a robot that signs the message again cannot sign the simulator's observation again
    monkeypatch.setattr(cli, "delegate", functools.partial(delegate, intercept=tamper(flip, resign=True)))
    code, out, err = run(*LIBRARY)
    record = json.loads(out)
    assert code == 7
    last = record["messages"][-1]
    assert last["receipt"] == {"status": "rejected", "code": "C202"}
    verdict = record["planner_verdict"]
    assert verdict["status"] == "unknown" and verdict["criteria"] == []
    assert verdict["reasons"][0]["code"] == "C202"
    assert verdict["reasons"][0]["pointer"] == "/payload/evidence/0/signature/value"
    assert "unknown (C202)" in err


def test_tampered_message_is_rejected_by_its_signature(monkeypatch):
    def claim(message):
        message["payload"]["outcome"]["status"] = "failed"

    monkeypatch.setattr(cli, "delegate", functools.partial(delegate, intercept=tamper(claim)))
    code, out, _ = run(*LIBRARY)
    assert code == 7
    assert json.loads(out)["planner_verdict"]["reasons"][0]["pointer"] == "/signature/value"


def _delivered_outcome():
    record, _ = record_of(*LIBRARY)
    goal = record["messages"][0]["document"]["payload"]["goal"]
    return goal, copy.deepcopy(record["messages"][3]["document"])


def _resign(message: dict) -> dict:
    observation = message["payload"]["evidence"][0]
    message["payload"]["evidence"][0] = sign_sim(observation, SIM_KEY_REF, SIM_KEY)
    return message


KEYS = {SIM_KEY_REF: SIM_KEY}


@pytest.mark.parametrize("change, reason", [
    (lambda o: o.update(value="rl:place:shelf"), "criterion_not_met"),
    (lambda o: o.update(confidence="0.5"), "confidence_too_low"),
    (lambda o: o.update(observed_at="1999-12-31T23:59:00.000Z"), "evidence_stale"),
    (lambda o: o.update(valid_for={"value": "1", "unit": "ms"}, observed_at="1999-12-31T23:59:59.000Z"),
     "evidence_stale"),
], ids=["wrong-value", "low-confidence", "too-old", "outlived-valid-for"])
def test_planner_re_evaluates_a_claimed_success(change, reason):
    goal, message = _delivered_outcome()
    change(message["payload"]["evidence"][0])
    verdict = verify_outcome(goal, _resign(message), {**KEYS, ARM + "#k1": simulation_key(ARM + "#k1")})
    assert verdict["status"] == "unknown"  # the robot claimed succeeded; the evidence does not carry it
    assert verdict["criteria"][0]["met"] is False and verdict["criteria"][0]["reason"] == reason
    assert verdict["reasons"] == [{"code": "claim_not_supported", "criterion": 0}]


def test_planner_checks_the_signature_it_trusts():
    goal, message = _delivered_outcome()
    message["payload"]["evidence"][0]["value"] = "rl:place:shelf"  # not re-signed
    verdict = verify_outcome(goal, message, KEYS)
    assert verdict["status"] == "unknown" and verdict["criteria"][0]["reason"] == "signature_invalid"


def test_planner_checks_goal_and_reporter():
    goal, message = _delivered_outcome()
    message["payload"]["outcome"]["goal"] = "rl:goal:other"
    message["payload"]["outcome"]["reported_by"] = "rl:robot:amr1"
    verdict = verify_outcome(goal, message, KEYS)
    assert verdict["status"] == "unknown"
    assert [r["code"] for r in verdict["reasons"]] == ["goal_mismatch", "reporter_mismatch"]


def test_a_succeeded_claim_needs_every_criterion():
    goal, message = _delivered_outcome()
    goal = copy.deepcopy(goal)
    goal["success_criteria"].append({**goal["success_criteria"][0], "subject": "rl:thing:box2"})
    verdict = verify_outcome(goal, message, KEYS)
    assert verdict["status"] == "unknown" and verdict["criteria"][1]["reason"] == "evidence_missing"


# -------- simulation only --------

def test_physical_mode_is_refused():
    source = (REPO_ROOT / "examples" / "handover.rob").read_text(encoding="utf-8")
    task = compile_source(source, "handover.rob").ir.tasks[0]
    world = load_world((REPO_ROOT / WORLD).read_bytes(), WORLD, task)
    with pytest.raises(DelegateError) as error:
        delegate(task, world, sender=PLANNER, recipient=ARM, library=[], robot_world=b"", mode="physical")
    assert error.value.code == "D701"


def test_record_and_messages_say_simulation_only():
    record, _ = record_of(*LIBRARY)
    assert record["mode"] == "simulation"
    assert {m["document"]["signature"]["alg"] for m in record["messages"]} == {"sim-hmac-sha256"}
    assert event(record, "authorization")["basis"] == "standing_authorization"


# -------- command line --------

def test_invalid_participant_is_a_usage_error():
    assert run("examples/handover.rob", "--world", WORLD, "--from", "planner", "--to", ARM)[0] == 2
    assert run("examples/handover.rob", "--world", WORLD, "--from", PLANNER, "--to", "rl:robot:a#b")[0] == 2


def test_plans_directory_without_rob_files_is_a_usage_error(tmp_path):
    code, _, err = run(*BASE, "--plans", str(tmp_path))
    assert code == 2 and "no .rob file" in err


def test_invalid_plan_file_exits_1(tmp_path):
    bad = write(tmp_path, "bad.rob", "robolang 1.3;\ntask t( {\n")
    code, out, err = run(*BASE, "--plans", bad)
    assert code == 1 and out == "" and "bad.rob" in err


def test_unexportable_goal_exits_6(tmp_path):
    source = plan_file(tmp_path, "t", "object_at box dst or object_at box src;", "move r to dst;")
    binds = [f"{k}={v}" for k, v in BINDINGS.items()]
    code, out, err = run(source, "--world", WORLD, "--from", PLANNER, "--to", ARM, "--bind", *binds)
    assert code == 6 and out == "" and "X601" in err


def test_robot_world_defaults_to_world(tmp_path):
    separate = run(*LIBRARY, "--robot-world", WORLD)
    assert separate == run(*LIBRARY)


def test_spec_documents_every_exit_code_and_the_limits():
    for code in ("`0`", "`4`", "`7`", "D701", "capability_unsupported", "rl:sim:world", "sim-hmac-sha256"):
        assert code in SPEC


CARRY = "move r to src; grasp r box; move r to dst; place r box at dst;"


def two_condition_case(tmp_path, robot_goal: str):
    ai = plan_file(tmp_path, "ai", "object_at box dst; robot_at r dst;", CARRY)
    mine = plan_file(tmp_path, "mine", robot_goal, CARRY)
    world = robot_world(tmp_path, {"ai": {"bindings": BINDINGS}, "mine": {"bindings": BINDINGS}})
    return record_of(ai, "--world", world, "--from", PLANNER, "--to", ARM, "--plans", mine)


def test_goal_conditions_match_in_any_order(tmp_path):
    record, code = two_condition_case(tmp_path, "robot_at r dst; object_at box dst;")
    assert code == 0 and record["planner_verdict"]["status"] == "succeeded"
    payload = record["messages"][3]["document"]["payload"]
    proposed = record["messages"][0]["document"]["payload"]["goal"]["desired_state"]
    results = payload["outcome"]["verification"]["criteria_results"]
    assert [r["criterion"] for r in results] == [0, 1]  # re-indexed to the proposal's order
    observed = {o["id"]: {k: o[k] for k in ("subject", "property", "value")} for o in payload["evidence"]}
    assert [observed[r["evidence"][0]] for r in results] == proposed


def test_and_goal_matches_separate_robot_goal_and_returns_one_result_per_atom(tmp_path):
    ai = plan_file(tmp_path, "ai", "object_at box dst and robot_at r dst;", CARRY)
    mine = plan_file(tmp_path, "mine", "robot_at r dst; object_at box dst;", CARRY)
    world = robot_world(tmp_path, {"ai": {"bindings": BINDINGS}, "mine": {"bindings": BINDINGS}})
    record, code = record_of(ai, "--world", world, "--from", PLANNER, "--to", ARM, "--plans", mine)
    assert code == 0 and record["planner_verdict"]["status"] == "succeeded"
    results = record["messages"][3]["document"]["payload"]["outcome"]["verification"]["criteria_results"]
    assert [(result["criterion"], result["met"]) for result in results] == [(0, True), (1, True)]


def test_a_subset_or_superset_of_the_goal_does_not_match(tmp_path):
    for goal in ("object_at box dst;", "object_at box dst; robot_at r dst; holding r box;"):
        record, code = two_condition_case(tmp_path, goal)
        assert code == 7 and record["planner_verdict"]["status"] == "rejected"
