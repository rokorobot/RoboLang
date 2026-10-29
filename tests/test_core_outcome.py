"""
Core-shaped outcomes in run records (spec/simulation-v1.0.md section 10.5, decision 0018).

Every run record of a 1.3 source carries `core_outcome`: the outcome and evidence of a Core 0.1
`outcome` message. The outcome and each observation validate against the `$defs` of the Core
message schema, the observation signatures verify, and C402 holds.
"""

from __future__ import annotations
import io
import json
import os
from pathlib import Path

import jsonschema
import pytest

from robolang.cli import main
from robolang.core import verify_sim
from robolang.sim.outcome import OBSERVATION_VALID_FOR, SIM, SIM_KEY

ROOT = Path(__file__).parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "v1.3" / "simulation"
CASES = json.loads((FIXTURES / "cases.json").read_text(encoding="utf-8"))["cases"]
SCHEMA = json.loads((ROOT / "spec" / "core" / "message-0.1.schema.json").read_text(encoding="utf-8"))
BASE_WORLD = "tests/fixtures/v1.3/simulation/worlds/base.json"
BASE_V11_WORLD = "tests/fixtures/simulation/worlds/base.json"
BIND = ["--bind", "r=arm1", "box=box1", "src=shelf", "dst=table"]

MULTI = """robolang 1.3;
task multi(robot r, object box, location src, location dst) {
    goal { holding r box; robot_at r dst; object_at box dst; }
    plan { move r to src; grasp r box; move r to dst; }
}
"""
AND_GOAL = MULTI.replace(
    "holding r box; robot_at r dst; object_at box dst;",
    "holding r box and robot_at r dst;",
)
V12 = """robolang 1.2;
task plain(robot r, location dst) { plan { move r to dst; } }
"""


def validator(name: str) -> jsonschema.Draft202012Validator:
    return jsonschema.Draft202012Validator({"$ref": f"#/$defs/{name}", "$defs": SCHEMA["$defs"]})


def simulate(source: str, world: str, *extra: str, task: str | None = None, seed: str = "0") -> dict:
    argv = ["simulate", source, "--world", world, "--seed", seed, *extra]
    if task is not None:
        argv += ["--task", task]
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(ROOT)
    try:
        main(argv, out=out, err=err)
    finally:
        os.chdir(previous)
    return json.loads(out.getvalue())


def run_case(case_id: str) -> dict:
    case = next(c for c in CASES if c["id"] == case_id)
    return simulate(case["source"], case["world"], task=case["task"], seed=case["seed"])


def check_core(record: dict) -> dict | None:
    """Everything every non-null `core_outcome` must satisfy; returns it."""
    core = record["core_outcome"]
    if core is None:
        return None
    assert list(core) == ["outcome", "evidence"]
    outcome, evidence = core["outcome"], core["evidence"]
    validator("outcome").validate(outcome)
    validator("payload_outcome").validate(core)
    assert outcome["goal"] == f"rl:goal:{record['inputs']['task']}"
    assert outcome["reported_by"] == SIM
    for observation in evidence:
        validator("observation").validate(observation)
        assert verify_sim(observation, SIM_KEY)
        assert observation["observed_at"] == outcome["reported_at"]
        assert observation["valid_for"] == OBSERVATION_VALID_FOR
        assert observation["source"] == {"participant": SIM, "sensor": "world", "method": "simulation_ground_truth"}
    ids = [o["id"] for o in evidence]
    assert len(set(ids)) == len(ids) and all(i.startswith(f"rl:obs:{record['run_id'][4:20]}-") for i in ids)
    verification = outcome["verification"]
    cited = [i for r in (verification or {"criteria_results": []})["criteria_results"] for i in r["evidence"]]
    assert sorted(cited) == sorted(ids)  # every observation is cited, and only those
    if outcome["status"] == "succeeded":  # C402
        assert verification is not None and all(r["met"] for r in verification["criteria_results"])
    return core


def runs():
    for case in CASES:
        if case["expect"]["exit"] != 3:
            yield pytest.param(case, id=case["id"])


@pytest.mark.parametrize("case", list(runs()))
def test_every_fixture_run_has_a_valid_core_outcome(case):
    record = simulate(case["source"], case["world"], task=case["task"], seed=case["seed"])
    core = check_core(record)
    assert list(record)[-1] == "core_outcome"
    state = record["state"]
    if core is None:
        return  # a goal without a Core form
    status = core["outcome"]["status"]
    assert status == {"SUCCEEDED": "succeeded", "CANCELLED": "cancelled"}.get(
        state, "unknown" if record["failure"] and record["failure"]["code"] == "S302" else "failed")
    assert (core["outcome"]["verification"] is None) == all(g["result"] == "not_evaluated" for g in record["goal"])
    if core["outcome"]["verification"]:
        met = [r["met"] for r in core["outcome"]["verification"]["criteria_results"]]
        assert met == [g["result"] == "true" for g in record["goal"]]


def test_goal_met():
    record = simulate("examples/handover.rob", "examples/world.json")
    core = check_core(record)
    outcome, (observation,) = core["outcome"], core["evidence"]
    assert outcome["status"] == "succeeded" and outcome["reasons"] == []
    assert outcome["verification"] == {
        "verified_by": SIM, "method": "criteria_evaluated", "evaluated_at": "2000-01-01T00:00:05.000Z",
        "criteria_results": [{"criterion": 0, "met": True, "evidence": [observation["id"]]}],
    }
    assert (observation["subject"], observation["property"], observation["value"]) == (
        "rl:thing:box1", "located_in", "rl:place:table")
    assert (observation["confidence"], observation["observed_at"]) == ("1", outcome["reported_at"])


def test_goal_false_object_elsewhere():
    core = check_core(run_case("g2_goal_false_never_moved"))
    outcome, (observation,) = core["outcome"], core["evidence"]
    assert outcome["status"] == "failed" and outcome["reasons"] == []
    (result,) = outcome["verification"]["criteria_results"]
    assert result == {"criterion": 0, "met": False, "evidence": [observation["id"]]}
    # What is actually true, not the criterion: the box is still on the shelf.
    assert (observation["subject"], observation["property"], observation["value"]) == (
        "rl:thing:box1", "located_in", "rl:place:shelf")


def test_goal_false_object_held_and_met_means_observed_triple_equals_criterion(tmp_path):
    source = tmp_path / "multi.rob"
    source.write_text(MULTI, encoding="utf-8")
    record = simulate(str(source), BASE_WORLD, *BIND)
    core = check_core(record)
    assert core["outcome"]["status"] == "failed"
    observed = {o["id"]: (o["subject"], o["property"], o["value"]) for o in core["evidence"]}
    criteria = [
        ("rl:thing:box1", "carried_by", "rl:robot:arm1"),
        ("rl:robot:arm1", "located_in", "rl:place:table"),
        ("rl:thing:box1", "located_in", "rl:place:table"),
    ]
    results = core["outcome"]["verification"]["criteria_results"]
    assert [r["met"] for r in results] == [True, True, False]
    for result, criterion in zip(results, criteria):
        (obs_id,) = result["evidence"]
        assert result["met"] == (observed[obs_id] == criterion)
    # The unmet criterion cites what is true: the box is carried, not at the table.
    assert observed[results[2]["evidence"][0]] == criteria[0]


def test_and_goal_has_one_core_result_per_outcome_atom(tmp_path):
    source = tmp_path / "and_goal.rob"
    source.write_text(AND_GOAL, encoding="utf-8")
    core = check_core(simulate(str(source), BASE_WORLD, *BIND))
    assert core["outcome"]["status"] == "succeeded"
    results = core["outcome"]["verification"]["criteria_results"]
    assert [(result["criterion"], result["met"]) for result in results] == [(0, True), (1, True)]
    assert len(core["evidence"]) == 2


def test_goal_unknown_is_status_unknown_with_no_observation():
    core = check_core(run_case("g3_goal_object_unknown_presence"))
    outcome = core["outcome"]
    assert outcome["status"] == "unknown" and outcome["reasons"] == [{"code": "evidence_missing"}]
    assert outcome["verification"]["criteria_results"] == [{"criterion": 0, "met": False, "evidence": []}]
    assert core["evidence"] == []


def test_goal_absent_object_is_failed_with_no_observation():
    core = check_core(run_case("g3_goal_object_absent"))
    assert core["outcome"]["status"] == "failed" and core["evidence"] == []


def test_unknown_next_to_a_known_criterion_keeps_the_known_observation():
    core = check_core(run_case("g3_goal_true_and_unknown"))
    assert core["outcome"]["status"] == "unknown"
    assert [bool(r["evidence"]) for r in core["outcome"]["verification"]["criteria_results"]] == [True, False]


@pytest.mark.parametrize("case_id", [
    "g6_precondition_fails_goal_not_evaluated", "g6_precondition_unknown", "g6_step_fails_goal_not_evaluated",
    "g6_authorization_fails_goal_not_evaluated",
])
def test_run_that_ended_early_has_no_verification(case_id):
    core = check_core(run_case(case_id))
    assert core["outcome"]["status"] == "failed" and core["outcome"]["verification"] is None
    assert core["outcome"]["reasons"] == [] and core["evidence"] == []


def test_cancelled():
    core = check_core(run_case("g6_cancelled_goal_not_evaluated"))
    assert core["outcome"]["status"] == "cancelled" and core["outcome"]["verification"] is None
    assert core["evidence"] == []


def test_goal_without_core_form_has_null_core_outcome():
    record = run_case("g4_goal_combined")
    assert record["state"] == "SUCCEEDED" and "core_outcome" in record and record["core_outcome"] is None


def test_succeeded_needs_every_criterion_met_c402():
    succeeded = [r for r in (run_case(c["id"]) for c in CASES if c["expect"]["exit"] == 0) if r["core_outcome"]]
    assert succeeded
    for record in succeeded:
        outcome = check_core(record)["outcome"]
        assert outcome["status"] == "succeeded"
        assert all(r["met"] for r in outcome["verification"]["criteria_results"])
    # A failed run never claims success, even when some criteria are met.
    for case_id in ("g3_goal_true_and_unknown", "g2_goal_false_never_moved"):
        assert run_case(case_id)["core_outcome"]["outcome"]["status"] != "succeeded"


def test_tampered_observation_fails_verification():
    observation = dict(check_core(simulate("examples/handover.rob", "examples/world.json"))["evidence"][0])
    observation["value"] = "rl:place:shelf"
    assert not verify_sim(observation, SIM_KEY)


def test_records_are_deterministic_and_the_run_id_is_unchanged_by_the_outcome():
    a = simulate("examples/handover.rob", "examples/world.json")
    b = simulate("examples/handover.rob", "examples/world.json")
    assert a == b
    assert a["run_id"].startswith("sim-") and a["core_outcome"]["evidence"][0]["id"] == f"rl:obs:{a['run_id'][4:20]}-0"


def test_records_of_1_1_and_1_2_sources_have_no_core_outcome(tmp_path):
    v11 = simulate("examples/pick_and_place.rob", BASE_V11_WORLD)
    assert v11["run_record_schema_version"] == "1.0" and "core_outcome" not in v11
    source = tmp_path / "plain.rob"
    source.write_text(V12, encoding="utf-8")
    v12 = simulate(str(source), BASE_WORLD, "--bind", "r=arm1", "dst=table")
    assert v12["run_record_schema_version"] == "1.1" and "core_outcome" not in v12
