"""
Core 0.2 producers (spec/core-v0.2.md, decision 0028 C5, C6 and the producer side of C1 to C3, PR B):
`export-goal --core 0.2`, the simulated outcome, `delegate --core 0.2` and the benchmark family that exports
"any of" goals (freeze criterion 3). Core 0.1 output stays byte for byte what it was.
"""

from __future__ import annotations
import io
import json
import os
from pathlib import Path

import pytest

from bench import lib
from robolang import bridge
from robolang.cli import main
from robolang.core import Context, check_message, sign_sim, time_ms
from robolang.delegate import verify_outcome
from robolang.export_goal import export_goal, simulation_key
from robolang.ir import compile_source
from robolang.sim.outcome import SIM_KEY, SIM_KEY_REF

ROOT = Path(__file__).resolve().parent.parent
SENDER, ARM2 = "rl:agent:planner-1", "rl:robot:arm2"
WAREHOUSE = "bench/worlds/warehouse.json"
ROUTE = "bench/reference_branch/b_route_by_cell.rob"
ROUTE_BIND = ["--task", "route", "--bind", "r=arm2", "box=box2", "src=shelf_b", "a=shelf_a", "b=table", "cell=cell"]
BOTH = frozenset({"0.1", "0.2"})

ROUTE_SOURCE = """robolang 1.4;
task route(robot r, object box, location src, location a, location b, region cell) {{
    plan {{
        move r to src;
        grasp r box;
        if {test} {{
            goal {{ object_at box {first}; }}
            move r to {first};
            place r box at {first};
        }} else {{
            goal {{ object_at box {second}; }}
            move r to {second};
            place r box at {second};
        }}
    }}
}}
"""


def invoke(*argv: str):
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(ROOT)
    try:
        code = main(list(argv), out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


def receiver(message: dict) -> Context:
    key_ref = message["signature"]["key"]
    return Context(receiver=message["recipient"], now_ms=time_ms(message["sent_at"]), clock_tolerance_ms=500,
                   max_message_lifetime_ms=60_000, keys={key_ref: simulation_key(key_ref)}, versions=BOTH)


def accepted(message: dict):
    receipt = check_message(json.dumps(message).encode("utf-8"), receiver(message))
    assert receipt.accepted, receipt
    return receipt


def export02(*extra: str, source: str = ROUTE, bind=ROUTE_BIND, to: str = ARM2):
    code, out, err = invoke("export-goal", source, "--world", WAREHOUSE, *bind, "--from", SENDER, "--to", to, *extra)
    return code, (json.loads(out) if out else None), err


def triple_sets(goal: dict) -> list[set]:
    return [{(d["subject"], d["property"], d["value"]) for d in a["desired_state"]} for a in goal["alternatives"]]


# -------- export-goal --------

HANDOVER = ["export-goal", "examples/handover.rob", "--world", "examples/world.json", "--from", SENDER,
            "--to", "rl:robot:arm1"]


def test_default_is_core_01_and_explicit_01_is_identical():
    _, default, _ = invoke(*HANDOVER)
    _, explicit, _ = invoke(*HANDOVER, "--core", "0.1")
    assert default == explicit
    message = json.loads(default)
    assert message["schema_version"] == "0.1" and "alternatives" not in message["payload"]["goal"]


def test_branching_goal_with_01_says_use_core_02():
    code, out, err = invoke("export-goal", ROUTE, "--world", WAREHOUSE, *ROUTE_BIND, "--from", SENDER, "--to", ARM2)
    assert code == 6 and not out and "--core 0.2" in err and "X603" in err


def test_a_plain_goal_is_one_alternative_in_02():
    code, out, err = invoke(*HANDOVER, "--core", "0.2")
    assert code == 0, err
    message = json.loads(out)
    assert message["schema_version"] == "0.2" and len(message["payload"]["goal"]["alternatives"]) == 1
    accepted(message)


def test_branching_exports_one_alternative_per_path_in_order():
    code, message, err = export02("--core", "0.2")
    assert code == 0, err
    goal = message["payload"]["goal"]
    assert message["schema_version"] == "0.2" and "desired_state" not in goal and "success_criteria" not in goal
    assert triple_sets(goal) == [{("rl:thing:box2", "located_in", "rl:place:shelf_a")},
                                 {("rl:thing:box2", "located_in", "rl:place:table")}]  # then before else
    accepted(message)


def exported_alternatives(tmp_path, source: str, bind: str):
    path = tmp_path / "t.rob"
    path.write_text(source, encoding="utf-8")
    return export02("--core", "0.2", source=str(path), bind=["--task", "route", "--bind", *bind.split()])


def test_identical_paths_are_one_alternative(tmp_path):
    source = ROUTE_SOURCE.format(test="region_clear cell", first="a", second="a")
    bind = "r=arm2 box=box2 src=shelf_b a=shelf_a b=table cell=cell"
    code, message, err = exported_alternatives(tmp_path, source, bind)
    assert code == 0, err
    assert len(message["payload"]["goal"]["alternatives"]) == 1
    # and, since it is one alternative, Core 0.1 can still send it
    path = tmp_path / "t.rob"
    code, out, err = invoke("export-goal", str(path), "--world", WAREHOUSE, "--task", "route", "--bind", "r=arm2",
                            "box=box2", "src=shelf_b", "a=shelf_a", "b=table", "cell=cell", "--from", SENDER,
                            "--to", ARM2)
    assert code == 0, err


def test_a_fail_path_adds_no_alternative():
    task = lib.load_set("branch")["b_missing_report"]
    checked = lib.check_answer(task, reference(task, lib.BRANCH_REFERENCE_DIR, "b_missing_report"))
    assert checked.valid, checked.report
    exported = export_goal(checked.task, checked.world, sender=SENDER, recipient="rl:robot:arm1", core="0.2")
    assert len(exported.message["payload"]["goal"]["alternatives"]) == 1
    with pytest.raises(bridge.PathsDiffer):  # Core 0.1 keeps X603 for a path that ends in fail
        export_goal(checked.task, checked.world, sender=SENDER, recipient="rl:robot:arm1")


def test_every_path_failing_is_not_exportable(tmp_path):
    source = """robolang 1.4;
task t(robot r, region cell) {
    plan { if region_clear cell { fail "a"; } else { fail "b"; } }
}
"""
    path = tmp_path / "t.rob"
    path.write_text(source, encoding="utf-8")
    code, out, err = invoke("export-goal", str(path), "--world", WAREHOUSE, "--bind", "r=arm2", "cell=cell",
                            "--from", SENDER, "--to", ARM2, "--core", "0.2")
    assert code == 6 and "no alternative" in err


OR_TASK = """robolang 1.3;
task t(robot r, object a, object b, location x, location y) {{
    goal {{ {goal} }}
    plan {{ move r to x; }}
}}
"""
OR_BIND = ["--bind", "r=arm1", "a=box1", "b=box2", "x=shelf_a", "y=shelf_b"]


def export_or(tmp_path, goal: str, *extra: str):
    path = tmp_path / "t.rob"
    path.write_text(OR_TASK.format(goal=goal), encoding="utf-8")
    code, out, err = invoke("export-goal", str(path), "--world", WAREHOUSE, *OR_BIND, "--from", SENDER,
                            "--to", "rl:robot:arm1", *extra)
    return code, (json.loads(out) if out else None), err


def test_top_level_or_of_conjunctions_exports_as_alternatives(tmp_path):
    goal = "(object_at a x and object_at b x) or (object_at a y and object_at b y);"
    code, message, err = export_or(tmp_path, goal, "--core", "0.2")
    assert code == 0, err
    assert triple_sets(message["payload"]["goal"]) == [
        {("rl:thing:box1", "located_in", "rl:place:shelf_a"), ("rl:thing:box2", "located_in", "rl:place:shelf_a")},
        {("rl:thing:box1", "located_in", "rl:place:shelf_b"), ("rl:thing:box2", "located_in", "rl:place:shelf_b")}]
    accepted(message)
    code, message, err = export_or(tmp_path, goal)
    assert code == 6 and "X601" in err and "--core 0.2" in err


@pytest.mark.parametrize("goal", [
    "object_at a x or not object_at a y;",            # not
    "object_at a x or robot_ready r;",                # robot_ready
    "robot_at r x; object_at a x or object_at a y;",  # mixed with a conjunction
    "not object_at a x;",
])
def test_what_is_not_expressible_stays_x601(tmp_path, goal):
    code, message, err = export_or(tmp_path, goal, "--core", "0.2")
    assert code == 6 and "X601" in err and message is None


def test_more_than_sixteen_alternatives_is_not_exportable():
    params = ", ".join(f"location l{i}" for i in range(17))
    goal = " or ".join(f"object_at box l{i}" for i in range(17))
    task = compile_source(f"robolang 1.3;\ntask t(robot r, object box, {params}) {{ goal {{ {goal}; }} "
                          "plan { move r to l0; } }\n", "t.rob").ir.tasks[0]
    bindings = {"r": "arm1", "box": "box1", **{f"l{i}": f"p{i}" for i in range(17)}}
    with pytest.raises(bridge.TooManyAlternatives, match="at most 16"):
        bridge.goal_alternatives(task, bindings)
    sixteen_goal = " or ".join(f"object_at box l{i}" for i in range(16))
    sixteen = compile_source(f"robolang 1.3;\ntask t(robot r, object box, {params}) {{ goal {{ {sixteen_goal}; }} "
                             "plan { move r to l0; } }\n", "t.rob").ir.tasks[0]
    assert len(bridge.goal_alternatives(sixteen, bindings)) == 16


# -------- the simulated outcome --------

def accepted_outcome(payload: dict, reported_by: str = "rl:robot:arm1") -> None:
    """The simulated outcome, as a signed `outcome` message, passes robolang.core 0.2."""
    sent_at = payload["outcome"]["reported_at"]
    key_ref = f"{reported_by}#k1"
    message = {"schema": "robolang.core.message", "schema_version": "0.2", "message_id": "m-" + "0" * 32,
               "conversation_id": "rl:conv:0", "sequence": 2, "in_reply_to": None, "profile": "a2r", "type": "outcome",
               "sender": reported_by, "recipient": SENDER, "sent_at": sent_at,
               "expires_at": bridge.core_time_after(sent_at, 1000), "payload": payload}
    context = Context(receiver=SENDER, now_ms=time_ms(sent_at), clock_tolerance_ms=500, max_message_lifetime_ms=60_000,
                      versions=BOTH, keys={key_ref: simulation_key(key_ref), SIM_KEY_REF: SIM_KEY})
    receipt = check_message(json.dumps(sign_sim(message, key_ref, simulation_key(key_ref))).encode("utf-8"), context)
    assert receipt.accepted, receipt


def simulate(source: str, world: str, bind, *extra: str) -> dict:
    code, out, err = invoke("simulate", source, "--world", world, *bind, *extra)
    assert out, err
    return json.loads(out)


def sim_world(tmp_path, cell_clear: bool) -> str:
    document = json.loads((ROOT / WAREHOUSE).read_text(encoding="utf-8"))
    document["regions"]["cell"]["clear"] = cell_clear
    path = tmp_path / f"w_{cell_clear}.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    return str(path)


def checked_outcome(record: dict) -> dict:
    accepted_outcome(record["core_outcome"])
    return record["core_outcome"]["outcome"]


@pytest.mark.parametrize("clear, met, place", [(True, 0, "shelf_a"), (False, 1, "table")])
def test_simulated_branching_run_names_the_path_taken(tmp_path, clear, met, place):
    record = simulate(ROUTE, sim_world(tmp_path, clear), ROUTE_BIND, "--core", "0.2")
    outcome = checked_outcome(record)
    assert outcome["status"] == "succeeded"
    verification = outcome["verification"]
    assert verification["met_alternative"] == met
    assert [(r["alternative"], r["criterion"], r["met"]) for r in verification["criteria_results"]] == [(met, 0, True)]
    observed = {o["id"]: o["value"] for o in record["core_outcome"]["evidence"]}
    assert observed[verification["criteria_results"][0]["evidence"][0]] == f"rl:place:{place}"


def test_default_simulate_is_unchanged_core_01(tmp_path):
    record = simulate(ROUTE, sim_world(tmp_path, True), ROUTE_BIND)
    assert "alternative" not in json.dumps(record["core_outcome"])


def test_other_alternative_holding_does_not_make_it_succeeded(tmp_path):
    """Takes the then path (shelf_a), ends with only the else goal true: failed, pursued alternative 0 (Q3)."""
    source = tmp_path / "t.rob"
    source.write_text("""robolang 1.4;
task route(robot r, object box, location src, location a, location b, region cell) {
    plan {
        if region_clear cell {
            goal { object_at box a; }
            move r to src; grasp r box; move r to b; place r box at b;
        } else {
            goal { object_at box b; }
            move r to src; grasp r box; move r to b; place r box at b;
        }
    }
}
""", encoding="utf-8")
    record = simulate(str(source), sim_world(tmp_path, True), ROUTE_BIND, "--core", "0.2")
    outcome = checked_outcome(record)
    assert outcome["status"] == "failed"
    verification = outcome["verification"]
    assert verification["met_alternative"] == 0
    assert [(r["alternative"], r["met"]) for r in verification["criteria_results"]] == [(0, False)]


def test_a_declared_failure_reports_failed_with_the_text(tmp_path):
    reference_task = lib.load_set("branch")["b_aisle_or_report"]
    answer = reference(reference_task, lib.BRANCH_REFERENCE_DIR, "b_aisle_or_report")
    bind = ["--task", answer["task"], "--bind", *[f"{k}={v}" for k, v in answer["bindings"].items()]]
    record = simulate(str(lib.BRANCH_REFERENCE_DIR / "b_aisle_or_report.rob"), reference_task["world"], bind,
                      "--core", "0.2")
    assert record["failure"]["code"] == "S262"
    outcome = checked_outcome(record)
    assert outcome["status"] == "failed" and outcome["verification"] is None
    assert outcome["reasons"] == [{"code": "declared", "text": record["failure"]["reason"]}]
    assert record["core_outcome"]["evidence"] == []


def test_a_goal_that_is_an_or_names_the_alternative_that_holds(tmp_path):
    path = tmp_path / "t.rob"
    path.write_text(OR_TASK.format(goal="(object_at a x) or (object_at a y);").replace(
        "move r to x;", "move r to x; grasp r a; move r to y; place r a at y;"), encoding="utf-8")
    record = simulate(str(path), WAREHOUSE, OR_BIND, "--core", "0.2")
    outcome = checked_outcome(record)
    assert outcome["status"] == "succeeded", record["failure"]
    verification = outcome["verification"]
    assert verification["met_alternative"] == 1  # the second operand holds; both are listed
    assert [(r["alternative"], r["met"]) for r in verification["criteria_results"]] == [(0, False), (1, True)]


# -------- delegate --------

def library_world(tmp_path, name: str, bindings: dict, cell_clear: bool = True) -> str:
    document = json.loads((ROOT / WAREHOUSE).read_text(encoding="utf-8"))
    document["regions"]["cell"]["clear"] = cell_clear
    document["tasks"] = {name: {"bindings": bindings}}
    path = tmp_path / f"robot_{name}.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    return str(path)


ROUTE_BINDINGS = {"r": "arm2", "box": "box2", "src": "shelf_b", "a": "shelf_a", "b": "table", "cell": "cell"}


def delegate(tmp_path, plan_source: str | None, cell_clear: bool = True, *extra: str):
    plans = ROUTE
    if plan_source is not None:
        plans = str(tmp_path / "mine.rob")
        Path(plans).write_text(plan_source, encoding="utf-8")
    world = library_world(tmp_path, "route", ROUTE_BINDINGS, cell_clear)
    code, out, err = invoke("delegate", ROUTE, "--world", WAREHOUSE, *ROUTE_BIND, "--from", SENDER, "--to", ARM2,
                            "--plans", plans, "--robot-world", world, *extra)
    return code, (json.loads(out) if out else None), err


def test_delegate_default_is_still_core_01_and_rejects_branching():
    code, out, err = invoke("delegate", ROUTE, "--world", WAREHOUSE, *ROUTE_BIND, "--from", SENDER, "--to", ARM2)
    assert code == 6 and "--core 0.2" in err and not out


@pytest.mark.parametrize("clear, met", [(True, 0), (False, 1)])
def test_delegate_core_02_runs_a_branching_task_and_names_the_alternative(tmp_path, clear, met):
    code, record, err = delegate(tmp_path, None, clear, "--core", "0.2")
    assert code == 0, err
    messages = record["messages"]
    assert {m["document"]["schema_version"] for m in messages} == {"0.2"}
    assert all(m["receipt"]["status"] == "accepted" for m in messages)
    verification = messages[3]["document"]["payload"]["outcome"]["verification"]
    assert verification["met_alternative"] == met
    verdict = record["planner_verdict"]
    assert verdict["status"] == "succeeded" and verdict["met_alternative"] == met and not verdict["reasons"]


def test_delegate_reindexes_to_the_proposals_alternatives(tmp_path):
    """The robot's task lists its alternatives in the other order: the outcome uses the proposal's indices."""
    mine = ROUTE_SOURCE.format(test="not region_clear cell", first="b", second="a")
    code, record, err = delegate(tmp_path, mine, True, "--core", "0.2")
    assert code == 0, err
    proposed = record["messages"][0]["document"]["payload"]["goal"]["alternatives"]
    assert proposed[0]["desired_state"][0]["value"] == "rl:place:shelf_a"
    verification = record["messages"][3]["document"]["payload"]["outcome"]["verification"]
    assert verification["met_alternative"] == 0  # cell clear: the robot's else path, its own alternative 1
    assert [(r["alternative"], r["criterion"]) for r in verification["criteria_results"]] == [(0, 0)]
    assert record["planner_verdict"]["status"] == "succeeded"


def test_delegate_does_not_match_a_task_with_other_alternatives(tmp_path):
    mine = ROUTE_SOURCE.format(test="region_clear cell", first="a", second="a")  # one alternative only
    code, record, err = delegate(tmp_path, mine, True, "--core", "0.2")
    assert code == 7 and record["planner_verdict"]["status"] == "rejected"
    considered = next(e for e in record["events"] if e["event"] == "plans_considered")
    assert considered["plans"][0]["result"] == "goal_differs"


def test_delegate_declared_failure_comes_back_failed(tmp_path):
    planner = tmp_path / "planner.rob"
    planner.write_text("""robolang 1.4;
task route(robot r, object box, location src, location a, location b, region cell) {
    plan {
        move r to src; grasp r box;
        if region_clear cell { goal { object_at box a; } move r to a; place r box at a; }
        else { fail "the cell is occupied"; }
    }
}
""", encoding="utf-8")
    world = library_world(tmp_path, "route", ROUTE_BINDINGS, cell_clear=False)
    code, out, err = invoke("delegate", str(planner), "--world", WAREHOUSE, *ROUTE_BIND, "--from", SENDER,
                            "--to", ARM2, "--plans", str(planner), "--robot-world", world, "--core", "0.2")
    record = json.loads(out)
    outcome = record["messages"][3]["document"]["payload"]["outcome"]
    assert code == 4 and outcome["status"] == "failed" and outcome["verification"] is None
    assert outcome["reasons"] == [{"code": "declared", "text": "the cell is occupied"}]
    assert record["planner_verdict"]["status"] == "failed"


def test_planner_rechecks_the_claimed_alternative_and_completeness(tmp_path):
    code, record, err = delegate(tmp_path, None, True, "--core", "0.2")
    goal = record["messages"][0]["document"]["payload"]["goal"]
    keys = {SIM_KEY_REF: SIM_KEY}
    outcome_message_ = record["messages"][3]["document"]

    def verdict(edit):
        message = json.loads(json.dumps(outcome_message_))
        edit(message["payload"]["outcome"])
        return verify_outcome(goal, message, keys)

    assert verdict(lambda o: None)["status"] == "succeeded"
    claim_other = verdict(lambda o: o["verification"].update(met_alternative=1))
    assert claim_other["status"] == "unknown"
    assert {"code": "claim_not_supported", "alternative": 1, "criterion": 0} in claim_other["reasons"]
    incomplete = verdict(lambda o: o["verification"].update(criteria_results=[]))
    assert incomplete["status"] == "unknown" and incomplete["reasons"]
    out_of_range = verdict(lambda o: o["verification"].update(met_alternative=5))
    assert out_of_range["status"] == "unknown"


# -------- the benchmark family (freeze criterion 3) --------

def reference(task: dict, directory: Path, task_id: str) -> dict:
    stem = json.loads((directory / f"{task_id}.json").read_text(encoding="utf-8"))
    return {"decision": "program", "category": "none", "reason": "ok",
            "rob": (directory / f"{task_id}.rob").read_text(encoding="utf-8"), **stem}


def family() -> list[tuple[str, dict, Path]]:
    found = []
    for name, directory in (("branch", lib.BRANCH_REFERENCE_DIR), ("hard", lib.HARD_REFERENCE_DIR)):
        for task_id, task in lib.load_set(name).items():
            if task["group"] == "branching" and lib.expectation(task)[0] == "program":
                found.append((task_id, task, directory))
    return found


FAMILY = family()


def test_the_family_is_set_3_and_set_2_branching_tasks():
    assert len(FAMILY) == 23


@pytest.mark.parametrize("task_id, task, directory", FAMILY, ids=[f[0] for f in FAMILY])
def test_branching_reference_exports_its_goal_paths_as_alternatives(task_id, task, directory):
    answer = reference(task, directory, task_id)
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    robots = [p.name for p in checked.task.parameters if p.type == "robot"]
    recipient = bridge.core_id("robot", answer["bindings"][robots[0]])
    message = export_goal(checked.task, checked.world, sender=SENDER, recipient=recipient, core="0.2").message
    accepted(message)
    expected = [frozenset(tuple(t) for t in p["goal"]) for p in lib.task_paths(task) if "fail" not in p]
    got = [frozenset(s) for s in triple_sets(message["payload"]["goal"])]
    assert set(got) == set(expected) and len(got) == len(set(got))


def test_the_f2_ambiguous_tasks_export_as_any_of():
    """m_one_vial_scanner ("one of the vials") and m_same_shelf ("the same shelf") written with a top-level `or`."""
    hard = lib.load_set("hard")
    assert {hard[t]["group"] for t in ("m_one_vial_scanner", "m_same_shelf")} == {"ambiguous"}
    world = json.loads((ROOT / hard["m_one_vial_scanner"]["world"]).read_text(encoding="utf-8"))
    vials = sorted(k for k in world["objects"] if k.startswith("vial"))
    assert len(vials) >= 2
    cases = {
        "m_one_vial_scanner": (
            "task deliver(robot r, object v1, object v2, location dst) {\n"
            "    goal { object_at v1 dst or object_at v2 dst; }\n    plan { move r to dst; }\n}",
            {"r": "arm1", "v1": vials[0], "v2": vials[1], "dst": "scanner"},
            [{(f"rl:thing:{vials[0]}", "located_in", "rl:place:scanner")},
             {(f"rl:thing:{vials[1]}", "located_in", "rl:place:scanner")}]),
        "m_same_shelf": (
            "task gather(robot r, object a, object b, location x, location y) {\n"
            "    goal { (object_at a x and object_at b x) or (object_at a y and object_at b y); }\n"
            "    plan { move r to x; }\n}",
            {"r": "arm1", "a": "box1", "b": "box2", "x": "shelf_a", "y": "shelf_b"},
            [{(f"rl:thing:{box}", "located_in", f"rl:place:{shelf}") for box in ("box1", "box2")}
             for shelf in ("shelf_a", "shelf_b")]),
    }
    for task_id, (task_source, bindings, expected) in cases.items():
        name = task_source.split()[1].split("(")[0]
        answer = {"decision": "program", "category": "none", "reason": "an any-of reading", "task": name,
                  "rob": "robolang 1.3;\n\n" + task_source + "\n", "bindings": bindings}
        checked = lib.check_answer(hard[task_id], answer)
        assert checked.valid, checked.report
        message = export_goal(checked.task, checked.world, sender=SENDER, recipient="rl:robot:arm1", core="0.2").message
        accepted(message)
        assert triple_sets(message["payload"]["goal"]) == expected
        with pytest.raises(bridge.NeedsCore02):
            export_goal(checked.task, checked.world, sender=SENDER, recipient="rl:robot:arm1")
