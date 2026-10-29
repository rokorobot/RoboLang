"""RoboLang 1.4 simulation (decision 0027, N4 PR 2): branches, `fail`, record 1.3, faults, Core."""

from __future__ import annotations
import io
import json
import os
from pathlib import Path

import pytest

from robolang.cli import main
from robolang.ir import compile_source
from robolang.sim import load_world, run, run_id, sha256_normalized, to_json, to_record

ROOT = Path(__file__).resolve().parent.parent
REFERENCE = ROOT / "bench" / "reference_branch"
TASKS = ROOT / "bench" / "tasks_branch"
IDS = sorted(p.name.removesuffix(".attempt.rob") for p in REFERENCE.glob("b_*.attempt.rob"))

# World patches that make the `when` of each path in expect_1_4["paths"] true, one list per path, in order.
# An op is ("<section>.<id>.<field>", value). Scripted events are dropped from a patched world, so that a
# condition cannot change between the start and the `if`.
PATCHES = {
    "b_aisle_or_report": [[("regions.aisle.clear", True)], [("regions.aisle.clear", False)]],
    "b_amr2_or_amr1": [[("robots.amr2.ready", True)], [("robots.amr2.ready", False)]],
    "b_amr_at_dock": [[("robots.amr1.location", "dock")], [("robots.amr1.location", "home")]],
    "b_arm_b_or_amr": [[("robots.arm_b.ready", True)], [("robots.arm_b.ready", False)]],
    "b_both_parts": [[("objects.p1.location", "conveyor_in"), ("objects.p2.location", "conveyor_in")],
                     [("objects.p2.location", "bin")]],
    "b_box_or_home": [[("objects.box1.location", "shelf")], [("objects.box1.location", "home")]],
    "b_camera_or_manual": [[("sensors.camera_top.available", True)], [("sensors.camera_top.available", False)]],
    "b_crate_or_charge": [[("objects.crate1.location", "dock")], [("objects.crate1.location", "table")]],
    "b_door_or_storage": [[("regions.door_zone.clear", True)], [("regions.door_zone.clear", False)]],
    "b_fence_b_route": [[("regions.fence_b.clear", True)], [("regions.fence_b.clear", False)]],
    "b_handover_or_carry": [[("regions.handoff_zone.clear", True)], [("regions.handoff_zone.clear", False)]],
    "b_missing_report": [[("objects.box1.location", "shelf_a")], [("objects.box1.location", "table")]],
    "b_plate_or_report": [[("objects.plate1.location", "storage")], [("objects.plate1.location", "bench")]],
    "b_ready_robot_goes": [[("robots.rover1.ready", True)], [("robots.rover1.ready", False)]],
    "b_route_by_cell": [[("regions.cell.clear", True)], [("regions.cell.clear", False)]],
    "b_scanner_or_fridge": [[("sensors.spectro.available", True)], [("sensors.spectro.available", False)]],
    "b_station_or_bin": [[("regions.fence_a.clear", True)], [("regions.fence_a.clear", False)]],
    "b_three_way": [[("regions.cell.clear", True)],
                    [("regions.cell.clear", False), ("regions.aisle.clear", True)],
                    [("regions.cell.clear", False), ("regions.aisle.clear", False)]],
    "b_vial_swap": [[("objects.vial2.location", "fridge")], [("objects.vial2.location", "bench")]],
    "b_whichever_ready": [[("robots.arm1.ready", True)], [("robots.arm1.ready", False)]],
}


def cli(*args: str) -> tuple[int, str, str]:
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(ROOT)
    try:
        code = main(list(args), out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


def source_of(task_id: str) -> str:
    text = (REFERENCE / f"{task_id}.attempt.rob").read_text(encoding="utf-8")
    return text.replace("robolang 1.3;", "robolang 1.4;", 1)


def definition(task_id: str) -> dict:
    return json.loads((TASKS / f"{task_id}.json").read_text(encoding="utf-8"))


def world_data(task_id: str, ops=None) -> bytes:
    world = json.loads((ROOT / definition(task_id)["world"]).read_text(encoding="utf-8"))
    if ops is not None:
        world["events"] = []
        for key, value in ops:
            section, name, field = key.split(".")
            entity = world[section][name]
            entity["location" if field == "location" else field] = value
    return json.dumps(world).encode("utf-8")


def simulate(task_id: str, ops=None, source: str | None = None, data: bytes | None = None):
    task = compile_source(source or source_of(task_id), "a.rob").ir.tasks[0]
    bindings = json.loads((REFERENCE / f"{task_id}.attempt.json").read_text(encoding="utf-8"))["bindings"]
    world = load_world(data or world_data(task_id, ops), "w.json", task, bindings)
    return task, world, run(task, world)


def triple_true(result, triple) -> bool:
    subject, prop, value = triple
    world = result.final_world
    if subject.startswith("rl:robot:"):
        return dict(world.robots)[subject[9:]].location == value[9:]
    state = dict(world.objects)[subject[9:]]
    if prop == "carried_by":
        return state.held_by == value[9:]
    return state.held_by is None and state.location == value[9:]


def messages(result) -> list[list[str]]:
    return [[s.message.channel, s.message.message] for s in result.steps if s.message and s.status == "succeeded"]


def check_path(result, path: dict) -> None:
    if "fail" in path:
        assert result.state == "FAILED" and result.failure.code == "S262"
        assert result.failure.reason == path["fail"] and result.failure.phase == "step"
        assert all(g.result is None for g in result.goal)  # a run that ends in fail has no goal evaluation
        assert messages(result) == path.get("messages", [])
    else:
        assert result.state == "SUCCEEDED", result.failure
        assert result.failure is None
        assert all(triple_true(result, t) for t in path["goal"])
        assert messages(result) == path.get("messages", [])


@pytest.mark.parametrize("task_id", IDS)
def test_every_reference_program_follows_each_of_its_paths(task_id):
    paths = definition(task_id)["expect_1_4"]["paths"]
    assert len(PATCHES[task_id]) == len(paths)
    for path, ops in zip(paths, PATCHES[task_id]):
        _, _, result = simulate(task_id, ops)
        check_path(result, path)
        taken = [d.branch for d in result.decisions]
        assert taken and all(d.condition_result.value in ("true", "false") for d in result.decisions)


@pytest.mark.parametrize("task_id", IDS)
def test_every_reference_program_runs_in_its_own_world(task_id):
    """The task's own world takes one of the paths (the unpatched world, with its scripted events)."""
    paths = definition(task_id)["expect_1_4"]["paths"]
    _, _, result = simulate(task_id)
    outcomes = []
    for path in paths:
        try:
            check_path(result, path)
            outcomes.append(True)
        except AssertionError:
            outcomes.append(False)
    assert any(outcomes)


# -------- unknown, fail, goals --------

ROUTE = """robolang 1.4;
task t(robot r, object box, location src, location a, location b, region cell) {
    plan {
        move r to src;
        grasp r box;
        if region_clear cell {
            goal { object_at box a; }
            move r to a;
            place r box at a;
        } else {
            goal { object_at box b; }
            move r to b;
            place r box at b;
        }
    }
}
"""
ROUTE_BINDINGS = {"r": "arm1", "box": "box1", "src": "shelf_a", "a": "table", "b": "handoff", "cell": "cell"}


def route(clear, source=ROUTE):
    world = json.loads((ROOT / "bench/worlds/warehouse.json").read_text(encoding="utf-8"))
    world["regions"]["cell"]["clear"] = clear
    world["events"] = []
    data = json.dumps(world).encode("utf-8")
    task = compile_source(source, "a.rob").ir.tasks[0]
    world = load_world(data, "w.json", task, ROUTE_BINDINGS)
    return task, data, run(task, world)


def test_an_unknown_condition_fails_the_run_before_either_branch_s261():
    world = json.loads(world_data("b_missing_report"))
    box = world["objects"]["box1"]
    del box["present"], box["location"]  # presence not known: object_at is unknown
    task, _, result = simulate("b_missing_report", data=json.dumps(world).encode("utf-8"))
    failure = result.failure
    assert (result.state, failure.code, failure.phase, failure.step) == ("FAILED", "S261", "step", 1)
    assert [(d.step, d.condition_result.value, d.branch) for d in result.decisions] == [("1", "unknown", "none")]
    assert result.steps[0].status == "failed" and result.steps[0].condition_result.value == "unknown"
    assert all(s.status == "not_run" for s in result.steps[1:]) and messages(result) == []
    assert result.goal == () and result.failure.reason is None


def test_the_effective_goal_is_the_task_goal_then_the_goals_of_the_branches_taken():
    source = ROUTE.replace("    plan {", "    goal { robot_at r src; }\n    plan {", 1).replace(
        "place r box at a;", "place r box at a;\n            move r to src;", 1).replace(
        "place r box at b;", "place r box at b;\n            move r to src;", 1)
    for clear, expected in ((True, "table"), (False, "handoff")):
        _, _, result = route(clear, source)
        assert result.state == "SUCCEEDED"
        assert [(g.index, g.source) for g in result.goal] == [(1, "task"), (2, "3.else" if not clear else "3.then")]
        assert [g.result.value for g in result.goal] == ["true", "true"]
        assert dict(result.final_world.objects)["box1"].location == expected


def test_a_goal_that_is_false_at_the_end_fails_with_s301():
    source = ROUTE.replace("place r box at a;", "place r box at a;\n            move r to src;", 1).replace(
        "goal { object_at box a; }", "goal { object_at box a; robot_at r a; }", 1)
    _, _, result = route(True, source)
    assert result.failure.code == "S301" and result.failure.phase == "goal"
    assert [g.result.value for g in result.goal] == ["true", "false"]


def test_a_branch_without_else_takes_the_empty_else():
    source = """robolang 1.4;
task t(robot r, location dst, region cell) {
    goal { robot_at r dst; }
    plan {
        if region_clear cell { communicate r to "ops" with "clear"; }
        move r to dst;
    }
}
"""
    task = compile_source(source, "a.rob").ir.tasks[0]
    for clear, sent in ((True, [["ops", "clear"]]), (False, [])):
        world = json.loads((ROOT / "bench/worlds/warehouse.json").read_text(encoding="utf-8"))
        world["regions"]["cell"]["clear"] = clear
        world["events"] = []
        loaded = load_world(json.dumps(world).encode("utf-8"), "w.json", task,
                            {"r": "arm1", "dst": "table", "cell": "cell"})
        result = run(task, loaded)
        assert result.state == "SUCCEEDED" and messages(result) == sent
        assert result.decisions[0].branch == ("then" if clear else "else")


def test_the_condition_is_judged_against_the_state_when_the_if_is_reached():
    """After a wait the scripted event has happened: the same `if` takes the other branch."""
    source = ROUTE.replace("        if region_clear cell", "        wait for 10 s;\n        if region_clear cell", 1)
    world = json.loads((ROOT / "bench/worlds/warehouse.json").read_text(encoding="utf-8"))
    world["regions"]["cell"]["clear"] = False
    world["events"] = [{"at": {"value": "5", "unit": "s"}, "fact": "region_clear", "target": "cell", "value": True}]
    task = compile_source(source, "a.rob").ir.tasks[0]
    result = run(task, load_world(json.dumps(world).encode("utf-8"), "w.json", task, ROUTE_BINDINGS))
    assert result.decisions[0].branch == "then" and str(result.decisions[0].t_ms) == "10000"


def test_nested_if_labels_and_indexes():
    source = """robolang 1.4;
task t(robot r, location dst, region cell, region aisle) {
    goal { robot_at r dst; }
    plan {
        if region_clear cell {
            if region_clear aisle { communicate r to "a" with "both"; } else { communicate r to "a" with "cell"; }
        } else {
            communicate r to "a" with "none";
        }
        move r to dst;
    }
}
"""
    task = compile_source(source, "a.rob").ir.tasks[0]
    world = json.loads((ROOT / "bench/worlds/warehouse.json").read_text(encoding="utf-8"))
    world["regions"]["cell"]["clear"] = True
    world["events"] = []
    result = run(task, load_world(json.dumps(world).encode("utf-8"), "w.json", task,
                                  {"r": "arm1", "dst": "table", "cell": "cell", "aisle": "aisle"}))
    assert [(s.index, s.path, s.status) for s in result.steps] == [
        (1, "1", "succeeded"), (2, "1.then.1", "succeeded"), (3, "1.then.1.then.1", "not_run"),
        (4, "1.then.1.else.1", "succeeded"), (5, "1.else.1", "not_run"), (6, "2", "succeeded")]
    assert [(d.step, d.branch) for d in result.decisions] == [("1", "then"), ("1.then.1", "else")]
    assert messages(result) == [["a", "cell"]]


# -------- faults and cancellation address steps in source order through both branches (B8) --------

def test_fault_indexing_counts_through_both_branches():
    # b_box_or_home: 1 if, 2 move, 3 grasp, 4 move, 5 place (then), 6 move home (else)
    def with_fault(step, ops):
        task = compile_source(source_of("b_box_or_home"), "a.rob").ir.tasks[0]
        world = json.loads(world_data("b_box_or_home", ops))
        bindings = json.loads((REFERENCE / "b_box_or_home.attempt.json").read_text(encoding="utf-8"))["bindings"]
        world["tasks"] = {"tidy": {"bindings": bindings, "faults": [{"step": step}]}}
        return run(task, load_world(json.dumps(world).encode("utf-8"), "w.json", task))

    then_world, else_world = PATCHES["b_box_or_home"]
    result = with_fault(6, else_world)
    assert (result.failure.code, result.failure.step, result.steps[5].path) == ("S251", 6, "1.else.1")
    assert [s.status for s in result.steps] == ["succeeded"] + ["not_run"] * 4 + ["failed"]
    assert with_fault(6, then_world).state == "SUCCEEDED"  # the fault is on a step the run never reaches
    assert with_fault(3, else_world).state == "SUCCEEDED"
    result = with_fault(3, then_world)
    assert (result.failure.code, result.failure.step) == ("S251", 3)


def test_a_fault_or_cancellation_outside_the_plan_or_on_an_if_is_a_world_error():
    from robolang.sim import WorldError
    task = compile_source(source_of("b_box_or_home"), "a.rob").ir.tasks[0]
    bindings = json.loads((REFERENCE / "b_box_or_home.attempt.json").read_text(encoding="utf-8"))["bindings"]
    for entry, text in (({"faults": [{"step": 7}]}, "outside the plan (1 to 6)"),
                        ({"faults": [{"step": 1}]}, "is an 'if'"),
                        ({"cancel": {"before_step": 7}}, "outside 1 to 6")):
        world = json.loads(world_data("b_box_or_home"))
        world["tasks"] = {"tidy": {"bindings": bindings, **entry}}
        with pytest.raises(WorldError) as caught:
            load_world(json.dumps(world).encode("utf-8"), "w.json", task)
        assert caught.value.code == "E511" and text in str(caught.value)


def test_cancellation_before_a_step_on_the_taken_path():
    task = compile_source(source_of("b_box_or_home"), "a.rob").ir.tasks[0]
    bindings = json.loads((REFERENCE / "b_box_or_home.attempt.json").read_text(encoding="utf-8"))["bindings"]
    # step 6 is on the else path, which the then world never reaches
    for before, state in ((4, "CANCELLED"), (6, "SUCCEEDED")):
        world = json.loads(world_data("b_box_or_home", PATCHES["b_box_or_home"][0]))
        world["tasks"] = {"tidy": {"bindings": bindings, "cancel": {"before_step": before}}}
        result = run(task, load_world(json.dumps(world).encode("utf-8"), "w.json", task))
        assert result.state == state
        if state == "CANCELLED":
            assert result.cancelled_before_step == 4 and result.goal[0].result is None


# -------- record 1.3 --------

def record_of(task_id, ops=None):
    source = source_of(task_id)
    data = world_data(task_id, ops)
    task, world, result = simulate(task_id, ops, source, data)
    identifier = run_id(source.encode(), data, task.name, "0")
    return to_record(result, task=task, run_id=identifier, source_sha256=sha256_normalized(source.encode()),
                     world_sha256=sha256_normalized(data), seed="0", language_version="1.4",
                     bindings_source="world")


def test_record_1_3_shape():
    record = record_of("b_missing_report", PATCHES["b_missing_report"][1])
    assert record["run_record_schema_version"] == "1.3" and record["inputs"]["ir_schema_version"] == "1.3"
    keys = list(record)
    assert keys.index("goal") + 1 == keys.index("decisions") and keys.index("decisions") + 1 == keys.index("steps")
    assert keys[-1] == "core_outcome"
    assert record["decisions"] == [{"step": "1", "condition_result": "false", "branch": "else", "t_ms": "0"}]
    assert record["failure"] == {"code": "S262", "phase": "step", "step": 7, "precondition": None,
                                 "safety_limit": None, "robot": None, "reason": "box1 missing"}
    assert [s["path"] for s in record["steps"]] == ["1", "1.then.1", "1.then.2", "1.then.3", "1.then.4",
                                                    "1.else.1", "1.else.2"]
    assert [s["status"] for s in record["steps"]] == ["succeeded"] + ["not_run"] * 4 + ["succeeded", "failed"]
    assert record["steps"][0]["step"] == {"kind": "if", "condition": record["steps"][0]["step"]["condition"]}
    assert record["steps"][0]["condition_result"] == "false"
    assert record["goal"] == [] and record["steps"][-1]["status"] == "failed"
    text = to_json(record)
    assert text.isascii() and text.endswith("\n")


def test_the_goal_array_says_where_each_condition_came_from():
    record = record_of("b_route_by_cell", PATCHES["b_route_by_cell"][0])
    assert [(g["from"], g["result"]) for g in record["goal"]] == [("3.then", "true")]
    assert record["state"] == "SUCCEEDED"


def test_the_core_outcome_reports_the_goal_of_the_taken_path():
    for path, ops in zip(("shelf_a", "table"), PATCHES["b_route_by_cell"]):
        record = record_of("b_route_by_cell", ops)
        outcome = record["core_outcome"]["outcome"]
        assert outcome["status"] == "succeeded" and len(outcome["verification"]["criteria_results"]) == 1
        assert outcome["verification"]["criteria_results"][0]["met"] is True
        assert "decision" not in json.dumps(record["core_outcome"])  # the decisions are in the record, not in Core
    fail = record_of("b_missing_report", PATCHES["b_missing_report"][1])["core_outcome"]["outcome"]
    assert fail["status"] == "failed" and fail["verification"] is None and fail["reasons"] == []


def test_determinism_same_inputs_same_record_and_run_id():
    a = to_json(record_of("b_three_way", PATCHES["b_three_way"][2]))
    b = to_json(record_of("b_three_way", PATCHES["b_three_way"][2]))
    assert a == b and json.loads(a)["run_id"].startswith("sim-")
    assert a != to_json(record_of("b_three_way", PATCHES["b_three_way"][1]))


def test_a_1_3_source_is_recorded_as_before(tmp_path):
    src = tmp_path / "a.rob"
    src.write_text("robolang 1.3;\ntask t(robot r, location dst) { goal { robot_at r dst; } "
                   "plan { move r to dst; } }\n", encoding="utf-8")
    code, out, _ = cli("simulate", str(src), "--world", "bench/worlds/warehouse.json", "--bind", "r=arm1", "dst=table")
    record = json.loads(out)
    assert code == 0 and record["run_record_schema_version"] == "1.2"
    assert "decisions" not in record and "path" not in record["steps"][0]
    assert "reason" not in record["failure"] if record["failure"] else True
    assert "from" not in record["goal"][0]


# -------- the CLI --------

def test_simulate_runs_a_branching_task(tmp_path):
    src = tmp_path / "b.rob"
    src.write_text(source_of("b_aisle_or_report"), encoding="utf-8")
    bind = ["--bind", *(f"{k}={v}" for k, v in json.loads(
        (REFERENCE / "b_aisle_or_report.attempt.json").read_text(encoding="utf-8"))["bindings"].items())]
    code, out, err = cli("simulate", str(src), "--world", "bench/worlds/warehouse.json", *bind)
    record = json.loads(out)
    assert code != 0 and record["state"] == "FAILED" and record["failure"]["code"] == "S262"
    assert "S262" in err


# -------- export-goal and delegate (B7) --------

def export(task_id, tmp_path, extra=()):
    src = tmp_path / f"{task_id}.rob"
    src.write_text(source_of(task_id), encoding="utf-8")
    bindings = json.loads((REFERENCE / f"{task_id}.attempt.json").read_text(encoding="utf-8"))["bindings"]
    return cli("export-goal", str(src), "--world", definition(task_id)["world"], "--bind",
               *(f"{k}={v}" for k, v in bindings.items()), "--from", "rl:agent:planner-1", *extra)


def test_export_goal_of_a_task_whose_paths_share_one_goal(tmp_path):
    code, out, err = export("b_whichever_ready", tmp_path, ["--to", "rl:robot:arm1"])
    assert code == 0, err
    goal = json.loads(out)["payload"]["goal"]
    assert goal["desired_state"] == [{"subject": "rl:thing:box1", "property": "located_in", "value": "rl:place:table"}]


@pytest.mark.parametrize("task_id", ["b_route_by_cell", "b_missing_report", "b_box_or_home"])
def test_export_goal_refuses_paths_with_different_goals_or_a_fail(task_id, tmp_path):
    code, out, err = export(task_id, tmp_path, ["--to", "rl:robot:" + {"b_route_by_cell": "arm2"}.get(task_id, "arm1")])
    assert code == 6 and out == ""
    assert "X603" in err and "B7" in err and "Core 0.2" in err


ONE_ROBOT = """robolang 1.4;
task t(robot r, object box, location src, location dst, region cell) {
    goal { object_at box dst; }
    plan {
        move r to src;
        grasp r box;
        if region_clear cell {
            move r to dst;
            place r box at dst;
        } else {
            communicate r to "ops" with "slow way";
            move r to dst;
            place r box at dst;
        }
    }
}
"""


def test_delegate_runs_a_branching_plan_with_one_common_goal(tmp_path):
    src = tmp_path / "t.rob"
    src.write_text(ONE_ROBOT, encoding="utf-8")
    world = json.loads((ROOT / "bench/worlds/warehouse.json").read_text(encoding="utf-8"))
    world["tasks"] = {"t": {"bindings": {"r": "amr1", "box": "crate1", "src": "dock", "dst": "handoff",
                                         "cell": "cell"}}}
    (tmp_path / "w.json").write_text(json.dumps(world), encoding="utf-8")
    code, out, err = cli("delegate", str(src), "--world", str(tmp_path / "w.json"), "--from", "rl:agent:planner-1",
                         "--to", "rl:robot:amr1")
    assert code == 0, err
    record = json.loads(out)
    assert record["planner_verdict"]["status"] == "succeeded"
    assert record["robot_run"]["decisions"][0]["branch"] == "then"
