"""RoboLang 1.5: step failure handlers (spec/robolang-v1.5.md, decision 0033 F1-F8)."""

from __future__ import annotations
import io
import json
import os
from pathlib import Path

import pytest

from robolang import RoboLangError, check, ir
from robolang.bridge import PathsDiffer, goal_atoms, path_goals
from robolang.cli import main
from robolang.ir import compile_source, to_json
from robolang.ir_reader import read_ir
from robolang.sim import load_world, run, to_record

ROOT = Path(__file__).resolve().parent.parent
WORLD = json.loads((ROOT / "examples" / "world.json").read_text(encoding="utf-8"))
PARAMS = "robot r, robot m, object box, location src, location dst, region zone, sensor cam"
BINDINGS = {"r": "arm1", "m": "amr1", "box": "box1", "src": "shelf", "dst": "table", "zone": "handoff_zone",
            "cam": "camera_top"}


def task(plan: str, goal: str | None = "goal { object_at box dst; }", version: str = "1.5") -> str:
    return f"robolang {version};\ntask t({PARAMS}) {{\n{goal or ''}\nplan {{\n{plan}\n}}\n}}\n"


def errors(source: str) -> list[tuple[str, int]]:
    with pytest.raises(RoboLangError) as caught:
        check(source)
    return [(d.code, d.line) for d in caught.value.diagnostics]


def codes(source: str) -> list[str]:
    return [code for code, _ in errors(source)]


def simulate(source: str, faults=(), events=None):
    world = dict(WORLD, tasks={"t": {"bindings": BINDINGS, **({"faults": [{"step": n} for n in faults]}
                                                              if faults else {})}})
    if events is not None:
        world["events"] = events
    data = json.dumps(world).encode()
    compiled = compile_source(source)
    plan = compiled.ir.tasks[0]
    result = run(plan, load_world(data, "w.json", plan))
    record = to_record(result, task=plan, run_id="sim-x", source_sha256="0", world_sha256="0", seed="0",
                       language_version=compiled.ir.language_version)
    return result, record


def steps(record) -> list[tuple[str, str, str, str | None]]:
    return [(s["path"], s["step"]["kind"], s["status"], s["handled"]) for s in record["steps"]]


DELIVER = """
move r to src;
grasp r box;
move r to dst;
place r box at dst;
"""

# -------- front end --------

@pytest.mark.parametrize("step", ["move r to dst", "grasp r box", "place r box at dst", "inspect r box using cam",
                                  "wait until region_clear zone timeout 5 s"])
def test_each_failable_step_takes_a_handler_that_compiles_to_else(step):
    source = task(f'{step} else {{\n communicate r to "ops" with "x";\n}}\nmove r to dst;',
                  goal="goal { robot_at r dst; }")
    first = json.loads(to_json(compile_source(source).ir))["tasks"][0]["steps"][0]
    assert list(first)[-1] == "else" and "goal_else" not in first
    assert first["else"] == [{"kind": "communicate", "robot": "r", "channel": "ops", "message": "x"}]


def test_a_step_without_a_handler_has_no_handler_keys_and_ir_is_schema_1_4():
    document = json.loads(to_json(compile_source(task(DELIVER)).ir))
    assert (document["language_version"], document["ir_schema_version"]) == ("1.5", "1.4")
    assert all("else" not in step for step in document["tasks"][0]["steps"])


@pytest.mark.parametrize("statement", ["wait for 1 s", 'communicate r to "a" with "b"'])
def test_steps_that_cannot_fail_for_the_world_take_no_handler_e201(statement):
    assert errors(task(f"{statement} else {{ move r to src; }}\n{DELIVER}")) == [("E201", 5)]


def test_a_handler_in_a_1_4_file_is_a_syntax_error():
    assert codes(task(f"move r to src else {{ move r to dst; }}\n{DELIVER}", version="1.4"))[0] == "E202"


def test_wait_until_needs_a_handler_in_1_5_e316_but_not_in_1_4():
    plan = f"wait until region_clear zone timeout 5 s;\n{DELIVER}"
    assert errors(task(plan)) == [("E316", 5)]
    check(task(plan, version="1.4"))


def test_fail_only_last_in_a_handler_e313():
    assert codes(task(f'move r to src else {{ fail "x"; move r to dst; }}\n{DELIVER}')) == ["E313"]
    check(task(f'move r to src else {{ move r to dst; fail "x"; }}\n{DELIVER}'))


def test_a_handler_counts_as_a_nesting_level_e216():
    def nest(depth):
        text = "move r to dst;"
        for _ in range(depth):
            text = f"move r to src else {{ {text} }}"
        return text
    check(task(f"{nest(4)}\n{DELIVER}"))
    assert codes(task(f"{nest(5)}\n{DELIVER}")) == ["E216"]
    assert codes(task(f"if region_clear zone {{ {nest(4)} }}\n{DELIVER}")) == ["E216"]


def test_every_path_needs_a_goal_the_ok_and_failed_sides_e309():
    with pytest.raises(RoboLangError) as caught:
        check(task(f"move r to src else {{ move r to src; }}\n{DELIVER}", goal=None))
    assert caught.value.code == "E309" and "path ok" in caught.value.diagnostic.message
    check(task(f'move r to src else {{ fail "x"; }}\n{DELIVER}'))


def test_a_handler_has_no_goal_block_e208_decision_0034():
    with pytest.raises(RoboLangError) as caught:
        check(task(f"move r to src else {{ goal {{ robot_at r src; }} move r to src; }}\n{DELIVER}"))
    assert [d.code for d in caught.value.diagnostics] == ["E208"]
    assert "a failure handler has no goal block" in caught.value.diagnostic.message


def test_holder_check_follows_both_outcomes_f8():
    check(task("move r to src;\ngrasp r box else { grasp r box; }\nmove r to dst;\nplace r box at dst;"))  # retry
    assert codes(task("grasp r box else { move r to src; }\ngrasp m box;\nplace m box at dst;")) == ["E315"]
    found = codes(task("grasp r box;\ngrasp m box else { place r box at dst; }\nplace m box at dst;"))
    assert found == ["E315"]  # always fails, so the handler always runs: still an authoring error


def test_ir_reads_back_identical_and_rejects_schema_1_4_keys_in_1_3():
    source = task(f"wait until region_clear zone timeout 5 s else {{ move r to src; }}\n{DELIVER}")
    compiled = compile_source(source).ir
    data = to_json(compiled).encode()
    assert read_ir(data) == compiled
    older = json.loads(data)
    older.update(ir_schema_version="1.3", language_version="1.4")
    with pytest.raises(Exception) as caught:
        read_ir(json.dumps(older).encode())
    assert [(d.code, d.pointer) for d in caught.value.diagnostics][:1] == [("I112", "/tasks/0/steps/0/else")]
    newer = json.loads(data)
    newer["tasks"][0]["steps"][0]["goal_else"] = None  # a handler has no goal (decision 0034)
    with pytest.raises(Exception) as caught:
        read_ir(json.dumps(newer).encode())
    assert [(d.code, d.pointer) for d in caught.value.diagnostics] == [("I112", "/tasks/0/steps/0/goal_else")]


def test_inspect_prints_the_handler(tmp_path):
    path = tmp_path / "t.rob"
    path.write_text(task('grasp r box else { fail "no box"; }\nmove r to dst;\nplace r box at dst;'), encoding="utf-8")
    out, err = io.StringIO(), io.StringIO()
    assert main(["inspect", str(path)], out=out, err=err) == 0
    assert "1. grasp r box\n      if it fails:\n        1.1. fail \"no box\"\n" in out.getvalue()


# -------- simulation --------

FALLBACK = """
move r to src;
grasp r box;
wait until region_clear zone timeout 2 s else {
    communicate r to "fleet" with "BUSY";
    wait until region_clear zone timeout 10 s else {
        place r box at src;
        fail "zone never cleared";
    }
}
move r to dst;
place r box at dst;
"""


def test_a_timed_out_wait_runs_its_handler_and_the_plan_goes_on():
    result, record = simulate(task(FALLBACK))  # the zone clears at 5 s
    assert record["run_record_schema_version"] == "1.4" and record["state"] == "SUCCEEDED"
    assert steps(record) == [
        ("1", "move", "succeeded", None), ("2", "grasp", "succeeded", None), ("3", "wait_until", "failed", "S241"),
        ("3.else.1", "communicate", "succeeded", None), ("3.else.2", "wait_until", "succeeded", None),
        ("3.else.2.else.1", "place", "not_run", None), ("3.else.2.else.2", "fail", "not_run", None),
        ("4", "move", "succeeded", None), ("5", "place", "succeeded", None)]
    assert "else" not in record["steps"][2]["step"] and record["steps"][2]["t_end_ms"] == "2000"
    assert record["failure"] is None


def test_a_handler_that_ends_in_fail_fails_the_run_s262():
    _, record = simulate(task(FALLBACK), events=[])  # the zone never clears
    assert record["state"] == "FAILED" and record["failure"]["code"] == "S262"
    assert record["failure"]["reason"] == "zone never cleared" and record["failure"]["step"] == 7
    assert [s["status"] for s in record["steps"]][-2:] == ["not_run", "not_run"]
    assert record["final_world"]["objects"]["box1"]["location"] == "shelf"


def test_a_scripted_fault_is_handled_and_a_retry_succeeds():
    plan = "move r to src;\ngrasp r box else { grasp r box; }\nmove r to dst;\nplace r box at dst;"
    _, record = simulate(task(plan), faults=[2])
    assert steps(record)[1:3] == [("2", "grasp", "failed", "S251"), ("2.else.1", "grasp", "succeeded", None)]
    assert record["state"] == "SUCCEEDED" and [s["index"] for s in record["steps"]] == [1, 2, 3, 4, 5]


def test_a_failure_in_a_handler_without_its_own_handler_fails_the_run():
    plan = "move r to src;\ngrasp r box else { grasp r box; }\nmove r to dst;\nplace r box at dst;"
    _, record = simulate(task(plan), faults=[2, 3])
    assert record["failure"]["code"] == "S251" and record["failure"]["step"] == 3


def test_a_handler_not_needed_is_not_run_and_step_numbers_count_through_it():
    plan = 'move r to src else { communicate r to "a" with "b"; }\ngrasp r box;\nmove r to dst;\nplace r box at dst;'
    _, record = simulate(task(plan), faults=[3])
    assert steps(record)[:3] == [("1", "move", "succeeded", None), ("1.else.1", "communicate", "not_run", None),
                                 ("2", "grasp", "failed", None)]
    assert record["failure"]["code"] == "S251"


def test_a_handler_path_is_judged_on_the_task_goal_alone():
    plan = ('move r to src else {\n communicate r to "ops" with "retrying";\n move r to src;\n}\n'
            "grasp r box;\nmove r to dst;\nplace r box at dst;")
    _, record = simulate(task(plan), faults=[1])
    assert [(g["from"], g["result"]) for g in record["goal"]] == [("task", "true")]
    assert record["state"] == "SUCCEEDED"


def test_a_1_4_record_has_no_handled_key():
    source = task(DELIVER, version="1.4")
    _, record = simulate(source)
    assert record["run_record_schema_version"] == "1.3" and "handled" not in record["steps"][0]


# -------- Core export, delegate, backends --------

def test_core_export_sees_the_ok_and_failed_paths_f7():
    retry = compile_source(task("move r to src;\ngrasp r box else { grasp r box; }\nmove r to dst;\n"
                                "place r box at dst;")).ir.tasks[0]
    assert [label for label, _ in path_goals(retry)] == ["ok", "failed"]
    assert [a.KIND for a in goal_atoms(retry)] == ["object_at"]  # one goal on every path: Core 0.1 can say it
    aborting = compile_source(task(f'move r to src else {{ fail "x"; }}\n{DELIVER}')).ir.tasks[0]
    with pytest.raises(PathsDiffer):
        goal_atoms(aborting)


def test_a_task_with_a_handler_is_branching_so_delegate_takes_it_with_one_robot_only_f6():
    plan = compile_source(task(f'move r to src else {{ fail "x"; }}\n{DELIVER}')).ir.tasks[0]
    assert plan.branching and ir.handler_of(plan.steps[0]) is not None


def test_the_ros2_backend_refuses_a_handler_before_any_motion():
    pytest.importorskip("jsonschema")
    import sys
    sys.path.insert(0, str(ROOT / "backends" / "ros2"))
    from robolang_ros2.executor import execute
    from test_ros2_executor import FakeOps, fixture
    _, bindings, site = fixture()
    source = ("robolang 1.5;\ntask transfer(robot r, object box, location src, location dst) {\n"
              "  goal { object_at box dst; }\n"
              "  plan { move r to src else { move r to src; } grasp r box; move r to dst; place r box at dst; }\n}\n")
    ops = FakeOps()
    record = execute(compile_source(source).ir.tasks[0], bindings, site, ops)
    assert record["failure"]["code"] == "S508" and ops.calls == [("guard", "ur")]


def test_validate_world_reports_a_certain_failure_inside_a_handler(tmp_path):
    world = dict(WORLD, robots=dict(WORLD["robots"], amr1=dict(WORLD["robots"]["amr1"], gripper=False)),
                 tasks={"t": {"bindings": BINDINGS}})
    (tmp_path / "w.json").write_text(json.dumps(world), encoding="utf-8")
    plan = f"move r to src else {{ grasp m box; place m box at src; }}\n{DELIVER}"
    (tmp_path / "t.rob").write_text(task(plan), encoding="utf-8")
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(tmp_path)
    try:
        code = main(["validate", "t.rob", "--world", "w.json"], out=out, err=err)
    finally:
        os.chdir(previous)
    assert code == 1 and "t.rob:5:22: error[S211]" in err.getvalue() and "(on path 1.else)" in err.getvalue()
