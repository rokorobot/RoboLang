"""Tests for the executor (src/robolang/sim/executor.py, spec/simulation-v1.0.md section 4)."""

from __future__ import annotations
import copy
import dataclasses
import json
import subprocess
import sys
from decimal import Decimal
from pathlib import Path

import pytest

from robolang import ir
from robolang.ir import compile_source
from robolang.sim import SimulationInputError, Truth, load_world, ms_text, run

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "simulation"
CASES = json.loads((FIXTURES / "cases.json").read_text(encoding="utf-8"))["cases"]
BASE = json.loads((FIXTURES / "worlds" / "base.json").read_text(encoding="utf-8"))
EXIT_STATE = {0: "SUCCEEDED", 4: "FAILED", 5: "CANCELLED"}
RUN_CASES = [
    c for c in CASES
    if c["expect"]["exit"] in EXIT_STATE and c.get("source") and c.get("world") and "source_glob" not in c
]


def task_of(source: str, name: str | None = None, filename: str = "<string>"):
    ir = compile_source(source, filename).ir
    return ir.task(name) if name is not None else (ir.tasks[0] if len(ir.tasks) == 1 else None)


def case_input(case):
    source = (ROOT / case["source"]).read_bytes().decode("utf-8")
    task = task_of(source, case["task"], case["source"])
    assert task is not None
    world = load_world((ROOT / case["world"]).read_bytes(), case["world"], task)
    return task, world


def truth_text(result):
    return "not_evaluated" if result is None else result.value


def view(result, expect):
    """The fields of `expect` as the fixture README defines them, taken from a RunResult."""
    out = {
        "states": list(result.states),
        "state": result.state,
        "cancelled_before_step": result.cancelled_before_step,
        "preconditions": [truth_text(p.result) for p in result.preconditions],
        "steps": [s.status for s in result.steps],
        "step_times": [
            None if s.t_start_ms is None else [ms_text(s.t_start_ms), ms_text(s.t_end_ms)] for s in result.steps
        ],
        "safety_limits": [lim.status for lim in result.safety_limits],
        "messages": [
            {"robot": s.message.robot, "channel": s.message.channel, "message": s.message.message,
             "status": s.message.status}
            for s in result.steps if s.message is not None
        ],
        "observations": [
            {"object": s.observation.object, "present": s.observation.present,
             "location": s.observation.location, "held_by": s.observation.held_by}
            for s in result.steps if s.observation is not None
        ],
        "events_applied": [
            {"t_ms": ms_text(e.t_ms), "fact": e.fact, "target": e.target, "value": e.value}
            for e in result.events_applied
        ],
        "end_t_ms": ms_text(result.end_t_ms),
    }
    f = result.failure
    out["failure"] = None if f is None else {
        "code": f.code, "phase": f.phase, "step": f.step, "precondition": f.precondition,
        "safety_limit": f.safety_limit, "robot": f.robot,
    }
    return out


def final_world_view(result):
    fw = result.final_world
    return {
        "robots": {k: {"ready": s.ready, "location": s.location, "holding": s.holding} for k, s in fw.robots},
        "objects": {k: {"present": s.present, "location": s.location, "held_by": s.held_by} for k, s in fw.objects},
        "regions": {k: {"clear": s.clear} for k, s in fw.regions},
        "sensors": {k: {"robot": s.robot, "available": s.available} for k, s in fw.sensors},
    }


@pytest.mark.parametrize("case", RUN_CASES, ids=[c["id"] for c in RUN_CASES])
def test_fixture_case(case):
    expect = case["expect"]
    task, world = case_input(case)
    result = run(task, world)
    actual = view(result, expect)
    assert result.state == EXIT_STATE[expect["exit"]]
    for key, wanted in expect.items():
        if key in ("run_id", "exit", "diagnostic"):
            continue
        if key == "final_world":
            actual_world = final_world_view(result)
            for kind, entities in wanted.items():
                for entity, facts in entities.items():
                    for fact, value in facts.items():
                        assert actual_world[kind][entity][fact] == value, (kind, entity, fact)
        else:
            assert actual[key] == wanted, key


# -------- determinism and purity --------

def test_same_inputs_give_equal_results():
    for case in RUN_CASES:
        task, world = case_input(case)
        assert run(task, world) == run(task, world), case["id"]


def test_run_does_not_modify_the_world():
    for case in RUN_CASES:
        task, world = case_input(case)
        run(task, world)
        assert world == case_input(case)[1], case["id"]  # a freshly loaded, equal snapshot


# -------- trust boundary --------

def pick_inputs(world_file="base.json"):
    case = {"source": "examples/pick_and_place.rob", "task": None,
            "world": f"tests/fixtures/simulation/worlds/{world_file}"}
    return case_input(case)


def mismatches():
    task, world = pick_inputs()
    params = task.parameters
    retyped = tuple(dataclasses.replace(p, type="location") if p.name == "cell" else p for p in params)
    other = task_of("task demo_other(robot r, location dst) { plan { move r to dst; } }")
    yield "another task", other, world
    yield "fewer parameters", dataclasses.replace(task, parameters=params[:-1]), world
    yield "parameter of another type", dataclasses.replace(task, parameters=retyped), world
    yield "undeclared name", dataclasses.replace(task, steps=(ir.Move(robot="r", target="nowhere"),)), world
    faulted_task, faulted_world = pick_inputs("fault_step_3.json")
    yield "script outside the plan", dataclasses.replace(faulted_task, steps=faulted_task.steps[:2]), faulted_world
    yield "not a task", "demo_pick_and_place", world


@pytest.mark.parametrize("label, task, world", list(mismatches()), ids=lambda v: v if isinstance(v, str) else "")
def test_run_rejects_a_task_and_world_that_do_not_belong_together(label, task, world):
    with pytest.raises(SimulationInputError):
        run(task, world)


# -------- direct tests --------

def world_for(task, doc_change=None, bindings=None, name=None):
    doc = copy.deepcopy(BASE)
    entry = doc["tasks"]["move_only"]
    doc["tasks"] = {name or task.name: {"bindings": bindings if bindings is not None else entry["bindings"]}}
    if doc_change:
        doc_change(doc)
    return load_world(json.dumps(doc).encode("utf-8"), "w.json", task)


WAIT_SOURCE = """
task t(robot r, region c, location dst) {
    plan {
        wait for 1 s;
        move r to dst;
    }
}
"""


def test_event_after_the_end_of_the_run_is_not_applied():
    task = task_of(WAIT_SOURCE)
    event = {"at": {"value": "5", "unit": "s"}, "fact": "region_clear", "target": "cell", "value": False}
    world = world_for(task, lambda d: d.update(events=[event]), {"r": "arm1", "c": "cell", "dst": "table"})
    result = run(task, world)
    assert result.events_applied == ()
    assert result.end_t_ms == Decimal(1000)
    assert dict(result.final_world.regions)["cell"].clear is True


def test_wait_until_already_true_advances_no_time():
    task = task_of("task t(robot r, region c) { plan { wait until region_clear c timeout 5 s; } }")
    world = world_for(task, bindings={"r": "arm1", "c": "cell"})
    result = run(task, world)
    assert result.state == "SUCCEEDED"
    assert result.end_t_ms == Decimal(0)
    step = result.steps[0]
    assert (step.t_start_ms, step.t_end_ms, step.condition_result) == (Decimal(0), Decimal(0), Truth.TRUE)


@pytest.mark.parametrize("clear, expected", [(False, Truth.FALSE), (None, Truth.UNKNOWN)])
def test_wait_until_timeout_records_the_last_condition_value(clear, expected):
    """Spec 4.5 and 5.3: a `wait until` that ran records its last value, also when it times out."""
    task = task_of("task t(robot r, region c) { plan { wait until region_clear c timeout 5 s; } }")
    region = {} if clear is None else {"clear": clear}
    world = world_for(task, lambda d: d["regions"].update(cell=region), {"r": "arm1", "c": "cell"})
    step = run(task, world).steps[0]
    assert (step.status, step.t_end_ms, step.condition_result) == ("failed", Decimal(5000), expected)


CARRY_SOURCE = """
task carry(robot r, object box, location src, location mid, location dst) {
    plan {
        move r to src;
        grasp r box;
        move r to mid;
        place r box at dst;
    }
}
"""


def test_move_carries_the_held_object():
    task = task_of(CARRY_SOURCE)
    bindings = {"r": "arm1", "box": "box1", "src": "shelf", "mid": "table", "dst": "home"}
    world = world_for(task, bindings=bindings)
    result = run(task, world)
    # place at 'home' fails: the robot is at 'table'. The object is still held, with no location.
    assert result.failure.code == "S222"
    assert dict(result.final_world.objects)["box1"].location is None
    assert dict(result.final_world.objects)["box1"].held_by == "arm1"
    assert dict(result.final_world.robots)["arm1"].holding == "box1"

    bindings["dst"] = "table"
    ok = run(task, world_for(task, bindings=bindings))
    assert ok.state == "SUCCEEDED"
    box = dict(ok.final_world.objects)["box1"]
    assert (box.location, box.held_by) == ("table", None)
    assert dict(ok.final_world.robots)["arm1"].holding is None


def test_move_while_holding_keeps_object_held():
    task = task_of("task t(robot r, object box, location src, location dst) "
                   "{ plan { move r to src; grasp r box; move r to dst; } }")
    result = run(task, world_for(task, bindings={"r": "arm1", "box": "box1", "src": "shelf", "dst": "table"}))
    box = dict(result.final_world.objects)["box1"]
    assert (box.present, box.location, box.held_by) == (True, None, "arm1")
    assert dict(result.final_world.robots)["arm1"].location == "table"


def test_robot_bound_twice():
    task = task_of("task t(robot a, robot b, location dst) { plan { move a to dst; move b to dst; } }")
    world = world_for(task, bindings={"a": "arm1", "b": "arm1", "dst": "table"})
    result = run(task, world)
    assert result.state == "SUCCEEDED"
    assert dict(result.final_world.robots)["arm1"].location == "table"
    assert result.bindings == (("a", "arm1"), ("b", "arm1"), ("dst", "table"))


def test_cancel_and_fault_semantics():
    task = task_of("task t(robot r, location dst) { plan { move r to dst; move r to dst; } }")
    cancelled = run(task, world_for(task, lambda d: d["tasks"]["t"].update(cancel={"before_step": 2}),
                                    {"r": "arm1", "dst": "table"}))
    assert cancelled.state == "CANCELLED" and cancelled.failure is None
    assert [s.status for s in cancelled.steps] == ["succeeded", "not_run"]
    faulted = run(task, world_for(task, lambda d: d["tasks"]["t"].update(faults=[{"step": 1}]),
                                  {"r": "arm1", "dst": "table"}))
    assert faulted.failure.code == "S251" and faulted.failure.step == 1
    assert dict(faulted.final_world.robots)["arm1"].location == "home"


# -------- isolation --------

def test_sim_executor_does_not_load_the_interpreter_or_adapters():
    code = (
        "import sys; sys.path.insert(0, 'src'); import robolang.sim as sim, robolang; "
        "task = robolang.ir.compile_source(open('examples/pick_and_place.rob', encoding='utf-8').read()).ir"
        ".task('demo_pick_and_place'); "
        "world = sim.load_world(open('tests/fixtures/simulation/worlds/base.json', 'rb').read(), 'base.json', task); "
        "result = sim.run(task, world); "
        "assert result.state == 'SUCCEEDED', result.state; "
        "loaded = [m for m in sys.modules if m.startswith('robolang_') or m == 'rclpy']; "
        "assert not loaded, loaded"
    )
    result = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
