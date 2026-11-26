"""
RoboLang 2.4: the robot's own state (spec/robolang-v2.4.md, decision 0048 S1-S11 and the clarifications of 2026-11-25).

Front end, Task IR 1.12 and Mission IR 1.4 and the reader, world file 1.9 (a robot's `charge` and `charge_events`), the
simulation (spec/simulation-v1.0.md section 22), the run record 1.12, missions, `delegate` and the backends' refusals.
No test here needs a model or a network.

To regenerate the IR snapshots after an intended change:

    ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_lang24.py
"""

from __future__ import annotations
import dataclasses
import io
import json
import os
import re
import sys
from pathlib import Path

import pytest

from robolang import RoboLangError, check, check_file
from robolang.cli import main
from robolang.ir import compile_source, reads_charge, to_json
from robolang.ir_reader import read_ir
from robolang.sim import WorldError, load_world, mission_task, run, run_mission, to_mission_record, to_record
from robolang.sim.check import check_task
from robolang.tokens import FUTURE_RESERVED, RESERVED_BY_VERSION

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "v2.4"
SNAPSHOTS = ROOT / "tests" / "snapshots" / "ir-v2.4"
UPDATE = os.environ.get("ROBOLANG_UPDATE_SNAPSHOTS") == "1"
SPEC = (ROOT / "spec" / "robolang-v2.4.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")

# -------- helpers --------

class Template(str):
    """`TASK % body`: a task whose goal is at the task level, or in the branches when the body writes its own."""

    GOAL = "robolang 2.4;\n\ntask t(robot r, robot b, location dest, location dock) {\n    goal { robot_at r dest; }\n"

    def __mod__(self, body):
        head = self.GOAL if "goal {" not in body else self.GOAL.split("    goal")[0]
        return head + "    plan {\n" + body + "\n    }\n}\n"


TASK = Template("")
BIND = {"r": "spot1", "b": "spot2", "dest": "stop_c", "dock": "dock_1"}


def world_data(charges=None, events=(), version="1.9") -> bytes:
    """A world with two robots; `charges` maps a robot to (value, uncertainty) or None (no charge)."""
    charges = {"spot1": ("64", "1"), "spot2": ("90", "1"), **(charges or {})}
    robots = {}
    for name, home in (("spot1", "stop_a"), ("spot2", "stop_b")):
        robot = {"ready": True, "location": home, "gripper": False,
                 "actions": {"charge": {"target": "location", "result": "charging"}}}
        if charges[name] is not None:
            value, spread = charges[name]
            robot["charge"] = {"value": value, "unit": "%", **({} if spread is None else {"uncertainty": spread})}
        robots[name] = robot
    return json.dumps({
        "world_schema": "robolang.sim.world", "world_schema_version": version,
        "locations": ["stop_a", "stop_b", "stop_c", "dock_1", "charger_1"],
        "robots": robots,
        "channels": {"lead": {"from": ["rl:agent:lead"]}},
        "charge_events": [{"at": {"value": str(at), "unit": unit}, "robot": robot,
                           "charge": {"value": value, "unit": "%", "uncertainty": "0"}}
                          for at, unit, robot, value in events],
        "tasks": {},
    }).encode()


def codes(source: str) -> list[tuple[str, int]]:
    try:
        check(source)
    except RoboLangError as error:
        return [(d.code, d.line) for d in error.diagnostics]
    return []


def program(body: str, name="t", **kwargs):
    task = compile_source(TASK % body).ir.task(name)
    return task, load_world(world_data(**kwargs), "w.json", task, BIND)


def simulate(body: str, **kwargs):
    task, world = program(body, **kwargs)
    return run(task, world)


def end(result) -> tuple[str, str | None]:
    return result.state, None if result.failure is None else result.failure.code


def cli(*args):
    out, err = io.StringIO(), io.StringIO()
    code = main(list(args), out=out, err=err)
    return code, out.getvalue(), err.getvalue()


DOCK_IF = """        measure r charge as c;
        if c %s {
            goal { robot_at r dock; }
            move r to dock;
        } else {
            goal { robot_at r dest; }
            move r to dest;
        }"""


def docked(op_and_threshold: str, charge):
    """The state of the run and whether the robot went to the dock, for `if c <op> <threshold>`."""
    result = simulate(DOCK_IF % op_and_threshold, charges={"spot1": charge})
    docked_to = next((s.step.target for s in result.steps if s.step.KIND == "move" and s.status == "succeeded"), None)
    return end(result), docked_to


# -------- fixtures and snapshots --------

HEADER = re.compile(
    r"\A// RoboLang v2\.4 fixture: (?P<set>valid|invalid)\n"
    r"// expect: (?:(?P<ok>ok)|error (?P<code>E\d{3}) \((?P<stage>lexical|syntax|static)\) line (?P<line>\d+))\n"
    r"// why: .+\n"
)
STAGE_DIGIT = {"lexical": "1", "syntax": "2", "static": "3"}
VALID = sorted((FIXTURES / "valid").glob("*.rob"))
INVALID = sorted((FIXTURES / "invalid").glob("*.rob"))


def header(path: Path) -> dict:
    match = HEADER.match(path.read_text(encoding="utf-8"))
    assert match, f"{path.name}: missing or malformed header"
    return match.groupdict()


@pytest.mark.parametrize("path", VALID, ids=lambda p: p.stem)
def test_valid_fixture_passes_and_matches_its_ir_snapshot(path):
    assert header(path)["ok"] and not check_file(path).warnings
    actual = to_json(compile_source(path.read_text(encoding="utf-8"), str(path)).ir)
    snapshot = SNAPSHOTS / f"{path.stem}.json"
    if UPDATE:
        snapshot.parent.mkdir(exist_ok=True)
        snapshot.write_text(actual, encoding="utf-8")
    assert actual == snapshot.read_text(encoding="utf-8").replace("\r\n", "\n")
    document = json.loads(actual)
    mission = document["ir_schema"] == "robolang.mission-ir"
    assert (document["ir_schema_version"], document["language_version"]) == ("1.4" if mission else "1.12", "2.4")
    assert document.get("task_ir_schema_version", "1.12") == "1.12"
    assert read_ir(actual.encode()) == compile_source(path.read_text(encoding="utf-8")).ir


@pytest.mark.parametrize("path", INVALID, ids=lambda p: p.stem)
def test_invalid_fixture_is_rejected_with_the_stated_code_and_line_and_makes_no_ir(path):
    stated = header(path)
    assert path.name.startswith(stated["code"].lower()) and stated["code"][1] == STAGE_DIGIT[stated["stage"]]
    with pytest.raises(RoboLangError) as caught:
        compile_source(path.read_text(encoding="utf-8"), str(path))
    assert (caught.value.code, str(caught.value.line)) == (stated["code"], stated["line"])


def test_fixture_sets_are_counted():
    assert len(VALID) == 9 and len(INVALID) == 13
    assert len(list(SNAPSHOTS.glob("*.json"))) == 9


# -------- the language --------

def test_2_4_adds_no_reserved_word_and_charge_and_robot_charge_are_not_reserved_s3():
    assert RESERVED_BY_VERSION["2.4"] == RESERVED_BY_VERSION["2.3"] == RESERVED_BY_VERSION["2.0"]
    for word in ("charge", "robot_charge"):
        assert word not in RESERVED_BY_VERSION["2.4"] and word not in FUTURE_RESERVED, word


def test_a_program_that_uses_the_words_as_names_parses_in_every_version():
    for version in ("1.4", "1.6", "2.0", "2.2", "2.3", "2.4"):
        source = (f"robolang {version};\ntask charge(robot robot_charge, location charge_to) {{\n"
                  "    goal { robot_at robot_charge charge_to; }\n    plan { move robot_charge to charge_to; }\n}\n")
        assert codes(source) == [], version


def test_below_2_4_charge_stays_its_current_error_and_robot_charge_is_no_condition():
    for version in ("1.6", "2.2", "2.3"):
        assert codes((TASK % "        measure r charge as c;").replace("robolang 2.4", f"robolang {version}"))[0][0] \
            == "E217", version
    wait = "        wait until robot_charge r >= 80 % timeout 5 min else { fail \"x\"; }"
    assert codes((TASK % wait).replace("robolang 2.4", "robolang 2.3"))[0][0] == "E204"


@pytest.mark.parametrize("op", ["<", "<=", ">", ">="])
def test_the_four_orderings_compare_a_charge_with_a_percent_literal_s2(op):
    assert codes(TASK % DOCK_IF % f"{op} 25 %") == []


def test_a_charge_compares_with_another_charge_but_not_with_another_kind_e319_e320():
    both = ("        measure r charge as a;\n        measure b charge as c;\n"
            "        if a >= c { move r to dest; } else { move r to dest; }")
    assert codes(TASK % both) == []
    assert codes(TASK % (DOCK_IF % "== 25 %"))[0][0] == "E320"
    assert codes(TASK % (DOCK_IF % "< 25 degC"))[0][0] == "E319"
    assert codes(TASK % (DOCK_IF % "< 25 s"))[0][0] == "E319"
    assert codes(TASK % (DOCK_IF % "< 25"))[0][0] == "E206"


def test_the_live_form_is_for_a_wait_until_and_a_missions_until_only_e322():
    wait = "        wait until robot_charge r >= 80 % timeout 5 min else { fail \"x\"; }\n        move r to dest;"
    assert codes(TASK % wait) == []
    assert codes(TASK % "        if robot_charge r < 20 % { move r to dest; } else { move r to dock; }")[0][0] == "E322"


def test_the_robot_of_a_charge_is_a_robot_parameter_e303_e304():
    assert codes(TASK % "        measure ghost charge as c;")[0][0] == "E303"
    assert codes(TASK % "        measure dest charge as c;")[0][0] == "E304"
    wait = "        wait until robot_charge dest >= 80 % timeout 5 min else { fail \"x\"; }"
    assert codes(TASK % wait)[0][0] == "E304"


def test_a_charge_binding_is_a_name_that_robot_charge_can_still_be():
    source = TASK % ("        measure r charge as robot_charge;\n        if robot_charge < 25 % "
                     "{ move r to dock; } else { move r to dest; }")
    assert codes(source) == []


def test_where_may_state_the_chosen_robots_charge_and_nothing_else_of_a_reading_s7():
    head = "robolang 2.4;\ntask t(robot a, robot b, location dest) {\n    goal { robot_at t dest; }\n    plan {\n"
    tail = "        move t to dest;\n    }\n}\n"

    def choose(where):
        return head + f'        choose robot t from (a, b) where {where} else {{ fail "x"; }}\n' + tail

    assert codes(choose("robot_charge t >= 50 %")) == []
    assert codes(choose("robot_charge t >= 50 % and robot_ready t")) == []
    assert codes(choose("robot_charge a >= 50 %"))[0][0] == "E330"
    assert codes(choose("robot_charge t == 50 %"))[0][0] == "E320"
    assert codes(choose("robot_charge t >= 50 degC"))[0][0] == "E319"
    assert codes(choose("clock_time >= 10:00"))[0][0] == "E330"


def test_a_missions_until_on_the_charge_is_not_a_bound_by_itself_s6():
    mission = ("robolang 2.4;\ntask round(robot r, location a) {\n    goal { robot_at r a; }\n"
               "    plan { move r to a; }\n}\nmission m(robot r, location a) {\n    repeat %s {\n"
               "        run round(r, a);\n    } until robot_charge r < 30 %%;\n}\n")
    assert codes(mission % "at most 3 runs") == []
    assert codes(mission % "for at most 120 min") == []
    assert codes(mission % "")[0][0] == "E325"


# -------- the simulation: readings and comparisons (S2) --------

def test_a_reading_is_sent_with_its_unit_and_recorded_with_its_uncertainty_s5():
    body = '        measure r charge as c;\n        communicate r to "lead" with c;\n        move r to dest;'
    task, world = program(body)
    result = run(task, world)
    assert end(result) == ("SUCCEEDED", None)
    record = to_record(result, task=task, run_id="x", source_sha256="a" * 64, world_sha256="b" * 64, seed="0",
                       language_version="2.4", world_schema_version="1.9")
    reading = record["steps"][0]["reading"]
    assert reading == {"subject": "spot1", "kind": "charge", "value": "64", "unit": "%", "uncertainty": "1",
                       "sensor": "robot", "t_ms": "0"}
    assert record["steps"][1]["message"]["value"]["unit"] == "%"
    assert record["run_record_schema_version"] == "1.12" and record["inputs"]["world_schema_version"] == "1.9"


@pytest.mark.parametrize("op,charge,expected", [
    ("< 25 %", ("18", "1"), "dock_1"), ("< 25 %", ("64", "1"), "stop_c"),
    ("< 25 %", ("25", None), "stop_c"), ("<= 25 %", ("25", "0"), "dock_1"),
    (">= 25 %", ("25", "0"), "dock_1"), ("> 25 %", ("25", "0"), "stop_c"),
])
def test_the_four_operators_at_and_beside_the_threshold(op, charge, expected):
    state, went = docked(op, charge)
    assert state == ("SUCCEEDED", None) and BIND[went] == expected


def test_a_reading_whose_uncertainty_holds_the_threshold_is_unknown_and_fails_closed_s2_point_9():
    for op, charge in (("< 25 %", ("24", "2")), ("< 25 %", ("26", "2")), (">= 60 %", ("61", "3"))):
        state, went = docked(op, charge)
        assert state == ("FAILED", "S261") and went is None, (op, charge)


def test_an_interval_that_ends_at_the_threshold_is_decided_as_1_6_v7_decides_it():
    """Point 9: the same closed-interval rule as an object's reading: 26 +- 1 is not below 25; 24 +- 1 may be 25."""
    assert docked("< 25 %", ("26", "1"))[0] == ("SUCCEEDED", None)
    assert docked("< 25 %", ("24", "1"))[0] == ("FAILED", "S261")


def test_a_robot_with_no_charge_has_an_unknown_one_and_a_reading_fails_s239():
    body = "        measure r charge as c;\n        move r to dest;"
    handled = "        measure r charge as c else { move r to dock; }\n        move r to dest;"
    task, world = program(handled, charges={"spot1": None})
    assert end(run(task, world)) == ("SUCCEEDED", None)  # a failure handler is a fallback: no check before the run
    assert [step.handled for step in run(task, world).steps][:1] == ["S239"]
    assert end(simulate(body, charges={"spot1": None})) == ("FAILED", "S012")  # no fallback: before the run


def test_another_robots_charge_may_be_read_point_4():
    body = ("        measure b charge as c;\n        if c < 40 % { move b to dock; } else { move b to dest; }\n"
            "        move r to dest;")
    assert codes(TASK % body) == []
    result = simulate(body, charges={"spot2": ("30", "0")})
    assert result.steps[0].reading.subject == "spot2"
    assert [BIND[s.step.target] for s in result.steps if s.step.KIND == "move" and s.status == "succeeded"] \
        == ["dock_1", "stop_c"]


# -------- scheduled events, waits and the clock (S4, points 1, 2, 7) --------

WAIT = ("        wait until robot_charge r >= 80 % timeout 40 min else { fail \"too slow\"; }\n"
        "        move r to dest;")


def test_a_wait_is_met_at_the_instant_of_the_event_that_meets_it():
    result = simulate(WAIT, charges={"spot1": ("35", "1")}, events=[(20, "min", "spot1", "90")])
    assert end(result) == ("SUCCEEDED", None) and result.steps[0].t_end_ms == 1200000
    assert [(e.t_ms, e.fact, e.target) for e in result.events_applied] == [(1200000, "charge", "spot1")]


def test_a_wait_the_events_never_meet_before_its_timeout_fails_before_the_run_s013():
    for events in ([(20, "min", "spot1", "78")], [(50, "min", "spot1", "95")], []):
        result = simulate(WAIT, charges={"spot1": ("35", "1")}, events=events)
        assert end(result) == ("FAILED", "S013"), events
    assert result.state == "FAILED" and result.steps[0].status == "not_run"


def test_a_wait_with_a_fallback_runs_and_times_out_at_the_deadline_s241():
    body = "        wait until robot_charge r >= 80 % timeout 40 min else { move r to dock; }\n        move r to dest;"
    result = simulate(body, charges={"spot1": ("35", "1")}, events=[(50, "min", "spot1", "95")])
    assert result.steps[0].handled == "S241" and result.end_t_ms == 2400000
    assert end(result) == ("SUCCEEDED", None)  # the handler took over, then the plan went on


def test_a_wait_that_is_already_met_ends_at_once_and_an_exact_threshold_is_at_least_it():
    for charge in (("85", "1"), ("80", "0")):
        result = simulate(WAIT, charges={"spot1": charge})
        assert end(result) == ("SUCCEEDED", None) and result.steps[0].t_end_ms == 0
    assert end(simulate(WAIT.replace(">=", ">"), charges={"spot1": ("80", "0")}))[1] == "S013"


def test_do_charge_changes_no_reading_and_only_wait_for_moves_time_points_2_and_7():
    body = ("        move r to dock;\n        do r charge dock;\n        measure r charge as c;\n"
            "        wait for 3 min;\n"
            "        measure r charge as d;\n        move r to dest;")
    task, world = program(body, charges={"spot1": ("35", "1")}, events=[(2, "min", "spot1", "50")])
    result = run(task, world)
    assert [s.reading.value for s in result.steps if s.reading] == [35, 50]
    assert result.steps[1].action.action == "charge" and result.steps[1].t_end_ms == 0


def test_an_event_may_not_be_at_the_start_and_names_a_robot_of_the_world_e504_e505():
    task = compile_source(TASK % "        move r to dest;").ir.task("t")

    def load(mutate):
        doc = json.loads(world_data(events=[(5, "min", "spot1", "50")]))
        mutate(doc)
        load_world(json.dumps(doc).encode(), "w.json", task, BIND)

    load(lambda d: None)
    for mutate, code in (
        (lambda d: d["charge_events"][0]["at"].update(value="0"), "E504"),
        (lambda d: d["charge_events"][0].update(robot="ghost"), "E505"),
        (lambda d: d["charge_events"][0]["charge"].update(value="101"), "E504"),
        (lambda d: d["robots"]["spot1"]["charge"].update(unit="degC"), "E504"),
        (lambda d: d["robots"]["spot1"]["charge"].update(value="-1"), "E504"),
        (lambda d: d["robots"]["spot1"]["charge"].update(uncertainty="x"), "E504"),
        (lambda d: d["robots"]["spot1"]["charge"].update(extra=1), "E503"),
        (lambda d: d["charge_events"][0].pop("robot"), "E504"),
    ):
        with pytest.raises(WorldError) as caught:
            load(mutate)
        assert caught.value.code == code, (code, caught.value)


def test_a_charge_is_a_world_file_1_9_field_only():
    task = compile_source(TASK % "        move r to dest;").ir.task("t")
    with pytest.raises(WorldError) as caught:
        load_world(world_data(version="1.8"), "w.json", task, BIND)
    assert caught.value.code in ("E503", "E504")


def test_validate_world_reports_the_planning_time_failures_s012_s013():
    rob = ROOT / "tests" / "fixtures" / "v2.4" / "valid" / "wait_until_charged_live.rob"
    task = compile_source(rob.read_text(encoding="utf-8")).ir.task("t")
    world = load_world(world_data(charges={"spot1": ("35", "1")}, events=[(5, "min", "spot1", "70")]),
                       "w.json", task, {"r": "spot1", "dest": "stop_c", "dock": "dock_1"})
    assert [f.code for f in check_task(task, world)] == ["S013"]
    world = load_world(world_data(charges={"spot1": None}), "w.json", task,
                       {"r": "spot1", "dest": "stop_c", "dock": "dock_1"})
    assert [f.code for f in check_task(task, world)] == ["S012"]


# -------- missions (S6, point 6) --------

MISSION = """robolang 2.4;

task round(robot r, location a, location b) {
    goal { robot_at r a; }
    plan { move r to b; wait for 10 min; move r to a; }
}

mission patrol(robot r, location a, location b) {
    repeat at most 6 runs {
        run round(r, a, b);
    } until robot_charge r < 30 %;
}
"""
MISSION_TWO_LINES = MISSION.replace("        run round(r, a, b);\n",
                                    "        run round(r, a, b);\n        run round(r, a, b);\n")


def mission_result(charges=None, events=(), source=MISSION):
    document = compile_source(source).ir
    mission = document.mission("patrol")
    bind = {"r": "spot1", "a": "stop_a", "b": "stop_b"}
    world = load_world(world_data(charges=charges, events=events), "w.json", mission_task(mission), bind)
    return run_mission(document, mission, world)


def test_a_missions_until_is_checked_after_each_run_and_the_count_ends_it_otherwise():
    result = mission_result(charges={"spot1": ("80", "1")})
    assert (result.outcome, result.ended_by, len(result.runs)) == ("bounded", "run_bound", 6)
    result = mission_result(charges={"spot1": ("80", "1")}, events=[(25, "min", "spot1", "28")])
    assert (result.outcome, result.ended_by, len(result.runs)) == ("completed", "until", 3)
    result = mission_result(charges={"spot1": ("27", "1")})
    assert (result.outcome, len(result.runs)) == ("completed", 1)
    result = mission_result(charges={"spot1": ("30", "0")})  # not below 30 %
    assert (result.outcome, len(result.runs)) == ("bounded", 6)


def test_each_run_line_is_a_place_where_the_until_is_checked_point_6():
    result = mission_result(charges={"spot1": ("80", "1")}, events=[(25, "min", "spot1", "28")],
                            source=MISSION_TWO_LINES)
    assert (result.outcome, len(result.runs), result.runs[-1].line) == ("completed", 3, 1)  # in the middle of a pass


def test_an_unknown_until_ends_the_mission_failed_with_the_earlier_runs_verified_s6_point_6():
    for charges in ({"spot1": None}, {"spot1": ("31", "2")}):
        result = mission_result(charges=charges)
        assert (result.outcome, result.ended_by, len(result.runs)) == ("failed", "until_unknown", 1), charges
        assert result.failure.code == "S402" and result.runs_verified == 1


def test_the_mission_record_is_1_4_and_lists_the_event():
    document = compile_source(MISSION).ir
    result = mission_result(charges={"spot1": ("80", "1")}, events=[(25, "min", "spot1", "28")])
    record = to_mission_record(result, document=document, run_id="x", source_sha256="a" * 64, world_sha256="b" * 64,
                               seed="0", world_schema_version="1.9")
    assert record["mission_record_schema_version"] == "1.4" and record["run_record_schema_version"] == "1.12"
    assert [(e["fact"], e["target"], e["value"]["value"]) for e in record["events_applied"]] \
        == [("charge", "spot1", "28")]


# -------- choosing (S7, point 5) --------

CHOOSE = """robolang 2.4;

task relief(robot a, robot b, location dest) {
    goal { robot_at t dest; }
    plan {
        choose robot t from (a, b) where robot_charge t >= 50 % else { fail "none"; }
        move t to dest;
    }
}
"""


def choose_run(spot1, spot2):
    task = compile_source(CHOOSE).ir.task("relief")
    world = load_world(world_data(charges={"spot1": spot1, "spot2": spot2}), "w.json", task,
                       {"a": "spot1", "b": "spot2", "dest": "stop_c"})
    result = run(task, world)
    return [c.chosen for c in result.choices], [c.excluded for c in result.choices][0], end(result)


def test_a_candidate_with_an_unknown_or_too_low_charge_is_left_out_of_a_choice():
    assert choose_run(("40", "0"), ("70", "0"))[0] == ["spot2"]
    assert choose_run(("90", "0"), ("80", "0"))[0] == ["spot1"]
    assert choose_run(("50", "0"), ("45", "0"))[0] == ["spot1"]
    chosen, excluded, state = choose_run(("52", "3"), ("80", "0"))
    assert chosen == ["spot2"] and excluded == (("spot1", "where_unknown"),)
    chosen, excluded, state = choose_run(None, ("80", "0"))
    assert chosen == ["spot2"] and excluded == (("spot1", "where_unknown"),)
    chosen, excluded, state = choose_run(("40", "0"), ("30", "0"))
    assert chosen == [None] and state == ("FAILED", "S262") and len(excluded) == 2


# -------- export-goal, delegate and the backends (S8) --------

def test_a_task_that_reads_the_charge_is_reported_by_reads_charge():
    for path in VALID:
        task = compile_source(path.read_text(encoding="utf-8")).ir.tasks[0]
        expected = path.stem not in ("a_mission_that_stops_when_the_battery_is_low",)  # its task reads nothing
        assert reads_charge(task) is expected, path.stem


def test_delegate_refuses_a_task_that_reads_the_charge_as_it_refuses_any_measure():
    from robolang import delegate
    for body in ("        measure r charge as c;\n        move r to dest;",
                 WAIT.replace("fail \"too slow\"", "fail \"x\""),
                 "        move r to dest;"):
        task = compile_source("robolang 2.4;\ntask t(robot r, location dest) {\n    goal { robot_at r dest; }\n"
                              f"    plan {{\n{body}\n    }}\n}}\n").ir.tasks[0]
        verdict = delegate._consider(None, task, b"", "w.json", "spot1", (), "0.1")[0]
        assert (verdict == "measure_unsupported") == reads_charge(task)


def test_export_goal_is_unaffected_a_goal_holds_no_binding(tmp_path):
    rob = tmp_path / "t.rob"
    rob.write_text("robolang 2.4;\ntask t(robot r, location dest) {\n    goal { robot_at r dest; }\n    plan {\n"
                   "        measure r charge as c;\n        move r to dest;\n    }\n}\n", encoding="utf-8")
    doc = json.loads(world_data())
    doc["tasks"] = {"t": {"bindings": {"r": "spot1", "dest": "stop_c"}}}
    world = tmp_path / "w.json"
    world.write_text(json.dumps(doc), encoding="utf-8")
    code, out, err = cli("export-goal", str(rob), "--world", str(world), "--from", "rl:agent:planner")
    assert code == 0, err


def test_the_ros2_backend_refuses_the_charge_before_any_motion_s8():
    pytest.importorskip("jsonschema")
    sys.path.insert(0, str(ROOT / "backends" / "ros2"))
    from robolang_ros2.executor import execute
    from test_ros2_executor import FakeOps, fixture
    _, bindings, site = fixture()
    for body in ("measure r charge as c; move r to dst;",
                 'wait until robot_charge r >= 80 % timeout 5 min else { fail "x"; } move r to dst;'):
        source = ("robolang 2.4;\ntask t(robot r, location dst) {\n  goal { robot_at r dst; }\n"
                  f"  plan {{ {body} }}\n}}\n")
        ops = FakeOps()
        record = execute(compile_source(source).ir.tasks[0], {"r": bindings["r"], "dst": bindings["dst"]}, site, ops)
        assert record["failure"]["code"] == "S509" and "charge" in record["failure"]["message"]
        assert ops.calls == [("guard", "ur")]


# -------- the reader, the CLI, version, spec and economy --------

def test_the_reader_reads_the_charge_forms_and_rejects_them_in_an_older_schema():
    for path in VALID:
        document = json.loads(to_json(compile_source(path.read_text(encoding="utf-8")).ir))
        assert read_ir(json.dumps(document).encode())
    older = json.loads(to_json(compile_source((FIXTURES / "valid" / "report_the_charge.rob").read_text("utf-8")).ir))
    older["ir_schema_version"], older["language_version"] = "1.11", "2.3"
    from robolang.ir_reader import IRError
    with pytest.raises(IRError) as caught:
        read_ir(json.dumps(older).encode())
    assert caught.value.diagnostics[0].code == "I115"


def test_version_shows_language_2_4(capsys):
    code, _, _ = cli("--version")
    out = capsys.readouterr().out
    assert code == 0
    assert "2.2, 2.3 and 2.4" in out and "1.10, 1.11 and 1.12" in out and "mission IR 1.0, 1.1, 1.2, 1.3 and 1.4" in out


def test_validate_simulate_and_inspect_work_on_a_2_4_file(tmp_path):
    rob = tmp_path / "t.rob"
    rob.write_text((FIXTURES / "valid" / "charge_below_a_threshold_then_dock.rob").read_text("utf-8"), encoding="utf-8")
    doc = json.loads(world_data())
    doc["tasks"] = {"round_or_dock": {"bindings": {"r": "spot1", "dock": "dock_1", "stop_c": "stop_c"}}}
    world = tmp_path / "w.json"
    world.write_text(json.dumps(doc), encoding="utf-8")
    code, out, err = cli("validate", str(rob))
    assert code == 0 and "valid RoboLang 2.4" in out, err
    code, out, err = cli("inspect", str(rob))
    assert code == 0 and "measure r charge as c" in out, err
    code, out, err = cli("validate", str(rob), "--world", str(world))
    assert code == 0, err
    code, out, err = cli("simulate", str(rob), "--world", str(world))
    assert code == 0 and json.loads(out)["steps"][0]["reading"]["kind"] == "charge", err
    code, ir_text, _ = cli("inspect", str(rob), "--json")
    ir_path = tmp_path / "t.json"
    ir_path.write_text(ir_text, encoding="utf-8")
    code, out, err = cli("simulate", "--ir", str(ir_path), "--world", str(world))
    assert code == 0 and json.loads(out)["inputs"]["source_kind"] == "ir", err


def test_every_new_code_is_in_the_specification_tables():
    table = set(re.findall(r"^\| `((?:[EISX]\d{3})(?:`, `[EISX]\d{3})*)` \|", SPEC, re.MULTILINE))
    listed = {code for row in table for code in row.split("`, `")}
    assert {"S012", "S013", "S239", "E322", "E330", "S509"} <= listed
    for code in ("S012", "S013", "S239"):
        assert f"`{code}`" in SIM_SPEC, code


def test_the_grammar_file_says_what_2_4_cannot_say_s10():
    """The grammar file is the grammar-only prompt: it must say what 2.4 cannot (S10) and what to keep."""
    grammar = (ROOT / "spec" / "grammar-v2.4.ebnf").read_text(encoding="utf-8")
    for phrase in ("self-tests", "faults", "time until empty", "energy for a route", "arithmetic on a charge",
                   "a trigger", "inexpressible", "infeasible", "robot's own low-battery protection",
                   "never overrides", "no ==", "unknown charge", "channels"):
        assert phrase in grammar, phrase


def test_the_economy_no_statement_one_kind_one_live_form_no_reserved_word():
    from robolang import tokens
    assert tokens.RESERVED_V2_0 == RESERVED_BY_VERSION["2.4"]
    assert tokens.STATEMENTS_V1_8 == ("move", "grasp", "place", "inspect", "wait", "communicate", "measure", "do")
    assert tokens.CHARGE_KIND == "charge" and tokens.LIVE_CHARGE == "robot_charge"
    assert set(tokens.KIND_UNITS_V2_4) - set(tokens.KIND_UNITS_V2_2) == {"charge"}
    from robolang.sim import world
    assert world.WORLD_SCHEMA_VERSIONS[-1] == "1.9" and world._CHARGES == ("1.9",)


def test_no_dataclass_of_the_charge_has_a_mutable_default_python_3_11():
    from robolang import ir, nodes
    from robolang.sim import charge, executor, mission, world
    for module in (charge, mission, executor, world, nodes, ir):
        for item in vars(module).values():
            if dataclasses.is_dataclass(item) and isinstance(item, type):
                for f in dataclasses.fields(item):
                    if f.default is not dataclasses.MISSING:
                        hash(f.default)


def test_the_safety_case_has_hazard_h28_and_the_robots_own_protection_comes_first():
    text = (ROOT / "docs" / "safety-case-draft.md").read_text(encoding="utf-8")
    assert "| H28 | A wrong or stale charge reading lets a robot go on with a low battery |" in text
    assert "low-battery protection always takes precedence" in text


def test_scoring_cost_planning_checks_do_not_walk_paths():
    """S012/S013 read the plan's own sequence once: a deep plan with many ifs costs nothing."""
    ifs = "".join("        if robot_ready r {\n" for _ in range(3)) + "        move r to dest;\n" + "        }\n" * 3
    task = compile_source("robolang 2.4;\ntask t(robot r, location dest) {\n    goal { robot_at r dest; }\n    plan {\n"
                          "        measure r charge as c;\n" + ifs + "        move r to dest;\n    }\n}\n").ir.tasks[0]
    world = load_world(world_data(), "w.json", task, {"r": "spot1", "dest": "stop_c"})
    assert check_task(task, world) == []
