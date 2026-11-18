"""
RoboLang 2.0: the mission layer (spec/robolang-v2.0.md, decision 0041 M1-M10 and clarifications 1-14).

Front end, Mission IR 1.0 and its reader, the mission run in the simulation (spec/simulation-v1.0.md section 18),
the mission run record, `simulate --mission`, and the refusals in `export-goal`, `delegate` and the ROS 2 backend.
No test here needs a model, a network or a time zone database.

To regenerate the IR snapshots after an intended change:

    ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_lang20.py
"""

from __future__ import annotations
import copy
import io
import json
import os
import re
from decimal import Decimal
from pathlib import Path

import pytest

from robolang import RoboLangError, check, check_file
from robolang.cli import main
from robolang.ir import compile_source, to_json
from robolang.ir_reader import IRError, read_ir
from robolang.sim import load_world, mission_task, run_mission, to_mission_record
from robolang.tokens import MISSION_WORDS, RESERVED_BY_VERSION

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "v2.0"
SNAPSHOTS = ROOT / "tests" / "snapshots" / "ir-v2.0"
UPDATE = os.environ.get("ROBOLANG_UPDATE_SNAPSHOTS") == "1"
SPEC = (ROOT / "spec" / "robolang-v2.0.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")

# -------- helpers --------

ROUND = """task round(robot r, location home, location far) {
    goal { robot_at r home; }
    plan { move r to far; wait for 5 s; move r to home; }
}
"""
CARRY = """task transfer(robot r, object item, location src, location dst) {
    goal { object_at item dst; }
    plan { move r to src; grasp r item; move r to dst; place r item at dst; }
}
"""
WORLD = {
    "world_schema": "robolang.sim.world",
    "world_schema_version": "1.5",
    "clock": {"start": "2026-11-18T21:30:00", "zone": "Europe/Prague"},
    "locations": ["gate_in", "gate_out", "dock", "table"],
    "regions": {"quay": {"clear": False}},
    "robots": {"tug1": {"ready": True, "location": "gate_in", "gripper": True}},
    "objects": {"box1": {"present": True, "location": "dock"}, "temp_pan": {"present": True, "location": "dock"}},
    "events": [],
    "tasks": {},
}
BIND = {"r": "tug1", "home": "gate_in", "far": "gate_out", "quay": "quay"}


def mission_source(body: str, params: str = "robot r, location home, location far, region quay",
                   tasks: str = ROUND) -> str:
    return f"robolang 2.0;\n\n{tasks}\nmission m({params}) {{\n{body}\n}}\n"


def codes(source: str) -> list[tuple[str, int]]:
    try:
        check(source)
    except RoboLangError as error:
        return [(d.code, d.line) for d in error.diagnostics]
    return []


def world_data(patch=None) -> bytes:
    world = copy.deepcopy(WORLD)
    if patch is not None:
        patch(world)
    return json.dumps(world).encode()


def mission_run(source: str, patch=None, bindings=None, fail_runs=(), name="m"):
    document = compile_source(source).ir
    mission = document.mission(name)
    world = load_world(world_data(patch), "w.json", mission_task(mission), bindings or BIND)
    return run_mission(document, mission, world, fail_runs=fail_runs)


def event(seconds: int, target: str = "quay", fact: str = "region_clear", value: bool = True) -> dict:
    return {"at": {"value": str(seconds), "unit": "s"}, "fact": fact, "target": target, "value": value}


def repeat(head: str, *runs: str, until: str = "", policy: str = "") -> str:
    """The body of a mission: `repeat HEAD { RUNS } until UNTIL;` and the failure policy."""
    lines = [f"    repeat {head} {{" if head else "    repeat {"]
    lines += [f"        {run}" for run in runs]
    lines.append("    }" + (f" until {until}" if until else "") + ";")
    return "\n".join(lines) + ("\n" + policy if policy else "")


RUN = "run round(r, home, far);"


def shuttle(extra: str = "", until: str = "", policy: str = "") -> str:
    return mission_source(f"    repeat {extra} {{\n        run round(r, home, far);\n    }}{until or ';'}\n{policy}")


# -------- the language --------

def test_two_point_zero_adds_seven_reserved_words_and_nothing_else():
    """B7: at most 7 new reserved words, 1 top-level declaration (`mission`)."""
    assert set(MISSION_WORDS) == {"mission", "repeat", "run", "runs", "stop", "continue", "retry"}
    assert RESERVED_BY_VERSION["2.0"] - RESERVED_BY_VERSION["1.9"] == set(MISSION_WORDS)
    assert len(MISSION_WORDS) <= 7


def test_a_1_9_file_cannot_hold_a_mission_and_its_words_stay_names():
    source = shuttle("at most 3 runs").replace("2.0", "1.9")
    assert codes(source)[0][0] == "E201"
    assert "expected 'task', found 'mission'" in str(pytest.raises(RoboLangError, check, source).value)
    names = ("robolang 1.9;\ntask run(robot r, location stop) {\n goal { robot_at r stop; }\n"
             " plan { move r to stop; }\n}\n")
    assert codes(names) == []  # `run` and `stop` are names before 2.0


def test_the_syntax_of_a_mission_in_every_form():
    full = mission_source(
        repeat("at most 200 runs, for at most 480 min", RUN, until="region_clear quay",
               policy="    on failure { retry 3; }"))
    program = check(full).program
    mission = program.missions[0]
    assert (mission.name.text, len(mission.runs), mission.repeat.max_runs.value) == ("m", 1, 200)
    assert mission.repeat.max_duration.value == 480 and mission.on_failure.retries.value == 3
    assert codes(mission_source(repeat("for at most 60 s, at most 5 runs", RUN))) == []
    assert codes(mission_source("    run round(r, home, far);")) == []  # one run, no repeat
    # a file of tasks only is a 2.0 file too
    assert codes("robolang 2.0;\n" + ROUND) == []


def test_a_mission_only_file_needs_its_tasks():
    assert codes("robolang 2.0;\nmission m(robot r) {\n  run round(r);\n}\n") == [("E326", 3)]
    assert codes("robolang 2.0;\n") == [("E210", 2)] or codes("robolang 2.0;\n")[0][0] == "E210"


def test_a_mission_with_a_syntax_error_does_not_hide_the_next_task_or_mission():
    source = (
        "robolang 2.0;\n" + ROUND +
        "mission a(robot r, location home, location far) {\n  repeat at most x runs {\n " + RUN + "\n };\n}\n"
        "mission b(robot r, location home, location far) {\n  repeat {\n " + RUN + "\n };\n}\n")
    assert [c for c, _ in codes(source)] == ["E201", "E325"]


def test_the_until_of_a_mission_may_read_a_value_but_not_a_binding():
    reading = mission_source(
        repeat("at most 5 runs", RUN, until="temperature of pan using probe < 40 degC"),
        "robot r, location home, location far, object pan, sensor probe")
    assert codes(reading) == []
    assert codes(reading.replace("temperature of pan using probe", "t"))[0][0] == "E317"
    both = mission_source(
        repeat("at most 5 runs", RUN, until="clock_time >= 22:00 or region_clear quay"))
    assert codes(both) == []


def test_a_time_of_day_until_is_a_bound_only_as_clock_time_at_or_after():
    until = "    repeat {\n        run round(r, home, far);\n    } until %s;"
    assert codes(mission_source(until % "clock_time >= 22:00")) == []
    assert codes(mission_source(until % "clock_time > 22:00 or region_clear quay")) == []
    for text in ("clock_time < 22:00", "clock_time >= 22:00 and region_clear quay", "not clock_time >= 22:00",
                 "clock_elapsed >= 5 min", "region_clear quay"):
        assert [c for c, _ in codes(mission_source(until % text))] == ["E325"], text


def test_every_message_names_what_to_do():
    text = str(pytest.raises(RoboLangError, check, shuttle()).value)
    assert "hard bound" in text and "at most N runs" in text
    run_a_mission = (mission_source("    " + RUN)
                     + "\nmission n(robot r, location home, location far) {\n run m(r, home, far, far);\n}\n")
    assert "a mission cannot run a mission" in str(pytest.raises(RoboLangError, check, run_a_mission).value)


# -------- fixtures --------

HEADER = re.compile(
    r"\A// RoboLang v2\.0 fixture: (?P<set>valid|invalid)\n"
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
        snapshot.write_text(actual, encoding="utf-8")
    assert actual == snapshot.read_text(encoding="utf-8")
    document = json.loads(actual)
    assert (document["ir_schema"], document["ir_schema_version"], document["task_ir_schema_version"]) == (
        "robolang.mission-ir", "1.0", "1.8")
    assert document["language_version"] == "2.0"
    assert read_ir(actual.encode()) == compile_source(path.read_text(encoding="utf-8")).ir


@pytest.mark.parametrize("path", INVALID, ids=lambda p: p.stem)
def test_invalid_fixture_is_rejected_with_the_stated_code_and_line_and_makes_no_ir(path):
    stated = header(path)
    assert path.name.startswith(stated["code"].lower()) and stated["code"][1] == STAGE_DIGIT[stated["stage"]]
    with pytest.raises(RoboLangError) as caught:
        compile_source(path.read_text(encoding="utf-8"), str(path))
    assert (caught.value.code, str(caught.value.line)) == (stated["code"], stated["line"])


def test_fixture_sets_are_counted():
    assert len(VALID) == 6 and len(INVALID) == 20


def test_every_new_code_is_in_the_specification_tables():
    table = set(re.findall(r"^\| `((?:[EISX]\d{3})(?:`, `[EISX]\d{3})*)` \|", SPEC, re.MULTILINE))
    listed = {code for row in table for code in row.split("`, `")}
    assert {"E325", "E326", "S401", "S402", "X607", "S514"} <= listed
    for code in ("S401", "S402"):
        assert re.search(rf"^\| `{code}` \|", SIM_SPEC, re.MULTILINE)
    assert "| S514 |" in (ROOT / "spec" / "ros2-backend-v0.1.md").read_text(encoding="utf-8")
    reader = (ROOT / "spec" / "ir-reader-v0.1.md").read_text(encoding="utf-8")
    assert "`I226`" in reader and "`I227`" in reader
    assert "X607" in (ROOT / "spec" / "export-goal-v0.1.md").read_text(encoding="utf-8")
    assert "X607" in (ROOT / "spec" / "delegate-v0.1.md").read_text(encoding="utf-8")


def test_the_grammar_file_says_what_2_0_cannot_say_erratum_e5():
    """The grammar file is the grammar-only prompt: it must say what 2.0 cannot (M2, M6, M7, G-CHOOSE)."""
    grammar = (ROOT / "spec" / "grammar-v2.0.ebnf").read_text(encoding="utf-8")
    for phrase in ("forever", "no bound", "every hour", "at 06:00", "on a message", "nest", "next object",
                   "inexpressible", "hard bound", "no hour unit", "480 min"):
        assert phrase in grammar, phrase


# -------- Mission IR 1.0 and the reader --------

def document_of(source: str | None = None) -> dict:
    source = source or mission_source(
        repeat("at most 20 runs, for at most 90 min", RUN, until="region_clear quay",
               policy="    on failure { retry 2; }"))
    return json.loads(to_json(compile_source(source).ir))


def problems(document) -> list[tuple[str, str]]:
    with pytest.raises(IRError) as caught:
        read_ir(json.dumps(document).encode())
    return [(d.code, d.pointer) for d in caught.value.diagnostics]


def test_mission_ir_shape_and_reading_back():
    document = document_of()
    assert list(document) == ["ir_schema", "ir_schema_version", "task_ir_schema_version", "language_version", "tasks",
                              "missions"]
    mission = document["missions"][0]
    assert list(mission) == ["name", "parameters", "repeat", "runs", "on_failure"]
    assert mission["repeat"] == {"max_runs": 20, "max_duration": {"value": "90", "unit": "min"},
                                 "until": {"kind": "region_clear", "region": "quay"}}
    assert mission["runs"] == [{"task": "round", "arguments": ["r", "home", "far"]}]
    assert mission["on_failure"] == {"policy": "retry", "retries": 2}
    text = to_json(compile_source(mission_source("    run round(r, home, far);")).ir)
    assert '"repeat"' not in text and '"on_failure": {\n        "policy": "stop"\n      }' in text
    assert read_ir(to_json(compile_source(mission_source("    run round(r, home, far);")).ir).encode()).missions


def test_a_file_of_tasks_only_in_2_0_is_task_ir_1_8_with_language_2_0():
    document = json.loads(to_json(compile_source("robolang 2.0;\n" + ROUND).ir))
    assert (document["ir_schema"], document["ir_schema_version"], document["language_version"]) == (
        "robolang.task-ir", "1.8", "2.0")
    assert "missions" not in document
    assert read_ir(json.dumps(document).encode()).language_version == "2.0"
    document["language_version"] = "1.9"  # Task IR 1.8 is also what 1.9 compiles to
    assert read_ir(json.dumps(document).encode()).language_version == "1.9"
    document["language_version"] = "1.8"
    assert problems(document) == [("I113", "/language_version")]


def test_the_reader_checks_the_header_of_a_mission_document():
    document = document_of()
    for key, value in (("ir_schema_version", "1.2"), ("task_ir_schema_version", "1.7"), ("language_version", "1.9")):
        bad = copy.deepcopy(document)
        bad[key] = value
        assert problems(bad)[0] == ("I113", f"/{key}")
    bad = copy.deepcopy(document)
    del bad["missions"]
    assert problems(bad) == [("I111", "")]
    bad = copy.deepcopy(document)
    bad["missions"] = []
    assert problems(bad) == [("I120", "/missions")]


def test_the_reader_restates_every_static_rule_of_a_mission():
    def broken(change):
        document = document_of()
        change(document["missions"][0])
        return problems(document)

    assert broken(lambda m: m["runs"][0].update(task="rounds")) == [("I227", "/missions/0/runs/0/task")]
    assert broken(lambda m: m["runs"][0].update(task="m")) == [("I227", "/missions/0/runs/0/task")]
    assert broken(lambda m: m["runs"][0]["arguments"].pop()) == [("I227", "/missions/0/runs/0/arguments")]
    assert broken(lambda m: m["runs"][0]["arguments"].__setitem__(2, "quay")) == [
        ("I204", "/missions/0/runs/0/arguments/2")]
    assert broken(lambda m: m["runs"][0]["arguments"].__setitem__(2, "nowhere")) == [
        ("I203", "/missions/0/runs/0/arguments/2")]
    assert broken(lambda m: m["repeat"].clear()) == [("I226", "/missions/0/repeat")]
    assert broken(lambda m: m["repeat"].update(max_runs=1001)) == [("I226", "/missions/0/repeat/max_runs")]
    assert broken(lambda m: m["repeat"].update(max_runs=0)) == [("I205", "/missions/0/repeat/max_runs")]
    assert broken(lambda m: m["on_failure"].update(retries=101)) == [("I226", "/missions/0/on_failure/retries")]
    assert broken(lambda m: m["on_failure"].update(retries=0)) == [("I205", "/missions/0/on_failure/retries")]
    assert broken(lambda m: m["repeat"].update(until={"kind": "compare", "left": {"kind": "binding", "name": "t"},
                                                      "op": "<", "right": {"kind": "quantity", "value": "4",
                                                                           "unit": "degC"}})) == [
        ("I218", "/missions/0/repeat/until/left")]
    assert broken(lambda m: m["parameters"].append({"name": "r", "type": "robot"})) == [
        ("I202", "/missions/0/parameters/4/name")]
    document = document_of()
    document["missions"][0]["name"] = "round"
    assert problems(document) == [("I201", "/missions/0/name")]


def test_the_reader_checks_the_form_of_a_mission():
    def broken(change):
        document = document_of()
        change(document["missions"][0])
        return problems(document)

    assert broken(lambda m: m.update(extra=1)) == [("I112", "/missions/0/extra")]
    assert broken(lambda m: m.pop("runs")) == [("I111", "/missions/0")]
    assert broken(lambda m: m["runs"].clear()) == [("I120", "/missions/0/runs")]
    assert broken(lambda m: m["repeat"].update(max_runs="3")) == [("I110", "/missions/0/repeat/max_runs")]
    assert broken(lambda m: m["repeat"].update(max_runs=True)) == [("I110", "/missions/0/repeat/max_runs")]
    assert broken(lambda m: m["on_failure"].update(policy="forever")) == [("I114", "/missions/0/on_failure/policy")]
    assert broken(lambda m: m["on_failure"].pop("retries")) == [("I111", "/missions/0/on_failure")]
    assert broken(lambda m: m.update(on_failure={"policy": "stop", "retries": 1})) == [
        ("I112", "/missions/0/on_failure/retries")]
    assert broken(lambda m: m["repeat"].update(max_duration={"value": "2", "unit": "h"})) == [
        ("I117", "/missions/0/repeat/max_duration/unit")]


# -------- the simulation: outcomes (0041 M3, M4, M5; clarifications 5-9) --------

def test_a_count_bound_ends_bounded_and_every_run_is_a_complete_verified_run():
    result = mission_run(shuttle("at most 30 runs"))
    assert (result.outcome, result.ended_by, len(result.runs), result.runs_verified) == ("bounded", "run_bound", 30, 30)
    assert all(r.result.state == "SUCCEEDED" and r.result.goal for r in result.runs)
    assert result.end_t_ms == 30 * 5000  # continuous time: five seconds of waiting in each run


def test_until_is_checked_after_each_run_and_ends_completed():
    source = shuttle("at most 12 runs", " until region_clear quay;")
    result = mission_run(source, lambda w: w["events"].append(event(12)))
    assert (result.outcome, result.ended_by, len(result.runs)) == ("completed", "until", 3)
    assert [r.until for r in result.runs] == ["false", "false", "true"]
    assert [r.t_start_ms for r in result.runs] == [0, 5000, 10000]  # an event at 12 s is first seen after run 3 ends


def test_the_bound_is_reached_when_the_until_never_holds():
    result = mission_run(shuttle("at most 12 runs", " until region_clear quay;"))
    assert (result.outcome, result.ended_by, len(result.runs)) == ("bounded", "run_bound", 12)


def test_a_duration_bound_is_checked_before_each_run_and_a_started_run_is_never_cut():
    result = mission_run(shuttle("for at most 12 s"))  # runs of 5 s: they start at 0, 5 and 10 s; the third ends at 15
    assert (result.outcome, result.ended_by) == ("bounded", "duration_bound")
    assert (len(result.runs), result.end_t_ms) == (3, 15000)
    result = mission_run(shuttle("for at most 15 s"))
    assert (len(result.runs), result.end_t_ms) == (3, 15000)  # 15 is not less than 15: no fourth run


def test_a_task_that_cannot_fit_the_duration_even_once_fails_before_the_run():
    result = mission_run(shuttle("for at most 4 s"))
    assert (result.outcome, result.ended_by, result.runs) == ("failed", "cannot_fit", ())
    assert result.failure.code == "S401" and "needs at least 5 s" in result.failure.message
    assert mission_run(shuttle("for at most 5 s")).outcome == "bounded"  # exactly one run fits


def test_a_wait_in_a_branch_that_fails_does_not_count_towards_the_least_time():
    tasks = ('task t(robot r, location a, region q) {\n goal { robot_at r a; }\n plan {\n'
             '  if region_clear q { wait for 1 s; } else { wait for 9 s; }\n  move r to a;\n }\n}\n')
    source = mission_source("    repeat for at most 2 s {\n        run t(r, home, quay);\n    };",
                            "robot r, location home, region quay", tasks)
    assert mission_run(source, bindings={"r": "tug1", "home": "gate_in", "quay": "quay"}).ended_by == "duration_bound"


def test_the_default_policy_is_stop_and_a_failed_run_is_never_counted_as_done():
    result = mission_run(shuttle("at most 30 runs"), fail_runs={7})
    assert (result.outcome, result.ended_by, len(result.runs), result.runs_verified) == ("failed", "run_failed", 7, 6)
    assert (result.failure.code, result.failure.run) == ("S251", 7)


def test_continue_logs_the_failed_run_and_goes_on():
    result = mission_run(shuttle("at most 12 runs", policy="    on failure { continue; }"), fail_runs={5})
    assert (result.outcome, result.ended_by, len(result.runs), result.runs_verified) == ("bounded", "run_bound", 12, 11)
    assert result.runs_failed == 1 and not result.runs[4].verified


def test_every_run_failed_under_continue_is_failed_not_bounded_clarification_6():
    result = mission_run(shuttle("at most 15 runs", policy="    on failure { continue; }"), fail_runs="all")
    assert (result.outcome, result.ended_by, len(result.runs), result.runs_verified) == ("failed", "all_failed", 15, 0)
    assert result.failure.code == "S251"


def test_until_after_a_failed_run_under_continue_is_not_a_completion():
    source = shuttle("at most 3 runs", " until region_clear quay;", "    on failure { continue; }")
    result = mission_run(source, lambda w: w["regions"].update(quay={"clear": True}), fail_runs={1})
    assert [(r.verified, r.until) for r in result.runs] == [(False, None), (True, "true")]  # run 2 is the first checked
    assert result.outcome == "completed"


def test_retry_without_repeat_is_one_run_and_at_most_n_more_clarification_5():
    source = mission_source("    run transfer(r, item, src, dst);\n    on failure { retry 12; }",
                            "robot r, object item, location src, location dst", CARRY)
    bind = {"r": "tug1", "item": "box1", "src": "dock", "dst": "table"}
    first = mission_run(source, bindings=bind)
    assert (first.outcome, first.ended_by, len(first.runs)) == ("completed", "run_verified", 1)
    two = mission_run(source, bindings=bind, fail_runs={1, 2})
    assert (two.outcome, len(two.runs), [r.attempt for r in two.runs]) == ("completed", 3, [1, 2, 3])
    gone = mission_run(source, lambda w: w["objects"].update(box1={"present": False}), bind)
    assert (gone.outcome, gone.ended_by, len(gone.runs)) == ("failed", "retries_used", 13)  # 1 run and 12 retries


def test_retries_count_against_the_bound_clarification_7():
    source = shuttle("at most 3 runs", policy="    on failure { retry 5; }")
    result = mission_run(source, fail_runs="all")
    assert (result.outcome, result.ended_by, len(result.runs)) == ("failed", "bound_with_retry", 3)
    ok = mission_run(source, fail_runs={1})  # one retry, then two more rounds: the bound is the run count
    assert (ok.outcome, len(ok.runs), ok.runs_verified) == ("bounded", 3, 2)


def test_several_run_lines_each_count_towards_the_bound_clarification_1():
    source = mission_source(repeat("at most 5 runs", RUN, RUN))
    result = mission_run(source)
    assert (len(result.runs), [r.line for r in result.runs]) == (5, [1, 2, 1, 2, 1])


def test_one_world_with_continuous_time_clarification_9():
    tasks = ('task carry(robot r, object item, location src, location dst) {\n goal { object_at item dst; }\n'
             ' plan { move r to src; grasp r item; move r to dst; place r item at dst; wait for 3 s; }\n}\n')
    source = mission_source(
        "    repeat at most 4 runs {\n        run carry(r, item, a, b);\n        run carry(r, item, b, a);\n    };",
        "robot r, object item, location a, location b", tasks)
    result = mission_run(source, bindings={"r": "tug1", "item": "box1", "a": "dock", "b": "table"})
    assert result.runs_verified == 4  # the second run finds the box where the first left it
    assert [r.result.end_t_ms for r in result.runs] == [3000, 6000, 9000, 12000]
    assert dict(result.final_world.objects)["box1"].location == "dock"  # carried to the table and back, twice
    same = mission_run(source.replace("run carry(r, item, b, a);", "run carry(r, item, a, b);"),
                       bindings={"r": "tug1", "item": "box1", "a": "dock", "b": "table"})
    assert (same.runs_verified, same.outcome, same.failure.code) == (1, "failed", "S215")  # the box is no longer at a


def test_elapsed_is_per_task_run_and_the_clock_time_goes_on():
    tasks = ('task stamp(robot r, location a) {\n goal { robot_at r a; }\n plan {\n  measure r elapsed as took;\n'
             '  communicate r to "ops" with took;\n  wait for 10 s;\n  move r to a;\n }\n}\n')
    result = mission_run(mission_source("    repeat at most 3 runs {\n        run stamp(r, home);\n    };",
                                        "robot r, location home", tasks), bindings={"r": "tug1", "home": "gate_in"})
    sent = [r.result.steps[1].message.value for r in result.runs]
    assert [m.value for m in sent] == [Decimal(0)] * 3 and [m.t_ms for m in sent] == [0, 10000, 20000]


def test_a_time_of_day_bound_ends_the_mission_and_one_day_only():
    source = shuttle("", " until clock_time >= 21:30:20;")
    result = mission_run(source)  # runs of 5 s from 21:30:00: after the fourth, 20 s have gone
    assert (result.outcome, result.ended_by, len(result.runs)) == ("completed", "until", 4)
    late = shuttle("", " until clock_time >= 23:59:59;")
    ending = mission_run(late, lambda w: w["clock"].update(start="2026-11-18T23:59:50"))
    assert (ending.outcome, ending.ended_by, ending.failure.code) == ("failed", "until_unknown", "S402")
    assert len(ending.runs) == 2  # 23:59:55 after run 1; after run 2 the day has ended (S238): unknown, not false
    unclocked = mission_run(source, lambda w: (w.update(world_schema_version="1.4"), w.pop("clock")))
    assert (unclocked.outcome, unclocked.failure.code) == ("failed", "S402")


def test_an_unknown_until_stops_the_mission_fail_closed_m3():
    source = mission_source(repeat("at most 5 runs", RUN, until="object_at box dock"),
                            "robot r, location home, location far, object box, location dock")
    result = mission_run(source, lambda w: w["objects"].update(box1={}),
                         {"r": "tug1", "home": "gate_in", "far": "gate_out", "box": "box1", "dock": "dock"})
    assert (result.outcome, result.ended_by, result.failure.code) == ("failed", "until_unknown", "S402")
    assert len(result.runs) == 1


def test_authorization_and_the_checks_before_a_run_apply_to_each_run_with_mission_time():
    tasks = ('task at_ten(robot r, location a) {\n goal { robot_at r a; }\n plan {\n'
             '  wait until clock_time >= 21:30:08 timeout 1 min else { fail "late"; }\n  move r to a;\n }\n}\n')
    source = mission_source(repeat("at most 3 runs", "run at_ten(r, home);"), "robot r, location home", tasks)
    result = mission_run(source, bindings={"r": "tug1", "home": "gate_in"})
    assert result.runs_verified == 3 and [r.result.end_t_ms for r in result.runs] == [8000, 8000, 8000]
    later = mission_run(source.replace("21:30:08", "21:29:00"), bindings={"r": "tug1", "home": "gate_in"})
    assert (later.outcome, later.failure.code) == ("failed", "S010")  # already passed at the start, as in 1.9


def test_a_task_that_carries_its_own_limits_and_pre_keeps_them_in_every_run():
    tasks = ('task guarded(robot r, location a, region q) {\n @safety no_go_region q;\n pre { robot_ready r; }\n'
             ' goal { robot_at r a; }\n plan { move r to a; }\n}\n')
    source = mission_source("    repeat at most 3 runs {\n        run guarded(r, home, quay);\n    };",
                            "robot r, location home, region quay", tasks)
    result = mission_run(source, lambda w: w["robots"]["tug1"].update(honours=["no_go_region"]),
                         {"r": "tug1", "home": "gate_in", "quay": "quay"})
    assert result.runs_verified == 3 and result.outcome == "bounded"
    assert [s.status for s in result.runs[0].result.safety_limits] == ["checked_against_world_not_enforced"]
    result = mission_run(source, lambda w: w["robots"]["tug1"].update(ready=False, honours=["no_go_region"]),
                         {"r": "tug1", "home": "gate_in", "quay": "quay"})
    assert (result.outcome, result.failure.code, len(result.runs)) == ("failed", "S101", 1)


def test_a_single_task_run_is_unchanged_by_the_mission_support():
    """run(task, world) with no state starts a world of its own: 1.x results do not move."""
    from robolang.sim import run
    document = compile_source("robolang 2.0;\n" + ROUND).ir
    world = load_world(world_data(), "w.json", document.tasks[0], {"r": "tug1", "home": "gate_in", "far": "gate_out"})
    first, second = run(document.tasks[0], world), run(document.tasks[0], world)
    assert first == second and first.state == "SUCCEEDED" and first.end_t_ms == 5000


def test_the_simulation_is_a_function_of_its_inputs():
    source = shuttle("at most 6 runs", " until region_clear quay;")
    one = mission_run(source, lambda w: w["events"].append(event(17)))
    two = mission_run(source, lambda w: w["events"].append(event(17)))
    assert one == two and (one.outcome, len(one.runs)) == ("completed", 4)


def test_a_world_with_mission_bindings_in_the_world_file():
    document = compile_source(shuttle("at most 2 runs")).ir
    data = json.loads(world_data())
    data["tasks"] = {"m": {"bindings": BIND}}
    world = load_world(json.dumps(data).encode(), "w.json", mission_task(document.missions[0]))
    assert run_mission(document, document.missions[0], world).runs_verified == 2


# -------- the mission run record and the command --------

def cli(*args: str):
    out, err = io.StringIO(), io.StringIO()
    code = main(list(args), out=out, err=err)
    return code, out.getvalue(), err.getvalue()


@pytest.fixture
def files(tmp_path):
    rob = tmp_path / "m.rob"
    rob.write_text(shuttle("at most 12 runs", " until region_clear quay;"), encoding="utf-8")

    def world(*events):
        path = tmp_path / "w.json"
        path.write_bytes(world_data(lambda w: w["events"].extend(events)))
        return str(path)

    return str(rob), world


def simulate(files, *events, extra=()):
    rob, world = files
    bind = [f"{k}={v}" for k, v in BIND.items()]
    return cli("simulate", rob, "--world", world(*events), "--mission", "m", "--bind", *bind, *extra)


def test_simulate_mission_prints_the_mission_record_and_exits_by_the_outcome(files):
    code, out, err = simulate(files, event(12))
    record = json.loads(out)
    assert code == 0 and "completed (until; 3 task runs, 3 verified)" in err
    assert list(record)[:7] == ["mission_record_schema", "mission_record_schema_version", "run_record_schema_version",
                                "mode", "run_id", "inputs", "mission"]
    assert (record["mission_record_schema"], record["mission_record_schema_version"], record["mode"]) == (
        "robolang.sim.mission-run", "1.0", "simulation")
    assert (record["outcome"], record["ended_by"], record["runs_started"], record["runs_verified"],
            record["runs_failed"], record["failure"]) == ("completed", "until", 3, 3, 0, None)
    assert record["inputs"]["mission"] == "m" and record["inputs"]["bindings_source"] == "cli"
    assert record["bindings"] == BIND and record["end_t_ms"] == "15000"
    first = record["runs"][0]
    assert (first["index"], first["line"], first["attempt"], first["task_name"]) == (1, 1, 1, "round")
    assert first["until"] == "false"
    assert first["state"] == "SUCCEEDED" and first["core_outcome"]["outcome"] and first["goal"]
    assert "inputs" not in first and "task" not in first and "final_world" not in first
    assert [t["name"] for t in record["tasks"]] == ["round"]
    assert record["runs"][2]["until"] == "true" and record["final_world"]["regions"]["quay"]["clear"] is True
    assert record["events_applied"] == [{"t_ms": "12000", "fact": "region_clear", "target": "quay", "value": True}]
    assert out == simulate(files, event(12))[1]  # deterministic


def test_the_mission_run_id_names_the_mission_and_differs_from_a_task_run_id(files):
    record = json.loads(simulate(files)[1])
    assert record["run_id"].startswith("sim-") and record["runs"][0]["run_id"] != record["run_id"]
    assert record["run_id"] != json.loads(simulate(files, event(12))[1])["run_id"]


def test_exit_codes_follow_the_outcome(files):
    assert simulate(files)[0] == 0  # bounded
    rob, world = files
    failing = Path(rob).read_text().replace("at most 12 runs", "at most 12 runs, for at most 1 s")
    Path(rob).write_text(failing)
    code, out, err = simulate(files)
    assert code == 4 and json.loads(out)["outcome"] == "failed" and "S401" in err
    Path(rob).write_text(Path(rob).read_text().replace("for at most 1 s", "for at most 90 s").replace(
        "until region_clear quay", "until clock_time >= 21:30:03"))
    data = json.loads(world_data())
    data["world_schema_version"] = "1.4"
    data.pop("clock")
    unclocked = Path(rob).parent / "unclocked.json"
    unclocked.write_text(json.dumps(data))
    bind = [f"{k}={v}" for k, v in BIND.items()]
    code, out, err = cli("simulate", rob, "--world", str(unclocked), "--mission", "m", "--bind", *bind)
    assert code == 4 and json.loads(out)["outcome"] == "failed" and "S402" in err


def test_task_and_mission_are_exclusive_and_each_names_the_other(files):
    rob, world = files
    code, _, err = cli("simulate", rob, "--world", world(), "--task", "m", "--bind", "r=tug1")
    assert code == 2 and "'m' is a mission" in err and "--mission" in err
    code, _, err = cli("simulate", rob, "--world", world(), "--task", "round", "--mission", "m")
    assert code == 2 and "--task or --mission" in err
    code, _, err = cli("simulate", rob, "--world", world(), "--mission", "nope")
    assert code == 2 and "no mission named 'nope'" in err and "missions in this file: m" in err
    code, _, err = cli("simulate", rob, "--world", world(), "--mission", "m", "--bind", "x=tug1")
    assert code == 2 and "'x' is not a parameter of mission 'm'" in err
    code, _, err = cli("simulate", rob, "--world", world(), "--mission", "m", "--bind", "r=tug1")
    assert code == 3 and "E506" in err  # the other parameters are unbound


def test_simulate_without_mission_runs_the_one_task_of_a_file_that_has_missions(files):
    rob, world = files
    code, out, _ = cli("simulate", rob, "--world", world(), "--bind", "r=tug1", "home=gate_in", "far=gate_out")
    assert code == 0 and json.loads(out)["inputs"]["language_version"] == "2.0"
    assert json.loads(out)["run_record_schema_version"] == "1.8"


def test_validate_and_inspect_describe_missions(files):
    rob, _ = files
    code, out, _ = cli("validate", rob)
    assert code == 0 and "valid RoboLang 2.0 (1 task, 1 mission)" in out
    code, out, _ = cli("inspect", rob)
    assert code == 0 and "mission m" in out and "repeat: at most 12 runs" in out and "until: region_clear quay" in out
    assert "run round(r, home, far)" in out and "on failure: stop" in out
    code, out, _ = cli("inspect", rob, "--json")
    assert json.loads(out)["ir_schema"] == "robolang.mission-ir"
    code, out, _ = cli("inspect", rob, "--mission", "m", "--json")
    assert code == 0 and len(json.loads(out)["missions"]) == 1
    code, out, _ = cli("inspect", rob, "--task", "round", "--json")
    assert json.loads(out)["ir_schema"] == "robolang.task-ir"
    code, _, err = cli("inspect", rob, "--mission", "x")
    assert code == 2 and "no mission named 'x'" in err


def test_ir_input_round_trips_through_every_command(files, tmp_path):
    rob, world = files
    ir_path = tmp_path / "m.json"
    ir_path.write_text(cli("inspect", rob, "--json")[1])
    code, out, _ = cli("validate", "--ir", str(ir_path))
    assert code == 0 and "valid Mission IR 1.0 (RoboLang 2.0, 1 task, 1 mission)" in out
    bind = [f"{k}={v}" for k, v in BIND.items()]
    from_source = cli("simulate", rob, "--world", world(), "--mission", "m", "--bind", *bind)
    from_ir = cli("simulate", "--ir", str(ir_path), "--world", world(), "--mission", "m", "--bind", *bind)
    assert from_ir[0] == from_source[0] == 0
    assert json.loads(from_ir[1])["inputs"]["source_kind"] == "ir"
    def strip(text):
        return [{k: v for k, v in r.items() if k not in ("run_id", "core_outcome")} for r in json.loads(text)["runs"]]

    assert strip(from_ir[1]) == strip(from_source[1])
    invalid = json.loads(ir_path.read_text())
    invalid["missions"][0]["runs"][0]["task"] = "zz"
    ir_path.write_text(json.dumps(invalid))
    code, _, err = cli("validate", "--ir", str(ir_path))
    assert code == 1 and "error[I227]" in err


def test_export_goal_and_delegate_refuse_a_mission_x607(files):
    rob, world = files
    for name_args in (["--task", "m"], ["--mission", "m"]):
        code, out, err = cli("export-goal", rob, "--world", world(), "--from", "rl:agent:p", *name_args)
        assert (code, out) == (6, "") and "error[X607]" in err and "'m' is a mission" in err
        code, out, err = cli("delegate", rob, "--world", world(), "--from", "rl:agent:p", "--to", "rl:robot:tug1",
                             *name_args)
        assert (code, out) == (6, "") and "error[X607]" in err
    code, _, err = cli("export-goal", rob, "--world", world(), "--from", "rl:agent:p", "--mission", "zz")
    assert code == 2 and "no mission named 'zz'" in err


def test_a_task_of_a_file_with_missions_is_still_exported_as_before(files):
    rob, world = files
    data = json.loads(Path(world()).read_text())
    data["world_schema_version"] = "1.1"
    data.pop("clock")
    Path(world()).write_text(json.dumps(data))
    code, out, err = cli("export-goal", rob, "--world", world(), "--from", "rl:agent:p", "--to", "rl:robot:tug1",
                         "--bind", "r=tug1", "home=gate_in", "far=gate_out")
    assert code == 0 and json.loads(out)["payload"]["goal"], err


def test_the_ros2_backend_refuses_a_mission_s514():
    source = (ROOT / "backends" / "ros2" / "robolang_ros2" / "cli.py").read_text(encoding="utf-8")
    assert '"S514"' in source and "compiled.mission(args.task)" in source
    assert source.index("S514") < source.index("from .ros import ROSOperations")  # before any ROS import


def test_the_record_of_a_mission_has_no_floats_and_is_ascii(files):
    out = simulate(files)[1]
    assert out.isascii() and not re.search(r"\d\.\d", re.sub(r'"[^"]*"', "", out))


def test_to_mission_record_embeds_each_runs_own_record():
    document = compile_source(shuttle("at most 2 runs")).ir
    mission = document.missions[0]
    world = load_world(world_data(), "w.json", mission_task(mission), BIND)
    result = run_mission(document, mission, world, fail_runs={2})
    record = to_mission_record(result, document=document, run_id="sim-" + "0" * 64, source_sha256="a" * 64,
                               world_sha256="b" * 64, seed="0", world_schema_version="1.5", world_clock=world.clock)
    assert [r["state"] for r in record["runs"]] == ["SUCCEEDED", "FAILED"]
    assert record["runs"][1]["failure"]["code"] == "S251" and record["failure"]["run"] == 2
    assert record["inputs"]["world_clock"] == {"start": "2026-11-18T21:30:00", "zone": "Europe/Prague"}
