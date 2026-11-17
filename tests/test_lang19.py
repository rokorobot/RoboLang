"""
RoboLang 1.9: clock readings (spec/robolang-v1.9.md, decision 0042 T1-T12 and K1-K11).

Front end, Task IR 1.8 and its reader, world file 1.5 (`clock`), the readings and the live clock in the simulation,
the check before the run (`S010`), `S237`, `S238`, `validate --world`, run record 1.8, and the refusals in `delegate`
and the ROS 2 backend. No test here needs a model, a network or a time zone database.

To regenerate the IR snapshots after an intended change:

    ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_lang19.py
"""

from __future__ import annotations
import copy
import io
import json
import os
import re
import sys
from pathlib import Path

import pytest

from robolang import RoboLangError, check, check_file, ir
from robolang.cli import main
from robolang.ir import compile_source, to_json
from robolang.ir_reader import IRError, read_ir
from robolang.sim import WorldError, load_world, run, to_record

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "v1.9"
SNAPSHOTS = ROOT / "tests" / "snapshots" / "ir-v1.9"
UPDATE = os.environ.get("ROBOLANG_UPDATE_SNAPSHOTS") == "1"
SPEC = (ROOT / "spec" / "robolang-v1.9.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")

# -------- helpers --------

PARAMS = "robot r, robot s, object box, location a, location b"
ENTITY = {"r": "r1", "s": "r2", "box": "box1", "a": "a", "b": "b"}
START = "2026-11-17T21:30:00"
BASE_WORLD = {
    "world_schema": "robolang.sim.world",
    "world_schema_version": "1.5",
    "clock": {"start": START, "zone": "Europe/Prague"},
    "locations": ["a", "b"],
    "robots": {"r1": {"ready": True, "location": "a", "gripper": True},
               "r2": {"ready": True, "location": "a", "gripper": True}},
    "objects": {"box1": {"present": True, "location": "a"}},
    "tasks": {},
}


def task(plan: str, goal: str = "goal { robot_at r b; }", pre: str = "", params: str = PARAMS, version: str = "1.9",
         name: str = "t") -> str:
    return f"robolang {version};\ntask {name}({params}) {{\n{pre}\n{goal}\nplan {{\n{plan}\n}}\n}}\n"


def codes(source: str) -> list[str]:
    try:
        check(source)
    except RoboLangError as error:
        return [d.code for d in error.diagnostics]
    return []


def world_data(patch=None, start=START, version="1.5", name="t", bindings=None) -> bytes:
    world = copy.deepcopy(BASE_WORLD)
    world["world_schema_version"] = version
    if start is None or version != "1.5":  # no clock: an older world file has none
        world.pop("clock")
    else:
        world["clock"]["start"] = start
    if patch is not None:
        patch(world)
    world["tasks"] = {name: {"bindings": bindings or dict(ENTITY)}}
    return json.dumps(world).encode()


def simulate(source: str, start=START, patch=None, version="1.5"):
    """Compile, run against a world file (1.5, started at `start`), return (result, record)."""
    compiled = compile_source(source)
    plan = compiled.ir.tasks[0]
    bindings = {p.name: ENTITY[p.name] for p in plan.parameters}
    loaded = load_world(world_data(patch, start, version, plan.name, bindings), "w.json", plan)
    result = run(plan, loaded)
    record = to_record(result, task=plan, run_id="sim-x", source_sha256="0", world_sha256="0", seed="0",
                       language_version=compiled.ir.language_version, world_schema_version=loaded.schema_version,
                       world_clock=loaded.clock)
    return result, record


def failure(record) -> tuple[str, int | None] | None:
    """(code, the 1-based step the failure names)."""
    return None if record["failure"] is None else (record["failure"]["code"], record["failure"]["step"])


def readings(record) -> list[dict]:
    """The readings a run took, in step order (a failure handler's steps are steps of their own in the record)."""
    return [step["reading"] for step in record["steps"] if step["reading"] is not None]


def run_cli(args, cwd):
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(cwd)
    try:
        code = main(args, out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


# -------- front end (T1, T2, T3, T5, T6) --------

def test_version_1_9_selects_ir_schema_1_8_and_older_versions_keep_theirs():
    document = json.loads(to_json(compile_source(task("measure r time as t;\nmove r to b;")).ir))
    assert (document["language_version"], document["ir_schema_version"]) == ("1.9", "1.8")
    assert ir.IR_SCHEMA_VERSIONS["1.8"] == "1.7" and ir.IR_SCHEMA_VERSIONS["1.9"] == "1.8"


@pytest.mark.parametrize("version", ["1.6", "1.7", "1.8"])
def test_the_clock_in_an_earlier_version_is_still_an_error(version):
    assert codes(task("measure r time as t;\nmove r to b;", version=version)) == ["E217"]
    assert codes(task("wait until robot_ready r timeout 22:00 else { fail \"x\"; }", version=version)) == ["E101"]


def test_clock_time_and_clock_elapsed_are_the_only_new_reserved_words_and_time_is_not_one():
    from robolang.tokens import RESERVED_BY_VERSION
    assert RESERVED_BY_VERSION["1.9"] - RESERVED_BY_VERSION["1.8"] == {"clock_time", "clock_elapsed"}
    assert codes(task("move r to b;", params="robot r, location b, location clock_time")) == ["E212"]
    assert codes(task("measure r time as time;\nmove r to b;")) == []  # a binding may be called time
    assert codes(task("move r to b;", version="1.8", params="robot r, location b, location clock_time")) == []


def test_a_clock_reading_has_no_object_and_no_sensor_and_binds_a_name_once():
    assert codes(task("measure r time as t;\nmove r to b;")) == []
    assert codes(task("measure r elapsed as d;\nmeasure r time as d;\nmove r to b;")) == ["E318"]
    assert codes(task("measure r time as b;\nmove r to b;")) == ["E318"]
    assert codes(task("measure b time as t;\nmove r to b;")) == ["E304"]
    of_box = task("measure r time of box using cam as t;\nmove r to b;", params=PARAMS + ", sensor cam")
    assert codes(of_box) == ["E201"]
    assert codes(task("measure r temperature as t;\nmove r to b;")) == ["E201"]
    assert codes(task("measure r time as t else { fail \"no clock\"; }\nmove r to b;")) == []


@pytest.mark.parametrize("text", ["00:00", "09:05", "21:30:00", "23:59:59"])
def test_a_time_literal_is_hh_mm_or_hh_mm_ss(text):
    assert codes(task(f"measure r time as t;\nif t < {text} {{ move r to b; }} else {{ fail \"x\"; }}")) == []


@pytest.mark.parametrize("text", ["24:00", "9:30", "21:60", "21:30:60", "12:5", "12:30:", "1:2:3", "12:30:45:10",
                                  "1.5:30"])
def test_a_malformed_time_literal_is_e105_and_there_is_no_24_00(text):
    assert codes(task(f"measure r time as t;\nif t < {text} {{ move r to b; }} else {{ fail \"x\"; }}")) == ["E105"]


def test_the_literal_is_one_token_and_22_00_is_22_00_00_in_the_ir():
    one = to_json(compile_source(task('measure r time as t;\nif t < 22:00 { move r to b; } else { fail "x"; }')).ir)
    two = to_json(compile_source(task('measure r time as t;\nif t < 22:00:00 { move r to b; } else { fail "x"; }')).ir)
    assert one == two and '"value": "22:00:00"' in one


def test_comparisons_take_orderings_and_kinds_never_mix_t3():
    cases = {
        "if t < 22:00": [], "if t >= 22:00": [], "if t <= other": [],
        "if t == 22:00": ["E320"], "if t != 22:00": ["E320"],
        "if t < 5 s": ["E319"], "if t < d": ["E319"], "if t < 60 degC": ["E319"], 'if t < "x"': ["E319"],
        "if d < 90 s": [], "if d >= 2 min": [], "if d > 500 ms": [], "if d == 90 s": ["E320"],
        "if d < 22:00": ["E319"], "if d < t": ["E319"], "if d < 5 kg": ["E319"],
    }
    for condition, expected in cases.items():
        source = task(f"measure r time as t;\nmeasure r time as other;\nmeasure r elapsed as d;\n"
                      f"{condition} {{ move r to b; }} else {{ fail \"x\"; }}")
        assert codes(source) == expected, condition


def test_a_temperature_never_compares_with_a_time_or_a_duration():
    params = PARAMS + ", sensor cam"
    base = "measure r temperature of box using cam as w;\n"
    assert codes(task(base + 'if w < 22:00 { move r to b; } else { fail "x"; }', params=params)) == ["E319"]
    assert codes(task(base + 'if w < 5 s { move r to b; } else { fail "x"; }', params=params)) == ["E319"]


def test_a_duration_unit_is_for_a_clock_comparison_only_in_1_9():
    old = task('measure r elapsed as d;\nif d < 5 s { move r to b; } else { fail "x"; }', version="1.8")
    assert codes(old) == ["E217", "E206"]
    # the 1.6 comparison of a temperature with a duration is E206 in 1.8 and E319 in 1.9
    params = PARAMS + ", sensor cam"
    base = "measure r temperature of box using cam as w;\nif w < 5 s { move r to b; } else { fail \"x\"; }"
    assert codes(task(base, params=params, version="1.8")) == ["E206"]
    assert codes(task(base, params=params)) == ["E319"]


def test_the_live_clock_is_for_wait_until_only_e322_t6():
    wait = 'wait until clock_time >= 23:00 timeout 90 min else { fail "late"; }\nmove r to b;'
    assert codes(task(wait)) == []
    assert codes(task('wait until clock_elapsed > 90 s timeout 5 min else { fail "slow"; }\nmove r to b;')) == []
    assert codes(task('if clock_time < 22:00 { move r to b; } else { fail "x"; }')) == ["E322"]
    assert codes(task('if clock_elapsed < 1 min { move r to b; } else { fail "x"; }')) == ["E322"]
    assert codes(task("move r to b;", goal="goal { clock_time >= 22:00; }")) == ["E321"]
    assert codes(task("move r to b;", pre="pre { clock_time >= 22:00; }")) == ["E321"]


def test_a_live_clock_compares_like_a_binding_t3():
    def wait(condition):
        return task(f'wait until {condition} timeout 90 min else {{ fail "x"; }}\nmove r to b;')
    assert codes(wait("clock_time == 23:00")) == ["E320"]
    assert codes(wait("clock_time >= 90 s")) == ["E319"]
    assert codes(wait("clock_elapsed >= 23:00")) == ["E319"]
    assert codes(wait("clock_time >= 23:00 and clock_elapsed < 2 min")) == []
    assert codes(wait("not (clock_time < 23:00) or robot_ready r")) == []
    assert codes(wait("clock_time >= 23:00 and robot_at r b")) == []
    assert codes(wait("clock_time")) == ["E201"]
    assert codes(wait("23:00 <= clock_time")) == ["E201"]  # a literal on the left is not a comparison


def test_a_wait_for_a_time_needs_its_timeout_and_its_handler_as_before():
    assert codes(task("wait until clock_time >= 23:00;\nmove r to b;")) == ["E209"]
    assert codes(task("wait until clock_time >= 23:00 timeout 90 min;\nmove r to b;")) == ["E316"]


def test_there_is_no_hour_unit_in_1_9_so_two_hours_are_120_min():
    """Decision 0042 writes `12 h`; the language has no such unit (0042 T2: 'as written today')."""
    assert codes(task('wait until clock_time >= 23:00 timeout 12 h else { fail "x"; }\nmove r to b;')) == ["E206"]
    assert codes(task('wait until clock_time >= 23:00 timeout 720 min else { fail "x"; }\nmove r to b;')) == []


def test_errors_of_a_clock_reading_are_reported_with_the_others():
    source = (
        "robolang 1.9;\ntask t(robot r, location b) {\n  goal { robot_at r b; }\n  plan {\n"
        "    measure r time as t;\n"  # 5
        "    measure r elapsed as t;\n"  # 6: E318
        "    if t < 22:00 { move r to b; } else { fail \"x\"; }\n"  # 7: E319 (t is the elapsed one)
        "    if clock_time < 22:00 { move r to b; } else { fail \"x\"; }\n"  # 8: E322
        "    wait until clock_time == 22:00 timeout 1 min else { fail \"x\"; }\n"  # 9: E320
        "  }\n}\n"
    )
    with pytest.raises(RoboLangError) as caught:
        check(source)
    assert [(d.code, d.span.line) for d in caught.value.diagnostics] == [
        ("E318", 6), ("E319", 7), ("E322", 8), ("E320", 9)]


# -------- fixtures --------

HEADER = re.compile(
    r"\A// RoboLang v1\.9 fixture: (?P<set>valid|invalid)\n"
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
    assert (document["language_version"], document["ir_schema_version"]) == ("1.9", "1.8")
    assert read_ir(actual.encode()) == compile_source(path.read_text(encoding="utf-8")).ir


@pytest.mark.parametrize("path", INVALID, ids=lambda p: p.stem)
def test_invalid_fixture_is_rejected_with_the_stated_code_and_line_and_makes_no_ir(path):
    stated = header(path)
    assert path.name.startswith(stated["code"].lower()) and stated["code"][1] == STAGE_DIGIT[stated["stage"]]
    with pytest.raises(RoboLangError) as caught:
        compile_source(path.read_text(encoding="utf-8"), str(path))
    assert (caught.value.code, str(caught.value.line)) == (stated["code"], stated["line"])


def test_fixture_sets_are_counted():
    assert len(VALID) == 6 and len(INVALID) == 11


def test_every_new_code_is_in_the_specification_tables():
    table = set(re.findall(r"^\| `((?:[EISX]\d{3})(?:`, `[EISX]\d{3})*)` \|", SPEC, re.MULTILINE))
    listed = {code for row in table for code in row.split("`, `")}
    assert {"E105", "S010", "S237", "S238", "E518", "S510"} <= listed
    for code in ("S010", "S237", "S238", "E518"):
        assert f"`{code}`" in SIM_SPEC
    assert "| S510 |" in (ROOT / "spec" / "ros2-backend-v0.1.md").read_text(encoding="utf-8")
    assert "`I126`" in (ROOT / "spec" / "ir-reader-v0.1.md").read_text(encoding="utf-8")


def test_the_grammar_file_says_what_the_clock_cannot_say_erratum_e5():
    """The grammar file is the grammar-only prompt: it must say what 1.9 cannot (T4, T11)."""
    grammar = (ROOT / "spec" / "grammar-v1.9.ebnf").read_text(encoding="utf-8")
    for phrase in ("another day", "tomorrow at 01:00", "repeating at a time", "arithmetic", "other formats",
                   "midnight is the next day", "inexpressible"):
        assert phrase in grammar, phrase


# -------- Task IR 1.8 and the reader --------

STEPS = task(
    'measure r time as now;\nmeasure r elapsed as took;\n'
    'if now < 21:30 { wait until clock_time >= 22:15:30 timeout 90 min else { fail "late"; } }'
    ' else { fail "after"; }\n'
    'wait until clock_elapsed > 90 s or took >= 2 min timeout 5 min else { fail "slow"; }\n'
    'communicate r to "lead" with now;\ncommunicate r to "ops" with took;\nmove r to b;')


def document_of(source: str = STEPS) -> dict:
    return json.loads(to_json(compile_source(source).ir))


def problems(document) -> list[tuple[str, str]]:
    with pytest.raises(IRError) as caught:
        read_ir(json.dumps(document).encode())
    return [(d.code, d.pointer) for d in caught.value.diagnostics]


def test_ir_shape_of_the_clock_and_reading_back():
    document = document_of()
    steps = document["tasks"][0]["steps"]
    assert steps[0] == {"kind": "read_clock", "robot": "r", "reading": "time", "binding": "now"}
    assert steps[1] == {"kind": "read_clock", "robot": "r", "reading": "elapsed", "binding": "took"}
    compare = steps[2]["condition"]
    assert compare == {"kind": "compare", "left": {"kind": "binding", "name": "now"}, "op": "<",
                       "right": {"kind": "time", "value": "21:30:00"}}
    wait = steps[2]["then"][0]["condition"]
    assert wait["left"] == {"kind": "clock", "reading": "time"} and wait["right"]["value"] == "22:15:30"
    assert steps[3]["condition"]["conditions"][0]["left"] == {"kind": "clock", "reading": "elapsed"}
    assert read_ir(to_json(compile_source(STEPS).ir).encode()) == compile_source(STEPS).ir
    handled = document_of(task('measure r time as t else { fail "no clock"; }\nmove r to b;'))
    assert list(handled["tasks"][0]["steps"][0]) == ["kind", "robot", "reading", "binding", "else"]


def test_the_reader_checks_the_robot_the_kind_the_time_and_the_units_of_the_clock():
    d = document_of()
    d["tasks"][0]["steps"][0]["robot"] = "b"
    assert problems(d) == [("I204", "/tasks/0/steps/0/robot")]
    d = document_of()
    d["tasks"][0]["steps"][0]["robot"] = "nobody"
    assert problems(d) == [("I203", "/tasks/0/steps/0/robot")]
    d = document_of()
    d["tasks"][0]["steps"][0]["reading"] = "temperature"
    assert ("I125", "/tasks/0/steps/0/reading") in problems(d)
    d = document_of()
    d["tasks"][0]["steps"][2]["condition"]["right"]["value"] = "24:00:00"
    assert problems(d) == [("I126", "/tasks/0/steps/2/condition/right/value")]
    d = document_of()
    d["tasks"][0]["steps"][2]["condition"]["right"]["value"] = "21:30"
    assert problems(d) == [("I126", "/tasks/0/steps/2/condition/right/value")]
    d = document_of()
    d["tasks"][0]["steps"][2]["condition"]["right"] = {"kind": "quantity", "value": "5", "unit": "s"}
    assert ("I220", "/tasks/0/steps/2/condition/right") in problems(d)
    d = document_of()
    d["tasks"][0]["steps"][2]["condition"]["op"] = "=="
    assert problems(d) == [("I221", "/tasks/0/steps/2/condition")]
    d = document_of()
    d["tasks"][0]["steps"][2]["then"][0]["condition"]["left"]["reading"] = "sometime"
    assert ("I125", "/tasks/0/steps/2/then/0/condition/left/reading") in problems(d)
    d = document_of()
    d["tasks"][0]["steps"][4]["binding"] = "nothing"
    assert problems(d) == [("I218", "/tasks/0/steps/4/binding")]
    d = document_of()
    d["tasks"][0]["steps"][3]["condition"]["conditions"][0]["right"]["unit"] = "h"
    assert ("I117", "/tasks/0/steps/3/condition/conditions/0/right/unit") in problems(d)
    d = document_of()
    del d["tasks"][0]["steps"][0]["binding"]
    assert problems(d) == [("I111", "/tasks/0/steps/0")]


def test_the_live_clock_outside_a_wait_until_is_i223_and_in_a_goal_i222():
    d = document_of()
    d["tasks"][0]["steps"][2]["condition"]["left"] = {"kind": "clock", "reading": "time"}
    assert problems(d) == [("I223", "/tasks/0/steps/2/condition/left")]


@pytest.mark.parametrize("schema, language", [("1.7", "1.8"), ("1.6", "1.7"), ("1.5", "1.6")])
def test_an_older_schema_does_not_have_the_clock_i115(schema, language):
    d = document_of(task('measure r time as t;\nmove r to b;'))
    d["ir_schema_version"], d["language_version"] = schema, language
    assert {code for code, _ in problems(d)} == {"I115"}
    d = document_of(task('wait until clock_time >= 23:00 timeout 90 min else { fail "x"; }\nmove r to b;'))
    d["ir_schema_version"], d["language_version"] = schema, language
    assert {code for code, _ in problems(d)} <= {"I114", "I115"} and problems(d)


def test_a_duration_unit_in_a_do_value_or_an_older_comparison_is_not_allowed():
    """Schema 1.8 allows ms, s and min on the right of a comparison only, never as a `do` value."""
    d = document_of(task('do r open box to 5 s;\nmove r to b;', goal='goal { robot_at r b; }'))
    assert problems(d) == [("I117", "/tasks/0/steps/0/value/unit")]


# -------- world file 1.5 --------

def plan_of(source: str = STEPS):
    return compile_source(source).ir.tasks[0]


def test_world_file_1_5_reads_the_clock():
    world = load_world(world_data(), "w.json", plan_of())
    assert world.schema_version == "1.5"
    assert (world.clock.start, world.clock.zone, world.clock.start_s, world.clock.date) == (
        START, "Europe/Prague", 21 * 3600 + 30 * 60, "2026-11-17")
    assert load_world(world_data(start=None), "w.json", plan_of()).clock is None


@pytest.mark.parametrize("version", ["1.0", "1.1", "1.2", "1.3", "1.4"])
def test_older_world_files_have_no_clock_and_reject_the_field(version):
    assert load_world(world_data(version=version), "w.json", plan_of()).clock is None
    with pytest.raises(WorldError) as caught:
        load_world(world_data(lambda w: w.update(clock={"start": START, "zone": "UTC"}), version=version), "w.json",
                   plan_of())
    assert (caught.value.code, caught.value.pointer) == ("E503", "/clock")


@pytest.mark.parametrize("patch, code, pointer", [
    (lambda w: w["clock"].update(start="2026-11-17 21:30:00"), "E518", "/clock/start"),
    (lambda w: w["clock"].update(start="2026-11-17T21:30:00Z"), "E518", "/clock/start"),
    (lambda w: w["clock"].update(start="2026-11-17T21:30"), "E518", "/clock/start"),
    (lambda w: w["clock"].update(start="2026-02-30T21:30:00"), "E518", "/clock/start"),
    (lambda w: w["clock"].update(start="2026-11-17T24:00:00"), "E518", "/clock/start"),
    (lambda w: w["clock"].update(start="2026-11-17T21:60:00"), "E518", "/clock/start"),
    (lambda w: w["clock"].update(zone="Prague"), "E518", "/clock/zone"),
    (lambda w: w["clock"].update(zone="Europe/Pra gue"), "E518", "/clock/zone"),
    (lambda w: w["clock"].update(zone=""), "E518", "/clock/zone"),
    (lambda w: w["clock"].update(start=1700000000), "E504", "/clock/start"),
    (lambda w: w["clock"].pop("zone"), "E504", "/clock"),
    (lambda w: w["clock"].update(offset="+01:00"), "E503", "/clock/offset"),
    (lambda w: w.update(clock="now"), "E504", "/clock"),
])
def test_world_file_1_5_clock_is_validated_strictly(patch, code, pointer):
    with pytest.raises(WorldError) as caught:
        load_world(world_data(patch), "w.json", plan_of())
    assert (caught.value.code, caught.value.pointer) == (code, pointer)


@pytest.mark.parametrize("zone", ["UTC", "Europe/Prague", "America/Argentina/Buenos_Aires", "Etc/GMT+5",
                                  "Asia/Ho_Chi_Minh"])
def test_a_zone_is_checked_by_form_and_needs_no_database(zone):
    load_world(world_data(lambda w: w["clock"].update(zone=zone)), "w.json", plan_of())


# -------- the simulation: readings (T1, T7, K1, K6, T8) --------

def read_and_send(kind: str, before: str = "", start=START, patch=None):
    plan = f'{before}measure r {kind} as v;\ncommunicate r to "lead" with v;\nmove r to b;'
    return simulate(task(plan), start, patch)


def test_a_time_reading_is_the_clock_start_plus_the_simulated_time_and_is_sent_as_hh_mm_ss():
    result, record = read_and_send("time", "wait for 90 s;\n")
    assert record["state"] == "SUCCEEDED"
    reading = record["steps"][1]["reading"]
    assert (reading["kind"], reading["subject"], reading["value"], reading["unit"], reading["uncertainty"]) == (
        "time", "r1", "77490", "s", "0", ) and reading["text"] == "21:31:30" and reading["sensor"] == "clock"
    assert reading["t_ms"] == "90000"
    message = record["steps"][2]["message"]
    assert message["message"] is None and message["value"] == reading


def test_an_elapsed_reading_is_the_simulated_time_since_running_and_is_sent_in_seconds_with_its_unit():
    result, record = read_and_send("elapsed", "wait for 37 s;\n")
    reading = record["steps"][1]["reading"]
    assert (reading["kind"], reading["value"], reading["unit"], reading["text"]) == ("elapsed", "37", "s", "37 s")
    assert read_and_send("elapsed", "wait for 2500 ms;\n")[1]["steps"][1]["reading"]["text"] == "2.5 s"
    first = read_and_send("elapsed")[1]["steps"][0]["reading"]  # at once: the run has just reached RUNNING (K6)
    assert (first["value"], first["text"], first["t_ms"]) == ("0", "0 s", "0")


def test_a_fraction_of_a_second_is_cut_off_the_text_and_kept_in_the_reading():
    _, record = read_and_send("time", "wait for 2500 ms;\n")
    reading = record["steps"][1]["reading"]
    assert reading["value"] == "77402.5" and reading["text"] == "21:30:02"


def test_elapsed_needs_no_clock_and_time_fails_closed_without_one_t7():
    result, record = read_and_send("elapsed", "wait for 5 s;\n", start=None)
    assert record["state"] == "SUCCEEDED" and record["steps"][1]["reading"]["value"] == "5"
    assert record["inputs"]["world_clock"] is None
    result, record = read_and_send("time", start=None)
    assert record["state"] == "FAILED" and failure(record) == ("S237", 1)
    assert record["steps"][0]["status"] == "failed" and record["steps"][1]["status"] == "not_run"
    for version in ("1.0", "1.3", "1.4"):  # a world file older than 1.5 has no clock
        _, rec = simulate(task("measure r time as v;\nmove r to b;"), version=version)
        assert failure(rec) == ("S237", 1)


def test_a_failure_handler_takes_over_a_reading_that_cannot_be_taken():
    plan = 'measure r time as v else { communicate r to "ops" with "no clock"; fail "no clock"; }\nmove r to b;'
    _, record = simulate(task(plan), start=None)
    assert failure(record) == ("S262", 3) and record["steps"][0]["handled"] == "S237"
    assert record["steps"][1]["message"]["message"] == "no clock"


def test_the_clock_is_the_worlds_for_every_robot_k1():
    plan = 'measure r time as one;\nmeasure s time as two;\nif one >= 21:30 and two <= 21:30 { move r to b; } ' \
           'else { fail "differ"; }'
    result, record = simulate(task(plan, goal="goal { robot_at r b; }"))
    assert record["state"] == "SUCCEEDED"
    assert [record["steps"][i]["reading"]["subject"] for i in (0, 1)] == ["r1", "r2"]
    assert record["steps"][0]["reading"]["value"] == record["steps"][1]["reading"]["value"]


def test_a_reading_is_exact_and_the_comparison_is_decided_at_the_boundary():
    for start, expected in (("2026-11-17T21:29:59", "SUCCEEDED"), ("2026-11-17T21:30:00", "FAILED"),
                            ("2026-11-17T21:30:01", "FAILED")):
        plan = 'measure r time as t;\nif t < 21:30 { move r to b; } else { fail "late"; }'
        assert simulate(task(plan), start)[1]["state"] == expected
    plan = 'measure r time as t;\nif t <= 21:30 { move r to b; } else { fail "late"; }'
    assert simulate(task(plan), "2026-11-17T21:30:00")[1]["state"] == "SUCCEEDED"
    assert simulate(task(plan), "2026-11-17T21:30:01")[1]["state"] == "FAILED"


def test_time_on_the_next_day_is_never_a_wrapped_time_s238_t4():
    plan = 'wait for 600 s;\nmeasure r time as t;\nif t < 22:00 { move r to b; } else { fail "x"; }'
    # 23:55 + 10 minutes is 00:05 the next day: 1.9 knows one day, so the reading fails rather than say 00:05
    _, record = simulate(task(plan), "2026-11-17T23:55:00")
    assert failure(record) == ("S238", 2) and record["steps"][1]["status"] == "failed"
    _, record = simulate(task(plan), "2026-11-17T23:49:59")  # 23:59:59 is still the day
    assert record["state"] == "FAILED" and failure(record) == ("S262", 5)
    plan = 'wait for 600 s;\nmeasure r elapsed as d;\nmove r to b;'  # elapsed does not know days
    assert simulate(task(plan), "2026-11-17T23:55:00")[1]["state"] == "SUCCEEDED"


def test_the_record_names_the_world_clock_and_older_sources_write_their_records_unchanged():
    _, record = read_and_send("time")
    assert record["run_record_schema_version"] == "1.8" and record["inputs"]["language_version"] == "1.9"
    assert record["inputs"]["world_clock"] == {"start": START, "zone": "Europe/Prague"}
    assert list(record["inputs"])[list(record["inputs"]).index("world_schema_version") + 1] == "world_clock"
    old = to_record(run(*_compiled("robolang 1.8;\ntask t(robot r, location b) {\n goal { robot_at r b; }\n"
                                   " plan { move r to b; }\n}\n")), task=_task18(), run_id="x", source_sha256="0",
                    world_sha256="0", seed="0", language_version="1.8", world_schema_version="1.5")
    assert old["run_record_schema_version"] == "1.7" and "world_clock" not in old["inputs"]


def _task18():
    return compile_source("robolang 1.8;\ntask t(robot r, location b) {\n goal { robot_at r b; }\n"
                          " plan { move r to b; }\n}\n").ir.tasks[0]


def _compiled(source: str):
    plan = compile_source(source).ir.tasks[0]
    return plan, load_world(world_data(name="t", bindings={"r": "r1", "b": "b"}), "w.json", plan)


def test_the_run_is_deterministic():
    one = json.dumps(read_and_send("time", "wait for 7 s;\n")[1], sort_keys=True)
    two = json.dumps(read_and_send("time", "wait for 7 s;\n")[1], sort_keys=True)
    assert one == two


# -------- the live clock in a wait until (T5, T6, K5, K10) --------

def wait_for(condition: str, timeout: str = "90 min", after: str = "measure r elapsed as v;\nmove r to b;",
             start=START):
    plan = f'wait until {condition} timeout {timeout} else {{ fail "did not come"; }}\n{after}'
    return simulate(task(plan), start)


def test_a_wait_for_a_time_ends_at_the_exact_instant_k5():
    result, record = wait_for("clock_time >= 23:00")  # 21:30 + 90 min
    assert record["state"] == "SUCCEEDED"
    assert readings(record)[0]["value"] == "5400" and record["end_t_ms"] == "5400000"
    _, record = wait_for("clock_time >= 21:30:07", start="2026-11-17T21:30:00")
    assert readings(record)[0]["value"] == "7"
    _, record = wait_for("clock_time >= 21:30:07", start="2026-11-17T21:30:06")
    assert readings(record)[0]["value"] == "1"


def test_the_wait_is_not_polled_it_ends_at_the_time_whatever_the_timeout_k5():
    _, record = wait_for("clock_time >= 21:31:11", timeout="5 min")
    assert readings(record)[0]["value"] == "71"


def test_greater_than_is_true_the_millisecond_after_and_greater_or_equal_at_the_time():
    _, ge = wait_for("clock_time >= 21:31")
    _, gt = wait_for("clock_time > 21:31")
    assert readings(ge)[0]["value"] == "60"
    assert readings(gt)[0]["value"] == "60.001"


def test_a_wait_that_is_true_at_once_does_not_wait():
    _, record = wait_for("clock_time >= 21:30", start="2026-11-17T21:30:00")
    assert record["end_t_ms"] == "0" and readings(record)[0]["value"] == "0"
    _, record = wait_for("clock_time < 21:45")
    assert record["end_t_ms"] == "0"


def test_a_wait_whose_time_comes_after_the_timeout_fails_s241_and_the_handler_decides():
    _, record = wait_for("clock_time >= 23:00", timeout="10 min")
    assert failure(record) == ("S262", 2) and record["steps"][0]["handled"] == "S241"
    assert record["end_t_ms"] == "600000"
    plan = 'wait until clock_time >= 23:00 timeout 10 min else { fail "late"; }\nmove r to b;'
    assert failure(simulate(task(plan))[1]) == ("S262", 2)


def test_without_a_handler_the_timeout_is_s241_but_1_9_needs_the_handler_e316():
    assert codes(task("wait until clock_time >= 23:00 timeout 10 min;\nmove r to b;")) == ["E316"]


def test_a_wait_on_the_time_fails_closed_with_no_clock_s237():
    _, record = wait_for("clock_time >= 14:00", start=None)
    assert record["steps"][0]["handled"] == "S237" and failure(record) == ("S262", 2)
    _, record = wait_for("clock_elapsed >= 90 s", start=None)  # elapsed needs no clock
    assert record["state"] == "SUCCEEDED" and readings(record)[0]["value"] == "90"


def test_clock_elapsed_in_a_wait_is_the_time_since_the_start_k10():
    _, record = wait_for("clock_elapsed >= 90 s", after="measure r time as v;\nmove r to b;")
    assert readings(record)[0]["text"] == "21:31:30"
    _, record = wait_for("clock_elapsed > 2 min", after="measure r elapsed as v;\nmove r to b;")
    assert readings(record)[0]["value"] == "120.001"
    _, record = simulate(task('wait for 30 s;\nwait until clock_elapsed >= 90 s timeout 5 min else { fail "x"; }\n'
                              'measure r elapsed as v;\nmove r to b;'), None)
    assert readings(record)[0]["value"] == "90"


def test_a_wait_for_a_time_compares_with_a_time_binding():
    plan = ('measure r time as start;\nwait for 90 s;\n'
            'wait until clock_time > start timeout 5 min else { fail "x"; }\nmeasure r elapsed as v;\nmove r to b;')
    _, record = simulate(task(plan))
    assert record["state"] == "SUCCEEDED" and readings(record)[-1]["value"] == "90"
    plan = ('measure r time as start;\nwait for 90 s;\nmeasure r time as later;\n'
            'wait until clock_time >= start timeout 5 min else { fail "x"; }\nmove r to b;')
    assert simulate(task(plan))[1]["state"] == "SUCCEEDED"


def test_a_wait_for_a_time_with_an_event_also_ends_on_the_event():
    def patch(world):
        world["events"] = [{"at": {"value": "20", "unit": "s"}, "fact": "robot_ready", "target": "r2", "value": True}]
        world["robots"]["r2"]["ready"] = False
    plan = ('wait until clock_time >= 22:00 or robot_ready s timeout 90 min else { fail "late"; }\n'
            'measure r elapsed as v;\nmove r to b;')
    _, record = simulate(task(plan), patch=patch)
    assert record["state"] == "SUCCEEDED" and readings(record)[0]["value"] == "20"


def test_the_wait_ends_with_the_end_of_the_day_s238_not_a_wrapped_time():
    plan = 'wait until clock_time < 05:00 timeout 12 min else { fail "x"; }\nmove r to b;'
    _, record = simulate(task(plan), "2026-11-17T23:55:00")
    assert record["steps"][0]["handled"] == "S238" and record["end_t_ms"] == "300000"


def test_a_pending_wait_for_a_time_two_waits_in_a_row_use_the_same_clock():
    plan = ('wait until clock_time >= 21:35 timeout 10 min else { fail "x"; }\n'
            'wait until clock_time >= 21:40 timeout 10 min else { fail "y"; }\nmeasure r elapsed as v;\nmove r to b;')
    _, record = simulate(task(plan))
    assert record["state"] == "SUCCEEDED" and readings(record)[0]["value"] == "600"


# -------- K3: a time that has already passed fails before the run --------

PASSED = 'wait until clock_time >= 09:00 timeout 90 min else { fail "late"; }\nmove r to b;'


def test_a_wait_for_a_time_that_has_passed_fails_before_authorized_with_s010_k3():
    result, record = simulate(task(PASSED), "2026-11-17T10:30:00")
    assert result.states == ("CREATED", "VALIDATED", "FAILED") and failure(record) == ("S010", 1)
    assert record["failure"]["phase"] == "authorization" and all(s["status"] == "not_run" for s in record["steps"])
    assert record["end_t_ms"] == "0"


def test_a_time_that_has_not_passed_or_is_the_start_does_not_fail():
    assert simulate(task(PASSED), "2026-11-17T08:00:00")[1]["state"] == "SUCCEEDED"
    assert simulate(task(PASSED), "2026-11-17T09:00:00")[1]["state"] == "SUCCEEDED"
    assert failure(simulate(task(PASSED), "2026-11-17T09:00:01")[1]) == ("S010", 1)


def test_greater_than_a_passed_time_fails_too_and_so_does_a_member_of_an_and():
    plan = 'wait until clock_time > 09:00 timeout 90 min else { fail "late"; }\nmove r to b;'
    assert failure(simulate(task(plan), "2026-11-17T10:30:00")[1]) == ("S010", 1)
    plan = 'wait until clock_time >= 09:00 and robot_ready r timeout 90 min else { fail "late"; }\nmove r to b;'
    assert failure(simulate(task(plan), "2026-11-17T10:30:00")[1]) == ("S010", 1)


def test_only_a_wait_that_needs_the_time_is_checked_not_or_not_less_than_or_a_binding():
    for condition in ("clock_time >= 09:00 or robot_ready r", "not (clock_time >= 09:00)", "clock_time < 09:00",
                      "clock_time <= 09:00", "clock_elapsed >= 5 s"):
        plan = f'wait until {condition} timeout 90 min else {{ fail "late"; }}\nmove r to b;'
        assert failure(simulate(task(plan), "2026-11-17T10:30:00")[1]) != ("S010", 1), condition
    plan = ('measure r time as early;\nwait for 600 s;\nwait until clock_time >= early timeout 5 min else '
            '{ fail "late"; }\nmove r to b;')
    assert simulate(task(plan), "2026-11-17T10:30:00")[1]["state"] == "SUCCEEDED"


def test_with_no_clock_the_wait_is_not_s010_it_fails_the_step_s237():
    _, record = simulate(task(PASSED), start=None)
    assert failure(record) == ("S262", 2) and record["steps"][0]["handled"] == "S237"


def test_if_it_is_already_after_nine_go_now_is_written_with_measure_and_if_and_runs():
    plan = 'measure r time as now;\nif now >= 09:00 { move r to b; } else { wait until clock_time >= 09:00 ' \
           'timeout 90 min else { fail "late"; }\nmove r to b; }'
    assert simulate(task(plan, goal="goal { robot_at r b; }"), "2026-11-17T10:30:00")[1]["state"] == "SUCCEEDED"


def test_s010_checks_the_plans_own_waits_and_validate_world_lists_every_one(tmp_path):
    source = task('wait until clock_time >= 08:00 timeout 90 min else { fail "a"; }\n'
                  'measure r time as now;\nif now >= 09:00 { move r to b; } else { wait until clock_time >= 07:00 '
                  'timeout 90 min else { fail "b"; } }\n'  # a branch may not run: not checked
                  'wait until clock_time > 10:00 timeout 90 min else { fail "c"; }\nmove r to b;')
    (tmp_path / "t.rob").write_text(source, encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data(start="2026-11-17T10:30:00"))
    code, out, err = run_cli(["validate", "t.rob", "--world", "w.json", "--bind", "r=r1", "--bind", "s=r2",
                              "--bind", "box=box1", "--bind", "a=a", "--bind", "b=b"], tmp_path)
    assert code == 1 and out == ""
    found = re.findall(r"t\.rob:(\d+):\d+: error\[(S\d{3})\]", err)
    assert [(int(line), c) for line, c in found] == [(6, "S010"), (9, "S010")]
    assert "has already passed" in err and "'wait until clock_time >= 08:00:00'" in err and "(on path" not in err


def test_validate_world_passes_a_task_whose_times_have_not_passed(tmp_path):
    (tmp_path / "t.rob").write_text(task(PASSED), encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data(start="2026-11-17T08:00:00"))
    args = ["validate", "t.rob", "--world", "w.json", "--bind", "r=r1", "--bind", "s=r2", "--bind", "box=box1",
            "--bind", "a=a", "--bind", "b=b"]
    code, out, err = run_cli(args, tmp_path)
    assert code == 0 and "no certain failure found" in (out + err)


def test_validate_world_for_an_ir_file_points_at_the_step(tmp_path):
    (tmp_path / "t.json").write_text(to_json(compile_source(task(PASSED)).ir))
    (tmp_path / "w.json").write_bytes(world_data(start="2026-11-17T10:30:00"))
    code, out, err = run_cli(["validate", "--ir", "t.json", "--world", "w.json", "--bind", "r=r1", "--bind", "s=r2",
                              "--bind", "box=box1", "--bind", "a=a", "--bind", "b=b"], tmp_path)
    assert code == 1 and "/tasks/0/steps/0" in err and "S010" in err


# -------- CLI --------

def test_simulate_prints_the_failure_meaning_and_inspect_shows_the_syntax(tmp_path):
    (tmp_path / "t.rob").write_text(task(PASSED), encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data(start="2026-11-17T10:30:00", bindings={
        "r": "r1", "s": "r2", "box": "box1", "a": "a", "b": "b"}))
    code, out, err = run_cli(["simulate", "t.rob", "--world", "w.json"], tmp_path)
    assert code == 4 and "S010" in err and "already passed" in err
    (tmp_path / "n.rob").write_text(task("measure r time as t;\nmove r to b;"), encoding="utf-8")
    (tmp_path / "n.json").write_bytes(world_data(start=None))
    code, out, err = run_cli(["simulate", "n.rob", "--world", "n.json"], tmp_path)
    assert code == 4 and "S237" in err and "needs the world's clock" in err
    (tmp_path / "i.rob").write_text(STEPS, encoding="utf-8")
    code, out, err = run_cli(["inspect", "i.rob"], tmp_path)
    for shown in ("measure r time as now", "measure r elapsed as took", "now < 21:30:00",
                  "clock_time >= 22:15:30", "clock_elapsed > 90 s"):
        assert shown in out, shown


def test_validate_reports_the_time_literal_error(tmp_path):
    (tmp_path / "t.rob").write_text(task('measure r time as t;\nif t < 24:00 { move r to b; } else { fail "x"; }'),
                                    encoding="utf-8")
    code, out, err = run_cli(["validate", "t.rob"], tmp_path)
    assert code == 1 and "error[E105]" in err and "24:00" in err


# -------- refusals: delegate and ROS 2 (T9) --------

def test_delegate_does_not_delegate_a_plan_that_reads_the_clock():
    from robolang import delegate
    plan = compile_source(task("measure r elapsed as d;\nmove r to b;")).ir.tasks[0]
    assert delegate._consider(None, plan, b"", "w.json", "r1", (), "0.1") == ("measure_unsupported", None)
    nested = compile_source(task('measure r time as t;\nif t < 22:00 { move r to b; } else { fail "x"; }')).ir.tasks[0]
    assert delegate._consider(None, nested, b"", "w.json", "r1", (), "0.1") == ("measure_unsupported", None)


def test_the_ros2_backend_refuses_a_clock_reading_with_s510_before_any_motion():
    pytest.importorskip("jsonschema")
    sys.path.insert(0, str(ROOT / "backends" / "ros2"))
    from robolang_ros2.executor import execute
    from test_ros2_executor import FakeOps, fixture
    _, bindings, site = fixture()
    for body in ("measure r time as t; move r to dst;", "measure r elapsed as t; move r to dst;"):
        source = ("robolang 1.9;\ntask transfer(robot r, object box, location src, location dst) {\n"
                  f"  goal {{ robot_at r dst; }}\n  plan {{ {body} }}\n}}\n")
        ops = FakeOps()
        record = execute(compile_source(source).ir.tasks[0], dict(bindings), site, ops)
        assert record["failure"]["code"] == "S510" and "wall clock" in record["failure"]["message"]
        assert ops.calls == [("guard", "ur")]


# -------- Python 3.11 and the economy --------

def test_no_dataclass_of_the_clock_has_a_mutable_default_python_3_11():
    """PR #151 erratum E4: Python 3.11 rejects an unhashable dataclass default when the module is imported."""
    import dataclasses
    from robolang import nodes
    from robolang.sim import clock, executor, world
    for module in (clock, executor, world, ir, nodes):
        for item in vars(module).values():
            if dataclasses.is_dataclass(item) and isinstance(item, type):
                for f in dataclasses.fields(item):
                    if f.default is not dataclasses.MISSING:
                        hash(f.default)  # a mappingproxy or a list default raises TypeError here


def test_the_economy_of_1_9_decision_0042():
    """0 statements, 2 kinds, 1 literal, 2 reserved words, 1 world field (bench-8 B7)."""
    from robolang.tokens import CLOCK_KINDS, RESERVED_BY_VERSION, STATEMENTS_V1_8
    assert CLOCK_KINDS == ("time", "elapsed")
    assert STATEMENTS_V1_8 == ("move", "grasp", "place", "inspect", "wait", "communicate", "measure", "do")
    assert RESERVED_BY_VERSION["1.9"] - RESERVED_BY_VERSION["1.8"] == {"clock_time", "clock_elapsed"}
    world = load_world(world_data(), "w.json", plan_of())
    assert set(BASE_WORLD) - {"world_schema", "world_schema_version", "locations", "robots", "objects", "tasks"} == {
        "clock"} and world.clock is not None
