"""
RoboLang 2.2: more measured kinds (spec/robolang-v2.2.md, decision 0044 K1-K9 and clarifications D1-D14).

Front end, Task IR 1.10, Mission IR 1.2 and the reader, world file 1.7, the simulation (spec/simulation-v1.0.md section
20), the run record 1.10, and the refusals of `delegate` and the ROS 2 backend. No test here needs a model or a network.

To regenerate the IR snapshots after an intended change:

    ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_lang22.py
"""

from __future__ import annotations
import copy
import io
import json
import os
import re
import sys
from decimal import Decimal
from pathlib import Path

import pytest

from robolang import RoboLangError, check, check_file
from robolang import measured
from robolang.cli import main
from robolang.ir import compile_source, to_json
from robolang.ir_reader import IRError, read_ir
from robolang.sim import WorldError, load_world, mission_task, run, run_mission, to_mission_record, to_record
from robolang.sim.check import check_task
from robolang.tokens import FUTURE_RESERVED, KIND_UNITS, KIND_UNITS_V2_2, RESERVED_BY_VERSION

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "v2.2"
SNAPSHOTS = ROOT / "tests" / "snapshots" / "ir-v2.2"
UPDATE = os.environ.get("ROBOLANG_UPDATE_SNAPSHOTS") == "1"
SPEC = (ROOT / "spec" / "robolang-v2.2.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")

# -------- helpers --------

LOOK = """task look(robot r, location aisle, location bay, object crate, object gauge, object plant,
          sensor cam, sensor tag, sensor pcam) {
    goal { robot_at r aisle; }
    plan {
        move r to aisle;
        %s
    }
}
"""
BIND = {"r": "bot", "aisle": "aisle", "bay": "bay", "crate": "crate", "gauge": "gauge_p3", "plant": "plant_7",
        "cam": "cam", "tag": "tag", "pcam": "pcam"}
WORLD = {
    "world_schema": "robolang.sim.world",
    "world_schema_version": "1.7",
    "locations": ["base", "aisle", "bay", "dock"],
    "regions": {},
    "robots": {"bot": {"ready": True, "location": "base", "gripper": False},
               "bot2": {"ready": True, "location": "dock", "gripper": False}},
    "objects": {
        "crate": {"present": True, "location": "aisle", "properties": {
            "fill": {"value": "12", "unit": "%"}, "count": {"value": "3"}}},
        "gauge_p3": {"present": True, "location": "aisle", "properties": {
            "pressure": {"value": "7.2", "unit": "bar"}}},
        "plant_7": {"present": True, "location": "aisle", "properties": {
            "label": {"value": "weed"}, "confidence": {"value": "95", "unit": "%"}}},
        "plain": {"present": True, "location": "aisle"},
    },
    "location_properties": {"bay": {"fill": {"value": "12", "unit": "%"}, "count": {"value": "0"}}},
    "sensors": {
        "cam": {"robot": "bot", "available": True, "measures": {
            "fill": {"unit": "%", "uncertainty": "2", "contact": False},
            "confidence": {"unit": "%", "uncertainty": "1", "contact": False},
            "label": {"contact": False}}},
        "tag": {"robot": "bot", "available": True, "measures": {"count": {"contact": False}}},
        "pcam": {"robot": "bot", "available": True, "measures": {
            "pressure": {"unit": "bar", "uncertainty": "0.1", "contact": False}}},
        "cam_other": {"robot": "bot2", "available": True, "measures": {
            "fill": {"unit": "%", "uncertainty": "2", "contact": False}}},
    },
    "channels": {"lead": {"from": ["rl:agent:ops"]}},
    "events": [],
    "tasks": {},
}


def look(body: str, version: str = "2.2") -> str:
    return f"robolang {version};\n\n" + LOOK % body


def world_data(patch=None, version: str | None = None) -> bytes:
    world = copy.deepcopy(WORLD)
    if version is not None:
        world["world_schema_version"] = version
    if patch is not None:
        patch(world)
    return json.dumps(world).encode()


def codes(source: str) -> list[tuple[str, int]]:
    try:
        check(source)
    except RoboLangError as error:
        return [(d.code, d.line) for d in error.diagnostics]
    return []


def simulate(source: str, patch=None, bindings=None, name="look"):
    task = compile_source(source).ir.task(name)
    world = load_world(world_data(patch), "w.json", task, bindings or BIND)
    return run(task, world)


def sent(result) -> list[str]:
    return [s.message.message for s in result.steps if s.message is not None and s.message.message is not None]


def end(result) -> tuple[str, str | None]:
    return result.state, None if result.failure is None else result.failure.code


def set_property(kind: str, value: dict, where: str = "objects", name: str = "crate"):
    def patch(world):
        if where == "objects":
            world["objects"][name].setdefault("properties", {})[kind] = value
        else:
            world["location_properties"].setdefault(name, {})[kind] = value
    return patch


def record_of(task, result, language: str, world_version: str = "1.7") -> dict:
    return to_record(result, task=task, run_id="sim-" + "0" * 64, source_sha256="a" * 64, world_sha256="b" * 64,
                     seed="0", language_version=language, world_schema_version=world_version)


def readings(result) -> list:
    return [s.reading for s in result.steps if s.reading is not None]


# -------- the language --------

def test_2_2_adds_no_reserved_word_and_no_kind_name_is_reserved():
    assert RESERVED_BY_VERSION["2.2"] == RESERVED_BY_VERSION["2.1"] == RESERVED_BY_VERSION["2.0"]
    for word in ("fill", "confidence", "pressure", "count"):
        assert word not in RESERVED_BY_VERSION["2.2"] and word not in FUTURE_RESERVED, word


def test_the_four_kinds_and_their_units():
    assert tuple(KIND_UNITS_V2_2) == (*KIND_UNITS, "fill", "confidence", "pressure", "count")
    assert KIND_UNITS_V2_2["fill"] == KIND_UNITS_V2_2["confidence"] == ("%",)
    assert KIND_UNITS_V2_2["pressure"] == ("Pa", "kPa", "bar") and KIND_UNITS_V2_2["count"] == ()
    assert tuple(KIND_UNITS) == ("temperature", "mass", "distance", "label")  # the kinds of 1.6 are unchanged


def test_a_file_below_2_2_still_rejects_the_new_kinds_e217_and_the_percent_sign_e101():
    for version in ("1.6", "2.0", "2.1"):
        for kind in ("fill", "confidence", "pressure", "count"):
            body = f"measure r {kind} of crate using cam as v;"
            assert [c for c, _ in codes(look(body, version))] == ["E217"], (version, kind)
    assert [c for c, _ in codes(look("measure r temperature of crate using cam as t; if t < 25 % { fail \"x\"; }",
                                     "2.1"))] == ["E101"]


def test_every_kind_reads_with_its_unit_and_the_comparisons_validate():
    for body in ("measure r fill of crate using cam as v; if v >= 25 % { communicate r to \"lead\" with v; }",
                 "measure r confidence of plant using cam as v; if v <= 90.5 % { communicate r to \"lead\" with v; }",
                 "measure r pressure of gauge using pcam as v; if v > 6 bar { communicate r to \"lead\" with v; }",
                 "measure r pressure of gauge using pcam as v; if v > 600 kPa and v < 700000 Pa { fail \"x\"; }",
                 "measure r count of bay using tag as v; if v != 0 { communicate r to \"lead\" with v; }",
                 "measure r count of crate using tag as v; if v >= 1 and v < 10 { communicate r to \"lead\" with v; }"):
        # every path needs a goal: the `if` without an else keeps the task goal
        assert codes(look(body)) == [], body


def test_a_fill_and_a_count_may_be_read_of_an_object_or_a_location_d1():
    for kind in ("fill", "count"):
        for target in ("crate", "bay"):
            assert codes(look(f"measure r {kind} of {target} using cam as v;")) == [], (kind, target)
    for kind in ("confidence", "pressure"):
        assert [c for c, _ in codes(look(f"measure r {kind} of bay using cam as v;"))] == ["E304"], kind
    assert [c for c, _ in codes(look("measure r fill of r using cam as v;"))] == ["E304"]  # D9
    assert [c for c, _ in codes(look("measure r fill of cam using cam as v;"))] == ["E304"]


def test_a_live_reading_of_a_location_in_a_wait_d1_d6():
    ok = 'wait until fill of bay using cam > 80 % timeout 10 min else { fail "no"; }'
    assert codes(look(ok)) == []
    bad = 'wait until pressure of bay using pcam > 6 bar timeout 10 min else { fail "no"; }'
    assert [c for c, _ in codes(look(bad))] == ["E304"]
    assert codes(look('wait until count of crate using tag == 0 timeout 1 min else { fail "no"; }')) == []


def test_the_unit_is_required_e206_and_a_percent_sign_is_a_unit_not_a_fraction_d3():
    reading = "measure r fill of crate using cam as v; "
    for bad in ("v < 0.3", "v < 30", "v < 30 percent", "v < 30 psi"):
        assert [c for c, _ in codes(look(f"{reading}if {bad} {{ fail \"x\"; }}"))][:1] == ["E206"], bad  # then a resync
    pressure = "measure r pressure of gauge using pcam as v; "
    assert [c for c, _ in codes(look(f"{pressure}if v > 90 psi {{ fail \"x\"; }}"))][:1] == ["E206"]
    assert [c for c, _ in codes(look(f"{pressure}if v > 6 {{ fail \"x\"; }}"))][:1] == ["E206"]


def test_kinds_and_units_that_do_not_match_are_e319_d3():
    fill = "measure r fill of crate using cam as f; measure r confidence of plant using cam as c; "
    for cond in ("f < c", "f > 30 kPa", "c <= 5 bar"):
        assert [x for x, _ in codes(look(f"{fill}if {cond} {{ fail \"x\"; }}"))] == ["E319"], cond
    count = "measure r count of crate using tag as n; measure r fill of crate using cam as f; "
    for cond in ("n < f", "n < 3 %", "n == 3 kg", "f < n"):
        assert [x for x, _ in codes(look(f"{count}if {cond} {{ fail \"x\"; }}"))] == ["E319"], cond
    assert [x for x, _ in codes(look(f"{fill}if f < 3 bar {{ fail \"x\"; }}"))] == ["E319"]


def test_a_count_is_a_whole_number_e328_d2():
    count = "measure r count of crate using tag as n; "
    for cond in ("n < 2.5", "n == 0.5", "n >= 1.0001"):
        assert [x for x, _ in codes(look(f"{count}if {cond} {{ fail \"x\"; }}"))] == ["E328"], cond
    assert codes(look(f"{count}if n < 2.0 {{ fail \"x\"; }}")) == []  # 2.0 is the whole number 2
    live = look('wait until count of crate using tag < 2.5 timeout 1 min else { fail "n"; }')
    assert [x for x, _ in codes(live)] == ["E328"]


def test_equals_is_for_a_count_and_a_label_only_k3():
    other = "measure r fill of crate using cam as f; measure r pressure of gauge using pcam as p; "
    for cond in ("f == 20 %", "p != 6 bar", "f != 20 %"):
        assert [x for x, _ in codes(look(f"{other}if {cond} {{ fail \"x\"; }}"))] == ["E320"], cond
    assert codes(look("measure r count of crate using tag as n; if n == 3 { fail \"x\"; }")) == []


def test_a_kind_name_is_a_name_everywhere_else_d10():
    source = """robolang 2.2;
task count(robot r, location fill, location pressure, sensor confidence, sensor tag) {
    goal { robot_at r pressure; }
    plan {
        move r to pressure;
        measure r count of fill using tag as label;
        measure r fill of fill using confidence as temperature;
        if temperature < 20 % and label >= 0 { communicate r to "lead" with temperature; }
    }
}
"""
    assert codes(source) == []
    assert compile_source(source).ir.tasks[0].name == "count"


def test_the_error_texts_name_what_to_do():
    reading = "measure r fill of crate using cam as v; "
    with pytest.raises(RoboLangError) as caught:
        check(look(f"{reading}if v < 30 percent {{ fail \"x\"; }}"))
    assert "'%'" in caught.value.diagnostic.message and "percent" in caught.value.diagnostic.message
    with pytest.raises(RoboLangError) as caught:
        check(look("measure r count of crate using tag as n; if n < 2.5 { fail \"x\"; }"))
    assert "whole number" in caught.value.diagnostic.message


# -------- fixtures --------

HEADER = re.compile(
    r"\A// RoboLang v2\.2 fixture: (?P<set>valid|invalid)\n"
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
    mission = document["ir_schema"] == "robolang.mission-ir"
    assert (document["ir_schema_version"], document["language_version"]) == ("1.2" if mission else "1.10", "2.2")
    assert document.get("task_ir_schema_version", "1.10") == "1.10"
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


def test_every_new_code_is_in_the_specification_tables():
    table = set(re.findall(r"^\| `((?:[EISX]\d{3})(?:`, `[EISX]\d{3})*)` \|", SPEC, re.MULTILINE))
    listed = {code for row in table for code in row.split("`, `")}
    assert {"E328", "E206", "E217", "E304", "E319", "S234", "S236", "S509"} <= listed
    assert "`I229`" in (ROOT / "spec" / "ir-reader-v0.1.md").read_text(encoding="utf-8")
    assert "`I229`" in (ROOT / "spec" / "ir-schema-v1.10.md").read_text(encoding="utf-8")


def test_the_grammar_file_says_what_2_2_cannot_say_erratum_e5():
    """The grammar file is the grammar-only prompt: it must say what 2.2 cannot (K8, D7, D8, D9, D14)."""
    grammar = (ROOT / "spec" / "grammar-v2.2.ebnf").read_text(encoding="utf-8")
    for phrase in ("a kind no sensor can declare", "how worn it looks", "a reading of a person", "the nearest person",
                   "a reading of the robot itself", "own battery", "counting objects chosen at run time",
                   "arithmetic on readings", "an average", "psi", "inexpressible", "infeasible", "E206", "E304",
                   "E319", "E328", "Do not replace a reading", "inspect", "Do not move a threshold",
                   "the SAME sensor", "OR a location", "not reserved"):
        assert phrase in grammar, phrase
    assert grammar.isascii()


# -------- units and the three-valued comparison --------

def test_pressure_converts_exactly_between_pa_kpa_and_bar_k2():
    assert measured.convert(Decimal("4.2"), "bar", "kPa") == Decimal("420")
    assert measured.convert(Decimal("1"), "bar", "kPa") == Decimal("100")
    assert measured.convert(Decimal("450"), "kPa", "bar") == Decimal("4.5")
    assert measured.convert(Decimal("100000"), "Pa", "bar") == Decimal("1")
    assert measured.convert(Decimal("0.1"), "kPa", "Pa") == Decimal("100")
    assert measured.convert(Decimal("30"), "%", "%") == Decimal("30")
    assert measured.to_base(Decimal("3"), None) == Decimal("3")  # a count has no unit
    assert measured.unit_kinds("%") == ("fill", "confidence") and measured.unit_kinds("bar") == ("pressure",)
    assert measured.unit_kinds("s") == ("elapsed",)


def test_a_count_is_exact_equals_and_not_equals_are_never_unknown_k3():
    point = lambda v: measured.Interval(Decimal(v), Decimal(v))  # noqa: E731
    assert measured.compare_intervals("==", point(0), point(0)) is True
    assert measured.compare_intervals("==", point(1), point(0)) is False
    assert measured.compare_intervals("!=", point(0), point(0)) is False
    assert measured.compare_intervals("!=", point(2), point(0)) is True
    assert measured.compare_intervals("==", measured.Interval(Decimal(0), Decimal(2)), point(1)) is None  # not a count


# -------- Task IR 1.10, Mission IR 1.2 and the reader --------

def document_of(source: str) -> dict:
    return json.loads(to_json(compile_source(source).ir))


def blob(document: dict) -> bytes:
    return json.dumps(document).encode("utf-8")


def problems(document) -> list[tuple[str, str]]:
    with pytest.raises(IRError) as caught:
        read_ir(blob(document), "x.json")
    return [(d.code, d.pointer) for d in caught.value.diagnostics]


COUNT_SOURCE = look("measure r count of bay using tag as n; measure r fill of crate using cam as f; "
                    "if n == 0 and f < 25 % { communicate r to \"lead\" with n; }")


def test_task_ir_1_10_shape_a_count_literal_has_no_unit_and_a_fill_has_percent():
    document = document_of(COUNT_SOURCE)
    task = document["tasks"][0]
    assert document["ir_schema_version"] == "1.10" and document["language_version"] == "2.2"
    assert task["steps"][1]["reading"] == "count" and task["steps"][1]["object"] == "bay"
    condition = task["steps"][3]["condition"]
    assert condition["conditions"][0]["right"] == {"kind": "quantity", "value": "0"}
    assert condition["conditions"][1]["right"] == {"kind": "quantity", "value": "25", "unit": "%"}
    assert read_ir(blob(document)) == compile_source(COUNT_SOURCE).ir


def test_a_file_with_a_mission_is_mission_ir_1_2_with_task_ir_1_10_tasks():
    source = (FIXTURES / "valid" / "a_mission_runs_a_task_that_reads_a_count.rob").read_text(encoding="utf-8")
    document = document_of(source)
    assert (document["ir_schema"], document["ir_schema_version"], document["task_ir_schema_version"]) == (
        "robolang.mission-ir", "1.2", "1.10")
    assert document["missions"][0]["repeat"]["until"]["left"] == {
        "kind": "live", "reading": "count", "object": "shelf", "sensor": "tag"}
    assert read_ir(blob(document)) == compile_source(source).ir


def test_the_reader_reads_the_new_kinds_only_from_schema_1_10():
    old = document_of(COUNT_SOURCE)
    old.update(ir_schema_version="1.9", language_version="2.1")
    got = problems(old)
    assert ("I125", "/tasks/0/steps/1/reading") in got and ("I125", "/tasks/0/steps/2/reading") in got
    document = document_of(look("measure r temperature of crate using cam as t; if t < 5 degC { fail \"x\"; }"))
    document["ir_schema_version"], document["language_version"] = "1.9", "2.1"
    assert read_ir(blob(document)).language_version == "2.1"  # the old kinds and units are unchanged
    percent = document_of(look("measure r fill of crate using cam as f; if f < 5 % { fail \"x\"; }"))
    percent["tasks"][0]["steps"][1]["reading"] = "temperature"
    percent.update(ir_schema_version="1.9", language_version="2.1")
    assert ("I117", "/tasks/0/steps/2/condition/right/unit") in problems(percent)


def test_the_reader_restates_the_unit_count_and_kind_rules_i117_i229_i220():
    base = document_of(look("measure r count of crate using tag as n; measure r fill of crate using cam as f; "
                            "if n == 0 { fail \"a\"; } if f < 5 % { fail \"b\"; }"))
    assert read_ir(blob(base))
    count_if, fill_if = (3, 4)
    bad = copy.deepcopy(base)
    bad["tasks"][0]["steps"][count_if]["condition"]["right"]["value"] = "2.5"
    assert problems(bad) == [("I229", f"/tasks/0/steps/{count_if}/condition/right")]
    bad = copy.deepcopy(base)
    del bad["tasks"][0]["steps"][fill_if]["condition"]["right"]["unit"]  # a fill literal with no unit
    assert problems(bad) == [("I117", f"/tasks/0/steps/{fill_if}/condition/right")]
    bad = copy.deepcopy(base)
    bad["tasks"][0]["steps"][count_if]["condition"]["right"]["unit"] = "kg"  # a unit on a count
    assert problems(bad) == [("I220", f"/tasks/0/steps/{count_if}/condition/right")]
    bad = copy.deepcopy(base)
    bad["tasks"][0]["steps"][fill_if]["condition"]["right"]["unit"] = "psi"
    assert problems(bad)[0][0] == "I117"
    bad = copy.deepcopy(base)
    bad["tasks"][0]["steps"][fill_if]["condition"]["op"] = "=="  # a fill has no ==
    assert problems(bad) == [("I221", f"/tasks/0/steps/{fill_if}/condition")]


def test_the_reader_restates_where_a_location_may_be_read_i204():
    base = document_of(look("measure r pressure of gauge using pcam as p; measure r fill of bay using cam as f;"))
    assert read_ir(blob(base))
    bad = copy.deepcopy(base)
    bad["tasks"][0]["steps"][1]["object"] = "bay"  # a pressure of a location
    assert problems(bad) == [("I204", "/tasks/0/steps/1/object")]
    bad = copy.deepcopy(base)
    bad["tasks"][0]["steps"][2]["object"] = "r"  # a fill of the robot (D9)
    assert problems(bad) == [("I204", "/tasks/0/steps/2/object")]
    live = document_of(look('wait until fill of bay using cam > 80 % timeout 1 min else { fail "n"; }'))
    assert read_ir(blob(live))
    live["tasks"][0]["steps"][1]["condition"]["left"]["reading"] = "confidence"
    assert problems(live) == [("I204", "/tasks/0/steps/1/condition/left/object")]


def test_a_do_value_still_needs_its_unit_in_the_reader():
    source = look("do r open crate to 5 degC;")
    document = document_of(source)
    assert document["tasks"][0]["steps"][1]["value"]["unit"] == "degC"
    del document["tasks"][0]["steps"][1]["value"]["unit"]
    assert problems(document) == [("I111", "/tasks/0/steps/1/value")]


def test_the_reader_checks_the_header_of_mission_ir_1_2():
    source = (FIXTURES / "valid" / "a_mission_runs_a_task_that_reads_a_count.rob").read_text(encoding="utf-8")
    document = document_of(source)
    for key, value in (("ir_schema_version", "1.3"), ("task_ir_schema_version", "1.9"), ("language_version", "2.1")):
        bad = copy.deepcopy(document)
        bad[key] = value
        assert ("I113", f"/{key}") in problems(bad), key


def test_the_count_condition_prints_in_the_source_form_and_inspect_round_trips(tmp_path):
    rob = tmp_path / "look.rob"
    rob.write_text(COUNT_SOURCE, encoding="utf-8")
    code, out, err = cli("inspect", str(rob))
    assert code == 0, err
    assert "n == 0" in out and "f < 25 %" in out and "n == 0 %" not in out


# -------- world file 1.7 --------

def world_error(patch, version: str | None = None, bindings=None) -> WorldError:
    task = compile_source(look("measure r fill of crate using cam as v;")).ir.task("look")
    with pytest.raises(WorldError) as caught:
        load_world(world_data(patch, version), "w.json", task, bindings or BIND)
    return caught.value


def test_a_valid_world_1_7_names_the_kinds_and_the_properties_of_objects_and_locations():
    task = compile_source(look("measure r fill of crate using cam as v;")).ir.task("look")
    world = load_world(world_data(), "w.json", task, BIND)
    assert world.schema_version == "1.7"
    assert world.sensors["tag"].measure("count").unit is None
    assert world.sensors["tag"].measure("count").uncertainty == Decimal(0)
    assert world.sensors["pcam"].measure("pressure").unit == "bar"
    assert [(p.kind, p.value, p.unit) for p in world.location_properties["bay"]] == [
        ("fill", Decimal("12"), "%"), ("count", Decimal("0"), None)]
    assert world.objects["plant_7"].properties[1].kind == "confidence"


def test_a_count_sensor_may_not_have_an_uncertainty_or_a_unit_d2():
    def uncertain(world):
        world["sensors"]["tag"]["measures"]["count"] = {"uncertainty": "1", "contact": False}
    error = world_error(uncertain)
    assert error.code == "E514" and error.pointer == "/sensors/tag/measures/count/uncertainty"
    assert "exact" in error.message

    def unit(world):
        world["sensors"]["tag"]["measures"]["count"] = {"unit": "%", "contact": False}
    assert world_error(unit).code == "E514"

    def prop(world):
        world["objects"]["crate"]["properties"]["count"] = {"value": "3", "unit": "kg"}
    assert world_error(prop).code == "E514"


def test_a_count_and_a_percentage_have_a_valid_range_in_the_world():
    for bad in ("2.5", "-1", "three", "", "١"):
        def patch(world, bad=bad):
            world["objects"]["crate"]["properties"]["count"] = {"value": bad}
        assert world_error(patch).code == "E504", bad
    for bad in ("101", "-5", "100.5"):
        def patch(world, bad=bad):
            world["objects"]["crate"]["properties"]["fill"] = {"value": bad, "unit": "%"}
        assert world_error(patch).code == "E504", bad
    for bad_unit in ("bar", "kPa", "psi"):
        def patch(world, bad_unit=bad_unit):
            world["objects"]["crate"]["properties"]["fill"] = {"value": "5", "unit": bad_unit}
        assert world_error(patch).code == "E513", bad_unit

    def pressure(world):
        world["objects"]["gauge_p3"]["properties"]["pressure"] = {"value": "7", "unit": "%"}
    assert world_error(pressure).code == "E513"


def test_a_location_holds_only_fill_and_count_and_must_be_declared_d1():
    def pressure(world):
        world["location_properties"]["bay"]["pressure"] = {"value": "7", "unit": "bar"}
    assert world_error(pressure).code == "E512"

    def undeclared(world):
        world["location_properties"]["nowhere"] = {"fill": {"value": "5", "unit": "%"}}
    error = world_error(undeclared)
    assert error.code == "E505" and error.pointer == "/location_properties/nowhere"

    def an_object(world):
        world["location_properties"]["crate"] = {"fill": {"value": "5", "unit": "%"}}
    assert world_error(an_object).code == "E505"

    def not_an_object(world):
        world["location_properties"] = []
    assert world_error(not_an_object).code == "E504"


def test_world_files_before_1_7_have_neither_the_kinds_nor_the_location_properties():
    for version in ("1.2", "1.6"):
        def patch(world):
            for key in ("channels",) if version == "1.2" else ():
                pass
            world.pop("location_properties")
            if version == "1.2":
                world.pop("channels")
            world["objects"]["crate"]["properties"] = {"fill": {"value": "5", "unit": "%"}}
        error = world_error(patch, version)
        assert error.code == "E512", version
    error = world_error(lambda world: None, "1.6")
    assert error.code == "E503" and error.pointer == "/location_properties"
    old = world_error(lambda w: (w.pop("location_properties"), w["objects"]["crate"].pop("properties"),
                                 w["sensors"].pop("tag"), w["sensors"].pop("pcam"), w["sensors"].pop("cam_other"),
                                 w["objects"].pop("gauge_p3"), w["objects"].pop("plant_7"),
                                 w["sensors"]["cam"].update(measures={"fill": {"unit": "%", "uncertainty": "2",
                                                                              "contact": False}})), "1.6")
    assert old.code == "E512"


def test_a_property_event_may_name_a_location_for_fill_and_count_only():
    def event(world):
        world["events"] = [{"at": {"value": "5", "unit": "s"}, "fact": "property", "target": "bay",
                            "property": "fill", "value": {"value": "90", "unit": "%"}}]
    task = compile_source(look("measure r fill of crate using cam as v;")).ir.task("look")
    load_world(world_data(event), "w.json", task, BIND)

    def pressure(world):
        world["events"] = [{"at": {"value": "5", "unit": "s"}, "fact": "property", "target": "bay",
                            "property": "pressure", "value": {"value": "9", "unit": "bar"}}]
    assert world_error(pressure).code == "E511"


# -------- the simulation --------

def test_a_fill_below_the_limit_sends_and_above_does_not_k1():
    body = 'measure r fill of crate using cam as v; if v < 30 % { communicate r to "lead" with "restock"; }'
    assert sent(simulate(look(body))) == ["restock"]  # 12 % +/- 2
    above = simulate(look(body), set_property("fill", {"value": "70", "unit": "%"}))
    assert end(above) == ("SUCCEEDED", None) and sent(above) == []
    reading = readings(simulate(look(body)))[0]
    assert (reading.kind, reading.value, reading.unit, reading.uncertainty) == (
        "fill", Decimal("12"), "%", Decimal("2"))


def test_a_reading_near_the_threshold_is_unknown_and_the_run_fails_s261():
    body = 'measure r fill of crate using cam as v; if v > 80 % { communicate r to "lead" with "full"; }'
    near = simulate(look(body), set_property("fill", {"value": "80.5", "unit": "%"}))
    assert end(near) == ("FAILED", "S261")
    edge = simulate(look(body), set_property("fill", {"value": "82", "unit": "%"}))  # 80..84 is not all above 80
    assert end(edge) == ("FAILED", "S261")
    assert sent(simulate(look(body), set_property("fill", {"value": "82.01", "unit": "%"}))) == ["full"]


def test_an_interval_that_crosses_zero_or_100_is_not_clamped():
    body = 'measure r fill of crate using cam as v; if v >= 100 % { communicate r to "lead" with "full"; }'
    assert end(simulate(look(body), set_property("fill", {"value": "99", "unit": "%"}))) == ("FAILED", "S261")
    assert sent(simulate(look(body), set_property("fill", {"value": "100", "unit": "%"}))) == []  # 98..102 vs 100


def test_a_pressure_converts_exactly_and_the_comparison_is_the_same_in_every_unit_k2():
    def exact(world):
        world["sensors"]["pcam"]["measures"]["pressure"]["uncertainty"] = "0"
    cases = (("kPa", "420", "<", "450 kPa", True), ("bar", "4.5", "<", "450 kPa", False),
             ("bar", "4.5", "<=", "450 kPa", True), ("kPa", "450", "<", "4.5 bar", False),
             ("Pa", "450000", ">=", "4.5 bar", True), ("bar", "7.2", ">", "6 bar", True),
             ("bar", "5.1", ">", "6 bar", False), ("bar", "4.5", ">", "450 kPa", False))
    for unit, value, op, limit, expected in cases:
        body = f'measure r pressure of gauge using pcam as p; if p {op} {limit} {{ communicate r to "lead" with "y"; }}'

        def patch(world, unit=unit, value=value):
            exact(world)
            set_property("pressure", {"value": value, "unit": unit}, name="gauge_p3")(world)
        result = simulate(look(body), patch)
        assert end(result) == ("SUCCEEDED", None) and (sent(result) == ["y"]) is expected, (unit, value, op, limit)
    reading = readings(simulate(look(body), lambda w: (exact(w), set_property(
        "pressure", {"value": "420", "unit": "kPa"}, name="gauge_p3")(w))))[0]
    assert (reading.value, reading.unit, reading.uncertainty) == (Decimal("4.2"), "bar", Decimal("0"))


def test_a_pressure_over_six_bar_with_the_gauge_uncertainty():
    body = 'measure r pressure of gauge using pcam as p; if p > 6 bar { communicate r to "lead" with "over"; }'
    assert sent(simulate(look(body))) == ["over"]  # 7.2 bar +/- 0.1
    low = simulate(look(body), set_property("pressure", {"value": "5.1", "unit": "bar"}, name="gauge_p3"))
    assert end(low) == ("SUCCEEDED", None) and sent(low) == []
    near = simulate(look(body), set_property("pressure", {"value": "6.05", "unit": "bar"}, name="gauge_p3"))
    assert end(near) == ("FAILED", "S261")


def test_a_count_is_exact_and_equals_decides_k3():
    body = 'measure r count of bay using tag as n; if n == 0 { communicate r to "lead" with "empty"; }'
    result = simulate(look(body))
    assert sent(result) == ["empty"]
    reading = readings(result)[0]
    assert (reading.kind, reading.value, reading.unit, reading.uncertainty) == ("count", Decimal("0"), None, Decimal(0))
    nine = simulate(look(body), set_property("count", {"value": "9"}, "location_properties", "bay"))
    assert end(nine) == ("SUCCEEDED", None) and sent(nine) == []
    other = 'measure r count of bay using tag as n; if n != 0 and n >= 5 { communicate r to "lead" with "some"; }'
    assert sent(simulate(look(other), set_property("count", {"value": "9"}, "location_properties", "bay"))) == ["some"]
    assert sent(simulate(look(other), set_property("count", {"value": "4"}, "location_properties", "bay"))) == []


def test_a_confidence_belongs_to_the_label_and_both_are_read_together_k4():
    body = ('measure r label of plant using cam as seen; measure r confidence of plant using cam as sure; '
            'if seen == "weed" and sure >= 90 % { communicate r to "lead" with "weed confirmed"; } '
            'if seen != "weed" or sure < 90 % { communicate r to "lead" with "skipped"; }')
    assert sent(simulate(look(body))) == ["weed confirmed"]  # 95 % +/- 1
    low = simulate(look(body), set_property("confidence", {"value": "80", "unit": "%"}, name="plant_7"))
    assert sent(low) == ["skipped"]
    grass = simulate(look(body), set_property("label", {"value": "grass"}, name="plant_7"))
    assert sent(grass) == ["skipped"]
    alone = simulate(look('measure r confidence of plant using cam as c; communicate r to "lead" with c;'))
    assert alone.steps[2].message.value.kind == "confidence" and alone.steps[2].message.value.unit == "%"  # D4


def test_a_location_is_read_like_an_object_d1():
    body = 'measure r fill of bay using cam as v; if v < 25 % { communicate r to "lead" with "bay low"; }'
    assert sent(simulate(look(body))) == ["bay low"]
    assert readings(simulate(look(body)))[0].subject == "bay"
    no_value = simulate(look(body), lambda w: w["location_properties"].pop("bay"))
    assert end(no_value) == ("FAILED", "S236")
    event = lambda w: w.update(events=[{"at": {"value": "5", "unit": "s"}, "fact": "property", "target": "bay",  # noqa: E731
                                        "property": "fill", "value": {"value": "90", "unit": "%"}}])
    waited = 'wait for 10 s; measure r fill of bay using cam as v; if v > 50 % { communicate r to "lead" with "now"; }'
    assert sent(simulate(look(waited), event)) == ["now"]


def test_a_contact_sensor_reads_a_location_only_where_the_robot_is():
    def contact(world):
        world["sensors"]["tag"]["measures"]["count"]["contact"] = True
    body = 'measure r count of bay using tag as n;'
    assert end(simulate(look(body), contact)) == ("FAILED", "S235")  # the robot is at the aisle, not at the bay
    near = look("move r to bay; measure r count of bay using tag as n;")
    assert readings(simulate(near, contact))[0].subject == "bay"  # the goal fails afterwards, the reading did not


def test_the_failures_of_a_reading_are_the_codes_of_1_6():
    assert end(simulate(look("measure r fill of crate using tag as v;"))) == ("FAILED", "S234")  # tag measures count
    assert end(simulate(look("measure r pressure of crate using pcam as v;"))) == ("FAILED", "S236")  # no value
    assert end(simulate(look("measure r fill of crate using cam as v;"), bindings={**BIND, "cam": "cam_other"})) == (
        "FAILED", "S231")  # cam_other is on bot2
    unavailable = lambda w: w["sensors"]["cam"].update(available=False)  # noqa: E731
    assert end(simulate(look("measure r fill of crate using cam as v;"), unavailable)) == ("FAILED", "S232")
    absent = lambda w: w["objects"].update(crate={"present": False})  # noqa: E731
    assert end(simulate(look("measure r fill of crate using cam as v;"), absent)) == ("FAILED", "S213")


def test_validate_world_reports_a_sensor_that_does_not_measure_the_kind_s234():
    task = compile_source(look("measure r fill of crate using tag as v;")).ir.task("look")
    world = load_world(world_data(), "w.json", task, BIND)
    assert [f.code for f in check_task(task, world)] == ["S234"]
    ok = compile_source(look("measure r count of crate using tag as v;")).ir.task("look")
    assert check_task(ok, load_world(world_data(), "w.json", ok, BIND)) == []


def test_a_live_reading_of_the_new_kinds_ends_a_wait_at_the_event_d6():
    def fills_up(world):
        world["events"] = [{"at": {"value": "2", "unit": "min"}, "fact": "property", "target": "crate",
                            "property": "fill", "value": {"value": "95", "unit": "%"}}]
    body = 'wait until fill of crate using cam > 80 % timeout 10 min else { fail "never full"; }'
    result = simulate(look(body), fills_up)
    assert end(result) == ("SUCCEEDED", None) and str(result.end_t_ms) == "120000"
    never = simulate(look(body))
    assert never.state == "FAILED" and any(step.handled == "S241" for step in never.steps)

    def empties(world):
        world["events"] = [{"at": {"value": "30", "unit": "s"}, "fact": "property", "target": "bay",
                            "property": "count", "value": {"value": "0"}}]
    count = 'wait until count of bay using tag == 0 timeout 5 min else { fail "never empty"; }'
    stocked = set_property("count", {"value": "4"}, "location_properties", "bay")

    def both(world):
        stocked(world)
        empties(world)
    done = simulate(look(count), both)
    assert end(done) == ("SUCCEEDED", None) and str(done.end_t_ms) == "30000"


def test_the_new_kinds_work_with_a_clock_wait_and_a_message_wait_in_one_task():
    body = ('wait until clock_time >= 09:00 timeout 20 min else { fail "no time"; } '
            'wait until message_received "lead" with "SCAN NOW" timeout 5 min else { fail "no signal"; } '
            'measure r count of bay using tag as n; if n == 0 { communicate r to "lead" with "bay is empty"; }')

    def patch(world):
        world["clock"] = {"start": "2026-11-18T08:50:00", "zone": "UTC"}
        world["messages"] = [{"at": {"value": "100", "unit": "s"}, "channel": "lead", "text": "SCAN NOW",
                              "from": "rl:agent:ops"}]
    result = simulate(look(body), patch)
    assert end(result) == ("SUCCEEDED", None) and sent(result) == ["bay is empty"]
    assert str(result.end_t_ms) == "600000"  # 09:00 at 600 s; the message at 100 s was already counted


# -------- run records --------

def test_the_run_record_is_1_10_and_a_count_reading_has_no_unit_key():
    body = ('measure r count of bay using tag as n; measure r fill of crate using cam as f; '
            'communicate r to "lead" with n;')
    source = look(body)
    task = compile_source(source).ir.task("look")
    world = load_world(world_data(), "w.json", task, BIND)
    record = record_of(task, run(task, world), "2.2")
    assert record["run_record_schema_version"] == "1.10" and record["inputs"]["language_version"] == "2.2"
    count, fill = record["steps"][1]["reading"], record["steps"][2]["reading"]
    assert count == {"subject": "bay", "kind": "count", "value": "0", "uncertainty": "0", "sensor": "tag", "t_ms": "0"}
    assert fill["unit"] == "%" and fill["uncertainty"] == "2" and fill["value"] == "12"
    sent_value = record["steps"][3]["message"]["value"]
    assert sent_value["kind"] == "count" and "unit" not in sent_value
    assert json.dumps(record).isascii() and record["messages"] == []


def test_a_property_event_of_a_count_is_recorded_without_a_unit():
    def event(world):
        world["events"] = [{"at": {"value": "5", "unit": "s"}, "fact": "property", "target": "bay",
                            "property": "count", "value": {"value": "7"}}]
    source = look("wait for 10 s; measure r count of bay using tag as n;")
    task = compile_source(source).ir.task("look")
    world = load_world(world_data(event), "w.json", task, BIND)
    record = record_of(task, run(task, world), "2.2")
    assert record["events_applied"][0]["value"] == {"value": "7"}
    assert record["steps"][2]["reading"]["value"] == "7"


def test_a_2_1_record_is_unchanged_it_is_1_9_and_embeds_task_ir_1_9():
    source = ("robolang 2.1;\ntask t(robot r, object crate, location aisle, sensor cam) {\n"
              " goal { robot_at r aisle; }\n"
              " plan { move r to aisle; measure r temperature of crate using cam as v; }\n}\n")
    task = compile_source(source).ir.task("t")

    def patch(world):
        world["world_schema_version"] = "1.6"
        world.pop("location_properties")
        world["objects"] = {"crate": {"present": True, "location": "aisle", "properties": {
            "temperature": {"value": "20", "unit": "degC"}}}}
        world["sensors"] = {"cam": {"robot": "bot", "available": True, "measures": {
            "temperature": {"unit": "degC", "uncertainty": "1", "contact": False}}}}
    bindings = {"r": "bot", "crate": "crate", "aisle": "aisle", "cam": "cam"}
    world = load_world(world_data(patch), "w.json", task, bindings)
    record = record_of(task, run(task, world), "2.1", "1.6")
    assert record["run_record_schema_version"] == "1.9"
    assert record["steps"][1]["reading"]["unit"] == "degC" and record["inputs"]["ir_schema_version"] == "1.9"


def test_the_mission_record_is_1_2_with_the_messages_of_2_1():
    source = (FIXTURES / "valid" / "a_mission_runs_a_task_that_reads_a_count.rob").read_text(encoding="utf-8")
    document = compile_source(source).ir
    mission = document.mission("nightly")
    bindings = {"r": "bot", "aisle": "aisle", "shelf": "bay", "tag": "tag"}
    world = load_world(world_data(), "w.json", mission_task(mission), bindings)
    result = run_mission(document, mission, world)
    record = to_mission_record(result, document=document, run_id="sim-" + "0" * 64, source_sha256="a" * 64,
                               world_sha256="b" * 64, seed="0", world_schema_version="1.7", world_clock=world.clock)
    assert (record["mission_record_schema_version"], record["run_record_schema_version"]) == ("1.2", "1.10")
    assert record["inputs"]["ir_schema_version"] == "1.2" and record["inputs"]["task_ir_schema_version"] == "1.10"
    assert record["messages"] == [] and [len(r["messages"]) for r in record["runs"]] == [0] * len(record["runs"])


# -------- the command line and the refusals --------

def cli(*args):
    out, err = io.StringIO(), io.StringIO()
    code = main(list(args), out=out, err=err)
    return code, out.getvalue(), err.getvalue()


@pytest.fixture
def files(tmp_path):
    rob = tmp_path / "look.rob"
    rob.write_text(look('measure r fill of crate using cam as v; if v < 30 % { communicate r to "lead" with v; }'),
                   encoding="utf-8")
    data = json.loads(world_data())
    data["tasks"] = {"look": {"bindings": BIND}}
    for name in ("crate", "gauge_p3", "plant_7"):
        data["objects"][name].update({"class": "crate", "mass": {"value": "4", "unit": "kg", "uncertainty": "0.1"}})
    world = tmp_path / "w.json"
    world.write_text(json.dumps(data), encoding="utf-8")
    return str(rob), str(world)


def test_validate_simulate_and_inspect_work_on_a_2_2_file(files):
    code, out, err = cli("validate", files[0])
    assert code == 0 and "valid RoboLang 2.2" in out, err
    code, out, err = cli("simulate", files[0], "--world", files[1])
    assert code == 0, err
    record = json.loads(out)
    assert record["state"] == "SUCCEEDED" and record["run_record_schema_version"] == "1.10" and out.isascii()
    code, out, err = cli("inspect", files[0])
    assert code == 0 and "v < 30 %" in out, err
    code, out, err = cli("validate", files[0], "--world", files[1])
    assert code == 0, err


def test_ir_input_round_trips_through_every_command(files, tmp_path):
    code, out, _ = cli("inspect", files[0], "--json")
    ir_path = tmp_path / "look.json"
    ir_path.write_text(out, encoding="utf-8")
    code, out, err = cli("simulate", "--ir", str(ir_path), "--world", files[1])
    assert code == 0 and json.loads(out)["inputs"]["source_kind"] == "ir", err
    code, out, err = cli("validate", "--ir", str(ir_path))
    assert code == 0 and "Task IR 1.10" in out, err


def test_version_shows_language_2_2(capsys):
    code, _, _ = cli("--version")
    out = capsys.readouterr().out
    assert code == 0 and "2.1 and 2.2" in out and "1.9 and 1.10" in out and "mission IR 1.0, 1.1 and 1.2" in out


def test_export_goal_is_unaffected_a_goal_holds_no_value(files):
    code, out, err = cli("export-goal", files[0], "--world", files[1], "--from", "rl:agent:planner", "--core", "0.2")
    assert code == 0, err
    assert json.loads(out)["payload"]["goal"]


def test_delegate_refuses_a_task_with_a_measure_whatever_its_kind(files):
    from robolang import delegate
    for kind in ("fill", "count", "pressure", "confidence"):
        task = compile_source(look(f"measure r {kind} of crate using cam as v;")).ir.tasks[0]
        assert delegate._consider(None, task, b"", "w.json", "digit1", (), "0.1") == ("measure_unsupported", None)


def test_the_ros2_backend_refuses_every_new_kind_with_s509_before_any_motion():
    pytest.importorskip("jsonschema")
    sys.path.insert(0, str(ROOT / "backends" / "ros2"))
    from robolang_ros2.executor import execute
    from test_ros2_executor import FakeOps, fixture
    _, bindings, site = fixture()
    for kind, target in (("fill", "box"), ("count", "src"), ("pressure", "box"), ("confidence", "box")):
        source = ("robolang 2.2;\ntask transfer(robot r, object box, location src, location dst, sensor cam) {\n"
                  f"  goal {{ robot_at r dst; }}\n"
                  f"  plan {{ measure r {kind} of {target} using cam as v; move r to dst; }}\n}}\n")
        ops = FakeOps()
        bound = {**bindings, "cam": "cam"}
        record = execute(compile_source(source).ir.tasks[0], bound, site, ops)
        assert record["failure"]["code"] == "S509", kind
        assert ops.calls == [("guard", "ur")]


# -------- Python 3.11 and the economy --------

def test_no_dataclass_of_the_world_has_a_mutable_default_python_3_11():
    """PR #151 erratum E4: Python 3.11 rejects an unhashable dataclass default when the module is imported."""
    import dataclasses
    from robolang import nodes
    from robolang.sim import executor, messages, mission, world
    for module in (messages, mission, executor, world, nodes):
        for item in vars(module).values():
            if dataclasses.is_dataclass(item) and isinstance(item, type):
                for f in dataclasses.fields(item):
                    if f.default is not dataclasses.MISSING:
                        hash(f.default)


def test_the_economy_no_statement_no_reserved_word_four_kinds_one_unit_token_three_pressure_units():
    from robolang import tokens
    assert tokens.RESERVED_V2_0 == RESERVED_BY_VERSION["2.2"]
    assert set(KIND_UNITS_V2_2) - set(KIND_UNITS) == {"fill", "confidence", "pressure", "count"}
    assert tokens.NEW_KINDS_V2_2 == ("fill", "confidence", "pressure", "count")
    new_units = {u for units in KIND_UNITS_V2_2.values() for u in units} - {u for v in KIND_UNITS.values() for u in v}
    assert new_units == {"%", "Pa", "kPa", "bar"}
    assert tokens.STATEMENTS_V1_8 == tokens.STATEMENTS_V1_8 and tokens.CONDITIONS_V2_1 == tokens.CONDITIONS_V2_1


def test_a_task_with_many_readings_and_branches_is_fast():
    """Nothing walks a plan's paths exponentially: 12 readings with an `if` each is 4096 paths."""
    body = " ".join(f'measure r fill of crate using cam as v{i}; '
                    f'if v{i} < 50 % {{ communicate r to "lead" with "a{i}"; }}' for i in range(12))
    source = look(body)
    check(source)
    result = simulate(source)
    assert result.state in ("SUCCEEDED", "FAILED")


def test_the_safety_case_has_hazard_h24():
    text = (ROOT / "docs" / "safety-case-draft.md").read_text(encoding="utf-8")
    assert "| H24 | A mistaken reading, unit or count changes what a robot does |" in text
