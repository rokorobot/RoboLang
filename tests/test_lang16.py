"""
RoboLang 1.6: measured values (spec/robolang-v1.6.md, decision 0036 V1-V10).

Front end, Task IR 1.5 and its reader, world file 1.2 and the simulation (readings, uncertainty, live readings,
typed messages), `validate --world`, the refusals in `delegate` and the ROS 2 backend, and the set 5 benchmark
requests that 1.6 makes expressible (bench/tasks_values/, written here as programs on world 1.2 copies built in
the tests: bench/ itself is not touched).

To regenerate the IR snapshots after an intended change:

    ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_lang16.py
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
FIXTURES = ROOT / "tests" / "fixtures" / "v1.6"
SNAPSHOTS = ROOT / "tests" / "snapshots" / "ir-v1.6"
UPDATE = os.environ.get("ROBOLANG_UPDATE_SNAPSHOTS") == "1"
SPEC = (ROOT / "spec" / "robolang-v1.6.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")

# -------- helpers --------

PARAMS = ("robot r, robot m, object pan, object pan2, object ghost, object vague, sensor probe, sensor loose, "
          "sensor dead, sensor maybe, sensor blind, location bench, location home")
ENTITY = {"r": "arm1", "m": "arm2", "pan": "pan1", "pan2": "pan2", "ghost": "ghost", "vague": "vague",
          "probe": "probe1", "loose": "loose", "dead": "dead", "maybe": "maybe", "blind": "blind",
          "bench": "bench", "home": "home"}
BASE_WORLD = {
    "world_schema": "robolang.sim.world",
    "world_schema_version": "1.2",
    "locations": ["bench", "home", "shelf"],
    "robots": {"arm1": {"ready": True, "location": "bench", "gripper": True},
               "arm2": {"ready": True, "location": "home", "gripper": True}},
    "objects": {
        "pan1": {"present": True, "location": "bench", "properties": {
            "temperature": {"value": "72", "unit": "degC"}, "mass": {"value": "1500", "unit": "g"},
            "label": {"value": "LOT-7731"}, "distance": {"value": "250", "unit": "mm"}}},
        "pan2": {"present": True, "location": "bench", "properties": {
            "temperature": {"value": "66", "unit": "degC"}, "label": {"value": "LOT-0001"}}},
        "ghost": {"present": False},
        "vague": {},
    },
    "sensors": {
        "probe1": {"robot": "arm1", "available": True, "measures": {
            "temperature": {"unit": "degC", "uncertainty": "0.5", "contact": True},
            "mass": {"unit": "kg", "uncertainty": "0.01", "contact": True},
            "label": {"contact": False},
            "distance": {"unit": "mm", "uncertainty": "1", "contact": False}}},
        "loose": {"available": True, "measures": {"temperature": {"unit": "degC", "uncertainty": "0.5",
                                                                  "contact": True}}},
        "dead": {"robot": "arm1", "available": False, "measures": {"temperature": {"unit": "degC",
                                                                                   "uncertainty": "0.5",
                                                                                   "contact": False}}},
        "maybe": {"robot": "arm1", "measures": {"temperature": {"unit": "degC", "uncertainty": "0.5",
                                                                "contact": False}}},
        "blind": {"robot": "arm1", "available": True},
    },
    "events": [],
    "tasks": {},
}


def task(plan: str, goal: str | None = "goal { robot_at r bench; }", params: str = PARAMS, version: str = "1.6",
         name: str = "t") -> str:
    return f"robolang {version};\ntask {name}({params}) {{\n{goal or ''}\nplan {{\n{plan}\n}}\n}}\n"


def errors(source: str) -> list[tuple[str, int]]:
    with pytest.raises(RoboLangError) as caught:
        check(source)
    return [(d.code, d.line) for d in caught.value.diagnostics]


def codes(source: str) -> list[str]:
    """The error codes of `source`, in order; empty when it is valid."""
    try:
        check(source)
    except RoboLangError as error:
        return [d.code for d in error.diagnostics]
    return []


def world_data(patch=None, events=None, faults=(), version="1.2", bindings=None, name="t", params=None) -> bytes:
    world = copy.deepcopy(BASE_WORLD)
    world["world_schema_version"] = version
    if patch is not None:
        patch(world)
    if events is not None:
        world["events"] = events
    wanted = params if params is not None else ENTITY
    world["tasks"] = {name: {"bindings": bindings or dict(wanted),
                             **({"faults": [{"step": n} for n in faults]} if faults else {})}}
    return json.dumps(world).encode()


def simulate(source: str, patch=None, events=None, faults=(), world=None, entities=None):
    """Compile, run against world 1.2 (or `world`), return (result, record). Bindings: the entities of the params."""
    compiled = compile_source(source)
    plan = compiled.ir.tasks[0]
    names = [p.name for p in plan.parameters]
    bindings = {n: (entities or ENTITY)[n] for n in names}
    data = world if world is not None else world_data(patch, events, faults, bindings=bindings, name=plan.name)
    loaded = load_world(data, "w.json", plan)
    result = run(plan, loaded)
    record = to_record(result, task=plan, run_id="sim-x", source_sha256="0", world_sha256="0", seed="0",
                       language_version=compiled.ir.language_version, world_schema_version=loaded.schema_version)
    return result, record


def failure(record) -> tuple[str, int] | None:
    return None if record["failure"] is None else (record["failure"]["code"], record["failure"]["step"])


def decision(record, index: int = 0) -> str:
    return record["decisions"][index]["condition_result"]


def measuring(patch_value=None, patch_sensor=None):
    """A world patch: `pan1`'s temperature, and `probe1`'s temperature uncertainty."""
    def patch(world):
        if patch_value is not None:
            world["objects"]["pan1"]["properties"]["temperature"] = {"value": patch_value, "unit": "degC"}
        if patch_sensor is not None:
            world["sensors"]["probe1"]["measures"]["temperature"]["uncertainty"] = patch_sensor
    return patch


# -------- front end --------

def test_version_1_6_selects_ir_schema_1_5_and_older_versions_keep_theirs():
    document = json.loads(to_json(compile_source(task("measure r mass of pan using probe as w;")).ir))
    assert (document["language_version"], document["ir_schema_version"]) == ("1.6", "1.5")
    assert ir.IR_SCHEMA_VERSIONS == {"1.1": "1.0", "1.2": "1.1", "1.3": "1.2", "1.4": "1.3", "1.5": "1.4",
                                     "1.6": "1.5"}


def test_only_measure_and_as_are_reserved_and_only_in_1_6():
    assert codes(task("move r to bench;", params="robot r, location measure")) == ["E212"]
    assert codes(task("move r to bench;", params="robot r, location as")) == ["E212"]
    check(task("move r to measure;", params="robot r, location measure, location as", version="1.5",
               goal="goal { robot_at r as; }"))
    # kinds, `of` and units are recognised by position: they stay valid names in 1.6
    check(task("move r to m;", params="robot r, location m, object label, object temperature, location of, "
                                      "location K, location degC, location kg",
               goal="goal { robot_at r m; }"))
    check(task("measure r label of label using probe as mass;\nif mass == \"x\" { move r to bench; } else "
               "{ move r to bench; }", params="robot r, object label, sensor probe, location bench"))


def test_comparison_operators_are_tokens_in_1_6_only():
    plan = 'measure r mass of pan using probe as w;\nif w < 5 kg { move r to bench; } else { move r to bench; }'
    check(task(plan))
    for version in ("1.5", "1.4"):
        assert codes(task(plan.replace("measure r mass of pan using probe as w;\n", ""), version=version))[0] == "E101"
    assert codes(task("measure r mass of pan using probe as w;", version="1.5"))[0] == "E203"
    assert codes(task("measure r mass of pan using probe as w;\nif w = 5 kg { move r to bench; }")) == ["E101"]


@pytest.mark.parametrize("text", [
    "if (w + 1 kg) < 5 kg", "if w < 5 kg + 1 kg", "if w / 2 < 5 kg", "if w < 5 kg * 2", "if w < -5 kg"])
def test_arithmetic_has_no_operator_to_write(text):
    source = task(f"measure r mass of pan using probe as w;\n{text} {{ move r to bench; }}")
    assert codes(source)[0] in ("E101", "E201")


def test_the_comparison_forms_parse_with_and_or_not_and_groups():
    plan = ("measure r mass of pan using probe as w;\nmeasure r label of pan using probe as lot;\n"
            'if not w >= 5 kg and (lot == "a" or lot != "b") { move r to bench; } else { move r to bench; }')
    compiled = compile_source(task(plan)).ir.tasks[0]
    condition = json.loads(to_json(compile_source(task(plan)).ir))["tasks"][0]["steps"][2]["condition"]
    assert condition["kind"] == "and" and [c["kind"] for c in condition["conditions"]] == ["not", "or"]
    assert compiled.steps[2].condition.conditions[0].condition.op == ">="


def test_quantities_are_unsigned_and_need_a_unit_of_a_known_kind():
    base = "measure r mass of pan using probe as w;\nif w < {} {{ move r to bench; }}"
    assert codes(task(base.format("-5 kg")))[0] == "E101"
    assert codes(task(base.format("5")))[0] == "E206"
    assert codes(task(base.format("5 lb")))[0] == "E206"
    assert codes(task(base.format("5 m/s")))[0] == "E206"
    assert codes(task(base.format("5 degC"))) == ["E319"]
    check(task(base.format("0.5 kg")))


@pytest.mark.parametrize("kind", ["temperature", "mass", "distance", "label"])
def test_the_four_kinds_measure(kind):
    check(task(f"measure r {kind} of pan using probe as v;"))


def test_a_wait_until_still_needs_a_handler_and_measure_takes_one():
    plan = "wait until temperature of pan using probe < 60 degC timeout 1 min;"
    assert codes(task(plan)) == ["E316"]
    with pytest.raises(RoboLangError) as caught:
        check(task(plan))
    assert "RoboLang 1.6" in caught.value.diagnostic.message
    check(task("measure r mass of pan using probe as w else { measure r mass of pan using probe as w else "
               "{ fail \"x\"; } }"))


def test_a_handler_of_measure_counts_as_a_nesting_level_e216():
    def chain(depth):  # a distinct binding name at every level
        text = "move r to bench;"
        for level in range(depth, 0, -1):
            text = f"measure r mass of pan using probe as w{level} else {{ {text} }}"
        return text
    check(task(chain(4)))
    assert codes(task(chain(5))) == ["E216"]


def test_every_path_needs_a_goal_through_a_handled_measure_e309():
    with pytest.raises(RoboLangError) as caught:
        check(task("measure r mass of pan using probe as w else { move r to bench; }", goal=None))
    assert caught.value.code == "E309" and "path ok" in caught.value.diagnostic.message
    check(task('measure r mass of pan using probe as w else { fail "x"; }'))


def test_the_holder_check_follows_both_sides_of_a_handled_measure():
    # the failed side of a measure had no effect: pan is still held by r when its handler runs
    assert codes(task("grasp r pan;\nmeasure r mass of pan using probe as w else { grasp m pan; }")) == ["E315"]
    check(task("grasp r pan;\nplace r pan at bench;\nmeasure r mass of pan using probe as w else "
               "{ grasp m pan; place m pan at bench; }"))
    plan = "grasp r pan else { fail \"x\"; }\nmeasure r mass of pan using probe as w;\ngrasp m pan;"
    assert codes(task(plan)) == ["E315"]


# -------- bindings (V3) --------

def test_a_binding_is_visible_after_its_measure_in_its_block_and_nested_blocks():
    check(task("measure r mass of pan using probe as w;\nif object_exists pan {\n if w > 1 kg {\n"
               ' communicate r to "ops" with w;\n }\n} else { communicate r to "ops" with w; }'))


@pytest.mark.parametrize("plan, expected", [
    ("if w > 1 kg { move r to bench; }\nmeasure r mass of pan using probe as w;", ["E317"]),
    ('communicate r to "ops" with w;\nmeasure r mass of pan using probe as w;', ["E317"]),
    ("if object_exists pan { measure r mass of pan using probe as w; }\n"
     'communicate r to "ops" with w;', ["E317"]),
    ("measure r mass of pan using probe as w else { move r to bench; }\n"
     'communicate r to "ops" with w;', ["E317"]),
    ("move r to bench else { measure r mass of pan using probe as w; }\n"
     'communicate r to "ops" with w;', ["E317"]),
    ('measure r mass of pan using probe as w else { communicate r to "ops" with w; fail "x"; }', ["E317"]),
    ('communicate r to "ops" with pan;', ["E317"]),
    ('communicate r to "ops" with nothing;', ["E317"]),
    ("measure r mass of pan using probe as pan;", ["E318"]),
    ("measure r mass of pan using probe as w;\nmeasure r mass of pan using probe as w;", ["E318"]),
    ("if object_exists pan { measure r mass of pan using probe as w; } else "
     "{ measure r mass of pan using probe as w; }", ["E318"]),
    ("measure r mass of pan using probe as w else { move r to bench; }\nmeasure r mass of pan using probe as w;",
     ["E318"]),
])
def test_binding_visibility_and_uniqueness(plan, expected):
    assert codes(task(plan)) == expected


@pytest.mark.parametrize("plan", [
    # the retry: the handler binds the same name, so it is defined on both paths (V3 option b)
    "measure r mass of pan using probe as w else { measure r mass of pan using probe as w; }\n"
    'communicate r to "ops" with w;',
    "measure r mass of pan using probe as w else { measure r mass of pan using probe as w else { fail \"x\"; } }\n"
    'communicate r to "ops" with w;',
    # a handler that ends in fail has no normal exit: the binding is visible after
    'measure r mass of pan using probe as w else { fail "no reading"; }\ncommunicate r to "ops" with w;',
    'measure r mass of pan using probe as w else { communicate r to "ops" with "no reading"; fail "x"; }\n'
    'communicate r to "ops" with w;',
])
def test_retry_rebinding_and_failing_handlers_keep_the_binding(plan):
    check(task(plan))


def test_a_retry_inside_an_if_in_the_handler_does_not_bind_it_after():
    plan = ("measure r mass of pan using probe as w else {\n if object_exists pan { measure r mass of pan using "
            "probe as w; } else { fail \"x\"; }\n}\ncommunicate r to \"ops\" with w;")
    assert codes(task(plan)) == ["E317"]


def test_the_retry_exception_is_only_inside_the_handler_of_the_same_binding():
    plan = ("measure r mass of pan using probe as a else { measure r mass of pan using probe as b; }\n"
            "measure r mass of pan using probe as b;")
    assert codes(task(plan)) == ["E318"]
    assert codes(task("measure r mass of pan using probe as a else { measure r temperature of pan using probe as a;"
                      " }")) == ["E319"]


# -------- kinds, units, comparisons (V5, V6, V1, V4) --------

MASS = "measure r mass of pan using probe as w;\nmeasure r temperature of pan using probe as t;\n" \
       "measure r label of pan using probe as lot;\nmeasure r distance of pan using probe as d;\n"


@pytest.mark.parametrize("condition, expected", [
    ("w < 5 g", []), ("w < 5 kg", []), ("t >= 5 K", []), ("t > 5 degC", []), ("d <= 5 mm", []), ("d <= 5 cm", []),
    ("d <= 5 m", []), ("lot == \"x\"", []), ("lot != \"x\"", []), ("w < w", []), ("lot == lot", []),
    ("w < 5 degC", ["E319"]), ("t < 5 kg", ["E319"]), ("d < 5 g", ["E319"]), ("lot == 5 g", ["E319"]),
    ("w == \"x\"", ["E319"]), ("w < t", ["E319"]), ("lot == w", ["E319"]),
    ("w == 5 kg", ["E320"]), ("w != 5 kg", ["E320"]), ("t == t", ["E320"]),
    ("lot < \"x\"", ["E320"]), ("lot >= lot", ["E320"]),
])
def test_kinds_units_and_operators_of_comparisons(condition, expected):
    plan = MASS + f"if {condition} {{ move r to bench; }} else {{ move r to bench; }}"
    assert codes(task(plan)) == expected


def test_comparisons_are_not_allowed_in_pre_or_goal_blocks_e321():
    for block, text in (("pre", "pre { temperature of pan using probe < 60 degC; }"),
                        ("goal", "goal { temperature of pan using probe < 60 degC; }")):
        source = f"robolang 1.6;\ntask t({PARAMS}) {{\n{text}\n"
        source += "goal { robot_at r bench; }\n" if block == "pre" else ""
        source += "plan { move r to bench; }\n}\n"
        assert codes(source) == ["E321"], block
    branch = ("measure r mass of pan using probe as w;\nif object_exists pan {\n goal { w > 1 kg; }\n"
              " move r to bench;\n} else { move r to bench; }")
    assert codes(task(branch)) == ["E321"]
    assert codes(task(branch.replace("goal { w > 1 kg; }", "goal { robot_at r bench; }"))) == []


def test_live_readings_only_in_wait_until_conditions_e322():
    live = "temperature of pan using probe < 60 degC"
    assert codes(task(f"if {live} {{ move r to bench; }}")) == ["E322"]
    assert codes(task(f"if object_exists pan and {live} {{ move r to bench; }}")) == ["E322"]
    check(task(f'wait until {live} timeout 1 min else {{ fail "x"; }}'))
    check(task(f'wait until object_exists pan and not {live} timeout 1 min else {{ fail "x"; }}'))
    # a live reading names an object and a sensor: they are checked like any use
    assert codes(task('wait until temperature of r using probe < 5 degC timeout 1 min else { fail "x"; }')) == ["E304"]
    assert codes(task('wait until temperature of pan using r < 5 degC timeout 1 min else { fail "x"; }')) == ["E304"]
    assert codes(task('wait until temperature of pan using "p" < 5 degC timeout 1 min else { fail "x"; }')) == ["E310"]


def test_live_readings_are_on_the_left_and_the_right_side_is_a_value():
    assert codes(task('wait until 60 degC > temperature of pan using probe timeout 1 min else { fail "x"; }'))[0] \
        == "E201"
    # the right side of an operator is a quantity, a string or a name: `temperature` here is read as a binding name
    assert codes(task('wait until temperature of pan using probe < temperature of pan using probe timeout 1 min '
                      'else { fail "x"; }'))[0] == "E209"
    assert codes(task('wait until colour of pan using probe < 5 degC timeout 1 min else { fail "x"; }'))[0] == "E204"


def test_communicate_takes_a_string_or_a_binding_and_nothing_else():
    assert codes(task('communicate r to "ops" with 5 kg;'))[0] == "E201"
    assert codes(task('measure r mass of pan using probe as w;\ncommunicate r to "ops" with w "kg";'))[0] == "E202"
    assert codes(task('measure r mass of pan using probe as w;\ncommunicate r to "ops" with "Pan is " w;'))[0] \
        == "E202"
    assert codes(task('communicate r to "ops" with "";')) == ["E307"]
    check(task('communicate r to "ops" with "text";'))


def test_a_misspelt_condition_still_names_the_conditions():
    with pytest.raises(RoboLangError) as caught:
        check(task("if robot_redy r { move r to bench; }"))
    assert caught.value.code == "E204" and "robot_ready" in caught.value.diagnostic.message


def test_measure_names_its_pieces_in_order():
    assert codes(task("measure r kilo of pan using probe as w;")) == ["E217"]
    assert codes(task("measure r mass pan using probe as w;"))[0] == "E201"
    assert codes(task("measure r mass of pan with probe as w;"))[0] == "E209"
    assert codes(task("measure r mass of pan using probe w;"))[0] == "E201"
    assert codes(task("measure r mass of pan using probe as w")) == ["E202"]
    assert codes(task("measure pan mass of pan using probe as w;")) == ["E304"]
    assert codes(task("measure r mass of r using probe as w;")) == ["E304"]
    assert codes(task("measure r mass of pan using bench as w;")) == ["E304"]
    assert codes(task('measure r mass of pan using "probe" as w;')) == ["E310"]


# -------- fixtures and snapshots --------

HEADER = re.compile(
    r"\A// RoboLang v1\.6 fixture: (?P<set>valid|invalid)\n"
    r"// expect: (?:(?P<ok>ok)|error (?P<code>E\d{3}) \((?P<stage>lexical|syntax|static)\) line (?P<line>\d+))\n"
    r"// why: .+\n"
)
STAGE_DIGIT = {"lexical": "1", "syntax": "2", "static": "3"}
VALID = sorted((FIXTURES / "valid").glob("*.rob"))
INVALID = sorted((FIXTURES / "invalid").glob("*.rob"))
NEW_STATIC_CODES = ("E217", "E317", "E318", "E319", "E320", "E321", "E322")


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
    assert (document["language_version"], document["ir_schema_version"]) == ("1.6", "1.5")
    assert read_ir(actual.encode()) == compile_source(path.read_text(encoding="utf-8")).ir


@pytest.mark.parametrize("path", INVALID, ids=lambda p: p.stem)
def test_invalid_fixture_is_rejected_with_the_stated_code_and_line_and_makes_no_ir(path):
    stated = header(path)
    assert path.name.startswith(stated["code"].lower()) and stated["code"][1] == STAGE_DIGIT[stated["stage"]]
    with pytest.raises(RoboLangError) as caught:
        compile_source(path.read_text(encoding="utf-8"), str(path))
    assert (caught.value.code, str(caught.value.line)) == (stated["code"], stated["line"])


def test_fixture_sets_cover_every_new_static_code_and_are_counted():
    assert len(VALID) == 5 and len(INVALID) == 23
    stated = {header(p)["code"] for p in INVALID}
    assert set(NEW_STATIC_CODES) <= stated


def test_every_new_code_is_in_the_specification_table():
    table = set(re.findall(r"^\| `((?:[EIS]\d{3})(?:`, `[EIS]\d{3})*)` \|", SPEC, re.MULTILINE))
    listed = {code for row in table for code in row.split("`, `")}
    needed = {*NEW_STATIC_CODES, "E512", "E513", "E514", "E515", "S234", "S235", "S236", "S509", "I124", "I125"}
    assert needed <= listed
    for code in ("S234", "S235", "S236", "E512", "E513", "E514", "E515"):
        assert f"`{code}`" in SIM_SPEC


# -------- Task IR 1.5 and the reader --------

def test_ir_shapes_of_measure_compare_and_communicate():
    source = task("measure r temperature of pan using probe as t else { fail \"x\"; }\n"
                  'if t < 60 degC { communicate r to "ops" with t; } else { communicate r to "ops" with "hot"; }\n'
                  'wait until temperature of pan using probe < 1.50 K timeout 1 min else { fail "y"; }')
    steps = json.loads(to_json(compile_source(source).ir))["tasks"][0]["steps"]
    assert steps[0] == {"kind": "measure", "robot": "r", "reading": "temperature", "object": "pan", "sensor": "probe",
                        "binding": "t", "else": [{"kind": "fail", "reason": "x"}]}
    assert steps[1]["condition"] == {"kind": "compare", "left": {"kind": "binding", "name": "t"}, "op": "<",
                                     "right": {"kind": "quantity", "value": "60", "unit": "degC"}}
    assert steps[1]["then"] == [{"kind": "communicate", "robot": "r", "channel": "ops", "binding": "t"}]
    assert steps[1]["else"] == [{"kind": "communicate", "robot": "r", "channel": "ops", "message": "hot"}]
    assert steps[2]["condition"]["left"] == {"kind": "live", "reading": "temperature", "object": "pan",
                                             "sensor": "probe"}
    assert steps[2]["condition"]["right"] == {"kind": "quantity", "value": "1.5", "unit": "K"}  # canonical decimal


def test_literals_are_kept_as_written_never_converted():
    source = task("measure r mass of pan using probe as w;\nif w > 1500 g and w < 2 kg { move r to bench; } else "
                  "{ move r to bench; }")
    condition = json.loads(to_json(compile_source(source).ir))["tasks"][0]["steps"][1]["condition"]
    assert [c["right"] for c in condition["conditions"]] == [{"kind": "quantity", "value": "1500", "unit": "g"},
                                                             {"kind": "quantity", "value": "2", "unit": "kg"}]


def test_ir_reads_back_identical_and_to_json_is_canonical():
    source = task("measure r mass of pan using probe as w else { measure r mass of pan using probe as w; }\n"
                  'wait until mass of pan using probe < 3 kg timeout 1 min else { fail "x"; }\n'
                  'communicate r to "ops" with w;')
    compiled = compile_source(source).ir
    data = to_json(compiled).encode()
    assert read_ir(data) == compiled and to_json(read_ir(data)).encode() == data


def ir_document(plan: str = "measure r mass of pan using probe as w;\ncommunicate r to \"ops\" with w;") -> dict:
    return json.loads(to_json(compile_source(task(plan)).ir))


def problems(document) -> list[tuple[str, str]]:
    with pytest.raises(IRError) as caught:
        read_ir(json.dumps(document).encode())
    return [(d.code, d.pointer) for d in caught.value.diagnostics]


def test_schema_1_4_does_not_have_the_new_kinds_i115_and_a_binding_in_communicate_is_unknown():
    document = ir_document()
    document.update(ir_schema_version="1.4", language_version="1.5")
    found = problems(document)
    assert ("I115", "/tasks/0/steps/0/kind") in found
    only_communicate = ir_document('communicate r to "ops" with "x";')
    only_communicate["tasks"][0]["steps"][0]["binding"] = "w"
    only_communicate.update(ir_schema_version="1.4", language_version="1.5")
    assert problems(only_communicate) == [("I112", "/tasks/0/steps/0/binding")]


def test_the_reader_is_strict_about_the_new_shapes():
    document = ir_document()
    document["tasks"][0]["steps"][1]["message"] = "x"
    assert problems(document) == [("I112", "/tasks/0/steps/1")]
    document = ir_document()
    del document["tasks"][0]["steps"][1]["binding"]
    assert problems(document) == [("I111", "/tasks/0/steps/1")]
    document = ir_document()
    document["tasks"][0]["steps"][0]["binding"] = "measure"
    assert problems(document)[0] == ("I118", "/tasks/0/steps/0/binding")
    document = ir_document()
    document["tasks"][0]["steps"][0]["extra"] = 1
    assert problems(document) == [("I112", "/tasks/0/steps/0/extra")]


def test_the_reader_checks_operands_units_and_where_a_live_reading_may_stand():
    plan = ("measure r mass of pan using probe as w;\nif w < 5 kg { move r to bench; } else { move r to bench; }")
    base = ir_document(plan)
    base["tasks"][0]["steps"][1]["condition"]["right"]["unit"] = "lb"
    assert problems(base) == [("I117", "/tasks/0/steps/1/condition/right/unit")]
    base = ir_document(plan)
    base["tasks"][0]["steps"][1]["condition"]["right"] = {"kind": "live", "reading": "mass", "object": "pan",
                                                          "sensor": "probe"}
    assert problems(base) == [("I114", "/tasks/0/steps/1/condition/right/kind")]
    base = ir_document(plan)
    base["tasks"][0]["steps"][1]["condition"]["left"] = {"kind": "quantity", "value": "1", "unit": "kg"}
    assert problems(base) == [("I114", "/tasks/0/steps/1/condition/left/kind")]
    base = ir_document(plan)
    base["tasks"][0]["steps"][1]["condition"]["right"]["value"] = "-5"
    assert problems(base) == [("I116", "/tasks/0/steps/1/condition/right/value")]
    base = ir_document(plan)
    base["tasks"][0]["preconditions"] = [copy.deepcopy(base["tasks"][0]["steps"][1]["condition"])]
    assert problems(base) == [("I222", "/tasks/0/preconditions/0")]


def test_a_schema_1_5_task_without_a_goal_key_is_i209_not_a_crash():
    document = ir_document()
    del document["tasks"][0]["goal"]
    assert problems(document) == [("I209", "/tasks/0")]


def test_the_reader_applies_the_binding_rules_to_retries_and_handlers():
    retry = ir_document("measure r mass of pan using probe as w else { measure r mass of pan using probe as w; }\n"
                        'communicate r to "ops" with w;')
    assert read_ir(json.dumps(retry).encode()) is not None
    # the compiler refuses a handler that does not rebind, so take the retry away from valid IR
    leaking = ir_document("measure r mass of pan using probe as w else { measure r mass of pan using probe as w; }\n"
                          'communicate r to "ops" with w;')
    leaking["tasks"][0]["steps"][0]["else"][0] = {"kind": "move", "robot": "r", "target": "bench"}
    assert problems(leaking) == [("I218", "/tasks/0/steps/1/binding")]


# -------- world file 1.2 (V9) --------

def world_error(patch=None, events=None, version="1.2") -> WorldError:
    plan = compile_source(task("move r to bench;")).ir.tasks[0]
    with pytest.raises(WorldError) as caught:
        load_world(world_data(patch, events, version=version), "w.json", plan)
    return caught.value


def without_values(world: dict):
    """A world patch: take away everything world file 1.2 adds."""
    for sensor in world["sensors"].values():
        sensor.pop("measures", None)
    for obj in world["objects"].values():
        obj.pop("properties", None)


def test_worlds_1_0_and_1_1_are_still_accepted_and_do_not_know_the_new_fields():
    plan = compile_source(task("move r to bench;")).ir.tasks[0]
    for version in ("1.0", "1.1"):
        assert load_world(world_data(without_values, version=version), "w.json", plan).schema_version == version
        assert world_error(version=version).code == "E503"  # `measures` / `properties` are unknown there
    assert world_error(events=[{"at": {"value": "1", "unit": "s"}, "fact": "property", "target": "pan1",
                                "property": "mass", "value": {"value": "1", "unit": "kg"}}], version="1.1").code \
        == "E503"


@pytest.mark.parametrize("patch, code, pointer", [
    (lambda w: w["sensors"]["probe1"]["measures"].update(colour={"contact": True}), "E512",
     "/sensors/probe1/measures/colour"),
    (lambda w: w["objects"]["pan1"]["properties"].update(colour={"value": "red"}), "E512",
     "/objects/pan1/properties/colour"),
    (lambda w: w["sensors"]["probe1"]["measures"]["temperature"].update(unit="kg"), "E513",
     "/sensors/probe1/measures/temperature/unit"),
    (lambda w: w["objects"]["pan1"]["properties"]["mass"].update(unit="degC"), "E513",
     "/objects/pan1/properties/mass/unit"),
    (lambda w: w["sensors"]["probe1"]["measures"]["label"].update(unit="g"), "E514",
     "/sensors/probe1/measures/label/unit"),
    (lambda w: w["sensors"]["probe1"]["measures"]["label"].update(uncertainty="1"), "E514",
     "/sensors/probe1/measures/label/uncertainty"),
    (lambda w: w["objects"]["pan1"]["properties"]["label"].update(unit="g"), "E514",
     "/objects/pan1/properties/label/unit"),
    (lambda w: w["sensors"]["probe1"]["measures"]["temperature"].pop("contact"), "E504",
     "/sensors/probe1/measures/temperature"),
    (lambda w: w["sensors"]["probe1"]["measures"]["temperature"].pop("unit"), "E504",
     "/sensors/probe1/measures/temperature"),
    (lambda w: w["sensors"]["probe1"]["measures"]["temperature"].update(uncertainty="-1"), "E504",
     "/sensors/probe1/measures/temperature/uncertainty"),
    (lambda w: w["sensors"]["probe1"]["measures"]["temperature"].update(contact="yes"), "E504",
     "/sensors/probe1/measures/temperature/contact"),
    (lambda w: w["sensors"]["probe1"]["measures"]["temperature"].update(extra=1), "E503",
     "/sensors/probe1/measures/temperature/extra"),
    (lambda w: w["objects"]["pan1"]["properties"]["temperature"].update(value="warm"), "E504",
     "/objects/pan1/properties/temperature/value"),
    (lambda w: w["objects"]["pan1"]["properties"]["label"].update(value=""), "E504",
     "/objects/pan1/properties/label/value"),
    (lambda w: w["objects"]["pan1"]["properties"]["label"].update(value=5), "E504",
     "/objects/pan1/properties/label/value"),
    (lambda w: w["objects"]["pan1"].update(properties=[]), "E504", "/objects/pan1/properties"),
    (lambda w: w["sensors"]["probe1"].update(measures=[]), "E504", "/sensors/probe1/measures"),
])
def test_world_file_1_2_is_validated_strictly(patch, code, pointer):
    error = world_error(patch)
    assert (error.code, error.pointer) == (code, pointer)


@pytest.mark.parametrize("event, code, pointer", [
    ({"at": {"value": "1", "unit": "s"}, "fact": "property", "target": "nowhere", "property": "mass",
      "value": {"value": "1", "unit": "kg"}}, "E515", "/events/0/target"),
    ({"at": {"value": "1", "unit": "s"}, "fact": "property", "target": "arm1", "property": "mass",
      "value": {"value": "1", "unit": "kg"}}, "E511", "/events/0/target"),
    ({"at": {"value": "1", "unit": "s"}, "fact": "property", "target": "pan1", "property": "colour",
      "value": {"value": "1", "unit": "kg"}}, "E512", "/events/0/property"),
    ({"at": {"value": "1", "unit": "s"}, "fact": "property", "target": "pan1", "property": "mass",
      "value": {"value": "1", "unit": "degC"}}, "E513", "/events/0/value/unit"),
    ({"at": {"value": "1", "unit": "s"}, "fact": "property", "target": "pan1", "property": "label",
      "value": {"value": "x", "unit": "g"}}, "E514", "/events/0/value/unit"),
    ({"at": {"value": "1", "unit": "s"}, "fact": "property", "target": "pan1",
      "value": {"value": "1", "unit": "kg"}}, "E504", "/events/0"),
    ({"at": {"value": "1", "unit": "s"}, "fact": "robot_ready", "target": "arm1", "property": "mass",
      "value": True}, "E503", "/events/0/property"),
])
def test_property_events_are_validated(event, code, pointer):
    error = world_error(events=[event])
    assert (error.code, error.pointer) == (code, pointer)


def test_world_values_may_be_below_zero_and_sensors_may_declare_zero_uncertainty():
    def patch(world):
        world["objects"]["pan1"]["properties"]["temperature"] = {"value": "-18", "unit": "degC"}
        world["sensors"]["probe1"]["measures"]["temperature"]["uncertainty"] = "0"
    result, record = simulate(task("measure r temperature of pan using probe as t;"), patch)
    assert record["steps"][0]["reading"]["value"] == "-18" and record["steps"][0]["reading"]["uncertainty"] == "0"
    assert result.state == "SUCCEEDED"


# -------- measure in simulation (V2, V9, V10) --------

def test_a_reading_is_recorded_and_the_run_record_is_1_5():
    source = task("measure r temperature of pan using probe as t;\nmeasure r label of pan using probe as lot;")
    result, record = simulate(source)
    assert result.state == "SUCCEEDED"
    assert (record["run_record_schema_version"], record["inputs"]["language_version"],
            record["inputs"]["ir_schema_version"], record["inputs"]["world_schema_version"]) == (
        "1.5", "1.6", "1.5", "1.2")
    first, second = record["steps"]
    assert first["reading"] == {"subject": "pan1", "kind": "temperature", "value": "72", "unit": "degC",
                                "uncertainty": "0.5", "sensor": "probe1", "t_ms": "0"}
    assert list(first)[list(first).index("observation") + 1] == "reading"
    assert second["reading"] == {"subject": "pan1", "kind": "label", "value": "LOT-7731", "sensor": "probe1",
                                 "t_ms": "0"}  # a label has no unit and no uncertainty
    assert first["observation"] is None and first["message"] is None and first["status"] == "succeeded"
    assert record["steps"][0]["step"]["binding"] == "t"
    assert record["final_world"]["robots"]["arm1"] == {"ready": True, "location": "bench", "holding": None}


def test_the_reading_is_in_the_sensors_unit_converted_exactly():
    def patch(world):
        world["objects"]["pan1"]["properties"]["temperature"] = {"value": "60", "unit": "degC"}
        world["objects"]["pan1"]["properties"]["distance"] = {"value": "2.5", "unit": "m"}
        world["sensors"]["probe1"]["measures"]["temperature"]["unit"] = "K"
    _, record = simulate(task("measure r temperature of pan using probe as t;\n"
                              "measure r distance of pan using probe as d;\nmeasure r mass of pan using probe as w;"),
                         patch)
    readings = [s["reading"] for s in record["steps"]]
    assert [(r["kind"], r["value"], r["unit"]) for r in readings] == [
        ("temperature", "333.15", "K"), ("distance", "2500", "mm"), ("mass", "1.5", "kg")]


def failing(plan: str, patch=None, events=None, entities=None):
    """Run `plan` where the measure at step 1 has a handler that records the step's failure code."""
    source = task(plan)
    return simulate(source, patch, events, entities=entities)


@pytest.mark.parametrize("measure, patch, code", [
    ("measure r mass of pan using loose as w;", None, "S231"),  # the sensor is declared, but not on any robot
    ("measure m mass of pan using probe as w;", None, "S231"),  # mounted on arm1, not on arm2
    ("measure r temperature of pan using dead as w;", None, "S232"),
    ("measure r temperature of pan using maybe as w;", None, "S233"),
    ("measure r temperature of pan using blind as w;", None, "S234"),  # declares nothing
    ("measure r distance of pan using probe as w;", lambda w: w["sensors"]["probe1"]["measures"].pop("distance"),
     "S234"),
    ("measure r mass of ghost using probe as w;", None, "S213"),
    ("measure r mass of vague using probe as w;", None, "S214"),
    ("measure r temperature of pan using probe as w;", lambda w: w["robots"]["arm1"].update(location="home"), "S235"),
    ("measure r label of pan2 using probe as w;",
     lambda w: w["objects"]["pan2"]["properties"].pop("label"), "S236"),
    ("measure r mass of pan2 using probe as w;", None, "S236"),  # pan2 has no mass property
])
def test_a_measure_that_cannot_read_fails_the_step_and_changes_nothing(measure, patch, code):
    entities = dict(ENTITY, pan2="pan2")
    goal = "goal { robot_at m home; }"  # true whatever the patch does to arm1
    _, plain = simulate(task(measure, goal=goal), patch)
    assert failure(plain) == (code, 1) and plain["state"] == "FAILED"
    step = plain["steps"][0]
    assert step["status"] == "failed" and step["reading"] is None
    _, handled = simulate(task(measure.replace(";", ' else { communicate r to "ops" with "no reading"; }'),
                               goal=goal), patch, entities=entities)
    assert handled["state"] == "SUCCEEDED" and handled["steps"][0]["handled"] == code
    assert handled["steps"][1]["message"] == {"robot": "arm1", "channel": "ops", "message": "no reading",
                                              "status": "recorded", "value": None}
    assert handled["steps"][0]["reading"] is None
    assert handled["final_world"] == plain["final_world"]  # nothing changed in either run


def test_the_order_of_the_measure_checks_is_the_order_of_the_specification():
    # a sensor that is unavailable, does not measure the kind, on an absent object, out of contact: S232 first
    def patch(world):
        world["robots"]["arm1"]["location"] = "home"
    assert failure(simulate(task("measure r mass of ghost using dead as w;"), patch)[1]) == ("S232", 1)
    assert failure(simulate(task("measure r mass of ghost using blind as w;"), patch)[1]) == ("S234", 1)
    assert failure(simulate(task("measure r mass of ghost using probe as w;"), patch)[1]) == ("S213", 1)
    assert failure(simulate(task("measure r temperature of pan using probe as w;"), patch)[1]) == ("S235", 1)


def test_a_non_contact_reading_works_from_anywhere_and_measure_needs_no_ready_robot():
    def patch(world):
        world["robots"]["arm1"].update(location="home", ready=False)
    _, record = simulate(task("measure r label of pan using probe as lot;", goal="goal { robot_at m home; }"), patch)
    assert record["state"] == "SUCCEEDED" and record["steps"][0]["reading"]["value"] == "LOT-7731"


def test_an_object_the_robot_holds_is_at_the_robot_for_a_contact_reading():
    _, record = simulate(task("grasp r pan;\nmove r to home;\nmeasure r temperature of pan using probe as t;\n"
                              "place r pan at home;", goal="goal { object_at pan home; }"))
    assert record["state"] == "SUCCEEDED" and record["steps"][2]["reading"]["value"] == "72"
    _, other = simulate(task("grasp m pan;\nmeasure r temperature of pan using probe as t;",
                             goal="goal { robot_at r bench; }"), lambda w: w["robots"]["arm2"].update(location="bench"))
    assert failure(other) == ("S235", 2)  # held by another robot: not at this one


def test_a_scripted_fault_on_a_measure_is_s251_and_a_handler_takes_it():
    source = task('measure r mass of pan using probe as w else { measure r mass of pan using probe as w; }\n'
                  'communicate r to "ops" with w;')
    _, record = simulate(source, faults=[1])
    assert record["state"] == "SUCCEEDED"
    assert [(s["path"], s["status"], s["handled"]) for s in record["steps"][:2]] == [("1", "failed", "S251"),
                                                                                    ("1.else.1", "succeeded", None)]
    assert record["steps"][2]["message"]["value"]["kind"] == "mass"
    _, both = simulate(source, faults=[1, 2])
    assert failure(both) == ("S251", 2)


def test_a_property_event_changes_what_a_later_measure_reads():
    events = [{"at": {"value": "5", "unit": "s"}, "fact": "property", "target": "pan1", "property": "temperature",
               "value": {"value": "30", "unit": "degC"}}]
    _, record = simulate(task("measure r temperature of pan using probe as a;\nwait for 5 s;\n"
                              "measure r temperature of pan using probe as b;"), events=events)
    assert [s["reading"]["value"] for s in (record["steps"][0], record["steps"][2])] == ["72", "30"]
    assert record["steps"][2]["reading"]["t_ms"] == "5000"
    assert record["events_applied"] == [{"t_ms": "5000", "fact": "property", "target": "pan1",
                                         "property": "temperature", "value": {"value": "30", "unit": "degC"}}]


def test_a_label_property_event_is_recorded_without_a_unit():
    events = [{"at": {"value": "1", "unit": "s"}, "fact": "property", "target": "pan1", "property": "label",
               "value": {"value": "LOT-9"}}]
    _, record = simulate(task("wait for 2 s;\nmeasure r label of pan using probe as lot;"), events=events)
    assert record["events_applied"][0]["value"] == {"value": "LOT-9"}
    assert record["steps"][1]["reading"]["value"] == "LOT-9"


# -------- uncertainty (V7) --------

COMPARE = ('measure r temperature of pan using probe as t;\n'
           'if t {op} 60 degC {{ communicate r to "ops" with "yes"; }} else {{ communicate r to "ops" with "no"; }}')


@pytest.mark.parametrize("op, value, uncertainty, expected", [
    ("<", "59", "0.5", "true"), ("<", "59.5", "0.5", "true"),  # v + u = 60 is not below 60...
    ("<", "60.5", "0.5", "false"), ("<", "61", "0.5", "false"), ("<", "59.8", "0.5", "unknown"),
    ("<=", "59.5", "0.5", "true"), ("<=", "60.5", "0.5", "unknown"), ("<=", "60.6", "0.5", "false"),
    (">", "60.6", "0.5", "true"), (">", "60.5", "0.5", "unknown"), (">", "59.5", "0.5", "false"),
    (">=", "60.5", "0.5", "true"), (">=", "59.8", "0.5", "unknown"), (">=", "59.4", "0.5", "false"),
    ("<", "59.99", "0", "true"), ("<", "60", "0", "false"), ("<=", "60", "0", "true"), (">", "60", "0", "false"),
    (">=", "60", "0", "true"),  # no uncertainty: the ordinary comparison, never unknown
])
def test_a_comparison_is_true_false_or_unknown_by_the_interval(op, value, uncertainty, expected):
    if (op, value, uncertainty) == ("<", "59.5", "0.5"):
        expected = "unknown"  # 59.5 + 0.5 = 60 is not below 60: not true; 59.5 - 0.5 < 60: not false
    result, record = simulate(task(COMPARE.format(op=op)), measuring(value, uncertainty))
    assert decision(record) == expected
    if expected == "unknown":
        assert failure(record) == ("S261", 2) and record["decisions"][0]["branch"] == "none"
        assert [s["status"] for s in record["steps"][2:]] == ["not_run", "not_run"]
    else:
        assert record["state"] == "SUCCEEDED"
        message = next(s["message"]["message"] for s in record["steps"] if s["message"])
        assert message == ("yes" if expected == "true" else "no")


def test_59_8_plus_or_minus_0_5_against_60_degc_fails_the_run_s261():
    _, record = simulate(task(COMPARE.format(op="<")), measuring("59.8", "0.5"))
    assert failure(record) == ("S261", 2) and record["state"] == "FAILED"
    assert record["decisions"] == [{"step": "2", "condition_result": "unknown", "branch": "none", "t_ms": "0"}]


def test_kelvin_against_celsius_compares_exactly():
    def patch(world):
        world["objects"]["pan1"]["properties"]["temperature"] = {"value": "60", "unit": "degC"}
        world["sensors"]["probe1"]["measures"]["temperature"].update(unit="K", uncertainty="0")
    for condition, expected in (("t < 60 degC", "false"), ("t >= 60 degC", "true"), ("t < 333.16 K", "true"),
                                ("t > 333.14 K", "true"), ("t <= 333.15 K", "true"), ("t > 333.15 K", "false")):
        plan = (f'measure r temperature of pan using probe as t;\nif {condition} {{ move r to bench; }} else '
                "{ move r to bench; }")
        assert decision(simulate(task(plan), patch)[1]) == expected, condition


def test_units_convert_exactly_within_a_kind_1500_g_against_1_2_kg_and_2_kg():
    def patch(world):
        world["sensors"]["probe1"]["measures"]["mass"].update(unit="kg", uncertainty="0.01")
    results = {}
    for condition in ("w > 1.2 kg", "w < 2 kg", "w > 2 kg", "w >= 1500 g", "w < 1500 g", "w > 1000 g", "w < 1.6 kg",
                      "w <= 1.51 kg"):
        plan = (f'measure r mass of pan using probe as w;\nif {condition} {{ move r to bench; }} else '
                "{ move r to bench; }")
        results[condition] = decision(simulate(task(plan), patch)[1])
    assert results == {"w > 1.2 kg": "true", "w < 2 kg": "true", "w > 2 kg": "false", "w >= 1500 g": "unknown",
                       "w < 1500 g": "unknown", "w > 1000 g": "true", "w < 1.6 kg": "true",
                       "w <= 1.51 kg": "true"}


def test_a_sensor_in_grams_and_a_literal_in_kilograms_agree():
    def patch(world):
        world["sensors"]["probe1"]["measures"]["mass"].update(unit="g", uncertainty="5")
    plan = 'measure r mass of pan using probe as w;\nif w > 1.2 kg and w < 2 kg { move r to bench; } else { fail "x"; }'
    result, record = simulate(task(plan), patch)
    assert record["state"] == "SUCCEEDED" and record["steps"][0]["reading"]["unit"] == "g"
    assert record["steps"][0]["reading"]["value"] == "1500" and record["steps"][0]["reading"]["uncertainty"] == "5"


def test_labels_are_exact_never_unknown():
    for condition, expected in (('lot == "LOT-7731"', "true"), ('lot == "LOT-0001"', "false"),
                                ('lot != "LOT-7731"', "false"), ('lot != "LOT-0001"', "true"),
                                ('lot == "lot-7731"', "false")):
        plan = (f'measure r label of pan using probe as lot;\nif {condition} {{ move r to bench; }} else '
                "{ move r to bench; }")
        assert decision(simulate(task(plan))[1]) == expected, condition


def test_two_bindings_compare_as_intervals_and_two_labels_compare_exactly():
    both = ("measure r temperature of pan using probe as a;\nmeasure r temperature of pan2 using probe as b;\n"
            "if a {op} b {{ move r to bench; }} else {{ move r to bench; }}")
    # pan1 72 +- 0.5 and pan2 66 +- 0.5: the intervals are apart
    for op, expected in (("<", "false"), ("<=", "false"), (">", "true"), (">=", "true")):
        assert decision(simulate(task(both.format(op=op)))[1]) == expected, op

    def close(world):
        world["objects"]["pan2"]["properties"]["temperature"] = {"value": "72.5", "unit": "degC"}
    for op, expected in (("<", "unknown"), ("<=", "unknown"), (">", "unknown"), (">=", "unknown")):
        _, record = simulate(task(both.format(op=op)), close)
        assert decision(record) == expected and failure(record) == ("S261", 3), op

    def touching(world):
        world["objects"]["pan2"]["properties"]["temperature"] = {"value": "73", "unit": "degC"}
    assert decision(simulate(task(both.format(op="<=")), touching)[1]) == "true"  # 72.5 <= 72.5 for every pair
    assert decision(simulate(task(both.format(op="<")), touching)[1]) == "unknown"  # 72.5 < 72.5 is not
    labels = ('measure r label of pan using probe as a;\nmeasure r label of pan2 using probe as b;\n'
              'if a == b { fail "same"; } else { move r to bench; }')
    result, record = simulate(task(labels))
    assert record["state"] == "SUCCEEDED" and decision(record) == "false"


def test_an_unknown_comparison_combines_by_the_three_valued_rules():
    unknown = "t < 60 degC"
    base = "measure r temperature of pan using probe as t;\nif {} {{ move r to bench; }} else {{ fail \"no\"; }}"
    patch = measuring("59.8", "0.5")
    assert failure(simulate(task(base.format(unknown)), patch)[1]) == ("S261", 2)
    assert failure(simulate(task(base.format(f"not {unknown}")), patch)[1]) == ("S261", 2)
    assert failure(simulate(task(base.format(f"{unknown} and object_exists pan")), patch)[1]) == ("S261", 2)
    assert decision(simulate(task(base.format(f"{unknown} and not object_exists pan")), patch)[1]) == "false"
    assert decision(simulate(task(base.format(f"{unknown} or object_exists pan")), patch)[1]) == "true"
    assert failure(simulate(task(base.format(f"{unknown} or not object_exists pan")), patch)[1]) == ("S261", 2)


# -------- live readings in wait until (V4) --------

WAIT = ('wait until temperature of pan using probe < 60 degC timeout 10 min else { fail "still hot"; }\n'
        'communicate r to "ops" with "cool";')
COOLS = [{"at": {"value": "4", "unit": "min"}, "fact": "property", "target": "pan1", "property": "temperature",
          "value": {"value": "55", "unit": "degC"}}]


def test_a_wait_succeeds_when_the_pan_cools_at_4_minutes_at_240000_ms():
    result, record = simulate(task(WAIT), events=COOLS)
    wait = record["steps"][0]
    assert record["state"] == "SUCCEEDED" and wait["status"] == "succeeded" and wait["handled"] is None
    assert (wait["t_start_ms"], wait["t_end_ms"], wait["condition_result"]) == ("0", "240000", "true")
    assert record["end_t_ms"] == "240000" and record["steps"][2]["message"]["message"] == "cool"


def test_a_wait_that_never_cools_runs_its_handler_after_the_timeout():
    _, record = simulate(task(WAIT))
    assert record["state"] == "FAILED" and failure(record) == ("S262", 2)
    assert record["failure"]["reason"] == "still hot"
    wait = record["steps"][0]
    assert (wait["status"], wait["handled"], wait["t_end_ms"]) == ("failed", "S241", "600000")
    assert wait["condition_result"] == "false"  # the last reading, 72 degC, is known to be above 60


def test_a_handled_wait_lets_the_plan_go_on():
    plan = ('wait until temperature of pan using probe < 60 degC timeout 1 min else { communicate r to "ops" with '
            '"still hot"; }\nmove r to bench;')
    _, record = simulate(task(plan))
    assert record["state"] == "SUCCEEDED" and record["steps"][0]["handled"] == "S241"
    assert record["steps"][1]["message"]["message"] == "still hot" and record["end_t_ms"] == "60000"


def test_a_live_reading_is_read_at_every_evaluation_not_once():
    # 72 at the start, 50 at 1 min, back to 70 at 2 min: the wait sees 50 at 1 minute and ends there
    events = [{"at": {"value": "1", "unit": "min"}, "fact": "property", "target": "pan1", "property": "temperature",
               "value": {"value": "50", "unit": "degC"}},
              {"at": {"value": "2", "unit": "min"}, "fact": "property", "target": "pan1", "property": "temperature",
               "value": {"value": "70", "unit": "degC"}}]
    _, record = simulate(task(WAIT), events=events)
    assert record["steps"][0]["t_end_ms"] == "60000" and record["state"] == "SUCCEEDED"
    # a measure taken at the start does not follow the world; the live reading does
    plan = ("measure r temperature of pan using probe as then;\n"
            'wait until temperature of pan using probe < 60 degC timeout 5 min else { fail "x"; }\n'
            'if then < 60 degC { fail "stale reading used"; } else { communicate r to "ops" with then; }')
    _, record = simulate(task(plan, goal="goal { robot_at r bench; }"), events=events[:1])
    assert record["state"] == "SUCCEEDED" and record["steps"][-1]["message"]["value"]["value"] == "72"


def test_a_live_reading_that_cannot_be_taken_is_unknown_and_the_wait_keeps_waiting():
    cool = {"at": {"value": "4", "unit": "min"}, "fact": "property", "target": "pan1", "property": "temperature",
            "value": {"value": "55", "unit": "degC"}}
    online = {"at": {"value": "30", "unit": "s"}, "fact": "sensor_available", "target": "dead", "value": True}

    def wait(sensor, patch=None, events=(cool,)):
        plan = (f'wait until temperature of pan using {sensor} < 60 degC timeout 10 min else {{ fail "x"; }}')
        return simulate(task(plan), patch, events=list(events))[1]
    assert wait("dead", events=(cool,))["state"] == "FAILED"  # never available
    record = wait("dead", events=(online, cool))
    assert record["state"] == "SUCCEEDED" and record["steps"][0]["t_end_ms"] == "240000"
    assert wait("blind")["state"] == "FAILED" and wait("maybe")["state"] == "FAILED"  # does not measure / unknown
    assert wait("probe", lambda w: (w["objects"]["pan1"].pop("location"),
                                    w["objects"]["pan1"].update(present=False)))["state"] == "FAILED"
    # a contact sensor with the robot elsewhere cannot be read; moving it there (an event cannot) is a step's job
    far = wait("probe", lambda w: w["robots"]["arm1"].update(location="home"))
    assert far["state"] == "FAILED" and far["steps"][0]["condition_result"] == "unknown"
    # a contact sensor on no robot cannot be read either, a non-contact one can
    assert wait("loose")["state"] == "FAILED"
    assert wait("loose", lambda w: w["sensors"]["loose"]["measures"]["temperature"].update(contact=False))[
        "state"] == "SUCCEEDED"


def test_live_readings_in_a_wait_combine_with_other_conditions():
    plan = ('wait until temperature of pan using probe < 60 degC and region_clear zone timeout 10 min else '
            '{ fail "x"; }')
    source = task(plan, params=PARAMS + ", region zone")

    def patch(w):
        w["regions"] = {"zone": {"clear": True}}
    entities = dict(ENTITY, zone="zone")
    _, record = simulate(source, patch, events=COOLS, entities=entities)
    assert record["state"] == "SUCCEEDED" and record["steps"][0]["t_end_ms"] == "240000"


# -------- retry (V3 b) --------

RETRY = ('measure r temperature of pan using probe as t else {\n measure r temperature of pan using probe as t else {\n'
         '  fail "no reading";\n }\n}\ncommunicate r to "ops" with t;')


def test_a_retried_reading_is_defined_on_both_paths_and_the_message_carries_it():
    _, first = simulate(task(RETRY))
    assert first["state"] == "SUCCEEDED" and first["steps"][0]["handled"] is None
    assert first["steps"][-1]["message"]["value"]["value"] == "72"
    _, second = simulate(task(RETRY), faults=[1])
    assert second["state"] == "SUCCEEDED" and second["steps"][0]["handled"] == "S251"
    assert second["steps"][1]["reading"]["value"] == "72" and second["steps"][-1]["message"]["value"]["kind"] == \
        "temperature"
    _, neither = simulate(task(RETRY), faults=[1, 2])
    assert failure(neither) == ("S262", 3) and neither["failure"]["reason"] == "no reading"
    assert all(s["message"] is None for s in neither["steps"])


def test_a_retry_that_reads_again_later_reads_the_world_at_that_time():
    events = [{"at": {"value": "5", "unit": "s"}, "fact": "property", "target": "pan1", "property": "temperature",
               "value": {"value": "30", "unit": "degC"}}]
    plan = ('measure r temperature of pan using dead as t else {\n wait for 10 s;\n'
            ' measure r temperature of pan using probe as t;\n}\ncommunicate r to "ops" with t;')
    _, record = simulate(task(plan), events=events)
    assert record["state"] == "SUCCEEDED" and record["steps"][0]["handled"] == "S232"
    assert record["steps"][2]["reading"]["value"] == "30" and record["steps"][1]["step"]["kind"] == "wait_for"
    assert record["steps"][-1]["message"]["value"] == record["steps"][2]["reading"]


# -------- typed messages (V8) --------

def test_a_message_carries_the_typed_value_and_a_text_message_has_value_null():
    plan = ('measure r temperature of pan using probe as t;\nmeasure r label of pan using probe as lot;\n'
            'communicate r to "ops" with t;\ncommunicate r to "ops" with lot;\ncommunicate r to "ops" with "text";')
    _, record = simulate(task(plan))
    messages = [s["message"] for s in record["steps"][2:]]
    assert messages[0] == {"robot": "arm1", "channel": "ops", "message": None, "status": "recorded",
                           "value": {"subject": "pan1", "kind": "temperature", "value": "72", "unit": "degC",
                                     "uncertainty": "0.5", "sensor": "probe1", "t_ms": "0"}}
    assert messages[1]["value"] == {"subject": "pan1", "kind": "label", "value": "LOT-7731", "sensor": "probe1",
                                    "t_ms": "0"}
    assert messages[2] == {"robot": "arm1", "channel": "ops", "message": "text", "status": "recorded", "value": None}
    assert record["steps"][2]["step"] == {"kind": "communicate", "robot": "r", "channel": "ops", "binding": "t"}


def test_the_value_sent_is_the_reading_taken_not_the_world_now():
    events = [{"at": {"value": "5", "unit": "s"}, "fact": "property", "target": "pan1", "property": "temperature",
               "value": {"value": "30", "unit": "degC"}}]
    plan = ('measure r temperature of pan using probe as t;\nwait for 10 s;\ncommunicate r to "ops" with t;')
    _, record = simulate(task(plan), events=events)
    value = record["steps"][2]["message"]["value"]
    assert (value["value"], value["t_ms"]) == ("72", "0") and record["steps"][2]["t_start_ms"] == "10000"


def test_the_record_is_deterministic():
    first = json.dumps(simulate(task(RETRY), faults=[1])[1], sort_keys=False)
    assert json.dumps(simulate(task(RETRY), faults=[1])[1], sort_keys=False) == first


# -------- older files are unchanged --------

def test_a_1_5_task_runs_identically_on_a_world_file_1_2_and_its_record_has_no_new_keys():
    source = task('move r to bench;\ninspect r pan using probe;\nwait for 1 s;\ncommunicate r to "ops" with "x";',
                  version="1.5")
    _, on_1_2 = simulate(source)
    _, on_1_1 = simulate(source, world=world_data(without_values, version="1.1"))
    assert on_1_2["inputs"].pop("world_schema_version") == "1.2"
    assert on_1_1["inputs"].pop("world_schema_version") == "1.1"
    assert on_1_2 == on_1_1
    assert on_1_2["run_record_schema_version"] == "1.4"
    step = on_1_2["steps"][-1]
    assert "reading" not in step and set(step["message"]) == {"robot", "channel", "message", "status"}
    assert list(on_1_2["steps"][0]) == ["index", "path", "step", "status", "handled", "t_start_ms", "t_end_ms",
                                        "observation", "message", "condition_result"]


@pytest.mark.parametrize("version, schema", [("1.1", "1.0"), ("1.2", "1.1"), ("1.3", "1.2"), ("1.4", "1.3"),
                                             ("1.5", "1.4")])
def test_earlier_languages_keep_their_ir_and_record_schemas(version, schema):
    goal = "" if version in ("1.1", "1.2") else "goal { robot_at r bench; }"
    source = task("move r to bench;", goal=goal, version=version, params="robot r, location bench")
    compiled = compile_source(source)
    assert compiled.ir.ir_schema_version == schema
    from robolang.sim.record import RUN_RECORD_SCHEMA_VERSIONS
    assert RUN_RECORD_SCHEMA_VERSIONS[version] == {"1.1": "1.0", "1.2": "1.1", "1.3": "1.2", "1.4": "1.3",
                                                   "1.5": "1.4"}[version]


def test_the_1_5_front_end_vocabulary_is_unchanged():
    assert codes(task("measure r mass of pan using probe as w;", version="1.5"))[0] == "E203"  # not a statement
    check(task("move r to bench;", version="1.5", params="robot r, location bench, location measure, location as"))


# -------- validate --world, inspect --------

def run_cli(args, cwd):
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(cwd)
    try:
        code = main(args, out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


def test_validate_world_reports_a_sensor_that_cannot_measure_the_kind_and_one_not_on_the_robot(tmp_path):
    plan = ("measure r temperature of pan using blind as a;\nmeasure r mass of pan using loose as b;\n"
            "measure m mass of pan using probe as c;\nmeasure r mass of pan using probe as d;")
    (tmp_path / "t.rob").write_text(task(plan), encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data())
    code, out, err = run_cli(["validate", "t.rob", "--world", "w.json"], tmp_path)
    lines = err.splitlines()
    assert code == 1
    assert [re.search(r"\[(S\d+)\]", line).group(1) for line in lines[:3]] == ["S234", "S231", "S231"]
    assert "t.rob:5:1: error[S234]" in lines[0] and "does not measure temperature" in lines[0]
    assert lines[3] == "t.rob: task 't' cannot succeed in w.json (3 errors)"


def test_validate_world_does_not_check_contact_or_live_readings_but_checks_inside_handlers(tmp_path):
    plan = ("measure r temperature of pan using probe as a;\n"
            'wait until temperature of pan using blind < 5 degC timeout 1 s else { fail "x"; }')
    (tmp_path / "t.rob").write_text(task(plan), encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data(lambda w: w["robots"]["arm1"].update(location="home")))
    code, out, err = run_cli(["validate", "t.rob", "--world", "w.json"], tmp_path)
    assert code == 0 and "no certain failure found" in out
    nested = task('move r to bench else { measure r mass of pan using blind as w; fail "x"; }')
    (tmp_path / "n.rob").write_text(nested, encoding="utf-8")
    code, out, err = run_cli(["validate", "n.rob", "--world", "w.json"], tmp_path)
    assert code == 1 and "error[S234]" in err and "(on path 1.else)" in err


def test_simulate_runs_a_1_6_task_and_prints_the_record(tmp_path):
    (tmp_path / "t.rob").write_text(task(WAIT), encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data(events=COOLS))
    code, out, err = run_cli(["simulate", "t.rob", "--world", "w.json"], tmp_path)
    assert code == 0, err
    record = json.loads(out)
    assert record["run_record_schema_version"] == "1.5" and record["inputs"]["world_schema_version"] == "1.2"
    assert record["steps"][0]["t_end_ms"] == "240000"


def test_inspect_prints_measure_compare_and_a_value_message(tmp_path):
    plan = ('measure r mass of pan using probe as w else { fail "x"; }\n'
            'if w < 2 kg and w >= 1500 g { communicate r to "ops" with w; } else { move r to bench; }\n'
            'wait until temperature of pan using probe < 60 degC and label of pan using probe == "A" timeout 1 min '
            'else { fail "y"; }')
    (tmp_path / "t.rob").write_text(task(plan), encoding="utf-8")
    code, out, err = run_cli(["inspect", "t.rob"], tmp_path)
    assert code == 0, err
    assert "1. measure r mass of pan using probe as w\n      if it fails:\n        1.1. fail \"x\"" in out
    assert "2. if w < 2 kg and w >= 1500 g" in out and "2.1. communicate r to \"ops\" with w" in out
    assert ('3. wait until temperature of pan using probe < 60 degC and label of pan using probe == "A" '
            "timeout 1 min") in out


def test_the_ir_input_path_reads_and_simulates_a_1_6_task(tmp_path):
    (tmp_path / "t.json").write_bytes(to_json(compile_source(task(WAIT)).ir).encode())
    (tmp_path / "w.json").write_bytes(world_data(events=COOLS))
    code, out, err = run_cli(["simulate", "--ir", "t.json", "--world", "w.json"], tmp_path)
    assert code == 0, err
    assert json.loads(out)["run_record_schema_version"] == "1.5"


# -------- elsewhere: export-goal, delegate, the ROS 2 backend --------

def test_goals_have_no_values_so_core_export_sees_only_the_world_facts():
    from robolang.bridge import goal_atoms
    plan = compile_source(task('measure r mass of pan using probe as w;\nif w < 2 kg { move r to bench; } else '
                               '{ move r to bench; }')).ir.tasks[0]
    assert [a.KIND for a in goal_atoms(plan)] == ["robot_at"]


def test_delegate_does_not_take_a_task_with_a_measure():
    from robolang import delegate
    plan = compile_source(task("move r to bench;\nmeasure r mass of pan using probe as w;")).ir.tasks[0]
    assert delegate._consider(None, plan, b"", "w.json", "arm1", (), "0.1") == ("measure_unsupported", None)
    nested = compile_source(task('move r to bench else { measure r mass of pan using probe as w; fail "x"; }')
                            ).ir.tasks[0]
    assert delegate._consider(None, nested, b"", "w.json", "arm1", (), "0.1") == ("measure_unsupported", None)
    assert "measure_unsupported" in (ROOT / "spec" / "delegate-v0.1.md").read_text(encoding="utf-8")


def test_the_ros2_backend_refuses_a_measure_with_s509_before_any_motion():
    pytest.importorskip("jsonschema")
    sys.path.insert(0, str(ROOT / "backends" / "ros2"))
    from robolang_ros2.executor import execute
    from test_ros2_executor import FakeOps, fixture
    _, bindings, site = fixture()
    source = ("robolang 1.6;\ntask transfer(robot r, object box, location src, location dst, sensor probe) {\n"
              "  goal { object_at box dst; }\n"
              "  plan { measure r mass of box using probe as w else { move r to src; } move r to src; "
              "grasp r box; move r to dst; place r box at dst; }\n}\n")
    ops = FakeOps()
    record = execute(compile_source(source).ir.tasks[0], dict(bindings, probe="probe1"), site, ops)
    assert record["failure"]["code"] == "S509" and ops.calls == [("guard", "ur")]


# -------- decision 0036 set 5: what 1.6 makes expressible --------

KITCHEN = json.loads((ROOT / "bench" / "worlds" / "kitchen_values.json").read_text(encoding="utf-8"))
PHARMACY = json.loads((ROOT / "bench" / "worlds" / "pharmacy.json").read_text(encoding="utf-8"))


def bench_world(base: dict, patch=None, events=None) -> dict:
    """A world 1.2 copy of a bench world (built here, bench/ is untouched): sensors that measure, values on objects."""
    world = copy.deepcopy(base)
    world["world_schema_version"] = "1.2"
    if patch is not None:
        patch(world)
    if events is not None:
        world["events"] = list(world["events"]) + events
    return world


def kitchen(patch=None, events=None, **values):
    def setup(world):
        world["sensors"]["probe_temp"]["measures"] = {"temperature": {"unit": "degC", "uncertainty": "0.5",
                                                                      "contact": True}}
        world["sensors"]["scale_1"]["measures"] = {"mass": {"unit": "kg", "uncertainty": "0.01", "contact": True}}
        for key, (kind, value, unit) in values.items():
            world["objects"][key].setdefault("properties", {})[kind] = {"value": value, "unit": unit}
        if patch:
            patch(world)
    return bench_world(KITCHEN, setup, events)


def pharmacy(patch=None, **values):
    def setup(world):
        world["sensors"]["scan_p1"]["measures"] = {"label": {"contact": False}}
        for key, value in values.items():
            world["objects"][key]["properties"] = {"label": {"value": value}}
        if patch:
            patch(world)
    return bench_world(PHARMACY, setup)


def bench_run(source: str, world: dict, bindings: dict, faults=()):
    compiled = compile_source(source)
    plan = compiled.ir.tasks[0]
    world = dict(world, tasks={plan.name: {"bindings": bindings,
                                           **({"faults": [{"step": n} for n in faults]} if faults else {})}})
    loaded = load_world(json.dumps(world).encode(), "w.json", plan)
    result = run(plan, loaded)
    record = to_record(result, task=plan, run_id="sim-x", source_sha256="0", world_sha256="0", seed="0",
                       language_version=compiled.ir.language_version, world_schema_version="1.2")
    return result, record


def where(record, obj: str, kind: str = "objects") -> str | None:
    return record["final_world"][kind][obj]["location"]


CHEF = {"r": "chef_arm", "pan": "pan_1", "probe": "probe_temp", "stove": "stove", "sink": "sink", "pass_": "pass"}

V01 = """robolang 1.6;
task probe_report(robot r, object pan, sensor probe, location stove, location sink) {
    goal { object_at pan sink; }
    plan {
        move r to stove;
        measure r temperature of pan using probe as t else { fail "no temperature reading"; }
        communicate r to "pass" with t;
        grasp r pan;
        move r to sink;
        place r pan at sink;
    }
}
"""


def test_v01_probe_report():
    bindings = {k: v for k, v in CHEF.items() if k not in ("pass_",)}
    _, record = bench_run(V01, kitchen(pan_1=("temperature", "45", "degC")), bindings)
    assert record["state"] == "SUCCEEDED" and where(record, "pan_1") == "sink"
    message = next(s["message"] for s in record["steps"] if s["message"])
    assert (message["channel"], message["message"]) == ("pass", None)
    assert message["value"] == {"subject": "pan_1", "kind": "temperature", "value": "45", "unit": "degC",
                                "uncertainty": "0.5", "sensor": "probe_temp", "t_ms": message["value"]["t_ms"]}
    _, gone = bench_run(V01, kitchen(lambda w: w["sensors"]["probe_temp"].update(available=False),
                                     pan_1=("temperature", "45", "degC")), bindings)
    assert failure(gone) == ("S262", 3) and where(gone, "pan_1") == "stove"  # no reading: stop, nothing moved


V02 = """robolang 1.6;
task cool_enough(robot r, object pan, sensor probe, location stove, location pass) {
    plan {
        move r to stove;
        measure r temperature of pan using probe as t else { fail "no temperature reading"; }
        if t < 60 degC {
            goal { object_at pan pass; }
            grasp r pan;
            move r to pass;
            place r pan at pass;
        } else {
            goal { object_at pan stove; }
            communicate r to "pass" with "too hot";
        }
    }
}
"""


def test_v02_cool_enough_on_each_of_the_three_probes():
    bindings = {"r": "chef_arm", "pan": "pan_1", "probe": "probe_temp", "stove": "stove", "pass": "pass"}
    _, cool = bench_run(V02, kitchen(pan_1=("temperature", "45", "degC")), bindings)
    assert cool["state"] == "SUCCEEDED" and where(cool, "pan_1") == "pass"
    assert [g["from"] for g in cool["goal"]] == ["3.then"] and not any(s["message"] for s in cool["steps"])
    _, hot = bench_run(V02, kitchen(pan_1=("temperature", "80", "degC")), bindings)
    assert hot["state"] == "SUCCEEDED" and where(hot, "pan_1") == "stove"
    assert [s["message"]["message"] for s in hot["steps"] if s["message"]] == ["too hot"]
    _, edge = bench_run(V02, kitchen(pan_1=("temperature", "59.8", "degC")), bindings)
    assert failure(edge) == ("S261", 4) and edge["state"] == "FAILED" and where(edge, "pan_1") == "stove"
    assert edge["decisions"][0]["condition_result"] == "unknown"


V03 = """robolang 1.6;
task wait_to_cool(robot r, object pan, sensor probe, location stove, location pass) {
    goal { object_at pan pass; }
    plan {
        move r to stove;
        wait until temperature of pan using probe < 60 degC timeout 10 min else {
            fail "pan did not cool in time";
        }
        grasp r pan;
        move r to pass;
        place r pan at pass;
    }
}
"""


def test_v03_wait_to_cool():
    bindings = {"r": "chef_arm", "pan": "pan_1", "probe": "probe_temp", "stove": "stove", "pass": "pass"}
    cools = [{"at": {"value": "4", "unit": "min"}, "fact": "property", "target": "pan_1", "property": "temperature",
              "value": {"value": "55", "unit": "degC"}}]
    world = kitchen(events=cools, pan_1=("temperature", "80", "degC"))
    _, cooled = bench_run(V03, world, bindings)
    assert cooled["state"] == "SUCCEEDED" and where(cooled, "pan_1") == "pass"
    assert cooled["steps"][1]["t_end_ms"] == "240000" and cooled["steps"][1]["handled"] is None
    _, never = bench_run(V03, kitchen(pan_1=("temperature", "80", "degC")), bindings)
    assert failure(never) == ("S262", 3) and never["failure"]["reason"] == "pan did not cool in time"
    assert never["steps"][1]["handled"] == "S241" and where(never, "pan_1") == "stove"
    assert never["end_t_ms"] == "600000"


V05 = """robolang 1.6;
task heavy_grams(robot r, object tray, sensor scale, location pass) {
    goal { object_at tray pass; }
    plan {
        measure r mass of tray using scale as w else { fail "no weight reading"; }
        if w > 1500 g {
            communicate r to "pass" with "heavy";
            fail "tray too heavy";
        } else {
            grasp r tray;
            move r to pass;
            place r tray at pass;
        }
    }
}
"""


def test_v05_heavy_grams_with_a_scale_that_reports_kilograms():
    bindings = {"r": "helper_arm", "tray": "tray_b", "scale": "scale_1", "pass": "pass"}
    _, light = bench_run(V05, kitchen(tray_b=("mass", "1.2", "kg")), bindings)
    assert light["state"] == "SUCCEEDED" and where(light, "tray_b") == "pass"
    assert light["steps"][0]["reading"]["unit"] == "kg"
    _, heavy = bench_run(V05, kitchen(tray_b=("mass", "2", "kg")), bindings)
    assert failure(heavy) == ("S262", 5) and heavy["failure"]["reason"] == "tray too heavy"
    assert [s["message"]["message"] for s in heavy["steps"] if s["message"]] == ["heavy"]
    assert where(heavy, "tray_b", "objects") == "prep_2"
    _, grams = bench_run(V05, kitchen(tray_b=("mass", "1800", "g")), bindings)  # 1800 g read as 1.8 kg > 1500 g
    assert failure(grams) == ("S262", 5)
    _, edge = bench_run(V05, kitchen(tray_b=("mass", "1.5", "kg")), bindings)  # 1.5 +- 0.01 kg against 1500 g
    assert failure(edge) == ("S261", 3)


V06 = """robolang 1.6;
task batch_gate(robot r, object vial, sensor scanner, location fridge, location packing) {
    goal { object_at vial packing; }
    plan {
        measure r label of vial using scanner as lot else { fail "no batch reading"; }
        if lot == "LOT-7731" {
            move r to fridge;
            grasp r vial;
            move r to packing;
            place r vial at packing;
        } else {
            fail "wrong batch";
        }
    }
}
"""


def test_v06_batch_gate():
    bindings = {"r": "picker1", "vial": "insulin_1", "scanner": "scan_p1", "fridge": "fridge_2", "packing": "packing"}
    _, good = bench_run(V06, pharmacy(insulin_1="LOT-7731"), bindings)
    assert good["state"] == "SUCCEEDED" and where(good, "insulin_1") == "packing"
    _, bad = bench_run(V06, pharmacy(insulin_1="LOT-0001"), bindings)
    assert failure(bad) == ("S262", 8) and bad["failure"]["reason"] == "wrong batch"
    assert where(bad, "insulin_1") == "fridge_2" and where(bad, "picker1", "robots") == "charger"


V10 = """robolang 1.6;
task no_reading_go_on(robot r, object pan, object bowl, sensor probe, location sink, location pantry, location prep) {
    goal { object_at bowl prep; }
    plan {
        move r to sink;
        measure r temperature of pan using probe as t else {
            communicate r to "pass" with "no reading";
        }
        move r to pantry;
        grasp r bowl;
        move r to prep;
        place r bowl at prep;
    }
}
"""


def test_v10_no_reading_go_on():
    bindings = {"r": "chef_arm", "pan": "pan_2", "bowl": "bowl_1", "probe": "probe_temp", "sink": "sink",
                "pantry": "pantry", "prep": "prep_1"}
    _, online = bench_run(V10, kitchen(pan_2=("temperature", "30", "degC")), bindings)
    assert online["state"] == "SUCCEEDED" and where(online, "bowl_1") == "prep_1"
    assert not any(s["message"] for s in online["steps"]) and online["steps"][1]["handled"] is None
    _, offline = bench_run(V10, kitchen(lambda w: w["sensors"]["probe_temp"].update(available=False),
                                        pan_2=("temperature", "30", "degC")), bindings)
    assert offline["state"] == "SUCCEEDED" and where(offline, "bowl_1") == "prep_1"
    assert offline["steps"][1]["handled"] == "S232"
    assert [s["message"]["message"] for s in offline["steps"] if s["message"]] == ["no reading"]
    _, empty = bench_run(V10, kitchen(), bindings)  # the pan has no temperature stated: nothing to read, carry on
    assert empty["state"] == "SUCCEEDED" and empty["steps"][1]["handled"] == "S236"


V11 = """robolang 1.6;
task report_after_branch(robot r, object pan, sensor probe, location stove, location pass) {
    plan {
        move r to stove;
        measure r temperature of pan using probe as t else { fail "no temperature reading"; }
        if t < 60 degC {
            goal { object_at pan pass; }
            grasp r pan;
            move r to pass;
            place r pan at pass;
            communicate r to "pass" with t;
        } else {
            goal { object_at pan stove; }
            communicate r to "pass" with t;
        }
    }
}
"""


def test_v11_report_the_reading_after_either_branch():
    bindings = {"r": "chef_arm", "pan": "pan_1", "probe": "probe_temp", "stove": "stove", "pass": "pass"}
    for value, place in (("45", "pass"), ("80", "stove")):
        _, record = bench_run(V11, kitchen(pan_1=("temperature", value, "degC")), bindings)
        sent = [s["message"] for s in record["steps"] if s["message"]]
        assert record["state"] == "SUCCEEDED" and where(record, "pan_1") == place
        assert len(sent) == 1 and sent[0]["channel"] == "pass" and sent[0]["value"]["value"] == value
        assert sent[0]["value"]["unit"] == "degC"


V13 = """robolang 1.6;
task relay_batch(robot reader, robot courier, object vial, sensor scanner) {
    goal { object_at vial fridge; }
    plan {
        measure reader label of vial using scanner as lot else { fail "no batch reading"; }
        communicate courier to "dispatch" with lot;
    }
}
""".replace("object_at vial fridge", "object_exists vial")


def test_v13_relay_batch_between_two_robots():
    bindings = {"reader": "picker1", "courier": "cart1", "vial": "insulin_1", "scanner": "scan_p1"}
    _, record = bench_run(V13, pharmacy(insulin_1="LOT-7731"), bindings)
    message = record["steps"][2]["message"]
    assert record["state"] == "SUCCEEDED" and (message["robot"], message["channel"]) == ("cart1", "dispatch")
    assert message["value"] == {"subject": "insulin_1", "kind": "label", "value": "LOT-7731", "sensor": "scan_p1",
                                "t_ms": "0"}  # cart1 sends what picker1 read, as a label: no unit, no uncertainty


V15 = """robolang 1.6;
task retry_reading(robot r, object pan, sensor probe, location stove) {
    goal { robot_at r stove; }
    plan {
        move r to stove;
        measure r temperature of pan using probe as t else {
            measure r temperature of pan using probe as t else {
                fail "no reading after two tries";
            }
        }
        communicate r to "pass" with t;
    }
}
"""


def test_v15_retry_reading():
    bindings = {"r": "chef_arm", "pan": "pan_1", "probe": "probe_temp", "stove": "stove"}
    world = kitchen(pan_1=("temperature", "45", "degC"))
    _, first = bench_run(V15, world, bindings)
    assert first["state"] == "SUCCEEDED" and first["steps"][-1]["message"]["value"]["value"] == "45"
    _, second = bench_run(V15, world, bindings, faults=[2])
    assert second["state"] == "SUCCEEDED" and second["steps"][1]["handled"] == "S251"
    assert second["steps"][-1]["message"]["value"]["value"] == "45"
    _, neither = bench_run(V15, world, bindings, faults=[2, 3])
    assert neither["state"] == "FAILED" and neither["failure"]["reason"] == "no reading after two tries"
    assert not any(s["message"] for s in neither["steps"])


@pytest.mark.parametrize("program", [
    'communicate r to "pass" with (t1 + t2) / 2;',
    'communicate r to "pass" with t1 + t2;',
    'measure r temperature of pan using probe as avg / 2;',
    'communicate r to "pass" with average;',
    'communicate r to "pass" with average(t1, t2);',
])
def test_x01_an_average_cannot_be_written(program):
    plan = ("move r to stove;\nmeasure r temperature of pan using probe as t1;\n"
            f"measure r temperature of pan2 using probe as t2;\n{program}")
    source = task(plan, params="robot r, object pan, object pan2, sensor probe, location stove")
    assert errors(source), program


@pytest.mark.parametrize("program", [
    'communicate r to "pass" with "Pan 1 is at " t " degC, ready";',
    'communicate r to "pass" with "Pan 1 is at " + t + " degC, ready";',
    'communicate r to "pass" with format("Pan 1 is at {} degC, ready", t);',
    'communicate r to "pass" with t, "degC";',
])
def test_x02_a_formatted_message_cannot_be_written(program):
    source = task(f"move r to stove;\nmeasure r temperature of pan using probe as t;\n{program}",
                  params="robot r, object pan, sensor probe, location stove")
    assert errors(source), program


def test_x02_braces_in_a_string_are_text_not_a_hole_for_the_reading():
    # `"... {t} ..."` is a valid message: it sends exactly those characters. The reading cannot be put in it.
    text = "Pan 1 is at {t} degC, ready"
    source = task(f'move r to stove;\nmeasure r temperature of pan using probe as t;\n'
                  f'communicate r to "pass" with "{text}";',
                  params="robot r, object pan, sensor probe, location stove", goal="goal { robot_at r stove; }")
    bindings = {"r": "chef_arm", "pan": "pan_1", "probe": "probe_temp", "stove": "stove"}
    _, record = bench_run(source, kitchen(pan_1=("temperature", "45", "degC")), bindings)
    message = record["steps"][2]["message"]
    assert message["message"] == text and message["value"] is None
