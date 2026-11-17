"""
RoboLang 1.8: declared actions (spec/robolang-v1.8.md, decision 0040 A1-A11 and C1-C15).

Front end, Task IR 1.7 and its reader, world file 1.4 (`actions`, `state`, `location_states`), the checks before the run
(`S006` to `S009`), `S271`, `in_state`, `validate --world`, run record 1.7, and the refusals in `export-goal`,
`delegate` and the ROS 2 backend. No test here needs a model or a network.

To regenerate the IR snapshots after an intended change:

    ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_lang18.py
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
FIXTURES = ROOT / "tests" / "fixtures" / "v1.8"
SNAPSHOTS = ROOT / "tests" / "snapshots" / "ir-v1.8"
UPDATE = os.environ.get("ROBOLANG_UPDATE_SNAPSHOTS") == "1"
SPEC = (ROOT / "spec" / "robolang-v1.8.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")

# -------- helpers --------

PARAMS = "robot r, robot m, object door, object kit, location ward, location closet, location fryer, region zone"
ENTITY = {"r": "moxi1", "m": "chef", "door": "closet_door", "kit": "kit", "ward": "ward", "closet": "closet",
          "fryer": "fryer", "zone": "zone"}
BASE_WORLD = {
    "world_schema": "robolang.sim.world",
    "world_schema_version": "1.4",
    "locations": ["ward", "closet", "fryer"],
    "regions": {"zone": {"clear": True, "contains": ["fryer"]}},
    "robots": {
        "moxi1": {"ready": True, "location": "closet", "gripper": True, "honours": ["no_go_region"],
                  "actions": {"open": {"target": "object", "result": "open"},
                              "close": {"target": "object", "result": "closed"},
                              "set": {"target": "location", "result": "heating", "value": "temperature"},
                              "tag": {"target": "object", "result": "labelled", "value": "label"},
                              "print": {"target": "location", "result": "printed"}}},
        "chef": {"ready": True, "location": "ward", "gripper": False},
    },
    "objects": {"closet_door": {"present": True, "location": "closet", "state": "closed"},
                "kit": {"present": True, "location": "closet"}},
    "location_states": {"fryer": "off"},
    "tasks": {},
}


def task(plan: str, goal: str = 'goal { in_state door "open"; }', pre: str = "", annotations: str = "",
         params: str = PARAMS, version: str = "1.8", name: str = "t") -> str:
    return f"robolang {version};\ntask {name}({params}) {{\n{annotations}\n{pre}\n{goal}\nplan {{\n{plan}\n}}\n}}\n"


def codes(source: str) -> list[str]:
    try:
        check(source)
    except RoboLangError as error:
        return [d.code for d in error.diagnostics]
    return []


def world_data(patch=None, version="1.4", name="t", bindings=None) -> bytes:
    world = copy.deepcopy(BASE_WORLD)
    world["world_schema_version"] = version
    if version != "1.4":  # an older world file has none of the 1.4 fields
        world.pop("location_states")
        for robot in world["robots"].values():
            robot.pop("actions", None)
        for obj in world["objects"].values():
            obj.pop("state", None)
        if version != "1.3":
            for region in world["regions"].values():
                region.pop("contains", None)
            for robot in world["robots"].values():
                robot.pop("honours", None)
    if patch is not None:
        patch(world)
    world["tasks"] = {name: {"bindings": bindings or dict(ENTITY)}}
    return json.dumps(world).encode()


def simulate(source: str, patch=None, version="1.4"):
    """Compile, run against a world file (1.4 by default), return (result, record)."""
    compiled = compile_source(source)
    plan = compiled.ir.tasks[0]
    bindings = {p.name: ENTITY[p.name] for p in plan.parameters}
    loaded = load_world(world_data(patch, version, plan.name, bindings), "w.json", plan)
    result = run(plan, loaded)
    record = to_record(result, task=plan, run_id="sim-x", source_sha256="0", world_sha256="0", seed="0",
                       language_version=compiled.ir.language_version, world_schema_version=loaded.schema_version)
    return result, record


def failure(record) -> tuple[str, int | None] | None:
    """(code, the 1-based step the failure names)."""
    return None if record["failure"] is None else (record["failure"]["code"], record["failure"]["step"])


def run_cli(args, cwd):
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(cwd)
    try:
        code = main(args, out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


# -------- front end (A1, A2, A4, A5, A6) --------

def test_version_1_8_selects_ir_schema_1_7_and_older_versions_keep_theirs():
    document = json.loads(to_json(compile_source(task("do r open door;")).ir))
    assert (document["language_version"], document["ir_schema_version"]) == ("1.8", "1.7")
    assert ir.IR_SCHEMA_VERSIONS["1.7"] == "1.6" and ir.IR_SCHEMA_VERSIONS["1.8"] == "1.7"


@pytest.mark.parametrize("version", ["1.3", "1.4", "1.5", "1.6", "1.7"])
def test_do_and_in_state_in_an_earlier_version_are_still_errors(version):
    assert codes(task("do r open door;", goal="goal { object_exists door; }", version=version)) == ["E203"]
    assert codes(task("wait for 1 s;", goal='goal { in_state door "open"; }', version=version)) == ["E204"]


def test_do_and_in_state_are_the_only_new_reserved_words():
    from robolang.tokens import RESERVED_BY_VERSION
    assert RESERVED_BY_VERSION["1.8"] - RESERVED_BY_VERSION["1.7"] == {"do", "in_state"}
    assert codes(task("do r open door;", params="robot r, object do")) == ["E212"]
    assert codes(task("do r open door;", params="robot r, object in_state")) == ["E212"]
    # the action is a name, not a word: any unreserved name parses, whether the robot declares it is the world's
    assert codes(task("do r polish_the_door door;")) == []
    assert codes(task("do r move door;")) == ["E212"]


def test_the_target_is_one_object_or_location_parameter_a4():
    assert codes(task("do r open door;")) == []
    assert codes(task("do r print fryer;")) == []
    assert codes(task("do r open zone;")) == ["E304"]
    assert codes(task("do r open m;")) == ["E304"]
    assert codes(task("do r open window;")) == ["E303"]
    assert codes(task("do window open door;")) == ["E303"]
    assert codes(task("do r open door kit;")) == ["E202"]  # several targets are not in 1.8 (A10)
    assert codes(task("do r open;")) == ["E201"]


def test_a_value_is_a_quantity_with_a_unit_or_a_string_a5():
    assert codes(task("do r set fryer to 175 degC;")) == []
    assert codes(task('do r tag door to "LOT-7";')) == []
    assert codes(task("do r set fryer to 175;")) == ["E206"]
    assert codes(task("do r set fryer to 175 furlong;")) == ["E206"]
    assert codes(task('do r tag door to "";')) == ["E307"]
    assert codes(task("do r set fryer to fryer;")) == ["E201"]
    assert codes(task("do r set fryer to;")) == ["E201"]


def test_in_state_takes_an_object_or_a_location_and_a_non_empty_label_a2():
    assert codes(task("do r open door;", goal='goal { in_state fryer "heating"; }')) == []
    assert codes(task("do r open door;", goal='goal { in_state r "ready"; }')) == ["E304"]
    assert codes(task("do r open door;", goal='goal { in_state door ""; }')) == ["E307"]
    assert codes(task("do r open door;", goal="goal { in_state door open; }")) == ["E201"]
    assert codes(task("do r open door;", goal="goal { in_state door; }")) == ["E201"]


def test_in_state_works_in_pre_goal_if_and_wait_until_and_with_not_and_or():
    source = task(
        'if in_state door "open" { goal { not in_state door "closed"; } wait for 1 s; } else { do r open door; }\n'
        'wait until in_state door "open" or in_state kit "x" timeout 2 s else { fail "no"; }',
        goal='goal { (in_state door "open" and not in_state fryer "off"); }',
        pre='pre { in_state door "closed"; }')
    assert codes(source) == []


def test_a_do_takes_a_failure_handler_and_stands_in_branches_and_handlers():
    assert codes(task('do r open door else { fail "stuck"; }')) == []
    assert codes(task('if robot_ready r { do r open door; } else { fail "no"; }')) == []
    assert codes(task('move r to ward else { do r close door; fail "x"; }')) == []


def test_errors_of_a_do_are_reported_with_the_others():
    assert codes(task("do r open window;\ndo r open zone;", goal='goal { in_state door ""; }')) == [
        "E307", "E303", "E304"]


# -------- fixtures and snapshots --------

HEADER = re.compile(
    r"\A// RoboLang v1\.8 fixture: (?P<set>valid|invalid)\n"
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
    assert (document["language_version"], document["ir_schema_version"]) == ("1.8", "1.7")
    assert read_ir(actual.encode()) == compile_source(path.read_text(encoding="utf-8")).ir


@pytest.mark.parametrize("path", INVALID, ids=lambda p: p.stem)
def test_invalid_fixture_is_rejected_with_the_stated_code_and_line_and_makes_no_ir(path):
    stated = header(path)
    assert path.name.startswith(stated["code"].lower()) and stated["code"][1] == STAGE_DIGIT[stated["stage"]]
    with pytest.raises(RoboLangError) as caught:
        compile_source(path.read_text(encoding="utf-8"), str(path))
    assert (caught.value.code, str(caught.value.line)) == (stated["code"], stated["line"])


def test_fixture_sets_are_counted():
    assert len(VALID) == 5 and len(INVALID) == 10


def test_every_new_code_is_in_the_specification_tables():
    table = set(re.findall(r"^\| `((?:[EISX]\d{3})(?:`, `[EISX]\d{3})*)` \|", SPEC, re.MULTILINE))
    listed = {code for row in table for code in row.split("`, `")}
    assert {"S006", "S007", "S008", "S009", "S271", "E517", "X606", "S508"} <= listed
    for code in ("S006", "S007", "S008", "S009", "S271", "E517"):
        assert f"`{code}`" in SIM_SPEC
    assert "`X606`" in (ROOT / "spec" / "export-goal-v0.1.md").read_text(encoding="utf-8")
    assert "in_state_unsupported" in (ROOT / "spec" / "delegate-v0.1.md").read_text(encoding="utf-8")


# -------- Task IR 1.7 and the reader --------

STEPS = task('do r open door;\ndo r set fryer to 175 degC else { fail "x"; }\ndo r tag door to "L";',
             goal='goal { in_state door "open"; }')


def test_ir_shape_of_do_and_in_state_and_reading_back():
    document = json.loads(to_json(compile_source(STEPS).ir))
    steps = document["tasks"][0]["steps"]
    assert steps[0] == {"kind": "do", "robot": "r", "action": "open", "target": "door"}
    assert list(steps[1]) == ["kind", "robot", "action", "target", "value", "else"]
    assert steps[1]["value"] == {"kind": "quantity", "value": "175", "unit": "degC"}
    assert steps[2]["value"] == {"kind": "text", "value": "L"}
    assert document["tasks"][0]["goal"] == [{"kind": "in_state", "target": "door", "label": "open"}]
    assert read_ir(to_json(compile_source(STEPS).ir).encode()) == compile_source(STEPS).ir


def document_of(source: str = STEPS) -> dict:
    return json.loads(to_json(compile_source(source).ir))


def problems(document) -> list[tuple[str, str]]:
    with pytest.raises(IRError) as caught:
        read_ir(json.dumps(document).encode())
    return [(d.code, d.pointer) for d in caught.value.diagnostics]


def test_the_reader_checks_the_names_the_types_and_the_shape_of_do_and_in_state():
    d = document_of()
    d["tasks"][0]["steps"][0]["target"] = "window"
    assert problems(d) == [("I203", "/tasks/0/steps/0/target")]
    d = document_of()
    d["tasks"][0]["steps"][0]["target"] = "zone"
    assert problems(d) == [("I204", "/tasks/0/steps/0/target")]
    d = document_of()
    d["tasks"][0]["goal"][0]["target"] = "r"
    assert problems(d) == [("I204", "/tasks/0/goal/0/target")]
    d = document_of()
    d["tasks"][0]["goal"][0]["label"] = ""
    assert problems(d) == [("I207", "/tasks/0/goal/0/label")]
    d = document_of()
    d["tasks"][0]["steps"][2]["value"]["value"] = ""
    assert problems(d) == [("I207", "/tasks/0/steps/2/value/value")]
    d = document_of()
    d["tasks"][0]["steps"][1]["value"]["unit"] = "furlong"
    assert problems(d) == [("I117", "/tasks/0/steps/1/value/unit")]
    d = document_of()
    d["tasks"][0]["steps"][0]["action"] = "move"
    assert problems(d) == [("I118", "/tasks/0/steps/0/action")]
    d = document_of()
    del d["tasks"][0]["steps"][0]["action"]
    assert problems(d) == [("I111", "/tasks/0/steps/0")]


def test_schema_1_6_does_not_have_do_or_in_state_i115():
    d = document_of()
    d["ir_schema_version"], d["language_version"] = "1.6", "1.7"
    assert {code for code, _ in problems(d)} == {"I115"}


# -------- world file 1.4 --------

def test_world_file_1_4_reads_actions_state_and_location_states():
    plan = compile_source(task("do r open door;")).ir.tasks[0]
    world = load_world(world_data(), "w.json", plan)
    action = world.robots["moxi1"].action("set")
    assert (action.target, action.result, action.value) == ("location", "heating", "temperature")
    assert world.robots["moxi1"].action("fly") is None and world.robots["chef"].actions == ()
    assert world.objects["closet_door"].state == "closed" and world.objects["kit"].state is None
    assert dict(world.location_states) == {"fryer": "off"} and world.schema_version == "1.4"


@pytest.mark.parametrize("version", ["1.0", "1.1", "1.2", "1.3"])
def test_older_world_files_do_not_know_the_new_fields(version):
    plan = compile_source(task("do r open door;")).ir.tasks[0]
    world = load_world(world_data(version=version), "w.json", plan)
    assert all(robot.actions == () for robot in world.robots.values()) and not world.location_states
    for patch, pointer in (
        (lambda w: w["robots"]["chef"].update(actions={}), "/robots/chef/actions"),
        (lambda w: w["objects"]["kit"].update(state="x"), "/objects/kit/state"),
        (lambda w: w.update(location_states={}), "/location_states"),
    ):
        with pytest.raises(WorldError) as caught:
            load_world(world_data(patch, version), "w.json", plan)
        assert (caught.value.code, caught.value.pointer) == ("E503", pointer)


@pytest.mark.parametrize("patch, code, pointer", [
    (lambda w: w["robots"]["moxi1"]["actions"]["open"].update(target="region"), "E517",
     "/robots/moxi1/actions/open/target"),
    (lambda w: w["robots"]["moxi1"]["actions"]["set"].update(value="duration"), "E517",
     "/robots/moxi1/actions/set/value"),
    (lambda w: w["robots"]["moxi1"]["actions"]["open"].pop("result"), "E504", "/robots/moxi1/actions/open"),
    (lambda w: w["robots"]["moxi1"]["actions"]["open"].update(result=""), "E504",
     "/robots/moxi1/actions/open/result"),
    (lambda w: w["robots"]["moxi1"]["actions"]["open"].update(speed="1"), "E503", "/robots/moxi1/actions/open/speed"),
    (lambda w: w["robots"]["moxi1"]["actions"].update({"Open": {"target": "object", "result": "x"}}), "E504",
     "/robots/moxi1/actions/Open"),
    (lambda w: w["robots"]["moxi1"].update(actions=[]), "E504", "/robots/moxi1/actions"),
    (lambda w: w["objects"]["kit"].update(state=3), "E504", "/objects/kit/state"),
    (lambda w: w["objects"]["kit"].update(state=""), "E504", "/objects/kit/state"),
    (lambda w: w["location_states"].update(elsewhere="x"), "E505", "/location_states/elsewhere"),
    (lambda w: w.update(location_states=[]), "E504", "/location_states"),
])
def test_world_file_1_4_is_validated_strictly(patch, code, pointer):
    plan = compile_source(task("do r open door;")).ir.tasks[0]
    with pytest.raises(WorldError) as caught:
        load_world(world_data(patch), "w.json", plan)
    assert (caught.value.code, caught.value.pointer) == (code, pointer)


# -------- the simulation (A7) --------

def test_do_sets_the_declared_result_records_requested_and_the_goal_verifies_it():
    result, record = simulate(task("do r open door;"))
    assert result.state == "SUCCEEDED"
    step = record["steps"][0]
    assert step["action"] == {"robot": "moxi1", "action": "open", "target": "closet_door", "value": None,
                              "result": "open", "status": "requested"}
    assert record["final_world"]["objects"]["closet_door"]["state"] == "open"
    assert record["goal"][0]["result"] == "true" and record["run_record_schema_version"] == "1.7"
    assert record["core_outcome"] is None  # an in_state goal has no Core form
    assert [s["action"] for s in record["steps"][1:]] == []


def test_a_task_that_does_not_do_it_fails_its_goal_a_request_is_not_a_result():
    result, record = simulate(task("wait for 1 s;"))
    assert failure(record) == ("S301", None) and record["steps"][0]["action"] is None
    assert record["goal"][0]["result"] == "false"


def test_a_value_is_recorded_and_a_location_target_changes_its_state():
    heat = task("move r to fryer;\ndo r set fryer to 175 degC;", goal='goal { in_state fryer "heating"; }')
    _, record = simulate(heat)
    assert record["failure"] is None
    assert record["steps"][1]["action"]["value"] == {"kind": "quantity", "value": "175", "unit": "degC"}
    assert record["final_world"]["location_states"] == {"fryer": "heating"}


def test_in_state_is_true_false_or_unknown_and_unknown_fails_closed():
    plan = 'do r open door;'
    # false: the door is closed and nothing opened it
    assert failure(simulate(task("wait for 1 s;"))[1]) == ("S301", None)
    # unknown: the kit has no state, so a goal on it fails closed with S302
    unknown = task(plan, goal='goal { in_state door "open"; in_state kit "open"; }')
    assert failure(simulate(unknown)[1]) == ("S302", None)
    # ... a pre on it is S102 and an if on it is S261, as for any unknown condition
    assert failure(simulate(task(plan, pre='pre { in_state kit "open"; }'))[1]) == ("S102", None)
    branch = task('if in_state kit "open" { wait for 1 s; } else { wait for 2 s; }\ndo r open door;')
    assert failure(simulate(branch)[1]) == ("S261", 1)
    # a false condition is S301 even when another is unknown
    both = task("wait for 1 s;", goal='goal { in_state door "open"; in_state kit "open"; }')
    assert failure(simulate(both)[1]) == ("S301", None)


def test_in_state_with_not_and_in_a_wait_until():
    ok = task('wait until not in_state door "open" timeout 1 s else { fail "x"; }\ndo r open door;')
    assert simulate(ok)[0].state == "SUCCEEDED"
    never = task('wait until in_state door "open" timeout 1 s else { fail "never"; }',
                 goal="goal { object_exists kit; }")
    _, record = simulate(never)
    assert failure(record) == ("S262", 2)


def test_the_robot_must_be_at_the_target_s271_c2():
    _, record = simulate(task("do r open door;"), patch=lambda w: w["robots"]["moxi1"].update(location="ward"))
    assert failure(record) == ("S271", 1) and record["steps"][0]["action"] is None
    assert record["final_world"]["objects"]["closet_door"]["state"] == "closed"
    # moving there first is what the request needs; a location target is the same
    assert simulate(task("move r to closet;\ndo r open door;"), patch=lambda w: w["robots"]["moxi1"].update(
        location="ward"))[0].state == "SUCCEEDED"
    _, record = simulate(task("do r set fryer to 175 degC;", goal='goal { in_state fryer "heating"; }'))
    assert failure(record) == ("S271", 1)
    # an object the robot holds is at the robot
    held = task("grasp r kit;\nmove r to ward;\ndo r open kit;", goal='goal { in_state kit "open"; }')
    assert failure(simulate(held, patch=lambda w: w["robots"]["moxi1"]["actions"].update(
        {"open": {"target": "object", "result": "open"}}))[1]) is None


def test_a_robot_that_is_not_ready_cannot_do_s201():
    _, record = simulate(task("do r open door;"), patch=lambda w: w["robots"]["moxi1"].update(ready=False))
    assert failure(record) == ("S201", 1)


def test_a_scripted_fault_fails_a_do_and_a_handler_catches_it_c6():
    source = task('do r open door else { fail "door stuck"; }')
    compiled = compile_source(source)
    plan = compiled.ir.tasks[0]
    bindings = {p.name: ENTITY[p.name] for p in plan.parameters}
    data = json.loads(world_data(None, "1.4", "t", bindings))
    data["tasks"]["t"]["faults"] = [{"step": 1}]
    result = run(plan, load_world(json.dumps(data).encode(), "w.json", plan))
    assert result.failure.code == "S262" and result.failure.reason == "door stuck"
    assert result.steps[0].status == "failed" and result.steps[0].handled == "S251" and result.steps[0].action is None
    data["tasks"]["t"]["faults"] = [{"step": 1}]
    plain = compile_source(task("do r open door;")).ir.tasks[0]
    assert run(plain, load_world(json.dumps(data).encode(), "w.json", plain)).failure.code == "S251"


def test_a_handler_that_recovers_reaches_the_goal():
    source = task('do r open door else { do r close door; do r open door; }')
    plan = compile_source(source).ir.tasks[0]
    data = json.loads(world_data(None, "1.4", "t", dict(ENTITY)))
    data["tasks"]["t"]["faults"] = [{"step": 1}]
    result = run(plan, load_world(json.dumps(data).encode(), "w.json", plan))
    assert result.state == "SUCCEEDED" and result.steps[0].handled == "S251"


def test_a_do_in_a_branch_runs_on_that_path_only():
    source = task('if in_state door "closed" { do r open door; } else { fail "already open"; }')
    result, _ = simulate(source)
    assert result.state == "SUCCEEDED"
    result, record = simulate(source, patch=lambda w: w["objects"]["closet_door"].update(state="open"))
    assert failure(record) == ("S262", 3)


def test_the_run_is_deterministic():
    one, two = simulate(task("do r open door;"))[1], simulate(task("do r open door;"))[1]
    assert json.dumps(one) == json.dumps(two)


# -------- checks before the run (A6, C7, C13) --------

@pytest.mark.parametrize("plan, goal, code, step", [
    ("do m open door;", 'goal { in_state door "open"; }', "S006", 1),  # chef declares no action
    ("do r fly door;", 'goal { in_state door "open"; }', "S006", 1),  # the name must match exactly (C12)
    ("do r open_door door;", 'goal { in_state door "open"; }', "S006", 1),
    ("do r open fryer;", 'goal { in_state fryer "open"; }', "S007", 1),  # an object action on a location
    ("do r print door;", 'goal { in_state door "printed"; }', "S007", 1),  # a location action on an object
    ("do r open door to 30 cm;", 'goal { in_state door "open"; }', "S008", 1),  # a value where none is taken
    ("do r set fryer;", 'goal { in_state fryer "heating"; }', "S008", 1),  # a value is missing
    ("do r set fryer to 2 kg;", 'goal { in_state fryer "heating"; }', "S008", 1),  # a mass for a temperature
    ('do r set fryer to "hot";', 'goal { in_state fryer "heating"; }', "S008", 1),  # a text for a temperature
    ("do r tag door to 5 g;", 'goal { in_state door "labelled"; }', "S008", 1),  # a quantity for a label
    ("wait for 1 s;\ndo r open door;\ndo r fly door;", 'goal { in_state door "open"; }', "S006", 3),
])
def test_s006_s007_s008_fail_before_authorized_and_nothing_runs(plan, goal, code, step):
    result, record = simulate(task(plan, goal))
    assert failure(record) == (code, step) and record["failure"]["phase"] == "authorization"
    assert "AUTHORIZED" not in record["states"] and all(s["status"] == "not_run" for s in record["steps"])


def test_a_world_file_1_3_robot_declares_no_actions_s006():
    _, record = simulate(task("do r open door;"), version="1.3")
    assert failure(record) == ("S006", 1)


def test_s009_a_required_state_that_no_declared_action_produces_c7():
    _, record = simulate(task("wait for 1 s;", goal='goal { in_state door "locked"; }'))
    assert failure(record) == ("S009", None) and record["failure"]["phase"] == "authorization"
    # the world already holds it: nothing to produce
    assert simulate(task("wait for 1 s;", goal='goal { in_state door "closed"; }'))[0].state == "SUCCEEDED"
    # a declared action of the right kind of target that produces it
    assert simulate(task("do r open door;", goal='goal { in_state door "open"; }'))[0].state == "SUCCEEDED"
    # the same result label on the wrong kind of target does not count
    assert failure(simulate(task("wait for 1 s;", goal='goal { in_state fryer "open"; }'))[1]) == ("S009", None)
    # only the robots that are parameters of the task count: chef declares nothing
    chef_only = task("wait for 1 s;", goal='goal { in_state door "open"; }', params="robot m, object door")
    plan = compile_source(chef_only).ir.tasks[0]
    loaded = load_world(world_data(None, "1.4", "t", {"m": "chef", "door": "closet_door"}), "w.json", plan)
    assert run(plan, loaded).failure.code == "S009"


def test_s009_only_for_what_is_required_on_every_path():
    for goal in ('goal { in_state door "locked" or object_exists kit; }', 'goal { not in_state door "locked"; }'):
        assert simulate(task("wait for 1 s;", goal=goal))[0].state in ("SUCCEEDED", "FAILED")
        assert failure(simulate(task("wait for 1 s;", goal=goal))[1]) is None
    both = task("wait for 1 s;", goal='goal { object_exists kit; (in_state door "locked" and object_exists kit); }')
    assert failure(simulate(both)[1]) == ("S009", None)


def test_the_checks_look_at_every_path_branches_and_handlers():
    source = task('if robot_ready r { wait for 1 s; } else { do r fly door; }\ndo r open door;')
    assert failure(simulate(source)[1]) == ("S006", 3)
    handler = task('do r open door else { do r fly door; }')
    assert failure(simulate(handler)[1]) == ("S006", 2)


def test_c13_a_do_on_a_location_is_a_target_for_s004():
    source = task("move r to closet;\nmove r to fryer;\ndo r print fryer;", goal='goal { in_state fryer "printed"; }',
                  annotations="@safety no_go_region zone;")
    _, record = simulate(source)
    assert failure(record)[0] == "S004"
    only_do = task("do r print fryer;", goal='goal { in_state fryer "printed"; }',
                   annotations="@safety no_go_region zone;")
    _, record = simulate(only_do, patch=lambda w: w["robots"]["moxi1"].update(location="fryer"))
    assert failure(record)[0] in ("S004", "S005")  # the target is forbidden (and so is the start)
    on_object = task("do r open door;", annotations="@safety no_go_region zone;")
    assert simulate(on_object)[0].state == "SUCCEEDED"  # an object is not a location target


def test_a_limit_failure_comes_before_an_action_failure():
    source = task("move r to fryer;\ndo r fly door;", annotations="@safety no_go_region zone;")
    assert failure(simulate(source)[1])[0] == "S004"


# -------- validate --world (every problem) --------

def test_validate_world_lists_every_problem_and_points_at_the_step_and_the_goal(tmp_path):
    source = task('do m open door;\nif robot_ready r { do r open fryer; } else { do r set fryer; }',
                  goal='goal { in_state door "locked"; }')
    (tmp_path / "t.rob").write_text(source, encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data())
    code, out, err = run_cli(["validate", "t.rob", "--world", "w.json"], tmp_path)
    assert code == 1 and out == ""
    found = re.findall(r"t\.rob:(\d+):\d+: error\[(S\d{3})\]", err)
    assert [c for _, c in found] == ["S006", "S007", "S008", "S009"]
    assert [int(line) for line, _ in found][:3] == [7, 8, 8] and "cannot succeed" in err
    assert "(on path 2.then)" in err and "(on path 2.else)" in err


def test_validate_world_passes_a_task_whose_actions_hold(tmp_path):
    (tmp_path / "t.rob").write_text(task("do r open door;"), encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data())
    code, out, err = run_cli(["validate", "t.rob", "--world", "w.json"], tmp_path)
    assert code == 0 and "no certain failure found" in (out + err)


def test_validate_world_for_an_ir_file_points_at_the_step_and_the_goal(tmp_path):
    source = task("do r fly door;", goal='goal { in_state door "x"; }')
    (tmp_path / "t.json").write_text(to_json(compile_source(source).ir))
    (tmp_path / "w.json").write_bytes(world_data())
    code, out, err = run_cli(["validate", "--ir", "t.json", "--world", "w.json"], tmp_path)
    assert code == 1 and "/tasks/0/steps/0" in err and "/tasks/0/goal/0" in err


def test_simulate_prints_the_failure_meaning_and_inspect_shows_the_syntax(tmp_path):
    (tmp_path / "t.rob").write_text(task('do r set fryer to 175 degC;\ndo r tag door to "L";',
                                         goal='goal { in_state door "labelled"; }'), encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data())
    code, out, err = run_cli(["simulate", "t.rob", "--world", "w.json"], tmp_path)
    assert code == 4 and "S271" in err and "do: the robot is not at the target" in err
    code, out, err = run_cli(["inspect", "t.rob"], tmp_path)
    assert 'do r set fryer to 175 degC' in out and 'do r tag door to "L"' in out and 'in_state door "labelled"' in out
    (tmp_path / "h.rob").write_text(task('do r open door else { fail "x"; }'), encoding="utf-8")
    assert "if it fails:" in run_cli(["inspect", "h.rob"], tmp_path)[1]


# -------- refusals: export-goal, delegate, ROS 2 (A8) --------

SENDER = "rl:agent:ai"


def test_export_goal_refuses_a_goal_with_in_state_x606_and_never_drops_it():
    from robolang.export_goal import ExportError, export_goal
    plan = compile_source(task("do r open door;")).ir.tasks[0]
    world = load_world(world_data(), "w.json", plan)
    for core in ("0.1", "0.2"):
        with pytest.raises(ExportError) as caught:
            export_goal(plan, world, sender=SENDER, recipient="rl:robot:moxi1", core=core)
        assert caught.value.code == "X606" and 'in_state door "open"' in caught.value.message
    branch = task('if robot_ready r { goal { in_state door "open"; } do r open door; } else { fail "x"; }',
                  goal="")
    plan = compile_source(branch).ir.tasks[0]
    assert [s.label for s in plan.goal_states] == ["open"]
    with pytest.raises(ExportError) as caught:
        export_goal(plan, load_world(world_data(), "w.json", plan), sender=SENDER, recipient="rl:robot:moxi1")
    assert caught.value.code == "X606"


def test_a_do_without_an_in_state_goal_exports_its_goal():
    from robolang.export_goal import export_goal
    def things(world):
        for obj in world["objects"].values():
            obj.update({"class": "box", "mass": {"value": "1", "unit": "kg", "uncertainty": "0.1"}})

    plain = compile_source(task("do r open door;", goal="goal { object_at kit closet; }")).ir.tasks[0]
    exported = export_goal(plain, load_world(world_data(things), "w.json", plain), sender=SENDER,
                           recipient="rl:robot:moxi1")
    assert "in_state" not in json.dumps(exported.message)


def test_the_export_goal_command_exits_6_with_x606(tmp_path):
    (tmp_path / "t.rob").write_text(task("do r open door;"), encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data())
    code, out, err = run_cli(["export-goal", "t.rob", "--world", "w.json", "--from", SENDER, "--to",
                              "rl:robot:moxi1"], tmp_path)
    assert code == 6 and "error[X606]" in err and out == ""


def test_delegate_refuses_a_goal_with_in_state_in_the_proposal_and_in_the_library(tmp_path):
    from robolang import delegate
    plan = compile_source(task("do r open door;")).ir.tasks[0]
    assert delegate._consider(None, plan, b"", "w.json", "moxi1", (), "0.1") == ("in_state_unsupported", None)
    (tmp_path / "t.rob").write_text(task("do r open door;"), encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data())
    code, out, err = run_cli(["delegate", "t.rob", "--world", "w.json", "--from", SENDER, "--to", "rl:robot:moxi1"],
                             tmp_path)
    assert code == 6 and "error[X606]" in err


def test_the_ros2_backend_refuses_do_and_in_state_with_s508_before_any_motion():
    pytest.importorskip("jsonschema")
    sys.path.insert(0, str(ROOT / "backends" / "ros2"))
    from robolang_ros2.executor import execute
    from test_ros2_executor import FakeOps, fixture
    _, bindings, site = fixture()
    for body, goal in (("do r open box;", "object_at box dst"), ("move r to dst;", 'in_state box "open"')):
        source = ("robolang 1.8;\ntask transfer(robot r, object box, location src, location dst) {\n"
                  f"  goal {{ {goal}; }}\n  plan {{ {body} }}\n}}\n")
        ops = FakeOps()
        record = execute(compile_source(source).ir.tasks[0], dict(bindings), site, ops)
        assert record["failure"]["code"] == "S508" and "declared actions" in record["failure"]["message"]
        assert ops.calls == [("guard", "ur")]
