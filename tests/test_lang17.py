"""
RoboLang 1.7: region limits (spec/robolang-v1.7.md, decision 0039 P1-P10 and C1-C7).

Front end, Task IR 1.6 and its reader, world file 1.3 (`contains`, `honours`), the checks before the run (`S003`,
`S004`, `S005`), `validate --world`, the refusals in `export-goal`, `delegate` and the ROS 2 backend. No test here
needs a model or a network.

To regenerate the IR snapshots after an intended change:

    ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_lang17.py
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
FIXTURES = ROOT / "tests" / "fixtures" / "v1.7"
SNAPSHOTS = ROOT / "tests" / "snapshots" / "ir-v1.7"
UPDATE = os.environ.get("ROBOLANG_UPDATE_SNAPSHOTS") == "1"
SPEC = (ROOT / "spec" / "robolang-v1.7.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")

# -------- helpers --------

PARAMS = "robot r, robot m, location a, location b, location c, location d, region zone, region lane, region open"
ENTITY = {"r": "arm1", "m": "arm2", "a": "a", "b": "b", "c": "c", "d": "d", "zone": "zone", "lane": "lane",
          "open": "open"}
BASE_WORLD = {
    "world_schema": "robolang.sim.world",
    "world_schema_version": "1.3",
    "locations": ["a", "b", "c", "d"],
    "regions": {"zone": {"clear": True, "contains": ["b"]}, "lane": {"clear": True, "contains": ["a", "c"]},
                "open": {"clear": True}},
    "robots": {"arm1": {"ready": True, "location": "a", "gripper": True, "honours": ["no_go_region", "in_region"],
                        "max_speed": [{"value": "1", "unit": "m/s"}]},
               "arm2": {"ready": True, "location": "a", "gripper": True, "max_speed": [{"value": "1", "unit": "m/s"}]}},
    "tasks": {},
}


def task(annotations: str, plan: str = "move r to c;", goal: str = "goal { robot_at r c; }", params: str = PARAMS,
         version: str = "1.7", name: str = "t") -> str:
    return f"robolang {version};\ntask {name}({params}) {{\n{annotations}\n{goal}\nplan {{\n{plan}\n}}\n}}\n"


def codes(source: str) -> list[str]:
    try:
        check(source)
    except RoboLangError as error:
        return [d.code for d in error.diagnostics]
    return []


def world_data(patch=None, version="1.3", name="t", bindings=None) -> bytes:
    world = copy.deepcopy(BASE_WORLD)
    world["world_schema_version"] = version
    if version != "1.3":  # a world file 1.2 has neither field
        for region in world["regions"].values():
            region.pop("contains", None)
        for robot in world["robots"].values():
            robot.pop("honours", None)
    if patch is not None:
        patch(world)
    world["tasks"] = {name: {"bindings": bindings or dict(ENTITY)}}
    return json.dumps(world).encode()


def simulate(source: str, patch=None, version="1.3"):
    """Compile, run against a world file (1.3 by default), return (result, record)."""
    compiled = compile_source(source)
    plan = compiled.ir.tasks[0]
    bindings = {p.name: ENTITY[p.name] for p in plan.parameters}
    loaded = load_world(world_data(patch, version, plan.name, bindings), "w.json", plan)
    result = run(plan, loaded)
    record = to_record(result, task=plan, run_id="sim-x", source_sha256="0", world_sha256="0", seed="0",
                       language_version=compiled.ir.language_version, world_schema_version=loaded.schema_version)
    return result, record


def failure(record) -> tuple[str, int | None] | None:
    """(code, the 1-based number of the limit that failed the run)."""
    return None if record["failure"] is None else (record["failure"]["code"], record["failure"]["safety_limit"])


def statuses(record) -> list[str]:
    return [entry["status"] for entry in record["safety_limits"]]


def run_cli(args, cwd):
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(cwd)
    try:
        code = main(args, out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


# -------- front end (P1, P2, P4) --------

def test_version_1_7_selects_ir_schema_1_6_and_older_versions_keep_theirs():
    document = json.loads(to_json(compile_source(task("@safety no_go_region zone;")).ir))
    assert (document["language_version"], document["ir_schema_version"]) == ("1.7", "1.6")
    assert ir.IR_SCHEMA_VERSIONS == {"1.1": "1.0", "1.2": "1.1", "1.3": "1.2", "1.4": "1.3", "1.5": "1.4",
                                     "1.6": "1.5", "1.7": "1.6", "1.8": "1.7", "1.9": "1.8", "2.0": "1.8", "2.1": "1.9"}


@pytest.mark.parametrize("version", ["1.1", "1.2", "1.3", "1.4", "1.5", "1.6"])
def test_a_region_limit_in_an_earlier_version_is_still_an_error(version):
    params = PARAMS if version != "1.1" else "robot r, location c, region zone"
    goal = "goal { robot_at r c; }" if version != "1.1" else ""
    assert codes(task("@safety no_go_region zone;", params=params, version=version, goal=goal)) == ["E211"]
    assert codes(task("@safety in_region zone;", params=params, version=version, goal=goal)) == ["E201"]


def test_the_two_words_are_still_reserved_and_no_word_is_new():
    from robolang.tokens import RESERVED_BY_VERSION
    assert RESERVED_BY_VERSION["1.7"] == RESERVED_BY_VERSION["1.6"]
    assert codes(task("", params="robot no_go_region, location c, region zone")) == ["E212"]


def test_both_kinds_mix_with_max_speed_in_any_order_and_keep_source_order_in_the_ir():
    source = task("@safety in_region lane;\n@safety max_speed 0.5 m/s;\n@safety no_go_region zone;")
    limits = compile_source(source).ir.tasks[0].safety
    assert [type(s).__name__ for s in limits] == ["RegionLimit", "SafetyLimit", "RegionLimit"]
    assert [s.kind for s in limits] == ["in_region", "max_speed", "no_go_region"]
    assert limits[0] == ir.RegionLimit("in_region", "lane")


def test_the_region_must_be_a_declared_region_parameter():
    assert codes(task("@safety no_go_region wet;")) == ["E303"]
    assert codes(task("@safety no_go_region a;")) == ["E304"]
    assert codes(task("@safety in_region r;")) == ["E304"]


def test_a_region_is_named_once_e323_and_at_most_eight_limits_e324():
    assert codes(task("@safety no_go_region zone;\n@safety no_go_region zone;")) == ["E323"]
    assert codes(task("@safety in_region zone;\n@safety no_go_region zone;")) == ["E323"]
    nine = "".join(f"@safety no_go_region g{i};\n" for i in range(9))
    params = PARAMS + "".join(f", region g{i}" for i in range(9))
    assert codes(task(nine, params=params)) == ["E324"]
    assert codes(task(nine.replace("g8", "g7"), params=params)) == ["E323"]  # g7 twice; the eighth is the last kept


def test_max_speed_rules_do_not_count_region_limits():
    assert codes(task("@safety max_speed 1 m/s;\n@safety no_go_region zone;\n@safety max_speed 1 rad/s;")) == []
    assert codes(task("@safety no_go_region zone;\n@safety max_speed 2 m/s;\n@safety max_speed 3 m/s;")) == ["E306"]


def test_the_error_for_one_move_names_the_annotation():
    with pytest.raises(RoboLangError) as caught:
        check(task("", plan="move r to c in_region lane;"))
    assert caught.value.code == "E211" and "@safety in_region R;" in str(caught.value)


def test_the_unknown_key_message_lists_the_region_limits_in_1_7_only():
    with pytest.raises(RoboLangError) as caught:
        check(task("@safety keep_out zone;"))
    assert "'max_speed', 'no_go_region', 'in_region'" in str(caught.value)
    with pytest.raises(RoboLangError) as caught:
        check(task("@safety keep_out zone;", version="1.6"))
    assert "'no_go_region'" not in str(caught.value)


# -------- fixtures and snapshots --------

HEADER = re.compile(
    r"\A// RoboLang v1\.7 fixture: (?P<set>valid|invalid)\n"
    r"// expect: (?:(?P<ok>ok)|error (?P<code>E\d{3}) \((?P<stage>lexical|syntax|static)\) line (?P<line>\d+))\n"
    r"// why: .+\n"
)
STAGE_DIGIT = {"lexical": "1", "syntax": "2", "static": "3"}
VALID = sorted((FIXTURES / "valid").glob("*.rob"))
INVALID = sorted((FIXTURES / "invalid").glob("*.rob"))
NEW_STATIC_CODES = ("E323", "E324")


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
    assert (document["language_version"], document["ir_schema_version"]) == ("1.7", "1.6")
    assert read_ir(actual.encode()) == compile_source(path.read_text(encoding="utf-8")).ir


@pytest.mark.parametrize("path", INVALID, ids=lambda p: p.stem)
def test_invalid_fixture_is_rejected_with_the_stated_code_and_line_and_makes_no_ir(path):
    stated = header(path)
    assert path.name.startswith(stated["code"].lower()) and stated["code"][1] == STAGE_DIGIT[stated["stage"]]
    with pytest.raises(RoboLangError) as caught:
        compile_source(path.read_text(encoding="utf-8"), str(path))
    assert (caught.value.code, str(caught.value.line)) == (stated["code"], stated["line"])


def test_fixture_sets_cover_every_new_static_code_and_are_counted():
    assert len(VALID) == 5 and len(INVALID) == 10
    assert set(NEW_STATIC_CODES) <= {header(p)["code"] for p in INVALID}


def test_every_new_code_is_in_the_specification_tables():
    table = set(re.findall(r"^\| `((?:[EISX]\d{3})(?:`, `[EISX]\d{3})*)` \|", SPEC, re.MULTILINE))
    listed = {code for row in table for code in row.split("`, `")}
    assert {"E323", "E324", "S003", "S004", "S005", "E516", "I224", "I225", "X605", "S508"} <= listed
    for code in ("S003", "S004", "S005", "E516"):
        assert f"`{code}`" in SIM_SPEC
    assert "`I224`" in (ROOT / "spec" / "ir-reader-v0.1.md").read_text(encoding="utf-8")
    assert "`X605`" in (ROOT / "spec" / "export-goal-v0.1.md").read_text(encoding="utf-8")


# -------- Task IR 1.6 and the reader --------

LIMITS = task("@safety max_speed 0.5 m/s;\n@safety no_go_region zone;\n@safety in_region lane;")


def test_ir_shape_of_region_limits_and_reading_back():
    text = to_json(compile_source(LIMITS).ir)
    safety = json.loads(text)["tasks"][0]["safety"]
    assert safety == [{"kind": "max_speed", "limit": {"value": "0.5", "unit": "m/s"}},
                      {"kind": "no_go_region", "region": "zone"}, {"kind": "in_region", "region": "lane"}]
    assert read_ir(text.encode()) == compile_source(LIMITS).ir
    assert to_json(read_ir(text.encode())) == text


def ir_document(mutate=None, source=LIMITS) -> dict:
    document = json.loads(to_json(compile_source(source).ir))
    if mutate:
        mutate(document["tasks"][0])
    return document


def problems(document) -> list[tuple[str, str]]:
    with pytest.raises(IRError) as caught:
        read_ir(json.dumps(document).encode())
    return [(d.code, d.pointer) for d in caught.value.diagnostics]


def test_the_reader_checks_the_names_the_count_and_the_shape_of_region_limits():
    def region_of(index, name):
        return lambda t: t["safety"][index].update(region=name)
    assert problems(ir_document(region_of(1, "wet"))) == [("I203", "/tasks/0/safety/1/region")]
    assert problems(ir_document(region_of(1, "a"))) == [("I204", "/tasks/0/safety/1/region")]
    assert problems(ir_document(region_of(2, "zone"))) == [("I224", "/tasks/0/safety/2")]

    def nine(t):
        t["parameters"] += [{"name": f"g{i}", "type": "region"} for i in range(9)]
        t["safety"] = [{"kind": "no_go_region", "region": f"g{i}"} for i in range(9)]
    assert problems(ir_document(nine)) == [("I225", "/tasks/0/safety/8")]
    assert problems(ir_document(lambda t: t["safety"][1].update(limit={"value": "1", "unit": "m/s"}))) == [
        ("I112", "/tasks/0/safety/1/limit")]
    assert problems(ir_document(lambda t: t["safety"][0].update(region="zone"))) == [
        ("I112", "/tasks/0/safety/0/region")]
    assert problems(ir_document(lambda t: t["safety"][1].pop("region"))) == [("I111", "/tasks/0/safety/1")]


def test_schema_1_5_does_not_have_region_limits_i115():
    def older(document):
        document.update(ir_schema_version="1.5", language_version="1.6")
    document = ir_document()
    older(document)
    with pytest.raises(IRError) as caught:
        read_ir(json.dumps(document).encode())
    assert [(d.code, d.pointer) for d in caught.value.diagnostics][0] == ("I115", "/tasks/0/safety/1/kind")


def test_the_max_speed_rules_still_apply_to_max_speed_entries_only():
    def twice(t):
        t["safety"].append({"kind": "max_speed", "limit": {"value": "1", "unit": "m/s"}})
    assert problems(ir_document(twice)) == [("I206", "/tasks/0/safety/3")]


# -------- world file 1.3 (P5, P6, C1) --------

def load(patch=None, version="1.3"):
    plan = compile_source(task("@safety no_go_region zone;")).ir.tasks[0]
    return load_world(world_data(patch, version), "w.json", plan)


def test_world_file_1_3_reads_contains_and_honours():
    world = load()
    assert world.schema_version == "1.3"
    assert world.regions["zone"].contains == ("b",) and world.regions["open"].contains is None
    assert world.robots["arm1"].honours == ("no_go_region", "in_region") and world.robots["arm2"].honours == ()


@pytest.mark.parametrize("version", ["1.0", "1.1", "1.2"])
def test_older_world_files_do_not_know_the_new_fields(version):
    def patch(world):
        world["regions"]["zone"]["contains"] = ["b"]
    with pytest.raises(WorldError) as caught:
        load(patch, version)
    assert (caught.value.code, caught.value.pointer) == ("E503", "/regions/zone/contains")

    def patch(world):
        world["robots"]["arm1"]["honours"] = ["no_go_region"]
    with pytest.raises(WorldError) as caught:
        load(patch, version)
    assert (caught.value.code, caught.value.pointer) == ("E503", "/robots/arm1/honours")


@pytest.mark.parametrize("patch,code,pointer", [
    (lambda w: w["regions"]["zone"].update(contains="b"), "E504", "/regions/zone/contains"),
    (lambda w: w["regions"]["zone"].update(contains=["b", "b"]), "E504", "/regions/zone/contains/1"),
    (lambda w: w["regions"]["zone"].update(contains=["nowhere"]), "E505", "/regions/zone/contains/0"),
    (lambda w: w["regions"]["zone"].update(contains=["zone"]), "E505", "/regions/zone/contains/0"),
    (lambda w: w["regions"]["zone"].update(contains=[3]), "E504", "/regions/zone/contains/0"),
    (lambda w: w["robots"]["arm1"].update(honours="no_go_region"), "E504", "/robots/arm1/honours"),
    (lambda w: w["robots"]["arm1"].update(honours=["keep_out"]), "E516", "/robots/arm1/honours/0"),
    (lambda w: w["robots"]["arm1"].update(honours=["in_region", "in_region"]), "E504", "/robots/arm1/honours/1"),
    (lambda w: w["robots"]["arm1"].update(honours=[7]), "E504", "/robots/arm1/honours/0"),
])
def test_world_file_1_3_is_validated_strictly(patch, code, pointer):
    with pytest.raises(WorldError) as caught:
        load(patch)
    assert (caught.value.code, caught.value.pointer) == (code, pointer)


def test_world_file_1_3_keeps_the_measured_value_fields_of_1_2():
    def patch(world):
        world["sensors"] = {"probe": {"robot": "arm1", "available": True,
                                      "measures": {"label": {"contact": False}}}}
        world["objects"] = {"box": {"present": True, "location": "a", "properties": {"label": {"value": "x"}}}}
    assert load(patch).sensors["probe"].measure("label") is not None


# -------- before the run: S003, S004, S005 (P5, P6, P7, C1, C6, C7) --------

def test_a_limit_that_passes_is_recorded_as_checked_against_the_world_and_never_as_enforced():
    result, record = simulate(task("@safety max_speed 0.5 m/s;\n@safety no_go_region zone;\n@safety in_region lane;"))
    assert record["state"] == "SUCCEEDED"
    assert statuses(record) == ["checked_against_profile_not_enforced"] + ["checked_against_world_not_enforced"] * 2
    assert statuses(record)[1:] == ["checked_against_world_not_enforced"] * 2
    assert record["safety_limits"][1]["limit"] == {"kind": "no_go_region", "region": "zone"}
    assert all(status.endswith("not_enforced") for status in statuses(record))
    assert record["run_record_schema_version"] == "1.6" and record["inputs"]["ir_schema_version"] == "1.6"
    assert record["inputs"]["language_version"] == "1.7" and record["inputs"]["world_schema_version"] == "1.3"


def test_a_region_limit_with_no_unit_neighbour_has_only_the_world_status():
    _, record = simulate(task("@safety no_go_region zone;"))
    assert statuses(record) == ["checked_against_world_not_enforced"]


def test_s004_a_target_inside_a_no_go_region_fails_before_authorized():
    result, record = simulate(task("@safety no_go_region zone;", plan="move r to b;", goal="goal { robot_at r b; }"))
    assert failure(record) == ("S004", 1) and statuses(record) == ["failed_check"]
    assert record["states"] == ["CREATED", "VALIDATED", "FAILED"] and "AUTHORIZED" not in record["states"]
    assert record["failure"]["phase"] == "authorization" and record["failure"]["robot"] == "arm1"
    assert all(step["status"] == "not_run" for step in record["steps"])


def test_s004_a_target_outside_an_in_region_that_lists_its_locations():
    _, record = simulate(task("@safety in_region lane;", plan="move r to d;", goal="goal { robot_at r d; }"))
    assert failure(record) == ("S004", 1)
    _, record = simulate(task("@safety in_region lane;", plan="move r to c;"))
    assert record["state"] == "SUCCEEDED"


def test_a_region_that_lists_no_locations_is_unknown_and_concludes_nothing():
    for kind in ("no_go_region", "in_region"):
        _, record = simulate(task(f"@safety {kind} open;", plan="move r to b;", goal="goal { robot_at r b; }"))
        assert record["state"] == "SUCCEEDED", kind
        assert statuses(record) == ["checked_against_world_not_enforced"]


def test_an_empty_contains_is_known_to_hold_nothing():
    def patch(world):
        world["regions"]["open"]["contains"] = []
    _, record = simulate(task("@safety no_go_region open;"), patch)
    assert record["state"] == "SUCCEEDED"
    _, record = simulate(task("@safety in_region open;"), patch)
    assert failure(record) == ("S004", 1)


def test_the_target_of_a_branch_and_of_a_robot_at_goal_count_on_every_path():
    plan = "if region_clear open { move r to c; } else { move r to b; }"
    _, record = simulate(task("@safety no_go_region zone;", plan=plan))
    assert failure(record) == ("S004", 1)  # the else branch is never taken; the request still names b
    by_goal = task("@safety no_go_region zone;", plan="move r to c;", goal="goal { robot_at m b; }")
    _, record = simulate(by_goal)
    assert failure(record) == ("S004", 1) and record["failure"]["robot"] == "arm2"


def test_a_place_target_inside_the_region_fails_before_the_run():
    source = ("robolang 1.7;\ntask t(robot r, object box, location a, location b, region zone) {\n"
              "@safety no_go_region zone;\ngoal { object_at box b; }\nplan { grasp r box; place r box at b; }\n}\n")
    plan = compile_source(source).ir.tasks[0]
    data = json.loads(world_data(None, "1.3", "t", {"r": "arm1", "box": "box", "a": "a", "b": "b", "zone": "zone"}))
    data["objects"] = {"box": {"present": True, "location": "a"}}
    result = run(plan, load_world(json.dumps(data).encode(), "w.json", plan))
    assert (result.failure.code, result.failure.phase) == ("S004", "authorization")


def test_s003_a_robot_that_does_not_honour_the_kind_fails_closed():
    plan = "move m to c;"
    _, record = simulate(task("@safety no_go_region zone;", plan=plan, goal="goal { robot_at m c; }"))
    assert failure(record) == ("S003", 1) and record["failure"]["robot"] == "arm2"

    def patch(world):
        world["robots"]["arm1"]["honours"] = ["no_go_region"]
    _, record = simulate(task("@safety in_region lane;"), patch)
    assert failure(record) == ("S003", 1)  # honours no_go_region only
    _, record = simulate(task("@safety no_go_region zone;"), patch)
    assert record["state"] == "SUCCEEDED"


def test_s003_a_world_file_1_2_robot_honours_nothing_c1():
    result, record = simulate(task("@safety no_go_region zone;"), version="1.2")
    assert failure(record) == ("S003", 1) and record["inputs"]["world_schema_version"] == "1.2"
    _, record = simulate(task("@safety max_speed 1 m/s;"), version="1.2")
    assert record["state"] == "SUCCEEDED"  # a speed limit does not need `honours`


def test_only_robots_that_move_must_honour_and_handlers_and_branches_count_c6():
    only_grasps = ("robolang 1.7;\ntask t(robot r, robot m, object box, location a, location b, region zone) {\n"
                   "@safety no_go_region zone;\ngoal { robot_at r a; }\nplan { grasp m box; move r to a; }\n}\n")
    plan = compile_source(only_grasps).ir.tasks[0]
    data = json.loads(world_data(None, "1.3", "t", {"r": "arm1", "m": "arm2", "box": "box", "a": "a", "b": "b",
                                                    "zone": "zone"}))
    data["objects"] = {"box": {"present": True, "location": "a"}}
    result = run(plan, load_world(json.dumps(data).encode(), "w.json", plan))
    assert result.failure is None or result.failure.code != "S003"  # arm2 only grasps: it honours nothing, it need not
    handler = task("@safety no_go_region zone;", plan="move r to c else { move m to c; }")
    _, record = simulate(handler)
    assert failure(record) == ("S003", 1) and record["failure"]["robot"] == "arm2"
    branch = task("@safety no_go_region zone;", plan="if region_clear open { move r to c; } else { move m to c; }")
    _, record = simulate(branch)
    assert failure(record) == ("S003", 1)


def test_s005_a_robot_that_starts_where_the_limit_forbids_c7():
    def start_in_zone(world):
        world["robots"]["arm1"]["location"] = "b"
    _, record = simulate(task("@safety no_go_region zone;"), start_in_zone)
    assert failure(record) == ("S005", 1)
    _, record = simulate(task("@safety in_region lane;"), lambda w: w["robots"]["arm1"].update(location="d"))
    assert failure(record) == ("S005", 1)
    _, record = simulate(task("@safety in_region open;"), lambda w: w["robots"]["arm1"].update(location="d"))
    assert record["state"] == "SUCCEEDED"  # an unlisted region is unknown


def test_the_order_inside_a_limit_is_honours_then_targets_then_start():
    both = task("@safety no_go_region zone;", plan="move m to b;", goal="goal { robot_at m b; }")
    _, record = simulate(both, lambda w: w["robots"]["arm2"].update(location="b"))
    assert failure(record) == ("S003", 1)
    _, record = simulate(both, lambda w: w["robots"]["arm2"].update(location="b", honours=["no_go_region"]))
    assert failure(record) == ("S004", 1)
    _, record = simulate(task("@safety no_go_region zone;", plan="move r to c;"),
                         lambda w: w["robots"]["arm1"].update(location="b"))
    assert failure(record) == ("S005", 1)


def test_limits_are_examined_in_source_order_and_those_after_the_failure_are_not_checked():
    source = task("@safety no_go_region open;\n@safety in_region lane;\n@safety max_speed 1 m/s;",
                  plan="move r to d;", goal="goal { robot_at r d; }")
    _, record = simulate(source, lambda w: w["robots"]["arm1"].update(max_speed=[{"value": "1", "unit": "m/s"}]))
    assert failure(record) == ("S004", 2)
    assert statuses(record) == ["checked_against_world_not_enforced", "failed_check", "not_checked"]


def test_a_speed_limit_failure_still_comes_first_when_it_is_first():
    source = task("@safety max_speed 5 m/s;\n@safety no_go_region zone;", plan="move r to b;",
                  goal="goal { robot_at r b; }")
    _, record = simulate(source, lambda w: w["robots"]["arm1"].update(max_speed=[{"value": "1", "unit": "m/s"}]))
    assert failure(record) == ("S001", 1) and statuses(record) == ["failed_check", "not_checked"]


def test_the_record_is_deterministic_and_has_no_enforced_status_anywhere():
    _, first = simulate(LIMITS)
    _, second = simulate(LIMITS)
    assert first == second
    assert not re.search(r'"[a-z_]*(?<!not_)enforced"', json.dumps(first))


def test_a_task_without_region_limits_runs_on_a_1_3_world_exactly_as_on_1_2():
    source = task("@safety max_speed 0.5 m/s;", version="1.7")
    _, on_13 = simulate(source, lambda w: w["robots"]["arm1"].update(max_speed=[{"value": "1", "unit": "m/s"}]))
    assert on_13["state"] == "SUCCEEDED" and statuses(on_13) == ["checked_against_profile_not_enforced"]
    old = task("@safety max_speed 0.5 m/s;", version="1.6", goal="goal { robot_at r c; }")
    _, record = simulate(old, lambda w: w["robots"]["arm1"].update(max_speed=[{"value": "1", "unit": "m/s"}]))
    assert record["run_record_schema_version"] == "1.5" and record["state"] == "SUCCEEDED"


# -------- validate --world, inspect, simulate (CLI) --------

def test_validate_world_lists_every_problem_of_every_region_limit(tmp_path):
    source = task("@safety no_go_region zone;\n@safety in_region lane;", plan="move m to b;\nmove m to d;",
                  goal="goal { robot_at m d; }")
    (tmp_path / "t.rob").write_text(source, encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data(lambda w: w["robots"]["arm2"].update(location="b")))
    code, out, err = run_cli(["validate", "t.rob", "--world", "w.json"], tmp_path)
    lines = err.splitlines()
    assert code == 1
    found = [re.search(r"\[(S\d+)\]", line).group(1) for line in lines if "error[" in line]
    assert found == ["S003", "S004", "S005", "S003", "S004", "S004", "S004", "S005"]
    assert "t.rob:3:1: error[S003]" in lines[0] and "does not declare that it honours no_go_region" in lines[0]
    assert "t.rob:4:1: error[S003]" in lines[3]
    assert lines[-1] == "t.rob: task 't' cannot succeed in w.json (8 errors)"


def test_validate_world_passes_a_task_whose_limits_hold_and_says_so(tmp_path):
    (tmp_path / "t.rob").write_text(LIMITS, encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data())
    code, out, err = run_cli(["validate", "t.rob", "--world", "w.json"], tmp_path)
    assert code == 0 and "no certain failure found" in out


def test_validate_world_for_an_ir_file_points_at_the_limit(tmp_path):
    (tmp_path / "t.json").write_bytes(to_json(compile_source(task("@safety no_go_region zone;",
                                                                  plan="move r to b;")).ir).encode())
    (tmp_path / "w.json").write_bytes(world_data())
    code, out, err = run_cli(["validate", "--ir", "t.json", "--world", "w.json"], tmp_path)
    assert code == 1 and "/tasks/0/safety/0" in err and "error[S004]" in err


def test_simulate_prints_the_failure_meaning_and_the_record(tmp_path):
    (tmp_path / "t.rob").write_text(task("@safety no_go_region zone;", plan="move r to b;"), encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data())
    code, out, err = run_cli(["simulate", "t.rob", "--world", "w.json"], tmp_path)
    assert code == 4 and json.loads(out)["failure"]["code"] == "S004"
    assert "S004" in err and "inside a no_go_region" in err


def test_inspect_says_a_region_limit_is_declared_and_not_enforced(tmp_path):
    (tmp_path / "t.rob").write_text(LIMITS, encoding="utf-8")
    code, out, err = run_cli(["inspect", "t.rob"], tmp_path)
    assert code == 0, err
    assert "no_go_region zone  (declared only; checked against the world, not enforced by this tool)" in out
    assert "RoboLang 1.7 | IR schema 1.6" in out


# -------- elsewhere: export-goal, delegate, the ROS 2 backend (P8) --------

SENDER = "rl:agent:planner-1"


def test_export_goal_refuses_a_task_with_a_region_limit_x605_and_never_drops_it():
    from robolang.export_goal import ExportError, export_goal
    plan = compile_source(task("@safety no_go_region zone;")).ir.tasks[0]
    world = load_world(world_data(), "w.json", plan)
    for core in ("0.1", "0.2"):
        with pytest.raises(ExportError) as caught:
            export_goal(plan, world, sender=SENDER, recipient="rl:robot:arm1", core=core)
        assert caught.value.code == "X605" and "no_go_region zone" in caught.value.message
    plain = compile_source(task("@safety max_speed 0.5 m/s;")).ir.tasks[0]
    exported = export_goal(plain, load_world(world_data(), "w.json", plain), sender=SENDER, recipient="rl:robot:arm1")
    assert exported.message["payload"]["goal"]["constraints"] == [
        {"kind": "max_speed", "limit": {"value": "0.5", "unit": "m/s"}}]


def test_the_export_goal_command_exits_6_with_x605(tmp_path):
    (tmp_path / "t.rob").write_text(task("@safety in_region lane;"), encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data())
    code, out, err = run_cli(["export-goal", "t.rob", "--world", "w.json", "--from", SENDER, "--to",
                              "rl:robot:arm1"], tmp_path)
    assert code == 6 and "error[X605]" in err and out == ""


def test_delegate_refuses_a_task_with_a_region_limit_in_the_proposal_and_in_the_library(tmp_path):
    from robolang import delegate
    plan = compile_source(task("@safety no_go_region zone;")).ir.tasks[0]
    assert delegate._consider(None, plan, b"", "w.json", "arm1", (), "0.1") == ("region_limits_unsupported", None)
    (tmp_path / "t.rob").write_text(task("@safety in_region lane;"), encoding="utf-8")
    (tmp_path / "w.json").write_bytes(world_data())
    code, out, err = run_cli(["delegate", "t.rob", "--world", "w.json", "--from", SENDER, "--to", "rl:robot:arm1"],
                             tmp_path)
    assert code == 6 and "error[X605]" in err
    assert "region_limits_unsupported" in (ROOT / "spec" / "delegate-v0.1.md").read_text(encoding="utf-8")


def test_the_ros2_backend_refuses_a_region_limit_with_s508_before_any_motion():
    pytest.importorskip("jsonschema")
    sys.path.insert(0, str(ROOT / "backends" / "ros2"))
    from robolang_ros2.executor import execute
    from test_ros2_executor import FakeOps, fixture
    _, bindings, site = fixture()
    source = ("robolang 1.7;\ntask transfer(robot r, object box, location src, location dst, region zone) {\n"
              "  @safety no_go_region zone;\n  goal { object_at box dst; }\n"
              "  plan { move r to src; grasp r box; move r to dst; place r box at dst; }\n}\n")
    ops = FakeOps()
    record = execute(compile_source(source).ir.tasks[0], dict(bindings, zone="table"), site, ops)
    assert record["failure"]["code"] == "S508" and "region limits" in record["failure"]["message"]
    assert ops.calls == [("guard", "ur")]
