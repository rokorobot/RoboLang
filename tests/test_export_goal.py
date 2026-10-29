"""Tests for `robolang export-goal` and the @verify annotation (spec/export-goal-v0.1.md, decision record 0019)."""

from __future__ import annotations
import io
import json
import os
from pathlib import Path

import pytest

from robolang import RoboLangError, check
from robolang.cli import main
from robolang.core import Context, check_message, time_ms
from robolang.export_goal import simulation_key
from robolang.ir import compile_source, to_dict

REPO_ROOT = Path(__file__).resolve().parent.parent
WORLD = "examples/world.json"
SENDER = "rl:agent:planner-1"
SPEC = (REPO_ROOT / "spec" / "export-goal-v0.1.md").read_text(encoding="utf-8")

HANDOVER = ["examples/handover.rob", "--to", "rl:robot:arm1"]
PICK_AND_PLACE = ["examples/pick_and_place_v13.rob"]
INSPECT = ["examples/inspect_object.rob"]
EXAMPLES = {"handover": HANDOVER, "pick_and_place_v13": PICK_AND_PLACE, "inspect_object": INSPECT}

TASK = """robolang 1.3;
task t(robot r, object box, location src, location dst) {{
{annotations}
    goal {{ {goal} }}
    plan {{ move r to src; grasp r box; move r to dst; place r box at dst; }}
}}
"""
BINDINGS = ["--bind", "r=arm1", "box=box1", "src=shelf", "dst=table"]


def run(*argv: str):
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(REPO_ROOT)
    try:
        code = main(["export-goal", *argv], out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


def export(source: list[str], *extra: str, world: str = WORLD):
    return run(*source, "--world", world, "--from", SENDER, *extra)


def write(tmp_path, name: str, text: str) -> str:
    path = tmp_path / name
    path.write_text(text, encoding="utf-8")
    return str(path)


def task_file(tmp_path, annotations: str = "", goal: str = "object_at box dst;") -> str:
    return write(tmp_path, "t.rob", TASK.format(annotations=annotations, goal=goal))


def receiver_context(message: dict) -> Context:
    key_ref = message["signature"]["key"]
    return Context(
        receiver=message["recipient"],
        now_ms=time_ms(message["sent_at"]),
        clock_tolerance_ms=500,
        max_message_lifetime_ms=60_000,
        keys={key_ref: simulation_key(key_ref)},
    )


def exported(source: list[str], *extra: str) -> tuple[dict, str]:
    code, out, err = export(source, *extra)
    assert code == 0, err
    return json.loads(out), out


# -------- the message --------

@pytest.mark.parametrize("name", EXAMPLES)
def test_example_message_is_accepted_by_core(name):
    message, out = exported(EXAMPLES[name])
    receipt = check_message(out.encode("utf-8"), receiver_context(message))
    assert receipt.accepted, receipt


@pytest.mark.parametrize("name", EXAMPLES)
def test_desired_state_equals_the_goal_atoms(name):
    message, _ = exported(EXAMPLES[name])
    goal = message["payload"]["goal"]
    expected = {
        "handover": [("rl:thing:box1", "located_in", "rl:place:table")],
        "pick_and_place_v13": [("rl:thing:box1", "located_in", "rl:place:table")],
        "inspect_object": [("rl:robot:arm1", "located_in", "rl:place:shelf")],
    }[name]
    triples = [(t["subject"], t["property"], t["value"]) for t in goal["desired_state"]]
    assert triples == expected
    criteria = [(c["subject"], c["property"], c["value"]) for c in goal["success_criteria"]]
    assert criteria == expected


def test_envelope_and_goal_fields():
    message, _ = exported(HANDOVER)
    assert message["type"] == "propose_goal" and message["profile"] == "a2r" and message["sequence"] == 1
    assert message["in_reply_to"] is None
    assert (message["sender"], message["recipient"]) == (SENDER, "rl:robot:arm1")
    assert message["sent_at"] == "2000-01-01T00:00:00.000Z"
    assert message["expires_at"] == "2000-01-01T00:00:30.000Z"
    assert message["signature"]["alg"] == "sim-hmac-sha256" and message["signature"]["key"] == f"{SENDER}#k1"
    goal = message["payload"]["goal"]
    assert goal["id"] == "rl:goal:handover" and goal["requested_by"] == SENDER and goal["priority"] == "normal"
    assert goal["constraints"] == [{"kind": "max_speed", "limit": {"value": "0.5", "unit": "m/s"}}]
    assert message["payload"]["things"] == [
        {"kind": "thing", "id": "rl:thing:box1", "class": "box",
         "mass": {"value": "2", "unit": "kg", "uncertainty": "0.1"}}
    ]


def test_at_sets_sent_at_and_expiry():
    message, _ = exported(PICK_AND_PLACE, "--at", "2026-10-19T10:14:00.400Z")
    assert message["sent_at"] == "2026-10-19T10:14:00.400Z"
    assert message["expires_at"] == "2026-10-19T10:14:30.400Z"
    assert check_message(json.dumps(message).encode(), receiver_context(message)).accepted


def test_recipient_defaults_to_the_only_robot():
    message, _ = exported(PICK_AND_PLACE)
    assert message["recipient"] == "rl:robot:arm1"


def test_several_robots_without_to_is_a_usage_error():
    code, out, err = export(["examples/handover.rob"])
    assert code == 2 and out == ""
    assert "--to" in err and "arm, mobile" in err


def test_bad_arguments_are_usage_errors():
    assert export(PICK_AND_PLACE, "--at", "yesterday")[0] == 2
    assert run("examples/pick_and_place_v13.rob", "--world", WORLD, "--from", "planner")[0] == 2
    assert run("examples/pick_and_place_v13.rob", "--world", WORLD, "--from", f"{SENDER}#k1")[0] == 2
    assert export(PICK_AND_PLACE, "--to", "arm1")[0] == 2


def test_output_is_deterministic():
    first = exported(HANDOVER)[1]
    assert exported(HANDOVER)[1] == first
    assert exported(HANDOVER, "--at", "2026-10-19T10:14:00Z")[1] != first


def test_ids_are_derived_from_the_content():
    one, _ = exported(PICK_AND_PLACE)
    other, _ = exported(PICK_AND_PLACE, "--to", "rl:robot:amr1")
    assert one["message_id"] != other["message_id"]
    assert one["conversation_id"] != other["conversation_id"]


def test_bind_replaces_the_world_bindings(tmp_path):
    source = task_file(tmp_path)
    code, out, err = export([source], *BINDINGS)
    assert code == 0, err
    assert json.loads(out)["payload"]["goal"]["id"] == "rl:goal:t"


def test_a_wrong_signature_key_is_rejected():
    message, out = exported(PICK_AND_PLACE)
    context = receiver_context(message)
    wrong = {f"{SENDER}#k1": simulation_key("rl:agent:other#k1")}
    other = Context(context.receiver, context.now_ms, 500, 60_000, wrong)
    assert not check_message(out.encode("utf-8"), other).accepted


# -------- @verify in the message --------

def test_verify_values_appear_in_every_criterion(tmp_path):
    annotations = "@verify min_confidence 0.75;\n    @verify max_age 2500ms;"
    source = task_file(tmp_path, annotations, "object_at box dst; robot_at r dst;")
    message, out = exported([source], *BINDINGS)
    criteria = message["payload"]["goal"]["success_criteria"]
    assert len(criteria) == 2
    assert all(c["min_confidence"] == "0.75" and c["max_age"] == {"value": "2500", "unit": "ms"} for c in criteria)
    assert all(c["accepted_sources"] == ["any_participant_in_domain"] for c in criteria)
    assert check_message(out.encode("utf-8"), receiver_context(message)).accepted


def test_one_verify_key_keeps_the_default_for_the_other(tmp_path):
    message, _ = exported([task_file(tmp_path, "@verify max_age 1min;")], *BINDINGS)
    criterion = message["payload"]["goal"]["success_criteria"][0]
    assert (criterion["min_confidence"], criterion["max_age"]) == ("0.9", {"value": "1", "unit": "min"})


def test_defaults_without_verify():
    message, _ = exported(PICK_AND_PLACE)
    criterion = message["payload"]["goal"]["success_criteria"][0]
    assert (criterion["min_confidence"], criterion["max_age"]) == ("0.9", {"value": "10", "unit": "s"})
    assert '**`"0.9"`**' in SPEC and '**`{ "value": "10", "unit": "s" }`**' in SPEC


# -------- constraints --------

def test_rad_and_unitless_limits_are_skipped_with_a_warning(tmp_path):
    source = task_file(tmp_path, "@safety max_speed 1 rad/s;")
    code, out, err = export([source], *BINDINGS)
    assert code == 0
    assert json.loads(out)["payload"]["goal"]["constraints"] == []
    assert "skipped max_speed 1 rad/s" in err
    unitless = task_file(tmp_path, "@safety max_speed 1;")
    code, out, err = export([unitless], *BINDINGS)
    assert code == 0 and json.loads(out)["payload"]["goal"]["constraints"] == []
    assert "skipped max_speed 1 (no unit)" in err


# -------- errors --------

@pytest.mark.parametrize(
    ("goal", "text"),
    [
        ("object_at box dst; object_at box src or robot_at r dst;", "goal condition 2 ('or')"),
        ("robot_ready r;", "goal condition 1 ('robot_ready')"),
        ("object_at box dst; not holding r box;", "goal condition 2 ('not')"),
    ],
)
def test_goal_without_a_core_form_exits_6(tmp_path, goal, text):
    code, out, err = export([task_file(tmp_path, goal=goal)], *BINDINGS)
    assert code == 6 and out == ""
    assert "X601" in err and text in err


def test_and_goal_exports_the_same_criteria_as_separate_entries(tmp_path):
    separate = task_file(tmp_path, goal="object_at box dst; robot_at r dst;")
    joined = task_file(tmp_path, goal="object_at box dst and robot_at r dst;")
    first = json.loads(export([separate], *BINDINGS)[1])["payload"]["goal"]
    second = json.loads(export([joined], *BINDINGS)[1])["payload"]["goal"]
    assert second["desired_state"] == first["desired_state"]
    assert second["success_criteria"] == first["success_criteria"]


def test_missing_mass_exits_6(tmp_path):
    world = json.loads((REPO_ROOT / WORLD).read_text(encoding="utf-8"))
    del world["objects"]["box1"]["mass"]
    code, out, err = export(PICK_AND_PLACE, world=write(tmp_path, "w.json", json.dumps(world)))
    assert code == 6 and out == ""
    assert "X602" in err and "box1" in err and "no mass" in err


def test_missing_class_exits_6(tmp_path):
    world = json.loads((REPO_ROOT / WORLD).read_text(encoding="utf-8"))
    del world["objects"]["box1"]["class"]
    code, _, err = export(PICK_AND_PLACE, world=write(tmp_path, "w.json", json.dumps(world)))
    assert code == 6 and "no class" in err


def test_a_1_0_world_has_no_class_or_mass(tmp_path):
    world = json.loads((REPO_ROOT / WORLD).read_text(encoding="utf-8"))
    world["world_schema_version"] = "1.0"
    code, _, err = export(PICK_AND_PLACE, world=write(tmp_path, "w.json", json.dumps(world)))
    assert code == 3 and "E503" in err and "class" in err


def test_invalid_source_exits_1_and_invalid_world_exits_3(tmp_path):
    assert export([write(tmp_path, "bad.rob", "robolang 1.3;\n")])[0] == 1
    assert export(PICK_AND_PLACE, world=write(tmp_path, "w.json", "{}"))[0] == 3


def test_a_1_2_source_has_no_goal(tmp_path):
    source = write(tmp_path, "old.rob", "robolang 1.2;\ntask t(robot r, location a) { plan { move r to a; } }\n")
    world = json.loads((REPO_ROOT / WORLD).read_text(encoding="utf-8"))
    world["tasks"]["t"] = {"bindings": {"r": "arm1", "a": "home"}}
    code, _, err = export([source], world=write(tmp_path, "w.json", json.dumps(world)))
    assert code == 6 and "X601" in err and "no goal" in err


# -------- the @verify language additions --------

def codes(source: str) -> list[tuple[str, int]]:
    try:
        check(source)
    except RoboLangError as error:
        return [(d.code, d.line) for d in error.diagnostics]
    return []


def program(annotations: str, version: str = "1.3") -> str:
    return f"robolang {version};\ntask t(robot r, object b, location l) {{\n{annotations}\n" \
           "    goal { object_at b l; }\n    plan { move r to l; }\n}\n"


def test_verify_is_valid_in_1_3():
    assert codes(program("@verify min_confidence 0.95;\n@verify max_age 10s;\n@safety max_speed 1 m/s;")) == []
    assert codes(program("@verify min_confidence 1;\n@verify min_confidence 0;")) == [("E311", 4)]


def test_verify_errors():
    assert codes(program("@verify min_confidence 1.5;")) == [("E312", 3)]
    assert codes(program("@verify min_confidence 0.5;\n@verify min_confidence 0.6;")) == [("E311", 4)]
    assert codes(program("@verify max_age 1s;\n@verify max_age 2s;")) == [("E311", 4)]
    assert codes(program("@verify max_age 0s;")) == [("E305", 3)]
    assert codes(program("@verify max_age 5;")) == [("E206", 3)]
    assert codes(program("@verify max_age 5 m/s;")) == [("E206", 3)]
    assert codes(program("@verify confidence 0.5;")) == [("E207", 3)]
    assert codes(program("@verify min_confidence 0.5")) == [("E202", 3)]
    assert codes(program("@verify min_confidence;")) == [("E201", 3)]


def test_verify_errors_are_all_reported():
    assert codes(program("@verify min_confidence 2;\n@verify max_age 0s;\n@verify min_confidence 1;")) == [
        ("E312", 3), ("E305", 4), ("E311", 5)]


def test_verify_after_pre_is_rejected():
    source = "robolang 1.3;\ntask t(robot r, location l) {\n pre { robot_ready r; }\n @verify max_age 1s;\n" \
             " goal { robot_at r l; }\n plan { move r to l; }\n}\n"
    assert codes(source) == [("E208", 4)]


@pytest.mark.parametrize("version", ["1.1", "1.2"])
def test_verify_is_not_part_of_older_versions(version):
    source = program("@verify max_age 1s;", version).replace("    goal { object_at b l; }\n", "")
    assert codes(source) == [("E101", 3)]


def test_verify_in_the_ir_only_when_declared():
    declared = compile_source(program("@verify max_age 10s;\n@verify min_confidence 0.90;")).ir
    assert to_dict(declared)["tasks"][0]["verify"] == {"min_confidence": "0.9", "max_age": {"value": "10", "unit": "s"}}
    assert list(to_dict(declared)["tasks"][0])[-3:] == ["goal", "verify", "steps"]
    only_age = compile_source(program("@verify max_age 10s;")).ir
    assert to_dict(only_age)["tasks"][0]["verify"] == {"max_age": {"value": "10", "unit": "s"}}
    assert "verify" not in to_dict(compile_source(program("")).ir)["tasks"][0]


def test_inspect_shows_verify(tmp_path):
    out, err = io.StringIO(), io.StringIO()
    assert main(["inspect", write(tmp_path, "v.rob", program("@verify min_confidence 0.95;"))], out=out, err=err) == 0
    assert "verify:\n    min_confidence 0.95" in out.getvalue()


# -------- world file 1.1 --------

def test_example_world_is_1_1_with_class_and_mass():
    world = json.loads((REPO_ROOT / WORLD).read_text(encoding="utf-8"))
    assert world["world_schema_version"] == "1.1"
    assert world["objects"]["box1"]["class"] == "box"
    assert world["objects"]["box1"]["mass"] == {"value": "2", "unit": "kg", "uncertainty": "0.1"}


@pytest.mark.parametrize(
    ("field", "value", "code"),
    [
        ("class", "Box", "E504"),
        ("class", "box1", "E504"),
        ("class", 3, "E504"),
        ("mass", {"value": "2", "unit": "lb", "uncertainty": "0"}, "E504"),
        ("mass", {"value": "0", "unit": "kg", "uncertainty": "0"}, "E504"),
        ("mass", {"value": "2", "unit": "kg"}, "E504"),
        ("mass", {"value": "2", "unit": "kg", "uncertainty": "-1"}, "E504"),
        ("mass", {"value": "2", "unit": "kg", "uncertainty": "0", "extra": 1}, "E503"),
        ("mass", 2, "E504"),
    ],
)
def test_world_1_1_rejects_a_malformed_class_or_mass(tmp_path, field, value, code):
    world = json.loads((REPO_ROOT / WORLD).read_text(encoding="utf-8"))
    world["objects"]["box1"][field] = value
    _, _, err = export(PICK_AND_PLACE, world=write(tmp_path, "w.json", json.dumps(world)))
    assert f"error[{code}]" in err


def test_simulate_records_the_world_version(tmp_path):
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(REPO_ROOT)
    try:
        assert main(["simulate", "examples/pick_and_place_v13.rob", "--world", WORLD], out=out, err=err) == 0
    finally:
        os.chdir(previous)
    assert json.loads(out.getvalue())["inputs"]["world_schema_version"] == "1.1"
