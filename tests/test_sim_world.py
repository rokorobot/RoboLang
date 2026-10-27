"""Tests for the world-file loader (src/robolang/sim/world.py, spec/simulation-v1.0.md section 3)."""

from __future__ import annotations
import copy
import dataclasses
import json
import subprocess
import sys
from decimal import Decimal
from pathlib import Path

import pytest

from robolang.ir import compile_source
from robolang.sim import Speed, WorldError, load_world

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "simulation"
CASES = json.loads((FIXTURES / "cases.json").read_text(encoding="utf-8"))["cases"]
BASE = json.loads((FIXTURES / "worlds" / "base.json").read_text(encoding="utf-8"))
PICK = "demo_pick_and_place"


def ir_of(path: str):
    return compile_source((ROOT / path).read_bytes().decode("utf-8"), path).ir


PICK_TASK = ir_of("examples/pick_and_place.rob").task(PICK)


def selected(case):
    ir = ir_of(case["source"])
    if case["task"] is not None:
        return ir.task(case["task"])
    return ir.tasks[0] if len(ir.tasks) == 1 else None


def load_file(name: str, task=PICK_TASK):
    path = FIXTURES / "worlds" / name
    return load_world(path.read_bytes(), name, task)


def dump(doc) -> bytes:
    return json.dumps(doc).encode("utf-8")


def mutated(change):
    doc = copy.deepcopy(BASE)
    change(doc)
    return doc


def error_of(doc, task=PICK_TASK) -> WorldError:
    data = doc if isinstance(doc, bytes) else dump(doc)
    with pytest.raises(WorldError) as raised:
        load_world(data, "w.json", task)
    return raised.value


def event(fact, target, value="1", unit="s", flag=True):
    return {"at": {"value": value, "unit": unit}, "fact": fact, "target": target, "value": flag}


# -------- fixtures --------

WORLD_CASES = [c for c in CASES if c.get("source") and c.get("world") and selected(c) is not None]


@pytest.mark.parametrize("case", WORLD_CASES, ids=lambda case: case["id"])
def test_fixture_case(case):
    task = selected(case)
    data = (ROOT / case["world"]).read_bytes()
    if case["expect"]["exit"] == 3:
        with pytest.raises(WorldError) as raised:
            load_world(data, case["world"], task)
        assert raised.value.code == case["expect"]["diagnostic"]
    else:
        load_world(data, case["world"], task)


def test_base_world_content():
    world = load_file("base.json")
    assert world.locations == ("home", "shelf", "table")
    assert {k: v.clear for k, v in world.regions.items()} == {"cell": True}
    arm1, arm2 = world.robots["arm1"], world.robots["arm2"]
    assert (arm1.ready, arm1.location, arm1.gripper) == (True, "home", True)
    assert arm1.max_speed == (Speed(Decimal("1"), "m/s"),)
    assert (arm2.ready, arm2.location, arm2.gripper, arm2.max_speed) == (True, "home", False, ())
    box = world.objects["box1"]
    assert (box.present, box.location, box.held_by) == (True, "shelf", None)
    camera = world.sensors["camera_top"]
    assert (camera.robot, camera.available) == ("arm1", True)
    assert world.events == ()
    assert world.task.name == PICK
    assert world.task.bindings == (("r", "arm1"), ("box", "box1"), ("src", "shelf"), ("dst", "table"), ("cell", "cell"))
    assert world.task.faults == () and world.task.cancel_before_step is None
    kinds = [world.kind_of(i) for i in ("home", "cell", "arm1", "box1", "nope")]
    assert kinds == ["location", "region", "robot", "object", None]


def test_loaded_world_is_read_only():
    """A validated world is a trustworthy starting snapshot: callers cannot change it."""
    world = load_file("base.json")
    arm2 = world.robots["arm2"]
    for mapping, key in ((world.robots, "arm1"), (world.objects, "box1"), (world.regions, "cell"),
                         (world.sensors, "camera_top")):
        with pytest.raises(TypeError):
            mapping[key] = arm2
        with pytest.raises(TypeError):
            del mapping[key]
        with pytest.raises(TypeError):
            mapping["new"] = arm2
    with pytest.raises(dataclasses.FrozenInstanceError):
        world.robots = {}
    with pytest.raises(dataclasses.FrozenInstanceError):
        world.robots["arm1"].ready = False
    assert world == load_file("base.json")


def test_events_and_script_fixtures():
    events = load_file("event_2500ms.json").events
    assert [e.at_ms for e in events] == [Decimal("2500")]
    assert type(events[0].at_ms) is Decimal
    assert [e.at_ms for e in load_file("event_5s.json").events] == [Decimal("5000")]
    assert load_file("fault_step_3.json").task.faults == (3,)
    assert load_file("cancel_step_2.json").task.cancel_before_step == 2


def test_unknown_facts():
    assert load_file("region_unknown.json").regions["cell"].clear is None
    assert load_file("ready_unknown.json").robots["arm1"].ready is None
    assert load_file("sensor_unknown.json").sensors["camera_top"].available is None


def test_unknown_field_fixture_pointer():
    with pytest.raises(WorldError) as raised:
        load_file("unknown_field.json")
    assert (raised.value.code, raised.value.pointer) == ("E503", "/robots/arm1/colour")


def test_events_are_stably_sorted():
    def change(doc):
        doc["events"] = [
            event("region_clear", "cell", "2", "s", False),
            event("robot_ready", "arm1", "500", "ms", False),
            event("sensor_available", "camera_top", "2000", "ms", False),
        ]

    world = load_world(dump(mutated(change)), "w.json", PICK_TASK)
    assert [e.target for e in world.events] == ["arm1", "cell", "camera_top"]


def test_faults_are_sorted():
    def change(doc):
        doc["tasks"][PICK]["faults"] = [{"step": 4}, {"step": 2}]

    assert load_world(dump(mutated(change)), "w.json", PICK_TASK).task.faults == (2, 4)


def test_bom_is_ignored():
    assert load_world(b"\xef\xbb\xbf" + dump(BASE), "w.json", PICK_TASK).task.name == PICK


# -------- E501 --------

def test_e501_not_utf8():
    assert error_of(b"\xff\xfe{}").code == "E501"


def test_e501_not_json():
    assert error_of(b"{").code == "E501"


def test_e501_duplicate_key():
    data = dump(BASE).replace(b'"locations"', b'"regions": {}, "locations"', 1)
    assert error_of(data).code == "E501"


def test_e501_nan():
    assert error_of(b'{"world_schema": NaN}').code == "E501"


# -------- E502 to E504 --------

def test_e502_missing_schema():
    error = error_of(mutated(lambda d: d.pop("world_schema")))
    assert (error.code, error.pointer) == ("E502", "")


def test_e502_wrong_version_precedes_other_errors():
    def change(doc):
        doc["world_schema_version"] = "2.0"
        doc["colour"] = 1

    error = error_of(mutated(change))
    assert (error.code, error.pointer) == ("E502", "/world_schema_version")


def test_e503_unknown_top_level_field():
    error = error_of(mutated(lambda d: d.update(extra=1)))
    assert (error.code, error.pointer) == ("E503", "/extra")


def test_e503_unknown_event_field():
    def change(doc):
        doc["events"] = [dict(event("robot_ready", "arm1"), x=1)]

    error = error_of(mutated(change))
    assert (error.code, error.pointer) == ("E503", "/events/0/x")


def test_e504_top_level_not_object():
    error = error_of(b"[]")
    assert (error.code, error.pointer) == ("E504", "")


def test_e504_float_step():
    error = error_of(mutated(lambda d: d["tasks"][PICK].update(faults=[{"step": 3.0}])))
    assert (error.code, error.pointer) == ("E504", f"/tasks/{PICK}/faults/0/step")


def test_e504_bool_step():
    assert error_of(mutated(lambda d: d["tasks"][PICK].update(cancel={"before_step": True}))).code == "E504"


def test_e504_missing_gripper():
    error = error_of(mutated(lambda d: d["robots"]["arm1"].pop("gripper")))
    assert (error.code, error.pointer) == ("E504", "/robots/arm1")


def test_e504_bad_quantity_value():
    error = error_of(mutated(lambda d: d["robots"]["arm1"]["max_speed"][0].update(value="1e3")))
    assert (error.code, error.pointer) == ("E504", "/robots/arm1/max_speed/0/value")


def test_e504_zero_quantity():
    assert error_of(mutated(lambda d: d["robots"]["arm1"]["max_speed"][0].update(value="0"))).code == "E504"


def test_e504_bad_unit():
    assert error_of(mutated(lambda d: d["robots"]["arm1"]["max_speed"][0].update(unit="ms"))).code == "E504"


def test_e504_bad_id():
    error = error_of(mutated(lambda d: d["locations"].append("9x")))
    assert (error.code, error.pointer) == ("E504", "/locations/3")


def test_e504_bad_boolean():
    assert error_of(mutated(lambda d: d["regions"]["cell"].update(clear="yes"))).code == "E504"


def test_e504_bad_event_fact():
    assert error_of(mutated(lambda d: d.update(events=[event("nope", "arm1")]))).code == "E504"


# -------- E505 to E509 --------

def test_e505_robot_location_undeclared():
    error = error_of(mutated(lambda d: d["robots"]["arm1"].update(location="mars")))
    assert (error.code, error.pointer) == ("E505", "/robots/arm1/location")


def test_e505_robot_location_is_a_region():
    assert error_of(mutated(lambda d: d["robots"]["arm1"].update(location="cell"))).code == "E505"


def test_e505_object_held_by_a_non_robot():
    assert error_of(mutated(lambda d: d["objects"].update(box1={"present": True, "held_by": "cell"}))).code == "E505"


def test_e505_sensor_robot_undeclared():
    assert error_of(mutated(lambda d: d["sensors"]["camera_top"].update(robot="arm9"))).code == "E505"


def test_e505_event_target_undeclared():
    assert error_of(mutated(lambda d: d.update(events=[event("sensor_available", "ghost")]))).code == "E505"


def test_e505_binding_undeclared():
    error = error_of(mutated(lambda d: d["tasks"][PICK]["bindings"].update(r="arm9")))
    assert (error.code, error.pointer) == ("E505", f"/tasks/{PICK}/bindings/r")


def test_e506_no_entry_for_task():
    error = error_of(mutated(lambda d: d["tasks"].pop(PICK)))
    assert (error.code, error.pointer) == ("E506", "/tasks")


def test_e506_unbound_parameter():
    error = error_of(mutated(lambda d: d["tasks"][PICK]["bindings"].pop("cell")))
    assert (error.code, error.pointer) == ("E506", f"/tasks/{PICK}/bindings")


def test_e507_region_bound_to_location_parameter():
    error = error_of(mutated(lambda d: d["tasks"][PICK]["bindings"].update(dst="cell")))
    assert (error.code, error.pointer) == ("E507", f"/tasks/{PICK}/bindings/dst")


def test_e508_extra_binding():
    error = error_of(mutated(lambda d: d["tasks"][PICK]["bindings"].update(extra="home")))
    assert (error.code, error.pointer) == ("E508", f"/tasks/{PICK}/bindings/extra")


def test_e509_location_and_robot_share_an_id():
    error = error_of(mutated(lambda d: d["locations"].append("arm1")))
    assert (error.code, error.pointer) == ("E509", "/robots/arm1")


def test_e509_duplicate_location():
    error = error_of(mutated(lambda d: d["locations"].append("home")))
    assert (error.code, error.pointer) == ("E509", "/locations/3")


# -------- E510, E511 --------

def test_e510_present_without_position():
    error = error_of(mutated(lambda d: d["objects"]["box1"].pop("location")))
    assert (error.code, error.pointer) == ("E510", "/objects/box1")


def test_e510_absent_with_position():
    assert error_of(mutated(lambda d: d["objects"]["box1"].update(present=False))).code == "E510"


def test_e510_unknown_presence_with_position():
    assert error_of(mutated(lambda d: d["objects"]["box1"].pop("present"))).code == "E510"


def test_e510_held_by_gripperless_robot():
    error = error_of(mutated(lambda d: d["objects"].update(box1={"present": True, "held_by": "arm2"})))
    assert (error.code, error.pointer) == ("E510", "/objects/box1/held_by")


def test_e510_robot_holds_two_objects():
    def change(doc):
        doc["objects"]["box1"] = {"present": True, "held_by": "arm1"}
        doc["objects"]["box2"] = {"present": True, "held_by": "arm1"}

    error = error_of(mutated(change))
    assert (error.code, error.pointer) == ("E510", "/objects/box2/held_by")


def test_e511_fault_step_out_of_range():
    error = error_of(mutated(lambda d: d["tasks"][PICK].update(faults=[{"step": 9}])))
    assert (error.code, error.pointer) == ("E511", f"/tasks/{PICK}/faults/0/step")


def test_e511_cancel_step_out_of_range():
    assert error_of(mutated(lambda d: d["tasks"][PICK].update(cancel={"before_step": 6}))).code == "E511"


def test_e511_repeated_fault_step():
    error = error_of(mutated(lambda d: d["tasks"][PICK].update(faults=[{"step": 2}, {"step": 2}])))
    assert (error.code, error.pointer) == ("E511", f"/tasks/{PICK}/faults/1/step")


def test_e511_duplicate_max_speed_unit():
    error = error_of(mutated(lambda d: d["robots"]["arm1"]["max_speed"].append({"value": "2", "unit": "m/s"})))
    assert (error.code, error.pointer) == ("E511", "/robots/arm1/max_speed/1")


def test_e511_event_target_of_wrong_kind():
    error = error_of(mutated(lambda d: d.update(events=[event("robot_ready", "cell")])))
    assert (error.code, error.pointer) == ("E511", "/events/0/target")


def test_event_sensor_names_are_their_own_namespace():
    """A sensor may share its name with an entity (3.2); each fact resolves its own kind."""
    def change(doc):
        doc["sensors"]["arm1"] = {"robot": "arm1", "available": True}
        doc["events"] = [event("sensor_available", "arm1"), event("robot_ready", "arm1")]

    assert [e.fact for e in load_world(dump(mutated(change)), "w.json", PICK_TASK).events] == [
        "sensor_available",
        "robot_ready",
    ]
    error = error_of(mutated(lambda d: d.update(events=[event("sensor_available", "arm2")])))
    assert (error.code, error.pointer) == ("E511", "/events/0/target")


# -------- other behaviour --------

def test_other_tasks_are_checked_for_form_only():
    def loose(doc):
        doc["tasks"]["move_only"] = {"bindings": {"zzz": "nowhere"}, "faults": [{"step": 99}]}

    load_world(dump(mutated(loose)), "w.json", PICK_TASK)

    def malformed(doc):
        doc["tasks"]["move_only"] = {"bindings": {"zzz": 1}}

    assert error_of(mutated(malformed)).code == "E504"


def test_pointer_escaping():
    error = error_of(mutated(lambda d: d["sensors"].update({"a/b~c": {"colour": 1}})))
    assert error.pointer == "/sensors/a~1b~0c/colour"


def test_format():
    error = error_of(mutated(lambda d: d["robots"]["arm1"].update(colour="red")))
    assert error.format() == "w.json: error[E503] at /robots/arm1/colour: unknown field 'colour'"
    assert error_of(b"{").format().startswith("w.json: error[E501] at (document): ")


# -------- isolation --------

def test_sim_does_not_load_the_interpreter_or_adapters():
    code = (
        "import sys; sys.path.insert(0, 'src'); import robolang.sim as sim, robolang; "
        "task = robolang.ir.compile_source(open('examples/pick_and_place.rob', encoding='utf-8').read()).ir"
        ".task('demo_pick_and_place'); "
        "sim.load_world(open('tests/fixtures/simulation/worlds/base.json', 'rb').read(), 'base.json', task); "
        "loaded = [m for m in sys.modules if m.startswith('robolang_') or m == 'rclpy']; "
        "assert not loaded, loaded"
    )
    result = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
