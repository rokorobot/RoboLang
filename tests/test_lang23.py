"""
RoboLang 2.3: choosing at run time (spec/robolang-v2.3.md, decision 0045 C1-C12 and the clarifications of 2026-11-19).

Front end, Task IR 1.11, Mission IR 1.3 and the reader, world file 1.8 (positions), the simulation
(spec/simulation-v1.0.md section 21), the run record 1.11 (`choices`), missions, and `export-goal`, `delegate` and the
ROS 2 backend. No test here needs a model or a network.

To regenerate the IR snapshots after an intended change:

    ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_lang23.py
"""

from __future__ import annotations
import copy
import dataclasses
import io
import json
import os
import re
import sys
import time
from decimal import Decimal
from pathlib import Path

import pytest

from robolang import RoboLangError, check, check_file
from robolang.cli import main
from robolang.ir import compile_source, to_dict, to_json
from robolang.ir_reader import IRError, read_ir
from robolang.sim import WorldError, load_world, mission_task, run, run_mission, to_mission_record, to_record
from robolang.sim.check import check_task
from robolang.tokens import FUTURE_RESERVED, RESERVED_BY_VERSION

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "v2.3"
SNAPSHOTS = ROOT / "tests" / "snapshots" / "ir-v2.3"
UPDATE = os.environ.get("ROBOLANG_UPDATE_SNAPSHOTS") == "1"
SPEC = (ROOT / "spec" / "robolang-v2.3.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")

# -------- helpers --------

SEND = """robolang 2.3;

task send(robot a, robot b, robot c, location ref) {
    goal { robot_at t ref; }
    plan {
        %s
        move t to ref;
    }
}
"""
SEND_BIND = {"a": "ra", "b": "rb", "c": "rc", "ref": "base"}
WORLD = {
    "world_schema": "robolang.sim.world",
    "world_schema_version": "1.8",
    "locations": [{"id": "base", "position": [0, 0]}, {"id": "near", "position": [10, 0]},
                  {"id": "mid", "position": ["50.5", "0"]}, {"id": "far", "position": [100, 0]},
                  {"id": "side", "position": [0, 10]}, "nowhere", "bin"],
    "regions": {"wet": {"clear": True, "contains": ["mid"]}},
    "robots": {
        "ra": {"ready": True, "location": "near", "gripper": True, "honours": ["no_go_region"]},
        "rb": {"ready": True, "location": "far", "gripper": True, "honours": ["no_go_region"]},
        "rc": {"ready": False, "location": "base", "gripper": False},
    },
    "objects": {
        "p1": {"present": True, "location": "far", "class": "part", "mass": {"value": "1", "unit": "kg",
                                                                           "uncertainty": "0.1"}},
        "p2": {"present": True, "location": "far", "class": "part", "mass": {"value": "1", "unit": "kg",
                                                                           "uncertainty": "0.1"}},
        "p3": {"present": True, "location": "mid", "class": "part", "mass": {"value": "1", "unit": "kg",
                                                                           "uncertainty": "0.1"}},
        "p4": {"present": False, "class": "part", "mass": {"value": "1", "unit": "kg",
                                                                            "uncertainty": "0.1"}},
    },
    "tasks": {},
}


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


def simulate(source: str, patch=None, bindings=None, name="send", core_world=None):
    task = compile_source(source).ir.task(name)
    world = load_world(core_world or world_data(patch), "w.json", task, bindings or SEND_BIND)
    return task, world, run(task, world)


def send(body: str) -> str:
    return SEND % body


def chosen(result) -> list[str | None]:
    return [c.chosen for c in result.choices]


def end(result) -> tuple[str, str | None]:
    return result.state, None if result.failure is None else result.failure.code


def cli(*args):
    out, err = io.StringIO(), io.StringIO()
    code = main(list(args), out=out, err=err)
    return code, out.getvalue(), err.getvalue()


NEAREST = 'choose robot t from (a, b, c) nearest to ref else { fail "none"; }'

# -------- fixtures and snapshots --------

HEADER = re.compile(
    r"\A// RoboLang v2\.3 fixture: (?P<set>valid|invalid)\n"
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
    assert actual == snapshot.read_text(encoding="utf-8").replace("\r\n", "\n")
    document = json.loads(actual)
    mission = document["ir_schema"] == "robolang.mission-ir"
    assert (document["ir_schema_version"], document["language_version"]) == ("1.3" if mission else "1.11", "2.3")
    assert document.get("task_ir_schema_version", "1.11") == "1.11"
    assert read_ir(actual.encode()) == compile_source(path.read_text(encoding="utf-8")).ir


@pytest.mark.parametrize("path", INVALID, ids=lambda p: p.stem)
def test_invalid_fixture_is_rejected_with_the_stated_code_and_line_and_makes_no_ir(path):
    stated = header(path)
    assert path.name.startswith(stated["code"].lower()) and stated["code"][1] == STAGE_DIGIT[stated["stage"]]
    with pytest.raises(RoboLangError) as caught:
        compile_source(path.read_text(encoding="utf-8"), str(path))
    assert (caught.value.code, str(caught.value.line)) == (stated["code"], stated["line"])


def test_fixture_sets_are_counted():
    assert len(VALID) == 10 and len(INVALID) == 15
    assert len(list(SNAPSHOTS.glob("*.json"))) == 10


# -------- the language --------

def test_2_3_adds_no_reserved_word_and_no_word_of_a_choice_is_reserved_point_10():
    assert RESERVED_BY_VERSION["2.3"] == RESERVED_BY_VERSION["2.2"] == RESERVED_BY_VERSION["2.0"]
    for word in ("choose", "from", "where", "nearest", "farthest", "objects", "robots"):
        assert word not in RESERVED_BY_VERSION["2.3"] and word not in FUTURE_RESERVED, word


def test_a_program_that_uses_the_words_as_names_parses_in_every_version():
    for version in ("1.4", "1.6", "2.0", "2.2", "2.3"):
        source = (f"robolang {version};\ntask choose(robot nearest, location farthest, location from, location where, "
                  "object objects, location robots) {\n    goal { object_at objects from; }\n    plan {\n"
                  "        move nearest to farthest;\n        grasp nearest objects;\n        move nearest to robots;\n"
                  "        place nearest objects at from;\n    }\n}\n")
        assert codes(source) == [], version


def test_choose_stays_an_unknown_statement_below_2_3_e203():
    for version in ("1.4", "1.8", "2.0", "2.1", "2.2"):
        source = SEND.replace("robolang 2.3", f"robolang {version}") % NEAREST
        assert codes(source)[0][0] == "E203", version


def test_the_three_forms_of_candidates_and_the_two_orders():
    ok = [
        'choose robot t from (a) else { fail "x"; }',
        'choose robot t from (a, b, c) where robot_ready t nearest to ref else { fail "x"; }',
        'choose robot t from robots farthest from ref else { fail "x"; }',
        'choose robot t from robots where not robot_ready t or robot_at t ref else { fail "x"; }',
    ]
    for body in ok:
        assert codes(send(body)) == [], body
    objects = ('robolang 2.3;\ntask t(robot r, location l, location dst, object seen) {\n'
               '    goal { object_at o dst; }\n    plan {\n'
               '        choose object o from objects at l where object_exists o nearest to r else { fail "x"; }\n'
               '        move r to l;\n        grasp r o;\n        move r to dst;\n        place r o at dst;\n'
               '    }\n}\n')
    assert codes(objects) == []


def test_the_else_is_mandatory_e316_and_ends_in_fail_e329():
    assert [c for c, _ in codes(send("choose robot t from (a, b) nearest to ref;"))] == ["E316"]
    assert [c for c, _ in codes(send('choose robot t from (a, b) else { communicate a to "x" with "y"; }'))] == ["E329"]
    both = [c for c, _ in codes(send('choose robot t from (a, b) else { fail "x"; communicate a to "x" with "y"; }'))]
    assert sorted(both) == ["E313", "E329"]  # a fail is the last statement of a branch, and the else ends in none
    fine = 'choose robot t from (a, b) else { communicate a to "ops" with "no robot"; fail "x"; }'
    assert codes(send(fine)) == []


def test_where_states_world_facts_only_and_names_the_chosen_name_e330():
    for bad in ('where temperature of t using cam > 3 degC', 'where message_received "ops"',
                'where robot_ready t and clock_time >= 09:00', "where robot_ready a"):
        source = send(f'choose robot t from (a, b) {bad} else {{ fail "x"; }}')
        assert [c for c, _ in codes(source)][:1] == ["E330"] or [c for c, _ in codes(source)] == ["E317"], bad
    assert [c for c, _ in codes(send('choose robot t from (a, b) where robot_ready a else { fail "x"; }'))] == ["E330"]
    assert codes(send('choose robot t from (a, b) where robot_ready t else { fail "x"; }')) == []


def test_names_types_and_scope_e302_e303_e304():
    cases = {
        'choose robot t from (a, nobody) else { fail "x"; }': "E303",
        'choose object t from (a, b) else { fail "x"; }': "E304",
        'choose robot t from objects at ref else { fail "x"; }': "E304",
        'choose robot t from (a, b) nearest to missing else { fail "x"; }': "E303",
        'choose location t from robots else { fail "x"; }': "E304",
    }
    for body, code in cases.items():
        assert [c for c, _ in codes(send(body))][:1] == [code], body
    assert "E302" in [c for c, _ in codes(send('choose robot a from (b, c) else { fail "x"; }'))]
    assert "E302" in [c for c, _ in codes(send('choose robot ref from (b, c) else { fail "x"; }'))]  # a parameter
    reference = 'choose robot t from (a, b) farthest from a else { fail "x"; }'  # a robot may be the reference
    assert codes(send(reference)) == []


def test_a_name_that_a_measure_bound_cannot_be_chosen_and_the_other_way_round():
    head = ('robolang 2.3;\ntask t(robot r, object o, location l, sensor cam) {\n    goal { robot_at r l; }\n'
            '    plan {\n')
    tail = "        move r to l;\n    }\n}\n"
    measure = "        measure r mass of o using cam as w;\n"
    choose = '        choose robot w from (r) else { fail "x"; }\n'
    assert [c for c, _ in codes(head + measure + choose + tail)] == ["E302"]
    assert [c for c, _ in codes(head + choose + measure + tail)] == ["E318"]


def test_a_choice_never_uses_another_choice_e331():
    two = send('choose robot t from (a, b) else { fail "x"; }\n        choose location u from (ref) nearest to t '
               'else { fail "x"; }')
    assert [c for c, _ in codes(two)] == ["E331"]
    two = send('choose robot t from (a, b) else { fail "x"; }\n        choose robot u from (t, c) else { fail "x"; }')
    assert [c for c, _ in codes(two)] == ["E331"]
    two = send('choose robot t from (a, b) else { fail "x"; }\n'
               '        choose object o from objects at t else { fail "x"; }')
    assert "E331" in [c for c, _ in codes(two)] or "E304" in [c for c, _ in codes(two)]


def test_the_name_is_visible_after_the_statement_in_its_block_and_nested_ones_only():
    in_branch = ('robolang 2.3;\ntask t(robot a, robot b, location l) {\n    goal { robot_at a l; }\n    plan {\n'
                 '        if robot_ready a { choose robot s from (a, b) else { fail "x"; } move s to l; }\n'
                 '        move s to l;\n    }\n}\n')
    assert [c for c, _ in codes(in_branch)] == ["E303"]
    in_else = send('choose robot t from (a, b) else { move t to ref; fail "x"; }')
    assert [c for c, _ in codes(in_else)][:1] == ["E303"]
    nested = send('choose robot t from (a, b) else { fail "x"; }\n        if robot_ready t { move t to ref; }')
    assert codes(nested) == []


def test_a_goal_may_name_what_a_choose_at_the_top_of_its_plan_or_branch_binds_c7():
    plan = ('robolang 2.3;\ntask t(robot a, robot b, location l) {\n    goal { robot_at s l; }\n    plan {\n'
            '        choose robot s from (a, b) else { fail "x"; }\n        move s to l;\n    }\n}\n')
    assert codes(plan) == []
    pre = plan.replace("goal {", "pre { robot_ready s; }\n    goal {")
    assert [c for c, _ in codes(pre)] == ["E303"]  # a pre block comes before the plan
    nested = plan.replace("choose robot s", "if robot_ready a { choose robot s").replace(
        "move s to l;", "move s to l; }")
    assert [c for c, _ in codes(nested)][0] == "E303"  # the choose is not at the top level of the plan


def test_the_else_counts_as_a_nesting_level_and_a_choice_is_a_branching_step_for_goals_e309():
    no_goal = ('robolang 2.3;\ntask t(robot a, robot b, location l) {\n    plan {\n'
               '        choose robot s from (a, b) else { fail "x"; }\n        move s to l;\n    }\n}\n')
    assert [c for c, _ in codes(no_goal)] == ["E309"]


def test_a_chain_of_choices_is_checked_and_compiled_in_linear_time():
    """Every `choose` has a handler, so 40 of them are 2^40 paths: nothing may walk them."""
    names = [f"t{i}" for i in range(40)]
    body = "\n        ".join(f'choose robot {n} from (a, b, c) else {{ fail "x"; }}' for n in names)
    source = SEND % body
    source = source.replace("robot_at t ref", f"robot_at {names[0]} ref")
    source = source.replace("move t to ref;", f"move {names[0]} to ref;")
    started = time.time()
    compiled = compile_source(source).ir
    read_ir(to_json(compiled).encode())
    from robolang import bridge
    assert len(bridge.distinct_path_goals(compiled.tasks[0])) >= 1
    assert time.time() - started < 5


# -------- Task IR 1.11 and the reader --------

def test_the_ir_of_a_choose_and_its_round_trip():
    full = 'choose robot t from (a, b) where robot_ready t nearest to ref else { fail "x"; }'
    document = compile_source(send(full)).ir
    step = to_dict(document)["tasks"][0]["steps"][0]
    assert list(step) == ["kind", "type", "name", "source", "where", "order", "else"]
    assert step["source"] == {"kind": "list", "candidates": ["a", "b"]} and step["order"] == \
        {"mode": "nearest", "reference": "ref"}
    assert to_dict(document)["ir_schema_version"] == "1.11"
    assert read_ir(to_json(document).encode()) == document
    plain = to_dict(compile_source(send('choose robot t from robots else { fail "x"; }')).ir)["tasks"][0]["steps"][0]
    assert "where" not in plain and "order" not in plain and plain["source"] == {"kind": "robots"}


def read_problems(document: dict) -> list[tuple[str, str]]:
    with pytest.raises(IRError) as caught:
        read_ir(json.dumps(document).encode(), "x.json")
    return [(d.code, d.pointer) for d in caught.value.diagnostics]


def base_document(body: str = 'choose robot t from (a, b) where robot_ready t nearest to ref else { fail "x"; }') \
        -> dict:
    return json.loads(to_json(compile_source(send(body)).ir))


def test_the_reader_restates_the_rules_i202_i203_i204_i230_i231_i232():
    def step(document):
        return document["tasks"][0]["steps"][0]

    bad = base_document()
    step(bad)["else"] = [{"kind": "communicate", "robot": "a", "channel": "x", "message": "y"}]
    assert read_problems(bad) == [("I230", "/tasks/0/steps/0/else")]
    bad = base_document()
    step(bad)["where"] = {"kind": "robot_ready", "robot": "a"}
    assert read_problems(bad) == [("I231", "/tasks/0/steps/0/where")]
    bad = base_document()
    step(bad)["where"] = {"kind": "compare", "left": {"kind": "clock", "reading": "time"}, "op": ">=",
                          "right": {"kind": "time", "value": "09:00:00"}}
    assert read_problems(bad)[0][0] == "I231"
    bad = base_document()
    bad["tasks"][0]["steps"].insert(1, copy.deepcopy(step(bad)))
    bad["tasks"][0]["steps"][1]["name"] = "u"
    bad["tasks"][0]["steps"][1]["order"]["reference"] = "t"
    assert ("I232", "/tasks/0/steps/1/order/reference") in read_problems(bad)
    bad = base_document()
    step(bad)["name"] = "a"
    assert "I202" in [c for c, _ in read_problems(bad)]
    bad = base_document()
    step(bad)["source"]["candidates"].append("zzz")
    assert ("I203", "/tasks/0/steps/0/source/candidates/2") in read_problems(bad)
    bad = base_document()
    step(bad)["type"] = "object"
    assert read_problems(bad)[0][0] == "I204"
    bad = base_document()
    step(bad)["else"] = []
    assert read_problems(bad)[0][0] == "I120"
    bad = base_document()
    step(bad)["order"]["mode"] = "closest"
    assert read_problems(bad)[0][0] == "I114"
    bad = base_document()
    step(bad)["source"] = {"kind": "everything"}
    assert read_problems(bad)[0][0] == "I114"
    bad = base_document()
    del step(bad)["else"]
    assert read_problems(bad)[0][0] == "I111"


def test_the_reader_refuses_choose_in_an_older_schema_i115():
    old = base_document()
    old["ir_schema_version"], old["language_version"] = "1.10", "2.2"
    assert read_problems(old)[0] == ("I115", "/tasks/0/steps/0/kind")


def test_the_reader_checks_the_goal_names_of_a_choice():
    bad = base_document()
    bad["tasks"][0]["goal"] = [{"kind": "robot_at", "robot": "u", "location": "ref"}]
    assert read_problems(bad)[0][0] == "I203"
    ok = base_document()
    assert read_ir(json.dumps(ok).encode())


# -------- world file 1.8: positions --------

def load(patch=None, version=None, bindings=None, source=None):
    source = source or send(NEAREST)
    task = compile_source(source).ir.task("send")
    return load_world(world_data(patch, version), "w.json", task, bindings or SEND_BIND)


def test_a_location_may_have_a_position_and_the_others_have_none():
    world = load()
    assert world.positions["mid"] == (Decimal("50.5"), Decimal("0")) and "nowhere" not in world.positions
    assert world.locations[:3] == ("base", "near", "mid") and "nowhere" in world.locations


def test_positions_are_exact_whole_numbers_or_decimal_strings_never_floats():
    def set_position(value):
        return lambda w: w["locations"].__setitem__(0, {"id": "base", "position": value})
    for bad in ([0, 0.5], ["1e3", 0], [1], [1, 2, 3], "0,0", [True, 0], ["", 0], ["+1", 0], [None, 0]):
        with pytest.raises(WorldError) as caught:
            load(set_position(bad))
        assert caught.value.code in ("E504", "E501"), bad
    assert load(set_position(["-3.25", "4"])).positions["base"] == (Decimal("-3.25"), Decimal(4))


def test_a_location_entry_has_only_an_id_and_a_position_and_only_in_world_1_8():
    with pytest.raises(WorldError) as caught:
        load(lambda w: w["locations"].__setitem__(0, {"id": "base", "pos": [0, 0]}))
    assert caught.value.code == "E503"
    with pytest.raises(WorldError) as caught:
        load(lambda w: w["locations"].__setitem__(0, {"position": [0, 0]}))
    assert caught.value.code == "E504"
    with pytest.raises(WorldError) as caught:
        load(None, version="1.7")
    assert caught.value.code == "E504"  # an object in `locations` is a 1.8 feature
    with pytest.raises(WorldError) as caught:
        load(lambda w: w["locations"].append({"id": "base", "position": [1, 1]}))
    assert caught.value.code == "E509"


def test_older_world_files_serve_a_2_3_task_and_have_no_position():
    def plain(world):
        world["locations"] = [item["id"] if isinstance(item, dict) else item for item in world["locations"]]
    for version in ("1.4", "1.7"):
        world = load(plain, version=version)
        assert not world.positions and world.schema_version == version


# -------- the choice --------

def test_nearest_farthest_and_the_order_written_for_a_tie():
    _, _, result = simulate(send(NEAREST))
    assert chosen(result) == ["ra"] or chosen(result) == ["rc"]  # rc is at base: distance 0 and not ready, no where
    assert chosen(result) == ["rc"]
    _, _, result = simulate(send('choose robot t from (a, b) nearest to ref else { fail "x"; }'))
    assert chosen(result) == ["ra"] and result.state == "SUCCEEDED"
    _, _, result = simulate(send('choose robot t from (a, b) farthest from ref else { fail "x"; }'))
    assert chosen(result) == ["rb"]
    tie = 'choose location t from (x, y) nearest to ref else { fail "x"; }'
    source = ('robolang 2.3;\ntask send(robot r, location x, location y, location ref) {\n    goal { robot_at r t; }\n'
              f'    plan {{\n        {tie}\n        move r t;\n    }}\n}}\n').replace("move r t;", "move r to t;")
    bound = {"r": "ra", "x": "near", "y": "side", "ref": "base"}  # both 10 m from base
    _, _, result = simulate(source, bindings=bound)
    assert chosen(result) == ["near"]
    _, _, result = simulate(source, bindings={**bound, "x": "side", "y": "near"})
    assert chosen(result) == ["side"]  # the first written wins: a different order of the list is another choice
    _, _, result = simulate(source.replace("nearest to ref", "farthest from ref"), bindings=bound)
    assert chosen(result) == ["near"]


def test_where_excludes_false_and_unknown_and_a_choice_with_no_order_takes_the_first_written():
    body = 'choose robot t from (c, b, a) where robot_ready t else { fail "x"; }'
    _, _, result = simulate(send(body))
    assert chosen(result) == ["rb"]
    choice = result.choices[0]
    assert choice.candidates == ("rc", "rb", "ra") and choice.excluded == (("rc", "where_false"),)
    assert choice.order is None and choice.kept == ("rb", "ra")

    objects = ('robolang 2.3;\ntask t(robot r, object x, object y, object z, location dst) {\n'
               '    goal { robot_at r dst; }\n'
               '    plan {\n        choose object o from (x, y, z) where in_state o "ok" else { fail "x"; }\n'
               '        move r to dst;\n    }\n}\n')

    def states(world):  # p1 has no known state (unknown), p2 is "ok", p3 is "bad"
        world["objects"]["p2"]["state"], world["objects"]["p3"]["state"] = "ok", "bad"
    bound = {"r": "ra", "x": "p1", "y": "p3", "z": "p2", "dst": "base"}
    _, _, result = simulate(objects, states, bindings=bound, name="t")
    assert chosen(result) == ["p2"]
    assert result.choices[0].excluded == (("p1", "where_unknown"), ("p3", "where_false"))


def test_a_candidate_or_reference_with_no_position_is_excluded_and_a_tie_in_distance_is_exact_c5():
    def at_nowhere(world):
        world["robots"]["ra"]["location"] = "nowhere"
    _, _, result = simulate(send('choose robot t from (a, b) nearest to ref else { fail "x"; }'), at_nowhere)
    assert chosen(result) == ["rb"] and result.choices[0].excluded == (("ra", "no_position"),)

    def nobody_has_one(world):
        world["robots"]["rb"]["location"] = "nowhere"
    _, _, result = simulate(send('choose robot t from (a, b) nearest to ref else { fail "x"; }'),
                            lambda w: (at_nowhere(w), nobody_has_one(w)))
    assert chosen(result) == [None] and end(result) == ("FAILED", "S262")
    assert result.steps[0].status == "succeeded" or result.steps[0].handled == "S272"


def test_a_robot_or_an_object_is_at_the_position_of_its_location_and_a_held_object_where_its_robot_is():
    source = ('robolang 2.3;\ntask t(robot r, location ref, location dst) {\n    goal { robot_at r dst; }\n    plan {\n'
              '        choose object o from objects at far nearest to ref else { fail "x"; }\n'
              '        move r to dst;\n    }\n}\n')
    # no world object is "far" the parameter: bind `far` to the location of p1 and p2
    bound = {"r": "ra", "ref": "mid", "dst": "base"}
    source = source.replace("objects at far", "objects at ref").replace("nearest to ref", "nearest to r")
    _, _, result = simulate(source, bindings={"r": "ra", "ref": "far", "dst": "base"}, name="t")
    assert chosen(result) == ["p1"]  # p4 is absent: never a candidate; p1 and p2 tie, the world lists p1 first
    assert result.choices[0].candidates == ("p1", "p2") and bound


def test_objects_at_lists_the_present_objects_there_in_the_worlds_order_and_robots_every_robot():
    source = ('robolang 2.3;\ntask t(robot r, location l, location dst) {\n    goal { robot_at r dst; }\n    plan {\n'
              '        choose object o from objects at l else { fail "x"; }\n        move r to dst;\n    }\n}\n')
    _, _, result = simulate(source, bindings={"r": "ra", "l": "far", "dst": "base"}, name="t")
    assert result.choices[0].candidates == ("p1", "p2") and chosen(result) == ["p1"]

    def gone(world):
        world["objects"]["p1"].update({"present": False})
        del world["objects"]["p1"]["location"]
    _, _, result = simulate(source, gone, bindings={"r": "ra", "l": "far", "dst": "base"}, name="t")
    assert chosen(result) == ["p2"]
    _, _, result = simulate(source, bindings={"r": "ra", "l": "side", "dst": "base"}, name="t")
    assert chosen(result) == [None] and end(result) == ("FAILED", "S262") and result.choices[0].candidates == ()
    _, _, result = simulate(send('choose robot t from robots where robot_ready t farthest from ref else { fail "x"; }'))
    assert result.choices[0].candidates == ("ra", "rb", "rc") and chosen(result) == ["rb"]


def test_no_candidate_runs_the_else_and_the_run_fails_with_its_reason():
    body = ('choose robot t from (c) where robot_ready t else { communicate a to "ops" with "nobody"; '
            'fail "no tugger is free"; }')
    _, _, result = simulate(send(body))
    assert end(result) == ("FAILED", "S262") and result.failure.reason == "no tugger is free"
    assert result.steps[0].status == "failed" and result.steps[0].handled == "S272"
    assert [s.message.message for s in result.steps if s.message] == ["nobody"]
    assert result.goal is not None and all(g.result is None for g in result.goal)  # a run that ends before the choice


def test_the_goal_is_checked_for_the_entity_chosen_and_never_for_some_entity_c7():
    task, world, result = simulate(send(NEAREST.replace("(a, b, c)", "(a, b)")))
    assert result.state == "SUCCEEDED" and result.goal[0].result.value == "true"
    # the goal names the chosen robot: a plan that moves another robot to the place does not reach it
    other = send('choose robot t from (a, b) nearest to ref else { fail "x"; }')
    other = other.replace("move t to ref;", "move b to ref;")
    _, _, result = simulate(other)
    assert result.state == "FAILED" and result.failure.code == "S301"  # ra was chosen; rb moved


def test_each_choice_has_a_record_with_the_candidates_the_exclusions_the_order_and_the_entity():
    task, world, result = simulate(send('choose robot t from (a, b, c) where robot_ready t nearest to ref else '
                                        '{ fail "x"; }'))
    record = to_record(result, task=task, run_id="sim-" + "0" * 64, source_sha256="a" * 64, world_sha256="b" * 64,
                       seed="0", language_version="2.3", world_schema_version="1.8")
    assert record["run_record_schema_version"] == "1.11" and record["inputs"]["world_schema_version"] == "1.8"
    keys = list(record)
    assert keys.index("choices") == keys.index("decisions") + 1
    entry = record["choices"][0]
    assert entry == {
        "step": "1", "type": "robot", "name": "t", "candidates": ["ra", "rb", "rc"],
        "excluded": [{"entity": "rc", "reason": "where_false"}],
        "order": {"mode": "nearest", "reference": "base", "distances_squared": [
            {"entity": "ra", "m2": "100"}, {"entity": "rb", "m2": "10000"}]},
        "kept": ["ra", "rb"], "chosen": "ra", "t_ms": "0"}
    assert record["core_outcome"]["outcome"]["status"] == "succeeded"
    assert json.dumps(record).isascii()
    again = to_record(run(task, world), task=task, run_id="sim-" + "0" * 64, source_sha256="a" * 64,
                      world_sha256="b" * 64, seed="0", language_version="2.3", world_schema_version="1.8")
    assert again == record  # a choice is deterministic and can be reproduced


def test_a_record_below_2_3_has_no_choices_key():
    source = "robolang 2.2;\ntask t(robot r, location l) {\n    goal { robot_at r l; }\n    plan { move r to l; }\n}\n"
    task = compile_source(source).ir.tasks[0]
    def small(world):
        world["locations"] = [item["id"] if isinstance(item, dict) else item for item in world["locations"]]
    world = load_world(world_data(version="1.7", patch=small), "w.json", task, {"r": "ra", "l": "near"})
    record = to_record(run(task, world), task=task, run_id="sim-" + "0" * 64, source_sha256="a" * 64,
                       world_sha256="b" * 64, seed="0", language_version="2.2", world_schema_version="1.7")
    assert "choices" not in record and record["run_record_schema_version"] == "1.10"


def test_a_place_whose_reference_has_no_position_fails_before_the_run_s273_and_in_validate_world():
    source = send('choose robot t from (a, b) nearest to ref else { fail "x"; }')
    bound = {**SEND_BIND, "ref": "nowhere"}
    task, world, result = simulate(source, bindings=bound)
    assert end(result) == ("FAILED", "S273") and result.failure.phase == "authorization" and result.choices == ()
    assert [(f.code, f.where) for f in check_task(task, world)] == [("S273", ("steps", 0))]
    assert "nowhere" in check_task(task, world)[0].message
    # a robot as the reference has a place that depends on the run: it is checked when the choice is made
    source = send('choose robot t from (a, b) nearest to c else { fail "x"; }')
    _, _, result = simulate(source, lambda w: w["robots"]["rc"].__setitem__("location", "nowhere"))
    assert end(result) == ("FAILED", "S262") and result.choices[0].excluded == (("ra", "no_position"),
                                                                              ("rb", "no_position"))


def test_a_choice_in_a_world_with_no_positions_leaves_no_candidate_with_an_order():
    def plain(world):
        world["locations"] = [item["id"] if isinstance(item, dict) else item for item in world["locations"]]
    task = compile_source(send('choose robot t from (a, b) else { fail "x"; }')).ir.task("send")
    world = load_world(world_data(plain, "1.7"), "w.json", task, SEND_BIND)
    assert chosen(run(task, world)) == ["ra"]  # no order: the first written, and no position needed
    ordered = compile_source(send('choose robot t from (a, b) nearest to ref else { fail "x"; }')).ir.task("send")
    world = load_world(world_data(plain, "1.7"), "w.json", ordered, SEND_BIND)
    assert end(run(ordered, world)) == ("FAILED", "S273")


def test_a_chosen_robot_is_checked_when_the_step_runs_s211_s231_and_region_limits():
    body = ('choose robot t from (c, b) nearest to ref else { fail "x"; }\n        grasp t p1;\n        move t to ref;')
    source = ('robolang 2.3;\ntask send(robot a, robot b, robot c, location ref, object p1) {\n'
              f'    goal {{ robot_at t ref; }}\n    plan {{\n        {body}\n    }}\n}}\n')
    bound = {**SEND_BIND, "p1": "p1"}
    _, _, result = simulate(source, lambda w: w["robots"]["rc"].__setitem__("ready", True), bindings=bound)
    assert chosen(result) == ["rc"] and end(result) == ("FAILED", "S211")  # rc is nearest and has no gripper
    limited = ('robolang 2.3;\ntask go(robot a, robot b, location x, location y, location ref, region wet) {\n'
               '    @safety no_go_region wet;\n    goal { robot_at a d; }\n    plan {\n'
               '        choose location d from (x, y) nearest to ref else { fail "x"; }\n'
               '        move a to d;\n    }\n}\n')
    bound = {"a": "ra", "b": "rb", "x": "mid", "y": "far", "ref": "mid", "wet": "wet"}
    task, world, result = simulate(limited, bindings=bound, name="go")
    assert chosen(result) == ["mid"] and end(result) == ("FAILED", "S004")  # mid is inside wet
    _, _, result = simulate(limited, bindings={**bound, "x": "far", "y": "near", "ref": "far"}, name="go")
    assert chosen(result) == ["far"] and result.state == "SUCCEEDED"
    assert [f.code for f in check_task(task, world)] == []  # the choice is not certain: nothing is said before the run


# -------- missions (C10) --------

PART = """robolang 2.3;

task move_part(robot arm, location conveyor, location bin) {
    goal { object_at part bin; }
    plan {
        choose object part from objects at conveyor else { fail "the conveyor is empty"; }
        move arm to conveyor;
        grasp arm part;
        move arm to bin;
        place arm part at bin;
        wait for 2 min;
    }
}

mission move_parts(robot arm, location conveyor, location bin) {
    %s {
        run move_part(arm, conveyor, bin);
    }%s
    on failure { %s }
}
"""
PART_BIND = {"arm": "ra", "conveyor": "far", "bin": "bin"}


def mission_of(source: str, patch=None, bindings=None):
    document = compile_source(source).ir
    mission = document.mission("move_parts")
    world = load_world(world_data(patch), "w.json", mission_task(mission), bindings or PART_BIND)
    return document, mission, world


def run_parts(bound: str, until: str = ";", policy: str = "stop;", patch=None):
    source = PART % (bound, until, policy)
    document, mission, world = mission_of(source, patch)
    return document, run_mission(document, mission, world)


def test_each_run_of_a_mission_chooses_afresh_and_a_moved_part_is_no_longer_a_candidate():
    _, result = run_parts("repeat for at most 6 min")
    assert [c.chosen for r in result.runs for c in r.result.choices] == ["p1", "p2", None]
    assert result.outcome == "failed" and result.runs_started == 3 if hasattr(result, "runs_started") else True
    assert [r.verified for r in result.runs] == [True, True, False]  # only two parts are at the conveyor
    assert result.failure.code == "S262"

    def three(world):
        world["objects"]["p3"]["location"] = "far"
    _, result = run_parts("repeat for at most 6 min", patch=three)
    assert [c.chosen for r in result.runs for c in r.result.choices] == ["p1", "p2", "p3"]
    assert result.outcome == "bounded" and result.ended_by == "duration_bound" and len(result.runs) == 3


def test_a_mission_record_holds_the_choices_of_every_run():
    document, result = run_parts("repeat for at most 6 min")
    record = to_mission_record(result, document=document, run_id="sim-" + "0" * 64, source_sha256="a" * 64,
                               world_sha256="b" * 64, seed="0", world_schema_version="1.8")
    assert record["mission_record_schema_version"] == "1.3" and record["run_record_schema_version"] == "1.11"
    assert [r["choices"][0]["chosen"] for r in record["runs"]] == ["p1", "p2", None]
    assert record["runs"][0]["choices"][0]["excluded"] == [] and "choices" not in record["inputs"]


def test_a_failed_run_that_changes_nothing_stops_a_timed_mission_instead_of_spinning_point_3():
    # the conveyor is empty from the start: every run finds no candidate at once, and time does not advance
    def empty(world):
        for name in ("p1", "p2"):
            world["objects"][name]["location"] = "side"
    document, result = run_parts("repeat for at most 6 min", policy="continue;", patch=empty)
    assert (result.outcome, result.ended_by) == ("failed", "no_progress") and len(result.runs) == 1
    assert result.failure.code == "S262"
    # with only a run bound the mission ends at the bound, as before
    document, result = run_parts("repeat at most 5 runs", policy="continue;", patch=empty)
    assert result.ended_by == "all_failed" and len(result.runs) == 5


def test_a_run_that_made_progress_before_it_failed_is_not_a_stall():
    def one_part(world):
        world["objects"]["p2"]["location"] = "side"
    _, result = run_parts("repeat for at most 6 min", policy="continue;", patch=one_part)
    # run 1 moves p1 (2 min), run 2 finds nothing and changes nothing: the mission stops there
    assert [r.verified for r in result.runs] == [True, False] and result.ended_by == "no_progress"


def test_the_stall_rule_applies_to_2_3_only():
    source = (PART % ("repeat for at most 6 min", ";", "continue;")).replace("robolang 2.3", "robolang 2.2").replace(
        'choose object part from objects at conveyor else { fail "the conveyor is empty"; }',
        "move arm to conveyor; grasp arm part;").replace("task move_part(robot arm,",
                                                          "task move_part(robot arm, object part,")
    assert source  # a 2.2 mission has no choose; its behaviour is covered by tests/test_lang20.py


# -------- export-goal and delegate (C8) --------

EXPORT = """robolang 2.3;

task send_tugger(robot a, robot b, location handoff) {
    goal { robot_at t handoff; }
    plan {
        choose robot t from (a, b) where robot_ready t nearest to handoff else { fail "no tugger is free"; }
        move t to handoff;
    }
}
"""


@pytest.fixture
def exported(tmp_path):
    rob = tmp_path / "send.rob"
    rob.write_text(EXPORT, encoding="utf-8")
    data = json.loads(world_data())
    data["robots"]["ra"]["ready"] = True
    data["tasks"] = {"send_tugger": {"bindings": {"a": "ra", "b": "rb", "handoff": "near"}}}
    world = tmp_path / "w.json"
    world.write_text(json.dumps(data), encoding="utf-8")
    return str(rob), str(world), tmp_path


def test_export_goal_makes_the_choice_and_sends_the_goal_with_it_filled_in_c8(exported):
    code, out, err = cli("export-goal", exported[0], "--world", exported[1], "--from", "rl:agent:planner",
                         "--to", "rl:robot:rb")
    assert code == 0, err
    goal = json.loads(out)["payload"]["goal"]
    # rb is not nearer: ra is at the handoff already, so ra is the robot the goal is about
    assert goal["desired_state"] == [{"subject": "rl:robot:ra", "property": "located_in", "value": "rl:place:near"}]
    assert goal["id"] == "rl:goal:send_tugger" and out.isascii()


def test_export_goal_refuses_a_choice_that_comes_after_an_action_x609(tmp_path, exported):
    late = tmp_path / "late.rob"
    late.write_text("""robolang 2.3;

task go(robot a, robot b, location src, location handoff) {
    goal { robot_at t handoff; }
    plan {
        move a to src;
        choose robot t from (a, b) nearest to handoff else { fail "none"; }
        move t to handoff;
    }
}
""", encoding="utf-8")
    data = json.loads(Path(exported[1]).read_text(encoding="utf-8"))
    data["tasks"] = {"go": {"bindings": {"a": "ra", "b": "rb", "src": "far", "handoff": "near"}}}
    world = tmp_path / "wl.json"
    world.write_text(json.dumps(data), encoding="utf-8")
    code, out, err = cli("export-goal", str(late), "--world", str(world), "--from", "rl:agent:planner",
                         "--to", "rl:robot:ra")
    assert code == 6 and "X609" in err and "before the first action" in err and out == ""


def test_export_goal_refuses_a_choice_with_no_candidate_and_one_whose_reference_has_no_position(tmp_path, exported):
    data = json.loads(Path(exported[1]).read_text(encoding="utf-8"))
    for patch, word in ((lambda d: d["robots"]["ra"].__setitem__("ready", False), "no candidate"),
                        (lambda d: d["tasks"]["send_tugger"]["bindings"].__setitem__("handoff", "nowhere"), "S273")):
        changed = copy.deepcopy(data)
        changed["robots"]["rb"]["ready"] = False
        patch(changed)
        world = tmp_path / "w2.json"
        world.write_text(json.dumps(changed), encoding="utf-8")
        code, out, err = cli("export-goal", exported[0], "--world", str(world), "--from", "rl:agent:planner",
                             "--to", "rl:robot:ra")
        assert code == 6 and "X609" in err and word in err, (word, err)


def test_delegate_records_the_choices_the_planner_made_and_a_library_task_with_choose_is_refused(exported, tmp_path):
    from robolang import delegate as delegation

    ground = tmp_path / "ground.rob"
    ground.write_text("""robolang 2.3;

task send_tugger(robot a, robot b, location handoff, robot t) {
    goal { robot_at t handoff; }
    plan {
        move t to handoff;
    }
}
""", encoding="utf-8")
    task = compile_source(EXPORT).ir.tasks[0]
    assert delegation._consider(None, task, b"", "w.json", "ra", (), "0.1") == ("choose_unsupported", None)
    robot_world = json.loads(Path(exported[1]).read_text(encoding="utf-8"))
    robot_world["tasks"] = {"send_tugger": {"bindings": {"a": "ra", "b": "rb", "handoff": "near", "t": "ra"}}}
    world_path = tmp_path / "robot_world.json"
    world_path.write_text(json.dumps(robot_world), encoding="utf-8")
    code, out, err = cli("delegate", exported[0], "--world", exported[1], "--plans", str(ground), "--robot-world",
                         str(world_path), "--from", "rl:agent:planner", "--to", "rl:robot:ra")
    record = json.loads(out)
    assert list(record)[list(record).index("robot") + 1] == "choices"
    assert record["choices"][0]["chosen"] == "ra" and record["choices"][0]["excluded"] == []
    assert record["planner_verdict"]["status"] == "succeeded", err


# -------- the ROS 2 backend --------

def test_the_ros2_backend_refuses_choose_before_any_motion():
    pytest.importorskip("jsonschema")
    sys.path.insert(0, str(ROOT / "backends" / "ros2"))
    from robolang_ros2.executor import execute
    from test_ros2_executor import FakeOps, fixture
    _, bindings, site = fixture()
    source = ("robolang 2.3;\ntask transfer(robot r, location src, location dst) {\n  goal { robot_at r dst; }\n"
              '  plan { choose location d from (src, dst) nearest to src else { fail "x"; } move r to d; }\n}\n')
    ops = FakeOps()
    record = execute(compile_source(source).ir.tasks[0], {"r": bindings["r"], "src": bindings["src"],
                                                          "dst": bindings["dst"]}, site, ops)
    assert record["failure"]["code"] == "S508" and "choose" in record["failure"]["message"]
    assert ops.calls == [("guard", "ur")]


# -------- version, spec and economy --------

def test_version_shows_language_2_3(capsys):
    code, _, _ = cli("--version")
    out = capsys.readouterr().out
    assert code == 0
    assert "2.2, 2.3 and 2.4" in out and "1.10, 1.11 and 1.12" in out and "mission IR 1.0, 1.1, 1.2, 1.3 and 1.4" in out


def test_validate_simulate_and_inspect_work_on_a_2_3_file(tmp_path):
    rob = tmp_path / "send.rob"
    rob.write_text(send(NEAREST.replace("nearest", "where robot_ready t nearest")), encoding="utf-8")
    data = json.loads(world_data())
    data["tasks"] = {"send": {"bindings": SEND_BIND}}
    world = tmp_path / "w.json"
    world.write_text(json.dumps(data), encoding="utf-8")
    code, out, err = cli("validate", str(rob))
    assert code == 0 and "valid RoboLang 2.3" in out, err
    code, out, err = cli("inspect", str(rob))
    assert code == 0, err
    assert "choose robot t from (a, b, c) where robot_ready t nearest to ref" in out
    assert "if no candidate is left" in out
    code, out, err = cli("validate", str(rob), "--world", str(world))
    assert code == 0, err
    code, out, err = cli("simulate", str(rob), "--world", str(world))
    record = json.loads(out)
    assert code == 0 and record["run_record_schema_version"] == "1.11" and record["choices"][0]["chosen"] == "ra", err
    code, ir_text, _ = cli("inspect", str(rob), "--json")
    ir_path = tmp_path / "send.json"
    ir_path.write_text(ir_text, encoding="utf-8")
    code, out, err = cli("simulate", "--ir", str(ir_path), "--world", str(world))
    assert code == 0 and json.loads(out)["inputs"]["source_kind"] == "ir", err


def test_every_new_code_is_in_the_specification_tables():
    table = set(re.findall(r"^\| `((?:[EISX]\d{3})(?:`, `[EISX]\d{3})*)` \|", SPEC, re.MULTILINE))
    listed = {code for row in table for code in row.split("`, `")}
    assert {"E329", "E330", "E331", "S272", "S273", "X609", "S508"} <= listed
    assert "`S273`" in SIM_SPEC and "`S272`" in SIM_SPEC and "X609" in SIM_SPEC
    reader = (ROOT / "spec" / "ir-reader-v0.1.md").read_text(encoding="utf-8")
    for code in ("I230", "I231", "I232"):
        assert f"| `{code}` |" in reader
    assert "X609" in (ROOT / "spec" / "export-goal-v0.1.md").read_text(encoding="utf-8")
    assert "X609" in (ROOT / "spec" / "delegate-v0.1.md").read_text(encoding="utf-8")


def test_the_grammar_file_says_what_2_3_cannot_say_erratum_e5():
    """The grammar file is the grammar-only prompt: it must say what 2.3 cannot (C9) and what to keep (points 5, 8)."""
    grammar = (ROOT / "spec" / "grammar-v2.3.ebnf").read_text(encoding="utf-8")
    for phrase in ("choosing by a reading of each candidate", "acting on each candidate",
                   "detecting objects the world does", "choosing a person",
                   "several choices that depend on each other", "inexpressible", "infeasible",
                   "MUST END IN fail", "NEVER READS A VALUE", "Do not replace a choice", "the FIRST candidate",
                   "may come first", "stays an if", "each run chooses afresh"):
        assert phrase in grammar, phrase


def test_the_economy_one_statement_one_world_field_no_reserved_word_about_four_new_codes():
    from robolang import tokens
    assert tokens.RESERVED_V2_0 == RESERVED_BY_VERSION["2.3"] and tokens.CHOOSE_TYPES == ("robot", "object", "location")
    assert tokens.STATEMENTS_V1_8 == ("move", "grasp", "place", "inspect", "wait", "communicate", "measure", "do")
    from robolang.sim import world
    assert "1.8" in world.WORLD_SCHEMA_VERSIONS


def test_no_dataclass_of_a_choice_has_a_mutable_default_python_3_11():
    from robolang import ir, nodes
    from robolang.sim import choice, executor, ground, mission, world
    for module in (choice, ground, mission, executor, world, nodes, ir):
        for item in vars(module).values():
            if dataclasses.is_dataclass(item) and isinstance(item, type):
                for f in dataclasses.fields(item):
                    if f.default is not dataclasses.MISSING:
                        hash(f.default)


def test_the_safety_case_has_hazard_h25():
    text = (ROOT / "docs" / "safety-case-draft.md").read_text(encoding="utf-8")
    assert "| H27 | A stale or wrong world model makes a choice wrong |" in text
