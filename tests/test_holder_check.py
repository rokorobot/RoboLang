"""E315 and I216: a grasp of something still held (spec/decisions/0032-holder-check.md)."""

from __future__ import annotations
import json
import textwrap

import pytest

from robolang import RoboLangError, check
from robolang.ir import compile_source, to_json
from robolang.ir_reader import IRError, read_ir

PARAMS = "robot arm, robot mobile, object box, object b2, location src, location dst, region cell"


def source(plan: str, version: str = "1.4") -> str:
    goal = "    goal { robot_at arm src; }\n" if version in ("1.3", "1.4", "1.5") else ""  # no goal block before 1.3
    return (f"robolang {version};\ntask t({PARAMS}) {{\n{goal}    plan {{\n"
            + "".join(f"        {line}\n" for line in textwrap.dedent(plan).strip().splitlines()) + "    }\n}\n")


def diagnostics(plan: str, version: str = "1.4") -> list[tuple[str, int, str]]:
    try:
        check(source(plan, version))
    except RoboLangError as error:
        return [(d.code, d.line, d.message) for d in error.diagnostics]
    return []


def test_another_robot_grasps_a_held_object():
    found = diagnostics("""
        move arm to src;
        grasp arm box;
        move mobile to src;
        grasp mobile box;
    """)
    assert found == [("E315", 8, "'grasp mobile box': 'box' is still held by 'arm' (grasped at 6:9); place it first")]


def test_a_robot_grasps_a_second_object():
    found = diagnostics("grasp arm box;\ngrasp arm b2;")
    assert found == [("E315", 6, "'grasp arm b2': 'arm' already holds 'box' (grasped at 5:9); place it first")]


def test_a_place_in_between_is_valid():
    assert diagnostics("grasp arm box;\nplace arm box at dst;\ngrasp mobile box;\ngrasp arm b2;") == []


def test_a_place_by_any_robot_ends_what_is_known():
    # mobile and arm may be bound to one robot, and b2 and box to one object: the last grasp can succeed
    assert diagnostics("grasp arm box;\ngrasp mobile b2;\nplace mobile b2 at dst;\ngrasp mobile box;") == []


def test_different_robots_and_objects_are_not_flagged():
    assert diagnostics("grasp arm box;\ngrasp mobile b2;") == []


def test_one_path_is_enough_and_each_grasp_is_reported_once():
    found = diagnostics("""
        grasp arm box;
        if region_clear cell {
            grasp arm b2;
        } else {
            move mobile to src;
        }
        grasp mobile box;
        grasp mobile b2;
    """)
    assert [(code, line) for code, line, _ in found] == [("E315", 7), ("E315", 11)]


def test_the_earliest_grasp_is_named_whichever_path_finds_it():
    found = diagnostics("""
        if region_clear cell {
            grasp arm box;
        } else {
            grasp arm box;
        }
        grasp mobile box;
    """)
    assert found == [("E315", 10, "'grasp mobile box': 'box' is still held by 'arm' (grasped at 6:13); place it first")]


def test_a_path_that_ends_in_fail_holds_nothing_afterwards():
    assert diagnostics("""
        if region_clear cell {
            grasp arm box;
            fail "stop";
        }
        grasp mobile box;
    """) == []


def test_a_path_stops_at_its_first_double_grasp():
    found = diagnostics("grasp arm box;\ngrasp mobile box;\ngrasp arm b2;")
    assert [(code, line) for code, line, _ in found] == [("E315", 6)]


@pytest.mark.parametrize("version", ["1.1", "1.3"])
def test_every_language_version_is_checked(version):
    found = diagnostics("grasp arm box;\ngrasp arm b2;", version)
    assert [code for code, _, _ in found] == ["E315"]


def test_not_checked_when_a_step_names_something_wrong():
    assert [code for code, _, _ in diagnostics("grasp arm box;\ngrasp arm nothing;\ngrasp arm b2;")] == ["E303"]


def test_ir_reader_reports_i216_at_the_grasp():
    plan = "grasp arm box;\nplace arm box at dst;\ngrasp mobile box;"
    document = json.loads(to_json(compile_source(source(plan)).ir))
    del document["tasks"][0]["steps"][1]
    with pytest.raises(IRError) as caught:
        read_ir(json.dumps(document).encode(), "x.json")
    assert [(d.code, d.pointer, d.message) for d in caught.value.diagnostics] == [
        ("I216", "/tasks/0/steps/1",
         "'grasp mobile box': 'box' is still held by 'arm' (grasped at /tasks/0/steps/0); place it first")]


def test_an_if_on_holding_is_followed_only_where_it_can_be_taken():
    retry = """
        move arm to src;
        grasp arm box else {
            move arm to src;
        }
        if not holding arm box {
            grasp arm box;
        }
    """
    assert diagnostics(retry, "1.5") == []  # the then branch runs only when arm does not hold box
    assert [c for c, _, _ in diagnostics("grasp arm box;\nif holding arm box {\n    grasp arm box;\n}")] == ["E315"]
    assert diagnostics("grasp arm box;\nif object_at box dst {\n    grasp mobile box;\n}") == []  # held: at no place
    assert [c for c, _, _ in diagnostics("grasp arm box;\nif region_clear cell {\n    grasp mobile box;\n}")] \
        == ["E315"]
