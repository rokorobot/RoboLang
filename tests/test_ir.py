"""
Tests for the versioned task IR (src/robolang/ir.py; spec/ir-schema-v1.0.md).

Golden snapshots live in tests/snapshots/ir/, one per valid fixture. To
regenerate them after an intended schema change:

    ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_ir.py
"""

from __future__ import annotations
import dataclasses
import json
import os
from decimal import Decimal
from pathlib import Path

import pytest

from robolang import RoboLangError, parse
from robolang import ir

TESTS = Path(__file__).parent
VALID = sorted((TESTS / "fixtures" / "valid").glob("*.rob"))
SNAPSHOTS = TESTS / "snapshots" / "ir"
UPDATE = os.environ.get("ROBOLANG_UPDATE_SNAPSHOTS") == "1"

HEAD = "task t(robot r, object box, location src, location dst, region cell) {\n"


def task(plan: str = "move r to src;", pre: str | None = None, safety: str = "") -> str:
    pre_block = f"    pre {{ {pre} }}\n" if pre is not None else ""
    return f"{HEAD}    {safety}\n{pre_block}    plan {{ {plan} }}\n}}\n"


def compiled(source: str) -> ir.TaskIR:
    return ir.compile_source(source).ir


def document(source: str) -> dict:
    return json.loads(ir.to_json(compiled(source)))


# -------- golden snapshots --------

@pytest.mark.parametrize("path", [pytest.param(p, id=p.stem) for p in VALID])
def test_ir_matches_golden_snapshot(path):
    actual = ir.to_json(ir.compile_source(path.read_text(encoding="utf-8")).ir)
    snapshot = SNAPSHOTS / f"{path.stem}.json"

    if UPDATE:
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        snapshot.write_bytes(actual.encode("ascii"))
    assert snapshot.exists(), f"missing snapshot {snapshot.name}"
    assert actual == snapshot.read_text(encoding="ascii")


def test_every_snapshot_belongs_to_a_fixture():
    assert {p.stem for p in SNAPSHOTS.glob("*.json")} == {p.stem for p in VALID}


# -------- schema --------

def test_document_header_has_separate_language_and_schema_versions():
    doc = document(task())

    assert list(doc) == ["ir_schema", "ir_schema_version", "language_version", "tasks"]
    assert doc["ir_schema"] == "robolang.task-ir"
    assert doc["ir_schema_version"] == ir.IR_SCHEMA_VERSION == "1.0"
    assert doc["language_version"] == ir.LANGUAGE_VERSION == "1.1"


def test_task_structure_and_key_order():
    (t,) = document(task(pre="robot_ready r;", safety="@safety max_speed 0.5 m/s;"))["tasks"]

    assert list(t) == ["name", "parameters", "safety", "preconditions", "steps"]
    assert t["name"] == "t"
    assert t["parameters"] == [
        {"name": "r", "type": "robot"},
        {"name": "box", "type": "object"},
        {"name": "src", "type": "location"},
        {"name": "dst", "type": "location"},
        {"name": "cell", "type": "region"},
    ]
    assert t["safety"] == [{"kind": "max_speed", "limit": {"value": "0.5", "unit": "m/s"}}]
    assert t["preconditions"] == [{"kind": "robot_ready", "robot": "r"}]
    assert t["steps"] == [{"kind": "move", "robot": "r", "target": "src"}]


def test_every_step_kind():
    steps = document(task(
        'move r to src; grasp r box; place r box at dst; inspect r box using "cam"; wait for 2.5s; '
        'wait until region_clear cell timeout 30s; communicate r to "fleet" with "DONE";'
    ))["tasks"][0]["steps"]

    assert steps == [
        {"kind": "move", "robot": "r", "target": "src"},
        {"kind": "grasp", "robot": "r", "object": "box"},
        {"kind": "place", "robot": "r", "object": "box", "target": "dst"},
        {"kind": "inspect", "robot": "r", "object": "box", "sensor": "cam"},
        {"kind": "wait_for", "duration": {"value": "2.5", "unit": "s"}},
        {
            "kind": "wait_until",
            "condition": {"kind": "region_clear", "region": "cell"},
            "timeout": {"value": "30", "unit": "s"},
        },
        {"kind": "communicate", "robot": "r", "channel": "fleet", "message": "DONE"},
    ]


def test_every_condition_kind():
    pre = document(task(pre='robot_ready r; region_clear cell; object_exists box; sensor_ok "cam";'))
    assert pre["tasks"][0]["preconditions"] == [
        {"kind": "robot_ready", "robot": "r"},
        {"kind": "region_clear", "region": "cell"},
        {"kind": "object_exists", "object": "box"},
        {"kind": "sensor_ok", "sensor": "cam"},
    ]


def test_absent_and_empty_pre_are_the_same_in_the_ir():
    assert compiled(task()) == compiled(task(pre=""))
    assert compiled(task()).tasks[0].preconditions == ()


def test_unitless_max_speed_keeps_a_null_unit():
    safety = document(task(safety="@safety max_speed 0.5;"))["tasks"][0]["safety"]
    assert safety == [{"kind": "max_speed", "limit": {"value": "0.5", "unit": None}}]


def test_both_speed_limits_keep_source_order():
    safety = compiled(task(safety="@safety max_speed 1.0 rad/s; @safety max_speed 0.5 m/s;")).tasks[0].safety
    assert [s.limit.unit for s in safety] == ["rad/s", "m/s"]


def test_units_are_kept_as_written_and_never_converted():
    steps = document(task("wait for 500ms; wait for 0.5s; wait for 2min;"))["tasks"][0]["steps"]
    assert [s["duration"] for s in steps] == [
        {"value": "500", "unit": "ms"},
        {"value": "0.5", "unit": "s"},
        {"value": "2", "unit": "min"},
    ]


def test_strings_hold_decoded_text():
    step = compiled(task(r'communicate r to "fleet" with "said \"done\"\n\tC:\\x";')).tasks[0].steps[0]
    assert step.message == 'said "done"\n\tC:\\x'


def test_multiple_tasks_keep_source_order_and_can_be_selected():
    tasks = compiled(task() + task().replace("task t(", "task u("))

    assert [t.name for t in tasks.tasks] == ["t", "u"]
    assert [t.name for t in tasks.select("u").tasks] == ["u"]
    assert tasks.select("u").ir_schema_version == tasks.ir_schema_version
    assert tasks.task("missing") is None
    with pytest.raises(KeyError):
        tasks.select("missing")


# -------- numeric precision --------

@pytest.mark.parametrize("written, text", [
    ("2.5", "2.5"),
    ("0.1", "0.1"),
    ("0.0000001", "0.0000001"),
    ("12345678901234567890.123456789012345678", "12345678901234567890.123456789012345678"),
    ("0.50", "0.5"),
    ("007", "7"),
    ("10", "10"),
    ("100.0", "100"),
])
def test_numbers_are_exact_decimal_strings(written, text):
    step = document(task(f"wait for {written}s;"))["tasks"][0]["steps"][0]
    assert step["duration"]["value"] == text
    assert Decimal(step["duration"]["value"]) == Decimal(written)


def test_ir_holds_decimals_and_json_holds_no_floats():
    source = task("wait for 0.1s;", safety="@safety max_speed 0.3 m/s;")

    assert compiled(source).tasks[0].steps[0].duration.value == Decimal("0.1")
    assert isinstance(compiled(source).tasks[0].safety[0].limit.value, Decimal)

    def walk(node):
        assert not isinstance(node, (float, int)) or isinstance(node, bool), node
        if isinstance(node, dict):
            for value in node.values():
                walk(value)
        elif isinstance(node, list):
            for value in node:
                walk(value)

    walk(document(source))


# -------- determinism --------

def test_same_source_gives_identical_json():
    source = (TESTS / "fixtures" / "valid" / "multiple_tasks.rob").read_text(encoding="utf-8")
    assert ir.to_json(compiled(source)) == ir.to_json(compiled(source))


def test_layout_comments_and_number_spelling_do_not_change_the_ir():
    plain = 'task t(robot r, location dst) { @safety max_speed 0.5 m/s; plan { move r to dst; wait for 2s; } }'
    spread = (
        "// header\n"
        "task t(\n    robot r,   // the arm\n    location dst\n)\n{\n"
        "    @safety max_speed 0.50m/s;\n\n"
        "    plan {\n        move r\n            to dst ;\n        wait for 2.0 s;\n    }\n}\n"
    )
    assert ir.to_json(compiled(plain)) == ir.to_json(compiled(spread))


def test_version_marker_does_not_change_the_ir():
    assert compiled(task()) == compiled("robolang 1.1;\n" + task())


def test_json_is_ascii_with_lf_and_trailing_newline():
    text = ir.to_json(compiled(task('communicate r to "flotte" with "prêt → ok";')))

    text.encode("ascii")
    assert "\r" not in text and text.endswith("}\n")
    assert json.loads(text)["tasks"][0]["steps"][0]["message"] == "prêt → ok"


# -------- only validated programs become IR --------

def test_ir_types_are_immutable():
    tasks = compiled(task())
    with pytest.raises(dataclasses.FrozenInstanceError):
        tasks.tasks[0].name = "other"
    assert isinstance(tasks.tasks, tuple) and isinstance(tasks.tasks[0].steps, tuple)


@pytest.mark.parametrize("plan, code", [
    ("move r to nowhere;", "E303"),
    ("move box to src;", "E304"),
    ("wait for 0s;", "E305"),
    ('inspect r box using "";', "E307"),
])
def test_compile_program_refuses_an_ast_that_fails_static_validation(plan, code):
    program = parse(task(plan)).program  # parses, but is not valid

    with pytest.raises(RoboLangError) as raised:
        ir.compile_program(program)
    assert raised.value.code == code


@pytest.mark.parametrize("source, code", [("task t( {", "E201"), (task("fly r to moon;"), "E203"), ("#", "E101")])
def test_compile_source_refuses_invalid_source(source, code):
    with pytest.raises(RoboLangError) as raised:
        ir.compile_source(source)
    assert raised.value.code == code


def test_compile_source_returns_warnings_and_program():
    result = ir.compile_source(task(safety="@safety max_speed 0.5;"))

    assert [w.code for w in result.warnings] == ["W401"]
    assert result.program.tasks[0].name.text == result.ir.tasks[0].name == "t"
    assert ir.compile_program(result.program) == result.ir
