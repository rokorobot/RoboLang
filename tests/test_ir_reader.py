"""Tests for the Task IR reader and re-validator (src/robolang/ir_reader.py; spec/ir-reader-v0.1.md; decision 0021)."""

from __future__ import annotations
import copy
import io
import json
import os
import re
from pathlib import Path

import pytest

from robolang import ir
from robolang.cli import main
from robolang.ir_reader import IRError, read_ir
from robolang.sim import sha256_normalized

TESTS = Path(__file__).parent
REPO_ROOT = TESTS.parent
SNAPSHOT_SETS = [("ir", "valid"), ("ir-v1.2", "v1.2/valid"), ("ir-v1.3", "v1.3/valid"), ("ir-v1.4", "v1.4/valid")]
SNAPSHOTS = [
    pytest.param(snapshot, TESTS / "fixtures" / fixtures / f"{snapshot.stem}.rob", id=f"{folder}/{snapshot.stem}")
    for folder, fixtures in SNAPSHOT_SETS
    for snapshot in sorted((TESTS / "snapshots" / folder).glob("*.json"))
]
INVALID = TESTS / "fixtures" / "ir_invalid"
EXPECTED = json.loads((INVALID / "expected.json").read_text(encoding="utf-8"))
WORLD = "examples/world.json"


def valid(name: str = "valid_1_2.json") -> dict:
    return json.loads((INVALID / name).read_text(encoding="utf-8"))


def blob(document: dict) -> bytes:
    return json.dumps(document).encode("utf-8")


def problems(data: bytes | dict) -> list[tuple[str, str]]:
    with pytest.raises(IRError) as caught:
        read_ir(blob(data) if isinstance(data, dict) else data, "x.json")
    return [(d.code, d.pointer) for d in caught.value.diagnostics]


# -------- canonical round trip --------

@pytest.mark.parametrize("snapshot,source", SNAPSHOTS)
def test_every_compiler_snapshot_reads_back_to_the_compilers_ir(snapshot, source):
    data = snapshot.read_bytes()
    compiled = ir.compile_source(source.read_text(encoding="utf-8")).ir

    read = read_ir(data, snapshot.name)

    assert read == compiled
    assert ir.to_json(read).encode("ascii") == data.replace(b"\r\n", b"\n")  # a Windows checkout may use CRLF


def test_snapshot_sets_are_complete():
    assert len(SNAPSHOTS) == 40


def test_reading_is_independent_of_layout_key_order_and_decimal_zeros():
    document = valid()
    document["tasks"][0] = dict(reversed(document["tasks"][0].items()))
    document["tasks"][0]["safety"][0]["limit"]["value"] = "0.500"
    expected = read_ir((INVALID / "valid_1_2.json").read_bytes())

    read = read_ir(json.dumps(document, indent=None, separators=(",", ":")).encode())

    assert read == expected
    assert ir.to_json(read) == ir.to_json(expected)


# -------- invalid documents: one fixture per code --------

@pytest.mark.parametrize("name", sorted(EXPECTED))
def test_invalid_fixture_reports_its_code_at_its_pointer(name):
    code, pointer = EXPECTED[name]

    assert problems((INVALID / name).read_bytes()) == [(code, pointer)]


def test_every_code_has_a_fixture():
    in_source = set(re.findall(r'"(I\d{3})"', (REPO_ROOT / "src/robolang/ir_reader.py").read_text(encoding="utf-8")))
    in_source.add("I105")  # raised from a table, tested below
    assert {code for code, _ in EXPECTED.values()} | {"I105", "I211"} >= in_source - {"I211"}
    assert "I211" not in in_source  # E311 (a repeated @verify key) cannot be written in JSON: it is I103


def test_json_nested_too_deeply_to_read_is_i105():
    assert problems(b"[" * 200000) == [("I105", "")]


def test_a_byte_order_mark_and_nan_are_refused():
    assert problems(b"\xef\xbb\xbf{}")[0][0] == "I101"
    assert problems(b'{"a": NaN}')[0][0] == "I102"
    assert problems(b'{"a": 1e3}')[0][0] == "I104"


def test_read_ir_needs_bytes():
    with pytest.raises(TypeError):
        read_ir("{}")  # type: ignore[arg-type]


def test_a_document_that_is_not_an_object():
    assert problems(b"[]") == [("I110", "")]
    assert problems(b"{}")[0][0] == "I111"


def test_diagnostic_format_names_file_pointer_and_code():
    with pytest.raises(IRError) as caught:
        read_ir(blob({**valid(), "tasks": []}), "plan.json")

    assert caught.value.diagnostic.format() == \
        "plan.json: /tasks: error[I120]: a document needs at least one task"
    assert caught.value.code == "I120"


# -------- the claimed versions are never trusted --------

def test_schema_1_0_document_cannot_use_combined_conditions_or_schema_1_2_fields():
    document = valid("valid_1_0.json")
    document["tasks"][0]["preconditions"] = [{"kind": "not", "condition": {"kind": "robot_ready", "robot": "r"}}]
    document["tasks"][0]["goal"] = []
    document["tasks"][0]["parameters"].append({"name": "cam", "type": "sensor"})
    holding = {"kind": "holding", "robot": "r", "object": "box"}
    document["tasks"][0]["steps"].append({"kind": "wait_until", "condition": holding,
                                          "timeout": {"value": "1", "unit": "s"}})

    assert problems(document) == [
        ("I112", "/tasks/0/goal"),
        ("I115", "/tasks/0/parameters/3/type"),
        ("I115", "/tasks/0/preconditions/0/kind"),
        ("I115", "/tasks/0/steps/1/condition/kind"),
    ]


def test_schema_1_1_document_cannot_use_goal_atoms_or_verify():
    document = valid("valid_1_0.json")
    document.update(ir_schema_version="1.1", language_version="1.2")
    document["tasks"][0]["verify"] = {"min_confidence": "0.5"}
    document["tasks"][0]["preconditions"] = [{"kind": "object_at", "object": "box", "location": "src"}]

    assert problems(document) == [("I112", "/tasks/0/verify"), ("I115", "/tasks/0/preconditions/0/kind")]


@pytest.mark.parametrize("schema,language", [("1.0", "1.3"), ("1.1", "1.1"), ("1.2", "1.2")])
def test_language_version_must_match_the_schema_version(schema, language):
    document = valid()
    document.update(ir_schema_version=schema, language_version=language)
    document["tasks"][0].pop("verify"), document["tasks"][0].pop("goal")
    document["tasks"][0]["parameters"].pop()

    assert ("I113", "/language_version") in problems(document)


def test_wrong_schema_identifier_and_unknown_version_stop_before_the_tasks_are_read():
    document = valid()
    document.update(ir_schema="other", ir_schema_version="1.4")
    document["tasks"] = "not even tasks"

    assert problems(document) == [("I113", "/ir_schema"), ("I113", "/ir_schema_version")]


def test_reserved_words_follow_the_language_of_the_schema():
    old = valid("valid_1_0.json")
    old["tasks"][0]["parameters"].append({"name": "and", "type": "robot"})  # a name in RoboLang 1.1
    new = valid()
    new["tasks"][0]["parameters"].append({"name": "goal", "type": "robot"})  # reserved in 1.3

    assert read_ir(blob(old)).tasks[0].parameters[-1].name == "and"
    assert problems(new) == [("I118", "/tasks/0/parameters/6/name")]


def test_schema_1_2_sensors_are_parameters_but_schema_1_1_sensors_are_world_ids():
    old = valid("valid_1_0.json")
    old["tasks"][0]["steps"].append({"kind": "inspect", "robot": "r", "object": "box", "sensor": "camera_top"})
    old["tasks"][0]["steps"].append({"kind": "inspect", "robot": "r", "object": "box", "sensor": ""})
    assert problems(old) == [("I207", "/tasks/0/steps/2/sensor")]

    new = valid()
    new["tasks"][0]["steps"][1]["sensor"] = "box"
    assert problems(new) == [("I204", "/tasks/0/steps/1/sensor")]


# -------- the static rules, restated --------

def test_sensor_parameter_used_as_anything_else_is_a_type_error():
    document = valid()
    document["tasks"][0]["steps"][0]["robot"] = "cam"
    document["tasks"][0]["preconditions"] = [{"kind": "sensor_ok", "sensor": "r"}]

    assert problems(document) == [("I204", "/tasks/0/preconditions/0/sensor"), ("I204", "/tasks/0/steps/0/robot")]


def test_max_age_and_timeout_must_be_positive_and_wait_until_conditions_are_checked():
    document = valid()
    document["tasks"][0]["verify"]["max_age"]["value"] = "0"
    document["tasks"][0]["steps"].append({"kind": "wait_until", "timeout": {"value": "0.0", "unit": "ms"},
                                          "condition": {"kind": "region_clear", "region": "nowhere"}})

    assert problems(document) == [
        ("I205", "/tasks/0/verify/max_age/value"),
        ("I203", "/tasks/0/steps/4/condition/region"),
        ("I205", "/tasks/0/steps/4/timeout/value"),
    ]


def test_unitless_limit_alone_is_accepted_and_stays_unitless():
    document = valid("valid_1_0.json")
    document["tasks"][0]["safety"] = [{"kind": "max_speed", "limit": {"value": "1", "unit": None}}]

    assert read_ir(blob(document)).tasks[0].safety[0].limit.unit is None


def test_goal_is_required_and_non_empty_in_schema_1_2_and_the_rest_is_still_checked():
    document = valid()
    document["tasks"][0]["goal"] = []
    document["tasks"][0]["steps"][0]["target"] = "nowhere"

    assert problems(document) == [("I209", "/tasks/0/goal"), ("I203", "/tasks/0/steps/0/target")]


def test_a_depth_of_8_is_accepted_and_9_is_not():
    def nested(levels: int) -> dict:
        condition = {"kind": "robot_ready", "robot": "r"}
        for _ in range(levels):
            condition = {"kind": "not", "condition": condition}
        document = valid("valid_1_0.json")
        document.update(ir_schema_version="1.1", language_version="1.2")
        document["tasks"][0]["preconditions"] = [condition]
        return document

    assert read_ir(blob(nested(8))).tasks
    assert problems(nested(9))[0][0] == "I121"


# -------- every error is reported --------

def test_all_errors_of_all_tasks_are_reported_in_document_order():
    document = valid()
    first = document["tasks"][0]
    second = copy.deepcopy(first)
    second["steps"][0]["target"] = "nowhere"            # I203 in task 2
    second["verify"]["min_confidence"] = "7"             # I212 in task 2
    first["name"] = "plan"                                # I118 in task 1: its meaning is not checked
    first["steps"][0]["target"] = "nowhere"
    first["steps"][1]["extra"] = True                     # I112 in task 1
    document["tasks"] = [first, second, copy.deepcopy(second)]

    assert problems(document) == [
        ("I118", "/tasks/0/name"),
        ("I112", "/tasks/0/steps/1/extra"),
        ("I212", "/tasks/1/verify/min_confidence"),
        ("I203", "/tasks/1/steps/0/target"),
        ("I201", "/tasks/2/name"),
        ("I212", "/tasks/2/verify/min_confidence"),
        ("I203", "/tasks/2/steps/0/target"),
    ]


def test_at_most_fifty_errors_are_listed_then_the_reader_says_there_are_more():
    document = valid()
    document["tasks"][0]["steps"] = [{"kind": "move", "robot": "r", "target": "nowhere"}] * 80

    with pytest.raises(IRError) as caught:
        read_ir(blob(document), "big.json")

    assert len(caught.value.diagnostics) == 50 and caught.value.truncated


def test_exactly_fifty_errors_are_not_truncated():
    document = valid()
    document["tasks"][0]["steps"] = [{"kind": "move", "robot": "r", "target": "nowhere"}] * 50

    with pytest.raises(IRError) as caught:
        read_ir(blob(document))

    assert len(caught.value.diagnostics) == 50 and not caught.value.truncated


def test_json_pointer_escapes_slash_and_tilde():
    document = valid()
    document["tasks"][0]["a/b~c"] = 1

    assert problems(document) == [("I112", "/tasks/0/a~1b~0c")]


# -------- CLI --------

def cli(*argv: str, cwd: Path = REPO_ROOT):
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(cwd)
    try:
        code = main(list(argv), out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


@pytest.fixture
def handover_ir(tmp_path):
    code, out, _ = cli("inspect", "examples/handover.rob", "--json")
    assert code == 0
    path = tmp_path / "handover.json"
    path.write_bytes(out.encode("ascii"))
    return path


def test_validate_ir_accepts_a_compiler_document(handover_ir):
    code, out, err = cli("validate", "--ir", str(handover_ir))

    assert code == 0 and err == ""
    assert out.startswith(f"{handover_ir}: valid Task IR 1.2 (RoboLang 1.3, 1 task)\n")
    assert "does not make a task safe" in out


def test_validate_ir_lists_every_error_and_a_summary(tmp_path):
    path = tmp_path / "bad.json"
    document = valid()
    document["tasks"][0]["steps"][0]["target"] = "nowhere"
    document["tasks"][0]["steps"][2]["duration"]["value"] = "0"
    path.write_bytes(blob(document))

    code, out, err = cli("validate", "--ir", str(path))

    assert code == 1 and out == ""
    assert err.splitlines() == [
        f"{path}: /tasks/0/steps/0/target: error[I203]: undeclared name 'nowhere': it is not a parameter of this task",
        f"{path}: /tasks/0/steps/2/duration/value: error[I205]: duration must be greater than zero",
        f"{path}: not valid (2 errors)",
    ]


def test_inspect_ir_matches_inspect_of_the_source(handover_ir):
    assert cli("inspect", "--ir", str(handover_ir), "--json")[1] == cli("inspect", "examples/handover.rob", "--json")[1]
    text_from_ir = cli("inspect", "--ir", str(handover_ir))[1].replace(str(handover_ir), "F")
    text_from_source = cli("inspect", "examples/handover.rob")[1].replace("examples/handover.rob", "F")
    assert text_from_ir == text_from_source


def test_exactly_one_of_file_and_ir(handover_ir, capsys):
    for argv in (["validate"], ["validate", "examples/handover.rob", "--ir", str(handover_ir)]):
        assert cli(*argv)[0] == 2
        assert "exactly one of FILE" in capsys.readouterr().err


def test_unreadable_ir_file_is_a_usage_error():
    assert cli("validate", "--ir", "no/such.json")[0] == 2


def normalized(record: dict, ir_bytes: bytes, source_bytes: bytes) -> dict:
    """A run record without what names its input: kind, hash, and the IDs and signatures derived from them."""
    text = json.dumps(record)
    text = text.replace(record["run_id"], "RUN").replace(sha256_normalized(ir_bytes), "SRC")
    text = text.replace(sha256_normalized(source_bytes), "SRC")
    text = text.replace(record["run_id"][4:20], "RUNPREFIX")  # observation IDs in core_outcome start with it
    result = json.loads(text)
    result["inputs"].pop("source_kind", None)
    return without_signatures(result)


def without_signatures(node):
    """Evidence signatures cover the observation ID, which is derived from the run ID."""
    if isinstance(node, dict):
        return {k: without_signatures(v) for k, v in node.items() if k != "signature"}
    if isinstance(node, list):
        return [without_signatures(v) for v in node]
    return node


@pytest.mark.parametrize("example,extra", [
    ("examples/handover.rob", []),
    ("examples/pick_and_place_v13.rob", []),
    ("examples/inspect_object.rob", []),
    ("examples/pick_and_place_v13.rob", ["--seed", "s2"]),
])
def test_simulate_from_ir_equals_simulate_from_source_except_the_input_fields(handover_ir, tmp_path, example, extra):
    ir_path = tmp_path / "task.json"
    ir_path.write_bytes(cli("inspect", example, "--json")[1].encode("ascii"))

    code_ir, out_ir, _ = cli("simulate", "--ir", str(ir_path), "--world", WORLD, *extra)
    code_src, out_src, _ = cli("simulate", example, "--world", WORLD, *extra)

    from_ir, from_source = json.loads(out_ir), json.loads(out_src)
    assert code_ir == code_src == 0
    assert from_ir["inputs"]["source_kind"] == "ir" and "source_kind" not in from_source["inputs"]
    assert from_ir["inputs"]["source_sha256"] == sha256_normalized(ir_path.read_bytes())
    assert from_ir["run_id"] != from_source["run_id"]
    assert normalized(from_ir, ir_path.read_bytes(), (REPO_ROOT / example).read_bytes()) == \
        normalized(from_source, ir_path.read_bytes(), (REPO_ROOT / example).read_bytes())


def test_the_ir_run_id_is_stable_and_depends_on_the_ir_bytes(handover_ir, tmp_path):
    first = json.loads(cli("simulate", "--ir", str(handover_ir), "--world", WORLD)[1])["run_id"]
    again = json.loads(cli("simulate", "--ir", str(handover_ir), "--world", WORLD)[1])["run_id"]
    spaced = tmp_path / "spaced.json"
    spaced.write_bytes(handover_ir.read_bytes().replace(b"\n", b"\n "))
    other = json.loads(cli("simulate", "--ir", str(spaced), "--world", WORLD)[1])["run_id"]

    assert first == again and first != other


def test_simulate_ir_with_bind_and_task(tmp_path):
    ir_path = tmp_path / "multi.json"
    ir_path.write_bytes(cli("inspect", "examples/pick_and_place_v13.rob", "--json")[1].encode("ascii"))
    name = json.loads(ir_path.read_text())["tasks"][0]["name"]

    code, out, _ = cli("simulate", "--ir", str(ir_path), "--task", name, "--world", WORLD,
                       "--bind", "r=arm1", "box=box1", "src=shelf", "dst=table", "cell=cell")

    assert code == 0
    assert json.loads(out)["inputs"]["bindings_source"] == "cli"


def test_simulate_refuses_an_invalid_ir_before_anything_runs(tmp_path):
    path = tmp_path / "bad.json"
    path.write_bytes(b'{"ir_schema": "robolang.task-ir", "ir_schema_version": "1.2"}')

    code, out, err = cli("simulate", "--ir", str(path), "--world", WORLD)

    assert code == 1 and out == "" and "error[I111]" in err


def test_export_goal_from_ir_matches_export_goal_from_source(handover_ir):
    args = ["--world", WORLD, "--from", "rl:agent:planner-1", "--to", "rl:robot:arm1"]

    from_ir = cli("export-goal", "--ir", str(handover_ir), *args)
    from_source = cli("export-goal", "examples/handover.rob", *args)

    assert from_ir[0] == from_source[0] == 0
    assert from_ir[1] == from_source[1]


def test_delegate_ir_needs_the_robots_own_plans(handover_ir):
    args = ["--world", WORLD, "--from", "rl:agent:planner-1", "--to", "rl:robot:arm1"]

    refused = cli("delegate", "--ir", str(handover_ir), *args)
    delegated = cli("delegate", "--ir", str(handover_ir), *args, "--plans", "examples/plans")
    from_source = cli("delegate", "examples/handover.rob", *args, "--plans", "examples/plans")

    assert refused[0] == 2 and "--ir needs --plans" in refused[2]
    assert delegated[0] == from_source[0] == 0
    assert delegated[1] == from_source[1]


def test_help_describes_the_ir_option(capsys):
    assert cli("simulate", "--help")[0] == 0
    assert "--ir IR" in capsys.readouterr().out


def test_the_spec_lists_exactly_the_codes_the_reader_uses():
    spec = (REPO_ROOT / "spec" / "ir-reader-v0.1.md").read_text(encoding="utf-8")
    source = (REPO_ROOT / "src" / "robolang" / "ir_reader.py").read_text(encoding="utf-8")
    table = set(re.findall(r"^\| `(I\d{3})` \|", spec, re.MULTILINE))

    assert table == set(re.findall(r'"(I\d{3})"', source)) | {"I101", "I102", "I103", "I104", "I105"}
