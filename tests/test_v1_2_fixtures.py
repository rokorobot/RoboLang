"""
Runs the LANG-1 conformance fixtures (tests/fixtures/v1.2) against the front end and the
simulator: acceptance tests L-T1 to L-T6 of spec/robolang-v1.2.md section 10.

- valid fixtures pass the complete front end with no diagnostics and match their golden IR
  snapshot (tests/snapshots/ir-v1.2/), with language 1.2 and IR schema 1.1 (L-T1);
- invalid fixtures are rejected with the stated code and line, and produce no IR (L-T2);
- compat fixtures (RoboLang 1.1) keep their 1.1 result (L-T3);
- simulation cases give the stated exit status and run-record fields (L-T5, L-T6).

To regenerate the snapshots after an intended IR change:

    ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_v1_2_fixtures.py
"""

from __future__ import annotations
import io
import json
import os
import re
from pathlib import Path

import pytest

from robolang import RoboLangError, check_file, parse_file
from robolang.cli import main
from robolang.ir import compile_source, to_dict, to_json

ROOT = Path(__file__).parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "v1.2"
SNAPSHOTS = ROOT / "tests" / "snapshots" / "ir-v1.2"
UPDATE = os.environ.get("ROBOLANG_UPDATE_SNAPSHOTS") == "1"
SPEC = (ROOT / "spec" / "robolang-v1.2.md").read_text(encoding="utf-8")
SPEC_V1_1 = (ROOT / "spec" / "robolang-v1.1.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")
CASES = json.loads((FIXTURES / "simulation" / "cases.json").read_text(encoding="utf-8"))["cases"]

HEADER = re.compile(
    r"\A// RoboLang (?P<version>v1\.[12]) fixture: (?P<set>valid|invalid|compat)\n"
    r"// expect: (?:(?P<ok>ok)|error (?P<code>E\d{3}) \((?P<stage>lexical|syntax|static)\) line (?P<line>\d+))\n"
    r"// why: .+\n"
)
STAGE_DIGIT = {"lexical": "1", "syntax": "2", "static": "3"}


def header(path: Path) -> dict:
    match = HEADER.match(path.read_text(encoding="utf-8"))
    assert match, f"{path.name}: missing or malformed header"
    return match.groupdict()


def files(folder: str) -> list:
    return [pytest.param(p, id=p.stem) for p in sorted((FIXTURES / folder).glob("*.rob"))]


def defined_codes(text: str) -> set[str]:
    return set(re.findall(r"^\| `([EWS]\d{3})` \|", text, re.MULTILINE))


def test_fixture_counts_match_the_specification():
    counts = {folder: len(list((FIXTURES / folder).glob("*.rob"))) for folder in ("valid", "invalid", "compat")}
    assert counts == {"valid": 10, "invalid": 15, "compat": 3}
    assert len(CASES) == 12
    for folder, count in counts.items():
        assert f"| `{folder}/` |" in SPEC and f"| {count} |" in SPEC


# -------- L-T1: valid fixtures --------

@pytest.mark.parametrize("path", files("valid"))
def test_valid_fixture_passes_and_matches_its_ir_snapshot(path):
    assert header(path)["version"] == "v1.2" and header(path)["ok"]
    assert not check_file(path).warnings

    actual = to_json(compile_source(path.read_text(encoding="utf-8"), str(path)).ir)
    snapshot = SNAPSHOTS / f"{path.stem}.json"
    if UPDATE:
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        snapshot.write_bytes(actual.encode("ascii"))
    assert snapshot.exists(), f"missing snapshot {snapshot.name}"
    assert actual == snapshot.read_text(encoding="ascii")
    document = json.loads(actual)
    assert (document["language_version"], document["ir_schema_version"]) == ("1.2", "1.1")


def test_every_snapshot_belongs_to_a_fixture():
    assert {p.stem for p in SNAPSHOTS.glob("*.json")} == {p.stem for p in (FIXTURES / "valid").glob("*.rob")}


# -------- L-T2: invalid fixtures --------

@pytest.mark.parametrize("path", files("invalid"))
def test_invalid_fixture_is_rejected_with_the_stated_code_and_line(path):
    expected = header(path)
    code = expected["code"]
    assert expected["version"] == "v1.2" and expected["set"] == "invalid"
    assert path.name.startswith(code.lower() + "_")
    assert code[1] == STAGE_DIGIT[expected["stage"]]
    assert code in defined_codes(SPEC) | defined_codes(SPEC_V1_1), f"{code} is not defined"

    if expected["stage"] == "static":
        parse_file(path)  # syntactically valid: the error is a static one
    with pytest.raises(RoboLangError) as raised:
        compile_source(path.read_text(encoding="utf-8"), str(path))
    assert (raised.value.code, raised.value.line) == (code, int(expected["line"])), raised.value


# -------- L-T3: compat fixtures --------

@pytest.mark.parametrize("path", files("compat"))
def test_compat_fixture_keeps_its_v1_1_result(path):
    expected = header(path)
    assert expected["version"] == "v1.1" and expected["set"] == "compat"
    if expected["ok"]:
        assert not check_file(path).warnings
        assert compile_source(path.read_text(encoding="utf-8")).ir.language_version == "1.1"
        return
    with pytest.raises(RoboLangError) as raised:
        check_file(path)
    assert (raised.value.code, raised.value.line) == (expected["code"], int(expected["line"])), raised.value


# -------- L-T4: canonical IR --------

def test_grouping_is_flattened_and_nothing_else_is_rewritten():
    task = compile_source((FIXTURES / "valid" / "flatten_grouping.rob").read_text(encoding="utf-8")).ir.tasks[0]
    first, second, third = (to_dict(c) for c in task.preconditions)
    assert first == second == third
    assert [c["kind"] for c in first["conditions"]] == ["robot_ready", "region_clear", "object_exists"]


# -------- L-T5, L-T6: simulation --------

def simulate(case) -> tuple[int, str, str]:
    argv = ["simulate", case["source"], "--world", case["world"], "--seed", case["seed"]]
    if case["task"] is not None:
        argv += ["--task", case["task"]]
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(ROOT)
    try:
        code = main(argv, out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


def test_simulation_fixture_files_are_all_used():
    sim = FIXTURES / "simulation"
    used = {case["source"] for case in CASES} | {case["world"] for case in CASES}
    for path in sorted((sim / "sources").glob("*.rob")) + sorted((sim / "worlds").glob("*.json")):
        assert path.relative_to(ROOT).as_posix() in used, f"{path.name} is not used by any case"
    run_codes = defined_codes(SIM_SPEC)
    for case in CASES:
        assert set(case["tests"]) <= {"L-T5", "L-T6"}
        if case["expect"]["failure"]:
            assert case["expect"]["failure"]["code"] in run_codes


@pytest.mark.parametrize("case", CASES, ids=lambda case: case["id"])
def test_simulation_case(case):
    expect = case["expect"]
    code, out, err = simulate(case)

    assert code == expect["exit"], err
    record = json.loads(out)
    assert record["run_record_schema_version"] == "1.1"
    assert (record["inputs"]["language_version"], record["inputs"]["ir_schema_version"]) == ("1.2", "1.1")
    assert record["mode"] == "simulation"
    for key in ("states", "state", "failure", "end_t_ms", "events_applied"):
        if key in expect:
            assert record[key] == expect[key], key
    assert [p["result"] for p in record["preconditions"]] == expect["preconditions"]
    assert [s["status"] for s in record["steps"]] == expect["steps"]
    if "step_times" in expect:
        times = [None if s["t_start_ms"] is None else [s["t_start_ms"], s["t_end_ms"]] for s in record["steps"]]
        assert times == expect["step_times"]
    assert simulate(case)[:2] == (code, out)  # deterministic: byte-identical record
