"""
Runs the RoboLang 1.3 conformance fixtures (tests/fixtures/v1.3) against the front end and the
simulator, as tests/test_v1_2_fixtures.py does for 1.2 (spec/robolang-v1.3.md section 10):

- valid fixtures pass the complete front end with no diagnostics and match their golden IR
  snapshot (tests/snapshots/ir-v1.3/), with language 1.3 and IR schema 1.2 (G-T1);
- invalid fixtures are rejected with the stated code and line, and produce no IR (G-T2);
- compat fixtures (RoboLang 1.1 and 1.2) keep their result (G-T3);
- simulation cases give the stated exit status and run-record fields (G-T4 to G-T12), and
  examples/pick_and_place_v13.rob ends SUCCEEDED with its goal met.

To regenerate the snapshots after an intended IR change:

    ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_v1_3_fixtures.py
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
from robolang.ir import compile_source, to_json

ROOT = Path(__file__).parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "v1.3"
SNAPSHOTS = ROOT / "tests" / "snapshots" / "ir-v1.3"
UPDATE = os.environ.get("ROBOLANG_UPDATE_SNAPSHOTS") == "1"
SPEC = (ROOT / "spec" / "robolang-v1.3.md").read_text(encoding="utf-8")
SPEC_V1_1 = (ROOT / "spec" / "robolang-v1.1.md").read_text(encoding="utf-8")
SPEC_V1_2 = (ROOT / "spec" / "robolang-v1.2.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")
CASES = json.loads((FIXTURES / "simulation" / "cases.json").read_text(encoding="utf-8"))["cases"]

HEADER = re.compile(
    r"\A// RoboLang (?P<version>v1\.[123]) fixture: (?P<set>valid|invalid|compat)\n"
    r"// expect: (?:(?P<ok>ok)|error (?P<code>E\d{3}) \((?P<stage>lexical|syntax|static)\) line (?P<line>\d+))\n"
    r"// why: .+\n"
)
STAGE_DIGIT = {"lexical": "1", "syntax": "2", "static": "3"}
EXPECTED_COUNTS = {"valid": 12, "invalid": 41, "compat": 8}


def header(path: Path) -> dict:
    match = HEADER.match(path.read_text(encoding="utf-8"))
    assert match, f"{path.name}: missing or malformed header"
    return match.groupdict()


def files(folder: str) -> list:
    return [pytest.param(p, id=p.stem) for p in sorted((FIXTURES / folder).glob("*.rob"))]


def defined_codes(text: str) -> set[str]:
    return set(re.findall(r"^\| `([EWS]\d{3})` \|", text, re.MULTILINE))


def test_fixture_counts_match_the_specification():
    counts = {folder: len(list((FIXTURES / folder).glob("*.rob"))) for folder in EXPECTED_COUNTS}
    assert counts == EXPECTED_COUNTS
    assert len(CASES) == 44
    for folder, count in counts.items():
        assert f"| `{folder}/` |" in SPEC and f"| {count} |" in SPEC
    assert "| `simulation/` |" in SPEC and "| 44 cases |" in SPEC


# -------- G-T1: valid fixtures --------


@pytest.mark.parametrize("path", files("valid"))
def test_valid_fixture_passes_and_matches_its_ir_snapshot(path):
    assert header(path)["version"] == "v1.3" and header(path)["ok"]
    assert not check_file(path).warnings

    actual = to_json(compile_source(path.read_text(encoding="utf-8"), str(path)).ir)
    snapshot = SNAPSHOTS / f"{path.stem}.json"
    if UPDATE:
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        snapshot.write_bytes(actual.encode("ascii"))
    assert snapshot.exists(), f"missing snapshot {snapshot.name}"
    assert actual == snapshot.read_text(encoding="ascii")
    document = json.loads(actual)
    assert (document["language_version"], document["ir_schema_version"]) == ("1.3", "1.2")
    assert all("goal" in task and task["goal"] for task in document["tasks"])


def test_every_snapshot_belongs_to_a_fixture():
    assert {p.stem for p in SNAPSHOTS.glob("*.json")} == {p.stem for p in (FIXTURES / "valid").glob("*.rob")}


# -------- G-T2: invalid fixtures --------


@pytest.mark.parametrize("path", files("invalid"))
def test_invalid_fixture_is_rejected_with_the_stated_code_and_line(path):
    expected = header(path)
    code = expected["code"]
    assert expected["version"] == "v1.3" and expected["set"] == "invalid"
    assert path.name.startswith(code.lower() + "_")
    assert code[1] == STAGE_DIGIT[expected["stage"]]
    assert code in defined_codes(SPEC) | defined_codes(SPEC_V1_2) | defined_codes(SPEC_V1_1), f"{code} is not defined"

    if expected["stage"] == "static":
        parse_file(path)  # syntactically valid: the error is a static one
    with pytest.raises(RoboLangError) as raised:
        compile_source(path.read_text(encoding="utf-8"), str(path))
    assert (raised.value.code, raised.value.line) == (code, int(expected["line"])), raised.value


def test_every_new_static_and_syntax_code_has_a_fixture():
    covered = {header(p)["code"] for p in (FIXTURES / "invalid").glob("*.rob")}
    assert {"E208", "E309", "E310", "E303", "E304", "E212", "E201"} <= covered


# -------- G-T3: compat fixtures --------


@pytest.mark.parametrize("path", files("compat"))
def test_compat_fixture_keeps_its_earlier_result(path):
    expected = header(path)
    assert expected["version"] in ("v1.1", "v1.2") and expected["set"] == "compat"
    language = expected["version"][1:]
    if expected["ok"]:
        assert not check_file(path).warnings
        assert compile_source(path.read_text(encoding="utf-8")).ir.language_version == language
        return
    with pytest.raises(RoboLangError) as raised:
        check_file(path)
    assert (raised.value.code, raised.value.line) == (expected["code"], int(expected["line"])), raised.value


# -------- G-T4 to G-T12: simulation --------


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


def subset(actual, expected, path="final_world"):
    """Every key given in `expected` matches `actual`; keys not given are not checked."""
    if isinstance(expected, dict):
        for key, value in expected.items():
            subset(actual[key], value, f"{path}.{key}")
    else:
        assert actual == expected, path


def test_simulation_fixture_files_are_all_used():
    sim = FIXTURES / "simulation"
    used = {case["source"] for case in CASES} | {case["world"] for case in CASES}
    for path in sorted((sim / "sources").glob("*.rob")) + sorted((sim / "worlds").glob("*.json")):
        assert path.relative_to(ROOT).as_posix() in used, f"{path.name} is not used by any case"
    run_codes = defined_codes(SIM_SPEC) | defined_codes(SPEC)
    for case in CASES:
        assert set(case["tests"]) <= {f"G-T{n}" for n in range(4, 13)}
        if case["expect"].get("failure"):
            assert case["expect"]["failure"]["code"] in run_codes


@pytest.mark.parametrize("case", CASES, ids=lambda case: case["id"])
def test_simulation_case(case):
    expect = case["expect"]
    code, out, err = simulate(case)

    assert code == expect["exit"], err
    if expect["exit"] == 3:  # the world file is not valid: no run record
        assert out == "" and f"error[{expect['diagnostic']}]" in err
        return
    record = json.loads(out)
    assert record["run_record_schema_version"] == "1.2"
    assert (record["inputs"]["language_version"], record["inputs"]["ir_schema_version"]) == ("1.3", "1.2")
    assert record["mode"] == "simulation"
    for key in ("states", "state", "failure", "cancelled_before_step", "end_t_ms", "events_applied"):
        if key in expect:
            assert record[key] == expect[key], key
    assert [g["result"] for g in record["goal"]] == expect["goal"]
    assert [s["status"] for s in record["steps"]] == expect["steps"]
    if "preconditions" in expect:
        assert [p["result"] for p in record["preconditions"]] == expect["preconditions"]
    if "observations" in expect:
        assert [s["observation"] for s in record["steps"] if s["observation"]] == expect["observations"]
    if "final_world" in expect:
        subset(record["final_world"], expect["final_world"])
    assert simulate(case)[:2] == (code, out)  # deterministic: byte-identical record


def test_the_goal_follows_the_preconditions_in_the_record():
    case = next(c for c in CASES if c["id"] == "g1_goal_met")
    record = json.loads(simulate(case)[1])
    keys = list(record)
    assert keys.index("goal") == keys.index("preconditions") + 1
    assert [g["condition"]["kind"] for g in record["goal"]] == ["object_at"]
