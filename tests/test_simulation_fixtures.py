"""
Consistency checks for the RL-BASIC-01 simulation fixtures (tests/fixtures/simulation).

No simulator exists yet (RL-BASIC-01b). These tests execute nothing: they check that
the fixtures agree with spec/simulation-v1.0.md and with the existing v1.1 front end,
so that a later implementation is tested against fixtures that are themselves sound.
"""

from __future__ import annotations
import hashlib
import json
import re
from pathlib import Path

import pytest

from robolang import RoboLangError
from robolang.ir import compile_source

ROOT = Path(__file__).parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "simulation"
SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")

EXIT_FOR_STATE = {"SUCCEEDED": 0, "FAILED": 4, "CANCELLED": 5}
ENTITY_KINDS = {"robot": "robots", "object": "objects", "location": "locations", "region": "regions"}


def _no_duplicate_keys(pairs):
    keys = [key for key, _ in pairs]
    assert len(keys) == len(set(keys)), f"duplicate key in {keys}"
    return dict(pairs)


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_no_duplicate_keys)


CASES = load_json(FIXTURES / "cases.json")["cases"]


def compile_file(path: str):
    return compile_source((ROOT / path).read_bytes().decode("utf-8"), path).ir


def sha256_normalized(path: str) -> str:
    """Specification section 6: drop a leading BOM, turn CR LF into LF, hash."""
    data = (ROOT / path).read_bytes()
    if data.startswith(b"\xef\xbb\xbf"):
        data = data[3:]
    return hashlib.sha256(data.replace(b"\r\n", b"\n")).hexdigest()


def selected_task(case):
    ir = compile_file(case["source"])
    if case["task"] is not None:
        return ir.task(case["task"])
    return ir.tasks[0] if len(ir.tasks) == 1 else None


def runs(case) -> bool:
    return case["expect"]["exit"] in EXIT_FOR_STATE.values()


def test_case_ids_are_unique_and_match_the_specification():
    ids = [case["id"] for case in CASES]
    assert len(ids) == len(set(ids))
    section_8 = SPEC.split("## 8.")[1].split("## 9.")[0]
    assert set(re.findall(r"`(t\d\d_\w+)`", section_8)) == set(ids)


def test_every_fixture_test_is_covered():
    covered = {test for case in CASES for test in case["tests"]}
    assert covered == {f"T{n}" for n in range(1, 18)}


def test_every_fixture_file_is_used():
    used = {case["world"] for case in CASES} | {case.get("source") for case in CASES}
    for path in sorted((FIXTURES / "worlds").glob("*.json")) + sorted((FIXTURES / "sources").glob("*.rob")):
        assert path.relative_to(ROOT).as_posix() in used, f"{path.name} is not used by any case"


def test_codes_are_defined_in_the_specification():
    defined = set(re.findall(r"^\| `([ES]\d{3})` \|", SPEC, re.MULTILINE))
    for case in CASES:
        expect = case["expect"]
        code = expect.get("diagnostic") or (expect.get("failure") or {}).get("code")
        if code:
            assert code in defined, f"{case['id']}: {code} is not defined"


@pytest.mark.parametrize("case", CASES, ids=lambda case: case["id"])
def test_case_is_consistent(case):
    expect = case["expect"]
    world = load_json(ROOT / case["world"])

    if expect["exit"] == 1:
        paths = sorted(ROOT.glob(case["source_glob"]))
        assert paths
        for path in paths:
            with pytest.raises(RoboLangError):
                compile_file(path.relative_to(ROOT).as_posix())
        return

    ir = compile_file(case["source"])
    if expect["exit"] == 2:
        assert case["task"] is None and len(ir.tasks) > 1
        return
    task = selected_task(case)
    assert task is not None

    if expect["exit"] == 3:
        assert re.fullmatch(r"E5\d\d", expect["diagnostic"])
        return

    assert world["world_schema"] == "robolang.sim.world"
    assert world["world_schema_version"] == "1.0"
    entry = world["tasks"][task.name]
    assert set(entry["bindings"]) == {p.name for p in task.parameters}
    for parameter in task.parameters:
        declared = world.get(ENTITY_KINDS[parameter.type], [])
        assert entry["bindings"][parameter.name] in declared, parameter.name
    for fault in entry.get("faults", []):
        assert 1 <= fault["step"] <= len(task.steps)
    if "cancel" in entry:
        assert 1 <= entry["cancel"]["before_step"] <= len(task.steps)

    assert expect["exit"] == EXIT_FOR_STATE[expect["state"]]
    assert expect["states"][-1] == expect["state"]
    assert len(expect["steps"]) == len(task.steps)
    assert len(expect["preconditions"]) == len(task.preconditions)
    if "safety_limits" in expect:
        assert len(expect["safety_limits"]) == len(task.safety)
    if "step_times" in expect:
        assert len(expect["step_times"]) == len(task.steps)
    failure = expect["failure"]
    if failure is not None:
        assert expect["state"] == "FAILED"
        if failure["step"] is not None:
            assert expect["steps"][failure["step"] - 1] == "failed"


@pytest.mark.parametrize(
    "case", [case for case in CASES if "run_id" in case["expect"]], ids=lambda case: case["id"]
)
def test_run_id_vectors_follow_section_6(case):
    task = selected_task(case)
    preimage = (
        f"robolang-sim-run/1.0\n{sha256_normalized(case['source'])}\n"
        f"{sha256_normalized(case['world'])}\n{task.name}\n{case['seed']}\n"
    )
    assert case["expect"]["run_id"] == "sim-" + hashlib.sha256(preimage.encode("ascii")).hexdigest()


def test_different_seeds_give_different_run_ids():
    ids = {case["seed"]: case["expect"]["run_id"] for case in CASES if case["id"].startswith("t11_")}
    assert len(ids) == 2 and len(set(ids.values())) == 2
