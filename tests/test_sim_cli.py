"""Tests for `robolang simulate` (spec/simulation-v1.0.md sections 2, 5, 6 and 8) against the acceptance fixtures."""

from __future__ import annotations
import io
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from robolang.cli import main
from robolang.sim import SEED, run_id, sha256_normalized

REPO_ROOT = Path(__file__).resolve().parent.parent
SIM = Path(__file__).parent / "fixtures" / "simulation"
BASE_WORLD = "tests/fixtures/simulation/worlds/base.json"
DEMO = "examples/pick_and_place.rob"
MULTI = "tests/fixtures/valid/multiple_tasks.rob"

CASES = json.loads((SIM / "cases.json").read_text(encoding="utf-8"))["cases"]

TOP_KEYS = [
    "run_record_schema", "run_record_schema_version", "mode", "run_id", "inputs", "task", "bindings",
    "states", "state", "failure", "cancelled_before_step", "safety_limits", "preconditions", "steps",
    "events_applied", "end_t_ms", "final_world",
]
INPUT_KEYS = [
    "source_sha256", "world_sha256", "task", "seed", "language_version", "ir_schema_version",
    "world_schema_version",
]
FAILURE_KEYS = ["code", "phase", "step", "precondition", "safety_limit", "robot"]
STEP_KEYS = [
    "index", "step", "status", "t_start_ms", "t_end_ms", "observation", "message", "condition_result",
]
OBSERVATION_KEYS = ["object", "present", "location", "held_by"]
MESSAGE_KEYS = ["robot", "channel", "message", "status"]
EVENT_KEYS = ["t_ms", "fact", "target", "value"]
WORLD_KEYS = ["robots", "objects", "regions", "sensors"]
LIMIT_STATUSES = {
    "unit_unspecified_not_enforced", "checked_against_profile_not_enforced", "failed_check", "not_checked",
}


def simulate(source: str, world: str, task: str | None = None, seed: str = "0", extra=()):
    argv = ["simulate", source, "--world", world]
    if task is not None:
        argv += ["--task", task]
    argv += ["--seed", seed, *extra]
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(REPO_ROOT)
    try:
        code = main(argv, out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


def _runs():
    for case in CASES:
        sources = [case["source"]] if case.get("source") else sorted(
            p.relative_to(REPO_ROOT).as_posix() for p in REPO_ROOT.glob(case["source_glob"])
        )
        assert sources, case["id"]
        for source in sources:
            yield pytest.param(case, source, id=f"{case['id']}:{Path(source).name}")


RUNS = list(_runs())


def _check_record_shape(record: dict) -> None:
    assert list(record) == TOP_KEYS
    assert list(record["inputs"]) == INPUT_KEYS
    assert record["run_record_schema"] == "robolang.sim.run"
    assert record["run_record_schema_version"] == "1.0"
    assert list(record["final_world"]) == WORLD_KEYS
    if record["failure"] is not None:
        assert list(record["failure"]) == FAILURE_KEYS
    for step in record["steps"]:
        assert list(step) == STEP_KEYS
        if step["observation"] is not None:
            assert list(step["observation"]) == OBSERVATION_KEYS
        if step["message"] is not None:
            assert list(step["message"]) == MESSAGE_KEYS
    for event in record["events_applied"]:
        assert list(event) == EVENT_KEYS
    for entry in record["safety_limits"]:
        assert list(entry) == ["limit", "status"]
    for entry in record["preconditions"]:
        assert list(entry) == ["index", "condition", "result"]
    for entities in record["final_world"].values():
        assert list(entities) == sorted(entities)


def _is_subset(expected, actual) -> bool:
    if isinstance(expected, dict):
        return isinstance(actual, dict) and all(k in actual and _is_subset(v, actual[k]) for k, v in expected.items())
    return expected == actual


@pytest.mark.parametrize(("case", "source"), RUNS)
def test_case(case, source):
    expect = case["expect"]
    code, out, err = simulate(source, case["world"], case["task"], case["seed"])

    assert code == expect["exit"]
    if code in (1, 2, 3):
        assert out == ""
        if code == 3:
            assert f"error[{expect['diagnostic']}]" in err
        return

    assert code in (0, 4, 5)
    assert out.endswith("}\n") and not out.endswith("\n\n")
    assert "\r" not in out
    out.encode("ascii")
    lines = err.strip().splitlines()
    assert lines[-1].startswith("simulation only: run sim-")
    assert sum(line.startswith("simulation only:") for line in lines) == 1

    record = json.loads(out)
    _check_record_shape(record)
    assert record["mode"] == "simulation"
    assert lines[-1].startswith(f"simulation only: run {record['run_id']}: {record['state']}")
    if record["failure"] is not None:
        assert lines[-1].endswith(f"({record['failure']['code']})")
    assert all(entry["status"] in LIMIT_STATUSES for entry in record["safety_limits"])

    for key in ("run_id", "states", "state", "failure", "cancelled_before_step", "end_t_ms", "events_applied"):
        if key in expect:
            assert record[key] == expect[key], key
    if "preconditions" in expect:
        assert [p["result"] for p in record["preconditions"]] == expect["preconditions"]
    if "steps" in expect:
        assert [s["status"] for s in record["steps"]] == expect["steps"]
    if "safety_limits" in expect:
        assert [s["status"] for s in record["safety_limits"]] == expect["safety_limits"]
    if "step_times" in expect:
        times = [None if s["t_start_ms"] is None else [s["t_start_ms"], s["t_end_ms"]] for s in record["steps"]]
        assert times == expect["step_times"]
    if "messages" in expect:
        assert [s["message"] for s in record["steps"] if s["message"] is not None] == expect["messages"]
    if "observations" in expect:
        assert [s["observation"] for s in record["steps"] if s["observation"] is not None] == expect["observations"]
    if "final_world" in expect:
        assert _is_subset(expect["final_world"], record["final_world"])

    for _ in range(case.get("repeat", 1) - 1):
        again = simulate(source, case["world"], case["task"], case["seed"])
        assert again[:2] == (code, out)  # T11: byte-identical


def test_the_demo_record_has_the_specified_nested_shape():
    code, out, _ = simulate(DEMO, BASE_WORLD)
    record = json.loads(out)

    assert code == 0
    _check_record_shape(record)
    assert record["inputs"]["language_version"] == "1.1"
    assert record["inputs"]["ir_schema_version"] == "1.0"
    assert record["inputs"]["world_schema_version"] == "1.0"
    assert record["inputs"]["task"] == record["task"]["name"]
    assert list(record["bindings"]) == [p["name"] for p in record["task"]["parameters"]]
    assert out == json.dumps(record, indent=2, ensure_ascii=True) + "\n"


def test_failed_run_record_has_the_failure_object():
    code, out, err = simulate(DEMO, "tests/fixtures/simulation/worlds/region_blocked.json")
    record = json.loads(out)

    assert code == 4
    assert list(record["failure"]) == FAILURE_KEYS
    assert "FAILED (S101)" in err


def test_no_output_for_invalid_source_or_world():
    code, out, err = simulate("tests/fixtures/invalid/e303_undeclared_name.rob", BASE_WORLD)
    assert (code, out) == (1, "") and "error[E303]" in err


# -------- usage errors (exit 2, nothing on stdout) --------

@pytest.mark.parametrize("seed", ["a b", "x" * 65, ""])
def test_bad_seed_is_a_usage_error(seed):
    code, out, err = simulate(DEMO, BASE_WORLD, seed=seed)
    assert (code, out) == (2, "") and "--seed" in err


def test_seed_of_64_characters_is_accepted():
    assert SEED.fullmatch("A" * 64) and simulate(DEMO, BASE_WORLD, seed="a.B_c-9" * 9)[0] == 0


def test_unknown_task_is_a_usage_error():
    code, out, err = simulate(DEMO, BASE_WORLD, task="nope")
    assert (code, out) == (2, "") and "no task named 'nope'" in err


def test_several_tasks_and_no_task_is_a_usage_error():
    out, err = io.StringIO(), io.StringIO()
    code = main(["simulate", str(REPO_ROOT / MULTI), "--world", str(REPO_ROOT / BASE_WORLD)], out=out, err=err)
    assert (code, out.getvalue()) == (2, "") and "several tasks" in err.getvalue()


def test_missing_world_option_is_a_usage_error():
    out, err = io.StringIO(), io.StringIO()
    assert main(["simulate", str(REPO_ROOT / DEMO)], out=out, err=err) == 2
    assert out.getvalue() == ""


def test_unreadable_world_is_a_usage_error():
    code, out, err = simulate(DEMO, "tests/fixtures/simulation/worlds/does_not_exist.json")
    assert (code, out) == (2, "") and "cannot read" in err


def test_unreadable_source_is_a_usage_error():
    code, out, _ = simulate("does_not_exist.rob", BASE_WORLD)
    assert (code, out) == (2, "")


# -------- run ID --------

def test_bom_and_crlf_variants_give_the_same_hash_and_id():
    plain = b"line one\nline two\n"
    variant = b"\xef\xbb\xbf" + plain.replace(b"\n", b"\r\n")

    assert sha256_normalized(plain) == sha256_normalized(variant)
    assert run_id(plain, b"{}", "t", "0") == run_id(variant, b"{}", "t", "0")
    assert sha256_normalized(plain + b" ") != sha256_normalized(plain)


def test_only_one_bom_is_dropped():
    assert sha256_normalized(b"\xef\xbb\xbf\xef\xbb\xbfx") != sha256_normalized(b"x")


def test_a_different_seed_task_or_input_gives_a_different_id():
    base = run_id(b"a", b"b", "t", "0")
    assert base.startswith("sim-") and len(base) == 4 + 64
    assert run_id(b"a", b"b", "t", "1") != base
    assert run_id(b"a", b"b", "u", "0") != base
    assert run_id(b"a", b"c", "t", "0") != base
    assert run_id(b"a", b"b", "t", "0") == base


def test_the_default_seed_is_zero():
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(REPO_ROOT)
    try:
        assert main(["simulate", DEMO, "--world", BASE_WORLD], out=out, err=err) == 0
    finally:
        os.chdir(previous)
    assert json.loads(out.getvalue())["inputs"]["seed"] == "0"


# -------- T18: no adapter, no ROS 2 --------

def test_simulate_does_not_load_the_interpreter_or_adapters():
    code = (
        "import sys; sys.path.insert(0, 'src'); from robolang.cli import main; "
        f"assert main(['simulate', '{DEMO}', '--world', '{BASE_WORLD}'], "
        "out=open(__import__('os').devnull, 'w'), err=open(__import__('os').devnull, 'w')) == 0; "
        "loaded = [m for m in sys.modules if m.startswith('robolang_') or m == 'rclpy']; "
        "assert not loaded, loaded"
    )
    result = subprocess.run([sys.executable, "-c", code], cwd=REPO_ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
