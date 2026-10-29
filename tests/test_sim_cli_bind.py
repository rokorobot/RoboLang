"""Tests for `robolang simulate --bind` (spec/simulation-v1.0.md section 2.5, decision record 0016)."""

from __future__ import annotations
import io
import json
import os
from pathlib import Path

import pytest

from robolang.cli import main
from robolang.sim import run_id

REPO_ROOT = Path(__file__).resolve().parent.parent
WORLDS = REPO_ROOT / "tests" / "fixtures" / "simulation" / "worlds"
BASE_WORLD = "tests/fixtures/simulation/worlds/base.json"
DEMO = "examples/pick_and_place.rob"
TASK = "demo_pick_and_place"
GOOD = ["r=arm1", "box=box1", "src=shelf", "dst=table", "cell=cell"]


def simulate(*extra: str, world: str = BASE_WORLD, source: str = DEMO, seed: str = "0"):
    argv = ["simulate", source, "--world", world, "--task", TASK, "--seed", seed, *extra]
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(REPO_ROOT)
    try:
        code = main(argv, out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


def bind(*pairs: str) -> list[str]:
    return [arg for pair in pairs for arg in ("--bind", pair)]


def world_with_task_entry(tmp_path, entry) -> str:
    """base.json with the demo task's entry replaced (or removed when `entry` is None)."""
    doc = json.loads((WORLDS / "base.json").read_text(encoding="utf-8"))
    if entry is None:
        del doc["tasks"][TASK]
    else:
        doc["tasks"][TASK] = entry
    path = tmp_path / "world.json"
    path.write_text(json.dumps(doc), encoding="utf-8")
    return str(path)


def test_repeated_flag_succeeds_and_records_cli_source():
    code, out, err = simulate(*bind(*GOOD))
    assert code == 0, err
    record = json.loads(out)
    assert record["state"] == "SUCCEEDED"
    assert record["bindings"] == dict(pair.split("=") for pair in GOOD)
    assert record["inputs"]["bindings_source"] == "cli"
    assert list(record["inputs"])[-1] == "bindings_source"


def test_several_pairs_in_one_flag_give_the_same_record():
    assert simulate("--bind", *GOOD) == simulate(*bind(*GOOD))


def test_world_bindings_keep_the_record_unchanged():
    code, out, _ = simulate()
    assert code == 0
    assert "bindings_source" not in json.loads(out)["inputs"]


def test_cli_bindings_override_world_bindings_without_merging():
    # base.json binds dst=table; --bind chooses shelf for dst and src alike. A merge would keep table.
    code, out, err = simulate(*bind("r=arm1", "box=box1", "src=shelf", "dst=shelf", "cell=cell"))
    record = json.loads(out)
    assert record["bindings"]["dst"] == "shelf", err
    assert record["inputs"]["bindings_source"] == "cli"


def test_partial_cli_bindings_do_not_fall_back_to_the_world_file():
    code, out, err = simulate(*bind("r=arm1"))
    assert code == 3 and out == ""
    assert err.strip().splitlines()[-1] == "--bind: error[E506] at /bindings: parameter 'box' is not bound"


def test_world_entry_may_be_missing_when_bind_covers_everything(tmp_path):
    world = world_with_task_entry(tmp_path, None)
    assert simulate(world=world)[0] == 3  # without --bind the missing entry is still E506
    code, out, err = simulate(*bind(*GOOD), world=world)
    assert code == 0, err
    assert json.loads(out)["state"] == "SUCCEEDED"


def test_world_faults_still_apply_and_its_bindings_are_ignored(tmp_path):
    entry = {"bindings": {"r": "nowhere"}, "faults": [{"step": 3}]}  # would be E505 if it were used
    code, out, err = simulate(*bind(*GOOD), world=world_with_task_entry(tmp_path, entry))
    record = json.loads(out)
    assert code == 4, err
    assert record["failure"]["code"] == "S251" and record["failure"]["step"] == 3
    assert record["bindings"]["r"] == "arm1"


def test_world_entry_without_bindings_is_valid_with_bind(tmp_path):
    world = world_with_task_entry(tmp_path, {"cancel": {"before_step": 2}})
    code, out, err = simulate(*bind(*GOOD), world=world)
    assert code == 5, err
    assert json.loads(out)["cancelled_before_step"] == 2
    code, _, err = simulate(world=world)
    assert code == 3 and "error[E504]" in err


@pytest.mark.parametrize(
    ("pairs", "diagnostic"),
    [
        (["r=arm1", "box=box1", "src=shelf", "dst=table"], "E506"),  # cell missing
        (["r=arm1", "box=box1", "src=shelf", "dst=table", "cell=nowhere"], "E505"),  # undeclared ID
        (["r=box1", "box=box1", "src=shelf", "dst=table", "cell=cell"], "E507"),  # object for a robot
        (["r=arm1", "box=box1", "src=shelf", "dst=cell", "cell=cell"], "E507"),  # region for a location
    ],
)
def test_binding_errors_against_the_world_exit_3(pairs, diagnostic):
    code, out, err = simulate(*bind(*pairs))
    assert code == 3 and out == ""
    assert err.strip().splitlines()[-1].startswith(f"--bind: error[{diagnostic}] at /bindings")


def test_wrong_kind_diagnostic_names_parameter_and_entity():
    _, _, err = simulate(*bind("r=box1", "box=box1", "src=shelf", "dst=table", "cell=cell"))
    assert err.strip().splitlines()[-1] == (
        "--bind: error[E507] at /bindings/r: parameter 'r' is a robot, 'box1' is an object"
    )


@pytest.mark.parametrize(
    "pairs",
    [
        [*GOOD, "colour=red"],  # not a parameter
        [*GOOD, "r2=arm1"],
    ],
)
def test_unknown_parameter_exits_2(pairs):
    code, out, err = simulate(*bind(*pairs))
    assert code == 2 and out == ""
    assert "is not a parameter of task 'demo_pick_and_place'" in err


@pytest.mark.parametrize("extra", [["r=arm1"], ["r=arm2"]])
def test_duplicate_parameter_exits_2(extra):
    code, out, err = simulate(*bind(*GOOD, *extra))
    assert code == 2 and out == ""
    assert "parameter 'r' is bound more than once" in err


@pytest.mark.parametrize("item", ["r", "r=", "=arm1", "r=arm1=x", "r =arm1", "r=arm 1", "1r=arm1", "r=1arm"])
def test_malformed_syntax_exits_2(item):
    code, out, err = simulate("--bind", item)
    assert code == 2 and out == ""
    assert f"invalid --bind '{item}'" in err


def test_bind_without_a_value_is_a_usage_error(capsys):
    code, out, _ = simulate("--bind")
    assert code == 2 and out == ""
    assert "--bind: expected at least one argument" in capsys.readouterr().err


def test_run_id_is_deterministic_and_follows_the_bindings():
    first = json.loads(simulate(*bind(*GOOD))[1])
    again = json.loads(simulate(*bind(*reversed(GOOD)))[1])  # flag order does not matter
    assert first == again
    other = json.loads(simulate(*bind("r=arm2", *GOOD[1:]))[1])
    assert other["run_id"] != first["run_id"]
    assert json.loads(simulate(*bind(*GOOD), seed="1")[1])["run_id"] != first["run_id"]
    assert first["run_id"] != json.loads(simulate()[1])["run_id"]  # same bindings from the world file: other source


def test_run_id_derivation():
    source = (REPO_ROOT / DEMO).read_bytes()
    world = (REPO_ROOT / BASE_WORLD).read_bytes()
    bindings = dict(pair.split("=") for pair in GOOD)
    record = json.loads(simulate(*bind(*GOOD))[1])
    assert record["run_id"] == run_id(source, world, TASK, "0", bindings)
    assert run_id(source, world, TASK, "0") == json.loads(simulate()[1])["run_id"]  # unchanged without --bind
