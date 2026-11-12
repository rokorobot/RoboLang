"""What a user who has only the PyPI package meets first: `robolang example`, world errors, the summary line."""

from __future__ import annotations
import io
import re
from pathlib import Path

from robolang import cli
from robolang.cli import main

ROOT = Path(__file__).resolve().parent.parent
SHIPPED = ROOT / "src" / "robolang" / "reference"


def run(*argv):
    out, err = io.StringIO(), io.StringIO()
    return main(list(argv), out=out, err=err), out.getvalue(), err.getvalue()


def test_example_writes_a_task_and_world_that_succeed_then_fail_without_place(tmp_path):
    code, out, err = run("example", str(tmp_path / "demo"))
    assert code == 0 and out == "" and "robolang simulate" in err
    rob, world = tmp_path / "demo" / "fetch.rob", tmp_path / "demo" / "world.json"
    assert run("simulate", str(rob), "--world", str(world))[0] == 0
    rob.write_text(re.sub(r".*place r box.*\n", "", rob.read_text()))
    code, _, err = run("simulate", str(rob), "--world", str(world))
    assert code == 4 and "FAILED (S301): a goal condition is false after the last step" in err


def test_example_never_overwrites(tmp_path):
    (tmp_path / "world.json").write_text("mine")
    code, _, err = run("example", str(tmp_path))
    assert code == 2 and "not overwriting" in err
    assert (tmp_path / "world.json").read_text() == "mine" and not (tmp_path / "fetch.rob").exists()


def test_readme_quickstart_shows_the_shipped_files():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for name in cli._EXAMPLE_FILES:
        assert (SHIPPED / name).read_text(encoding="utf-8") in readme


def test_every_failure_code_in_the_spec_has_a_meaning():
    spec = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")
    assert set(re.findall(r"^\| `(S\d{3})` \|", spec, flags=re.M)) == set(cli._FAILURE_MEANING)


def world(tmp_path, text):
    rob = tmp_path / "t.rob"
    rob.write_text("robolang 1.6;\ntask t(robot r, location d) { goal { robot_at r d; } plan { move r to d; } }\n")
    path = tmp_path / "w.json"
    path.write_text(text)
    return run("simulate", str(rob), "--world", str(path))


def test_an_empty_world_names_every_missing_field_and_the_accepted_versions(tmp_path):
    code, _, err = world(tmp_path, "{}")
    assert code == 3 and "error[E502]" in err
    assert "'world_schema', 'world_schema_version', 'locations', 'tasks'" in err
    assert "'1.0', '1.1', '1.2'" in err


def test_a_robot_names_all_its_missing_fields_at_once(tmp_path):
    code, _, err = world(tmp_path, '{"world_schema": "robolang.sim.world", "world_schema_version": "1.2", '
                                   '"locations": ["x"], "robots": {"a": {}}, "tasks": {}}')
    assert code == 3 and "error[E504] at /robots/a: missing required fields 'location', 'gripper'" in err
