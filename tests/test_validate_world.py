"""Tests for `robolang validate --world` (spec/simulation-v1.0.md section 11, development plan N2)."""

from __future__ import annotations
import io
import json
from pathlib import Path

import pytest

from robolang.cli import main
from robolang.sim import load_world
from robolang.sim import run as simulate_task
from robolang.sim.check import check_task
from robolang import ir

WORLD = str(Path(__file__).resolve().parent.parent / "examples" / "world.json")
TASK = """robolang 1.3;

task carry(robot r, object box, location src, location dst, sensor cam) {
    @safety max_speed SPEED;
    goal { object_at box dst; }
    plan {
        move r to src;
        grasp r box;
        inspect r box using cam;
        move r to dst;
        place r box at dst;
    }
}
"""
BIND = ["--bind", "box=box1", "src=shelf", "dst=table", "cam=camera_top"]


def validate(tmp_path, *extra: str, speed: str = "0.5 m/s", source: str | None = None):
    path = tmp_path / "task.rob"
    path.write_text(source or TASK.replace("SPEED", speed), encoding="utf-8")
    out, err = io.StringIO(), io.StringIO()
    code = main(["validate", str(path), *extra], out=out, err=err)
    return code, out.getvalue(), err.getvalue().replace(str(path), "task.rob")


def test_a_task_that_fits_the_world_is_valid(tmp_path):
    code, out, err = validate(tmp_path, "--world", WORLD, *BIND, "r=arm1")
    assert code == 0, err
    assert "valid RoboLang 1.3" in out and f"checked against {WORLD} (--bind)" in out


def test_every_certain_failure_is_reported_at_its_source_position(tmp_path):
    # amr1: 1.5 m/s maximum, no rad/s, gripper, but camera_top is mounted on arm1
    source = TASK.replace("@safety max_speed SPEED;", "@safety max_speed 2 m/s;\n    @safety max_speed 1 rad/s;")
    code, out, err = validate(tmp_path, "--world", WORLD, *BIND, "r=amr1", source=source)
    assert code == 1 and out == ""
    assert err.splitlines() == [
        "task.rob:4:5: error[S001]: max_speed 2 m/s is above the maximum of robot 'amr1' (1.5 m/s)",
        "task.rob:5:5: error[S002]: max_speed is in rad/s, but robot 'amr1' declares no maximum in rad/s "
        "(its units: m/s)",
        "task.rob:10:9: error[S231]: sensor 'camera_top' is mounted on 'arm1', not on robot 'amr1'",
        f"task.rob: task 'carry' cannot succeed in {WORLD} (3 errors)",
    ]


def test_grasp_without_a_gripper(tmp_path):
    world = json.loads(open(WORLD, encoding="utf-8").read())
    world["robots"]["arm1"]["gripper"] = False
    path = tmp_path / "world.json"
    path.write_text(json.dumps(world), encoding="utf-8")
    code, _, err = validate(tmp_path, "--world", str(path), *BIND, "r=arm1")
    assert code == 1 and err == "task.rob:8:9: error[S211]: robot 'arm1' has no gripper\n"


def test_bindings_and_world_errors_are_the_simulate_ones(tmp_path):
    code, _, err = validate(tmp_path, "--world", WORLD, *BIND, "r=arm9")
    assert code == 3 and err == "--bind: error[E505] at /bindings/r: 'arm9' is not declared\n"


def test_task_and_bind_need_world(tmp_path, capsys):
    assert validate(tmp_path, *BIND, "r=arm1")[0] == 2
    assert "--task and --bind need --world" in capsys.readouterr().err


def test_a_syntax_error_is_reported_before_the_world_is_read(tmp_path):
    code, _, err = validate(tmp_path, "--world", "missing.json", *BIND, "r=arm1", speed="0.5 m/")
    assert code == 1 and "error[E" in err and "missing.json" not in err


def test_ir_input_reports_json_pointers(tmp_path):
    rob = tmp_path / "task.rob"
    rob.write_text(TASK.replace("SPEED", "2 m/s"), encoding="utf-8")
    ir_path = tmp_path / "task.json"
    out = io.StringIO()
    assert main(["inspect", str(rob), "--json"], out=out, err=io.StringIO()) == 0
    ir_path.write_text(out.getvalue(), encoding="utf-8")
    err = io.StringIO()
    code = main(["validate", "--ir", str(ir_path), "--world", WORLD, *BIND, "r=arm1"], out=io.StringIO(), err=err)
    assert code == 1
    assert err.getvalue().endswith("/tasks/0/safety/0: error[S001]: max_speed 2 m/s is above the maximum of robot "
                                   "'arm1' (1 m/s)\n")


@pytest.mark.parametrize("robot, speed", [("arm1", "0.5 m/s"), ("arm1", "2 m/s"), ("amr1", "1 rad/s"),
                                          ("amr1", "1 m/s")])
def test_a_finding_is_the_failure_the_simulation_hits(robot, speed):
    """Each finding is a real run failure: the first one is the code `simulate` ends with."""
    task = ir.compile_source(TASK.replace("SPEED", speed), "task.rob").ir.tasks[0]
    bindings = {"r": robot, "box": "box1", "src": "shelf", "dst": "table", "cam": "camera_top"}
    world = load_world(open(WORLD, "rb").read(), WORLD, task, bindings)
    findings = check_task(task, world)
    result = simulate_task(task, world)
    if findings:
        assert result.state == "FAILED" and result.failure.code == findings[0].code
    else:
        assert result.state == "SUCCEEDED"


# -------- RoboLang 1.4: every path checked as a straight line (decision 0027 B6) --------

BRANCHING = """robolang 1.4;

task carry(robot r, object box, location src, location dst, sensor cam, region cell) {
    goal { not holding r box; }
    plan {
        move r to src;
        grasp r box;
        if region_clear cell {
            goal { object_at box dst; }
            inspect r box using cam;
            move r to dst;
            place r box at dst;
        } else {
            if robot_ready r {
                goal { object_at box src; }
                inspect r box using cam;
                place r box at src;
            } else {
                fail "stuck";
            }
        }
        inspect r box using cam;
    }
}
"""
BRANCH_BIND = ["--bind", "box=box1", "src=shelf", "dst=table", "cam=camera_top", "cell=bay"]


def branch_world(tmp_path):
    world = json.loads(Path(WORLD).read_text(encoding="utf-8"))
    world["regions"], world["events"] = {"bay": {"clear": True}}, []
    path = tmp_path / "world.json"
    path.write_text(json.dumps(world), encoding="utf-8")
    return str(path)


def test_a_finding_in_a_branch_names_its_path_and_is_reported_once(tmp_path):
    world = branch_world(tmp_path)
    code, out, err = validate(tmp_path, "--world", world, *BRANCH_BIND, "r=amr1", source=BRANCHING)
    assert code == 1 and out == ""
    assert err.splitlines() == [
        "task.rob:10:13: error[S231]: sensor 'camera_top' is mounted on 'arm1', not on robot 'amr1' (on path 3.then)",
        "task.rob:16:17: error[S231]: sensor 'camera_top' is mounted on 'arm1', not on robot 'amr1' "
        "(on path 3.else.1.then)",
        "task.rob:22:9: error[S231]: sensor 'camera_top' is mounted on 'arm1', not on robot 'amr1'",
        f"task.rob: task 'carry' cannot succeed in {world} (3 errors)",
    ]


def test_a_branching_task_that_fits_the_world_is_checked_on_every_path(tmp_path):
    code, out, err = validate(tmp_path, "--world", branch_world(tmp_path), *BRANCH_BIND, "r=arm1", source=BRANCHING)
    assert code == 0, err
    assert "every path of its branches checked" in out and "were not checked" not in out


def test_a_finding_in_a_branch_of_task_ir_points_into_the_branch(tmp_path):
    rob, ir_path = tmp_path / "t.rob", tmp_path / "t.ir.json"
    rob.write_text(BRANCHING, encoding="utf-8")
    out = io.StringIO()
    assert main(["inspect", str(rob), "--json"], out=out, err=io.StringIO()) == 0
    ir_path.write_text(out.getvalue(), encoding="utf-8")
    err = io.StringIO()
    code = main(["validate", "--ir", str(ir_path), "--world", branch_world(tmp_path), *BRANCH_BIND, "r=amr1"],
                out=io.StringIO(), err=err)
    assert code == 1
    assert "/tasks/0/steps/2/then/0: error[S231]" in err.getvalue()
    assert "/tasks/0/steps/2/else/0/then/0: error[S231]" in err.getvalue()


def test_a_path_finding_is_a_failure_the_simulation_hits(tmp_path):
    """On the path that takes the branch with the bad step, the run ends in the finding's code."""
    task = ir.compile_source(BRANCHING, "task.rob").ir.tasks[0]
    bindings = {"r": "amr1", "box": "box1", "src": "shelf", "dst": "table", "cam": "camera_top", "cell": "bay"}
    world = load_world(Path(branch_world(tmp_path)).read_bytes(), "w.json", task, bindings)
    assert {f.code for f in check_task(task, world)} == {"S231"}
    result = simulate_task(task, world)  # the region is clear: path 7.then
    assert result.state == "FAILED" and result.failure.code == "S231"
