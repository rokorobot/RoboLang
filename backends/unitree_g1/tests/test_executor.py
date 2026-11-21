"""The executor with fake operations: refusals before any motion, the stop, a record that never says SUCCEEDED."""

import json
from pathlib import Path

import pytest

from robolang import ir
from robolang_g1 import executor, site as S
from robolang_g1.executor import BackendFailure, Stop, execute

SITES = Path(__file__).resolve().parents[1] / "robolang_g1" / "sites"
SITE = S.load_site(SITES / "pick_place.json")
BIND = {"r": "robot_1", "a": "loc_a"}


def task_of(source: str, name: str = "t"):
    return ir.compile_source(source, "t.rob").ir.task(name)


class FakeOps:
    """What `G1Operations` offers the executor, recording what was asked. `fail` maps an operation to a failure."""

    def __init__(self, fail=None, held_fall=False):
        self.calls, self.fail, self.pose, self.events, self.observations = [], fail or {}, [0.0, 0.0, 0.0], [], []
        self.identity = {"stance_model": "free"}
        self.t = 0
        self.perc = None

    def guard(self, model=None):
        self.calls.append("guard")
        if "guard" in self.fail:
            raise self.fail["guard"]

    def now_ms(self):
        return self.t

    def versions(self):
        return {"arch": "test"}

    def status(self):
        return {
            "sim_time": self.t / 1000,
            "wall_elapsed": self.t / 1000,
            "slow_windows": [],
            "stalls": 0,
            "worst_second_ratio": 1.0,
            "step_compute_ms": {"max": 1.0},
            "gc": None,
            "camera": None,
        }

    def check(self):
        self.calls.append("check")

    def _do(self, name, *args):
        self.calls.append((name, *args))
        self.t += 1000
        if name in self.fail:
            raise self.fail[name]
        return {"did": name}

    def move(self, station):
        return self._do("move", station)

    def grasp(self, obj):
        return self._do("grasp", obj)

    def place(self, obj, loc):
        return self._do("place", obj, loc)

    def wait_ms(self, d):
        return self._do("wait", d)

    def hold(self):
        self.calls.append("hold")
        return {"velocity": "0", "arms": "held", "hands": "unchanged"}

    def confirm_stopped(self):
        self.calls.append("confirm")
        return {"confirmed": True, "after_ms": 1500}


GO = (
    "robolang 1.3;\ntask t(robot r, location a) {\n  goal { robot_at r a; }\n  plan { move r to a; wait for 2 s; }\n}\n"
)


def moves(ops):
    return [c for c in ops.calls if isinstance(c, tuple)]


def test_a_run_whose_steps_all_ran_is_steps_completed_and_never_succeeded():
    ops = FakeOps()
    rec = execute(task_of(GO), BIND, SITE, ops)
    assert rec["state"] == "STEPS_COMPLETED" and rec["failure"] is None and rec["stop"] is None
    assert rec["goal_evaluated"] is False and rec["core_outcome"] is None
    assert [g["result"] for g in rec["goal"]] == ["not_evaluated"]
    assert [s["status"] for s in rec["steps"]] == ["succeeded", "succeeded"]
    assert rec["backend"]["kind"] == "unitree-g1-mujoco" and rec["backend"]["grasp_model"] == "weld_after_hand_close"
    assert rec["backend"]["stance_model"] == "free" and rec["realtime"]["realtime_valid"] is True
    assert moves(ops) == [("move", "bench"), ("wait", 2000)]
    assert "SUCCEEDED" not in json.dumps(rec)


def test_the_guard_runs_first_and_a_refused_guard_sends_nothing():
    ops = FakeOps(fail={"guard": BackendFailure("S501", "not a simulation")})
    rec = execute(task_of(GO), BIND, SITE, ops)
    assert rec["failure"]["code"] == "S501" and rec["steps"] == [] and rec["stop"] is None
    assert ops.calls == ["guard"]


@pytest.mark.parametrize(
    "code, source, bind",
    [
        (
            "S509",
            "robolang 1.6;\ntask t(robot r, object o, location a, sensor scale) {\n  goal { robot_at r a; }\n"
            "  plan { measure r mass of o using scale as m; move r to a; }\n}\n",
            {"r": "robot_1", "o": "box_1", "a": "loc_a", "scale": "s1"},
        ),
        (
            "S510",
            "robolang 1.9;\ntask t(robot r, location a) {\n  goal { robot_at r a; }\n"
            "  plan { measure r time as now; move r to a; }\n}\n",
            BIND,
        ),
        (
            "S515",
            'robolang 2.1;\ntask t(robot r, location a) {\n  pre { message_received "ops" with "GO"; }\n'
            "  goal { robot_at r a; }\n  plan { move r to a; }\n}\n",
            BIND,
        ),
        (
            "S508",
            "robolang 1.3;\ntask t(robot r, object o, location a, sensor cam) {\n  goal { robot_at r a; }\n"
            "  plan { move r to a; inspect r o using cam; }\n}\n",
            {"r": "robot_1", "o": "box_1", "a": "loc_a", "cam": "c"},
        ),
        (
            "S508",
            "robolang 1.3;\ntask t(robot r, location a) {\n  pre { robot_ready r; }\n  goal { robot_at r a; }\n"
            "  plan { move r to a; }\n}\n",
            BIND,
        ),
    ],
)
def test_refusals_come_before_any_step_with_the_ros2_backends_codes(code, source, bind):
    ops = FakeOps()
    rec = execute(task_of(source), bind, SITE, ops)
    assert rec["failure"]["code"] == code and rec["steps"] == [] and rec["stop"] is None
    assert moves(ops) == [] and "hold" not in ops.calls  # nothing moved, so nothing was held


def test_unbound_entities_and_a_foreign_robot_are_site_errors():
    for bind in ({"r": "robot_1", "a": "nowhere"}, {"r": "robot_9", "a": "loc_a"}, {"r": "robot_1"}):
        rec = execute(task_of(GO), bind, SITE, FakeOps())
        assert rec["failure"]["code"] == "S507" and rec["steps"] == []


def test_a_failed_step_puts_the_robot_in_the_biped_safe_state_and_records_it():
    ops = FakeOps(fail={"move": BackendFailure("S504", "too slow")})
    rec = execute(task_of(GO), BIND, SITE, ops)
    assert rec["state"] == "FAILED" and rec["failure"] == {"code": "S504", "message": "too slow", "step": 1}
    assert [s["status"] for s in rec["steps"]] == ["failed"]
    stop = rec["stop"]
    assert stop["kind"] == "controlled_stand" and stop["reason"] == "step_failed"
    assert stop["safe_state"] == {"velocity": "zero", "arms": "hold", "hands": "unchanged", "balance": "kept"}
    assert stop["hold"]["hands"] == "unchanged" and stop["confirmation"]["confirmed"] is True
    assert ops.calls[-2:] == ["hold", "confirm"]


@pytest.mark.parametrize("code, reason", [("S511", "cancelled"), ("S512", "expired"), ("S513", "revoked")])
def test_cancel_deadline_and_revoke_stop_the_same_way(code, reason):
    ops = FakeOps(fail={"move": BackendFailure(code, executor.STOP_MESSAGES[reason])})
    rec = execute(task_of(GO), BIND, SITE, ops)
    assert rec["stop"]["reason"] == reason and rec["stop"]["code"] == code and "hold" in ops.calls


def test_a_fall_sends_no_command_at_all_not_even_the_stop():
    ops = FakeOps(fail={"move": BackendFailure("S516", "tilt 52 degrees")})
    rec = execute(task_of(GO), BIND, SITE, ops)
    assert rec["failure"]["code"] == "S516"
    assert rec["stop"]["kind"] == "no_further_command" and rec["stop"]["hold"] == "none sent"
    assert "hold" not in ops.calls and "confirm" not in ops.calls


def test_the_stop_file_and_the_deadline(tmp_path):
    f = tmp_path / "stop"
    stop = Stop(stop_file=str(f), after_ms=5000)
    stop.start(1000)
    assert stop.due(1000) is None and stop.due(5999) is None
    assert stop.due(6000) == "expired"
    stop2 = Stop(stop_file=str(f))
    assert stop2.due(0) is None
    f.write_text("revoke")
    assert stop2.due(0) == "revoked"
    f.write_text("cancel")  # the first reason sticks
    assert stop2.due(0) == "revoked"
    stop3 = Stop(stop_file=str(f))
    assert stop3.due(0) == "cancelled"


def test_a_mission_is_refused_by_the_cli_before_the_guard(tmp_path):
    from robolang_g1 import cli

    rob = tmp_path / "m.rob"
    rob.write_text(
        "robolang 2.0;\ntask p(robot r, location a, location b) {\n  goal { robot_at r a; }\n"
        "  plan { move r to b; move r to a; }\n}\nmission rounds(robot r, location a, location b) {\n"
        "  repeat at most 2 runs { run p(r, a, b); };\n  on failure { continue; }\n}\n"
    )
    out = tmp_path / "rec.json"
    status = cli.main([
        str(rob),
        "--task",
        "rounds",
        "--site",
        str(SITES / "pick_place.json"),
        "--out",
        str(out),
        "--bind",
        "r=robot_1",
        "--bind",
        "a=loc_a",
        "--bind",
        "b=loc_b",
    ])
    assert status == 1 and json.loads(out.read_text())["failure"]["code"] == "S514"


def test_the_burst_tables_pick_the_step_that_cancels_the_error():
    from robolang_g1 import ops

    assert ops._burst("forward", 0.10) == (0.4, 0.0, 0.3)
    assert ops._burst("forward", 0.16)[2] == 0.4
    assert ops._burst("forward", -0.07) == (-0.4, 0.0, 0.2)
    vx, vy, seconds = ops._burst("lateral", -0.05)
    assert (vx, vy) == (0.1, -0.3) and seconds == 0.5
    assert ops._burst("lateral", 0.20)[1] == 0.3 and ops._burst("forward", 1.2)[2] == pytest.approx(4.0)
