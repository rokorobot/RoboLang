"""The executor with fake operations: refusals before any motion, preconditions and the goal from one snapshot each
(S101, S102, S301, S302, `SUCCEEDED`), the stop, and the record."""

import json
from pathlib import Path

import pytest
from test_observe import hand, object_reading, snap, station_reading

from robolang import ir
from robolang_g1 import executor, site as S
from robolang_g1.executor import BackendFailure, Stop, execute

SITES = Path(__file__).resolve().parents[1] / "robolang_g1" / "sites"
SITE = S.load_site(SITES / "pick_place.json")
BIND = {"r": "robot_1", "a": "loc_a"}


def task_of(source: str, name: str = "t"):
    return ir.compile_source(source, "t.rob").ir.task(name)


AT_BENCH = station_reading(S.station(SITE, "bench"), S.station(SITE, "bench")["pose"], stamp=0)


class FakeOps:
    """What `G1Operations` offers the executor, recording what was asked. `fail` maps an operation to a failure.
    `snaps` are the snapshots the sensors give, in turn (the last one repeats): by default the robot is at the bench."""

    def __init__(self, fail=None, held_fall=False, snaps=None):
        self.calls, self.fail, self.pose, self.events, self.observations = [], fail or {}, [0.0, 0.0, 0.0], [], []
        self.identity = {"stance_model": "free"}
        self.t = 0
        self.perc = None
        self.snaps = list(snaps) if snaps is not None else [snap([AT_BENCH])]

    def snapshot(self, show=frozenset(), ready=False):
        self.calls.append(("snapshot", sorted(show), ready))
        if "snapshot" in self.fail:
            raise self.fail["snapshot"]
        s = self.snaps.pop(0) if len(self.snaps) > 1 else self.snaps[0]
        return s, ({"observation_motion": {"performed": True, "object": sorted(show)[0]}} if show else {})

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


def test_a_run_whose_goal_is_true_from_the_observations_succeeds_and_says_how():
    ops = FakeOps()
    rec = execute(task_of(GO), BIND, SITE, ops)
    assert rec["state"] == "SUCCEEDED" and rec["failure"] is None and rec["stop"] is None
    assert rec["goal_evaluated"] is True and [g["result"] for g in rec["goal"]] == ["true"]
    assert [s["status"] for s in rec["steps"]] == ["succeeded", "succeeded"]
    assert rec["backend"]["kind"] == "unitree-g1-mujoco" and rec["backend"]["grasp_model"] == "weld_after_hand_close"
    assert rec["backend"]["stance_model"] == "free" and rec["realtime"]["realtime_valid"] is True
    assert rec["backend"]["arm_work_stance"] == SITE["robot"]["arm_stance"]  # the weld during arm work is on record
    assert rec["verify"] == {"min_confidence": "0.9", "max_age_ms": 10000}
    assert moves(ops) == [("move", "bench"), ("wait", 2000), ("snapshot", [], False)]  # one, after the last step
    (snapshot,) = rec["snapshots"]
    assert snapshot["purpose"] == "goal" and snapshot["markers"]["20"]["confidence"] == 1.0
    assert "age_ms" in snapshot["markers"]["20"] and snapshot["t_ms"] == 5000
    (atom,) = rec["goal"][0]["atoms"]
    assert atom["result"] == "true" and atom["offset_m"] == 0.0 and atom["tolerance_m"] == 0.05
    assert rec["core_outcome"]["outcome"]["status"] == "succeeded"
    assert rec["core_outcome"]["evidence"][0]["value"] == "rl:place:loc_a"


def test_a_goal_that_is_false_from_the_observations_is_s301_with_no_stop_and_no_step():
    far = station_reading(S.station(SITE, "bench"), (1.0, 0.0, 0.0), stamp=0)  # the robot is 1.45 m from the bench
    ops = FakeOps(snaps=[snap([far])])
    rec = execute(task_of(GO), BIND, SITE, ops)
    assert rec["state"] == "FAILED" and rec["failure"]["code"] == "S301" and rec["failure"]["step"] is None
    assert [g["result"] for g in rec["goal"]] == ["false"] and rec["stop"] is None and "hold" not in ops.calls
    assert rec["goal"][0]["atoms"][0]["offset_m"] == pytest.approx(1.45, abs=1e-3)
    assert rec["core_outcome"]["outcome"]["status"] == "failed" and rec["goal_evaluated"] is True


@pytest.mark.parametrize(
    "bad",
    [
        snap([]),
        snap([station_reading(S.station(SITE, "bench"), (2.45, 0, 0), conf=0.8, stamp=0)]),
        snap([station_reading(S.station(SITE, "bench"), (2.45, 0, 0), stamp=-20000)], t=2000),
    ],
    ids=["not seen", "low confidence", "stale"],
)
def test_a_goal_that_cannot_be_decided_is_s302_never_a_success(bad):
    ops = FakeOps(snaps=[bad])
    rec = execute(task_of(GO), BIND, SITE, ops)
    assert rec["state"] == "FAILED" and rec["failure"]["code"] == "S302" and rec["failure"]["step"] is None
    assert [g["result"] for g in rec["goal"]] == ["unknown"] and rec["stop"] is None
    core = rec["core_outcome"]
    assert core["outcome"]["status"] == "unknown" and core["evidence"] == []
    assert "SUCCEEDED" not in json.dumps(rec)


def test_a_goal_with_one_false_and_one_unknown_atom_is_false():
    two = task_of("robolang 1.3;\ntask t(robot r, location a, location b) {\n  goal { robot_at r a; robot_at r b; }\n"
                  "  plan { move r to a; }\n}\n")
    far = station_reading(S.station(SITE, "bench"), (1.0, 0.0, 0.0), stamp=0)
    rec = execute(two, {"r": "robot_1", "a": "loc_a", "b": "loc_b"}, SITE, FakeOps(snaps=[snap([far])]))
    assert [g["result"] for g in rec["goal"]] == ["false", "unknown"] and rec["failure"]["code"] == "S301"


PRE = (
    "robolang 1.3;\ntask t(robot r, object o, location a, location b) {\n  pre { object_at o a; }\n"
    "  goal { robot_at r b; }\n  plan { move r to b; }\n}\n"
)
PRE_BIND = {"r": "robot_1", "o": "box_1", "a": "loc_a", "b": "loc_b"}
BENCH_ST = S.station(SITE, "bench")
ON_BENCH = [AT_BENCH, object_reading(AT_BENCH, (0.55, 0.64, 1.0), stamp=0)]


def test_a_true_precondition_lets_the_run_start_and_the_snapshot_is_taken_before_any_command():
    at_shelf = station_reading(S.station(SITE, "shelf"), S.station(SITE, "shelf")["pose"], stamp=0)
    ops = FakeOps(snaps=[snap(ON_BENCH, t=0), snap([at_shelf])])
    rec = execute(task_of(PRE), PRE_BIND, SITE, ops)
    assert rec["state"] == "SUCCEEDED" and [p["result"] for p in rec["preconditions"]] == ["true"]
    assert [c[0] for c in moves(ops)] == ["snapshot", "move", "snapshot"]  # the precondition snapshot comes first
    assert [s["purpose"] for s in rec["snapshots"]] == ["preconditions", "goal"]


def test_a_false_precondition_is_s101_and_nothing_is_sent():
    elsewhere = object_reading(AT_BENCH, (0.55, 0.64, 1.4), stamp=0)
    ops = FakeOps(snaps=[snap([AT_BENCH, elsewhere])])
    rec = execute(task_of(PRE), PRE_BIND, SITE, ops)
    assert rec["state"] == "FAILED" and rec["failure"] == {"code": "S101", "message": rec["failure"]["message"],
                                                           "step": None}
    assert [p["result"] for p in rec["preconditions"]] == ["false"] and rec["steps"] == [] and rec["stop"] is None
    assert [c[0] for c in moves(ops)] == ["snapshot"] and "hold" not in ops.calls  # no step, no hold, no command
    assert rec["goal_evaluated"] is False and [g["result"] for g in rec["goal"]] == ["not_evaluated"]
    assert rec["preconditions"][0]["atoms"][0]["region_clearance_m"] < 0


def test_an_unknown_precondition_is_s102_and_nothing_is_sent():
    ops = FakeOps(snaps=[snap([])])  # the camera sees nothing
    rec = execute(task_of(PRE), PRE_BIND, SITE, ops)
    assert rec["failure"]["code"] == "S102" and rec["steps"] == [] and rec["stop"] is None
    assert [p["result"] for p in rec["preconditions"]] == ["unknown"] and [c[0] for c in moves(ops)] == ["snapshot"]


def test_the_goal_snapshot_asks_to_see_a_held_object_and_for_readiness_only_when_the_goal_needs_them():
    holding = "robolang 1.3;\ntask t(robot r, object o, location a) {\n  goal { holding r o; robot_ready r; }\n" \
              "  plan { move r to a; grasp r o; }\n}\n"
    marker = object_reading(AT_BENCH, (0.0, 0.0, 0.0), stamp=0)
    closed = hand("right", "closed", tuple(marker.position + [0, 0, -0.05]))
    from test_observe import ready

    ops = FakeOps(snaps=[snap([marker], [closed], ready(stamp_ms=0), t=0)])
    rec = execute(task_of(holding), {"r": "robot_1", "o": "box_1", "a": "loc_a"}, SITE, ops)
    assert ("snapshot", ["box"], True) in moves(ops) and rec["snapshots"][0]["purpose"] == "goal"
    assert rec["state"] == "SUCCEEDED" and rec["snapshots"][0]["observation_motion"] == 0
    (motion,) = rec["observation_motion"]  # recorded apart from the plan's steps, with the atoms that asked for it
    assert motion["phase"] == "goal" and motion["performed"] is True and motion["object"] == "box"
    assert [w["kind"] for w in motion["why"]] == ["holding"] and len(rec["steps"]) == 2
    assert rec["core_outcome"] is None  # robot_ready has no Core form
    ops2 = FakeOps()
    execute(task_of(GO), BIND, SITE, ops2)
    assert ("snapshot", [], False) in moves(ops2)


def test_the_goal_snapshot_reads_readiness_only_when_the_goal_needs_it():
    both = (
        "robolang 1.3;\ntask t(robot r, location a) {\n  goal { robot_at r a; robot_ready r; }\n"
        "  plan { move r to a; }\n}\n"
    )
    from test_observe import ready

    ops = FakeOps(snaps=[snap([AT_BENCH], [], ready(stamp_ms=0), t=0)])
    rec = execute(task_of(both), BIND, SITE, ops)
    assert ("snapshot", [], True) in moves(ops) and rec["state"] == "SUCCEEDED"
    assert rec["core_outcome"] is None and rec["observation_motion"] == []  # robot_ready has no Core form; no motion


def test_a_failure_while_showing_the_held_object_stops_the_robot_and_is_not_a_verdict():
    ops = FakeOps(fail={"snapshot": BackendFailure("S503", "arm joint 23 is off")})
    rec = execute(task_of(GO), BIND, SITE, ops)
    assert rec["failure"]["code"] == "S503" and rec["failure"]["step"] == 2 and rec["stop"]["reason"] == "step_failed"
    assert ops.calls[-2:] == ["hold", "confirm"] and rec["goal_evaluated"] is False


def test_a_task_without_a_goal_is_refused():
    source = "robolang 1.2;\ntask t(robot r, location a) { plan { move r to a; } }\n"
    rec = execute(task_of(source), BIND, SITE, FakeOps())
    assert rec["failure"]["code"] == "S508" and rec["steps"] == []


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
            "robolang 1.3;\ntask t(robot r, location a, sensor s) {\n  pre { sensor_ok s; }\n  goal { robot_at r a; }\n"
            "  plan { move r to a; }\n}\n",
            {"r": "robot_1", "a": "loc_a", "s": "s1"},
        ),
        (
            "S508",
            "robolang 1.3;\ntask t(robot r, location a, sensor s) {\n  goal { robot_at r a and sensor_ok s; }\n"
            "  plan { move r to a; }\n}\n",
            {"r": "robot_1", "a": "loc_a", "s": "s1"},
        ),
        (
            "S507",
            "robolang 1.3;\ntask t(robot r, object o, location a) {\n  pre { object_at o a; }\n"
            "  goal { robot_at r a; }\n  plan { move r to a; }\n}\n",
            {"r": "robot_1", "o": "box_1", "a": "loc_c"},  # loc_c is the pad: a station without a region
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
    assert ops._burst("lateral", 0.20)[1] == 0.3
    assert ops._burst("forward", 1.2)[2] == pytest.approx(1.2 / ops.FORWARD_WALK_M_S)
    assert ops._burst("forward", 2.0)[2] == pytest.approx(4.0)  # a walk is never longer than this
