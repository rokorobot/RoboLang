"""Phase 2 checks E1-E7 of the G1 executor (decision 0046 section 4, item 2).

    python -m robolang_g1.phase2 --out phase2.json [--run-dir DIR] [--only E1,E5] [--stance free|supported]

Each scenario starts its own stand-in (oracle on, test hooks on) and runs `robolang_g1.cli` against it as a separate
process, exactly as a user would. The **checks** read the oracle (MuJoCo ground truth written by the stand-in) to judge
the physical effect; the executor never does. Exit status 0 only if every check passes. For every stand-in run the
`realtime_valid` flag and `step_compute_ms` are reported (not pass or fail).

E1 move: three stations in turn, twice, and a covered marker (S517). E2 grasp: lifted and welded, twice; an absent box
(S506 before the hand closes); a box out of reach (S502 before any arm motion). E3 place: in the region, released, hand
retracted. E4 the two reference tasks, unchanged, through the CLI: since phase 3 they end `SUCCEEDED` (exit 0) from the
observations, not `STEPS_COMPLETED`; E1, E2, E3 and E7 ask the same of their short tasks (every step ran and the goal
is true). E5 stop: cancel mid-walk, revoke mid-reach, a
deadline while holding, and a push (S516). E6 refusals. E7 the free stance during arm work.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

from . import SIM_API_VERSION, SIM_SERVICE, assets
from . import site as site_mod
from .harness import factory_initialized, in_frame, yaw_of  # noqa: F401
from .run_task import Session

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
SITES = HERE / "sites"
BENCH = REPO / "bench" / "reference"
STATION_TOL_M, STATION_TOL_RAD = 0.05, 0.10  # E1
LIFT_MIN_M = 0.05  # E2
REGION_SLACK = 0.0  # E3: the object's centre inside the region, no slack
STAY_M = 0.03  # E7
STOP_WITHIN_MS = 3000  # E5: upright and settled within this of the stop
SETTLED_PATH_M, UPRIGHT_DEG = 0.015, 15.0
PUSH_N, PUSH_S = (0.0, 500.0, 0.0), 0.4  # E5: the force on the torso (world frame) and how long
BIND_PP = {"r": "robot_1", "box": "box_1", "src": "loc_a", "dst": "loc_b"}  # e_pick_place.rob
BIND_CH = {"r": "robot_1", "crate": "crate_1", "dock": "loc_dock", "dst": "loc_dst"}  # w_carry_hold.rob

TOUR = """robolang 1.3;
task tour(robot r, location a, location b, location c) {
    goal { robot_at r c; }
    plan { move r to a; move r to b; move r to c; }
}
"""
GO = """robolang 1.3;
task go(robot r, location a) {
    goal { robot_at r a; }
    plan { move r to a; }
}
"""
GRAB = """robolang 1.3;
task grab(robot r, object o, location l) {
    goal { holding r o; }
    plan { move r to l; grasp r o; }
}
"""
GRAB_AND_WAIT = """robolang 1.3;
task grab_and_wait(robot r, object o, location l) {
    goal { holding r o; }
    plan { move r to l; grasp r o; wait for 300 s; }
}
"""
REFUSALS = {
    "S508": (
        "robolang 1.3;\ntask t(robot r, object o, location a, sensor cam) {\n    goal { robot_at r a; }\n"
        "    plan { move r to a; inspect r o using cam; }\n}\n",
        "t",
        {"r": "robot_1", "o": "box_1", "a": "loc_a", "cam": "cam_1"},
    ),
    "S509": (
        "robolang 1.6;\ntask t(robot r, object o, location a, sensor scale) {\n    goal { robot_at r a; }\n"
        "    plan { measure r mass of o using scale as m; move r to a; }\n}\n",
        "t",
        {"r": "robot_1", "o": "box_1", "a": "loc_a", "scale": "scale_1"},
    ),
    "S510": (
        "robolang 1.9;\ntask t(robot r, location a) {\n    goal { robot_at r a; }\n"
        "    plan { measure r time as now; move r to a; }\n}\n",
        "t",
        {"r": "robot_1", "a": "loc_a"},
    ),
    "S514": (
        "robolang 2.0;\ntask patrol(robot r, location a, location b) {\n    goal { robot_at r a; }\n"
        "    plan { move r to b; move r to a; }\n}\nmission rounds(robot r, location a, location b) {\n"
        "    repeat at most 2 runs {\n        run patrol(r, a, b);\n    };\n    on failure { continue; }\n}\n",
        "rounds",
        {"r": "robot_1", "a": "loc_a", "b": "loc_b"},
    ),
    "S515": (
        'robolang 2.1;\ntask t(robot r, location a) {\n    pre { message_received "ops" with "GO"; }\n'
        "    goal { robot_at r a; }\n    plan { move r to a; }\n}\n",
        "t",
        {"r": "robot_1", "a": "loc_a"},
    ),
}


# ------------------------------------------------------------------------------------------------ oracle helpers


def pelvis_xy(sample) -> np.ndarray:
    return np.array(sample["pelvis"]["pos"][:2])


def tilt_deg(sample) -> float:
    w, x, y, z = sample["pelvis"]["quat"]
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def station_error(sample, station: dict) -> tuple[float, float]:
    gx, gy, gyaw = station["pose"]
    x, y = pelvis_xy(sample)
    return float(math.hypot(x - gx, y - gy)), float(abs(site_mod.wrap(yaw_of(sample["pelvis"]["quat"]) - gyaw)))


def in_region(point, site: dict, location: str) -> bool:
    """Whether a world point is inside the location's region (a box in its station's marker frame)."""
    loc = site_mod.location(site, location)
    R, p = site_mod.marker_frame(site_mod.station(site, loc["station"])["marker"])
    q = R.T @ (np.asarray(point, float) - p)
    return bool(
        np.all(q >= np.array(loc["region"]["min"]) - REGION_SLACK)
        and np.all(q <= np.array(loc["region"]["max"]) + REGION_SLACK)
    )


def path_length(samples, key="pelvis") -> float:
    pts = np.array([s[key]["pos"] if isinstance(s[key], dict) else s[key] for s in samples])
    return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum()) if len(pts) > 1 else 0.0


def stop_agrees(record: dict, tail, expect_code: str) -> dict:
    """E5/E1: the recorded stop is for the expected code, the stop was confirmed within 3 s, and the oracle agrees that
    the robot is upright and still (not fallen, tilt and pelvis path over the confirmation window small, velocity 0)."""
    stop = record.get("stop") or {}
    c = stop.get("confirmation") or {}
    out = {
        "code": (record.get("failure") or {}).get("code"),
        "confirmed": bool(c.get("confirmed")),
        "after_ms": c.get("after_ms"),
        "recorded": {
            k: c.get(k) for k in ("max_gyro_rad_s", "max_joint_velocity_rad_s", "max_tilt_deg", "window_start_ms")
        },
    }
    ok = out["code"] == expect_code and out["confirmed"] and (c.get("after_ms") or 1e9) <= STOP_WITHIN_MS
    if out["confirmed"]:
        w = tail.between(c["window_start_ms"] / 1000, c["window_end_ms"] / 1000)
        last = tail.nearest(c["window_end_ms"] / 1000)
        out["oracle"] = {
            "pelvis_path_m": round(path_length(w), 4) if w else None,
            "max_tilt_deg": round(max(tilt_deg(s) for s in w), 2) if w else None,
            "fallen": bool(last["fallen"]),
            "velocity_command": last["velocity_command"],
            "wrist_path_m": {
                side: round(
                    path_length(w, "wrists")
                    if False
                    else float(np.linalg.norm(np.array(w[-1]["wrists"][side]) - np.array(w[0]["wrists"][side]))),
                    4,
                )
                for side in ("left", "right")
            }
            if w
            else None,
        }
        o = out["oracle"]
        ok = (
            ok
            and bool(w)
            and o["pelvis_path_m"] <= SETTLED_PATH_M
            and o["max_tilt_deg"] <= UPRIGHT_DEG
            and not o["fallen"]
            and o["velocity_command"] == [0.0, 0.0, 0.0]
        )
    out["pass"] = bool(ok)
    return out


def realtime_of(session: Session) -> dict:
    f = session.final or {}
    worst, stalls = f.get("worst_second_ratio"), f.get("stalls")
    return {
        "realtime_ratio": f.get("realtime_ratio"),
        "worst_second_ratio": worst,
        "stalls": stalls,
        "realtime_valid": bool(worst is not None and worst >= 0.95 and stalls == 0),
        "step_compute_ms": f.get("step_compute_ms"),
        "gc": (f.get("gc") or {}).get("max_ms"),
        "slow_steps": (f.get("gc") or {}).get("slow_steps"),
        # (sim time, ratio, loop thread CPU s, loop thread run-queue wait s) of every second below 0.95: a loop that
        # waited for a CPU, against one that was computing (docs/evidence/g1-phase4-prereqs.md)
        "slow_windows": f.get("slow_windows"),
        # the head camera's rate (the spec is 5 Hz): rendering in software is the stand-in's biggest use of CPU, and a
        # host that is short of it keeps the physics loop in time and lets the camera fall to 2.5-3 Hz
        "camera": f.get("camera"),
    }


# ------------------------------------------------------------------------------------------------ context


class Ctx:
    def __init__(self, run_dir: Path, stance: str, standin_args: list[str] | None = None):
        self.dir, self.stance, self.runs, self.stay = run_dir, stance, [], []
        self.standin_args = standin_args or []  # extra stand-in arguments for every session (--imu-bias, development)
        self.sites = {n: site_mod.load_site(SITES / f"{n}.json") for n in ("pick_place", "carry_hold")}
        self.n = 0

    def source(self, name: str, text: str) -> Path:
        p = self.dir / f"{name}.rob"
        p.write_text(text)
        return p

    def variant(self, base: str, edit) -> Path:
        """A scene file made from a site by `edit(dict)`: the executor is given the original, the stand-in this."""
        raw = json.loads((SITES / f"{base}.json").read_text())
        edit(raw)
        p = self.dir / f"scene_{self.n}.json"
        self.n += 1
        p.write_text(json.dumps(raw))
        return p

    def session(self, name: str, scene: Path | str) -> Session:
        return Session(scene, self.dir / name, self.stance, standin_args=self.standin_args)

    def done(self, name: str, session: Session, record: dict | None, extra: dict | None = None) -> None:
        self.runs.append({
            "scenario": name,
            **realtime_of(session),
            "failure": record and record.get("failure"),
            "record": str((self.dir / name / "record.json").relative_to(self.dir)),
            **(extra or {}),
        })


def run_scenario(ctx: Ctx, name: str, scene, site_name: str, source: Path, task: str, bind: dict, **kw):
    """One scenario: a fresh stand-in on `scene`, the CLI on the executor's site; returns what a check needs."""
    site_path = site_name if isinstance(site_name, Path) else SITES / f"{site_name}.json"
    out = ctx.dir / name / "record.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    with ctx.session(name, scene) as s:
        status, record, log = s.run(source, task, site_path, bind, out, **kw)
        samples = list(s.tail.samples)
        commands = s.commands()
    ctx.done(name, s, record, {"exit": status})
    return status, record, log, s, samples, commands


def between(samples, t0_ms, t1_ms):
    return [x for x in samples if t0_ms / 1000 <= x["sim_time"] <= t1_ms / 1000]


class _Tail:
    """The oracle samples of a finished run, with the lookups the checks use (the live tail is stopped by then)."""

    def __init__(self, samples):
        self.samples = samples

    def between(self, t0, t1):
        return [s for s in self.samples if t0 <= s["sim_time"] <= t1]

    def nearest(self, t):
        return min(self.samples, key=lambda s: abs(s["sim_time"] - t))

    def latest(self):
        return self.samples[-1]


# ------------------------------------------------------------------------------------------------ the checks


def check_e1(ctx: Ctx) -> dict:
    site = ctx.sites["pick_place"]
    src = ctx.source("tour", TOUR)
    runs = []
    for k in range(2):
        status, rec, log, s, samples, _ = run_scenario(
            ctx,
            f"e1_tour_{k + 1}",
            SITES / "pick_place.json",
            "pick_place",
            src,
            "tour",
            {"r": "robot_1", "a": "loc_a", "b": "loc_b", "c": "loc_c"},
        )
        tail, errs = _Tail(samples), []
        for step, station in zip((rec or {}).get("steps", []), ("bench", "shelf", "pad")):
            e = station_error(tail.nearest(step["t_end_ms"] / 1000), site_mod.station(site, station))
            errs.append({
                "station": station,
                "status": step["status"],
                "error_m": round(e[0], 4),
                "error_rad": round(e[1], 4),
                "wall_s": step["wall_s"],
                "sim_s": round((step["t_end_ms"] - step["t_start_ms"]) / 1000, 1),
            })
        ok = (
            rec is not None
            and rec["state"] == "SUCCEEDED"
            and len(errs) == 3
            and all(
                e["status"] == "succeeded" and e["error_m"] <= STATION_TOL_M and e["error_rad"] <= STATION_TOL_RAD
                for e in errs
            )
        )
        runs.append({"pass": ok, "state": rec and rec["state"], "failure": rec and rec["failure"], "stations": errs})
    # a covered station marker: S517 and a confirmed stop
    covered = ctx.variant("pick_place", lambda d: d["stations"][0]["marker"].update(covered=True))
    status, rec, log, s, samples, _ = run_scenario(
        ctx, "e1_covered", covered, "pick_place", ctx.source("go", GO), "go", {"r": "robot_1", "a": "loc_a"}
    )
    stop = stop_agrees(rec, _Tail(samples), "S517") if rec and rec.get("stop") else {"pass": False, "error": "no stop"}
    return {
        "pass": all(r["pass"] for r in runs) and stop["pass"],
        "details": {
            "tour_runs": runs,
            "covered_marker": {
                "state": rec and rec["state"],
                "stop": stop,
                "message": rec and rec["failure"]["message"],
            },
        },
    }


def grasp_effect(rec: dict, samples: list, obj: str) -> dict:
    """What the oracle saw during the grasp step: how far the object rose, whether it is welded to the hand, how far the
    robot moved (E7)."""
    step = next(s for s in rec["steps"] if s["kind"] == "grasp")
    a0, a1 = step["details"][
        "arm_work_ms"
    ]  # the arms move from a0 to a1; before it the robot may be put back on station
    w = between(samples, a0, a1)
    z = [x["objects"][obj]["pos"][2] for x in w]
    p0 = pelvis_xy(w[0])
    last = w[-1]
    return {
        "lift_m": round(max(z) - z[0], 4),
        "attached": last["attached"],
        "hand": step.get("details", {}).get("hand"),
        "marker_at_hand_m": step.get("details", {}).get("marker_at_hand_m"),
        "robot_moved_m": round(max(float(np.linalg.norm(pelvis_xy(x) - p0)) for x in w), 4),
        "grasp_s": round((step["t_end_ms"] - step["t_start_ms"]) / 1000, 1),
    }


def check_e2(ctx: Ctx) -> dict:
    src = ctx.source("grab", GRAB)
    bind = {"r": "robot_1", "o": "box_1", "l": "loc_a"}
    runs = []
    for k in range(2):
        status, rec, log, s, samples, _ = run_scenario(
            ctx, f"e2_grasp_{k + 1}", SITES / "pick_place.json", "pick_place", src, "grab", bind
        )
        if rec and rec["state"] == "SUCCEEDED":
            fx = grasp_effect(rec, samples, "box")
            ctx.stay.append({"scenario": f"e2_grasp_{k + 1}", "step": "grasp", "robot_moved_m": fx["robot_moved_m"]})
            ok = (
                fx["lift_m"] >= LIFT_MIN_M
                and fx["attached"].get("box") == fx["hand"]
                and (fx["marker_at_hand_m"] or 1) <= 0.06
            )
            runs.append({"pass": ok, **fx})
        else:
            runs.append({"pass": False, "state": rec and rec["state"], "failure": rec and rec["failure"]})
    # the box is not there: S506 before the hand closes (no hand, arm or attach command reached the robot)
    absent = ctx.variant("pick_place", lambda d: (d["objects"].clear(), d["bindings"]["objects"].clear()))
    status, rec, log, s, samples, commands = run_scenario(ctx, "e2_absent", absent, "pick_place", src, "grab", bind)
    absent_ok = bool(
        rec
        and rec["failure"]
        and rec["failure"]["code"] == "S506"
        and rec["failure"]["step"] == 2
        and commands["hand_left"]["count"] == commands["hand_right"]["count"] == 0
        and commands["arm_sdk"]["count"] == commands["attach"]["count"] == 0
    )
    # the box is on the far side of a deeper table, out of reach: S502 before any arm motion

    def far(d):
        d["tables"][0].update(pos=[3.03, -0.065, 0.375], half_size=[0.35, 0.185, 0.375])
        d["objects"][0]["pos"] = [3.03, -0.10, 0.79]  # 0.58 m ahead of the pelvis: seen, and beyond the arm

    status, rec2, log, s, samples2, commands2 = run_scenario(
        ctx, "e2_far", ctx.variant("pick_place", far), "pick_place", src, "grab", bind
    )
    box0, box1 = samples2[0]["objects"]["box"]["pos"], samples2[-1]["objects"]["box"]["pos"]
    far_ok = bool(
        rec2
        and rec2["failure"]
        and rec2["failure"]["code"] == "S502"
        and rec2["failure"]["step"] == 2
        and commands2["arm_sdk"]["count"] == 0
        and commands2["hand_right"]["count"] == 0
        and float(np.linalg.norm(np.array(box1) - np.array(box0))) < 0.01
    )
    return {
        "pass": all(r["pass"] for r in runs) and absent_ok and far_ok,
        "details": {
            "grasp_runs": runs,
            "absent": {"pass": absent_ok, "failure": rec and rec["failure"], "commands": commands},
            "out_of_reach": {"pass": far_ok, "failure": rec2 and rec2["failure"], "commands": commands2},
        },
    }


def pick_place_effect(rec: dict, samples: list, site: dict) -> dict:
    last = samples[-1]
    box = last["objects"]["box"]["pos"]
    right = np.array(last["wrists"]["right"]) - np.array(last["pelvis"]["pos"])
    w, x, y, z = last["pelvis"]["quat"]
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
    wrist_pelvis = R.T @ right  # the wrist in the pelvis frame; its carry position is (0.002, -0.238, -0.056)
    return {
        "box_world": [round(v, 3) for v in box],
        "in_dst_region": in_region(box, site, "shelf"),
        "released": "box" not in last["attached"],
        "box_on_table": abs(box[2] - 0.79) < 0.02,
        "wrist_from_carry_m": round(float(np.linalg.norm(wrist_pelvis - np.array([0.002, -0.238, -0.056]))), 3),
    }


def check_e3_e4(ctx: Ctx) -> tuple[dict, dict]:
    """E3 and E4 share two runs: e_pick_place.rob and w_carry_hold.rob, unchanged, through the CLI."""
    site_pp, site_ch = ctx.sites["pick_place"], ctx.sites["carry_hold"]
    status, rec, log, s, samples, _ = run_scenario(
        ctx, "e4_pick_place", SITES / "pick_place.json", "pick_place", BENCH / "e_pick_place.rob", "pick_place", BIND_PP
    )
    e3 = {"pass": False, "details": {"state": rec and rec["state"], "failure": rec and rec["failure"]}}
    e4a = dict(e3)
    if rec and rec["state"] == "SUCCEEDED":
        fx = pick_place_effect(rec, samples, site_pp)
        place = next(x for x in rec["steps"] if x["kind"] == "place")
        w = between(samples, *place["details"]["arm_work_ms"])
        ctx.stay.append({
            "scenario": "e4_pick_place",
            "step": "grasp",
            **{k: v for k, v in grasp_effect(rec, samples, "box").items() if k == "robot_moved_m"},
        })
        ctx.stay.append({
            "scenario": "e4_pick_place",
            "step": "place",
            "robot_moved_m": round(max(float(np.linalg.norm(pelvis_xy(x) - pelvis_xy(w[0]))) for x in w), 4),
        })
        e3 = {
            "pass": bool(
                fx["in_dst_region"] and fx["released"] and fx["box_on_table"] and fx["wrist_from_carry_m"] < 0.08
            ),
            "details": fx,
        }
        gone = station_error(samples[-1], site_mod.station(site_pp, "shelf"))
        e4a = {
            "pass": bool(
                fx["in_dst_region"]
                and all(x["status"] == "succeeded" for x in rec["steps"])
                and len(rec["steps"]) == 4
                and rec["goal_evaluated"] is True
                and rec["state"] == "SUCCEEDED"
                and status == 0
            ),
            "details": {
                "steps": [
                    (x["kind"], x["status"], round((x["t_end_ms"] - x["t_start_ms"]) / 1000, 1)) for x in rec["steps"]
                ],
                "end_state": fx,
                "robot_at_dst_error": [round(gone[0], 4), round(gone[1], 4)],
                "exit": status,
                "record_state": rec["state"],
                "goal_evaluated": rec["goal_evaluated"],
            },
        }
    status, rec, log, s, samples, _ = run_scenario(
        ctx,
        "e4_carry_hold",
        SITES / "carry_hold.json",
        "carry_hold",
        BENCH / "w_carry_hold.rob",
        "carry_crate",
        BIND_CH,
    )
    e4b: dict = {"pass": False, "details": {"state": rec and rec["state"], "failure": rec and rec["failure"]}}
    if rec and rec["state"] == "SUCCEEDED":
        last = samples[-1]
        err = station_error(last, site_mod.station(site_ch, "dst"))
        grasp = grasp_effect(rec, samples, "crate")
        ctx.stay.append({"scenario": "e4_carry_hold", "step": "grasp", "robot_moved_m": grasp["robot_moved_m"]})
        crate = np.array(last["objects"]["crate"]["pos"])
        right = np.array(last["wrists"]["right"])
        ok = (
            last["attached"].get("crate") == "right"
            and err[0] <= STATION_TOL_M
            and err[1] <= STATION_TOL_RAD
            and not last["fallen"]
            and float(np.linalg.norm(crate - right)) < 0.25
            and len(rec["steps"]) == 3
            and rec["goal_evaluated"] is True
            and rec["state"] == "SUCCEEDED"
            and status == 0
        )
        e4b = {
            "pass": bool(ok),
            "details": {
                "crate_attached_to": last["attached"],
                "robot_at_dst_error": [round(err[0], 4), round(err[1], 4)],
                "crate_to_wrist_m": round(float(np.linalg.norm(crate - right)), 3),
                "fallen": last["fallen"],
                "steps": [
                    (x["kind"], x["status"], round((x["t_end_ms"] - x["t_start_ms"]) / 1000, 1)) for x in rec["steps"]
                ],
                "exit": status,
                "record_state": rec["state"],
                "goal_evaluated": rec["goal_evaluated"],
                "grasp": grasp,
            },
        }
    e4 = {"pass": e4a["pass"] and e4b["pass"], "details": {"e_pick_place": e4a, "w_carry_hold": e4b}}
    return e3, e4


class _Push:
    """A client for the stand-in's test push (only answered by a stand-in started with --test-hooks)."""

    def __init__(self):
        from unitree_sdk2py.rpc.client import Client

        from . import API_PUSH

        class PushClient(Client):
            def __init__(self):
                super().__init__(SIM_SERVICE, False)

            def Init(self):
                self._SetApiVerson(SIM_API_VERSION)
                self._RegistApi(API_PUSH, 0)

            def push(self, force, duration, torque=(0.0, 0.0, 0.0)):
                body = {"force": list(force), "duration": duration, "torque": list(torque)}
                return self._Call(API_PUSH, json.dumps(body))

        self.client = PushClient()
        self.client.SetTimeout(2.0)
        self.client.Init()

    def push(self, force, duration, torque=(0.0, 0.0, 0.0)):
        return self.client.push(force, duration, torque)


def check_e5(ctx: Ctx) -> dict:
    out: dict = {}
    pp = SITES / "pick_place.json"
    go, grab = ctx.source("go", GO), ctx.source("grab", GRAB)
    bind_go, bind_grab = {"r": "robot_1", "a": "loc_a"}, {"r": "robot_1", "o": "box_1", "l": "loc_a"}

    # cancel while walking
    stop_file = ctx.dir / "e5_cancel.stop"
    stop_file.unlink(missing_ok=True)
    state: dict = {}

    def cancel_when_walking(proc):
        if (
            "t" not in state
            and session[0].tail.latest()["velocity_command"][0] > 0.3
            and session[0].tail.latest()["sim_time"] > 6
        ):
            state["t"] = session[0].tail.latest()["sim_time"]
        if "t" in state and session[0].tail.latest()["sim_time"] > state["t"] + 1.0 and not stop_file.exists():
            stop_file.write_text("cancel")
            state["written"] = session[0].tail.latest()["sim_time"]
            state["walking_at_cancel"] = session[0].tail.latest()["velocity_command"]

    session: list = [None]
    out["cancel_mid_walk"] = _e5_run(
        ctx, "e5_cancel", pp, go, "go", bind_go, "S511", session, state, watch=cancel_when_walking, stop_file=stop_file
    )
    # revoke while the arm is reaching
    stop_file = ctx.dir / "e5_revoke.stop"
    stop_file.unlink(missing_ok=True)
    state = {}

    def revoke_mid_reach(proc):
        t = session[0].tail.latest()
        if "t" not in state and t["commands"]["arm_sdk"]["count"] > 0:
            state["t"] = t["sim_time"]
        if "t" in state and t["sim_time"] > state["t"] + 1.2 and not stop_file.exists():
            stop_file.write_text("revoke")
            state["written"] = t["sim_time"]

    out["revoke_mid_reach"] = _e5_run(
        ctx,
        "e5_revoke",
        pp,
        grab,
        "grab",
        bind_grab,
        "S513",
        session,
        state,
        watch=revoke_mid_reach,
        stop_file=stop_file,
        hands_untouched=True,
    )
    # a deadline that passes while the robot holds the box (after the grasp, during a wait)
    state = {}
    out["deadline_holding"] = _e5_run(
        ctx,
        "e5_deadline",
        pp,
        ctx.source("hold", GRAB_AND_WAIT),
        "grab_and_wait",
        bind_grab,
        "S512",
        session,
        state,
        deadline_s=130,
        holding="box",
    )
    # a push while walking: S516, and no command after the fall
    pusher: list = []
    state = {}

    def push_when_walking(proc):
        t = session[0].tail.latest()
        if "t" not in state and t["velocity_command"][0] > 0.3 and t["sim_time"] > 6:
            state["t"] = t["sim_time"]
        if "t" in state and t["sim_time"] > state["t"] + 1.5 and "pushed" not in state:
            if not pusher:
                pusher.append(_Push())
            state["pushed"] = t["sim_time"]
            state["push_reply"] = pusher[0].push(PUSH_N, PUSH_S)[0]

    out["push_falls"] = _e5_run(
        ctx, "e5_push", pp, go, "go", bind_go, "S516", session, state, watch=push_when_walking, fall=True
    )
    return {"pass": all(v["pass"] for v in out.values()), "details": out}


def _e5_run(
    ctx: Ctx,
    name: str,
    scene,
    source: Path,
    task: str,
    bind: dict,
    code: str,
    session_box: list,
    state: dict,
    *,
    watch=None,
    stop_file: Path | None = None,
    deadline_s: float | None = None,
    hands_untouched: bool = False,
    holding: str | None = None,
    fall: bool = False,
) -> dict:
    site_path = SITES / "pick_place.json"
    out = ctx.dir / name / "record.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    with ctx.session(name, scene) as s:
        session_box[0] = s
        status, rec, log = s.run(
            source, task, site_path, bind, out, watch=watch, stop_file=stop_file, deadline_s=deadline_s
        )
        time.sleep(0.5)
        samples, commands = list(s.tail.samples), s.commands()
    ctx.done(name, s, rec, {"exit": status})
    tail = _Tail(samples)
    result: dict = {
        "exit": status,
        "state": rec and rec["state"],
        "failure": rec and rec["failure"],
        "trigger": dict(state),
    }
    if rec is None or not rec.get("failure"):
        result["pass"] = False
        return result
    final = samples[-1]
    if fall:
        fell = final["fallen"]
        stop = rec.get("stop") or {}
        last_cmd = {k: v["last_sim_time"] for k, v in commands.items()}
        after = {
            k: (v is not None and fell and final["fell_at"] is not None and v > final["fell_at"])
            for k, v in last_cmd.items()
        }
        result.update({
            "fallen_per_oracle": fell,
            "fell_at": final["fell_at"],
            "last_command_sim_time": last_cmd,
            "command_after_fall": after,
            "stop_kind": stop.get("kind"),
            "tilt_at_detection": rec["failure"]["message"],
        })
        result["pass"] = bool(
            rec["failure"]["code"] == code
            and fell
            and not any(after.values())
            and stop.get("kind") == "no_further_command"
        )
        return result
    agree = stop_agrees(rec, tail, code)
    result["stop"] = agree
    result["pass"] = bool(agree["pass"])
    if hands_untouched:  # a reach stopped before the hand was commanded: nothing welded, no hand command
        result["hand_commands"] = [commands["hand_left"]["count"], commands["hand_right"]["count"]]
        result["pass"] = (
            result["pass"]
            and commands["hand_left"]["count"] == commands["hand_right"]["count"] == 0
            and not final["attached"]
        )
    if holding is not None:  # the object is still welded to the hand, and no hand command went out after the stop
        t_stop = (rec["stop"]["confirmation"] or {}).get("t_stop_ms", 0) / 1000
        before = tail.nearest(t_stop - 0.1)["commands"]
        result["held_object_attached"] = final["attached"]
        result["hand_commands_before_after_stop"] = [
            [before["hand_left"]["count"], before["hand_right"]["count"]],
            [commands["hand_left"]["count"], commands["hand_right"]["count"]],
        ]
        result["pass"] = bool(
            result["pass"]
            and holding in final["attached"]
            and before["hand_left"]["count"] == commands["hand_left"]["count"]
            and before["hand_right"]["count"] == commands["hand_right"]["count"]
        )
    return result


def check_e6(ctx: Ctx) -> dict:
    out = {}
    path = SITES / "pick_place.json"
    with ctx.session("e6_refusals", path) as s:
        for code, (text, task, bind) in REFUSALS.items():
            before = copy.deepcopy(s.commands())
            rec_path = ctx.dir / "e6_refusals" / f"{code}.json"
            status, rec, log = s.run(ctx.source(f"refuse_{code}", text), task, path, bind, rec_path, timeout=120)
            time.sleep(0.3)
            after = s.commands()
            sent = {k: after[k]["count"] - before[k]["count"] for k in after}
            got = (rec or {}).get("failure", {}) or {}
            out[code] = {
                "pass": bool(got.get("code") == code and not any(sent.values()) and status == 1),
                "exit": status,
                "code": got.get("code"),
                "commands_sent": sent,
                "message": got.get("message"),
            }
    ctx.done("e6_refusals", s, None)
    return {"pass": all(v["pass"] for v in out.values()), "details": out}


def free_stance_trial(ctx: Ctx) -> dict:
    """The same grasp with the executor told `arm_stance: free` (reported, not judged): how far the robot moves."""
    exec_site = ctx.variant("pick_place", lambda d: d["robot"].update(arm_stance="free"))
    status, rec, log, s, samples, _ = run_scenario(
        ctx,
        "e7_free_stance",
        SITES / "pick_place.json",
        exec_site,
        ctx.source("grab", GRAB),
        "grab",
        {"r": "robot_1", "o": "box_1", "l": "loc_a"},
    )
    out: dict = {"state": rec and rec["state"], "failure": rec and rec["failure"]}
    step = next((x for x in (rec or {}).get("steps", []) if x["kind"] == "grasp"), None)
    started = [e["t_ms"] for e in ((step or {}).get("details") or {}).get("events", []) if e.get("event") == "arm_work"]
    if step and started:  # a failed step keeps its events, so the window is known either way
        w = between(samples, started[-1], step["t_end_ms"])
        if w:
            out["robot_moved_m"] = round(max(float(np.linalg.norm(pelvis_xy(x) - pelvis_xy(w[0]))) for x in w), 4)
            out["pelvis_path_m"] = round(path_length(w), 4)
    return out


def check_e7(ctx: Ctx) -> dict:
    worst = max((x["robot_moved_m"] for x in ctx.stay), default=None)
    try:
        trial = free_stance_trial(ctx)
    except Exception as exc:
        trial = {"error": f"{type(exc).__name__}: {exc}"}
    return {
        "pass": bool(ctx.stay and worst <= STAY_M),
        "details": {
            "limit_m": STAY_M,
            "worst_m": worst,
            "stance_model": ctx.stance,
            "steps": ctx.stay,
            "free_stance_trial": trial,
            "note": "the robot's largest pelvis displacement from where it was when the step began, per grasp "
            "or place step (oracle); the free stance takes a recovery step of 0.2-0.35 m about every "
            "45 s, and a step that long would show here",
        },
    }


# ------------------------------------------------------------------------------------------------ move stress


def _sample_at(samples: list, t_s: float) -> dict:
    """The oracle sample nearest to simulation time `t_s`, searched from the end (the tail is live and append-only)."""
    i = len(samples) - 1
    while i > 0 and samples[i]["sim_time"] > t_s:
        i -= 1
    j = min(i + 1, len(samples) - 1)
    return samples[j] if abs(samples[j]["sim_time"] - t_s) < abs(samples[i]["sim_time"] - t_s) else samples[i]


def _oracle_pose(sample) -> list[float]:
    x, y = pelvis_xy(sample)
    return [round(float(x), 4), round(float(y), 4), round(float(yaw_of(sample["pelvis"]["quat"])), 4)]


def _event_counts(events: list) -> dict:
    counts: dict[str, int] = {}
    for e in events:
        counts[e["event"]] = counts.get(e["event"], 0) + 1
    return counts


def check_move_stress(ctx: Ctx, arrivals: int = 20, carry: bool = False, creep: float = 0.0) -> dict:
    """`move` between the bench station and the shelf station and back, `arrivals` arrivals at the shelf (and as many at
    the bench), in one stand-in and one executor (the free stance, as the checks run). Each arrival is judged by the
    oracle at the moment `move` returned: within the station tolerance (5 cm, 0.1 rad) and no failure. A move that
    fails is recorded (code and message) and the run goes on with the next one. Every `look` and every velocity command
    is also traced with the oracle's pose beside the belief, so the heading's behaviour can be read afterwards; the
    executor does not know of it (the trace wraps its methods from here).

        python -m robolang_g1.phase2 --only move-stress [--arrivals 20] [--carry] --out stress.json

    With `creep` (N m) the stand-in's test hook puts a steady torque about the vertical on the torso for the whole run:
    a heading creep added to the stance's own (about 0.01 rad/s), which the x86_64 CI runs showed more of.

    With `carry` the robot first grasps the box at the bench and holds it through every move (the arm in its carry
    pose, as in `e_pick_place`'s move to the shelf, which is where the x86_64 CI runs failed).
    """
    from .executor import BackendFailure
    from .ops import G1Operations

    site = ctx.sites["pick_place"]
    moves: list[dict] = []
    trace: list[dict] = []
    name = "move_stress" + ("_carry" if carry else "") + (f"_creep{creep:g}" if creep else "")
    with ctx.session(name, SITES / "pick_place.json") as s:
        samples = s.tail.samples
        ops = G1Operations(site)
        ops.guard(site["name"])
        if creep:
            _Push().push((0.0, 0.0, 0.0), 1e6, (0.0, 0.0, creep))
        look, command = ops.look, ops.command

        def traced_look(*a, **kw):
            before = getattr(ops, "fix", None)
            readings = look(*a, **kw)
            t = ops.now_ms()
            trace.append({"k": "look", "t": t, "belief": [round(float(v), 4) for v in ops.pose],
                          "fix": getattr(ops, "fix", None) is not before,
                          "oracle": _oracle_pose(_sample_at(samples, t / 1000))})
            return readings

        def traced_command(vx, vy, wz, seconds, wait=True, shuffle=False):
            t = ops.now_ms()
            trace.append({"k": "cmd", "t": t, "cmd": [round(vx, 3), round(vy, 3), round(wz, 3), round(seconds, 3)],
                          "shuffle": shuffle, "oracle": _oracle_pose(_sample_at(samples, t / 1000))})
            return command(vx, vy, wz, seconds, wait, shuffle)

        ops.look, ops.command = traced_look, traced_command

        def one(station: str, n: int) -> None:
            t0, wall0, first = ops.now_ms(), time.monotonic(), len(trace)
            failure, details = None, None
            try:
                details = ops.move(station)
            except BackendFailure as exc:
                failure = {"code": exc.code, "message": exc.message}
            t1 = ops.now_ms()
            err = station_error(_sample_at(samples, t1 / 1000), site_mod.station(site, station))
            ok = failure is None and err[0] <= STATION_TOL_M and err[1] <= STATION_TOL_RAD
            moves.append({
                "n": n, "station": station, "pass": ok, "failure": failure,
                "oracle_error_m": round(err[0], 4), "oracle_error_rad": round(err[1], 4),
                "sim_s": round((t1 - t0) / 1000, 1), "wall_s": round(time.monotonic() - wall0, 1),
                "servo_looks": len((details or {}).get("iterations", [])),
                "events": _event_counts(ops.events),
                # what the heading tracker has learned so far (the executor's own numbers) beside the IMU's true bias
                # and yaw error (the oracle's: the executor cannot see them)
                "heading": {**ops.tracker.summary(), "creep_rad_s": round(ops.creep, 4),
                            "creep_imu_reads": ops.creep_reads},
                "imu_truth": {k: v for k, v in _sample_at(samples, t1 / 1000).get("imu", {}).items() if k != "seed"},
                "trace": [first, len(trace)],
            })
            m = moves[-1]
            print(f"  move {n:2d} {station:5s} {'ok  ' if ok else 'FAIL'} {m['sim_s']:6.1f} s  "
                  f"{m['oracle_error_m'] * 100:5.1f} cm  {m['oracle_error_rad']:.3f} rad  "
                  f"{failure['code'] if failure else ''}", flush=True)
            (ctx.dir / f"{name}_trace.json").write_text(json.dumps({"moves": moves, "trace": trace}))  # as it goes

        one("bench", 0)  # from the start pose: not counted
        grasp = None
        if carry:
            try:
                grasp = {"pass": True, "carry_pose": ops.grasp("box").get("carry_pose")}
            except BackendFailure as exc:
                grasp = {"pass": False, "failure": {"code": exc.code, "message": exc.message}}
                moves.append({"n": 0, "station": "grasp", "pass": False, "failure": grasp["failure"]})
                arrivals = 0
        for k in range(arrivals):
            one("shelf", 2 * k + 1)
            one("bench", 2 * k + 2)
        status = s.sim.Status()[1]
    ctx.done(name, s, None)
    (ctx.dir / f"{name}_trace.json").write_text(json.dumps({"moves": moves, "trace": trace}))
    shelf = [m for m in moves if m["station"] == "shelf"]
    bench = [m for m in moves[1:] if m["station"] == "bench"]

    def stats(group):
        if not group:
            return {"arrivals": 0, "pass": 0}
        times = sorted(m["sim_s"] for m in group)
        return {
            "arrivals": len(group),
            "pass": sum(m["pass"] for m in group),
            "sim_s": {"min": times[0], "median": times[len(times) // 2], "max": times[-1],
                      "mean": round(sum(times) / len(times), 1)},
            "worst_error_m": max(m["oracle_error_m"] for m in group),
            "worst_error_rad": max(m["oracle_error_rad"] for m in group),
        }

    return {
        "pass": bool(shelf) and len(shelf) == arrivals and all(m["pass"] for m in shelf)
        and all(m["pass"] for m in bench) and (grasp is None or grasp["pass"]),
        "details": {
            "protocol": f"bench, then {arrivals} x (shelf, bench): one stand-in, free stance, the oracle judges each"
            + (", the box held throughout" if carry else ""),
            "grasp": grasp,
            "creep_torque_nm": creep,
            "tolerance": {"m": STATION_TOL_M, "rad": STATION_TOL_RAD},
            "shelf": stats(shelf), "bench": stats(bench), "moves": moves,
            "realtime_ratio": status.get("ratio") if isinstance(status, dict) else None,
        },
    }


# ------------------------------------------------------------------------------------------------ main


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="RoboLang G1 phase 2 checks E1-E7")
    ap.add_argument("--out", default="phase2.json")
    ap.add_argument("--run-dir")
    ap.add_argument("--stance", default="free", choices=["free", "supported"])
    ap.add_argument("--only", default="", help="development: comma list of E1,E2,E34,E5,E6 to run (E7 follows E2/E34); "
                    "`move-stress` runs the shelf-move stress check (never part of a full run)")
    ap.add_argument("--arrivals", type=int, default=20, help="move-stress: arrivals at the shelf")
    ap.add_argument("--carry", action="store_true", help="move-stress: hold the box through every move")
    ap.add_argument("--creep", type=float, default=0.0, help="move-stress: a steady torque (N m) about the vertical")
    ap.add_argument("--imu-bias", type=float, help="the stand-in IMU's gyro z bias in rad/s (default: drawn per start, "
                    "sim/imu.py): a worst case for move-stress")
    ap.add_argument("--standin-arg", action="append", default=[], metavar="ARG",
                    help="development: an extra argument for every stand-in (e.g. --standin-arg=--ideal-imu)")
    args = ap.parse_args(argv)
    out = Path(args.out)
    ctx = Ctx(Path(args.run_dir) if args.run_dir else out.with_suffix(".run"), args.stance,
              (["--imu-bias", str(args.imu_bias)] if args.imu_bias is not None else []) + args.standin_arg)
    ctx.dir.mkdir(parents=True, exist_ok=True)
    only = set(filter(None, args.only.split(",")))
    result: dict = {
        "checks": {},
        "stance_model": args.stance,
        "versions": assets.versions(),
        "arch": os.uname().machine,
    }
    t0 = time.perf_counter()

    def record(name, fn):
        started = time.perf_counter()
        try:
            r = fn()
        except Exception as exc:  # a failed check must not stop the report
            import traceback

            r = {"pass": False, "details": {"error": f"{type(exc).__name__}: {exc}", "trace": traceback.format_exc()}}
        r["elapsed_s"] = round(time.perf_counter() - started, 1)
        result["checks"][name] = r
        print(f"{name}: {'pass' if r['pass'] else 'FAIL'}  ({r['elapsed_s']} s)", flush=True)
        out.write_text(json.dumps({**result, "runs": ctx.runs}, indent=2, default=str))

    def wanted(*names):
        return not only or bool(only & set(names))

    if wanted("E1"):
        record("E1", lambda: check_e1(ctx))
    if wanted("E2"):
        record("E2", lambda: check_e2(ctx))
    if wanted("E3", "E4", "E34"):
        pair: list = []

        def e34():
            pair.extend(check_e3_e4(ctx))
            return pair[0]

        record("E3", e34)
        if pair:
            result["checks"]["E4"] = pair[1]
            print(f"E4: {'pass' if pair[1]['pass'] else 'FAIL'}", flush=True)
        else:
            result["checks"]["E4"] = {"pass": False, "details": {"error": "E3's run did not finish"}}
    if wanted("E5"):
        record("E5", lambda: check_e5(ctx))
    if wanted("E6"):
        record("E6", lambda: check_e6(ctx))
    if "move-stress" in only:
        record("move-stress", lambda: check_move_stress(ctx, args.arrivals, args.carry, args.creep))
    if ctx.stay or not only:
        record("E7", lambda: check_e7(ctx))
    for name in ("E1", "E2", "E3", "E4", "E5", "E6", "E7"):
        if not only:
            result["checks"].setdefault(name, {"pass": False, "details": {"error": "not run"}})
    result["go"] = all(c["pass"] for c in result["checks"].values())
    result["realtime_valid_all_runs"] = all(r["realtime_valid"] for r in ctx.runs)
    result["elapsed_s"] = round(time.perf_counter() - t0, 1)
    out.write_text(json.dumps({**result, "runs": ctx.runs}, indent=2, default=str))
    print(f"go: {result['go']}  (realtime_valid in every run: {result['realtime_valid_all_runs']}); wrote {out}")
    return 0 if result["go"] else 1


if __name__ == "__main__":
    sys.exit(main())
