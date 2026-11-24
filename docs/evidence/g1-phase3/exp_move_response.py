# ruff: noqa
"""Experiment for the shelf-move fix (reads the oracle; not the backend): what a burst of velocity command does, and what the
stance does by itself.

    MUJOCO_GL=egl python docs/evidence/g1-phase3/exp_move_response.py OUTDIR [--carry] [--repeats 3] [--rest 3] [--set all|yaw]

One stand-in, free stance. The robot stands in the open (near the start pose), settles, and for each command in the table
(the table has both signs of every lateral and yaw command; the robot is walked back to its home point between trials)
the script sends one `SetVelocity` burst and
records the oracle's pelvis pose (in the robot's own frame at the start of the burst: forward, left, heading change) at
the end of the command and 1, 2, 3, 5 and 8 s after it, and when the stance reports itself settled (the executor's own
`settle`). With `--carry` the robot first grasps the box at the bench and carries it through all of it. `--rest N` adds N
trials of 20 s with no command: the heading and position drift of the stance. Output: OUTDIR/response.json (every trial)
and a table on stdout.
"""
import argparse, json, math, sys, time
from pathlib import Path

import numpy as np

from robolang_g1 import site as S
from robolang_g1.executor import BackendFailure
from robolang_g1.harness import yaw_of
from robolang_g1.ops import G1Operations
from robolang_g1.phase2 import SITES, _sample_at
from robolang_g1.run_task import Session

ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--carry", action="store_true")
ap.add_argument("--repeats", type=int, default=3)
ap.add_argument("--rest", type=int, default=3)
ap.add_argument("--set", default="all", choices=["all", "yaw", "lateral", "pairs"])
args = ap.parse_args()
out = Path(args.out)
out.mkdir(parents=True, exist_ok=True)

# an entry is a tuple of bursts (vx, vy, wz, seconds), sent one right after the other; the response is measured after the
# last. The families the servo uses today, with every sign of yaw, and pairs of lateral bursts (opposite sides, the same
# yaw): a turn that goes nowhere
FORWARD = [((0.4, 0.0, w, s),) for w in (-0.2, 0.0, 0.2) for s in (0.3, 0.4)]
BACK = [((-0.4, 0.0, w, s),) for w in (-0.2, 0.0, 0.2) for s in (0.2, 0.3, 0.4, 0.8)]
LATERAL = [((0.1, vy, w, s),) for vy in (0.3, -0.3) for w in (-0.2, 0.0, 0.2) for s in (0.5, 0.8, 1.2)]
SHUFFLE = [((0.0, vy, w, 1.2),) for vy in (0.3, -0.3) for w in (-0.2, 0.2)]
PAIRS = [((0.1, a, w, s), (0.1, -a, w, s)) for a in (0.3, -0.3) for w in (-0.2, 0.2) for s in (0.5, 0.8)]
if args.set == "yaw":
    TABLE = LATERAL + SHUFFLE + PAIRS
elif args.set == "lateral":
    TABLE = LATERAL
elif args.set == "pairs":
    TABLE = PAIRS
else:
    TABLE = FORWARD + BACK + LATERAL + SHUFFLE + PAIRS
AFTER = (0.0, 1.0, 2.0, 3.0, 5.0, 8.0)  # seconds after the command ends

site = S.load_site(SITES / "pick_place.json")
trials = []


def rel(p0, p1):
    c, s = math.cos(p0[2]), math.sin(p0[2])
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    return [dx * c + dy * s, -dx * s + dy * c, S.wrap(p1[2] - p0[2])]


with Session(SITES / "pick_place.json", out / "run", "free") as sess:
    samples = sess.tail.samples
    ops = G1Operations(site)
    ops.guard(site["name"])

    def pose(t_s=None):
        smp = samples[-1] if t_s is None else _sample_at(samples, t_s)
        return [float(smp["pelvis"]["pos"][0]), float(smp["pelvis"]["pos"][1]), float(yaw_of(smp["pelvis"]["quat"]))]

    def go_home(home, tol=0.5):
        """Back toward `home` (x, y, heading) with the oracle's help: this is the experiment's own tidying."""
        for _ in range(30):
            p = pose()
            e = rel(p, home)
            if math.hypot(e[0], e[1]) < tol and abs(e[2]) < 0.3:
                return
            ops.check()
            if abs(e[2]) > 0.3:
                ops.turn_by(float(np.clip(e[2], -0.5, 0.5)))  # (belief-based; only to tidy)
                continue
            vx = float(np.clip(e[0] * 2, -0.4, 0.5)) if abs(e[0]) > 0.2 else 0.0
            vy = float(np.clip(e[1] * 2, -0.3, 0.3)) if abs(e[1]) > 0.2 else 0.0
            if math.hypot(vx, vy) < 0.31:
                s = 0.31 / max(math.hypot(vx, vy), 1e-6)
                vx, vy = vx * s, vy * s
            ops.robot.set_velocity(vx, vy, 0.0, 1.5)
            ops.sleep(1.5)
            ops.settle()

    ops.move("bench")
    if args.carry:
        ops.grasp("box")
        print("carrying", ops.held["object"], flush=True)
    # open floor: well away from the tables, facing +x like the stations
    home = [1.0, 0.0, 0.0]
    ops.walk_to(home[0], home[1], 0.0, stop_short=0.3)
    ops.settle()
    print("home", [round(v, 3) for v in pose()], flush=True)

    def trial(cmd, label=None):
        go_home(home)
        ops.settle(12.0)
        p0, t0 = pose(), ops.now_ms()
        ops.check()
        for burst in cmd:
            code = ops.robot.set_velocity(*burst[:3], burst[3])
            assert code == 0, code
            ops.sleep(burst[3])
        t_end = ops.now_ms()
        row = {"cmd": [list(b) for b in cmd], "t0_ms": t0, "start": [round(v, 4) for v in p0], "after": {}}
        for a in AFTER:
            target = t_end + a * 1000
            while ops.now_ms() < target:
                ops.check()
                time.sleep(0.02)
            row["after"][str(a)] = [round(v, 4) for v in rel(p0, pose(target / 1000))]
        ok = ops.settle(9.0)
        row["settled_after_s"] = round((ops.now_ms() - t_end) / 1000, 1) if ok else None
        row["settled"] = [round(v, 4) for v in rel(p0, pose())]
        row["phase"] = samples[-1]["stance"]["phase"]
        trials.append(row)
        return row

    t_start = time.monotonic()
    for k in range(args.repeats):
        for cmd in TABLE:
            c = cmd
            r = trial(c)
            f = r["settled"]
            print(f"{[tuple(b) for b in c]}  moved fwd {f[0]:+.3f} left {f[1]:+.3f} yaw {f[2]:+.3f}  (+3 s: yaw {r['after']['3.0'][2]:+.3f}, +8 s: {r['after']['8.0'][2]:+.3f})", flush=True)
        (out / "response.json").write_text(json.dumps({"carry": args.carry, "trials": trials}))
    for k in range(args.rest):
        go_home(home)
        ops.settle(12.0)
        p0, t0 = pose(), ops.now_ms()
        series = []
        while ops.now_ms() < t0 + 20000:
            ops.check()
            time.sleep(0.5)
            series.append([round((ops.now_ms() - t0) / 1000, 1)] + [round(v, 4) for v in rel(p0, pose())])
        trials.append({"cmd": None, "rest": series, "start": [round(v, 4) for v in p0]})
        print("rest 20 s:", series[-1], flush=True)
    (out / "response.json").write_text(json.dumps({"carry": args.carry, "trials": trials}))
    print("wall minutes", round((time.monotonic() - t_start) / 60, 1))
