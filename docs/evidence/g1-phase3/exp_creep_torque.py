# ruff: noqa
"""Experiment (reads the oracle; not the backend): the heading creep of the free stance with a steady torque about the vertical
on the torso (the stand-in's test hook, `--creep` of the move stress check): the heading rate over 20 s at rest.

    MUJOCO_GL=egl python docs/evidence/g1-phase3/exp_creep_torque.py OUTDIR TORQUE [TORQUE ...] [--carry]
"""
import json, math, sys, time
from pathlib import Path

from robolang_g1 import site as S
from robolang_g1.harness import yaw_of
from robolang_g1.ops import G1Operations
from robolang_g1.phase2 import SITES, _Push, _sample_at
from robolang_g1.run_task import Session

args = [a for a in sys.argv[1:] if not a.startswith("--")]
carry = "--carry" in sys.argv
out, torques = Path(args[0]), [float(a) for a in args[1:]]
out.mkdir(parents=True, exist_ok=True)
site = S.load_site(SITES / "pick_place.json")
results = []
with Session(SITES / "pick_place.json", out / "run", "free") as sess:
    samples = sess.tail.samples
    ops = G1Operations(site)
    ops.guard(site["name"])
    ops.move("bench")
    if carry:
        ops.grasp("box")
    ops.walk_to(1.0, 0.0, 0.0, stop_short=0.3)
    pusher = _Push()
    for torque in torques:
        pusher.push((0, 0, 0), 1e6, (0, 0, torque))
        ops.settle(12.0)
        ops.sleep(2.0)
        p0, t0 = samples[-1], ops.now_ms()
        yaw0 = yaw_of(p0["pelvis"]["quat"])
        series = []
        while ops.now_ms() < t0 + 20000:
            ops.check()
            time.sleep(1.0)
            s = samples[-1]
            series.append(((ops.now_ms() - t0) / 1000, S.wrap(yaw_of(s["pelvis"]["quat"]) - yaw0),
                           math.hypot(s["pelvis"]["pos"][0] - p0["pelvis"]["pos"][0], s["pelvis"]["pos"][1] - p0["pelvis"]["pos"][1])))
        t, dyaw, dpos = series[-1]
        results.append({"torque": torque, "rate_rad_s": dyaw / t, "moved_m": dpos, "series": series})
        print(f"torque {torque:+.1f} N m: heading {dyaw:+.3f} rad in {t:.0f} s = {dyaw / t:+.4f} rad/s, moved {dpos:.3f} m, tilt ok", flush=True)
    pusher.push((0, 0, 0), 1.0, (0, 0, 0))
(out / "creep.json").write_text(json.dumps(results))
