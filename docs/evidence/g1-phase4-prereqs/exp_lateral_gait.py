# ruff: noqa
"""Measurement only (g1-phase4-prereqs.md): which sideways commands does the loaded robot answer?

    python exp_lateral_gait.py OUTDIR [--empty]

One stand-in on the part_precheck site, the executor in this process: `move` to the bench and `grasp` the box (unless --empty), then
a series of 3.4 s commands (vx, +-0.3, wz) with a settle between, each judged by the oracle: how far the pelvis went sideways
and forward, and how much it turned. `walk_to` sends (0, 0.3, 0) when the target is straight to the side and the heading is within
0.04 rad, which is a command of size 0.300: the walking policy ignores commands below about 0.3.
"""
import json, math, sys, time
from pathlib import Path

from robolang_g1 import site as site_mod
from robolang_g1.ops import G1Operations
from robolang_g1.phase2 import SITES, _sample_at
from robolang_g1.run_task import Session

out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
empty = "--empty" in sys.argv
site = site_mod.load_site(SITES / "part_precheck.json")
CONFIGS = [(0.0, 0.0), (0.1, 0.0), (0.16, 0.0), (0.2, 0.0), (0.3, 0.0), (0.0, 0.2), (0.0, -0.2)]
rows = []
with Session(SITES / "part_precheck.json", out, "free") as s:
    samples = s.tail.samples
    ops = G1Operations(site)
    ops.guard(site["name"])
    ops.move("bench")
    if not empty:
        ops.grasp("box")
    def pose():
        o = _sample_at(samples, ops.now_ms() / 1000)["pelvis"]
        w, x, y, z = o["quat"]
        return o["pos"][0], o["pos"][1], math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    for rep in range(0 if "--legs-only" in sys.argv else 2):
        for vx, wz in CONFIGS:
            for sign in (1, -1):
                ops.settle()
                x0, y0, h0 = pose()
                ops.command(vx, 0.3 * sign, wz * sign if wz else 0.0, 3.4)
                ops.settle()
                x1, y1, h1 = pose()
                dx, dy = x1 - x0, y1 - y0
                left = -dx * math.sin(h0) + dy * math.cos(h0); fwd = dx * math.cos(h0) + dy * math.sin(h0)
                d = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
                rows.append({"cmd": [vx, 0.3 * sign, wz * sign if wz else 0.0, 3.4], "norm": round(math.hypot(vx, 0.3, wz), 3),
                             "left_m": round(left, 3), "fwd_m": round(fwd, 3), "turn_rad": round(d, 3)})
                print(rows[-1], flush=True)
    if "--legs" in sys.argv:  # the two-diagonal leg the executor now walks for a leg straight to the side (ops.walk_legs)
        from robolang_g1.ops import walk_legs
        for rep in range(3):
            for sign in (1, -1):
                for vx0 in (0.0, 0.03, -0.03):
                    ops.settle()
                    x0, y0, h0 = pose()
                    for leg in walk_legs(vx0, 0.3 * sign, 0.0, 3.4):
                        ops.command(*leg)
                    ops.settle()
                    x1, y1, h1 = pose()
                    dx, dy = x1 - x0, y1 - y0
                    left = -dx * math.sin(h0) + dy * math.cos(h0); fwd = dx * math.cos(h0) + dy * math.sin(h0)
                    d = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
                    rows.append({"cmd": "walk_legs", "vx0": vx0, "vy": 0.3 * sign, "left_m": round(left, 3),
                                 "fwd_m": round(fwd, 3), "turn_rad": round(d, 3)})
                    print(rows[-1], flush=True)
(out / "lateral_gait.json").write_text(json.dumps({"loaded": not empty, "rows": rows}, indent=1))
