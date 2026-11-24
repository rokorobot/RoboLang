# ruff: noqa: E501, E741, E702
"""Experiment (reads the oracle; not the backend): Displacement of short forward, backward and sideways bursts from rest, four repeats each: the step table of ops.py.

Run from the repository root with a stand-in-capable environment (see docs/evidence/g1-phase2-executor.md).
"""

import time
import tempfile
import math
from pathlib import Path
from robolang_g1 import guard
from robolang_g1.harness import StandIn, OracleTail, yaw_of, in_frame
from robolang_g1.robot import Robot

run = Path(tempfile.mkdtemp(prefix="exp_"))
si = StandIn("backends/unitree_g1/robolang_g1/sites/pick_place.json", run, "free")
si.wait_ready()
client, ident = guard.require_simulation(timeout=30)
tail = OracleTail(si.oracle)
tail.start()
tail.wait(lambda s: s["sim_time"] > 0, 30)
robot = Robot()
robot.wait_state()
time.sleep(4)


def burst(vx, vy, wz, T):
    s0 = tail.latest()
    p0, y0 = s0["pelvis"]["pos"], yaw_of(s0["pelvis"]["quat"])
    robot.set_velocity(vx, vy, wz, T)
    time.sleep(T + 3.0)
    s1 = tail.latest()
    f, l = in_frame(p0, s1["pelvis"]["pos"], y0)
    dy = (yaw_of(s1["pelvis"]["quat"]) - y0 + math.pi) % (2 * math.pi) - math.pi
    return f, l, dy


for vx, T in ((0.4, 0.1), (0.4, 0.2), (0.4, 0.3), (0.4, 0.4), (0.35, 0.25), (-0.4, 0.2), (-0.4, 0.3), (-0.4, 0.4)):
    res = [burst(vx, 0, 0, T) for _ in range(4)]
    print(
        f"vx {vx:+.2f} T {T:.2f}: fwd "
        + " ".join(f"{r[0]:+.3f}" for r in res)
        + "   yaw "
        + " ".join(f"{r[2]:+.3f}" for r in res),
        flush=True,
    )
for vy, T in ((0.3, 0.3), (0.3, 0.5), (-0.3, 0.3), (-0.3, 0.5)):
    res = [burst(0.1, vy, 0, T) for _ in range(4)]
    print(
        f"(0.1, {vy:+.2f}) T {T:.2f}: left "
        + " ".join(f"{r[1]:+.3f}" for r in res)
        + "   fwd "
        + " ".join(f"{r[0]:+.3f}" for r in res),
        flush=True,
    )
tail.stop()
si.stop()
