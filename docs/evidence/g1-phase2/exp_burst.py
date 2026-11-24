# ruff: noqa: E501, E741, E702
"""Experiment (reads the oracle; not the backend): Displacement of short bursts, repeated (an early version of exp_fine).

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


def run_cmd(vx, vy, wz, T):
    s0 = tail.latest()
    p0, y0 = s0["pelvis"]["pos"], yaw_of(s0["pelvis"]["quat"])
    robot.set_velocity(vx, vy, wz, T)
    time.sleep(T + 3.0)
    s1 = tail.latest()
    p1, y1 = s1["pelvis"]["pos"], yaw_of(s1["pelvis"]["quat"])
    f, l = in_frame(p0, p1, y0)
    dy = (y1 - y0 + math.pi) % (2 * math.pi) - math.pi
    print(
        f"cmd ({vx:+.2f},{vy:+.2f},{wz:+.2f}) for {T:.1f}s -> fwd {f:+.3f} left {l:+.3f} yaw {dy:+.3f}  fallen {s1['fallen']}",
        flush=True,
    )


for rep in range(2):
    for T in (0.3, 0.5, 0.8, 1.2):
        run_cmd(0.35, 0, 0, T)
    for T in (0.5, 0.8, 1.2):
        run_cmd(-0.35, 0, 0, T)
    for T in (0.5, 0.8, 1.2):
        run_cmd(0.1, 0.3, 0, T)
    for T in (0.5, 0.8, 1.2):
        run_cmd(0.1, -0.3, 0, T)
    for T in (0.8, 1.5):
        run_cmd(0.1, 0.3, 0.2, T)
tail.stop()
si.stop()
