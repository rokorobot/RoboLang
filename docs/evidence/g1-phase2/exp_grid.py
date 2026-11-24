# ruff: noqa: E501, E741, E702
"""Experiment (reads the oracle; not the backend): What the walking policy does for each combination of vx, vy, wz (3 s commands): the command-size deadband.

Run from the repository root with a stand-in-capable environment (see docs/evidence/g1-phase2-executor.md).
"""

import time
import tempfile
import math
import itertools
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
T = 3.0
for vx, vy, wz in itertools.product((0.0, 0.1, 0.2, 0.3), (0.0, 0.15, 0.3), (0.0, 0.2)):
    if vx == vy == wz == 0:
        continue
    s0 = tail.latest()
    p0, y0 = s0["pelvis"]["pos"], yaw_of(s0["pelvis"]["quat"])
    robot.set_velocity(vx, vy, wz, T)
    time.sleep(T + 2.8)
    s1 = tail.latest()
    p1, y1 = s1["pelvis"]["pos"], yaw_of(s1["pelvis"]["quat"])
    f, l = in_frame(p0, p1, y0)
    dy = (y1 - y0 + math.pi) % (2 * math.pi) - math.pi
    print(
        f"cmd {vx:.2f} {vy:.2f} {wz:.2f} (norm {math.sqrt(vx * vx + vy * vy + wz * wz):.2f}) -> fwd {f:+.2f} left {l:+.2f} yaw {dy:+.3f}  speed {math.hypot(f, l) / T:.2f} m/s yaw rate {dy / T:+.3f} rad/s  fallen {s1['fallen']}",
        flush=True,
    )
    if s1["fallen"]:
        break
tail.stop()
si.stop()
