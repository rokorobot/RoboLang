# ruff: noqa: E501, E741, E702
"""Experiment (reads the oracle; not the backend): Turning in place by shuffling (alternating sideways commands with a yaw rate), and arcs, in both directions.

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


def state():
    s = tail.latest()
    return s["pelvis"]["pos"], yaw_of(s["pelvis"]["quat"])


def report(label, p0, y0):
    p1, y1 = state()
    f, l = in_frame(p0, p1, y0)
    dy = (y1 - y0 + math.pi) % (2 * math.pi) - math.pi
    print(f"{label}: fwd {f:+.3f} left {l:+.3f} yaw {dy:+.3f} ({math.degrees(dy):+.1f} deg)", flush=True)


for sign in (+1, -1):
    for k, (vx, vy_pattern) in enumerate(((0.0, (0.3, -0.3)), (0.1, (0.3, -0.3)))):
        p0, y0 = state()
        for rep in range(3):
            for vy in vy_pattern:
                robot.set_velocity(vx, vy, 0.2 * sign, 1.2)
                time.sleep(1.2 + 0.2)
        time.sleep(3.0)
        report(f"shuffle wz {0.2 * sign:+.1f} vx {vx} (3 x [+vy,-vy] bursts of 1.2 s)", p0, y0)
for sign in (+1, -1):
    p0, y0 = state()
    robot.set_velocity(0.3, 0, 0.2 * sign, 5.0)
    time.sleep(8.5)
    report(f"arc (0.3,0,{0.2 * sign:+.1f}) 5s", p0, y0)
    p0, y0 = state()
    robot.set_velocity(-0.3, 0, 0.2 * sign, 5.0)
    time.sleep(8.5)
    report(f"reverse arc (-0.3,0,{0.2 * sign:+.1f}) 5s", p0, y0)
tail.stop()
si.stop()
