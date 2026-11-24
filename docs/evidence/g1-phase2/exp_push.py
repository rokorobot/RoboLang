# ruff: noqa: E501, E741, E702
"""Experiment (reads the oracle; not the backend): How hard a push on the torso topples the walking robot (the E5 fall test).

Run from the repository root with a stand-in-capable environment (see docs/evidence/g1-phase2-executor.md).
"""

import time
import tempfile
from pathlib import Path
from robolang_g1.run_task import Session
from robolang_g1.phase2 import _Push, tilt_deg
from robolang_g1.robot import Robot

site = "backends/unitree_g1/robolang_g1/sites/pick_place.json"
for force, dur in ((500, 0.4), (800, 0.4), (1200, 0.4)):
    with Session(site, Path(tempfile.mkdtemp(prefix="push_"))) as s:
        robot = Robot()
        robot.wait_state()
        robot.set_velocity(0.5, 0, 0, 20.0)
        time.sleep(4)
        push = _Push()
        code = push.push((0, force, 0), dur)
        t0 = s.tail.latest()["sim_time"]
        tmax = 0
        for _ in range(60):
            time.sleep(0.1)
            sm = s.tail.latest()
            tmax = max(tmax, tilt_deg(sm))
        sm = s.tail.latest()
        print(
            f"push {force} N x {dur} s: reply {code[0]} fallen {sm['fallen']} max tilt {tmax:.0f} deg, final tilt {tilt_deg(sm):.0f}, pelvis z {sm['pelvis']['pos'][2]:.2f}",
            flush=True,
        )
