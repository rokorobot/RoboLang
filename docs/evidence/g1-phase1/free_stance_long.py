"""G1 phase 1: how long does the `free` stance stand still? 200 s at zero command, from rest and after a 4 s walk.

Usage: free_stance_long.py   (needs the assets and robolang-g1; about 30 s)

Prints the recovery steps: windows of 5 s in which the pelvis moved more than 5 cm (the first 10 s are skipped), with
the time since the start (or since the stop) and the distance. The simulation is deterministic: the same steps come
out every time. Reads MuJoCo state: it is an experiment, not the backend.
"""
import numpy as np

from robolang_g1.sim.standin import Simulator


def trial(site, walk: bool):
    sim = Simulator(site, stance="free", camera=False)
    d, r = sim.data, sim.robot

    def run(seconds):
        for _ in range(round(seconds / sim.model.opt.timestep)):
            sim.step()

    run(1.0)
    if walk:
        sim.set_velocity(0.5, 0, 0, 4.0)
        run(4.0)
    t0, steps = sim.sim_time, []
    for window in range(40):
        p0 = d.xpos[r.pelvis][:2].copy()
        run(5)
        moved = float(np.linalg.norm(d.xpos[r.pelvis][:2] - p0))
        if moved > 0.05 and window >= 2:
            steps.append((round(sim.sim_time - t0), round(moved, 2)))
    return steps, sim.fell_at


if __name__ == "__main__":
    from pathlib import Path

    import robolang_g1

    site = Path(robolang_g1.__file__).parent / "sites" / "phase1.json"
    for walk in (False, True):
        steps, fell = trial(site, walk)
        print("after a 4 s walk" if walk else "from rest      ", "steps (s since start/stop, m):", steps, "fell:", fell)
