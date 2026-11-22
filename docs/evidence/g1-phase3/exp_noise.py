# ruff: noqa
"""Experiment for phase 3 (reads the oracle; not the backend). Run from the repository root with a stand-in-capable environment (see docs/evidence/g1-phase3-observations.md)."""
import json, sys, time
from pathlib import Path
import numpy as np
from robolang_g1 import perception as P, site as site_mod
from robolang_g1.run_task import Session
from robolang_g1.phase2 import SITES
from robolang_g1.ops import G1Operations

raw = {}
orig = P.fuse_frames
def spy(mid, dets, frames, *a, **k):
    raw.setdefault(mid, []).append(list(dets))
    return orig(mid, dets, frames, *a, **k)
P.fuse_frames = spy

poses = json.loads(sys.argv[1])
out = Path("/tmp/exp/noise_out"); out.mkdir(exist_ok=True)
base = json.loads((SITES / "pick_place.json").read_text())
for k, start in enumerate(poses):
    base["robot"]["start"] = start
    scene = out / f"scene{k}.json"; scene.write_text(json.dumps(base))
    site = site_mod.load_site(scene)
    with Session(scene, out / f"run{k}", "free") as s:
        ops = G1Operations(site); ops.guard("g1")
        time.sleep(2)
        raw.clear()
        for rep in range(3):
            readings, got = ops.perc.snapshot(frames=10, check=ops.check)
            time.sleep(0.5)
        print(f"== start {start}", flush=True)
        for mid, snaps in sorted(raw.items()):
            for dets in snaps:
                Pm = np.array([d.position for d in dets]); med = np.median(Pm, axis=0)
                dist = np.linalg.norm(Pm - med, axis=1)
                rots = [d.rotation for d in dets]
                tot = [sum(P._angle(a, b) for b in rots) for a in rots]; medoid = rots[int(np.argmin(tot))]
                ang = [P._angle(d.rotation, medoid) for d in dets]
                ups = [d.up_error_rad for d in dets if d.up_error_rad is not None]
                print(f" marker {mid} n={len(dets)} range={np.median([d.range_m for d in dets]):.2f} pos_dev max={dist.max()*100:.2f}cm "
                      f"p90={np.percentile(dist,90)*100:.2f}cm rot_dev max={max(ang):.4f} up_err max={max(ups) if ups else float('nan'):.4f}", flush=True)
