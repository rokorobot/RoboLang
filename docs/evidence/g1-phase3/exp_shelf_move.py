# ruff: noqa
"""Experiment for phase 3 (reads nothing from the oracle unless it says so; not the backend). See docs/evidence/g1-phase3-observations.md."""
import json, math, sys
from pathlib import Path
import numpy as np
from robolang_g1.run_task import Session
from robolang_g1.phase2 import SITES, _Tail
from robolang_g1 import site as site_mod
out = Path("/tmp/exp/shelf_move"); out.mkdir(exist_ok=True)
raw = json.loads((SITES / "pick_place.json").read_text())
st = site_mod.load_site(SITES / "pick_place.json")
raw["robot"]["start"] = st["stations"][0]["pose"]
scene = out / "scene.json"; scene.write_text(json.dumps(raw))
src = out / "go.rob"
src.write_text("robolang 1.3;\ntask go(robot r, location a) {\n    goal { robot_at r a; }\n    plan { move r to a; }\n}\n")
def yaw(q): return math.atan2(2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2))
for k in range(int(sys.argv[1])):
    d = out / f"run{k}"
    with Session(scene, d, "free") as s:
        status, rec, log = s.run(src, "go", scene, {"r": "robot_1", "a": "loc_b"}, d / "record.json")
        samples = list(s.tail.samples)
    tail = _Tail(samples)
    print(f"== run {k}: exit {status} {rec['state']} {rec['failure'] and rec['failure']['code']} move {(rec['steps'][0]['t_end_ms']-rec['steps'][0]['t_start_ms'])/1000:.0f} s", flush=True)
    for e in rec["steps"][0]["details"]["events"]:
        if e["event"] in ("localized", "replan", "scan", "rejected"):
            o = tail.nearest(e["t_ms"] / 1000)
            true = (round(o["pelvis"]["pos"][0], 2), round(o["pelvis"]["pos"][1], 2), round(yaw(o["pelvis"]["quat"]), 2))
            print(f"  t={e['t_ms']/1000:6.1f} {e['event']:9s} marker={e.get('marker')} from={e.get('from')} to={e.get('to')} oracle={true}", flush=True)
