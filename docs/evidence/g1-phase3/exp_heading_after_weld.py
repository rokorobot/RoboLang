# ruff: noqa
"""Experiment for phase 3 (reads the oracle; not the backend): one w_carry_hold run, the oracle heading at the dst station over time with the stance and the snapshot times."""
import json, math, sys
from pathlib import Path
from robolang_g1.run_task import Session
from robolang_g1.phase2 import SITES, BIND_CH, yaw_of
from robolang_g1 import site as S
REPO = Path("/code/robolang")
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
site = S.load_site(SITES / "carry_hold.json")
dst = [s for s in site["stations"] if s["name"] == "dst"][0]["pose"]
with Session(SITES / "carry_hold.json", out, "free") as s:
    status, rec, log = s.run(REPO / "bench/reference/w_carry_hold.rob", "carry_crate", SITES / "carry_hold.json", BIND_CH, out / "record.json")
    samples = list(s.tail.samples)
print("exit", status, rec["state"])
t_move_end = rec["steps"][-1]["t_end_ms"] / 1000
snap = rec["snapshots"][0]["t_ms"] / 1000
om = rec["observation_motion"][0]
print("move end", t_move_end, "snapshot", snap, "motion", om["t_start_ms"] / 1000, om["t_end_ms"] / 1000)
for e in rec["steps"][-1]["details"]["events"][-5:]: print(e)
print([ (c["result"], [(a.get("offset_m"), a.get("offset_rad")) for a in c["atoms"]]) for c in rec["goal"]])
last = -9
for x in samples:
    t = x["sim_time"]
    if t >= t_move_end - 6 and t - last >= 0.5:
        last = t
        p = x["pelvis"]["pos"]
        print(f"{t:7.1f} d={math.hypot(p[0]-dst[0], p[1]-dst[1])*100:5.1f}cm head={S.wrap(yaw_of(x['pelvis']['quat'])-dst[2]):+.4f} phase={x['stance']['phase']} model={x['stance'].get('model')}")
