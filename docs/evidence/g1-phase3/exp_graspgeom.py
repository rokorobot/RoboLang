# ruff: noqa
"""Experiment for phase 3 (reads nothing from the oracle unless it says so; not the backend). See docs/evidence/g1-phase3-observations.md."""
import math, itertools
import numpy as np
exec(open("/tmp/exp/pathclear.py").read().split("def legs(")[0])
site = S.load_site(SITES / "pick_place.json")
o = bare(site)
tables = site["tables"]
clr = K.Clearance(site["camera"], n_boxes=len(tables))
st = o.stations["bench"]; pose = tuple(st["pose"])
o.pose = list(pose)
centre = np.array((0.271, -0.154, 0.0007))
clr.set_boxes(tables, pose, 0.03, 0.015)
closed = np.array(OPS.HAND_CLOSE["right"])
rows = []
for deg, rise, up in itertools.product((0, 15, 30, 45), (-0.01, 0.0, 0.01, 0.02), (0.03, 0.06, 0.09)):
    OPS.GRASP_RISE, OPS.APPROACH_UP = rise, up
    try:
        plan = o._plan_reach("right", lambda Rr: centre + np.array([0, 0, rise]), extra=(("lift", (0.0, 0.0, 0.10)),), degs=(deg,))
    except Exception as e:
        continue
    Rh, ph = kin.grasp_frame("right", plan["grasp"])
    off, rot = Rh.T @ (centre - ph), Rh.T
    clr.set_held("right", (0.04, 0.04, 0.04), off, rot)
    clear = o._clear_pose("right")
    legs_open = [(q_carry, clear), (clear, plan["pre"]), (plan["pre"], plan["grasp"])]
    legs_closed = [(plan["grasp"], plan["lift"]), (plan["lift"], plan["pre"]), (plan["pre"], clear), (clear, q_carry)]
    worst = {"arm": 9, "hand": 9, "held": 9}
    for qa, qb in legs_open:
        for a in np.linspace(0, 1, 10):
            r = clr.to_boxes("right", qa + a * (qb - qa), held=False)
            for g in ("arm", "hand"): worst[g] = min(worst[g], r[g][0])
    for k, (qa, qb) in enumerate(legs_closed):
        for a in np.linspace(0, 1, 10):
            r = clr.to_boxes("right", qa + a * (qb - qa), held=True, hand_q=closed)
            for g in ("arm", "hand"): worst[g] = min(worst[g], r[g][0])
            if k > 0: worst["held"] = min(worst["held"], r["held"][0])   # not on the first leg: it rests on the table
    rows.append((deg, rise, up, round(worst["arm"] * 100, 1), round(worst["hand"] * 100, 1), round(worst["held"] * 100, 1), round(plan["position_error_m"] * 100, 2)))
print("deg rise up  arm hand held(after lift)  [cm, margins 3 cm / 1.5 cm]")
for r in sorted(rows, key=lambda r: -min(r[3], r[4], r[5]))[:14]: print(r)
print("current (15, -0.01, 0.03):", [r for r in rows if r[:3] == (15, -0.01, 0.03)])

print("---- legs for (15, 0.02, 0.03)")
OPS.GRASP_RISE, OPS.APPROACH_UP = 0.02, 0.03
plan = o._plan_reach("right", lambda Rr: centre + np.array([0, 0, 0.02]), extra=(("lift", (0.0, 0.0, 0.10)),), degs=(15,))
Rh, ph = kin.grasp_frame("right", plan["grasp"])
off, rot = Rh.T @ (centre - ph), Rh.T
clr.set_held("right", (0.04, 0.04, 0.04), off, rot)
clear = o._clear_pose("right")
wps = [("carry", q_carry, 0), ("clear", clear, 0), ("pre", plan["pre"], 0), ("grasp", plan["grasp"], 0), ("lift", plan["lift"], 1), ("pre'", plan["pre"], 1), ("clear'", clear, 1), ("carry'", q_carry, 1)]
for (n0, qa, h0), (n1, qb, h1) in zip(wps, wps[1:]):
    worst = {}
    for a in np.linspace(0, 1, 12):
        r = clr.to_boxes("right", qa + a * (qb - qa), held=bool(h1), hand_q=closed if h1 else None)
        for g, v in r.items():
            if g not in worst or v[0] < worst[g][0]: worst[g] = (round(v[0] * 100, 1), v[1].replace("right_", ""), round(float(a), 2))
    print(f"  {n0}->{n1}: {worst}")
Rp, pp = kin.grasp_frame("right", plan["pre"]); Rg, pg = kin.grasp_frame("right", plan["grasp"]); Rc_, pc_ = kin.grasp_frame("right", clear)
print("pinch: clear", np.round(pc_, 3), "pre", np.round(pp, 3), "grasp", np.round(pg, 3), " table top in pelvis z =", 0.75 - 0.788, "table front x =", 2.68 - pose[0], "y range", 2.15 - 2.40 if False else (-0.25, 0.12))
