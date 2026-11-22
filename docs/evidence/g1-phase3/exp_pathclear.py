# ruff: noqa
"""Experiment for phase 3 (reads nothing from the oracle unless it says so; not the backend). See docs/evidence/g1-phase3-observations.md."""
import sys, math, json
import numpy as np
from pathlib import Path
from robolang_g1 import kinematics as K, assets, ops as OPS, site as S
from robolang_g1.sim.policy import load_deploy

cfg = load_deploy(assets.asset(*assets.POLICY_DIR, "params", "deploy.yaml"))
q_carry = np.zeros(29); q_carry[cfg["joint_ids_map"]] = cfg["default_joint_pos"]; q_carry[12:15] = 0
kin = K.Kinematics()
SITES = Path("/code/robolang/backends/unitree_g1/robolang_g1/sites")

def bare(site):
    o = OPS.G1Operations.__new__(OPS.G1Operations)
    o.kin, o.site, o.q_carry, o.events = kin, site, q_carry.copy(), []
    o.stations = {s["name"]: s for s in site["stations"]}
    return o

def legs(o, plan, names_q):
    return list(zip(names_q, names_q[1:]))

def scan(clr, side, wps, held_from, label, xy, z, samples=12):
    """min distance per group per leg; the held object counts from leg index `held_from` on"""
    out = []
    for k, ((n0, q0), (n1, q1)) in enumerate(zip(wps, wps[1:])):
        best = {}
        for a in np.linspace(0, 1, samples):
            q = q0 + a * (q1 - q0)
            r = clr.to_boxes(side, q, held=(k >= held_from))
            for grp, (d, link, box) in r.items():
                if grp not in best or d < best[grp][0]:
                    best[grp] = (round(d, 4), link, box, round(float(a), 2))
        out.append((f"{n0}->{n1}", best))
    return out

def run(site_name, station, obj_name, centre_grasp, centre_place, place_station=None, deg=15, half=(0.04, 0.04, 0.04)):
    site = S.load_site(SITES / f"{site_name}.json")
    o = bare(site)
    tables = site["tables"]
    clr = K.Clearance(site["camera"], n_boxes=len(tables))
    print(f"==== {site_name}: grasp at {station}")
    st = o.stations[station]
    pose = tuple(st["pose"])
    R = OPS._frame(math.radians(deg))
    centre = np.array(centre_grasp)
    plan = o._plan_reach("right", lambda Rr: centre + np.array([0, 0, OPS.GRASP_RISE]), extra=(("lift", (0.0, 0.0, 0.10)),))
    Rh, ph = kin.grasp_frame("right", plan["grasp"])
    off, rot = Rh.T @ (centre - ph), Rh.T
    o.pose = list(pose); o.held = {"object": obj_name, "hand": "right", "offset_in_hand": off, "rotation_in_hand": rot}
    clear = o._clear_pose("right")
    for xy, z in ((0.0, 0.0), (0.03, 0.015)):
        clr.set_boxes(tables, pose, xy, z)
        clr.set_held("right", half, off, rot)
        wps = [("carry", q_carry), ("clear", clear), ("pre", plan["pre"]), ("grasp", plan["grasp"]), ("lift", plan["lift"]), ("pre'", plan["pre"]), ("clear'", clear), ("carry'", q_carry)]
        print(f"-- grasp path, margins sideways {xy} vertical {z}")
        for leg, best in scan(clr, "right", wps, held_from=3, label="grasp", xy=xy, z=z):
            print(f"   {leg:18s} " + "  ".join(f"{g}: {v[0]*100:5.1f} cm ({v[1].replace('right_','')}/{v[2]})" for g, v in best.items()))
    # place at the place station
    ps = place_station or station
    st2 = o.stations[ps]; pose2 = tuple(st2["pose"])
    o.pose = list(pose2)
    centre2 = np.array(centre_place)
    plan2 = o._plan_reach("right", lambda Rr: centre2 - Rr @ off, extra=(("above", (0.0, 0.0, 0.10)),), degs=(deg,))
    for xy, z in ((0.0, 0.0), (0.03, 0.015)):
        clr.set_boxes(tables, pose2, xy, z)
        clr.set_held("right", half, off, rot)
        wps = [("carry", q_carry), ("clear", clear), ("pre", plan2["pre"]), ("place", plan2["grasp"]), ("pre'", plan2["pre"]), ("clear'", clear), ("carry'", q_carry)]
        print(f"-- place path at {ps}, margins sideways {xy} vertical {z}")
        for leg, best in scan(clr, "right", wps, held_from=0, label="place", xy=xy, z=z):
            print(f"   {leg:18s} " + "  ".join(f"{g}: {v[0]*100:5.1f} cm ({v[1].replace('right_','')}/{v[2]})" for g, v in best.items()))
    # the default carry pose with the object
    clr.set_held("right", half, off, rot)
    clr.set_boxes([], pose, 0, 0)
    d, link = clr.held_to_links("right", q_carry)
    print(f"-- default carry pose, object {tuple(2*np.array(half))} welded as grasped: nearest link {link} at {d*100:.2f} cm")

run("pick_place", "bench", "box", (0.271, -0.154, 0.0007), (0.262, -0.110, -0.0026), place_station="shelf")
run("carry_hold", "dock", "crate", (0.271, -0.154, 0.0007), (0.262, -0.110, -0.0026), half=(0.045, 0.045, 0.04))
