# ruff: noqa
"""Experiment for phase 3 (reads nothing from the oracle unless it says so; not the backend). See docs/evidence/g1-phase3-observations.md."""
import sys; sys.path.insert(0, "/code/robolang/backends/unitree_g1/tests")
import numpy as np, math, itertools, mujoco
from test_kinematics import *
from robolang_g1 import kinematics as K, assets
from robolang_g1.sim.policy import load_deploy
cfg = load_deploy(assets.asset(*assets.POLICY_DIR, "params", "deploy.yaml"))
q = np.zeros(29); q[cfg["joint_ids_map"]] = cfg["default_joint_pos"]
kin = K.Kinematics()
o = clearance_ops(kin, q, "pick_place", "bench")
boxes = o.site["tables"]
closed = np.array(ops.HAND_CLOSE["right"])
clr = o.clr
print("deg rise | open hand clearance at the grasp (cm, real table) | closed-finger top above the object's centre (cm; the top face is 4.0)")
for deg, rise in itertools.product((0, 15, 30, 45), (-0.01, 0.0, 0.01, 0.02, 0.03)):
    try:
        plan = o._plan_reach("right", lambda R: BOX_CENTRE + np.array([0, 0, rise]), extra=(("lift", (0, 0, 0.1)),), degs=(deg,))
    except Exception as e:
        print(deg, rise, "no IK"); continue
    clr.set_boxes(boxes, tuple(o.pose), 0.0, 0.0)
    f = clr.to_boxes("right", plan["grasp"], held=False)
    # closed-finger geometry at the grasp pose: highest point of any finger geom above the object's centre
    clr._pose(plan["grasp"], {"right": closed})
    top = -9
    for g in clr.hand_geoms["right"]:
        if clr.model.geom_group[g] == 1:  # finger capsules
            sz = clr.model.geom_size[g]; pos = clr.data.geom_xpos[g]; R = clr.data.geom_xmat[g].reshape(3, 3)
            axis = R[:, 2]; half = sz[1] if len(sz) > 1 else 0
            top = max(top, max(pos[2] + half * abs(axis[2]), pos[2]) + sz[0])
    print(f"{deg:3d} {rise:5.2f} | hand {f['hand'][0]*100:5.1f} ({f['hand'][1].replace('right_','')}) arm {f['arm'][0]*100:4.1f} | finger top z {(top - BOX_CENTRE[2])*100:5.1f} cm")

print("---- do the closed fingers reach over the object's top face (z > 4 cm, inside its 8x8 cm footprint)?")
def cover(deg, rise):
    plan = o._plan_reach("right", lambda R: BOX_CENTRE + np.array([0, 0, rise]), extra=(("lift", (0, 0, 0.1)),), degs=(deg,))
    clr._pose(plan["grasp"], {"right": closed})
    n = 0; worst = 0.0
    for g in clr.hand_geoms["right"]:
        if clr.model.geom_group[g] != 1: continue
        sz = clr.model.geom_size[g]; pos = clr.data.geom_xpos[g].copy(); R = clr.data.geom_xmat[g].reshape(3, 3)
        for t in np.linspace(-sz[1], sz[1], 5):
            pt = pos + R[:, 2] * t
            rel = pt - BOX_CENTRE
            # the object is flat in the pelvis frame: footprint |x|,|y| <= 0.04 around its centre
            if abs(rel[0]) <= 0.04 and abs(rel[1]) <= 0.04 and rel[2] - sz[0] > 0.04:
                n += 1; worst = max(worst, rel[2] - sz[0] - 0.04)
    return n, round(worst * 100, 1)
for deg, rise in ((0, 0.0), (0, 0.01), (0, 0.02), (15, 0.02), (15, 0.03), (15, 0.0)):
    print(deg, rise, "finger points over the top face:", cover(deg, rise))
