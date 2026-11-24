"""G1 phase 1, decision U1: does the whole-body policy still walk with Dex3 hands? Compares robot models.

Usage: hand_model_check.py [dt]   (default 0.002). Needs the assets (backends/unitree_g1/scripts/fetch_assets.sh) and
robolang-g1 installed. Each model gets the same treatment as in robolang_g1/sim/scene.py: torque motors in SDK order
0-28 (hands after), the joint dynamics of unitree_mujoco's G1, the policy's gains. Then: 3 s standing free at zero
command, then SetVelocity-like 0.5 m/s for 4 s. Reads MuJoCo state: it is an experiment, not the backend.

Variants: `hands` = the model as shipped; `fingers ignore body` = finger links no longer collide with the robot (they
still collide with the world); `no hands` = unitree_mujoco's g1_29dof.xml, the model the policy is deployed on.
"""
import sys

import mujoco
import numpy as np

from robolang_g1 import assets
from robolang_g1.sim.policy import WholeBodyPolicy
from robolang_g1.sim.scene import BODY_JOINTS, HAND_JOINTS

MODELS = {
    "unitree_mujoco g1_29dof (no hands)": (("unitree_mujoco", "unitree_robots", "g1", "g1_29dof.xml"), False),
    "unitree_rl_gym g1_29dof_with_hand": (assets.HAND_MODEL, False),
    "unitree_rl_gym g1_29dof_with_hand, fingers ignore body": (assets.HAND_MODEL, True),
    "menagerie g1_with_hands": (("mujoco_menagerie", "unitree_g1", "g1_with_hands.xml"), False),
    "menagerie g1_with_hands, fingers ignore body": (("mujoco_menagerie", "unitree_g1", "g1_with_hands.xml"), True),
}


def build(path, dt, fingers_ignore_body):
    spec = mujoco.MjSpec.from_file(str(assets.asset(*path)))
    force = {a.target: float(a.forcerange[1]) for a in spec.actuators if a.forcerange[1] > 0}  # menagerie keeps it here
    hands = any("_hand_" in j.name for j in spec.joints)
    for j in spec.joints:
        if j.name in BODY_JOINTS:
            j.armature, j.damping = 0.01, [0.05, 0, 0]
            j.frictionloss = 0.1 if "wrist_pitch" in j.name or "wrist_yaw" in j.name else 0.2
        elif "_hand_" in j.name:
            j.armature, j.damping, j.frictionloss = 0.005, [0.02, 0, 0], 0.0
        force.setdefault(j.name, float(j.actfrcrange[1]))
    for a in list(spec.actuators):
        spec.delete(a)
    for k in list(spec.keys):
        spec.delete(k)
    for j in BODY_JOINTS + (HAND_JOINTS["left"] + HAND_JOINTS["right"] if hands else []):
        spec.add_actuator(name=j, target=j, trntype=mujoco.mjtTrn.mjTRN_JOINT, gear=[1, 0, 0, 0, 0, 0],
                          ctrllimited=True, ctrlrange=[-force[j], force[j]])
    if not any(g.name == "floor" and g.parent.name == "world" for g in spec.geoms):
        spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05])
    if fingers_ignore_body:
        for g in spec.geoms:
            if "_hand_" in g.parent.name and "palm" not in g.parent.name:
                g.contype, g.conaffinity = 4, 0  # collides only with geoms whose conaffinity has bit 4
            elif g.name == "floor":
                g.conaffinity = 5
    spec.option.timestep = dt
    spec.body("pelvis").pos = [0, 0, 0.788]
    return spec.compile(), hands


def run(path, fingers_ignore_body, dt):
    m, hands = build(path, dt, fingers_ignore_body)
    d = mujoco.MjData(m)
    pol_dir = assets.asset(*assets.POLICY_DIR)
    pol = WholeBodyPolicy(pol_dir / "exported" / "policy.onnx", pol_dir / "params" / "deploy.yaml")
    qadr = np.array([m.jnt_qposadr[m.joint(j).id] for j in BODY_JOINTS])
    dadr = np.array([m.jnt_dofadr[m.joint(j).id] for j in BODY_JOINTS])
    hq = [m.jnt_qposadr[m.joint(j).id] for j in HAND_JOINTS["left"] + HAND_JOINTS["right"]] if hands else []
    hd = [m.jnt_dofadr[m.joint(j).id] for j in HAND_JOINTS["left"] + HAND_JOINTS["right"]] if hands else []
    d.qpos[qadr] = pol.default
    mujoco.mj_forward(m, d)
    pel, every = m.body("pelvis").id, round(0.02 / dt)
    state = {"n": 0, "target": pol.default.copy(), "sat": 0, "steps": 0}
    lim = m.actuator_ctrlrange[:, 1]

    def step(command):
        if state["n"] % every == 0:
            state["target"] = pol.act(d.qvel[3:6], d.qpos[3:7], command, d.qpos[qadr], d.qvel[dadr])
        ctrl = np.zeros(m.nu)
        ctrl[:29] = pol.kp * (state["target"] - d.qpos[qadr]) - pol.kd * d.qvel[dadr]
        if hands:
            ctrl[29:] = 1.5 * (0 - d.qpos[hq]) - 0.2 * d.qvel[hd]
        d.ctrl[:] = np.clip(ctrl, -lim, lim)
        state["sat"] += int(np.sum(np.abs(ctrl[15:29]) >= lim[15:29] - 1e-9))
        state["steps"] += 1
        mujoco.mj_step(m, d)
        state["n"] += 1

    for _ in range(round(3 / dt)):
        step([0, 0, 0])
    rest = d.xpos[pel][:2].copy()
    sat_rest = state["sat"] / (state["steps"] * 14)
    w, x, y, z = d.xquat[pel]
    yaw0 = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    p0, zmin = d.xpos[pel].copy(), 9.0
    for _ in range(round(4 / dt)):
        step([0.5, 0, 0])
        zmin = min(zmin, d.xpos[pel][2])
    dp = (d.xpos[pel] - p0)[:2]
    h = np.array([np.cos(yaw0), np.sin(yaw0)])
    w, x, y, z = d.xquat[pel]
    return {"mass_kg": round(float(m.body_mass.sum()), 2), "rest_drift_cm": round(float(np.linalg.norm(rest) * 100), 1),
            "arm_torque_saturated_at_rest": f"{sat_rest:.0%}", "forward_m": round(float(dp @ h), 2),
            "sideways_m": round(float(dp @ np.array([-h[1], h[0]])), 2),
            "heading_deg": round(float(np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) - yaw0)), 1),
            "min_pelvis_z": round(float(zmin), 3)}


if __name__ == "__main__":
    dt = float(sys.argv[1]) if len(sys.argv) > 1 else 0.002
    print(f"dt = {dt}   P2 needs forward 1.6-2.4 m and sideways <= 0.15 m in 4 s")
    for name, (path, ignore) in MODELS.items():
        print(f"{name:58s}", run(path, ignore, dt), flush=True)
