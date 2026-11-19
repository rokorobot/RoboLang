"""G1 spike: unitree_rl_lab G1-29dof velocity policy (ONNX) in MuJoCo.

Stand, walk, stand still for 10 s, reach with the right arm while standing, then walk with both arms held in a
carry pose by an override (as `rt/arm_sdk` would). Reads MuJoCo state only as an oracle for the spike itself.

Port note: in `deploy.yaml`, `stiffness` and `damping` are in motor (SDK) order, applied to `motor_cmd[i]` by
`State_RLBase.h`; `default_joint_pos`, actions and observations are in policy order, mapped by `joint_ids_map`
(policy index i -> motor `joint_ids_map[i]`). The first port of 2026-11-19 used policy order for the gains: it fell.
"""

import collections
import json
import os
import sys
import time

import mujoco
import numpy as np
import onnxruntime as ort
import yaml

scene, onnx_path, cfg_path = sys.argv[1:4]
cfg = yaml.safe_load(open(cfg_path))
jmap = np.array(cfg["joint_ids_map"])
kp_s = np.array(cfg["stiffness"])  # motor order
kd_s = np.array(cfg["damping"])  # motor order
dflt = np.array(cfg["default_joint_pos"])  # policy order
off = np.array(cfg["actions"]["JointPositionAction"]["offset"])
m = mujoco.MjModel.from_xml_path(scene)
d = mujoco.MjData(m)
dt = float(os.environ.get("DT", "0.002"))  # 0.005 makes the explicit arm PD chatter (dt * kd / I > 2)
dec = round(0.02 / dt)  # policy at 50 Hz
m.opt.timestep = dt
sess = ort.InferenceSession(onnx_path)
iname = sess.get_inputs()[0].name
d.qpos[7 + jmap] = dflt
mujoco.mj_forward(m, d)


def grav(q):
    w, x, y, z = q
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])


H = 5
TERMS = ["w", "g", "c", "q", "dq", "a"]
hist = {k: collections.deque(maxlen=H) for k in TERMS}
act = np.zeros(29)
tgt_s = np.zeros(29)
tgt_s[jmap] = dflt
torso = m.body("torso_link").id
pelvis = m.body("pelvis").id
REACH = {22: -0.9, 25: 0.6}  # motor index: right shoulder pitch, right elbow (forward reach)
ARMS = list(range(15, 29))  # motor indices of both arms
carry = np.zeros(29)
carry[jmap] = dflt  # the trained arm pose, held by the override


def terms(cmd):
    return {
        "w": d.qvel[3:6] * 0.2,
        "g": grav(d.qpos[3:7]),
        "c": np.array(cmd),
        "q": d.qpos[7:36][jmap] - dflt,
        "dq": d.qvel[6:35][jmap] * 0.05,
        "a": act.copy(),
    }


PHASES = [  # name, seconds, command (vx, vy, yaw rate), arm override, seconds ignored before measuring sway
    ("stand0", 3, [0, 0, 0], None, 1),
    ("walk", 5, [0.5, 0, 0], None, 1),
    ("stand10", 10, [0, 0, 0], None, 2),
    ("reach", 5, [0, 0, 0], "reach", 2),
    ("carry_walk", 5, [0.3, 0, 0], "carry", 1),
    ("carry_stop", 4, [0, 0, 0], "carry", 2),
]
res = {}
n = 0
t0 = time.perf_counter()
for name, dur, cmd, arms, settle in PHASES:
    tr, zmin, start = [], 9.0, d.xpos[pelvis][:2].copy()
    for k in range(int(dur / dt)):
        if n % dec == 0:
            for key, v in terms(cmd).items():
                if not hist[key]:
                    hist[key].extend([v] * H)
                else:
                    hist[key].append(v)
            obs = np.concatenate([np.concatenate(list(hist[key])) for key in TERMS]).astype(np.float32)[None]
            act = sess.run(None, {iname: obs})[0][0]
            tgt_s[jmap] = act * 0.25 + off
            if arms == "reach":
                for j, v in REACH.items():
                    tgt_s[j] = v
            elif arms == "carry":
                tgt_s[ARMS] = carry[ARMS]
        tau = (tgt_s - d.qpos[7:36]) * kp_s - d.qvel[6:35] * kd_s
        d.ctrl[:] = np.clip(tau, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
        mujoco.mj_step(m, d)
        n += 1
        zmin = min(zmin, float(d.xpos[pelvis][2]))
        if k * dt >= settle:
            tr.append(d.xpos[torso].copy())
    tr = np.array(tr)
    res[name] = {
        "moved_xy_m": np.round(d.xpos[pelvis][:2] - start, 3).tolist(),
        "torso_sway_cm_xyz": np.round((tr.max(0) - tr.min(0)) * 100, 1).tolist(),
        "pelvis_min_z": round(zmin, 3),
        "upright": zmin > 0.6,
    }
wall = time.perf_counter() - t0
print(
    json.dumps(
        {"phases": res, "sim_s": round(n * dt, 1), "wall_s": round(wall, 2), "realtime_factor": round(n * dt / wall, 1)}
    )
)
