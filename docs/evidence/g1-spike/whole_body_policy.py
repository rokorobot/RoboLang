"""G1 spike: unitree_rl_lab G1-29dof velocity policy (ONNX) in MuJoCo.

Stand, walk, stand still, then reach with the right arm while standing.
"""

import sys
import json
import collections
import numpy as np
import mujoco
import onnxruntime as ort
import yaml

scene, onnx_path, cfg_path = sys.argv[1:4]
cfg = yaml.safe_load(open(cfg_path))
jmap = np.array(cfg["joint_ids_map"])
kp = np.array(cfg["stiffness"])
kd = np.array(cfg["damping"])
dflt = np.array(cfg["default_joint_pos"])
off = np.array(cfg["actions"]["JointPositionAction"]["offset"])
m = mujoco.MjModel.from_xml_path(scene)
d = mujoco.MjData(m)
dt = 0.005
m.opt.timestep = dt
dec = 4
sess = ort.InferenceSession(onnx_path)
iname = sess.get_inputs()[0].name
print("onnx input", sess.get_inputs()[0].shape, "output", sess.get_outputs()[0].shape, file=sys.stderr)
# SDK-ordered arrays from policy-ordered ones
kp_s = np.zeros(29)
kd_s = np.zeros(29)
kp_s[jmap] = kp
kd_s[jmap] = kd
d.qpos[7 + jmap] = dflt
mujoco.mj_forward(m, d)


def grav(q):
    w, x, y, z = q
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])


H = 5
hist = {k: collections.deque(maxlen=H) for k in ["w", "g", "c", "q", "dq", "a"]}
act = np.zeros(29)
tgt_s = np.zeros(29)
tgt_s[jmap] = dflt
torso = m.body("torso_link").id
R_ARM = {22: -0.9, 25: 0.6}  # SDK idx: right shoulder pitch, right elbow (approx reach forward)


def terms(cmd):
    qp = d.qpos[7:36][jmap]
    qv = d.qvel[6:35][jmap]
    return {
        "w": d.qvel[3:6] * 0.2,
        "g": grav(d.qpos[3:7]),
        "c": np.array(cmd),
        "q": qp - dflt,
        "dq": qv * 0.05,
        "a": act.copy(),
    }


res = {}
n = 0

PHASES = [
    ("stand0", 4, [0, 0, 0], False),
    ("walk", 4, [0.5, 0, 0], False),
    ("stand", 6, [0, 0, 0], False),
    ("reach", 5, [0, 0, 0], True),
]
Z = []
for name, dur, cmd, reach in PHASES:
    tr = []
    for _ in range(int(dur / dt)):
        if n % dec == 0:
            t = terms(cmd)
            for k, v in t.items():
                if not hist[k]:
                    [hist[k].append(v) for _ in range(H)]
                else:
                    hist[k].append(v)
            obs = np.concatenate([np.concatenate(list(hist[k])) for k in ["w", "g", "c", "q", "dq", "a"]]).astype(
                np.float32
            )[None]
            act = sess.run(None, {iname: obs})[0][0]
            tgt_s[jmap] = act * 0.25 + off
            if reach:
                for j, v in R_ARM.items():
                    tgt_s[j] = v
        tau = (tgt_s - d.qpos[7:36]) * kp_s - d.qvel[6:35] * kd_s
        d.ctrl[:] = np.clip(tau, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
        mujoco.mj_step(m, d)
        n += 1
        tr.append(d.xpos[torso].copy())
        if n % 50 == 0:
            Z.append(round(float(d.xpos[1][2]), 2))
    tr = np.array(tr[len(tr) // 3 :])
    res[name] = {
        "torso_end": np.round(tr[-1], 3).tolist(),
        "sway_cm_last_2/3": np.round((tr.max(0) - tr.min(0)) * 100, 1).tolist(),
        "upright": bool(d.xpos[1][2] > 0.6),
    }
print(json.dumps(res))
print("pelvis z every 0.25 s:", Z[:40])
