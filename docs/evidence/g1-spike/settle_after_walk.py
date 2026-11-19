"""G1 spike: does the unitree_rl_lab whole-body policy come to rest at zero command after walking?

Stand 2 s, walk 5 s at 0.5 m/s, then one way of stopping, then 15 s at zero command. Prints, for each second after the
stop: distance moved (m), yaw (deg) and direction of motion (deg). Reads MuJoCo state only as the spike's oracle.

Ways of stopping (argument 4):
  none     zero command only
  reset    also clear the policy's observation history and last action
  reverse  0.6 s at -0.3 m/s first
  support  a spring-damper on the pelvis (like unitree_mujoco's elastic band) while the joints ease to the default
           pose for 2 s (Unitree's FixStand), history cleared, then the support is released
"""

import collections
import json
import sys

import mujoco
import numpy as np
import onnxruntime as ort
import yaml

scene, onnx_path, cfg_path, mode = sys.argv[1:5]
cfg = yaml.safe_load(open(cfg_path))
jmap = np.array(cfg["joint_ids_map"])
kp_s, kd_s = np.array(cfg["stiffness"]), np.array(cfg["damping"])  # motor order
dflt = np.array(cfg["default_joint_pos"])  # policy order
off = np.array(cfg["actions"]["JointPositionAction"]["offset"])
m = mujoco.MjModel.from_xml_path(scene)
d = mujoco.MjData(m)
dt, dec, H = 0.005, 4, 5
m.opt.timestep = dt
sess = ort.InferenceSession(onnx_path)
iname = sess.get_inputs()[0].name
d.qpos[7 + jmap] = dflt
mujoco.mj_forward(m, d)
pelvis = m.body("pelvis").id
TERMS = ["w", "g", "c", "q", "dq", "a"]
hist = {k: collections.deque(maxlen=H) for k in TERMS}
state = {"act": np.zeros(29), "n": 0, "support": None}
tgt_s = np.zeros(29)
tgt_s[jmap] = dflt
stand_q = tgt_s.copy()


def grav(q):
    w, x, y, z = q
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])


def yaw():
    w, x, y, z = d.qpos[3:7]
    return float(np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


def step(cmd, fixstand=None):
    if fixstand is None and state["n"] % dec == 0:
        obs_t = {
            "w": d.qvel[3:6] * 0.2,
            "g": grav(d.qpos[3:7]),
            "c": np.array(cmd),
            "q": d.qpos[7:36][jmap] - dflt,
            "dq": d.qvel[6:35][jmap] * 0.05,
            "a": state["act"].copy(),
        }
        for key, v in obs_t.items():
            if not hist[key]:
                hist[key].extend([v] * H)
            else:
                hist[key].append(v)
        obs = np.concatenate([np.concatenate(list(hist[key])) for key in TERMS]).astype(np.float32)[None]
        state["act"] = sess.run(None, {iname: obs})[0][0]
        tgt_s[jmap] = state["act"] * 0.25 + off
    tgt = tgt_s if fixstand is None else fixstand
    tau = (tgt - d.qpos[7:36]) * kp_s - d.qvel[6:35] * kd_s
    d.ctrl[:] = np.clip(tau, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
    d.xfrc_applied[pelvis] = 0
    if state["support"] is not None:
        w, x, y, z = d.qpos[3:7]
        up = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
        force = 4000 * (state["support"] - d.xpos[pelvis]) - 400 * d.qvel[0:3]
        torque = 600 * np.cross(up, [0, 0, 1]) - 60 * d.qvel[3:6]
        d.xfrc_applied[pelvis] = np.r_[force, torque]
    mujoco.mj_step(m, d)
    state["n"] += 1


def reset_history():
    for k in hist:
        hist[k].clear()
    state["act"] = np.zeros(29)


for _ in range(int(2 / dt)):
    step([0, 0, 0])
for _ in range(int(5 / dt)):
    step([0.5, 0, 0])
if mode == "reset":
    reset_history()
elif mode == "reverse":
    for _ in range(int(0.6 / dt)):
        step([-0.3, 0, 0])
elif mode == "support":
    state["support"] = np.r_[d.xpos[pelvis][:2], 0.79]
    start_q = d.qpos[7:36].copy()
    for i in range(int(2 / dt)):
        a = min(1.0, i * dt / 1.5)
        step(None, fixstand=(1 - a) * start_q + a * stand_q)
    reset_history()
    state["support"] = None
seconds, zmin = [], 9.0
for _ in range(15):
    p0 = d.xpos[pelvis][:2].copy()
    for _ in range(int(1 / dt)):
        step([0, 0, 0])
        zmin = min(zmin, float(d.xpos[pelvis][2]))
    v = d.xpos[pelvis][:2] - p0
    seconds.append([round(float(np.linalg.norm(v)), 2), round(yaw()), round(float(np.degrees(np.arctan2(v[1], v[0]))))])
print(json.dumps({"mode": mode, "upright": zmin > 0.6, "per_second_m_yaw_heading": seconds}))
