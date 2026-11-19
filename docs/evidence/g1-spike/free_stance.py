"""G1 phase 1 research: can the whole-body policy stand free (unsupported) after walking?

Stand 4 s from rest, walk 5 s at 0.5 m/s, stop in one of these ways, then 10 s at zero command, unsupported. Prints the
distance moved and the torso sway over the last 8 s. Reads MuJoCo state only as the experiment's oracle.

  none     zero command only
  ramp     command ramped from 0.5 to 0 m/s over 3 s
  spring   2 s at zero command, then a spring-damper on the pelvis (gains from GAINS="kp,kd,kr"), faded in over 1 s;
           legs eased to the standing pose (Unitree's FixStand) for 3 s; history cleared; 1 s of policy; faded out
  weld     2 s at zero command, then the pelvis welded (a MuJoCo constraint) to a mocap body at its current pose; the
           mocap moved over 1 s to upright at standing height; legs eased to the standing pose for 3 s; history cleared;
           weld released: the same still state the policy stands from at the start
"""

import collections
import json
import os
import sys

import mujoco
import numpy as np
import onnxruntime as ort
import yaml

scene, onnx_path, cfg_path, mode = sys.argv[1:5]
KP, KD, KR = (float(x) for x in os.environ.get("GAINS", "300,200,200").split(","))
cfg = yaml.safe_load(open(cfg_path))
jmap = np.array(cfg["joint_ids_map"])
kp_s, kd_s = np.array(cfg["stiffness"]), np.array(cfg["damping"])  # motor order
dflt = np.array(cfg["default_joint_pos"])  # policy order
off = np.array(cfg["actions"]["JointPositionAction"]["offset"])

spec = mujoco.MjSpec.from_file(scene)
spec.worldbody.add_body(name="support", mocap=True)
weld = spec.add_equality(type=mujoco.mjtEq.mjEQ_WELD, name1="pelvis", name2="support")
weld.objtype = mujoco.mjtObj.mjOBJ_BODY
weld.data[:11] = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1]  # anchor, relative position, relative quaternion, torque scale
weld.active = False
m = spec.compile()
d = mujoco.MjData(m)
dt, dec, H = 0.005, 4, 5
m.opt.timestep = dt
sess = ort.InferenceSession(onnx_path)
iname = sess.get_inputs()[0].name
d.qpos[7 + jmap] = dflt
mujoco.mj_forward(m, d)
pel, tor = m.body("pelvis").id, m.body("torso_link").id
TERMS = ["w", "g", "c", "q", "dq", "a"]
hist = {k: collections.deque(maxlen=H) for k in TERMS}
st = {"act": np.zeros(29), "n": 0, "spring": None, "w": 0.0}
tgt = np.zeros(29)
tgt[jmap] = dflt
stand_q = tgt.copy()


def grav(q):
    w, x, y, z = q
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])


def step(cmd, fix=None):
    if fix is None and st["n"] % dec == 0:
        obs_t = {
            "w": d.qvel[3:6] * 0.2,
            "g": grav(d.qpos[3:7]),
            "c": np.array(cmd, float),
            "q": d.qpos[7:36][jmap] - dflt,
            "dq": d.qvel[6:35][jmap] * 0.05,
            "a": st["act"].copy(),
        }
        for k, v in obs_t.items():
            if not hist[k]:
                hist[k].extend([v] * H)
            else:
                hist[k].append(v)
        obs = np.concatenate([np.concatenate(list(hist[k])) for k in TERMS]).astype(np.float32)[None]
        st["act"] = sess.run(None, {iname: obs})[0][0]
        tgt[jmap] = st["act"] * 0.25 + off
    target = tgt if fix is None else fix
    tau = (target - d.qpos[7:36]) * kp_s - d.qvel[6:35] * kd_s
    d.ctrl[:] = np.clip(tau, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
    d.xfrc_applied[pel] = 0
    if st["spring"] is not None and st["w"] > 0:
        w, x, y, z = d.qpos[3:7]
        up = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
        force = np.zeros(3)
        force[:2] = KP * (st["spring"][:2] - d.xpos[pel][:2]) - KD * d.qvel[0:2]
        torque = KR * np.cross(up, [0, 0, 1]) - KR / 10 * d.qvel[3:6]
        torque[2] = 0
        d.xfrc_applied[pel] = st["w"] * np.r_[force, torque]
    mujoco.mj_step(m, d)
    st["n"] += 1


def run(sec, cmd, fix=None):
    for _ in range(int(sec / dt)):
        step(cmd, fix)


def ease_legs_to_stand(sec):
    q0 = d.qpos[7:36].copy()
    for i in range(int(sec / dt)):
        a = min(1.0, i * dt / (sec * 2 / 3))
        step(None, (1 - a) * q0 + a * stand_q)
    for k in hist:
        hist[k].clear()
    st["act"] = np.zeros(29)


run(4, [0, 0, 0])
z_stand = float(d.xpos[pel][2])
run(5, [0.5, 0, 0])
p_stop = d.xpos[pel].copy()
if mode == "ramp":
    for i in range(int(3 / dt)):
        step([0.5 * (1 - i * dt / 3), 0, 0])
elif mode == "spring":
    run(2, [0, 0, 0])
    st["spring"] = d.xpos[pel].copy()
    for i in range(int(1 / dt)):
        st["w"] = (i + 1) * dt
        step([0, 0, 0])
    ease_legs_to_stand(3)
    run(1, [0, 0, 0])
    for i in range(int(2 / dt)):
        st["w"] = 1 - i * dt / 2
        step([0, 0, 0])
    st["spring"], st["w"] = None, 0.0
elif mode == "weld":
    run(2, [0, 0, 0])
    d.mocap_pos[0], d.mocap_quat[0] = d.xpos[pel], d.xquat[pel]
    d.eq_active[0] = 1
    w, x, y, z = d.xquat[pel]
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    q_up = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
    p0, q0 = d.mocap_pos[0].copy(), d.mocap_quat[0].copy()
    p1 = np.r_[p0[:2], z_stand]
    for i in range(int(1 / dt)):
        a = (i + 1) * dt
        d.mocap_pos[0] = (1 - a) * p0 + a * p1
        qa = (1 - a) * q0 + a * q_up
        d.mocap_quat[0] = qa / np.linalg.norm(qa)
        step([0, 0, 0], fix=d.qpos[7:36].copy())
    ease_legs_to_stand(3)
    d.eq_active[0] = 0
p_rel = d.xpos[pel].copy()
trace, zmin = [], 9.0
for i in range(int(10 / dt)):
    step([0, 0, 0])
    zmin = min(zmin, float(d.xpos[pel][2]))
    if i * dt >= 2:
        trace.append(d.xpos[tor].copy())
trace = np.array(trace)
print(
    json.dumps(
        {
            "mode": mode,
            "moved_while_stopping_m": np.round(p_rel[:2] - p_stop[:2], 3).tolist(),
            "moved_in_10s_m": np.round(d.xpos[pel][:2] - p_rel[:2], 3).tolist(),
            "torso_sway_cm_last_8s": np.round((trace.max(0) - trace.min(0)) * 100, 1).tolist(),
            "upright": zmin > 0.6,
        }
    )
)
