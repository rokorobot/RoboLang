"""G1 spike: 29-DoF G1 in MuJoCo on CPU.

Walk with unitree_rl_gym's policy, render the head camera, detect an ArUco marker.
"""

import os
import sys
import time
import json
import numpy as np
import mujoco
import torch
import cv2

scene, policy_path, outdir = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(outdir, exist_ok=True)

# ArUco texture for a box marker
aruco = cv2.aruco
dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
m_img = aruco.generateImageMarker(dic, 7, 200)
m_img = cv2.copyMakeBorder(m_img, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
tex = os.path.join(outdir, "marker7.png")
cv2.imwrite(tex, cv2.cvtColor(m_img, cv2.COLOR_GRAY2BGR))

spec = mujoco.MjSpec.from_file(scene)
torso = spec.body("torso_link") or spec.body("pelvis")
ARMS = os.environ.get("ARMS", "fold")
# head camera, approx. G1 d435 mount (pitched down); spike only
cam = torso.add_camera(name="head", pos=[0.0576, 0.0175, 0.43], fovy=58)
R = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)  # columns: cam x, y, z in body frame (looks along +x, up +z)
qc = np.zeros(4)
mujoco.mju_mat2Quat(qc, R.flatten())
qp = np.zeros(4)
mujoco.mju_axisAngle2Quat(qp, [0, 1, 0], 0.83)  # pitch down
qq = np.zeros(4)
mujoco.mju_mulQuat(qq, qp, qc)
cam.quat = qq
spec.add_texture(name="m7", type=mujoco.mjtTexture.mjTEXTURE_2D, file=tex)
mat = spec.add_material(name="m7")
mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "m7"
w = spec.worldbody
table = w.add_body(name="table", pos=[2.6, 0, 0.35])
table.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.3, 0.4, 0.35], rgba=[0.6, 0.45, 0.3, 1])
box = w.add_body(name="box1", pos=[2.45, 0, 0.75])
box.add_freejoint()
box.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.05, 0.05, 0.05], rgba=[0.8, 0.2, 0.2, 1], mass=0.2)
box.add_geom(
    type=mujoco.mjtGeom.mjGEOM_BOX,
    size=[0.045, 0.045, 0.001],
    pos=[0, 0, 0.0505],
    material="m7",
    contype=0,
    conaffinity=0,
)
m = spec.compile()
d = mujoco.MjData(m)

dt, dec = 0.002, 10
m.opt.timestep = dt
legs_kp = np.array([100, 100, 100, 150, 40, 40] * 2, np.float32)
legs_kd = np.array([2, 2, 2, 4, 2, 2] * 2, np.float32)
default = np.array([-0.1, 0, 0, 0.3, -0.2, 0] * 2, np.float32)
up_kp = np.array([200, 200, 200] + [60] * 14, np.float32)
up_kd = np.array([5, 5, 5] + [2] * 14, np.float32)
arm_pose = np.zeros(17, np.float32)
arm_pose[[3, 10]] = 0.3
arm_pose[[4]] = 0.25
arm_pose[[11]] = -0.25
arm_pose[[6, 13]] = 1.0  # sh pitch, roll, elbow
policy = torch.jit.load(policy_path)
nb = m.nbody
if ARMS == "zero":
    arm_pose[:] = 0
d.qpos[7:19] = default
if m.nu > 12:
    d.qpos[19:36] = arm_pose
mujoco.mj_forward(m, d)


def grav(qt):
    w_, x, y, z = qt
    return np.array([2 * (-z * x + w_ * y), -2 * (z * y + w_ * x), 1 - 2 * (w_ * w_ + z * z)])


obs = np.zeros(47, np.float32)
action = np.zeros(12, np.float32)
target = default.copy()
pel = m.body("pelvis").id
log = []
counter = 0
phases = [(2.0, [0, 0, 0]), (5.0, [0.5, 0, 0]), (2.0, [0, 0, 0])]
t0 = time.perf_counter()
fallen = False
for dur, c in phases:
    cmd = np.array(c, np.float32)
    for _ in range(int(dur / dt)):
        tau = np.zeros(m.nu)
        tau[:12] = (target - d.qpos[7:19]) * legs_kp - d.qvel[6:18] * legs_kd
        if m.nu > 12:
            tau[12:] = (arm_pose - d.qpos[19:36]) * up_kp - d.qvel[18:35] * up_kd
        d.ctrl[:] = np.clip(tau, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
        mujoco.mj_step(m, d)
        counter += 1
        if counter % dec == 0:
            ph = (counter * dt) % 0.8 / 0.8
            obs[:3] = d.qvel[3:6] * 0.25
            obs[3:6] = grav(d.qpos[3:7])
            obs[6:9] = cmd * [2, 2, 0.25]
            obs[9:21] = d.qpos[7:19] - default
            obs[21:33] = d.qvel[6:18] * 0.05
            obs[33:45] = action
            obs[45:47] = [np.sin(2 * np.pi * ph), np.cos(2 * np.pi * ph)]
            with torch.no_grad():
                action = policy(torch.from_numpy(obs).unsqueeze(0)).numpy().squeeze()
            target = action * 0.25 + default
        if counter % 250 == 0:
            log.append([round(counter * dt, 2), *np.round(d.xpos[pel], 3).tolist()])
        if d.xpos[pel][2] < 0.45:
            fallen = True
phys_wall = time.perf_counter() - t0
sim_s = counter * dt

# render head camera
r = mujoco.Renderer(m, 480, 640)
t1 = time.perf_counter()
N = 10
for _ in range(N):
    r.update_scene(d, camera="head")
    img = r.render()
render_ms = (time.perf_counter() - t1) / N * 1000
cv2.imwrite(os.path.join(outdir, "head.png"), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
r2 = mujoco.Renderer(m, 480, 640)
cam3 = mujoco.MjvCamera()
cam3.lookat[:] = [1.4, 0, 0.6]
cam3.distance = 3.2
cam3.azimuth = 120
cam3.elevation = -15
r2.update_scene(d, camera=cam3)
cv2.imwrite(os.path.join(outdir, "overview.png"), cv2.cvtColor(r2.render(), cv2.COLOR_RGB2BGR))
det = aruco.ArucoDetector(dic, aruco.DetectorParameters())
corners, ids, _ = det.detectMarkers(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
print(
    json.dumps(
        {
            "mujoco": mujoco.__version__,
            "gl": os.environ.get("MUJOCO_GL"),
            "sim_s": sim_s,
            "phys_wall_s": round(phys_wall, 2),
            "realtime_factor": round(sim_s / phys_wall, 2),
            "fallen": fallen,
            "pelvis_final": np.round(d.xpos[pel], 3).tolist(),
            "render_ms_640x480": round(render_ms, 1),
            "aruco_ids": None if ids is None else ids.flatten().tolist(),
            "trace": log[::2],
        },
        indent=1,
    )
)
