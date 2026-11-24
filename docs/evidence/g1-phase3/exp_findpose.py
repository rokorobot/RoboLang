# ruff: noqa
import math, itertools, json, sys
import mujoco, numpy as np
from robolang_g1 import kinematics as K, assets, ops as OPS
from robolang_g1.sim import scene
from robolang_g1.sim.policy import load_deploy

cfg = load_deploy(assets.asset(*assets.POLICY_DIR, "params", "deploy.yaml"))
q_carry = np.zeros(29); q_carry[cfg["joint_ids_map"]] = cfg["default_joint_pos"]
q_carry[12:15] = 0.0
HALF = np.array([0.045, 0.045, 0.04])   # the crate (9 x 9 x 8 cm), 0.3 kg
DEG = 15
R_t = OPS._frame(math.radians(DEG))
GRASP_RISE = OPS.GRASP_RISE

# a private model with the object as a rigid box on the right wrist, in the pose a grasp at DEG would hold it
spec = scene.robot_spec(None)
wrist = spec.body("right_wrist_yaw_link")
off_world = np.array([0, 0, 0.01])   # the object's centre is 1 cm above the pinch point (GRASP_RISE) when grasped
centre_w = K.PINCH["right"] + R_t.T @ off_world
quat = np.zeros(4); mujoco.mju_mat2Quat(quat, R_t.T.flatten())
wrist.add_geom(name="held", type=mujoco.mjtGeom.mjGEOM_BOX, size=list(HALF), pos=list(centre_w), quat=list(quat),
               contype=0, conaffinity=0, group=5)
for side in ("left", "right"):
    spec.body(scene.HAND_BODY[side]).add_site(name=f"grasp_{side}", pos=list(K.PINCH[side]), size=[0.005, 0, 0])
model = spec.compile(); data = mujoco.MjData(model)
kin = K.Kinematics()
idx = scene.Robot.of(model)

def bodyname(g): return model.body(model.geom_bodyid[g]).name
right_arm = lambda n: n.startswith("right_shoulder") or n in ("right_elbow_link",) or n.startswith("right_wrist") or n.startswith("right_hand")
all_geoms = [g for g in range(model.ngeom) if model.geom_group[g] in (0, 1, 5) and bodyname(g) != "world"]
held = model.geom("held").id
body_geoms = [g for g in all_geoms if not right_arm(bodyname(g)) and g != held and model.geom_group[g] == 0 and model.geom_contype[g] == 1]
arm_geoms = [g for g in all_geoms if right_arm(bodyname(g)) and g != held and model.geom_contype[g] in (1, 4) and (model.geom_group[g] == 0 or model.geom_type[g] == 3)]
fromto = np.zeros(6)
def mind(gs1, gs2):
    best = (9.0, None)
    for a in gs1:
        for b in gs2:
            d = mujoco.mj_geomDistance(model, data, a, b, 0.5, fromto)
            if d < best[0]: best = (d, (bodyname(a), bodyname(b)))
    return best

def evaluate(q):
    d = data
    d.qpos[0:3] = 0; d.qpos[3:7] = (1, 0, 0, 0); d.qpos[idx.qadr] = q
    mujoco.mj_kinematics(model, d); mujoco.mj_comPos(model, d)
    obj_body = mind([held], body_geoms)
    arm_body = mind([g for g in arm_geoms if bodyname(g) not in ("right_shoulder_pitch_link", "right_shoulder_roll_link", "right_shoulder_yaw_link")], body_geoms)
    # marker of the object's top face (world up in the pelvis frame), seen from the camera
    Rc, pc = kin.camera_pose(q)
    centre = d.geom_xpos[held].copy(); Rg = d.geom_xmat[held].reshape(3, 3)
    top = Rg @ np.array([0, 0, 1.0])
    marker = centre + Rg @ np.array([0, 0, HALF[2] + 0.0006])
    v = marker - pc
    cam_axis, up, right = -Rc[:, 2], Rc[:, 1], Rc[:, 0]
    depth = v @ cam_axis
    vert = math.degrees(math.atan2(v @ up, depth)); horiz = math.degrees(math.atan2(v @ right, depth))
    # marker normal vs the line to the camera
    to_cam = -v / np.linalg.norm(v)
    tilt = math.degrees(math.acos(max(-1, min(1, top @ to_cam))))
    return dict(obj_body=obj_body, arm_body=arm_body, marker=marker.round(3).tolist(), vert=vert, horiz=horiz, depth=depth, normal_vs_camera_deg=tilt)

if __name__ == "__main__":
    print("current carry pose:", evaluate(q_carry))
    ready = OPS.G1Operations._ready_pose  # (bare) needs self.q_carry
    class B: pass
    b = B(); b.q_carry = q_carry
    seed_ready = OPS.G1Operations._ready_pose(b, "right")
    rng = np.random.default_rng(5)
    out = []
    for x, y, z in itertools.product(np.arange(0.14, 0.31, 0.04), (-0.05, -0.10, -0.15, -0.20), np.arange(0.00, 0.21, 0.04)):
        p = np.array([x, y, z])
        for seed in (seed_ready, q_carry):
            q, info = kin.solve("right", p, R_t, seed, posture=seed_ready, rot_weight=0.5, iters=250, waist_max=0.0)
            if q is not None: break
        if q is None: continue
        e = evaluate(q)
        e.update(pinch=p.round(3).tolist(), ik_err=round(info["position_error_m"], 4), q=q.round(4).tolist())
        out.append(e)
    print(len(out), "reachable candidates")
    def view(e): return max(abs(e["vert"]) / 29, abs(e["horiz"]) / 36.5)
    good = [e for e in out if e["obj_body"][0] >= 0.03 and e["arm_body"][0] >= 0.01 and view(e) <= 0.7 and e["depth"] >= 0.25]
    good.sort(key=lambda e: (e["pinch"][0] + 0.5 * max(e["pinch"][2], 0)))   # compact first: near the body, low
    for e in good[:12]:
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in e.items() if k != "q"})
    json.dump(good[:12], open("/tmp/exp/pose_candidates.json", "w"))

    print("---- stricter: object >= 5 cm and arm >= 4 cm from the body, marker within 70% of the field")
    strict = [e for e in out if e["obj_body"][0] >= 0.05 and e["arm_body"][0] >= 0.04 and view(e) <= 0.7 and e["depth"] >= 0.25]
    strict.sort(key=lambda e: (e["pinch"][0], e["pinch"][2]))
    for e in strict[:8]:
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in e.items() if k != "q"})
    json.dump(strict[:8], open("/tmp/exp/pose_candidates.json", "w"))

    print("---- closest to the walking controller's own arm pose (joint distance), marker within 90% of the field, object >= 3 cm clear")
    near = [e for e in out if e["obj_body"][0] >= 0.03 and e["arm_body"][0] >= 0.01 and view(e) <= 0.9 and e["depth"] >= 0.25]
    for e in near:
        e["dq_rms"] = float(np.sqrt(np.mean((np.array(e["q"])[15:29] - q_carry[15:29]) ** 2)))
        e["dq_right_max"] = float(np.max(np.abs(np.array(e["q"])[22:29] - q_carry[22:29])))
    near.sort(key=lambda e: e["dq_rms"])
    for e in near[:6]:
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in e.items() if k != "q"})
    p1 = next((e for e in out if e["pinch"] == [0.22, -0.15, 0.04]), None)
    if p1: print("P1 deviation: rms", float(np.sqrt(np.mean((np.array(p1["q"])[15:29] - q_carry[15:29]) ** 2))), "right-arm max", float(np.max(np.abs(np.array(p1["q"])[22:29] - q_carry[22:29]))))
    json.dump(near[:6], open("/tmp/exp/pose_near.json", "w"))
