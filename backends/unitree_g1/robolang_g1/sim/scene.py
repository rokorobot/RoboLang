"""Build the MuJoCo scene: the G1 with Dex3-1 hands, the head camera, and a site (floor, tables, objects, markers).

The robot is Unitree's `g1_29dof_with_hand.xml` (see docs/evidence/g1-phase1-harness.md for why), normalised with
MjSpec so the rest of the backend can rely on one layout:

* 43 torque actuators: the 29 body joints in Unitree's SDK order 0-28 (as in `unitree_mujoco`'s `g1_29dof.xml`),
  then the left hand (7, Dex3 order) and the right hand (7). `data.ctrl` is indexed that way.
* joint armature, damping and friction as in `unitree_mujoco`'s G1 (the model the policy is deployed on in simulation).
* a pelvis IMU (quaternion, gyro, accelerometer), a head camera `head` on the torso, a mocap body `support` welded
  (inactive) to the pelvis for the supported stance, and one inactive weld per object and hand for `attach`.

Qpos and dof addresses are looked up by joint name (`Robot`); nothing assumes the free joint's 7 and the 29 body
joints come first, because the hand model interleaves the hands with the arms.

A site is a small JSON file (`SITE_SCHEMA`; an example is `sites/phase1.json`): the floor, tables as boxes, objects
with a size and an ArUco 4x4_50 marker, and stations (a standing pose and its fiducial marker). All sizes are MuJoCo
half-extents in metres, poses are in the world frame (x forward at yaw 0, z up).
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .. import assets

SITE_SCHEMA = "robolang-g1-site/1"
TIMESTEP = 0.002  # 500 Hz, the robot's own low-level rate; see the note in standin.py on why not 0.005
CAMERA_SIZE = (640, 480)  # width, height
CAMERA_FOVY = 58.0
# The G1's head camera (an Intel RealSense D435) is a fixed joint on the torso link in Unitree's with-hand description:
# unitree_rl_gym g1_29dof_with_hand.urdf:623-624: origin xyz 0.0576235 0.01753 0.41987, rpy 0 0.8307767 0 (pitched
# down).
# A MuJoCo camera looks along its -z with +y up, so the orientation below builds "looks along the torso's +x, up is +z"
# and then tilts it down about +y.
CAMERA_POS = (0.0576235, 0.01753, 0.41987)
CAMERA_PITCH = 0.8307767
MARKER_BORDER = 0.2  # white border around an ArUco marker, as a fraction of its side
STAND_HEIGHT = 0.788  # pelvis height of the standing pose (measured: the policy's standing pose on this model)
# Geom groups. The robot's meshes come twice (a visual copy in group 1 and a collision copy in group 0); the camera
# draws group 1 only, which halves the triangles it has to rasterise on the CPU. The world is in group 1 too.
VISIBLE = 1
HIDDEN = 2  # robot parts a head camera never sees (it sits inside the head): the camera draws neither group 0 nor 2
NOT_SEEN = re.compile(r"hip|knee|ankle|pelvis|waist|^torso_link$")
FINGER_LINK = re.compile(r"_hand_(thumb|middle|index)_\d_link$")
FINGER_RADIUS, FINGER_TIP = 0.009, 0.04  # the finger capsules: radius, and how far the last link of a finger reaches

BODY_JOINTS = [
    f"{side}_{j}_joint"
    for side, names in (
        ("left", ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")),
        ("right", ("hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll")),
    )
    for j in names
] + ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"] + [
    f"{side}_{j}_joint"
    for side in ("left", "right")
    for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
]
# Dex3-1 joint order (7 per hand), as in Unitree's HandCmd_/HandState_ motor indices.
HAND_JOINTS = {
    "left": [f"left_hand_{j}_joint" for j in ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0",
                                                "index_1")],
    "right": [f"right_hand_{j}_joint" for j in ("thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0",
                                                  "middle_1")],
}
HAND_BODY = {"left": "left_wrist_yaw_link", "right": "right_wrist_yaw_link"}  # the palm geoms live in the wrist link
N_BODY = 29
HAND_ACT = {"left": N_BODY, "right": N_BODY + 7}  # first ctrl index of each hand
ARM_JOINTS = range(12, 29)  # joints Unitree's arm_sdk can take over (waist and both arms)
FACES = {  # face of an object box -> (outward normal, rotation that makes the marker plate's +z that normal)
    "top": ((0, 0, 1), (1, 0, 0, 0)),
    "-x": ((-1, 0, 0), (0.7071068, 0, -0.7071068, 0)),
    "+x": ((1, 0, 0), (0.7071068, 0, 0.7071068, 0)),
    "-y": ((0, -1, 0), (0.7071068, 0.7071068, 0, 0)),
    "+y": ((0, 1, 0), (0.7071068, -0.7071068, 0, 0)),
}


class SiteError(ValueError):
    """The site file is not valid; the message names the entry and the field."""


def load_site(site) -> dict:
    """Read and check a site (a path or an already parsed dict); returns it with defaults filled in."""
    data = json.loads(Path(site).read_text()) if not isinstance(site, dict) else dict(site)
    if data.get("schema") != SITE_SCHEMA:
        raise SiteError(f"schema must be {SITE_SCHEMA!r}")
    names: set[tuple[str, str]] = set()

    def need(entry, field, size=None, where=""):
        value = entry.get(field)
        if value is None:
            raise SiteError(f"{where}: missing {field!r}")
        if size is not None and (not isinstance(value, list) or len(value) != size
                                 or not all(isinstance(x, (int, float)) for x in value)):
            raise SiteError(f"{where}: {field!r} must be a list of {size} numbers")
        return value

    def name_of(entry, kind):
        name = entry.get("name")
        if not isinstance(name, str) or not name or (kind, name) in names:
            raise SiteError(f"{kind}: missing or duplicate name {name!r}")
        names.add((kind, name))
        return f"{kind} {name}"

    def check_marker(marker, where, size_required):
        mid = need(marker, "id", where=where)
        if not isinstance(mid, int) or not 0 <= mid < 50:
            raise SiteError(f"{where}: marker id must be an integer 0-49 (ArUco 4x4_50)")
        if size_required and not (isinstance(marker.get("size"), (int, float)) and marker["size"] > 0):
            raise SiteError(f"{where}: marker needs a positive 'size' (side of the black square, m)")

    data.setdefault("name", "site")
    data.setdefault("floor", {})
    data.setdefault("robot", {})
    for kind in ("tables", "objects", "stations"):
        data.setdefault(kind, [])
    start = data["robot"].setdefault("start", [0.0, 0.0, 0.0])
    if not isinstance(start, list) or len(start) != 3:
        raise SiteError("robot.start must be [x, y, yaw]")
    for t in data["tables"]:
        where = name_of(t, "table")
        need(t, "pos", 3, where)
        need(t, "half_size", 3, where)
    for o in data["objects"]:
        where = name_of(o, "object")
        need(o, "pos", 3, where)
        need(o, "half_size", 3, where)
        if not isinstance(o.get("mass", 0.2), (int, float)) or o.get("mass", 0.2) <= 0:
            raise SiteError(f"{where}: mass must be positive")
        marker = o.get("marker")
        if marker is not None:
            check_marker(marker, where, False)
            if marker.setdefault("face", "-x") not in FACES:
                raise SiteError(f"{where}: marker face must be one of {sorted(FACES)}")
            # the plate (marker plus its white border) fills the smaller side of the face
            plate = float(min(np.delete(o["half_size"], int(np.argmax(np.abs(FACES[marker["face"]][0]))))))
            marker.setdefault("size", 2 * plate / (1 + 2 * MARKER_BORDER))
    for s in data["stations"]:
        where = name_of(s, "station")
        need(s, "pose", 3, where)
        marker = need(s, "marker", where=where)
        check_marker(marker, where, True)
        need(marker, "pos", 3, where)
        need(marker, "facing", where=where)
    return data


def marker_png(marker_id: int) -> bytes:
    """An ArUco 4x4_50 marker with a white border, as PNG bytes (the texture of a marker plate)."""
    import cv2

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    side = 200
    image = cv2.aruco.generateImageMarker(dictionary, marker_id, side)
    border = int(side * MARKER_BORDER)
    image = cv2.copyMakeBorder(image, border, border, border, border, cv2.BORDER_CONSTANT, value=255)
    return cv2.imencode(".png", image)[1].tobytes()


def _quat_mul(a, b):
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.asarray(a, float), np.asarray(b, float))
    return out


def _yaw_quat(yaw: float):
    return np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])


def head_camera_quat() -> np.ndarray:
    """Orientation of the head camera in the torso frame (MuJoCo cameras look along their -z with +y up)."""
    r = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)  # columns: camera x, y, z in the torso frame
    q_base = np.zeros(4)
    mujoco.mju_mat2Quat(q_base, r.flatten())
    q_pitch = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q_pitch, [0, 1, 0], CAMERA_PITCH)
    return _quat_mul(q_pitch, q_base)


def camera_intrinsics() -> tuple[float, float, float, float]:
    """fx, fy, cx, cy in pixels (square pixels, principal point at the image centre)."""
    w, h = CAMERA_SIZE
    f = (h / 2) / np.tan(np.radians(CAMERA_FOVY) / 2)
    return f, f, w / 2, h / 2


def _add_marker_plate(spec, body, name, marker_id, plate_half, pos, quat):
    """A thin textured plate (visual only) whose +z face carries the marker."""
    spec.assets[f"{name}.png"] = marker_png(marker_id)
    spec.add_texture(name=name, type=mujoco.mjtTexture.mjTEXTURE_2D, file=f"{name}.png")
    material = spec.add_material(name=name)
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = name
    material.specular = 0
    material.shininess = 0
    body.add_geom(name=f"{name}_plate", type=mujoco.mjtGeom.mjGEOM_BOX, size=[plate_half, plate_half, 0.0005],
                  pos=list(pos), quat=list(quat), material=name, contype=0, conaffinity=0, mass=0.0, density=0.0,
                  group=1)


def build_spec(site) -> mujoco.MjSpec:
    """The robot and the site as one MjSpec (not yet compiled)."""
    site = load_site(site)
    spec = mujoco.MjSpec.from_file(str(assets.asset(*assets.HAND_MODEL)))
    spec.option.timestep = TIMESTEP
    spec.visual.global_.offwidth, spec.visual.global_.offheight = CAMERA_SIZE
    spec.visual.quality.offsamples = 4
    spec.stat.extent = 4.0  # znear is a fraction of this; the camera sits inside the head link, so keep it small
    spec.visual.map.znear = 0.002
    spec.visual.headlight.ambient = [0.45, 0.45, 0.45]
    spec.visual.headlight.diffuse = [0.6, 0.6, 0.6]
    spec.visual.headlight.specular = [0, 0, 0]
    pelvis = spec.body("pelvis")
    x, y, yaw = site["robot"]["start"]
    pelvis.pos = [x, y, STAND_HEIGHT]
    pelvis.quat = list(_yaw_quat(yaw))

    # Joints: the dynamics unitree_mujoco gives the G1 (the policy is deployed there in simulation).
    force = {}
    for joint in spec.joints:
        if joint.name in BODY_JOINTS:
            joint.armature, joint.damping = 0.01, [0.05, 0, 0]
            joint.frictionloss = 0.1 if "wrist_pitch" in joint.name or "wrist_yaw" in joint.name else 0.2
        elif "_hand_" in joint.name:
            joint.armature, joint.damping, joint.frictionloss = 0.005, [0.02, 0, 0], 0.0
        else:
            continue
        force[joint.name] = float(joint.actfrcrange[1])
    for actuator in list(spec.actuators):
        spec.delete(actuator)
    for joint in BODY_JOINTS + HAND_JOINTS["left"] + HAND_JOINTS["right"]:
        spec.add_actuator(name=joint.removesuffix("_joint"), target=joint, trntype=mujoco.mjtTrn.mjTRN_JOINT,
                          gear=[1, 0, 0, 0, 0, 0], ctrllimited=True, ctrlrange=[-force[joint], force[joint]])
    for geom in list(spec.geoms):
        if geom.name == "floor" and geom.parent.name == "world":
            spec.delete(geom)
        elif FINGER_LINK.search(geom.parent.name):
            spec.delete(geom)  # replaced by a capsule below
        elif NOT_SEEN.search(geom.parent.name) and geom.group == 1:
            geom.group = HIDDEN
    # The Dex3 finger meshes are 240k triangles, which costs 120 ms per camera frame on the CPU and would break the 5 Hz
    # camera. The finger links keep Unitree's joints, masses and inertias; their shape is a capsule along the link.
    for link in spec.bodies:
        if FINGER_LINK.search(link.name):
            children = [b for b in spec.bodies if b.parent is not None and b.parent.name == link.name]
            tip = np.array(children[0].pos) if children else np.array(link.pos) / np.linalg.norm(link.pos) * FINGER_TIP
            link.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[0, 0, 0, *tip], size=[FINGER_RADIUS, 0, 0],
                          group=VISIBLE, rgba=[0.85, 0.85, 0.88, 1], mass=0.0, density=0.0)
    pelvis.add_site(name="imu", pos=[0.04525, 0, -0.08339], size=[0.01, 0, 0])
    spec.add_sensor(name="imu_quat", type=mujoco.mjtSensor.mjSENS_FRAMEQUAT, objtype=mujoco.mjtObj.mjOBJ_SITE,
                    objname="imu")
    spec.add_sensor(name="imu_gyro", type=mujoco.mjtSensor.mjSENS_GYRO, objtype=mujoco.mjtObj.mjOBJ_SITE,
                    objname="imu")
    spec.add_sensor(name="imu_acc", type=mujoco.mjtSensor.mjSENS_ACCELEROMETER, objtype=mujoco.mjtObj.mjOBJ_SITE,
                    objname="imu")

    torso = spec.body("torso_link")
    torso.add_camera(name="head", pos=list(CAMERA_POS), quat=list(head_camera_quat()), fovy=CAMERA_FOVY)

    # The support: a mocap body the pelvis can be welded to (inactive until the supported stance uses it).
    spec.worldbody.add_body(name="support", mocap=True)
    weld = spec.add_equality(name="support_weld", type=mujoco.mjtEq.mjEQ_WELD, name1="pelvis", name2="support")
    weld.objtype = mujoco.mjtObj.mjOBJ_BODY
    weld.data[:11] = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1]
    weld.active = False

    # The world.
    floor = site["floor"]
    half_x, half_y = floor.get("half_size", [10.0, 3.0])
    cx, cy = floor.get("center", [0.0, 0.0])
    spec.add_texture(name="ground", type=mujoco.mjtTexture.mjTEXTURE_2D, builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=[0.42, 0.45, 0.48], rgb2=[0.36, 0.39, 0.42], width=256, height=256)
    ground = spec.add_material(name="ground")
    ground.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "ground"
    ground.texrepeat = [half_x, half_y]
    ground.specular = 0
    spec.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[half_x, half_y, 0.05],
                            pos=[cx, cy, 0], material="ground", group=VISIBLE,
                            friction=[floor.get("friction", 1.0), 0.005, 0.0001])
    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX, builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.55, 0.65, 0.75], rgb2=[0.2, 0.25, 0.3], width=256, height=512)
    spec.worldbody.add_light(pos=[0, 0, 4], dir=[0, 0, -1], type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
                                diffuse=[0.5, 0.5, 0.5], castshadow=0)
    for table in site["tables"]:
        spec.worldbody.add_geom(name=table["name"], type=mujoco.mjtGeom.mjGEOM_BOX, pos=table["pos"],
                                size=table["half_size"], group=VISIBLE,
                                rgba=table.get("rgba", [0.55, 0.42, 0.3, 1]))
    for obj in site["objects"]:
        body = spec.worldbody.add_body(name=obj["name"], pos=obj["pos"])
        body.add_freejoint()
        half = obj["half_size"]
        body.add_geom(name=f"{obj['name']}_box", type=mujoco.mjtGeom.mjGEOM_BOX, size=half,
                      rgba=obj.get("rgba", [0.8, 0.2, 0.2, 1]), mass=obj.get("mass", 0.2), friction=[1.0, 0.01, 0.001],
                      group=VISIBLE)
        marker = obj.get("marker")
        if marker:
            normal, quat = FACES[marker["face"]]
            plate = marker["size"] * (1 + 2 * MARKER_BORDER) / 2
            offset = np.array(normal, float) * (np.abs(normal) @ np.array(half) + 0.0006)
            _add_marker_plate(spec, body, f"marker_{obj['name']}", marker["id"], plate, offset, quat)
        for side, body_name in HAND_BODY.items():
            weld = spec.add_equality(name=f"attach_{obj['name']}_{side}", type=mujoco.mjtEq.mjEQ_WELD,
                                     name1=body_name, name2=obj["name"])
            weld.objtype = mujoco.mjtObj.mjOBJ_BODY
            weld.data[:11] = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1]
            weld.active = False
    for station in site["stations"]:
        m = station["marker"]
        side = float(m["size"]) * (1 + 2 * MARKER_BORDER)
        pos = list(m["pos"])
        # a vertical panel facing `facing` (yaw of its outward normal in the world), standing on a thin post
        panel = spec.worldbody.add_body(name=f"station_{station['name']}", pos=pos,
                                        quat=list(_quat_mul(_yaw_quat(float(m["facing"])), FACES["+x"][1])))
        _add_marker_plate(spec, panel, f"marker_station_{station['name']}", m["id"], side / 2, [0, 0, 0],
                          [1, 0, 0, 0])
        panel.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[side / 2, side / 2, 0.004], pos=[0, 0, -0.0045],
                       rgba=[0.9, 0.9, 0.9, 1], contype=0, conaffinity=0)
    return spec


@dataclass
class Robot:
    """Name-based indices into a compiled model: how the SDK's joint order maps onto qpos, dof and ctrl."""

    qadr: np.ndarray  # (29,) qpos address of each body joint, SDK order
    dadr: np.ndarray  # (29,) dof (qvel) address
    hand_qadr: dict[str, np.ndarray]  # side -> (7,)
    hand_dadr: dict[str, np.ndarray]
    pelvis: int
    torso: int
    imu_quat: int  # sensordata addresses
    imu_gyro: int
    imu_acc: int
    camera: int
    support_mocap: int
    support_weld: int

    @classmethod
    def of(cls, m: mujoco.MjModel) -> "Robot":
        def jq(name):
            return int(m.jnt_qposadr[m.joint(name).id])

        def jd(name):
            return int(m.jnt_dofadr[m.joint(name).id])

        def sensor(name):
            return int(m.sensor_adr[m.sensor(name).id])

        return cls(
            qadr=np.array([jq(j) for j in BODY_JOINTS]),
            dadr=np.array([jd(j) for j in BODY_JOINTS]),
            hand_qadr={s: np.array([jq(j) for j in names]) for s, names in HAND_JOINTS.items()},
            hand_dadr={s: np.array([jd(j) for j in names]) for s, names in HAND_JOINTS.items()},
            pelvis=m.body("pelvis").id,
            torso=m.body("torso_link").id,
            imu_quat=sensor("imu_quat"),
            imu_gyro=sensor("imu_gyro"),
            imu_acc=sensor("imu_acc"),
            camera=m.camera("head").id,
            support_mocap=int(m.body_mocapid[m.body("support").id]),
            support_weld=m.equality("support_weld").id,
        )


def build(site) -> tuple[mujoco.MjModel, Robot, dict]:
    """Compile the scene: returns the model, the robot's indices and the checked site (marker sizes filled in)."""
    site = load_site(site)
    model = build_spec(site).compile()
    return model, Robot.of(model), site
