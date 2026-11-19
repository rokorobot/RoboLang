"""The scene: layout of the model, the site schema, markers, and (when a GL context exists) the head camera."""
import copy
import json
import os

import mujoco
import numpy as np
import pytest
from conftest import SITE, needs_assets

from robolang_g1.sim import scene


def site() -> dict:
    return json.loads(SITE.read_text())


@needs_assets
def test_model_layout():
    m, robot, loaded = scene.build(SITE)
    assert m.nu == 43 and m.opt.timestep == scene.TIMESTEP
    # the 29 body actuators are in the SDK's order 0-28, then 7 + 7 hand joints
    names = [m.actuator(i).name for i in range(m.nu)]
    assert names[:29] == [j.removesuffix("_joint") for j in scene.BODY_JOINTS]
    assert names[29:36] == [j.removesuffix("_joint") for j in scene.HAND_JOINTS["left"]]
    assert names[36:] == [j.removesuffix("_joint") for j in scene.HAND_JOINTS["right"]]
    assert scene.BODY_JOINTS[0] == "left_hip_pitch_joint" and scene.BODY_JOINTS[12] == "waist_yaw_joint"
    assert scene.BODY_JOINTS[15] == "left_shoulder_pitch_joint"
    assert scene.BODY_JOINTS[22] == "right_shoulder_pitch_joint"
    # torque motors, so ctrl is a torque; the hand model interleaves hand joints with the arms in qpos, not in ctrl
    assert np.all(m.actuator_gainprm[:, 0] == 1) and np.all(m.actuator_biastype == 0)
    assert robot.qadr[22] > robot.hand_qadr["left"][-1]  # right arm comes after the left hand in qpos
    # camera, IMU, support weld, one attach weld per object and hand
    assert m.camera("head").id == robot.camera
    assert m.neq == 1 + 2 * len(loaded["objects"])
    assert m.body("support").mocapid[0] == robot.support_mocap
    assert m.sensor("imu_gyro").dim == 3 and m.sensor("imu_quat").dim == 4


@needs_assets
def test_head_camera_looks_forward_and_down():
    m, robot, _ = scene.build(SITE)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    forward = -d.cam_xmat[robot.camera].reshape(3, 3)[:, 2]  # a MuJoCo camera looks along its -z
    up = d.cam_xmat[robot.camera].reshape(3, 3)[:, 1]
    assert forward[0] > 0.5 and forward[2] < -0.5  # ahead of the robot (+x), tilted down about 47 degrees
    assert abs(forward[1]) < 1e-6 and up[2] > 0.5
    assert np.degrees(np.arcsin(-forward[2])) == pytest.approx(np.degrees(scene.CAMERA_PITCH), abs=0.5)


@needs_assets
def test_robot_starts_standing_where_the_site_says():
    s = site()
    s["robot"]["start"] = [1.0, -2.0, 0.5]
    m, robot, _ = scene.build(s)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert np.allclose(d.xpos[robot.pelvis][:2], [1.0, -2.0]) and d.xpos[robot.pelvis][2] == pytest.approx(0.788)


def test_aruco_marker_texture_decodes():
    import cv2

    for marker_id in (0, 7, 49):
        png = scene.marker_png(marker_id)
        image = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_GRAYSCALE)
        detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        _, ids, _ = detector.detectMarkers(image)
        assert ids is not None and ids.flatten().tolist() == [marker_id]


def test_site_defaults_and_marker_size():
    loaded = scene.load_site(SITE)
    box = next(o for o in loaded["objects"] if o["name"] == "box_far")
    assert box["marker"]["size"] == pytest.approx(0.12 / 1.4)  # the plate is 12 cm: marker plus its 20% border
    assert loaded["robot"]["start"] == [-9.5, 0.0, 0.0]
    minimal = scene.load_site({"schema": scene.SITE_SCHEMA})
    assert minimal["tables"] == [] and minimal["robot"]["start"] == [0.0, 0.0, 0.0]


@pytest.mark.parametrize("edit, message", [
    (lambda s: s.update(schema="x"), "schema"),
    (lambda s: s["tables"][0].pop("half_size"), "half_size"),
    (lambda s: s["tables"][0].update(pos=[1, 2]), "pos"),
    (lambda s: s["objects"][0]["marker"].update(id=77), "marker id"),
    (lambda s: s["objects"][0]["marker"].update(face="left"), "face"),
    (lambda s: s["objects"][1].update(name="box_far"), "duplicate"),
    (lambda s: s["objects"][0].update(mass=-1), "mass"),
    (lambda s: s["stations"][0]["marker"].pop("size"), "size"),
    (lambda s: s["robot"].update(start=[0, 0]), "start"),
])
def test_site_errors_name_the_problem(edit, message):
    s = copy.deepcopy(site())
    edit(s)
    with pytest.raises(scene.SiteError, match=message):
        scene.load_site(s)


@needs_assets
@pytest.mark.skipif(not os.environ.get("MUJOCO_GL"), reason="needs a GL context: set MUJOCO_GL=egl")
def test_head_camera_sees_and_measures_a_marker():
    import cv2

    s = site()
    s["robot"]["start"] = [-0.2, 0.0, 0.0]
    m, robot, loaded = scene.build(s)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    options = mujoco.MjvOption()
    options.geomgroup[0] = options.geomgroup[scene.HIDDEN] = 0
    renderer = mujoco.Renderer(m, height=480, width=640)
    renderer.update_scene(d, camera="head", scene_option=options)
    image = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    assert ids is not None and 7 in ids.flatten().tolist()
