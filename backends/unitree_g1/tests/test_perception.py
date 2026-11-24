"""Perception on rendered frames (a test may read MuJoCo state to judge it; the executor never does)."""

import math
import os
from pathlib import Path

import cv2
import mujoco
import numpy as np
import pytest
from conftest import needs_assets

from robolang_g1 import assets, perception
from robolang_g1 import kinematics as K
from robolang_g1 import site as S
from robolang_g1.robot import LowSample
from robolang_g1.sim import scene
from robolang_g1.sim.policy import load_deploy

pytestmark = [
    needs_assets,
    pytest.mark.skipif(not os.environ.get("MUJOCO_GL"), reason="needs a GL context: MUJOCO_GL=egl"),
]
SITE = Path(__file__).resolve().parents[1] / "robolang_g1" / "sites" / "pick_place.json"


class Fixture:
    def __init__(self):
        cfg = load_deploy(assets.asset(*assets.POLICY_DIR, "params", "deploy.yaml"))
        self.q = np.zeros(29)
        self.q[cfg["joint_ids_map"]] = cfg["default_joint_pos"]
        self.model, self.robot, self.site = scene.build(SITE)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, 480, 640)
        self.opts = mujoco.MjvOption()
        self.opts.geomgroup[0] = self.opts.geomgroup[scene.HIDDEN] = 0
        q = self.q

        class FakeRobot:
            def sample_at(self, ms):
                return LowSample(int(ms), q, np.zeros(29), (1, 0, 0, 0), (0, 0, 0))

        self.perc = perception.Perception.__new__(perception.Perception)
        self.perc.robot, self.perc.kin, self.perc.camera = (
            FakeRobot(),
            K.Kinematics(self.site["camera"]),
            self.site["camera"],
        )
        fx, fy, cx, cy = S.intrinsics(self.site["camera"])
        self.perc.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
        self.perc.sizes = {s["marker"]["id"]: s["marker"]["size"] for s in self.site["stations"]}
        self.perc.sizes |= {o["marker"]["id"]: o["marker"]["size"] for o in self.site["objects"]}
        self.perc.station_markers = {s["marker"]["id"]: s["marker"] for s in self.site["stations"]}
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.perc.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)

    def frame(self, x, y, yaw):
        d = self.data
        d.qpos[0:3], d.qpos[3:7], d.qpos[self.robot.qadr] = (
            [x, y, scene.STAND_HEIGHT],
            [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)],
            self.q,
        )
        mujoco.mj_forward(self.model, d)
        self.renderer.update_scene(d, camera="head", scene_option=self.opts)
        return cv2.cvtColor(self.renderer.render(), cv2.COLOR_RGB2BGR)


@pytest.fixture(scope="module")
def fx():
    return Fixture()


def test_station_marker_gives_the_pelvis_pose_within_a_few_cm_and_a_hundredth_of_a_radian(fx):
    station = S.station(fx.site, "bench")
    x, y, yaw = station["pose"]
    seen = fx.perc.frame_poses(fx.frame(x, y, yaw), 1000)
    R, p, rng = seen[station["marker"]["id"]]
    reading = perception.Reading(station["marker"]["id"], p, R, 1.0, 1000, 1, 1, rng)
    ex, ey, eyaw, up = perception.pelvis_pose_in_map(station["marker"], reading)
    assert math.hypot(ex - x, ey - y) < 0.02 and abs(S.wrap(eyaw - yaw)) < 0.01
    assert up == pytest.approx([0, 0, 1], abs=0.02)  # an upright pelvis: the IMU-consistent solution
    assert 1.4 < rng < 2.0


def test_the_object_marker_gives_the_boxs_centre_within_a_centimetre_and_the_camera_sees_it_from_the_station(fx):
    x, y, yaw = S.station(fx.site, "bench")["pose"]
    seen = fx.perc.frame_poses(fx.frame(x, y, yaw), 1000)
    obj = S.obj(fx.site, "box")
    R, p, rng = seen[obj["marker"]["id"]]
    reading = perception.Reading(obj["marker"]["id"], p, R, 1.0, 1000, 1, 1, rng)
    centre = perception.object_centre(reading, obj["half_size"][2])  # the marker is on the top face
    truth = fx.data.xpos[fx.model.body("box").id] - fx.data.xpos[fx.robot.pelvis]
    assert np.linalg.norm(centre - truth) < 0.012


def test_a_covered_marker_is_not_detected(fx):
    site = S.load_site(SITE)
    site["stations"][0]["marker"]["covered"] = True
    model = scene.build_spec(site).compile()
    data = mujoco.MjData(model)
    robot = scene.Robot.of(model)
    x, y, yaw = site["stations"][0]["pose"]
    data.qpos[0:3], data.qpos[3:7], data.qpos[robot.qadr] = [x, y, scene.STAND_HEIGHT], [1, 0, 0, 0], fx.q
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, 480, 640)
    renderer.update_scene(data, camera="head", scene_option=fx.opts)
    seen = fx.perc.frame_poses(cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR), 1000)
    assert 20 not in seen and 7 in seen  # the station's marker is hidden, the box's is not


def test_fusing_frames_takes_the_median_and_the_fraction_that_agrees(fx):
    R = np.eye(3)
    obs = [(R, np.array([1.0, 0.0, 0.0]), 1.0, t) for t in (100, 200, 300, 400)] + [
        (R, np.array([1.2, 0, 0]), 1.0, 500)
    ]
    reading = perception._fuse(7, obs, 10)
    assert reading.position == pytest.approx([1.0, 0, 0]) and reading.confidence == pytest.approx(0.4)
    assert reading.stamp_ms == 400 and reading.detections == 5 and reading.age_ms(1400) == 1000


def test_the_visible_cone_of_the_camera():
    cam = {"width": 640, "height": 480, "fovy": 58.0}
    R, p = np.eye(3), np.zeros(3)  # looks along -z
    assert perception.in_view(R, p, [0, 0, -2], cam)
    assert not perception.in_view(R, p, [0, 0, 2], cam)  # behind
    assert not perception.in_view(R, p, [3, 0, -2], cam)  # far off to the side


def test_the_pose_that_puts_the_vertical_where_the_imu_does_is_chosen_over_the_better_fitting_one():
    """A planar marker has two poses; at a long range the wrong one often fits the image better. The IMU says which one
    has the pelvis upright."""
    import math

    from robolang_g1 import site as S
    from robolang_g1.perception import pick_by_gravity

    marker = {"pos": [3.75, 0.45, 0.19], "facing": math.pi}
    R_map = S.marker_frame(marker)[0]

    def pose_for(roll):  # the marker as seen by a pelvis that is `roll` rad off upright about its x axis
        c, s = math.cos(roll), math.sin(roll)
        R_pelvis_in_map = S.rot_z(0.1) @ np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        return (R_pelvis_in_map.T @ R_map, np.array([2.0, 0.3, -0.6]), 2.3)

    upright, tilted = pose_for(0.0), pose_for(0.13)
    imu = np.array([0.0, 0.0, 1.0])  # an upright pelvis
    assert pick_by_gravity([tilted, upright], marker, imu)[0] is upright  # the better fit (first) is the wrong one
    assert pick_by_gravity([upright, tilted], marker, imu)[0] is upright
