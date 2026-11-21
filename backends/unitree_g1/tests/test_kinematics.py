"""The executor's own kinematic model: FK from measured joints (camera, hand) and damped-least-squares IK."""

import math

import mujoco
import numpy as np
import pytest
from conftest import SITE, needs_assets

from robolang_g1 import assets, ops
from robolang_g1 import kinematics as K
from robolang_g1.sim import scene
from robolang_g1.sim.policy import load_deploy

pytestmark = needs_assets


@pytest.fixture(scope="module")
def kin():
    return K.Kinematics()


@pytest.fixture(scope="module")
def default():
    cfg = load_deploy(assets.asset(*assets.POLICY_DIR, "params", "deploy.yaml"))
    q = np.zeros(29)
    q[cfg["joint_ids_map"]] = cfg["default_joint_pos"]
    return q


def test_camera_pose_from_joints_matches_the_scenes_camera(kin, default):
    """The model the executor uses and the stand-in's scene are built from one description: the head camera is where
    the scene puts it (a test may read the scene; the executor does not)."""
    m, r, _ = scene.build(SITE)
    d = mujoco.MjData(m)
    d.qpos[0:3], d.qpos[3:7], d.qpos[r.qadr] = [0, 0, 0], [1, 0, 0, 0], default
    mujoco.mj_forward(m, d)
    R, p = kin.camera_pose(default)
    assert p == pytest.approx(d.cam_xpos[r.camera], abs=1e-6)
    assert R == pytest.approx(d.cam_xmat[r.camera].reshape(3, 3), abs=1e-6)


def test_the_camera_follows_the_waist(kin, default):
    q = default.copy()
    q[14] = 0.3  # waist pitch leans the torso, and the head camera with it
    assert kin.camera_pose(q)[1][0] > kin.camera_pose(default)[1][0] + 0.05


def test_ik_reaches_a_pose_made_by_forward_kinematics(kin, default):
    rng = np.random.default_rng(1)
    for _ in range(10):
        q = default.copy()
        q[K.ARM_JOINTS["right"]] += rng.uniform(-0.5, 0.5, 7)
        R, p = kin.grasp_frame("right", q)
        solved, info = kin.solve("right", p, R, default, posture=default)
        assert solved is not None, info
        R2, p2 = kin.grasp_frame("right", solved)
        assert np.linalg.norm(p2 - p) < 0.01


def test_ik_leaves_every_joint_but_the_arm_alone_and_respects_limits(kin, default):
    R, p = kin.grasp_frame("left", default)
    q, _ = kin.solve("left", p + [0.03, 0.02, 0.05], R, default, posture=default)
    assert q is not None
    other = [j for j in range(29) if j not in K.ARM_JOINTS["left"]]
    assert np.array_equal(q[other], default[other])
    assert np.all(q >= kin.limits[:, 0] - 1e-9) and np.all(q <= kin.limits[:, 1] + 1e-9)


def test_leaning_the_waist_widens_the_reach_and_a_far_target_has_no_solution(kin, default):
    o = ops.G1Operations.__new__(ops.G1Operations)
    o.kin, o.q_carry = kin, default.copy()
    R = ops._frame(math.radians(15))
    ready = o._ready_pose("right")
    far = np.array([0.42, -0.1, -0.002])
    assert kin.solve("right", far, R, ready, posture=ready, rot_weight=0.5, iters=400)[0] is None  # arm alone: too far
    leaning, info = kin.solve("right", far, R, ready, posture=ready, rot_weight=0.5, iters=400, waist_max=0.3)
    assert leaning is not None and 0 < leaning[K.WAIST_PITCH] <= 0.3 + 1e-9
    with pytest.raises(Exception, match="S502|reach"):
        o._plan_reach("right", lambda R: np.array([0.8, -0.1, 0.0]))  # beyond the arm's length: refused at once


def test_a_reach_plan_gives_a_pregrasp_a_grasp_and_a_lift_in_the_same_hand_orientation(kin, default):
    o = ops.G1Operations.__new__(ops.G1Operations)
    o.kin, o.q_carry = kin, default.copy()
    centre = np.array([0.284, -0.07, -0.011])
    plan = o._plan_reach("right", lambda R: centre + [0, 0, ops.GRASP_RISE], extra=(("lift", (0, 0, 0.1)),))
    R, p = kin.grasp_frame("right", plan["grasp"])
    assert np.linalg.norm(p - plan["pinch"]) < 0.012
    assert kin.grasp_frame("right", plan["lift"])[1][2] == pytest.approx(p[2] + 0.1, abs=0.015)
    assert kin.grasp_frame("right", plan["pre"])[1][0] < p[0]  # the pre-grasp is behind the grasp
