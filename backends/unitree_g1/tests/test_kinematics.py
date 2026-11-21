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


# ------------------------------------------------------------ the clearance check before an observation motion


def bare_ops(kin, default, site, held_object="box"):
    """A `G1Operations` with no robot behind it, only what `_view_clearance` and `_plan_reach` read: the kinematic
    model, the carry pose, the belief pose, and a held object whose grasp was planned the way `grasp` does."""
    from types import SimpleNamespace

    from robolang_g1 import site as S

    o = ops.G1Operations.__new__(ops.G1Operations)
    o.kin, o.site, o.q_carry, o.events = kin, site, default.copy(), []
    o.q_carry[14] = 0.0
    o.robot = SimpleNamespace(low_q=o.q_carry.copy())
    o.pose = list(S.station(site, "shelf")["pose"])  # the pelvis in the map, at the shelf station
    marker_at = np.array([0.28, -0.10, 0.04])  # the object's centre on the table in front of the robot (pelvis frame)
    plan = o._plan_reach("right", lambda R: marker_at + np.array([0, 0, ops.GRASP_RISE]),
                         extra=(("lift", (0.0, 0.0, 0.10)),))
    R_hand, p_hand = kin.grasp_frame("right", plan["grasp"])
    centre = marker_at  # where the object is: the hand holds it at the grasp pose
    o.held = {"object": held_object, "hand": "right", "offset_in_hand": R_hand.T @ (centre - p_hand),
              "rotation_in_hand": R_hand.T, "fingers_down_deg": plan["fingers_down_deg"],
              "lift_waypoint": plan["lift"]}
    return o


def test_arm_spheres_cover_the_arm_and_move_with_the_joints(kin, default):
    a = kin.arm_spheres("right", default)
    assert {link for *_, link in a} == {"upper arm", "forearm", "hand"} and len(a) > 12
    assert all(0.04 < r < 0.06 for _, r, _ in a)
    q = default.copy()
    q[22] = -1.2  # right shoulder pitch: the arm swings forward
    assert max(c[0] for c, _, _ in kin.arm_spheres("right", q)) > max(c[0] for c, _, _ in a) + 0.1
    assert max(float(np.linalg.norm(c)) for c, _, _ in a) < 0.8  # all within an arm's length of the pelvis


def test_the_view_path_is_clear_over_the_shelf_table_and_not_through_an_obstacle_on_it(kin, default):
    import copy
    import json
    from pathlib import Path

    from robolang_g1 import site as S

    site = S.load_site(Path(__file__).resolve().parents[1] / "robolang_g1" / "sites" / "pick_place.json")
    o = bare_ops(kin, default, site)
    ok = o._view_clearance()
    assert ok["clear"] is True and ok["boxes"] == ["bench_table", "shelf_table"]
    assert ok["closest"]["clearance_m"] > 0 and ok["margins_m"] == {"sideways": 0.03, "vertical": 0.015}
    # an obstacle declared where the hand is shown (pelvis at the shelf station; the view pose is ~0.28 m ahead, 0.1 m
    # to the right, 0.14 m above the pelvis): the path is not clear, and the record says what is closest
    blocked = copy.deepcopy(json.loads(json.dumps(site, default=list)))
    blocked["obstacles"] = [{"name": "post", "pos": [2.73, 2.30, 0.93], "half_size": [0.04, 0.04, 0.12]}]
    o2 = bare_ops(kin, default, blocked)
    bad = o2._view_clearance()
    assert bad["clear"] is False and bad["closest"]["box"] == "post" and bad["closest"]["clearance_m"] < 0
    far = dict(blocked, obstacles=[{"name": "post", "pos": [2.73, 3.30, 0.93], "half_size": [0.04, 0.04, 0.12]}])
    assert bare_ops(kin, default, far)._view_clearance()["clear"] is True  # a metre away: no matter
    nothing = dict(site, tables=[], obstacles=[])
    assert bare_ops(kin, default, nothing)._view_clearance()["clear"] is True  # nothing declared: nothing to hit
