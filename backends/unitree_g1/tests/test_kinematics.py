"""The executor's own kinematic model: FK from measured joints (camera, hand) and damped-least-squares IK."""

import math
from pathlib import Path

import mujoco
import numpy as np
import pytest
from conftest import SITE, needs_assets

from robolang_g1 import assets, ops
from robolang_g1.executor import BackendFailure
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


# ------------------------------------------------------------ clearance: the carry pose, the arm paths, the view path

SITES_DIR = Path(__file__).resolve().parents[1] / "robolang_g1" / "sites"
BOX_CENTRE = np.array([0.271, -0.154, 0.0007])  # the box on the bench, in the pelvis frame (a real grasp's)


def clearance_ops(kin, default, site_name="pick_place", station="shelf"):
    """A `G1Operations` with no robot behind it, only what planning and the path checks read."""
    from types import SimpleNamespace

    from robolang_g1 import site as S

    o = ops.G1Operations.__new__(ops.G1Operations)
    site = S.load_site(SITES_DIR / f"{site_name}.json")
    o.kin, o.site, o.q_carry, o.events = kin, site, default.copy(), []
    o.q_carry[14] = 0.0
    o.clr = K.Clearance(site["camera"], n_boxes=len(S.declared_boxes(site)))
    o.robot = SimpleNamespace(low_q=o.q_carry.copy())
    o.pose = list(S.station(site, station)["pose"])  # the pelvis in the map, at the station
    o.stations = {s["name"]: s for s in site["stations"]}
    return o


def grasp_plan(o, half=(0.04, 0.04, 0.04), centre=BOX_CENTRE):
    """The grasp as `grasp` plans it (a reach that passes `_grasp_accept`), and the held object it would give."""
    plan = o._plan_reach(
        "right",
        lambda R: centre + np.array([0, 0, ops.GRASP_RISE]),
        extra=(("lift", (0.0, 0.0, 0.10)),),
        accept=o._grasp_accept("right", centre, half, o.q_carry.copy()),
    )
    R_hand, p_hand = o.kin.grasp_frame("right", plan["grasp"])
    o.held = {
        "object": "box",
        "hand": "right",
        "offset_in_hand": R_hand.T @ (centre - p_hand),
        "rotation_in_hand": R_hand.T,
        "fingers_down_deg": plan["fingers_down_deg"],
        "lift_waypoint": plan["lift"],
        "carry": plan["carry"],
    }
    return plan


def test_the_walking_controllers_arm_pose_has_the_held_object_against_the_hip_and_the_carry_pose_clears_it(
    kin, default
):
    """The brief's hip-clearance rule: a held object clears every link of the robot by at least 2 cm in the carry pose
    (the holding hand's own palm and fingers excepted), measured on the robot's own model with the object welded as in a
    real grasp. The walking controller's own arm pose does not (0.1-0.6 cm against the right hip)."""
    for half in ((0.04, 0.04, 0.04), (0.045, 0.045, 0.04)):  # the box and the crate
        o = clearance_ops(kin, default)
        plan = grasp_plan(o, half)
        h = o.held
        fresh = K.Clearance(o.site["camera"], n_boxes=0)  # a model that planned nothing
        d_default, link_default = fresh.held_to_links_with(
            "right", o.q_carry, half, h["offset_in_hand"], h["rotation_in_hand"]
        )
        assert d_default < 0.02, (half, d_default, link_default)
        d, link = fresh.held_to_links_with("right", plan["carry"], half, h["offset_in_hand"], h["rotation_in_hand"])
        assert d >= ops.CARRY_CLEARANCE_M, (half, d, link)
        dev = float(np.linalg.norm(plan["carry"][22:29] - o.q_carry[22:29]))
        assert 0 < dev < 0.6  # a few centimetres of the hand, not a new pose


def test_the_grasp_and_place_paths_clear_the_declared_tables_with_the_margins_and_the_check_refuses_when_they_do_not(
    kin, default
):
    import copy

    for site_name, station in (("pick_place", "bench"), ("pick_place", "shelf"), ("carry_hold", "dock")):
        o = clearance_ops(kin, default, site_name, station)
        plan = grasp_plan(o)
        info = plan["clearance"]
        assert info["clear"] is True and info["closest"]["over_limit_m"] > 0, (site_name, station, info["closest"])
        assert info["margins_m"] == {"sideways": 0.03, "vertical": 0.015, "contact_overlap": 0.01}
        place = o._plan_reach(
            "right",
            lambda R: BOX_CENTRE + np.array([0, 0.04, -0.003]) - R @ o.held["offset_in_hand"],
            extra=(("above", (0.0, 0.0, 0.10)),),
            degs=(o.held["fingers_down_deg"],),
            accept=o._place_accept("right", (0.04, 0.04, 0.04), o.q_carry.copy()),
        )
        assert place["clearance"]["clear"] is True, place["clearance"]["closest"]
    # a post declared where the hand has to go: no reach has a clear path, S502 before any arm motion
    o = clearance_ops(kin, default, "pick_place", "bench")
    o.site = copy.deepcopy(o.site)
    o.site["obstacles"] = [{"name": "post", "pos": [2.73, 0.0, 0.85], "half_size": [0.05, 0.12, 0.10]}]
    o.clr = K.Clearance(o.site["camera"], n_boxes=3)
    with pytest.raises(BackendFailure) as err:
        grasp_plan(o)
    assert err.value.code == "S502" and "no clear path" in err.value.message


def test_the_planned_grasp_keeps_the_open_fingers_out_of_the_table_where_phase_2s_went_3_cm_into_it(kin, default):
    """With phase 2's grasp (hand pitched 15 degrees, the pinch 1 cm below the object's centre) the open fingers reached
    3 cm into the table in the robot's own model; with the planned one (flat, the pinch at the centre) the hand just
    touches the table top, within the 1 cm the contact legs allow."""
    o = clearance_ops(kin, default, "pick_place", "bench")
    boxes = o.site["tables"]
    for deg, rise, inside in ((15, -0.01, True), (0, ops.GRASP_RISE, False)):
        plan = o._plan_reach(
            "right", lambda R: BOX_CENTRE + np.array([0, 0, rise]), extra=(("lift", (0, 0, 0.1)),), degs=(deg,)
        )
        o.clr.set_boxes(boxes, tuple(o.pose), 0.0, 0.0)  # the real table, no margin
        found = o.clr.to_boxes("right", plan["grasp"], held=False)
        assert (found["hand"][0] < -ops.CONTACT_OVERLAP_M) is inside, (deg, rise, found["hand"])


def test_the_view_path_is_clear_over_the_shelf_table_and_not_through_an_obstacle_on_it(kin, default):
    import copy

    o = clearance_ops(kin, default)
    grasp_plan(o)
    ok = o._view_clearance()
    assert ok["clear"] is True and ok["boxes"] == ["bench_table", "shelf_table"]
    assert (
        ok["closest"]["over_limit_m"] > 0
        and ok["margins_m"]["sideways"] == 0.03
        and ok["margins_m"]["vertical"] == 0.015
    )
    # an obstacle declared where the hand is shown (pelvis at the shelf station; the view pose is ~0.28 m ahead, 0.1 m
    # to the right, 0.14 m above the pelvis): the path is not clear, and the record says what is closest
    blocked = clearance_ops(kin, default)
    blocked.site = copy.deepcopy(blocked.site)
    blocked.site["obstacles"] = [{"name": "post", "pos": [2.73, 2.30, 0.93], "half_size": [0.04, 0.04, 0.12]}]
    blocked.clr = K.Clearance(blocked.site["camera"], n_boxes=3)
    blocked.held = o.held
    bad = blocked._view_clearance()
    assert bad["clear"] is False and bad["closest"]["box"] == "post" and bad["closest"]["clearance_m"] < 0
    nothing = clearance_ops(kin, default)
    nothing.held = o.held
    nothing.site = dict(nothing.site, tables=[], obstacles=[])
    nothing.clr = K.Clearance(nothing.site["camera"], n_boxes=0)
    assert nothing._view_clearance()["clear"] is True  # nothing declared: nothing to hit


def test_clearance_distances_are_signed_and_follow_the_arm(kin, default):
    clr = K.Clearance(None, n_boxes=1)
    clr.set_boxes([{"name": "wall", "pos": [5.0, 0.0, 0.0], "half_size": [0.1, 5.0, 5.0]}], (0.0, 0.0, 0.0), 0.0, 0.0)
    far = clr.to_boxes("right", default)
    assert far["arm"][0] >= 0.5 and far["hand"][0] >= 0.5 and far["arm"][2] == ""  # 5 m ahead: out of reach (0.5 m)
    # the box moved to where the right hand is (pelvis frame): the hand is inside it, the distance is negative
    hand = kin.grasp_frame("right", default)[1]
    clr.set_boxes(
        [{"name": "wall", "pos": [hand[0] + 0.0, hand[1], hand[2] + 0.788], "half_size": [0.05, 0.05, 0.05]}],
        (0.0, 0.0, 0.0),
        0.0,
        0.0,
    )
    assert clr.to_boxes("right", default)["hand"][0] < 0
    q = default.copy()
    q[22] = -1.2  # right shoulder pitch: the arm swings forward, away from the box
    assert clr.to_boxes("right", q)["hand"][0] > 0.05


# ----------------------------------------------------------------------------------------- the approach point's shift


def approach_ops(kin, site_name):
    from pathlib import Path

    from robolang_g1 import site as S

    o = ops.G1Operations.__new__(ops.G1Operations)
    o.kin = kin
    o.site = S.load_site(Path(__file__).resolve().parents[1] / "robolang_g1" / "sites" / f"{site_name}.json")
    o.stations = {s["name"]: s for s in o.site["stations"]}
    o.tracker, o.robot = None, None  # no robot: the belief heading is the one set
    o._xy, o._heading0 = [0.0, 0.0], 0.0
    return o


def test_a_shuffle_moves_the_belief_back_as_the_robot_does_and_leaves_the_heading_to_the_imu(kin):
    o = approach_ops(kin, "pick_place")
    o.pose = [1.0, 2.0, math.pi / 2]  # facing +y
    o._dead_reckon(0.0, 0.3, 0.2, ops.SHUFFLE_S, True, 0, 1200)
    assert o.pose[0] == pytest.approx(1.0, abs=1e-9) and o.pose[1] == pytest.approx(2.0 - ops.SHUFFLE_BACK_M)
    assert o.pose[2] == pytest.approx(math.pi / 2)  # the commanded yaw rate does not turn the belief (heading.py does)
