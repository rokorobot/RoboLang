"""Verdicts from observations (`observe.py`): fake snapshots built from the site's geometry, no simulator, no DDS.

Every case that matters for safety is here: a reading that is missing, below `min_confidence`, too old, or near the edge
of a region or a tolerance never gives `true`, and a false success needs a `true` for every goal atom.
"""

import math
from decimal import Decimal
from pathlib import Path

import numpy as np
import pytest

from robolang import ir
from robolang.core import check_meaning, verify_sim
from robolang.core.schema import _Validator, load_schema
from robolang_g1 import observe as O
from robolang_g1 import site as S
from robolang_g1.perception import Detection, Reading, fuse_frames
from robolang_g1.sim.scene import STAND_HEIGHT

SITES = Path(__file__).resolve().parents[1] / "robolang_g1" / "sites"
SITE = S.load_site(SITES / "pick_place.json")
BENCH, SHELF, PAD = (S.station(SITE, n) for n in ("bench", "shelf", "pad"))
NAMES = {"r": "g1", "box": "box", "src": "bench", "dst": "shelf", "pad": "pad"}
BOX_ID = 7
POLICY = O.Policy(Decimal("0.9"), 10_000)
T = 5000


def reading(mid, position, rotation=None, conf=1.0, stamp=T - 200):
    rotation = np.eye(3) if rotation is None else rotation
    return Reading(mid, np.array(position, float), rotation, conf, stamp, 10, 10, 1.0, round(conf * 10))


def station_reading(station, pelvis, **kw):
    """What the head camera reports for a station marker when the pelvis stands at `pelvis` = (x, y, yaw)."""
    R_m, p_m = S.marker_frame(station["marker"])
    R_p = S.rot_z(pelvis[2])
    p_p = np.array([pelvis[0], pelvis[1], STAND_HEIGHT])
    return reading(station["marker"]["id"], R_p.T @ (p_m - p_p), R_p.T @ R_m, **kw)


def object_reading(station_r, in_station_frame, **kw):
    """The object marker, given where it is in the station marker's frame (x right, y up, z out)."""
    p = station_r.rotation @ np.array(in_station_frame, float) + station_r.position
    return reading(BOX_ID, p, **kw)


def hand(side="right", state="open", grasp_point=(0.3, -0.1, 0.1)):
    q = {"open": np.zeros(7), "closed": np.array(O.HAND_CLOSED[side]), "between": np.full(7, 0.5)}[state]
    return O.Hand(side, state, tuple(q), grasp_point, T - 100)


def snap(markers=(), hands=(), ready=None, t=T):
    return O.Snapshot(t, 10, {m.marker_id: m for m in markers}, {h.side: h for h in hands}, (0, 0, 1), ready)


def at_station(station=BENCH, offset=(0.0, 0.0, 0.0)):
    x, y, yaw = station["pose"]
    return station_reading(station, (x + offset[0], y + offset[1], yaw + offset[2]))


IN_BENCH_REGION = (0.55, 0.64, 1.0)  # a box on the bench: x right, y up, z out of the plate
OUT_OF_REGION = (0.55, 0.64, 1.3)  # 0.21 m beyond the region's far face (z 0.91-1.09)


def cond(kind, *args):
    return {"object_at": ir.ObjectAt, "robot_at": ir.RobotAt, "holding": ir.Holding, "robot_ready": ir.RobotReady}[
        kind
    ](*args)


def verdict(condition, snapshot, policy=POLICY):
    return O.evaluate(condition, NAMES, SITE, snapshot, policy)


# ----------------------------------------------------------------------------------------------- object_at


def test_object_at_true_false_and_unknown():
    bench = at_station()
    on_bench = snap([bench, object_reading(bench, IN_BENCH_REGION)])
    assert verdict(cond("object_at", "box", "src"), on_bench) == "true"
    elsewhere = snap([bench, object_reading(bench, OUT_OF_REGION)])
    assert verdict(cond("object_at", "box", "src"), elsewhere) == "false"
    assert verdict(cond("object_at", "box", "src"), snap([bench])) == "unknown"  # the object is not seen
    assert verdict(cond("object_at", "box", "src"), snap([object_reading(bench, IN_BENCH_REGION)])) == "unknown"


def test_object_at_is_located_through_the_stations_marker_whatever_the_robots_pose():
    for pelvis in [(2.45, 0.0, 0.0), (2.3, 0.2, 0.15), (2.6, -0.1, -0.1)]:  # not exactly on station: still located
        station_r = station_reading(BENCH, pelvis)
        s = snap([station_r, object_reading(station_r, IN_BENCH_REGION)])
        assert verdict(cond("object_at", "box", "src"), s) == "true"


def test_object_at_is_unknown_within_the_margin_of_the_regions_boundary():
    bench = at_station()
    far_face = SITE["locations"][0]["region"]["max"][2]
    for dz, expected in [(-0.02, "true"), (-0.01, "unknown"), (0.01, "unknown"), (0.02, "false")]:
        r = object_reading(bench, (0.55, 0.64, far_face + dz))
        assert verdict(cond("object_at", "box", "src"), snap([bench, r])) == expected, dz


def test_object_at_needs_both_readings_usable():
    bench = at_station()
    box = object_reading(bench, IN_BENCH_REGION)
    assert verdict(cond("object_at", "box", "src"), snap([bench, box])) == "true"
    low = snap([at_station(), object_reading(bench, IN_BENCH_REGION, conf=0.8)])
    assert verdict(cond("object_at", "box", "src"), low) == "unknown"  # below the default 0.9
    low_station = snap([station_reading(BENCH, BENCH["pose"], conf=0.8), box])
    assert verdict(cond("object_at", "box", "src"), low_station) == "unknown"
    stale = snap([station_reading(BENCH, BENCH["pose"], stamp=T - 10_001), box])
    assert verdict(cond("object_at", "box", "src"), stale) == "unknown"
    assert verdict(cond("object_at", "box", "src"), snap([bench, box], t=T - 200 + 10_001)) == "unknown"  # one ms old
    assert verdict(cond("object_at", "box", "src"), snap([bench, box], t=T - 200 + 10_000)) == "true"  # exactly max_age


def test_an_object_seen_at_a_closed_hand_is_not_at_a_place_even_with_no_station_in_view():
    held = object_reading(at_station(), IN_BENCH_REGION, stamp=T - 100)
    in_hand = hand("right", "closed", tuple(held.position + [0.0, 0.0, -0.05]))
    assert verdict(cond("object_at", "box", "src"), snap([held], [in_hand])) == "false"
    assert verdict(cond("object_at", "box", "src"), snap([at_station(), held], [in_hand])) == "false"  # held wins
    open_hand = hand("right", "open", tuple(held.position + [0.0, 0.0, -0.05]))
    assert verdict(cond("object_at", "box", "src"), snap([at_station(), held], [open_hand])) == "true"  # on a palm


# ------------------------------------------------------------------------------------------------ holding


def test_holding_true_when_the_hand_is_closed_and_the_marker_is_at_it():
    marker = reading(BOX_ID, (0.3, -0.1, 0.15))
    closed = hand("right", "closed", (0.3, -0.1, 0.10))  # 5 cm away: like a grasped box
    assert verdict(cond("holding", "r", "box"), snap([marker], [closed])) == "true"
    assert verdict(cond("holding", "r", "box"), snap([marker], [closed, hand("left")])) == "true"


def test_holding_false_when_open_or_when_the_marker_is_seen_away_from_the_closed_hand():
    marker = reading(BOX_ID, (0.3, -0.1, 0.15))
    assert verdict(cond("holding", "r", "box"), snap([marker], [hand("right"), hand("left")])) == "false"
    assert verdict(cond("holding", "r", "box"), snap([], [hand("right"), hand("left")])) == "false"  # open: no marker
    away = hand("right", "closed", (0.3, -0.3, 0.10))
    assert verdict(cond("holding", "r", "box"), snap([marker], [away, hand("left")])) == "false"


def test_holding_is_unknown_when_it_cannot_be_decided():
    closed = hand("right", "closed", (0.3, -0.1, 0.10))
    marker = reading(BOX_ID, (0.3, -0.1, 0.15))
    assert verdict(cond("holding", "r", "box"), snap([], [closed, hand("left")])) == "unknown"  # marker not seen
    assert (
        verdict(cond("holding", "r", "box"), snap([reading(BOX_ID, marker.position, conf=0.7)], [closed, hand("left")]))
        == "unknown"
    )
    assert verdict(cond("holding", "r", "box"), snap([marker], [hand("right", "between"), hand("left")])) == "unknown"
    assert verdict(cond("holding", "r", "box"), snap([marker], [])) == "unknown"  # no hand state at all


def test_hand_state_from_the_joints():
    assert O.hand_state("right", np.zeros(7)) == "open"
    assert O.hand_state("right", O.HAND_CLOSED["right"]) == "closed"
    assert O.hand_state("left", O.HAND_CLOSED["left"]) == "closed"
    assert O.hand_state("right", O.HAND_CLOSED["left"]) == "between"  # the other hand's pose is not this hand's
    assert O.hand_state("right", np.array(O.HAND_CLOSED["right"]) * 0.5) == "between"


# ------------------------------------------------------------------------------------------------- robot_at


def robot_at_detail(offset=(0, 0, 0), station=BENCH, location="src", **kw):
    snapshot = snap([station_reading(station, tuple(np.array(station["pose"]) + offset), **kw)])
    v = O.atom(cond("robot_at", "r", location), NAMES, SITE, snapshot, POLICY)
    return v.result, v.detail


def test_robot_at_true_within_the_tolerance_and_reports_the_measured_offset():
    result, detail = robot_at_detail((0.02, 0.01, 0.03))
    assert result == "true"
    assert detail["offset_m"] == pytest.approx(math.hypot(0.02, 0.01), abs=1e-4)
    assert detail["offset_rad"] == pytest.approx(0.03, abs=1e-4)
    assert detail["tolerance_m"] == 0.05 and detail["tolerance_rad"] == 0.1


@pytest.mark.parametrize(
    "offset, expected",
    [
        ((0.04, 0.0, 0.0), "true"),
        ((0.048, 0.0, 0.0), "unknown"),  # inside 5 cm but inside the 5 mm margin of it
        ((0.052, 0.0, 0.0), "unknown"),
        ((0.06, 0.0, 0.0), "false"),
        ((0.0, 0.0, 0.085), "true"),
        ((0.0, 0.0, 0.095), "unknown"),  # inside 0.1 rad, within the 0.01 rad margin
        ((0.0, 0.0, 0.105), "unknown"),
        ((0.0, 0.0, 0.12), "false"),
        ((0.0, -0.08, 0.0), "false"),
        ((0.5, 0.0, 0.0), "false"),
    ],
)
def test_robot_at_near_the_tolerance_is_unknown_never_true(offset, expected):
    assert robot_at_detail(offset)[0] == expected


def test_robot_at_unknown_when_its_marker_is_not_usable_and_no_other_station_confirms():
    assert O.evaluate(cond("robot_at", "r", "src"), NAMES, SITE, snap([]), POLICY) == "unknown"
    assert robot_at_detail(conf=0.8)[0] == "unknown"
    assert robot_at_detail(stamp=T - 10_001)[0] == "unknown"


def test_robot_at_false_when_another_station_confirms_the_robot_is_there():
    at_shelf = snap([at_station(SHELF)])  # the bench's marker is not seen, the shelf's puts the robot at the shelf
    assert O.evaluate(cond("robot_at", "r", "src"), NAMES, SITE, at_shelf, POLICY) == "false"
    assert O.evaluate(cond("robot_at", "r", "dst"), NAMES, SITE, at_shelf, POLICY) == "true"
    # near the shelf but not confirmed there: no verdict about the bench from it
    nearly = snap([at_station(SHELF, (0.07, 0.0, 0.0))])
    assert O.evaluate(cond("robot_at", "r", "src"), NAMES, SITE, nearly, POLICY) == "unknown"
    # a station less than twice the tolerance away does not rule this one out
    near_station = {**SITE, "stations": [BENCH, {**SHELF, "pose": [BENCH["pose"][0] + 0.06, BENCH["pose"][1], 0.0]}]}
    s = snap([station_reading(near_station["stations"][1], near_station["stations"][1]["pose"])])
    assert O.atom(cond("robot_at", "r", "src"), NAMES, near_station, s, POLICY).result == "unknown"


def test_a_wrongly_signed_vertical_does_not_change_robot_at_here_because_the_frame_fusion_drops_it():
    """The planar-target ambiguity is handled in `fuse_frames` (a frame whose vertical disagrees with the IMU's does not
    agree); an all-disagreeing marker has confidence 0 and is not usable."""
    dets = [Detection(np.array([1.0, 0.2, -0.6]), np.eye(3), 2.0, T - 100 * k, 0.1, True) for k in range(10)]
    r = fuse_frames(20, dets, 10)
    assert r.confidence == 0.0
    assert O.atom(cond("robot_at", "r", "src"), NAMES, SITE, snap([r]), POLICY).result == "unknown"


# ---------------------------------------------------------------------------------------------- robot_ready


def ready(**kw):
    base = dict(lowstate_fresh=True, motor_error=0, tilt_rad=math.radians(1), fsm_id=500, stamp_ms=T - 10)
    return O.Ready(**{**base, **kw})


@pytest.mark.parametrize(
    "kw, expected",
    [
        ({}, "true"),
        ({"lowstate_fresh": False}, "unknown"),
        ({"lowstate_fresh": None}, "unknown"),
        ({"motor_error": 4}, "false"),
        ({"motor_error": None}, "unknown"),
        ({"tilt_rad": math.radians(30)}, "false"),
        ({"tilt_rad": None}, "unknown"),
        ({"fsm_id": None}, "unknown"),  # the walking service did not answer
        ({"fsm_id": 1}, "false"),  # damped
        ({"fsm_id": 0}, "false"),  # zero torque
    ],
)
def test_robot_ready(kw, expected):
    assert verdict(cond("robot_ready", "r"), snap(ready=ready(**kw))) == expected
    assert verdict(cond("robot_ready", "r"), snap()) == "unknown"  # not read


# ---------------------------------------------------------------------------------------- logic and policy


def test_three_valued_logic_follows_the_symbolic_simulator():
    bench = at_station()
    s = snap([bench, object_reading(bench, IN_BENCH_REGION)], [hand("right"), hand("left")])
    true, false, unknown = cond("object_at", "box", "src"), cond("holding", "r", "box"), cond("object_at", "box", "dst")
    assert verdict(true, s) == "true" and verdict(false, s) == "false" and verdict(unknown, s) == "unknown"
    assert verdict(ir.And((true, false)), s) == "false" and verdict(ir.And((true, unknown)), s) == "unknown"
    assert verdict(ir.And((false, unknown)), s) == "false" and verdict(ir.And((true, true)), s) == "true"
    assert verdict(ir.Or((false, unknown)), s) == "unknown" and verdict(ir.Or((true, unknown)), s) == "true"
    assert verdict(ir.Or((false, false)), s) == "false"
    assert verdict(ir.Not(true), s) == "false" and verdict(ir.Not(false), s) == "true"
    assert verdict(ir.Not(unknown), s) == "unknown"


def task_of(source):
    return ir.compile_source(source, "t.rob").ir.task("t")


def test_policy_defaults_and_verify_thresholds():
    plain = task_of("robolang 1.3;\ntask t(robot r, location a) { goal { robot_at r a; } plan { move r to a; } }\n")
    assert O.Policy.of(plain) == O.Policy(Decimal("0.9"), 10_000)
    tuned = task_of(
        "robolang 1.3;\ntask t(robot r, location a) {\n  @verify min_confidence 0.7;\n  @verify max_age 2s;\n"
        "  goal { robot_at r a; } plan { move r to a; }\n}\n"
    )
    assert O.Policy.of(tuned) == O.Policy(Decimal("0.7"), 2_000)
    bench = at_station()
    box = object_reading(bench, IN_BENCH_REGION, conf=0.8)
    s = snap([bench, box])
    assert verdict(cond("object_at", "box", "src"), s, POLICY) == "unknown"
    assert verdict(cond("object_at", "box", "src"), s, O.Policy.of(tuned)) == "true"  # 0.8 >= 0.7
    assert (
        verdict(
            cond("object_at", "box", "src"),
            snap([bench, object_reading(bench, IN_BENCH_REGION)]),
            O.Policy(Decimal("0.9"), 100),
        )
        == "unknown"
    )  # a reading 200 ms old against max_age 100 ms


def test_unsupported_conditions_and_the_objects_a_goal_asks_about():
    assert O.unsupported(cond("holding", "r", "box")) is None
    assert O.unsupported(ir.And((cond("holding", "r", "box"), ir.RegionClear("z")))) == "region_clear"
    assert O.unsupported(ir.Not(ir.SensorOk("s"))) == "sensor_ok"
    assert O.held_objects([ir.And((cond("holding", "r", "box"), cond("robot_at", "r", "dst")))], NAMES) == {"box"}
    assert O.held_objects([cond("robot_at", "r", "dst")], NAMES) == set()
    assert O.uses_ready([ir.Or((cond("robot_ready", "r"), cond("robot_at", "r", "src")))])
    assert not O.uses_ready([cond("robot_at", "r", "src")])


# ------------------------------------------------------------------------------------------ frame fusion


def detections(positions, rotations=None, up=None, oriented=False, stamp0=1000):
    return [
        Detection(
            np.array(p, float),
            np.eye(3) if rotations is None else rotations[i],
            1.7,
            stamp0 + 200 * i,
            None if up is None else up[i],
            oriented,
        )
        for i, p in enumerate(positions)
    ]


def test_fusion_confidence_counts_the_frames_that_did_not_see_the_marker():
    seen = detections([(1.0, 0.2, 0.3)] * 9)
    r = fuse_frames(7, seen, 10)  # seen in 9 of the 10 frames taken
    assert r.confidence == pytest.approx(0.9) and r.detections == 9 and r.frames == 10
    assert Decimal(str(r.confidence)) >= POLICY.min_confidence  # exactly the default: usable
    assert fuse_frames(7, detections([(1.0, 0.2, 0.3)] * 8), 10).confidence == pytest.approx(0.8)


def test_fusion_median_position_spread_and_the_age_of_the_newest_agreeing_frame():
    pts = [(1.0, 0.2, 0.3)] * 8 + [(1.0, 0.2, 0.36), (1.0, 0.2, 0.27)]  # two frames 6 and 3 cm off the median
    r = fuse_frames(7, detections(pts), 10)
    assert r.agreeing == 8 and r.confidence == pytest.approx(0.8) and r.position[2] == pytest.approx(0.3)
    assert r.stamp_ms == 1000 + 200 * 7  # the newest agreeing frame, not the outliers after it
    assert r.age_ms(3000) == 3000 - r.stamp_ms
    assert r.spread_m == pytest.approx(0, abs=1e-9)


def test_fusion_tolerance_is_two_centimetres():
    inside = fuse_frames(7, detections([(1.0, 0.2, 0.3)] * 8 + [(1.0, 0.2, 0.315)] * 2), 10)  # 1.5 cm off: agrees
    assert inside.confidence == 1.0
    outside = fuse_frames(7, detections([(1.0, 0.2, 0.3)] * 8 + [(1.0, 0.2, 0.35)] * 2), 10)  # 5 cm off: does not
    assert outside.confidence == pytest.approx(0.8)


def test_fusion_station_markers_must_agree_in_rotation_and_with_the_imus_vertical():
    def tilt(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    pts = [(1.0, 0.2, -0.6)] * 10
    good = fuse_frames(20, detections(pts, [tilt(0.01)] * 10, [0.005] * 10, oriented=True), 10)
    assert good.confidence == 1.0
    flipped = fuse_frames(20, detections(pts, [tilt(0.01)] * 7 + [tilt(0.25)] * 3, [0.005] * 10, oriented=True), 10)
    assert flipped.confidence == pytest.approx(0.7)  # three frames have the other solution's rotation
    off_vertical = fuse_frames(20, detections(pts, [tilt(0.01)] * 10, [0.005] * 8 + [0.06] * 2, oriented=True), 10)
    assert off_vertical.confidence == pytest.approx(0.8)
    unoriented = fuse_frames(7, detections(pts, [tilt(0.01)] * 7 + [tilt(0.25)] * 3), 10)  # an object marker's
    assert unoriented.confidence == 1.0  # rotation is not used, so it is not asked to agree


# --------------------------------------------------------------------------------------------- Core outcome


def core_ok(core):
    """What the robolang.core checks demand of an outcome: the schema, the evidence's signatures, and C402."""
    schema = load_schema("message-0.1.schema.json")
    validator = _Validator(schema)
    assert validator.errors(core, {"$ref": "#/$defs/payload_outcome"}) == []
    for observation in core["evidence"]:
        assert verify_sim(observation, O.SIM_KEY) and observation["source"]["participant"] == O.SIM_PARTICIPANT
    from robolang.core import Context

    receipt = check_meaning(
        {"type": "outcome", "schema_version": "0.1", "payload": core},
        Context("rl:sim:x", 0, 0, 0, {O.SIM_KEY_REF: O.SIM_KEY}),
    )
    assert receipt.accepted, receipt
    return core["outcome"]


GOAL_SRC = """robolang 1.3;
task t(robot r, object box, location src, location dst) {
    goal { object_at box src; holding r box; }
    plan { move r to src; }
}
"""
BIND = {"r": "robot_1", "box": "box_1", "src": "loc_a", "dst": "loc_b"}


def test_core_outcome_of_a_success_cites_signed_observations_and_is_accepted():
    task = task_of(GOAL_SRC)
    bench = at_station()
    box = object_reading(bench, IN_BENCH_REGION)
    # "box at src" is true, "holding" is not: this is not a success; use a goal of the first atom only for succeeded
    ok_task = task_of(GOAL_SRC.replace(" holding r box;", ""))
    s = snap([bench, box], [hand("right"), hand("left")])
    core = O.core_outcome(ok_task, BIND, NAMES, SITE, s, POLICY, "SUCCEEDED", None, "abc")
    outcome = core_ok(core)
    assert outcome["status"] == "succeeded" and outcome["verification"]["criteria_results"] == [
        {"criterion": 0, "met": True, "evidence": ["rl:obs:abc-0"]}
    ]
    (evidence,) = core["evidence"]
    assert (evidence["subject"], evidence["property"], evidence["value"]) == (
        "rl:thing:box_1",
        "located_in",
        "rl:place:loc_a",
    )
    assert evidence["confidence"] == "1" and evidence["valid_for"] == {"value": "10000", "unit": "ms"}
    failed = O.core_outcome(task, BIND, NAMES, SITE, s, POLICY, "FAILED", "S301", "abc")
    assert core_ok(failed)["status"] == "failed"
    met = [r["met"] for r in failed["outcome"]["verification"]["criteria_results"]]
    assert met == [True, False]  # the verification still says which criteria the readings support


def test_core_outcome_of_an_unknown_goal_has_no_evidence_and_is_unknown():
    task = task_of(GOAL_SRC.replace(" holding r box;", ""))
    core = O.core_outcome(task, BIND, NAMES, SITE, snap([]), POLICY, "FAILED", "S302", "abc")
    outcome = core_ok(core)
    assert outcome["status"] == "unknown" and core["evidence"] == []
    assert outcome["reasons"] == [{"code": "evidence_missing"}]
    assert outcome["verification"]["criteria_results"][0]["met"] is False


def test_a_goal_without_a_core_form_has_no_core_outcome():
    task = task_of("robolang 1.3;\ntask t(robot r, location a) { goal { robot_ready r; } plan { move r to a; } }\n")
    assert (
        O.core_outcome(
            task,
            {"r": "robot_1", "a": "loc_a"},
            {"r": "g1", "a": "bench"},
            SITE,
            snap(),
            POLICY,
            "SUCCEEDED",
            None,
            "x",
        )
        is None
    )
    ornot = task_of("robolang 1.3;\ntask t(robot r, location a) { goal { not robot_at r a; } plan { move r to a; } }\n")
    assert (
        O.core_outcome(
            ornot,
            {"r": "robot_1", "a": "loc_a"},
            {"r": "g1", "a": "bench"},
            SITE,
            snap(),
            POLICY,
            "FAILED",
            "S301",
            "x",
        )
        is None
    )


def test_confidence_text_rounds_down_so_it_never_overstates():
    assert O.confidence_text(0.999) == "0.99" and O.confidence_text(1.0) == "1" and O.confidence_text(0.9) == "0.9"
