"""The choice of a correction from the measured response tables (response.py): pure, no stand-in."""

import math
import random

import pytest

from robolang_g1 import response
from robolang_g1.ops import SERVO_TOL_M, SERVO_TOL_RAD
from robolang_g1.response import CONFIRM_DWELL_S, RESPONSE, RESPONSE_CARRY, expected, plan_correction

CREEP = 0.007


def test_the_tables_hold_measured_corrections_of_one_or_two_bursts_and_the_wait():
    for table in (RESPONSE, RESPONSE_CARRY):
        assert table[()] == (0.0, 0.0, 0.0)
        for key, (forward, left, heading) in table.items():
            assert len(key) <= 2 and all(len(burst) == 4 for burst in key)
            assert math.hypot(forward, left) < 0.35 and abs(heading) < 0.25  # nothing here is a walk or a turn
            for vx, vy, wz, seconds in key:
                assert 0.0 < seconds <= 1.2 and abs(wz) <= 0.2 and abs(vx) <= 0.4 and abs(vy) <= 0.3
                assert math.hypot(vx, vy, wz) >= 0.3  # the policy ignores a command of a smaller norm


def test_an_error_in_the_heading_alone_is_not_made_worse_and_the_robot_goes_nowhere():
    """A turn that goes nowhere: two sideways bursts, one each way, with the yaw rate held (or just waiting: the robot's
    heading creeps to the left by itself)."""
    for carrying in (False, True):
        for heading in (0.09, -0.09, 0.15, -0.15):
            key, _ = plan_correction((0.0, 0.0, heading), SERVO_TOL_M, SERVO_TOL_RAD, CREEP, carrying=carrying)
            forward, left, turned = expected(key, carrying)
            assert abs(heading - turned - CREEP * 5.5) <= abs(heading) + 0.02, (heading, key)
            assert math.hypot(forward, left) < 0.15, (heading, key)


def test_an_error_in_position_is_cancelled_by_a_burst_that_moves_the_robot_that_way():
    for error, axis, sign in (((0.12, 0.0, 0.0), 0, 1), ((-0.12, 0.0, 0.0), 0, -1), ((0.0, 0.12, 0.0), 1, 1),
                              ((0.0, -0.12, 0.0), 1, -1)):
        for carrying in (False, True):
            key, cost = plan_correction(error, SERVO_TOL_M, SERVO_TOL_RAD, CREEP, carrying=carrying)
            moved = (RESPONSE_CARRY if carrying else RESPONSE)[key][axis]
            assert moved * sign > 0.03, (error, carrying, key)


def test_a_correction_that_does_not_do_the_same_twice_is_not_in_the_tables():
    """The 0.2 s step back takes a step half the time (0.07 m or nothing, measured over 80 servo corrections)."""
    for table in (RESPONSE, RESPONSE_CARRY):
        assert not any(len(key) == 1 and key[0][0] == -0.4 and key[0][3] == 0.2 for key in table)


def test_a_loaded_robot_is_planned_from_its_own_table():
    """The same sideways pair of bursts moves an empty robot 0.07 m to the left and a loaded one 0.0 m: the tables hold
    what each does, and the plan for the same error is made from the table of the robot's state."""
    pair = ((0.1, -0.3, 0.2, 0.8), (0.1, 0.3, 0.2, 0.8))
    assert RESPONSE[pair][1] > 0.05 and abs(RESPONSE_CARRY[pair][1]) < 0.02
    plans = set()
    for error in ((0.0, 0.1, -0.1), (0.05, 0.0, 0.08), (-0.1, 0.04, 0.06), (0.0, -0.08, 0.1), (0.07, 0.07, -0.05)):
        plans.add(plan_correction(error, SERVO_TOL_M, SERVO_TOL_RAD, CREEP)[0]
                  == plan_correction(error, SERVO_TOL_M, SERVO_TOL_RAD, CREEP, carrying=True)[0])
    assert plans == {True, False}  # some errors are cancelled the same way, some not


def test_a_banned_correction_is_not_chosen_again():
    error = (0.0, 0.0, -0.09)
    first, _ = plan_correction(error, SERVO_TOL_M, SERVO_TOL_RAD, CREEP)
    second, _ = plan_correction(error, SERVO_TOL_M, SERVO_TOL_RAD, CREEP, banned={first})
    assert second != first


@pytest.mark.parametrize("carrying", [False, True])
@pytest.mark.parametrize("creep_true, gain", [(0.007, 1.0), (0.02, 1.0), (0.007, 0.7), (0.007, 1.3)])
def test_the_plan_converges_on_a_noisy_plant_that_differs_from_the_table(carrying, creep_true, gain):
    """The servo loop of `ops._servo` on a plant made from the table: each correction does `gain` times what the table
    says, plus noise as large as the spreads of the bursts that vary most (2 cm, 0.03 rad), the heading creeps
    `creep_true` (the table's `CREEP`: 0.007), the marker is seen to 1 cm and 0.01 rad, and the robot learns the creep
    from the corrections that turn it little and from the wait before a confirmation. From any error inside the
    approach tolerance the pose must be confirmed, within the confirm limits, in a few corrections."""
    table = RESPONSE_CARRY if carrying else RESPONSE
    confirm_m, confirm_rad = 0.042, 0.09
    rnd = random.Random(7)
    counts = []
    for _ in range(100):
        e = [rnd.uniform(-0.08, 0.08), rnd.uniform(-0.08, 0.08), rnd.uniform(-0.07, 0.07)]
        creep, n = CREEP, 0
        while True:
            ex, ey, eyaw = e
            if math.hypot(ex, ey) <= SERVO_TOL_M and abs(eyaw - creep * CONFIRM_DWELL_S) <= SERVO_TOL_RAD:
                after_wait = eyaw - creep_true * CONFIRM_DWELL_S
                creep = 0.5 * creep + 0.5 * ((eyaw - after_wait) / CONFIRM_DWELL_S + rnd.gauss(0, 0.005))
                if math.hypot(ex, ey) <= confirm_m and abs(after_wait + rnd.gauss(0, 0.01)) <= confirm_rad:
                    break
                e = [ex, ey, after_wait]
                continue
            key, _ = plan_correction((ex, ey, eyaw), SERVO_TOL_M, SERVO_TOL_RAD, creep, carrying=carrying)
            fx, fy, fw = table[key]
            n += 1
            assert n <= 40, (e, creep)
            noise = (rnd.gauss(0, 0.02), rnd.gauss(0, 0.02), rnd.gauss(0, 0.03)) if key else (0.0, 0.0, 0.0)
            e = [ex - gain * fx - noise[0], ey - gain * fy - noise[1],
                 eyaw - gain * fw - creep_true * response.SETTLE_LOOK_S - noise[2]]
            if abs(fw) <= 0.06:  # the creep, from what the heading did beyond what the table says
                rate = ((eyaw - e[2]) - fw) / response.SETTLE_LOOK_S + rnd.gauss(0, 0.005)
                creep = 0.7 * creep + 0.3 * rate
        counts.append(n)
    assert sum(counts) / len(counts) < 8 and sorted(counts)[94] <= 25  # (a creep three times the measured one: 7)


def _ops():
    from robolang_g1 import site as site_mod
    from robolang_g1.ops import G1Operations
    from robolang_g1.phase2 import SITES

    return G1Operations(site_mod.load_site(SITES / "pick_place.json"))


def test_the_belief_after_a_correction_is_what_the_table_says_it_does():
    ops = _ops()
    correction = ((0.1, -0.3, 0.2, 0.8),)
    forward, left, turned = expected(correction)
    ops.pose = [0.0, 0.0, 0.31]
    ops._expect([1.0, 2.0, math.pi / 2], correction)  # facing +y: forward is +y, left is -x
    assert turned != 0  # ... and the table's heading is not the belief's: that is the IMU's (heading.py)
    assert ops.pose == pytest.approx([1.0 - left, 2.0 + forward, 0.31])


def test_the_creep_is_learned_from_a_correction_that_turned_the_robot_little_and_not_from_one_that_turned_it_a_lot():
    ops = _ops()
    assert ops.creep == pytest.approx(0.007)
    wait = ()  # nothing commanded: the heading moved 0.04 rad in 2 s of waiting
    ops.pose = [1.0, 1.0, 0.04]
    ops._learn(wait, [1.0, 1.0, 0.0], 2000)
    assert 0.007 < ops.creep <= 0.3 * 0.02 + 0.7 * 0.007 + 1e-9  # a step toward 0.02, not all the way
    before = ops.creep
    big_turn = ((0.1, -0.3, 0.2, 0.8), (0.1, 0.3, 0.2, 0.8))
    ops.pose = [1.0, 1.0, 0.30]
    ops._learn(big_turn, [1.0, 1.0, 0.0], 5000)  # it turns 0.15: its own mismatch must not pass for creep
    assert ops.creep == before
    ops.pose = [1.0, 1.0, 0.05]
    ops._learn(wait, [1.0, 1.0, 0.0], 500)  # too short a wait to say anything
    assert ops.creep == before


def test_a_correction_that_does_not_do_what_the_table_says_is_believed_less_each_time_it_is_sent():
    """A loaded robot does not take the step to the left that the table says a 0.5 s sideways burst makes beside the
    shelf: after it has been seen not to, the robot's belief of what it does is that much less."""
    ops = _ops()
    ops.held = {"object": "box", "hand": "right"}
    burst = ((0.1, 0.3, 0.2, 0.5),)
    table = RESPONSE_CARRY[burst]
    assert table[1] > 0.02
    for _ in range(4):
        ops.pose = [2.0, 0.5, 0.0]
        ops._learn(burst, [2.0, 0.5, 0.0], 3000)  # it did nothing: the belief did not move
    assert ops.seen[(True, burst)][0] == 4
    believed = response.expected(burst, True, ops.seen)
    assert believed[1] <= table[1] / 2 + 1e-9 and believed[0] <= table[0] / 2 + 1e-9
    ops._expect([2.0, 0.5, 0.0], burst)  # the belief after it is what has been seen, not what the table says
    assert ops.pose[1] - 0.5 == pytest.approx(believed[1]) and ops.pose[1] - 0.5 < table[1]
    # and an observation far off the table's (a reading that jumped) is not counted
    ops.pose = [2.5, 0.5, 0.0]
    ops._learn(burst, [2.0, 0.5, 0.0], 3000)
    assert ops.seen[(True, burst)][0] == 4


def test_the_servo_stops_inside_its_own_tolerances_and_goes_on_from_wider_ones_when_it_stagnates():
    from robolang_g1 import ops

    assert ops.servo_band(False, False, 0.035, 0.05) == (0.035, 0.05)
    assert ops.servo_band(True, False, 0.08, 0.07) == (0.08, 0.07)
    # stagnated, the final servo goes on to the confirming look from the confirm limit less what the creep may add ...
    assert ops.servo_band(False, True, 0.035, 0.05) == (ops.STATION_TOL_M, ops.RELAXED_RAD)
    assert ops.RELAXED_RAD < ops.STATION_TOL_RAD and ops.STATION_TOL_M <= 0.05  # (the station tolerance, 5 cm 0.1 rad)
    # ... and the approach point, which no confirmation follows, from a wider band the final servo can still walk out of
    band_m, band_rad = ops.servo_band(True, True, 0.08, 0.07)
    assert band_m < ops.COARSE_M and band_rad < ops.COARSE_RAD and band_m > 0.08 and band_rad > 0.07


def test_a_leg_straight_to_the_side_is_walked_as_two_diagonals_the_policy_answers():
    from robolang_g1 import ops

    # the commands walk_to sends toward a target to the side: (about 0, 0.3, 0), which the loaded robot answers or not
    legs = ops.walk_legs(0.01, 0.3, 0.0, 3.4)
    assert len(legs) == 2 and sum(leg[3] for leg in legs) == 3.4
    assert all(abs(leg[0]) == ops.LATERAL_VX and leg[1] == 0.3 for leg in legs)
    assert abs(legs[0][0] * legs[0][3] + legs[1][0] * legs[1][3]) < 1e-9  # no net forward command
    assert legs[1][0] > 0 > legs[0][0]  # the leg's own forward component pointed forward: it ends that way
    assert ops.walk_legs(-0.05, -0.3, 0.1, 3.4)[1][0] < 0  # and backward the other way, the yaw rate kept
    assert all(leg[2] == 0.1 for leg in ops.walk_legs(-0.05, -0.3, 0.1, 3.4))
    # a short leg is one diagonal; a leg with a forward or backward part of its own, or mostly forward, is left alone
    assert ops.walk_legs(0.0, 0.3, 0.0, 1.0) == [(0.2, 0.3, 0.0, 1.0)]
    assert ops.walk_legs(0.25, 0.3, 0.0, 3.4) == [(0.25, 0.3, 0.0, 3.4)]
    assert ops.walk_legs(-0.41, 0.28, 0.0, 3.4) == [(-0.41, 0.28, 0.0, 3.4)]
    assert ops.walk_legs(0.5, 0.0, 0.0, 3.4) == [(0.5, 0.0, 0.0, 3.4)]


def test_a_move_to_a_station_the_robot_stands_beside_is_the_servo_alone():
    """The same station, or one a step away (the dock 12 cm behind a destination): no walk out to the approach point
    0.5 m short and back. Any other move, a first move, or a marker not seen now: the usual walk (None)."""
    from robolang_g1 import ops as ops_mod

    class Reading:
        confidence, range_m = 0.95, 1.4

    ops = _ops()
    shelf = ops.stations["shelf"]
    calls = []
    ops.look = lambda frames=4, ids=None: {shelf["marker"]["id"]: Reading()}
    ops.now_ms = lambda: 0

    class Tracker:
        def summary(self):
            return {}

    ops.tracker = Tracker()
    ops._servo = lambda station, reading, deadline: calls.append(station["name"]) or {"settled": True}
    gx, gy, gyaw = shelf["pose"]
    ops.pose = [gx - 0.12, gy, gyaw]
    assert ops._direct(shelf, 10_000) is None and calls == []  # no station arrived at yet: a first move walks
    ops.station = "bench"
    out = ops._direct(shelf, 10_000)
    assert out["direct"] is True and calls == ["shelf"] and ops.station == "shelf"
    ops.pose = [gx - ops_mod.DIRECT_M - 0.01, gy, gyaw]
    assert ops._direct(shelf, 10_000) is None  # too far: walk
    ops.pose = [gx, gy, gyaw + ops_mod.DIRECT_RAD + 0.01]
    assert ops._direct(shelf, 10_000) is None  # turned too far: walk
    ops.pose = [gx, gy, gyaw]
    ops.look = lambda frames=4, ids=None: {}
    assert ops._direct(shelf, 10_000) is None and calls == ["shelf"]  # its marker not seen now: walk


def test_a_place_out_of_reach_at_the_centre_aims_nearer_the_robot_inside_the_region():
    """S502 at the region's centre: the place tries 1.5 cm and then 3 cm nearer the pelvis's centre line, across the
    region, never closer than 3 cm to its edge; S502 when none is reachable."""
    import numpy as np

    from robolang_g1 import site as site_mod
    from robolang_g1.executor import BackendFailure
    from robolang_g1.ops import PLACE_INSIDE_M

    class Stop(Exception):
        pass

    ops = _ops()
    shelf = ops.stations["shelf"]
    loc = site_mod.location(ops.site, "shelf")
    R = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # the plate facing the robot: x to its right

    class Reading:
        confidence, rotation, position = 0.95, R, np.array([1.35, 0.45, -0.6])

    ops.held = {"object": "box", "hand": "right", "offset_in_hand": np.zeros(3), "fingers_down_deg": 0}
    ops.station = "shelf"
    ops._reposition = lambda: {}
    ops._arm_stance = lambda model: model
    ops._place_accept = lambda side, half, start: None
    ops.robot = type("R", (), {"low_q": np.zeros(29)})()
    ops.look = lambda frames=4, ids=None: {shelf["marker"]["id"]: Reading()}
    ops.now_ms = lambda: 0
    tried = []

    def plan_reach(side, target_for, **kw):
        tried.append(target_for(np.eye(3)))
        if len(tried) < reach_at:
            raise BackendFailure("S502", "no inverse-kinematics solution")
        raise Stop

    ops._plan_reach = plan_reach
    for reach_at, n in ((1, 1), (2, 2), (3, 3)):
        tried.clear()
        with pytest.raises(Stop):
            ops.place("box", "shelf")
        assert len(tried) == n
    lateral = [t[1] for t in tried]
    centre = 0.45 - (loc["region"]["min"][0] + loc["region"]["max"][0]) / 2  # the region centre is to the right
    assert lateral == pytest.approx([centre, centre + 0.015, centre + 0.03])  # toward the centre line
    reach_at = 99
    tried.clear()
    with pytest.raises(BackendFailure) as err:
        ops.place("box", "shelf")
    assert err.value.code == "S502" and len(tried) == 3
    assert (loc["region"]["max"][0] - loc["region"]["min"][0]) / 2 - 0.03 >= PLACE_INSIDE_M - 1e-9


def test_a_stop_is_not_confirmed_while_the_body_still_slides_under_the_velocity_bounds():
    """x86 CI, E5: every joint under 0.09 rad/s and the IMU under 0.045 rad/s, but the legs travelled 0.03 rad in the
    window and the body slid 1.6 cm. The travel bound waits for a still window; a quiet stance confirms at once."""
    import numpy as np

    from robolang_g1 import ops as ops_mod
    from robolang_g1.robot import LowSample

    def run(drift_rad_s, until_ms=None):
        ops = _ops()
        clock = {"t": 1000}
        samples = []

        def tick():
            t = clock["t"]
            moving = until_ms is None or t < until_ms
            q = np.zeros(29)
            q[ops_mod.LEG_AND_WAIST] = (drift_rad_s if moving else 0.0) * (min(t, until_ms or t) - 1000) / 1000
            dq = np.zeros(29)
            dq[ops_mod.LEG_AND_WAIST] = drift_rad_s if moving else 0.0
            samples.append(LowSample(t, q, dq, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.01)))
            clock["t"] += 4

        class Robot:
            @staticmethod
            def samples_since(t0):
                return [s for s in samples if s.tick >= t0]

        ops.robot = Robot()
        ops.tilt_rad = lambda: 0.0

        def now():
            tick()
            return clock["t"]

        ops.now_ms = now
        ops_mod.time.sleep, sleep = (lambda s: None), ops_mod.time.sleep
        try:
            return ops.confirm_stopped()
        finally:
            ops_mod.time.sleep = sleep

    quiet = run(0.0)
    assert quiet["confirmed"] and quiet["leg_travel_rad"] <= 0.001 and quiet["after_ms"] <= 600
    sliding = run(0.06)  # 0.06 rad/s: under the velocity bound, 0.03 rad over a window
    assert sliding["max_joint_velocity_rad_s"] <= ops_mod.CONFIRM_JOINT_VEL and not sliding["confirmed"]
    settles = run(0.06, until_ms=2000)  # slides for a second, then stands still
    assert settles["confirmed"] and settles["window_start_ms"] >= 2000 - 4


def test_an_arrival_or_arm_work_is_not_confirmed_from_a_reading_the_localization_rejected():
    """F1 (acceptance run 3): every reading of the station's plate was rejected (its vertical not the IMU's), the belief
    stayed at the dead reckoning, 3.5 cm from the station, while the robot stood 0.45 m away. Seen is not enough: the
    look must have fixed the pose from the marker."""
    from robolang_g1.executor import BackendFailure
    from robolang_g1.ops import MARKER_MIN_CONF

    class Reading:
        confidence, range_m = 0.95, 1.75

    ops = _ops()
    shelf = ops.stations["shelf"]
    mid = shelf["marker"]["id"]
    ops.now_ms = lambda: 0
    ops.settle = lambda: True
    ops.station = "shelf"
    ops.pose = list(shelf["pose"])  # the belief says: at the station

    def look(fixed):
        def f(frames=4, ids=None):
            ops.last_look = (0, fixed)
            return {mid: Reading()}
        return f

    ops.look = look(True)
    assert ops._confirm_pose(shelf) is True
    assert ops._reposition()["error_m"] == 0.0
    ops.look = look(False)  # the same reading, rejected
    assert ops._confirm_pose(shelf) is False
    with pytest.raises(BackendFailure) as err:
        ops._reposition()
    assert err.value.code == "S517" and "no pose fix" in str(err.value)
    assert Reading.confidence >= MARKER_MIN_CONF
