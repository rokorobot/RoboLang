"""The choice of a correction from the measured response tables (response.py): pure, no stand-in."""

import math
import random

import pytest

from robolang_g1 import response
from robolang_g1.ops import SERVO_TOL_M, SERVO_TOL_RAD
from robolang_g1.response import CONFIRM_DWELL_S, RESPONSE, RESPONSE_CARRY, plan_correction

CREEP = 0.007


def test_the_tables_hold_measured_corrections_of_one_or_two_bursts_and_the_wait():
    for table in (RESPONSE, RESPONSE_CARRY):
        assert table[()] == (0.0, 0.0, 0.0)
        assert set(table) == set(RESPONSE) == set(RESPONSE_CARRY)
        for key, (forward, left, heading) in table.items():
            assert len(key) <= 2 and all(len(burst) == 4 for burst in key)
            assert math.hypot(forward, left) < 0.35 and abs(heading) < 0.25  # nothing here is a walk or a turn
            for vx, vy, wz, seconds in key:
                assert 0.0 < seconds <= 1.2 and abs(wz) <= 0.2 and abs(vx) <= 0.4 and abs(vy) <= 0.3
                assert math.hypot(vx, vy, wz) >= 0.3  # the policy ignores a command of a smaller norm


def test_a_small_error_in_the_heading_alone_is_cancelled_without_going_anywhere():
    """A turn that goes nowhere: two sideways bursts, one each way, with the yaw rate held (or just waiting: the robot's
    heading creeps to the left by itself)."""
    for sign in (1, -1):
        key, cost = plan_correction((0.0, 0.0, sign * 0.09), SERVO_TOL_M, SERVO_TOL_RAD, CREEP)
        forward, left, heading = RESPONSE[key]
        assert math.hypot(forward, left) < 0.06 and cost < 1.5, key


def test_an_error_in_position_is_cancelled_by_a_burst_that_moves_the_robot_that_way():
    for error, axis, sign in (((0.12, 0.0, 0.0), 0, 1), ((-0.12, 0.0, 0.0), 0, -1), ((0.0, 0.12, 0.0), 1, 1),
                              ((0.0, -0.12, 0.0), 1, -1)):
        for carrying in (False, True):
            key, cost = plan_correction(error, SERVO_TOL_M, SERVO_TOL_RAD, CREEP, carrying=carrying)
            moved = (RESPONSE_CARRY if carrying else RESPONSE)[key][axis]
            assert moved * sign > 0.03, (error, carrying, key)


def test_a_loaded_robot_is_planned_from_its_own_table():
    """A step to the left with a turn to the right moves an empty robot 0.16 m and a loaded one not at all: the loaded
    robot is not sent that way."""
    burst = ((0.0, 0.3, -0.2, 1.2),)
    assert RESPONSE[burst][1] > 0.1 and RESPONSE_CARRY[burst][1] < 0.02
    error = (0.0, 0.15, -0.15)
    assert plan_correction(error, SERVO_TOL_M, SERVO_TOL_RAD, CREEP, carrying=False)[0] == burst
    assert plan_correction(error, SERVO_TOL_M, SERVO_TOL_RAD, CREEP, carrying=True)[0] != burst


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
    assert sum(counts) / len(counts) < 7 and sorted(counts)[94] <= 20
