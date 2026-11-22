"""The belief heading between marker fixes comes from the IMU's yaw (heading.py), and the stand-in's IMU is not ideal
(sim/imu.py): bias, noise and a yaw that drifts."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from robolang_g1 import heading, ops
from robolang_g1.sim.imu import BIAS_SIGMA, Imu
from robolang_g1.site import wrap


def quat_about_z(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def test_the_yaw_of_a_quaternion_is_the_rotation_about_the_vertical():
    assert heading.yaw_of(quat_about_z(0.7)) == pytest.approx(0.7)
    assert heading.yaw_of(quat_about_z(-3.0)) == pytest.approx(-3.0)
    tilted = np.array([math.cos(0.05), math.sin(0.05), 0, 0])  # a roll of 0.1 rad does not change the yaw
    assert heading.yaw_of(tilted) == pytest.approx(0.0, abs=1e-12)


def test_between_fixes_the_heading_is_the_fixs_plus_what_the_imu_turned():
    t = heading.HeadingTracker(1.0, 5.0, 0)  # the map heading 1.0 when the IMU (its own frame) says 5.0 - 2*pi
    assert t.heading(5.0, 0) == pytest.approx(1.0)
    assert t.heading(5.3, 2000) == pytest.approx(1.3)
    assert t.heading(5.0 - 2 * math.pi + 0.25, 3000) == pytest.approx(1.25)  # across the IMU's wrap at +-pi
    assert t.heading(wrap(5.0 + 3.5), 3000) == pytest.approx(wrap(1.0 + 3.5))  # across the map's too


def test_a_marker_fix_resets_the_belief_and_says_how_far_it_had_drifted():
    t = heading.HeadingTracker(0.0, 0.0, 0)
    jump = t.fix(0.12, 0.10, 8000)  # the IMU thinks 0.10 turned, the marker says 0.12
    assert jump == pytest.approx(0.02)
    assert t.heading(0.10, 8000) == pytest.approx(0.12)
    assert t.heading(0.35, 9000) == pytest.approx(0.12 + 0.25)


def test_the_bias_is_learned_from_the_fixes_and_the_heading_then_stays_right_without_one():
    rng = np.random.default_rng(1)
    bias, t = 0.003, heading.HeadingTracker(0.0, 0.0, 0)
    truth, offset = 0.0, 0.4  # the true heading in the map; the IMU's own frame differs from the map's by `offset`
    for k in range(1, 13):  # a fix every 10 s; the robot turns by a different amount each time
        truth += rng.uniform(-0.3, 0.3)
        t.fix(wrap(truth + rng.normal(0, 0.005)), truth - offset + bias * 10 * k, 10_000 * k)
    assert t.bias == pytest.approx(bias, rel=0.35)  # (a marker's heading is good to a few thousandths: not exact)
    # 30 s with no marker, the robot standing: the IMU's yaw has drifted by bias x 30 = 0.09 rad
    imu_later = truth - offset + bias * 150
    error = abs(t.heading(imu_later, 150_000) - truth)
    assert error < 0.35 * bias * 30  # a third or less of the drift an uncorrected IMU would have
    uncorrected = heading.HeadingTracker(wrap(truth), truth - offset + bias * 120, 120_000)
    assert abs(uncorrected.heading(imu_later, 150_000) - truth) == pytest.approx(bias * 30)


def test_fixes_closer_than_the_minimum_wait_until_the_pair_is_long_enough_to_teach():
    t = heading.HeadingTracker(0.0, 0.0, 0)
    t.fix(0.0, 0.0, 1000)  # the reference
    for k in range(2, 5):  # fixes every second: each is closer than MIN_PAIR_S to the reference, none teaches
        t.fix(0.0, 0.003 * (k - 1), 1000 * k)
    assert t.bias == 0.0
    t.fix(0.0, 0.003 * 5, 6000)  # now 5 s from the reference: 0.015 rad of IMU turn the marker did not see
    assert 0.0 < t.bias < 0.003  # a step toward 0.003 (the prior holds it back)


def test_an_impossible_rate_between_two_fixes_is_a_bad_fix_not_a_bias():
    t = heading.HeadingTracker(0.0, 0.0, 0)
    t.fix(0.0, 0.0, 1000)
    t.fix(0.0, 1.0, 11_000)  # 0.1 rad/s of "gyro bias"
    assert t.bias == 0.0
    assert abs(t.jumps[-1][1]) > 0.9  # (and the fix still resets the belief)


def test_the_bias_is_clipped_and_the_summary_reports_the_fixes():
    t = heading.HeadingTracker(0.0, 0.0, 0)
    for k in range(1, 30):
        t.fix(0.0, 0.019 * 10 * k, 10_000 * k)  # 0.019 rad/s: believed only up to the clip
    assert abs(t.bias) <= heading.BIAS_CLIP
    s = t.summary()
    assert s["source"] == "imu_yaw" and s["fixes"] == 29 and s["bias_rad_s"] == pytest.approx(t.bias, abs=1e-5)


# ---------------------------------------------------------------------------------------------- the operations


class FakeRobot:
    """What the operations read of the robot: lowstate samples with the IMU quaternion."""

    def __init__(self):
        self.samples = []
        self.last = None

    def add(self, tick_ms, yaw):
        self.last = SimpleNamespace(tick=tick_ms, quat=quat_about_z(yaw))
        self.samples.append(self.last)

    def samples_since(self, tick_ms):
        return [s for s in self.samples if s.tick >= tick_ms]

    def sample_at(self, tick_ms):
        return min(self.samples, key=lambda s: abs(s.tick - tick_ms)) if self.samples else None


def make_ops():
    o = ops.G1Operations.__new__(ops.G1Operations)
    o.robot, o._xy, o._heading0 = FakeRobot(), [1.0, 2.0], 0.5
    o.robot.add(0, 2.0)  # the IMU's own frame: 2.0 rad when the map heading is 0.5
    o.tracker = heading.HeadingTracker(0.5, 2.0, 0)
    return o


def test_the_belief_pose_reads_the_imu_now():
    o = make_ops()
    assert o.pose == pytest.approx([1.0, 2.0, 0.5])
    o.robot.add(5000, 2.2)  # the robot turned 0.2 rad (a heading creep nobody commanded)
    assert o.pose == pytest.approx([1.0, 2.0, 0.7])


def test_a_walk_that_turns_as_it_goes_is_dead_reckoned_along_the_mean_heading_not_the_one_it_started_with():
    o = make_ops()
    for k in range(0, 3001, 20):  # 3 s turning 0.4 rad: the heading goes from 0.5 to 0.9
        o.robot.add(k, 2.0 + 0.4 * k / 3000)
    o._dead_reckon(0.5, 0.0, 0.0, 3.0, False, 0, 3000)
    run = ops.WALK_GAIN * 0.5 * 3.0
    mean = 0.5 + 0.2
    assert o.pose[0] == pytest.approx(1.0 + run * math.cos(mean), abs=0.01)
    assert o.pose[1] == pytest.approx(2.0 + run * math.sin(mean), abs=0.01)
    along_start = (1.0 + run * math.cos(0.5), 2.0 + run * math.sin(0.5))  # what the commanded heading would have said
    assert math.dist(o.pose[:2], along_start) > 0.1


def test_a_station_marker_fix_sets_the_position_and_the_heading_and_the_imu_carries_it_on():
    o = make_ops()
    o.robot.add(4000, 2.3)  # the IMU: turned 0.3 since the start, so the belief is 0.8
    jump = o._fix_pose((3.0, 4.0, 0.9), 4000)  # the marker says 0.9
    assert jump == pytest.approx(0.1)
    assert o.pose == pytest.approx([3.0, 4.0, 0.9])
    o.robot.add(6000, 2.35)
    assert o.pose[2] == pytest.approx(0.95)


def test_setting_the_pose_takes_it_as_true_and_learns_nothing():
    o = make_ops()
    o.robot.add(1000, 2.1)
    o.pose = [9.0, 8.0, 1.5]
    assert o.pose == pytest.approx([9.0, 8.0, 1.5]) and o.tracker.bias == 0.0 and o.tracker.marker is None


# ---------------------------------------------------------------------------------------------- the stand-in's IMU


def run_imu(imu, seconds, hz=250, true_yaw=0.3):
    out = None
    for k in range(int(seconds * hz) + 1):
        out = imu.read(k / hz, quat_about_z(true_yaw), (0.0, 0.0, 0.0), (0.0, 0.0, 9.81))
    return out


def test_an_ideal_imu_publishes_the_simulators_sensors_untouched():
    q, g, a = run_imu(Imu(ideal=True), 1.0)
    assert tuple(q) == quat_about_z(0.3) and tuple(g) == (0.0, 0.0, 0.0) and tuple(a) == (0.0, 0.0, 9.81)


def test_the_yaw_drifts_at_the_gyros_bias_and_roll_and_pitch_are_not_touched():
    imu = Imu(seed=3, bias=0.004)
    imu.bias[:2] = 0.0
    q, gyro, acc = run_imu(imu, 60.0)
    # (the noise adds a random walk of a hundredth or two)
    assert heading.yaw_of(q) - 0.3 == pytest.approx(0.004 * 60, abs=0.03)
    assert imu.yaw_error == pytest.approx(heading.yaw_of(q) - 0.3, abs=1e-6)
    assert abs(np.linalg.norm(q) - 1) < 1e-12
    tilted = (math.cos(0.05), math.sin(0.05), 0.0, 0.0)  # rolled by 0.1 rad
    q2, _, _ = Imu(seed=3, bias=0.004).read(0.0, tilted, (0, 0, 0), (0, 0, 9.81))
    w, x, y, z = q2
    assert 1 - 2 * (x * x + y * y) == pytest.approx(math.cos(0.1), abs=1e-9)  # the vertical is where it was
    assert abs(gyro[2] - 0.004) < 0.012 and abs(acc[2] - 9.81) < 0.2  # the gyro reads the bias, with noise on it


def test_the_bias_is_drawn_per_start_from_the_documented_spread_and_a_seed_repeats_it():
    biases = [Imu(seed=s).bias[2] for s in range(300)]
    assert np.std(biases) == pytest.approx(BIAS_SIGMA, rel=0.2) and abs(np.mean(biases)) < 0.0005
    assert np.array_equal(Imu(seed=7).bias, Imu(seed=7).bias) and not np.array_equal(Imu(seed=7).bias, Imu(seed=8).bias)
    assert Imu(seed=1).truth()["gyro_bias"] == pytest.approx(list(Imu(seed=1).bias), abs=1e-6)
