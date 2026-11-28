"""The robot's heading between two station-marker fixes, from the IMU in `rt/lowstate` (phase 4 prerequisite).

`move` keeps a belief pose of the pelvis in the map. A station marker in view puts it right (position and heading).
Between fixes, the heading used to be dead-reckoned from the velocities that were *commanded*; under a steady heading
creep (emulated with `--creep`) the walk turns the robot three times faster than the stance does, the commands know
nothing of it, and the belief was 0.4 rad out before the next fix (S517, docs/evidence/g1-phase4-prereqs.md).

The change in heading since the last fix is now the change in the IMU's yaw, which is the robot's own sensor (the pelvis
IMU's quaternion in `rt/lowstate`; never the oracle, MuJoCo state or `SportModeState`):

    heading(t) = fix heading + wrap(IMU yaw(t) - IMU yaw at the fix) - bias x (t - t of the fix)

**Bias.** An IMU's yaw drifts: the gyro has a bias (and, integrated, noise), and with no magnetometer nothing pulls the
fused yaw back. The stand-in's IMU is not ideal (`sim/imu.py`: turn-on bias, bias random walk, white noise). The tracker
learns the bias from the fixes themselves: between two marker fixes the marker says how far the robot really turned, the
IMU says how far it thinks it turned, and the difference over the time between them is the bias. It is a least-squares
estimate over all pairs (weighted by their length, because a marker's heading is good to about 0.01 rad whatever the
time between two fixes), with a prior at zero (a pair shorter than `MIN_PAIR_S` says too little to learn from) and an
exponential forgetting (`FORGET_S`), so a bias that wanders is followed. It is clipped to `BIAS_CLIP`: a rate that large
is not a gyro bias, it is a bad fix. With no fix yet the bias is 0 and the heading is the IMU's change since the start.

**Held.** While the pelvis is welded (the supported stance the executor asks for during arm work, 0046 U4) it cannot
turn, so the belief heading stays where it is and the IMU's drift in the meantime is no news (an arm step of 25 s with a
bias of 0.002 rad/s would otherwise turn the belief by 0.05 rad with the robot standing still). A pair of fixes across a
held interval is the best the tracker gets: the marker says the robot did not turn, so all the IMU turned is bias.

Pure (no DDS, no ground truth): the executor's `ops.py` feeds it.
"""

from __future__ import annotations

import math

from .site import wrap

BIAS_CLIP = 0.01  # rad/s: a gyro bias larger than this is not believed (0.57 deg/s)
MIN_PAIR_S = 4.0  # fixes closer than this in time teach nothing about the bias (the reference waits for a later one)
PRIOR_S2 = 25.0  # the prior at zero bias, in s^2: sigma(marker heading pair)^2 / sigma(bias)^2 = (0.01 / 0.002)^2
FORGET_S = 300.0  # the weight of an old pair halves every 0.69 x this
MAX_JUMPS = 200  # jumps kept for the record


def yaw_of(quat) -> float:
    """The heading (rotation about the vertical) of an IMU quaternion (w, x, y, z), as a ZYX Euler angle."""
    w, x, y, z = quat
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class HeadingTracker:
    """The belief heading in the map: the last fix's heading plus the IMU's change in yaw since, less the bias."""

    def __init__(self, heading: float, imu_yaw: float, t_ms: int):
        # (map heading, IMU yaw, sim ms) at the last fix, or at the start
        self.anchor = (float(heading), float(imu_yaw), int(t_ms))
        self.marker = None  # the same, at the last fix by a station marker (the only one that teaches the bias)
        self.bias = 0.0  # rad/s, in the IMU's yaw
        self.fixes = 0
        self.held = False
        self.jumps: list[tuple[int, float, float]] = []  # (sim ms, rad the fix moved the belief, s since the last fix)
        self._sum_dt_r = 0.0
        self._sum_dt2 = 0.0

    def heading(self, imu_yaw: float, t_ms: int) -> float:
        """The belief heading at simulation time `t_ms`, when the IMU's yaw is `imu_yaw`."""
        h0, i0, t0 = self.anchor
        if self.held:
            return h0
        return wrap(h0 + wrap(imu_yaw - i0) - self.bias * (t_ms - t0) / 1000.0)

    def hold(self, held: bool, imu_yaw: float, t_ms: int) -> None:
        """The pelvis is welded (`held`) or free again: while held the heading is the one it had, IMU or no."""
        if held != self.held:
            self.anchor = (self.heading(imu_yaw, t_ms), float(imu_yaw), int(t_ms))
            self.held = held

    def reset(self, heading: float, imu_yaw: float, t_ms: int) -> None:
        """Take `heading` as true from now on, without learning anything from it (the start pose, or a set pose)."""
        self.anchor = (float(heading), float(imu_yaw), int(t_ms))
        self.marker = None

    def fix(self, heading: float, imu_yaw: float, t_ms: int) -> float:
        """A station marker gave the heading `heading` at simulation time `t_ms`, when the IMU's yaw was `imu_yaw`.
        Returns how far the fix moved the belief (marker minus belief, rad): the IMU's error since the last fix."""
        jump = wrap(heading - self.heading(imu_yaw, t_ms))
        since = (t_ms - self.anchor[2]) / 1000.0
        fix = (float(heading), float(imu_yaw), int(t_ms))
        if self.marker is None:
            self.marker = fix
        else:
            h1, i1, t1 = self.marker
            dt = (t_ms - t1) / 1000.0
            if dt >= MIN_PAIR_S:  # (a closer fix leaves the reference where it is: the pair grows until it can teach)
                resid = wrap(imu_yaw - i1) - wrap(heading - h1)  # what the IMU turned, less what the marker saw
                if abs(resid) / dt <= 2 * BIAS_CLIP:  # a larger rate is a bad fix, not a gyro
                    keep = math.exp(-dt / FORGET_S)
                    self._sum_dt_r = keep * self._sum_dt_r + dt * resid
                    self._sum_dt2 = keep * self._sum_dt2 + dt * dt
                    self.bias = max(-BIAS_CLIP, min(BIAS_CLIP, self._sum_dt_r / (PRIOR_S2 + self._sum_dt2)))
                self.marker = fix
        self.anchor = fix
        self.fixes += 1
        self.jumps.append((int(t_ms), round(jump, 4), round(since, 1)))
        del self.jumps[:-MAX_JUMPS]
        return jump

    def summary(self) -> dict:
        """For the run record: what the IMU got wrong between fixes, as the fixes showed it."""
        moved = [abs(j) for _, j, _ in self.jumps]
        return {
            "source": "imu_yaw",
            "fixes": self.fixes,
            "bias_rad_s": round(self.bias, 5),
            "jump_rad_max": round(max(moved), 4) if moved else None,
            "jump_rad_mean": round(sum(moved) / len(moved), 4) if moved else None,
        }
