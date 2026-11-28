"""The stand-in's IMU: what `rt/lowstate` carries is not MuJoCo's ideal sensor (phase 4 prerequisite).

MuJoCo's `framequat`, `gyro` and `accelerometer` sensors are exact. A real IMU is not, and a heading taken from one
(`heading.py`) would look better against a perfect one than the robot allows. This model spoils the three signals the
way a MEMS IMU with an on-board attitude filter does:

* **Gyroscope**: a bias per axis (the part left after the turn-on calibration: normal, sigma `BIAS_SIGMA`, drawn at
  start-up), a bias that wanders (random walk, `BIAS_WALK` per root second) and white noise (`GYRO_NOISE` rms per
  sample, about 1.8e-4 rad/s per root hertz at 250 Hz: a good MEMS part, not a poor one).
* **Quaternion**: the fused attitude. Roll and pitch are held by gravity, so they stay right (what the executor uses the
  vertical for is unchanged). **Yaw is not**: there is no magnetometer, so the filter integrates the gyro, bias and
  noise included, and the yaw drifts at the rate of the gyro's error about the vertical. This is the number the
  executor uses for the change in heading between marker fixes, and what it must cope with: at `BIAS_SIGMA` one sigma
  the yaw is out by 0.12 rad after a minute, and by 0.5 rad after four with nothing to correct it.
* **Accelerometer**: white noise (`ACC_NOISE` rms per axis).

The state is advanced at the `lowstate` rate on simulation time. `seed` fixes the draw (the stand-in prints it); `bias`
sets the gyro's z bias exactly (rad/s), which the stress check uses to test a worst case. The true yaw error and bias
are in the oracle, for the checks; nothing else reads them. Documented in spec/g1-backend-v0.1.md, "The IMU".
"""

from __future__ import annotations

import numpy as np

BIAS_SIGMA = 0.002  # rad/s per axis, 1 sigma: 0.11 deg/s, what a calibrated consumer MEMS gyro keeps after turn-on
BIAS_WALK = 2e-5  # rad/s per sqrt(s): the bias moves 0.0003 rad/s in 200 s
GYRO_NOISE = 0.002  # rad/s rms per sample per axis
ACC_NOISE = 0.02  # m/s^2 rms per sample per axis


class Imu:
    def __init__(self, seed: int | None = None, bias: float | None = None, ideal: bool = False):
        self.seed = int(np.random.SeedSequence().entropy % 2**31) if seed is None else int(seed)
        self.ideal = ideal
        self._rng = np.random.default_rng(self.seed)
        self.bias = np.zeros(3) if ideal else self._rng.normal(0.0, BIAS_SIGMA, 3)
        if bias is not None:
            self.bias[2] = bias
        self.yaw_error = 0.0  # rad: the published yaw minus the true yaw
        self._t: float | None = None

    def read(self, sim_time: float, quat, gyro, acc):
        """The sensors as published at `sim_time`: (quaternion (w, x, y, z), gyroscope, accelerometer)."""
        if self.ideal:
            return quat, gyro, acc
        dt = 0.0 if self._t is None else max(sim_time - self._t, 0.0)
        self._t = sim_time
        self.bias += self._rng.normal(0.0, BIAS_WALK * np.sqrt(dt), 3) if dt else 0.0
        error = self.bias + self._rng.normal(0.0, GYRO_NOISE, 3)  # the gyro's error, body frame
        w, x, y, z = quat
        up = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])  # the vertical, body frame
        self.yaw_error += dt * float(up @ error)  # the filter integrates the gyro: its error about the vertical is yaw
        half = 0.5 * self.yaw_error  # published = (rotation about the world vertical by yaw_error) x true
        c, s = np.cos(half), np.sin(half)
        out = (c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w)
        return out, np.asarray(gyro) + error, np.asarray(acc) + self._rng.normal(0.0, ACC_NOISE, 3)

    def truth(self) -> dict:
        """For the oracle: the bias and the yaw error as they are (the executor cannot see them)."""
        return {"seed": self.seed, "gyro_bias": [round(float(b), 6) for b in self.bias],
                "yaw_error_rad": round(float(self.yaw_error), 5), "ideal": self.ideal}
