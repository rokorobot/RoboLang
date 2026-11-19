"""What the stand-in does when the robot is told to stand still: the `Stance` interface and the `supported` stance.

Why this exists (decision 0046, U4 and open point 1): the whole-body policy walks well, but after walking it never
comes to rest on its own. At zero command it keeps stepping on (about 0.15 m/s) and every way of stopping it free
failed, on x86 and on aarch64. Unitree's own controller does this on the robot; the stand-in cannot. So the stand-in
holds the torso while the robot stands, and says so: every record carries `stance_model: "supported"`.

The interface is deliberately small. A stance is asked once per physics step for the 29 joint targets and may reach
the simulation (model, data, robot, policy) to do it. It owns the policy while it is in use. The stand-in adds the
`arm_sdk` blend and the hands on top of the targets, so a stance only decides about the body and the legs:

    stance = make_stance("supported", body)
    targets = stance.targets(command, policy_due)     # every physics step; (29,) joint position targets, SDK order
    stance.phase, stance.standing                      # for the oracle and for the harness

`command` is the current velocity command (vx, vy, wz) after the SetVelocity duration has been applied, so it is
exactly (0, 0, 0) when the robot is asked to stand. `policy_due` is True every fourth physics step (50 Hz).
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .policy import WholeBodyPolicy
from .scene import STAND_HEIGHT, Robot

STOP_S = 2.0  # the policy keeps running at zero command this long before the support takes the pelvis
RAISE_S = 1.0  # the support moves the pelvis to upright at standing height while the legs go limp
EASE_S = 3.0  # the legs ease to the standing pose (linear for the first two thirds), then are held there


@dataclass
class Body:
    """What a stance may touch: the simulation of the robot it stands for."""

    model: mujoco.MjModel
    data: mujoco.MjData
    robot: Robot
    policy: WholeBodyPolicy


class Stance:
    """Base class. `name` is recorded as `stance_model`; subclasses implement `targets`, `phase` and `standing`."""

    name = "abstract"

    def __init__(self, body: Body):
        self.body = body

    def targets(self, command, policy_due: bool) -> np.ndarray:
        raise NotImplementedError

    @property
    def phase(self) -> str:
        raise NotImplementedError

    @property
    def standing(self) -> bool:
        """True when the robot stands still and is ready for arm work."""
        raise NotImplementedError


def _yaw(quat) -> float:
    w, x, y, z = quat
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _upright(quat) -> np.ndarray:
    yaw = _yaw(quat)
    return np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])


class SupportedStance(Stance):
    """The torso is welded to a mocap body (a MuJoCo constraint) whenever the robot stands.

    Phases: `hold` (welded, legs at the standing pose) -> `walking` (free, the policy runs) -> `stopping` (still the
    policy, at zero command, for STOP_S) -> `raising` (welded to the mocap, which moves to upright; legs limp) ->
    `easing` (legs eased to the standing pose, then held) -> `hold`. A non-zero command releases the weld, clears the
    policy history and walks; during `stopping` it just goes on walking. A soft spring on the pelvis was tried first
    and blew up numerically at dt 0.005; the weld to a mocap body is stable.
    """

    name = "supported"

    def __init__(self, body: Body):
        super().__init__(body)
        m, d, r = body.model, body.data, body.robot
        self._weld, self._mocap = r.support_weld, r.support_mocap
        self._stand_q = body.policy.default.copy()
        self._target = self._stand_q.copy()
        self._phase_t0 = 0.0
        self._q0 = self._stand_q.copy()
        self._p0 = np.zeros(3)
        self._q_start = np.zeros(4)
        self._pending_policy = False
        # start standing, held: the pelvis is where the scene put it, at standing height
        mujoco.mj_forward(m, d)
        pos = d.xpos[r.pelvis].copy()
        self._engage(np.array([pos[0], pos[1], STAND_HEIGHT]), _upright(d.xquat[r.pelvis]))
        self._phase = "hold"

    # ----------------------------------------------------------------------------- the pelvis weld

    def _engage(self, pos, quat) -> None:
        d = self.body.data
        d.mocap_pos[self._mocap], d.mocap_quat[self._mocap] = pos, quat
        d.eq_active[self._weld] = 1

    def _release(self) -> None:
        self.body.data.eq_active[self._weld] = 0

    # ----------------------------------------------------------------------------- joint state

    def _joints(self):
        d, r = self.body.data, self.body.robot
        return d.qpos[r.qadr], d.qvel[r.dadr]

    def _policy_targets(self, command) -> np.ndarray:
        d, r = self.body.data, self.body.robot
        q, dq = self._joints()
        return self.body.policy.act(d.sensordata[r.imu_gyro:r.imu_gyro + 3], d.sensordata[r.imu_quat:r.imu_quat + 4],
                                    command, q, dq)

    def _walk_again(self) -> None:
        """Release the pelvis and hand the legs to the policy, starting from a clean history."""
        self._release()
        self.body.policy.reset()
        self._pending_policy = True  # run the policy on the very next step, not at the next 50 Hz tick
        self._phase = "walking"

    # ----------------------------------------------------------------------------- the interface

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def standing(self) -> bool:
        return self._phase == "hold"

    def targets(self, command, policy_due: bool) -> np.ndarray:
        d, r = self.body.data, self.body.robot
        t = d.time
        moving = any(abs(c) > 1e-9 for c in command)
        if moving and self._phase in ("hold", "raising", "easing"):
            self._walk_again()
        elif moving and self._phase == "stopping":
            self._phase = "walking"

        if self._phase == "hold":
            return self._stand_q
        if self._phase in ("walking", "stopping"):
            if policy_due or self._pending_policy:
                self._target = self._policy_targets(command)
                self._pending_policy = False
            if self._phase == "walking" and not moving:
                self._phase, self._phase_t0 = "stopping", t
            elif self._phase == "stopping" and t - self._phase_t0 >= STOP_S:
                self._phase, self._phase_t0 = "raising", t
                self._p0, self._q_start = d.xpos[r.pelvis].copy(), d.xquat[r.pelvis].copy()
                self._engage(self._p0, self._q_start)
            return self._target
        q, _ = self._joints()
        if self._phase == "raising":
            a = min(1.0, (t - self._phase_t0) / RAISE_S)
            d.mocap_pos[self._mocap] = (1 - a) * self._p0 + a * np.array([self._p0[0], self._p0[1], STAND_HEIGHT])
            q_up = _upright(self._q_start)
            blend = (1 - a) * self._q_start + a * (q_up if q_up @ self._q_start >= 0 else -q_up)
            d.mocap_quat[self._mocap] = blend / np.linalg.norm(blend)
            if t - self._phase_t0 >= RAISE_S:
                self._phase, self._phase_t0, self._q0 = "easing", t, q.copy()
            return q.copy()  # legs limp (target = where they are) while the torso is moved
        # easing
        a = min(1.0, (t - self._phase_t0) / (EASE_S * 2 / 3))
        target = (1 - a) * self._q0 + a * self._stand_q
        if t - self._phase_t0 >= EASE_S:
            self.body.policy.reset()
            self._phase = "hold"
        return target


STANCES = {"supported": SupportedStance}


def make_stance(name: str, body: Body) -> Stance:
    if name not in STANCES:
        raise ValueError(f"unknown stance {name!r}; available: {sorted(STANCES)}")
    return STANCES[name](body)
