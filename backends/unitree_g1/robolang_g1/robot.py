"""The robot as a program sees it: Unitree's SDK clients and topics, and the head camera's frames.

Shared by the harness (`harness.py`) and the executor (`ops.py`). Everything the backend does to or learns from the G1
goes through here, and every item is an interface the real robot has: `LocoClient` (RPC service `sport`),
`rt/arm_sdk`, `rt/dex3/*/cmd` and `/state`, `rt/lowstate`, and the head camera. There is no other way in: nothing here
reads MuJoCo state, the oracle file or `SportModeState`.

Time is simulation time, taken from `LowState_.tick` (ms). The SDK's timeouts are wall-clock, as on the robot; the
stand-in paces the simulation to the wall clock (0046 U13), so waiting a wall-clock second here is one second of both.
"""

from __future__ import annotations

import collections
import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HISTORY_S = 12.0  # how much lowstate history is kept (frames are matched to joints measured at their time)
# arm_sdk gains: waist, shoulders and elbows, wrists. The shoulder holds a forward reach against about 7 N m of
# gravity, so tracking within 0.05 rad needs kp above about 150 (Unitree's example kp 60 would give 0.12 rad). kd stays
# well below 11: the arm joints are stable only for dt x kd / I < 2 at the stand-in's 2 ms step (see harness evidence).
ARM_GAINS = {"waist": (500.0, 10.0), "arm": (200.0, 5.0), "wrist": (60.0, 2.0)}
HAND_GAINS = (1.5, 0.2)  # Dex3 kp, kd as in Unitree's hand examples


def arm_gains(policy_kp, policy_kd) -> tuple[np.ndarray, np.ndarray]:
    """The gains given to arm_sdk for joints 12-28 (the rest keep the policy's)."""
    kp, kd = np.array(policy_kp, float), np.array(policy_kd, float)
    for j in range(12, 15):
        kp[j], kd[j] = ARM_GAINS["waist"]
    for j in range(15, 29):
        kp[j], kd[j] = ARM_GAINS["wrist" if (j - 15) % 7 >= 4 else "arm"]
    return kp, kd


@dataclass(frozen=True)
class LowSample:
    """One `rt/lowstate` message: the 29 body joints, the pelvis IMU, and the simulation time in ms."""

    tick: int
    q: np.ndarray
    dq: np.ndarray
    quat: tuple  # (w, x, y, z)
    gyro: tuple
    motor_error: int = 0  # the 29 body motors' `motorstate` words ORed together: 0 when no motor reports an error


class Robot:
    """The SDK's own clients and topics. Create it after the guard has run (the channel factory must exist)."""

    def __init__(self):
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_, unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, LowCmd_, LowState_

        self.loco = LocoClient()
        self.loco.SetTimeout(3.0)
        self.loco.Init()
        self.low_q, self.low_tick, self.low_count, self.low_wall = np.zeros(29), 0, 0, 0.0
        self.last: LowSample | None = None
        self.history: collections.deque[LowSample] = collections.deque(maxlen=int(HISTORY_S * 250))
        self.hand_q = {"left": np.zeros(7), "right": np.zeros(7)}
        self.hand_dq = {"left": np.zeros(7), "right": np.zeros(7)}
        self.hand_wall = {"left": 0.0, "right": 0.0}  # when each hand's state last arrived (wall clock)
        self.hand_tick = {"left": 0, "right": 0}  # simulation time (lowstate tick) when it did
        self.commands_sent = {"set_velocity": 0, "arm_sdk": 0, "hand": 0}
        self.silenced = False  # after a fall: nothing more is sent, whatever the caller asks
        sub = ChannelSubscriber("rt/lowstate", LowState_)
        sub.Init(self._on_low, 10)
        self._subs = [sub]
        for side in ("left", "right"):
            hs = ChannelSubscriber(f"rt/dex3/{side}/state", HandState_)
            hs.Init(lambda m, side=side: self._on_hand(side, m), 10)
            self._subs.append(hs)
        self.arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_pub.Init()
        self.hand_pub = {s: ChannelPublisher(f"rt/dex3/{s}/cmd", HandCmd_) for s in ("left", "right")}
        for pub in self.hand_pub.values():
            pub.Init()
        self._arm_msg, self._hand_msg = unitree_hg_msg_dds__LowCmd_(), unitree_hg_msg_dds__HandCmd_()
        self.arm_target = None  # (q, kp, kd) the arm thread publishes; None until set
        self.arm_weight = 0.0
        self._arm_thread = None
        self._arm_stop = threading.Event()
        self._lock = threading.Lock()

    # ------------------------------------------------------------------------------------------ state

    def _on_low(self, msg):
        q = np.array([m.q for m in msg.motor_state[:29]])
        dq = np.array([m.dq for m in msg.motor_state[:29]])
        imu = msg.imu_state
        errors = 0
        for m in msg.motor_state[:29]:
            errors |= int(m.motorstate)
        sample = LowSample(int(msg.tick), q, dq, tuple(imu.quaternion), tuple(imu.gyroscope), errors)
        self.low_wall = time.monotonic()
        self.low_q, self.low_tick, self.low_count, self.last = q, sample.tick, self.low_count + 1, sample
        self.history.append(sample)

    def _on_hand(self, side, msg):
        self.hand_q[side] = np.array([x.q for x in msg.motor_state[:7]])
        self.hand_dq[side] = np.array([x.dq for x in msg.motor_state[:7]])
        self.hand_wall[side], self.hand_tick[side] = time.monotonic(), self.low_tick

    @property
    def now_ms(self) -> int:
        """Simulation time in ms (`lowstate.tick`)."""
        return self.low_tick

    def samples_since(self, tick_ms: int) -> list[LowSample]:
        return [s for s in list(self.history) if s.tick >= tick_ms]

    def sample_at(self, tick_ms: float) -> LowSample | None:
        """The lowstate sample closest in time to `tick_ms` (a camera frame's stamp), or None if there is none."""
        hist = list(self.history)
        return min(hist, key=lambda s: abs(s.tick - tick_ms)) if hist else None

    def wait_state(self, timeout: float = 10.0) -> None:
        """Wait until lowstate and both hand states have arrived."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.last is not None and self.low_count > 5:
                return
            time.sleep(0.02)
        raise TimeoutError("no rt/lowstate")

    def fsm_id(self) -> tuple[int, int | None]:
        """`LocoClient.GetFsmId` (the SDK's own call, a read): (code, the walking service's state id or None)."""
        return self.loco.GetFsmId()

    # ------------------------------------------------------------------------------------------ walking

    def set_velocity(self, vx: float, vy: float, wz: float, duration: float) -> int:
        """`LocoClient.SetVelocity` (the SDK's own call). Returns its code; nothing is sent once silenced."""
        if self.silenced:
            return -1
        self.commands_sent["set_velocity"] += 1
        return self.loco.SetVelocity(float(vx), float(vy), float(wz), float(duration))

    # ------------------------------------------------------------------------------------------ arm_sdk

    def start_arm_sdk(self):
        """Publish `rt/arm_sdk` at 50 Hz with the current target and weight (does nothing until `set_arm`)."""
        if self._arm_thread is not None and self._arm_thread.is_alive():
            return
        self._arm_stop.clear()
        self._arm_thread = threading.Thread(target=self._arm_loop, daemon=True)
        self._arm_thread.start()

    def stop_arm_sdk(self):
        self._arm_stop.set()
        if self._arm_thread:
            self._arm_thread.join(timeout=2)

    def _arm_loop(self):
        msg = self._arm_msg
        while not self._arm_stop.is_set():
            with self._lock:
                target, weight = self.arm_target, self.arm_weight
            if target is not None and not self.silenced:
                q, kp, kd = target
                for i in range(29):
                    c = msg.motor_cmd[i]
                    c.q, c.dq, c.tau, c.kp, c.kd = float(q[i]), 0.0, 0.0, float(kp[i]), float(kd[i])
                msg.motor_cmd[29].q = float(weight)  # kNotUsedJoint: the arm_sdk weight
                self.arm_pub.Write(msg)
                self.commands_sent["arm_sdk"] += 1
            time.sleep(0.02)

    def set_arm(self, q, kp, kd, weight):
        with self._lock:
            self.arm_target, self.arm_weight = (np.array(q, float), np.array(kp, float), np.array(kd, float)), weight

    def ramp_arm(self, q_to, kp, kd, w_to, seconds, q_from=None, w_from=None, check=None):
        """Move the arm_sdk targets and weight linearly over `seconds` (`check()` may raise to abort)."""
        q0 = self.low_q.copy() if q_from is None else np.array(q_from, float)
        w0 = self.arm_weight if w_from is None else w_from
        steps = max(1, int(seconds / 0.02))
        for i in range(1, steps + 1):
            if check is not None:
                check()
            a = i / steps
            self.set_arm((1 - a) * q0 + a * np.array(q_to), kp, kd, (1 - a) * w0 + a * w_to)
            time.sleep(0.02)

    def hold_arm(self) -> bool:
        """Hold the arms (waist included) where they are now: the target becomes the measured joints, the gains and
        the weight stay. Returns False when arm_sdk was never used (the walking controller's own pose is the hold)."""
        with self._lock:
            if self.arm_target is None:
                return False
            _, kp, kd = self.arm_target
            self.arm_target = (self.low_q.copy(), kp, kd)
        return True

    # ------------------------------------------------------------------------------------------ hands

    def hand_command(self, side, q, kp=HAND_GAINS[0], kd=HAND_GAINS[1], seconds=2.0, check=None):
        """Publish a Dex3 command for `seconds`. The robot keeps the last command when publishing stops."""
        msg = self._hand_msg
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            if check is not None:
                check()
            if self.silenced:
                return
            for i in range(7):
                c = msg.motor_cmd[i]
                c.q, c.dq, c.tau, c.kp, c.kd = float(q[i]), 0.0, 0.0, kp, kd
            self.hand_pub[side].Write(msg)
            self.commands_sent["hand"] += 1
            time.sleep(0.02)

    # ------------------------------------------------------------------------------------------ after a fall

    def silence(self) -> None:
        """Send nothing more, ever (S516: a fallen robot is not commanded). Stops the arm_sdk publisher."""
        self.silenced = True
        self.stop_arm_sdk()


# ------------------------------------------------------------------------------------------------ head camera


def read_frame(head: Path, after_frame: int = 0, timeout: float = 5.0):
    """A consistent (BGR image, meta): the JSON, the PNG, and the JSON again must name the same frame.

    `head` is the directory the stand-in writes (`latest.png`, `latest.json` with `frame` and `sim_time`); on the
    robot the camera is read directly. `meta["sim_time"]` is the frame's simulation time in seconds.
    """
    import cv2

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            a = json.loads((head / "latest.json").read_text())
            image = cv2.imread(str(head / "latest.png"))
            b = json.loads((head / "latest.json").read_text())
        except (OSError, ValueError):
            time.sleep(0.02)
            continue
        if a["frame"] == b["frame"] and a["frame"] > after_frame and image is not None:
            return image, a
        time.sleep(0.02)
    raise TimeoutError("no new camera frame")
