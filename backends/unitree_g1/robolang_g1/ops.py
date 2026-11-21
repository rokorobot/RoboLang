"""What the executor does to the robot: walk to a station, reach, grasp, place, wait, stop. Only through `robot.py`.

`G1Operations` is the executor's view of the robot. It owns the **belief pose** of the robot in the map (dead reckoning
from the velocities it commands, corrected whenever a station marker is seen), the head-camera perception, and the
robot's own kinematic model. It never reads MuJoCo state, the oracle or `SportModeState`.

How the stand-in walks, as measured (docs/evidence/g1-phase2-executor.md), shapes `move`: the whole-body policy ignores
a command whose size is below about 0.3 (a pure yaw of 0.2 rad/s turns the robot by almost nothing), forward walking
comes in steps of 0.06-0.15 m, and it turns in place only by *shuffling*: alternating sideways commands of +-0.3 m/s,
each with a yaw rate of 0.2 rad/s, which turns it about 0.12 rad per 1.2 s burst and leaves its position nearly where it
was. So a move is: turn to face the target (shuffle), walk in legs and look between them, turn to the station's
heading (shuffle), then correct the last few centimetres with short bursts, looking at the station marker each time. The
executor does not depend on any of these numbers being exact: it commands, looks and corrects.
"""
from __future__ import annotations

import math
import time

import numpy as np

from . import assets, guard
from . import site as site_mod
from .executor import STOP_CODES, STOP_MESSAGES, BackendFailure, Stop
from .kinematics import ARM_JOINTS, WAIST_PITCH, Kinematics
from .perception import Perception, object_centre, pelvis_pose_in_map
from .robot import Robot, arm_gains

# ---- safety and stop
TILT_LIMIT_RAD = math.radians(40)  # S516: the IMU's tilt from upright beyond this means the robot is falling or fallen
CONFIRM_TIMEOUT_S = 3.0  # a stop must be confirmed (upright and still, from lowstate) within this, in simulation time
CONFIRM_WINDOW_MS = 500
CONFIRM_GYRO = 0.06  # rad/s: the pelvis rotates no faster than this ...
CONFIRM_JOINT_VEL = 0.12  # rad/s: ... and no joint moves faster than this (the held arms included)
CONFIRM_TILT_RAD = math.radians(15)
SETTLE_GYRO, SETTLE_LEG_VEL, SETTLE_WINDOW_MS, SETTLE_TIMEOUT_S = 0.03, 0.05, 600, 9.0
# ---- localization and walking
STATION_TOL_M, STATION_TOL_RAD = 0.047, 0.09  # `move` promises 5 cm and 0.1 rad (0046 U5): this, after settling, leaves margin
SERVO_TOL_M, SERVO_TOL_RAD = 0.044, 0.085
MARKER_MAX_RANGE_M, MARKER_MIN_CONF = 2.9, 0.5  # a station marker corrects the belief only when seen this well
MAX_UP_DISAGREEMENT_RAD = 0.03  # ... and only if the vertical it implies is the IMU's, to within this
APPROACH_M = 0.5  # the walk ends this far before the station, facing it, and the marker is looked for there
WALK_SPEED = 0.5
WALK_GAIN = 0.95  # metres of forward travel (coast included) per metre commanded (measured 0.74-1.08)
SHUFFLE_S = 1.2  # one burst of the turn-in-place shuffle ...
SHUFFLE_RAD = {1: 0.127, -1: 0.111}  # ... turns this much to the left (+) and to the right (-): measured
LEG_MAX_M = 1.6  # the longest walk between two looks
SCAN_STEP_RAD = 0.6  # a scan for a marker turns this much between looks
MOVE_TIMEOUT_S, SERVO_TIMEOUT_S, MARKER_TIMEOUT_S = 240.0, 90.0, 70.0  # simulation time: S504, S504, S517
# ---- arms
ARM_TRACK_TOL_RAD = 0.12  # S503: a joint further than this from its target when a motion has finished
HAND_TOL_RAD = 0.25  # S505
HAND_CLOSE = {"right": [0.0, -0.6, -1.0, 1.0, 1.2, 1.0, 1.2], "left": [0.0, 0.6, 1.0, -1.0, -1.2, -1.0, -1.2]}
SEGMENT_S = 1.6  # the shortest arm motion between two poses (min-jerk, and always at least 1 s)
HELD_RADIUS_M = 0.06  # the object's marker is "at the hand" within this of the hand's grasp point
APPROACH_BACK, APPROACH_UP = 0.09, 0.03  # the pre-grasp: behind the grasp point along the fingers, and above it
GRASP_RISE = -0.01  # the pinch point is this far above the object's centre: below it, the fingers stay under the top face
FINGERS_DOWN_DEG = (15, 30, 0, 45)  # hand orientations tried, in this order
WAIST_LEAN_RAD = (0.0, 0.15, 0.3)  # waist pitch allowed, tried in this order
HAND_REST_Y = {"right": -0.10, "left": 0.10}


def _frame(theta: float) -> np.ndarray:
    """The grasp frame with the fingers forward and `theta` rad below the horizontal, the thumb toward the midline."""
    xh = np.array([math.cos(theta), 0, -math.sin(theta)])
    yh = np.array([0, 1.0, 0])
    return np.column_stack([xh, yh, np.cross(xh, yh)])


# What a burst of velocity command does to a robot that stands still, as measured (docs/evidence/g1-phase2-executor.md):
# it takes a step when the command lasts long enough, so displacements come in steps. (command seconds, metres moved).
FORWARD_BURSTS = [(0.3, 0.10), (0.4, 0.15)]  # vx 0.4; shorter than 0.3 s does nothing
BACK_BURSTS = [(0.2, 0.07), (0.3, 0.10), (0.4, 0.147), (0.8, 0.235)]  # vx -0.4
LEFT_BURSTS = [(0.8, 0.05), (1.2, 0.17)]  # (vx 0.1, vy 0.3): less than 0.8 s does not step to the left
RIGHT_BURSTS = [(0.5, 0.05), (0.8, 0.09), (1.2, 0.15)]  # (vx 0.1, vy -0.3)


def _burst(axis: str, error: float) -> tuple[float, float, float]:
    """The (vx, vy, seconds) of the burst that should cancel `error` (metres along the axis: forward, or left), chosen
    from the measured steps. Long errors are walked: about 0.3 m per second."""
    table = {"forward": FORWARD_BURSTS if error > 0 else BACK_BURSTS,
             "lateral": LEFT_BURSTS if error > 0 else RIGHT_BURSTS}[axis]
    want = abs(error)
    if want > table[-1][1] * 1.3:
        seconds = min(want / 0.30, 4.0)
    else:
        seconds = min(table, key=lambda entry: abs(entry[1] - want))[0]
    sign = 1.0 if error > 0 else -1.0
    return (0.4 * sign, 0.0, seconds) if axis == "forward" else (0.1, 0.3 * sign, seconds)


def _minjerk(a: float) -> float:
    return a**3 * (10 - 15 * a + 6 * a * a)


class G1Operations:
    """The operations of the executor. `guard()` must run first: it creates the DDS participant and the clients."""

    def __init__(self, site: dict, stop: Stop | None = None, head_dir: str | None = None):
        self.site, self.stop, self.head_dir = site, stop, head_dir
        self.stations = {s["name"]: s for s in site["stations"]}
        self.by_marker = {s["marker"]["id"]: s for s in site["stations"]}
        self.pose = list(site["robot"]["start"])  # the belief: pelvis (x, y, yaw) in the map
        self.station: str | None = None  # the station the robot last arrived at
        self.held: dict | None = None  # {"object", "hand"} while an object is welded to a hand
        self.events: list[dict] = []  # what the current step did, for its record
        self.observations: list[dict] = []  # marker readings taken, for the record
        self.identity: dict | None = None
        self.sim = None
        self.robot: Robot | None = None
        self.kin: Kinematics | None = None
        self.perc: Perception | None = None
        self.q_carry: np.ndarray | None = None
        self.fallen = False
        self._last_score = 0.0

    # ------------------------------------------------------------------------------------------ set-up

    def guard(self, model: str | None = None) -> None:
        """The simulation guard (S501), then the clients. Nothing is sent to the robot before it passes."""
        try:
            self.sim, self.identity = guard.require_simulation()
        except guard.GuardError as exc:
            raise BackendFailure(guard.CODE, exc.reason) from exc
        head = self.head_dir or self.identity.get("head_camera_dir")
        if not head:
            raise BackendFailure("S501", "the robot reports no head camera frame source and none was given")
        self.robot = Robot()
        self.robot.wait_state()
        self.kin = Kinematics(self.site["camera"])
        self.perc = Perception(self.robot, self.kin, self.site, head)
        self.q_carry = self.robot.low_q.copy()  # the arms as the walking controller holds them: the carry pose
        self.q_carry[WAIST_PITCH - 2:WAIST_PITCH + 1] = 0.0
        self.arm_kp, self.arm_kd = arm_gains(np.zeros(29), np.zeros(29))

    def now_ms(self) -> int:
        return self.robot.now_ms

    def versions(self) -> dict:
        return assets.versions()

    def status(self) -> dict | None:
        code, status = self.sim.Status()
        return status if code == 0 else None

    # ------------------------------------------------------------------------------------------ checks

    def tilt_rad(self) -> float:
        w, x, y, z = self.robot.last.quat
        return math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))

    def check(self) -> None:
        """Called in every wait and every motion loop: a fall (S516), then a stop request or the deadline."""
        if self.fallen or self.tilt_rad() > TILT_LIMIT_RAD:
            if not self.fallen:
                self.fallen = True
                self.robot.silence()  # from here on nothing is sent, whatever is asked (S516)
            raise BackendFailure("S516", f"the robot's IMU tilt reached {math.degrees(self.tilt_rad()):.0f} degrees "
                                         f"(limit {math.degrees(TILT_LIMIT_RAD):.0f}): it fell or is falling")
        if self.stop is not None:
            reason = self.stop.due(self.now_ms())
            if reason is not None:
                raise BackendFailure(STOP_CODES[reason], STOP_MESSAGES[reason])

    def sleep(self, seconds: float) -> None:
        """Wait `seconds` of simulation time, checking the stop and the robot's tilt every 20 ms."""
        end = self.now_ms() + int(seconds * 1000)
        while self.now_ms() < end:
            self.check()
            time.sleep(0.02)

    def wait_ms(self, duration: int) -> None:
        self.sleep(duration / 1000)

    def settled(self, window_ms: int = SETTLE_WINDOW_MS) -> bool:
        s = self.robot.samples_since(self.now_ms() - window_ms)
        if len(s) < window_ms / 4 * 0.6:
            return False
        gyro = max(np.linalg.norm(x.gyro) for x in s)
        legs = max(float(np.max(np.abs(x.dq[:12]))) for x in s)
        return gyro <= SETTLE_GYRO and legs <= SETTLE_LEG_VEL

    def settle(self, timeout_s: float = SETTLE_TIMEOUT_S) -> bool:
        """Wait until the stance is quiet: the pelvis IMU's rate and the leg joint velocities, from lowstate only."""
        end = self.now_ms() + int(timeout_s * 1000)
        self.sleep(0.5)
        while self.now_ms() < end:
            self.check()
            if self.settled():
                return True
            time.sleep(0.02)
        return False

    # ------------------------------------------------------------------------------------------ looking

    def look(self, frames: int = 4, ids=None) -> dict:
        """Take frames, correct the belief from the best station marker seen, and return the readings."""
        readings = self.perc.observe(ids=ids, frames=frames, check=self.check)
        self.observations.append({"t_ms": self.now_ms(), "frames": frames,
                                  "markers": {str(k): v.as_dict(self.now_ms()) for k, v in sorted(readings.items())}})
        best = None
        for mid, reading in readings.items():
            if mid in self.by_marker and reading.range_m <= MARKER_MAX_RANGE_M and reading.confidence >= MARKER_MIN_CONF:
                if best is None or reading.range_m < best[1].range_m:
                    best = (mid, reading)
        if best is not None:
            mid, reading = best
            *new, up = pelvis_pose_in_map(self.by_marker[mid]["marker"], reading)
            w, x, y, z = self.robot.last.quat  # the IMU's vertical, in the pelvis frame
            imu_up = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])
            disagree = float(np.arccos(np.clip(up @ imu_up, -1, 1)))
            if disagree > MAX_UP_DISAGREEMENT_RAD:  # the wrong solution of the marker's pose: ignore this reading
                self.events.append({"t_ms": self.now_ms(), "event": "rejected", "marker": mid,
                                    "vertical_vs_imu_rad": round(disagree, 3)})
            else:
                self.events.append({"t_ms": self.now_ms(), "event": "localized", "marker": mid,
                                    "from": [round(float(v), 3) for v in self.pose], "to": [round(v, 3) for v in new]})
                self.pose = list(new)
        return readings

    # ------------------------------------------------------------------------------------------ walking

    def _dead_reckon(self, vx: float, vy: float, wz: float, seconds: float) -> None:
        x, y, yaw = self.pose
        forward = WALK_GAIN * vx * seconds
        left = 0.65 * vy * seconds
        self.pose = [x + forward * math.cos(yaw) - left * math.sin(yaw), y + forward * math.sin(yaw) + left * math.cos(yaw),
                     site_mod.wrap(yaw + (SHUFFLE_RAD[1 if wz > 0 else -1] * np.sign(wz) * seconds / SHUFFLE_S if wz else 0.0))]

    def command(self, vx: float, vy: float, wz: float, seconds: float, wait: bool = True) -> None:
        """One SetVelocity, the belief advanced by what it should do, and (unless `wait` is False) the wait for it."""
        self.check()
        code = self.robot.set_velocity(vx, vy, wz, seconds)
        if code != 0:
            raise BackendFailure("S503", f"SetVelocity({vx:.2f}, {vy:.2f}, {wz:.2f}, {seconds:.2f}) returned {code}")
        self._dead_reckon(vx, vy, wz, seconds)
        if wait:
            self.sleep(seconds)

    def turn_by(self, angle: float) -> None:
        """Turn in place by `angle` rad (+ left) from the belief heading. See `turn_to`."""
        self.turn_to(site_mod.wrap(self.pose[2] + angle))

    def turn_to(self, yaw: float, tolerance: float = 0.06) -> None:
        """Turn in place to the map heading `yaw`: the shuffle (bursts alternating sideways with the yaw rate held), in
        chunks of six bursts with a look after each, so a station marker in view corrects the heading on the way."""
        for _ in range(12):
            remaining = site_mod.wrap(yaw - self.pose[2])
            if abs(remaining) <= tolerance:
                return
            sign = 1 if remaining > 0 else -1
            bursts = min(6, max(1, round(abs(remaining) / SHUFFLE_RAD[sign])))
            for k in range(bursts):
                self.command(0.0, 0.3 * (1 if k % 2 == 0 else -1), 0.2 * sign, SHUFFLE_S)
            self.settle(4.0)
            self.look(frames=2)

    def _angle_to(self, x: float, y: float) -> float:
        return site_mod.wrap(math.atan2(y - self.pose[1], x - self.pose[0]) - self.pose[2])

    def walk_to(self, x: float, y: float, stop_short: float = 0.25, deadline_ms: int | None = None) -> None:
        """Face (x, y), then walk to within `stop_short` of it in legs, looking between them."""
        while True:
            self.check()
            if deadline_ms is not None and self.now_ms() > deadline_ms:
                raise BackendFailure("S504", "the walk did not reach the approach point in time")
            dist = math.hypot(x - self.pose[0], y - self.pose[1])
            if dist <= stop_short:
                return
            bearing = self._angle_to(x, y)
            if abs(bearing) > 0.2:
                self.turn_by(bearing)
                continue  # look, then check the bearing again with the belief the look left
            self.look(frames=2)
            leg = min(dist - stop_short, LEG_MAX_M)
            seconds = max(0.5, leg / (WALK_GAIN * WALK_SPEED))
            self.command(WALK_SPEED, 0.0, 0.0, seconds)
            self.settle()
            self.look()

    def move(self, station_name: str) -> dict:
        """Walk to the station, then servo on its marker to within 5 cm and 0.1 rad, then wait for the stance to settle."""
        station = self.stations[station_name]
        gx, gy, gyaw = station["pose"]
        self.events = []
        t0 = self.now_ms()
        deadline = t0 + int(MOVE_TIMEOUT_S * 1000)
        self.look(frames=3)
        ax, ay = gx - APPROACH_M * math.cos(gyaw), gy - APPROACH_M * math.sin(gyaw)
        if math.hypot(ax - self.pose[0], ay - self.pose[1]) > 0.4:
            self.walk_to(ax, ay, deadline_ms=deadline)
        self.turn_to(gyaw, tolerance=0.1)  # face the station: its marker is then in the camera's view
        marker = self._find_marker(station, deadline)
        result = self._servo(station, marker, deadline)
        self.station = station_name
        result["events"] = self.events
        return result

    def _find_marker(self, station: dict, deadline_ms: int):
        """Look at the approach point for the station's marker; if it is not there, scan by turning. S517 if not seen."""
        mid = station["marker"]["id"]
        scan_end = self.now_ms() + int(MARKER_TIMEOUT_S * 1000)
        mx, my = station["marker"]["pos"][:2]
        first = True
        while True:
            readings = self.look(frames=3 if first else 4)
            reading = readings.get(mid)
            if reading is not None and reading.confidence >= MARKER_MIN_CONF and reading.range_m <= MARKER_MAX_RANGE_M:
                return reading
            if self.now_ms() > scan_end or self.now_ms() > deadline_ms:
                raise BackendFailure("S517", f"station marker {mid} was not seen within {MARKER_TIMEOUT_S:.0f} s "
                                             f"(scanned from the approach point; belief {[round(float(v), 2) for v in self.pose]})")
            # not seen: face where the belief says it is, then sweep round in steps (a station marker seen on the way
            # corrects the belief, and the next step aims again)
            aim = self._angle_to(mx, my) if first else SCAN_STEP_RAD
            first = False
            self.events.append({"t_ms": self.now_ms(), "event": "scan", "turn_rad": round(float(aim), 3)})
            if abs(aim) > 0.06:
                self.turn_by(aim)

    def _servo(self, station: dict, reading, deadline_ms: int) -> dict:
        """Correct the last centimetres: look at the station marker, command a short burst along the axis that is furthest
        out (yaw by the shuffle, forward or back, sideways), settle, look again. The policy moves in steps (0.06-0.15 m
        forward, 0.05 m sideways, 0.12 rad of yaw), so a burst is chosen from the error and what a burst of that length
        has been measured to do; a correction that did not take is repeated a little longer, once."""
        gx, gy, gyaw = station["pose"]
        mid = station["marker"]["id"]
        end = min(deadline_ms, self.now_ms() + int(SERVO_TIMEOUT_S * 1000))
        iterations = []
        while True:
            self.check()
            dx, dy = gx - self.pose[0], gy - self.pose[1]
            ex = dx * math.cos(self.pose[2]) + dy * math.sin(self.pose[2])
            ey = -dx * math.sin(self.pose[2]) + dy * math.cos(self.pose[2])
            eyaw = site_mod.wrap(gyaw - self.pose[2])
            iterations.append([round(float(ex), 3), round(float(ey), 3), round(float(eyaw), 3)])
            if math.hypot(ex, ey) <= SERVO_TOL_M and abs(eyaw) <= SERVO_TOL_RAD:
                if self.settle() and self._confirm_pose(station):
                    return {"iterations": iterations, "error_m": round(math.hypot(ex, ey), 4),
                            "error_rad": round(abs(eyaw), 4), "settled": True}
                continue
            if self.now_ms() > end:
                raise BackendFailure("S504", f"did not converge on station {station['name']} in time (errors forward, left, "
                                             f"yaw, last {len(iterations[-14:])} looks: {iterations[-14:]})")
            # the axis that is furthest out, each error in units of its tolerance
            scores = {"yaw": abs(eyaw) / 0.05, "forward": abs(ex) / SERVO_TOL_M, "lateral": abs(ey) / SERVO_TOL_M}
            # (yaw is corrected first and tighter than asked: a heading a few degrees off, with a sideways error, takes
            # the station marker out of the camera's view)
            axis = max(scores, key=scores.get)
            if axis == "yaw":
                self.turn_by(float(np.clip(0.9 * eyaw, -0.5, 0.5)))
            elif axis == "forward":
                vx, vy, seconds = _burst("forward", ex)
                self.command(vx, vy, 0.0, seconds)
            else:
                vx, vy, seconds = _burst("lateral", ey)
                self.command(vx, vy, 0.0, seconds)
            self.settle()
            readings = self.look(frames=4, ids={mid})
            for _ in range(3):  # the marker slipped out of view: face where it should be, and look again
                if mid in readings and readings[mid].confidence >= MARKER_MIN_CONF:
                    break
                aim = self._angle_to(*station["marker"]["pos"][:2])
                self.events.append({"t_ms": self.now_ms(), "event": "reacquire", "turn_rad": round(float(aim), 3)})
                if abs(aim) > 0.06:
                    self.turn_by(aim)
                readings = self.look(frames=4, ids={mid})

    def _confirm_pose(self, station: dict) -> bool:
        """After the stance has settled: look once more, and be within the tolerance again."""
        readings = self.look(frames=5, ids={station["marker"]["id"]})
        gx, gy, gyaw = station["pose"]
        if station["marker"]["id"] not in readings:
            return False
        return (math.hypot(gx - self.pose[0], gy - self.pose[1]) <= STATION_TOL_M
                and abs(site_mod.wrap(gyaw - self.pose[2])) <= STATION_TOL_RAD)

    # ------------------------------------------------------------------------------------------ arms

    def _ready_pose(self, side: str) -> np.ndarray:
        """A pose with the arm raised in front, hand over the table edge: the seed the reach is solved from."""
        q = self.q_carry.copy()
        sp, roll, elbow = (22, 23, 25) if side == "right" else (15, 16, 18)
        q[sp], q[elbow], q[roll] = -0.6, 0.9, -0.15 if side == "right" else 0.15
        return q

    def _clear_pose(self, side: str) -> np.ndarray:
        """The carry pose with the arm swung out and forward: the hand clear of the hip. The walking controller's own
        arm pose has the thumb against the thigh, and lifting from there kicks the wrist (a torque of 5 N m)."""
        q = self.q_carry.copy()
        sp, roll = (22, 23) if side == "right" else (15, 16)
        q[roll] += -0.4 if side == "right" else 0.4
        q[sp] -= 0.3
        return q

    def _arm_begin(self) -> None:
        """Start publishing arm_sdk with the measured joints and weight 0: nothing changes until the weight rises."""
        if self.robot.arm_target is None:
            self.robot.set_arm(self.robot.low_q.copy(), self.arm_kp, self.arm_kd, 0.0)
        self.robot.start_arm_sdk()

    def _arm_move(self, q_to: np.ndarray, seconds: float, weight_to: float = 1.0, label: str = "") -> None:
        """Min-jerk from the last commanded joints to `q_to` over at least SEGMENT_S, with the arm_sdk weight ramped
        alongside (so the first motion blends in rather than jumps). Then wait for the joints to arrive: a joint still
        further than ARM_TRACK_TOL_RAD from its target after 1.5 s is S503."""
        seconds = max(seconds, SEGMENT_S if self.robot.arm_weight < weight_to else 1.0)
        q0, w0 = self.robot.arm_target[0].copy(), self.robot.arm_weight
        q_to = np.array(q_to, float)
        steps = int(seconds / 0.02)
        t0 = self.now_ms()
        for i in range(1, steps + 1):
            self.check()
            a = i / steps
            self.robot.set_arm(q0 + _minjerk(a) * (q_to - q0), self.arm_kp, self.arm_kd, w0 + (weight_to - w0) * _minjerk(a))
            time.sleep(0.02)
        joints = list(range(12, 29))
        end = self.now_ms() + 1500
        while True:
            self.check()
            err = float(np.max(np.abs(self.robot.low_q[joints] - q_to[joints])))
            if err <= ARM_TRACK_TOL_RAD:
                break
            if self.now_ms() > end:
                worst = 12 + int(np.argmax(np.abs(self.robot.low_q[joints] - q_to[joints])))
                raise BackendFailure("S503", f"arm joint {worst} is {err:.3f} rad from its target after {label or 'a motion'} "
                                             f"(tolerance {ARM_TRACK_TOL_RAD})")
            time.sleep(0.02)
        self.events.append({"t_ms": self.now_ms(), "event": "arm", "label": label, "seconds": round(seconds, 2),
                            "tracking_rad": round(err, 4), "took_ms": self.now_ms() - t0})

    def _hand_to(self, side: str, q: list[float], label: str) -> None:
        self.robot.hand_command(side, q, seconds=2.0, check=self.check)
        self.sleep(0.5)
        err = float(np.max(np.abs(self.robot.hand_q[side] - np.array(q))))
        self.events.append({"t_ms": self.now_ms(), "event": "hand", "label": label, "error_rad": round(err, 4)})
        if err > HAND_TOL_RAD:
            raise BackendFailure("S505", f"the {side} hand did not reach its {label} pose: {err:.3f} rad off "
                                         f"(tolerance {HAND_TOL_RAD})")

    def _solve(self, side: str, target, R, seeds, waist: float):
        for seed in seeds:
            q, info = self.kin.solve(side, target, R, seed, posture=self._ready_pose(side), rot_weight=0.5, iters=250,
                                     waist_max=waist)
            if q is not None:
                return q, info
        return None, None

    def _plan_reach(self, side: str, target_for, extra: tuple = (), degs=FINGERS_DOWN_DEG) -> dict:
        """Inverse kinematics for a reach: `target_for(R)` gives the pinch point (pelvis frame) for a hand orientation R.
        Solves the pre-grasp (behind and above it), the grasp, and any `extra` offsets (a lift), each in the same hand
        orientation and joined by small joint moves. Tries the hand's pitch and the waist's lean in order; S502 if no
        combination works, before any motion."""
        rng = np.random.default_rng(7)
        joints = ARM_JOINTS[side]
        ready = self._ready_pose(side)
        shoulder = np.array([0.0, HAND_REST_Y[side], 0.29])
        p0 = np.asarray(target_for(_frame(math.radians(degs[0]))), float)
        if np.linalg.norm(p0 - shoulder) > 0.62:  # beyond the arm's length with the waist leaning: no search
            raise BackendFailure("S502", f"no inverse-kinematics solution: the target is {np.linalg.norm(p0 - shoulder):.2f} "
                                         "m from the shoulder, further than the arm reaches")
        for waist in WAIST_LEAN_RAD:
            for deg in degs:
                R = _frame(math.radians(deg))
                p = np.asarray(target_for(R), float)
                seeds = [ready, self.q_carry] + [self._jitter(ready, joints, rng) for _ in range(5)]
                pre = p + R @ np.array([-APPROACH_BACK, 0, 0]) + np.array([0, 0, APPROACH_UP])
                q_pre, _ = self._solve(side, pre, R, seeds, waist)
                if q_pre is None:
                    continue
                q_grasp, info = self._solve(side, p, R, [q_pre], waist)
                if q_grasp is None:
                    continue
                plan = {"pre": q_pre, "grasp": q_grasp, "fingers_down_deg": deg, "waist_rad": float(q_grasp[WAIST_PITCH]),
                        "position_error_m": round(info["position_error_m"], 4), "pinch": p}
                ok = max(abs(q_grasp[joints] - q_pre[joints])) < 1.2
                for name, offset in extra:
                    q, _ = self._solve(side, p + np.array(offset), R, [q_grasp], waist)
                    if q is None:
                        ok = False
                        break
                    plan[name] = q
                if ok:
                    return plan
        raise BackendFailure("S502", "no inverse-kinematics solution for the reach (hand pitch 0-45 degrees, waist lean up "
                                     f"to {WAIST_LEAN_RAD[-1]} rad): the target {np.round(p0, 3).tolist()} (pelvis frame) is "
                                     "out of reach")

    def _jitter(self, q: np.ndarray, joints, rng) -> np.ndarray:
        out = q.copy()
        out[joints] = np.clip(q[joints] + rng.uniform(-0.8, 0.8, len(joints)), self.kin.limits[joints, 0],
                              self.kin.limits[joints, 1])
        return out

    def _reposition(self) -> dict:
        """At a station, before arm work: look at its marker again and, if the robot is no longer within tolerance (the
        free stance takes a recovery step about every 45 s), servo back. S517 if the marker is not seen."""
        if self.station is None:
            raise BackendFailure("S503", "an arm step needs the robot to have moved to a station first")
        station = self.stations[self.station]
        mid = station["marker"]["id"]
        gx, gy, gyaw = station["pose"]
        self.settle()
        info = {}
        for attempt in range(3):
            readings = self.look(frames=5, ids={mid})
            reading = readings.get(mid)
            if reading is None or reading.confidence < MARKER_MIN_CONF:
                raise BackendFailure("S517", f"station marker {mid} was not seen before arm work")
            err = math.hypot(gx - self.pose[0], gy - self.pose[1])
            info = {"error_m": round(err, 4), "error_rad": round(abs(site_mod.wrap(gyaw - self.pose[2])), 4),
                    "reservoed": attempt > 0}
            if err <= STATION_TOL_M and abs(site_mod.wrap(gyaw - self.pose[2])) <= STATION_TOL_RAD:
                return info
            self._servo(station, reading, self.now_ms() + int(SERVO_TIMEOUT_S * 1000))
        raise BackendFailure("S504", f"could not return to station {self.station} before arm work")

    def _arm_stance(self, model: str) -> str:
        """Ask the stand-in for the balance support the site names for arm work (0046 U4: the free stance moved the
        robot about 12 cm when the arm reached; `supported` welds the pelvis, which stands in for balance the robot's
        own controller provides). Waits for the stand-in to report it holding. Returns the model now in force."""
        if self.identity.get("stance_model") == model and model == "free":
            return model
        self.settle()
        code, info = self.sim.SetStance(model)
        if code != 0:
            raise BackendFailure("S503", f"the stand-in did not switch to the {model} stance (code {code})")
        end = self.now_ms() + 6000
        while self.now_ms() < end:
            self.check()
            status = self.status()
            if status and status["stance"]["model"] == model and status["stance"]["phase"] in ("hold", "standing"):
                break
            time.sleep(0.1)
        self.events.append({"t_ms": self.now_ms(), "event": "stance", "model": model})
        return model

    def grasp(self, obj_name: str) -> dict:
        """Reach, close the hand, check the object's marker is at the hand, weld it, lift to the carry pose (0046 U5, U7)."""
        self.events = []
        obj = site_mod.obj(self.site, obj_name)
        face, mid = obj["marker"]["face"], obj["marker"]["id"]
        details = {"object": obj_name, "station_check": self._reposition()}
        if self.held is not None:
            raise BackendFailure("S506", f"the robot already holds {self.held['object']}")
        readings = self.look(frames=10)
        seen = readings.get(mid)
        if seen is None or seen.confidence < 0.6:
            raise BackendFailure("S506", f"the marker of {obj_name} (id {mid}) was not seen at the station: the object is "
                                         "absent or has been moved (checked before the hand moves)")
        axis = int(np.argmax(np.abs(site_mod.FACE_NORMALS[face])))
        centre = object_centre(seen, obj["half_size"][axis])
        side = obj["grasp"]["hand"] if obj["grasp"]["hand"] != "nearest" else ("right" if centre[1] < 0 else "left")
        details.update(hand=side, object_in_pelvis=[round(float(v), 4) for v in centre], marker_confidence=seen.confidence,
                       marker_range_m=round(seen.range_m, 3))
        plan = self._plan_reach(side, lambda R: centre + np.array([0, 0, GRASP_RISE]),
                                extra=(("lift", (0.0, 0.0, 0.10)),))  # S502 here, before any motion
        details["reach"] = {k: plan[k] for k in ("fingers_down_deg", "waist_rad", "position_error_m")}
        details["arm_work_stance"] = self._arm_stance(self.site["robot"]["arm_stance"])
        self._arm_begin()
        self._arm_move(self._clear_pose(side), 1.6, label="clear of the hip")
        self._arm_move(plan["pre"], 2.0, label="pre-grasp")
        self._arm_move(plan["grasp"], 1.6, label="grasp")
        self._hand_to(side, HAND_CLOSE[side], "closed")
        # the object must be seen at the hand, from the measured joints, before it is welded
        held = self.look(frames=5, ids={mid}).get(mid)
        _, hand_p = self.kin.grasp_frame(side, self.robot.low_q)
        if held is None or float(np.linalg.norm(held.position - hand_p)) > HELD_RADIUS_M:
            raise BackendFailure("S506", f"the marker of {obj_name} is not seen at the {side} hand "
                                         f"({'not seen' if held is None else f'{np.linalg.norm(held.position - hand_p):.3f} m from the hand'})")
        details["marker_at_hand_m"] = round(float(np.linalg.norm(held.position - hand_p)), 4)
        hand_R, _ = self.kin.grasp_frame(side, self.robot.low_q)
        offset_hand = hand_R.T @ (object_centre(held, obj["half_size"][axis]) - hand_p)  # where it sits in the hand
        code, attached = self.sim.Attach(obj_name, side)
        if code != 0:
            raise BackendFailure("S506", f"the simulator could not attach {obj_name} to the {side} hand (code {code})")
        self.held = {"object": obj_name, "hand": side, "offset_in_hand": offset_hand, "fingers_down_deg": plan["fingers_down_deg"]}
        details["object_in_hand_m"] = [round(float(v), 4) for v in offset_hand]
        self.events.append({"t_ms": self.now_ms(), "event": "attached", "relative_position": attached.get("relative_position")})
        self._arm_move(plan["lift"], 1.6, label="lift")
        self._arm_move(plan["pre"], 1.6, label="retreat")
        self._arm_move(self._clear_pose(side), 1.6, label="clear of the hip")
        self._arm_move(self.q_carry, 1.6, label="carry")
        self._arm_stance("free")  # walking is the policy's: back to the free stance
        details["events"] = self.events
        return details

    def place(self, obj_name: str, location: str) -> dict:
        """Reach over the location's region, lower, open the hand, detach, retract to the carry pose."""
        self.events = []
        if self.held is None or self.held["object"] != obj_name:
            raise BackendFailure("S506", f"place requires a prior successful grasp of {obj_name}")
        loc = site_mod.location(self.site, location)
        station = self.stations[loc["station"]]
        if self.station != station["name"]:
            raise BackendFailure("S503", f"place at {location} needs the robot at station {station['name']}")
        obj = site_mod.obj(self.site, obj_name)
        side = self.held["hand"]
        details = {"object": obj_name, "location": location, "hand": side, "station_check": self._reposition()}
        # the region's centre, in the pelvis frame, through the station marker seen now
        readings = self.look(frames=10, ids={station["marker"]["id"]})
        reading = readings.get(station["marker"]["id"])
        if reading is None or reading.confidence < MARKER_MIN_CONF:
            raise BackendFailure("S517", f"station marker {station['marker']['id']} was not seen before placing")
        lo, hi = np.array(loc["region"]["min"], float), np.array(loc["region"]["max"], float)
        target_marker = np.array([(lo[0] + hi[0]) / 2, lo[1] + obj["half_size"][2] + 0.002, (lo[2] + hi[2]) / 2])
        # the marker frame is (x right, y up, z out) with a vertical panel: y is the height above the marker's centre
        centre = reading.rotation @ target_marker + reading.position
        details["target_in_pelvis"] = [round(float(v), 4) for v in centre]
        plan = self._plan_reach(side, lambda R: centre - R @ self.held["offset_in_hand"],
                                extra=(("above", (0.0, 0.0, 0.10)),), degs=(self.held["fingers_down_deg"],))
        details["reach"] = {k: plan[k] for k in ("fingers_down_deg", "waist_rad", "position_error_m")}
        details["arm_work_stance"] = self._arm_stance(self.site["robot"]["arm_stance"])
        self._arm_begin()
        self._arm_move(self._clear_pose(side), 1.0, label="clear of the hip")
        self._arm_move(plan["pre"], 2.0, label="pre-place")
        self._arm_move(plan["grasp"], 1.6, label="place")
        self._hand_to(side, [0.0] * 7, "open")
        code, _ = self.sim.Detach(obj_name)
        if code != 0:
            raise BackendFailure("S506", f"the simulator could not detach {obj_name} (code {code})")
        self.held = None
        self.events.append({"t_ms": self.now_ms(), "event": "detached"})
        self._arm_move(plan["pre"], 1.6, label="retreat")
        self._arm_move(self._clear_pose(side), 1.6, label="clear of the hip")
        self._arm_move(self.q_carry, 1.6, weight_to=0.0, label="carry")
        self.robot.stop_arm_sdk()
        self._arm_stance("free")
        details["events"] = self.events
        return details

    # ------------------------------------------------------------------------------------------ stop

    def hold(self) -> dict:
        """The safe state (0038, hazard H25): zero velocity, the arms held where they are through arm_sdk, the hands
        unchanged, the robot kept standing. Never Damp, zero torque, SwitchToUserCtrl or rt/lowcmd."""
        if self.fallen:
            return {"velocity": "none sent: the robot fell", "arms": "none sent", "hands": "unchanged"}
        code = self.robot.set_velocity(0.0, 0.0, 0.0, 1.0)
        arms = "held" if self.robot.hold_arm() else "walking controller's own"
        return {"velocity": "0" if code == 0 else f"failed ({code})", "arms": arms, "hands": "unchanged"}

    def confirm_stopped(self) -> dict:
        """Confirm from lowstate that the robot is upright and still: over a window of 0.5 s of simulation time the
        pelvis IMU's rate, the tilt and every joint's velocity are within tolerance, within 3 s of the stop."""
        t0 = self.now_ms()
        result = {"confirmed": False, "timeout_ms": int(CONFIRM_TIMEOUT_S * 1000), "window_ms": CONFIRM_WINDOW_MS,
                  "tolerances": {"gyro_rad_s": CONFIRM_GYRO, "joint_velocity_rad_s": CONFIRM_JOINT_VEL,
                                 "tilt_deg": round(math.degrees(CONFIRM_TILT_RAD), 1)}, "t_stop_ms": t0}
        while self.now_ms() - t0 <= CONFIRM_TIMEOUT_S * 1000:
            now = self.now_ms()
            s = self.robot.samples_since(now - CONFIRM_WINDOW_MS)
            if s and s[0].tick >= t0 and len(s) >= CONFIRM_WINDOW_MS / 4 * 0.6 and s[-1].tick - s[0].tick >= CONFIRM_WINDOW_MS * 0.9:
                gyro = max(float(np.linalg.norm(x.gyro)) for x in s)
                joints = max(float(np.max(np.abs(x.dq))) for x in s)
                tilt = max(math.acos(max(-1.0, min(1.0, 1 - 2 * (x.quat[1] ** 2 + x.quat[2] ** 2)))) for x in s)
                result.update(max_gyro_rad_s=round(gyro, 4), max_joint_velocity_rad_s=round(joints, 4),
                              max_tilt_deg=round(math.degrees(tilt), 2), samples=len(s))
                if gyro <= CONFIRM_GYRO and joints <= CONFIRM_JOINT_VEL and tilt <= CONFIRM_TILT_RAD:
                    result.update(confirmed=True, window_start_ms=s[0].tick, window_end_ms=s[-1].tick,
                                  after_ms=s[-1].tick - t0)
                    return result
            time.sleep(0.02)
        result["after_ms"] = self.now_ms() - t0
        return result
