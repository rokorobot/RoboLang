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
from .heading import HeadingTracker, yaw_of
from .kinematics import ARM_JOINTS, WAIST_PITCH, Clearance, Kinematics
from .observe import HAND_CLOSED, HELD_RADIUS_M, READY_LOWSTATE_AGE_S, Hand, Ready, Snapshot
from .observe import hand_state
from .perception import (
    MAX_UP_DISAGREEMENT_RAD,
    Perception,
    imu_up,
    object_centre,
    pelvis_pose_in_map,
)
from .response import CONFIRM_DWELL_S, expected, plan_correction
from .robot import Robot, arm_gains

# ---- safety and stop
TILT_LIMIT_RAD = math.radians(40)  # S516: the IMU's tilt from upright beyond this means the robot is falling or fallen
CONFIRM_TIMEOUT_S = 3.0  # a stop must be confirmed (upright and still, from lowstate) within this, in simulation time
CONFIRM_WINDOW_MS = 500
CONFIRM_GYRO = 0.045  # rad/s: the pelvis rotates no faster than this ...
CONFIRM_JOINT_VEL = 0.09  # rad/s: ... and no joint moves faster than this (the held arms included)
# ... and the legs and waist have not moved over the window, as a whole: a robot coasting to rest after a walk passed
# the two bounds above at 0.089 rad/s while its whole body still slid 1.6 cm in the window (x86 CI, E5 twice). Joint
# travel over the window catches what a velocity bound let through: 0.01 rad at the hip is under a centimetre at the
# pelvis. From lowstate only.
CONFIRM_LEG_TRAVEL_RAD = 0.01
LEG_AND_WAIST = slice(0, 15)  # SDK order: the 12 leg joints, then the 3 waist joints
CONFIRM_TILT_RAD = math.radians(15)
SETTLE_GYRO, SETTLE_LEG_VEL, SETTLE_WINDOW_MS, SETTLE_TIMEOUT_S = 0.03, 0.05, 600, 9.0
# ---- localization and walking
# before arm work the robot is put back on station only if it has moved further than this
REPOSITION_M, REPOSITION_RAD = 0.08, 0.15
# `move` promises 5 cm and 0.1 rad (0046 U5): after settling it must still be within this, which leaves margin
STATION_TOL_M, STATION_TOL_RAD = 0.042, 0.09
# The servo stops inside these, tighter than the confirm limits above: the heading creeps while the robot waits to be
# confirmed (CREEP_PRIOR), and a servo that stops at the confirm limit is often confirmed outside it.
SERVO_TOL_M, SERVO_TOL_RAD = 0.035, 0.05
# rad/s: a standing robot's heading drifts left at about this (measured 0.006 empty, 0.008 loaded; then observed)
CREEP_PRIOR = 0.007
# The creep as the IMU sees it: the yaw's slope over the last CREEP_WINDOW_MS, if that window began at least
# CREEP_QUIET_MS after the last velocity command ended (the stance's own transient is over)
CREEP_WINDOW_MS, CREEP_QUIET_MS = 1000, 2000
COARSE_M, COARSE_RAD = 0.25, 0.3  # an error beyond this is for the coarse bursts (below), not the measured corrections
# A step is 5-17 cm and a burst that moves less takes no step at all, so an error just outside the servo's tolerance may
# have no correction that improves it (measured: a loaded robot 4 cm to the left of the shelf's station for 150 s).
# After this many looks without a better one the servo goes on to the confirming look, which holds the robot to the
# confirm limits above and to nothing less: the station tolerance (5 cm, 0.1 rad) is not touched.
STAGNANT_LOOKS = 4
RELAXED_RAD = 0.07  # the servo's heading tolerance then: the confirm limit less what the creep may add
# The approach point (0.5 m before the station, not confirmed) is only to line the robot up for the last walk, and the
# final servo takes any error it is left with that is under COARSE_M and COARSE_RAD. When the servo to it stagnates
# (under a creep of 0.025 rad/s the heading is inside 0.07 rad for a moment in each 5 s look, and one failed move spent
# its 150 s at the approach point 0.1 m and 0.1 rad off) it goes on from this much error:
APPROACH_RELAXED_M, APPROACH_RELAXED_RAD = 0.12, 0.15
MARKER_MAX_RANGE_M, MARKER_MIN_CONF = 2.9, 0.5  # a station marker corrects the belief only when seen this well
# ... and only if the vertical it implies is the IMU's, to within MAX_UP_DISAGREEMENT_RAD (perception.py)
APPROACH_M = 0.5  # the walk ends this far before the station, facing it, and the marker is looked for there
# ... after passing through a point this much further out on the station's axis, where the marker is in view
STAGING_M = 1.5
WALK_SPEED = 0.5
WALK_GAIN, BACK_GAIN, SIDE_GAIN = 0.95, 0.9, 0.64  # travel (coast included) per unit commanded, measured
SHUFFLE_S = 1.2  # one burst of the turn-in-place shuffle ...
SHUFFLE_BACK_M = 0.035  # ... and drifts this far backward (measured: 0.03-0.04 m a burst), which the belief must carry
SHUFFLE_RAD = {1: 0.127, -1: 0.111}  # ... turns this much to the left (+) and to the right (-): measured (the plan's
# aim; the belief's heading comes from the IMU, heading.py)
LEG_MAX_M = 1.6  # the longest walk between two looks
SCAN_STEP_RAD = 0.6  # a scan for a marker turns this much between looks
# ... and if they did not find it, from further than this from the approach point, a walk to it (to within the stop)
APPROACH_AGAIN_M, APPROACH_AGAIN_STOP_M = 0.15, 0.08
SIDESTEPS, SIDESTEP_M = 3, 0.2  # ... after this many sidesteps of about this much toward the marker's side
MOVE_TIMEOUT_S, SERVO_TIMEOUT_S, MARKER_TIMEOUT_S = 300.0, 150.0, 90.0  # simulation time: S504, S504, S517
# A move to a station the robot already stands beside (the same station, or one a step away, as a dock 12 cm behind a
# destination) is the servo alone, from where it stands: walking out to the approach point 0.5 m short and coming in
# again moved a robot with a box in hand by several centimetres (S502 at the place that followed) and, once, made the
# belief jump 0.5 m on the way back (S504). Within COARSE_M the servo's measured corrections apply.
DIRECT_M, DIRECT_RAD = COARSE_M, 0.2
# ---- arms
ARM_TRACK_TOL_RAD = 0.12  # S503: a joint further than this from its target when a motion has finished
HAND_TOL_RAD = 0.25  # S505
HAND_CLOSE = HAND_CLOSED  # the Dex3 closing pose (observe.py judges a hand "closed" against the same pose)
SEGMENT_S = 1.6  # the shortest arm motion between two poses (min-jerk, and always at least 1 s)
# (HELD_RADIUS_M, observe.py: the object's marker is "at the hand" within this of the grasp point; 5-6 cm when held)
APPROACH_BACK, APPROACH_UP = 0.09, 0.03  # the pre-grasp: behind the grasp point along the fingers, and above it
# The pinch point is this far above the object's centre, and the hand's pitch is tried flat first. In phase 2 the
# pitch was 15 degrees and the pinch 1 cm below the centre; in the robot's own model the open fingers then reached
# 3.2 cm into the table at the grasp (the scene's hands touch nothing, so it was not seen). Measured over pitches of
# 0-30 degrees and pinches of -1 to +3 cm (docs/evidence/g1-phase3-observations.md): no hand pose keeps the open
# hand off the table by a margin and the closed fingers off the object's top face (where the marker is) at once: a
# pinch 1 cm up cleared the table by 0.8 cm and the closed fingers hid a corner of the marker (S506, "not seen at the
# hand"). Flat with the pinch at the centre, the hand just touches the table top (0 to -0.7 cm; the contact legs allow
# 1 cm) and the closed fingers stay level with the object's top face.
GRASP_RISE = 0.0
FINGERS_DOWN_DEG = (0, 15, 30, 45)  # hand orientations tried, in this order
WAIST_LEAN_RAD = (0.0, 0.15, 0.3)  # waist pitch allowed, tried in this order
HAND_REST_Y = {"right": -0.10, "left": 0.10}
# ---- snapshots (phase 3)
SNAPSHOT_FRAMES = 10  # head-camera frames per snapshot (2 s at the camera's 5 Hz)
HAND_STATE_AGE_S = 1.0  # a hand state older than this (wall clock) is not used
# The arm paths of grasp, place and the observation motion are checked, before any arm motion, against the site's
# declared tables and obstacles with the robot's own geometry (kinematics.Clearance). Margins: 3 cm sideways and 1.5 cm
# vertical; on the legs where the hand is meant to be at the object (the last approach, the first lift, the lowering and
# the release) the hand and the object, which rests on the table there, are checked with no vertical margin and may
# overlap the table by 1 cm (the object's position, from its marker, is known to about a centimetre).
CONTACT_OVERLAP_M = 0.01  # (see above)
# A place aims at its region's centre; if the arm cannot reach it (S502), at a point this far nearer the robot's side
# across the region, in turn, if the region is wide enough that the object's marker still lands PLACE_INSIDE_M inside
# it. Measured: with three slots on one table the outer one was out of reach in 2 of 3 runs when the robot stood 2-5 cm
# off, inside its tolerance (docs/evidence/g1-phase4-acceptance.md).
PLACE_SHIFTS_M = (0.015, 0.03)
PLACE_INSIDE_M = 0.03  # twice the verdict's margin (observe.REGION_MARGIN_M)
PLACE_ABOVE_M = 0.012  # the object is released this far above the table's surface and drops (the hand then clears it)
RAISED_ABOVE_PRE_M = (0.12, 0.10, 0.08, 0.06, 0.04)  # the hand is raised this far above the pre-grasp height, in turn
RAISED_OUT_M = (0.0, 0.05, 0.10)  # the raised pose beside the body is tried this much further out, in turn
CARRY_CLEARANCE_M = 0.02  # a held object clears every link of the robot by at least this in the carry pose ...
CARRY_PLAN_M = (0.021,)  # ... planned for this: the walking pose moved as little as clearing it by 2 cm takes
# Before the arm shows a held object, the whole path of the arm and the object is checked against the site's declared
# tables and obstacles. The belief pose is corrected by the station marker (0.3-1.5 cm from the truth in the phase 2
# runs) and the pelvis height is known to about 1.5 cm, so the declared boxes are grown by 3 cm sideways and 1.5 cm
# up and down before the arm's spheres and the object's points are tested against them.
CLEARANCE_XY_M, CLEARANCE_Z_M, CLEARANCE_SAMPLES = 0.03, 0.015, 10
RAISE_M = 0.08  # the raised waypoint is this far above the view pose (the joint-space line down to it dips less)


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


# a forward walk at vx 0.4 covers this much a second, coast included (fit over 85 walks of 0.9-2.5 s: sd 1.2 cm)
FORWARD_WALK_M_S = 0.35


def _burst(axis: str, error: float) -> tuple[float, float, float]:
    """The (vx, vy, seconds) of the burst that should cancel `error` (metres along the axis: forward, or left), chosen
    from the measured steps. Long errors are walked: forward 0.35 m per second (it was taken as 0.30: every last walk
    to the station overshot by 0.1 m and the servo spent its next corrections on that), sideways about 0.3."""
    table = {
        "forward": FORWARD_BURSTS if error > 0 else BACK_BURSTS,
        "lateral": LEFT_BURSTS if error > 0 else RIGHT_BURSTS,
    }[axis]
    want = abs(error)
    if want > table[-1][1] * 1.3:
        seconds = min(want / (FORWARD_WALK_M_S if axis == "forward" and error > 0 else 0.30), 4.0)
    else:
        seconds = min(table, key=lambda entry: abs(entry[1] - want))[0]
    sign = 1.0 if error > 0 else -1.0
    return (0.4 * sign, 0.0, seconds) if axis == "forward" else (0.1, 0.3 * sign, seconds)


def servo_band(approach: bool, stagnant: bool, tol_m: float, tol_rad: float) -> tuple[float, float]:
    """The position (m) and heading (rad) the servo stops inside. Normally its own tolerances (`tol_m`, `tol_rad`);
    after STAGNANT_LOOKS looks without a better error, the confirm limit less what the creep may add for the final
    servo (it then goes on to the confirming look, which still decides), and APPROACH_RELAXED_* for the approach point
    (no confirmation follows it)."""
    if not stagnant:
        return tol_m, tol_rad
    return (APPROACH_RELAXED_M, APPROACH_RELAXED_RAD) if approach else (STATION_TOL_M, RELAXED_RAD)


# A leg straight to the side is a command the walking policy answers at random. Measured on the loaded robot (box in the
# right hand, 3.4 s commands, docs/evidence/g1-phase4-prereqs/exp_lateral_gait.py): (0, 0.3) to the left moved it 0.52
# and 0.59 m; (0.1, 0.3) to the left 0.005 and 0.005 m (to the right 0.51 and 0.58); (0.16, 0.3) 0.13 and 0.76 m;
# (0, 0.3, -0.2) 0.003 and 0.34 m; and from (0.2, 0.3) up it always moved (0.69-0.84 m left, 0.78 right). A leg of the
# walk that is mostly sideways is therefore walked as two diagonals, back and forward at that speed: it keeps the robot
# on its line and keeps the command out of the policy's dead zone. (With the heading dead-reckoned from the commands
# the heading error was seldom under 0.04 rad at the start of such a leg, so the leg carried a yaw rate; with the IMU's
# heading it often does not, and a run of `e_pick_place`'s kind stood still for 20 s: S517.)
LATERAL_VY, LATERAL_VX = 0.25, 0.2  # a leg with |vy| at least the first and |vx| under the second is "mostly sideways"
LATERAL_MIN_S = 1.6  # shorter than this it is one diagonal (a policy needs about a second to settle into a gait)


def walk_legs(vx: float, vy: float, wz: float, seconds: float) -> list[tuple[float, float, float, float]]:
    """The commands of one leg of the walk: the leg itself, or, if it is mostly sideways (see above), two diagonals that
    cancel in the forward direction, the second the way the leg's own forward component pointed."""
    if abs(vy) < LATERAL_VY or abs(vx) >= LATERAL_VX:
        return [(vx, vy, wz, seconds)]
    way = 1.0 if vx >= 0 else -1.0
    if seconds < LATERAL_MIN_S:
        return [(way * LATERAL_VX, vy, wz, seconds)]
    return [(-way * LATERAL_VX, vy, wz, seconds / 2), (way * LATERAL_VX, vy, wz, seconds / 2)]


def _minjerk(a: float) -> float:
    return a**3 * (10 - 15 * a + 6 * a * a)


class G1Operations:
    """The operations of the executor. `guard()` must run first: it creates the DDS participant and the clients."""

    tracker: HeadingTracker | None = None  # the belief heading (made in guard(), once there is an IMU to read)
    robot: Robot | None = None

    def __init__(self, site: dict, stop: Stop | None = None, head_dir: str | None = None):
        self.site, self.stop, self.head_dir = site, stop, head_dir
        self.stations = {s["name"]: s for s in site["stations"]}
        self.by_marker = {s["marker"]["id"]: s for s in site["stations"]}
        start = site["robot"]["start"]
        # The belief: pelvis (x, y, yaw) in the map (the `pose` property). Position is dead-reckoned from the commands
        # and put right by a station marker; the heading is the last marker fix's plus the IMU's change in yaw since
        # (heading.py), the dead reckoning of the commanded yaw rates having been 0.4 rad out under heading creep.
        self._xy = [float(start[0]), float(start[1])]
        self._heading0 = float(start[2])
        self.station: str | None = None  # the station the robot last arrived at
        self.held: dict | None = None  # {"object", "hand"} while an object is welded to a hand
        self.events: list[dict] = []  # what the current step did, for its record
        self.observations: list[dict] = []  # marker readings taken, for the record
        self.identity: dict | None = None
        self.sim = None
        self.robot: Robot | None = None
        self.kin: Kinematics | None = None
        self.clr: Clearance | None = None
        self.perc: Perception | None = None
        self.q_carry: np.ndarray | None = None
        self.fallen = False
        self.creep = CREEP_PRIOR  # rad/s: the heading's creep to the left while standing (observed in `_servo`)
        self.creep_reads = 0  # how many times the IMU has told it (`_note_creep_imu`)
        self.fix: tuple[int, float] | None = None  # (simulation time, heading) of the last station-marker fix
        self.last_look: tuple[int, bool] = (0, False)  # (simulation time, whether it was a fix) of the last look
        self.moved_ms = 0  # the simulation time at which the last velocity command was to end
        self.seen: dict = {}  # what the robot has watched each correction do (response.py: `expected`)
        self.stance_now: str | None = None  # the stand-in's balance support, as last set

    # ------------------------------------------------------------------------------------------ set-up

    def guard(self, model: str | None = None) -> None:
        """The simulation guard (S501), then the clients. Nothing is sent to the robot before it passes."""
        try:
            self.sim, self.identity = guard.require_simulation()
        except guard.GuardError as exc:
            raise BackendFailure(guard.CODE, exc.reason) from exc
        self.stance_now = self.identity.get("stance_model")
        head = self.head_dir or self.identity.get("head_camera_dir")
        if not head:
            raise BackendFailure("S501", "the robot reports no head camera frame source and none was given")
        self.robot = Robot()
        self.robot.wait_state()
        self.tracker = HeadingTracker(self._heading0, yaw_of(self.robot.last.quat), self.robot.last.tick)
        self._hold_heading(self.identity.get("stance_model") == "supported")  # (a stand-in started welded)
        self.kin = Kinematics(self.site["camera"])
        self.clr = Clearance(self.site["camera"], n_boxes=len(site_mod.declared_boxes(self.site)))
        self.perc = Perception(self.robot, self.kin, self.site, head)
        self.q_carry = self.robot.low_q.copy()  # the arms as the walking controller holds them: the carry pose
        self.q_carry[WAIST_PITCH - 2 : WAIST_PITCH + 1] = 0.0
        self.arm_kp, self.arm_kd = arm_gains(np.zeros(29), np.zeros(29))

    def now_ms(self) -> int:
        return self.robot.now_ms

    def versions(self) -> dict:
        return assets.versions()

    def status(self) -> dict | None:
        code, status = self.sim.Status()
        return status if code == 0 else None

    # ------------------------------------------------------------------------------------------ belief

    @property
    def pose(self) -> list[float]:
        """The belief pose of the pelvis in the map, [x, y, yaw]: the heading is read from the IMU now (heading.py)."""
        return [self._xy[0], self._xy[1], self._heading()]

    @pose.setter
    def pose(self, value) -> None:
        """Set the belief outright (a pose given, not seen: the heading is true from now on, nothing is learned)."""
        self._xy = [float(value[0]), float(value[1])]
        self._heading0 = float(value[2])
        last = getattr(self.robot, "last", None)
        if self.tracker is not None and last is not None:
            self.tracker.reset(self._heading0, yaw_of(last.quat), last.tick)

    def _heading(self) -> float:
        last = getattr(self.robot, "last", None)
        if self.tracker is None or last is None:
            return self._heading0
        return self.tracker.heading(yaw_of(last.quat), last.tick)

    def _hold_heading(self, welded: bool) -> None:
        """The pelvis is welded for arm work (`supported`), or free: a welded pelvis cannot turn, so the belief heading
        holds while it is and the IMU's drift is not followed (heading.py)."""
        last = getattr(self.robot, "last", None)
        if self.tracker is not None and last is not None:
            self.tracker.hold(welded, yaw_of(last.quat), last.tick)

    def _fix_pose(self, new, stamp_ms: int) -> float:
        """A station marker gave the pose `new` at simulation time `stamp_ms`: the position and the heading are right as
        of then, and the IMU carries the heading on from there. Returns how far the fix moved the heading."""
        self._xy = [float(new[0]), float(new[1])]
        sample = self.robot.sample_at(stamp_ms)
        if sample is None:
            sample = self.robot.last
        return self.tracker.fix(float(new[2]), yaw_of(sample.quat), sample.tick)

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
            raise BackendFailure(
                "S516",
                f"the robot's IMU tilt reached {math.degrees(self.tilt_rad()):.0f} degrees "
                f"(limit {math.degrees(TILT_LIMIT_RAD):.0f}): it fell or is falling",
            )
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
        self.observations.append(
            {
                "t_ms": self.now_ms(),
                "frames": frames,
                "markers": {str(k): v.as_dict(self.now_ms()) for k, v in sorted(readings.items())},
            }
        )
        best = None
        self.last_look = (self.now_ms(), False)
        for mid, reading in readings.items():
            if (
                mid in self.by_marker
                and reading.range_m <= MARKER_MAX_RANGE_M
                and reading.confidence >= MARKER_MIN_CONF
            ):
                if best is None or reading.range_m < best[1].range_m:
                    best = (mid, reading)
        if best is not None:
            mid, reading = best
            *new, up = pelvis_pose_in_map(self.by_marker[mid]["marker"], reading)
            w, x, y, z = self.robot.last.quat  # the IMU's vertical, in the pelvis frame
            imu_up = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])
            disagree = float(np.arccos(np.clip(up @ imu_up, -1, 1)))
            if disagree > MAX_UP_DISAGREEMENT_RAD:  # the wrong solution of the marker's pose: ignore this reading
                self.events.append(
                    {
                        "t_ms": self.now_ms(),
                        "event": "rejected",
                        "marker": mid,
                        "vertical_vs_imu_rad": round(disagree, 3),
                    }
                )
            else:
                before = [round(float(v), 3) for v in self.pose]
                jump = self._fix_pose(new, reading.stamp_ms)
                self.events.append(
                    {
                        "t_ms": self.now_ms(),
                        "event": "localized",
                        "marker": mid,
                        "from": before,
                        "to": [round(v, 3) for v in new],
                        "heading_jump_rad": round(jump, 4),
                    }
                )
                self.fix = (self.now_ms(), float(new[2]))
                self.last_look = (self.now_ms(), True)
        return readings

    # ------------------------------------------------------------------------------------------ snapshots (phase 3)

    def snapshot(self, show: set | frozenset = frozenset(), ready: bool = False) -> tuple[Snapshot, dict]:
        """One observation snapshot for verdicts, and a note for the record: about SNAPSHOT_FRAMES head-camera frames
        fused per marker, the hands (state from `rt/dex3/*/state`, grasp point from the measured joints), the IMU's
        vertical and, if `ready`, what `robot_ready` needs.

        A held object named in `show` is first brought into the camera's view (the carry pose hangs it out of view) and
        the arm put back afterwards: an **observation motion**, outside the plan's steps, arm only. It is made only if
        the whole path of the arm and the object is shown clear of the site's declared tables and obstacles
        (`_view_clearance`); if not, the arm does not move, the snapshot is taken as it is (so the atoms about the
        object stay `unknown`) and the note says why. The note is `{"observation_motion": {...}}` or empty."""
        if self.held is None or self.held["object"] not in show:
            return self._take(ready), {}
        # weld first, then plan: the free stance drifts in heading by about 0.01 rad a second, and the planning takes
        # seconds under load (measured: the heading read 0.02 rad further out at the snapshot than at the last look)
        stance = self._arm_stance(self.site["robot"]["arm_stance"], settle=False)
        clearance = self._view_clearance()
        if not clearance["clear"]:
            note = {
                "performed": False,
                "object": self.held["object"],
                "hand": self.held["hand"],
                "reason": "the path of the arm and the object is not shown clear of the declared tables and obstacles",
                "clearance": clearance,
            }
            return self._take(ready), {"observation_motion": note}
        return self._snapshot_showing(ready, clearance, stance)

    def _view_path(self) -> list[tuple[str, np.ndarray]] | None:
        """The waypoints of the observation motion: the carry pose, the pose with the hand clear of the hip, the same
        hand position raised 8 cm above the view pose's height beside the body (so the hand comes in over a table in
        front of the robot, not through its edge), and the view pose, where the grasp lifted the object. Only the
        raised one is new: inverse kinematics for the hand at the clear pose's place and the view pose's height and
        orientation. None if there is no solution."""
        side, lift = self.held["hand"], self.held["lift_waypoint"]
        clear = self._clear_pose(side)
        R_lift, p_lift = self.kin.grasp_frame(side, lift)
        _, p_clear = self.kin.grasp_frame(side, clear)
        rng = np.random.default_rng(11)
        seeds = [clear, lift, self._ready_pose(side)] + [self._jitter(clear, ARM_JOINTS[side], rng) for _ in range(5)]
        up, _ = self._solve(side, np.array([p_clear[0], p_clear[1], p_lift[2] + RAISE_M]), R_lift, seeds, 0.0)
        if up is None:
            return None
        return [("carry", self.robot.low_q.copy()), ("clear of the hip", clear), ("raised", up), ("show", lift)]

    def _view_clearance(self) -> dict:
        """Whether the arm and the held object stay clear of the site's declared tables and obstacles all the way from
        the carry pose to the view pose (and back along the same path): `_path_clearance` over the legs of
        `_view_path`, with the object in the hand and the fingers closed. `clear` is true only if every distance is over
        its limit; if no path can be planned, or in doubt, it is false and the arm stays."""
        held, side = self.held, self.held["hand"]
        path = self._view_path()
        if path is None:
            return {"clear": False, "why": "no inverse-kinematics solution for the raised waypoint", "closest": None}
        half = site_mod.obj(self.site, held["object"])["half_size"]
        legs = [self._leg(a, b, held=True, closed=True) for a, b in zip(path, path[1:])]
        out = self._path_clearance(side, legs, (half, held["offset_in_hand"], held["rotation_in_hand"]))
        out["waypoints"] = [name for name, _ in path]
        return out

    @staticmethod
    def _leg(a: tuple, b: tuple, contact: bool = False, held: bool = False, closed: bool = False) -> dict:
        """One straight move in joint space between two named poses, for `_path_clearance`."""
        return {"from": a[0], "q0": a[1], "to": b[0], "q1": b[1], "contact": contact, "held": held, "closed": closed}

    def _take(self, ready: bool) -> Snapshot:
        readings, frames = self.perc.snapshot(ids=set(self.perc.sizes), frames=SNAPSHOT_FRAMES, check=self.check)
        q, hands = self.robot.low_q.copy(), {}
        for side in ("left", "right"):
            if time.monotonic() - self.robot.hand_wall[side] > HAND_STATE_AGE_S:
                continue  # no state for this hand, or a stale one: it is left out, which makes `holding` unknown
            hq = self.robot.hand_q[side].copy()
            hands[side] = Hand(
                side,
                hand_state(side, hq),
                tuple(float(x) for x in hq),
                tuple(float(x) for x in self.kin.grasp_frame(side, q)[1]),
                self.robot.hand_tick[side],
            )
        up = tuple(float(x) for x in imu_up(self.robot.last.quat))
        return Snapshot(self.now_ms(), frames, readings, hands, up, self._ready() if ready else None)

    def _ready(self) -> Ready:
        """What `robot_ready` rests on: lowstate arriving (fresh), no motor error word, the IMU's tilt, and the walking
        service's state id from `GetFsmId` (a read)."""
        r = self.robot
        count, tick = r.low_count, r.low_tick
        self.sleep(0.2)
        sample = r.last
        fresh = (
            r.low_count - count >= 10 and r.low_tick > tick and time.monotonic() - r.low_wall <= READY_LOWSTATE_AGE_S
        )
        try:
            code, fsm = r.fsm_id()
        except Exception:  # the walking service did not answer: the atom is unknown
            code, fsm = -1, None
        return Ready(bool(fresh), sample.motor_error, self._tilt_of(sample), fsm if code == 0 else None, sample.tick)

    @staticmethod
    def _tilt_of(sample) -> float:
        w, x, y, z = sample.quat
        return math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))

    def _snapshot_showing(self, ready: bool, clearance: dict, stance: str) -> tuple[Snapshot, dict]:
        """Bring the held object into the camera's view, take the snapshot, and put the arm back in the carry pose.

        The carry pose hangs the object at the side of the thigh, outside the camera's field of view (measured: 46
        degrees below the axis against 29 of field), so the arm shows it where the grasp lifted it: the object's place
        at the table, 10 cm up, where the camera sees the object. The path is `_view_path`: the grasp's own `clear`
        and `lift` poses and one raised waypoint, shown clear of the declared tables and obstacles by
        `_view_clearance` (passed in). The arm works in the site's arm-work stance, as in `grasp` and `place`. A stop
        request stops it like any arm motion (`_arm_move` checks): the executor then holds the arms where they are
        and leaves the hands."""
        held, mark = self.held, len(self.events)
        side = held["hand"]
        t0 = self.now_ms()
        (_, _), (_, clear), (_, raised), (_, lift) = self._view_path()
        self.events.append({"t_ms": t0, "event": "show_held"})
        self._arm_begin()
        self._arm_move(clear, 1.6, label="clear of the hip")
        self._arm_move(raised, 1.6, label="raised")
        self._arm_move(lift, 1.6, label="show")
        self.sleep(1.0)  # let the arm and the object settle before the frames are taken
        snap = self._take(ready)
        self._arm_move(raised, 1.6, label="back raised")
        self._arm_move(clear, 1.6, label="back clear of the hip")
        self._arm_move(held["carry"], 1.6, label="carry")
        # The pelvis stays welded: nothing walks after the goal snapshot, and releasing it let the free stance turn the
        # robot by 0.017 rad in the next second (measured), after the snapshot that read it. `move` frees it again.
        note = {
            "performed": True,
            "object": held["object"],
            "hand": side,
            "waypoints": ["clear of the hip", "raised", "show", "raised", "clear of the hip", "carry"],
            "clearance": clearance,
            "arm_stance": stance,
            "t_start_ms": t0,
            "t_end_ms": self.now_ms(),
            "events": self.events[mark:],
        }
        return snap, {"observation_motion": note}

    # ------------------------------------------------------------------------------------------ walking

    def _dead_reckon(
        self, vx: float, vy: float, wz: float, seconds: float, shuffle: bool, t0_ms: int, t1_ms: int
    ) -> None:
        """Advance the belief's *position* by what a command should have done (measured gains: the policy ramps up,
        coasts, and sidesteps at about 0.64 of what is asked; a shuffle goes nowhere but a little backward). The heading
        is not dead-reckoned: it is the IMU's (`pose`), and the displacement is taken along the heading the robot had
        while the command ran (the mean over [t0, t1] of the IMU's), not the one it had when the command was sent: a
        walk that turns as it goes curves."""
        yaw = self._mean_heading(t0_ms, t1_ms)
        if shuffle:
            back = SHUFFLE_BACK_M * seconds / SHUFFLE_S
            forward, left = -back, 0.0
        else:
            forward = (WALK_GAIN if vx > 0 else BACK_GAIN) * vx * seconds
            left = SIDE_GAIN * vy * seconds
        x, y = self._xy
        self._xy = [
            x + forward * math.cos(yaw) - left * math.sin(yaw),
            y + forward * math.sin(yaw) + left * math.cos(yaw),
        ]

    def _mean_heading(self, t0_ms: int, t1_ms: int) -> float:
        """The belief heading averaged over simulation time [t0, t1] from the lowstate samples in it (the heading now if
        there are none)."""
        h_now = self._heading()
        if self.tracker is None or self.robot is None:
            return h_now
        samples = [x for x in self.robot.samples_since(t0_ms) if x.tick <= t1_ms][::10]
        if not samples:
            return h_now
        rel = [site_mod.wrap(self.tracker.heading(yaw_of(x.quat), x.tick) - h_now) for x in samples]
        return site_mod.wrap(h_now + sum(rel) / len(rel))

    def command(
        self, vx: float, vy: float, wz: float, seconds: float, wait: bool = True, shuffle: bool = False
    ) -> None:
        """One SetVelocity, the belief advanced by what it should do, and (unless `wait` is False) the wait for it."""
        self.check()
        t0 = self.now_ms()
        code = self.robot.set_velocity(vx, vy, wz, seconds)
        if code != 0:
            raise BackendFailure("S503", f"SetVelocity({vx:.2f}, {vy:.2f}, {wz:.2f}, {seconds:.2f}) returned {code}")
        self.moved_ms = self.now_ms() + int(seconds * 1000)
        try:
            if wait:
                self.sleep(seconds)
        finally:
            self._dead_reckon(vx, vy, wz, seconds, shuffle, t0, self.now_ms())

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
                self.command(0.0, 0.3 * (1 if k % 2 == 0 else -1), 0.2 * sign, SHUFFLE_S, shuffle=True)
            self.settle(4.0)
            self.look(frames=2)

    def _angle_to(self, x: float, y: float) -> float:
        return site_mod.wrap(math.atan2(y - self.pose[1], x - self.pose[0]) - self.pose[2])

    def walk_to(self, x: float, y: float, yaw: float, stop_short: float = 0.25, deadline_ms: int | None = None) -> None:
        """Walk to within `stop_short` of (x, y) in legs, looking between them, ending with the heading `yaw`.

        The stand-in turns slowly and badly in place, and strafes well, so the robot does not turn to face where it
        goes:
        each leg commands the direction to the target in the robot's frame (forward, diagonal, sideways or backward,
        whichever it is) and a yaw rate that works toward the final heading as it goes."""
        while True:
            self.check()
            if deadline_ms is not None and self.now_ms() > deadline_ms:
                raise BackendFailure("S504", "the walk did not reach the approach point in time")
            dx, dy = x - self.pose[0], y - self.pose[1]
            dist = math.hypot(dx, dy)
            if dist <= stop_short:
                return
            heading = self.pose[2]
            phi = site_mod.wrap(math.atan2(dy, dx) - heading)  # the direction to go, in the robot's frame
            scale = min(WALK_SPEED, 0.3 / max(abs(math.sin(phi)), 1e-6))  # sideways speed is at most 0.3
            vx, vy = scale * math.cos(phi), scale * math.sin(phi)
            if vx < 0:
                vx = max(vx, -0.5)
            speed_made = math.hypot(WALK_GAIN * vx if vx > 0 else BACK_GAIN * vx, SIDE_GAIN * vy)
            seconds = float(np.clip((dist - stop_short) / max(speed_made, 0.1), 0.5, 3.4))
            err = site_mod.wrap(yaw - heading)
            wz = float(np.clip(1.5 * err, -0.2, 0.2)) if abs(err) > 0.04 else 0.0
            if wz and math.hypot(vx, vy) < 0.3:  # the policy needs a command of about 0.3 to move at all
                vx = 0.3
            for leg in walk_legs(vx, vy, wz, seconds):
                self.command(*leg)
            self.settle()
            self.look(frames=3)

    def move(self, station_name: str) -> dict:
        """Walk to the station, servo on its marker to within 5 cm and 0.1 rad, wait for the stance to settle."""
        station = self.stations[station_name]
        gx, gy, gyaw = station["pose"]
        self.events = []
        self._arm_stance("free")  # walking is the policy's (a no-op unless an observation left the pelvis welded)
        t0 = self.now_ms()
        deadline = t0 + int(MOVE_TIMEOUT_S * 1000)
        self.look(frames=3)
        direct = self._direct(station, deadline)
        if direct is not None:
            return direct
        ax, ay = gx - APPROACH_M * math.cos(gyaw), gy - APPROACH_M * math.sin(gyaw)
        sx, sy = gx - (APPROACH_M + STAGING_M) * math.cos(gyaw), gy - (APPROACH_M + STAGING_M) * math.sin(gyaw)
        scan_end = self.now_ms() + int(MARKER_TIMEOUT_S * 1000)  # one budget for all the searching, replans included
        marker = None
        for _ in range(4):
            # A station's marker stands behind its table: from the side the table hides it. Come in along the axis.
            if math.hypot(ax - self.pose[0], ay - self.pose[1]) > 0.4 + STAGING_M:
                self.walk_to(sx, sy, gyaw, stop_short=0.35, deadline_ms=deadline)
            if math.hypot(ax - self.pose[0], ay - self.pose[1]) > 0.4:
                self.walk_to(ax, ay, gyaw, deadline_ms=deadline)
            self.turn_to(gyaw, tolerance=0.1)  # face the station: its marker is then in the camera's view
            marker = self._find_marker(station, deadline, scan_end, (ax, ay))
            if marker is not None:
                break
        if marker is None:
            raise BackendFailure("S517", f"station marker {station['marker']['id']} was not seen after four approaches")
        # Line up at the approach point first (the table is 0.5 m away); the last walk is then straight.
        self._servo(station, marker, deadline, target=(ax, ay, gyaw), tol_m=0.08, tol_rad=0.07)
        result = self._servo(station, marker, deadline)
        self.station = station_name
        result["events"] = self.events
        result["heading"] = {  # what the IMU's heading got wrong between fixes; the creep the servo aimed against
            **self.tracker.summary(),
            "creep_rad_s": round(self.creep, 4),
            "creep_imu_reads": self.creep_reads,
        }
        return result

    def _direct(self, station: dict, deadline_ms: int) -> dict | None:
        """The servo alone, if the robot has arrived at a station before and stands within DIRECT_M and DIRECT_RAD of
        this one, and sees its marker now. None otherwise: then `move` walks the usual way."""
        if self.station is None:
            return None
        gx, gy, gyaw = station["pose"]
        if (
            math.hypot(gx - self.pose[0], gy - self.pose[1]) > DIRECT_M
            or abs(site_mod.wrap(gyaw - self.pose[2])) > DIRECT_RAD
        ):
            return None
        mid = station["marker"]["id"]
        reading = self.look(frames=5, ids={mid}).get(mid)
        if reading is None or reading.confidence < MARKER_MIN_CONF or reading.range_m > MARKER_MAX_RANGE_M:
            return None
        self.events.append({"t_ms": self.now_ms(), "event": "direct", "from": [round(float(v), 3) for v in self.pose]})
        result = self._servo(station, reading, deadline_ms)
        self.station = station["name"]
        result["direct"] = True
        result["events"] = self.events
        result["heading"] = {**self.tracker.summary(), "creep_rad_s": round(self.creep, 4),
                             "creep_imu_reads": self.creep_reads}
        return result

    def _find_marker(self, station: dict, deadline_ms: int, scan_end_ms: int, approach: tuple):
        """Look at the approach point for the station's marker, scanning by turning if it is not in view. Returns its
        reading; or None when the belief, corrected by another marker, puts the robot more than 0.35 m from the
        approach point (it was not where it thought: walk there again, a table may be hiding the marker from here). A
        belief that has moved only by the dead reckoning of the robot's own sidesteps is not corrected: it is not a
        reason to give up (a loaded robot's sidestep often takes no step, and four such "replans" were S517 in 47 s).
        S517 when the search budget is spent."""
        mid = station["marker"]["id"]
        mx, my = station["marker"]["pos"][:2]
        gx, gy, gyaw = station["pose"]
        # which side of the station's axis the marker stands on: a table that hides it from the axis's other side is
        # cleared by stepping that way (measured: the robot 0.2-0.4 m to the right of the shelf's axis did not see the
        # marker, and turning to look for it only lost the heading)
        toward = 1.0 if -(mx - gx) * math.sin(gyaw) + (my - gy) * math.cos(gyaw) > 0 else -1.0
        sidesteps = 0
        approached_again = False
        first = True
        while True:
            readings = self.look(frames=3 if first else 4)
            reading = readings.get(mid)
            if reading is not None and reading.confidence >= MARKER_MIN_CONF and reading.range_m <= MARKER_MAX_RANGE_M:
                return reading
            away = math.hypot(self.pose[0] - approach[0], self.pose[1] - approach[1])
            if away > 0.35 and self.last_look[1]:  # (a belief that only the robot's own steps moved is not a reason)
                self.events.append({"t_ms": self.now_ms(), "event": "replan", "from_approach_m": round(float(away), 3)})
                return None
            if self.now_ms() > scan_end_ms or self.now_ms() > deadline_ms:
                raise BackendFailure(
                    "S517",
                    f"station marker {mid} was not seen within {MARKER_TIMEOUT_S:.0f} s "
                    f"(scanned from the approach point; belief {[round(float(v), 2) for v in self.pose]})",
                )
            if sidesteps < SIDESTEPS:  # not seen: first step toward the marker's side, looking after each step
                sidesteps += 1
                vx, vy, seconds = _burst("lateral", toward * SIDESTEP_M)
                self.events.append({"t_ms": self.now_ms(), "event": "sidestep", "toward": toward, "n": sidesteps})
                self.command(vx, vy, 0.0, seconds)
                self.settle()
                first = False
                continue
            if not approached_again and away > APPROACH_AGAIN_M:
                # The sidesteps did not clear the table (a robot with a box in each hand often takes no sidestep): walk
                # to the approach point itself, on the station's axis, where the site keeps the plate in view, and look
                # from there before turning away to scan (acceptance runs 1 and 3, F2: stopped 0.35-0.42 m short and to
                # the side of it, the table in the way, no sidestep taken; the scan turned the robot round, S517).
                approached_again = True
                self.events.append(
                    {"t_ms": self.now_ms(), "event": "approach_again", "from_approach_m": round(float(away), 3)}
                )
                self.walk_to(approach[0], approach[1], gyaw, stop_short=APPROACH_AGAIN_STOP_M, deadline_ms=deadline_ms)
                self.turn_to(gyaw, tolerance=0.1)
                continue
            # still not seen: face where the belief says it is, then sweep round in steps (a station marker seen on the
            # way corrects the belief, and the next step aims again)
            aim = self._angle_to(mx, my) if first else SCAN_STEP_RAD
            first = False
            self.events.append({"t_ms": self.now_ms(), "event": "scan", "turn_rad": round(float(aim), 3)})
            if abs(aim) > 0.06:
                self.turn_by(aim)

    def _servo(
        self,
        station: dict,
        reading,
        deadline_ms: int,
        target: tuple | None = None,
        tol_m: float = SERVO_TOL_M,
        tol_rad: float = SERVO_TOL_RAD,
    ) -> dict:
        """Correct the last centimetres: look at the station marker, command the correction that the measured table
        (`RESPONSE`) says leaves the smallest error (position and heading together: a sideways burst turns the robot
        too, and a turn moves it sideways), settle, look again. Stop inside the servo tolerances, which are tighter than
        the confirm limits, with the heading aimed against its creep (CREEP_PRIOR, then what the waits showed). Errors
        beyond COARSE_M and COARSE_RAD are walked and turned the old way, by the burst sized to the error. `target`
        (default: the station's pose) lets the same loop bring the robot to the approach point first, with looser
        tolerances and no confirmation: the station's table is then 0.5 m away. A correction that has not moved the
        robot for three bursts (a leg against the table's corner, say) is left alone for a while."""
        gx, gy, gyaw = target or station["pose"]
        mid = station["marker"]["id"]
        end = min(deadline_ms, self.now_ms() + int(SERVO_TIMEOUT_S * 1000))
        iterations: list = []
        picks: list = []  # (correction, the error it was chosen for) of the last bursts
        banned: dict = {}  # correction -> iteration until which it is left alone
        unseen = 0  # corrections in a row after which the station's marker was not seen
        best, best_look = float("inf"), 0  # the smallest error (in tolerances) so far, and at which look
        while True:
            self.check()
            self._note_creep_imu()
            dx, dy = gx - self.pose[0], gy - self.pose[1]
            ex = dx * math.cos(self.pose[2]) + dy * math.sin(self.pose[2])
            ey = -dx * math.sin(self.pose[2]) + dy * math.cos(self.pose[2])
            eyaw = site_mod.wrap(gyaw - self.pose[2])
            iterations.append([round(float(ex), 3), round(float(ey), 3), round(float(eyaw), 3)])
            # (the approach point is aimed against the creep too: the final servo starts after a settle and a look, 3-4
            # s in which the heading creeps 0.08 rad at 0.025 rad/s)
            creep = self.creep
            # the heading the stance will have when it is confirmed: it creeps to the left while the robot waits
            eyaw_confirmed = eyaw - creep * CONFIRM_DWELL_S
            score = math.hypot(ex, ey) / tol_m + abs(eyaw_confirmed) / tol_rad
            if score < best - 0.1:
                best, best_look = score, len(iterations)
            stagnant = len(iterations) - best_look >= STAGNANT_LOOKS
            band_m, band_rad = servo_band(target is not None, stagnant, tol_m, tol_rad)
            if math.hypot(ex, ey) <= band_m and abs(eyaw_confirmed) <= band_rad:
                if target is not None:
                    self.settle()
                    return {"iterations": iterations, "settled": True}
                before = self.fix
                if self.settle() and self._confirm_pose(station):
                    self._note_creep(before)
                    return {
                        "iterations": iterations,
                        "error_m": round(math.hypot(ex, ey), 4),
                        "error_rad": round(abs(eyaw), 4),
                        "settled": True,
                    }
                self._note_creep(before)
                continue
            if self.now_ms() > end:
                raise BackendFailure(
                    "S504",
                    f"did not converge on station {station['name']} in time (errors forward, left, "
                    f"yaw, last {len(iterations[-14:])} looks: {iterations[-14:]})",
                )
            if len(picks) >= 3 and len({k for k, _ in picks[-3:]}) == 1:
                first, last = picks[-3][1], picks[-1][1]
                if max(abs(first[0] - last[0]), abs(first[1] - last[1])) < 0.01 and abs(first[2] - last[2]) < 0.02:
                    banned[picks[-1][0]] = len(iterations) + 3  # three alike, and the error has not moved
                    named = picks[-1][0][0] if picks[-1][0] else "wait"
                    self.events.append({"t_ms": self.now_ms(), "event": "blocked", "burst": named})
                    picks.clear()
            error = (ex, ey, eyaw)
            seen_before, correction = self.last_look, None
            if math.hypot(ex, ey) > COARSE_M or abs(eyaw) > COARSE_RAD:
                self._coarse(error)
            else:
                skip = {k for k, until in banned.items() if until > len(iterations)}
                correction, cost = plan_correction(error, tol_m, tol_rad, creep, skip, self.held is not None, self.seen)
                picks.append((correction, error))
                pose_before = list(self.pose)
                for vx, vy, wz, seconds in correction:
                    self.command(vx, vy, wz, seconds)
                self._expect(pose_before, correction)  # if the marker is not seen next, the belief is what was measured
            self.settle()
            readings = self.look(frames=4, ids={mid})
            if correction is not None and seen_before[1] and self.last_look[1]:
                self._learn(correction, pose_before, self.last_look[0] - seen_before[0])
            # the marker is not seen: a table hides it from a robot that has drifted off the station's axis, and facing
            # where it should be only costs the heading. Look again, then go on from the belief (the corrections
            # return the robot toward the axis); only a marker not seen for several corrections is turned to face.
            for _ in range(2):
                if mid in readings and readings[mid].confidence >= MARKER_MIN_CONF:
                    break
                self.events.append({"t_ms": self.now_ms(), "event": "unseen"})
                readings = self.look(frames=5, ids={mid})
            if mid in readings and readings[mid].confidence >= MARKER_MIN_CONF:
                unseen = 0
            else:
                unseen += 1
                for _ in range(3 if unseen > 3 else 0):  # still nothing: face where it should be, and look again
                    aim = self._angle_to(*station["marker"]["pos"][:2])
                    self.events.append({"t_ms": self.now_ms(), "event": "reacquire", "turn_rad": round(float(aim), 3)})
                    if abs(aim) > 0.06:
                        self.turn_by(aim)
                    readings = self.look(frames=4, ids={mid})
                    if mid in readings and readings[mid].confidence >= MARKER_MIN_CONF:
                        unseen = 0
                        break

    def _expect(self, pose_before: list, correction) -> None:
        """The belief after a correction, from what the table says it does (the commands' own dead reckoning uses gains
        that are right on average for a walk, and far off for a step to the side with a turn)."""
        forward, left, _ = expected(correction, self.held is not None, self.seen)  # (the heading is the IMU's)
        x, y, yaw = pose_before
        self._xy = [
            x + forward * math.cos(yaw) - left * math.sin(yaw),
            y + forward * math.sin(yaw) + left * math.cos(yaw),
        ]

    def _coarse(self, error: tuple) -> None:
        """An error too large for the measured corrections: the furthest-out axis, by the burst sized to it (a turn by
        the shuffle), with the heading's own correction added to the walk as a yaw rate."""
        ex, ey, eyaw = error
        scores = {"yaw": abs(eyaw) / COARSE_RAD, "forward": abs(ex) / COARSE_M, "lateral": abs(ey) / COARSE_M}
        axis = max(scores, key=scores.get)
        bias = 0.2 * float(np.sign(eyaw)) if abs(eyaw) > 0.04 else 0.0
        if axis == "yaw":
            self.turn_by(float(np.clip(0.9 * eyaw, -0.5, 0.5)))
        elif axis == "forward":
            vx, vy, seconds = _burst("forward", ex)
            self.command(vx, vy, bias, seconds)
        else:
            vx, vy, seconds = _burst("lateral", ey)
            self.command(vx, vy, bias, seconds)

    def _learn(self, correction, pose_before: list, elapsed_ms: int) -> None:
        """Fold in what a correction did, from the belief before it and the belief after it (both were seen at the
        marker): to what the table says it does, and to the creep. Observations far from the table's are left out
        (a reading that jumped is not the robot's step)."""
        x0, y0, yaw0 = pose_before
        dx, dy = self.pose[0] - x0, self.pose[1] - y0
        waited = elapsed_ms / 1000 - sum(burst[3] for burst in correction)
        saw = (
            dx * math.cos(yaw0) + dy * math.sin(yaw0),
            -dx * math.sin(yaw0) + dy * math.cos(yaw0),
            site_mod.wrap(self.pose[2] - yaw0) - self.creep * waited,
        )
        carrying = self.held is not None
        row = expected(correction, carrying)
        if all(abs(saw[i] - row[i]) <= limit for i, limit in enumerate((0.15, 0.15, 0.2))):
            record = self.seen.setdefault((carrying, correction), [0, 0.0, 0.0, 0.0])
            record[0] += 1
            for i in range(3):
                record[1 + i] += saw[i]
        if abs(row[2]) <= 0.06 and waited >= 1.5:  # the creep, from what the heading did beyond what the table says
            rate = (site_mod.wrap(self.pose[2] - yaw0) - row[2]) / waited
            self.creep = float(np.clip(0.7 * self.creep + 0.3 * rate, -0.03, 0.05))

    def _note_creep_imu(self) -> None:
        """The heading's creep, read from the IMU: how fast its yaw turns over a quiet second, the gyro's bias (learned
        from the fixes, heading.py) taken out. A standing robot that no command has moved for two seconds turns at the
        creep and at nothing else, and the IMU has it at once; the markers show it only between two looks with nothing
        commanded in between, which a servo that keeps correcting does not have (so it aimed against 0.007 rad/s
        while the stance, under a torque, was turning 0.025: docs/evidence/g1-phase4-prereqs.md). Blended into `creep`
        as the other estimates are, and left alone when the window is not quiet or has too few samples."""
        if self.tracker is None or self.tracker.held or self.robot is None:
            return
        now = self.now_ms()
        start = now - CREEP_WINDOW_MS
        if start < self.moved_ms + CREEP_QUIET_MS:
            return
        samples = self.robot.samples_since(start)
        if len(samples) < 0.8 * CREEP_WINDOW_MS / 4:
            return
        t = np.array([x.tick for x in samples]) / 1000.0
        yaw = np.unwrap([yaw_of(x.quat) for x in samples])
        rate = float(np.polyfit(t - t[0], yaw, 1)[0]) - self.tracker.bias
        self.creep = float(np.clip(0.7 * self.creep + 0.3 * np.clip(rate, -0.02, 0.05), -0.03, 0.05))
        self.creep_reads += 1

    def _note_creep(self, before) -> None:
        """The heading's creep, as the robot saw it: how far the marker's heading moved between two looks with nothing
        commanded in between (the last servo look and the confirming one)."""
        after = self.fix
        if before is None or after is None or after[0] <= before[0] or before[0] <= self.moved_ms:
            return
        dt = (after[0] - before[0]) / 1000
        if 0.8 <= dt <= 8.0:
            rate = site_mod.wrap(after[1] - before[1]) / dt
            self.creep = float(np.clip(0.5 * self.creep + 0.5 * rate, -0.03, 0.05))
            self.events.append({"t_ms": self.now_ms(), "event": "creep", "rad_s": round(rate, 4)})

    def _confirm_pose(self, station: dict) -> bool:
        """After the stance has settled: look once more, fix the pose from the station's marker, and be within the
        tolerance again. A reading the localization rejected (its vertical not the IMU's: the wrong one of the two poses
        a flat marker allows) fixes nothing, and the belief it leaves is the dead reckoning: no confirmation from it
        (acceptance run 3: an arrival confirmed 3.5 cm off while the robot stood 0.45 m away)."""
        readings = self.look(frames=5, ids={station["marker"]["id"]})
        gx, gy, gyaw = station["pose"]
        if station["marker"]["id"] not in readings or not self.last_look[1]:
            return False
        return (
            math.hypot(gx - self.pose[0], gy - self.pose[1]) <= STATION_TOL_M
            and abs(site_mod.wrap(gyaw - self.pose[2])) <= STATION_TOL_RAD
        )

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
            self.robot.set_arm(
                q0 + _minjerk(a) * (q_to - q0), self.arm_kp, self.arm_kd, w0 + (weight_to - w0) * _minjerk(a)
            )
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
                raise BackendFailure(
                    "S503",
                    f"arm joint {worst} is {err:.3f} rad from its target after {label or 'a motion'} "
                    f"(tolerance {ARM_TRACK_TOL_RAD})",
                )
            time.sleep(0.02)
        self.events.append(
            {
                "t_ms": self.now_ms(),
                "event": "arm",
                "label": label,
                "seconds": round(seconds, 2),
                "tracking_rad": round(err, 4),
                "took_ms": self.now_ms() - t0,
            }
        )

    def _hand_to(self, side: str, q: list[float], label: str) -> None:
        self.robot.hand_command(side, q, seconds=2.0, check=self.check)
        self.sleep(0.5)
        err = float(np.max(np.abs(self.robot.hand_q[side] - np.array(q))))
        self.events.append({"t_ms": self.now_ms(), "event": "hand", "label": label, "error_rad": round(err, 4)})
        if err > HAND_TOL_RAD:
            raise BackendFailure(
                "S505", f"the {side} hand did not reach its {label} pose: {err:.3f} rad off (tolerance {HAND_TOL_RAD})"
            )

    def _solve(self, side: str, target, R, seeds, waist: float):
        for seed in seeds:
            q, info = self.kin.solve(
                side, target, R, seed, posture=self._ready_pose(side), rot_weight=0.5, iters=250, waist_max=waist
            )
            if q is not None:
                return q, info
        return None, None

    def _plan_reach(self, side: str, target_for, extra: tuple = (), degs=FINGERS_DOWN_DEG, accept=None) -> dict:
        """Inverse kinematics for a reach: `target_for(R)` gives the pinch point (pelvis frame) for hand orientation R.
        Solves the pre-grasp (behind and above it), the grasp, any `extra` offsets (a lift), and two **raised** poses
        (the hand where it is when clear of the hip lifted above the pre-grasp's height, and that height straight above
        the pre-grasp), so that the hand comes in over a table and not through its edge, each in the same hand
        orientation and joined by small joint moves. Tries the hand's pitch and the waist's lean in order; with
        `accept(plan) -> (ok, info)` a combination is also tried against that (the path clearance); S502 if no
        combination works, before any motion."""
        rng = np.random.default_rng(7)
        joints = ARM_JOINTS[side]
        ready = self._ready_pose(side)
        shoulder = np.array([0.0, HAND_REST_Y[side], 0.29])
        p0 = np.asarray(target_for(_frame(math.radians(degs[0]))), float)
        if np.linalg.norm(p0 - shoulder) > 0.62:  # beyond the arm's length with the waist leaning: no search
            raise BackendFailure(
                "S502",
                f"no inverse-kinematics solution: the target is {np.linalg.norm(p0 - shoulder):.2f} "
                "m from the shoulder, further than the arm reaches",
            )
        rejected = None
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
                clear_p = self.kin.grasp_frame(side, self._clear_pose(side))[1]
                outward = -1.0 if side == "right" else 1.0
                pairs = []  # (above, raised): as high as the arm goes, and as far out beside the body as needed
                for height in RAISED_ABOVE_PRE_M:
                    q_above, _ = self._solve(side, pre + np.array([0, 0, height]), R, [q_pre], waist)
                    if q_above is None:
                        continue
                    for out in RAISED_OUT_M:
                        raised = np.array([clear_p[0], clear_p[1] + outward * out, pre[2] + height])
                        q_raised, _ = self._solve(side, raised, R, [self._clear_pose(side), q_above, q_pre], waist)
                        if q_raised is not None:
                            pairs.append((q_above, q_raised))
                for q_above, q_raised in pairs:
                    plan = {
                        "above": q_above,
                        "raised": q_raised,
                        "pre": q_pre,
                        "grasp": q_grasp,
                        "fingers_down_deg": deg,
                        "waist_rad": float(q_grasp[WAIST_PITCH]),
                        "position_error_m": round(info["position_error_m"], 4),
                        "pinch": p,
                    }
                    ok = max(abs(q_grasp[joints] - q_pre[joints])) < 1.2
                    for name, offset in extra:
                        q, _ = self._solve(side, p + np.array(offset), R, [q_grasp], waist)
                        if q is None:
                            ok = False
                            break
                        plan[name] = q
                    if not ok:
                        continue
                    if accept is not None:
                        ok, verdict = accept(plan)
                        if not ok:
                            rejected = verdict
                            continue
                        plan["clearance"] = verdict
                    return plan
        if rejected is not None:
            raise BackendFailure(
                "S502",
                "no clear path for the arm: every reach that has an inverse-kinematics solution comes within the "
                f"margins of a declared table or obstacle, or leaves no carry pose that clears the object ({rejected})",
            )
        raise BackendFailure(
            "S502",
            "no inverse-kinematics solution for the reach (hand pitch 0-45 degrees, waist lean up "
            f"to {WAIST_LEAN_RAD[-1]} rad): the target {np.round(p0, 3).tolist()} (pelvis frame) is "
            "out of reach",
        )

    def _grasp_accept(self, side: str, centre, half, walking: np.ndarray):
        """The test a reach must pass to be taken for a grasp: there is a carry pose for the object as this grasp would
        hold it, and every arm path clears the declared tables and obstacles (`_path_clearance`): carry to clear of the
        hip, raised beside the body, the pre-grasp, the grasp, the lift (contact legs: the hand is at the object on its
        table), and back to the carry pose. `walking` is where the arms start."""

        def accept(plan: dict):
            clear = self._clear_pose(side)
            R_hand, p_hand = self.kin.grasp_frame(side, plan["grasp"])
            offset, rotation = R_hand.T @ (centre - p_hand), R_hand.T  # the object's place in the hand, as planned
            carry = self._plan_carry(side, half, offset, rotation)
            if carry is None:
                return False, "no carry pose clears the object from every link by 2 cm"
            plan["carry"] = carry
            w = [
                ("carry", walking),
                ("clear of the hip", clear),
                ("raised", plan["raised"]),
                ("above", plan["above"]),
                ("pre-grasp", plan["pre"]),
                ("grasp", plan["grasp"]),
                ("lift", plan["lift"]),
                ("carry", carry),
            ]
            held = {"held": True, "closed": True}
            legs = [
                self._leg(w[0], w[1]),
                self._leg(w[1], w[2]),
                self._leg(w[2], w[3]),
                self._leg(w[3], w[4]),
                self._leg(w[4], w[5], contact=True),
                self._leg(w[5], w[6], contact=True, **held),
                self._leg(w[6], w[4], **held),
                self._leg(w[4], w[3], **held),
                self._leg(w[3], w[2], **held),
                self._leg(w[2], w[1], **held),
                self._leg(w[1], w[7], **held),
            ]
            info = self._path_clearance(side, legs, (half, offset, rotation))
            return info["clear"], info

        return accept

    def _place_accept(self, side: str, half, start: np.ndarray):
        """The same for a place: the object is carried to the clear pose, raised, brought over the table, lowered
        (contact), released, and the open hand backs out the same way; `start` is the carry pose as measured."""
        held = self.held
        geometry = (half, held["offset_in_hand"], held["rotation_in_hand"])

        def accept(plan: dict):
            clear = self._clear_pose(side)
            w = [
                ("carry", start),
                ("clear of the hip", clear),
                ("raised", plan["raised"]),
                ("above", plan["above"]),
                ("pre-place", plan["pre"]),
                ("place", plan["grasp"]),
                ("carry", self.q_carry),
            ]
            holding = {"held": True, "closed": True}
            legs = [
                self._leg(w[0], w[1], **holding),
                self._leg(w[1], w[2], **holding),
                self._leg(w[2], w[3], **holding),
                self._leg(w[3], w[4], **holding),
                self._leg(w[4], w[5], contact=True, **holding),
                self._leg(w[5], w[4], contact=True),  # the object is released: the open hand backs away
                self._leg(w[4], w[3]),
                self._leg(w[3], w[2]),
                self._leg(w[2], w[1]),
                self._leg(w[1], w[6]),
            ]
            info = self._path_clearance(side, legs, geometry)
            return info["clear"], info

        return accept

    def _plan_carry(self, side: str, half, offset_in_hand, rotation_in_hand) -> np.ndarray | None:
        """The carry pose for a held object: the walking controller's own arm pose moved only as far as it takes for the
        object, welded as it is held, to clear **every link of the robot** (the holding hand's own palm and fingers
        excepted, they touch it by design) by the first of CARRY_PLAN_M that exists, from the robot's own model
        (MuJoCo distances). The default pose has the object against the hip (0.1-0.6 cm). Alternates a pull of the arm's
        joints back toward the default with a gradient step that restores the clearance, and keeps the closest feasible
        pose. None if there is none."""
        self.clr.set_held(side, half, offset_in_hand, rotation_in_hand)
        joints = ARM_JOINTS[side]
        lo, hi = self.kin.limits[joints, 0], self.kin.limits[joints, 1]
        q0 = self.q_carry.copy()

        def clearance(q) -> float:
            return self.clr.held_to_links(side, q)[0]

        def gradient(q) -> np.ndarray:
            base, g = clearance(q), np.zeros(len(joints))
            for i, j in enumerate(joints):
                q2 = q.copy()
                q2[j] += 1e-3
                g[i] = (clearance(q2) - base) / 1e-3
            return g

        rng = np.random.default_rng(5)

        def climb(start: np.ndarray, target: float):
            """One search from `start`: (distance from the walking pose, pose) of the closest feasible, or None."""
            q, best = start.copy(), None
            for outer in range(120):
                if best is None and outer >= 20:
                    break  # not found by now: out of reach from here
                for _ in range(40):  # restore the clearance
                    if clearance(q) >= target:
                        break
                    g = gradient(q)
                    if np.linalg.norm(g) < 1e-9:
                        break
                    q[joints] = np.clip(q[joints] + 0.02 * g / np.linalg.norm(g), lo, hi)
                if clearance(q) >= target:
                    dev = float(np.linalg.norm(q[joints] - q0[joints]))
                    if best is None or dev < best[0] - 1e-4:
                        best = (dev, q.copy())
                    elif dev >= best[0] - 1e-4:
                        break
                q[joints] = q[joints] - 0.25 * (q[joints] - q0[joints])  # toward the walking controller's pose
            return best

        best = None
        for target in CARRY_PLAN_M:
            for attempt in range(6):  # the first from the walking pose, then from small random steps away from it
                start = q0.copy()
                if attempt:
                    start[joints] = np.clip(q0[joints] + rng.uniform(-0.12, 0.12, len(joints)), lo, hi)
                best = climb(start, target)
                if best is not None:
                    break
            if best is not None:
                break
        for attempt in range(8):  # nothing near the walking pose: look further away (the clearance rule comes first)
            if best is not None:
                break
            start = q0.copy()
            start[joints] = np.clip(q0[joints] + rng.uniform(-0.35, 0.35, len(joints)), lo, hi)
            best = climb(start, CARRY_PLAN_M[-1])
        self.clr.clear_held(side)
        return None if best is None else best[1]

    def _path_clearance(self, side: str, legs: list[dict], held=None) -> dict:
        """Whether the arm (and the held object) stays clear of the site's declared tables and obstacles along `legs`,
        from the robot's own geometry (`Clearance`) and the belief pose at this station; every leg is the straight line
        in joint space that `_arm_move` follows. A leg is `{"from", "q0", "to", "q1", "contact", "held", "closed"}`;
        `held` = (half_size, offset in hand, rotation in hand) of the object, for the legs that carry it. Margins:
        CLEARANCE_XY_M sideways and CLEARANCE_Z_M vertical, except on a `contact` leg (the hand is meant to be at the
        object on a table): the hand has no vertical margin and may not touch the table, and the object may overlap its
        table by CONTACT_OVERLAP_M. `clear` is true only if every sampled distance is over its limit."""
        boxes = site_mod.declared_boxes(self.site)
        x, y, yaw = self.pose
        out = {
            "clear": False,
            "boxes": [b["name"] for b in boxes],
            "pelvis_in_map": [round(float(v), 3) for v in (x, y, yaw)],
            "margins_m": {"sideways": CLEARANCE_XY_M, "vertical": CLEARANCE_Z_M, "contact_overlap": CONTACT_OVERLAP_M},
            "legs": [f"{lg['from']}->{lg['to']}" for lg in legs],
            "closest": None,
        }
        if not boxes:
            out["clear"] = True
            return out
        if held is not None:
            self.clr.set_held(side, *held)
        closed = np.array(HAND_CLOSE[side])
        worst = None
        for lg in legs:
            self.clr.set_boxes(boxes, (x, y, yaw), CLEARANCE_XY_M, 0.0 if lg["contact"] else CLEARANCE_Z_M)
            for a in np.linspace(0.0, 1.0, CLEARANCE_SAMPLES):
                q = lg["q0"] + a * (lg["q1"] - lg["q0"])
                found = self.clr.to_boxes(side, q, held=lg["held"], hand_q=closed if lg["closed"] else None)
                for group, (dist, link, box) in found.items():
                    limit = -CONTACT_OVERLAP_M if (lg["contact"] and group in ("hand", "held")) else 0.0
                    over = dist - limit
                    if worst is None or over < worst["over_limit_m"]:
                        worst = {
                            "over_limit_m": round(float(over), 4),
                            "clearance_m": round(float(dist), 4),
                            "group": group,
                            "link": link,
                            "box": box,
                            "leg": f"{lg['from']}->{lg['to']}",
                            "at": round(float(a), 2),
                        }
        if held is not None:
            self.clr.clear_held(side)
        out["closest"], out["clear"] = worst, bool(worst["over_limit_m"] > 0)
        return out

    def _jitter(self, q: np.ndarray, joints, rng) -> np.ndarray:
        out = q.copy()
        out[joints] = np.clip(
            q[joints] + rng.uniform(-0.8, 0.8, len(joints)), self.kin.limits[joints, 0], self.kin.limits[joints, 1]
        )
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
            if not self.last_look[1]:  # seen, but the localization rejected it: the belief is not from it (F1)
                raise BackendFailure("S517", f"station marker {mid} gave no pose fix before arm work")
            err = math.hypot(gx - self.pose[0], gy - self.pose[1])
            info = {
                "error_m": round(err, 4),
                "error_rad": round(abs(site_mod.wrap(gyaw - self.pose[2])), 4),
                "reservoed": attempt > 0,
            }
            if err <= REPOSITION_M and abs(site_mod.wrap(gyaw - self.pose[2])) <= REPOSITION_RAD:
                return info  # the reach is planned from what the camera sees now, so a few centimetres do not matter
            self._servo(station, reading, self.now_ms() + int(SERVO_TIMEOUT_S * 1000))
        raise BackendFailure("S504", f"could not return to station {self.station} before arm work")

    def _arm_stance(self, model: str, settle: bool = True) -> str:
        """Ask the stand-in for the balance support the site names for arm work (0046 U4: the free stance moved the
        robot about 12 cm when the arm reached; `supported` welds the pelvis, which stands in for balance the robot's
        own controller provides). Waits for the stand-in to report it holding. Returns the model now in force. `settle`
        False skips the wait for a quiet stance first: the free stance drifts in heading by about 0.01 rad a second, so
        the observation motion, which follows a `move` that has just settled, welds at once (measured: 0.03 rad of
        heading between the move's last look and the goal snapshot otherwise)."""
        if self.stance_now == model:
            return model
        if settle:
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
        self.stance_now = model
        self._hold_heading(model == "supported")
        self.events.append({"t_ms": self.now_ms(), "event": "stance", "model": model})
        return model

    def grasp(self, obj_name: str) -> dict:
        """Reach, close the hand, check the object's marker is at the hand, weld it, lift to carry (0046 U5, U7)."""
        self.events = []
        obj = site_mod.obj(self.site, obj_name)
        face, mid = obj["marker"]["face"], obj["marker"]["id"]
        details = {"object": obj_name, "station_check": self._reposition()}
        if self.held is not None:
            raise BackendFailure("S506", f"the robot already holds {self.held['object']}")
        # The pelvis is welded before the object is measured and the reach planned, not after: the free stance drifts
        # in heading (about 0.01 rad a second) and a plan can take a minute on a loaded host (measured: 0.33 rad and
        # 0.3 m of drift, the hand closed on air: S506), so the plan is made for the pose the arm will really work from.
        details["arm_work_stance"] = self._arm_stance(self.site["robot"]["arm_stance"])
        details["arm_work_ms"] = [self.now_ms(), None]  # from here the stance is the arm-work one (the checks judge it)
        self.events.append({"t_ms": self.now_ms(), "event": "arm_work"})
        readings = self.look(frames=10)
        seen = readings.get(mid)
        if seen is None or seen.confidence < 0.6:
            raise BackendFailure(
                "S506",
                f"the marker of {obj_name} (id {mid}) was not seen at the station: the object is "
                "absent or has been moved (checked before the hand moves)",
            )
        axis = int(np.argmax(np.abs(site_mod.FACE_NORMALS[face])))
        centre = object_centre(seen, obj["half_size"][axis])
        side = obj["grasp"]["hand"] if obj["grasp"]["hand"] != "nearest" else ("right" if centre[1] < 0 else "left")
        details.update(
            hand=side,
            object_in_pelvis=[round(float(v), 4) for v in centre],
            marker_confidence=seen.confidence,
            marker_range_m=round(seen.range_m, 3),
        )
        half = obj["half_size"]
        accept = self._grasp_accept(side, centre, half, self.robot.low_q.copy())
        plan = self._plan_reach(
            side, lambda R: centre + np.array([0, 0, GRASP_RISE]), extra=(("lift", (0.0, 0.0, 0.10)),), accept=accept
        )  # S502 here, before any motion
        details["reach"] = {k: plan[k] for k in ("fingers_down_deg", "waist_rad", "position_error_m")}
        details["clearance"] = plan["clearance"]
        self._arm_begin()
        self._arm_move(self._clear_pose(side), 1.6, label="clear of the hip")
        self._arm_move(plan["raised"], 1.6, label="raised")
        self._arm_move(plan["above"], 1.6, label="above the object")
        self._arm_move(plan["pre"], 1.6, label="pre-grasp")
        self._arm_move(plan["grasp"], 1.6, label="grasp")
        self._hand_to(side, HAND_CLOSE[side], "closed")
        # the object must be seen at the hand, from the measured joints, before it is welded
        held = self.look(frames=5, ids={mid}).get(mid)
        _, hand_p = self.kin.grasp_frame(side, self.robot.low_q)
        if held is None or float(np.linalg.norm(held.position - hand_p)) > HELD_RADIUS_M:
            raise BackendFailure(
                "S506",
                f"the marker of {obj_name} is not seen at the {side} hand "
                f"({'not seen' if held is None else f'{np.linalg.norm(held.position - hand_p):.3f} m from the hand'})",
            )
        details["marker_at_hand_m"] = round(float(np.linalg.norm(held.position - hand_p)), 4)
        hand_R, _ = self.kin.grasp_frame(side, self.robot.low_q)
        offset_hand = hand_R.T @ (object_centre(held, obj["half_size"][axis]) - hand_p)  # where it sits in the hand
        # the carry pose for the object as it is really held (planned before for the object as the grasp would hold it)
        carry = self._plan_carry(side, half, offset_hand, hand_R.T)
        if carry is None:
            raise BackendFailure("S502", "no carry pose clears the held object from every link of the robot by 2 cm")
        code, attached = self.sim.Attach(obj_name, side)
        if code != 0:
            raise BackendFailure("S506", f"the simulator could not attach {obj_name} to the {side} hand (code {code})")
        self.held = {
            "object": obj_name,
            "hand": side,
            "offset_in_hand": offset_hand,
            "fingers_down_deg": plan["fingers_down_deg"],
            "rotation_in_hand": hand_R.T,  # the object lies flat (the world's axes) when it is grasped
            "carry": carry,  # where the arm holds it while the robot walks: clear of every link by CARRY_CLEARANCE_M
            "lift_waypoint": plan["lift"],  # where the object was lifted: the view pose (_snapshot_showing)
        }
        details["object_in_hand_m"] = [round(float(v), 4) for v in offset_hand]
        self.events.append(
            {
                "t_ms": self.now_ms(),
                "event": "attached",
                "relative_position": attached.get("relative_position"),
            }
        )
        self._arm_move(plan["lift"], 1.6, label="lift")
        self._arm_move(plan["pre"], 1.6, label="retreat")
        self._arm_move(plan["above"], 1.6, label="above the object")
        self._arm_move(plan["raised"], 1.6, label="raised")
        self._arm_move(self._clear_pose(side), 1.6, label="clear of the hip")
        self._arm_move(carry, 1.6, label="carry")
        d, link = self.clr.held_to_links_with(side, self.robot.low_q, half, offset_hand, hand_R.T)
        details["carry_pose"] = {
            "object_to_nearest_link_m": round(float(d), 4),
            "link": link,
            "arm_deviation_rad": round(
                float(np.linalg.norm(carry[ARM_JOINTS[side]] - self.q_carry[ARM_JOINTS[side]])), 3
            ),
        }
        self._arm_stance("free")  # walking is the policy's: back to the free stance
        details["arm_work_ms"][1] = self.now_ms()
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
        details["arm_work_stance"] = self._arm_stance(self.site["robot"]["arm_stance"])  # welded first, as in grasp
        details["arm_work_ms"] = [self.now_ms(), None]
        self.events.append({"t_ms": self.now_ms(), "event": "arm_work"})
        # the region's centre, in the pelvis frame, through the station marker seen now
        readings = self.look(frames=10, ids={station["marker"]["id"]})
        reading = readings.get(station["marker"]["id"])
        if reading is None or reading.confidence < MARKER_MIN_CONF:
            raise BackendFailure("S517", f"station marker {station['marker']['id']} was not seen before placing")
        lo, hi = np.array(loc["region"]["min"], float), np.array(loc["region"]["max"], float)
        target_marker = np.array(
            [(lo[0] + hi[0]) / 2, lo[1] + obj["half_size"][2] + PLACE_ABOVE_M, (lo[2] + hi[2]) / 2]
        )
        # the marker frame is (x right, y up, z out) with a vertical panel: y is the height above the marker's centre
        half = obj["half_size"]
        accept = self._place_accept(side, half, self.robot.low_q.copy())
        # across the region (the marker frame's x), the way that brings the target toward the pelvis's centre line
        toward = -1.0 if (reading.rotation @ np.array([1.0, 0.0, 0.0]))[1] * (
            reading.rotation @ target_marker + reading.position
        )[1] > 0 else 1.0
        room = (hi[0] - lo[0]) / 2 - PLACE_INSIDE_M
        shifts = [0.0] + [d for d in PLACE_SHIFTS_M if d <= room + 1e-9]
        for k, shift in enumerate(shifts):
            centre = reading.rotation @ (target_marker + np.array([toward * shift, 0.0, 0.0])) + reading.position
            try:
                plan = self._plan_reach(
                    side,
                    lambda R, c=centre: c - R @ self.held["offset_in_hand"],
                    extra=(("above", (0.0, 0.0, 0.10)),),
                    degs=(self.held["fingers_down_deg"],),
                    accept=accept,
                )
            except BackendFailure as exc:
                if exc.code != "S502" or k == len(shifts) - 1:
                    raise
                self.events.append({"t_ms": self.now_ms(), "event": "place_shift", "shift_m": shifts[k + 1],
                                    "because": str(exc)[:120]})
                continue
            break
        details["target_in_pelvis"] = [round(float(v), 4) for v in centre]
        details["target_shift_m"] = round(float(shift), 3)
        details["reach"] = {k: plan[k] for k in ("fingers_down_deg", "waist_rad", "position_error_m")}
        details["clearance"] = plan["clearance"]
        self._arm_begin()
        self._arm_move(self._clear_pose(side), 1.0, label="clear of the hip")
        self._arm_move(plan["raised"], 1.6, label="raised")
        self._arm_move(plan["above"], 1.6, label="above the target")
        self._arm_move(plan["pre"], 1.6, label="pre-place")
        self._arm_move(plan["grasp"], 1.6, label="place")
        self._hand_to(side, [0.0] * 7, "open")
        code, _ = self.sim.Detach(obj_name)
        if code != 0:
            raise BackendFailure("S506", f"the simulator could not detach {obj_name} (code {code})")
        self.held = None
        self.events.append({"t_ms": self.now_ms(), "event": "detached"})
        self._arm_move(plan["pre"], 1.6, label="retreat")
        self._arm_move(plan["above"], 1.6, label="above the target")
        self._arm_move(plan["raised"], 1.6, label="raised")
        self._arm_move(self._clear_pose(side), 1.6, label="clear of the hip")
        self._arm_move(self.q_carry, 1.6, weight_to=0.0, label="carry")
        self.robot.stop_arm_sdk()
        self._arm_stance("free")
        details["arm_work_ms"][1] = self.now_ms()
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
        pelvis IMU's rate, the tilt and every joint's velocity are within tolerance, and the legs and waist have moved
        no more than CONFIRM_LEG_TRAVEL_RAD from the window's start to its end, within 3 s of the stop."""
        t0 = self.now_ms()
        result = {
            "confirmed": False,
            "timeout_ms": int(CONFIRM_TIMEOUT_S * 1000),
            "window_ms": CONFIRM_WINDOW_MS,
            "tolerances": {
                "gyro_rad_s": CONFIRM_GYRO,
                "joint_velocity_rad_s": CONFIRM_JOINT_VEL,
                "leg_travel_rad": CONFIRM_LEG_TRAVEL_RAD,
                "tilt_deg": round(math.degrees(CONFIRM_TILT_RAD), 1),
            },
            "t_stop_ms": t0,
        }
        while self.now_ms() - t0 <= CONFIRM_TIMEOUT_S * 1000:
            if (
                self.tilt_rad() > TILT_LIMIT_RAD
            ):  # it fell while the stop was being confirmed: nothing more is sent (S516)
                self.fallen = True
                self.robot.silence()
                raise BackendFailure("S516", "the robot fell while the stop was being confirmed")
            now = self.now_ms()
            s = self.robot.samples_since(now - CONFIRM_WINDOW_MS)
            if (
                s
                and s[0].tick >= t0
                and len(s) >= CONFIRM_WINDOW_MS / 4 * 0.6
                and s[-1].tick - s[0].tick >= CONFIRM_WINDOW_MS * 0.9
            ):
                gyro = max(float(np.linalg.norm(x.gyro)) for x in s)
                joints = max(float(np.max(np.abs(x.dq))) for x in s)
                travel = float(np.max(np.abs(s[-1].q[LEG_AND_WAIST] - s[0].q[LEG_AND_WAIST])))
                tilt = max(math.acos(max(-1.0, min(1.0, 1 - 2 * (x.quat[1] ** 2 + x.quat[2] ** 2)))) for x in s)
                result.update(
                    max_gyro_rad_s=round(gyro, 4),
                    max_joint_velocity_rad_s=round(joints, 4),
                    leg_travel_rad=round(travel, 4),
                    max_tilt_deg=round(math.degrees(tilt), 2),
                    samples=len(s),
                )
                if (
                    gyro <= CONFIRM_GYRO
                    and joints <= CONFIRM_JOINT_VEL
                    and travel <= CONFIRM_LEG_TRAVEL_RAD
                    and tilt <= CONFIRM_TILT_RAD
                ):
                    result.update(
                        confirmed=True, window_start_ms=s[0].tick, window_end_ms=s[-1].tick, after_ms=s[-1].tick - t0
                    )
                    return result
            time.sleep(0.02)
        result["after_ms"] = self.now_ms() - t0
        return result
