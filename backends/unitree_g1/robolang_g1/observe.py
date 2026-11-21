"""Condition verdicts from the robot's own observations (decision 0046 U8). No DDS, no simulator, no ground truth.

The shape is the ROS 2 backend's `observe.py` (`backends/ros2/robolang_ros2/observe.py`): a `Snapshot` of what the
sensors reported at one moment, a `Policy` (`@verify`: minimum confidence and maximum age), a three-valued `evaluate`
that follows the symbolic simulator's Kleene logic, and `core_outcome`. A reading that is missing, below
`min_confidence` or older than `max_age` makes its atom `unknown`, never `true`; **in every doubtful case the atom is
`unknown`**, so a task fails closed (S102, S302). What differs from the ROS 2 backend is what is observed:

* `object_at o l`: the marker of `o` is inside `l`'s region, **located through `l`'s station marker in the same
  snapshot** (the region is a box in that marker's frame, so no map and no robot pose are involved). An object seen at a
  closed hand is carried, so it is not at `l`.
* `holding r o`: the hand is closed (from `rt/dex3/*/state`) and the marker of `o` is within `HELD_RADIUS_M` of the
  hand's grasp point, which comes from the measured joints through the robot's own kinematic model.
* `robot_at r l`: the station marker of `l` is seen at the station's expected offset, within 5 cm and 0.1 rad. The
  robot's pose follows from that one marker (the marker's pose in the site, composed with its pose in the pelvis frame).
* `robot_ready r`: fresh `lowstate`, no motor error, upright from the IMU, and the walking service says it is standing
  or walking (`GetFsmId`).

Measurement uncertainty is handled with margins, never in favour of `true`: a value within `REGION_MARGIN_M` of a
region's boundary, or within the robot_at margins of its tolerance, is `unknown`. The margins are measured in
`docs/evidence/g1-phase3-observations.md`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from decimal import Decimal

import numpy as np

from robolang import ir
from robolang.bridge import NotExportable, core_time, flatten_atoms, triple_of
from robolang.core.signatures import SIM_HMAC, Key, sign_sim

from . import site as site_mod
from .perception import Reading, pelvis_pose_in_map

# RoboLang 1.3 section 4: the exporter's defaults without @verify.
DEFAULT_MIN_CONFIDENCE = Decimal("0.9")
DEFAULT_MAX_AGE_MS = 10_000

# robot_at (0046 U8): the station marker is seen at the station's expected offset within these.
ROBOT_AT_M, ROBOT_AT_RAD = 0.05, 0.1
# ... and the verdict is `true` only if the measured offset is at least this far inside, `false` only if at least this
# far outside, `unknown` in between: what the camera cannot tell from the tolerance's edge.
ROBOT_AT_MARGIN_M, ROBOT_AT_MARGIN_RAD = 0.005, 0.01
# object_at: the marker's position must be this far inside the region to be `true`, this far outside to be `false`.
REGION_MARGIN_M = 0.015
# holding: the object's marker is "at the hand" within this of the hand's grasp point (5-6 cm when held right).
HELD_RADIUS_M = 0.09
# holding: a hand is open or closed when its seven joints are within this of the open pose (all zero) or the closed one.
HAND_TOLERANCE_RAD = 0.25
HAND_CLOSED = {"right": [0.0, -0.6, -1.0, 1.0, 1.2, 1.0, 1.2], "left": [0.0, 0.6, 1.0, -1.0, -1.2, -1.0, -1.2]}
# robot_ready: the IMU's tilt from upright must be below this, lowstate must have arrived in the last second of wall
# time, and the walking service's state id must be one of these (500 is the SDK's `Start`: standing and walking).
READY_TILT_RAD = math.radians(15)
READY_LOWSTATE_AGE_S = 1.0
FSM_STANDING_OR_WALKING = (500,)

SIM_PARTICIPANT = "rl:sim:g1-mujoco"
SIM_KEY_REF = SIM_PARTICIPANT + "#sim-1"
# Like the symbolic simulator's and the Gazebo backend's key: FIXED, PUBLISHED, SIMULATION-ONLY. A signature shows that
# the documented procedure ran; it is not a secret.
SIM_KEY = Key(SIM_PARTICIPANT, SIM_HMAC, b"robolang-g1-mujoco-simulation-only-key-not-a-secret")

SUPPORTED = (ir.ObjectAt, ir.RobotAt, ir.Holding, ir.RobotReady, ir.And, ir.Or, ir.Not)


# ---------------------------------------------------------------------------------------------- what a snapshot holds


@dataclass(frozen=True)
class Hand:
    """One Dex3 hand as `rt/dex3/<side>/state` reports it, and where its grasp point is (from the measured joints)."""

    side: str
    state: str  # "closed", "open" or "between"
    q: tuple[float, ...]
    grasp_point: tuple[float, float, float]  # pelvis frame, from the measured body joints
    stamp_ms: int  # simulation time (lowstate tick) when the state arrived


@dataclass(frozen=True)
class Ready:
    """The pieces of `robot_ready`; None where something could not be read (which makes the atom `unknown`)."""

    lowstate_fresh: bool | None
    motor_error: int | None  # the OR of the body motors' error words
    tilt_rad: float | None
    fsm_id: int | None
    stamp_ms: int  # simulation time of the lowstate sample it was taken from


@dataclass(frozen=True)
class Snapshot:
    """What the robot's sensors reported at `t_ms` (simulation time): the fused marker readings of one snapshot of about
    `frames` head-camera frames, the hands, the IMU's vertical, and the readiness data when it was asked for."""

    t_ms: int
    frames: int
    markers: dict[int, Reading]
    hands: dict[str, Hand] = field(default_factory=dict)
    up: tuple[float, float, float] | None = None  # the IMU's vertical in the pelvis frame
    ready: Ready | None = None


@dataclass(frozen=True)
class Policy:
    min_confidence: Decimal
    max_age_ms: int

    @classmethod
    def of(cls, task: ir.Task) -> "Policy":
        verify = task.verify
        confidence = verify.min_confidence if verify and verify.min_confidence is not None else None
        age = verify.max_age if verify and verify.max_age is not None else None
        if age is None:
            max_age = DEFAULT_MAX_AGE_MS
        else:
            from robolang.sim.values import duration_ms_of

            max_age = int(duration_ms_of(age))
        return cls(DEFAULT_MIN_CONFIDENCE if confidence is None else Decimal(confidence), max_age)


@dataclass(frozen=True)
class Verdict:
    """The result of one atom (`true`, `false` or `unknown`), the oldest stamp and the lowest confidence among the
    readings it rests on, and the numbers behind it (offsets, distances, margins) for the record."""

    result: str
    stamp_ms: int | None = None
    confidence: float | None = None
    detail: dict = field(default_factory=dict)


def hand_state(side: str, q) -> str:
    """"closed" or "open" when the seven joints are within HAND_TOLERANCE_RAD of that pose, else "between"."""
    q = np.asarray(q, float)
    if float(np.max(np.abs(q - np.array(HAND_CLOSED[side])))) <= HAND_TOLERANCE_RAD:
        return "closed"
    if float(np.max(np.abs(q))) <= HAND_TOLERANCE_RAD:
        return "open"
    return "between"


def unsupported(condition: ir.Condition) -> str | None:
    """The kind of the first condition that has no observation here, else None."""
    if isinstance(condition, (ir.And, ir.Or)):
        return next((k for k in map(unsupported, condition.conditions) if k), None)
    if isinstance(condition, ir.Not):
        return unsupported(condition.condition)
    return None if isinstance(condition, SUPPORTED) else condition.KIND


def atoms_of(condition: ir.Condition):
    """Every atom in a condition, at any depth."""
    if isinstance(condition, (ir.And, ir.Or)):
        for child in condition.conditions:
            yield from atoms_of(child)
    elif isinstance(condition, ir.Not):
        yield from atoms_of(condition.condition)
    else:
        yield condition


def uses_ready(conditions) -> bool:
    """Whether any of the conditions contains `robot_ready` (its observation reads the walking service as well)."""
    return any(isinstance(a, ir.RobotReady) for c in conditions for a in atoms_of(c))


# ---------------------------------------------------------------------------------------------- three-valued logic


def evaluate(condition: ir.Condition, names: dict[str, str], site: dict, snap: Snapshot, policy: Policy) -> str:
    """Kleene three-valued result of a condition, as in the symbolic simulator (spec/simulation-v1.0.md 4.4)."""
    if isinstance(condition, ir.And):
        results = [evaluate(c, names, site, snap, policy) for c in condition.conditions]
        return "false" if "false" in results else "unknown" if "unknown" in results else "true"
    if isinstance(condition, ir.Or):
        results = [evaluate(c, names, site, snap, policy) for c in condition.conditions]
        return "true" if "true" in results else "unknown" if "unknown" in results else "false"
    if isinstance(condition, ir.Not):
        return {"true": "false", "false": "true", "unknown": "unknown"}[
            evaluate(condition.condition, names, site, snap, policy)
        ]
    return atom(condition, names, site, snap, policy).result


def atoms_report(conditions, names: dict[str, str], site: dict, snap: Snapshot, policy: Policy) -> list[dict]:
    """Every atom of the conditions with its verdict and numbers: the record's account of how each verdict came out."""
    out = []
    for condition in conditions:
        for a in atoms_of(condition):
            v = atom(a, names, site, snap, policy)
            out.append({"atom": ir.to_dict(a), "result": v.result, **v.detail})
    return out


def _usable(reading: Reading | None, snap: Snapshot, policy: Policy) -> bool:
    return (
        reading is not None
        and Decimal(str(reading.confidence)) >= policy.min_confidence
        and 0 <= snap.t_ms - reading.stamp_ms <= policy.max_age_ms
    )


def _why(reading: Reading | None, snap: Snapshot, policy: Policy) -> str:
    """Why a reading is not usable: for the record's detail."""
    if reading is None:
        return "not seen"
    if Decimal(str(reading.confidence)) < policy.min_confidence:
        return f"confidence {reading.confidence:.2f} below {policy.min_confidence}"
    return f"age {snap.t_ms - reading.stamp_ms} ms above {policy.max_age_ms}"


def atom(condition, names: dict[str, str], site: dict, snap: Snapshot, policy: Policy) -> Verdict:
    if isinstance(condition, ir.ObjectAt):
        return _object_at(names[condition.obj], names[condition.location], site, snap, policy)
    if isinstance(condition, ir.RobotAt):
        return _robot_at(names[condition.location], site, snap, policy)
    if isinstance(condition, ir.Holding):
        return _holding(names[condition.obj], site, snap, policy)
    if isinstance(condition, ir.RobotReady):
        return _robot_ready(snap)
    raise ValueError(f"no observation for {condition.KIND}")


# ---------------------------------------------------------------------------------------------- the atoms


def marker_in_station_frame(marker: Reading, station: Reading) -> np.ndarray:
    """A marker's position in a station marker's frame (x right, y up, z out of the plate), both from one snapshot."""
    return station.rotation.T @ (marker.position - station.position)


def region_clearance(point: np.ndarray, region: dict) -> float:
    """How far a point is inside a box region: positive inside (the distance to the nearest face), negative outside
    (minus the distance to the box)."""
    lo, hi = np.array(region["min"], float), np.array(region["max"], float)
    inside = np.minimum(point - lo, hi - point)
    if np.all(inside >= 0):
        return float(inside.min())
    return -float(np.linalg.norm(np.maximum(np.maximum(lo - point, point - hi), 0.0)))


def _held_by(marker: Reading, snap: Snapshot) -> str | None:
    """The side of a closed hand whose grasp point the marker is within HELD_RADIUS_M of, if any."""
    for side, hand in sorted(snap.hands.items()):
        if hand.state == "closed" and np.linalg.norm(marker.position - np.array(hand.grasp_point)) <= HELD_RADIUS_M:
            return side
    return None


def _object_at(obj: str, location: str, site: dict, snap: Snapshot, policy: Policy) -> Verdict:
    marker = snap.markers.get(site_mod.obj(site, obj)["marker"]["id"])
    loc = site_mod.location(site, location)
    station = snap.markers.get(site_mod.station(site, loc["station"])["marker"]["id"])
    if not _usable(marker, snap, policy):
        return Verdict("unknown", detail={"why": f"the marker of {obj}: {_why(marker, snap, policy)}"})
    stamp, conf = marker.stamp_ms, marker.confidence
    side = _held_by(marker, snap)
    if side is not None:  # carried, so not at a place (RoboLang: a held object has no location)
        return Verdict("false", stamp, conf, {"seen_at_hand": side})
    if not _usable(station, snap, policy):
        return Verdict("unknown", detail={"why": f"the station marker of {location}: {_why(station, snap, policy)}"})
    point = marker_in_station_frame(marker, station)
    clearance = region_clearance(point, loc["region"])
    detail = {"marker_in_station_frame": [round(float(x), 4) for x in point], "region_clearance_m": round(clearance, 4)}
    stamp, conf = min(marker.stamp_ms, station.stamp_ms), min(marker.confidence, station.confidence)
    if abs(clearance) < REGION_MARGIN_M:
        return Verdict("unknown", stamp, conf, {**detail, "why": "within the margin of the region's boundary"})
    return Verdict("true" if clearance > 0 else "false", stamp, conf, detail)


def _holding(obj: str, site: dict, snap: Snapshot, policy: Policy) -> Verdict:
    hands = snap.hands
    if not hands:
        return Verdict("unknown", detail={"why": "no hand state"})
    states = {side: h.state for side, h in sorted(hands.items())}
    detail = {"hands": states}
    marker = snap.markers.get(site_mod.obj(site, obj)["marker"]["id"])
    usable = _usable(marker, snap, policy)
    if usable:
        side = _held_by(marker, snap)
        if side is not None:
            hand = hands[side]
            detail.update(hand=side, marker_to_hand_m=round(float(np.linalg.norm(
                marker.position - np.array(hand.grasp_point))), 4))
            return Verdict("true", min(marker.stamp_ms, hand.stamp_ms), marker.confidence, detail)
    if all(state == "open" for state in states.values()):
        return Verdict("false", None, None, {**detail, "why": "both hands are open"})
    if "between" in states.values():
        return Verdict("unknown", detail={**detail, "why": "a hand is neither open nor closed"})
    if not usable:
        return Verdict("unknown", detail={**detail, "why": f"the marker of {obj}: {_why(marker, snap, policy)}"})
    nearest = min(np.linalg.norm(marker.position - np.array(h.grasp_point)) for h in hands.values()
                  if h.state == "closed")
    return Verdict("false", marker.stamp_ms, marker.confidence,
                   {**detail, "why": "the marker is seen away from the closed hand",
                    "marker_to_hand_m": round(float(nearest), 4)})


def station_offset(station_site: dict, marker: Reading) -> tuple[float, float]:
    """How far (metres in plan, radians of heading) the pelvis is from the station's pose, from one seen marker."""
    x, y, yaw, _ = pelvis_pose_in_map(station_site["marker"], marker)
    gx, gy, gyaw = station_site["pose"]
    return math.hypot(x - gx, y - gy), abs(site_mod.wrap(yaw - gyaw))


def _robot_at(location: str, site: dict, snap: Snapshot, policy: Policy) -> Verdict:
    target = site_mod.station(site, site_mod.location(site, location)["station"])
    marker = snap.markers.get(target["marker"]["id"])
    detail: dict = {"station": target["name"], "tolerance_m": ROBOT_AT_M, "tolerance_rad": ROBOT_AT_RAD}
    if _usable(marker, snap, policy):
        err_m, err_rad = station_offset(target, marker)
        detail.update(offset_m=round(err_m, 4), offset_rad=round(err_rad, 4))
        if _inside(err_m, err_rad):
            return Verdict("true", marker.stamp_ms, marker.confidence, detail)
        if err_m > ROBOT_AT_M + ROBOT_AT_MARGIN_M or err_rad > ROBOT_AT_RAD + ROBOT_AT_MARGIN_RAD:
            return Verdict("false", marker.stamp_ms, marker.confidence, detail)
        return Verdict("unknown", marker.stamp_ms, marker.confidence,
                       {**detail, "why": "within the margin of the tolerance"})
    detail["why"] = f"the station marker of {location}: {_why(marker, snap, policy)}"
    # Not located by its own marker: false if another station's marker confirms that the robot is at that station
    # (within the tolerance), and that station is further from this one than twice the tolerance.
    for other in site["stations"]:
        seen = snap.markers.get(other["marker"]["id"])
        if other["name"] == target["name"] or not _usable(seen, snap, policy):
            continue
        err_m, err_rad = station_offset(other, seen)
        apart = math.hypot(other["pose"][0] - target["pose"][0], other["pose"][1] - target["pose"][1])
        turn = abs(site_mod.wrap(other["pose"][2] - target["pose"][2]))
        if _inside(err_m, err_rad) and (apart > 2 * ROBOT_AT_M or turn > 2 * ROBOT_AT_RAD):
            return Verdict("false", seen.stamp_ms, seen.confidence,
                           {**detail, "confirmed_at_station": other["name"], "offset_m": round(err_m, 4),
                            "offset_rad": round(err_rad, 4)})
    return Verdict("unknown", detail=detail)


def _inside(err_m: float, err_rad: float) -> bool:
    """Within the tolerance by at least the margin: what `true` needs."""
    return err_m <= ROBOT_AT_M - ROBOT_AT_MARGIN_M and err_rad <= ROBOT_AT_RAD - ROBOT_AT_MARGIN_RAD


def _robot_ready(snap: Snapshot) -> Verdict:
    r = snap.ready
    if r is None:
        return Verdict("unknown", detail={"why": "readiness was not read"})
    detail = {"lowstate_fresh": r.lowstate_fresh, "motor_error": r.motor_error, "fsm_id": r.fsm_id,
              "tilt_deg": None if r.tilt_rad is None else round(math.degrees(r.tilt_rad), 2)}

    def verdict(result: str, why: str | None = None) -> Verdict:
        return Verdict(result, r.stamp_ms, 1.0, detail if why is None else {**detail, "why": why})

    if r.lowstate_fresh is not True or r.tilt_rad is None or r.motor_error is None:
        return verdict("unknown", "lowstate is not fresh")
    if r.motor_error:
        return verdict("false", "a motor reports an error")
    if r.tilt_rad > READY_TILT_RAD:
        return verdict("false", "not upright")
    if r.fsm_id is None:
        return verdict("unknown", "the walking service did not answer")
    if r.fsm_id not in FSM_STANDING_OR_WALKING:
        return verdict("false", "the walking service is not standing or walking")
    return verdict("true")


# ---------------------------------------------------------------------------------------------- the record's snapshot


def snapshot_record(snap: Snapshot, purpose: str, extra: dict | None = None) -> dict:
    """A snapshot for the run record: every marker reading with its confidence and age, the hands, the readiness."""
    out = {
        "purpose": purpose,
        "t_ms": snap.t_ms,
        "frames": snap.frames,
        "markers": {str(k): v.as_dict(snap.t_ms) for k, v in sorted(snap.markers.items())},
        "hands": {s: {"state": h.state, "grasp_point": [round(float(x), 4) for x in h.grasp_point],
                      "q": [round(float(x), 3) for x in h.q]} for s, h in sorted(snap.hands.items())},
    }
    if snap.ready is not None:
        r = snap.ready
        out["ready"] = {"lowstate_fresh": r.lowstate_fresh, "motor_error": r.motor_error, "fsm_id": r.fsm_id,
                        "tilt_deg": None if r.tilt_rad is None else round(math.degrees(r.tilt_rad), 2)}
    out.update(extra or {})
    return out


# ---------------------------------------------------------------------------------------------- Core outcome


def core_outcome(task: ir.Task, bindings: dict[str, str], names: dict[str, str], site: dict, snap: Snapshot | None,
                 policy: Policy, state: str, failure_code: str | None, run_id: str) -> dict | None:
    """Core 0.1 `outcome` payload with observation evidence (as the ROS 2 backend builds it); None when the goal has no
    Core form (a goal with `robot_ready`, `or` or `not`). `bindings` are the `.rob` entity IDs the Core facts are made
    of; `names` map the task's parameters to the site's names. A criterion is met only by a `true` verdict, and then
    the evidence is the signed observation of what the readings support: the atom's own statement."""
    try:
        atoms = flatten_atoms(task.goal)
        criteria = [triple_of(a, bindings) for a in atoms]
    except NotExportable:
        return None
    status = "succeeded" if state == "SUCCEEDED" else "unknown" if failure_code == "S302" else "failed"
    verification, evidence = None, []
    end = core_time(snap.t_ms) if snap else core_time(0)
    if snap is not None:
        results = []
        for i, (condition, criterion) in enumerate(zip(atoms, criteria)):
            verdict = atom(condition, names, site, snap, policy)
            cited = []
            if verdict.result == "true" and verdict.stamp_ms is not None:
                evidence.append(sign_sim({
                    "kind": "observation", "id": f"rl:obs:{run_id}-{i}", **criterion.as_dict(),
                    "observed_at": core_time(verdict.stamp_ms),
                    "valid_for": {"value": str(policy.max_age_ms), "unit": "ms"},
                    "confidence": confidence_text(verdict.confidence if verdict.confidence is not None else 0.0),
                    "source": {"participant": SIM_PARTICIPANT, "sensor": "head_camera+dex3+lowstate",
                               "method": _method(condition)},
                }, SIM_KEY_REF, SIM_KEY))
                cited = [evidence[-1]["id"]]
            results.append({"criterion": i, "met": verdict.result == "true", "evidence": cited})
        verification = {"verified_by": SIM_PARTICIPANT, "method": "criteria_evaluated", "evaluated_at": end,
                        "criteria_results": results}
    outcome = {
        "kind": "outcome", "goal": f"rl:goal:{task.name}", "status": status, "verification": verification,
        "reasons": [{"code": "evidence_missing"}] if failure_code == "S302" else [],
        "reported_by": SIM_PARTICIPANT, "reported_at": end,
    }
    return {"outcome": outcome, "evidence": evidence}


def _method(condition) -> str:
    return {"object_at": "aruco_fiducial_in_region_of_station_marker",
            "holding": "aruco_fiducial_at_closed_hand",
            "robot_at": "aruco_station_marker_offset"}.get(condition.KIND, "observation")


def confidence_text(value: float) -> str:
    """Core Confidence: a decimal string from 0 to 1, rounded down to two places so it never overstates."""
    return str(Decimal(math.floor(max(0.0, min(1.0, value)) * 100)) / 100)
