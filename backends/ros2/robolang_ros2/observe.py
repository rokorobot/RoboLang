"""Condition verdicts from sensor readings only (decision 0029 X6). Pure Python: no ROS, no Gazebo ground truth.

A reading that is missing, older than `max_age` or below `min_confidence` makes its atom `unknown`, never `true`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from decimal import Decimal

from robolang import ir
from robolang.bridge import NotExportable, Triple, core_id, core_time, flatten_atoms, triple_of
from robolang.core.signatures import SIM_HMAC, Key, sign_sim

# RoboLang 1.3 section 4: the exporter defaults without @verify.
DEFAULT_MIN_CONFIDENCE = Decimal("0.9")
DEFAULT_MAX_AGE_MS = 10_000
# robot_at: the tool frame is within these of the location's site pose.
POSITION_TOLERANCE_M = 0.02
ANGLE_TOLERANCE_RAD = 0.05
# holding: gripper closed (Robotiq knuckle near its closed command) and the object's marker near the tool.
GRIPPER_CLOSED_RAD = 0.8
GRIPPER_TOLERANCE_RAD = 0.05
HOLDING_DISTANCE_M = 0.15

GAZEBO = "rl:sim:gazebo"
GAZEBO_KEY_REF = GAZEBO + "#sim-1"
# Like the symbolic simulator's key: FIXED, PUBLISHED, SIMULATION-ONLY. Signatures show the documented procedure ran.
GAZEBO_KEY = Key(GAZEBO, SIM_HMAC, b"robolang-gazebo-simulation-only-key-not-a-secret")

SUPPORTED = (ir.ObjectAt, ir.RobotAt, ir.Holding, ir.And, ir.Or, ir.Not)


@dataclass(frozen=True)
class Reading:
    """One fused sensor estimate in `base_link`. `position` is xyz, or (joint angle,) for the gripper."""

    sensor: str
    method: str
    position: tuple[float, ...]
    orientation: tuple[float, float, float, float] | None
    confidence: float
    stamp_ms: int


@dataclass(frozen=True)
class Snapshot:
    """What the sensors reported at `t_ms`. A marker that was not detected is absent from `markers`."""

    t_ms: int
    markers: dict[int, Reading]
    tool: Reading | None
    gripper: Reading | None


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
    result: str  # "true", "false" or "unknown"
    observed: Triple | None  # the Core statement the readings support, if any
    reading: Reading | None


def unsupported(condition: ir.Condition) -> str | None:
    """The kind of the first condition that has no observation here, else None."""
    if isinstance(condition, (ir.And, ir.Or)):
        return next((k for k in map(unsupported, condition.conditions) if k), None)
    if isinstance(condition, ir.Not):
        return unsupported(condition.condition)
    return None if isinstance(condition, SUPPORTED) else condition.KIND


def evaluate(condition: ir.Condition, bound: dict[str, str], site, snap: Snapshot, policy: Policy) -> str:
    """Kleene three-valued result of a condition, as in the symbolic simulator (spec/simulation-v1.0.md 4.4)."""
    if isinstance(condition, ir.And):
        results = [evaluate(c, bound, site, snap, policy) for c in condition.conditions]
        return "false" if "false" in results else "unknown" if "unknown" in results else "true"
    if isinstance(condition, ir.Or):
        results = [evaluate(c, bound, site, snap, policy) for c in condition.conditions]
        return "true" if "true" in results else "unknown" if "unknown" in results else "false"
    if isinstance(condition, ir.Not):
        return {"true": "false", "false": "true", "unknown": "unknown"}[
            evaluate(condition.condition, bound, site, snap, policy)]
    return atom(condition, bound, site, snap, policy).result


def atom(condition, bound: dict[str, str], site, snap: Snapshot, policy: Policy) -> Verdict:
    usable = lambda r: (r is not None and Decimal(str(r.confidence)) >= policy.min_confidence  # noqa: E731
                        and 0 <= snap.t_ms - r.stamp_ms <= policy.max_age_ms)
    if isinstance(condition, ir.ObjectAt):
        obj = bound[condition.obj]
        marker = snap.markers.get(site.markers[obj])
        if not usable(marker):
            return Verdict("unknown", None, marker)
        where = _first(bound[condition.location], site.regions, lambda region: inside(marker.position, region))
        observed = None if where is None else Triple(core_id("object", obj), "located_in", core_id("location", where))
        return Verdict("true" if where == bound[condition.location] else "false", observed, marker)
    if isinstance(condition, ir.RobotAt):
        robot, tool = bound[condition.robot], snap.tool
        if not usable(tool):
            return Verdict("unknown", None, tool)
        where = _first(bound[condition.location], site.poses, lambda pose: at_pose(tool, pose))
        observed = None if where is None else Triple(core_id("robot", robot), "located_in", core_id("location", where))
        return Verdict("true" if where == bound[condition.location] else "false", observed, tool)
    if isinstance(condition, ir.Holding):
        robot, obj, grip = bound[condition.robot], bound[condition.obj], snap.gripper
        if not usable(grip):
            return Verdict("unknown", None, grip)
        if abs(grip.position[0] - GRIPPER_CLOSED_RAD) > GRIPPER_TOLERANCE_RAD:
            return Verdict("false", None, grip)  # an open gripper holds nothing
        marker = snap.markers.get(site.markers[obj])
        if not usable(marker) or not usable(snap.tool):
            return Verdict("unknown", None, marker)
        if math.dist(marker.position, snap.tool.position) > HOLDING_DISTANCE_M:
            return Verdict("false", None, marker)
        fused = Reading("overhead_camera+gripper", "aruco_fiducial_near_closed_gripper", marker.position, None,
                        min(marker.confidence, grip.confidence, snap.tool.confidence),
                        min(marker.stamp_ms, grip.stamp_ms, snap.tool.stamp_ms))
        return Verdict("true", Triple(core_id("object", obj), "carried_by", core_id("robot", robot)), fused)
    raise ValueError(f"no observation for {condition.KIND}")


def _first(target: str, places: dict, test) -> str | None:
    """The location a reading supports: the target if it matches, else the first other match by ID."""
    if test(places[target]):
        return target
    return next((loc for loc in sorted(places) if test(places[loc])), None)


def intrinsics(camera: dict) -> tuple[float, float, float]:
    """(focal length in pixels, cx, cy) of an ideal Gazebo pinhole camera from its horizontal field of view."""
    return (camera["width"] / 2 / math.tan(camera["horizontal_fov"] / 2), camera["width"] / 2, camera["height"] / 2)


def camera_to_base(point, camera: dict) -> tuple[float, float, float]:
    """A point in the OpenCV optical frame (z forward, x right, y down) to `base_link`.

    A Gazebo camera looks along its link's +x with image right at -y and image down at -z, so the point in the
    camera link frame is (z, -x, -y). The site gives the link's pose in `base_link` (a calibration, not ground truth).
    """
    x, y, z = point
    link = (z, -x, -y)
    qx, qy, qz, qw = camera["orientation"]
    rotation = (
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
    )
    return tuple(p + sum(r * v for r, v in zip(row, link)) for p, row in zip(camera["position"], rotation))


def fuse(positions: list[tuple[float, float, float]], stamps: list[int], frames: int,
         sensor: str, method: str, spread_m: float = 0.01) -> Reading | None:
    """One estimate from per-frame detections: the median, with confidence = agreeing detections / frames."""
    if not positions or frames <= 0:
        return None
    median = tuple(sorted(axis)[len(axis) // 2] for axis in zip(*positions))
    agreeing = [stamp for p, stamp in zip(positions, stamps) if math.dist(p, median) <= spread_m]
    return Reading(sensor, method, median, None, len(agreeing) / frames, max(agreeing))


def inside(position, region: dict) -> bool:
    return all(lo <= x <= hi for x, lo, hi in zip(position, region["min"], region["max"]))


def at_pose(tool: Reading, pose: dict) -> bool:
    if math.dist(tool.position, pose["position"]) > POSITION_TOLERANCE_M:
        return False
    if tool.orientation is None:
        return False
    dot = abs(sum(a * b for a, b in zip(tool.orientation, pose["orientation"])))
    return 2 * math.acos(min(1.0, dot)) <= ANGLE_TOLERANCE_RAD


def core_outcome(task: ir.Task, bound: dict[str, str], site, snap: Snapshot | None, policy: Policy,
                 state: str, failure_code: str | None, run_id: str) -> dict | None:
    """Core 0.1 `outcome` payload with observation evidence; None when the goal has no Core form."""
    try:
        atoms = flatten_atoms(task.goal)
        criteria = [triple_of(a, bound) for a in atoms]
    except NotExportable:
        return None
    status = ("succeeded" if state == "SUCCEEDED" else
              "unknown" if failure_code == "S302" else "failed")
    verification, evidence = None, []
    end = core_time(snap.t_ms) if snap else core_time(0)
    if snap is not None:
        results = []
        for i, (condition, criterion) in enumerate(zip(atoms, criteria)):
            verdict = atom(condition, bound, site, snap, policy)
            cited = []
            if verdict.observed is not None and verdict.reading is not None:
                r = verdict.reading
                evidence.append(sign_sim({
                    "kind": "observation", "id": f"rl:obs:{run_id}-{i}", **verdict.observed.as_dict(),
                    "observed_at": core_time(r.stamp_ms),
                    "valid_for": {"value": str(policy.max_age_ms), "unit": "ms"},
                    "confidence": confidence_text(r.confidence),
                    "source": {"participant": GAZEBO, "sensor": r.sensor, "method": r.method},
                }, GAZEBO_KEY_REF, GAZEBO_KEY))
                cited = [evidence[-1]["id"]]
            results.append({"criterion": i, "met": verdict.result == "true" and verdict.observed == criterion,
                            "evidence": cited})
        verification = {"verified_by": GAZEBO, "method": "criteria_evaluated", "evaluated_at": end,
                        "criteria_results": results}
    outcome = {
        "kind": "outcome", "goal": f"rl:goal:{task.name}", "status": status, "verification": verification,
        "reasons": [{"code": "evidence_missing"}] if failure_code == "S302" else [],
        "reported_by": GAZEBO, "reported_at": end,
    }
    return {"outcome": outcome, "evidence": evidence}


def confidence_text(value: float) -> str:
    """Core Confidence: a decimal string from 0 to 1, rounded down to two places so it never overstates."""
    return str(Decimal(math.floor(max(0.0, min(1.0, value)) * 100)) / 100)
