"""Validated IR to ROS operations; preconditions and the goal are decided from sensor observations."""
from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from typing import Protocol

from robolang import ir

from . import observe


class BackendFailure(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


class Operations(Protocol):
    def guard(self, model: str) -> None: ...
    def now_ms(self) -> int: ...
    def move(self, pose: dict) -> None: ...
    def grip(self, closed: bool) -> None: ...
    def attach(self, model: str) -> None: ...
    def detach(self, model: str) -> None: ...
    def wait_ms(self, duration: int) -> None: ...
    def versions(self) -> dict[str, str]: ...
    def observe(self, marker_ids: list[int]) -> observe.Snapshot: ...


@dataclass(frozen=True)
class Site:
    model: str
    robot: str
    object_models: dict[str, str]
    poses: dict[str, dict]
    grasps: dict[str, dict]
    markers: dict[str, int] = field(default_factory=dict)  # object ID -> ArUco ID
    regions: dict[str, dict] = field(default_factory=dict)  # location ID -> {"min", "max"} box in base_link
    cameras: tuple[dict, ...] = ()
    obstacles: tuple[dict, ...] = ()  # {"min", "max"} boxes in base_link, added to MoveIt's planning scene
    approach_height: float = 0.0  # grasp and place come straight down from this far above their poses


def execute(task: ir.Task, bindings: dict[str, str], site: Site, ops: Operations) -> dict:
    """Run a validated single-arm task. The goal is verified from sensor readings only, never from ground truth."""
    policy = observe.Policy.of(task)
    record = {
        "run_record_schema": "robolang.ros2.run",
        "run_record_schema_version": "0.2",
        "mode": "simulation",
        "backend": {"kind": "ros2-gazebo", "versions": ops.versions(),
                    "grasp_model": "detachable_joint_after_gripper_close"},
        "task": ir.to_dict(task),
        "bindings": dict(sorted(bindings.items())),
        "verify": {"min_confidence": str(policy.min_confidence), "max_age_ms": policy.max_age_ms},
        "state": "FAILED",
        "failure": None,
        "preconditions": [{"index": index, "condition": ir.to_dict(condition), "result": "not_evaluated"}
                          for index, condition in enumerate(task.preconditions, 1)],
        "steps": [],
        "goal": [{"index": index, "condition": ir.to_dict(condition), "result": "not_evaluated"}
                 for index, condition in enumerate(task.goal, 1)],
        "observations": [],
        "core_outcome": None,
    }
    snap = None
    try:
        # The guard is the first operation that may inspect the graph. No controller is contacted before it.
        ops.guard(site.model)
        if {p.name for p in task.parameters} != bindings.keys():
            raise BackendFailure("S507", "bindings must name every task parameter exactly once")
        robot_params = [p.name for p in task.parameters if p.type == "robot"]
        if len(robot_params) != 1 or bindings[robot_params[0]] != site.robot:
            raise BackendFailure("S507", "this site supports exactly its declared robot")
        for condition in (*task.preconditions, *task.goal):
            kind = observe.unsupported(condition)
            if kind:
                raise BackendFailure("S508", f"{kind} has no observation in this backend")
            _check_observable(condition, bindings, site)
        if task.safety:
            raise BackendFailure("S508", "declared speed limits are not enforced by this backend yet")
        for step in task.steps:
            if not isinstance(step, (ir.Move, ir.Grasp, ir.Place, ir.WaitFor)):
                raise BackendFailure("S508", f"{step.KIND} is not supported by this backend yet")
            if isinstance(step, (ir.Move, ir.Grasp, ir.Place)) and bindings[step.robot] != site.robot:
                raise BackendFailure("S507", "step names a robot outside this site")
            if isinstance(step, (ir.Move, ir.Place)):
                _pose(site, bindings[step.target])
            if isinstance(step, (ir.Grasp, ir.Place)):
                _object(site, bindings[step.obj])
            if isinstance(step, ir.Grasp):
                _grasp_pose(site, bindings[step.obj])
        if task.preconditions:
            snap = _observe(task, bindings, site, ops, record)
            for entry, condition in zip(record["preconditions"], task.preconditions):
                entry["result"] = observe.evaluate(condition, bindings, site, snap, policy)
            results = [entry["result"] for entry in record["preconditions"]]
            if "false" in results:
                raise BackendFailure("S101", "a precondition is false")
            if "unknown" in results:
                raise BackendFailure("S102", "a precondition is unknown from the observations")
        held = None
        for index, step in enumerate(task.steps, 1):
            start = ops.now_ms()
            entry = {"index": index, "kind": step.KIND, "status": "running", "t_start_ms": start}
            record["steps"].append(entry)
            try:
                if isinstance(step, ir.Move):
                    ops.move(_pose(site, bindings[step.target]))
                elif isinstance(step, ir.Grasp):
                    model = _object(site, bindings[step.obj])
                    grasp = _grasp_pose(site, bindings[step.obj])
                    _approach(ops, site, grasp)
                    ops.move(grasp)
                    # The detachable joint attaches whatever is declared; only attach what the cameras see.
                    seen = observe.object_under_tool(bindings[step.obj], site,
                                                     _observe(task, bindings, site, ops, record), policy)
                    if seen != "true":
                        raise BackendFailure("S506", f"object not observed under the gripper ({seen})")
                    ops.grip(True)
                    ops.attach(model)
                    held = model
                    _approach(ops, site, grasp)
                elif isinstance(step, ir.Place):
                    if held != _object(site, bindings[step.obj]):
                        raise BackendFailure("S506", "place requires a prior successful grasp of this object")
                    model = _object(site, bindings[step.obj])
                    pose = _pose(site, bindings[step.target])
                    ops.move(pose)
                    if site.approach_height:
                        ops.move(_lowered(pose, site.approach_height))
                    ops.grip(False)
                    ops.detach(model)
                    held = None
                    if site.approach_height:
                        ops.move(pose)
                elif isinstance(step, ir.WaitFor):
                    from robolang.sim.values import duration_ms_of
                    ops.wait_ms(int(duration_ms_of(step.duration)))
                else:
                    raise BackendFailure("S508", f"{step.KIND} is not supported by this backend yet")
            except BackendFailure:
                entry["status"] = "failed"
                entry["t_end_ms"] = ops.now_ms()
                raise
            entry["status"] = "succeeded"
            entry["t_end_ms"] = ops.now_ms()
        snap = _observe(task, bindings, site, ops, record)
        for entry, condition in zip(record["goal"], task.goal):
            entry["result"] = observe.evaluate(condition, bindings, site, snap, policy)
        results = [entry["result"] for entry in record["goal"]]
        if "false" in results:
            raise BackendFailure("S301", "a goal condition is false from the observations")
        if "unknown" in results:
            raise BackendFailure("S302", "a goal condition is unknown from the observations")
        record["state"] = "SUCCEEDED"
    except BackendFailure as exc:
        record["state"] = "FAILED"
        step = record["steps"][-1]["index"] if record["steps"] and exc.code not in ("S301", "S302") else None
        record["failure"] = {"code": exc.code, "message": exc.message, "step": step}
    goal_snap = snap if record["state"] == "SUCCEEDED" or (record["failure"] or {}).get("code") in (
        "S301", "S302") else None
    run_id = hashlib.sha256(json.dumps([record["task"], record["bindings"], record["steps"][:1]],
                                       sort_keys=True).encode()).hexdigest()[:16]
    record["core_outcome"] = observe.core_outcome(task, bindings, site, goal_snap, policy, record["state"],
                                                  (record["failure"] or {}).get("code"), run_id)
    return record


def _observe(task: ir.Task, bindings: dict[str, str], site: Site, ops: Operations, record: dict):
    """One sensor snapshot, kept in the record so every verdict can be audited."""
    snap = ops.observe(sorted(set(site.markers.values())))
    record["observations"].append({
        "t_ms": snap.t_ms,
        "markers": {str(k): _reading(v) for k, v in sorted(snap.markers.items())},
        "tool": _reading(snap.tool), "gripper": _reading(snap.gripper),
    })
    return snap


def _reading(reading) -> dict | None:
    if reading is None:
        return None
    return {"sensor": reading.sensor, "method": reading.method,
            "position": [round(x, 4) for x in reading.position],
            "orientation": None if reading.orientation is None else [round(x, 4) for x in reading.orientation],
            "confidence": reading.confidence, "stamp_ms": reading.stamp_ms}


def _check_observable(condition, bindings: dict[str, str], site: Site) -> None:
    """Every atom must name a marked object, a location with a region or pose, and the site's robot."""
    if isinstance(condition, (ir.And, ir.Or)):
        for child in condition.conditions:
            _check_observable(child, bindings, site)
    elif isinstance(condition, ir.Not):
        _check_observable(condition.condition, bindings, site)
    elif isinstance(condition, (ir.ObjectAt, ir.Holding)):
        if bindings[condition.obj] not in site.markers:
            raise BackendFailure("S507", f"no marker for object {bindings[condition.obj]!r}")
        if isinstance(condition, ir.ObjectAt) and bindings[condition.location] not in site.regions:
            raise BackendFailure("S507", f"no region for location {bindings[condition.location]!r}")
        if isinstance(condition, ir.Holding) and bindings[condition.robot] != site.robot:
            raise BackendFailure("S507", "condition names a robot outside this site")
    elif isinstance(condition, ir.RobotAt):
        if bindings[condition.robot] != site.robot:
            raise BackendFailure("S507", "condition names a robot outside this site")
        _pose(site, bindings[condition.location])


def _lowered(pose: dict, height: float) -> dict:
    x, y, z = pose["position"]
    return {**pose, "position": [x, y, z - height]}


def _approach(ops: Operations, site: Site, pose: dict) -> None:
    if site.approach_height:
        ops.move(_lowered(pose, -site.approach_height))


def _pose(site: Site, location: str) -> dict:
    try:
        return site.poses[location]
    except KeyError:
        raise BackendFailure("S507", f"no pose for location {location!r}") from None


def _object(site: Site, obj: str) -> str:
    try:
        return site.object_models[obj]
    except KeyError:
        raise BackendFailure("S507", f"no Gazebo model for object {obj!r}") from None


def _grasp_pose(site: Site, obj: str) -> dict:
    try:
        return site.grasps[obj]
    except KeyError:
        raise BackendFailure("S507", f"no grasp pose for object {obj!r}") from None
