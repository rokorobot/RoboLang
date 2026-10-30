"""Validated IR to ROS operations. Success of the task is reserved for phase 3 observations."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from robolang import ir


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


@dataclass(frozen=True)
class Site:
    model: str
    robot: str
    object_models: dict[str, str]
    poses: dict[str, dict]
    grasps: dict[str, dict]


def execute(task: ir.Task, bindings: dict[str, str], site: Site, ops: Operations) -> dict:
    """Run a validated single-arm task; never turn completed steps into a verified goal verdict."""
    record = {
        "run_record_schema": "robolang.ros2.run",
        "run_record_schema_version": "0.1",
        "mode": "simulation",
        "backend": {"kind": "ros2-gazebo", "versions": ops.versions(),
                    "grasp_model": "detachable_joint_after_gripper_close"},
        "task": task.name,
        "bindings": dict(sorted(bindings.items())),
        "state": "UNKNOWN",
        "failure": None,
        "steps": [],
        "goal": "not_evaluated",
        "core_outcome": None,
    }
    try:
        # The guard is the first operation that may inspect the graph. No controller is contacted before it.
        ops.guard(site.model)
        if {p.name for p in task.parameters} != bindings.keys():
            raise BackendFailure("S507", "bindings must name every task parameter exactly once")
        robot_params = [p.name for p in task.parameters if p.type == "robot"]
        if len(robot_params) != 1 or bindings[robot_params[0]] != site.robot:
            raise BackendFailure("S507", "this site supports exactly its declared robot")
        if task.preconditions:
            raise BackendFailure("S508", "preconditions need the phase 3 observation adapter")
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
                    ops.move(_grasp_pose(site, bindings[step.obj]))
                    ops.grip(True)
                    ops.attach(model)
                    held = model
                elif isinstance(step, ir.Place):
                    if held != _object(site, bindings[step.obj]):
                        raise BackendFailure("S506", "place requires a prior successful grasp of this object")
                    model = _object(site, bindings[step.obj])
                    ops.move(_pose(site, bindings[step.target]))
                    ops.grip(False)
                    ops.detach(model)
                    held = None
                elif isinstance(step, ir.WaitFor):
                    from robolang.sim.values import duration_ms_of
                    ops.wait_ms(int(duration_ms_of(step.duration)))
                else:
                    raise BackendFailure("S508", f"{step.KIND} is not supported by this backend yet")
            except BackendFailure:
                entry["status"] = "failed"
                raise
            entry["status"] = "succeeded"
            entry["t_end_ms"] = ops.now_ms()
    except BackendFailure as exc:
        record["state"] = "FAILED"
        record["failure"] = {"code": exc.code, "message": exc.message,
                             "step": record["steps"][-1]["index"] if record["steps"] else None}
    return record


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
