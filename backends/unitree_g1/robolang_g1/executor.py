"""Validated IR to robot operations, with the stop of decision 0038 adapted to a biped, and the run record.

The shape is the ROS 2 backend's executor (`backends/ros2/robolang_ros2/executor.py`): guard first, then the refusals
(everything the backend cannot run or observe is refused before any motion, with the ROS 2 backend's codes), then the
steps in order, then the record. What differs:

* the stop is a biped's: zero velocity, the arms held where they are through `arm_sdk`, the hands unchanged, the robot
  kept standing, confirmed from `lowstate` (decision 0046 U9, hazard H25). A fall (S516) sends nothing at all;
* **goal and precondition verdicts are phase 3**. A task with a precondition is refused (S508): its atoms cannot be
  observed yet. The goal is recorded as `not_evaluated`, and the run never says SUCCEEDED from completed steps alone:
  it ends `STEPS_COMPLETED` (exit status 4 of the CLI) or `FAILED`.
"""

from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path

from robolang import ir
from robolang.sim.messages import has_message

from . import site as site_mod

SCHEMA, SCHEMA_VERSION = "robolang.g1.run", "0.1"
GRASP_MODEL = "weld_after_hand_close"
STOP_CODES = {"cancelled": "S511", "expired": "S512", "revoked": "S513"}
STOP_MESSAGES = {
    "cancelled": "the run was cancelled",
    "expired": "the run's deadline passed",
    "revoked": "the run's authorization was revoked",
}
# The safe state of a biped (0046 U9): not power-off, which is falling.
SAFE_STATE = {"velocity": "zero", "arms": "hold", "hands": "unchanged", "balance": "kept"}
REALTIME_MIN = 0.95  # U13: a run in which a second falls more than 5% behind real time is not valid for acceptance


class BackendFailure(Exception):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


class Stop:
    """When the run must stop: a deadline on the simulation clock, or a request written to a stop file.

    The stop file holds `cancel` or `revoke`. `deadline_ms` is absolute simulation time; `after_ms` counts from
    `start`, which the executor calls once the guard has passed.
    """

    def __init__(self, deadline_ms: int | None = None, stop_file: str | None = None, after_ms: int | None = None):
        self.deadline_ms = deadline_ms
        self.after_ms = after_ms
        self.stop_file = Path(stop_file) if stop_file else None
        self._requested: str | None = None

    def start(self, now_ms: int) -> None:
        if self.after_ms is not None and self.deadline_ms is None:
            self.deadline_ms = now_ms + self.after_ms

    def request(self, reason: str) -> None:
        if reason not in STOP_CODES:
            raise ValueError(reason)
        self._requested = self._requested or reason

    def due(self, now_ms: int) -> str | None:
        """The reason to stop now, or None. A request wins over the deadline; the first reason sticks."""
        if self._requested is None and self.stop_file is not None:
            try:
                word = self.stop_file.read_text(encoding="utf-8").strip()
            except OSError:
                word = ""
            if word in ("cancel", "revoke"):
                self._requested = "cancelled" if word == "cancel" else "revoked"
        if self._requested is not None:
            return self._requested
        if self.deadline_ms is not None and now_ms >= self.deadline_ms:
            self._requested = "expired"
            return self._requested
        return None


def verify_policy(task: ir.Task) -> dict:
    """The `@verify` thresholds, or the RoboLang 1.3 defaults (0.9, 10 s). Phase 3 uses them; the record keeps them."""
    verify = task.verify
    from robolang.sim.values import duration_ms_of

    age = int(duration_ms_of(verify.max_age)) if verify and verify.max_age is not None else 10_000
    conf = str(verify.min_confidence) if verify and verify.min_confidence is not None else "0.9"
    return {"min_confidence": conf, "max_age_ms": age}


def resolve(task: ir.Task, bindings: dict[str, str], site: dict) -> dict[str, str]:
    """{parameter: site name} for the task's robot, objects and locations: the entity IDs mapped through the site's
    `bindings` (S507 if one is missing). The robot must be the site's. Parameters of other types (sensors, regions,
    numbers) are left out: a step or condition that uses one is refused later."""
    if {p.name for p in task.parameters} != bindings.keys():
        raise BackendFailure("S507", "bindings must name every task parameter exactly once")
    kinds = {"robot": "robots", "object": "objects", "location": "locations"}
    out = {}
    for param in task.parameters:
        kind = kinds.get(param.type)
        if kind is None:
            continue
        entity = bindings[param.name]
        try:
            out[param.name] = site["bindings"][kind][entity]
        except KeyError:
            raise BackendFailure("S507", f"the site has no {param.type} bound to {entity!r}") from None
    robots = [out[p.name] for p in task.parameters if p.type == "robot"]
    if len(robots) != 1 or robots[0] != site["robot"]["name"]:
        raise BackendFailure("S507", "this site supports exactly its declared robot")
    return out


def _check_goal_atoms(condition, names: dict[str, str], site: dict) -> None:
    """Every goal atom must be one a later phase can observe: a marked object, a location with a region or station."""
    if isinstance(condition, (ir.And, ir.Or)):
        for child in condition.conditions:
            _check_goal_atoms(child, names, site)
    elif isinstance(condition, ir.Not):
        _check_goal_atoms(condition.condition, names, site)
    elif isinstance(condition, (ir.ObjectAt, ir.Holding, ir.RobotAt, ir.RobotReady)):
        if isinstance(condition, (ir.ObjectAt, ir.Holding)):
            if not site_mod.obj(site, names[condition.obj]).get("marker"):
                raise BackendFailure("S507", f"no marker for object {condition.obj!r}")
        if isinstance(condition, ir.ObjectAt) and "region" not in site_mod.location(site, names[condition.location]):
            raise BackendFailure("S507", f"no region for location {condition.location!r}")
    else:
        raise BackendFailure("S508", f"{condition.KIND} has no observation in this backend")


def refuse(task: ir.Task, bindings: dict[str, str], site: dict) -> dict[str, str]:
    """Everything this backend cannot run or observe, refused before any motion. Returns {parameter: site name}."""
    names = _Names(resolve(task, bindings, site))
    if any(isinstance(step, ir.Measure) for _, step in ir.walk(task.steps)):
        raise BackendFailure("S509", "measure reads values, and this backend has no value sensors")
    if any(isinstance(step, ir.ReadClock) for _, step in ir.walk(task.steps)):
        raise BackendFailure(
            "S510",
            "a clock reading (measure r time, measure r elapsed) is refused: this backend has "
            "no wall clock, and `elapsed` needs value bindings it does not have",
        )
    if has_message(task):
        raise BackendFailure(
            "S515",
            "message_received is refused: this backend has no message service, and no "
            "channel or sender is declared for it",
        )
    if any(isinstance(step, ir.Do) for _, step in ir.walk(task.steps)) or task.goal_states:
        raise BackendFailure(
            "S508", "declared actions (do) and states (in_state) can be neither run nor observed by this backend"
        )
    if task.preconditions:
        raise BackendFailure(
            "S508",
            "preconditions are evaluated from observations in phase 3 of this backend; in "
            "phase 2 a task with a precondition is refused before any motion",
        )
    for condition in task.goal or ():
        _check_goal_atoms(condition, names, site)
    if task.region_limits:
        raise BackendFailure(
            "S508", "declared region limits (no_go_region, in_region) are not enforced by this backend"
        )
    if task.safety:
        raise BackendFailure("S508", "declared speed limits are not enforced by this backend")
    for step in task.steps:
        if not isinstance(step, (ir.Move, ir.Grasp, ir.Place, ir.WaitFor)):
            raise BackendFailure("S508", f"{step.KIND} is not supported by this backend yet")
        if ir.handler_of(step) is not None:
            raise BackendFailure("S508", "a failure handler is not supported by this backend yet")
        if isinstance(step, (ir.Move, ir.Grasp, ir.Place)) and names[step.robot] != site["robot"]["name"]:
            raise BackendFailure("S507", "step names a robot outside this site")
        if isinstance(step, (ir.Move, ir.Place)):
            _station_of(site, names[step.target])
        if isinstance(step, ir.Place) and "region" not in site_mod.location(site, names[step.target]):
            raise BackendFailure("S507", f"no region for location {step.target!r}")
        if isinstance(step, (ir.Grasp, ir.Place)) and not site_mod.obj(site, names[step.obj]).get("marker"):
            raise BackendFailure("S507", f"no marker for object {step.obj!r}")
    return names


class _Names(dict):
    """A name that is not a bound robot, object or location (a sensor or a region, say) is S507, not a KeyError."""

    def __missing__(self, key):
        raise BackendFailure("S507", f"{key!r} is not a robot, object or location this site binds")


def _station_of(site: dict, location: str) -> str:
    try:
        return site_mod.location(site, location)["station"]
    except StopIteration:
        raise BackendFailure("S507", f"no location {location!r} in the site") from None


def execute(task: ir.Task, bindings: dict[str, str], site: dict, ops, stop: Stop | None = None) -> dict:
    """Run a validated task. The steps run in order and stop at the first failure. The goal is not evaluated here.

    `stop` ends the run early (cancel, deadline, revocation; decision 0038 P7). After a stop or a failed step the
    robot is put in the biped's safe state, and the record's `stop` says why and what was done.
    """
    record = {
        "run_record_schema": SCHEMA,
        "run_record_schema_version": SCHEMA_VERSION,
        "mode": "simulation",
        "backend": {"kind": "unitree-g1-mujoco", "versions": None, "grasp_model": GRASP_MODEL, "stance_model": None},
        "task": ir.to_dict(task),
        "bindings": dict(sorted(bindings.items())),
        "site": {"name": site["name"], "schema": site["schema"]},
        "verify": verify_policy(task),
        "state": "FAILED",
        "failure": None,
        "preconditions": [
            {"index": i, "condition": ir.to_dict(c), "result": "not_evaluated"}
            for i, c in enumerate(task.preconditions, 1)
        ],
        "steps": [],
        "goal": [
            {"index": i, "condition": ir.to_dict(c), "result": "not_evaluated"}
            for i, c in enumerate(task.goal or (), 1)
        ],
        "goal_evaluated": False,
        "goal_note": "goal verdicts from observations are phase 3 of this backend: the goal was not evaluated, and "
        "completed steps are not a success",
        "observations": [],
        "stop": None,
        "realtime": None,
        "core_outcome": None,
    }
    status_before = None
    try:
        ops.guard(site["name"])  # the first operation: nothing is sent to the robot before it passes (S501)
        record["backend"]["versions"] = ops.versions()
        record["backend"]["stance_model"] = ops.identity.get("stance_model")
        status_before = ops.status()
        if stop is not None:
            stop.start(ops.now_ms())
        names = refuse(task, bindings, site)
        record["start_pose"] = [round(v, 4) for v in ops.pose]
        for index, step in enumerate(task.steps, 1):
            ops.check()
            entry = {"index": index, "kind": step.KIND, "status": "running", "t_start_ms": ops.now_ms()}
            wall = time.monotonic()
            record["steps"].append(entry)
            try:
                if isinstance(step, ir.Move):
                    entry["details"] = ops.move(_station_of(site, names[step.target]))
                elif isinstance(step, ir.Grasp):
                    entry["details"] = ops.grasp(names[step.obj])
                elif isinstance(step, ir.Place):
                    entry["details"] = ops.place(names[step.obj], names[step.target])
                else:  # WaitFor
                    from robolang.sim.values import duration_ms_of

                    ops.wait_ms(int(duration_ms_of(step.duration)))
            except BackendFailure:
                entry["status"] = "failed"
                entry["details"] = {"events": getattr(ops, "events", [])[-40:]}  # what the step did before it failed
                raise
            finally:
                entry["t_end_ms"] = ops.now_ms()
                entry["wall_s"] = round(time.monotonic() - wall, 2)
            entry["status"] = "succeeded"
        record["state"] = "STEPS_COMPLETED"
    except BackendFailure as exc:
        record["state"] = "FAILED"
        stepped = bool(record["steps"])
        record["failure"] = {
            "code": exc.code,
            "message": exc.message,
            "step": record["steps"][-1]["index"] if stepped else None,
        }
        if stepped:
            record["stop"] = _hold(ops, exc.code)
    if getattr(ops, "perc", None) is not None:
        record["observations"] = ops.observations
        record["pose_belief_end"] = [round(v, 4) for v in ops.pose]
    record["realtime"] = _realtime(status_before, ops)
    run_id = hashlib.sha256(
        json.dumps([record["task"], record["bindings"], record["steps"][:1]], sort_keys=True).encode()
    ).hexdigest()[:16]
    record["run_id"] = run_id
    return record


def _realtime(before: dict | None, ops) -> dict | None:
    """The stand-in's real-time report for this run (U13): the ratio over the run, whether any second in it fell more
    than
    5% behind real time or any step stalled, and the compute time per step. `realtime_valid` is False if so."""
    try:
        after = ops.status()
    except Exception:  # the record must still be written
        after = None
    if before is None or after is None:
        return None
    t0, t1 = before["sim_time"], after["sim_time"]
    wall = after["wall_elapsed"] - before["wall_elapsed"]
    slow = [w for w in after["slow_windows"] if t0 <= w[0] <= t1]
    stalls = after["stalls"] - before["stalls"]
    return {
        "realtime_valid": bool(not slow and stalls == 0),
        "ratio": round((t1 - t0) / wall, 4) if wall > 0 else None,
        "sim_s": round(t1 - t0, 2),
        "wall_s": round(wall, 2),
        "slow_seconds": slow,
        "stalls": stalls,
        "worst_second_ratio_overall": after["worst_second_ratio"],
        "step_compute_ms": after["step_compute_ms"],
        "gc": after.get("gc"),
        "camera": after.get("camera"),
    }


def _hold(ops, code: str) -> dict:
    """The safe state after a stop or a failed step: zero velocity, arms held, hands as they are, standing, confirmed
    from lowstate. After a fall (S516) nothing is sent."""
    reason = next((r for r, c in STOP_CODES.items() if c == code), "fallen" if code == "S516" else "step_failed")
    if code == "S516":
        return {
            "reason": reason,
            "code": code,
            "kind": "no_further_command",
            "safe_state": {"commands": "none: a fallen robot is not commanded"},
            "hold": "none sent",
            "confirmation": {"confirmed": False, "error": "the robot fell; nothing was sent"},
        }
    entry = {"reason": reason, "code": code, "kind": "controlled_stand", "safe_state": dict(SAFE_STATE)}
    try:
        entry["hold"] = ops.hold()
    except Exception as exc:  # the record must still be written; the cell's own stops remain the safety function
        entry["hold"] = f"failed: {exc}"
        entry["confirmation"] = {"confirmed": False, "error": "no hold was sent"}
        return entry
    try:  # safety case H18/H25: a hold that was sent is not yet a robot that stands still
        entry["confirmation"] = ops.confirm_stopped()
    except BackendFailure as exc:  # a fall while confirming
        entry["confirmation"] = {"confirmed": False, "error": f"{exc.code}: {exc.message}"}
    except Exception as exc:
        entry["confirmation"] = {"confirmed": False, "error": str(exc)}
    return entry
