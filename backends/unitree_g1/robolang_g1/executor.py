"""Validated IR to robot operations, with the biped stop of decision 0038, the verdicts, and the run record.

The shape is the ROS 2 backend's executor (`backends/ros2/robolang_ros2/executor.py`): guard first, then the refusals
(everything the backend cannot run or observe is refused before any motion, with the ROS 2 backend's codes), the
preconditions from one observation snapshot at the start pose (S101 false, S102 unknown), the steps in order, and the
goal from one snapshot after the last step (all true: `SUCCEEDED`; any false: S301; otherwise S302), then the record.
The verdicts come only from the robot's own interfaces (`observe.py`); the executor never reads ground truth. What
differs from the ROS 2 backend:

* the stop is a biped's: zero velocity, the arms held where they are through `arm_sdk`, the hands unchanged, the robot
  kept standing, confirmed from `lowstate` (decision 0046 U9, hazard H25). A fall (S516) sends nothing at all;
* the snapshots are taken with the robot as the plan left it, with **one exception** (0046 U8 exception, approved by the
  owner 2026-11-21): after the last step, when a goal atom names an object the robot holds, the arm shows the object to
  the head camera (the carry pose hangs it out of view) and goes back, if the path is shown clear of the site's declared
  tables and obstacles; the record's `observation_motion` says when, why, along which waypoints and with what
  clearance, and a motion that was refused (the atoms stay `unknown`) is recorded too.
"""

from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path

from robolang import ir
from robolang.sim.messages import has_message

from . import observe
from . import site as site_mod

SCHEMA, SCHEMA_VERSION = "robolang.g1.run", "0.2"  # 0.2: verdicts, snapshots, goal_evaluated
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
    """The `@verify` thresholds, or the RoboLang 1.3 defaults (0.9, 10 s), as the record keeps them."""
    policy = observe.Policy.of(task)
    return {"min_confidence": str(policy.min_confidence), "max_age_ms": policy.max_age_ms}


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


def _check_atoms(condition, names: dict[str, str], site: dict) -> None:
    """Every atom of a precondition or goal condition must be one this backend can observe: a marked object, a location
    with a region (`object_at`) or a station, and the site's robot."""
    if isinstance(condition, (ir.And, ir.Or)):
        for child in condition.conditions:
            _check_atoms(child, names, site)
    elif isinstance(condition, ir.Not):
        _check_atoms(condition.condition, names, site)
    elif isinstance(condition, (ir.ObjectAt, ir.Holding, ir.RobotAt, ir.RobotReady)):
        if isinstance(condition, (ir.ObjectAt, ir.Holding)):
            if not site_mod.obj(site, names[condition.obj]).get("marker"):
                raise BackendFailure("S507", f"no marker for object {condition.obj!r}")
        if isinstance(condition, ir.ObjectAt) and "region" not in site_mod.location(site, names[condition.location]):
            raise BackendFailure("S507", f"no region for location {condition.location!r}")
        if isinstance(condition, (ir.Holding, ir.RobotAt, ir.RobotReady)) and (
            names[condition.robot] != site["robot"]["name"]
        ):
            raise BackendFailure("S507", "condition names a robot outside this site")
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
    if not task.goal:
        raise BackendFailure("S508", "a task without a goal has nothing to observe: success is decided from the goal")
    for condition in (*task.preconditions, *task.goal):
        _check_atoms(condition, names, site)
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
    """Run a validated task. The preconditions are decided from one snapshot at the start pose, before any command that
    moves the robot (false: S101, unknown: S102). The steps run in order and stop at the first failure. The goal is
    decided from one snapshot after the last step: all true is `SUCCEEDED`, any false S301, otherwise S302, each with
    `step: null` and no stop (nothing is moving). Verdicts come from observations only.

    `stop` ends the run early (cancel, deadline, revocation; decision 0038 P7). After a stop or a failed step the
    robot is put in the biped's safe state, and the record's `stop` says why and what was done.
    """
    policy = observe.Policy.of(task)
    record = {
        "run_record_schema": SCHEMA,
        "run_record_schema_version": SCHEMA_VERSION,
        "mode": "simulation",
        # stance_model: when the robot walks or stands; arm_work_stance: while the arms work (0046 U4: `supported`
        # is a weld standing in for the robot's own balance, so a run with it never shows that the arm work balances)
        "backend": {"kind": "unitree-g1-mujoco", "versions": None, "grasp_model": GRASP_MODEL, "stance_model": None,
                    "arm_work_stance": site["robot"]["arm_stance"]},
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
        "snapshots": [],  # every observation snapshot, with each marker reading's confidence and age
        # arm motions made only to observe (0046 U8 avoids motion outside the plan): when, why, the path, the clearance
        "observation_motion": [],
        "observations": [],  # the markers the executor looked at while it moved and reached (not verdicts)
        "stop": None,
        "realtime": None,
        "core_outcome": None,
    }
    status_before, goal_snap, names = None, None, None
    try:
        ops.guard(site["name"])  # the first operation: nothing is sent to the robot before it passes (S501)
        record["backend"]["versions"] = ops.versions()
        record["backend"]["stance_model"] = ops.identity.get("stance_model")
        status_before = ops.status()
        if stop is not None:
            stop.start(ops.now_ms())
        names = refuse(task, bindings, site)
        record["start_pose"] = [round(v, 4) for v in ops.pose]
        if task.preconditions:  # one snapshot at the start pose: reading it commands nothing
            ready = observe.uses_ready(task.preconditions)
            snap = _snapshot(ops, record, "preconditions", frozenset(), ready, (), names)  # never an arm motion
            _verdicts(record["preconditions"], task.preconditions, names, site, snap, policy)
            results = [e["result"] for e in record["preconditions"]]
            if "false" in results:
                raise BackendFailure("S101", "a precondition is false from the observations")
            if "unknown" in results:
                raise BackendFailure("S102", "a precondition is unknown from the observations")
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
        # one snapshot after the last step; a held object the goal speaks of is shown to the camera first
        snap = _snapshot(ops, record, "goal", observe.held_objects(task.goal, names), observe.uses_ready(task.goal),
                         task.goal, names)
        goal_snap = snap
        record["goal_evaluated"] = True
        _verdicts(record["goal"], task.goal, names, site, snap, policy)
        results = [e["result"] for e in record["goal"]]
        if "false" in results:
            raise BackendFailure("S301", "a goal condition is false from the observations")
        if "unknown" in results:
            raise BackendFailure("S302", "a goal condition is unknown from the observations")
        record["state"] = "SUCCEEDED"
    except BackendFailure as exc:
        record["state"] = "FAILED"
        stepped = bool(record["steps"])
        decided = exc.code in ("S301", "S302")  # a verdict, not a failure to act: nothing is moving, so no stop
        record["failure"] = {
            "code": exc.code,
            "message": exc.message,
            "step": record["steps"][-1]["index"] if stepped and not decided else None,
        }
        if stepped and not decided:
            record["stop"] = _hold(ops, exc.code)
    if getattr(ops, "perc", None) is not None:
        record["observations"] = ops.observations
        record["pose_belief_end"] = [round(v, 4) for v in ops.pose]
    record["realtime"] = _realtime(status_before, ops)
    run_id = hashlib.sha256(
        json.dumps([record["task"], record["bindings"], record["steps"][:1]], sort_keys=True).encode()
    ).hexdigest()[:16]
    record["run_id"] = run_id
    if names is not None:
        record["core_outcome"] = observe.core_outcome(
            task, bindings, names, site, goal_snap, policy, record["state"], (record["failure"] or {}).get("code"),
            run_id,
        )
    return record


def _snapshot(ops, record: dict, purpose: str, show, ready: bool, conditions, names):
    """One observation snapshot, kept in the record so every verdict can be audited. If the robot moved its arm only to
    be able to observe (or refused to, `performed: false`), that is recorded apart from the plan's steps, with the atoms
    that asked for it."""
    snap, note = ops.snapshot(show=show, ready=ready)
    entry = observe.snapshot_record(snap, purpose)
    motion = note.get("observation_motion")
    if motion is not None:
        why = [
            ir.to_dict(a)
            for c in conditions
            for a in observe.atoms_of(c)
            if isinstance(a, (ir.Holding, ir.ObjectAt)) and names[a.obj] == motion["object"]
        ]
        record["observation_motion"].append({"phase": purpose, "why": why, **motion})
        entry["observation_motion"] = len(record["observation_motion"]) - 1
    record["snapshots"].append(entry)
    return snap


def _verdicts(entries: list, conditions, names, site: dict, snap, policy) -> None:
    """Each condition's three-valued result, and the numbers behind every atom of it (offsets, distances, why)."""
    for entry, condition in zip(entries, conditions):
        entry["result"] = observe.evaluate(condition, names, site, snap, policy)
        entry["atoms"] = observe.atoms_report([condition], names, site, snap, policy)


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
