"""
The run record and the run ID (spec/simulation-v1.0.md sections 5 and 6).

The record holds codes and structured values only, in a fixed key order. `to_json` follows the
IR determinism rules: two-space indentation, ASCII only, LF, one trailing newline.
"""

from __future__ import annotations
import hashlib
import json
import re
from collections.abc import Mapping

from .. import ir
from .executor import RunResult
from .outcome import core_outcome
from ..tokens import CLOCK_KINDS
from .clock import clock_text
from .values import ms_text
from .world import WORLD_SCHEMA_VERSION

RUN_RECORD_SCHEMA = "robolang.sim.run"
RUN_RECORD_SCHEMA_VERSION = "1.0"
# Language version -> run record schema version (robolang-v1.2.md section 8.3; v1.3 section 8.3).
RUN_RECORD_SCHEMA_VERSIONS = {
    "1.1": "1.0", "1.2": "1.1", "1.3": "1.2", "1.4": "1.3", "1.5": "1.4", "1.6": "1.5", "1.7": "1.6",
    "1.8": "1.7", "1.9": "1.8", "2.0": "1.8", "2.1": "1.9", "2.2": "1.10",
}
MISSION_RECORD_SCHEMA = "robolang.sim.mission-run"  # RoboLang 2.0 (decision 0041): the record of a mission
MISSION_RECORD_SCHEMA_VERSION = "1.0"
MISSION_RECORD_SCHEMA_VERSIONS = {"2.0": "1.0", "2.1": "1.1", "2.2": "1.2"}  # 1.1 (2.1) adds `messages`; 1.2: 2.2
SEED = re.compile(r"[A-Za-z0-9._-]{1,64}")
DEFAULT_SEED = "0"


def sha256_normalized(data: bytes) -> str:
    """Lowercase hex SHA-256 of `data` without one leading UTF-8 BOM and with CR LF read as LF (section 6)."""
    if data.startswith(b"\xef\xbb\xbf"):
        data = data[3:]
    return hashlib.sha256(data.replace(b"\r\n", b"\n")).hexdigest()


def run_id(source: bytes, world: bytes, task: str, seed: str, cli_bindings: Mapping[str, str] | None = None,
           source_kind: str = "source") -> str:
    """
    The ID of an input-equivalent simulation run (section 6). `--bind` bindings extend the preimage, and
    so does a Task IR input (`source_kind` "ir"): a source file and an IR file never share an ID.
    """
    kind = "" if source_kind == "source" else f"source_kind {source_kind}\n"
    preimage = (f"robolang-sim-run/1.0\n{kind}{sha256_normalized(source)}\n{sha256_normalized(world)}\n"
                f"{task}\n{seed}\n")
    if cli_bindings is not None:
        preimage += "".join(f"bind {name}={target}\n" for name, target in sorted(cli_bindings.items()))
    return "sim-" + hashlib.sha256(preimage.encode("ascii")).hexdigest()


def mission_run_id(source: bytes, world: bytes, mission: str, seed: str, cli_bindings: Mapping[str, str] | None = None,
                   source_kind: str = "source") -> str:
    """The ID of an input-equivalent mission run: as `run_id`, with the mission named in its own preimage."""
    kind = "" if source_kind == "source" else f"source_kind {source_kind}\n"
    preimage = (f"robolang-sim-mission/1.0\n{kind}{sha256_normalized(source)}\n{sha256_normalized(world)}\n"
                f"{mission}\n{seed}\n")
    if cli_bindings is not None:
        preimage += "".join(f"bind {name}={target}\n" for name, target in sorted(cli_bindings.items()))
    return "sim-" + hashlib.sha256(preimage.encode("ascii")).hexdigest()


def _time(value) -> str | None:
    return None if value is None else ms_text(value)


def _failure(failure, branching: bool) -> dict | None:
    if failure is None:
        return None
    return {
        "code": failure.code,
        "phase": failure.phase,
        "step": failure.step,
        "precondition": failure.precondition,
        "safety_limit": failure.safety_limit,
        "robot": failure.robot,
        **({"reason": failure.reason} if branching else {}),
    }


def _observation(observation) -> dict | None:
    if observation is None:
        return None
    return {
        "object": observation.object,
        "present": observation.present,
        "location": observation.location,
        "held_by": observation.held_by,
    }


def _received(message) -> dict:
    """RoboLang 2.1 (decision 0043 S6): a message that arrived, and whether it counted."""
    return {
        "t_ms": ms_text(message.t_ms),
        "channel": message.channel,
        "text": message.text,
        "from": message.sender,
        "status": "accepted" if message.accepted else "ignored",
    }


def _reading(reading) -> dict | None:
    """
    A measured value (RoboLang 1.6, decision 0036 V8). A label has no `unit` and no `uncertainty` key; a count (2.2) has
    an `uncertainty` of "0" and no `unit`.
    """
    if reading is None:
        return None
    quantity = reading.uncertainty is not None
    return {
        "subject": reading.subject,
        "kind": reading.kind,
        "value": ir.decimal_text(reading.value) if quantity else reading.value,
        **({"unit": reading.unit} if reading.unit is not None else {}),
        **({"uncertainty": ir.decimal_text(reading.uncertainty)} if quantity else {}),
        "sensor": reading.sensor,
        "t_ms": ms_text(reading.t_ms),
        **({"text": clock_text(reading.kind, reading.value)} if reading.kind in CLOCK_KINDS else {}),
    }


def _message(message, values: bool = False) -> dict | None:
    if message is None:
        return None
    return {
        "robot": message.robot,
        "channel": message.channel,
        "message": message.message,
        "status": message.status,
        **({"value": _reading(message.value)} if values else {}),
    }


def _event(event) -> dict:
    out = {"t_ms": ms_text(event.t_ms), "fact": event.fact, "target": event.target}
    if event.fact == "property":  # world file 1.2: the object's property took this value
        prop = event.value
        quantity = not isinstance(prop.value, str)  # a label is text; a count is a number with no unit
        return {**out, "property": prop.kind,
                "value": {"value": ir.decimal_text(prop.value) if quantity else prop.value,
                          **({"unit": prop.unit} if prop.unit is not None else {})}}
    return {**out, "value": event.value}


def _value(value) -> dict | None:
    """The value of a `do` as the IR writes it (a quantity or a text), or None."""
    return None if value is None else ir.to_dict(value)


def _action(request) -> dict | None:
    """RoboLang 1.8 (decision 0040 A7): what a `do` requested. The effect is never claimed here, only requested."""
    if request is None:
        return None
    return {
        "robot": request.robot,
        "action": request.action,
        "target": request.target,
        "value": _value(request.value),
        "result": request.result,
        "status": request.status,
    }


def _step(step, branching: bool, handlers: bool = False, values: bool = False, actions: bool = False) -> dict:
    # An `if` is written without its branches, and a step without its failure handler: they are steps of their
    # own, in the same array.
    if isinstance(step.step, ir.If):
        shown = {"kind": "if", "condition": ir.to_dict(step.step.condition)}
    else:
        shown = ir.to_dict(step.step)
        shown.pop("else", None)
    return {
        "index": step.index,
        **({"path": step.path} if branching else {}),
        "step": shown,
        "status": step.status,
        **({"handled": step.handled} if handlers else {}),
        "t_start_ms": _time(step.t_start_ms),
        "t_end_ms": _time(step.t_end_ms),
        "observation": _observation(step.observation),
        **({"reading": _reading(step.reading)} if values else {}),
        "message": _message(step.message, values),
        **({"action": _action(step.action)} if actions else {}),
        "condition_result": None if step.condition_result is None else step.condition_result.value,
    }


def _results(results, branching: bool = False) -> list[dict]:
    """Per-condition results of the `pre` or `goal` block; a 1.4 goal says where each condition came from."""
    return [
        {
            "index": p.index,
            **({"from": p.source} if branching else {}),
            "condition": ir.to_dict(p.condition),
            "result": "not_evaluated" if p.result is None else p.result.value,
        }
        for p in results
    ]


def _decision(decision) -> dict:
    return {
        "step": decision.step,
        "condition_result": decision.condition_result.value,
        "branch": decision.branch,
        "t_ms": ms_text(decision.t_ms),
    }


def _final_world(world, actions: bool = False) -> dict:
    """`actions` (a 1.8 record): each object also has its `state`, and `location_states` follows `sensors`."""
    return {
        "robots": {
            k: {"ready": v.ready, "location": v.location, "holding": v.holding} for k, v in world.robots
        },
        "objects": {
            k: {"present": v.present, "location": v.location, "held_by": v.held_by,
                **({"state": v.state} if actions else {})} for k, v in world.objects
        },
        "regions": {k: {"clear": v.clear} for k, v in world.regions},
        "sensors": {k: {"robot": v.robot, "available": v.available} for k, v in world.sensors},
        **({"location_states": dict(world.location_states)} if actions else {}),
    }


def to_record(
    result: RunResult,
    *,
    task: ir.Task,
    run_id: str,
    source_sha256: str,
    world_sha256: str,
    seed: str,
    language_version: str = ir.LANGUAGE_VERSION,
    bindings_source: str = "world",
    world_schema_version: str = WORLD_SCHEMA_VERSION,
    source_kind: str = "source",
    core: str = "0.1",
    world_clock=None,
) -> dict:
    """
    The run record (section 5) as a dict with the key order of the specification: schema 1.0
    for a RoboLang 1.1 source, 1.1 for a 1.2 source (same shape, embedding Task IR 1.1), 1.2 for a
    1.3 source (adds `goal`, after `preconditions`, and `core_outcome`, last), 1.3 for a 1.4 source (adds
    `decisions` after `goal`, a `path` on every step, a `from` on every goal entry, a `reason` on `failure`), 1.4 for
    a 1.5 source (adds `handled` after `status` on every step: the code of a failure a handler took over, or null),
    1.5 for a 1.6 source (adds `reading` after `observation` on every step, and `value` after `status` on every
    message: the typed value of a measurement, or null), 1.6 for a 1.7 source (the same shape: it embeds Task IR 1.6,
    and `safety_limits` may hold region limits with the status `checked_against_world_not_enforced`), 1.7 for a 1.8
    source (adds `action` after `message` on every step: what a `do` requested, with the status `requested`, or null;
    `state` on every `final_world` object, and `final_world.location_states`), 1.8 for a 1.9 source (embeds Task IR
    1.8; `inputs.world_clock` after `world_schema_version`: the world's `start` and `zone`, or null; a reading of
    `time` or `elapsed` has a `text` after `t_ms`, as it is sent in a message: `HH:MM:SS` or `37 s`), 1.9 for a 2.1
    source (embeds Task IR 1.9; `messages` after `events_applied`: every message that arrived from the start of the run
    to its end, each with `t_ms`, `channel`, `text`, `from` and `status` "accepted" or "ignored").
    """
    inputs = {
        "source_sha256": source_sha256,
        **({} if source_kind == "source" else {"source_kind": source_kind}),  # only for IR input (section 5.1)
        "world_sha256": world_sha256,
        "task": result.task_name,
        "seed": seed,
        "language_version": language_version,
        "ir_schema_version": ir.IR_SCHEMA_VERSIONS[language_version],
        "world_schema_version": world_schema_version,
    }
    if language_version in ("1.9", "2.0", "2.1", "2.2"):
        inputs["world_clock"] = None if world_clock is None else {"start": world_clock.start, "zone": world_clock.zone}
    if bindings_source != "world":  # a record without the key has world-file bindings (section 5.1)
        inputs["bindings_source"] = bindings_source
    branching = language_version in ("1.4", "1.5", "1.6", "1.7", "1.8", "1.9", "2.0", "2.1", "2.2")
    handlers = language_version in ("1.5", "1.6", "1.7", "1.8", "1.9", "2.0", "2.1", "2.2")
    values = language_version in ("1.6", "1.7", "1.8", "1.9", "2.0", "2.1", "2.2")
    actions = language_version in ("1.8", "1.9", "2.0", "2.1", "2.2")
    return {
        "run_record_schema": RUN_RECORD_SCHEMA,
        "run_record_schema_version": RUN_RECORD_SCHEMA_VERSIONS[language_version],
        "mode": "simulation",
        "run_id": run_id,
        "inputs": inputs,
        "task": ir.to_dict(task),
        "bindings": dict(result.bindings),
        "states": list(result.states),
        "state": result.state,
        "failure": _failure(result.failure, branching),
        "cancelled_before_step": result.cancelled_before_step,
        "safety_limits": [{"limit": ir.to_dict(r.limit), "status": r.status} for r in result.safety_limits],
        "preconditions": _results(result.preconditions),
        **({} if result.goal is None else {"goal": _results(result.goal, branching)}),
        **({"decisions": [_decision(d) for d in result.decisions]} if branching else {}),
        "steps": [_step(s, branching, handlers, values, actions) for s in result.steps],
        "events_applied": [_event(e) for e in result.events_applied],
        **({"messages": [_received(m) for m in result.messages]} if language_version in ("2.1", "2.2") else {}),
        "end_t_ms": ms_text(result.end_t_ms),
        "final_world": _final_world(result.final_world, actions),
        **({} if result.goal is None else {"core_outcome": core_outcome(result, task, run_id, core=core)}),
    }


def to_mission_record(
    result,
    *,
    document: ir.TaskIR,
    run_id: str,
    source_sha256: str,
    world_sha256: str,
    seed: str,
    bindings_source: str = "world",
    world_schema_version: str = WORLD_SCHEMA_VERSION,
    source_kind: str = "source",
    core: str = "0.1",
    world_clock=None,
) -> dict:
    """
    The mission run record, schema 1.0 (spec/simulation-v1.0.md section 18): the mission, the outcome and why the
    mission ended, and every task run in order with its own run record (what `to_record` holds, less the parts the
    mission holds once: the inputs, the task IR and the final world). A run's `core_outcome` is the outcome of that run.
    """
    mission = result.mission
    inputs = {
        "source_sha256": source_sha256,
        **({} if source_kind == "source" else {"source_kind": source_kind}),
        "world_sha256": world_sha256,
        "mission": mission.name,
        "seed": seed,
        "language_version": document.language_version,
        "ir_schema_version": ir.MISSION_IR_SCHEMA_VERSIONS[document.language_version],
        "task_ir_schema_version": document.ir_schema_version,
        "world_schema_version": world_schema_version,
        "world_clock": None if world_clock is None else {"start": world_clock.start, "zone": world_clock.zone},
    }
    if bindings_source != "world":
        inputs["bindings_source"] = bindings_source
    failure = result.failure
    runs = []
    for entry in result.runs:
        per_run_id = "sim-" + hashlib.sha256(f"{run_id}\nrun {entry.index}".encode("ascii")).hexdigest()
        body = to_record(entry.result, task=entry.task, run_id=per_run_id, source_sha256=source_sha256,
                         world_sha256=world_sha256, seed=seed, language_version=document.language_version,
                         world_schema_version=world_schema_version, source_kind=source_kind, core=core,
                         world_clock=world_clock)
        for key in ("run_record_schema", "run_record_schema_version", "mode", "run_id", "inputs", "task",
                    "final_world"):
            body.pop(key)
        runs.append({
            "index": entry.index,
            "line": entry.line,
            "attempt": entry.attempt,
            "task_name": entry.task.name,
            "run_id": per_run_id,
            "t_start_ms": ms_text(entry.t_start_ms),
            "t_end_ms": ms_text(entry.result.end_t_ms),
            "until": entry.until,
            **body,
        })
    return {
        "mission_record_schema": MISSION_RECORD_SCHEMA,
        "mission_record_schema_version": MISSION_RECORD_SCHEMA_VERSIONS[document.language_version],
        "run_record_schema_version": RUN_RECORD_SCHEMA_VERSIONS[document.language_version],
        "mode": "simulation",
        "run_id": run_id,
        "inputs": inputs,
        "mission": ir.to_dict(mission),
        "tasks": [ir.to_dict(task) for task in {r.task.name: r.task for r in result.runs}.values()],
        "bindings": dict(result.bindings),
        "outcome": result.outcome,
        "ended_by": result.ended_by,
        "failure": None if failure is None else {"code": failure.code, "run": failure.run, "message": failure.message},
        "runs_started": len(result.runs),
        "runs_verified": result.runs_verified,
        "runs_failed": result.runs_failed,
        "runs": runs,
        "events_applied": [_event(e) for e in result.events_applied],
        **({"messages": [_received(m) for m in result.messages]}
           if document.language_version in ("2.1", "2.2") else {}),
        "end_t_ms": ms_text(result.end_t_ms),
        "final_world": _final_world(result.final_world, True),
    }


def to_json(record: dict) -> str:
    """Deterministic JSON text of a run record (section 5.5)."""
    return json.dumps(record, indent=2, ensure_ascii=True) + "\n"
