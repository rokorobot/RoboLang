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
from .values import ms_text
from .world import WORLD_SCHEMA_VERSION

RUN_RECORD_SCHEMA = "robolang.sim.run"
RUN_RECORD_SCHEMA_VERSION = "1.0"
# Language version -> run record schema version (robolang-v1.2.md section 8.3; v1.3 section 8.3).
RUN_RECORD_SCHEMA_VERSIONS = {"1.1": "1.0", "1.2": "1.1", "1.3": "1.2", "1.4": "1.3", "1.5": "1.4", "1.6": "1.5"}
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


def _reading(reading) -> dict | None:
    """A measured value (RoboLang 1.6, decision 0036 V8). A label has no `unit` and no `uncertainty` key."""
    if reading is None:
        return None
    quantity = reading.unit is not None
    return {
        "subject": reading.subject,
        "kind": reading.kind,
        "value": ir.decimal_text(reading.value) if quantity else reading.value,
        **({"unit": reading.unit, "uncertainty": ir.decimal_text(reading.uncertainty)} if quantity else {}),
        "sensor": reading.sensor,
        "t_ms": ms_text(reading.t_ms),
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
        quantity = prop.unit is not None
        return {**out, "property": prop.kind,
                "value": {"value": ir.decimal_text(prop.value) if quantity else prop.value,
                          **({"unit": prop.unit} if quantity else {})}}
    return {**out, "value": event.value}


def _step(step, branching: bool, handlers: bool = False, values: bool = False) -> dict:
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


def _final_world(world) -> dict:
    return {
        "robots": {
            k: {"ready": v.ready, "location": v.location, "holding": v.holding} for k, v in world.robots
        },
        "objects": {
            k: {"present": v.present, "location": v.location, "held_by": v.held_by} for k, v in world.objects
        },
        "regions": {k: {"clear": v.clear} for k, v in world.regions},
        "sensors": {k: {"robot": v.robot, "available": v.available} for k, v in world.sensors},
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
) -> dict:
    """
    The run record (section 5) as a dict with the key order of the specification: schema 1.0
    for a RoboLang 1.1 source, 1.1 for a 1.2 source (same shape, embedding Task IR 1.1), 1.2 for a
    1.3 source (adds `goal`, after `preconditions`, and `core_outcome`, last), 1.3 for a 1.4 source (adds
    `decisions` after `goal`, a `path` on every step, a `from` on every goal entry, a `reason` on `failure`), 1.4 for
    a 1.5 source (adds `handled` after `status` on every step: the code of a failure a handler took over, or null),
    1.5 for a 1.6 source (adds `reading` after `observation` on every step, and `value` after `status` on every
    message: the typed value of a measurement, or null).
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
    if bindings_source != "world":  # a record without the key has world-file bindings (section 5.1)
        inputs["bindings_source"] = bindings_source
    branching = language_version in ("1.4", "1.5", "1.6")
    handlers = language_version in ("1.5", "1.6")
    values = language_version == "1.6"
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
        "steps": [_step(s, branching, handlers, values) for s in result.steps],
        "events_applied": [_event(e) for e in result.events_applied],
        "end_t_ms": ms_text(result.end_t_ms),
        "final_world": _final_world(result.final_world),
        **({} if result.goal is None else {"core_outcome": core_outcome(result, task, run_id, core=core)}),
    }


def to_json(record: dict) -> str:
    """Deterministic JSON text of a run record (section 5.5)."""
    return json.dumps(record, indent=2, ensure_ascii=True) + "\n"
