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
from .values import ms_text
from .world import WORLD_SCHEMA_VERSION

RUN_RECORD_SCHEMA = "robolang.sim.run"
RUN_RECORD_SCHEMA_VERSION = "1.0"
# Language version -> run record schema version (robolang-v1.2.md section 8.3; v1.3 section 8.3).
RUN_RECORD_SCHEMA_VERSIONS = {"1.1": "1.0", "1.2": "1.1", "1.3": "1.2"}
SEED = re.compile(r"[A-Za-z0-9._-]{1,64}")
DEFAULT_SEED = "0"


def sha256_normalized(data: bytes) -> str:
    """Lowercase hex SHA-256 of `data` without one leading UTF-8 BOM and with CR LF read as LF (section 6)."""
    if data.startswith(b"\xef\xbb\xbf"):
        data = data[3:]
    return hashlib.sha256(data.replace(b"\r\n", b"\n")).hexdigest()


def run_id(source: bytes, world: bytes, task: str, seed: str, cli_bindings: Mapping[str, str] | None = None) -> str:
    """The ID of an input-equivalent simulation run (section 6). `--bind` bindings extend the preimage."""
    preimage = f"robolang-sim-run/1.0\n{sha256_normalized(source)}\n{sha256_normalized(world)}\n{task}\n{seed}\n"
    if cli_bindings is not None:
        preimage += "".join(f"bind {name}={target}\n" for name, target in sorted(cli_bindings.items()))
    return "sim-" + hashlib.sha256(preimage.encode("ascii")).hexdigest()


def _time(value) -> str | None:
    return None if value is None else ms_text(value)


def _failure(failure) -> dict | None:
    if failure is None:
        return None
    return {
        "code": failure.code,
        "phase": failure.phase,
        "step": failure.step,
        "precondition": failure.precondition,
        "safety_limit": failure.safety_limit,
        "robot": failure.robot,
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


def _message(message) -> dict | None:
    if message is None:
        return None
    return {
        "robot": message.robot,
        "channel": message.channel,
        "message": message.message,
        "status": message.status,
    }


def _step(step) -> dict:
    return {
        "index": step.index,
        "step": ir.to_dict(step.step),
        "status": step.status,
        "t_start_ms": _time(step.t_start_ms),
        "t_end_ms": _time(step.t_end_ms),
        "observation": _observation(step.observation),
        "message": _message(step.message),
        "condition_result": None if step.condition_result is None else step.condition_result.value,
    }


def _results(results) -> list[dict]:
    """Per-condition results of the `pre` or `goal` block."""
    return [
        {
            "index": p.index,
            "condition": ir.to_dict(p.condition),
            "result": "not_evaluated" if p.result is None else p.result.value,
        }
        for p in results
    ]


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
) -> dict:
    """
    The run record (section 5) as a dict with the key order of the specification: schema 1.0
    for a RoboLang 1.1 source, 1.1 for a 1.2 source (same shape, embedding Task IR 1.1), 1.2 for a
    1.3 source (adds `goal`, after `preconditions`).
    """
    inputs = {
        "source_sha256": source_sha256,
        "world_sha256": world_sha256,
        "task": result.task_name,
        "seed": seed,
        "language_version": language_version,
        "ir_schema_version": ir.IR_SCHEMA_VERSIONS[language_version],
        "world_schema_version": world_schema_version,
    }
    if bindings_source != "world":  # a record without the key has world-file bindings (section 5.1)
        inputs["bindings_source"] = bindings_source
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
        "failure": _failure(result.failure),
        "cancelled_before_step": result.cancelled_before_step,
        "safety_limits": [{"limit": ir.to_dict(r.limit), "status": r.status} for r in result.safety_limits],
        "preconditions": _results(result.preconditions),
        **({} if result.goal is None else {"goal": _results(result.goal)}),
        "steps": [_step(s) for s in result.steps],
        "events_applied": [
            {"t_ms": ms_text(e.t_ms), "fact": e.fact, "target": e.target, "value": e.value}
            for e in result.events_applied
        ],
        "end_t_ms": ms_text(result.end_t_ms),
        "final_world": _final_world(result.final_world),
    }


def to_json(record: dict) -> str:
    """Deterministic JSON text of a run record (section 5.5)."""
    return json.dumps(record, indent=2, ensure_ascii=True) + "\n"
