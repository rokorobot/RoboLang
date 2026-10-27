"""
The run record and the run ID of the reference simulation (ARCH-3; spec/reference-simulation-v0.1.md
section 9).

The record is enough to reconstruct a run: every message with its fate and every receipt it
got (a held message gets a `held` receipt and a second one when it is released), and every
event. It holds codes and structured values only. Times are integer milliseconds after the
scenario's epoch. The JSON text follows the IR determinism rules: fixed key order, two-space
indentation, ASCII only, LF, one trailing newline.

SIMULATION ONLY: every record says `"mode": "simulation"`; nothing in it is evidence about a
real robot, network or signature scheme.
"""

from __future__ import annotations
import hashlib
import json
import re

from ..core import time_ms
from .engine import RunResult, run_case
from .scenario import load_scenario

RUN_RECORD_SCHEMA = "robolang.arch02.run"
RUN_RECORD_SCHEMA_VERSION = "0.1"
CORE_VERSION = "0.1"
SEED = re.compile(r"[A-Za-z0-9._-]{1,64}")


def sha256_normalized(data: bytes) -> str:
    """Lowercase hex SHA-256 without one leading UTF-8 BOM and with CR LF read as LF (section 9.1)."""
    if data.startswith(b"\xef\xbb\xbf"):
        data = data[3:]
    return hashlib.sha256(data.replace(b"\r\n", b"\n")).hexdigest()


def run_id(scenario: bytes, case: str, seed: str) -> str:
    """The ID of an input-equivalent reference run (section 9.1)."""
    preimage = f"robolang-arch02-run/0.1\n{sha256_normalized(scenario)}\n{case}\n{seed}\n"
    return "arch02-" + hashlib.sha256(preimage.encode("utf-8")).hexdigest()


def to_record(result: RunResult, *, scenario: bytes, epoch: str) -> dict:
    """The run record 0.1 of `result`, as a dict with the key order of section 9."""
    origin = time_ms(epoch)
    messages = []
    for sent in result.sent:
        receipts = [d for d in result.deliveries if d.message == sent.index]
        first = next((d for d in receipts if d.kind == "original"), None)
        messages.append({
            "t_sent_ms": sent.t_sent_ms - origin,
            "t_delivered_ms": None if first is None else first.t_ms - origin,
            "fate": sent.fate,
            "document": sent.document,
            "receipts": [
                {"t_ms": d.t_ms - origin, "receiver": d.receiver, "kind": d.kind, "receipt": d.receipt, "code": d.code}
                for d in receipts
            ],
        })
    goals, custody, commitments = dict(result.goal_states), dict(result.custody), dict(result.commitments)
    participants = sorted(set(goals) | set(custody) | set(commitments))
    return {
        "run_record_schema": RUN_RECORD_SCHEMA,
        "run_record_schema_version": RUN_RECORD_SCHEMA_VERSION,
        "mode": "simulation",
        "run_id": run_id(scenario, result.case, result.seed),
        "inputs": {
            "scenario_sha256": sha256_normalized(scenario),
            "case": result.case,
            "seed": result.seed,
            "core_version": CORE_VERSION,
        },
        "messages": messages,
        "events": [
            {"t_ms": e.t_ms - origin, "kind": e.kind, "participant": e.participant, "detail": dict(e.detail)}
            for e in result.events
        ],
        "final": {
            "planner_status": result.planner_status,
            "participants": {
                pid: {
                    "goal_state": goals.get(pid),
                    "commitment_state": commitments.get(pid),
                    "custody": custody.get(pid),
                }
                for pid in participants
            },
        },
        "designated_custody": dict(result.designated_custody),
        "final_world": [{"subject": f.subject, "property": f.property, "value": f.value} for f in result.final_world],
        "end_t_ms": result.end_t_ms - origin,
    }


def to_json(record: dict) -> str:
    """Deterministic JSON text of a run record."""
    return json.dumps(record, indent=2, ensure_ascii=True) + "\n"


def simulate_case(scenario: bytes, case: dict, seed: str | None = None) -> dict:
    """
    Run one case against the scenario file's bytes and return its run record.

    `seed` replaces the case's seed; it only distinguishes runs of identical inputs.
    """
    if seed is not None:
        if not SEED.fullmatch(seed):
            raise ValueError("a seed is 1 to 64 characters from A-Z a-z 0-9 . _ -")
        case = dict(case, seed=seed)
    case = dict(case, seed=case.get("seed", "0"))
    parsed = load_scenario(scenario)
    return to_record(run_case(parsed, case), scenario=scenario, epoch=parsed["parameters"]["epoch"])
