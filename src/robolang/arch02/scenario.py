"""
Scenario loading for the reference simulation (spec/reference-simulation-v0.1.md sections 3, 10, 11).

The scenario file is read strictly, `override` faults are applied by JSON Pointer, and any
descriptor or standing authorization they change is signed again by its signer, so that an
override changes content, not signature validity.
"""

from __future__ import annotations
import copy
import time as _time
from typing import Any

from ..core import Key, read_document, sign_sim, time_ms

SCENARIO = ("robolang.arch02.scenario", "0.1")
_UNIT_MS = {"ms": 1, "s": 1000, "min": 60000}


class ScenarioError(ValueError):
    """The scenario file or a case does not have the form the specification requires."""


def ms(quantity: dict) -> int:
    """Milliseconds of a duration quantity. Simulation times are whole milliseconds."""
    value = quantity["value"]
    if "." in value:
        raise ScenarioError(f"durations in a scenario are whole numbers: {value}")
    return int(value) * _UNIT_MS[quantity["unit"]]


def time_text(t_ms: int) -> str:
    """A Core time with millisecond precision."""
    seconds, millis = divmod(t_ms, 1000)
    return _time.strftime("%Y-%m-%dT%H:%M:%S", _time.gmtime(seconds)) + f".{millis:03d}Z"


def _resolve_parent(document, pointer: str):
    tokens = [t.replace("~1", "/").replace("~0", "~") for t in pointer.split("/")[1:]]
    if not tokens:
        raise ScenarioError("an override needs a non-empty path")
    node = document
    for token in tokens[:-1]:
        node = node[int(token)] if isinstance(node, list) else node[token]
    last = tokens[-1]
    if isinstance(node, list):
        last = int(last)
        node[last]  # must exist
    elif last not in node:
        raise ScenarioError(f"override path does not exist: {pointer}")
    return node, last, tokens


def apply_overrides(scenario: dict, faults: list[dict]) -> dict:
    """A copy of `scenario` with every `override` applied and changed documents re-signed."""
    result = copy.deepcopy(scenario)
    changed: set[tuple[str, str]] = set()
    for fault in faults:
        if fault["kind"] != "override":
            continue
        parent, last, tokens = _resolve_parent(result, fault["path"])
        parent[last] = copy.deepcopy(fault["value"])
        if tokens[0] in ("descriptors", "standing_authorizations"):
            changed.add((tokens[0], tokens[1]))
    keys = keys_of(result)
    for collection, owner in sorted(changed):
        document = result[collection][owner]
        key_ref = document["signature"]["key"]
        result[collection][owner] = sign_sim(document, key_ref, keys[key_ref])
    return result


def counterparts(scenario: dict, participant: str) -> list[str]:
    """The participants it deals with: its peers (the robots it assists) and its own authority."""
    peers = sorted(robot for robot, assistants in scenario["peers"].items() if participant in assistants)
    return peers + [scenario["authorities"][participant]]


def trusted_key_refs(scenario: dict, participant: str) -> list[str]:
    """
    The keys of an assistant's trust context (spec 5.3, decision record 0013): its own key, then the
    keys of its counterparts. The gate 2 view (6.4) supplies exactly these; the engine verifies with them.
    """
    records = {entry["id"]: entry for entry in scenario["participants"]}
    return [f"{participant}#k1"] + [ref for pid in counterparts(scenario, participant) for ref in records[pid]["keys"]]


def keys_of(scenario: dict) -> dict[str, Key]:
    return {
        ref: Key(entry["participant"], entry["alg"], bytes.fromhex(entry["secret"]))
        for ref, entry in scenario["keys"].items()
    }


def load_scenario(data: bytes) -> dict[str, Any]:
    scenario = read_document(data)
    if not isinstance(scenario, dict) or (
        scenario.get("scenario_schema"), scenario.get("scenario_schema_version")
    ) != SCENARIO:
        raise ScenarioError("not a robolang.arch02.scenario 0.1 document")
    time_ms(scenario["parameters"]["epoch"])
    return scenario
