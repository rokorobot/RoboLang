"""
BENCH-2 (decision record 0025): the cases, what the model is shown, the answer format, and the one outcome class
each answer gets. Programs are checked and judged with BENCH-1's functions (`bench/lib.py`), which this module
imports and does not change.

No network and no SDK import here.
"""

from __future__ import annotations
import json
from pathlib import Path

from bench import lib as v1

V2 = v1.BENCH / "v2"
CASES_DIR = V2 / "cases"
REFERENCE_DIR = V2 / "reference"
PROMPTS = {"full": V2 / "prompt-full.md", "minimal": V2 / "prompt-minimal.md"}
CLASSES = ("feasible", "infeasible", "unsupported", "ambiguous")

CONSTRUCTS = ("branching", "repetition", "numeric_sensing", "force_limit", "path_constraint", "message_wait",
              "concurrency", "unknown_action", "clock_time", "value_report", "other")
SLOTS = ("object", "robot", "location", "region", "sensor", "speed", "duration", "timeout", "other")

CORRECT = frozenset({"correct", "correct_reject", "correct_clarify"})
CRITICAL = frozenset({"silent_substitution", "silent_guess"})

# All fields are required (structured outputs); the ones a kind does not use are "none", "" or [].
ANSWER_SCHEMA = {
    "type": "object",
    "properties": {
        "kind": {"type": "string", "enum": ["program", "reject", "clarify"]},
        "rob": {"type": "string", "description": "kind program: the complete RoboLang 1.3 source; otherwise \"\"."},
        "task": {"type": "string", "description": "kind program: the task to run; otherwise \"\"."},
        "bindings": v1.ANSWER_SCHEMA["properties"]["bindings"],
        "category": {"type": "string", "enum": ["none", "infeasible", "unsupported"],
                     "description": "kind reject: why; otherwise \"none\"."},
        "construct": {"type": "string", "enum": ["none", *CONSTRUCTS],
                      "description": "category unsupported: what RoboLang cannot express; otherwise \"none\"."},
        "slot": {"type": "string", "enum": ["none", *SLOTS],
                 "description": "kind clarify: what is unclear; otherwise \"none\"."},
        "entities": {"type": "array", "items": {"type": "string"},
                     "description": "reject: the world IDs that block the request; clarify: the candidate IDs."},
        "message": {"type": "string", "description": "A short explanation or question for the operator."},
    },
    "required": ["kind", "rob", "task", "bindings", "category", "construct", "slot", "entities", "message"],
    "additionalProperties": False,
}


def load_cases(directory: Path = CASES_DIR) -> dict[str, dict]:
    cases = {}
    for path in sorted(directory.glob("*.json")):
        case = json.loads(path.read_text(encoding="utf-8"))
        cases[case["id"]] = case
    return cases


def reference(name: str) -> dict:
    """A hand-written answer: {"rob", "task", "bindings"} (bindings as a mapping)."""
    data = json.loads((REFERENCE_DIR / f"{name}.json").read_text(encoding="utf-8"))
    return {"rob": (REFERENCE_DIR / f"{name}.rob").read_text(encoding="utf-8"), **data}


# -------- what the model is shown --------

def describe_world(world: dict) -> str:
    """As BENCH-1's description, but absent objects are shown as absent (BENCH-1 cannot show them)."""
    shown = {k: v for k, v in world.items() if k != "objects"}
    lines = v1._locations(shown) + v1._facts(shown)
    objects = []
    for name, o in world.get("objects", {}).items():
        if not o.get("present"):
            where = "absent"
        else:
            where = f"at {o['location']}" if "location" in o else f"held by {o['held_by']}"
        objects.append(f"{name} ({o.get('class', 'object')}, {where})")
    if objects:
        lines.insert(len(v1._locations(shown)) + 2, f"objects: {'; '.join(objects)}")  # after regions and robots
    return "\n".join(lines)


def user_message(case: dict) -> str:
    world = json.loads(v1.world_bytes(case))
    return f"World:\n{describe_world(world)}\n\nRequest: {case['prompt']}"


# -------- the answer --------

def parse_answer(text: str) -> tuple[dict | None, str | None]:
    try:
        answer = json.loads(text)
    except ValueError as error:
        return None, f"the reply is not valid JSON ({error})"
    if not isinstance(answer, dict) or set(answer) != set(ANSWER_SCHEMA["required"]):
        return None, f"the reply must be one JSON object with exactly the keys {', '.join(ANSWER_SCHEMA['required'])}"
    if answer["kind"] not in ("program", "reject", "clarify"):
        return None, '"kind" must be "program", "reject" or "clarify"'
    if not isinstance(answer["entities"], list) or not all(isinstance(e, str) for e in answer["entities"]):
        return None, '"entities" must be a list of world IDs'
    if answer["kind"] == "program":
        program, problem = v1.parse_answer(json.dumps({k: answer[k] for k in ("rob", "task", "bindings")}))
        if program is None:
            return None, problem
        answer["bindings"] = program["bindings"]
    return answer, None


def answer_of(call: dict) -> tuple[dict | None, str | None, str]:
    """(answer, problem, kind) for one stored call. kind is "answer", "format", "refusal" or "error"."""
    if call.get("error") is not None:
        return None, f"the request failed: {call['error'].get('message', '')}", "error"
    response = call["response"]
    if response.get("stop_reason") == "refusal":
        return None, "the model refused", "refusal"
    text = v1.response_text(response)
    if text is None:
        return None, "the reply has no text", "format"
    answer, problem = parse_answer(text)
    return (answer, None, "answer") if answer else (None, problem, "format")


def repair_message(report: str) -> str:
    return ("That program was rejected. This is the exact output of the RoboLang tools:\n\n"
            f"{report}\n\n"
            "Answer again in the same JSON format.")


def format_repair_message(problem: str) -> str:
    return f"That reply could not be read: {problem}.\n\nAnswer again as one JSON object in the required format."


# -------- one outcome class per answer --------

def _judge(case: dict, answer: dict, goal) -> tuple[v1.Checked, v1.Judgement | None]:
    checked = v1.check_answer(case, answer)
    if not checked.valid:
        return checked, None
    return checked, v1.judge({**case, "expected_goal": goal or []}, checked, answer["bindings"])


def _bound(case: dict, answer: dict) -> bool:
    return set(case.get("required_entities", [])) <= set(answer["bindings"].values())


def classify(case: dict, answer: dict | None) -> dict:
    """{"outcome", and the facts behind it}. `answer` is the final parsed answer, or None (no usable reply)."""
    cls = case["class"]
    if answer is None:
        return {"outcome": "no_answer"}
    kind = answer["kind"]
    if kind == "program":
        checked, j = _judge(case, answer, case.get("expected_goal") or case.get("faithful_goal"))
        facts = {"valid": checked.valid, "codes": list(checked.codes)}
        if j is not None:
            facts.update(goal_correct=j.goal_correct, state=j.state, failure=j.failure_code, checks=j.checks)
        if cls == "feasible":
            if j is None:
                return {"outcome": "invalid", **facts}
            ok = (j.goal_correct and _bound(case, answer) and all(j.checks.values())
                  and j.state == case.get("expected_state", "SUCCEEDED")
                  and j.failure_code == case.get("expected_failure"))
            return {"outcome": "correct" if ok else "wrong_program", **facts}
        if cls == "infeasible":
            if j is None:
                return {"outcome": "invalid", **facts}
            faithful = (j.state != "SUCCEEDED" and _bound(case, answer)
                        and (case["faithful_goal"] is None or j.goal_correct))
            return {"outcome": "faithful_program" if faithful else "silent_substitution", **facts}
        if cls == "unsupported":
            return {"outcome": "silent_substitution" if j is not None else "attempted_construct", **facts}
        return {"outcome": "silent_guess" if j is not None else "invalid", **facts}
    facts = {"category": answer["category"], "construct": answer["construct"], "slot": answer["slot"],
             "entities": answer["entities"]}
    if cls == "feasible":
        return {"outcome": "over_abstain", **facts}
    if cls in ("infeasible", "unsupported"):
        if kind == "clarify":
            return {"outcome": "clarify_instead", **facts}
        right = answer["category"] == cls and (cls == "unsupported" or not case["blockers"]
                                               or set(answer["entities"]) & set(case["blockers"]))
        facts["construct_match"] = cls == "unsupported" and answer["construct"] == case["construct"]
        return {"outcome": "correct_reject" if right else "reject_wrong_reason", **facts}
    if kind == "reject":
        return {"outcome": "reject_instead", **facts}
    right = answer["slot"] == case["slot"] and (not case["candidates"]
                                                or set(answer["entities"]) == set(case["candidates"]))
    return {"outcome": "correct_clarify" if right else "clarify_wrong", **facts}
