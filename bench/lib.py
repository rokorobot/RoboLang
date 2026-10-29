"""
What `run.py` and `score.py` share: the tasks, the world description shown to a model, the answer format,
the checks on an answer (the same ones `robolang validate` and `robolang simulate` make), and the prices.

No network and no SDK import here. An answer is judged only with the installed `robolang` package.
"""

from __future__ import annotations
import io
import json
import re
import tempfile
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path

from robolang import bridge
from robolang import ir as task_ir
from robolang.cli import main as robolang_cli
from robolang.sim import World, WorldError, load_world
from robolang.sim import run as simulate_task

ROOT = Path(__file__).resolve().parent.parent
BENCH = ROOT / "bench"
TASKS_DIR = BENCH / "tasks"
REFERENCE_DIR = BENCH / "reference"
PROMPT_FILE = BENCH / "prompt.md"

# $ per million tokens: (input, output, cache read). A cache write (5 minutes) costs 1.25 x input.
# From the claude-api skill's model table (cached 2026-10-06). The one place prices live.
PRICES = {
    "claude-fable-5-1": (10.00, 50.00, 0.25),
    "claude-fable-5": (10.00, 50.00, 1.00),
    "claude-opus-5-5": (4.00, 20.00, 0.20),
    "claude-opus-5": (5.00, 25.00, 0.50),
    "claude-opus-4-8": (5.00, 25.00, 0.50),
    "claude-opus-4-7": (5.00, 25.00, 0.50),
    "claude-opus-4-6": (5.00, 25.00, 0.50),
    "claude-sonnet-5-5": (2.00, 10.00, 0.20),
    "claude-sonnet-5": (2.00, 10.00, 0.20),
    "claude-sonnet-4-6": (3.00, 15.00, 0.30),
    "claude-haiku-5-5": (0.10, 0.50, 0.01),
    "claude-haiku-4-5": (1.00, 5.00, 0.10),
}
CACHE_WRITE_FACTOR = 1.25

# The answer: structured output (`output_config.format`), then read back by `parse_answer`.
ANSWER_SCHEMA = {
    "type": "object",
    "properties": {
        "rob": {"type": "string", "description": "The complete RoboLang 1.3 source."},
        "task": {"type": "string", "description": "The name of the task in the source to run."},
        "bindings": {
            "type": "object",
            "description": "Task parameter name -> world entity ID.",
            "additionalProperties": {"type": "string"},
        },
    },
    "required": ["rob", "task", "bindings"],
    "additionalProperties": False,
}

# Pseudo codes for what is not a `robolang` diagnostic. Real codes (E..., W..., S...) are never changed.
FORMAT_CODE = "B001"      # the reply was not an answer in the required format
NO_TASK_CODE = "B002"     # the named task is not in the source
REFUSED_CODE = "B003"     # the model refused (stop_reason "refusal"): no answer
API_ERROR_CODE = "B004"   # the request failed: no answer

_CODE = re.compile(r"(?:error|warning)\[([A-Z]\d{3})\]")


def load_tasks(directory: Path = TASKS_DIR) -> dict[str, dict]:
    """Task ID -> task definition, sorted by ID."""
    tasks = {}
    for path in sorted(directory.glob("*.json")):
        task = json.loads(path.read_text(encoding="utf-8"))
        tasks[task["id"]] = task
    return tasks


def world_bytes(task_def: dict) -> bytes:
    return (ROOT / task_def["world"]).read_bytes()


# -------- what the model is shown --------

def _locations(world: dict) -> list[str]:
    return [f"locations: {', '.join(world['locations'])}"]


def _facts(world: dict) -> list[str]:
    lines = []
    regions = world.get("regions", {})
    if regions:
        shown = [f"{n} ({'clear' if r.get('clear') else 'not clear'})" for n, r in regions.items()]
        lines.append(f"regions: {', '.join(shown)}")
    robots = []
    for name, r in world.get("robots", {}).items():
        speeds = ", ".join(f"{q['value']} {q['unit']}" for q in r.get("max_speed", []))
        parts = [f"at {r['location']}", "has a gripper" if r["gripper"] else "no gripper",
                 "ready" if r.get("ready") else "not ready"]
        if speeds:
            parts.append(f"max speed {speeds}")
        robots.append(f"{name} ({', '.join(parts)})")
    if robots:
        lines.append(f"robots: {'; '.join(robots)}")
    objects = []
    for name, o in world.get("objects", {}).items():
        where = f"at {o['location']}" if "location" in o else f"held by {o['held_by']}"
        objects.append(f"{name} ({o.get('class', 'object')}, {where})")
    if objects:
        lines.append(f"objects: {'; '.join(objects)}")
    sensors = [f"{n} (on {s['robot']}, {'available' if s.get('available') else 'not available'})"
               for n, s in world.get("sensors", {}).items()]
    if sensors:
        lines.append(f"sensors: {'; '.join(sensors)}")
    for event in world.get("events", []):
        at = event["at"]
        state = "true" if event["value"] else "false"
        when = f"{at['value']} {at['unit']}"
        lines.append(f"scheduled: after {when}, {event['fact']} of {event['target']} becomes {state}")
    return lines


def describe_world(world: dict) -> str:
    """The world file as the operator would describe it: the entity IDs a binding may use, and their state."""
    return "\n".join(_locations(world) + _facts(world))


def user_message(task_def: dict) -> str:
    world = json.loads(world_bytes(task_def))
    return f"World:\n{describe_world(world)}\n\nRequest: {task_def['prompt']}"


# -------- the answer --------

def parse_answer(text: str) -> tuple[dict | None, str | None]:
    """The answer object, or (None, what is wrong with the reply)."""
    try:
        answer = json.loads(text)
    except ValueError as error:
        return None, f"the reply is not valid JSON ({error})"
    if not isinstance(answer, dict) or set(answer) != {"rob", "task", "bindings"}:
        return None, 'the reply must be one JSON object with exactly the keys "rob", "task" and "bindings"'
    bindings = answer["bindings"]
    if (not isinstance(answer["rob"], str) or not isinstance(answer["task"], str) or not isinstance(bindings, dict)
            or not all(isinstance(k, str) and isinstance(v, str) for k, v in bindings.items())):
        return None, '"rob" and "task" must be strings and "bindings" an object of strings'
    return answer, None


def response_text(response: dict | None) -> str | None:
    """The first text block of a response, or None."""
    for block in (response or {}).get("content", []):
        if block.get("type") == "text":
            return block.get("text", "")
    return None


@dataclass
class Checked:
    """The result of the checks `robolang validate` and `robolang simulate` would make on an answer."""

    valid: bool
    report: str = ""                    # the exact diagnostics, for a repair round
    codes: tuple[str, ...] = ()
    task: task_ir.Task | None = None
    world: World | None = None
    language_version: str = ""


def _codes(text: str) -> tuple[str, ...]:
    return tuple(_CODE.findall(text))


def _validate_source(rob: str) -> tuple[str, str]:
    """(exit status as text, stderr) of `robolang validate answer.rob` on `rob`."""
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "answer.rob"
        path.write_text(rob, encoding="utf-8")
        err = io.StringIO()
        status = robolang_cli(["validate", str(path)], io.StringIO(), err)
    return str(status), err.getvalue().replace(str(path), "answer.rob")


def check_answer(task_def: dict, answer: dict, world_data: bytes | None = None) -> Checked:
    """Source valid, task present, bindings accepted by the world: what `simulate` needs to start."""
    status, stderr = _validate_source(answer["rob"])
    if status != "0":
        return Checked(False, f"robolang validate answer.rob\n{stderr}".rstrip(), _codes(stderr))
    result = task_ir.compile_source(answer["rob"], "answer.rob")
    task = result.ir.task(answer["task"])
    if task is None:
        names = ", ".join(t.name for t in result.ir.tasks)
        report = f"error[{NO_TASK_CODE}]: no task named '{answer['task']}' in answer.rob; tasks in this file: {names}"
        return Checked(False, report, (NO_TASK_CODE,))
    try:
        world = load_world(world_data or world_bytes(task_def), "world.json", task, answer["bindings"])
    except WorldError as error:
        report = "robolang simulate answer.rob --world world.json --bind " \
                 + " ".join(f"{k}={v}" for k, v in answer["bindings"].items()) + f"\n{error.format()}"
        return Checked(False, report, (error.code,))
    return Checked(True, "", _codes(stderr), task, world, result.ir.language_version)


# -------- judging a valid answer --------

@dataclass
class Judgement:
    goal_correct: bool
    plan_succeeds: bool
    state: str
    failure_code: str | None
    checks: dict[str, bool]             # the optional checks the task defines, by name


def _merge(base: dict, patch: dict) -> dict:
    """`base` with `patch` laid over it: objects merge key by key, anything else is replaced."""
    merged = dict(base)
    for key, value in patch.items():
        merged[key] = _merge(base[key], value) if isinstance(value, dict) and isinstance(base.get(key), dict) else value
    return merged


def _safety_set(limits) -> set[tuple[str, str, str | None]]:
    return {(s.kind, task_ir.decimal_text(s.limit.value), s.limit.unit) for s in limits}


def _check_safety(task_def: dict, checked: Checked) -> bool:
    expected = {(s["kind"], task_ir.decimal_text(Decimal(s["value"])), s["unit"]) for s in task_def["expected_safety"]}
    return _safety_set(checked.task.safety) == expected


def _check_probe(task_def: dict, checked: Checked, probe: dict, bindings: dict[str, str]) -> bool:
    """A correct plan must not succeed in the changed world: it noticed what the request said about it."""
    patched = _merge(json.loads(world_bytes(task_def)), probe["world_patch"])
    try:
        world = load_world(json.dumps(patched).encode("utf-8"), "world.json", checked.task, bindings)
        return simulate_task(checked.task, world).state != "SUCCEEDED"
    except WorldError:
        return False


def judge(task_def: dict, checked: Checked, bindings: dict[str, str]) -> Judgement:
    """Goal, simulation and the task's optional checks, for an answer that `check_answer` accepted."""
    expected = {tuple(t) for t in task_def["expected_goal"]}
    try:
        exported = {(t.subject, t.property, t.value) for t in bridge.goal_triples(checked.task, bindings)}
    except bridge.NotExportable:
        exported = None
    result = simulate_task(checked.task, checked.world)
    checks = {}
    if "expected_safety" in task_def:
        checks["safety"] = _check_safety(task_def, checked)
    if "expected_messages" in task_def:
        sent = {(s.message.channel, s.message.message) for s in result.steps if s.message}
        checks["messages"] = {tuple(m) for m in task_def["expected_messages"]} <= sent
    if "expected_end_ms" in task_def:
        checks["end_ms"] = result.end_t_ms == Decimal(task_def["expected_end_ms"])
    if "probes" in task_def:
        checks["probes"] = all(_check_probe(task_def, checked, p, bindings) for p in task_def["probes"])
    return Judgement(exported == expected, result.state == "SUCCEEDED",
                     result.state, result.failure.code if result.failure else None, checks)


# -------- a model's reply --------

def answer_of(call: dict) -> tuple[dict | None, str | None, str]:
    """(answer, problem, kind) for one stored call. kind is "answer", "format", "refusal" or "error"."""
    if call.get("error") is not None:
        return None, f"the request failed: {call['error'].get('message', '')}", "error"
    response = call["response"]
    if response.get("stop_reason") == "refusal":
        return None, "the model refused", "refusal"
    text = response_text(response)
    if text is None:
        return None, "the reply has no text", "format"
    answer, problem = parse_answer(text)
    return (answer, None, "answer") if answer else (None, problem, "format")


def repair_message(report: str) -> str:
    return ("That answer was rejected. This is the exact output of the RoboLang tools:\n\n"
            f"{report}\n\n"
            "Fix the problem and answer again in the same format: the complete corrected source, "
            "the task name and the bindings.")


def format_repair_message(problem: str) -> str:
    return (f"That reply could not be read: {problem}.\n\n"
            "Answer again as one JSON object with the keys \"rob\", \"task\" and \"bindings\".")


def served_by(call: dict) -> dict:
    """Which model produced the reply, and whether it was a fallback (server-side refusal fallback)."""
    response = call.get("response") or {}
    requested = call["request"]["model"]
    usage = response.get("usage") or {}
    ran = any(i.get("type") == "fallback_message" for i in usage.get("iterations") or [])
    ran = ran or any(b.get("type") == "fallback" for b in response.get("content", []))
    model = response.get("model") or requested
    return {"model": model, "fallback": ran or not model.startswith(requested)}


TOKEN_FIELDS = ("input_tokens", "cache_creation_input_tokens", "cache_read_input_tokens", "output_tokens")


def attempts(call: dict) -> list[tuple[str, dict]]:
    """(model, usage) per attempt: `usage.iterations` when the API gives it, else the top-level usage."""
    response = call.get("response") or {}
    usage = response.get("usage") or {}
    requested = call["request"]["model"]
    if usage.get("iterations"):
        served = response.get("model") or requested
        return [(i.get("model") or (served if i.get("type") == "fallback_message" else requested), i)
                for i in usage["iterations"] if i.get("type") in ("message", "fallback_message")]
    return [(response.get("model") or requested, usage)] if usage else []


def call_tokens(call: dict) -> dict[str, int]:
    totals = dict.fromkeys(TOKEN_FIELDS, 0)
    for _, usage in attempts(call):
        for name in TOKEN_FIELDS:
            totals[name] += usage.get(name) or 0
    return totals


def usd(model: str, tokens: dict[str, int]) -> float | None:
    """Cost of `tokens` at `model`'s price; None for a model without a price."""
    if model not in PRICES:
        return None
    price_in, price_out, price_read = PRICES[model]
    return (tokens["input_tokens"] * price_in + tokens["cache_creation_input_tokens"] * price_in * CACHE_WRITE_FACTOR
            + tokens["cache_read_input_tokens"] * price_read + tokens["output_tokens"] * price_out) / 1_000_000


def call_cost(call: dict) -> float | None:
    """Dollars for one call, summed over its attempts at each attempt's own model; None if any is unpriced."""
    costs = []
    for model, usage in attempts(call):
        costs.append(usd(model, {name: usage.get(name) or 0 for name in TOKEN_FIELDS}))
    return None if None in costs else sum(costs)
