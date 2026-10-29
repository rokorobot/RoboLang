"""
What `run.py` and `score.py` share: the tasks, the world description shown to a model, the answer format,
the checks on an answer (the same ones `robolang validate` and `robolang simulate` make), and the prices.

No network and no SDK import here. An answer is judged only with the installed `robolang` package.
"""

from __future__ import annotations
import io
import itertools
import json
import re
import tempfile
from collections import Counter
from dataclasses import dataclass
from decimal import Decimal
from math import prod
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
HARD_TASKS_DIR = BENCH / "tasks_hard"
HARD_REFERENCE_DIR = BENCH / "reference_hard"
SET3_TASKS_DIR = BENCH / "tasks_set3"
SET3_REFERENCE_DIR = BENCH / "reference_set3"
PROMPT_FILE = BENCH / "prompt.md"
REFERENCE_FILE = ROOT / "docs" / "language-reference.md"
MINIMAL_PROMPT_FILE = BENCH / "prompt_minimal.md"
GRAMMAR_FILE = ROOT / "spec" / "grammar-v1.3.ebnf"
GRAMMAR_MARK = "{{GRAMMAR}}"  # in prompt_minimal.md: replaced by the text of the grammar file
REFERENCE_MARK = "{{LANGUAGE_REFERENCE}}"  # in prompt.md: replaced by the shipped, tested reference

# The task sets: name -> (tasks directory, reference directory). Set 1 is the default and stays as it was.
SETS = {"1": (TASKS_DIR, REFERENCE_DIR), "hard": (HARD_TASKS_DIR, HARD_REFERENCE_DIR),
        "3": (SET3_TASKS_DIR, SET3_REFERENCE_DIR)}  # set 3: decision record 0025
PROMPTS = ("full", "minimal")

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
DECISIONS = ("program", "cannot", "clarify")
CATEGORIES = ("inexpressible", "infeasible", "none")
ANSWER_SCHEMA = {
    "type": "object",
    "properties": {
        "decision": {"type": "string", "enum": list(DECISIONS),
                     "description": "\"program\" to answer with RoboLang source, \"cannot\" to decline the request, "
                                    "\"clarify\" to ask which reading is meant."},
        "category": {"type": "string", "enum": list(CATEGORIES),
                     "description": "Why it cannot be done: \"inexpressible\" (RoboLang 1.3 cannot say it) or "
                                    "\"infeasible\" (this world or robot cannot do it). \"none\" for a program."},
        "reason": {"type": "string", "description": "One sentence: what the program does, or why it cannot be done."},
        "entities": {"type": "array", "items": {"type": "string"},
                     "description": "World entity IDs: for cannot, the ones that block the request; for clarify, every "
                                    "candidate the request could mean. Empty for a program."},
        "rob": {"type": "string", "description": "The complete RoboLang 1.3 source. Empty when decision is cannot."},
        "task": {"type": "string", "description": "The name of the task in the source to run. Empty when cannot."},
        "bindings": {  # a list of pairs: structured outputs require additionalProperties false on objects
            "type": "array",
            "description": "One entry per task parameter: the parameter name and the world entity ID bound to it. "
                           "Empty when decision is cannot.",
            "items": {
                "type": "object",
                "properties": {"param": {"type": "string"}, "id": {"type": "string"}},
                "required": ["param", "id"],
                "additionalProperties": False,
            },
        },
    },
    "required": ["decision", "category", "reason", "entities", "rob", "task", "bindings"],
    "additionalProperties": False,
}
OLD_ANSWER_KEYS = {"rob", "task", "bindings"}  # saved answers of benchmark set 1: read as decision "program"
SET2_ANSWER_KEYS = {"decision", "category", "reason", "rob", "task", "bindings"}  # saved before "entities": none

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


def load_set(name: str) -> dict[str, dict]:
    return load_tasks(SETS[name][0])


def load_prompt(name: str = "full") -> str:
    """A prompt assembled from its stable wrapper and the canonical grammar or language reference."""
    if name == "minimal":
        text = MINIMAL_PROMPT_FILE.read_text(encoding="utf-8")
        return text.replace(GRAMMAR_MARK, GRAMMAR_FILE.read_text(encoding="utf-8").rstrip("\n"))
    text = PROMPT_FILE.read_text(encoding="utf-8")
    return text.replace(REFERENCE_MARK, REFERENCE_FILE.read_text(encoding="utf-8").rstrip("\n"))


def expectation(task_def: dict) -> tuple[str, str]:
    """(decision, category) a task expects. Tasks without `expect` (set 1) expect a program."""
    expect = task_def.get("expect") or {}
    return expect.get("decision", "program"), expect.get("category", "none")


def expected_goals(task_def: dict) -> list[set[tuple[str, str, str]]]:
    """The acceptable goal sets: `expected_goals` (defensible readings), else `expected_goal`, else none."""
    sets = task_def["expected_goals"] if "expected_goals" in task_def else [task_def.get("expected_goal", [])]
    return [{tuple(t) for t in goals} for goals in sets if goals]


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
        where = "not present"  # a world object may be absent: it has neither a location nor a holder
        if "location" in o:
            where = f"at {o['location']}"
        elif "held_by" in o:
            where = f"held by {o['held_by']}"
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


def notes_for(task_def: dict) -> dict[str, str]:
    """Entity ID -> description, from `<world>.notes.json` next to the world file, if there is one."""
    path = (ROOT / task_def["world"]).with_suffix(".notes.json")
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}


def user_message(task_def: dict) -> str:
    world = json.loads(world_bytes(task_def))
    notes = notes_for(task_def)
    described = "".join(f"\n  {name}: {text}" for name, text in notes.items())
    return (f"World:\n{describe_world(world)}" + (f"\ndescriptions:{described}" if notes else "")
            + f"\n\nRequest: {task_def['prompt']}")


# -------- the answer --------

def parse_answer(text: str) -> tuple[dict | None, str | None]:
    """The answer object, or (None, what is wrong with the reply). The old three-key form is a program."""
    try:
        answer = json.loads(text)
    except ValueError as error:
        return None, f"the reply is not valid JSON ({error})"
    if isinstance(answer, dict) and set(answer) == OLD_ANSWER_KEYS:
        answer = {"decision": "program", "category": "none", "reason": "", **answer}
    if isinstance(answer, dict) and set(answer) == SET2_ANSWER_KEYS:
        answer = {"entities": [], **answer}
    if not isinstance(answer, dict) or set(answer) != set(ANSWER_SCHEMA["properties"]):
        return None, ('the reply must be one JSON object with exactly the keys "decision", "category", "reason", '
                      '"entities", "rob", "task" and "bindings"')
    if answer["decision"] not in DECISIONS or answer["category"] not in CATEGORIES \
            or not isinstance(answer["reason"], str) or not isinstance(answer["entities"], list) \
            or not all(isinstance(e, str) for e in answer["entities"]):
        return None, ('"decision" must be "program", "cannot" or "clarify", "category" "inexpressible", "infeasible" '
                      'or "none", "reason" a string and "entities" a list of entity IDs')
    bindings = answer["bindings"]
    if isinstance(bindings, list):  # the wire form: [{"param", "id"}, ...]; read back as a mapping
        pairs = [(b.get("param"), b.get("id")) if isinstance(b, dict) and set(b) == {"param", "id"} else (None, None)
                 for b in bindings]
        names = [name for name, _ in pairs]
        if (None, None) in pairs or len(set(names)) != len(names):
            return None, '"bindings" must list each parameter once, as {"param": ..., "id": ...}'
        bindings = answer["bindings"] = dict(pairs)
    if (not isinstance(answer["rob"], str) or not isinstance(answer["task"], str) or not isinstance(bindings, dict)
            or not all(isinstance(k, str) and isinstance(v, str) for k, v in bindings.items())):
        return None, '"rob" and "task" must be strings and every binding a parameter name and an entity ID'
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


def _validate_source(rob: str, world: bytes | None = None, task: str = "", bindings: dict | None = None):
    """(exit status as text, stderr) of `robolang validate answer.rob`, with `--world` when `world` is given."""
    with tempfile.TemporaryDirectory() as directory:
        path, world_path = Path(directory) / "answer.rob", Path(directory) / "world.json"
        path.write_text(rob, encoding="utf-8")
        argv = ["validate", str(path)]
        if world is not None:
            world_path.write_bytes(world)
            argv += ["--world", str(world_path), "--task", task]
            argv += ["--bind", *(f"{k}={v}" for k, v in bindings.items())] if bindings else []
        err = io.StringIO()
        status = robolang_cli(argv, io.StringIO(), err)
    return str(status), err.getvalue().replace(str(path), "answer.rob").replace(str(world_path), "world.json")


def _world_command(answer: dict) -> str:
    return "robolang validate answer.rob --world world.json --task " + answer["task"] + " --bind " \
        + " ".join(f"{k}={v}" for k, v in answer["bindings"].items())


def check_answer(task_def: dict, answer: dict, world_data: bytes | None = None) -> Checked:
    """Source valid, task present, and `validate --world` passes under the bindings: what `simulate` needs to start,
    without a failure the run would certainly hit (speed limit vs profile, gripper, sensor mount: N2)."""
    status, stderr = _validate_source(answer["rob"])
    if status != "0":
        return Checked(False, f"robolang validate answer.rob\n{stderr}".rstrip(), _codes(stderr))
    result = task_ir.compile_source(answer["rob"], "answer.rob")
    task = result.ir.task(answer["task"])
    if task is None:
        names = ", ".join(t.name for t in result.ir.tasks)
        report = f"error[{NO_TASK_CODE}]: no task named '{answer['task']}' in answer.rob; tasks in this file: {names}"
        return Checked(False, report, (NO_TASK_CODE,))
    data = world_data or world_bytes(task_def)
    try:
        world = load_world(data, "world.json", task, answer["bindings"])
    except WorldError as error:
        report = _world_command(answer) + f"\n{error.format()}"
        return Checked(False, report, (error.code,))
    world_status, world_stderr = _validate_source(answer["rob"], data, answer["task"], answer["bindings"])
    if world_status != "0":
        return Checked(False, f"{_world_command(answer)}\n{world_stderr}".rstrip(), _codes(world_stderr))
    return Checked(True, "", _codes(stderr), task, world, result.ir.language_version)


# -------- judging a valid answer --------

@dataclass
class Judgement:
    goal_correct: bool
    plan_succeeds: bool
    state: str
    failure_code: str | None
    checks: dict[str, bool]             # the optional checks the task defines, by name
    goal_exact: bool = False            # the exported triples equal an expected set (goal_correct before N6)


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


# -------- goals compared by meaning --------

MAX_GOAL_STATES = 200_000  # beyond this many abstract states, goals are compared by their triples only
_ELSEWHERE = ("", "")  # the value of a subject that is in none of the states the goals mention


def _goal_formula(condition: task_ir.Condition, bindings: dict[str, str]):
    """A goal condition as a formula: ("atom", key), ("and" | "or", parts) or ("not", part).

    An outcome atom's key is its Core triple; any other condition (robot_ready, ...) is a free proposition."""
    triple = bridge.triple_of(condition, bindings)
    if triple is not None:
        return "atom", (triple.subject, triple.property, triple.value)
    if isinstance(condition, (task_ir.And, task_ir.Or)):
        return condition.KIND, tuple(_goal_formula(c, bindings) for c in condition.conditions)
    if isinstance(condition, task_ir.Not):
        return "not", _goal_formula(condition.condition, bindings)
    return "atom", ("?", repr(condition), "")


def _atoms(formula, out: set) -> set:
    kind, body = formula
    if kind == "atom":
        out.add(body)
    elif kind == "not":
        _atoms(body, out)
    else:
        for part in body:
            _atoms(part, out)
    return out


def _holds(formula, state: dict) -> bool:
    kind, body = formula
    if kind == "atom":
        return state[body] if body[0] == "?" else state[body[0]] == body[1:]
    if kind == "not":
        return not _holds(body, state)
    return (all if kind == "and" else any)(_holds(part, state) for part in body)


def goal_means(task: task_ir.Task, bindings: dict[str, str], readings: list[set[tuple[str, str, str]]]) -> bool | None:
    """Whether the goal holds in exactly the states where one expected reading holds, or where any of them does.

    A state gives each subject (thing or robot) one value: a place it is located in, a robot carrying it, or
    elsewhere; so `holding r b` and `object_at b t` exclude each other, as in the simulator. Only the subjects and
    values the formulas mention can change their truth, so enumerating those is exhaustive. None when there is no
    goal or no reading, or too many states."""
    if task.goal is None or not readings:
        return None
    goal = ("and", tuple(_goal_formula(c, bindings) for c in task.goal))
    expected = [("and", tuple(("atom", t) for t in sorted(r))) for r in readings]
    candidates = expected + ([("or", tuple(expected))] if len(expected) > 1 else [])
    domains: dict = {}
    for atom in _atoms(goal, set()).union(*(_atoms(f, set()) for f in expected)):
        if atom[0] == "?":
            domains[atom] = [False, True]
        elif atom[1:] not in domains.setdefault(atom[0], [_ELSEWHERE]):
            domains[atom[0]].append(atom[1:])
    if prod(len(values) for values in domains.values()) > MAX_GOAL_STATES:
        return None
    for combo in itertools.product(*domains.values()):
        state = dict(zip(domains, combo))
        truth = _holds(goal, state)
        candidates = [c for c in candidates if _holds(c, state) == truth]
        if not candidates:
            return False
    return True


def judge(task_def: dict, checked: Checked, bindings: dict[str, str]) -> Judgement:
    """Goal, simulation and the task's optional checks, for an answer that `check_answer` accepted."""
    try:
        exported = {(t.subject, t.property, t.value) for t in bridge.goal_triples(checked.task, bindings)}
    except bridge.NotExportable:
        exported = None
    result = simulate_task(checked.task, checked.world)
    checks = {}
    if "expected_safety" in task_def:
        checks["safety"] = _check_safety(task_def, checked)
    if "required_entities" in task_def:  # the robot, object or sensor the request named is the one bound
        checks["entities"] = set(task_def["required_entities"]) <= set(bindings.values())
    if "expected_messages" in task_def:
        sent = {(s.message.channel, s.message.message) for s in result.steps if s.message}
        checks["messages"] = {tuple(m) for m in task_def["expected_messages"]} <= sent
    if "expected_message_counts" in task_def:  # [channel, message, how many times]: a repeated action really repeats
        sent_n = Counter((s.message.channel, s.message.message) for s in result.steps if s.message)
        checks["message_counts"] = all(sent_n[(c, m)] == n for c, m, n in task_def["expected_message_counts"])
    if "expected_end_ms" in task_def:
        checks["end_ms"] = result.end_t_ms == Decimal(task_def["expected_end_ms"])
    if "probes" in task_def:
        checks["probes"] = all(_check_probe(task_def, checked, p, bindings) for p in task_def["probes"])
    goal_exact = exported is not None and exported in expected_goals(task_def)
    meaning = goal_means(checked.task, bindings, expected_goals(task_def))
    return Judgement(goal_exact if meaning is None else meaning, result.state == "SUCCEEDED",
                     result.state, result.failure.code if result.failure else None, checks, goal_exact)


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
            "Answer again as one JSON object with the keys \"decision\", \"category\", \"reason\", "
            "\"entities\", \"rob\", \"task\" and \"bindings\".")


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
