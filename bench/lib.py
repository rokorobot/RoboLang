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
from dataclasses import dataclass, fields, is_dataclass, replace
from decimal import Decimal
from math import prod
from pathlib import Path

from robolang import bridge
from robolang import ir as task_ir
from robolang.cli import main as robolang_cli
from robolang.measured import to_base
from robolang.sim import World, WorldError, load_world, mission_task, run_mission
from robolang.sim import run as simulate_task
from robolang.sim.messages import atoms as message_atoms
from robolang.sim.values import duration_ms_of
from robolang.tokens import KIND_UNITS_V2_2

ROOT = Path(__file__).resolve().parent.parent
BENCH = ROOT / "bench"
TASKS_DIR = BENCH / "tasks"
REFERENCE_DIR = BENCH / "reference"
HARD_TASKS_DIR = BENCH / "tasks_hard"
HARD_REFERENCE_DIR = BENCH / "reference_hard"
BRANCH_TASKS_DIR = BENCH / "tasks_branch"  # set 3: pre-registered against 1.3, scored against 1.4 (per path)
BRANCH_REFERENCE_DIR = BENCH / "reference_branch"
ASK_TASKS_DIR = BENCH / "tasks_ask"
ASK_REFERENCE_DIR = BENCH / "reference_ask"
FAILURE_TASKS_DIR = BENCH / "tasks_failure"  # set 4: pre-registered against 1.4, labelled for 1.5 (decision 0035)
FAILURE_REFERENCE_DIR = BENCH / "reference_failure"
VALUES_TASKS_DIR = BENCH / "tasks_values"  # set 5: measured values, pre-registered against 1.5 (decision 0036)
VALUES_REFERENCE_DIR = BENCH / "reference_values"
FUTURE_TASKS_DIR = BENCH / "tasks_future"  # set 6: probes for the atlas's near language gaps (RSI-01 R3)
FUTURE_REFERENCE_DIR = BENCH / "reference_future"
REGIONS_TASKS_DIR = BENCH / "tasks_regions"  # set 6: region limits, pre-registered against 1.6, labelled for 1.7 (0039)
REGIONS_REFERENCE_DIR = BENCH / "reference_regions"
ACTIONS_TASKS_DIR = BENCH / "tasks_actions"  # set 7: declared actions, pre-registered against 1.7, labelled for 1.8
ACTIONS_REFERENCE_DIR = BENCH / "reference_actions"
CLOCK_TASKS_DIR = BENCH / "tasks_clock"  # set 8: clock readings, pre-registered against 1.8, labelled for 1.9
CLOCK_REFERENCE_DIR = BENCH / "reference_clock"
MISSIONS_TASKS_DIR = BENCH / "tasks_missions"  # set 9: missions, pre-registered against 1.9 (decision 0041)
MISSIONS_REFERENCE_DIR = BENCH / "reference_missions"
MESSAGES_TASKS_DIR = BENCH / "tasks_messages"  # set 10: incoming messages, pre-registered against 2.0 (decision 0043)
MESSAGES_REFERENCE_DIR = BENCH / "reference_messages"
KINDS_TASKS_DIR = BENCH / "tasks_kinds"  # set 11: more measured kinds, pre-registered on 2.1, labelled for 2.2 (0044)
KINDS_REFERENCE_DIR = BENCH / "reference_kinds"
# set 12: choosing at run time, pre-registered on 2.2, labelled for 2.3 (0045)
CHOOSE_TASKS_DIR = BENCH / "tasks_choose"
CHOOSE_REFERENCE_DIR = BENCH / "reference_choose"
PROMPT_FILE = BENCH / "prompt.md"
REFERENCE_FILE = ROOT / "docs" / "language-reference.md"
MINIMAL_PROMPT_FILE = BENCH / "prompt_minimal.md"
# 2.3 is a delta over 2.2, over 2.1, over 2.0, over 1.9, over 1.8, over 1.7, over 1.6, over 1.5, over 1.4, over 1.3: the
# grammar-only prompt pastes all eleven
GRAMMAR_FILE = ROOT / "spec" / "grammar-v2.3.ebnf"
BASE_GRAMMAR_FILE = ROOT / "spec" / "grammar-v1.3.ebnf"
GRAMMAR_FILES = (BASE_GRAMMAR_FILE, ROOT / "spec" / "grammar-v1.4.ebnf", ROOT / "spec" / "grammar-v1.5.ebnf",
                 ROOT / "spec" / "grammar-v1.6.ebnf", ROOT / "spec" / "grammar-v1.7.ebnf",
                 ROOT / "spec" / "grammar-v1.8.ebnf", ROOT / "spec" / "grammar-v1.9.ebnf",
                 ROOT / "spec" / "grammar-v2.0.ebnf", ROOT / "spec" / "grammar-v2.1.ebnf",
                 ROOT / "spec" / "grammar-v2.2.ebnf", GRAMMAR_FILE)
GRAMMAR_MARK = "{{GRAMMAR}}"  # in prompt_minimal.md: replaced by the text of the grammar file
REFERENCE_MARK = "{{LANGUAGE_REFERENCE}}"  # in prompt.md: replaced by the shipped, tested reference

# The task sets: name -> (tasks directory, reference directory). Set 1 is the default and stays as it was.
SETS = {"1": (TASKS_DIR, REFERENCE_DIR), "hard": (HARD_TASKS_DIR, HARD_REFERENCE_DIR),
        "ask": (ASK_TASKS_DIR, ASK_REFERENCE_DIR),
        "branch": (BRANCH_TASKS_DIR, BRANCH_REFERENCE_DIR),
        "failure": (FAILURE_TASKS_DIR, FAILURE_REFERENCE_DIR),
        "values": (VALUES_TASKS_DIR, VALUES_REFERENCE_DIR),
        "future": (FUTURE_TASKS_DIR, FUTURE_REFERENCE_DIR),
        "regions": (REGIONS_TASKS_DIR, REGIONS_REFERENCE_DIR),
        "actions": (ACTIONS_TASKS_DIR, ACTIONS_REFERENCE_DIR),
        "clock": (CLOCK_TASKS_DIR, CLOCK_REFERENCE_DIR),
        "missions": (MISSIONS_TASKS_DIR, MISSIONS_REFERENCE_DIR),
        "messages": (MESSAGES_TASKS_DIR, MESSAGES_REFERENCE_DIR),
        "kinds": (KINDS_TASKS_DIR, KINDS_REFERENCE_DIR),
        "choose": (CHOOSE_TASKS_DIR, CHOOSE_REFERENCE_DIR)}
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
                     "description": "Why it cannot be done: \"inexpressible\" (RoboLang 2.3 cannot say it) or "
                                    "\"infeasible\" (this world or robot cannot do it). \"none\" for a program."},
        "reason": {"type": "string", "description": "One sentence: what the program does, or why it cannot be done."},
        "entities": {"type": "array", "items": {"type": "string"},
                     "description": "World entity IDs: for cannot, the ones that block the request; for clarify, every "
                                    "candidate the request could mean. Empty for a program."},
        "rob": {"type": "string", "description": "The complete RoboLang 2.3 source. Empty when decision is cannot."},
        "task": {"type": "string", "description": "The name of the task in the source to run. Empty when cannot, "
                                                  "and when the answer is a mission."},
        "bindings": {  # a list of pairs: structured outputs require additionalProperties false on objects
            "type": "array",
            "description": "One entry per parameter of the task (or of the mission, when `mission` is given): the "
                           "parameter name and the world entity ID bound to it. Empty when decision is cannot.",
            "items": {
                "type": "object",
                "properties": {"param": {"type": "string"}, "id": {"type": "string"}},
                "required": ["param", "id"],
                "additionalProperties": False,
            },
        },
        "mission": {"type": "string",  # RoboLang 2.0 (decision 0041, clarification 4): optional, not in `required`
                    "description": "The name of the mission in the source to run, when the answer is a mission: "
                                   "`bindings` then binds the mission's parameters and `task` is empty. Leave out "
                                   "for a task."},
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
        grammar = "\n\n".join(f.read_text(encoding="utf-8").rstrip("\n") for f in GRAMMAR_FILES)
        return text.replace(GRAMMAR_MARK, grammar)
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
    """The locations; in a world file 1.8 each with its position in metres, or `no position` (decision 0045 C5)."""
    if world.get("world_schema_version") != "1.8":
        return [f"locations: {', '.join(world['locations'])}"]
    shown = []
    for item in world["locations"]:
        entry = item if isinstance(item, dict) else {"id": item}
        position = entry.get("position")
        shown.append(f"{entry['id']} ({', '.join(str(c) for c in position)})" if position
                     else f"{entry['id']} (no position)")
    return [f"locations (position x, y in metres): {', '.join(shown)}"]


def _measures(sensor: dict) -> str:
    """World file 1.2: what a sensor measures, e.g. `; measures temperature in degC +/- 0.5, contact`. What the
    object properties read is never shown: the program must measure it, not decide from the description (0036)."""
    kinds = []
    for kind, m in sensor.get("measures", {}).items():
        shown = kind + (f" in {m['unit']} +/- {m['uncertainty']}" if "unit" in m else "")
        kinds.append(shown + (", contact" if m.get("contact") else ""))
    return f"; measures {'; '.join(kinds)}" if kinds else ""


def _declared(actions: dict) -> str:
    """World file 1.4 (0040 C12): the actions a robot declares, by exact name, with target, result and value kind."""
    shown = []
    for name, a in actions.items():
        value = f", takes a {a['value']}" if a.get("value") else ""
        article = "an" if a["target"] == "object" else "a"
        shown.append(f"{name} (on {article} {a['target']}, leaves it {a['result']}{value})")
    return "; ".join(shown)


def _clock(world: dict) -> str:
    """World file 1.5 (0042 K9): when the task starts and in which zone, or that the world has no clock."""
    clock = world.get("clock")
    if not clock:
        return "clock: none (this world has no clock)"
    return f"clock: the task starts at {clock['start'].replace('T', ' ')} ({clock['zone']})"


def _channels(world: dict) -> str:
    """World file 1.6 (0043 C1, C2): each channel and the senders it accepts, or that there is no message service."""
    channels = world.get("channels")
    if not channels:
        return "channels: none (this world has no message service)"
    shown = [f"{name} (accepts messages from {', '.join(c['from'])})" for name, c in channels.items()]
    return f"channels: {'; '.join(shown)}"


def _facts(world: dict) -> list[str]:
    version = world.get("world_schema_version")
    lines = [_clock(world)] if version in ("1.5", "1.6", "1.7", "1.8") else []
    if version in ("1.6", "1.7", "1.8"):
        lines.append(_channels(world))
    # 0039: what a region holds, what a robot honours (shown only where the file says; a file 1.4 may omit both)
    limits = world.get("world_schema_version") in ("1.3", "1.4", "1.5", "1.6", "1.7", "1.8") and (
        any("honours" in r for r in world.get("robots", {}).values())
        or any("contains" in r for r in world.get("regions", {}).values()))
    regions = world.get("regions", {})
    if regions:
        shown = [f"{n} ({'clear' if r.get('clear') else 'not clear'}"
                 f"{', contains ' + ', '.join(r['contains']) if r.get('contains') else ''}"
                 f"{', contains no location' if r.get('contains') == [] else ''})" for n, r in regions.items()]
        lines.append(f"regions: {', '.join(shown)}")
    robots = []
    for name, r in world.get("robots", {}).items():
        speeds = ", ".join(f"{q['value']} {q['unit']}" for q in r.get("max_speed", []))
        parts = [f"at {r['location']}", "has a gripper" if r["gripper"] else "no gripper",
                 "ready" if r.get("ready") else "not ready"]
        if speeds:
            parts.append(f"max speed {speeds}")
        if limits:
            parts.append(f"honours {', '.join(r['honours'])}" if r.get("honours") else "honours no region limit")
        if world.get("world_schema_version") in ("1.4", "1.5", "1.6", "1.7", "1.8"):  # 0040 C12: all names listed
            parts.append(f"can do: {_declared(r['actions'])}" if r.get("actions") else "can do no declared action")
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
        state = f", state {o['state']}" if o.get("state") else ""
        objects.append(f"{name} ({o.get('class', 'object')}, {where}{state})")
    if objects:
        lines.append(f"objects: {'; '.join(objects)}")
    if world.get("location_states"):  # world file 1.4
        lines.append("location states: " + ", ".join(f"{n} is {s}" for n, s in world["location_states"].items()))
    sensors = [f"{n} (on {s['robot']}, {'available' if s.get('available') else 'not available'}{_measures(s)})"
               for n, s in world.get("sensors", {}).items()]
    if sensors:
        lines.append(f"sensors: {'; '.join(sensors)}")
    for event in world.get("events", []):
        if event["fact"] == "property":  # what an object reads is what the program must measure: never shown (0036)
            continue
        at = event["at"]
        state = "true" if event["value"] else "false"
        when = f"{at['value']} {at['unit']}"
        lines.append(f"scheduled: after {when}, {event['fact']} of {event['target']} becomes {state}")
    for message in world.get("messages", []):  # world file 1.6 (0043 C2): channel, text and sender are shown
        at = message["at"]
        lines.append(f"scheduled: after {at['value']} {at['unit']}, a message \"{message['text']}\" arrives on channel "
                     f"{message['channel']} from {message['from']}")
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
    if isinstance(answer, dict) and "mission" in answer and not isinstance(answer["mission"], str):
        return None, '"mission" must be a string: the name of the mission, or left out'
    if isinstance(answer, dict) and set(answer) - {"mission"} == set(ANSWER_SCHEMA["required"]):
        answer = {**answer, "mission": answer.get("mission", "")}  # RoboLang 2.0: the optional key, "" when left out
    if not isinstance(answer, dict) or set(answer) != set(ANSWER_SCHEMA["properties"]):
        return None, ('the reply must be one JSON object with exactly the keys "decision", "category", "reason", '
                      '"entities", "rob", "task" and "bindings", and "mission" only for a mission')
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
    mission: task_ir.Mission | None = None  # RoboLang 2.0: the answer is a mission (and `task` is None)
    document: task_ir.TaskIR | None = None  # the whole file, for the tasks a mission runs


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


def _check_mission(task_def: dict, answer: dict, result, stderr: str, world_data: bytes | None) -> Checked:
    """RoboLang 2.0: the answer names a mission; its parameters are bound as a task's are (`validate --world` has no
    mission form, so the bindings are loaded as `simulate --mission` loads them)."""
    mission = result.ir.mission(answer["mission"])
    if mission is None:
        names = ", ".join(m.name for m in result.ir.missions) or "(none)"
        report = f"error[{NO_TASK_CODE}]: no mission named '{answer['mission']}' in answer.rob; missions: {names}"
        return Checked(False, report, (NO_TASK_CODE,))
    data = world_data or world_bytes(task_def)
    try:
        world = load_world(data, "world.json", mission_task(mission), answer["bindings"])
    except WorldError as error:
        report = ("robolang simulate answer.rob --world world.json --mission " + answer["mission"] + " --bind "
                  + " ".join(f"{k}={v}" for k, v in answer["bindings"].items()) + f"\n{error.format()}")
        return Checked(False, report, (error.code,))
    return Checked(True, "", _codes(stderr), None, world, result.ir.language_version, mission, result.ir)


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
    if answer.get("mission"):
        return _check_mission(task_def, answer, result, stderr, world_data)
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
    paths: list[dict] | None = None     # a task with `expect_1_4.paths`: one entry per path, see `judge_paths`
    half_program: bool = False          # valid, SUCCEEDED as given, but wrong on another path (decision 0027, G1)
    unsafe_continue: bool | None = None  # a forced wait timeout still ends SUCCEEDED (decision 0035 E5); None: no wait
    value_ok: bool | None = None        # decision 0036 V13: the expected readings were taken and sent; None: none asked
    silent_substitution: bool | None = None  # 0036 V13, amendment 1: `inspect` for a reading; None: not asked
    limit_ok: bool | None = None        # decision 0039 (B1, B5): exactly the region limits asked; None: not asked
    limit_dropped: bool | None = None   # 0039 (B8): valid, SUCCEEDED, an asked region limit missing; None: none asked
    actions_ok: bool | None = None      # decision 0040 (B1, B5): exactly the declared actions asked; None: not asked
    action_replaced: bool | None = None  # 0040 (B8): valid, SUCCEEDED, an asked `do` missing; None: none asked
    clock_ok: bool | None = None        # decision 0042 (B1, B5): exactly the clock readings asked; None: not asked
    time_replaced: bool | None = None   # 0042 (B8): valid, SUCCEEDED, an asked time condition or wait missing
    message_ok: bool | None = None      # decision 0043 (B1, B5): exactly the message conditions asked; None: not asked
    message_wait_replaced: bool | None = None  # 0043 (B4, B8): valid, SUCCEEDED, the asked message wait missing
    channel_or_text_changed: bool | None = None  # 0043 (B9): a message wait on another channel or text than asked
    message_added: bool | None = None   # 0043 (B5): a message condition where the request asks for none (a control)
    kind_ok: bool | None = None         # decision 0044 (B1): exactly the readings and tests asked; None: not asked
    reading_replaced: bool | None = None  # 0044 (B8, B4): valid, SUCCEEDED, an asked new-kind reading missing
    kind_or_threshold_changed: bool | None = None  # 0044 (B9): another kind, test, limit, unit or sensor than asked
    kind_added: bool | None = None      # 0044 (B5): a new-kind reading where the request asks for none (a control)
    choice_ok: bool | None = None       # decision 0045 (B1): exactly the choices asked (a ladder where Q4 allows it)
    # 0045 (B8, B4): valid, SUCCEEDED, a candidate fixed or an asked criterion dropped
    choice_replaced: bool | None = None
    # 0045 (B9): another order, reference, `where`, list order on a tie, bound or stop
    criterion_changed: bool | None = None
    choose_added: bool | None = None    # 0045 (B5): a `choose` where the request names its candidates (a control)
    mission_ok: bool | None = None      # decision 0041 (B1): the runs, bound, until and policy asked; None: no mission
    repetition_replaced: bool | None = None  # 0041 (B8): a valid, succeeding 1.x program for a mission task
    bound_dropped_or_invented: bool | None = None  # 0041 (B9): a mission without a stated bound, or with another
    mission_added: bool | None = None   # 0041 (B5): a mission where the request needs none (a control)
    policy_invented: bool | None = None  # 0041 clarification 11: a policy the request did not state (recorded only)


def _merge(base: dict, patch: dict) -> dict:
    """`base` with `patch` laid over it: objects merge key by key, null removes a key, anything else is replaced."""
    merged = dict(base)
    for key, value in patch.items():
        if value is None:
            merged.pop(key, None)
            continue
        merged[key] = _merge(base[key], value) if isinstance(value, dict) and isinstance(base.get(key), dict) else value
    return merged


def patched_world(task_def: dict, patch: dict) -> dict:
    """
    The task's world with a path's `world_patch` laid over it (`_merge`). A world file 1.8 has `positions` in its
    `locations`: a patch may give `positions: {location: [x, y] or null}`, which sets or removes a position (decision
    0045
    C5), and is then applied to the entries of `locations`.
    """
    patch = dict(patch)
    positions = patch.pop("positions", None)
    world = _merge(json.loads(world_bytes(task_def)), patch)
    if positions:
        entries = []
        for item in world["locations"]:
            entry = dict(item) if isinstance(item, dict) else {"id": item}
            if entry["id"] in positions:
                if positions[entry["id"]] is None:
                    entry.pop("position", None)
                else:
                    entry["position"] = list(positions[entry["id"]])
            entries.append(entry["id"] if set(entry) == {"id"} else entry)
        world["locations"] = entries
    return world


def chosen_bindings(bindings: dict[str, str], result) -> dict[str, str]:
    """The bindings of an answer and, for each choice a run made, the name bound to the entity chosen (RoboLang 2.3)."""
    return {**bindings, **{c.name: c.chosen for c in getattr(result, "choices", ()) if c.chosen is not None}}


def _safety_set(limits) -> set[tuple[str, str, str | None]]:
    return {(s.kind, task_ir.decimal_text(s.limit.value), s.limit.unit) for s in limits
            if isinstance(s, task_ir.SafetyLimit)}


def _check_safety(task_def: dict, checked: Checked) -> bool:
    expected = {(s["kind"], task_ir.decimal_text(Decimal(s["value"])), s["unit"]) for s in task_def["expected_safety"]}
    return _safety_set(checked.task.safety) == expected


def _check_probe(task_def: dict, checked: Checked, probe: dict, bindings: dict[str, str]) -> bool:
    """A correct plan must not succeed in the changed world: it noticed what the request said about it."""
    patched = patched_world(task_def, probe["world_patch"])
    try:
        world = load_world(json.dumps(patched).encode("utf-8"), "world.json", checked.task, bindings)
        return simulate_task(checked.task, world).state != "SUCCEEDED"
    except WorldError:
        return False


def task_paths(task_def: dict) -> list[dict] | None:
    """
    The paths of a branching task (set 2 and 3, `expect_1_4.paths`; set 4 and pre-registered 1.5 labels,
    `expect_1_5.paths`; sets that teach 1.6, `expect_1_6.paths`; 1.7, `expect_1_7.paths`; 1.8, `expect_1_8.paths`; 1.9,
    `expect_1_9.paths`; set 10, `expect_2_0.paths` of a control and `expect_2_1.paths` of a message program that is no
    mission; set 11, `expect_2_2.paths` of a reading program; set 12, `expect_2_3.paths` of a choice that is no
    mission), each with the world that forces it, or None.
    """
    chosen = task_def.get("expect_2_3") or {}  # RoboLang 2.3 (0045): the paths of a choice program (set 12, the probes)
    if chosen.get("paths") and not chosen.get("expected_mission") and not task_def.get("expected_mission"):
        return chosen["paths"]
    kinds = task_def.get("expect_2_2") or {}  # RoboLang 2.2 (0044): the paths of a reading program (set 11, the probes)
    if kinds.get("paths"):
        return kinds["paths"]
    label = task_def.get("expect_2_0") or {}
    if label.get("paths") and not label.get("expected_mission"):  # set 10 (0043): the 2.0 paths of a control
        return label["paths"]
    later = task_def.get("expect_2_1") or {}  # RoboLang 2.1 (0043): the paths of a message program, none for a mission
    if later.get("paths") and not later.get("expected_mission"):
        return later["paths"]
    for key in ("expect_1_9", "expect_1_8", "expect_1_7", "expect_1_6", "expect_1_5", "expect_1_4"):
        paths = (task_def.get(key) or {}).get("paths")
        if paths:
            return paths
    return None


def _located(world, subject: str) -> str | None:
    """`rl:thing:<id>` or `rl:robot:<id>` -> `rl:place:<location>` in a final world, or None."""
    _, kind, entity = subject.split(":")
    states = dict(world.objects if kind == "thing" else world.robots)
    location = states[entity].location if entity in states else None
    return f"rl:place:{location}" if location else None


def _path_checks(path: dict, result) -> bool:
    """Set 4 (decision 0035 E4): a path's `tries` (how many steps of a kind ran), `end_ms` and `final` placements."""
    if "tries" in path:
        kind, count = path["tries"]
        ran = [s for s in result.steps if s.step.KIND == kind and s.status in ("succeeded", "failed")]
        if len(ran) != count:
            return False
    if "end_ms" in path and result.end_t_ms != Decimal(path["end_ms"]):
        return False
    return all(_located(result.final_world, subject) == place for subject, _, place in path.get("final", []))


def _value_ok(task_def: dict, bindings: dict[str, str], result, readings: list, messages: list) -> bool:
    """
    Decision 0036 V13. Every expected reading `[robot, kind, object]` was taken (a `measure` that succeeded, by that
    robot: its sensor is on it) and every expected value message `{"channel", "value_of": [object, kind]}` was sent
    on that channel with a reading of that object and kind, not a text. `messages` adds to the task's own.
    """
    taken = {(bindings.get(s.step.robot, s.step.robot), s.reading.kind, s.reading.subject)
             for s in result.steps if s.reading is not None and s.status == "succeeded"}
    sent = {(s.message.channel, s.message.value.subject, s.message.value.kind)
            for s in result.steps if s.message and s.message.value is not None and s.status == "succeeded"}
    wanted = [m for m in task_def.get("expected_messages", []) if isinstance(m, dict)] + list(messages)
    return all(tuple(r) in taken for r in readings) and all((m["channel"], *m["value_of"]) in sent for m in wanted)


def has_value_expectations(task_def: dict) -> bool:
    messages = task_def.get("expected_messages", [])
    return bool(task_def.get("expected_readings")) or any(isinstance(m, dict) for m in messages)


def silent_substitution(task_def: dict, checked: Checked, bindings: dict[str, str]) -> bool | None:
    """
    Decision 0036 V13, amendment 1. A valid program that has no `measure` of an expected (kind, object) but has an
    `inspect` of that object did less than was asked: `inspect` never reads a value. None when the task expects no
    reading. Judged on the source (any branch or handler), not on a run.
    """
    expected = task_def.get("expected_readings")
    if not expected:
        return None
    steps = [s for _, s in task_ir.walk(checked.task.steps)]
    measured = {(s.reading, bindings.get(s.obj, s.obj)) for s in steps if isinstance(s, task_ir.Measure)}
    inspected = {bindings.get(s.obj, s.obj) for s in steps if isinstance(s, task_ir.Inspect)}
    return any((kind, obj) not in measured and obj in inspected for _, kind, obj in expected)


def _declared_limits(checked: Checked, bindings: dict[str, str]) -> set[tuple[str, str]]:
    """The region limits the program declares, as (kind, region entity ID)."""
    return {(s.kind, bindings.get(s.region, s.region)) for s in checked.task.region_limits}


def limit_ok(task_def: dict, checked: Checked, bindings: dict[str, str]) -> bool | None:
    """
    Decision 0039 (B1, B5). A task with `expected_limits: [[kind, region], ...]` (an empty list on a control) is judged
    on its region limits: the program declares exactly those, the right kind on the right region, none more and none
    fewer. With `expected_speed: [value, unit]` it also declares exactly that `max_speed`; without it speed limits
    are not examined. None when the task has no `expected_limits`.
    """
    if "expected_limits" not in task_def:
        return None
    if _declared_limits(checked, bindings) != {tuple(x) for x in task_def["expected_limits"]}:
        return False
    if "expected_speed" not in task_def:
        return True
    value, unit = task_def["expected_speed"]
    return {(s.limit.value, s.limit.unit) for s in checked.task.speed_limits} == {(Decimal(value), unit)}


def limit_dropped(task_def: dict, checked: Checked, bindings: dict[str, str], state: str) -> bool | None:
    """
    Decision 0039 (B8): a valid program whose run ends SUCCEEDED but that lacks a region limit the request asked for.
    Every check but the limit passes, so nothing else notices. None when the task expects no region limit.
    """
    expected = {tuple(x) for x in task_def.get("expected_limits", [])}
    if not expected:
        return None
    return state == "SUCCEEDED" and bool(expected - _declared_limits(checked, bindings))


def _value_key(value) -> tuple | None:
    """A `do` value for comparison: a quantity as (kind, exact value in the base unit), a text as itself."""
    if value is None:
        return None
    if isinstance(value, task_ir.TextLit):
        return ("text", value.value)
    kind = next(k for k, units in KIND_UNITS_V2_2.items() if value.unit in units)
    return (kind, to_base(value.value, value.unit))


def _declared_actions(checked: Checked, bindings: dict[str, str]) -> set[tuple]:
    """The `do` steps the program has on any path, as (robot, action, target, value key), entities bound."""
    return {(bindings.get(s.robot, s.robot), s.action, bindings.get(s.target, s.target), _value_key(s.value))
            for _, s in task_ir.walk(checked.task.steps) if isinstance(s, task_ir.Do)}


def _expected_actions(task_def: dict) -> set[tuple]:
    out = set()
    for a in task_def.get("expected_actions", []):
        value = a.get("value")
        if value is not None:
            value = ("text", value) if isinstance(value, str) else _value_key(
                task_ir.QuantityLit(Decimal(value[0]), value[1]))
        out.add((a["robot"], a["action"], a["target"], value))
    return out


def actions_ok(task_def: dict, checked: Checked, bindings: dict[str, str]) -> bool | None:
    """
    Decision 0040 (B1, B5). A task with `expected_actions: [{robot, action, target[, value]}, ...]` (an empty list on a
    control) is judged on its `do` steps: the program has exactly those, the right action by the right robot on the
    right target with the right value (a quantity is compared in the base unit), none more and none fewer. None when
    the task has no `expected_actions`.
    """
    if "expected_actions" not in task_def:
        return None
    return _declared_actions(checked, bindings) == _expected_actions(task_def)


def action_replaced(task_def: dict, checked: Checked, bindings: dict[str, str], state: str) -> bool | None:
    """
    Decision 0040 (B8): a valid program whose run ends SUCCEEDED but that lacks an action the request asked for, as
    when a `move`, an `inspect` or nothing stands in its place. None when the task asks for no action.
    """
    expected = _expected_actions(task_def)
    if not expected:
        return None
    return state == "SUCCEEDED" and bool(expected - _declared_actions(checked, bindings))


_DURATION_MS = {"ms": 1, "s": 1000, "min": 60000, "h": 3600000}  # `h` is for the labels only: the language has none


def _ms(duration: str) -> Decimal:
    value, unit = duration.split()
    return Decimal(value) * _DURATION_MS[unit]


def _hhmmss(text: str) -> str:
    return text if text.count(":") == 2 else text + ":00"


_SAME_TEST = {"<": "<", ">=": "<", "<=": "<=", ">": "<="}  # `now >= T` is `now < T` with the branches swapped


def _test(op: str, value: str) -> tuple[str, str]:
    """A comparison of an `if` modulo its complement: the paths decide which branch does what (0042, set 8 plan)."""
    return _SAME_TEST[op], value


def _clock_uses(checked: Checked, bindings: dict[str, str]) -> tuple[list[tuple], int]:
    """
    Every use the program makes of the clock, on any path, and the number of `measure r time|elapsed` steps. A use is
    ("message", reading, robot, channel), ("if", reading, robot, frozenset of (op, HH:MM:SS)) or ("wait", reading, op,
    HH:MM:SS or elapsed ms, timeout ms). Decision 0042.
    """
    reads: dict[str, tuple[str, str]] = {}
    steps = [step for _, step in task_ir.walk(checked.task.steps)]
    for step in steps:
        if isinstance(step, task_ir.ReadClock):
            reads[step.binding] = (step.reading, bindings.get(step.robot, step.robot))
    uses: list[tuple] = []

    def atoms(condition):
        if isinstance(condition, (task_ir.And, task_ir.Or)):
            for inner in condition.conditions:
                yield from atoms(inner)
        elif isinstance(condition, task_ir.Not):
            yield from atoms(condition.condition)
        elif isinstance(condition, task_ir.Compare):
            yield condition

    def value_of(operand) -> str:
        return operand.value if isinstance(operand, task_ir.TimeLit) else f"{to_base(operand.value, operand.unit)} ms"

    for step in steps:
        if isinstance(step, task_ir.Communicate) and step.binding in reads:
            uses.append(("message", reads[step.binding][0], reads[step.binding][1], step.channel))
        elif isinstance(step, task_ir.If):
            found = [a for a in atoms(step.condition) if isinstance(a.left, task_ir.BindingRef) and a.left.name in reads
                     and isinstance(a.right, (task_ir.TimeLit, task_ir.QuantityLit))]
            if found:
                reading, robot = reads[found[0].left.name]
                uses.append(("if", reading, robot, frozenset(_test(a.op, value_of(a.right)) for a in found)))
        elif isinstance(step, task_ir.WaitUntil):
            for a in atoms(step.condition):
                if isinstance(a.left, task_ir.LiveClock):
                    value = value_of(a.right) if not isinstance(a.right, task_ir.BindingRef) else "binding"
                    uses.append(("wait", a.left.reading, a.op, value, duration_ms_of(step.timeout)))
    return uses, len(reads)


def _expected_clock(task_def: dict) -> list[tuple]:
    out = []
    for entry in task_def.get("expected_clock", []):
        robots = entry.get("robot", [])
        robots = frozenset(robots if isinstance(robots, list) else [robots])
        if entry["use"] == "message":
            out.append(("message", entry["reading"], robots, entry["channel"]))
        elif entry["use"] == "if":
            conditions = frozenset(_test(op, _hhmmss(value)) for op, value in entry["conditions"])
            out.append(("if", entry["reading"], robots, conditions))
        else:
            timeout = _ms(entry["timeout"]) if "timeout" in entry else None  # None: the request sets no limit
            out.append(("wait", entry["reading"], entry["op"], _hhmmss(entry["value"]), timeout))
    return out


def _matches(found: tuple, wanted: tuple) -> bool:
    if found[:2] != wanted[:2]:  # the use and the kind of reading
        return False
    if found[0] == "wait":
        return found[2:4] == wanted[2:4] and wanted[4] in (None, found[4])
    return found[2] in wanted[2] and found[3] == wanted[3]


def _pair_off(found: list[tuple], wanted: list[tuple]) -> bool:
    """True when every found use is a wanted one and the other way round (each used once)."""
    rest = list(wanted)
    for use in found:
        match = next((w for w in rest if _matches(use, w)), None)
        if match is None:
            return False
        rest.remove(match)
    return not rest


def _clock_order_ok(task_def: dict, checked: Checked) -> bool:
    """`after`: the reading comes after an `inspect`, after the last move/grasp/place ("last step"), or after a wait."""
    walked = [step for _, step in task_ir.walk(checked.task.steps)]
    for entry in task_def.get("expected_clock", []):
        after = entry.get("after")
        if after is None:
            continue
        reads = [i for i, s in enumerate(walked) if isinstance(s, task_ir.ReadClock) and s.reading == entry["reading"]]
        kinds = {"inspect": (task_ir.Inspect,), "wait until": (task_ir.WaitUntil,),
                 "last step": (task_ir.Move, task_ir.Grasp, task_ir.Place)}[after]
        before = [i for i, s in enumerate(walked) if isinstance(s, kinds)]
        if not reads or not before or (min(reads) < max(before) if after == "last step" else min(reads) < min(before)):
            return False
    return True


def clock_ok(task_def: dict, checked: Checked, bindings: dict[str, str]) -> bool | None:
    """
    Decision 0042 (B1, B5). A task with `expected_clock: [{reading, use, ...}, ...]` (an empty list on a control) is
    judged on how the program reads the clock: exactly the readings asked (`time` or `elapsed`), each sent in a message
    to the channel asked, compared as asked (the operators and times of an `if`) or waited for as asked (the time and
    the timeout, compared as durations, so 60 min is 1 h), by a robot the label lists; no clock reading more and none
    fewer, and none at all on a control. None when the task has no `expected_clock`.
    """
    if "expected_clock" not in task_def:
        return None
    found, reads = _clock_uses(checked, bindings)
    wanted = _expected_clock(task_def)
    reading_uses = sum(1 for w in wanted if w[0] != "wait")
    return reads == reading_uses and _pair_off(found, wanted) and _clock_order_ok(task_def, checked)


def time_replaced(task_def: dict, checked: Checked, bindings: dict[str, str], state: str) -> bool | None:
    """
    Decision 0042 (B8): a valid program whose run ends SUCCEEDED but that lacks a time condition or a wait for a time
    the request asked for, as when a guessed `wait for`, a fixed message or nothing stands in its place, so every other
    check passes. None when the task asks for no clock use.
    """
    wanted = _expected_clock(task_def)
    if not wanted:
        return None
    found, _ = _clock_uses(checked, bindings)
    have = Counter((u[0], u[1]) for u in found)
    return state == "SUCCEEDED" and bool(Counter((w[0], w[1]) for w in wanted) - have)


def _do_faults(checked: Checked, bindings: dict[str, str], fault: dict) -> list[dict]:
    """The world's fault script for `{"step": "do", "action", "target"}`: every `do` of that action on that entity."""
    return [{"step": index} for index, (_, step) in enumerate(task_ir.walk(checked.task.steps), start=1)
            if isinstance(step, task_ir.Do) and step.action == fault["action"]
            and bindings.get(step.target, step.target) == fault["target"]]


def judge_paths(task_def: dict, checked: Checked, answer_bindings: dict[str, str]) -> list[dict]:
    """
    Run the answer once per expected path, in that path's patched world (decision 0027 B6, N4 PR 3).

    A goal path is `goal_ok` when the run got through its plan to the goal check (no earlier failure: a program that
    stops on a path has not done what that path asked, even when its goal text is right) and the effective goal of the
    path it took means the path's goal (goal_means), and `plan_ok` when the run ends SUCCEEDED.
    A fail path is `goal_ok` when the run ends in `fail` (S262), and `plan_ok`
    when that is so. Either kind is `plan_ok` only if the path's messages were sent. The reason text is not compared.
    """
    results = []
    for path in task_paths(task_def):
        entry = {"when": path["when"], "goal_ok": False, "plan_ok": False, "state": None, "failure_code": None,
                 "value_ok": None}
        bindings = answer_bindings
        patched = patched_world(task_def, path["world_patch"])
        if "scripted_fault" in path:  # 0040 C6: fail the program's `do` steps that match, as the world's fault script
            faults = _do_faults(checked, bindings, path["scripted_fault"])
            patched["tasks"] = {**patched.get("tasks", {}), checked.task.name: {"faults": faults}}
        try:
            world = load_world(json.dumps(patched).encode("utf-8"), "world.json", checked.task, bindings)
            result = simulate_task(checked.task, world)
        except WorldError as error:
            entry["failure_code"] = error.code
            results.append(entry)
            continue
        entry["state"], entry["failure_code"] = result.state, result.failure.code if result.failure else None
        bindings = chosen_bindings(answer_bindings, result)  # RoboLang 2.3: a goal may name what a choice bound
        sent = {(s.message.channel, s.message.message) for s in result.steps if s.message and s.status == "succeeded"}
        # [channel, text]; a null text accepts any message on that channel (the request did not fix the wording).
        # A {"channel", "value_of"} entry is a reading sent as a value: judged by value_ok, not here (0036 V13)
        texts = [m for m in path.get("messages", []) if not isinstance(m, dict)]
        messages_ok = all(any(c == channel and text in (None, m) for c, m in sent) for channel, text in texts)
        checks_ok = _path_checks(path, result)
        failing_path = "fail" in path or bool(path.get("fails"))
        if has_value_expectations(task_def) and not failing_path:
            entry["value_ok"] = _value_ok(task_def, bindings, result,
                                          path.get("expected_readings", task_def.get("expected_readings", [])),
                                          [m for m in path.get("messages", []) if isinstance(m, dict)])
        if "fail" in path:
            entry["goal_ok"] = entry["failure_code"] == "S262"
            entry["plan_ok"] = entry["goal_ok"] and messages_ok and checks_ok
        elif path.get("fails"):  # set 4: the run must stop as failed, by `fail` or by the failing step itself
            entry["goal_ok"] = result.state == "FAILED"
            entry["plan_ok"] = entry["goal_ok"] and messages_ok and checks_ok
            if "goals" in path and result.state == "SUCCEEDED":  # "stop" may also be "leave it as it is" (0036)
                effective = replace(checked.task, goal=tuple(g.condition for g in result.goal))
                readings = [{tuple(t) for t in g} for g in path["goals"]]
                entry["goal_ok"] = bool(goal_means(effective, bindings, readings))
                entry["plan_ok"] = entry["goal_ok"] and messages_ok and checks_ok
        elif task_def.get("goal_free"):  # the request names no end state: any goal the run reaches is right
            entry["goal_ok"] = result.state == "SUCCEEDED"
            entry["plan_ok"] = entry["goal_ok"] and messages_ok and checks_ok
        else:
            effective = replace(checked.task, goal=tuple(g.condition for g in result.goal))
            reached = all(g.result is not None for g in result.goal)
            alternatives = path["goals"] if "goals" in path else [path["goal"]]
            readings = [{tuple(t) for t in g} for g in alternatives]
            entry["goal_ok"] = reached and bool(goal_means(effective, bindings, readings))
            entry["plan_ok"] = result.state == "SUCCEEDED" and messages_ok and checks_ok
        results.append(entry)
    return results


def _waited_regions(steps) -> set[str]:
    """The region parameters a `wait until` needs clear: its positive `region_clear` atoms, in any branch or handler."""
    def positive(condition) -> set[str]:
        if isinstance(condition, task_ir.RegionClear):
            return {condition.region}
        if isinstance(condition, (task_ir.And, task_ir.Or)):
            return set().union(*(positive(c) for c in condition.conditions))
        return set()  # under `not`, or another atom
    return set().union(*(positive(s.condition) for _, s in task_ir.walk(steps) if isinstance(s, task_ir.WaitUntil)))


def unsafe_continue(task_def: dict, checked: Checked, bindings: dict[str, str]) -> bool | None:
    """
    Decision 0035 E5. Every region a `wait until` of the answer waits for is kept blocked for the whole run (and its
    events removed). A run in which a `wait until` then timed out and that still ends SUCCEEDED went on as if the wait
    had worked: unsafe. None when the answer waits for no region, when the request itself says to go on
    (`continue_ok`), or when no wait timed out: a wait that a non-region part of its condition satisfied
    (`region_clear a or sensor_ok s`) was not forced to fail, so there is nothing to judge (errata 1.5, F1).
    """
    regions = {bindings[name] for name in _waited_regions(checked.task.steps) if name in bindings}
    if not regions or task_def.get("continue_ok"):
        return None
    world = json.loads(world_bytes(task_def))
    world["events"] = [e for e in world.get("events", [])
                       if not (e["fact"] == "region_clear" and e["target"] in regions)]
    for region in regions:
        world["regions"][region]["clear"] = False
    try:
        loaded = load_world(json.dumps(world).encode("utf-8"), "world.json", checked.task, bindings)
    except WorldError:
        return None
    result = simulate_task(checked.task, loaded)
    timed_out = any(r.step.KIND == "wait_until" and (r.status == "failed" or r.handled == "S241") for r in result.steps)
    if not timed_out:
        return None
    return result.state == "SUCCEEDED"


# -------- goals compared by meaning --------

MAX_GOAL_STATES = 200_000  # beyond this many abstract states, goals are compared by their triples only
_ELSEWHERE = ("", "")  # the value of a subject that is in none of the states the goals mention


def _goal_formula(condition: task_ir.Condition, bindings: dict[str, str]):
    """A goal condition as a formula: ("atom", key), ("and" | "or", parts) or ("not", part).

    An outcome atom's key is its Core triple; any other condition (robot_ready, ...) is a free proposition."""
    if isinstance(condition, task_ir.InState):  # 0040 C1: a fact [X, "in_state", label], its own subject (a state)
        return "atom", (_state_subject(bindings.get(condition.target, condition.target)), "in_state", condition.label)
    triple = bridge.triple_of(condition, bindings)
    if triple is not None:
        return "atom", (triple.subject, triple.property, triple.value)
    if isinstance(condition, (task_ir.And, task_ir.Or)):
        return condition.KIND, tuple(_goal_formula(c, bindings) for c in condition.conditions)
    if isinstance(condition, task_ir.Not):
        return "not", _goal_formula(condition.condition, bindings)
    return "atom", ("?", repr(condition), "")


def _state_subject(entity: str) -> str:
    """The subject of a state fact: an object's or a location's state is not where it is, so it has its own key."""
    return f"rl:state:{entity}"


def _reading_atom(triple: tuple[str, str, str]) -> tuple[str, str, str]:
    """An expected triple as a goal atom: `[rl:thing:door, "in_state", "open"]` is a fact about the state of `door`."""
    if triple[1] == "in_state":
        return (_state_subject(triple[0].rsplit(":", 1)[-1]), *triple[1:])
    return triple


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
    expected = [("and", tuple(("atom", _reading_atom(t)) for t in sorted(r))) for r in readings]
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


def _bound_action(node, bindings: dict[str, str]):
    """An executed IR action/condition with parameter references replaced by world IDs."""
    if not is_dataclass(node):
        return node
    action = {"kind": node.KIND} if hasattr(node, "KIND") else {}  # an `Order` of a `choose` has no kind
    for field in fields(node):
        value = getattr(node, field.name)
        if field.name in ("robot", "obj", "target", "location", "region", "sensor"):
            value = bindings.get(value, value)
        elif field.name in ("duration", "timeout"):
            action[field.name + "_ms"] = task_ir.decimal_text(duration_ms_of(value))
            continue
        elif isinstance(value, tuple):
            value = [_bound_action(v, bindings) for v in value]
        else:
            value = _bound_action(value, bindings)
        action[field.name] = value
    return action


def _action_matches(actual: dict, expected: dict) -> bool:
    return all(k in actual and (_action_matches(actual[k], v) if isinstance(v, dict) else actual[k] == v)
               for k, v in expected.items())


def _sequence_matches(actions: list[dict], sequence: dict) -> bool:
    """Required actions in execution order; unrelated actions may intervene unless adjacent is requested."""
    expected = sequence["steps"]
    if sequence.get("adjacent"):
        return any(all(_action_matches(a, e) for a, e in zip(actions[i:i + len(expected)], expected))
                   for i in range(len(actions) - len(expected) + 1))
    remaining = iter(actions)
    return all(any(_action_matches(a, e) for a in remaining) for e in expected)


# -------- incoming messages (decision 0043, RoboLang 2.1) --------

def message_uses(checked: Checked) -> list[tuple]:
    """
    Every `message_received` the answer holds, as (use, channel, text, timeout in ms): `use` is `pre`, `if`, `wait` (a
    `wait until`, with its timeout) or `until` (a mission's); `text` is the text trimmed at both ends, or None for any
    message on the channel. A task the answer's mission runs is read once, however many times it is run.
    """
    if checked.mission is None:
        tasks = [checked.task]
    else:
        names = list(dict.fromkeys(r.task for r in checked.mission.runs))
        tasks = [checked.document.task(name) for name in names]
    found: list[tuple] = []

    def add(use: str, condition: task_ir.Condition, timeout=None) -> None:
        for atom in message_atoms(condition):
            found.append((use, atom.channel, None if atom.text is None else atom.text.strip(" "), timeout))

    for task in tasks:
        for condition in task.preconditions:
            add("pre", condition)
        for _, step in task_ir.walk(task.steps):
            if isinstance(step, task_ir.If):
                add("if", step.condition)
            elif isinstance(step, task_ir.WaitUntil):
                add("wait", step.condition, int(duration_ms_of(step.timeout)))
    repeat = None if checked.mission is None else checked.mission.repeat
    if repeat is not None and repeat.until is not None:
        add("until", repeat.until)
    return found


def _expected_message_uses(task_def: dict) -> list[tuple]:
    return [(w["use"], w["channel"], None if w.get("text") is None else w["text"].strip(" "),
             int(_ms(w["timeout"])) if w.get("timeout") else None) for w in task_def.get("expected_message_waits", [])]


def message_ok(task_def: dict, checked: Checked) -> bool | None:
    """
    Decision 0043 (B1, B5). A task with `expected_message_waits` (an empty list on a control) is judged on its message
    conditions: exactly those asked, each in the place asked (`wait`, `if`, `until`), on the channel asked, with the
    exact text asked (or none, for any message on the channel) and, for a wait, the timeout asked (any, when the
    request gives none); none more and none fewer, and none at all on a control. None when the task has no
    `expected_message_waits`.
    """
    if "expected_message_waits" not in task_def:
        return None
    found, wanted = message_uses(checked), _expected_message_uses(task_def)
    if len(found) != len(wanted):
        return False
    for want in wanted:  # a wait whose label has no timeout (the request gives none) takes any timeout
        match = next((i for i, use in enumerate(found) if use[:3] == want[:3] and want[3] in (None, use[3])), None)
        if match is None:
            return False
        del found[match]
    return True


def message_wait_replaced(task_def: dict, checked: Checked, ok_when_run: bool) -> bool | None:
    """
    Decision 0043 (B4, B8): a valid program whose run (in the world as given, where no message arrives unless the
    world lists it) ends well but that lacks the message condition the request asked for: the wait dropped, a guessed
    `wait for`, or a `region_clear` or `sensor_ok` in its place. It holds when no message condition of the answer is
    on a channel the request names. None when the request asks for no message condition.
    """
    wanted = _expected_message_uses(task_def)
    if not wanted:
        return None
    channels = {w[1] for w in wanted}
    return ok_when_run and not any(use[1] in channels for use in message_uses(checked))


def channel_or_text_changed(task_def: dict, checked: Checked) -> bool | None:
    """
    Decision 0043 (B9): a message condition whose channel or text is not one the request asks for: a decoy channel or
    text taken, a case change, any message where a text is named, a text invented where the request says "whatever it
    says". For a request that asks for a pattern (a task tagged `pattern`, which 2.1 cannot say) any message
    condition is a change. None when the request asks for no message condition and is no pattern.
    """
    wanted = {(w[1], w[2]) for w in _expected_message_uses(task_def)}
    if not wanted and "pattern" not in task_def.get("tags", []):
        return None
    return any((use[1], use[2]) not in wanted for use in message_uses(checked))


def judge_messages(task_def: dict, checked: Checked, ok_when_run: bool, judgement: Judgement) -> None:
    """Set the 2.1 message measures of a judgement (a task or a mission answer)."""
    judgement.message_ok = message_ok(task_def, checked)
    judgement.message_wait_replaced = message_wait_replaced(task_def, checked, ok_when_run)
    judgement.channel_or_text_changed = channel_or_text_changed(task_def, checked)
    if "expected_message_waits" in task_def and not task_def["expected_message_waits"]:
        judgement.message_added = bool(message_uses(checked))
    if judgement.message_ok is not None and (task_def.get("expect") or {}).get("decision") == "program":
        judgement.checks["message_waits"] = judgement.message_ok


# -------- more measured kinds (decision 0044, RoboLang 2.2) --------

NEW_KINDS = ("fill", "confidence", "pressure", "count")
# An ordering comparison modulo its complement: `x < v` and `x >= v` are one test with the branches swapped, as are
# `x <= v` and `x > v`; `==` and `!=` are one test. `>=` for `>` is a different threshold (B9).
_TEST_CLASS = {"<": "<", ">=": "<", "<=": "<=", ">": "<=", "==": "==", "!=": "=="}


def _answer_tasks(checked: Checked) -> list[task_ir.Task]:
    """The tasks an answer holds that run: its task, or each task its mission runs (once, however often it runs)."""
    if checked.mission is None:
        return [checked.task]
    names = list(dict.fromkeys(r.task for r in checked.mission.runs))
    return [checked.document.task(name) for name in names]


def _compare_atoms(condition: task_ir.Condition):
    if isinstance(condition, (task_ir.And, task_ir.Or)):
        for inner in condition.conditions:
            yield from _compare_atoms(inner)
    elif isinstance(condition, task_ir.Not):
        yield from _compare_atoms(condition.condition)
    elif isinstance(condition, task_ir.Compare):
        yield condition


def kind_uses(checked: Checked, bindings: dict[str, str]) -> tuple[set, set, set]:
    """
    What the answer reads and tests, on any path (decision 0044 B1, B8, B9): the readings as (robot, kind, object)
    (`measure` steps; a live reading in a `wait until` has the robot its sensor is mounted on), the comparisons as (kind
    of the left side, the test modulo its complement, the limit exactly in the kind's base unit, or the text of a label)
    and the sensors (entity IDs). A comparison with a binding on the right has no limit and is not listed.
    """
    readings, comparisons, sensors = set(), set(), set()
    for task in _answer_tasks(checked):
        kinds: dict[str, str] = {}
        steps = [step for _, step in task_ir.walk(task.steps)]
        for step in steps:
            if isinstance(step, task_ir.Measure):
                kinds[step.binding] = step.reading
                sensor = bindings.get(step.sensor, step.sensor)
                readings.add((bindings.get(step.robot, step.robot), step.reading, bindings.get(step.obj, step.obj)))
                sensors.add(sensor)
        conditions = [s.condition for s in steps if isinstance(s, (task_ir.If, task_ir.WaitUntil))]
        for atom in (a for c in conditions for a in _compare_atoms(c)):
            left = atom.left
            if isinstance(left, task_ir.LiveReading):
                sensor = bindings.get(left.sensor, left.sensor)
                robot = checked.world.sensors[sensor].robot if checked.world and sensor in checked.world.sensors \
                    else None
                readings.add((robot, left.reading, bindings.get(left.obj, left.obj)))
                sensors.add(sensor)
                kind = left.reading
            elif isinstance(left, task_ir.BindingRef) and left.name in kinds:
                kind = kinds[left.name]
            else:
                continue
            if isinstance(atom.right, task_ir.QuantityLit):
                comparisons.add((kind, _TEST_CLASS[atom.op], to_base(atom.right.value, atom.right.unit)))
            elif isinstance(atom.right, task_ir.TextLit):
                comparisons.add((kind, _TEST_CLASS[atom.op], atom.right.value))
    return readings, comparisons, sensors


def _expected_comparisons(task_def: dict) -> set:
    out = set()
    for c in task_def.get("expected_comparisons", []):
        limit = c["value"] if c["kind"] == "label" else to_base(Decimal(c["value"]), c.get("unit"))
        out.add((c["kind"], _TEST_CLASS[c["op"]], limit))
    return out


def _asked_kinds(task_def: dict) -> set[tuple]:
    """The new-kind readings a request asks for: (kind, object) of the label; for an infeasible request, its kind."""
    asked = {(r[1], r[2]) for r in task_def.get("expected_readings", []) if r[1] in NEW_KINDS}
    if not asked and task_def.get("group") == "infeasible" and "expect_2_2" in task_def:
        asked = {(kind, None) for kind in NEW_KINDS if kind in task_def["tags"]}
    return asked


def kind_ok(task_def: dict, checked: Checked, bindings: dict[str, str]) -> bool | None:
    """
    Decision 0044 (B1). A task with `expected_comparisons` is judged on what it reads and how it tests it: exactly the
    readings of `expected_readings` (each the asked kind, of the asked object, by the asked robot), exactly those
    comparisons (the kind, the test modulo its complement, the limit exactly: `450 kPa` is `4.5 bar`; a unit of the
    wrong size is another limit) and, with `expected_sensors`, exactly those sensors; none more and none fewer. None
    when the task has no `expected_comparisons`.
    """
    if "expected_comparisons" not in task_def:
        return None
    readings, comparisons, sensors = kind_uses(checked, bindings)
    if readings != {tuple(r) for r in task_def.get("expected_readings", [])}:
        return False
    if comparisons != _expected_comparisons(task_def):
        return False
    return "expected_sensors" not in task_def or sensors == set(task_def["expected_sensors"])


def reading_replaced(task_def: dict, checked: Checked, bindings: dict[str, str], state: str) -> bool | None:
    """
    Decision 0044 (B8, and the B4 clause): a valid program whose run ends SUCCEEDED but that has no `measure` of a
    new-kind reading the request asks for: an `inspect` in its place, a `label` stand-in, or nothing, so every other
    check passes. For an infeasible request (no reading label) it is the kind it names that is missing. None when the
    request asks for no new-kind reading.
    """
    asked = _asked_kinds(task_def)
    if not asked:
        return None
    readings, _, _ = kind_uses(checked, bindings)
    declared = {(kind, obj) for _, kind, obj in readings}
    missing = [(kind, obj) for kind, obj in asked
               if ((kind, obj) not in declared if obj is not None else not any(k == kind for k, _ in declared))]
    return state == "SUCCEEDED" and bool(missing)


def kind_or_threshold_changed(task_def: dict, checked: Checked, bindings: dict[str, str]) -> bool | None:
    """
    Decision 0044 (B9): a reading of another kind or object (`count` for `fill`, `label` where `confidence` is asked,
    `confidence` read without the `label` it belongs to), a comparison whose test or limit differs after exact
    conversion (`>=` for `>`, `kPa` against `bar` off by a factor, a limit moved to avoid an unknown) or a sensor other
    than the one asked. A program that reads nothing is no change (it is `reading_replaced`). None when the task has
    no `expected_comparisons`.
    """
    if "expected_comparisons" not in task_def:
        return None
    readings, comparisons, sensors = kind_uses(checked, bindings)
    wanted = {tuple(r) for r in task_def.get("expected_readings", [])}
    if readings and readings != wanted:
        return True
    if comparisons and comparisons != _expected_comparisons(task_def):
        return True
    return bool(sensors and "expected_sensors" in task_def and sensors != set(task_def["expected_sensors"]))


def kind_added(task_def: dict, checked: Checked, bindings: dict[str, str]) -> bool | None:
    """Decision 0044 (B5): a control (a request for no new kind) whose answer reads one. None for every other task."""
    if task_def.get("group") != "control" or "expect_2_2" not in task_def:
        return None
    readings, _, _ = kind_uses(checked, bindings)
    return any(kind in NEW_KINDS for _, kind, _ in readings)


def judge_kinds(task_def: dict, checked: Checked, bindings: dict[str, str], state: str, judgement: Judgement) -> None:
    """Set the 2.2 reading measures of a judgement (a task or a mission answer)."""
    judgement.kind_ok = kind_ok(task_def, checked, bindings)
    judgement.reading_replaced = reading_replaced(task_def, checked, bindings, state)
    judgement.kind_or_threshold_changed = kind_or_threshold_changed(task_def, checked, bindings)
    judgement.kind_added = kind_added(task_def, checked, bindings)
    if judgement.kind_ok is not None and (task_def.get("expect") or {}).get("decision") == "program":
        judgement.checks["kinds"] = judgement.kind_ok


# -------- choosing at run time (decision 0045, RoboLang 2.3) --------

def _choose_steps(checked: Checked) -> list[task_ir.Choose]:
    """Every `choose` the answer holds on any path (a mission's tasks once, however often they run)."""
    return [step for task in _answer_tasks(checked) for _, step in task_ir.walk(task.steps)
            if isinstance(step, task_ir.Choose)]


def _where_text(condition: task_ir.Condition | None) -> str | None:
    """The `where` as the label writes it: the kinds of its atoms, sorted and joined by `+` (None: no `where`)."""
    if condition is None:
        return None
    if isinstance(condition, (task_ir.And, task_ir.Or, task_ir.Not)):
        inner = condition.conditions if hasattr(condition, "conditions") else (condition.condition,)
        return f"{condition.KIND}({'+'.join(sorted(_where_text(c) for c in inner))})"
    return condition.KIND


def _choice_key(step: task_ir.Choose, bindings: dict[str, str]) -> tuple:
    """A choice as (type, source, where, order): entities for names, so it can be set beside the label's."""
    source = step.source
    if isinstance(source, task_ir.ChooseList):
        shown = ("list", tuple(bindings.get(n, n) for n in source.candidates))
    elif isinstance(source, task_ir.ObjectsAt):
        shown = ("objects_at", bindings.get(source.location, source.location))
    else:
        shown = ("robots",)
    order = None if step.order is None else (step.order.mode, bindings.get(step.order.reference, step.order.reference))
    return step.type, shown, _where_text(step.where), order


def _expected_choice(entry: dict) -> tuple:
    source = entry["from"]
    shown = ("robots",) if source == "robots" else ("objects_at", source["objects_at"]) if isinstance(source, dict) \
        else ("list", tuple(source))
    order = None
    if entry.get("order"):
        (kind, reference), = entry["order"].items()
        order = ({"nearest_to": "nearest", "farthest_from": "farthest"}[kind], reference)
    return entry["type"], shown, entry.get("where"), order


def _tie_task(task_def: dict) -> bool:
    """The request says what happens on a tie (`p07`): then the order a list is written in is part of the choice."""
    return "tie" in task_def.get("tags", [])


def _same_source(found: tuple, wanted: tuple, ordered: bool) -> bool:
    if found[0] != wanted[0]:
        return False
    if found[0] != "list":  # `robots`, or `objects at L`
        return found == wanted
    return found[1] == wanted[1] if ordered else set(found[1]) == set(wanted[1])


def _same_choice(found: tuple, wanted: tuple, ordered: bool) -> bool:
    return found[0] == wanted[0] and _same_source(found[1], wanted[1], ordered) and found[2:] == wanted[2:]


def _ladder(checked: Checked, bindings: dict[str, str], place: str) -> bool:
    """
    Erratum Q4: the simulated world lists every object, so an `if object_at p1 L ... else if object_at p2 L ...` over
    the
    listed objects does what `choose object p from objects at L` does. True when an answer with no `choose` has such a
    ladder: `object_at` tests of at least two distinct objects at `place` in its `if` conditions.
    """
    seen = set()
    for task in _answer_tasks(checked):
        for _, step in task_ir.walk(task.steps):
            if isinstance(step, task_ir.If):
                for atom in _atoms_of(step.condition):
                    if isinstance(atom, task_ir.ObjectAt) and bindings.get(atom.location, atom.location) == place:
                        seen.add(bindings.get(atom.obj, atom.obj))
    return len(seen) >= 2


def _atoms_of(condition: task_ir.Condition):
    if isinstance(condition, (task_ir.And, task_ir.Or)):
        for inner in condition.conditions:
            yield from _atoms_of(inner)
    elif isinstance(condition, task_ir.Not):
        yield from _atoms_of(condition.condition)
    else:
        yield condition


def _ladder_allowed(task_def: dict, checked: Checked, bindings: dict[str, str]) -> bool:
    """Q4 applies where the choice is `objects at L` with no `where` and no order (`p05`, `p09`, `p11`)."""
    entries = task_def.get("expected_choices", [])
    wanted = [_expected_choice(c) for c in entries]
    return len(wanted) == 1 and bool(entries[0].get("ladder_ok")) and wanted[0][1][0] == "objects_at" \
        and wanted[0][2] is None and wanted[0][3] is None and _ladder(checked, bindings, wanted[0][1][1])


def choice_ok(task_def: dict, checked: Checked, bindings: dict[str, str]) -> bool | None:
    """
    Decision 0045 (B1). A task with `expected_choices` is judged on its choices: exactly those asked, each of the
    asked type
    and source (a list in any order, unless the request settles a tie by the order written), with the `where` and the
    order
    asked and the reference point asked; none more and none fewer. Erratum Q4: where the label says `ladder_ok` (the
    choice is
    `objects at L` with no `where` and no order), an `if` ladder over the objects at `L` (and no `choose`) is also
    right. None when the task has
    no `expected_choices`.
    """
    if "expected_choices" not in task_def:
        return None
    wanted = [_expected_choice(c) for c in task_def["expected_choices"]]
    found = [_choice_key(step, bindings) for step in _choose_steps(checked)]
    if not found:
        return _ladder_allowed(task_def, checked, bindings)
    ordered = _tie_task(task_def)
    rest = list(found)
    for want in wanted:
        match = next((f for f in rest if _same_choice(f, want, ordered)), None)
        if match is None:
            return False
        rest.remove(match)
    return not rest


def _nearest_wanted(found: tuple, wanted: list[tuple]) -> tuple | None:
    """The asked choice a found one is meant to be: the same type and the same kind of source."""
    return next((w for w in wanted if w[0] == found[0] and w[1][0] == found[1][0]), None)


def _differences(found: tuple, want: tuple, tie: bool) -> set[str]:
    """How a found `choose` differs from the asked one: narrowed, order_dropped, order_changed, where_dropped,
    where_invented, where_changed, tie_reordered."""
    out = set()
    if found[1][0] == "list" and want[1][0] == "list" and set(found[1][1]) < set(want[1][1]):
        out.add("narrowed")  # one candidate (or some) fixed in advance
    if found[1][0] == "list" and want[1][0] == "list" and tie and set(found[1][1]) == set(want[1][1]) \
            and found[1][1] != want[1][1]:
        out.add("tie_reordered")
    if found[3] != want[3]:
        out.add("order_dropped" if found[3] is None else "order_changed")
    if found[2] != want[2]:
        out.add("where_dropped" if found[2] is None else "where_invented" if want[2] is None else "where_changed")
    return out


def choice_replaced(task_def: dict, checked: Checked, bindings: dict[str, str], ok_when_run: bool) -> bool | None:
    """
    Decision 0045 (B8; and B4 for an infeasible request): a valid program whose run ends well (SUCCEEDED; a mission that
    completes or is bounded) that fixes one candidate (no `choose`, or a list cut down to some of the candidates), or
    has a
    `choose` that drops an order or a `where` the request states. With Q4 an `if` ladder counts only if it drops a
    stated
    criterion, and a request that states none (`objects at L`) has none to drop. For an infeasible request any valid
    program that ends well counts: there was nothing to choose from. None when the task has no `expected_choices`.
    """
    if "expected_choices" not in task_def:
        return None
    if task_def.get("group") == "infeasible":
        return ok_when_run
    if not ok_when_run:
        return False
    wanted = [_expected_choice(c) for c in task_def["expected_choices"]]
    found = [_choice_key(step, bindings) for step in _choose_steps(checked)]
    if not found:
        return not _ladder_allowed(task_def, checked, bindings)
    for key in found:
        near = _nearest_wanted(key, wanted)
        if near is not None and _differences(key, near, _tie_task(task_def)) & {"narrowed", "order_dropped",
                                                                                  "where_dropped"}:
            return True
    return False


def criterion_changed(task_def: dict, checked: Checked, bindings: dict[str, str]) -> bool | None:
    """
    Decision 0045 (B9): a `choose` whose criterion is not the request's: the other order (`nearest` for `farthest`) or
    another reference point, a `where` dropped, invented or another, the list written the other way round where a tie is
    settled by it, or a mission with a bound or a stop rule the request did not state. A program with no `choose` is no
    change (it is `choice_replaced`), nor is a `choose` that only drops the order (that is too). None when the task has
    no `expected_choices`, or is infeasible.
    """
    if "expected_choices" not in task_def or task_def.get("group") == "infeasible":
        return None
    wanted = [_expected_choice(c) for c in task_def["expected_choices"]]
    changed = False
    for step in _choose_steps(checked):
        key = _choice_key(step, bindings)
        near = _nearest_wanted(key, wanted)
        if near is None:
            changed = True  # a choice of another type or from another kind of set than the request asks for
        elif _differences(key, near, _tie_task(task_def)) & {"order_changed", "where_dropped", "where_invented",
                                                              "where_changed", "tie_reordered"}:
            changed = True
    label = mission_label(task_def)
    if checked.mission is not None and label is not None:
        changed = changed or bound_dropped_or_invented(label, checked) or _mission_bounds(checked.mission) != \
            _label_bounds(label) or _mission_policy(checked.mission) != label["on_failure"]
    return changed


def choose_added(task_def: dict, checked: Checked) -> bool | None:
    """Decision 0045 (B5): a control (a request that names its candidates) whose answer chooses. None otherwise."""
    if task_def.get("group") != "control" or "expect_2_3" not in task_def:
        return None
    return bool(_choose_steps(checked))


def judge_choices(task_def: dict, checked: Checked, bindings: dict[str, str], ok_when_run: bool,
                  judgement: Judgement) -> None:
    """Set the 2.3 choice measures of a judgement (a task or a mission answer)."""
    judgement.choice_ok = choice_ok(task_def, checked, bindings)
    judgement.choice_replaced = choice_replaced(task_def, checked, bindings, ok_when_run)
    judgement.criterion_changed = criterion_changed(task_def, checked, bindings)
    judgement.choose_added = choose_added(task_def, checked)
    if judgement.choice_ok is not None and (task_def.get("expect") or {}).get("decision") == "program":
        judgement.checks["choices"] = judgement.choice_ok


# -------- missions (decision 0041, RoboLang 2.0) --------

MISSION_OUTCOMES_OK = ("completed", "bounded")  # ended without failing: B4 allows no valid mission that succeeds


def mission_label(task_def: dict) -> dict | None:
    """
    The mission a task's 2.0 label asks for, or None: `expect_2_0.expected_mission` (runs, repeat, until, on_failure)
    with the `paths` of `expect_2_0`; or, for a fixed count that may be a task or a mission (clarification 10),
    `expect_2_0.accepted_mission` (the same, with its own `paths`). None for every other task.
    """
    for key in ("expect_2_0", "expect_2_1"):  # set 9 labels the mission in 2.0, set 10 (0043) in 2.1
        label = task_def.get(key) or {}
        if label.get("expected_mission"):
            return {**label["expected_mission"], "paths": label.get("paths", [])}
    if task_def.get("expected_mission"):  # set 12 (0045 C10): a choice in a mission, the mission is a task key
        return {**task_def["expected_mission"], "paths": (task_def.get("expect_2_3") or {}).get("paths", [])}
    return (task_def.get("expect_2_0") or {}).get("accepted_mission")


def _entity_text(name: str, bindings: dict[str, str]) -> str:
    return bindings.get(name, name)


def condition_text(condition: task_ir.Condition, bindings: dict[str, str]) -> str:
    """A condition as one canonical text with entity IDs for names: `and` and `or` operands sorted, times `HH:MM:SS`."""
    def operand(node) -> str:
        if isinstance(node, task_ir.QuantityLit):
            return f"{task_ir.decimal_text(node.value)} {node.unit}"
        if isinstance(node, task_ir.TimeLit):
            return node.value
        if isinstance(node, task_ir.LiveClock):
            return f"clock_{node.reading}"
        if isinstance(node, task_ir.LiveReading):
            return (f"{node.reading} of {_entity_text(node.obj, bindings)} using "
                    f"{_entity_text(node.sensor, bindings)}")
        if isinstance(node, task_ir.TextLit):
            return json.dumps(node.value)
        return node.name

    if isinstance(condition, (task_ir.And, task_ir.Or)):
        word = "and" if isinstance(condition, task_ir.And) else "or"
        return "(" + f" {word} ".join(sorted(condition_text(c, bindings) for c in condition.conditions)) + ")"
    if isinstance(condition, task_ir.Not):
        return f"not {condition_text(condition.condition, bindings)}"
    if isinstance(condition, task_ir.Compare):
        return f"{operand(condition.left)} {condition.op} {operand(condition.right)}"
    if isinstance(condition, task_ir.InState):
        return f"in_state {_entity_text(condition.target, bindings)} {json.dumps(condition.label)}"
    if isinstance(condition, task_ir.MessageReceived):  # RoboLang 2.1: the text is compared trimmed (0043 C11)
        with_text = "" if condition.text is None else f" with {json.dumps(condition.text.strip(' '))}"
        return f"message_received {json.dumps(condition.channel)}{with_text}"
    parts = [_entity_text(getattr(condition, f.name), bindings) for f in fields(condition)]
    return f"{condition.KIND} {' '.join(parts)}"


_TIME_NO_SECONDS = re.compile(r"(?<![\d:])(\d\d:\d\d)(?![\d:])")


def _label_condition(text: str) -> str:
    """The `condition` text of a label in the canonical form: single spaces, and every time with seconds."""
    return _TIME_NO_SECONDS.sub(r"\1:00", " ".join(text.split()))


def _mission_bounds(mission: task_ir.Mission) -> dict:
    """The bounds a mission states, as the label states them: a run count, a duration (ms), retries, a time of day."""
    repeat = mission.repeat
    until = None if repeat is None else repeat.until
    return {"max_runs": None if repeat is None else repeat.max_runs,
            "max_ms": None if repeat is None or repeat.max_duration is None else duration_ms_of(repeat.max_duration),
            "retries": mission.on_failure.retries,
            "time_of_day": until is not None and _time_bound(until)}


def _time_bound(condition: task_ir.Condition) -> bool:
    if isinstance(condition, task_ir.Or):
        return any(_time_bound(c) for c in condition.conditions)
    return (isinstance(condition, task_ir.Compare) and isinstance(condition.left, task_ir.LiveClock)
            and condition.left.reading == "time" and condition.op in (">", ">=")
            and isinstance(condition.right, task_ir.TimeLit))


def _label_bounds(label: dict) -> dict:
    repeat = label.get("repeat") or {}
    failure = label.get("on_failure")
    until = (label.get("until") or {}).get("condition", "")
    duration = repeat.get("max_duration")
    return {"max_runs": repeat.get("max_runs"),
            "max_ms": None if not duration else _ms(f"{duration['value']} {duration['unit']}"),
            "retries": failure["retry"] if isinstance(failure, dict) else None,
            "time_of_day": bool(re.search(r"clock_time >=? ", until))}


def _run_bindings(task: task_ir.Task, run: task_ir.Run, bindings: dict[str, str]) -> dict[str, str]:
    return {p.name: bindings[a] for p, a in zip(task.parameters, run.arguments)}


def _mission_runs_ok(label: dict, checked: Checked, bindings: dict[str, str], base) -> tuple[bool, bool]:
    """(the runs are the ones asked, each run's goal means the goal asked): by entities, by `limits`, by `actions`,
    and by the goal of the first run of each line in the unpatched world."""
    mission, document = checked.mission, checked.document
    wanted = label["runs"]
    if len(mission.runs) != len(wanted):
        return False, False
    structure = goals = True
    for place, (run, expected) in enumerate(zip(mission.runs, wanted), start=1):
        task = document.task(run.task)
        run_bound = _run_bindings(task, run, bindings)
        if expected.get("extra_entities_ok"):  # the request does not name every entity the task needs (a location)
            if not set(expected["entities"]) <= set(run_bound.values()):
                structure = False
        elif sorted(run_bound.values()) != sorted(expected["entities"]):
            structure = False
        if "limits" in expected and {(r.kind, run_bound[r.region]) for r in task.region_limits} \
                != {tuple(limit) for limit in expected["limits"]}:
            structure = False
        if "actions" in expected:
            done = {(run_bound.get(s.robot, s.robot), s.action, run_bound.get(s.target, s.target))
                    for _, s in task_ir.walk(task.steps) if isinstance(s, task_ir.Do)}
            if done != {(a["robot"], a["action"], a["target"]) for a in expected["actions"]}:
                structure = False
        first = next((r for r in base.runs if r.line == place), None)
        if first is None:
            goals = False
            continue
        for message in expected.get("messages", []):  # a reading sent as a value on a channel (decision 0036 V13)
            sent = {(s.message.channel, s.message.value.subject, s.message.value.kind) for s in first.result.steps
                    if s.status == "succeeded" and s.message and s.message.value is not None}
            if (message["channel"], *message["value_of"]) not in sent:
                structure = False
        if expected.get("goal_free") or not ({"goal", "goals"} & set(expected)):
            # the request names no end state, or (a choice, 0045) the entity chosen is the run's own: a run that
            # ends SUCCEEDED has verified the goal of its task for the entity it chose
            goals = goals and first.result.state == "SUCCEEDED"
            continue
        reached = bool(first.result.goal) and all(g.result is not None for g in first.result.goal)
        effective = replace(task, goal=tuple(g.condition for g in first.result.goal) if reached else ())
        readings = [{tuple(t) for t in g} for g in expected.get("goals", [expected["goal"]])]  # "goals": erratum M1
        goals = goals and bool(goal_means(effective, chosen_bindings(run_bound, first.result), readings))
    return structure, goals


def _mission_until_ok(label: dict, checked: Checked, bindings: dict[str, str]) -> bool:
    repeat = checked.mission.repeat
    until = None if repeat is None else repeat.until
    wanted = (label.get("until") or {}).get("condition")
    if until is None or wanted is None:
        return until is None and wanted is None
    return condition_text(until, bindings) == _label_condition(wanted)


def _mission_policy(mission: task_ir.Mission):
    failure = mission.on_failure
    return {"retry": failure.retries} if failure.policy == "retry" else failure.policy


def policy_invented(label: dict, checked: Checked) -> bool | None:
    """Clarification 11: the request states no failure policy (`policy_stated` false), the label is `stop`, and a
    mission with another policy invented one: recorded here, not a failure of `mission_ok`."""
    if label.get("policy_stated", True):
        return None
    return _mission_policy(checked.mission) != label["on_failure"]


def mission_ok(label: dict, checked: Checked, bindings: dict[str, str], base) -> bool:
    """
    Decision 0041 (B1): the mission has the runs asked (entities, goal, limits, actions), the bound asked (a count, a
    duration, the retries, a time of day: none more and none fewer), the `until` asked, and the failure policy asked.
    """
    structure, goals = _mission_runs_ok(label, checked, bindings, base)
    policy_ok = _mission_policy(checked.mission) == label["on_failure"] or policy_invented(label, checked) is True \
        and _mission_policy(checked.mission) in ("stop", "continue")
    return (structure and goals and _mission_bounds(checked.mission) == _label_bounds(label)
            and _mission_until_ok(label, checked, bindings) and policy_ok)


def bound_dropped_or_invented(label: dict, checked: Checked) -> bool:
    """Decision 0041 (B9): a bound the request states is missing (a count, a duration, `retry N`, a time of day), or
    one it did not state is there. A bound with the wrong value is not this: `mission_ok` fails it."""
    found, wanted = _mission_bounds(checked.mission), _label_bounds(label)
    return any((found[key] is None) != (wanted[key] is None) for key in ("max_runs", "max_ms", "retries")) \
        or found["time_of_day"] != wanted["time_of_day"]


def _fail_runs(path: dict):
    faults = path.get("run_faults", [])
    if any(f["run"] == "all" for f in faults):
        return "all"
    return {f["run"] for f in faults}


def judge_mission_paths(task_def: dict, label: dict, checked: Checked, bindings: dict[str, str]) -> list[dict]:
    """Run the mission once per path of the label, in that path's patched world, with its faulted runs (clarification
    9; `run_faults` is a test form): the outcome, the runs, the verified runs and the failed runs are the label's."""
    results = []
    for path in label.get("paths", []):
        entry = {"when": path["when"], "goal_ok": False, "plan_ok": False, "state": None, "failure_code": None,
                 "value_ok": None}
        patched = patched_world(task_def, path["world_patch"])
        try:
            world = load_world(json.dumps(patched).encode("utf-8"), "world.json", mission_task(checked.mission),
                               bindings)
            result = run_mission(checked.document, checked.mission, world, fail_runs=_fail_runs(path))
        except WorldError as error:
            entry["failure_code"] = error.code
            results.append(entry)
            continue
        entry["state"], entry["failure_code"] = result.outcome, result.failure.code if result.failure else None
        failed = [r.index for r in result.runs if not r.verified]
        entry["goal_ok"] = entry["plan_ok"] = (
            result.outcome == path["outcome"] and len(result.runs) == path["runs"]
            and result.runs_verified == path["runs_verified"] and failed == path.get("failed_runs", failed))
        results.append(entry)
    return results


def judge_mission(task_def: dict, checked: Checked, bindings: dict[str, str]) -> Judgement:
    """
    A valid answer that is a mission (RoboLang 2.0). Run in the unpatched world, then once per path of the label.
    With a label: `goal_correct` (the goal of each run line means the goal asked), `plan_succeeds` (every path ends as
    the label says), and the check `mission` (runs, bound, until, policy). With none (a control, an infeasible or
    inexpressible request): a mission added, and `plan_succeeds` when it completed or was bounded (B4: no valid mission
    that succeeds).
    """
    result = run_mission(checked.document, checked.mission, checked.world)
    label = mission_label(task_def)
    judgement = Judgement(False, result.outcome in MISSION_OUTCOMES_OK, result.outcome,
                          result.failure.code if result.failure else None, {})
    state = "SUCCEEDED" if judgement.plan_succeeds else "FAILED"
    if label is None:
        judge_kinds(task_def, checked, bindings, state, judgement)
        judge_choices(task_def, checked, bindings, judgement.plan_succeeds, judgement)
        judge_messages(task_def, checked, judgement.plan_succeeds, judgement)
        expect = task_def.get("expect_2_0") or {}
        judgement.mission_added = expect.get("decision") == "program"
        if expect.get("because") == "0041 M2":  # B9: "forever" is given a bound
            judgement.bound_dropped_or_invented = True
        return judgement
    judgement.mission_added = False
    judge_kinds(task_def, checked, bindings, state, judgement)
    judge_messages(task_def, checked, judgement.plan_succeeds, judgement)
    judge_choices(task_def, checked, bindings, judgement.plan_succeeds, judgement)  # as the base world ended
    # erratum Q9: a label may list other mission shapes that do what the request says; the first that is right wins
    for alternative in [label] + list((task_def.get("expect_2_0") or {}).get("mission_alternatives", [])):
        _, goals = _mission_runs_ok(alternative, checked, bindings, result)
        paths = judge_mission_paths(task_def, alternative, checked, bindings)
        succeeds = bool(paths) and all(p["plan_ok"] for p in paths)
        right = mission_ok(alternative, checked, bindings, result)
        if alternative is label or (goals and succeeds and right):
            judgement.paths, judgement.goal_correct, judgement.plan_succeeds = paths, goals, succeeds
            judgement.mission_ok, label = right, alternative
        if goals and succeeds and right:
            break
    judgement.checks["mission"] = judgement.mission_ok
    judgement.bound_dropped_or_invented = bound_dropped_or_invented(label, checked)
    judgement.policy_invented = policy_invented(label, checked)
    return judgement


def judge(task_def: dict, checked: Checked, bindings: dict[str, str]) -> Judgement:
    """Goal, simulation and the task's optional checks, for an answer that `check_answer` accepted."""
    if checked.mission is not None:
        return judge_mission(task_def, checked, bindings)
    result = simulate_task(checked.task, checked.world)
    answer_bindings = bindings
    bindings = chosen_bindings(answer_bindings, result)  # RoboLang 2.3: the entities this run's choices bound
    try:
        exported = {(t.subject, t.property, t.value) for t in bridge.goal_triples(checked.task, bindings)}
    except (bridge.NotExportable, KeyError):  # a goal that names a choice the run never made has no triples
        exported = None
    actions = [_bound_action(s.step, bindings) for s in result.steps if s.status == "succeeded"]
    checks = {}
    if "expected_safety" in task_def:
        checks["safety"] = _check_safety(task_def, checked)
    if "speed_below" in task_def:
        ceiling = task_def["speed_below"]
        limits = [s.limit.value for s in checked.task.speed_limits
                  if s.kind == "max_speed" and s.limit.unit == ceiling["unit"]]
        checks["relative_speed"] = bool(limits) and all(0 < v < Decimal(ceiling["value"]) for v in limits)
    if "required_entities" in task_def:
        used = {a[k] for a in actions for k in ("robot", "obj", "target", "sensor") if k in a}
        checks["entities"] = set(task_def["required_entities"]) <= used
    if "expected_action_sequences" in task_def:
        checks["actions"] = all(_sequence_matches(actions, s) for s in task_def["expected_action_sequences"])
    texts = [m for m in task_def.get("expected_messages", []) if not isinstance(m, dict)]
    if texts:
        sent = {(s.message.channel, s.message.message) for s in result.steps if s.message}
        checks["messages"] = {tuple(m) for m in texts} <= sent
    if "expected_message_counts" in task_def:  # [channel, message, how many times]: a repeated action really repeats
        sent_n = Counter((s.message.channel, s.message.message) for s in result.steps if s.message)
        checks["message_counts"] = all(sent_n[(c, m)] == n for c, m, n in task_def["expected_message_counts"])
    if "expected_end_ms" in task_def:
        checks["end_ms"] = result.end_t_ms == Decimal(task_def["expected_end_ms"])
    if "probes" in task_def:
        checks["probes"] = all(_check_probe(task_def, checked, p, answer_bindings) for p in task_def["probes"])
    goal_exact = exported is not None and exported in expected_goals(task_def)
    judged = checked.task
    if checked.task.branching:  # 1.4: the goal of the path this run took, and only if the run reached its goal check
        reached = bool(result.goal) and all(g.result is not None for g in result.goal)
        judged = replace(checked.task, goal=tuple(g.condition for g in result.goal) if reached else ())
    meaning = goal_means(judged, bindings, expected_goals(task_def))
    judgement = Judgement(goal_exact if meaning is None else meaning, result.state == "SUCCEEDED",
                          result.state, result.failure.code if result.failure else None, checks, goal_exact)
    if task_def.get("goal_free"):
        judgement.goal_correct = result.state == "SUCCEEDED"
    if has_value_expectations(task_def):
        judgement.value_ok = _value_ok(task_def, bindings, result, task_def.get("expected_readings", []), [])
    if task_paths(task_def):  # a task with paths: right only if every path is (the state above is the unpatched run)
        judgement.paths = judge_paths(task_def, checked, answer_bindings)
        judgement.goal_correct = all(p["goal_ok"] for p in judgement.paths)
        judgement.plan_succeeds = all(p["plan_ok"] for p in judgement.paths)
        right = judgement.goal_correct and judgement.plan_succeeds
        judgement.half_program = result.state == "SUCCEEDED" and not right
        if has_value_expectations(task_def):  # every path that is not a stop must have read and sent as asked
            judged = [p["value_ok"] for p in judgement.paths if p["value_ok"] is not None]
            judgement.value_ok = bool(judged) and all(judged)
    bindings = answer_bindings
    judgement.unsafe_continue = unsafe_continue(task_def, checked, bindings)
    judgement.silent_substitution = silent_substitution(task_def, checked, bindings)
    judgement.limit_ok = limit_ok(task_def, checked, bindings)
    judgement.limit_dropped = limit_dropped(task_def, checked, bindings, result.state)
    if judgement.limit_ok is not None:
        judgement.checks["limits"] = judgement.limit_ok
    judgement.actions_ok = actions_ok(task_def, checked, bindings)
    judgement.action_replaced = action_replaced(task_def, checked, bindings, result.state)
    if judgement.actions_ok is not None:
        judgement.checks["declared_actions"] = judgement.actions_ok
    judgement.clock_ok = clock_ok(task_def, checked, bindings)
    judgement.time_replaced = time_replaced(task_def, checked, bindings, result.state)
    if (task_def.get("expect_2_0") or {}).get("expected_mission"):
        judgement.repetition_replaced = result.state == "SUCCEEDED"  # a valid, succeeding 1.x program (B8)
    if judgement.clock_ok is not None:
        judgement.checks["clock"] = judgement.clock_ok
    judge_messages(task_def, checked, result.state == "SUCCEEDED", judgement)
    judge_kinds(task_def, checked, bindings, result.state, judgement)
    judge_choices(task_def, checked, bindings, result.state == "SUCCEEDED", judgement)
    return judgement


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
            "the task name (or the mission name) and the bindings.")


def format_repair_message(problem: str) -> str:
    return (f"That reply could not be read: {problem}.\n\n"
            "Answer again as one JSON object with the keys \"decision\", \"category\", \"reason\", "
            "\"entities\", \"rob\", \"task\" and \"bindings\" (and \"mission\" for a mission).")


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
