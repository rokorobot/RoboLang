"""
The authoring benchmark (bench/, decision record 0024): the 40 tasks and their reference solutions, the
scorer on canned answers, the repair round, and the guarantees about the API key.

Runs in CI without the `anthropic` package and without network: `bench/run.py` is driven by a fake client.
"""

from __future__ import annotations
import io
import json
import sys
import threading
import tomllib
from pathlib import Path
from types import SimpleNamespace

import pytest

from bench import lib, run, score

REPO = Path(__file__).resolve().parent.parent
TASKS = lib.load_tasks()
KEY = "sk-ant-api03-TESTKEY-0123456789abcdef"


# -------- the 40 tasks --------

def reference_answer(task_id: str) -> dict:
    ref = json.loads((lib.REFERENCE_DIR / f"{task_id}.json").read_text(encoding="utf-8"))
    return {"rob": (lib.REFERENCE_DIR / f"{task_id}.rob").read_text(encoding="utf-8"), **ref}


def test_there_are_forty_tasks_each_with_a_reference():
    assert len(TASKS) == 40
    assert {p.stem for p in lib.REFERENCE_DIR.glob("*.rob")} == set(TASKS)
    assert {p.stem for p in lib.REFERENCE_DIR.glob("*.json")} == set(TASKS)


def test_task_files_are_well_formed():
    for task_id, task in TASKS.items():
        assert {"id", "prompt", "world", "expected_goal", "tags"} <= set(task), task_id
        assert task["id"] == task_id and (lib.TASKS_DIR / f"{task_id}.json").is_file()
        assert (REPO / task["world"]).is_file() and task["tags"] and task["prompt"].endswith((".", "?"))
        assert task["expected_goal"] and all(len(t) == 3 for t in task["expected_goal"])
        assert {t[1] for t in task["expected_goal"]} <= {"located_in", "carried_by"}


def test_tasks_cover_the_required_features():
    tags = {t for task in TASKS.values() for t in task["tags"]}
    assert {"pick_place", "inspect", "sensor_param", "wait_until", "timeout", "precondition", "and", "or", "not",
            "safety", "units_in_words", "robot_at", "holding", "multi_condition", "handover", "two_robot",
            "multiple_objects", "tricky", "tricky_synonym", "implied_precondition", "wait_for", "communicate"} <= tags
    assert {t["world"] for t in TASKS.values()} == {"examples/world.json", "bench/worlds/warehouse.json",
                                                   "bench/worlds/lab.json", "bench/worlds/assembly.json"}


@pytest.mark.parametrize("task_id", sorted(TASKS))
def test_reference_is_valid_exports_the_expected_goal_and_succeeds(task_id):
    task = TASKS[task_id]
    answer = reference_answer(task_id)
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == "1.3"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct
    assert judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert all(judgement.checks.values()), judgement.checks


def test_the_optional_checks_bite():
    """A plan that ignores what the request says about the world fails the task's probe worlds and limits."""
    task = TASKS["e_pick_place_cell_clear"]
    answer = reference_answer("e_pick_place_cell_clear")
    answer["rob"] = answer["rob"].replace("pre { region_clear cell; }", "")
    judgement = lib.judge(task, lib.check_answer(task, answer), answer["bindings"])
    assert judgement.plan_succeeds and judgement.goal_correct and judgement.checks == {"probes": False}

    task = TASKS["e_speed_half"]
    answer = reference_answer("e_speed_half")
    answer["rob"] = answer["rob"].replace("0.5 m/s", "0.4 m/s")
    assert lib.judge(task, lib.check_answer(task, answer), answer["bindings"]).checks == {"safety": False}
    # above the robot's profile: `validate --world` rejects it before any simulation (N2)
    answer["rob"] = answer["rob"].replace("0.4 m/s", "5 m/s")
    checked = lib.check_answer(task, answer)
    assert not checked.valid and checked.codes == ("S001",)
    assert checked.report.startswith("robolang validate answer.rob --world world.json --task ")
    assert "answer.rob:" in checked.report and "error[S001]" in checked.report

    task = TASKS["e_wait_zone"]
    answer = reference_answer("e_wait_zone")
    answer["rob"] = answer["rob"].replace("wait until region_clear zone timeout 30s;", "")
    judgement = lib.judge(task, lib.check_answer(task, answer), answer["bindings"])
    assert judgement.plan_succeeds and judgement.checks == {"probes": False}


def test_every_world_entity_is_shown_to_the_model():
    for task in TASKS.values():
        world = json.loads(lib.world_bytes(task))
        message = lib.user_message(task)
        ids = list(world["locations"])
        ids += [n for key in ("regions", "robots", "objects", "sensors") for n in world.get(key, {})]
        assert all(i in message for i in ids), task["id"]
        assert message.endswith(task["prompt"])


def test_the_prompts_example_is_valid_and_runs():
    text = lib.PROMPT_FILE.read_text(encoding="utf-8")
    example = text.split("# Example")[1].split('    {"decision"', 1)[1]
    answer, problem = lib.parse_answer('{"decision"' + example)  # read exactly as a model's reply is read
    assert problem is None, problem
    world = {"world_schema": "robolang.sim.world", "world_schema_version": "1.1",
             "locations": ["dock", "shelf", "table"], "regions": {"cell": {"clear": False}},
             "robots": {"arm1": {"ready": True, "location": "dock", "gripper": True,
                                 "max_speed": [{"value": "1", "unit": "m/s"}]}},
             "objects": {"box1": {"present": True, "location": "shelf"}},
             "sensors": {"cam1": {"robot": "arm1", "available": True}},
             "events": [{"at": {"value": "10", "unit": "s"}, "fact": "region_clear", "target": "cell", "value": True}],
             "tasks": {}}
    checked = lib.check_answer({}, answer, json.dumps(world).encode())
    assert checked.valid, checked.report
    from robolang.sim import run as simulate
    assert simulate(checked.task, checked.world).state == "SUCCEEDED"
    assert text.startswith("You write RoboLang 1.3 programs.")


# -------- the fake client --------

def message(answer, model="claude-opus-5-5", stop_reason="end_turn", usage=None, content=None, **extra) -> dict:
    text = answer if isinstance(answer, str) else json.dumps(answer)
    return {"id": "msg_test", "type": "message", "role": "assistant", "model": model, "stop_reason": stop_reason,
            "content": content if content is not None else [{"type": "thinking", "thinking": ""},
                                                            {"type": "text", "text": text}],
            "usage": usage or {"input_tokens": 100, "cache_creation_input_tokens": 0,
                               "cache_read_input_tokens": 3000, "output_tokens": 500}, **extra}


class FakeClient:
    """Stands in for `anthropic.Anthropic`: `script(request)` returns a reply (a dict) or an exception to raise."""

    def __init__(self, script):
        self.script, self.requests, self._lock = script, [], threading.Lock()
        self.messages = self.beta = None
        owner = self

        class Messages:
            def create(self, **request):
                with owner._lock:
                    owner.requests.append(request)
                result = owner.script(request)
                if isinstance(result, Exception):
                    raise result
                return result

        self.messages = Messages()
        self.beta = SimpleNamespace(messages=Messages())


def task_of(request: dict) -> str:
    """The task a request is about, from the request text in its first user message."""
    first = request["messages"][0]["content"]
    return next(t for t, d in TASKS.items() if first.endswith(f"Request: {d['prompt']}"))


def scripted(first: dict[str, dict], second: dict[str, dict] | None = None, model="claude-opus-5-5"):
    """task ID -> answer for the first request, and (after a repair round) for the second."""
    def script(request):
        round_two = len(request["messages"]) > 1
        table = (second or {}) if round_two else first
        return message(table[task_of(request)], model=request["model"] if model is None else model)
    return script


def run_to(tmp_path: Path, client: FakeClient, models=("claude-opus-5-5",), tasks=None, run_id="test", **kw) -> Path:
    chosen = {t: TASKS[t] for t in (tasks or TASKS)}
    out_dir = tmp_path / run_id
    system = lib.PROMPT_FILE.read_text(encoding="utf-8")
    run.run_benchmark(client, list(models), chosen, "medium", 1, out_dir, system, workers=1, log=io.StringIO(), **kw)
    return out_dir


# -------- the scorer on canned answers --------

def variant(task_id: str, drop: str = "", **bindings) -> dict:
    """The reference answer with some source text removed and some parameters bound elsewhere."""
    answer = reference_answer(task_id)
    return dict(answer, rob=answer["rob"].replace(drop, "") if drop else answer["rob"],
                bindings=dict(answer["bindings"], **bindings))


def canned_run(tmp_path: Path, **kw) -> tuple[Path, FakeClient]:
    first = {
        "e_pick_place": reference_answer("e_pick_place"),                           # valid at once
        "e_hold_box": variant("e_hold_box", drop="goal { holding r box; }"),        # no goal (E309), repaired below
        "e_park_amr": variant("e_park_amr", home="dock"),                           # valid, but ends at the dock
        "w_clear_dock": variant("w_clear_dock", r="dock"),                          # a location bound to a robot (E507)
        "e_speed_half": variant("e_speed_half", drop="        move r to src;\n"),   # grasps from afar (S215)
    }
    second = {
        "e_hold_box": reference_answer("e_hold_box"),                               # repaired
        "w_clear_dock": variant("w_clear_dock", r="dock"),                          # still wrong
    }
    client = FakeClient(scripted(first, second))
    return run_to(tmp_path, client, tasks=list(first), **kw), client


def test_scorer_on_canned_answers(tmp_path):
    out_dir, client = canned_run(tmp_path)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}

    ok = by["e_pick_place"]  # valid, right goal, succeeds
    assert (ok["valid_first"], ok["valid_after_repair"], ok["goal_correct"], ok["plan_succeeds"]) == (True,) * 4
    assert ok["repaired"] is False and ok["codes"] == []

    repaired = by["e_hold_box"]  # invalid, then repaired
    assert (repaired["valid_first"], repaired["valid_after_repair"]) == (False, True)
    assert repaired["goal_correct"] and repaired["plan_succeeds"] and repaired["codes"] == ["E309"]

    wrong_goal = by["e_park_amr"]  # valid, runs, but ends somewhere else
    assert wrong_goal["valid_first"] and wrong_goal["plan_succeeds"] and not wrong_goal["goal_correct"]

    wrong_bindings = by["w_clear_dock"]  # a location bound to a robot parameter, twice
    assert (wrong_bindings["valid_first"], wrong_bindings["valid_after_repair"]) == (False, False)
    assert not wrong_bindings["goal_correct"] and not wrong_bindings["plan_succeeds"]
    assert wrong_bindings["codes"] == ["E507", "E507"]

    plan_fails = by["e_speed_half"]  # right goal, but the plan cannot reach it
    assert plan_fails["valid_first"] and plan_fails["goal_correct"] and not plan_fails["plan_succeeds"]
    assert plan_fails["failure_code"] == "S215" and plan_fails["checks_ok"] is True

    text = score.score_dir(out_dir)
    assert (out_dir / "report.md").read_text(encoding="utf-8") == text
    for name in score.MEASURES:
        assert name in text
    assert "| valid_first | 3/5 | 3/5 (60.0%) |" in text and "| valid_after_repair | 4/5 | 4/5 (80.0%) |" in text
    assert "| goal_correct | 3/5 |" in text and "| plan_succeeds | 3/5 |" in text
    assert "| E507 | 2 |" in text and "| E309 | 1 |" in text and "| pick_place |" in text
    assert "| input (uncached) |" in text and "$0." in text


def test_the_report_is_deterministic(tmp_path):
    out_dir, _ = canned_run(tmp_path)
    first = score.score_dir(out_dir)
    assert score.score_dir(out_dir) == first
    assert score.main([str(out_dir)], out=io.StringIO()) == 0


def test_scoring_needs_no_network_and_no_sdk():
    assert "anthropic" not in sys.modules
    source = "".join((REPO / "bench" / f"{name}.py").read_text(encoding="utf-8") for name in ("score", "lib"))
    for module in ("socket", "ssl", "http", "urllib", "requests", "httpx", "anthropic"):
        assert f"import {module}" not in source and f"from {module}" not in source


# -------- the repair round --------

def test_the_repair_round_sends_the_exact_diagnostics(tmp_path):
    out_dir, client = canned_run(tmp_path)
    repairs = [r for r in client.requests if len(r["messages"]) > 1]
    assert len(repairs) == 2  # e_hold_box and w_clear_dock; the others were valid or are not repaired
    by_task = {task_of(r): r for r in repairs}

    request = by_task["e_hold_box"]
    first_reply = json.loads(request["messages"][1]["content"])
    assert first_reply["task"] == "pick_up" and "goal" not in first_reply["rob"]
    assert request["messages"][1]["role"] == "assistant" and request["messages"][2]["role"] == "user"
    diagnostics = lib.check_answer(TASKS["e_hold_box"], first_reply).report
    assert "error[E309]" in diagnostics and diagnostics.startswith("robolang validate answer.rob")
    assert diagnostics in request["messages"][2]["content"]
    assert "answer.rob:" in request["messages"][2]["content"]  # the same text `robolang validate` prints

    binding = by_task["w_clear_dock"]["messages"][2]["content"]
    assert "error[E507]" in binding and "/bindings/r" in binding

    saved = json.loads((out_dir / "claude-opus-5-5" / "e_hold_box.1.json").read_text(encoding="utf-8"))
    assert [c["kind"] for c in saved["calls"]] == ["initial", "repair"]


def test_a_valid_first_answer_gets_no_repair_round(tmp_path):
    client = FakeClient(scripted({"e_pick_place": reference_answer("e_pick_place")}))
    run_to(tmp_path, client, tasks=["e_pick_place"])
    assert len(client.requests) == 1


def test_an_unreadable_reply_is_repaired_once(tmp_path):
    replies = iter(["not json at all", reference_answer("e_pick_place")])
    client = FakeClient(lambda request: message(next(replies)))
    out_dir = run_to(tmp_path, client, tasks=["e_pick_place"])
    assert len(client.requests) == 2 and "could not be read" in client.requests[1]["messages"][2]["content"]
    row = score.load_rows(out_dir)[1][0]
    assert (row["valid_first"], row["valid_after_repair"], row["plan_succeeds"]) == (False, True, True)
    assert row["codes"] == [lib.FORMAT_CODE]


def test_at_most_one_repair_round(tmp_path):
    bad = dict(reference_answer("e_pick_place"), task="nope")
    client = FakeClient(lambda request: message(bad))
    out_dir = run_to(tmp_path, client, tasks=["e_pick_place"])
    assert len(client.requests) == 2
    assert score.load_rows(out_dir)[1][0]["codes"] == [lib.NO_TASK_CODE, lib.NO_TASK_CODE]


# -------- the request --------

def test_request_uses_structured_output_effort_and_a_cached_system_prompt(tmp_path):
    client = FakeClient(scripted({"e_pick_place": reference_answer("e_pick_place")}))
    run_to(tmp_path, client, tasks=["e_pick_place"])
    request = client.requests[0]
    assert request["output_config"]["effort"] == "medium"
    assert request["output_config"]["format"] == {"type": "json_schema", "schema": lib.ANSWER_SCHEMA}
    assert request["system"] == [{"type": "text", "text": lib.PROMPT_FILE.read_text(encoding="utf-8"),
                                  "cache_control": {"type": "ephemeral"}}]
    assert request["max_tokens"] == 16000
    for name in ("temperature", "top_p", "top_k", "thinking", "api_key", "prefill"):
        assert name not in request


def test_opus_and_sonnet_get_refusal_fallbacks_and_haiku_does_not(tmp_path):
    models = ("claude-opus-5-5", "claude-sonnet-5-5", "claude-haiku-5-5")
    client = FakeClient(lambda request: message(reference_answer("e_pick_place"), model=request["model"]))
    run_to(tmp_path, client, models=models, tasks=["e_pick_place"])
    by_model = {r["model"]: r for r in client.requests}
    for model in ("claude-opus-5-5", "claude-sonnet-5-5"):
        assert by_model[model]["fallbacks"] == "default" and by_model[model]["betas"] == [run.FALLBACK_BETA]
    assert "fallbacks" not in by_model["claude-haiku-5-5"] and "betas" not in by_model["claude-haiku-5-5"]
    assert run.FALLBACK_BETA == "server-side-fallback-2026-07-01"
    assert set(models) - run.FALLBACK_MODELS == {"claude-haiku-5-5"}


def test_no_fallbacks_flag(tmp_path):
    client = FakeClient(lambda request: message(reference_answer("e_pick_place")))
    run_to(tmp_path, client, tasks=["e_pick_place"], fallbacks=False)
    assert "fallbacks" not in client.requests[0]


# -------- fallback-served answers are scored separately --------

def test_fallback_served_answers_are_scored_in_their_own_table(tmp_path):
    fallback_usage = {"input_tokens": 0, "cache_creation_input_tokens": 0, "cache_read_input_tokens": 0,
                      "output_tokens": 0,
                      "iterations": [
                          {"type": "message", "model": "claude-opus-5-5", "input_tokens": 10,
                           "cache_creation_input_tokens": 0, "cache_read_input_tokens": 0, "output_tokens": 0},
                          {"type": "fallback_message", "model": "claude-opus-4-8", "input_tokens": 100,
                           "cache_creation_input_tokens": 0, "cache_read_input_tokens": 3000, "output_tokens": 500}]}

    def script(request):
        if task_of(request) == "e_hold_box":
            return message(reference_answer("e_hold_box"), model="claude-opus-4-8", usage=fallback_usage,
                           content=[{"type": "fallback", "from": {"model": "claude-opus-5-5"},
                                     "to": {"model": "claude-opus-4-8"}},
                                    {"type": "text", "text": json.dumps(reference_answer("e_hold_box"))}])
        return message(reference_answer("e_pick_place"))

    out_dir = run_to(tmp_path, FakeClient(script), tasks=["e_pick_place", "e_hold_box"])
    rows = {r["task_id"]: r for r in score.load_rows(out_dir)[1]}
    assert rows["e_pick_place"]["group"] == "requested" and rows["e_hold_box"]["group"] == "fallback"
    assert rows["e_hold_box"]["served_by"] == ["claude-opus-4-8"]
    # cost sums the attempts, each at its own model's price
    expected = lib.usd("claude-opus-5-5", {"input_tokens": 10, "cache_creation_input_tokens": 0,
                                           "cache_read_input_tokens": 0, "output_tokens": 0}) \
        + lib.usd("claude-opus-4-8", {"input_tokens": 100, "cache_creation_input_tokens": 0,
                                      "cache_read_input_tokens": 3000, "output_tokens": 500})
    assert rows["e_hold_box"]["usd"] == pytest.approx(expected)
    text = score.report(*score.load_rows(out_dir))
    summary = text.split("## Summary")[1].split("\n## ")[0]
    assert "| claude-opus-5-5 | 1 |" in summary          # the fallback-served answer is not the requested model's
    assert "answers served by a fallback model (claude-opus-4-8)" in text


def test_a_refusal_is_a_failed_answer_with_no_repair(tmp_path):
    refusal = message("", model="claude-haiku-5-5", stop_reason="refusal", content=[],
                      stop_details={"type": "refusal", "category": "cyber"})
    client = FakeClient(lambda request: refusal)
    out_dir = run_to(tmp_path, client, tasks=["e_pick_place"], models=("claude-haiku-5-5",))
    assert len(client.requests) == 1
    row = score.load_rows(out_dir)[1][0]
    assert row["outcome"] == "refusal" and row["codes"] == [lib.REFUSED_CODE] and row["group"] == "requested"
    assert not any(row[m] for m in score.MEASURES)


# -------- cost --------

def test_cost_uses_the_price_table():
    call = {"request": {"model": "claude-sonnet-5-5"},
            "response": {"model": "claude-sonnet-5-5",
                         "usage": {"input_tokens": 1_000_000, "cache_creation_input_tokens": 1_000_000,
                                   "cache_read_input_tokens": 1_000_000, "output_tokens": 1_000_000}}}
    assert lib.call_cost(call) == pytest.approx(2.00 + 2.00 * 1.25 + 0.20 + 10.00)
    unpriced = {"request": {"model": "claude-nothing"}, "response": {"usage": {"input_tokens": 1}}}
    assert lib.call_cost(unpriced) is None
    assert set(lib.PRICES) >= {"claude-opus-5-5", "claude-sonnet-5-5", "claude-haiku-5-5"}


def test_the_estimate_is_printed_and_needs_yes_to_spend(tmp_path):
    made = []
    out, err = io.StringIO(), io.StringIO()
    code = run.main(["--out", str(tmp_path)], client_factory=lambda key: made.append(key), out=out, err=err,
                    environ={"ANTHROPIC_API_KEY": KEY})
    assert code == 2 and made == [] and list(tmp_path.iterdir()) == []
    text = out.getvalue()
    assert "Estimated cost" in text and "claude-opus-5-5" in text and "total ~$" in text and "--yes" in text
    estimate = run.estimate(["claude-opus-5-5"], TASKS, "medium", 1, lib.PROMPT_FILE.read_text(encoding="utf-8"))
    assert 0 < estimate["usd"] < 20


def test_no_key_no_run(tmp_path):
    code = run.main(["--yes", "--out", str(tmp_path)], client_factory=lambda key: pytest.fail("no client"),
                    out=io.StringIO(), err=io.StringIO(), environ={}, env_file=tmp_path / "no.env")
    assert code == 2


def test_key_is_read_from_the_environment_or_bench_env(tmp_path):
    env_file = tmp_path / ".env"
    env_file.write_text("# comment\nANTHROPIC_API_KEY='from-file'\n", encoding="utf-8")
    assert run.read_api_key({"ANTHROPIC_API_KEY": "from-env"}, env_file) == "from-env"
    assert run.read_api_key({}, env_file) == "from-file"
    assert run.read_api_key({}, tmp_path / "missing") is None


# -------- the key never reaches the results --------

def test_results_never_contain_the_key(tmp_path):
    def script(request):
        if task_of(request) == "e_hold_box":  # an error message that echoes the key, as a proxy might
            return RuntimeError(f"upstream said: x-api-key {KEY} is overloaded")
        return message(reference_answer(task_of(request)))

    out, err = io.StringIO(), io.StringIO()
    client = FakeClient(script)
    code = run.main(["--yes", "--tasks", "e_pick_place,e_hold_box,e_park_amr", "--models", "claude-opus-5-5",
                     "--out", str(tmp_path), "--run-id", "k", "--workers", "1"],
                    client_factory=lambda key: client, out=out, err=err, environ={"ANTHROPIC_API_KEY": KEY})
    assert code == 0
    files = list((tmp_path / "k").rglob("*.json"))
    assert len(files) == 4  # run.json and three records
    for path in files:
        assert KEY not in path.read_text(encoding="utf-8") and "sk-ant-" not in path.read_text(encoding="utf-8")
    assert KEY not in out.getvalue() and KEY not in err.getvalue()
    failed = json.loads((tmp_path / "k" / "claude-opus-5-5" / "e_hold_box.1.json").read_text(encoding="utf-8"))
    assert "[redacted]" in failed["calls"][0]["error"]["message"] and failed["calls"][0]["response"] is None
    assert score.load_rows(tmp_path / "k")[1][0]["codes"] == [lib.API_ERROR_CODE]
    for request in client.requests:
        assert KEY not in json.dumps(request)


def test_a_fatal_error_stops_the_run_and_keeps_what_was_saved(tmp_path):
    class Unauthorized(Exception):
        status_code = 401

    client = FakeClient(lambda request: Unauthorized("invalid x-api-key"))
    err = io.StringIO()
    code = run.main(["--yes", "--tasks", "e_pick_place,e_hold_box", "--models", "claude-opus-5-5,claude-haiku-5-5",
                     "--out", str(tmp_path), "--run-id", "f"],
                    client_factory=lambda key: client, out=io.StringIO(), err=err, environ={"ANTHROPIC_API_KEY": KEY})
    assert code == 1 and "stopped" in err.getvalue() and len(client.requests) == 1


def test_an_interrupted_run_resumes_without_asking_again(tmp_path):
    client = FakeClient(lambda request: message(reference_answer("e_pick_place")))
    out_dir = run_to(tmp_path, client, tasks=["e_pick_place"], run_id="r")
    again = FakeClient(lambda request: pytest.fail("saved answers are not asked again"))
    run_to(tmp_path, again, tasks=["e_pick_place"], run_id="r")
    assert len(client.requests) == 1 and len(list(out_dir.rglob("*.json"))) == 2


# -------- the package stays clean --------

def test_the_sdk_is_not_imported_and_not_a_package_dependency():
    assert "anthropic" not in sys.modules
    pyproject = tomllib.loads((REPO / "pyproject.toml").read_text(encoding="utf-8"))
    assert pyproject["project"]["dependencies"] == []
    assert "anthropic" not in json.dumps(pyproject["project"].get("optional-dependencies", {}))
    assert "anthropic" not in (REPO / "requirements.txt").read_text(encoding="utf-8")
    assert "anthropic" not in (REPO / "requirements-dev.txt").read_text(encoding="utf-8")
    assert "anthropic" in (REPO / "bench" / "requirements.txt").read_text(encoding="utf-8")
    assert "bench" not in " ".join(pyproject["tool"]["setuptools"]["packages"])


def test_the_installed_package_does_not_import_bench():
    for path in (REPO / "src").rglob("*.py"):
        source = path.read_text(encoding="utf-8")
        assert "import bench" not in source and "from bench" not in source


def test_bindings_are_read_from_the_list_form_and_checked():
    answer, problem = lib.parse_answer(json.dumps(
        {"rob": "x", "task": "t", "bindings": [{"param": "r", "id": "arm1"}, {"param": "box", "id": "box1"}]}))
    assert problem is None and answer["bindings"] == {"r": "arm1", "box": "box1"}
    for bad in ([{"param": "r", "id": "a"}, {"param": "r", "id": "b"}], [{"param": "r"}], ["r=arm1"]):
        assert lib.parse_answer(json.dumps({"rob": "x", "task": "t", "bindings": bad}))[0] is None


def test_the_answer_schema_meets_structured_output_rules():
    """Every object in the schema closes its properties: the API rejects `additionalProperties` that is not false."""
    def objects(node):
        if isinstance(node, dict):
            if node.get("type") == "object":
                yield node
            for value in node.values():
                yield from objects(value)
    assert all(o.get("additionalProperties") is False for o in objects(lib.ANSWER_SCHEMA))

