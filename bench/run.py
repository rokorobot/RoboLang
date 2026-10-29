"""
Ask Claude models to write RoboLang for the benchmark tasks (set 1: 40 syntax tasks; `--set hard`: 40 robustness
tasks; `--set ask`: 80 tasks that add asking, decision record 0025), check each answer, allow one repair round for
an invalid program, and save every request and response under bench/results/<run-id>/ (decision record 0024).

    python bench/run.py                       # prints the estimated cost and stops
    python bench/run.py --yes                 # spends it (needs ANTHROPIC_API_KEY, or bench/.env)
    python bench/run.py --models claude-haiku-5-5 --tasks e_pick_place,w_two_boxes --yes
    python bench/run.py --set hard --prompt minimal    # set 2, with only the grammar and the answer format

Then `python bench/score.py bench/results/<run-id>` scores the saved answers offline.

Only this file ever imports the `anthropic` SDK, and only when it makes a client.
"""

from __future__ import annotations
import argparse
import hashlib
import json
import math
import os
import re
import sys
import threading
import warnings
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

if __package__ in (None, ""):  # started as `python bench/run.py`
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bench import lib  # noqa: E402

DEFAULT_MODELS = "claude-opus-5-5,claude-sonnet-5-5,claude-haiku-5-5"
EFFORTS = ("low", "medium", "high", "xhigh", "max")
MAX_TOKENS = 16000  # non-streaming: stays under the SDK's HTTP timeout

# Server-side refusal fallbacks (`fallbacks: "default"`) exist for these; Claude Haiku 5.5 has none.
FALLBACK_MODELS = frozenset({"claude-fable-5-1", "claude-opus-5-5", "claude-opus-5", "claude-sonnet-5-5"})
FALLBACK_BETA = "server-side-fallback-2026-07-01"

# Cost estimate assumptions (a guess, printed with the estimate): characters per token for code and prose,
# output tokens per request by effort (thinking plus the answer), and the share of answers that need a repair.
CHARS_PER_TOKEN = 3.5
OUTPUT_TOKENS = {"low": 1500, "medium": 3500, "high": 8000, "xhigh": 16000, "max": 16000}
REPAIR_SHARE = 0.25
REPAIR_EXTRA_INPUT = 900  # the first answer and the diagnostics, sent back
RESULTS_DIR = lib.BENCH / "results"
ENV_FILE = lib.BENCH / ".env"
SECRET_PATTERN = re.compile(r"sk-ant-[A-Za-z0-9_\-]{8,}")


class FatalError(Exception):
    """The run cannot go on (bad key, unknown model, malformed request): stop and keep what was saved."""


# -------- key, client --------

def read_api_key(environ=os.environ, env_file: Path = ENV_FILE) -> str | None:
    """ANTHROPIC_API_KEY from the environment, else from bench/.env (KEY=VALUE lines)."""
    if environ.get("ANTHROPIC_API_KEY"):
        return environ["ANTHROPIC_API_KEY"]
    if env_file.is_file():
        for line in env_file.read_text(encoding="utf-8").splitlines():
            name, _, value = line.strip().partition("=")
            if name.strip() == "ANTHROPIC_API_KEY" and value.strip():
                return value.strip().strip("'\"")
    return None


def make_client(api_key: str):
    """The official SDK client. Imported here so that tests and the scorer never need the package."""
    try:
        import anthropic
    except ImportError:
        raise SystemExit("run.py needs the Anthropic SDK: pip install -r bench/requirements.txt") from None
    return anthropic.Anthropic(api_key=api_key, max_retries=4)


# -------- requests --------

def build_request(model: str, effort: str, system_text: str, messages: list[dict],
                  fallbacks: bool = True, max_tokens: int = MAX_TOKENS) -> dict:
    """The keyword arguments of one `messages.create` call. The long, stable system prompt is cached."""
    request = {
        "model": model,
        "max_tokens": max_tokens,
        "system": [{"type": "text", "text": system_text, "cache_control": {"type": "ephemeral"}}],
        "messages": messages,
        "output_config": {"effort": effort, "format": {"type": "json_schema", "schema": lib.ANSWER_SCHEMA}},
    }
    if fallbacks and model in FALLBACK_MODELS:
        request["betas"] = [FALLBACK_BETA]
        request["fallbacks"] = "default"
    return request


def to_plain(value):
    """An SDK response (or a test's stand-in) as plain JSON data."""
    if hasattr(value, "to_dict"):
        with warnings.catch_warnings():  # pydantic warns about block types newer than its union; the data is right
            warnings.simplefilter("ignore", UserWarning)
            return value.to_dict()
    return json.loads(json.dumps(value))


def error_record(error: Exception) -> dict:
    return {"type": type(error).__name__, "status_code": getattr(error, "status_code", None),
            "request_id": getattr(error, "request_id", None), "message": str(error)}


def send(client, request: dict) -> dict:
    """One call: {"response": plain dict} or {"error": {...}}. The SDK has already retried 429 and 5xx."""
    try:
        create = client.beta.messages.create if "betas" in request else client.messages.create
        return {"response": to_plain(create(**request)), "error": None}
    except Exception as error:  # noqa: BLE001 - any failure becomes data; fatal ones are decided just below
        record = error_record(error)
        # 401/403/404: the key or the model is wrong for every request. 400 is only fatal for a model's first request.
        return {"response": None, "error": record, "fatal": record["status_code"] in (401, 403, 404),
                "bad_request": record["status_code"] == 400}


def run_call(client, kind: str, request: dict) -> dict:
    result = send(client, request)
    call = {"kind": kind, "request": request, "response": result["response"], "error": result["error"]}
    call["served_by"] = lib.served_by(call)
    call["fatal"] = bool(result.get("fatal"))
    call["bad_request"] = bool(result.get("bad_request"))
    return call


def run_answer(client, model: str, task_def: dict, repeat: int, effort: str, system_text: str,
               fallbacks: bool = True) -> dict:
    """One task, one model: request, check, and at most one repair round."""
    user = {"role": "user", "content": lib.user_message(task_def)}
    calls = [run_call(client, "initial", build_request(model, effort, system_text, [user], fallbacks))]
    answer, problem, kind = lib.answer_of(calls[0])
    follow_up = None
    if kind == "answer" and answer["decision"] == "program":  # a decline is final: only invalid programs are repaired
        checked = lib.check_answer(task_def, answer)
        if not checked.valid:
            follow_up = lib.repair_message(checked.report)
    elif kind == "format" and lib.response_text(calls[0]["response"]) is not None:
        follow_up = lib.format_repair_message(problem)
    if follow_up is not None:
        previous = {"role": "assistant", "content": lib.response_text(calls[0]["response"])}
        messages = [user, previous, {"role": "user", "content": follow_up}]
        calls.append(run_call(client, "repair", build_request(model, effort, system_text, messages, fallbacks)))
    return {"task_id": task_def["id"], "model": model, "repeat": repeat, "effort": effort, "calls": calls}


# -------- saving --------

def redact(text: str, secrets: list[str]) -> str:
    """Nothing the API key could appear in is ever written: the key itself, and anything shaped like one."""
    for secret in secrets:
        if secret:
            text = text.replace(secret, "[redacted]")
    return SECRET_PATTERN.sub("[redacted]", text)


def record_path(out_dir: Path, model: str, task_id: str, repeat: int) -> Path:
    return out_dir / model / f"{task_id}.{repeat}.json"


def save_json(path: Path, data, secrets: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(redact(json.dumps(data, indent=2, ensure_ascii=False) + "\n", secrets), encoding="utf-8")


# -------- estimate --------

def tokens_of(text: str) -> int:
    return math.ceil(len(text) / CHARS_PER_TOKEN)


def estimate(models: list[str], tasks: dict[str, dict], effort: str, repeats: int, system_text: str) -> dict:
    """Expected tokens and dollars per model. A guess: the first request writes the cache, the rest read it."""
    system_tokens = tokens_of(system_text)
    users = [tokens_of(lib.user_message(t)) for t in tasks.values()]
    answers = len(tasks) * repeats
    requests = answers + math.ceil(answers * REPAIR_SHARE)
    per_model = {}
    for model in models:
        price_in, price_out, price_read = lib.PRICES[model]
        repairs = math.ceil(answers * REPAIR_SHARE)
        user_tokens = sum(users) * repeats + repairs * (REPAIR_EXTRA_INPUT + sum(users) // len(users))
        output = requests * min(OUTPUT_TOKENS[effort], MAX_TOKENS)
        cost = (system_tokens * price_in * lib.CACHE_WRITE_FACTOR + (requests - 1) * system_tokens * price_read
                + user_tokens * price_in + output * price_out) / 1_000_000
        per_model[model] = {"requests": requests, "input_tokens": system_tokens * requests + user_tokens,
                            "output_tokens": output, "usd": cost}
    return {"models": per_model, "usd": sum(m["usd"] for m in per_model.values()), "system_tokens": system_tokens}


def print_estimate(est: dict, effort: str, out) -> None:
    print(f"Estimated cost (effort {effort}; a guess, see bench/README.md):", file=out)
    for model, row in est["models"].items():
        print(f"  {model:20} {row['requests']:4} requests  ~{row['input_tokens']:>8,} in  "
              f"~{row['output_tokens']:>8,} out  ~${row['usd']:.2f}", file=out)
    print(f"  total ~${est['usd']:.2f}  (system prompt ~{est['system_tokens']:,} tokens, "
          "cached after the first request)", file=out)


# -------- the run --------

def run_benchmark(client, models: list[str], tasks: dict[str, dict], effort: str, repeats: int, out_dir: Path,
                  system_text: str, workers: int = 4, fallbacks: bool = True, secrets: list[str] | None = None,
                  log=sys.stderr, task_set: str = "1", prompt_name: str = "full",
                  max_usd: float | None = None) -> list[dict]:
    """Run every (model, task, repeat) that has no saved record yet; returns the records made now.
    With `max_usd`, no new request starts once the run (including records saved earlier) has cost that much."""
    secrets = secrets or []
    meta = {"run_id": out_dir.name, "set": task_set, "prompt": prompt_name, "models": models, "effort": effort,
            "repeats": repeats, "tasks": list(tasks), "fallbacks": fallbacks, "max_tokens": MAX_TOKENS,
            "prompt_sha256": hashlib.sha256(system_text.encode("utf-8")).hexdigest(),
            "started": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")}
    meta_path = out_dir / "run.json"
    if meta_path.exists():
        old = json.loads(meta_path.read_text(encoding="utf-8"))
        old = {"set": "1", "prompt": "full", **old}  # runs saved before set 2 have neither key
        if any(old[k] != meta[k] for k in ("set", "prompt", "models", "effort", "repeats", "tasks", "prompt_sha256")):
            raise FatalError(f"{out_dir} holds a different run; choose another --run-id")
    else:
        save_json(meta_path, meta, secrets)
    jobs = [(m, t, r) for m in models for t in tasks.values() for r in range(1, repeats + 1)
            if not record_path(out_dir, m, t["id"], r).exists()]
    made = []
    spent = sum(lib.call_cost(c) or 0 for p in out_dir.glob("*/*.json")
                for c in json.loads(p.read_text(encoding="utf-8"))["calls"])
    lock = threading.Lock()

    stop = threading.Event()  # set by a fatal error: requests not yet started are skipped, so nothing more is spent

    def work(job, gate: bool = False) -> dict | None:
        nonlocal spent
        model, task_def, repeat = job
        with lock:
            if max_usd is not None and spent >= max_usd and not stop.is_set():
                stop.set()
                print(f"  stopped at the cap: ${spent:.2f} >= ${max_usd:.2f}; rerun with the same --run-id to resume",
                      file=log)
        if stop.is_set():
            return None
        record = run_answer(client, model, task_def, repeat, effort, system_text, fallbacks)
        save_json(record_path(out_dir, model, task_def["id"], repeat), record, secrets)
        with lock:
            spent += sum(lib.call_cost(c) or 0 for c in record["calls"])
        status = "ok" if record["calls"][-1].get("error") is None else "request failed"
        print(f"  {model} {task_def['id']} #{repeat}: {len(record['calls'])} call(s), {status}", file=log)
        fatal = [c for c in record["calls"] if c["fatal"] or (gate and c["bad_request"])]
        if fatal:
            stop.set()
            error = fatal[0]["error"]
            raise FatalError(f"{model} {task_def['id']}: {error['type']} {error['status_code']}: {error['message']}")
        return record

    first = {}
    for job in jobs:  # the first request of each model runs alone: it writes the prompt cache and finds config errors
        first.setdefault(job[0], job)
    for job in first.values():
        record = work(job, gate=True)
        if record is not None:
            made.append(record)
    rest = [job for job in jobs if job not in first.values()]
    with ThreadPoolExecutor(max_workers=max(1, workers)) as pool:
        made += [record for record in pool.map(work, rest) if record is not None]
    return made


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Ask Claude models to write RoboLang for the benchmark tasks.")
    parser.add_argument("--models", default=DEFAULT_MODELS, help="comma-separated model IDs (default: %(default)s)")
    parser.add_argument("--effort", default="medium", choices=EFFORTS, help="output_config.effort (default: medium)")
    parser.add_argument("--repeats", type=int, default=1, help="answers per task and model (default: 1)")
    parser.add_argument("--set", dest="task_set", default="1", choices=tuple(lib.SETS),
                        help="task set: 1 (syntax, the default) or hard (robustness)")
    parser.add_argument("--prompt", dest="prompt_name", default="full", choices=lib.PROMPTS,
                        help="system prompt: full (prompt.md) or minimal (only the grammar and the answer format)")
    parser.add_argument("--tasks", help="comma-separated task IDs (default: the whole set)")
    parser.add_argument("--workers", type=int, default=4, help="parallel requests (default: 4)")
    parser.add_argument("--run-id", help="results directory name; an existing one is resumed (default: UTC time)")
    parser.add_argument("--out", type=Path, default=RESULTS_DIR, help="results parent directory")
    parser.add_argument("--no-fallbacks", action="store_true", help="do not enable server-side refusal fallbacks")
    parser.add_argument("--yes", action="store_true", help="spend the estimated cost; without it nothing is sent")
    parser.add_argument("--max-usd", type=float, help="start no new request once the run has cost this much")
    return parser.parse_args(argv)


def main(argv=None, client_factory=make_client, out=sys.stdout, err=sys.stderr, environ=os.environ,
         env_file: Path = ENV_FILE) -> int:
    args = parse_args(argv)
    models = [m.strip() for m in args.models.split(",") if m.strip()]
    unknown = [m for m in models if m not in lib.PRICES]
    if unknown or not models or args.repeats < 1:
        print(f"unknown model(s) {unknown} (known: {', '.join(lib.PRICES)}) or bad --repeats", file=err)
        return 2
    tasks = lib.load_set(args.task_set)
    if args.tasks:
        wanted = [t.strip() for t in args.tasks.split(",")]
        missing = [t for t in wanted if t not in tasks]
        if missing:
            print(f"unknown task(s): {', '.join(missing)}", file=err)
            return 2
        tasks = {t: tasks[t] for t in wanted}
    system_text = lib.load_prompt(args.prompt_name)
    print_estimate(estimate(models, tasks, args.effort, args.repeats, system_text), args.effort, out)
    if not args.yes:
        print("Nothing was sent. Add --yes to spend it.", file=out)
        return 2
    key = read_api_key(environ, env_file)
    if not key:
        print("No API key: set ANTHROPIC_API_KEY or put it in bench/.env", file=err)
        return 2
    out_dir = args.out / (args.run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    try:
        run_benchmark(client_factory(key), models, tasks, args.effort, args.repeats, out_dir, system_text,
                      args.workers, not args.no_fallbacks, [key], err, args.task_set, args.prompt_name, args.max_usd)
    except FatalError as error:
        print(f"stopped: {error}\nWhat was saved is in {out_dir}", file=err)
        return 1
    print(f"Saved to {out_dir}. Score it with: python bench/score.py {out_dir}", file=out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
