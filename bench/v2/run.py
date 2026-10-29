"""
Ask Claude models to answer the 80 BENCH-2 cases (decision record 0025): a program, a rejection or a question.
Invalid programs get one repair round. Every request and response is saved under bench/results/<run-id>/.

    python bench/v2/run.py                                  # prints the estimated cost and stops
    python bench/v2/run.py --yes --max-usd 10               # the pilot: full prompt, 1 repeat
    python bench/v2/run.py --variant minimal --repeats 3 --yes --max-usd 50

Then `python bench/v2/score.py bench/results/<run-id>`. Reuses `bench/run.py` for the key, client, sending and
saving; that file is not changed.
"""

from __future__ import annotations
import argparse
import hashlib
import json
import os
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path

if __package__ in (None, ""):  # started as `python bench/v2/run.py`
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from bench import lib as v1  # noqa: E402
from bench import run as v1run  # noqa: E402
from bench.v2 import lib  # noqa: E402

# Calibration (0025 design §4.1): run3's real output was ~250 tokens per answer for Opus and Sonnet and ~740 for
# Haiku at effort medium; BENCH-2 answers are assumed about 3x that. A repair round costs about 1.5 calls.
OUTPUT_TOKENS = {"claude-haiku-5-5": 1500}
DEFAULT_OUTPUT = 800
REPAIR_SHARE = 0.10


def build_request(model: str, effort: str, system_text: str, messages: list[dict], fallbacks: bool = True) -> dict:
    request = v1run.build_request(model, effort, system_text, messages, fallbacks)
    request["output_config"]["format"]["schema"] = lib.ANSWER_SCHEMA
    return request


def run_answer(client, model: str, case: dict, repeat: int, effort: str, system_text: str,
               fallbacks: bool = True) -> dict:
    user = {"role": "user", "content": lib.user_message(case)}
    calls = [v1run.run_call(client, "initial", build_request(model, effort, system_text, [user], fallbacks))]
    answer, problem, kind = lib.answer_of(calls[0])
    follow_up = None
    if kind == "answer" and answer["kind"] == "program":
        checked = v1.check_answer(case, answer)
        if not checked.valid:
            follow_up = lib.repair_message(checked.report)
    elif kind == "format" and v1.response_text(calls[0]["response"]) is not None:
        follow_up = lib.format_repair_message(problem)
    if follow_up is not None:
        previous = {"role": "assistant", "content": v1.response_text(calls[0]["response"])}
        messages = [user, previous, {"role": "user", "content": follow_up}]
        calls.append(v1run.run_call(client, "repair", build_request(model, effort, system_text, messages, fallbacks)))
    return {"case_id": case["id"], "model": model, "repeat": repeat, "effort": effort, "calls": calls}


def estimate(models: list[str], cases: dict[str, dict], repeats: int, system_text: str) -> dict:
    system = v1run.tokens_of(system_text)
    user = sum(v1run.tokens_of(lib.user_message(c)) for c in cases.values()) / len(cases)
    answers = len(cases) * repeats
    per_model = {}
    for model in models:
        price_in, price_out, price_read = v1.PRICES[model]
        out = OUTPUT_TOKENS.get(model, DEFAULT_OUTPUT)
        one = (user * price_in + system * price_read + out * price_out) / 1_000_000
        per_model[model] = answers * one * (1 + REPAIR_SHARE * 1.5) + system * price_in * v1.CACHE_WRITE_FACTOR / 1e6
    return {"models": per_model, "usd": sum(per_model.values()), "answers": answers}


def run_benchmark(client, models, cases, effort, repeats, out_dir: Path, system_text, variant, max_usd,
                  workers=4, fallbacks=True, secrets=None, log=sys.stderr) -> float:
    """Run every (model, case, repeat) without a saved record, stopping new work once `max_usd` is spent."""
    secrets = secrets or []
    meta = {"run_id": out_dir.name, "benchmark": "bench-2", "variant": variant, "models": models, "effort": effort,
            "repeats": repeats, "cases": list(cases), "fallbacks": fallbacks,
            "prompt_sha256": hashlib.sha256(system_text.encode("utf-8")).hexdigest(),
            "started": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")}
    meta_path = out_dir / "run.json"
    if meta_path.exists():
        old = json.loads(meta_path.read_text(encoding="utf-8"))
        if any(old[k] != meta[k] for k in ("variant", "models", "effort", "repeats", "cases", "prompt_sha256")):
            raise v1run.FatalError(f"{out_dir} holds a different run; choose another --run-id")
    else:
        v1run.save_json(meta_path, meta, secrets)
    spent = 0.0
    for path in out_dir.glob("*/*.json"):
        spent += sum(v1.call_cost(c) or 0 for c in json.loads(path.read_text(encoding="utf-8"))["calls"])
    jobs = [(m, c, r) for m in models for c in cases.values() for r in range(1, repeats + 1)
            if not v1run.record_path(out_dir, m, c["id"], r).exists()]
    lock, stop = threading.Lock(), threading.Event()

    def work(job, gate=False):
        nonlocal spent
        model, case, repeat = job
        with lock:
            if stop.is_set() or spent >= max_usd:
                stop.set()
                return
        record = run_answer(client, model, case, repeat, effort, system_text, fallbacks)
        v1run.save_json(v1run.record_path(out_dir, model, case["id"], repeat), record, secrets)
        cost = sum(v1.call_cost(c) or 0 for c in record["calls"])
        with lock:
            spent += cost
        print(f"  {model} {case['id']} #{repeat}: {len(record['calls'])} call(s), ${cost:.4f}, total ${spent:.2f}",
              file=log)
        fatal = [c for c in record["calls"] if c["fatal"] or (gate and c["bad_request"])]
        if fatal:
            stop.set()
            error = fatal[0]["error"]
            raise v1run.FatalError(f"{model} {case['id']}: {error['type']} {error['status_code']}: {error['message']}")

    first = {}
    for job in jobs:  # each model's first request runs alone: it writes the cache and finds config errors
        first.setdefault(job[0], job)
    for job in first.values():
        work(job, gate=True)
    with ThreadPoolExecutor(max_workers=max(1, workers)) as pool:
        list(pool.map(work, [j for j in jobs if j not in first.values()]))
    if stop.is_set() and spent >= max_usd:
        print(f"stopped at the cap: ${spent:.2f} >= ${max_usd:.2f}; rerun with the same --run-id to resume", file=log)
    return spent


def main(argv=None, client_factory=v1run.make_client, out=sys.stdout, err=sys.stderr, environ=os.environ,
         env_file: Path = v1run.ENV_FILE) -> int:
    parser = argparse.ArgumentParser(description="Run BENCH-2.")
    parser.add_argument("--models", default=v1run.DEFAULT_MODELS)
    parser.add_argument("--variant", default="full", choices=sorted(lib.PROMPTS))
    parser.add_argument("--effort", default="medium", choices=v1run.EFFORTS)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--cases", help="comma-separated case IDs (default: all 80)")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--run-id")
    parser.add_argument("--out", type=Path, default=v1run.RESULTS_DIR)
    parser.add_argument("--max-usd", type=float, default=10.0, help="stop starting requests once spent (default 10)")
    parser.add_argument("--no-fallbacks", action="store_true")
    parser.add_argument("--yes", action="store_true")
    args = parser.parse_args(argv)
    models = [m.strip() for m in args.models.split(",") if m.strip()]
    if any(m not in v1.PRICES for m in models) or args.repeats < 1:
        print("unknown model or bad --repeats", file=err)
        return 2
    cases = lib.load_cases()
    if args.cases:
        wanted = [c.strip() for c in args.cases.split(",")]
        if any(c not in cases for c in wanted):
            print("unknown case", file=err)
            return 2
        cases = {c: cases[c] for c in wanted}
    system_text = lib.PROMPTS[args.variant].read_text(encoding="utf-8")
    est = estimate(models, cases, args.repeats, system_text)
    print(f"BENCH-2, prompt {args.variant}, effort {args.effort}, {est['answers']} answers per model "
          f"(calibrated on run3):", file=out)
    for model, usd in est["models"].items():
        print(f"  {model:20} ~${usd:.2f}", file=out)
    print(f"  total ~${est['usd']:.2f}; hard cap ${args.max_usd:.2f}", file=out)
    if not args.yes:
        print("Nothing was sent. Add --yes to spend it.", file=out)
        return 2
    key = v1run.read_api_key(environ, env_file)
    if not key:
        print("No API key: set ANTHROPIC_API_KEY or put it in bench/.env", file=err)
        return 2
    out_dir = args.out / (args.run_id or "v2-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    try:
        spent = run_benchmark(client_factory(key), models, cases, args.effort, args.repeats, out_dir, system_text,
                              args.variant, args.max_usd, args.workers, not args.no_fallbacks, [key], err)
    except v1run.FatalError as error:
        print(f"stopped: {error}\nWhat was saved is in {out_dir}", file=err)
        return 1
    print(f"Saved to {out_dir} (${spent:.2f}). Score it with: python bench/v2/score.py {out_dir}", file=out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
