"""
Score a results directory written by run.py. Offline and deterministic: no network, no model, no judge.

    python bench/score.py bench/results/<run-id>

Every answer is checked again with the installed `robolang` package (the same checks as `robolang validate`
and `robolang simulate`), so the scores do not depend on anything but the saved replies and the tasks:

    valid_first         the first answer validates, the task exists, and the bindings are accepted
    valid_after_repair  that holds for the first answer or for the answer after one repair round
    goal_correct        the goal, exported under the bindings, equals the task's expected set of Core triples
    plan_succeeds       the simulation under the bindings ends SUCCEEDED
    checks_ok           the task's optional checks pass (safety limits, messages, duration, probe worlds)

goal_correct, plan_succeeds and checks_ok are judged on the final answer, and are false when it is not valid.
Answers served by a fallback model are scored in their own tables, never in the requested model's.
Writes report.md and scores.json next to the answers.
"""

from __future__ import annotations
import argparse
import json
import sys
from collections import Counter
from pathlib import Path

if __package__ in (None, ""):  # started as `python bench/score.py`
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bench import lib  # noqa: E402

MEASURES = ("valid_first", "valid_after_repair", "goal_correct", "plan_succeeds")
NO_ANSWER_CODE = {"refusal": lib.REFUSED_CODE, "error": lib.API_ERROR_CODE, "format": lib.FORMAT_CODE}


def _final(task_def: dict, call: dict, codes: list[str]):
    """(answer, checked, kind) for one call; adds the diagnostic codes seen."""
    answer, _, kind = lib.answer_of(call)
    if kind != "answer":
        codes.append(NO_ANSWER_CODE[kind])
        return None, None, kind
    checked = lib.check_answer(task_def, answer)
    codes.extend(checked.codes)
    return answer, checked, kind


def evaluate_record(task_def: dict, record: dict) -> dict:
    """The scores of one saved answer (one record file)."""
    calls = record["calls"]
    codes: list[str] = []
    answer, checked, kind = _final(task_def, calls[0], codes)
    valid_first = bool(checked and checked.valid)
    final = (answer, checked)
    if not valid_first and len(calls) > 1:
        repaired, repaired_checked, _ = _final(task_def, calls[1], codes)
        if repaired_checked:
            final = (repaired, repaired_checked)
    valid_after = bool(final[1] and final[1].valid)
    judgement = lib.judge(task_def, final[1], final[0]["bindings"]) if valid_after else None
    tokens = dict.fromkeys(lib.TOKEN_FIELDS, 0)
    costs = []
    for call in calls:
        for name, count in lib.call_tokens(call).items():
            tokens[name] += count
        costs.append(lib.call_cost(call))
    checks = judgement.checks if judgement else {}
    has_checks = any(k in task_def for k in ("expected_safety", "expected_messages", "expected_end_ms", "probes"))
    return {
        "task_id": task_def["id"], "model": record["model"], "repeat": record["repeat"], "tags": task_def["tags"],
        "group": "fallback" if any(lib.served_by(c)["fallback"] for c in calls) else "requested",
        "served_by": sorted({lib.served_by(c)["model"] for c in calls if c.get("response")}),
        "outcome": kind, "repaired": len(calls) > 1,
        "valid_first": valid_first, "valid_after_repair": valid_after,
        "goal_correct": bool(judgement and judgement.goal_correct),
        "plan_succeeds": bool(judgement and judgement.plan_succeeds),
        "checks_ok": (bool(judgement) and all(checks.values())) if has_checks else None,
        "state": judgement.state if judgement else None, "failure_code": judgement.failure_code if judgement else None,
        "codes": codes, "tokens": tokens, "usd": None if None in costs else sum(costs),
    }


def load_rows(results_dir: Path) -> tuple[dict, list[dict]]:
    meta = json.loads((results_dir / "run.json").read_text(encoding="utf-8"))
    tasks = lib.load_tasks()
    rows = []
    for path in sorted(results_dir.glob("*/*.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        rows.append(evaluate_record(tasks[record["task_id"]], record))
    rows.sort(key=lambda r: (r["model"], r["task_id"], r["repeat"]))
    return meta, rows


# -------- tables --------

def _share(k: int, n: int) -> str:
    return f"{k}/{n} ({100 * k / n:.1f}%)" if n else "-"


def _usd(value: float | None) -> str:
    return "n/a" if value is None else f"${value:.4f}"


def _rates_table(rows: list[dict]) -> list[str]:
    lines = ["| measure | answers | rate |", "|---|---:|---:|"]
    for name in MEASURES:
        k = sum(r[name] for r in rows)
        lines.append(f"| {name} | {k}/{len(rows)} | {_share(k, len(rows))} |")
    checked = [r for r in rows if r["checks_ok"] is not None]
    k = sum(r["checks_ok"] for r in checked)
    lines.append(f"| checks_ok (tasks with optional checks) | {k}/{len(checked)} | {_share(k, len(checked))} |")
    return lines


def _shares(rows: list[dict]) -> str:
    """The four rates of `rows` as table cells."""
    return " | ".join(_share(sum(r[m] for r in rows), len(rows)) for m in MEASURES)


def _tokens_table(rows: list[dict]) -> list[str]:
    total = {k: sum(r["tokens"][k] for r in rows) for k in lib.TOKEN_FIELDS}
    costs = [r["usd"] for r in rows]
    cost = None if None in costs else sum(costs)
    return ["| input (uncached) | cache write | cache read | output | cost |", "|---:|---:|---:|---:|---:|",
            f"| {total['input_tokens']:,} | {total['cache_creation_input_tokens']:,} "
            f"| {total['cache_read_input_tokens']:,} | {total['output_tokens']:,} | {_usd(cost)} |"]


def _codes_table(rows: list[dict]) -> list[str]:
    counts = Counter(code for r in rows for code in r["codes"])
    if not counts:
        return ["No diagnostics."]
    lines = ["| code | seen |", "|---|---:|"]
    lines += [f"| {code} | {n} |" for code, n in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))[:10]]
    return lines


def _tags_table(rows: list[dict]) -> list[str]:
    tags = sorted({t for r in rows for t in r["tags"]})
    lines = ["| tag | answers | " + " | ".join(MEASURES) + " |", "|---|---:|" + "---:|" * len(MEASURES)]
    for tag in tags:
        sel = [r for r in rows if tag in r["tags"]]
        lines.append(f"| {tag} | {len(sel)} | {_shares(sel)} |")
    return lines


def _group(rows: list[dict]) -> list[str]:
    noans = Counter(r["outcome"] for r in rows if r["outcome"] != "answer")
    lines = _rates_table(rows) + [""]
    if noans:
        lines += ["No usable first answer: " + ", ".join(f"{k} {v}" for k, v in sorted(noans.items())) + ".", ""]
    lines += ["Tokens and cost of all requests for these answers:", ""] + _tokens_table(rows)
    lines += ["", "Most common diagnostic codes (all rounds):", ""] + _codes_table(rows)
    lines += ["", "By tag:", ""] + _tags_table(rows)
    return lines


def report(meta: dict, rows: list[dict]) -> str:
    models = sorted({r["model"] for r in rows})
    lines = [f"# RoboLang authoring benchmark: {meta['run_id']}", "",
             f"Effort {meta['effort']}, {meta['repeats']} repeat(s), {len(meta['tasks'])} task(s), "
             f"prompt sha256 {meta['prompt_sha256'][:12]}, refusal fallbacks "
             f"{'enabled' if meta['fallbacks'] else 'disabled'} (Claude Haiku 5.5 has none).",
             "Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.", "",
             "## Summary (answers served by the requested model)", "",
             "| model | answers | " + " | ".join(MEASURES) + " | cost |",
             "|---|---:|" + "---:|" * len(MEASURES) + "---:|"]
    for model in models:
        own = [r for r in rows if r["model"] == model and r["group"] == "requested"]
        costs = [r["usd"] for r in own]
        lines.append(f"| {model} | {len(own)} | {_shares(own)} | {_usd(None if None in costs else sum(costs))} |")
    for model in models:
        own = [r for r in rows if r["model"] == model and r["group"] == "requested"]
        lines += ["", f"## {model}", "", f"{len(own)} answer(s) served by {model}.", ""] + (_group(own) if own else [])
        fallback = [r for r in rows if r["model"] == model and r["group"] == "fallback"]
        if fallback:
            served = ", ".join(sorted({m for r in fallback for m in r["served_by"]}))
            lines += ["", f"### {model}: answers served by a fallback model ({served})", "",
                      "A refusal fallback answered these. They are not the requested model's answers and are not "
                      "counted above.", ""] + _group(fallback)
    return "\n".join(lines) + "\n"


def score_dir(results_dir: Path) -> str:
    """Write report.md and scores.json in `results_dir`; return the report text."""
    meta, rows = load_rows(results_dir)
    text = report(meta, rows)
    (results_dir / "report.md").write_text(text, encoding="utf-8")
    (results_dir / "scores.json").write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
    return text


def main(argv=None, out=sys.stdout) -> int:
    parser = argparse.ArgumentParser(description="Score a benchmark results directory, offline.")
    parser.add_argument("results_dir", type=Path)
    args = parser.parse_args(argv)
    if not (args.results_dir / "run.json").is_file():
        print(f"{args.results_dir} is not a results directory (no run.json)", file=sys.stderr)
        return 2
    text = score_dir(args.results_dir)
    print(text, file=out)
    print(f"Wrote {args.results_dir / 'report.md'}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
