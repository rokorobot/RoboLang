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

Set 2 (`--set hard`) adds the decision. A task expects a program, or "cannot" with a category (inexpressible,
infeasible). The four rates above are over the tasks that expect a program; on top of them, per model:

    decision_correct    the first answer's decision is the expected one
    false_program       a program for a task that expects "cannot" (the dangerous error), and how many of those
                        were valid and ended SUCCEEDED after repair, i.e. not caught by the tools
    false_refusal       "cannot" for a task that expects a program
    category_correct    of the correct "cannot" decisions, the category is the expected one

The decision is judged on the first answer, before any tool output; a repair round (only for invalid programs)
may change it, which is reported as false_program_final. An answer without a `decision` (set 1) is a program.
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
    if answer["decision"] == "cannot":  # a decline has no program to check
        return answer, None, kind
    checked = lib.check_answer(task_def, answer)
    codes.extend(checked.codes)
    return answer, checked, kind


def evaluate_record(task_def: dict, record: dict) -> dict:
    """The scores of one saved answer (one record file)."""
    calls = record["calls"]
    codes: list[str] = []
    answer, checked, kind = _final(task_def, calls[0], codes)
    first = answer if kind == "answer" else None
    valid_first = bool(checked and checked.valid)
    final = (answer, checked)
    if not valid_first and len(calls) > 1:
        repaired, repaired_checked, repaired_kind = _final(task_def, calls[1], codes)
        if repaired_kind == "answer":
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
    has_checks = any(k in task_def for k in ("expected_safety", "expected_messages", "expected_message_counts",
                                              "expected_end_ms", "probes"))
    expect, expect_category = lib.expectation(task_def)
    decision = first["decision"] if first else None
    decision_final = final[0]["decision"] if final[0] else None
    category_final = final[0]["category"] if final[0] else None
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
        # the decision (set 2; set 1 answers are programs and its tasks expect programs)
        "kind": task_def.get("group", "-"), "expect": expect, "expect_category": expect_category,
        "decision": decision, "category": first["category"] if first else None,
        "reason": first["reason"] if first else "",
        "decision_final": decision_final,
        "decision_correct": decision == expect,
        "category_correct": ((first["category"] == expect_category) if expect == "cannot" and decision == "cannot"
                             else None),
        "false_program": expect == "cannot" and decision == "program",
        "false_program_final": expect == "cannot" and decision_final == "program",
        "false_program_undetected": bool(expect == "cannot" and decision_final == "program" and judgement
                                         and judgement.plan_succeeds),
        "false_refusal": expect == "program" and decision == "cannot",
        "false_refusal_final": expect == "program" and decision_final == "cannot",
        "category_final": category_final,
    }


def load_rows(results_dir: Path) -> tuple[dict, list[dict]]:
    meta = json.loads((results_dir / "run.json").read_text(encoding="utf-8"))
    tasks = lib.load_set(meta.get("set", "1"))  # runs saved before set 2 have no "set": they are set 1
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


def _programs(rows: list[dict]) -> list[dict]:
    """The rows of tasks that expect a program: the four rates and the tag table are over these."""
    return [r for r in rows if r["expect"] == "program"]


def _rates_table(rows: list[dict]) -> list[str]:
    rows = _programs(rows)
    lines = ["| measure | answers | rate |", "|---|---:|---:|"]
    for name in MEASURES:
        k = sum(r[name] for r in rows)
        lines.append(f"| {name} | {k}/{len(rows)} | {_share(k, len(rows))} |")
    checked = [r for r in rows if r["checks_ok"] is not None]
    k = sum(r["checks_ok"] for r in checked)
    lines.append(f"| checks_ok (tasks with optional checks) | {k}/{len(checked)} | {_share(k, len(checked))} |")
    return lines


def _shares(rows: list[dict]) -> str:
    """The four rates of the program-expecting `rows` as table cells."""
    rows = _programs(rows)
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
    rows = _programs(rows)
    tags = sorted({t for r in rows for t in r["tags"]})
    lines = ["| tag | answers | " + " | ".join(MEASURES) + " |", "|---|---:|" + "---:|" * len(MEASURES)]
    for tag in tags:
        sel = [r for r in rows if tag in r["tags"]]
        lines.append(f"| {tag} | {len(sel)} | {_shares(sel)} |")
    return lines


def _count(rows: list[dict], field: str) -> str:
    return _share(sum(bool(r[field]) for r in rows), len(rows))


def _robustness_row(rows: list[dict]) -> list[str]:
    """Cells: decision_correct, false_program (and after repair, undetected), false_refusal, category_correct."""
    cannot = [r for r in rows if r["expect"] == "cannot"]
    declined = [r for r in cannot if r["decision"] == "cannot"]
    undetected = sum(r["false_program_undetected"] for r in cannot)
    return [_count(rows, "decision_correct"), _count(cannot, "false_program") if cannot else "-",
            f"{sum(r['false_program_final'] for r in cannot)}/{len(cannot)}" if cannot else "-",
            f"{undetected}/{len(cannot)}" if cannot else "-",
            _count(_programs(rows), "false_refusal") if _programs(rows) else "-",
            _share(sum(r["category_correct"] for r in declined), len(declined)) if declined else "-"]


ROBUSTNESS_HEAD = ["decision_correct", "false_program", "false_program after repair",
                   "of which valid and SUCCEEDED", "false_refusal", "category_correct (of declines)"]


def _robustness_table(rows: list[dict]) -> list[str]:
    lines = ["| " + " | ".join(ROBUSTNESS_HEAD) + " |", "|" + "---:|" * len(ROBUSTNESS_HEAD),
             "| " + " | ".join(_robustness_row(rows)) + " |"]
    return lines


def _kinds_table(rows: list[dict]) -> list[str]:
    """One row per task kind (inexpressible, infeasible, decoy, ambiguous, control)."""
    lines = ["| kind | answers | " + " | ".join(ROBUSTNESS_HEAD) + " | goal_correct | plan_succeeds |",
             "|---|---:|" + "---:|" * (len(ROBUSTNESS_HEAD) + 2)]
    for kind in sorted({r["kind"] for r in rows}):
        sel = [r for r in rows if r["kind"] == kind]
        feasible = _programs(sel)
        goal = _count(feasible, "goal_correct") if feasible else "-"
        succeeds = _count(feasible, "plan_succeeds") if feasible else "-"
        lines.append(f"| {kind} | {len(sel)} | " + " | ".join(_robustness_row(sel)) + f" | {goal} | {succeeds} |")
    return lines


def _hard(rows: list[dict]) -> bool:
    return any(r["expect"] == "cannot" for r in rows)


def _group(rows: list[dict]) -> list[str]:
    noans = Counter(r["outcome"] for r in rows if r["outcome"] != "answer")
    lines = _rates_table(rows) + [""]
    if _hard(rows):
        lines += ["Robustness (decision judged on the first answer; the four rates above are over the tasks that "
                  "expect a program):", ""] + _robustness_table(rows) + ["", "By kind of task:", ""] \
                 + _kinds_table(rows) + [""]
    if noans:
        lines += ["No usable first answer: " + ", ".join(f"{k} {v}" for k, v in sorted(noans.items())) + ".", ""]
    lines += ["Tokens and cost of all requests for these answers:", ""] + _tokens_table(rows)
    lines += ["", "Most common diagnostic codes (all rounds):", ""] + _codes_table(rows)
    lines += ["", "By tag (tasks that expect a program):" if _hard(rows) else "By tag:", ""] + _tags_table(rows)
    return lines


def report(meta: dict, rows: list[dict]) -> str:
    models = sorted({r["model"] for r in rows})
    lines = [f"# RoboLang authoring benchmark: {meta['run_id']}", "",
             f"Effort {meta['effort']}, {meta['repeats']} repeat(s), {len(meta['tasks'])} task(s), "
             f"prompt sha256 {meta['prompt_sha256'][:12]}, refusal fallbacks "
             f"{'enabled' if meta['fallbacks'] else 'disabled'} (Claude Haiku 5.5 has none).",
             *([f"Task set {meta.get('set', '1')}, prompt {meta.get('prompt', 'full')}."]
               if (meta.get("set", "1"), meta.get("prompt", "full")) != ("1", "full") else []),
             "Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.", "",
             "## Summary (answers served by the requested model)", "",
             "| model | answers | " + " | ".join(MEASURES) + " | cost |",
             "|---|---:|" + "---:|" * len(MEASURES) + "---:|"]
    for model in models:
        own = [r for r in rows if r["model"] == model and r["group"] == "requested"]
        costs = [r["usd"] for r in own]
        lines.append(f"| {model} | {len(own)} | {_shares(own)} | {_usd(None if None in costs else sum(costs))} |")
    if _hard(rows):
        lines += ["", f"The four rates are over the {len(_programs([r for r in rows if r['model'] == models[0]]))} "
                  "task(s) that expect a program (the others expect a decline). Robustness, per model:", "",
                  "| model | answers | " + " | ".join(ROBUSTNESS_HEAD) + " |",
                  "|---|---:|" + "---:|" * len(ROBUSTNESS_HEAD)]
        for model in models:
            own = [r for r in rows if r["model"] == model and r["group"] == "requested"]
            lines.append(f"| {model} | {len(own)} | " + " | ".join(_robustness_row(own)) + " |")
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
