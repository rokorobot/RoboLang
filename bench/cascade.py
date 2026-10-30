"""
A model cascade scored offline from saved answers: a cheap model answers first, and a stronger one answers only
when the first one's answer is escalated.

    python bench/cascade.py RESULTS_DIR [--first claude-haiku-5-5] [--second claude-sonnet-5-5] [--summary FILE]

Needs a scored run (scores.json, from bench/score.py) in which both models answered the same tasks and repeats.
A live cascade asks the second model the same question independently, so its answer is the second model's saved
answer. Escalation uses only what a deployed system knows, never the labels:

    escalate    the first model's final answer is not a valid program (after its repair round),
                or it declined or asked a question

The cascade's row for a task is the first model's row, or the second model's when escalated. Its cost and tokens
are the first model's plus the second's when escalated. Prints a gate-style table, the escalation rate and the
cost per answer, and writes cascade.json next to scores.json. Informational: no floor applies. No network.
"""

from __future__ import annotations
import argparse
import json
import sys
from pathlib import Path

if __package__ in (None, ""):  # started as `python bench/cascade.py`
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bench import gate  # noqa: E402

FIRST, SECOND = "claude-haiku-5-5", "claude-sonnet-5-5"


def escalated(row: dict) -> bool:
    """True when a deployed cascade would hand the task to the second model."""
    return row.get("decision_final", "program") != "program" or not row["valid_after_repair"]


def cascade_rows(rows: list[dict], first: str = FIRST, second: str = SECOND) -> list[dict]:
    """One row per (task, repeat) both models answered: the first model's, or the second's when escalated."""
    def mine(model):
        return {(r["task_id"], r["repeat"]): r for r in rows if r["model"] == model and r.get("group") == "requested"}

    a, b = mine(first), mine(second)
    out = []
    for key in sorted(a.keys() & b.keys()):
        up = escalated(a[key])
        row = dict(b[key] if up else a[key])
        costs = [a[key].get("usd")] + ([b[key].get("usd")] if up else [])
        row.update(model=f"cascade:{first}>{second}", group="requested", escalated=up,
                   usd=None if None in costs else sum(costs))
        out.append(row)
    return out


def report(rows: list[dict], first: str = FIRST, second: str = SECOND, name: str = "cascade") -> str:
    """Markdown: the two models and the cascade side by side, then escalations and cost per answer."""
    chain = cascade_rows(rows, first, second)
    keys = {(r["task_id"], r["repeat"]) for r in chain}
    shown = [r for r in rows if r["model"] in (first, second) and (r["task_id"], r["repeat"]) in keys
             and r.get("group") == "requested"] + chain
    lines = [gate.table(name, gate.summarize(shown)), ""]
    if chain:
        up = sum(r["escalated"] for r in chain)
        lines.append(f"Escalated to {second}: {up}/{len(chain)} ({100 * up / len(chain):.1f}%).")
        lines.append("")
        lines += ["| model | cost | per 1,000 answers |", "|---|---:|---:|"]
        for model in (first, second, chain[0]["model"]):
            costs = [r.get("usd") for r in shown if r["model"] == model]
            if costs and None not in costs:
                lines.append(f"| {model} | ${sum(costs):.4f} | ${1000 * sum(costs) / len(costs):.2f} |")
    else:
        lines.append(f"No task was answered by both {first} and {second}.")
    return "\n".join(lines) + "\n"


def main(argv=None, out=sys.stdout) -> int:
    parser = argparse.ArgumentParser(description="Score a two-model cascade from a scored benchmark run, offline.")
    parser.add_argument("results_dir", type=Path)
    parser.add_argument("--first", default=FIRST)
    parser.add_argument("--second", default=SECOND)
    parser.add_argument("--summary", type=Path, help="also write the markdown to this file")
    args = parser.parse_args(argv)
    scores = args.results_dir / "scores.json"
    if not scores.is_file():
        print(f"{args.results_dir} has no scores.json: run bench/score.py first", file=sys.stderr)
        return 2
    rows = json.loads(scores.read_text(encoding="utf-8"))
    text = report(rows, args.first, args.second, args.results_dir.name)
    (args.results_dir / "cascade.json").write_text(
        json.dumps(cascade_rows(rows, args.first, args.second), indent=2) + "\n", encoding="utf-8")
    if args.summary:
        args.summary.write_text(text, encoding="utf-8")
    print(text, file=out, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
