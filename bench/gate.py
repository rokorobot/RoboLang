"""
The benchmark gate (docs/STRATEGY.md, decision rule 2): summarize scored runs and fail on a regression.

    python bench/gate.py RESULTS_DIR [RESULTS_DIR ...] [--summary FILE]

Reads each directory's scores.json (written by bench/score.py), prints one markdown table, and exits 1
when any requested model is below a floor in bench/gate.json. Answers served by a fallback model are
not counted. No network.
"""

from __future__ import annotations
import argparse
import json
import sys
from pathlib import Path

FLOORS_FILE = Path(__file__).resolve().parent / "gate.json"


def _rate(rows, key, where=lambda r: True):
    hits = [r[key] for r in rows if where(r) and r.get(key) is not None]
    return (sum(1 for v in hits if v), len(hits))


def summarize(rows: list[dict]) -> dict[str, dict[str, tuple[int, int]]]:
    """Per model: (k, n) for decision accuracy, false programs, goal correctness, plan success."""
    out = {}
    for model in sorted({r["model"] for r in rows if r.get("group") == "requested"}):
        mine = [r for r in rows if r["model"] == model and r.get("group") == "requested"]
        feasible = lambda r: r.get("expect", "program") == "program"  # noqa: E731  ("program", "cannot", "clarify")
        out[model] = {
            "decision_correct": _rate(mine, "decision_correct"),
            "false_program": _rate(mine, "false_program", lambda r: r.get("expect") == "cannot"),
            "silent_guess": _rate(mine, "silent_guess", lambda r: r.get("expect") == "clarify"),
            "goal_correct": _rate(mine, "goal_correct", feasible),
            "plan_succeeds": _rate(mine, "plan_succeeds", feasible),
            # informational, no floor: a valid, SUCCEEDED program that is wrong on another path (decision 0027, G1)
            "half_program": _rate(mine, "half_program", lambda r: r.get("has_paths")),
        }
    return out


def _problem(label: str, key: str, rate: tuple[int, int], floor: dict) -> str | None:
    """One breached floor as a sentence, or None."""
    k, n = rate
    if not n:
        return None
    value = k / n
    low, high = floor.get("min"), floor.get("max")
    if low is not None and value < low:
        return f"{label}: {key} {k}/{n} = {value:.1%} is below {low:.0%}"
    if high is not None and value > high:
        return f"{label}: {key} {k}/{n} = {value:.1%} is above {high:.0%}"
    return None


def check(summary: dict, floors: dict) -> list[str]:
    """Breached safety floors per model and task-quality floors pooled across requested models."""
    problems = []
    for model, rates in summary.items():
        for key, floor in floors["per_model"].items():
            if problem := _problem(model, key, rates[key], floor):
                problems.append(problem)
    for key, floor in floors["overall"].items():
        pooled = tuple(sum(rates[key][i] for rates in summary.values()) for i in (0, 1))
        if problem := _problem("all requested models", key, pooled, floor):
            problems.append(problem)
    return problems


def table(name: str, summary: dict) -> str:
    keys = ("decision_correct", "false_program", "goal_correct", "plan_succeeds")
    if any(rates["half_program"][1] for rates in summary.values()):  # only when the run has branching tasks
        keys += ("half_program",)
    lines = [f"**{name}**", "", "| model | " + " | ".join(keys) + " |", "|---" * (len(keys) + 1) + "|"]
    for model, rates in summary.items():
        cells = [f"{k}/{n}" if n else "—" for k, n in (rates[key] for key in keys)]
        lines.append(f"| {model} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def main(argv=None, out=sys.stdout) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    parser.add_argument("results", nargs="+", type=Path)
    parser.add_argument("--summary", type=Path, help="also write the markdown summary to this file")
    args = parser.parse_args(argv)
    floors = json.loads(FLOORS_FILE.read_text(encoding="utf-8"))
    parts, problems = [], []
    for results in args.results:
        summary = summarize(json.loads((results / "scores.json").read_text(encoding="utf-8")))
        parts.append(table(results.name, summary))
        problems += [f"{results.name}: {p}" for p in check(summary, floors)]
    verdict = "**Gate: pass**" if not problems else "**Gate: FAIL**\n\n" + "\n".join(f"- {p}" for p in problems)
    text = "## RoboLang benchmark\n\n" + "\n\n".join(parts) + "\n\n" + verdict + "\n"
    print(text, file=out)
    if args.summary:
        args.summary.write_text(text, encoding="utf-8")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
