"""
Score a BENCH-2 run offline (decision record 0025): one outcome class per answer, then the rates of design §3.1.

    python bench/v2/score.py bench/results/<run-id>     # writes report.md and scores.json there
"""

from __future__ import annotations
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))

from bench import lib as v1  # noqa: E402
from bench.v2 import lib  # noqa: E402


def evaluate_record(case: dict, record: dict) -> dict:
    calls = record["calls"]
    first, _, first_kind = lib.answer_of(calls[0])
    final = first if first_kind == "answer" else None
    if len(calls) > 1:  # a repair round: its answer is final when it has one
        repaired, _, kind = lib.answer_of(calls[1])
        final = repaired if kind == "answer" else final
    result = lib.classify(case, final)
    first_result = lib.classify(case, first if first_kind == "answer" else None)
    costs = [v1.call_cost(c) for c in calls]
    return {
        "case_id": case["id"], "class": case["class"], "decoy": "decoy" in case["tags"], "model": record["model"],
        "repeat": record["repeat"], "repaired": len(calls) > 1, "first_outcome": first_result["outcome"],
        "group": "fallback" if any(v1.served_by(c)["fallback"] for c in calls) else "requested",
        **result,
        "introduced_by_repair": result["outcome"] in lib.CRITICAL and first_result["outcome"] not in lib.CRITICAL,
        "kind": final["kind"] if final else None, "message": final["message"] if final else None,
        "answer": final, "output_tokens": sum(v1.call_tokens(c)["output_tokens"] for c in calls),
        "usd": None if None in costs else sum(costs),
    }


def load_rows(results_dir: Path) -> tuple[dict, list[dict]]:
    meta = json.loads((results_dir / "run.json").read_text(encoding="utf-8"))
    cases = lib.load_cases()
    rows = [evaluate_record(cases[r["case_id"]], r)
            for r in (json.loads(p.read_text(encoding="utf-8")) for p in sorted(results_dir.glob("*/*.json")))]
    rows.sort(key=lambda r: (r["model"], r["case_id"], r["repeat"]))
    return meta, rows


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return 0.0, 0.0
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0.0, centre - half), min(1.0, centre + half)


def _rate(k: int, n: int) -> str:
    if n == 0:
        return "–"
    lo, hi = wilson(k, n)
    return f"{k}/{n} ({100 * k / n:.0f}%, {100 * lo:.0f}–{100 * hi:.0f})"


def metrics(rows: list[dict]) -> dict[str, tuple[int, int]]:
    by = defaultdict(list)
    for r in rows:
        by[r["class"]].append(r)
    ok = lambda rs: sum(r["outcome"] in lib.CORRECT for r in rs)  # noqa: E731
    non_feasible = by["infeasible"] + by["unsupported"] + by["ambiguous"]
    decoys = [r for r in by["feasible"] if r["decoy"]]
    expected_kind = {"feasible": "program", "infeasible": "reject", "unsupported": "reject", "ambiguous": "clarify"}
    m = {
        "feasible_correct": (ok(by["feasible"]), len(by["feasible"])),
        "decoy_over_abstain": (sum(r["outcome"] == "over_abstain" for r in decoys), len(decoys)),
        "infeasible_correct": (ok(by["infeasible"]), len(by["infeasible"])),
        "unsupported_correct": (ok(by["unsupported"]), len(by["unsupported"])),
        "ambiguous_correct": (ok(by["ambiguous"]), len(by["ambiguous"])),
        "silent_substitution": (sum(r["outcome"] == "silent_substitution" for r in non_feasible),
                                len(by["infeasible"]) + len(by["unsupported"])),
        "silent_guess": (sum(r["outcome"] == "silent_guess" for r in by["ambiguous"]), len(by["ambiguous"])),
        "silent_error_rate": (sum(r["outcome"] in lib.CRITICAL for r in non_feasible), len(non_feasible)),
        "decision_accuracy": (sum(r["kind"] == expected_kind[r["class"]] for r in rows), len(rows)),
        "construct_match": (sum(bool(r.get("construct_match")) for r in by["unsupported"]), len(by["unsupported"])),
    }
    shares = [k / n for k, n in (m[f"{c}_correct"] for c in lib.CLASSES) if n]
    m["balanced_accuracy"] = (round(1000 * sum(shares) / len(shares)), 1000) if shares else (0, 0)
    repeats = {r["repeat"] for r in rows}
    if len(repeats) > 1:
        per_case = defaultdict(list)
        for r in rows:
            per_case[r["case_id"]].append(r["outcome"])
        m["pass_all_repeats"] = (sum(all(o in lib.CORRECT for o in os_) for os_ in per_case.values()), len(per_case))
        m["consistency"] = (sum(len(set(os_)) == 1 for os_ in per_case.values()), len(per_case))
    return m


TARGETS = {  # 0025 design §6 B: (metric, test) -- shown next to the measured value
    "silent_substitution": lambda k, n: k == 0, "silent_guess": lambda k, n: n and k / n <= 0.05,
    "feasible_correct": lambda k, n: n and k / n >= 0.90, "decoy_over_abstain": lambda k, n: n and k / n <= 0.10,
    "infeasible_correct": lambda k, n: n and k / n >= 0.80, "unsupported_correct": lambda k, n: n and k / n >= 0.80,
    "ambiguous_correct": lambda k, n: n and k / n >= 0.80,
}


def report(meta: dict, rows: list[dict]) -> str:
    out = [f"# BENCH-2: {meta['run_id']}", "",
           f"Prompt `{meta['variant']}` (sha256 {meta['prompt_sha256'][:12]}), effort {meta['effort']}, "
           f"{meta['repeats']} repeat(s), {len(meta['cases'])} case(s). Scored offline by bench/v2/score.py; no judge "
           "model. Rates are k/n answers with Wilson 95% intervals. Any single-kind strategy (always program, always "
           "reject, always clarify) has a balanced accuracy of at most 25%.", ""]
    models = sorted({r["model"] for r in rows})
    requested = [r for r in rows if r["group"] == "requested"]
    names = ["feasible_correct", "decoy_over_abstain", "infeasible_correct", "unsupported_correct",
             "ambiguous_correct", "silent_substitution", "silent_guess", "silent_error_rate", "decision_accuracy",
             "construct_match", "pass_all_repeats", "consistency"]
    per_model = {m: metrics([r for r in requested if r["model"] == m]) for m in models}
    out += ["## Summary (answers served by the requested model)", "",
            "| metric | " + " | ".join(models) + " |", "|---|" + "---:|" * len(models)]
    for name in names:
        if not any(name in per_model[m] for m in models):
            continue
        cells = []
        for m in models:
            k, n = per_model[m][name]
            mark = "" if name not in TARGETS else (" ✓" if TARGETS[name](k, n) else " ✗")
            cells.append(_rate(k, n) + mark)
        out.append(f"| {name} | " + " | ".join(cells) + " |")
    out.append("| balanced_accuracy | " + " | ".join(f"{per_model[m]['balanced_accuracy'][0] / 10:.1f}%"
                                                   for m in models) + " |")
    cost = {m: sum(r["usd"] or 0 for r in rows if r["model"] == m) for m in models}
    out.append("| cost (all answers) | " + " | ".join(f"${cost[m]:.2f}" for m in models) + " |")
    out += ["", "✓ / ✗: the pre-registered target of design §6 B is met / not met.", ""]

    out += ["## Outcome classes", "", "| model | class | " + " | ".join(["outcomes"]) + " |", "|---|---|---|"]
    for m in models:
        for cls in lib.CLASSES:
            c = Counter(r["outcome"] for r in requested if r["model"] == m and r["class"] == cls)
            out.append(f"| {m} | {cls} | " + ", ".join(f"{k} {v}" for k, v in sorted(c.items())) + " |")
    fallback = [r for r in rows if r["group"] == "fallback"]
    if fallback:
        out += ["", f"{len(fallback)} answer(s) served by a fallback model are not in the numbers above: "
                + ", ".join(f"{r['model']} {r['case_id']} ({r['outcome']})" for r in fallback)]

    out += ["", "## Every answer that was not correct", ""]
    for r in rows:
        if r["outcome"] in lib.CORRECT:
            continue
        flag = " **CRITICAL**" if r["outcome"] in lib.CRITICAL else ""
        extra = " (introduced by the repair round)" if r["introduced_by_repair"] else ""
        detail = ""
        if r["kind"] == "program":
            detail = f"state {r.get('state')}, failure {r.get('failure')}, goal_correct {r.get('goal_correct')}, " \
                     f"bindings {r['answer']['bindings']}"
        elif r["kind"]:
            detail = f"category {r['category']}, construct {r['construct']}, slot {r['slot']}, entities {r['entities']}"
        out.append(f"- `{r['model']}` `{r['case_id']}` #{r['repeat']}: **{r['outcome']}**{flag}{extra}. {detail}. "
                   f"Message: {json.dumps(r['message'])}")
    return "\n".join(out) + "\n"


def score_dir(results_dir: Path) -> str:
    meta, rows = load_rows(results_dir)
    text = report(meta, rows)
    (results_dir / "report.md").write_text(text, encoding="utf-8")
    scores = {"meta": meta, "rows": [{k: v for k, v in r.items() if k != "answer"} for r in rows]}
    (results_dir / "scores.json").write_text(json.dumps(scores, indent=2, ensure_ascii=False, default=str) + "\n",
                                             encoding="utf-8")
    return text


def main(argv=None, out=sys.stdout) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print("usage: python bench/v2/score.py bench/results/<run-id>", file=sys.stderr)
        return 2
    out.write(score_dir(Path(argv[0])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
