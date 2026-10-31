"""H3 acceptance verdict (decision 0029 section 3) over run directories written by run_task.sh.

Usage: acceptance.py RESULTS_DIR OUT_JSON. Every directory under RESULTS_DIR holding a scene.json is one run.
Prints a Markdown report and exits 0 only when the pass bar holds.
"""
import json
import sys
from pathlib import Path

SUCCESS_RATE, AGREEMENT = 0.9, 0.98


def load(path: Path, name: str):
    try:
        return json.loads((path / name).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def main():
    runs = []
    for scene_file in sorted(Path(sys.argv[1]).rglob("scene.json")):
        directory = scene_file.parent
        scene, record, oracle = load(directory, "scene.json"), load(directory, "executor.json"), load(
            directory, "oracle.json")
        runs.append({
            "run": str(directory.relative_to(sys.argv[1])), "scene": scene["scene"], "expect": scene["expect"],
            "expect_code": scene.get("expect_code"),
            "state": record["state"] if record else "NO_RECORD",
            "code": ((record or {}).get("failure") or {}).get("code"),
            "message": ((record or {}).get("failure") or {}).get("message"),
            "checks": (oracle or {}).get("goal", []),
            # A record whose oracle did not finish cannot be checked: an oracle error, which also blocks acceptance.
            "oracle_error": bool(record and record["goal"] and any(g["result"] != "not_evaluated"
                                                                   for g in record["goal"]) and oracle is None),
            "false_success": bool(oracle and oracle["false_success"]),
        })
    family = [r for r in runs if r["expect"] == "SUCCEEDED"]
    cases = [r for r in runs if r["expect"] == "FAILED"]
    checks = [c for r in runs for c in r["checks"]]
    succeeded = sum(r["state"] == "SUCCEEDED" for r in family)
    agreed = sum(c["agree"] for c in checks)
    false_successes = sum(r["false_success"] for r in runs)
    oracle_errors = sum(r["oracle_error"] for r in runs)
    cases_ok = all(r["state"] == "FAILED" and r["code"] == r["expect_code"] for r in cases)
    rate = succeeded / len(family) if family else 0.0
    agreement = agreed / len(checks) if checks else 0.0
    passed = bool(family) and rate >= SUCCESS_RATE and agreement >= AGREEMENT and false_successes == 0 and cases_ok \
        and len(cases) == 4 and oracle_errors == 0
    summary = {"passed": passed, "family_runs": len(family), "succeeded": succeeded, "success_rate": round(rate, 4),
               "goal_checks": len(checks), "agreed": agreed, "agreement": round(agreement, 4),
               "false_successes": false_successes, "oracle_errors": oracle_errors, "failure_cases": len(cases),
               "failure_cases_ok": cases_ok,
               "runs": runs}
    Path(sys.argv[2]).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"## H3 acceptance: {'PASSED' if passed else 'NOT PASSED'}\n")
    print("| Measure | Result | Bar |\n|---|---:|---:|")
    print(f"| Family runs SUCCEEDED | {succeeded}/{len(family)} ({rate:.1%}) | ≥ {SUCCESS_RATE:.0%} |")
    print(f"| Goal checks agreeing with the oracle | {agreed}/{len(checks)} ({agreement:.1%}) | ≥ {AGREEMENT:.0%} |")
    print(f"| False successes | {false_successes} | 0 |")
    print(f"| Runs the oracle could not check | {oracle_errors} | 0 |")
    failed_as_expected = sum(r["state"] == "FAILED" and r["code"] == r["expect_code"] for r in cases)
    print(f"| Failure cases ending FAILED with their expected code | {failed_as_expected}/{len(cases)} | 4/4 |\n")
    print("| Run | Expected | State | Code | Goal checks (observed/oracle) |\n|---|---|---|---|---|")
    for r in runs:
        goal = ", ".join(f"{c['observed']}/{str(c['oracle']).lower()}" for c in r["checks"]) or (
            "oracle error" if r["oracle_error"] else "-")
        expected = r["expect"] + (f" {r['expect_code']}" if r["expect_code"] else "")
        print(f"| {r['run']} | {expected} | {r['state']} | {r['code'] or ''} | {goal} |")
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
