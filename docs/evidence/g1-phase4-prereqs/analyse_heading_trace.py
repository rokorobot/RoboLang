# ruff: noqa
"""Measurement only (g1-phase4-prereqs.md): how wrong was the belief heading between marker fixes?

    python analyse_heading_trace.py TRACE.json [TRACE.json ...]

A move-stress trace (`phase2 --only move-stress` writes `<name>_trace.json`) has every `look` with the belief pose and the
oracle's pose beside it, and whether that look was a marker fix. At a fix the belief is the marker's; at a look that was
not a fix (the marker was not in view or not good enough) the belief is what the executor carried on from the last fix:
dead-reckoned from the commands before the IMU change, the IMU's yaw after it. The difference from the oracle's heading
at those looks is the error of the heading between fixes (the executor never sees the oracle; only this analysis does).
"""
import json
import math
import statistics
import sys


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def analyse(path: str) -> dict:
    trace = json.load(open(path))["trace"]
    blind, fixes, last_fix_t = [], 0, None
    for e in trace:
        if e["k"] != "look":
            continue
        if e["fix"]:
            fixes += 1
            last_fix_t = e["t"]
            continue
        blind.append((abs(wrap(e["belief"][2] - e["oracle"][2])), (e["t"] - last_fix_t) / 1000 if last_fix_t else None))
    errs = sorted(b[0] for b in blind)
    n = len(errs)
    pick = (lambda q: round(errs[min(n - 1, int(q * n))], 3)) if n else (lambda q: None)
    return {"trace": path, "looks": fixes + n, "fixes": fixes, "blind_looks": n,
            "heading_error_rad": {"mean": round(statistics.mean(errs), 3) if n else None, "median": pick(0.5),
                                   "p90": pick(0.9), "max": round(errs[-1], 3) if n else None,
                                   "over_0.1": sum(e > 0.1 for e in errs), "over_0.3": sum(e > 0.3 for e in errs)}}


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(json.dumps(analyse(p)))
