# ruff: noqa
"""Measurement only (g1-phase4-prereqs.md): one table row per `loop_profile.json` that `exp_loop_profile.py` wrote.

    python summarise_profiles.py NAME=PATH [NAME=PATH ...]

For each: the worst second and the stalls the stand-in itself reports, the loop's compute time per pass (mean, p99, max,
and the share over the 2 ms budget), and where the loop thread's time went, summed over the run's seconds: its CPU, the
time it waited in the kernel's run queue for a CPU (wake-ups included), and the rest of what it was blocked inside a
pass (the GIL, a lock, a write: `passes wall - passes CPU - run-queue wait inside passes`). Also the seconds that fell
below 0.99 of real time, with their run-queue waits.
"""
import json
import sys


def row(name: str, path: str) -> dict:
    j = json.load(open(path))
    t = j["summary"]["total_ms"]["all"]
    cpu = wall = rq = over = 0.0
    blocked = 0.0
    low = []
    for s in j.get("per_second", []):
        loop = s["loop"]
        if loop["runq_delay_ms"] is None:
            continue
        cpu += loop["passes_cpu_ms"]
        wall += loop["passes_wall_ms"]
        rq += loop["runq_delay_ms"]
        over += loop["overshoot_ms"]
        blocked += max(loop["passes_wall_ms"] - loop["passes_cpu_ms"] - max(loop["runq_delay_ms"] - loop["overshoot_ms"], 0), 0)
        if s["ratio"] < 0.99:
            low.append((s["sim_time"], s["ratio"], loop["runq_delay_ms"]))
    n = max(len(j.get("per_second", [])), 1)
    return {
        "run": name, "worst_second": round(j["worst_ratio"], 4), "stalls": j["stalls"],
        "step_ms mean/p99/max": f"{t['mean']}/{t['p99']}/{t['max']}", "over 2 ms": f"{100 * t['over_2ms']:.1f}%",
        "loop CPU ms/s": round(cpu / n), "run-queue wait ms/s": round(rq / n), "other blocking ms/s": round(blocked / n),
        "seconds < 0.99": low,
    }


if __name__ == "__main__":
    rows = [row(*a.split("=", 1)) for a in sys.argv[1:]]
    keys = list(rows[0])
    print("| " + " | ".join(keys) + " |")
    print("|" + "---|" * len(keys))
    for r in rows:
        print("| " + " | ".join(str(r[k]) for k in keys) + " |")
