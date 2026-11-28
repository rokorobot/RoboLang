# ruff: noqa
"""Measurement only (g1-phase4-prereqs.md): the stand-in's real time in every run of a phase 2 or phase 3 result.

    python summarise_runs.py RESULT.json [--short]

One row per stand-in run: `realtime_valid`, the worst second, the stalls, the compute time per step (mean / p99 / max, share over
the 2 ms budget), the camera's rate, and the slow seconds with the loop's CPU and run-queue wait where the stand-in reports them.
"""
import json
import sys

d = json.load(open(sys.argv[1]))
runs = d["runs"]
print(f"go {d.get('go')}, {len(runs)} stand-in runs, valid in {sum(r['realtime_valid'] for r in runs)}, "
      f"elapsed {d.get('elapsed_s')} s, arch {d.get('arch')}")
print("| run | valid | worst second | stalls | step compute ms: mean / p99 / max | over 2 ms | camera Hz | slow seconds (sim time, ratio, loop CPU s, run-queue wait s) |")
print("|---|---|---|---|---|---|---|---|")
for r in runs:
    sc = r["step_compute_ms"]
    cam = (r.get("camera") or {}).get("hz", "")
    print(f"| {r['scenario']} | {r['realtime_valid']} | {r['worst_second_ratio']} | {r['stalls']} | {sc['mean']} / {sc['p99']} / {sc['max']} | "
          f"{100 * sc['over_budget']:.1f}% | {cam} | {r.get('slow_windows') or ''} |")
v = [r["worst_second_ratio"] for r in runs if r["realtime_valid"]]
print(f"\nvalid runs: lowest worst second {min(v)}, mean step compute {sum(r['step_compute_ms']['mean'] for r in runs) / len(runs):.3f} ms, "
      f"p99 mean {sum(r['step_compute_ms']['p99'] for r in runs) / len(runs):.2f} ms")
