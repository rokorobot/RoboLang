# ruff: noqa
"""Measurement only: print the slow seconds and the summary of a loop_profile.json (exp_loop_profile.py)."""
import json
import sys

for path in sys.argv[1:]:
    j = json.load(open(path + "/loop_profile.json"))
    t = j["summary"]["total_ms"]["all"]
    print(f"{path}: worst {j['worst_ratio']:.4f} stalls {j['stalls']}  step ms mean {t['mean']} p99 {t['p99']} max {t['max']}  "
          f"over 2 ms {t['over_2ms']}  overshoot p99 {j['summary']['sleep_overshoot_ms']['p99']}")
    ratios = sorted(j["all_ratios"])
    print("  lowest seconds:", ratios[:6])
    for s in j["slow_seconds"]:
        th = sorted(s["threads"].items(), key=lambda kv: -kv[1]["cpu_s"])[:4]
        print(f"  t={s['sim_time']} ratio {s['ratio']}  longest pass {s['pass_max_ms']} ms, passes>5ms {s['passes_over_5ms']}, "
              f"top: {[(c, k.split(':')[1] + k.split(' ')[-2][-18:] if False else k[:70]) for c, k in s['top_processes'][:3]]}")
        for k, v in th:
            print(f"     {k:16s} cpu {v['cpu_s']:.2f}s run-queue delay {v['delay_s']:.2f}s")
