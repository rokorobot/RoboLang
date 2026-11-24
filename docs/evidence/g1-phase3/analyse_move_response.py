# ruff: noqa
"""Summarise exp_move_response.py's response.json files: per burst (or pair), the mean and spread of the oracle's pose change
(forward, left, heading) AT seconds after the command ended (default 3: about when the executor's look follows), the
heading with the stance's own creep (the rest trials' rate times AT) taken out, and what the stance does with no command.

    python docs/evidence/g1-phase3/analyse_move_response.py response_empty.json [response_carry.json ...] [--at 3.0] [--python]

`--python` prints the table as the executor's RESPONSE literal."""
import json, statistics as st, sys
from collections import defaultdict

args = sys.argv[1:]
AT = "3.0"
if "--at" in args:
    i = args.index("--at")
    AT = f"{float(args[i + 1]):.1f}"
    del args[i : i + 2]
paths = [a for a in args if not a.startswith("--")]
as_python = "--python" in args
for path in paths:
    d = json.load(open(path))
    groups, rests = defaultdict(list), []
    for t in d["trials"]:
        if t["cmd"] is None:
            rests.append(t["rest"])
        else:
            groups[tuple(tuple(b) for b in t["cmd"])].append(t)
    creep = st.mean(r[-1][3] / r[-1][0] for r in rests) if rests else 0.0
    print(f"# {path}  carry={d['carry']}  trials={len(d['trials'])}  response at {AT} s after the command, creep {creep:+.4f} rad/s taken out")
    for key, ts in sorted(groups.items()):
        rows = [[*t["after"][AT][:2], t["after"][AT][2] - creep * float(AT)] for t in ts]
        n = len(rows)
        mean = [st.mean(r[i] for r in rows) for i in range(3)]
        sd = [st.pstdev([r[i] for r in rows]) if n > 1 else 0.0 for i in range(3)]
        settle = [float(AT)]
        if as_python:
            print(f"    {key!r}: ({mean[0]:.3f}, {mean[1]:.3f}, {mean[2]:.3f}),  # n={n}, sd {sd[0]:.3f} {sd[1]:.3f} {sd[2]:.3f}")
        else:
            print(f"{key!s:70s} n={n} fwd {mean[0]:+.3f}±{sd[0]:.3f} left {mean[1]:+.3f}±{sd[1]:.3f} yaw {mean[2]:+.3f}±{sd[2]:.3f}  settle {st.mean(settle) if settle else float('nan'):.1f} s")
    for k, series in enumerate(rests):
        t, x, y, yaw = series[-1]
        print(f"rest {k}: after {t:.0f} s: moved fwd {x:+.3f} left {y:+.3f}, yaw {yaw:+.4f} rad = {yaw / t:+.4f} rad/s")
