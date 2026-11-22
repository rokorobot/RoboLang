# ruff: noqa
"""Table of one or more move-stress results (`python -m robolang_g1.phase2 --only move-stress --out X.json`): per station the
arrivals that converged within 5 cm and 0.1 rad by the oracle, the time per move, and the oracle's error at the end.

    python docs/evidence/g1-phase3/summarise_move_stress.py LABEL=X.json [LABEL=Y.json ...]
"""
import json, statistics as st, sys


def q(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p * len(xs)))]


print(f"{'run':24s} {'station':6s} {'ok':>5s} | {'time s: mean':>12s} {'median':>7s} {'p90':>6s} {'max':>6s} | {'cm: mean':>8s} {'max':>5s} | {'rad: mean':>9s} {'p90':>6s} {'max':>6s}")
for arg in sys.argv[1:]:
    label, path = arg.split("=", 1)
    d = json.load(open(path))["checks"]["move-stress"]["details"]
    for station in ("shelf", "bench"):
        ms = [m for m in d["moves"] if m["station"] == station and m["n"] > 0 or (m["station"] == station and station == "shelf")]
        if station == "bench":
            ms = [m for m in d["moves"] if m["station"] == "bench" and m["n"] > 0]
        t = [m["sim_s"] for m in ms]
        cm = [100 * m["oracle_error_m"] for m in ms]
        rad = [m["oracle_error_rad"] for m in ms]
        ok = sum(m["pass"] for m in ms)
        print(f"{label:24s} {station:6s} {ok:>2d}/{len(ms):<2d} | {st.mean(t):12.1f} {st.median(t):7.1f} {q(t, .9):6.1f} {max(t):6.1f} | {st.mean(cm):8.1f} {max(cm):5.1f} | {st.mean(rad):9.3f} {q(rad, .9):6.3f} {max(rad):6.3f}")
