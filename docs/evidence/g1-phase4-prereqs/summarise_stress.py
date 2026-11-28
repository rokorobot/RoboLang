# ruff: noqa
"""Measurement only (g1-phase4-prereqs.md): a table of move-stress runs, finished or not.

    python summarise_stress.py LABEL=PATH [LABEL=PATH ...]

PATH is a `phase2 --only move-stress --out X.json` result, or the `<name>_trace.json` the check writes after every move (so a
run that was stopped is summarised too). Per run and station: the arrivals that converged within 5 cm and 0.1 rad by the
oracle at the moment `move` returned, the failures by code, the time per move and the oracle's error at the end. The first
move (from the start pose to the bench) is not an arrival; it is listed with the others in `moves`.
"""
import json
import statistics as st
import sys


def q(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p * len(xs)))]


def load(path):
    d = json.load(open(path))
    return d["checks"]["move-stress"]["details"]["moves"] if "checks" in d else d["moves"]


print(f"{'run':28s} {'station':6s} {'ok':>6s} {'failures':18s} | {'time s: mean':>12s} {'median':>7s} {'p90':>6s} {'max':>6s} | {'cm: mean':>8s} {'max':>5s} | {'rad: mean':>9s} {'p90':>6s} {'max':>6s}")
for arg in sys.argv[1:]:
    label, path = arg.split("=", 1)
    moves = [m for m in load(path) if m["station"] in ("shelf", "bench")]
    for station in ("shelf", "bench"):
        ms = [m for m in moves if m["station"] == station and (m["n"] > 0 or station == "shelf")]
        if not ms:
            continue
        t = [m["sim_s"] for m in ms]
        cm = [100 * m["oracle_error_m"] for m in ms]
        rad = [m["oracle_error_rad"] for m in ms]
        codes = {}
        for m in ms:
            if m["failure"]:
                codes[m["failure"]["code"]] = codes.get(m["failure"]["code"], 0) + 1
        ok = sum(m["pass"] for m in ms)
        print(f"{label:28s} {station:6s} {ok:>2d}/{len(ms):<3d} {str(codes or '-'):18s} | {st.mean(t):12.1f} {st.median(t):7.1f} {q(t, .9):6.1f} {max(t):6.1f} | {st.mean(cm):8.1f} {max(cm):5.1f} | {st.mean(rad):9.3f} {q(rad, .9):6.3f} {max(rad):6.3f}")
