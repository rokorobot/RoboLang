# ruff: noqa
"""Build the executor's response tables (backends/unitree_g1/robolang_g1/response.py) from the measurements.

    python docs/evidence/g1-phase3/build_response_table.py [--write]

Two sources, per state (empty hands, the box in the right hand):

* the lab: `exp_move_response.py`'s trials (move_response_{empty,carry}.json): each correction from a robot that stood still,
  its pose change 3 s after the command, the stance's own heading creep taken out (rest trials);
* the servo's own corrections: every correction a `move` stress run sent (move_stress_*_trace.json: velocity commands and the
  oracle's pose beside each look), its pose change up to the next look, the creep taken out with the time that passed.

The table keeps the mean of all of them for each correction whose spread (the sample standard deviation of all of them) is at most
MAX_SPREAD in forward, left and heading: a correction that does not do the same twice is not worth sending. `--write` replaces
the tables in response.py; without it the tables and what was dropped are printed.
"""
import collections, json, math, re, statistics as st, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RESPONSE_PY = HERE.parents[2] / "backends/unitree_g1/robolang_g1/response.py"
MAX_SPREAD = (0.025, 0.03, 0.04)  # m, m, rad
AT = "3.0"
TRACES = {
    False: ["move_stress_before_empty_trace.json", "move_stress_after_v1_empty_trace.json"],
    True: ["move_stress_before_carry_trace.json", "move_stress_after_v1_carry_trace.json"],
}
LAB = {False: "move_response_empty.json", True: "move_response_carry.json"}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def lab_rows(path):
    d = json.load(open(path))
    groups, rests = collections.defaultdict(list), []
    for t in d["trials"]:
        if t["cmd"] is None:
            rests.append(t["rest"])
        else:
            groups[tuple(tuple(b) for b in t["cmd"])].append(t)
    creep = st.mean(r[-1][3] / r[-1][0] for r in rests)
    return {k: [(t["after"][AT][0], t["after"][AT][1], t["after"][AT][2] - creep * float(AT)) for t in ts]
            for k, ts in groups.items()}, creep


def usage_rows(paths, creep, keys):
    rows = collections.defaultdict(list)
    for path in paths:
        tr = json.load(open(path))["trace"]
        i = 0
        while i < len(tr):
            if tr[i]["k"] != "cmd" or tr[i]["shuffle"]:
                i += 1
                continue
            j, key = i, []
            while j < len(tr) and tr[j]["k"] == "cmd" and not tr[j]["shuffle"]:
                key.append(tuple(tr[j]["cmd"]))
                j += 1
            key = tuple(key)
            if j < len(tr) and tr[j]["k"] == "look" and key in keys:
                p0, p1 = tr[i]["oracle"], tr[j]["oracle"]
                waited = (tr[j]["t"] - tr[i]["t"]) / 1000 - sum(b[3] for b in key)
                c, s = math.cos(p0[2]), math.sin(p0[2])
                dx, dy = p1[0] - p0[0], p1[1] - p0[1]
                rows[key].append((dx * c + dy * s, -dx * s + dy * c, wrap(p1[2] - p0[2]) - creep * waited))
            i = j
    return rows


def build(carry):
    lab, creep = lab_rows(HERE / LAB[carry])
    use = usage_rows([HERE / t for t in TRACES[carry] if (HERE / t).exists()], creep, set(lab))
    table, dropped = {(): (0.0, 0.0, 0.0)}, []
    for key in sorted(lab):
        rows = lab[key] + use.get(key, [])
        mean = tuple(st.mean(r[i] for r in rows) for i in range(3))
        spread = tuple(st.stdev([r[i] for r in rows]) for i in range(3))
        row = dict(n=len(rows), n_servo=len(use.get(key, [])), mean=mean, spread=spread)
        if all(spread[i] <= MAX_SPREAD[i] for i in range(3)):
            table[key] = tuple(round(v, 3) for v in mean)
        else:
            dropped.append((key, row))
    return table, dropped, creep


def literal(name, table):
    body = "\n".join(f"    {k!r}: {v!r}," for k, v in sorted(table.items()))
    return f"{name}: dict[Correction, tuple[float, float, float]] = {{\n{body}\n}}\n"


if __name__ == "__main__":
    out = {}
    for carry in (False, True):
        table, dropped, creep = build(carry)
        name = "RESPONSE_CARRY" if carry else "RESPONSE"
        out[name] = literal(name, table)
        print(f"{name}: {len(table) - 1} corrections kept, {len(dropped)} dropped for a spread over {MAX_SPREAD}; creep {creep:+.4f} rad/s")
        for key, r in dropped:
            print(f"   dropped {key}: n={r['n']} ({r['n_servo']} from the servo) spread {tuple(round(v, 3) for v in r['spread'])}")
    if "--write" in sys.argv:
        s = RESPONSE_PY.read_text()
        for name, text in out.items():
            s = re.sub(rf"{name}: dict\[Correction.*?\n\}}\n", lambda m: text, s, flags=re.S)
        RESPONSE_PY.write_text(s)
        print("wrote", RESPONSE_PY)
