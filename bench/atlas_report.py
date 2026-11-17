"""
Render docs/plans/humanoid-activities.md from bench/atlas.json (RSI-01, docs/plans/rsi-01.md).

    python bench/atlas_report.py            # write the plan
    python bench/atlas_report.py --check    # exit 1 if the plan is out of date
    python bench/atlas_report.py --snapshot 2026-11-17 --note "What changed"
                                            # add a dated entry to the atlas history, then write the plan

The weekly foresight loop runs --snapshot after it edits the atlas, so the plan's development log shows, per week,
which sub-activities were added or removed, which changed maturity or horizon, and how the directions moved.

The atlas is the only source: add or change sub-activities there, then run this. Priorities:

- sub-activity:  p x nearness(horizon)                      (nearness: now 1, 1-3y 0.7, 3-7y 0.4, 7y+ 0.1)
- direction:     sum of its sub-activities' priorities, each split evenly over the directions the sub-activity names;
                 its language need is the sum of the priorities of its open gaps
- language gap:  sum of the priorities of the sub-activities that need it; "near" if one of them is pilot or
                 deployment with horizon now or 1-3y (eligible for syntax, strategy rule 1 and D5)
"""

from __future__ import annotations
import json
import sys
from pathlib import Path

BENCH = Path(__file__).resolve().parent
ATLAS = BENCH / "atlas.json"
PLAN = BENCH.parent / "docs" / "plans" / "humanoid-activities.md"

DIRECTIONS = ["perception", "reasoning", "prediction", "action", "learning", "cooperation", "safety", "communication"]
NEARNESS = {"now": 1.0, "1-3y": 0.7, "3-7y": 0.4, "7y+": 0.1}


def priority(act: dict) -> float:
    return act["forecast"]["p"] * NEARNESS[act["horizon"]]


def is_near(act: dict) -> bool:
    return act["maturity"] in ("pilot", "deployment") and act["horizon"] in ("now", "1-3y")


def scores(atlas: dict) -> tuple[dict[str, float], dict[str, float], set[str]]:
    direction = {d: 0.0 for d in DIRECTIONS}
    gap = {g["id"]: 0.0 for g in atlas["gaps"]}
    near = set()
    for act in atlas["activities"]:
        dirs = act.get("directions") or []
        for d in dirs:
            direction[d] += priority(act) / len(dirs)
        for g in act["needs"]:
            gap[g] += priority(act)
            if is_near(act):
                near.add(g)
    return direction, gap, near


def snapshot(atlas: dict, date: str, note: str) -> dict:
    """Append a history entry: the state of every sub-activity and the direction priorities on `date`."""
    direction, _, _ = scores(atlas)
    entry = {"date": date, "note": note,
             "activities": {a["id"]: f"{a['maturity']} {a['horizon']}" for a in atlas["activities"]},
             "directions": {d: round(direction[d], 2) for d in DIRECTIONS}}
    atlas.setdefault("history", []).append(entry)
    return entry


def rank(directions: dict) -> list[str]:
    return sorted(DIRECTIONS, key=lambda d: (-directions.get(d, 0.0), DIRECTIONS.index(d)))


def log(atlas: dict) -> list[str]:
    history = atlas.get("history") or []
    if not history:
        return ["No entries yet: the first one is written with `--snapshot`."]
    out = ["Newest first. Ranking: the directions in order of priority, with the change of rank since the entry before.",
           "", "| Date | Note | Added | Removed | Changed | Ranking |", "|---|---|---|---|---|---|"]
    rows = []
    for i, entry in enumerate(history):
        before = history[i - 1] if i else {"activities": {}, "directions": {}}
        now, old = entry["activities"], before["activities"]
        added = sorted(set(now) - set(old))
        removed = sorted(set(old) - set(now))
        changed = [f"{k}: {old[k]} → {now[k]}" for k in sorted(set(now) & set(old)) if now[k] != old[k]]
        new_rank, old_rank = rank(entry["directions"]), rank(before["directions"])
        ranking = []
        for pos, d in enumerate(new_rank, 1):
            if not entry["directions"][d]:
                ranking.append("others 0")
                break
            move = old_rank.index(d) + 1 - pos if i else 0
            ranking.append(f"{pos}. {d} {entry['directions'][d]:.2f}" + (f" ({move:+d})" if move else ""))
        rows.append(f"| {entry['date']} | {entry['note']} | {', '.join(added) or '—'} | {', '.join(removed) or '—'} | "
                    f"{'; '.join(changed) or '—'} | {' · '.join(ranking)} |")
    return out + rows[::-1]


def render(atlas: dict) -> str:
    direction, gap, near = scores(atlas)
    acts = atlas["activities"]
    gaps = {g["id"]: g for g in atlas["gaps"]}
    order = sorted(DIRECTIONS, key=lambda d: (-direction[d], DIRECTIONS.index(d)))
    out = [
        "# Humanoid activities and the direction of RoboLang",
        "",
        "**Generated** from [`bench/atlas.json`](../../bench/atlas.json) by `python bench/atlas_report.py`; do not edit by",
        "hand. Plan: [RSI-01](rsi-01.md). Sub-activities are added to the atlas as the foresight loop finds them; the",
        "priorities below follow from them.",
        "",
        "**Priorities.** A sub-activity's priority is `p × nearness` (`p`: its forecast's probability; nearness: now 1,",
        "1–3y 0.7, 3–7y 0.4, 7y+ 0.1). A direction's priority is the sum of its sub-activities' priorities, each split",
        "evenly over the directions it names. A language gap's priority is the sum over the sub-activities that need it;",
        "it is **near** when one of them is a pilot or deployment within three years, and only near gaps are eligible for",
        "new syntax (strategy rule 1, D5). Out-of-scope gaps are listed but never proposed without a decision record.",
        "",
        "## Directions by priority",
        "",
        "*Priority* comes from the sub-activities (what robots will do). *Language need* is the sum of the priorities of",
        "the direction's open gaps (what RoboLang lacks there); the two differ when sub-activities of one direction need",
        "gaps of another, e.g. tote handling (action) needs clock readings (reasoning).",
        "",
        "| # | Direction | Priority | Language need | Sub-activities | Near gaps |",
        "|---|---|---|---|---|---|",
    ]
    for i, d in enumerate(order, 1):
        n = sum(1 for a in acts if d in (a.get("directions") or []))
        ng = sorted((g for g in near if gaps[g].get("direction") == d), key=lambda g: -gap[g])
        need = sum(gap[g["id"]] for g in gaps.values() if g.get("direction") == d and g["scope"] == "open")
        out.append(f"| {i} | {d} | {direction[d]:.2f} | {need:.2f} | {n} | {', '.join(ng) or '—'} |")
    out += ["", "## Development log", ""] + log(atlas)
    for d in order:
        out += ["", f"## {d.capitalize()}", ""]
        mine = sorted((a for a in acts if d in (a.get("directions") or [])), key=lambda a: (-priority(a), a["id"]))
        if mine:
            out += ["| Sub-activity | Maturity | Horizon | p | Priority | Needs | Sources |", "|---|---|---|---|---|---|---|"]
            for a in mine:
                src = " ".join(f"[{k}]({u})" for k, u in enumerate(a["sources"], 1))
                out.append(f"| `{a['id']}` {a['name']} | {a['maturity']} | {a['horizon']} | {a['forecast']['p']:.2f} | "
                           f"{priority(a):.2f} | {', '.join(a['needs']) or '—'} | {src} |")
        else:
            out.append("No sub-activities yet.")
        mygaps = sorted((g for g in gaps.values() if g.get("direction") == d), key=lambda g: (-gap[g["id"]], g["id"]))
        if mygaps:
            out += ["", "| Language gap | Scope | Priority | Near | Benchmark tasks |", "|---|---|---|---|---|"]
            for g in mygaps:
                out.append(f"| `{g['id']}` {g['name']} | {g['scope']} | {gap[g['id']]:.2f} | "
                           f"{'yes' if g['id'] in near else '—'} | {len(g['tasks'])} |")
    unassigned = [g["id"] for g in gaps.values() if g.get("direction") not in DIRECTIONS]
    if unassigned:
        out += ["", f"Gaps without a direction: {', '.join(unassigned)}."]
    return "\n".join(out) + "\n"


def main(argv: list[str]) -> int:
    atlas = json.loads(ATLAS.read_text(encoding="utf-8"))
    if "--snapshot" in argv:
        date = argv[argv.index("--snapshot") + 1]
        note = argv[argv.index("--note") + 1] if "--note" in argv else ""
        snapshot(atlas, date, note)
        ATLAS.write_text(json.dumps(atlas, indent=2, ensure_ascii=False) + "\n", encoding="utf-8", newline="\n")
    text = render(atlas)
    if "--check" in argv:
        if PLAN.read_text(encoding="utf-8") != text:
            print(f"{PLAN} is out of date: run python bench/atlas_report.py", file=sys.stderr)
            return 1
        return 0
    PLAN.write_text(text, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
