"""
RSI-01 activity atlas (bench/atlas.json, docs/plans/rsi-01.md). Gaps: every `inexpressible` benchmark task belongs to
exactly one gap, and every task a gap names exists and is inexpressible. Activities: the fields loop A fills in, with
a forecast that can be marked true or false on its due date. No network.
"""

from __future__ import annotations
import datetime
import json
import re
from pathlib import Path

from bench import lib

ATLAS = json.loads((Path(lib.BENCH) / "atlas.json").read_text(encoding="utf-8"))
GAPS = {g["id"]: g for g in ATLAS["gaps"]}
MATURITY = {"concept", "research", "demo", "pilot", "deployment"}
LEADING = {"research", "roadmap", "funding", "standard", "demand", "forecast"}
HORIZON = {"now", "1-3y", "3-7y", "7y+"}
DIRECTIONS = {"perception", "reasoning", "prediction", "action", "learning", "cooperation", "safety", "communication",
              "autonomy"}


def inexpressible_tasks() -> set[str]:
    """Tasks the current language must decline as `inexpressible`: by their `expect` label, not their group. Set 6
    ("regions", decision 0039) is pre-registered against 1.6, so its programs and infeasible tasks of 1.7 are here
    until L4 relabels them."""
    found = set()
    for tasks_dir, _ in lib.SETS.values():
        for path in Path(tasks_dir).glob("*.json"):
            task = json.loads(path.read_text(encoding="utf-8"))
            if lib.expectation(task) == ("cannot", "inexpressible"):
                found.add(task["id"])
    return found


def test_schema():
    assert ATLAS["atlas_schema"] == "robolang.atlas"
    assert ATLAS["atlas_schema_version"] == "0.1"


def test_gaps():
    assert len(GAPS) == len(ATLAS["gaps"]), "duplicate gap id"
    for gap in ATLAS["gaps"]:
        assert re.fullmatch(r"G-[A-Z]+", gap["id"]), gap["id"]
        assert gap["name"] and gap["scope"] in {"open", "out", "closed"}, gap["id"]
        if gap["scope"] == "closed":  # closed by a released language version (L6): no task, no sub-activity needs it
            assert gap["closed_in"] and gap["scope_note"] and not gap["tasks"], gap["id"]
        assert gap["direction"] in DIRECTIONS, f"{gap['id']}: unknown direction"
        assert gap["scope"] == "open" or gap["scope_note"], f"{gap['id']}: an out-of-scope gap names its decision"


def test_every_inexpressible_task_is_in_exactly_one_gap():
    named = [t for gap in ATLAS["gaps"] for t in gap["tasks"]]
    assert len(named) == len(set(named)), "a task is in two gaps"
    assert set(named) == inexpressible_tasks()


def test_activities():
    ids = [a["id"] for a in ATLAS["activities"]]
    assert len(ids) == len(set(ids)), "duplicate activity id"
    for act in ATLAS["activities"]:
        where = act["id"]
        assert re.fullmatch(r"ACT-[A-Z]{2,5}-\d{3}", where), where
        assert act["name"] and act["domain"] and act["basis"], where
        assert act["directions"] and set(act["directions"]) <= DIRECTIONS, f"{where}: directions"
        assert len(act["directions"]) <= 3 and len(set(act["directions"])) == len(act["directions"]), where
        assert act["maturity"] in MATURITY and act["horizon"] in HORIZON, where
        assert act["requests"] and all(isinstance(r, str) and r for r in act["requests"]), where
        assert all(n in GAPS for n in act["needs"]), f"{where}: unknown gap in needs"
        assert all(GAPS[n]["scope"] != "closed" for n in act["needs"]), f"{where}: needs a closed gap"
        forecast = act["forecast"]
        assert forecast["claim"], where
        datetime.date.fromisoformat(forecast["due"])
        assert 0 < forecast["p"] < 1, where
        assert forecast.get("outcome") in (None, True, False), where
        assert act["sources"] and all(s.startswith("https://") for s in act["sources"]), where
        for signal in act.get("leading", []):
            assert signal["kind"] in LEADING and signal["claim"], f"{where}: leading signal"
            assert signal["url"].startswith("https://"), f"{where}: leading signal url"
        assert act["maturity"] != "concept" or act.get("leading"), f"{where}: a concept rests on leading signals"


def test_every_direction_is_used():
    used = {g["direction"] for g in ATLAS["gaps"]} | {d for a in ATLAS["activities"] for d in a["directions"]}
    assert used == DIRECTIONS, f"directions with no gap or activity: {DIRECTIONS - used}"


def test_every_direction_is_the_main_one_of_two_activities():
    main = [a["directions"][0] for a in ATLAS["activities"]]
    assert all(main.count(d) >= 2 for d in DIRECTIONS), {d: main.count(d) for d in DIRECTIONS}
