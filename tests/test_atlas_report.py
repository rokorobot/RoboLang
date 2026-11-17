"""
The humanoid activity plan (docs/plans/humanoid-activities.md) is generated from bench/atlas.json by
bench/atlas_report.py and must be up to date; the priority rules match its docstring. No network.
"""

from __future__ import annotations
import json

from bench import atlas_report as ar


def test_plan_is_up_to_date():
    plan = ar.PLAN.read_text(encoding="utf-8").replace("\r\n", "\n")  # Windows checkouts may turn LF into CRLF
    assert plan == ar.render(json.loads(ar.ATLAS.read_text(encoding="utf-8"))), "run python bench/atlas_report.py"


def test_priorities():
    def act(i, dirs, needs, maturity, horizon, p):
        return {"id": i, "name": i, "directions": dirs, "needs": needs, "maturity": maturity, "horizon": horizon,
                "forecast": {"p": p}, "sources": ["https://example.org"]}
    atlas = {"gaps": [{"id": "G-A", "name": "a", "scope": "open", "direction": "safety", "tasks": []},
                      {"id": "G-B", "name": "b", "scope": "open", "direction": "action", "tasks": []}],
             "activities": [act("A1", ["safety", "action"], ["G-A"], "pilot", "1-3y", 0.8),
                            act("A2", ["action"], ["G-A", "G-B"], "demo", "now", 0.5)]}
    direction, gap, near = ar.scores(atlas)
    assert round(direction["safety"], 3) == 0.28 and round(direction["action"], 3) == 0.78
    assert round(gap["G-A"], 3) == 1.06 and round(gap["G-B"], 3) == 0.5
    assert near == {"G-A"}
    atlas["activities"].append({**act("A3", ["prediction"], [], "concept", "7y+", 0.3),
                                "leading": [{"kind": "research", "claim": "c", "url": "https://example.org"}]})
    assert ar.future(atlas)["prediction"] == 0.3 and ar.future(atlas)["action"] == 0.0
    assert "| `A3` A3 | prediction | concept | 7y+ | 0.30 |" in ar.render(atlas)
    text = ar.render(atlas)
    assert text.index("| 1 | action |") < text.index("| 2 | safety |")


def test_development_log():
    def act(i, maturity, horizon):
        return {"id": i, "name": i, "directions": ["action"], "needs": [], "maturity": maturity, "horizon": horizon,
                "forecast": {"p": 0.5}, "sources": ["https://example.org"]}
    atlas = {"gaps": [], "activities": [act("A1", "demo", "1-3y")]}
    ar.snapshot(atlas, "2026-11-17", "first")
    atlas["activities"] = [act("A1", "pilot", "1-3y"),
                           {**act("A2", "pilot", "now"), "directions": ["safety"]}]
    ar.snapshot(atlas, "2026-11-24", "second")
    text = ar.render(atlas)
    newer, older = text.index("| 2026-11-24 | second |"), text.index("| 2026-11-17 | first |")
    assert newer < older
    row = text[newer:text.index("\n", newer)]
    assert "| A2 |" in row and "A1: demo 1-3y → pilot 1-3y" in row
    assert "1. safety 0.50 (+6) · 2. action 0.35 (-1) · others 0" in row


def test_suggested_order():
    def act(i, needs, p):
        return {"id": i, "name": i, "directions": ["action"], "needs": needs, "maturity": "pilot", "horizon": "now",
                "forecast": {"p": p}, "sources": ["https://example.org"]}
    gaps = [{"id": g, "name": g, "scope": "open", "direction": "action", "tasks": []} for g in ("G-A", "G-B", "G-C")]
    atlas = {"gaps": gaps, "activities": [act("A1", ["G-A", "G-B"], 0.9), act("A2", ["G-C"], 0.2)]}
    assert [g for g, _ in ar.suggested_order(atlas)] == ["G-A", "G-B", "G-C"]  # A and B together unlock 0.9
    atlas["current_horizon"] = {"name": "H", "gaps": ["G-C"]}
    order = ar.suggested_order(atlas)
    assert order[0] == ("G-C", 0.2) and round(order[-1][1], 3) == 1.1
