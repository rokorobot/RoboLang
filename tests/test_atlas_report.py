"""
The humanoid activity plan (docs/plans/humanoid-activities.md) is generated from bench/atlas.json by
bench/atlas_report.py and must be up to date; the priority rules match its docstring. No network.
"""

from __future__ import annotations
import json

from bench import atlas_report as ar


def test_plan_is_up_to_date():
    assert ar.PLAN.read_text(encoding="utf-8") == ar.render(json.loads(ar.ATLAS.read_text(encoding="utf-8"))), \
        "run python bench/atlas_report.py"


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
    text = ar.render(atlas)
    assert text.index("| 1 | action |") < text.index("| 2 | safety |")
