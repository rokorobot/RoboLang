"""The benchmark gate (bench/gate.py): summaries, floors and the exit status. No network."""

from __future__ import annotations
import io
import json

from bench import gate


def row(model="m", expect="program", group="requested", **scores):
    base = {"model": model, "expect": expect, "group": group,
            "decision_correct": True, "false_program": False, "goal_correct": True, "plan_succeeds": True}
    return {**base, **scores}


def write(tmp_path, name, rows):
    path = tmp_path / name
    path.mkdir()
    (path / "scores.json").write_text(json.dumps(rows), encoding="utf-8")
    return path


def test_rates_count_only_requested_answers_and_the_right_tasks():
    rows = [row(), row(expect="cannot", goal_correct=None, plan_succeeds=None),
            row(expect="cannot", false_program=True, decision_correct=False, goal_correct=None, plan_succeeds=None),
            row(group="fallback", goal_correct=False)]
    rates = gate.summarize(rows)["m"]
    assert rates["decision_correct"] == (2, 3)
    assert rates["false_program"] == (1, 2)  # over cannot-tasks only
    assert rates["goal_correct"] == (1, 1)  # over feasible tasks; the fallback answer is not counted


def test_a_breached_floor_fails_and_a_clean_run_passes(tmp_path):
    clean = write(tmp_path, "clean", [row() for _ in range(20)])
    bad = write(tmp_path, "bad", [row() for _ in range(9)] + [row(goal_correct=False) for _ in range(2)])
    out = io.StringIO()
    assert gate.main([str(clean)], out=out) == 0 and "Gate: pass" in out.getvalue()
    out = io.StringIO()
    assert gate.main([str(bad), "--summary", str(tmp_path / "s.md")], out=out) == 1
    assert "goal_correct 9/11" in out.getvalue() and (tmp_path / "s.md").read_text().startswith("## RoboLang")


def test_floors_file_has_the_four_measures():
    floors = json.loads(gate.FLOORS_FILE.read_text(encoding="utf-8"))
    assert set(floors) == {"decision_correct", "false_program", "goal_correct", "plan_succeeds"}
