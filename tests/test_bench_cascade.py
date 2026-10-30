"""bench/cascade.py: a Haiku-first, Sonnet-on-escalation cascade scored offline from saved answers. No network."""

from __future__ import annotations
import io
import json

from bench import cascade


def row(model, task, **kw):
    base = {"model": model, "task_id": task, "repeat": 1, "group": "requested", "expect": "program",
            "decision": "program", "decision_final": "program", "valid_after_repair": True,
            "decision_correct": True, "false_program": False, "goal_correct": True, "plan_succeeds": True,
            "usd": 0.001 if "haiku" in model else 0.01}
    return {**base, **kw}


ROWS = [
    row(cascade.FIRST, "ok"), row(cascade.SECOND, "ok", goal_correct=False),          # kept: Haiku's answer
    row(cascade.FIRST, "bad", valid_after_repair=False, goal_correct=False, plan_succeeds=False),
    row(cascade.SECOND, "bad"),                                                        # escalated: invalid
    row(cascade.FIRST, "ask", decision="clarify", decision_final="clarify", decision_correct=False),
    row(cascade.SECOND, "ask"),                                                        # escalated: a question
    row(cascade.FIRST, "only_haiku"),                                                  # no Sonnet answer: skipped
]


def test_escalation_uses_only_what_a_deployed_system_sees():
    assert not cascade.escalated(ROWS[0])
    assert cascade.escalated(ROWS[2]) and cascade.escalated(ROWS[4])
    assert cascade.escalated(row(cascade.FIRST, "x", decision_final="cannot", valid_after_repair=False))


def test_the_cascade_takes_the_second_answer_only_when_escalated_and_pays_for_both():
    chain = {r["task_id"]: r for r in cascade.cascade_rows(ROWS)}
    assert set(chain) == {"ok", "bad", "ask"}
    assert chain["ok"]["goal_correct"] and not chain["ok"]["escalated"] and chain["ok"]["usd"] == 0.001
    assert chain["bad"]["goal_correct"] and chain["bad"]["escalated"] and abs(chain["bad"]["usd"] - 0.011) < 1e-12
    assert chain["ask"]["decision_correct"] and chain["ask"]["model"].startswith("cascade:")


def test_the_report_and_the_cli(tmp_path):
    text = cascade.report(ROWS)
    assert "cascade:claude-haiku-5-5>claude-sonnet-5-5 | 3/3" in text and "Escalated to claude-sonnet-5-5: 2/3" in text
    (tmp_path / "scores.json").write_text(json.dumps(ROWS), encoding="utf-8")
    out = io.StringIO()
    assert cascade.main([str(tmp_path), "--summary", str(tmp_path / "c.md")], out=out) == 0
    assert (tmp_path / "cascade.json").is_file() and (tmp_path / "c.md").read_text(encoding="utf-8") == out.getvalue()
    assert cascade.main([str(tmp_path / "missing")]) == 2
