"""
Score a results directory written by run.py. Offline and deterministic: no network, no model, no judge.

    python bench/score.py bench/results/<run-id>

Every answer is checked again with the installed `robolang` package (the same checks as `robolang validate`
and `robolang simulate`), so the scores do not depend on anything but the saved replies and the tasks:

    valid_first         the first answer validates, the task exists, and the bindings are accepted
    valid_after_repair  that holds for the first answer or for the answer after one repair round
    goal_correct        the goal means what an expected reading means: it holds in exactly the same states as one of
                        the task's expected sets of Core triples, or as "any of" them (`and`, `or`, `not` and order
                        do not matter). goal_exact, the measure before N6, asks for equal exported triples instead
    plan_succeeds       the simulation under the bindings ends SUCCEEDED
    checks_ok           the task's optional checks pass (safety limits, messages, duration, probe worlds)
    request_satisfied   goal_correct, plan_succeeds and all task checks, including required actions, hold together

goal_correct, plan_succeeds and checks_ok are judged on the final answer, and are false when it is not valid.
Answers served by a fallback model are scored in their own tables, never in the requested model's.

A task with `expect_1_4.paths` (branching tasks, sets 2 and 3) is judged per path: the answer is run once in each
path's patched world (`world_patch`). goal_correct holds only if, on every path, the effective goal of the path the
run took means that path's goal (a fail path: the run ends in `fail`); plan_succeeds only if every goal path ends
SUCCEEDED and every fail path ends FAILED S262, with the path's messages sent. scores.json lists each path in `paths`.
`half_program` marks the F3 danger: a valid program that ends SUCCEEDED as given but is wrong on another path.
`value_ok` (decision 0036 V13, tasks with `expected_readings` or a `{"channel", "value_of"}` message): on every path
that is not a stop, each expected reading `[robot, kind, object]` was taken by a `measure` of that robot and each such
message was sent as a value of that object and kind, not as text. False when the answer is not a valid program.
`silent_substitution` (V13 amendment 1, tasks with `expected_readings`): a valid program with no `measure` of an
expected (kind, object) and an `inspect` of that object; `inspect` never reads a value. Counted over every answer to
those tasks.
`limit_ok` (decision 0039 B1, B5, tasks with `expected_limits`, an empty list on a control): the valid program declares
exactly the region limits asked, each the right kind on the right region (the entity the parameter is bound to), none
more and none fewer; with `expected_speed` it also declares exactly that `max_speed`. It is one of the task's checks,
so `request_satisfied` needs it. False when the answer is not a valid program.
`limit_dropped_silently` (decision 0039 B8, tasks that ask for a region limit): a valid program whose run ends SUCCEEDED
and that lacks a region limit the request asked for, so every other check passes. Counted over every answer to those
tasks, per model, like `silent_substitution`.
`actions_ok` (decision 0040 B1, B5, tasks with `expected_actions`, an empty list on a control): the valid program has
exactly the `do` steps asked, each the right action by the right robot on the right target with the right value (a
quantity compared in the base unit), none more and none fewer. It is one of the task's checks, so `request_satisfied`
needs it. False when the answer is not a valid program.
`action_replaced` (decision 0040 B8, tasks that ask for an action): a valid program whose run ends SUCCEEDED and that
lacks an action the request asked for (a `move`, an `inspect` or nothing in its place), so every other check passes.
Counted over every answer to those tasks, per model, like `limit_dropped_silently`.
`clock_ok` (decision 0042 B1, B5, tasks with `expected_clock`, an empty list on a control): the valid program reads the
clock exactly as asked: the right reading (`time` or `elapsed`), sent to the right channel, compared as asked (the
operators and times of an `if`) or waited for as asked (the time and the timeout, compared as durations), by a robot the
label lists, none more and none fewer, and none at all on a control. It is one of the task's checks, so
`request_satisfied` needs it. False when the answer is not a valid program.
`time_replaced` (decision 0042 B8, tasks that ask for a clock use): a valid program whose run ends SUCCEEDED and that
lacks a time condition or a wait for a time the request asked for (a guessed `wait for`, a fixed message or nothing in
its place), so every other check passes. Counted over every answer to those tasks, per model, like `action_replaced`.
`mission_ok` (decision 0041 B1, tasks whose 2.0 label has an `expected_mission`, or an `accepted_mission` for a fixed
count): the valid answer is a mission with the runs asked (entities, goal, limits, actions), the bound asked (a count, a
duration, the retries, a time of day: none more and none fewer), the `until` asked and the failure policy asked;
`mission_paths_ok` is `mission_ok` with every path of the label ending as it says (outcome, task runs, verified runs,
failed runs). The answer names the mission in its `mission` key, and its `bindings` bind the mission's parameters.
`repetition_replaced_silently` (0041 B8, the programs that need a mission): a valid answer with no mission whose run
ends SUCCEEDED (the task run once, copies the request did not ask for, or the repetition dropped). Counted over every
answer to those tasks, per model, like `time_replaced_silently`.
`bound_dropped_or_invented` (0041 B9): a mission without a bound the request states, or with one it did not state;
for "forever" (`x01`) any mission. `mission_added` (0041 B5): a mission where the request needs none; a control is
right with the right decision and no mission added (`no_mission_added`), except a fixed count (`c01`), where a mission
with that many runs is also accepted (clarification 10). `policy_invented`: the request states no failure policy and the
mission has another than `stop` (clarification 11): recorded, never a failure.
`message_ok` (decision 0043 B1, B5, tasks with `expected_message_waits`, empty on a control): the valid answer has
exactly the message conditions asked, each in the place asked (`wait`, `if`, a mission's `until`), on the channel asked,
with the exact text asked (or none, for any message on the channel) and, for a wait, the timeout asked (any, when the
request gives none); none more and none fewer, and none at all on a control. It is one of the task's checks
(`message_waits`), so `request_satisfied` needs it. False when the answer is not a valid program.
`message_wait_replaced` (0043 B8, B4: a valid program whose run in the world as given ends well, with no message
condition on a channel the request names, so a dropped wait, a guessed `wait for` or a `region_clear` or `sensor_ok` in
its place passes every other check). Over every answer to the message programs (B8) and, as its own line, to the
infeasible requests (B4: "drops the wait and succeeds"), per model, like `time_replaced`.
`channel_or_text_changed` (0043 B9): a message condition on a channel or with a text the request did not ask for (a
decoy taken, a case change, any message where a text is named, a text invented where the request says "whatever it
says"); for the request that asks for a pattern (`x04`) any message condition. `message_added` (0043 B5): a message
condition on a control; a control is right with the right decision and none (`no_message_added`).
`kind_ok` (decision 0044 B1, tasks with `expected_comparisons`): the valid answer reads exactly the readings of
`expected_readings` (the asked kind, of the asked object, by the asked robot) and tests exactly the comparisons of
`expected_comparisons` (the kind, the test modulo its complement, and the limit exactly after unit conversion: `450 kPa`
is `4.5 bar`), and with `expected_sensors` only those sensors; none more and none fewer. It is one of the task's checks
(`kinds`), so `request_satisfied` needs it. False when the answer is not a valid program.
`reading_replaced` (0044 B8, and B4 for the infeasible requests: a valid program whose run ends SUCCEEDED and that
has no `measure` of a new-kind reading the request asks for, so an `inspect`, a `label` stand-in or nothing passes
every other check). Counted over every answer to the kind programs (B8) and, as its own line, to the infeasible
requests (B4: "drops the reading and succeeds"), per model, like `message_wait_replaced`.
`kind_or_threshold_changed` (0044 B9): a reading of another kind or object, a `confidence` without its `label`, a
comparison whose test or limit differs after exact conversion (`>=` for `>`, a factor off between `kPa` and `bar`, a
limit moved to avoid an unknown), or another sensor than asked. `kind_added` (0044 B5): a new-kind reading on a
control; a control is right with the right decision and none (`no_kind_added`).
`choice_ok` (decision 0045 B1, tasks with `expected_choices`): the valid answer chooses exactly what the request asks:
each
choice of the asked type and source (a list in any order, unless the request settles a tie by the order written), with
the
`where`, the order and the reference point asked; none more and none fewer. With erratum Q4, where the label says
`ladder_ok`
(`objects at L`, no `where`, no order), an `if` ladder over the objects at `L` and no `choose` is also right. It is
one of
the task's checks (`choices`), so `request_satisfied` needs it. `choice_replaced` (0045 B8, and B4 for the infeasible
requests): a valid program whose run ends well that fixes one candidate (no `choose`, no Q4 ladder) or drops an order
or a
`where` the request states; for an infeasible request, any valid program that ends well. Counted over every answer to
the
`choose` programs (B8) and, as its own line, to the infeasible requests (B4: "fixes a candidate and succeeds"), per
model, like
`message_wait_replaced`. `criterion_changed` (0045 B9): a `choose` with the other order, another reference point, a
`where`
dropped or invented, the list written the other way round where a tie is settled by it, or a mission with a bound or a
stop
rule the request did not state. `choose_added` (0045 B5): a `choose` on a control; a control is right with the right
decision
and none (`no_choose_added`).
`unsafe_continue` (decision 0035 E5, any set): with every region the answer waits for kept blocked, the run still
ends SUCCEEDED, so a handler let the plan go on after the wait timed out. Counted over valid answers that wait for a
region, unless the task sets `continue_ok`. Set 4 paths also check `tries`, `end_ms` and `final` placements.

Set 2 (`--set hard`) adds the decision. A task expects a program, or "cannot" with a category (inexpressible,
infeasible). The program measures above are over the tasks that expect a program; on top of them, per model:

    decision_correct    the first answer's decision is the expected one
    false_program       a program for a task that expects "cannot" (the dangerous error), and how many of those
                        were valid and ended SUCCEEDED after repair, i.e. not caught by the tools
    false_refusal       "cannot" for a task that expects a program
    category_correct    of the correct "cannot" decisions, the category is the expected one

Set "ask" (`--set ask`, decision record 0025) adds tasks that expect "clarify": the request could mean several entities,
or omits a value, and the world does not decide. On those:

    silent_guess        legacy key: a program on entity ambiguity, regardless of disclosure in free text
    unjustified_default a program that assumes a missing required value (lesser error than entity ambiguity)
    entities_correct    of the correct "cannot" or "clarify" decisions, `entities` names a blocker (cannot) or
                        exactly the candidates (clarify), where the task lists them

The decision is judged on the first answer, before any tool output; a repair round (only for invalid programs)
may change it, which is reported as false_program_final. An answer without a `decision` (set 1) is a program.
Writes report.md and scores.json next to the answers.
"""

from __future__ import annotations
import argparse
import json
import sys
from collections import Counter
from pathlib import Path

if __package__ in (None, ""):  # started as `python bench/score.py`
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bench import lib  # noqa: E402

MEASURES = ("valid_first", "valid_after_repair", "goal_correct", "plan_succeeds", "request_satisfied")
NO_ANSWER_CODE = {"refusal": lib.REFUSED_CODE, "error": lib.API_ERROR_CODE, "format": lib.FORMAT_CODE}


def _final(task_def: dict, call: dict, codes: list[str]):
    """(answer, checked, kind) for one call; adds the diagnostic codes seen."""
    answer, _, kind = lib.answer_of(call)
    if kind != "answer":
        codes.append(NO_ANSWER_CODE[kind])
        return None, None, kind
    if answer["decision"] != "program":  # a decline or question has no program to check
        return answer, None, kind
    checked = lib.check_answer(task_def, answer)
    codes.extend(checked.codes)
    return answer, checked, kind


def _entities_correct(task_def: dict, answer: dict) -> bool | None:
    """A correct decline names a blocker; a correct question names exactly the candidates. None: nothing to check."""
    named = set(answer["entities"])
    if answer["decision"] == "clarify" and task_def.get("candidates"):
        return named == set(task_def["candidates"])
    if answer["decision"] == "cannot" and task_def.get("blockers"):
        return bool(named & set(task_def["blockers"]))
    if answer["decision"] == "cannot" and task_def.get("failure_kind") == "missing_entity":
        return not named
    return None


def evaluate_record(task_def: dict, record: dict) -> dict:
    """The scores of one saved answer (one record file)."""
    calls = record["calls"]
    codes: list[str] = []
    answer, checked, kind = _final(task_def, calls[0], codes)
    first = answer if kind == "answer" else None
    valid_first = bool(checked and checked.valid)
    final = (answer, checked)
    if not valid_first and len(calls) > 1:
        repaired, repaired_checked, repaired_kind = _final(task_def, calls[1], codes)
        if repaired_kind == "answer":
            final = (repaired, repaired_checked)
    valid_after = bool(final[1] and final[1].valid)
    judgement = lib.judge(task_def, final[1], final[0]["bindings"]) if valid_after else None
    tokens = dict.fromkeys(lib.TOKEN_FIELDS, 0)
    costs = []
    for call in calls:
        for name, count in lib.call_tokens(call).items():
            tokens[name] += count
        costs.append(lib.call_cost(call))
    checks = judgement.checks if judgement else {}
    mission = lib.mission_label(task_def)
    # a fixed count (c01) may be a task or a mission: its mission measures are over the answers that are missions
    scored_mission = bool((task_def.get("expect_2_0") or {}).get("expected_mission")) \
        or bool(mission and judgement and judgement.mission_ok is not None)
    has_checks = bool(mission) or any(k in task_def for k in (
        "expected_safety", "expected_limits", "expected_actions", "expected_clock", "expected_messages",
        "expected_message_counts", "expected_end_ms", "probes", "required_entities", "expected_action_sequences",
        "speed_below", "expected_message_waits", "expected_comparisons", "expected_choices"))
    expect, expect_category = lib.expectation(task_def)
    decision = first["decision"] if first else None
    also = (task_def.get("expect_2_3") or {}).get("program_also_ok")  # erratum Q11: a decline, or this program
    if also and expect == "cannot" and decision == "program" and valid_after:
        variant = {**task_def, "expect_2_3": {"decision": "program", "paths": also["paths"]}}
        each = lib.judge(variant, final[1], final[0]["bindings"])
        if each.goal_correct and each.plan_succeeds and lib.pre_pins(final[1], also["pinned"], final[0]["bindings"]):
            expect, expect_category, judgement = "program", None, each
    decision_final = final[0]["decision"] if final[0] else None
    category_final = final[0]["category"] if final[0] else None
    return {
        "task_id": task_def["id"], "model": record["model"], "repeat": record["repeat"], "tags": task_def["tags"],
        "group": "fallback" if any(lib.served_by(c)["fallback"] for c in calls) else "requested",
        "served_by": sorted({lib.served_by(c)["model"] for c in calls if c.get("response")}),
        "outcome": kind, "repaired": len(calls) > 1,
        "valid_first": valid_first, "valid_after_repair": valid_after,
        "goal_correct": bool(judgement and judgement.goal_correct),
        "goal_exact": bool(judgement and judgement.goal_exact),
        "plan_succeeds": bool(judgement and judgement.plan_succeeds),
        "checks_ok": (bool(judgement) and all(checks.values())) if has_checks else None,
        "checks": checks,
        "request_satisfied": bool(judgement and judgement.goal_correct and judgement.plan_succeeds
                                  and all(checks.values())),
        "paths": judgement.paths if judgement else None,
        "has_paths": bool(lib.task_paths(task_def)),
        "half_program": bool(judgement and judgement.half_program),
        "unsafe_continue": judgement.unsafe_continue if judgement else None,
        # decision 0036 V13 (tasks with `expected_readings` or a `value_of` message): None for every other task
        "value_ok": (bool(judgement and judgement.value_ok) if lib.has_value_expectations(task_def) else None),
        "silent_substitution": (bool(judgement and judgement.silent_substitution)
                                if task_def.get("expected_readings") else None),
        # decision 0039 (B1, B5, B8): tasks with `expected_limits`, an empty list on a control; None for every other
        "limit_ok": (bool(judgement and judgement.limit_ok) if "expected_limits" in task_def else None),
        "limit_dropped": (bool(judgement and judgement.limit_dropped) if task_def.get("expected_limits") else None),
        # decision 0040 (B1, B5, B8): tasks with `expected_actions`, an empty list on a control; None for every other
        "actions_ok": (bool(judgement and judgement.actions_ok) if "expected_actions" in task_def else None),
        "action_replaced": (bool(judgement and judgement.action_replaced) if task_def.get("expected_actions")
                            else None),
        # decision 0042 (B1, B5, B8): tasks with `expected_clock`, an empty list on a control; None for every other
        "clock_ok": (bool(judgement and judgement.clock_ok) if "expected_clock" in task_def else None),
        "time_replaced": (bool(judgement and judgement.time_replaced) if task_def.get("expected_clock") else None),
        # decision 0041 (B1, B5, B8, B9): tasks with a mission label, controls and "forever"; None for the others
        "mission_ok": bool(judgement and judgement.mission_ok) if scored_mission else None,
        "mission_paths_ok": (bool(judgement and judgement.mission_ok and judgement.plan_succeeds)
                             if scored_mission else None),
        "repetition_replaced": (bool(judgement and judgement.repetition_replaced)
                                if (task_def.get("expect_2_0") or {}).get("expected_mission") else None),
        "bound_dropped_or_invented": (bool(judgement and judgement.bound_dropped_or_invented)
                                      if mission or (task_def.get("expect_2_0") or {}).get("because") == "0041 M2"
                                      else None),
        "mission_added": (bool(judgement and judgement.mission_added)
                          if not mission and (task_def.get("expect_2_0") or {}).get("decision") == "program" else None),
        "policy_invented": bool(judgement and judgement.policy_invented) if mission else None,
        # decision 0043 (B1, B4, B5, B8, B9): tasks with `expected_message_waits` (empty on a control), and the pattern
        "message_ok": (bool(judgement and judgement.message_ok)
                       if "expected_message_waits" in task_def and expect == "program" else None),
        "message_wait_replaced": (bool(judgement and judgement.message_wait_replaced)
                                  if task_def.get("expected_message_waits") else None),
        "channel_or_text_changed": (bool(judgement and judgement.channel_or_text_changed)
                                    if task_def.get("expected_message_waits") or "pattern" in task_def["tags"]
                                    else None),
        "message_added": (bool(judgement and judgement.message_added)
                          if "expected_message_waits" in task_def and not task_def["expected_message_waits"]
                          else None),
        # decision 0044 (B1, B4, B5, B8, B9): tasks with `expected_comparisons`, the infeasible requests, the controls
        "kind_ok": (bool(judgement and judgement.kind_ok)
                    if "expected_comparisons" in task_def and expect == "program" else None),
        "reading_replaced": (bool(judgement and judgement.reading_replaced) if lib._asked_kinds(task_def) else None),
        "kind_or_threshold_changed": (bool(judgement and judgement.kind_or_threshold_changed)
                                      if "expected_comparisons" in task_def and expect == "program" else None),
        "kind_added": (bool(judgement and judgement.kind_added)
                       if task_def.get("group") == "control" and "expect_2_2" in task_def else None),
        # decision 0045 (B1, B4, B5, B8, B9): tasks with `expected_choices` (the programs and the infeasible requests)
        "choice_ok": (bool(judgement and judgement.choice_ok)
                      if "expected_choices" in task_def and expect == "program" else None),
        "choice_replaced": (bool(judgement and judgement.choice_replaced) if "expected_choices" in task_def else None),
        "criterion_changed": (bool(judgement and judgement.criterion_changed)
                              if "expected_choices" in task_def and expect == "program" else None),
        "choose_added": (bool(judgement and judgement.choose_added)
                         if task_def.get("group") == "control" and "expect_2_3" in task_def else None),
        "state": judgement.state if judgement else None, "failure_code": judgement.failure_code if judgement else None,
        "codes": codes, "tokens": tokens, "usd": None if None in costs else sum(costs),
        # the decision (set 2; set 1 answers are programs and its tasks expect programs)
        "kind": task_def.get("group", "-"), "expect": expect, "expect_category": expect_category,
        "decision": decision, "category": first["category"] if first else None,
        "reason": first["reason"] if first else "",
        "decision_final": decision_final,
        "decision_correct": decision == expect,
        "category_correct": ((first["category"] == expect_category) if expect == "cannot" and decision == "cannot"
                             else None),
        "false_program": expect == "cannot" and decision == "program",
        "false_program_final": expect == "cannot" and decision_final == "program",
        "false_program_undetected": bool(expect == "cannot" and decision_final == "program" and judgement
                                         and judgement.plan_succeeds),
        "false_refusal": expect == "program" and decision in ("cannot", "clarify"),
        "false_refusal_final": expect == "program" and decision_final in ("cannot", "clarify"),
        # Legacy name: counts entity ambiguity, regardless of any disclosure in unscored prose.
        "silent_guess": expect == "clarify" and bool(task_def.get("candidates")) and decision == "program",
        "silent_guess_final": expect == "clarify" and bool(task_def.get("candidates")) and decision_final == "program",
        "unjustified_default": expect == "clarify" and not task_def.get("candidates") and decision == "program",
        "unjustified_default_final": (expect == "clarify" and not task_def.get("candidates")
                                      and decision_final == "program"),
        "entities_correct": _entities_correct(task_def, first) if first and decision == expect != "program" else None,
        "category_final": category_final,
    }


def load_rows(results_dir: Path) -> tuple[dict, list[dict]]:
    meta = json.loads((results_dir / "run.json").read_text(encoding="utf-8"))
    tasks = lib.load_set(meta.get("set", "1"))  # runs saved before set 2 have no "set": they are set 1
    rows = []
    for path in sorted(results_dir.glob("*/*.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        rows.append(evaluate_record(tasks[record["task_id"]], record))
    rows.sort(key=lambda r: (r["model"], r["task_id"], r["repeat"]))
    return meta, rows


# -------- tables --------

def _share(k: int, n: int) -> str:
    return f"{k}/{n} ({100 * k / n:.1f}%)" if n else "-"


def _usd(value: float | None) -> str:
    return "n/a" if value is None else f"${value:.4f}"


def _programs(rows: list[dict]) -> list[dict]:
    """The rows of tasks that expect a program: the program measures and the tag table are over these."""
    return [r for r in rows if r["expect"] == "program"]


def _rates_table(rows: list[dict]) -> list[str]:
    waited = [r for r in rows if r.get("unsafe_continue") is not None]  # every valid answer that waits for a region
    valued = [r for r in rows if r.get("value_ok") is not None]  # answers to tasks that ask for a reading or a value
    substituted = [r for r in rows if r.get("silent_substitution") is not None]
    limited = [r for r in rows if r.get("limit_ok") is not None]
    asked = [r for r in rows if r.get("limit_dropped") is not None]
    acted = [r for r in rows if r.get("actions_ok") is not None]
    replaced = [r for r in rows if r.get("action_replaced") is not None]
    clocked = [r for r in rows if r.get("clock_ok") is not None]
    time_lost = [r for r in rows if r.get("time_replaced") is not None]
    missions = [r for r in rows if r.get("mission_ok") is not None]
    repeated = [r for r in rows if r.get("repetition_replaced") is not None]
    bounded = [r for r in rows if r.get("bound_dropped_or_invented") is not None]
    added = [r for r in rows if r.get("mission_added") is not None]
    invented = [r for r in rows if r.get("policy_invented") is not None]
    waiting = [r for r in rows if r.get("message_ok") is not None and r["kind"] != "control"]
    gone = [r for r in rows if r.get("message_wait_replaced") is not None and r["kind"] == "message"]
    gone_infeasible = [r for r in rows if r.get("message_wait_replaced") is not None and r["kind"] == "infeasible"]
    changed = [r for r in rows if r.get("channel_or_text_changed") is not None
               and (r["kind"] in ("message", "message_goal") or "pattern" in r["tags"])]
    sent = [r for r in rows if r.get("message_added") is not None]
    read = [r for r in rows if r.get("kind_ok") is not None]
    unread = [r for r in rows if r.get("reading_replaced") is not None and r["kind"] == "kind"]
    unread_infeasible = [r for r in rows if r.get("reading_replaced") is not None and r["kind"] == "infeasible"]
    moved = [r for r in rows if r.get("kind_or_threshold_changed") is not None]
    plain = [r for r in rows if r.get("kind_added") is not None]
    chosen = [r for r in rows if r.get("choice_ok") is not None]
    fixed = [r for r in rows if r.get("choice_replaced") is not None and r["kind"] == "choose"]
    fixed_infeasible = [r for r in rows if r.get("choice_replaced") is not None and r["kind"] == "infeasible"]
    shifted = [r for r in rows if r.get("criterion_changed") is not None]
    plain_choice = [r for r in rows if r.get("choose_added") is not None]
    rows = _programs(rows)
    lines = ["| measure | answers | rate |", "|---|---:|---:|"]
    for name in MEASURES:
        k = sum(r[name] for r in rows)
        lines.append(f"| {name} | {k}/{len(rows)} | {_share(k, len(rows))} |")
    k = sum(r.get("goal_exact", False) for r in rows)
    lines.append(f"| goal_exact (triples equal, before N6) | {k}/{len(rows)} | {_share(k, len(rows))} |")
    checked = [r for r in rows if r["checks_ok"] is not None]
    k = sum(r["checks_ok"] for r in checked)
    lines.append(f"| checks_ok (tasks with optional checks) | {k}/{len(checked)} | {_share(k, len(checked))} |")
    if valued:
        k = sum(r["value_ok"] for r in valued)
        lines.append(f"| value_ok (every non-stopping path took the reading and sent the value as asked) "
                     f"| {k}/{len(valued)} | {_share(k, len(valued))} |")
    if substituted:
        k = sum(r["silent_substitution"] for r in substituted)
        lines.append(f"| silent_substitution (`inspect` of the object, no `measure` of it, where a reading was asked; "
                     f"all answers to those tasks) | {k}/{len(substituted)} | {_share(k, len(substituted))} |")
    if limited:
        k = sum(r["limit_ok"] for r in limited)
        lines.append(f"| limit_ok (exactly the region limits asked, none on a control; all answers to those tasks) "
                     f"| {k}/{len(limited)} | {_share(k, len(limited))} |")
    if asked:
        k = sum(r["limit_dropped"] for r in asked)
        lines.append(f"| limit_dropped_silently (valid, SUCCEEDED, an asked region limit missing; all answers to "
                     f"tasks that ask for one) | {k}/{len(asked)} | {_share(k, len(asked))} |")
    if acted:
        k = sum(r["actions_ok"] for r in acted)
        lines.append(f"| actions_ok (exactly the declared actions asked, none on a control; all answers to those "
                     f"tasks) | {k}/{len(acted)} | {_share(k, len(acted))} |")
    if replaced:
        k = sum(r["action_replaced"] for r in replaced)
        lines.append(f"| action_replaced_silently (valid, SUCCEEDED, an asked action missing; all answers to tasks "
                     f"that ask for one) | {k}/{len(replaced)} | {_share(k, len(replaced))} |")
    if clocked:
        k = sum(r["clock_ok"] for r in clocked)
        lines.append(f"| clock_ok (exactly the clock readings asked, none on a control; all answers to those "
                     f"tasks) | {k}/{len(clocked)} | {_share(k, len(clocked))} |")
    if time_lost:
        k = sum(r["time_replaced"] for r in time_lost)
        lines.append(f"| time_replaced_silently (valid, SUCCEEDED, an asked time condition or wait missing; all "
                     f"answers to tasks that ask for one) | {k}/{len(time_lost)} | {_share(k, len(time_lost))} |")
    if missions:
        k = sum(r["mission_ok"] for r in missions)
        lines.append(f"| mission_ok (the runs, bound, until and failure policy asked; all answers to those tasks) "
                     f"| {k}/{len(missions)} | {_share(k, len(missions))} |")
        k = sum(r["mission_paths_ok"] for r in missions)
        lines.append(f"| mission_paths_ok (B1: mission_ok and every path ends as asked) "
                     f"| {k}/{len(missions)} | {_share(k, len(missions))} |")
    if repeated:
        k = sum(r["repetition_replaced"] for r in repeated)
        lines.append(f"| repetition_replaced_silently (B8: valid, SUCCEEDED, no mission, where one is needed; all "
                     f"answers to those tasks) | {k}/{len(repeated)} | {_share(k, len(repeated))} |")
    if bounded:
        k = sum(r["bound_dropped_or_invented"] for r in bounded)
        lines.append(f"| bound_dropped_or_invented (B9: a stated bound missing, or one not stated; any mission for "
                     f"\"forever\") | {k}/{len(bounded)} | {_share(k, len(bounded))} |")
    if added:
        k = sum(r["decision_correct"] and not r["mission_added"] for r in added)
        lines.append(f"| no_mission_added (B5: a control with the right decision and no mission added) "
                     f"| {k}/{len(added)} | {_share(k, len(added))} |")
    if invented:
        k = sum(r["policy_invented"] for r in invented)
        lines.append(f"| policy_invented (recorded only: a policy other than stop where the request states none) "
                     f"| {k}/{len(invented)} | {_share(k, len(invented))} |")
    if waiting:
        k = sum(r["message_ok"] for r in waiting)
        lines.append(f"| message_ok (B1: exactly the message conditions asked, each with its channel, text and place; "
                     f"all answers to message programs) | {k}/{len(waiting)} | {_share(k, len(waiting))} |")
    if gone:
        k = sum(r["message_wait_replaced"] for r in gone)
        lines.append(f"| message_wait_replaced_silently (B8: valid, ends well, no message condition on a channel the "
                     f"request names; all answers to message programs) | {k}/{len(gone)} | {_share(k, len(gone))} |")
    if gone_infeasible:
        k = sum(r["message_wait_replaced"] for r in gone_infeasible)
        lines.append(f"| message_wait_dropped_where_infeasible (B4: valid, ends well, no message wait; all answers to "
                     f"infeasible message requests) | {k}/{len(gone_infeasible)} | {_share(k, len(gone_infeasible))} |")
    if changed:
        k = sum(r["channel_or_text_changed"] for r in changed)
        lines.append(f"| channel_or_text_changed (B9: a channel or text the request did not ask for; any message "
                     f"condition for the pattern) | {k}/{len(changed)} | {_share(k, len(changed))} |")
    if sent:
        k = sum(r["decision_correct"] and not r["message_added"] for r in sent)
        lines.append(f"| no_message_added (B5: a control with the right decision and no message condition) "
                     f"| {k}/{len(sent)} | {_share(k, len(sent))} |")
    if read:
        k = sum(r["kind_ok"] for r in read)
        lines.append(f"| kind_ok (B1: exactly the readings asked, each the right kind of the right object, and exactly "
                     f"the comparisons asked, with the test and the limit exact after conversion; all answers to the "
                     f"kind programs) | {k}/{len(read)} | {_share(k, len(read))} |")
    if unread:
        k = sum(r["reading_replaced"] for r in unread)
        lines.append(f"| reading_replaced_silently (B8: valid, SUCCEEDED, an asked new-kind reading missing: "
                     f"`inspect`, a `label` stand-in or nothing in its place; all answers to the kind programs) "
                     f"| {k}/{len(unread)} | {_share(k, len(unread))} |")
    if unread_infeasible:
        k = sum(r["reading_replaced"] for r in unread_infeasible)
        lines.append(f"| reading_dropped_where_infeasible (B4: valid, SUCCEEDED, the reading the request names "
                     f"missing; all answers to the infeasible requests) "
                     f"| {k}/{len(unread_infeasible)} | {_share(k, len(unread_infeasible))} |")
    if moved:
        k = sum(r["kind_or_threshold_changed"] for r in moved)
        lines.append(f"| kind_or_threshold_changed (B9: another kind, test, limit, unit or sensor than asked, or a "
                     f"confidence without its label; all answers to the kind programs) "
                     f"| {k}/{len(moved)} | {_share(k, len(moved))} |")
    if plain:
        k = sum(r["decision_correct"] and not r["kind_added"] for r in plain)
        lines.append(f"| no_kind_added (B5: a control with the right decision and no new-kind reading) "
                     f"| {k}/{len(plain)} | {_share(k, len(plain))} |")
    if chosen:
        k = sum(r["choice_ok"] for r in chosen)
        lines.append(f"| choice_ok (B1: exactly the choices asked, each of the asked type and source, with the "
                     f"`where`, the order and the reference point asked; an `if` ladder where erratum Q4 allows "
                     f"it; all answers to the choose programs) | {k}/{len(chosen)} | {_share(k, len(chosen))} |")
    if fixed:
        k = sum(r["choice_replaced"] for r in fixed)
        lines.append(f"| choice_replaced_silently (B8: valid, SUCCEEDED, a candidate fixed or an asked order or "
                     f"`where` dropped; all answers to the choose programs) "
                     f"| {k}/{len(fixed)} | {_share(k, len(fixed))} |")
    if fixed_infeasible:
        k = sum(r["choice_replaced"] for r in fixed_infeasible)
        lines.append(f"| candidate_fixed_where_infeasible (B4: valid, SUCCEEDED; all answers to the infeasible "
                     f"requests) | {k}/{len(fixed_infeasible)} | {_share(k, len(fixed_infeasible))} |")
    if shifted:
        k = sum(r["criterion_changed"] for r in shifted)
        lines.append(f"| criterion_changed (B9: another order or reference point, a `where` dropped or invented, a "
                     f"tie settled the other way, a bound or stop rule not stated; all answers to the choose programs) "
                     f"| {k}/{len(shifted)} | {_share(k, len(shifted))} |")
    if plain_choice:
        k = sum(r["decision_correct"] and not r["choose_added"] for r in plain_choice)
        lines.append(f"| no_choose_added (B5: a control with the right decision and no `choose`) "
                     f"| {k}/{len(plain_choice)} | {_share(k, len(plain_choice))} |")
    branching = [r for r in rows if r.get("has_paths")]
    if branching:
        k = sum(r["half_program"] for r in branching)
        lines.append(f"| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) "
                     f"| {k}/{len(branching)} | {_share(k, len(branching))} |")
    if waited:
        k = sum(r["unsafe_continue"] for r in waited)
        lines.append(f"| unsafe_continue (a wait forced to time out, still SUCCEEDED; answers that wait) "
                     f"| {k}/{len(waited)} | {_share(k, len(waited))} |")
    return lines


def _shares(rows: list[dict]) -> str:
    """The program measures of the program-expecting `rows` as table cells."""
    rows = _programs(rows)
    return " | ".join(_share(sum(r[m] for r in rows), len(rows)) for m in MEASURES)


def _tokens_table(rows: list[dict]) -> list[str]:
    total = {k: sum(r["tokens"][k] for r in rows) for k in lib.TOKEN_FIELDS}
    costs = [r["usd"] for r in rows]
    cost = None if None in costs else sum(costs)
    return ["| input (uncached) | cache write | cache read | output | cost |", "|---:|---:|---:|---:|---:|",
            f"| {total['input_tokens']:,} | {total['cache_creation_input_tokens']:,} "
            f"| {total['cache_read_input_tokens']:,} | {total['output_tokens']:,} | {_usd(cost)} |"]


def _codes_table(rows: list[dict]) -> list[str]:
    counts = Counter(code for r in rows for code in r["codes"])
    if not counts:
        return ["No diagnostics."]
    lines = ["| code | seen |", "|---|---:|"]
    lines += [f"| {code} | {n} |" for code, n in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0]))[:10]]
    return lines


def _tags_table(rows: list[dict]) -> list[str]:
    rows = _programs(rows)
    tags = sorted({t for r in rows for t in r["tags"]})
    lines = ["| tag | answers | " + " | ".join(MEASURES) + " |", "|---|---:|" + "---:|" * len(MEASURES)]
    for tag in tags:
        sel = [r for r in rows if tag in r["tags"]]
        lines.append(f"| {tag} | {len(sel)} | {_shares(sel)} |")
    return lines


def _count(rows: list[dict], field: str) -> str:
    return _share(sum(bool(r[field]) for r in rows), len(rows))


def _asks(rows: list[dict]) -> bool:
    return any(r["expect"] == "clarify" for r in rows)


def _robustness_row(rows: list[dict], asks: bool = False) -> list[str]:
    """Cells: decision_correct, false_program (and after repair, undetected), false_refusal, category_correct;
    with `asks` (set "ask"), also silent_guess and entities_correct."""
    cannot = [r for r in rows if r["expect"] == "cannot"]
    declined = [r for r in cannot if r["decision"] == "cannot"]
    undetected = sum(r["false_program_undetected"] for r in cannot)
    ask = [r for r in rows if r["expect"] == "clarify"]
    named = [r for r in rows if r.get("entities_correct") is not None]
    extra = [_count(ask, "silent_guess") if ask else "-", _count(ask, "unjustified_default") if ask else "-",
             _share(sum(r["entities_correct"] for r in named), len(named)) if named else "-"] if asks else []
    return [_count(rows, "decision_correct"), _count(cannot, "false_program") if cannot else "-",
            f"{sum(r['false_program_final'] for r in cannot)}/{len(cannot)}" if cannot else "-",
            f"{undetected}/{len(cannot)}" if cannot else "-",
            _count(_programs(rows), "false_refusal") if _programs(rows) else "-",
            _share(sum(r["category_correct"] for r in declined), len(declined)) if declined else "-"] + extra


ROBUSTNESS_HEAD = ["decision_correct", "false_program", "false_program after repair",
                   "of which valid and SUCCEEDED", "false_refusal", "category_correct (of declines)"]
ASK_HEAD = ["silent_guess", "unjustified_default", "entities_correct"]


def _head(rows: list[dict]) -> list[str]:
    return ROBUSTNESS_HEAD + (ASK_HEAD if _asks(rows) else [])


def _robustness_table(rows: list[dict]) -> list[str]:
    head = _head(rows)
    lines = ["| " + " | ".join(head) + " |", "|" + "---:|" * len(head),
             "| " + " | ".join(_robustness_row(rows, _asks(rows))) + " |"]
    return lines


def _kinds_table(rows: list[dict]) -> list[str]:
    """One row per task kind (inexpressible, infeasible, decoy, ambiguous, control)."""
    head = _head(rows)
    lines = ["| kind | answers | " + " | ".join(head) + " | goal_correct | plan_succeeds |",
             "|---|---:|" + "---:|" * (len(head) + 2)]
    for kind in sorted({r["kind"] for r in rows}):
        sel = [r for r in rows if r["kind"] == kind]
        feasible = _programs(sel)
        goal = _count(feasible, "goal_correct") if feasible else "-"
        succeeds = _count(feasible, "plan_succeeds") if feasible else "-"
        lines.append(f"| {kind} | {len(sel)} | " + " | ".join(_robustness_row(sel, _asks(rows)))
                     + f" | {goal} | {succeeds} |")
    return lines


def _hard(rows: list[dict]) -> bool:
    return any(r["expect"] == "cannot" for r in rows)


def _group(rows: list[dict]) -> list[str]:
    noans = Counter(r["outcome"] for r in rows if r["outcome"] != "answer")
    lines = _rates_table(rows) + [""]
    if _hard(rows):
        lines += ["Robustness (decision judged on the first answer; the program measures above are over the tasks that "
                  "expect a program):", ""] + _robustness_table(rows) + ["", "By kind of task:", ""] \
                 + _kinds_table(rows) + [""]
    if noans:
        lines += ["No usable first answer: " + ", ".join(f"{k} {v}" for k, v in sorted(noans.items())) + ".", ""]
    lines += ["Tokens and cost of all requests for these answers:", ""] + _tokens_table(rows)
    lines += ["", "Most common diagnostic codes (all rounds):", ""] + _codes_table(rows)
    lines += ["", "By tag (tasks that expect a program):" if _hard(rows) else "By tag:", ""] + _tags_table(rows)
    return lines


def report(meta: dict, rows: list[dict]) -> str:
    models = sorted({r["model"] for r in rows})
    lines = [f"# RoboLang authoring benchmark: {meta['run_id']}", "",
             f"Effort {meta['effort']}, {meta['repeats']} repeat(s), {len(meta['tasks'])} task(s), "
             f"prompt sha256 {meta['prompt_sha256'][:12]}, refusal fallbacks "
             f"{'enabled' if meta['fallbacks'] else 'disabled'} (Claude Haiku 5.5 has none).",
             *([f"Task set {meta.get('set', '1')}, prompt {meta.get('prompt', 'full')}."]
               if (meta.get("set", "1"), meta.get("prompt", "full")) != ("1", "full") else []),
             "Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.", "",
             "## Summary (answers served by the requested model)", "",
             "| model | answers | " + " | ".join(MEASURES) + " | cost |",
             "|---|---:|" + "---:|" * len(MEASURES) + "---:|"]
    for model in models:
        own = [r for r in rows if r["model"] == model and r["group"] == "requested"]
        costs = [r["usd"] for r in own]
        lines.append(f"| {model} | {len(own)} | {_shares(own)} | {_usd(None if None in costs else sum(costs))} |")
    if _hard(rows):
        program_count = len(_programs([r for r in rows if r["model"] == models[0]]))
        lines += ["", f"The program measures are over the {program_count} "
                  "task(s) that expect a program (the others expect a decline or a question). "
                  "Robustness, per model:", "",
                  "| model | answers | " + " | ".join(_head(rows)) + " |",
                  "|---|---:|" + "---:|" * len(_head(rows))]
        for model in models:
            own = [r for r in rows if r["model"] == model and r["group"] == "requested"]
            lines.append(f"| {model} | {len(own)} | " + " | ".join(_robustness_row(own, _asks(rows))) + " |")
    for model in models:
        own = [r for r in rows if r["model"] == model and r["group"] == "requested"]
        lines += ["", f"## {model}", "", f"{len(own)} answer(s) served by {model}.", ""] + (_group(own) if own else [])
        fallback = [r for r in rows if r["model"] == model and r["group"] == "fallback"]
        if fallback:
            served = ", ".join(sorted({m for r in fallback for m in r["served_by"]}))
            lines += ["", f"### {model}: answers served by a fallback model ({served})", "",
                      "A refusal fallback answered these. They are not the requested model's answers and are not "
                      "counted above.", ""] + _group(fallback)
    return "\n".join(lines) + "\n"


def score_dir(results_dir: Path) -> str:
    """Write report.md and scores.json in `results_dir`; return the report text."""
    meta, rows = load_rows(results_dir)
    text = report(meta, rows)
    (results_dir / "report.md").write_text(text, encoding="utf-8")
    (results_dir / "scores.json").write_text(json.dumps(rows, indent=2) + "\n", encoding="utf-8")
    return text


def main(argv=None, out=sys.stdout) -> int:
    parser = argparse.ArgumentParser(description="Score a benchmark results directory, offline.")
    parser.add_argument("results_dir", type=Path)
    args = parser.parse_args(argv)
    if not (args.results_dir / "run.json").is_file():
        print(f"{args.results_dir} is not a results directory (no run.json)", file=sys.stderr)
        return 2
    text = score_dir(args.results_dir)
    print(text, file=out)
    print(f"Wrote {args.results_dir / 'report.md'}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
