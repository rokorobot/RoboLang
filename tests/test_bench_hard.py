"""
Benchmark set 2, "hard" (bench/tasks_hard/, decision record 0024, section "Set 2"): robustness, not syntax.

Every feasible task has a reference that must be valid, reach its goal and succeed. Every task that expects a decline
has a documented `why` and an "obvious attempt" that must fail for the stated reason. The scorer is driven with
canned answers for each headline metric. No network and no SDK; set 1 scoring is covered by test_bench.py.
"""

from __future__ import annotations
import io
import json
import re
from collections import Counter
from pathlib import Path

import pytest
from test_bench import FakeClient, message, reference_answer

from bench import lib, run, score
from robolang import tokens

REPO = Path(__file__).resolve().parent.parent
HARD = lib.load_set("hard")
FEASIBLE = {t: d for t, d in HARD.items() if lib.expectation(d)[0] == "program" and d["group"] != "branching"}
BRANCHING = {t: d for t, d in HARD.items() if d["group"] == "branching"}  # 1.3 declined them, 1.4 expresses them
CANNOT = {t: d for t, d in HARD.items() if lib.expectation(d)[0] == "cannot"}
INEXPRESSIBLE = {t for t, d in CANNOT.items() if lib.expectation(d)[1] == "inexpressible"}
INFEASIBLE = {t for t, d in CANNOT.items() if lib.expectation(d)[1] == "infeasible"}


def hard_reference(task_id: str) -> dict:
    ref = json.loads((lib.HARD_REFERENCE_DIR / f"{task_id}.json").read_text(encoding="utf-8"))
    return {"decision": "program", "category": "none", "reason": "ok",
            "rob": (lib.HARD_REFERENCE_DIR / f"{task_id}.rob").read_text(encoding="utf-8"), **ref}


def attempt_answer(task_id: str) -> dict:
    """The obvious attempt at a task that expects a decline, as a program answer."""
    stem = lib.HARD_REFERENCE_DIR / f"{task_id}.attempt"
    bindings = Path(f"{stem}.json")
    ref = json.loads(bindings.read_text(encoding="utf-8")) if bindings.is_file() else {"task": "x", "bindings": {}}
    return {"decision": "program", "category": "none", "reason": "an attempt",
            "rob": Path(f"{stem}.rob").read_text(encoding="utf-8"), **ref}


def decline(category: str, reason: str = "cannot be done") -> dict:
    return {"decision": "cannot", "category": category, "reason": reason, "rob": "", "task": "", "bindings": []}


# -------- the set --------

def test_the_mix_of_tasks():
    assert len(HARD) == 40
    assert Counter(d["group"] for d in HARD.values()) == {"inexpressible": 7, "branching": 3, "infeasible": 8,
                                                          "decoy": 8, "ambiguous": 5, "control": 9}
    assert len(INEXPRESSIBLE) == 7 and len(INFEASIBLE) == 8
    assert (len(FEASIBLE) + len(BRANCHING)) / len(HARD) >= 0.35  # refusing everything must score badly
    for task_id, task in HARD.items():
        assert task["id"] == task_id and (lib.HARD_TASKS_DIR / f"{task_id}.json").is_file()
        assert (REPO / task["world"]).is_file() and task["tags"] and task["prompt"].endswith((".", "?"))
        assert task["group"] in {"inexpressible", "branching", "infeasible", "decoy", "ambiguous", "control"}


def test_references_and_attempts_are_where_they_should_be():
    files = list(lib.HARD_REFERENCE_DIR.iterdir())
    refs = {p.name.split(".")[0] for p in files if p.name.endswith((".rob", ".json")) and ".attempt" not in p.name}
    attempts = {p.name.split(".")[0] for p in files if ".attempt." in p.name}
    assert refs == set(FEASIBLE) | set(BRANCHING) and attempts == set(CANNOT)
    assert {p.stem for p in lib.HARD_REFERENCE_DIR.glob("*.rob") if ".attempt" not in p.stem} \
        == set(FEASIBLE) | set(BRANCHING)


def test_expectations_are_well_formed():
    for task_id, task in HARD.items():
        decision, category = lib.expectation(task)
        if task["group"] == "branching":  # relabelled for 1.4 (N4 PR 3): a program, judged per path
            assert (decision, category) == ("program", "none")
            assert task["expect_1_3"] == {"decision": "cannot", "category": "inexpressible"}
            assert not {"expected_goal", "expected_goals"} & set(task) and len(task["why"]) > 40
            paths = task["expect_1_4"]["paths"]
            assert task["expect_1_4"]["decision"] == "program" and len(paths) == 2
            assert all(p["world_patch"]["events"] == [] and p["when"] and ("goal" in p) != ("fail" in p) for p in paths)
        elif decision == "program":
            assert category == "none" and "why" not in task and "attempt_fails_with" not in task
            goals = lib.expected_goals(task)
            assert goals and all(g for g in goals), task_id
            assert ("expected_goals" in task) == (task["group"] == "ambiguous"), task_id
            if task["group"] == "ambiguous":
                assert len(goals) >= 2 and all(goals.count(g) == 1 for g in goals)
            for goal in goals:
                assert {t[1] for t in goal} <= {"located_in", "carried_by"}
        else:
            assert decision == "cannot" and category in ("inexpressible", "infeasible")
            assert not {"expected_goal", "expected_goals"} & set(task) and task["group"] == category
            assert len(task["why"]) > 40 and re.fullmatch(r"[ES]\d{3}", task["attempt_fails_with"])


# -------- the feasible tasks --------

@pytest.mark.parametrize("task_id", sorted(FEASIBLE))
def test_reference_is_valid_exports_an_accepted_goal_and_succeeds(task_id):
    task = FEASIBLE[task_id]
    answer = hard_reference(task_id)
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == "1.3"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.goal_exact
    assert judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    has_checks = any(k in task for k in ("expected_safety", "expected_messages", "expected_message_counts",
                                         "expected_end_ms", "probes", "expected_action_sequences"))
    assert bool(judgement.checks) == has_checks and all(judgement.checks.values())


def test_ambiguous_tasks_accept_each_defensible_goal_and_nothing_else():
    task = FEASIBLE["m_park_night"]
    for place, accepted in (("dock", True), ("charging", True), ("table", False)):
        answer = hard_reference("m_park_night")
        answer["bindings"]["spot"] = place
        judgement = lib.judge(task, lib.check_answer(task, answer), answer["bindings"])
        assert judgement.goal_correct is accepted and judgement.plan_succeeds
    # a goal set is accepted whole: half of a reading is not a reading
    task = FEASIBLE["m_same_shelf"]
    answer = hard_reference("m_same_shelf")
    answer["rob"] = answer["rob"].replace("        object_at b dst;\n", "")
    judgement = lib.judge(task, lib.check_answer(task, answer), answer["bindings"])
    assert not judgement.goal_correct


def test_pass_on_requires_recipient_custody_instead_of_a_nearby_location():
    task = HARD["m_pass_p1"]
    answer = hard_reference("m_pass_p1")
    answer["rob"] = answer["rob"].replace("holding receiver box;", "object_at box dst;")
    answer["rob"] = answer["rob"].replace("        grasp receiver box;\n", "")
    judgement = lib.judge(task, lib.check_answer(task, answer), answer["bindings"])
    assert judgement.plan_succeeds and not judgement.goal_correct and not judgement.checks["actions"]


def test_the_new_optional_checks_bite():
    task = FEASIBLE["k_round_trip_twice"]  # a repeated action really repeats
    answer = hard_reference("k_round_trip_twice")
    once = answer["rob"].replace('        communicate r to "ops" with "TRIP";\n', "", 1)
    answer["rob"] = once
    judgement = lib.judge(task, lib.check_answer(task, answer), answer["bindings"])
    assert judgement.plan_succeeds and judgement.goal_correct and judgement.checks == {"message_counts": False}
    task = FEASIBLE["k_inspect_thrice"]
    answer = hard_reference("k_inspect_thrice")
    answer["rob"] = answer["rob"].replace("wait for 5 s;", "wait for 1 s;")
    assert lib.judge(task, lib.check_answer(task, answer), answer["bindings"]).checks == {"end_ms": False}
    task = FEASIBLE["k_not_pre"]  # the probe: the request said "only if the aisle is not clear"
    answer = hard_reference("k_not_pre")
    answer["rob"] = answer["rob"].replace("    pre { not region_clear aisle; }\n", "")
    assert lib.judge(task, lib.check_answer(task, answer), answer["bindings"]).checks == {"probes": False}


@pytest.mark.parametrize("task_id", sorted(BRANCHING))
def test_branching_reference_is_valid_1_4_and_right_on_every_path(task_id):
    task = BRANCHING[task_id]
    answer = hard_reference(task_id)
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == "1.4"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds and not judgement.half_program, judgement.paths
    assert len(judgement.paths) == 2 and all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths)


# -------- the tasks that expect a decline --------

def test_inexpressible_requests_fail_in_the_language():
    """The natural attempt is rejected by `robolang validate`, with the code recorded in the task."""
    for task_id in sorted(INEXPRESSIBLE):
        task = HARD[task_id]
        checked = lib.check_answer(task, attempt_answer(task_id))
        assert not checked.valid, task_id
        assert checked.codes[0] == task["attempt_fails_with"], (task_id, checked.codes)


def test_the_inexpressible_attempts_still_fail_under_the_1_4_marker():
    """1.4 adds `if`, `else` and `fail` only: a retry, loop, count, clock or message wait is still rejected."""
    for task_id in sorted(INEXPRESSIBLE):
        answer = attempt_answer(task_id)
        answer["rob"] = answer["rob"].replace("robolang 1.3;", "robolang 1.4;", 1)
        checked = lib.check_answer(HARD[task_id], answer)
        assert not checked.valid and checked.codes[0] == HARD[task_id]["attempt_fails_with"], (task_id, checked.codes)


def test_the_language_has_no_construct_for_what_the_inexpressible_tasks_ask():
    """What is left inexpressible in 1.4: no loop, retry, count, clock or message wait (decision 0027 section 1)."""
    grammar = (lib.ROOT / "spec" / "grammar-v1.3.ebnf").read_text(encoding="utf-8")
    statement = re.search(r"statement\s*=\s*\(([^)]*)\)", grammar).group(1)
    assert set(re.findall(r"\w+", statement)) == {"move", "grasp", "place", "inspect", "wait", "communicate"}
    atoms = set(re.findall(r'^\s+(?:atom\s+=\s+)?\|?\s*"(\w+)"\s*,\s*(?:IDENT|sensor_ref)', grammar, re.M))
    assert atoms == {"robot_ready", "region_clear", "object_exists", "sensor_ok", "object_at", "robot_at", "holding"}
    assert "message_received" in tokens.FUTURE_RESERVED  # reserved, not implemented (E211)
    for word in ("if", "else", "while", "loop", "every", "retry", "repeat", "for_each"):
        assert word not in tokens.RESERVED_V1_3 and f'"{word}"' not in grammar  # unknown word: E203, not a keyword
    assert 'duration = NUMBER , DURATION_UNIT' in re.sub(r"\s+", " ", grammar).replace("duration      =", "duration =")
    assert re.search(r'"wait"\s*,\s*"until"\s*,\s*condition\s*,\s*"timeout"\s*,\s*duration', grammar)  # always bounded
    assert re.search(r'"communicate"\s*,\s*IDENT\s*,\s*"to"\s*,\s*STRING\s*,\s*"with"\s*,\s*STRING', grammar)
    # 1.4 adds `if`, `else` and `fail` and nothing else: the loop and retry words are still not in the grammar
    grammar14 = lib.GRAMMAR_FILE.read_text(encoding="utf-8")
    for word in ("if", "else", "fail"):
        assert f'"{word}"' in grammar14
    for word in ("while", "loop", "every", "retry", "repeat", "for_each", "message_received"):
        assert f'"{word}"' not in grammar14


def test_infeasible_requests_fail_in_the_world():
    """The obvious program fails `validate`, the world check, or the simulation, with the recorded code."""
    for task_id in sorted(INFEASIBLE):
        task = HARD[task_id]
        answer = attempt_answer(task_id)
        checked = lib.check_answer(task, answer)
        if not checked.valid:  # caught by `validate --world` before any simulation (N2)
            assert checked.codes == (task["attempt_fails_with"],), task_id
            assert checked.codes[0] in {"E505", "S001", "S002", "S211", "S231"}, task_id
            continue
        assert task["attempt_fails_with"] not in {"E505", "S001", "S002", "S211", "S231"}, task_id
        judgement = lib.judge(task, checked, answer["bindings"])
        assert not judgement.plan_succeeds and judgement.state == "FAILED", task_id
        assert judgement.failure_code == task["attempt_fails_with"], (task_id, judgement.failure_code)


def test_the_infeasible_worlds_really_lack_what_is_asked():
    cleanroom = json.loads((REPO / "bench/worlds/cleanroom.json").read_text(encoding="utf-8"))
    assert cleanroom["objects"]["flask_9"]["present"] is False
    lab = json.loads((REPO / "bench/worlds/lab.json").read_text(encoding="utf-8"))
    assert "incubator" not in lab["locations"] and lab["robots"]["rover1"]["gripper"] is False
    assert lab["sensors"]["cam_rover"]["robot"] == "rover1"
    assert {q["unit"] for q in lab["robots"]["rover1"]["max_speed"]} == {"m/s"}
    warehouse = json.loads((REPO / "bench/worlds/warehouse.json").read_text(encoding="utf-8"))
    assert "pallet5" not in warehouse["objects"] and warehouse["robots"]["amr2"]["ready"] is False
    assert not [e for e in warehouse["events"] if e["target"] == "amr2"]
    base = json.loads((REPO / "examples/world.json").read_text(encoding="utf-8"))
    assert max(float(q["value"]) for q in base["robots"]["arm1"]["max_speed"]) < 2


# -------- large and decoy worlds --------

def normal(name: str) -> str:
    return re.sub(r"[_\s]", "", name).lower()


def entities(world: dict) -> list[str]:
    return list(world["locations"]) + [n for key in ("regions", "robots", "objects", "sensors") for n in world[key]]


@pytest.mark.parametrize("name", ["depot", "cleanroom"])
def test_decoy_worlds_are_large_with_near_miss_names_and_descriptions(name):
    world = json.loads((REPO / f"bench/worlds/{name}.json").read_text(encoding="utf-8"))
    notes = json.loads((REPO / f"bench/worlds/{name}.notes.json").read_text(encoding="utf-8"))
    ids = entities(world)
    assert len(ids) >= 30 and len(set(ids)) == len(ids)
    assert set(notes) == set(ids) and all(isinstance(t, str) and t for t in notes.values())
    clashes = [n for n, c in Counter(map(normal, ids)).items() if c > 1]
    assert len(clashes) >= 8, "box_1 vs box1, shelf_a vs shelf_A: the names must be near misses"
    assert any(a != b and a.lower() == b.lower() for a in ids for b in ids)  # a case-only difference


def test_decoy_tasks_use_the_large_worlds_and_show_their_descriptions():
    decoys = {t: d for t, d in HARD.items() if d["group"] == "decoy"}
    assert len(decoys) == 8 and {d["world"] for d in decoys.values()} == {"bench/worlds/depot.json",
                                                                          "bench/worlds/cleanroom.json"}
    for task in decoys.values():
        message_text = lib.user_message(task)
        assert "\ndescriptions:\n  " in message_text and f"  {next(iter(lib.notes_for(task)))}: " in message_text
        assert message_text.endswith(task["prompt"])
        assert not re.search(r"\b(box_1|box1|vial1|crate01|part_07|tote_A)\b", task["prompt"])  # described, not named


def test_set_1_messages_are_unchanged_by_the_notes_and_absent_objects():
    for task in lib.load_tasks().values():
        assert "descriptions:" not in lib.user_message(task) and lib.notes_for(task) == {}
    cleanroom = json.loads((REPO / "bench/worlds/cleanroom.json").read_text(encoding="utf-8"))
    assert "flask_9 (flask, not present)" in lib.describe_world(cleanroom)


def test_every_entity_of_every_hard_world_is_shown_to_the_model():
    for task in HARD.values():
        world = json.loads(lib.world_bytes(task))
        text = lib.user_message(task)
        assert all(i in text for i in entities(world)), task["id"]


# -------- the answer format --------

def test_old_answers_are_programs_and_new_ones_are_read_fully():
    old = {"rob": "x", "task": "t", "bindings": [{"param": "r", "id": "arm1"}]}
    answer, problem = lib.parse_answer(json.dumps(old))
    assert problem is None and (answer["decision"], answer["category"], answer["reason"]) == ("program", "none", "")
    answer, problem = lib.parse_answer(json.dumps(decline("infeasible", "no gripper")))
    assert problem is None and answer["decision"] == "cannot" and answer["bindings"] == {}
    for bad in (dict(decline("infeasible"), decision="maybe"), dict(decline("infeasible"), category="other"),
                dict(decline("infeasible"), reason=3), {k: v for k, v in decline("none").items() if k != "reason"},
                dict(old, decision="program")):
        assert lib.parse_answer(json.dumps(bad))[0] is None


def test_the_schema_requires_every_field_and_closes_the_objects():
    schema = lib.ANSWER_SCHEMA
    assert list(schema["properties"]) == ["decision", "category", "reason", "entities", "rob", "task", "bindings"]
    assert schema["required"] == list(schema["properties"]) and schema["additionalProperties"] is False
    assert schema["properties"]["decision"]["enum"] == ["program", "cannot", "clarify"]
    assert schema["properties"]["category"]["enum"] == ["inexpressible", "infeasible", "none"]


# -------- the prompts --------

def test_the_minimal_prompt_is_the_grammar_and_the_answer_format_only():
    full, minimal = lib.load_prompt("full"), lib.load_prompt("minimal")
    grammar = lib.GRAMMAR_FILE.read_text(encoding="utf-8").rstrip("\n")
    assert grammar in minimal and lib.GRAMMAR_MARK not in minimal
    assert "# Example" not in minimal and "# The language" not in minimal and "E303" not in minimal
    assert len(minimal) < len(full)
    paragraph = next(p for p in full.split("\n\n") if p.startswith("You may decline."))
    assert paragraph in minimal.split("\n\n")  # both prompts say the same about declining, once
    assert full.count("You may decline.") == 1 and "cannot" in paragraph
    assert not any(task_id in full + minimal for task_id in HARD)  # no hint at task specifics


# -------- the scorer on canned answers --------

def hard_run(tmp_path: Path, replies: dict, repairs: dict | None = None, models=("claude-opus-5-5",)) -> Path:
    def script(request):
        first = request["messages"][0]["content"]
        task_id = next(t for t, d in HARD.items() if first.endswith(f"Request: {d['prompt']}"))
        table = (repairs or {}) if len(request["messages"]) > 1 else replies
        return message(table[task_id], model=request["model"])

    out_dir = tmp_path / "hard"
    run.run_benchmark(FakeClient(script), list(models), {t: HARD[t] for t in replies}, "medium", 1, out_dir,
                      lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="hard", prompt_name="full")
    return out_dir


def canned(tmp_path: Path) -> tuple[Path, dict]:
    replies = {
        "x_wait_message": decline("inexpressible"),                            # right
        "x_wait_forever": decline("infeasible"),                               # declined, wrong category
        "x_every_hour": attempt_answer("x_every_hour"),                        # program, invalid: repaired below
        "x_retry_grasp": dict(reference_answer("e_pick_place"), decision="program", category="none",
                              reason="r"),                                     # valid, succeeds: undetected
        "n_no_gripper_pick": attempt_answer("n_no_gripper_pick"),              # program, valid, fails in the simulation
        "n_missing_place": decline("infeasible"),                              # right
        "k_inspect_thrice": decline("inexpressible"),                          # false refusal
        "d_red_upper_box": hard_reference("d_red_upper_box"),                  # right
        "m_park_night": dict(hard_reference("m_park_night"), reason="dock"),   # accepted reading
        "m_same_shelf": dict(hard_reference("m_same_shelf")),                  # accepted reading
    }
    replies["m_park_night"]["bindings"] = {"r": "amr1", "spot": "dock"}
    repairs = {"x_every_hour": decline("inexpressible")}                       # the diagnostics made it decline
    return hard_run(tmp_path, replies, repairs), replies


def test_scorer_headline_metrics_on_canned_answers(tmp_path):
    out_dir, _ = canned(tmp_path)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "hard" and meta["prompt"] == "full"

    right = by["x_wait_message"]
    assert right["decision_correct"] and right["category_correct"] is True and not right["false_program"]
    assert right["valid_first"] is False and right["codes"] == []  # a decline has no program to check

    wrong_category = by["x_wait_forever"]
    assert wrong_category["decision_correct"] and wrong_category["category_correct"] is False

    repaired = by["x_every_hour"]  # a program for a cannot-task: the dangerous error, even though repair rescued it
    assert repaired["false_program"] and not repaired["false_program_final"] and not repaired["decision_correct"]
    assert repaired["decision"] == "program" and repaired["decision_final"] == "cannot" and repaired["repaired"]
    assert repaired["codes"] == ["E203", "E201"]

    undetected = by["x_retry_grasp"]  # valid, ends SUCCEEDED: nothing in the tools would have caught it
    assert undetected["false_program"] and undetected["false_program_final"] and undetected["false_program_undetected"]

    caught = by["n_no_gripper_pick"]  # a program for an infeasible request: `validate --world` rejected it (N2)
    assert caught["false_program"] and not caught["false_program_undetected"] and not caught["valid_first"]
    assert "S211" in caught["codes"]

    assert by["n_missing_place"]["decision_correct"] and by["n_missing_place"]["category_correct"] is True

    refusal = by["k_inspect_thrice"]
    assert refusal["false_refusal"] and refusal["false_refusal_final"] and not refusal["decision_correct"]
    assert not refusal["valid_after_repair"] and not refusal["goal_correct"] and refusal["category_correct"] is None

    ok = by["d_red_upper_box"]
    assert ok["decision_correct"] and ok["goal_correct"] and ok["plan_succeeds"] and not ok["false_refusal"]
    assert by["m_park_night"]["goal_correct"] and by["m_park_night"]["decision_correct"]
    assert by["m_same_shelf"]["goal_correct"] and by["m_same_shelf"]["plan_succeeds"]


def test_the_report_has_the_headline_figures_and_a_table_per_kind(tmp_path):
    out_dir, _ = canned(tmp_path)
    text = score.score_dir(out_dir)
    assert "Task set hard, prompt full." in text
    for column in score.ROBUSTNESS_HEAD:
        assert column in text
    # 10 answers, 6 of which expect a decline (x_wait_message, x_wait, x_every, x_retry, n_gripper, n_missing_place):
    # decision right 6 (the three declines, d_red, m_park, m_same); false programs x_every, x_retry, n_gripper (3),
    # 2 after repair, 1 of them valid and SUCCEEDED (x_retry); 1 false refusal of 4; categories right in 2 of 3 declines
    assert "| decision_correct |" in text
    robustness = text.split("Robustness, per model:")[1].split("\n## ")[0]
    assert "| claude-opus-5-5 | 10 | 6/10 (60.0%) | 3/6 (50.0%) | 2/6 | 1/6 | 1/4 (25.0%) | 2/3 (66.7%) |" in robustness
    kinds = text.split("By kind of task:")[1]
    for kind in ("inexpressible", "infeasible", "decoy", "ambiguous", "control"):
        assert f"| {kind} |" in kinds
    # the four rates are over the four feasible tasks only (d_red_upper_box, m_park_night, m_same_shelf,
    # k_inspect_thrice)
    assert "| goal_correct | 3/4 | 3/4 (75.0%) |" in text
    assert score.score_dir(out_dir) == text  # deterministic


def test_a_decline_is_never_repaired_and_an_invalid_program_is_(tmp_path):
    client_requests = []

    def script(request):
        client_requests.append(request)
        return message(decline("inexpressible"))

    out_dir = tmp_path / "d"
    run.run_benchmark(FakeClient(script), ["claude-opus-5-5"], {"x_wait_message": HARD["x_wait_message"]}, "medium", 1,
                      out_dir, lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="hard")
    assert len(client_requests) == 1
    row = score.load_rows(out_dir)[1][0]
    assert row["decision_correct"] and row["category_correct"] and not row["repaired"]


def test_set_1_results_score_the_same_with_the_old_and_the_new_answer_form(tmp_path):
    from test_bench import canned_run
    old_dir, _ = canned_run(tmp_path)  # answers have the three old keys; run.json is written by the current code
    meta = json.loads((old_dir / "run.json").read_text(encoding="utf-8"))
    del meta["set"], meta["prompt"]  # as saved before set 2
    (old_dir / "run.json").write_text(json.dumps(meta), encoding="utf-8")
    old_text = score.score_dir(old_dir)
    assert "Task set" not in old_text and "Robustness" not in old_text and "By kind" not in old_text
    assert "| valid_first | 3/5 | 3/5 (60.0%) |" in old_text and "| goal_correct | 3/5 |" in old_text
    old_rows = score.load_rows(old_dir)[1]
    for path in old_dir.glob("*/*.json"):  # the same answers in the new form, with decision "program"
        record = json.loads(path.read_text(encoding="utf-8"))
        for call in record["calls"]:
            for block in call["response"]["content"]:
                if block["type"] == "text":
                    block["text"] = json.dumps({"decision": "program", "category": "none", "reason": "",
                                                **json.loads(block["text"])})
        path.write_text(json.dumps(record), encoding="utf-8")
    new_rows = score.load_rows(old_dir)[1]
    for a, b in zip(old_rows, new_rows):
        assert a == b


# -------- the flags --------

def test_set_hard_prints_the_estimate_and_spends_nothing(tmp_path):
    out, err = io.StringIO(), io.StringIO()
    code = run.main(["--set", "hard", "--out", str(tmp_path)], client_factory=lambda key: pytest.fail("no client"),
                    out=out, err=err, environ={"ANTHROPIC_API_KEY": "sk-ant-api03-TESTKEY-0123456789abcdef"})
    assert code == 2 and list(tmp_path.iterdir()) == []
    text = out.getvalue()
    assert "Estimated cost" in text and "Nothing was sent" in text and "total ~$" in text
    estimate = run.estimate(["claude-opus-5-5"], HARD, "medium", 1, lib.load_prompt("full"))
    assert 0 < estimate["usd"] < 20 and estimate["models"]["claude-opus-5-5"]["requests"] >= 40


def test_the_default_set_is_1_and_the_flags_are_checked(tmp_path):
    args = run.parse_args([])
    assert (args.task_set, args.prompt_name) == ("1", "full")
    assert run.parse_args(["--set", "hard", "--prompt", "minimal"]).prompt_name == "minimal"
    err = io.StringIO()
    assert run.main(["--set", "hard", "--tasks", "e_pick_place", "--out", str(tmp_path)], out=io.StringIO(), err=err,
                    environ={}) == 2 and "unknown task" in err.getvalue()
    with pytest.raises(SystemExit):
        run.parse_args(["--set", "4"])
    with pytest.raises(SystemExit):
        run.parse_args(["--prompt", "tiny"])


def test_the_minimal_prompt_run_records_set_and_prompt(tmp_path):
    prompts = []

    def script(request):
        prompts.append(request["system"][0]["text"])
        return message(decline("inexpressible"))

    key = "sk-ant-api03-TESTKEY-0123456789abcdef"
    code = run.main(["--yes", "--set", "hard", "--prompt", "minimal", "--tasks", "x_wait_message", "--models",
                     "claude-haiku-5-5", "--out", str(tmp_path), "--run-id", "m"],
                    client_factory=lambda api_key: FakeClient(script), out=io.StringIO(), err=io.StringIO(),
                    environ={"ANTHROPIC_API_KEY": key})
    assert code == 0 and prompts == [lib.load_prompt("minimal")]
    meta = json.loads((tmp_path / "m" / "run.json").read_text(encoding="utf-8"))
    assert (meta["set"], meta["prompt"]) == ("hard", "minimal")
    rerun = FakeClient(lambda request: pytest.fail("saved answers are not asked again"))
    with pytest.raises(run.FatalError):  # another prompt under the same run ID is another run
        run.run_benchmark(rerun, ["claude-haiku-5-5"], {"x_wait_message": HARD["x_wait_message"]}, "medium", 1,
                          tmp_path / "m", lib.load_prompt("full"), log=io.StringIO(), task_set="hard",
                          prompt_name="full")


# -------- goals compared by meaning (N6) --------

VIAL_TASK = """robolang 1.3;

task deliver(robot r, object box, object other, location src, location dst) {
    goal { GOAL }
    plan {
        move r to src;
        grasp r box;
        move r to dst;
        place r box at dst;
    }
}
"""


@pytest.mark.parametrize("goal, correct, exact", [
    ("object_at box dst;", True, True),
    ("object_at box dst or object_at other dst;", True, False),        # "any of" the two readings
    ("object_at other dst or object_at box dst;", True, False),
    ("object_at box dst and object_at box dst;", True, True),          # `and` exports flat since N1
    ("not (not object_at box dst);", True, False),
    ("object_at box dst; object_at box dst;", True, True),
    ("object_at box dst and not holding r box;", True, False),        # implied: a placed vial is not held
    ("holding r box;", False, False),
    ("object_at box dst or holding r box;", False, False),
    ("object_at box dst and object_at other dst;", False, False),     # both is neither reading
    ("object_at box dst and robot_ready r;", False, False),           # claims more than the request
    ("not object_at box src;", False, False),
])
def test_goal_correct_compares_meaning_not_triples(goal, correct, exact):
    task = HARD["m_one_vial_scanner"]
    answer = {"decision": "program", "category": "none", "reason": "ok", "task": "deliver",
              "rob": VIAL_TASK.replace("GOAL", goal),
              "bindings": {"r": "arm1", "box": "vial1", "other": "vial2", "src": "bench", "dst": "scanner"}}
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    judgement = lib.judge(task, checked, answer["bindings"])
    assert (judgement.goal_correct, judgement.goal_exact) == (correct, exact)


def test_goal_means_falls_back_to_triples_when_too_large(monkeypatch):
    task = HARD["m_one_vial_scanner"]
    answer = hard_reference("m_one_vial_scanner")
    checked = lib.check_answer(task, answer)
    readings = lib.expected_goals(task)
    assert lib.goal_means(checked.task, answer["bindings"], readings) is True
    monkeypatch.setattr(lib, "MAX_GOAL_STATES", 1)
    assert lib.goal_means(checked.task, answer["bindings"], readings) is None
    assert lib.goal_means(checked.task, answer["bindings"], []) is None


def test_a_disjunction_of_compound_readings_is_correct():
    """"Put box1 and box2 on the same shelf": either shelf, stated as one goal."""
    task = HARD["m_same_shelf"]
    answer = hard_reference("m_same_shelf")
    answer["rob"] = answer["rob"].replace("object_at a dst;\n        object_at b dst;",
                                          "(object_at a dst and object_at b dst) or "
                                          "(object_at a src_b and object_at b src_b);")
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and not judgement.goal_exact and judgement.plan_succeeds
