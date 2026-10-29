"""
Benchmark set 3, "branch" (bench/tasks_branch/, decision record 0027 section 3): pre-registered against RoboLang 1.3,
labelled for RoboLang 1.4 in N4 PR 3.

20 branching requests that 1.3 could not express (`expect_1_3` keeps that label) and 1.4 does (`expect`: a program;
`expect_1_4.paths`: the goal and the world patch of every path; the reference is the 1.4 program), 8 requests that stay
inexpressible in 1.4, and 8 conditional-sounding requests that 1.3 already expresses with `pre` or `wait until`.
No network and no SDK.
"""

from __future__ import annotations
import json
import re
from collections import Counter
from pathlib import Path

import pytest

from bench import lib, score

REPO = Path(__file__).resolve().parent.parent
SET = lib.load_set("branch")
REF = lib.BRANCH_REFERENCE_DIR
FEASIBLE = {t: d for t, d in SET.items() if lib.expectation(d)[0] == "program"}
BRANCHING = {t: d for t, d in FEASIBLE.items() if d["group"] == "branching"}
CANNOT = {t: d for t, d in SET.items() if lib.expectation(d)[0] == "cannot"}
TRIPLE = re.compile(r"rl:(thing|robot):\w+")


def reference(task_id: str) -> dict:
    ref = json.loads((REF / f"{task_id}.json").read_text(encoding="utf-8"))
    return {"decision": "program", "category": "none", "reason": "ok",
            "rob": (REF / f"{task_id}.rob").read_text(encoding="utf-8"), **ref}


def attempt(task_id: str) -> dict:
    bindings = REF / f"{task_id}.attempt.json"
    ref = json.loads(bindings.read_text(encoding="utf-8")) if bindings.is_file() else {"task": "t", "bindings": {}}
    return {"decision": "program", "category": "none", "reason": "an attempt",
            "rob": (REF / f"{task_id}.attempt.rob").read_text(encoding="utf-8"), **ref}


def judged(task_id: str, answer: dict):
    task = SET[task_id]
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    return lib.judge(task, checked, answer["bindings"])


def test_the_mix_of_tasks():
    assert len(SET) == 36
    assert Counter(d["group"] for d in SET.values()) == {"branching": 20, "inexpressible": 8, "control": 8}
    for task_id, task in SET.items():
        assert task["id"] == task_id and (REPO / task["world"]).is_file() and task["tags"]
        assert task["prompt"].endswith(".") and "expect_1_4" in task


def test_files_are_where_they_should_be():
    names = {p.name for p in REF.iterdir()}
    assert {n.split(".")[0] for n in names if ".attempt." in n} == set(CANNOT)
    assert {n.split(".")[0] for n in names if ".attempt." not in n} == set(FEASIBLE)
    for task_id in BRANCHING:
        assert (REF / f"{task_id}.rob").read_text(encoding="utf-8").splitlines()[1] == "robolang 1.4;"


def test_expectations_are_well_formed():
    for task_id, task in SET.items():
        decision, category = lib.expectation(task)
        later = task["expect_1_4"]
        if task["group"] == "control":
            assert (decision, category) == ("program", "none") and later == {"decision": "program"}
            assert lib.expected_goals(task) and "why" not in task and "expect_1_3" not in task
            continue
        assert len(task["why"]) > 40
        if task["group"] == "inexpressible":
            assert (decision, category) == ("cannot", "inexpressible")
            assert later == {"decision": "cannot", "category": "inexpressible"} and "expect_1_3" not in task
            continue
        # branching: a program under 1.4; the 1.3 label stays for history
        assert (decision, category) == ("program", "none") and task["expect"] == {"decision": "program"}
        assert task["expect_1_3"] == {"decision": "cannot", "category": "inexpressible"}
        assert later["decision"] == "program" and len(later["paths"]) >= 2, task_id
        for path in later["paths"]:
            assert path["when"] and ("goal" in path) != ("fail" in path), task_id
            assert path["world_patch"]["events"] == [] and len(path["world_patch"]) >= 2, task_id
            for triple in path.get("goal", []):
                assert len(triple) == 3 and triple[1] in {"located_in", "carried_by"} and TRIPLE.match(triple[0])
            for channel, message in path.get("messages", []):
                assert f'"{channel}"' in task["prompt"] and f'"{message}"' in task["prompt"], task_id


@pytest.mark.parametrize("task_id", sorted(FEASIBLE))
def test_reference_is_valid_reaches_the_goal_on_every_path_and_its_probes_bite(task_id):
    answer = reference(task_id)
    judgement = judged(task_id, answer)
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.paths)
    assert all(judgement.checks.values()), judgement.checks
    assert not judgement.half_program
    if task_id in BRANCHING:
        assert [p["when"] for p in judgement.paths] == [p["when"] for p in SET[task_id]["expect_1_4"]["paths"]]
        assert all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths)
        text = answer["rob"]
        assert text.startswith("//") and "robolang 1.4;" in text


# One-branch answers, one per tag of the baseline (G1, G2): what the models wrote. The condition is read off the world
# description and guarded with `pre` (or not at all). Each is valid and SUCCEEDED in the world as given, and wrong on
# the other path.
HALF = {
    "b_route_by_cell": """robolang 1.4;
task route(robot r, object box, location src, location a, location b, region cell) {
    pre { region_clear cell; }
    goal { object_at box a; }
    plan { move r to src; grasp r box; move r to a; place r box at a; }
}
""",
    "b_whichever_ready": """robolang 1.4;
task deliver(robot r1, robot r2, object box, location src, location dst) {
    pre { robot_ready r1; }
    goal { object_at box dst; }
    plan { move r1 to src; grasp r1 box; move r1 to dst; place r1 box at dst; }
}
""",
    "b_missing_report": """robolang 1.4;
task deliver(robot r, object box, location src, location dst) {
    pre { object_at box src; }
    goal { object_at box dst; }
    plan { move r to src; grasp r box; move r to dst; place r box at dst; }
}
""",
}


@pytest.mark.parametrize("task_id", sorted(HALF))
def test_a_one_branch_answer_is_a_half_program(task_id):
    answer = reference(task_id)
    judgement = judged(task_id, dict(answer, rob=HALF[task_id]))
    assert judgement.state == "SUCCEEDED"  # as given, the world takes the path it was written for
    assert not judgement.goal_correct and not judgement.plan_succeeds and judgement.half_program, judgement.paths
    assert [(p["goal_ok"], p["plan_ok"]) for p in judgement.paths] == [(True, True), (False, False)]
    assert judgement.paths[1]["failure_code"] == "S101"


def test_a_program_without_branches_is_judged_on_every_path():
    """No guard: it runs on both paths, but its goal is wrong on one, so it is a half-program."""
    answer = reference("b_route_by_cell")
    straight = HALF["b_route_by_cell"].replace("    pre { region_clear cell; }\n", "")
    judgement = judged("b_route_by_cell", dict(answer, rob=straight))
    assert judgement.plan_succeeds and not judgement.goal_correct and judgement.half_program
    assert [p["goal_ok"] for p in judgement.paths] == [True, False]


def test_a_branching_answer_with_the_wrong_goal_on_one_branch_is_not_correct():
    answer = reference("b_route_by_cell")
    wrong = answer["rob"].replace("goal { object_at box b; }", "goal { object_at box a; }")
    judgement = judged("b_route_by_cell", dict(answer, rob=wrong))
    assert not judgement.goal_correct and not judgement.plan_succeeds
    assert [p["goal_ok"] for p in judgement.paths] == [True, False]
    assert judgement.paths[1]["failure_code"] == "S301"


def test_a_fail_path_needs_fail_and_its_messages():
    answer = reference("b_missing_report")
    no_message = answer["rob"].replace('            communicate r to "alerts" with "box1 missing";\n', "")
    judgement = judged("b_missing_report", dict(answer, rob=no_message))
    assert judgement.goal_correct and not judgement.plan_succeeds
    assert [(p["goal_ok"], p["plan_ok"]) for p in judgement.paths] == [(True, True), (True, False)]


def test_scores_json_carries_each_path_and_the_half_program_flag(tmp_path):
    task = SET["b_route_by_cell"]
    record = {"task_id": task["id"], "model": "m", "repeat": 1, "calls": [{
        "request": {"model": "m"},
        "response": {"content": [{"type": "text", "text": json.dumps(
            {**{k: v for k, v in reference("b_route_by_cell").items() if k != "bindings"},
             "rob": HALF["b_route_by_cell"], "entities": [],
             "bindings": [{"param": k, "id": v} for k, v in reference("b_route_by_cell")["bindings"].items()]})}],
            "stop_reason": "end_turn"}}]}
    row = score.evaluate_record(task, record)
    assert row["has_paths"] and row["half_program"] and not row["goal_correct"] and not row["plan_succeeds"]
    assert [p["when"] for p in row["paths"]] == ["region_clear cell", "not region_clear cell"]
    assert row["codes"] == []  # valid: nothing is refused any more (B005 is gone)
    text = score.report({"run_id": "r", "effort": "medium", "repeats": 1, "tasks": [task["id"]],
                         "prompt_sha256": "0" * 12, "fallbacks": False, "set": "branch"}, [row])
    assert "half_program" in text


@pytest.mark.parametrize("task_id", sorted(CANNOT))
def test_the_attempt_fails_in_1_3_with_the_recorded_code(task_id):
    task = SET[task_id]
    checked = lib.check_answer(task, attempt(task_id))
    assert not checked.valid and task["attempt_fails_with"] in checked.codes, (task_id, checked.report)


def test_the_inexpressible_attempts_do_not_branch_and_are_1_3():
    for task_id in CANNOT:
        text = (REF / f"{task_id}.attempt.rob").read_text(encoding="utf-8")
        assert "robolang 1.3;" in text and " if " not in text, task_id


def test_the_old_branching_attempts_are_gone_and_the_1_3_label_is_kept():
    assert not list(REF.glob("b_*.attempt.*"))
    for task in BRANCHING.values():
        assert task["attempt_fails_with"] == "E203"  # what the 1.3 attempt hit, kept for history


def test_a_branching_answer_to_a_task_without_paths_is_judged_on_the_path_it_took():
    """Set 1 "if part7 is on the table … otherwise do nothing": a 1.4 answer with branch goals is right (PR #73 run)."""
    task = lib.load_set("1")["w_part_precheck"]
    answer = {"decision": "program", "category": "none", "reason": "", "task": "t",
              "bindings": {"r": "arm2", "part": "part7", "src": "table", "dst": "conveyor"},
              "rob": "robolang 1.4;\n\ntask t(robot r, object part, location src, location dst) {\n    plan {\n"
                     "        if object_at part src {\n            goal { object_at part dst; }\n"
                     "            move r to src;\n            grasp r part;\n            move r to dst;\n"
                     "            place r part at dst;\n        } else {\n            fail \"not on the table\";\n"
                     "        }\n    }\n}\n"}
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds


def test_a_missing_object_path_accepts_either_reading_and_any_alert_text():
    """x_missing_box_branch: `object_exists box` is as right as `object_at box src`; the alert text was not given."""
    task = lib.load_set("hard")["x_missing_box_branch"]
    base = (lib.HARD_REFERENCE_DIR / "x_missing_box_branch.rob").read_text(encoding="utf-8")
    bindings = json.loads((lib.HARD_REFERENCE_DIR / "x_missing_box_branch.json").read_text(encoding="utf-8"))
    for condition in ("object_at box src", "object_exists box"):
        rob = re.sub(r"if [^{]+\{", f"if {condition} {{", base, count=1)
        rob = re.sub(r'with "[^"]*";', 'with "box1 is missing";', rob)
        answer = {"decision": "program", "category": "none", "reason": "", "rob": rob, **bindings}
        checked = lib.check_answer(task, answer)
        assert checked.valid, checked.report
        judgement = lib.judge(task, checked, answer["bindings"])
        assert judgement.goal_correct and judgement.plan_succeeds and not judgement.half_program, judgement.paths


def test_a_null_in_a_world_patch_removes_the_key():
    assert lib._merge({"a": {"b": 1, "c": 2}}, {"a": {"c": None}}) == {"a": {"b": 1}}
