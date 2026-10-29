"""
Benchmark set 3, "branch" (bench/tasks_branch/, decision record 0027 section 3): pre-registered against RoboLang 1.3.

20 branching requests that 1.3 cannot express (a decline is right today; each has its 1.4 program and the goal of
every path in `expect_1_4`), 8 requests that stay inexpressible in 1.4, and 8 conditional-sounding requests that
1.3 already expresses with `pre` or `wait until`. No network and no SDK.
"""

from __future__ import annotations
import json
import re
from collections import Counter
from pathlib import Path

import pytest

from bench import lib

REPO = Path(__file__).resolve().parent.parent
SET = lib.load_set("branch")
REF = lib.BRANCH_REFERENCE_DIR
FEASIBLE = {t: d for t, d in SET.items() if lib.expectation(d)[0] == "program"}
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


def test_expectations_are_well_formed():
    for task_id, task in SET.items():
        decision, category = lib.expectation(task)
        later = task["expect_1_4"]
        if task["group"] == "control":
            assert (decision, category) == ("program", "none") and later == {"decision": "program"}
            assert lib.expected_goals(task) and "why" not in task
            continue
        assert (decision, category) == ("cannot", "inexpressible") and len(task["why"]) > 40
        if task["group"] == "inexpressible":
            assert later == {"decision": "cannot", "category": "inexpressible"}
            continue
        assert later["decision"] == "program" and len(later["paths"]) >= 2, task_id
        for path in later["paths"]:
            assert path["when"] and ("goal" in path) != ("fail" in path), task_id
            for triple in path.get("goal", []):
                assert len(triple) == 3 and triple[1] in {"located_in", "carried_by"} and TRIPLE.match(triple[0])
            for channel, message in path.get("messages", []):
                assert f'"{channel}"' in task["prompt"] and f'"{message}"' in task["prompt"], task_id


@pytest.mark.parametrize("task_id", sorted(FEASIBLE))
def test_control_reference_is_valid_reaches_the_goal_and_its_probes_bite(task_id):
    task = SET[task_id]
    answer = reference(task_id)
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert all(judgement.checks.values()), judgement.checks


@pytest.mark.parametrize("task_id", sorted(CANNOT))
def test_the_attempt_fails_in_1_3_with_the_recorded_code(task_id):
    task = SET[task_id]
    checked = lib.check_answer(task, attempt(task_id))
    assert not checked.valid and task["attempt_fails_with"] in checked.codes, (task_id, checked.report)


def test_every_branching_attempt_is_the_1_4_program():
    """The attempt is the program 1.4 should accept, under the 1.3 marker: it becomes the reference when 1.4 lands."""
    for task_id, task in CANNOT.items():
        text = (REF / f"{task_id}.attempt.rob").read_text(encoding="utf-8")
        if task["group"] == "branching":
            assert "robolang 1.3;" in text and " if " in text and (REF / f"{task_id}.attempt.json").is_file()
            fails = [p for p in task["expect_1_4"]["paths"] if "fail" in p]
            assert all(f'fail "{p["fail"]}";' in text for p in fails), task_id
