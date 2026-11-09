"""
Benchmark set 4, "failure" (bench/tasks_failure/, decision record 0035): pre-registered against RoboLang 1.4,
labelled for RoboLang 1.5 in 0035 PR 2 (E2).

8 `recover` requests that need a failure handler: a program in 1.5 (`expect`; `expect_1_5` holds the paths that force
each failure through the world: E4), inexpressible in 1.4 (`expect_1_4` keeps that label). 4 `inexpressible` requests
that stay inexpressible in 1.5. 4 `control` requests that both versions express without a handler.
No network and no SDK.
"""

from __future__ import annotations
import json
from collections import Counter
from pathlib import Path

import pytest

from bench import lib, score

REPO = Path(__file__).resolve().parent.parent
SET = lib.load_set("failure")
REF = lib.FAILURE_REFERENCE_DIR
GROUPS = {"recover": 8, "inexpressible": 4, "control": 4}
CANNOT = {"decision": "cannot", "category": "inexpressible"}


def answer(rob: str, task: str = "t", bindings: dict | None = None) -> dict:
    return {"decision": "program", "category": "none", "reason": "-", "rob": rob, "task": task,
            "bindings": bindings or {}}


def reference(task_id: str) -> dict:
    ref = json.loads((REF / f"{task_id}.json").read_text(encoding="utf-8"))
    return answer((REF / f"{task_id}.rob").read_text(encoding="utf-8"), ref["task"], ref["bindings"])


def judged(task_id: str, given: dict):
    checked = lib.check_answer(SET[task_id], given)
    assert checked.valid, checked.report
    return lib.judge(SET[task_id], checked, given["bindings"])


def test_the_mix_of_tasks():
    assert len(SET) == 16 and Counter(d["group"] for d in SET.values()) == GROUPS
    for task_id, task in SET.items():
        assert task["id"] == task_id and (REPO / task["world"]).is_file() and task["tags"]
        assert task["prompt"].endswith(".") and "expect_1_5" in task


def test_expectations_are_labelled_for_1_5():
    for task_id, task in SET.items():
        later = task["expect_1_5"]
        if task["group"] == "recover":
            assert task["expect"] == {"decision": "program"} and task["expect_1_4"] == CANNOT
            assert later["decision"] == "program" and len(later["paths"]) >= 2
            assert task["attempt_fails_with"] == "E202" and len(task["why"]) > 40
        elif task["group"] == "inexpressible":
            assert task["expect"] == later == CANNOT and len(task["why"]) > 40
        else:
            assert task["expect"] == {"decision": "program"} and "why" not in task and len(later["paths"]) >= 2
        for path in later.get("paths", []):
            assert path["when"] and sum(k in path for k in ("goal", "fail", "fails")) == 1, task_id
            assert "events" in path["world_patch"], task_id
            for channel, message in path.get("messages", []):
                assert f'"{channel}"' in task["prompt"] and f'"{message}"' in task["prompt"], task_id


def test_files_are_where_they_should_be():
    names = {p.name for p in REF.iterdir()}
    programs = {t for t, d in SET.items() if d["group"] != "inexpressible"}
    attempts = {t for t, d in SET.items() if d["group"] != "control"}
    assert {n.split(".")[0] for n in names if ".attempt." in n} == attempts
    assert {n.split(".")[0] for n in names if ".attempt." not in n} == programs
    for task_id in programs:
        assert "robolang 1.5;" in (REF / f"{task_id}.rob").read_text(encoding="utf-8")


@pytest.mark.parametrize("task_id", sorted(t for t, d in SET.items() if d["group"] != "inexpressible"))
def test_the_1_5_reference_is_right_on_every_path(task_id):
    judgement = judged(task_id, reference(task_id))
    assert judgement.goal_correct and judgement.plan_succeeds, judgement.paths
    assert [p["when"] for p in judgement.paths] == [p["when"] for p in SET[task_id]["expect_1_5"]["paths"]]
    assert judgement.unsafe_continue in (None, False)


@pytest.mark.parametrize("task_id", sorted(t for t, d in SET.items() if d["group"] != "control"))
def test_the_attempt_fails_with_its_code(task_id):
    checked = lib.check_answer(SET[task_id], answer((REF / f"{task_id}.attempt.rob").read_text(encoding="utf-8")))
    assert not checked.valid and checked.codes[0] == SET[task_id]["attempt_fails_with"]


# What 1.4 can write instead (E4): an `if` after a full `wait for`, or one grasp. Each is valid and wrong on a path.
APPROXIMATIONS = {
    "f01_retry_grasp_once": """robolang 1.4;
task deliver(robot r, object box, location src, location dst) {
    goal { object_at box dst; }
    plan { move r to src; grasp r box; move r to dst; place r box at dst; }
}
""",
    "f03_wait_then_report": """robolang 1.4;
task deliver(robot r, object box, location src, location dst, region aisle) {
    goal { object_at box dst; }
    plan {
        wait for 10 s;
        if region_clear aisle {
            move r to src; grasp r box; move r to dst; place r box at dst;
        } else {
            communicate r to "ops" with "aisle blocked";
            fail "aisle blocked";
        }
    }
}
""",
}


@pytest.mark.parametrize("task_id", sorted(APPROXIMATIONS))
def test_a_1_4_approximation_is_wrong_on_some_path(task_id):
    ref = reference(task_id)
    judgement = judged(task_id, answer(APPROXIMATIONS[task_id], ref["task"], ref["bindings"]))
    assert not (judgement.goal_correct and judgement.plan_succeeds)
    assert any(p["goal_ok"] and not p["plan_ok"] for p in judgement.paths), judgement.paths  # timing or tries


def test_unsafe_continue_catches_a_handler_that_goes_on():
    ref = reference("f20_wait_or_fail")
    going_on = ref["rob"].replace('fail "the aisle did not clear";', 'communicate r to "ops" with "late";')
    assert judged("f20_wait_or_fail", answer(going_on, ref["task"], ref["bindings"])).unsafe_continue is True
    assert judged("f20_wait_or_fail", ref).unsafe_continue is False
    plain = ref["rob"].replace("robolang 1.5;", "robolang 1.4;").replace(
        ' else {\n            fail "the aisle did not clear";\n        }', ";")
    assert judged("f20_wait_or_fail", answer(plain, ref["task"], ref["bindings"])).unsafe_continue is False
    assert judged("f06_skip_inspection", reference("f06_skip_inspection")).unsafe_continue is None  # continue_ok


def test_the_scorer_reports_unsafe_continue():
    rows = [{"expect": "program", "unsafe_continue": value, "half_program": False, "has_paths": False,
             "checks_ok": None, "goal_exact": False, **{m: True for m in score.MEASURES}}
            for value in (True, False, None)]
    assert any(line.startswith("| unsafe_continue") and "1/2" in line for line in score._rates_table(rows))


def test_x_failover_on_fault_is_labelled_for_1_5():
    branch = lib.load_set("branch")["x_failover_on_fault"]
    assert branch["expect"] == {"decision": "program"} and branch["expect_1_4"] == CANNOT
    assert branch["group"] == "recover" and branch["expect_1_5"]["decision"] == "program"
    rob = """robolang 1.5;
task move_part(robot a, robot b, object p, location src, location dst) {
    goal { object_at p dst; }
    plan {
        move a to src else { move b to src; }
        if robot_at a src { grasp a p; move a to dst; place a p at dst; }
        else { grasp b p; move b to dst; place b p at dst; }
    }
}
"""
    bindings = {"a": "arm_a", "b": "arm_b", "p": "p1", "src": "conveyor_in", "dst": "station1"}
    checked = lib.check_answer(branch, answer(rob, "move_part", bindings))
    assert checked.valid, checked.report
    judgement = lib.judge(branch, checked, bindings)
    assert judgement.goal_correct and judgement.plan_succeeds, judgement.paths
