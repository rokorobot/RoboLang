"""
Benchmark set 5, "values" (bench/tasks_values/, decision record 0036, plan docs/plans/bench-5-values.md): measured
values, pre-registered against RoboLang 1.5. Labels approved by the owner on 2026-11-09.

15 `value` requests that need a reading: inexpressible in 1.5 (`expect`), a program in 1.6 (`expect_1_6`). 6
`inexpressible` requests that stay inexpressible in 1.6. 3 `infeasible` requests: inexpressible in 1.5, infeasible in
1.6. 2 `control` requests that need no reading. The 1.6 paths are written as `planned_paths` and become world patches
once world file 1.2 exists. No network and no SDK.
"""

from __future__ import annotations
import json
from collections import Counter
from pathlib import Path

import pytest

from bench import lib

REPO = Path(__file__).resolve().parent.parent
SET = lib.load_set("values")
REF = lib.VALUES_REFERENCE_DIR
CANNOT = {"decision": "cannot", "category": "inexpressible"}


def test_the_mix_of_tasks():
    assert len(SET) == 26
    assert Counter(d["group"] for d in SET.values()) == {"value": 15, "inexpressible": 6, "infeasible": 3, "control": 2}
    for task_id, task in SET.items():
        assert task["id"] == task_id and (REPO / task["world"]).is_file() and task["tags"]
        assert task["prompt"].endswith(".") and "expect_1_6" in task


def test_expectations_are_pre_registered_against_1_5():
    for task_id, task in SET.items():
        later = task["expect_1_6"]
        if task["group"] == "control":
            assert task["expect"] == later == {"decision": "program"} and lib.expected_goals(task)
            continue
        assert task["expect"] == CANNOT and len(task["why"]) > 40, task_id
        if task["group"] == "value":
            assert later["decision"] == "program" and later["planned_paths"], task_id
        elif task["group"] == "inexpressible":
            assert later == CANNOT
        else:
            assert later == {"decision": "cannot", "category": "infeasible"} and task["entities_1_6"]


def test_the_world_variant_differs_only_where_it_says():
    base = json.loads((REPO / "bench/worlds/kitchen.json").read_text(encoding="utf-8"))
    variant = json.loads((REPO / "bench/worlds/kitchen_values.json").read_text(encoding="utf-8"))
    assert base["objects"]["knife_1"]["held_by"] == "chef_arm" and not base["sensors"]["scale_1"]["available"]
    assert variant["objects"]["knife_1"] == {"present": True, "location": "prep_1", "class": "knife"}
    assert variant["sensors"]["scale_1"]["available"] is True
    for key in ("objects", "sensors"):
        base[key], variant[key] = dict(base[key]), dict(variant[key])
    base["objects"].pop("knife_1"), variant["objects"].pop("knife_1")
    base["sensors"].pop("scale_1"), variant["sensors"].pop("scale_1")
    assert base == variant
    assert SET["i01_scale_offline"]["world"] == "bench/worlds/kitchen.json"


@pytest.mark.parametrize("task_id", sorted(t for t, d in SET.items() if d["group"] == "control"))
def test_the_control_references_validate_and_reach_the_goal(task_id):
    ref = json.loads((REF / f"{task_id}.json").read_text(encoding="utf-8"))
    rob = (REF / f"{task_id}.rob").read_text(encoding="utf-8")
    assert "robolang 1.5;" in rob and "measure" not in rob.split("\n", 1)[1]  # line 1 is the file comment
    answer = {"decision": "program", "category": "none", "reason": "-", "rob": rob, **ref}
    checked = lib.check_answer(SET[task_id], answer)
    assert checked.valid, checked.report
    judgement = lib.judge(SET[task_id], checked, ref["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds


def test_reference_files_exist_only_for_programs():
    assert {p.name.split(".")[0] for p in REF.iterdir()} == {t for t, d in SET.items() if d["group"] == "control"}
