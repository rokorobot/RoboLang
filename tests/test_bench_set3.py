"""
Benchmark set 3 (bench/tasks_set3/, decision record 0025): 80 tasks in three worlds that add the third answer,
"clarify". Every expected answer is checked mechanically where it can be (design docs/plans/bench-2.md, section 5):

- a feasible task's reference is valid, exports its goal, succeeds and passes its checks;
- an infeasible task's faithful attempt fails for the recorded reason, and (unless the task explains why no world
  fix exists) succeeds once the blocker is removed;
- an inexpressible task has a written `why`, and its attempt, where there is one, is rejected by `robolang validate`;
- a clarify task has one succeeding reference per reading, and the readings differ.

The scorer is driven with canned answers for each outcome. No network and no SDK.
"""

from __future__ import annotations
import io
import json
from collections import Counter
from pathlib import Path

import pytest
from test_bench import FakeClient, message

from bench import lib, run, score

SET3 = lib.load_set("3")
REFS = lib.SET3_REFERENCE_DIR
BY_GROUP = {g: sorted(t for t, d in SET3.items() if d["group"] == g)
            for g in ("feasible", "decoy", "infeasible", "inexpressible", "clarify")}


def answer(stem: str, decision="program") -> dict:
    data = json.loads((REFS / f"{stem}.json").read_text(encoding="utf-8"))
    return {"decision": decision, "category": "none", "reason": "r", "entities": [],
            "rob": (REFS / f"{stem}.rob").read_text(encoding="utf-8"), **data}


def other(decision: str, category="none", entities=()) -> dict:
    return {"decision": decision, "category": category, "reason": "?", "entities": list(entities),
            "rob": "", "task": "", "bindings": []}


def run_in(task: dict, ans: dict, patch: dict | None = None):
    data = None if patch is None else json.dumps(lib._merge(json.loads(lib.world_bytes(task)), patch)).encode()
    checked = lib.check_answer(task, ans, data)
    return checked, (lib.simulate_task(checked.task, checked.world) if checked.valid else None)


def test_the_mix_of_tasks():
    assert Counter(d["group"] for d in SET3.values()) == {"feasible": 12, "decoy": 8, "infeasible": 20,
                                                          "inexpressible": 20, "clarify": 20}
    for task in SET3.values():
        assert task["world"] in {"bench/worlds/pharmacy.json", "bench/worlds/yard.json", "bench/worlds/kitchen.json"}
        assert task["prompt"].endswith(".") and task["tags"]
        decision, category = lib.expectation(task)
        assert (decision, category) == {"feasible": ("program", "none"), "decoy": ("program", "none"),
                                        "infeasible": ("cannot", "infeasible"),
                                        "inexpressible": ("cannot", "inexpressible"),
                                        "clarify": ("clarify", "none")}[task["group"]]


@pytest.mark.parametrize("name", ["pharmacy", "yard", "kitchen"])
def test_the_worlds_are_large(name):
    world = json.loads((lib.ROOT / "bench" / "worlds" / f"{name}.json").read_text(encoding="utf-8"))
    assert len(world["locations"]) + sum(len(world[k]) for k in ("regions", "robots", "objects", "sensors")) >= 25


@pytest.mark.parametrize("task_id", BY_GROUP["feasible"] + BY_GROUP["decoy"])
def test_feasible_reference_is_valid_reaches_its_goal_and_passes_its_checks(task_id):
    task = SET3[task_id]
    checked = lib.check_answer(task, answer(task_id))
    assert checked.valid, checked.report
    judgement = lib.judge(task, checked, answer(task_id)["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds and all(judgement.checks.values()), judgement


@pytest.mark.parametrize("task_id", BY_GROUP["infeasible"])
def test_infeasible_attempt_fails_for_its_reason_and_its_fix_succeeds(task_id):
    task = SET3[task_id]
    checked, result = run_in(task, answer(f"{task_id}.attempt"))
    code = result.failure.code if result else checked.codes[0]
    assert code == task["attempt_fails_with"] and task["why"]
    if "fix_patch" in task:
        _, fixed = run_in(task, answer(f"{task_id}.attempt"), task["fix_patch"])
        assert fixed is not None and fixed.state == "SUCCEEDED"
    else:  # one gripper for two objects, or an end state that contradicts itself: no world can fix it
        assert task["attempt_fails_with"] in ("S212", "S301")


@pytest.mark.parametrize("task_id", BY_GROUP["inexpressible"])
def test_inexpressible_has_a_why_and_its_attempt_is_rejected(task_id):
    task = SET3[task_id]
    assert task["why"]
    if "attempt_fails_with" in task:
        checked = lib.check_answer(task, answer(f"{task_id}.attempt"))
        assert not checked.valid and task["attempt_fails_with"] in checked.codes


@pytest.mark.parametrize("task_id", BY_GROUP["clarify"])
def test_every_reading_of_a_clarify_task_succeeds_and_the_readings_differ(task_id):
    task = SET3[task_id]
    stems = sorted(p.stem for p in REFS.glob(f"{task_id}.reading*.json"))
    assert len(stems) >= 2 and task["slot"]
    assert all(c in json.dumps(json.loads(lib.world_bytes(task))) for c in task["candidates"])
    seen = set()
    for stem in stems:
        _, result = run_in(task, answer(stem))
        assert result is not None and result.state == "SUCCEEDED", stem
        ans = answer(stem)
        seen.add((ans["rob"], tuple(sorted(ans["bindings"].items()))))
    assert len(seen) == len(stems)


def test_a_bound_entity_the_request_did_not_name_fails_the_check():
    task = SET3["d2_a_picker"]
    swapped = answer("d2_a_picker")
    swapped["bindings"]["r"] = "picker2"
    checked = lib.check_answer(task, swapped)
    assert lib.judge(task, checked, swapped["bindings"]).checks["entities"] is False


# -------- the scorer --------

def set3_run(tmp_path: Path, replies: dict, repairs: dict | None = None) -> Path:
    def script(request):
        first = request["messages"][0]["content"]
        task_id = next(t for t, d in SET3.items() if first.endswith(f"Request: {d['prompt']}"))
        table = (repairs or {}) if len(request["messages"]) > 1 else replies
        return message(table[task_id], model=request["model"])

    out_dir = tmp_path / "set3"
    run.run_benchmark(FakeClient(script), ["claude-opus-5-5"], {t: SET3[t] for t in replies}, "medium", 1, out_dir,
                      lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="3", prompt_name="full")
    return out_dir


def test_scorer_on_canned_answers(tmp_path):
    replies = {
        "a01_which_vial": other("clarify", entities=["vial_a2", "vial_a1"]),        # right, right candidates
        "a03_which_pan": other("clarify", entities=["pan_1"]),                     # right decision, wrong candidates
        "a13_keep_it_slow": other("clarify"),                                     # right: a value is missing
        "a06_which_crane": answer("a06_which_crane.reading1"),                    # picked one: silent guess
        "i01_picker2_not_ready": other("cannot", "infeasible", ["picker2"]),      # right, names the blocker
        "i04_cart_no_gripper": other("cannot", "infeasible", ["vial_a2"]),        # right decision, wrong blocker
        "i17_placed_and_held": other("clarify", entities=["pan_2"]),              # asked instead of declining
        "u09_avoid_quay": answer("u09_avoid_quay.attempt"),                       # E211, repaired below
        "d1_bowl_in_fridge": other("clarify", entities=["bowl_1", "bowl_2"]),     # asked on a decoy: false refusal
        "f01_straddle_gate": answer("f01_straddle_gate"),                         # right
    }
    without_limit = answer("u09_avoid_quay.attempt")
    without_limit["rob"] = without_limit["rob"].replace("    @safety no_go_region z;\n", "")
    out_dir = set3_run(tmp_path, replies, {"u09_avoid_quay": without_limit})  # the repair dropped the constraint
    _, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert by["a01_which_vial"]["decision_correct"] and by["a01_which_vial"]["entities_correct"] is True
    assert by["a03_which_pan"]["decision_correct"] and by["a03_which_pan"]["entities_correct"] is False
    assert by["a13_keep_it_slow"]["decision_correct"] and by["a13_keep_it_slow"]["entities_correct"] is None
    assert by["a06_which_crane"]["silent_guess"] and not by["a06_which_crane"]["decision_correct"]
    assert by["i01_picker2_not_ready"]["entities_correct"] is True
    assert by["i04_cart_no_gripper"]["entities_correct"] is False
    assert not by["i17_placed_and_held"]["decision_correct"] and not by["i17_placed_and_held"]["false_program"]
    u09 = by["u09_avoid_quay"]
    assert u09["false_program"] and u09["false_program_final"] and u09["false_program_undetected"]
    assert by["d1_bowl_in_fridge"]["false_refusal"]
    assert by["f01_straddle_gate"]["decision_correct"] and by["f01_straddle_gate"]["goal_correct"]
    text = score.score_dir(out_dir)
    assert "silent_guess" in text and "entities_correct" in text and "| clarify |" in text


def test_set_3_is_a_choice_of_the_runner(tmp_path):
    assert run.parse_args(["--set", "3", "--max-usd", "5"]).max_usd == 5
    out = io.StringIO()
    assert run.main(["--set", "3", "--out", str(tmp_path)], out=out, err=io.StringIO(), environ={}) == 2
    assert "Nothing was sent" in out.getvalue() and not any(tmp_path.iterdir())


def test_the_cap_stops_new_requests(tmp_path):
    client = FakeClient(lambda request: message(other("clarify"), model=request["model"]))
    run.run_benchmark(client, ["claude-opus-5-5"], {t: SET3[t] for t in BY_GROUP["clarify"][:3]}, "medium", 1,
                      tmp_path / "r", lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="3",
                      max_usd=0.000001)
    assert len(client.requests) == 1
