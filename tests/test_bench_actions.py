"""
Benchmark set 7, "actions" (bench/tasks_actions/, decision record 0040, plan docs/plans/bench-7-actions.md): declared
actions (`do r ACTION TARGET [to VALUE];`, `in_state X "label"`), pre-registered against RoboLang 1.7 before any code
(L3 of iteration I2). `expect` is the 1.7 label (the baseline), `expect_1_8` the label for acceptance.

10 `action` requests that become programs in 1.8, 5 `infeasible` in 1.8 (0040 A6, A7), 5 `inexpressible` that stay out
(0040 A4, A10, A11; G-PROX, G-TEACH, G-LOOP, G-CLOCK), 4 `control` requests that are programs in both versions. 1.7
declines every non-control task, and every one has an obvious attempt that 1.7 rejects with `E203` (`do` is no
statement). Only the controls have a 1.7 reference; the 1.8 references wait for L4. No network and no SDK; the model
run is the `Benchmark` workflow with `actions: true`.
"""

from __future__ import annotations
import io
import json
import re
from collections import Counter
from pathlib import Path

import pytest
from test_bench import FakeClient, message

from bench import lib, run, score

REPO = Path(__file__).resolve().parent.parent
SET = lib.load_set("actions")
REF = lib.ACTIONS_REFERENCE_DIR
ATLAS = json.loads((REPO / "bench" / "atlas.json").read_text(encoding="utf-8"))
GAPS = {g["id"]: g for g in ATLAS["gaps"]}
CANNOT = {"decision": "cannot", "category": "inexpressible"}
ACTION = {t: d for t, d in SET.items() if d["group"] == "action"}
INFEASIBLE = {t: d for t, d in SET.items() if d["group"] == "infeasible"}
INEXPRESSIBLE = {t: d for t, d in SET.items() if d["group"] == "inexpressible"}
CONTROLS = {t: d for t, d in SET.items() if d["group"] == "control"}
DECLINED = {**ACTION, **INFEASIBLE, **INEXPRESSIBLE}  # what 1.7 must decline: an attempt each, a gap each
DO_LINE = re.compile(r"^\s*do [^\n]*\n", re.M)  # an attempt keeps each `do` statement on one line
WORLDS_1_2 = {"ward", "fry_station", "site", "kitchen_values", "yard", "pharmacy", "site_1_3"}


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def reference(task_id: str) -> dict:
    ref = json.loads(read(REF / f"{task_id}.json"))
    rob = read(REF / f"{task_id}.rob")
    return {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob, **ref}


def attempt_answer(task_id: str, strip: bool = False) -> dict:
    """The obvious attempt as a program answer; `strip` removes its `do` (the half program a model may write in 1.7)."""
    stem = REF / f"{task_id}.attempt"
    rob = read(Path(f"{stem}.rob"))
    if strip:
        rob = DO_LINE.sub("", rob)
    return {"decision": "program", "category": "none", "reason": "an attempt", "entities": [], "rob": rob,
            **json.loads(read(Path(f"{stem}.json")))}


def decline(category: str = "inexpressible") -> dict:
    return {"decision": "cannot", "category": category, "reason": "cannot be done", "rob": "", "task": "",
            "bindings": []}


# -------- the set --------

def test_the_mix_of_tasks():
    assert len(SET) == 24
    groups = Counter(d["group"] for d in SET.values())
    assert groups == {"action": 10, "infeasible": 5, "inexpressible": 5, "control": 4}
    for task_id, task in SET.items():
        assert task["id"] == task_id and (lib.ACTIONS_TASKS_DIR / f"{task_id}.json").is_file()
        assert (REPO / task["world"]).is_file() and task["tags"] and task["prompt"].endswith(".")
        assert task["prompt"].isascii() and "\r" not in read(lib.ACTIONS_TASKS_DIR / f"{task_id}.json")
    prefixes = {"action": "a", "infeasible": "i", "inexpressible": "x", "control": "c"}
    for task_id, task in SET.items():
        assert re.fullmatch(prefixes[task["group"]] + r"\d\d_[a-z0-9_]+", task_id), task_id


def test_the_ids_are_new_ones_and_no_prompt_is_a_copy_of_a_standing_task():
    elsewhere = {}
    for name in lib.SETS:
        if name != "actions":
            elsewhere.update(lib.load_set(name))
    assert not set(SET) & set(elsewhere)
    assert not {t["prompt"] for t in SET.values()} & {t["prompt"] for t in elsewhere.values()}


def test_labels_are_1_7_in_expect_and_1_8_in_expect_1_8():
    for task_id, task in SET.items():
        assert task["expect_1_8"]["decision"] in ("program", "cannot"), task_id
        if task["group"] == "control":
            assert task["expect"] == {"decision": "program"} and lib.expected_goals(task), task_id
            assert task["expect_1_8"] == {"decision": "program"} and task["expected_actions"] == []
            assert not {"why", "attempt_fails_with", "gap", "blockers"} & set(task)
            continue
        assert task["expect"] == CANNOT and lib.expectation(task) == ("cannot", "inexpressible"), task_id
        assert len(task["why"]) > 40 and task["attempt_fails_with"] == "E203", task_id
        assert not {"expected_goal", "expected_goals", "expect_1_7", "expect_1_6"} & set(task)
        assert "world_1_4" in task["expect_1_8"], task_id
    for task_id, task in ACTION.items():
        later = task["expect_1_8"]
        assert later["decision"] == "program" and later["paths"] and "category" not in later
        for path in later["paths"]:
            assert path["when"] and ("goal" in path or "goals" in path or path.get("fails")) and "world_patch" in path
        actions = task["expected_actions"]
        assert 1 <= len(actions) <= 2 and all({"robot", "action", "target"} <= set(a) for a in actions), task_id
        assert all(set(a) <= {"robot", "action", "target", "value"} for a in actions)
    for task_id, task in INFEASIBLE.items():
        later = task["expect_1_8"]
        assert later["decision"] == "cannot" and later["category"] == "infeasible", task_id
        assert later["because"] in ("0040 A6", "0040 A7") and task["blockers"], task_id
    for task_id, task in INEXPRESSIBLE.items():
        later = task["expect_1_8"]
        assert {k: later[k] for k in ("decision", "category")} == CANNOT, task_id
        assert later["because"] in ("0040 A4", "0040 A10", "0040 A11"), task_id
    reasons = sorted(t["expect_1_8"]["because"] for t in INFEASIBLE.values())
    assert reasons == ["0040 A6"] * 4 + ["0040 A7"]


def test_the_action_tasks_cover_the_cases_the_pre_registration_lists():
    tags = Counter(tag for t in ACTION.values() for tag in t["tags"])
    for tag in ("open", "fetch", "set_value", "wait_until", "location_target", "if", "handler", "two_robots",
                "region_limit", "dock", "charge", "decoy_names", "near_miss_action"):
        assert tags[tag] >= 1, tag
    values = [a["value"] for t in ACTION.values() for a in t["expected_actions"] if "value" in a]
    assert values == [["175", "degC"]]
    assert len({t["world"] for t in ACTION.values()}) >= 5
    paths = [len(t["expect_1_8"]["paths"]) for _, t in sorted(ACTION.items())]
    assert paths.count(2) == 3 and paths.count(1) == 7  # a02 (the oil never heats), a04 (if), a05 (a handler runs)
    # the decoy: the profile declares `open_door` and the world holds a door object next to a closet location
    decoy = SET["a10_decoy_open_closet"]
    assert decoy["expected_actions"][0]["action"] == "open_door"
    assert decoy["expect_1_8"]["world_1_4"]["robots"]["moxi1"]["actions"].keys() == {"open_door"}
    ward = json.loads(read(REPO / decoy["world"]))
    assert "supply_closet" in ward["locations"] and ward["objects"]["closet_door"]["class"] == "door"
    # the region limit is a 1.7 one: the world copy that lists what printer1 honours
    limited = SET["a07_print_dry"]
    assert limited["expected_limits"] == [["no_go_region", "wet_floor"]]
    assert json.loads(read(REPO / limited["world"]))["world_schema_version"] == "1.3"


def test_the_infeasible_tasks_each_break_one_check_of_0040_a6_and_a7():
    kinds = {t: set(d["tags"]) for t, d in INFEASIBLE.items()}
    assert sorted(k for tags in kinds.values() for k in tags if k not in ("open", "lock", "set_value")) == [
        "result_not_the_state", "robot_ability", "value_for_no_value_action", "wrong_target_kind", "wrong_value_kind"]
    runner = SET["i01_runner_cannot_open"]["expect_1_8"]["world_1_4"]["robots"]
    assert runner["runner"]["actions"] == {} and runner["chef_arm"]["actions"]["open"]["target"] == "object"
    lock = SET["i02_lock_a_berth"]["expect_1_8"]["world_1_4"]["robots"]["crane1"]["actions"]["lock"]
    assert lock["target"] == "object"
    set_action = SET["i03_fryer_in_kilograms"]["expect_1_8"]["world_1_4"]["robots"]["flippy1"]["actions"]["set"]
    assert set_action["value"] == "temperature" and " 2 kg" in SET["i03_fryer_in_kilograms"]["prompt"]
    assert "value" not in SET["i04_open_by_distance"]["expect_1_8"]["world_1_4"]["robots"]["moxi1"]["actions"]["open"]
    locked = SET["i05_leave_it_locked"]["expect_1_8"]["world_1_4"]["robots"]["moxi1"]["actions"]
    assert "locked" not in {a["result"] for a in locked.values()}


def test_no_world_1_4_file_exists_yet_and_no_world_file_is_new():
    """1.4 facts (`actions`, `state`) are recorded in the labels only: L4 writes the world copies."""
    worlds = {Path(t["world"]).stem for t in SET.values()}
    assert worlds <= WORLDS_1_2
    for name in worlds:
        text = read(REPO / f"bench/worlds/{name}.json")
        assert '"actions"' not in text and '"state"' not in text
        assert json.loads(text)["world_schema_version"] in ("1.1", "1.2", "1.3")
    for task_id, task in SET.items():
        assert "world_1_4" not in task
        has_facts = "world_1_4" in task["expect_1_8"]
        assert has_facts == (task["group"] != "control"), task_id
    for task_id, task in DECLINED.items():
        facts = task["expect_1_8"]["world_1_4"]
        assert set(facts) <= {"robots", "object_states", "location_states"}, task_id
        world = json.loads(read(REPO / task["world"]))
        assert set(facts["robots"]) <= set(world["robots"]), task_id
        assert set(facts.get("object_states", {})) <= set(world["objects"]), task_id
        assert set(facts.get("location_states", {})) <= set(world["locations"]), task_id
        for profile in facts["robots"].values():
            for declared in profile["actions"].values():
                assert declared["target"] in ("object", "location") and declared["result"], task_id
                assert declared.get("value", "temperature") in ("temperature", "mass", "distance", "label")
    for task_id, task in ACTION.items():  # the action asked is declared by the robot asked, on the declared target kind
        facts = task["expect_1_8"]["world_1_4"]
        world = json.loads(read(REPO / task["world"]))
        for wanted in task["expected_actions"]:
            declared = facts["robots"][wanted["robot"]]["actions"][wanted["action"]]
            kind_of = "object" if wanted["target"] in world["objects"] else "location"
            assert wanted["target"] in world["objects"] or wanted["target"] in world["locations"], task_id
            assert declared["target"] == kind_of, task_id
            assert ("value" in wanted) == ("value" in declared), task_id


# -------- the references and the attempts --------

def test_references_and_attempts_are_where_they_should_be():
    names = [p.name for p in REF.iterdir()]
    refs = {n.split(".")[0] for n in names if ".attempt" not in n}
    attempts = {n.split(".")[0] for n in names if ".attempt." in n}
    assert refs == set(CONTROLS) and attempts == set(DECLINED)  # no 1.8 reference yet: that is L4
    for task_id in DECLINED:
        assert (REF / f"{task_id}.attempt.rob").is_file() and (REF / f"{task_id}.attempt.json").is_file()
    for name in names:
        assert "\r" not in read(REF / name)  # Windows-safe: LF only, never compared as CRLF


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_the_obvious_attempt_fails_with_the_recorded_code(task_id):
    task = DECLINED[task_id]
    answer = attempt_answer(task_id)
    assert answer["rob"].startswith(f"// bench/reference_actions/{task_id}.attempt.rob: the obvious attempt")
    assert answer["rob"].count("robolang 1.7;") == 1 and "\r" not in answer["rob"]
    checked = lib.check_answer(task, answer)
    assert not checked.valid and checked.codes[0] == task["attempt_fails_with"], (task_id, checked.codes)


def test_each_attempt_uses_the_one_construct_the_request_needs():
    for task_id in DECLINED:
        rob = read(REF / f"{task_id}.attempt.rob")
        assert len(DO_LINE.findall(rob)) >= 1 and "in_state" not in rob  # `do` is the one thing 1.7 cannot parse
    assert read(REF / "a02_set_then_wait_reading.attempt.rob").count("do r set fryer to 175 degC;") == 1
    assert "do r open door to 30 cm;" in read(REF / "i04_open_by_distance.attempt.rob")
    assert "do r set fryer to 2 kg;" in read(REF / "i03_fryer_in_kilograms.attempt.rob")
    assert "do r hand kit;" in read(REF / "x01_hand_to_ana.attempt.rob")  # the second target (Ana) is dropped
    assert "@safety no_go_region wet;" in read(REF / "a07_print_dry.attempt.rob")  # the 1.7 limit is kept
    assert "else { communicate" in read(REF / "a05_open_or_report.attempt.rob")  # the handler is on the `do`
    assert len(DO_LINE.findall(read(REF / "a06_two_cranes_lock.attempt.rob"))) == 2


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_no_probe_is_a_bug_in_disguise(task_id):
    """An attempt fails for its `do`: without it, it is a valid 1.7 program in the task's world that ends SUCCEEDED.
    That program is the silent replacement a model may write (a `move`, a `communicate` or nothing instead of the
    action), and it is what the baseline counts as a false program (the plan's B8)."""
    answer = attempt_answer(task_id, strip=True)
    assert "do " not in answer["rob"].split("\n", 1)[1]
    checked = lib.check_answer(DECLINED[task_id], answer)
    assert checked.valid, (task_id, checked.report)
    assert checked.language_version == "1.7"
    judgement = lib.judge(dict(DECLINED[task_id], goal_free=True), checked, answer["bindings"])
    assert judgement.plan_succeeds, (task_id, judgement.state, judgement.failure_code)


def test_the_stripped_attempt_of_a_branching_task_follows_the_path_the_world_forces():
    """a04: `pass_area` clear, the branch with the action runs (as the half program, it succeeds); not clear, the
    other branch sends its message and stops, exactly as the 1.8 label says."""
    task = SET["a04_open_if_clear"]
    answer = attempt_answer("a04_open_if_clear", strip=True)
    checked = lib.check_answer(task, answer)
    paths = [dict(p, goal=[], world_patch=p["world_patch"]) for p in task["expect_1_8"]["paths"]]
    labelled = dict(task, goal_free=True, expect_1_7={"decision": "program", "paths": paths})
    judged = lib.judge_paths(labelled, checked, answer["bindings"])
    assert [(p["state"], p["goal_ok"] and p["plan_ok"]) for p in judged] == [("SUCCEEDED", True), ("FAILED", True)]


def test_the_attempt_binds_the_entities_the_request_names():
    for task_id, task in ACTION.items():
        bound = set(json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"].values())
        for wanted in task["expected_actions"]:
            assert {wanted["robot"], wanted["target"]} <= bound, task_id
    for task_id in INFEASIBLE:
        assert json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"]


# -------- the controls --------

@pytest.mark.parametrize("task_id", sorted(CONTROLS))
def test_the_control_reference_is_valid_exports_an_accepted_goal_and_succeeds_on_every_path(task_id):
    task = CONTROLS[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 1.7;") == 1 and answer["rob"].isascii()
    assert not re.search(r"\bdo\b|in_state", answer["rob"].split("\n", 1)[1])
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == "1.7"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert not judgement.half_program
    paths = lib.task_paths(task) or []
    assert [p["when"] for p in judgement.paths or []] == [p["when"] for p in paths]
    assert all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths or []), judgement.paths
    if "expected_readings" in task:
        assert judgement.value_ok


def test_the_controls_are_a_move_a_fetch_a_measure_and_a_wait_and_want_no_action():
    assert all(t["expected_actions"] == [] for t in CONTROLS.values())
    assert "measure r temperature of oil using probe as t;" in read(REF / "c02_oil_reading.rob")
    waits = read(REF / "c03_wait_oil_by_itself.rob")
    assert "wait until temperature of oil using probe > 170 degC timeout 60 s" in waits
    assert [bool(p.get("fails")) for p in lib.task_paths(CONTROLS["c03_wait_oil_by_itself"])] == [False, True]
    assert all(lib.task_paths(CONTROLS[t]) is None for t in ("c01_fetch_no_door", "c04_plain_move_printer"))
    # the fetch control is a task whose world holds a closed door: a model that adds `do r open` fails the control
    door = SET["a01_open_then_fetch"]["expect_1_8"]["world_1_4"]["object_states"]["closet_door"]
    assert door == "closed" and CONTROLS["c01_fetch_no_door"]["world"] == SET["a01_open_then_fetch"]["world"]


# -------- the atlas --------

def test_every_task_1_7_declines_is_in_exactly_one_atlas_gap_and_no_control_is():
    in_gaps = [(t, g["id"]) for g in ATLAS["gaps"] for t in g["tasks"] if t in SET]
    assert len(in_gaps) == len({t for t, _ in in_gaps}) == len(DECLINED)
    assert {t for t, _ in in_gaps} == set(DECLINED)
    for task_id, gap in in_gaps:
        assert DECLINED[task_id]["gap"] == gap and GAPS[gap]["scope"] == "open"
    assert Counter(gap for _, gap in in_gaps) == {"G-ACT": 16, "G-PROX": 1, "G-TEACH": 1, "G-LOOP": 1, "G-CLOCK": 1}
    assert {t["gap"] for t in INEXPRESSIBLE.values()} == {"G-ACT", "G-PROX", "G-TEACH", "G-LOOP", "G-CLOCK"}
    assert {t["gap"] for t in ACTION.values()} | {t["gap"] for t in INFEASIBLE.values()} == {"G-ACT"}


# -------- the scorer and the workflow --------

def actions_run(tmp_path: Path, replies: dict) -> Path:
    def script(request):
        first = request["messages"][0]["content"]
        task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
        return message(replies[task_id], model=request["model"])

    out_dir = tmp_path / "actions"
    run.run_benchmark(FakeClient(script), ["claude-sonnet-5-5"], {t: SET[t] for t in replies}, "medium", 1, out_dir,
                      lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="actions", prompt_name="full")
    return out_dir


def test_the_scorer_counts_a_silent_replacement_as_false_and_a_decline_of_a_control_as_a_refusal(tmp_path):
    replies = {
        "a01_open_then_fetch": decline(),                                   # right under 1.7
        "i01_runner_cannot_open": decline("infeasible"),                    # declined, category differs (reported only)
        "a03_print_gridline": attempt_answer("a03_print_gridline", strip=True),  # a move for a print: a false program
        "c04_plain_move_printer": reference("c04_plain_move_printer"),      # right
        "c01_fetch_no_door": decline(),                                     # false refusal of a control
    }
    out_dir = actions_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "actions" and meta["prompt"] == "full"
    assert by["a01_open_then_fetch"]["decision_correct"] and by["a01_open_then_fetch"]["category_correct"]
    declined = by["i01_runner_cannot_open"]
    assert declined["decision_correct"] and declined["category_correct"] is False
    assert by["a03_print_gridline"]["false_program"] and not by["a03_print_gridline"]["decision_correct"]
    assert by["c04_plain_move_printer"]["decision_correct"] and by["c04_plain_move_printer"]["goal_correct"]
    assert by["c01_fetch_no_door"]["false_refusal"] and not by["c01_fetch_no_door"]["decision_correct"]
    text = score.score_dir(out_dir)
    assert "Task set actions, prompt full." in text
    kinds = text.split("By kind of task:")[1]
    for kind in ("action", "infeasible", "control"):
        assert f"| {kind} |" in kinds


def test_the_set_is_in_the_run_script_and_only_runs_on_demand_in_the_workflow():
    assert lib.SETS["actions"] == (lib.ACTIONS_TASKS_DIR, lib.ACTIONS_REFERENCE_DIR)
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    assert workflow.count("--set actions") == 2
    lines = [ln for ln in workflow.splitlines() if "--set actions" in ln]
    for part in ("--repeats 3", "--models claude-sonnet-5-5,claude-haiku-5-5", "--max-usd 3"):
        assert all(part in ln for ln in lines), part
    assert any("--prompt full" in ln and "--run-id actions" in ln for ln in lines)
    assert any("--prompt minimal" in ln and "--run-id actions-minimal" in ln for ln in lines)
    assert all("claude-opus" not in ln and "--set actions --prompt" in ln for ln in lines)
    steps = re.split(r"\n      - ", workflow)
    step = next(s for s in steps if s.startswith("name: Run (set actions"))
    assert "inputs.actions" in step and "pull_request" not in step
    pr_step = next(s for s in steps if s.startswith("name: Run (pull request)"))
    weekly = next(s for s in steps if s.startswith("name: Run (weekly or on demand)"))
    assert "actions" not in pr_step and "!inputs.actions" in weekly and "--set actions" not in weekly
    declared = r"      actions:\n        description: .*\n        type: boolean\n        default: false"
    assert re.search(declared, workflow)
    assert "actions actions-minimal" in workflow
