"""
Benchmark set 7, "actions" (bench/tasks_actions/, decision record 0040, plan docs/plans/bench-7-actions.md): declared
actions (`do r ACTION TARGET [to VALUE];`, `in_state X "label"`), pre-registered against RoboLang 1.7 (L3 of iteration
I2) and labelled for 1.8 in L4 (errata docs/evidence/bench-errata-1-8.md): `expect` is the 1.8 label, `expect_1_7` keeps
the 1.7 one, and `expect_1_8` holds the paths (the goal and the world patch of each) and the world 1.4 facts.

10 `action` requests that are programs in 1.8, 5 `infeasible` in 1.8 (0040 A6, A7), 5 `inexpressible` that stay out
(0040 A4, A10, A11; G-PROX, G-TEACH, G-LOOP, G-CLOCK, G-SELF), 4 `control` requests that are programs in both versions.
Every program task has a 1.8 reference that is right on every path and does exactly the actions asked (`actions_ok`).
The 1.7 attempts stay as the evidence of the baseline: 1.7 rejects each with `E203` (`do` is no statement). Also here:
the scorer's `actions_ok` and `action_replaced` (0040 B1, B5, B8). No network and no SDK; the model run is the
`Benchmark` workflow with `set_only: actions`.
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
CLOCK = {t: d for t, d in SET.items() if d["group"] == "clock"}  # x05_open_at_six: a program in 1.9 (0042)
CONTROLS = {t: d for t, d in SET.items() if d["group"] == "control"}
DECLINED = {**ACTION, **INFEASIBLE, **INEXPRESSIBLE, **CLOCK}  # what 1.7 declined: an attempt each (the baseline)
PROGRAMS = {**ACTION, **CONTROLS, **CLOCK}  # what 1.9 answers with a program: a reference each
DO_LINE = re.compile(r"^\s*do [^\n]*\n", re.M)  # an attempt keeps each `do` statement on one line


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
    assert groups == {"action": 10, "infeasible": 5, "inexpressible": 4, "control": 4, "clock": 1}
    for task_id, task in SET.items():
        assert task["id"] == task_id and (lib.ACTIONS_TASKS_DIR / f"{task_id}.json").is_file()
        assert (REPO / task["world"]).is_file() and task["tags"] and task["prompt"].endswith(".")
        assert task["prompt"].isascii() and "\r" not in read(lib.ACTIONS_TASKS_DIR / f"{task_id}.json")
    prefixes = {"action": "a", "infeasible": "i", "inexpressible": "x", "control": "c", "clock": "x"}
    for task_id, task in SET.items():
        assert re.fullmatch(prefixes[task["group"]] + r"\d\d_[a-z0-9_]+", task_id), task_id


def test_the_ids_are_new_ones_and_no_prompt_is_a_copy_of_a_standing_task():
    elsewhere = {}
    for name in lib.SETS:
        if name != "actions":
            elsewhere.update(lib.load_set(name))
    assert not set(SET) & set(elsewhere)
    assert not {t["prompt"] for t in SET.values()} & {t["prompt"] for t in elsewhere.values()}


def test_labels_are_1_8_in_expect_and_1_7_in_expect_1_7():
    for task_id, task in SET.items():
        assert task["expect_1_8"]["decision"] in ("program", "cannot"), task_id
        if task["group"] == "control":
            assert task["expect"] == {"decision": "program"} and lib.expected_goals(task), task_id
            assert task["expect_1_8"] == {"decision": "program"} and task["expected_actions"] == []
            assert lib.task_paths(task) == (task.get("expect_1_7") or {}).get("paths")  # c03's paths: 1.7 ones
            assert not {"why", "attempt_fails_with", "gap", "blockers"} & set(task)
            continue
        assert task["expect_1_7"] == CANNOT and len(task["why"]) > 40 and task["attempt_fails_with"] == "E203", task_id
        assert not {"expected_goal", "expected_goals", "expect_1_6"} & set(task)
        if task["group"] == "clock":  # 0042: the 1.8 label stays (inexpressible), `expect` is the 1.9 program
            assert task["expect"] == {"decision": "program"} and task["expect_1_8"]["decision"] == "cannot", task_id
            assert task["expect_1_9"]["decision"] == "program" and lib.task_paths(task) == task["expect_1_9"]["paths"]
            assert task["expected_clock"] and task["expected_actions"], task_id
            continue
        assert "world_1_4" in task["expect_1_8"], task_id
        later = task["expect_1_8"]
        assert lib.expectation(task) == (later["decision"], later.get("category", "none")), task_id
    for task_id, task in ACTION.items():
        later = task["expect_1_8"]
        assert task["expect"] == {"decision": "program"} and later["decision"] == "program" and "category" not in later
        assert lib.task_paths(task) == later["paths"] and later["paths"]
        for path in later["paths"]:
            assert path["when"] and ("goal" in path or "goals" in path or path.get("fails")) and "world_patch" in path
        actions = task["expected_actions"]
        assert 1 <= len(actions) <= 2 and all({"robot", "action", "target"} <= set(a) for a in actions), task_id
        assert all(set(a) <= {"robot", "action", "target", "value"} for a in actions)
    for task_id, task in INFEASIBLE.items():
        later = task["expect_1_8"]
        assert task["expect"] == {"decision": "cannot", "category": "infeasible"} == {
            k: later[k] for k in ("decision", "category")}, task_id
        assert later["because"] in ("0040 A6", "0040 A7") and task["blockers"], task_id
    for task_id, task in INEXPRESSIBLE.items():
        later = task["expect_1_8"]
        assert task["expect"] == CANNOT and {k: later[k] for k in ("decision", "category")} == CANNOT, task_id
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
    world = json.loads(read(REPO / decoy["world"]))
    assert world["robots"]["moxi1"]["actions"].keys() == {"open_door"}
    assert "supply_closet" in world["locations"] and world["objects"]["closet_door"]["class"] == "door"
    # the region limit is a 1.7 one: the world copy lists what printer1 honours
    limited = SET["a07_print_dry"]
    assert limited["expected_limits"] == [["no_go_region", "wet_floor"]]
    printer = json.loads(read(REPO / limited["world"]))["robots"]["printer1"]
    assert printer["honours"] == ["no_go_region", "in_region"]


def test_the_infeasible_tasks_each_break_one_check_of_0040_a6_and_a7():
    kinds = {t: set(d["tags"]) for t, d in INFEASIBLE.items()}
    assert sorted(k for tags in kinds.values() for k in tags if k not in ("open", "lock", "set_value")) == [
        "result_not_the_state", "robot_ability", "value_for_no_value_action", "wrong_target_kind", "wrong_value_kind"]
    runner = world_of("i01_runner_cannot_open")["robots"]
    assert runner["runner"]["actions"] == {} and runner["chef_arm"]["actions"]["open"]["target"] == "object"
    assert world_of("i02_lock_a_berth")["robots"]["crane1"]["actions"]["lock"]["target"] == "object"
    set_action = world_of("i03_fryer_in_kilograms")["robots"]["flippy1"]["actions"]["set"]
    assert set_action["value"] == "temperature" and " 2 kg" in SET["i03_fryer_in_kilograms"]["prompt"]
    assert "value" not in world_of("i04_open_by_distance")["robots"]["moxi1"]["actions"]["open"]
    locked = world_of("i05_leave_it_locked")["robots"]["moxi1"]["actions"]
    assert "locked" not in {a["result"] for a in locked.values()} and locked.keys() == {"open", "close"}


def world_of(task_id: str) -> dict:
    return json.loads(read(REPO / SET[task_id]["world"]))


# -------- the world 1.4 copies --------

def test_every_world_is_a_1_4_copy_and_holds_the_facts_its_label_rests_on():
    """The 1.4 facts a label needs are recorded in `expect_1_8.world_1_4`; the world copies hold every one of them."""
    for task_id, task in SET.items():
        world = world_of(task_id)
        if task["group"] == "clock":  # x05_open_at_six: a 1.5 copy of kitchen_values_1_4 (0042), tested in set 8's way
            assert world["world_schema_version"] == "1.5" and world["tasks"] == {} and "clock" in world
            continue
        assert world["world_schema_version"] == "1.4" and world["tasks"] == {}, task_id
        assert Path(task["world"]).stem.endswith("_1_4") and "\r" not in read(REPO / task["world"])
        if task["group"] == "control":
            assert "world_1_4" not in task["expect_1_8"]
            continue
        facts = task["expect_1_8"]["world_1_4"]
        assert set(facts) <= {"robots", "object_states", "location_states"}, task_id
        for robot, profile in facts["robots"].items():
            for name, declared in profile["actions"].items():  # a shared world may declare more (a09 and u16: drone1)
                assert world["robots"][robot]["actions"][name] == declared, (task_id, robot, name)
        for obj, state in facts.get("object_states", {}).items():
            assert world["objects"][obj]["state"] == state, (task_id, obj)
        for location, state in facts.get("location_states", {}).items():
            assert world["location_states"][location] == state, (task_id, location)
        for profile in facts["robots"].values():
            for declared in profile["actions"].values():
                assert declared["target"] in ("object", "location") and declared["result"]
                assert declared.get("value", "temperature") in ("temperature", "mass", "distance", "label")
    for task_id, task in ACTION.items():  # the action asked is declared by the robot asked, on the declared target kind
        world = world_of(task_id)
        for wanted in task["expected_actions"]:
            declared = world["robots"][wanted["robot"]]["actions"][wanted["action"]]
            kind_of = "object" if wanted["target"] in world["objects"] else "location"
            assert declared["target"] == kind_of, task_id
            assert ("value" in wanted) == ("value" in declared), task_id


def test_the_world_files_are_the_old_ones_with_only_the_1_4_fields_added_and_loaded_by_the_simulator():
    from robolang.ir import compile_source
    from robolang.sim import load_world
    source = "robolang 1.8;\ntask t(robot r, location l) { goal { robot_at r l; } plan { move r to l; } }\n"
    plan = compile_source(source).ir.tasks[0]
    names = {Path(t["world"]).stem for t in SET.values() if t["group"] != "clock"}
    assert len(names) == 11  # one file for each profile a request needs: the decoy and the two-action ward differ
    bases = {"ward_door": "ward", "ward_close": "ward", "ward_hand": "ward", "fry_station_selftest": "fry_station",
             "site_limits": "site_1_3"}
    for name in sorted(names):
        data = (REPO / "bench" / "worlds" / f"{name}.json").read_bytes()
        world = json.loads(data)
        robot = next(iter(world["robots"]))
        load_world(data, f"{name}.json", plan, {"r": robot, "l": world["locations"][0]})
        stem = name[:-4]
        base = json.loads(read(REPO / "bench" / "worlds" / f"{bases.get(stem, stem)}.json"))
        for key in ("locations", "regions", "sensors", "events"):
            assert world.get(key) == base.get(key), (name, key)
        assert set(world["robots"]) == set(base["robots"]) and set(world["objects"]) == set(base["objects"])
    assert {Path(t["world"]).stem for t in CONTROLS.values()} <= names
    assert SET["c01_fetch_no_door"]["world"] == SET["a01_open_then_fetch"]["world"]  # a closed door, not asked for
    assert world_of("c01_fetch_no_door")["objects"]["closet_door"]["state"] == "closed"


def test_the_model_is_shown_each_robots_declared_actions_and_the_states_c12():
    shown = lib.describe_world(world_of("a10_decoy_open_closet"))
    assert "can do: open_door (on an object, leaves it open)" in shown
    assert "closet_door (door, at supply_closet, state closed)" in shown
    assert "can do: open (" not in shown  # the decoy: the profile says open_door, not open
    fry = lib.describe_world(world_of("a02_set_then_wait_reading"))
    assert "set (on a location, leaves it heating, takes a temperature)" in fry
    assert "location states: fryer is off" in fry
    kitchen = lib.describe_world(world_of("i01_runner_cannot_open"))
    assert "runner (at pass, no gripper, ready, can do no declared action)" in kitchen.replace("max speed", "").replace(
        ", max 0.5 m/s", "") or "can do no declared action" in kitchen
    # an older world is described as before: no action or state text appears
    old = lib.describe_world(json.loads(read(REPO / "bench" / "worlds" / "ward.json")))
    assert "can do" not in old and "state" not in old and "location states" not in old
    assert "honours" not in lib.describe_world(world_of("a03_print_gridline"))  # a 1.4 file with no `honours`
    assert "honours no_go_region, in_region" in lib.describe_world(world_of("a07_print_dry"))


# -------- the references and the attempts --------

def test_references_and_attempts_are_where_they_should_be():
    names = [p.name for p in REF.iterdir()]
    refs = {n.split(".")[0] for n in names if ".attempt" not in n}
    attempts = {n.split(".")[0] for n in names if ".attempt." in n}
    assert refs == set(PROGRAMS) and attempts == set(DECLINED) | set(ACTION) - set(CONTROLS)
    assert attempts == {t for t, d in SET.items() if d["group"] != "control"}
    for task_id in DECLINED:
        assert (REF / f"{task_id}.attempt.rob").is_file() and (REF / f"{task_id}.attempt.json").is_file()
    for name in names:
        assert "\r" not in read(REF / name)  # Windows-safe: LF only, never compared as CRLF


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_the_1_7_attempt_still_fails_with_the_recorded_code(task_id):
    """The baseline evidence (docs/evidence/bench-set7-baseline.md): 1.7 has no `do`."""
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
    action), and it is what the baseline counted as a false program (the plan's B8)."""
    answer = attempt_answer(task_id, strip=True)
    assert "do " not in answer["rob"].split("\n", 1)[1]
    task = dict(DECLINED[task_id], goal_free=True, expect_1_8={"decision": "program"}, expect_1_9=None)  # no paths
    checked = lib.check_answer(task, answer)
    assert checked.valid, (task_id, checked.report)
    assert checked.language_version == "1.7"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.plan_succeeds, (task_id, judgement.state, judgement.failure_code)


@pytest.mark.parametrize("task_id, code", [("i01_runner_cannot_open", "S006"), ("i02_lock_a_berth", "S007"),
                                           ("i03_fryer_in_kilograms", "S008"), ("i04_open_by_distance", "S008"),
                                           ("a10_decoy_open_closet", "S006"), ("i05_leave_it_locked", "S006")])
def test_the_attempts_as_1_8_programs_fail_the_world_check_with_the_code_of_their_kind(task_id, code):
    """The same attempt, written as a 1.8 file, is valid and then stopped by the world check (0040 A6) before it runs:
    the robot lacks the action, the target is the wrong kind, the value is of the wrong kind or given for none."""
    answer = attempt_answer(task_id)
    answer["rob"] = answer["rob"].replace("robolang 1.7;", "robolang 1.8;")
    checked = lib.check_answer(DECLINED[task_id], answer)
    assert not checked.valid and checked.codes == (code,), (task_id, checked.report)


def test_i05_a_program_for_a_state_no_declared_action_leaves_fails_with_s009():
    answer = {"decision": "program", "category": "none", "reason": "-", "entities": [], "task": "keep_locked",
              "bindings": {"r": "moxi1", "door": "closet_door"},
              "rob": 'robolang 1.8;\n\ntask keep_locked(robot r, object door) {\n    goal { in_state door "locked"; }\n'
                     "    plan { wait for 1 s; }\n}\n"}
    checked = lib.check_answer(SET["i05_leave_it_locked"], answer)
    assert not checked.valid and checked.codes == ("S009",), checked.report


def test_the_stripped_attempt_of_a_branching_task_follows_the_path_the_world_forces():
    """a04: `pass_area` clear, the branch with the action runs (as the half program, it succeeds); not clear, the
    other branch sends its message and stops, exactly as the 1.8 label says."""
    task = SET["a04_open_if_clear"]
    answer = attempt_answer("a04_open_if_clear", strip=True)
    checked = lib.check_answer(task, answer)
    paths = [dict(p, goal=[], world_patch=p["world_patch"]) for p in task["expect_1_8"]["paths"]]
    labelled = dict(task, goal_free=True, expect_1_8={"decision": "program", "paths": paths})
    judged = lib.judge_paths(labelled, checked, answer["bindings"])
    assert [(p["state"], p["goal_ok"] and p["plan_ok"]) for p in judged] == [("SUCCEEDED", True), ("FAILED", True)]


def test_the_attempt_binds_the_entities_the_request_names():
    for task_id, task in ACTION.items():
        bound = set(json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"].values())
        for wanted in task["expected_actions"]:
            assert {wanted["robot"], wanted["target"]} <= bound, task_id
    for task_id in INFEASIBLE:
        assert json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"]


# -------- the 1.8 references --------

@pytest.mark.parametrize("task_id", sorted(PROGRAMS))
def test_the_reference_is_valid_succeeds_on_every_path_and_does_exactly_the_actions_asked(task_id):
    task = PROGRAMS[task_id]
    answer = reference(task_id)
    version = "1.9" if task_id in CLOCK else "1.8"
    assert answer["rob"].count(f"robolang {version};") == 1 and answer["rob"].isascii() and "\r" not in answer["rob"]
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == version
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert judgement.actions_ok is True and not judgement.action_replaced and not judgement.half_program
    assert all(judgement.checks.values()), judgement.checks
    paths = lib.task_paths(task) or []
    assert [p["when"] for p in judgement.paths or []] == [p["when"] for p in paths]
    assert all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths or []), judgement.paths
    if task["group"] == "control":
        assert not re.search(r"\bdo\b|in_state", answer["rob"].split("\n", 1)[1])
    else:  # the reference states the result the request wants and verifies it with in_state
        assert "in_state" in answer["rob"] and len(DO_LINE.findall(answer["rob"])) == len(task["expected_actions"])
    if "expected_readings" in task:
        assert judgement.value_ok


def test_every_path_of_the_action_tasks_is_reached_by_a_world_patch_or_a_scripted_fault():
    a05 = SET["a05_open_or_report"]["expect_1_8"]["paths"][1]
    assert a05["scripted_fault"] == {"step": "do", "action": "open", "target": "closet_door"} and a05["fails"]
    answer = reference("a05_open_or_report")
    checked = lib.check_answer(SET["a05_open_or_report"], answer)
    paths = lib.judge_paths(SET["a05_open_or_report"], checked, answer["bindings"])
    # the second path scripts the world's fault on the program's `do`: the handler reports and the run stops (S262)
    assert [(p["state"], p["failure_code"]) for p in paths] == [("SUCCEEDED", None), ("FAILED", "S262")]
    assert lib._do_faults(checked, answer["bindings"], a05["scripted_fault"]) == [{"step": 2}]
    # a program with no `do` on that door has nothing to fail: the path cannot stop, so it is judged wrong
    bare = dict(answer, rob=re.sub(r"do r open door else \{.*?\}\n", "", answer["rob"], flags=re.S))
    bare_paths = lib.judge_paths(SET["a05_open_or_report"], lib.check_answer(SET["a05_open_or_report"], bare),
                                 answer["bindings"])
    assert bare_paths[1]["state"] == "FAILED" and not bare_paths[1]["plan_ok"]  # S301, and no message was sent


def test_a06_gains_the_move_that_c2_requires():
    rob = read(REF / "a06_two_cranes_lock.rob")
    assert "move b to berth;" in rob and rob.index("move b to berth;") < rob.index("do b lock y;")
    world = world_of("a06_two_cranes_lock")
    assert world["robots"]["crane2"]["location"] == "berth_2" and world["objects"]["cont_102"]["location"] == "berth_1"
    answer = reference("a06_two_cranes_lock")
    no_move = dict(answer, rob=rob.replace("        move b to berth;\n", ""))
    checked = lib.check_answer(SET["a06_two_cranes_lock"], no_move)
    assert checked.valid and lib.judge(SET["a06_two_cranes_lock"], checked, answer["bindings"]).failure_code == "S271"


def test_the_goal_is_compared_by_meaning_with_in_state_as_a_fact_about_the_state_c1():
    task = SET["a01_open_then_fetch"]
    answer = reference("a01_open_then_fetch")
    for goal, ok in (('in_state door "open"; object_at kit dst;', True),
                     ('object_at kit dst; in_state door "open";', True),
                     ('in_state door "open"; object_at kit dst; not in_state door "closed";', True),
                     ('in_state door "closed"; object_at kit dst;', False),  # another label
                     ('object_at kit dst;', False),  # the opening is not asked for in the goal
                     ('in_state door "open"; holding r kit;', False)):  # a state and a place are independent facts
        rob = re.sub(r"goal \{[^}]*\}", "goal { " + goal + " }", answer["rob"], count=1)
        checked = lib.check_answer(task, dict(answer, rob=rob))
        assert checked.valid, (goal, checked.report)
        assert bool(lib.judge(task, checked, answer["bindings"]).goal_correct) is ok, goal
    # a09 accepts the dock with or without the drone at the workshop
    dock = SET["a09_dock_drone"]
    both = reference("a09_dock_drone")
    with_robot = dict(both, rob=both["rob"].replace('goal { in_state home "docked"; }',
                                                     'goal { in_state home "docked"; robot_at r home; }'))
    checked = lib.check_answer(dock, with_robot)
    assert checked.valid and lib.judge(dock, checked, both["bindings"]).goal_correct


# -------- the controls --------

def test_the_controls_are_a_move_a_fetch_a_measure_and_a_wait_and_want_no_action():
    assert all(t["expected_actions"] == [] for t in CONTROLS.values())
    assert "measure r temperature of oil using probe as t;" in read(REF / "c02_oil_reading.rob")
    waits = read(REF / "c03_wait_oil_by_itself.rob")
    assert "wait until temperature of oil using probe > 170 degC timeout 60 s" in waits
    assert [bool(p.get("fails")) for p in lib.task_paths(CONTROLS["c03_wait_oil_by_itself"])] == [False, True]
    assert all(lib.task_paths(CONTROLS[t]) is None for t in ("c01_fetch_no_door", "c04_plain_move_printer"))
    # the fetch control is a task whose world holds a closed door: a model that adds `do r open` fails the control
    assert world_of("c01_fetch_no_door")["objects"]["closet_door"]["state"] == "closed"
    # c04: printer1 declares `print` in its world and the request does not ask for it
    assert "print" in world_of("c04_plain_move_printer")["robots"]["printer1"]["actions"]


def test_a_control_that_adds_an_action_is_judged_wrong_even_though_everything_else_passes():
    task = SET["c01_fetch_no_door"]
    answer = reference("c01_fetch_no_door")
    opened = dict(answer, rob=answer["rob"].replace(
        "    plan {\n", "    plan {\n        move r to src;\n        do r open door;\n", 1).replace(
        "task ", "task ", 1).replace("(robot r,", "(robot r, object door,", 1))
    opened["bindings"] = dict(answer["bindings"], door="closet_door")
    checked = lib.check_answer(task, opened)
    assert checked.valid, checked.report
    judgement = lib.judge(task, checked, opened["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds
    assert judgement.actions_ok is False and judgement.checks["declared_actions"] is False
    assert judgement.action_replaced is None  # a control asks for no action, so nothing can be replaced


# -------- the scorer: actions_ok and action_replaced --------

def judged_row(task_id: str, rob: str, bindings: dict | None = None):
    answer = reference(task_id)
    answer = dict(answer, rob=rob, bindings=bindings or answer["bindings"])
    checked = lib.check_answer(SET[task_id], answer)
    assert checked.valid, checked.report
    return lib.judge(SET[task_id], checked, answer["bindings"])


def test_actions_ok_wants_the_right_action_on_the_right_target_with_the_right_value_and_no_other():
    ref = read(REF / "a02_set_then_wait_reading.rob")
    assert judged_row("a02_set_then_wait_reading", ref).actions_ok is True
    # the same value in another unit of its kind is the same value: 448.15 K is 175 degC
    kelvin = ref.replace("175 degC", "448.15 K")
    assert judged_row("a02_set_then_wait_reading", kelvin).actions_ok is True
    # another value, no value, or a second `do` is not what was asked (the last two are also stopped by the world)
    assert judged_row("a02_set_then_wait_reading", ref.replace("175 degC", "180 degC")).actions_ok is False
    extra = ref.replace("        move r to src;", '        do r set fryer to 175 degC;\n        move r to src;')
    assert judged_row("a02_set_then_wait_reading", extra).actions_ok is True  # the same `do` twice is one action
    wrong_robot = read(REF / "a06_two_cranes_lock.rob").replace("do b lock y;", "do a lock y;")
    judgement = judged_row("a06_two_cranes_lock", wrong_robot)
    assert judgement.actions_ok is False and judgement.plan_succeeds  # the goal holds: crane1 did crane2's job
    swapped = read(REF / "a06_two_cranes_lock.rob").replace("do a lock x;", "do b lock x;")
    assert judged_row("a06_two_cranes_lock", swapped).actions_ok is False
    one_only = read(REF / "a06_two_cranes_lock.rob").replace("        move b to berth;\n        do b lock y;\n", "")
    one = judged_row("a06_two_cranes_lock", one_only)
    assert one.actions_ok is False and one.action_replaced is False  # fails its goal: not a *silent* replacement


def test_action_replaced_is_a_valid_succeeding_program_without_the_asked_action():
    rob = read(REF / "a03_print_gridline.rob")
    replaced = rob.replace('in_state line "printed";', "robot_at r line;").replace("        do r print line;\n", "")
    judgement = judged_row("a03_print_gridline", replaced)
    assert judgement.plan_succeeds and not judgement.goal_correct and judgement.action_replaced is True
    assert judged_row("a03_print_gridline", rob).action_replaced is False
    unasked = judged_row("c04_plain_move_printer", read(REF / "c04_plain_move_printer.rob"))
    assert unasked.action_replaced is None and unasked.actions_ok is True


# -------- the atlas --------

def test_every_task_1_7_declined_is_labelled_for_1_8_and_g_act_is_closed():
    in_gaps = {t: g["id"] for g in ATLAS["gaps"] for t in g["tasks"] if t in SET}
    assert set(in_gaps) == set(INEXPRESSIBLE)  # the programs and the infeasible tasks left the atlas (as set 6's did)
    assert in_gaps == {"x01_hand_to_ana": "G-PROX", "x02_selftest_result": "G-SELF", "x03_teach_restock": "G-TEACH",
                       "x04_open_until_open": "G-UNBOUNDED"}  # x05_open_at_six is a 1.9 program, G-CLOCK lists none
    for task_id, gap in in_gaps.items():
        assert DECLINED[task_id]["gap"] == gap and GAPS[gap]["scope"] in ("open", "out")  # G-UNBOUNDED: out (0041)
    assert GAPS["G-ACT"]["scope"] == "closed" and GAPS["G-ACT"]["closed_in"] == "RoboLang 1.8"  # L6 of I2
    assert {t["gap"] for t in ACTION.values()} | {t["gap"] for t in INFEASIBLE.values()} == {"G-ACT"}
    assert not {t for g in ATLAS["gaps"] for t in g["tasks"]} & (set(CONTROLS) | set(ACTION) | set(INFEASIBLE))


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


def test_the_scorer_counts_a_silent_replacement_and_reports_actions_ok_and_action_replaced_per_model(tmp_path):
    a03 = reference("a03_print_gridline")
    replaced = dict(a03, rob=a03["rob"].replace('in_state line "printed";', "robot_at r line;").replace(
        "        do r print line;\n", ""))
    replies = {
        "a01_open_then_fetch": reference("a01_open_then_fetch"),            # right
        "a02_set_then_wait_reading": reference("a02_set_then_wait_reading"),  # right
        "a03_print_gridline": replaced,                                     # a move for a print: the silent one
        "a10_decoy_open_closet": attempt_answer("a10_decoy_open_closet", strip=True),  # also valid, not the action
        "i01_runner_cannot_open": decline("infeasible"),                    # right
        "i02_lock_a_berth": decline(),                                      # right decision, category differs
        "c04_plain_move_printer": reference("c04_plain_move_printer"),      # right
        "c01_fetch_no_door": decline(),                                     # false refusal of a control
        "x01_hand_to_ana": decline(),                                       # right
    }
    out_dir = actions_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "actions" and meta["prompt"] == "full"
    assert by["a01_open_then_fetch"]["decision_correct"] and by["a01_open_then_fetch"]["request_satisfied"]
    assert by["a01_open_then_fetch"]["actions_ok"] is True and by["a01_open_then_fetch"]["action_replaced"] is False
    assert by["a03_print_gridline"]["decision_correct"] and not by["a03_print_gridline"]["goal_correct"]
    assert by["a03_print_gridline"]["actions_ok"] is False and by["a03_print_gridline"]["action_replaced"] is True
    decoy = by["a10_decoy_open_closet"]
    assert decoy["action_replaced"] is True and not decoy["request_satisfied"]
    assert by["i01_runner_cannot_open"]["decision_correct"] and by["i01_runner_cannot_open"]["category_correct"]
    runner = by["i01_runner_cannot_open"]
    assert runner["actions_ok"] is None and runner["action_replaced"] is None
    declined = by["i02_lock_a_berth"]
    assert declined["decision_correct"] and declined["category_correct"] is False
    assert by["c04_plain_move_printer"]["decision_correct"] and by["c04_plain_move_printer"]["goal_correct"]
    control = by["c04_plain_move_printer"]
    assert control["actions_ok"] is True and control["action_replaced"] is None
    assert by["c01_fetch_no_door"]["false_refusal"] and by["c01_fetch_no_door"]["actions_ok"] is False
    assert by["x01_hand_to_ana"]["decision_correct"]
    text = score.score_dir(out_dir)
    assert "Task set actions, prompt full." in text
    assert "| actions_ok (exactly the declared actions asked, none on a control; all answers to those tasks) " \
           "| 3/6 | 3/6 (50.0%) |" in text
    assert "| action_replaced_silently (valid, SUCCEEDED, an asked action missing; all answers to tasks that ask " \
           "for one) | 2/4 | 2/4 (50.0%) |" in text
    kinds = text.split("By kind of task:")[1]
    for kind in ("action", "infeasible", "control"):
        assert f"| {kind} |" in kinds


def test_a_program_for_an_infeasible_task_is_a_false_program_the_world_check_may_catch():
    answer = attempt_answer("i01_runner_cannot_open")
    answer["rob"] = answer["rob"].replace("robolang 1.7;", "robolang 1.8;")
    checked = lib.check_answer(SET["i01_runner_cannot_open"], answer)
    assert not checked.valid and checked.codes == ("S006",)  # caught by validate --world: no run would start


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
    assert "inputs.set_only == 'actions'" in step and "pull_request" not in step
    pr_step = next(s for s in steps if s.startswith("name: Run (pull request)"))
    weekly = next(s for s in steps if s.startswith("name: Run (weekly or on demand)"))
    assert "actions" not in pr_step and "inputs.set_only == ''" in weekly and "--set actions" not in weekly
    assert re.search(r"      set_only:\n        description: .*\n        type: string\n        default: \"\"", workflow)
    assert "actions actions-minimal" in workflow
