"""
Benchmark set 9, "missions" (bench/tasks_missions/, decision record 0041, plan docs/plans/bench-9-missions.md): the
mission layer of RoboLang 2.0 (a top-level `mission` that runs tasks one run after another with a hard bound, an
`until` condition and a failure policy), pre-registered against RoboLang 1.9 before any code (L3 of iteration I4).
`expect` is the 1.9 label (the baseline), `expect_2_0` the label for acceptance, with the `expected_mission` of a
program and the mission outcome of each path.

10 `mission` requests that become programs in 2.0, 5 `infeasible` in 2.0, 7 `inexpressible` that stay out (0041 M2,
M3, M6, M7; G-LOOP, G-MSG, G-CHOOSE), 4 `control` requests that are programs in both versions (a fixed sequence, a
step retry, a bounded wait: no mission). 1.9 declines every non-control task, and every one has an obvious attempt
that 1.9 rejects (`E201`: a top-level `mission` where a task is expected); those attempts stay as the evidence of the
baseline. Only the controls have a 1.9 reference; the 2.0 references wait for L4. No network and no SDK; the model run
is the `Benchmark` workflow with `set_only: missions`.
"""

from __future__ import annotations
import io
import json
import re
from collections import Counter
from math import ceil
from pathlib import Path

import pytest
from test_bench import FakeClient, message

from bench import lib, run, score

REPO = Path(__file__).resolve().parent.parent
SET = lib.load_set("missions")
REF = lib.MISSIONS_REFERENCE_DIR
ATLAS = json.loads((REPO / "bench" / "atlas.json").read_text(encoding="utf-8"))
GAPS = {g["id"]: g for g in ATLAS["gaps"]}
CANNOT = {"decision": "cannot", "category": "inexpressible"}
PROGRAMS = {t: d for t, d in SET.items() if d["group"] == "mission"}
INFEASIBLE = {t: d for t, d in SET.items() if d["group"] == "infeasible"}
INEXPRESSIBLE = {t: d for t, d in SET.items() if d["group"] == "inexpressible"}
CONTROLS = {t: d for t, d in SET.items() if d["group"] == "control"}
DECLINED = {**PROGRAMS, **INFEASIBLE, **INEXPRESSIBLE}  # what 1.9 must decline: an attempt each, a gap each
MISSION_LINE = re.compile(r"^[^\n]*// @mission\n", re.M)  # an attempt marks every line that is the mission
OUTCOMES = ("completed", "bounded", "failed")


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def reference(task_id: str) -> dict:
    ref = json.loads(read(REF / f"{task_id}.json"))
    rob = read(REF / f"{task_id}.rob")
    return {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob, **ref}


def attempt_answer(task_id: str, strip: bool = False) -> dict:
    """The obvious attempt as a program answer; `strip` removes its mission part (the silent replacement a model may
    write in 1.9: the same task run once, the repetition left out)."""
    stem = REF / f"{task_id}.attempt"
    rob = read(Path(f"{stem}.rob"))
    if strip:
        rob = MISSION_LINE.sub("", rob)
    return {"decision": "program", "category": "none", "reason": "an attempt", "entities": [], "rob": rob,
            **json.loads(read(Path(f"{stem}.json")))}


def decline(category: str = "inexpressible") -> dict:
    return {"decision": "cannot", "category": category, "reason": "cannot be done", "rob": "", "task": "",
            "bindings": []}


def world_ids(task: dict) -> set[str]:
    world = json.loads(read(REPO / task["world"]))
    return {*world["robots"], *world["objects"], *world["locations"], *world.get("sensors", {}),
            *world.get("regions", {})}


# -------- the set --------

def test_the_mix_of_tasks():
    assert len(SET) == 26
    groups = Counter(d["group"] for d in SET.values())
    assert groups == {"mission": 10, "infeasible": 5, "inexpressible": 7, "control": 4}
    for task_id, task in SET.items():
        assert task["id"] == task_id and (lib.MISSIONS_TASKS_DIR / f"{task_id}.json").is_file()
        assert (REPO / task["world"]).is_file() and task["tags"] and task["prompt"].endswith(".")
        assert task["prompt"].isascii() and "\r" not in read(lib.MISSIONS_TASKS_DIR / f"{task_id}.json")
    prefixes = {"mission": "m", "infeasible": "i", "inexpressible": "x", "control": "c"}
    for task_id, task in SET.items():
        assert re.fullmatch(prefixes[task["group"]] + r"\d\d_[a-z0-9_]+", task_id), task_id


def test_the_ids_are_new_ones_and_no_prompt_is_a_copy_of_a_standing_task():
    elsewhere = {}
    for name in lib.SETS:
        if name != "missions":
            elsewhere.update(lib.load_set(name))
    assert not set(SET) & set(elsewhere)
    assert not {t["prompt"] for t in SET.values()} & {t["prompt"] for t in elsewhere.values()}


def test_labels_are_1_9_in_expect_and_2_0_in_expect_2_0():
    for task_id, task in SET.items():
        later = task["expect_2_0"]
        assert later["decision"] in ("program", "cannot"), task_id
        if task["group"] == "control":
            assert task["expect"] == {"decision": "program"} and lib.expected_goals(task), task_id
            assert later == {"decision": "program", "expected_mission": None}
            assert not {"why", "attempt_fails_with", "gap", "blockers"} & set(task)
            continue
        assert task["expect"] == CANNOT and lib.expectation(task) == ("cannot", "inexpressible"), task_id
        assert len(task["why"]) > 40 and task["attempt_fails_with"] == "E201", task_id
        assert not {"expected_goal", "expected_goals", "expect_1_9", "paths"} & set(task)
        assert task["gap"] in GAPS and GAPS[task["gap"]]["scope"] == "open", task_id
    for task_id, task in INFEASIBLE.items():
        later = task["expect_2_0"]
        assert later["decision"] == "cannot" and later["category"] == "infeasible", task_id
        assert later["because"].startswith("0041 M") and task["blockers"], task_id
    for task_id, task in INEXPRESSIBLE.items():
        later = task["expect_2_0"]
        assert {k: later[k] for k in ("decision", "category")} == CANNOT, task_id
        assert later["because"].startswith("0041 "), task_id
    reasons = sorted(t["expect_2_0"]["because"] for t in INEXPRESSIBLE.values())
    assert reasons == ["0041 (G-CHOOSE, I7)", "0041 M2", "0041 M3 (open point 2)", "0041 M3 (open point 2)",
                       "0041 M6", "0041 M6", "0041 M7"]


def test_the_mission_programs_cover_the_cases_the_pre_registration_lists():
    tags = Counter(tag for t in PROGRAMS.values() for tag in t["tags"])
    for tag in ("repeat", "count_bound", "duration_bound", "until", "continue", "retry_n", "stop_on_failure",
                "alternate", "until_after_first_run", "bounded_outcome", "region_limit", "declared_action",
                "wait_in_task", "no_count_stated"):
        assert tags[tag] >= 1, tag
    outcomes = Counter(p["outcome"] for t in PROGRAMS.values() for p in t["expect_2_0"]["paths"])
    assert set(outcomes) == set(OUTCOMES) and all(outcomes[o] >= 3 for o in OUTCOMES)
    assert len({t["world"] for t in PROGRAMS.values()}) >= 5
    policies = Counter("retry" if isinstance(m, dict) else m for m in
                       (t["expect_2_0"]["expected_mission"]["on_failure"] for t in PROGRAMS.values()))
    assert policies == {"stop": 4, "continue": 4, "retry": 1} or set(policies) == {"stop", "continue", "retry"}
    # a task that still carries its own 1.7 limit and its own 1.8 `do`, run by a mission
    limits = SET["m08_rounds_out_of_the_wet_floor"]["expect_2_0"]["expected_mission"]["runs"][0]["limits"]
    assert limits == [["no_go_region", "wet_floor"]]
    actions = SET["m09_open_the_oven_door"]["expect_2_0"]["expected_mission"]["runs"][0]["actions"]
    assert actions == [{"robot": "chef_arm", "action": "open", "target": "oven_door_1"}]
    # two run lines in one repeat (open point 1), and an until that already holds after run 1
    assert len(SET["m05_alternate_the_carry"]["expect_2_0"]["expected_mission"]["runs"]) == 2
    first = SET["m06_carry_until_there"]["expect_2_0"]["paths"][0]
    assert first["outcome"] == "completed" and first["runs"] == 1
    # a mission that ends by its bound, not by its until
    assert [p["outcome"] for p in SET["m07_rounds_until_quay_clear"]["expect_2_0"]["paths"]] == ["completed", "bounded"]


def test_each_expected_mission_names_entities_that_the_world_has_and_states_a_bound():
    for task_id, task in PROGRAMS.items():
        mission = task["expect_2_0"]["expected_mission"]
        assert set(mission) == {"runs", "repeat", "until", "on_failure"}, task_id
        assert mission["runs"], task_id
        for entry in mission["runs"]:
            assert entry["task"] and entry["goal"] and entry["entities"], task_id
            assert set(entry["entities"]) <= world_ids(task), (task_id, entry["entities"])
            for subject, prop, value in entry["goal"]:
                assert subject.startswith("rl:") and prop in ("located_in", "in_state"), task_id
        assert mission["on_failure"] in ("stop", "continue") or set(mission["on_failure"]) == {"retry"}, task_id
        if task_id == "m02_retry_the_carry":  # a single run with `retry 12`: no `repeat` clause is needed
            assert mission["repeat"] is None and mission["on_failure"] == {"retry": 12}
            continue
        bound = mission["repeat"]
        assert bound["max_runs"] or bound["max_duration"], task_id  # M2: every repeat has a hard bound
        if mission["until"]:
            assert mission["until"]["condition"], task_id


def test_the_bound_in_the_label_is_the_bound_in_the_request_no_more_and_no_less():
    """0041's measure: a bound dropped or invented. Every bound a label has is stated in its request, and every
    bound a request states is in its label."""
    words = {"h": "hour", "min": "minute", "s": "second"}
    for task_id, task in PROGRAMS.items():
        mission = task["expect_2_0"]["expected_mission"]
        prompt = task["prompt"]
        bound = mission["repeat"] or {"max_runs": None, "max_duration": None}
        if bound["max_runs"]:
            counted = rf"\b{bound['max_runs']} (such rounds|rounds|carries|attempts|times|runs|scans)"
            assert re.search(counted, prompt), task_id
        else:
            assert {"duration_bound", "retry"} & set(task["tags"]), task_id
        if bound["max_duration"]:
            duration = bound["max_duration"]
            assert f"{duration['value']} {words[duration['unit']]}" in prompt, task_id
        else:
            assert not re.search(r"for (at most )?\d+ (hours|minutes)", prompt), task_id
    # a duration only: no run count in the label (the count is a result, 8 runs), as the request states none
    only = SET["m10_scan_for_two_hours"]["expect_2_0"]["expected_mission"]["repeat"]
    assert only == {"max_runs": None, "max_duration": {"value": 2, "unit": "h"}}
    # a count only, no duration
    assert SET["m01_shuttle_thirty_rounds"]["expect_2_0"]["expected_mission"]["repeat"] == {
        "max_runs": 30, "max_duration": None}
    # the retry count of m02 is the one in the request
    assert "12 more times" in SET["m02_retry_the_carry"]["prompt"]


def test_the_paths_are_forced_by_the_world_and_their_outcomes_follow_the_label():
    for task_id, task in PROGRAMS.items():
        mission = task["expect_2_0"]["expected_mission"]
        paths = task["expect_2_0"]["paths"]
        assert len(paths) >= 2, task_id
        assert len({p["when"] for p in paths}) == len(paths), task_id
        for path in paths:
            assert path["when"] and isinstance(path["world_patch"], dict), task_id
            assert path["outcome"] in OUTCOMES and path["runs"] >= 1, task_id
            assert 0 <= path["runs_verified"] <= path["runs"], task_id
            failed = path.get("failed_runs", [])
            assert path["runs_verified"] + len(failed) <= path["runs"], (task_id, path["when"])
            if mission["repeat"] and mission["repeat"]["max_runs"]:
                assert path["runs"] <= mission["repeat"]["max_runs"], (task_id, path["when"])
            if path["outcome"] == "failed":  # `stop` at the first failed run
                assert mission["on_failure"] in ("stop", {"retry": 12}), task_id
                assert failed and failed[-1] == path["runs"], (task_id, path["when"])
            if path["outcome"] == "completed":  # verified by an until, or a verified single run
                assert mission["until"] or mission["repeat"] is None, (task_id, path["when"])
                assert path["runs_verified"] >= 1 and (not failed or failed[-1] < path["runs"]), task_id
            if path["outcome"] == "bounded":
                assert mission["repeat"], task_id
                if mission["repeat"]["max_runs"] and not mission["repeat"]["max_duration"]:
                    assert path["runs"] == mission["repeat"]["max_runs"], (task_id, path["when"])
            faults = path.get("run_faults", [])
            assert all(f["run"] == "all" or f["run"] >= 1 for f in faults), task_id
            for number in failed:
                assert 1 <= number <= path["runs"], task_id
    # a retry stops at 1 + 12 runs
    gone = SET["m02_retry_the_carry"]["expect_2_0"]["paths"][-1]
    assert gone["outcome"] == "failed" and gone["runs"] == 13 and gone["runs_verified"] == 0
    assert gone["world_patch"]["objects"]["part7"]["present"] is False


def test_a_duration_bound_gives_the_count_of_runs_that_the_label_says():
    """A run starts only while the mission has run for less than its duration (a started run is never cut): runs of
    10 minutes in 3 hours are 18, runs of 15 minutes in 2 hours are 8."""
    for task_id, per_run_min, bound_min in (("m03_inspect_for_three_hours", 10, 180),
                                             ("m10_scan_for_two_hours", 15, 120)):
        paths = SET[task_id]["expect_2_0"]["paths"]
        assert f"wait for {per_run_min} min;" in read(REF / f"{task_id}.attempt.rob")
        bounded = next(p for p in paths if p["outcome"] == "bounded")
        assert bounded["runs"] == ceil(bound_min / per_run_min)
    # the until path of m03: the event at 25 minutes is first seen after run 3 (30 minutes), not after run 2
    patch = SET["m03_inspect_for_three_hours"]["expect_2_0"]["paths"][0]["world_patch"]["events"]
    assert patch[0]["at"] == {"value": "1500", "unit": "s"} and 2 * 600 < 1500 <= 3 * 600
    # m05: waits of 3 seconds, the aisle clears at 8 seconds in the world: after run 3 (9 seconds)
    world = json.loads(read(REPO / SET["m05_alternate_the_carry"]["world"]))
    assert world["events"][0]["at"] == {"value": "8", "unit": "s"} and 2 * 3 < 8 <= 3 * 3
    # m07: 5 seconds a round, the quay clears at 12 seconds: after run 3 (15 seconds)
    quay = SET["m07_rounds_until_quay_clear"]["expect_2_0"]["paths"][0]["world_patch"]["events"][0]
    assert quay["target"] == "quay" and quay["at"] == {"value": "12", "unit": "s"} and 2 * 5 < 12 <= 3 * 5


def test_the_infeasible_tasks_each_break_one_rule_of_0041():
    kinds = {t: set(d["tags"]) - {"mission", "count_bound", "duration_bound", "retry", "retry_n"}
             for t, d in INFEASIBLE.items()}
    assert kinds == {"i01_twelve_rounds_in_half_an_hour": {"bound_too_small"},
                     "i02_the_same_box_again": {"task_fails_on_repeat"},
                     "i03_until_a_pallet_that_is_not_there": {"until", "no_such_entity"},
                     "i04_more_retries_than_runs": {"contradiction"},
                     "i05_a_bound_shorter_than_one_round": {"bound_too_small"}}
    # the duration needed is more than the duration allowed: 12 rounds of 5 minutes, 30 minutes
    assert "wait for 5 min;" in read(REF / "i01_twelve_rounds_in_half_an_hour.attempt.rob")
    assert 12 * 5 > 30 and "12 such rounds" in SET["i01_twelve_rounds_in_half_an_hour"]["prompt"]
    # the same box again: it is at the table after run 1, so the grasp at shelf_a of run 2 cannot work
    again = SET["i02_the_same_box_again"]
    assert again["world"].endswith("warehouse.json") and "shelf_a to the table" in again["prompt"]
    # the until names an entity the world lacks
    assert "pallet_9" in SET["i03_until_a_pallet_that_is_not_there"]["prompt"]
    assert "pallet_9" not in world_ids(SET["i03_until_a_pallet_that_is_not_there"])
    # more retries than the runs the request allows in all
    assert "12 runs in all" in SET["i04_more_retries_than_runs"]["prompt"] and "20 more times" in (
        SET["i04_more_retries_than_runs"]["prompt"])
    # one round takes 2 minutes and the whole job may take 90 seconds
    assert "wait for 2 min;" in read(REF / "i05_a_bound_shorter_than_one_round.attempt.rob")
    assert "90 seconds" in SET["i05_a_bound_shorter_than_one_round"]["prompt"]
    # every one of them is an entity-and-bound request that needs no other change: the mission lines are what differ
    for task_id in INFEASIBLE:
        assert "mission" in read(REF / f"{task_id}.attempt.rob")


def test_the_inexpressible_tasks_each_stay_out_for_one_reason_of_0041():
    reasons = {t: set(d["tags"]) - {"mission", "count_bound", "duration_bound", "clock_time"}
               for t, d in INEXPRESSIBLE.items()}
    assert reasons == {"x01_never_stop": {"no_bound", "forever"},
                       "x02_every_hour_on_the_hour": {"every", "time_trigger"},
                       "x03_for_each_go_message": {"message_trigger", "no_message_waits"},
                       "x04_rounds_inside_rounds": {"nested", "repeat_in_repeat"},
                       "x05_every_new_part": {"stream", "choose_object"},
                       "x06_until_it_is_cool": {"until", "measured_condition"},
                       "x07_until_ten_to_ten": {"until", "clock_condition"}}
    assert {t: d["gap"] for t, d in INEXPRESSIBLE.items()} == {
        "x01_never_stop": "G-LOOP", "x02_every_hour_on_the_hour": "G-LOOP", "x03_for_each_go_message": "G-MSG",
        "x04_rounds_inside_rounds": "G-LOOP", "x05_every_new_part": "G-CHOOSE", "x06_until_it_is_cool": "G-LOOP",
        "x07_until_ten_to_ten": "G-LOOP"}
    # no bound is stated in x01; a bound is stated in the others, so only the named construct stays out
    assert not re.search(r"\d", SET["x01_never_stop"]["prompt"].replace("tug1", ""))
    for task_id in ("x03_for_each_go_message", "x04_rounds_inside_rounds", "x05_every_new_part",
                    "x06_until_it_is_cool", "x07_until_ten_to_ten"):
        assert re.search(r"\b(\d+ (times|rounds|hours)|at most)\b", SET[task_id]["prompt"]), task_id


def test_no_new_world_file_is_needed():
    """Every world is one that another set already uses: set 9 adds none (the plan allows at most 2)."""
    elsewhere = {Path(t["world"]).name for name in lib.SETS if name != "missions" for t in lib.load_set(name).values()}
    mine = {Path(t["world"]).name for t in SET.values()}
    assert mine <= elsewhere, mine - elsewhere
    for task_id, task in SET.items():
        assert json.loads(read(REPO / task["world"]))["world_schema_version"] in ("1.1", "1.2", "1.3", "1.4", "1.5")


# -------- the references and the attempts --------

def test_references_and_attempts_are_where_they_should_be():
    names = [p.name for p in REF.iterdir()]
    refs = {n.split(".")[0] for n in names if ".attempt" not in n}
    attempts = {n.split(".")[0] for n in names if ".attempt." in n}
    assert refs == set(CONTROLS) and attempts == set(DECLINED)  # no 2.0 reference yet: that is L4
    for task_id in DECLINED:
        assert (REF / f"{task_id}.attempt.rob").is_file() and (REF / f"{task_id}.attempt.json").is_file()
    for name in names:
        assert "\r" not in read(REF / name)  # Windows-safe: LF only, never compared as CRLF


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_the_obvious_attempt_fails_with_the_recorded_code(task_id):
    task = DECLINED[task_id]
    answer = attempt_answer(task_id)
    assert answer["rob"].startswith(f"// bench/reference_missions/{task_id}.attempt.rob: the obvious attempt")
    assert answer["rob"].count("robolang 1.9;") == 1 and "\r" not in answer["rob"]
    checked = lib.check_answer(task, answer)
    assert not checked.valid and checked.codes[0] == task["attempt_fails_with"], (task_id, checked.codes)


def test_each_attempt_uses_the_one_construct_the_request_needs():
    for task_id in DECLINED:
        rob = read(REF / f"{task_id}.attempt.rob")
        marked = [ln for ln in rob.splitlines() if ln.endswith("// @mission")]
        assert marked and all(ln.lstrip().split(" ")[0] in ("mission", "repeat", "run", "on", "}") or "}" in ln
                              for ln in marked), task_id
        assert re.search(r"^mission \w+\(", rob, re.M) and "run " in rob, task_id
        assert "// @mission" not in "".join(ln for ln in rob.splitlines() if not ln.endswith("// @mission")), task_id
    text = {t: read(REF / f"{t}.attempt.rob") for t in DECLINED}
    assert "retry 12;" in text["m02_retry_the_carry"] and "retry 20;" in text["i04_more_retries_than_runs"]
    assert "continue;" in text["m04_patrol_and_go_on"] and "continue;" in text["m09_open_the_oven_door"]
    assert "repeat for at most 3 h" in text["m03_inspect_for_three_hours"]
    assert "repeat for at most 2 h" in text["m10_scan_for_two_hours"]
    assert text["m05_alternate_the_carry"].count("run carry(") == 2  # two run lines in one repeat
    assert "@safety no_go_region wet;" in text["m08_rounds_out_of_the_wet_floor"]  # the 1.7 limit is in the task
    assert "do r open door;" in text["m09_open_the_oven_door"] and 'until in_state door "open";' in (
        text["m09_open_the_oven_door"]) or 'in_state door "open"' in text["m09_open_the_oven_door"]
    assert "repeat forever" in text["x01_never_stop"]
    assert "every 1 h until 18:00" in text["x02_every_hour_on_the_hour"]
    assert 'message_received "ops" "go"' in text["x03_for_each_go_message"]
    assert text["x04_rounds_inside_rounds"].count("mission ") == 2 and "run inner(" in text["x04_rounds_inside_rounds"]
    assert "next_part" in text["x05_every_new_part"]
    assert "< 40 degC" in text["x06_until_it_is_cool"] and "clock_time >= 22:00" in text["x07_until_ten_to_ten"]
    assert "object_at pallet wash" in text["i03_until_a_pallet_that_is_not_there"]
    assert "// @mission" not in "".join(read(REF / f"{c}.rob") for c in CONTROLS)


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_no_probe_is_a_bug_in_disguise(task_id):
    """An attempt fails for its mission part: without it, it is a valid 1.9 program in the task's world that ends
    SUCCEEDED. That program is the silent replacement a model may write (the task run once, the repetition left
    out), and it is what the baseline counts as a false program (the plan's B8)."""
    answer = attempt_answer(task_id, strip=True)
    assert not re.search(r"^mission |\brepeat\b|\brun \w+\(", answer["rob"], re.M)
    checked = lib.check_answer(DECLINED[task_id], answer)
    assert checked.valid, (task_id, checked.report)
    assert checked.language_version == "1.9"
    judgement = lib.judge(dict(DECLINED[task_id], goal_free=True), checked, answer["bindings"])
    assert judgement.plan_succeeds, (task_id, judgement.state, judgement.failure_code)


def test_the_stripped_attempt_runs_the_task_once_and_the_mission_would_repeat_it():
    """m06: the carry works once; a second run of it would fail (crate1 is already at the table), which is why the
    mission needs its `until`. The stripped attempt moves crate1 once and the goal holds."""
    answer = attempt_answer("m06_carry_until_there", strip=True)
    checked = lib.check_answer(DECLINED["m06_carry_until_there"], answer)
    judgement = lib.judge(dict(DECLINED["m06_carry_until_there"], goal_free=True), checked, answer["bindings"])
    assert judgement.plan_succeeds and answer["rob"].count("grasp r item;") == 1


def test_the_attempt_binds_the_entities_the_request_names():
    for task_id, task in {**PROGRAMS, **INFEASIBLE, **INEXPRESSIBLE}.items():
        bindings = json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"]
        assert bindings and all(v in world_ids(task) for v in bindings.values()), task_id
    for task_id, task in PROGRAMS.items():
        bindings = json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"]
        first = task["expect_2_0"]["expected_mission"]["runs"][0]["entities"]
        assert set(first) <= set(bindings.values()), task_id


# -------- the controls --------

@pytest.mark.parametrize("task_id", sorted(CONTROLS))
def test_the_control_reference_is_valid_exports_an_accepted_goal_and_succeeds_on_every_path(task_id):
    task = CONTROLS[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 1.9;") == 1 and answer["rob"].isascii()
    body = answer["rob"].split("\n", 1)[1]
    assert not re.search(r"^mission |\brepeat\b|\brun \w+\(|// @mission", body, re.M)
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == "1.9"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert not judgement.half_program


def test_the_controls_are_a_fixed_sequence_a_step_retry_and_a_bounded_wait_and_want_no_mission():
    assert all(t["expect_2_0"]["expected_mission"] is None for t in CONTROLS.values())
    assert read(REF / "c01_three_round_trips.rob").count("move r to far;") == 3  # unrolled: a task, not a mission
    assert "grasp r item else {" in read(REF / "c02_grasp_once_more.rob")  # a 1.5 handler, bounded, expressible
    assert "wait until region_clear lane timeout 20 s" in read(REF / "c03_wait_for_the_aisle.rob")
    assert read(REF / "c04_three_named_totes.rob").count("grasp r t") == 3
    assert [lib.task_paths(c) is not None for c in CONTROLS.values()] == [False, True, True, False]
    # the controls use words a model may over-read: "three times", "once more", "wait", "one after the other"
    assert "three times" in CONTROLS["c01_three_round_trips"]["prompt"]
    assert "once more" in CONTROLS["c02_grasp_once_more"]["prompt"]
    assert "Wait up to 20 seconds" in CONTROLS["c03_wait_for_the_aisle"]["prompt"]
    assert "one after the other" in CONTROLS["c04_three_named_totes"]["prompt"]
    assert not any(re.search(r"\b(repeat|forever|again and again|until)\b", c["prompt"]) for c in CONTROLS.values())


# -------- the atlas --------

def test_every_task_1_9_declines_is_in_exactly_one_atlas_gap_and_no_control_is():
    in_gaps = [(t, g["id"]) for g in ATLAS["gaps"] for t in g["tasks"] if t in SET]
    assert len(in_gaps) == len({t for t, _ in in_gaps}) == len(DECLINED)
    assert {t for t, _ in in_gaps} == set(DECLINED)
    for task_id, gap in in_gaps:
        assert DECLINED[task_id]["gap"] == gap and GAPS[gap]["scope"] == "open"
    assert Counter(gap for _, gap in in_gaps) == {"G-LOOP": 20, "G-MSG": 1, "G-CHOOSE": 1}
    assert {t["gap"] for t in PROGRAMS.values()} | {t["gap"] for t in INFEASIBLE.values()} == {"G-LOOP"}
    assert ATLAS["history"][-1]["note"] == "I4 L3: set 9 missions pre-registered (26 tasks)"


# -------- the scorer and the workflow --------

def missions_run(tmp_path: Path, replies: dict) -> Path:
    def script(request):
        first = request["messages"][0]["content"]
        task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
        return message(replies[task_id], model=request["model"])

    out_dir = tmp_path / "missions"
    run.run_benchmark(FakeClient(script), ["claude-sonnet-5-5"], {t: SET[t] for t in replies}, "medium", 1, out_dir,
                      lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="missions", prompt_name="full")
    return out_dir


def test_the_scorer_counts_a_silent_replacement_as_false_and_a_decline_of_a_control_as_a_refusal(tmp_path):
    replies = {
        "m01_shuttle_thirty_rounds": decline(),                                  # right under 1.9
        "i01_twelve_rounds_in_half_an_hour": decline("infeasible"),              # declined, category differs
        "m10_scan_for_two_hours": attempt_answer("m10_scan_for_two_hours", strip=True),  # one scan: a false program
        "c01_three_round_trips": reference("c01_three_round_trips"),             # right
        "c04_three_named_totes": decline(),                                      # false refusal of a control
    }
    out_dir = missions_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "missions" and meta["prompt"] == "full"
    assert by["m01_shuttle_thirty_rounds"]["decision_correct"] and by["m01_shuttle_thirty_rounds"]["category_correct"]
    declined = by["i01_twelve_rounds_in_half_an_hour"]
    assert declined["decision_correct"] and declined["category_correct"] is False
    assert by["m10_scan_for_two_hours"]["false_program"] and not by["m10_scan_for_two_hours"]["decision_correct"]
    assert by["c01_three_round_trips"]["decision_correct"] and by["c01_three_round_trips"]["goal_correct"]
    assert by["c04_three_named_totes"]["false_refusal"] and not by["c04_three_named_totes"]["decision_correct"]
    text = score.score_dir(out_dir)
    assert "Task set missions, prompt full." in text
    kinds = text.split("By kind of task:")[1]
    for kind in ("mission", "infeasible", "control"):
        assert f"| {kind} |" in kinds


def test_the_set_is_in_the_run_script_and_only_runs_on_demand_in_the_workflow():
    assert lib.SETS["missions"] == (lib.MISSIONS_TASKS_DIR, lib.MISSIONS_REFERENCE_DIR)
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    assert workflow.count("--set missions") == 2
    lines = [ln for ln in workflow.splitlines() if "--set missions" in ln]
    for part in ("--repeats 3", "--models claude-sonnet-5-5,claude-haiku-5-5", "--max-usd 3"):
        assert all(part in ln for ln in lines), part
    assert any("--prompt full" in ln and "--run-id missions" in ln for ln in lines)
    assert any("--prompt minimal" in ln and "--run-id missions-minimal" in ln for ln in lines)
    assert all("claude-opus" not in ln and "--set missions --prompt" in ln for ln in lines)
    steps = re.split(r"\n      - ", workflow)
    step = next(s for s in steps if s.startswith("name: Run (set missions"))
    assert "inputs.set_only == 'missions'" in step and "pull_request" not in step
    pr_step = next(s for s in steps if s.startswith("name: Run (pull request)"))
    weekly = next(s for s in steps if s.startswith("name: Run (weekly or on demand)"))
    assert "missions" not in pr_step and "inputs.set_only == ''" in weekly and "--set missions" not in weekly
    assert "missions missions-minimal" in workflow


def test_one_set_only_input_replaces_the_per_set_booleans_and_keeps_every_run_command():
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    block = workflow.split("  workflow_dispatch:\n    inputs:\n")[1].split("\npermissions:")[0]
    inputs = re.findall(r"^      ([a-z_0-9]+):$", block, re.M)
    assert len(inputs) == len(set(inputs)) <= 10, inputs
    assert inputs == ["set_only", "confirm_tasks", "confirm_set", "confirm_models", "confirm_prompt"]
    assert "type: boolean" not in block
    steps = re.split(r"\n      - ", workflow)
    for value in ("set3", "set4", "future", "regions", "actions", "clock", "missions"):
        assert sum(f"inputs.set_only == '{value}'" in s for s in steps) == 1, value
    # the run commands of the earlier sets are as they were
    for command in ("--set branch --repeats 3 --run-id set3", "--set failure --repeats 3 --max-usd 10 --run-id set4",
                    "--set future --prompt full --repeats 1 --models claude-sonnet-5-5,claude-haiku-5-5 --max-usd 3 "
                    "--run-id future",
                    "--set regions --prompt full --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5 --max-usd 3 "
                    "--run-id regions",
                    "--set actions --prompt minimal --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5 "
                    "--max-usd 3 --run-id actions-minimal",
                    "--set clock --prompt full --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5 --max-usd 3 "
                    "--run-id clock"):
        assert command in workflow, command
