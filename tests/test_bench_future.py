"""
Benchmark set "future" (bench/tasks_future/, RSI-01 step R3, docs/evidence/rsi-r3-gap-verification.md): probes of the
atlas's near language gaps.

25 `inexpressible` probes, each needing exactly one open gap that a near activity claims, with an obvious attempt
that RoboLang 1.6 rejects with a named diagnostic (LE-7). 8 `control` requests from near activities that 1.6 can
express, so the set also measures false refusals; three of them record a refuted claim of the atlas. No network and
no SDK; the model run is the `Benchmark` workflow with `set_only: future`.

RoboLang 1.7 (decision 0039, errata docs/evidence/bench-errata-1-7.md) relabelled the three `G-PATH` probes as programs
(group `region`): `expect` is the 1.7 label, `expect_1_6` keeps the 1.6 one, and their attempts are replaced by 1.7
references. So 22 probes remain `inexpressible`.

RoboLang 1.8 (decision 0040, errata docs/evidence/bench-errata-1-8.md) relabelled the three `G-ACT` probes as programs
(group `action`): `expect` is the 1.8 label, `expect_1_7` keeps the 1.7 one, their attempts are replaced by 1.8
references, and their worlds are world file 1.4 copies. So 19 probes remain `inexpressible`.
"""

from __future__ import annotations
import io
import json
import re
from collections import Counter
from pathlib import Path

import pytest
from test_bench import FakeClient, message

from bench import atlas_report, lib, run, score

REPO = Path(__file__).resolve().parent.parent
SET = lib.load_set("future")
REF = lib.FUTURE_REFERENCE_DIR
ATLAS = json.loads((REPO / "bench" / "atlas.json").read_text(encoding="utf-8"))
GAPS = {g["id"]: g for g in ATLAS["gaps"]}
ACTIVITIES = {a["id"]: a for a in ATLAS["activities"]}
CANNOT = {"decision": "cannot", "category": "inexpressible"}
PROBES = {t: d for t, d in SET.items() if d["group"] == "inexpressible"}
REGIONS = {t: d for t, d in SET.items() if d["group"] == "region"}  # 1.7: programs with a region limit (0039)
ACTIONS = {t: d for t, d in SET.items() if d["group"] == "action"}  # 1.8: programs with declared actions (0040)
CLOCKS = {t: d for t, d in SET.items() if d["group"] == "clock"}  # 1.9: programs that read the clock (0042)
CONTROLS = {t: d for t, d in SET.items() if d["group"] == "control"}
# R3 refutations: (activity, gap) -> the control whose reference program expresses the claim
REFUTED = {("ACT-HOS-001", "G-PERCEIVE"): "c_fries_too_dark", ("ACT-SOL-001", "G-PERCEIVE"): "c_table_out_of_tolerance"}
# withdrawn after the model run: it held only for parts the world names; the task stays as a closed-world control
WITHDRAWN = {("ACT-MFG-001", "G-CHOOSE"): "c_parts_left_in_container"}
NEW_WORLDS = ["fry_station", "fulfilment", "site", "ward"]


def reference(task_id: str) -> dict:
    ref = json.loads((REF / f"{task_id}.json").read_text(encoding="utf-8"))
    rob = (REF / f"{task_id}.rob").read_text(encoding="utf-8")
    return {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob, **ref}


def attempt_answer(task_id: str) -> dict:
    """The obvious attempt at a probe, as a program answer (`.attempt.json` only exists when the attempt parses)."""
    stem = REF / f"{task_id}.attempt"
    bindings = Path(f"{stem}.json")
    ref = json.loads(bindings.read_text(encoding="utf-8")) if bindings.is_file() else {"task": "x", "bindings": {}}
    return {"decision": "program", "category": "none", "reason": "an attempt",
            "rob": Path(f"{stem}.rob").read_text(encoding="utf-8"), **ref}


def decline(category: str = "inexpressible") -> dict:
    return {"decision": "cannot", "category": category, "reason": "cannot be done", "rob": "", "task": "",
            "bindings": []}


# -------- the set --------

def test_the_mix_of_tasks():
    assert len(SET) == 33
    assert Counter(d["group"] for d in SET.values()) == {"inexpressible": 16, "control": 8, "region": 3,
                                                           "action": 3, "clock": 3}
    assert set(REGIONS) == {"f_path_avoid_wet_floor", "f_path_stay_in_zone", "f_path_keep_off_road"}
    assert set(ACTIONS) == {"f_act_set_fryer", "f_act_open_closet", "f_act_print_layout"}
    assert set(CLOCKS) == {"f_clock_scan_stamp", "f_clock_shift_cutoff", "f_clock_clean_at_2300"}
    # probes per gap: change on purpose (R3 evidence, docs/evidence/rsi-r3-gap-verification.md)
    assert Counter(d["gap"] for d in PROBES.values()) == {
        "G-CALC": 3, "G-MSG": 3, "G-CONC": 2, "G-CHOOSE": 3, "G-PERCEIVE": 2,
        "G-SELF": 3}  # G-PATH, G-ACT and G-CLOCK: their probes are 1.7, 1.8 and 1.9 programs now
    for task_id, task in SET.items():
        assert task["id"] == task_id and (lib.FUTURE_TASKS_DIR / f"{task_id}.json").is_file()
        assert (REPO / task["world"]).is_file() and task["tags"] and task["prompt"].endswith(".")
        assert task["prompt"].isascii()


def test_every_task_names_a_near_activity_and_every_probe_an_open_gap_that_activity_claims():
    for task_id, task in SET.items():
        activity = ACTIVITIES[task["activity"]]
        assert atlas_report.is_near(activity), task_id
        if task["group"] == "control":
            assert "gap" not in task
            continue
        gap = GAPS[task["gap"]]
        if task["group"] in ("region", "action", "clock"):  # 1.7-1.9 programs: gaps closed at L6, left every `needs`
            assert gap["scope"] == "closed" and task_id not in gap["tasks"], task_id
            continue
        assert gap["scope"] == "open" and task["gap"] != "G-LOOP", task_id
        assert task["gap"] in activity["needs"], f"{task_id}: {task['activity']} does not claim {task['gap']}"
        # a probe is in the atlas under its gap; a relabelled one (a 1.7 program, decision 0039) left it
        assert (task_id in gap["tasks"]) == (task["group"] == "inexpressible"), task_id
    # each of the nine gaps has probes from the activities that claim it, in the atlas, and nowhere else
    in_atlas = {t for g in GAPS.values() for t in g["tasks"]}
    assert set(PROBES) <= in_atlas and not (set(CONTROLS) | set(REGIONS) | set(ACTIONS) | set(CLOCKS)) & in_atlas


def test_every_gap_a_near_activity_claims_is_probed_except_the_proven_loop():
    claimed = {g for a in ATLAS["activities"] if atlas_report.is_near(a) for g in a["needs"]
               if GAPS[g]["scope"] == "open" and g != "G-LOOP"}  # G-LOOP: 12 standing tasks prove it; reopened by 0041
    assert claimed == {t["gap"] for t in PROBES.values()}
    assert {t["gap"] for t in CLOCKS.values()} == {"G-CLOCK"} and GAPS["G-CLOCK"]["scope"] == "closed"  # L6 of I3
    assert {t["gap"] for t in ACTIONS.values()} == {"G-ACT"} and GAPS["G-ACT"]["scope"] == "closed"  # L6 of I2
    assert {t["gap"] for t in REGIONS.values()} == {"G-PATH"} and GAPS["G-PATH"]["scope"] == "closed"
    assert {g for g, v in GAPS.items() if v["scope"] == "out"} == {"G-FALLBACK", "G-UNBOUNDED"}  # 0041 M2/M7
    assert GAPS["G-LOOP"]["scope"] == "closed" and GAPS["G-LOOP"]["closed_in"] == "RoboLang 2.0"  # L6 of I4


def test_refuted_claims_are_removed_from_needs_and_recorded_with_a_control():
    for (activity, gap), control in REFUTED.items():
        act = ACTIVITIES[activity]
        assert gap not in act["needs"], (activity, gap)
        assert f"R3 (2026-11-17): {gap} refuted, 1.6 expresses it with" in act["basis"], activity
        assert CONTROLS[control]["activity"] == activity
    for (activity, gap), control in WITHDRAWN.items():
        act = ACTIVITIES[activity]
        assert gap in act["needs"] and "refutation is withdrawn" in act["basis"], (activity, gap)
        assert CONTROLS[control]["activity"] == activity
    # a refuted claim has no probe: a probe for it would have been a control (and every probe's pair is still claimed)
    assert not {(t["activity"], t["gap"]) for t in PROBES.values()} & set(REFUTED)


def test_references_and_attempts_are_where_they_should_be():
    names = [p.name for p in REF.iterdir()]
    refs = {n.split(".")[0] for n in names if n.endswith((".rob", ".json")) and ".attempt" not in n}
    attempts = {n.split(".")[0] for n in names if ".attempt." in n}
    programs = set(CONTROLS) | set(REGIONS) | set(ACTIONS) | set(CLOCKS)
    assert refs == programs and attempts == set(PROBES)
    assert {p.stem for p in REF.glob("*.rob") if ".attempt" not in p.stem} == programs
    assert {p.stem for p in REF.glob("*.json") if ".attempt" not in p.stem} == programs
    assert {p.name for p in REF.glob("*.attempt.*")} >= {f"{t}.attempt.rob" for t in PROBES}
    assert {p.name for p in REF.glob("*.attempt.json")} <= {f"{t}.attempt.json" for t in PROBES}
    for name in names:
        assert "\r" not in (REF / name).read_text(encoding="utf-8")  # Windows-safe: LF only, never compared as CRLF


def test_expectations_are_well_formed():
    for task_id, task in PROBES.items():
        assert task["expect"] == CANNOT and lib.expectation(task) == ("cannot", "inexpressible")
        assert not {"expected_goal", "expected_goals", "expect_1_6"} & set(task)
        assert len(task["why"]) > 40 and re.fullmatch(r"[ES]\d{3}", task["attempt_fails_with"]), task_id
        assert "language reference" in task["why"] or "reserved" in task["why"], task_id  # cites the yardstick
    for task_id, task in REGIONS.items():  # decision 0039: a program with one region limit, on a world file 1.3
        assert task["expect"] == {"decision": "program"} and task["expect_1_6"] == CANNOT, task_id
        assert lib.expectation(task) == ("program", "none") and len(task["expected_limits"]) == 1
        assert task["expect_1_7"]["decision"] == "program" and len(lib.task_paths(task)) == 1
        assert task["world"] == "bench/worlds/site_1_3.json" and len(task["why"]) > 40
        assert not {"attempt_fails_with", "expected_goal"} & set(task)
    for task_id, task in ACTIONS.items():  # decision 0040: a program with its declared actions, on a world file 1.4
        assert task["expect"] == {"decision": "program"} and task["expect_1_7"] == CANNOT, task_id
        assert lib.expectation(task) == ("program", "none") and task["expect_1_8"]["decision"] == "program"
        assert 1 <= len(task["expected_actions"]) <= 1 and lib.task_paths(task) == task["expect_1_8"]["paths"]
        assert json.loads((REPO / task["world"]).read_text(encoding="utf-8"))["world_schema_version"] == "1.4"
        assert len(task["why"]) > 40 and not {"attempt_fails_with", "expected_goal"} & set(task)
    for task_id, task in CLOCKS.items():  # decision 0042: a program that reads the clock, on a world file 1.5
        assert task["expect"] == {"decision": "program"} and task["expect_1_8"] == CANNOT, task_id
        assert lib.expectation(task) == ("program", "none") and task["expect_1_9"]["decision"] == "program"
        assert task["expected_clock"] and lib.task_paths(task) == task["expect_1_9"]["paths"]
        assert task["gap"] == "G-CLOCK"
        assert json.loads((REPO / task["world"]).read_text(encoding="utf-8"))["world_schema_version"] == "1.5"
        assert len(task["why"]) > 40 and not {"attempt_fails_with", "expected_goal"} & set(task)
    for task_id, task in CONTROLS.items():
        assert lib.expectation(task) == ("program", "none") and lib.expected_goals(task), task_id
        assert not {"why", "attempt_fails_with", "gap"} & set(task)
        if "expect_1_6" in task:
            assert task["expect_1_6"]["decision"] == "program" and lib.task_paths(task)
            for path in lib.task_paths(task):
                assert path["when"] and "world_patch" in path
                assert sum(k in path for k in ("goal", "goals", "fails", "fail")) == 1, path


# -------- the probes --------

def test_every_attempt_fails_with_the_recorded_code():
    """The obvious attempt is rejected by `check_answer`; the first diagnostic code is the one recorded."""
    for task_id, task in PROBES.items():
        answer = attempt_answer(task_id)
        assert answer["rob"].startswith(f"// bench/reference_future/{task_id}.attempt.rob: the obvious attempt")
        assert "robolang 1.6;" in answer["rob"]
        checked = lib.check_answer(task, answer)
        assert not checked.valid, task_id
        assert checked.codes[0] == task["attempt_fails_with"], (task_id, checked.codes)


def test_no_probe_is_a_bug_in_disguise():
    """An attempt fails for the construct it uses, not for a slip in the rest of the program: with its one offending
    expression made harmless, it is a valid 1.6 program in the probe's world and binds every name it uses."""
    stand_ins = {  # probe: (offending text, harmless text, bindings)
        "f_calc_total_weight": ("ma + mb", "ma", {"r": "digit1", "a": "tote_1", "b": "tote_2", "scale": "scale_digit1",
                                                  "src": "inbound_conveyor"}),
        "f_calc_over_limit": ("m - 20 kg", "m", {"r": "stretch1", "c": "case_1", "scale": "scale_stretch1",
                                                 "src": "trailer_4"}),
        "f_calc_text_line": ('"case_2 weighs " + m + " kg"', "m", {"r": "stretch1", "c": "case_2",
                                                                    "scale": "scale_stretch1", "src": "trailer_4"}),
        "f_self_report_charge": ("battery_level", '"low"', {"r": "spot1", "spot": "stop_b"}),
    }
    for task_id, (bad, good, bindings) in stand_ins.items():
        answer = attempt_answer(task_id)
        assert bad in answer["rob"], task_id
        answer["rob"] = answer["rob"].replace(bad, good)
        answer["task"] = re.search(r"^task (\w+)\(", answer["rob"], re.M).group(1)
        answer["bindings"] = bindings
        checked = lib.check_answer(PROBES[task_id], answer)
        assert checked.valid, (task_id, checked.report)


@pytest.mark.parametrize("task_id", sorted(ACTIONS))
def test_the_1_8_reference_does_exactly_the_action_asked_on_every_path(task_id):
    """Decision 0040: the relabelled probe is a program with one declared action. Its reference is valid, right on
    every path, `actions_ok`; the same program with the `do` left out and its `in_state` kept fails the goal, and with
    both left out it is a program that succeeds without the action (`action_replaced`)."""
    task = ACTIONS[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 1.8;") == 1 and answer["rob"].isascii()
    checked = lib.check_answer(task, answer)
    assert checked.valid and checked.language_version == "1.8", checked.report
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds and judgement.actions_ok
    assert not judgement.action_replaced and all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths)
    no_do = dict(answer, rob=re.sub(r"^\s*do [^\n]*\n", "", answer["rob"], flags=re.M))
    missing = lib.judge(task, lib.check_answer(task, no_do), answer["bindings"])
    assert not missing.plan_succeeds and not missing.actions_ok and missing.goal_correct  # S301: the state is not there
    stand_in = {"f_act_set_fryer": ('in_state fryer "heating";', "robot_at r fryer;"),  # a `move` for the action
                "f_act_open_closet": ('in_state door "open"; ', ""),
                "f_act_print_layout": ('in_state wall "printed";', "robot_at r wall;")}[task_id]
    no_state = dict(no_do, rob=no_do["rob"].replace(*stand_in))
    replaced = lib.judge(task, lib.check_answer(task, no_state), answer["bindings"])
    assert replaced.plan_succeeds and not replaced.goal_correct and replaced.action_replaced and not replaced.actions_ok


@pytest.mark.parametrize("task_id", sorted(REGIONS))
def test_the_1_7_reference_declares_exactly_its_limit_and_succeeds(task_id):
    """Decision 0039: the relabelled probe is a program. Its reference is valid, reaches the goal, declares the one
    region limit the request asks for, and the same program without the limit is a half program (limit_dropped)."""
    task = REGIONS[task_id]
    answer = reference(task_id)
    checked = lib.check_answer(task, answer)
    assert checked.valid and checked.language_version == "1.7", checked.report
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds and judgement.limit_ok and not judgement.limit_dropped
    assert [p["goal_ok"] and p["plan_ok"] for p in judgement.paths] == [True]
    stripped = dict(answer, rob=re.sub(r"^\s*@safety [^\n]*\n", "", answer["rob"], flags=re.M))
    half = lib.judge(task, lib.check_answer(task, stripped), answer["bindings"])
    assert half.goal_correct and half.plan_succeeds and not half.limit_ok and half.limit_dropped


# -------- the controls --------

@pytest.mark.parametrize("task_id", sorted(CONTROLS))
def test_the_reference_is_valid_exports_an_accepted_goal_and_succeeds(task_id):
    task = CONTROLS[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 1.6;") == 1 and answer["rob"].isascii()
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == "1.6"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert judgement.silent_substitution in (None, False) and not judgement.half_program
    paths = lib.task_paths(task) or []
    assert [p["when"] for p in judgement.paths] == [p["when"] for p in paths]
    assert all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths), judgement.paths


def test_the_controls_use_what_the_atlas_says_they_refute():
    rob = {t: reference(t)["rob"] for t in CONTROLS}
    assert "measure r label of basket using cam as shade;" in rob["c_fries_too_dark"]
    assert 'shade == "too_dark"' in rob["c_fries_too_dark"]
    assert "measure r distance of t14 using laser as off;" in rob["c_table_out_of_tolerance"]
    assert "off <= 5 mm" in rob["c_table_out_of_tolerance"]
    assert rob["c_parts_left_in_container"].count("if object_at") == 3
    assert rob["c_retry_flag_carry_on"].count("else {") == 3 and "if holding r first" in rob["c_retry_flag_carry_on"]
    assert "region_clear lane timeout 30 s" in rob["c_wait_lane_clear"]
    assert "robot_ready" in rob["c_whichever_tugger_ready"]
    assert "sensor_ok cam" in rob["c_camera_fault_stop"] and '"2 totes moved"' in rob["c_count_two_totes"]


def test_a_control_ends_the_run_failed_only_where_the_request_says_stop():
    for task_id, count in (("c_wait_lane_clear", 1), ("c_whichever_tugger_ready", 1), ("c_camera_fault_stop", 1)):
        paths = lib.task_paths(CONTROLS[task_id])
        assert sum(bool(p.get("fails") or p.get("fail")) for p in paths) == count, task_id
    for task_id in ("c_fries_too_dark", "c_table_out_of_tolerance", "c_parts_left_in_container",
                    "c_retry_flag_carry_on", "c_count_two_totes"):
        assert not any(p.get("fails") or p.get("fail") for p in lib.task_paths(CONTROLS[task_id])), task_id


# -------- the worlds --------

def test_the_new_worlds_are_world_1_2_files_and_nothing_else_new_is_added():
    worlds = {p.stem for p in (REPO / "bench" / "worlds").glob("*.json") if ".notes" not in p.name}
    assert set(NEW_WORLDS) <= worlds
    used = {t["world"] for t in SET.values()}
    # site_1_3: a 0039 world copy; the 1_4 files: 0040 world copies
    assert {Path(w).stem for w in used} <= set(NEW_WORLDS) | {"warehouse", "site_1_3", "ward_1_4", "site_1_4",
                                                              "fry_station_hot_1_4", "warehouse_1_5_1420",
                                                              "fulfilment_1_5_2115", "site_1_5_2220"}  # 0042 copies
    for name in NEW_WORLDS:
        world = json.loads((REPO / f"bench/worlds/{name}.json").read_text(encoding="utf-8"))
        assert world["world_schema_version"] == "1.2" and world["tasks"] == {}
        shown = lib.describe_world(world)
        assert "properties" not in shown and "too_dark" not in shown and "WIND" not in shown
        assert not (REPO / f"bench/worlds/{name}.notes.json").exists()


def test_the_worlds_hold_what_the_controls_and_probes_rely_on():
    fulfilment = json.loads((REPO / "bench/worlds/fulfilment.json").read_text(encoding="utf-8"))
    assert {"tugger_a", "tugger_b"} <= set(fulfilment["robots"])
    tuggers = [fulfilment["robots"][t] for t in ("tugger_a", "tugger_b")]
    assert all(t["ready"] and not t["gripper"] for t in tuggers)
    assert fulfilment["regions"]["worker_lane"]["clear"] is False
    assert [e["at"]["value"] for e in fulfilment["events"] if e["target"] == "worker_lane"] == ["10"]
    site = json.loads((REPO / "bench/worlds/site.json").read_text(encoding="utf-8"))
    assert site["sensors"]["laser_installer1"]["measures"]["distance"]["unit"] == "mm"
    assert "label" in site["sensors"]["cam_weeder1"]["measures"]
    assert site["objects"]["table_14"]["properties"]["distance"] == {"value": "3", "unit": "mm"}
    fry = json.loads((REPO / "bench/worlds/fry_station.json").read_text(encoding="utf-8"))
    assert "label" in fry["sensors"]["cam_flippy1"]["measures"]
    assert fry["objects"]["basket_1"]["properties"]["label"] == {"value": "too_dark"}
    # no sensor in any of the four worlds measures anything but a kind the language reads
    for name in NEW_WORLDS:
        world = json.loads((REPO / f"bench/worlds/{name}.json").read_text(encoding="utf-8"))
        for sensor in world["sensors"].values():
            assert set(sensor.get("measures", {})) <= {"temperature", "mass", "distance", "label"}


# -------- the scorer and the workflow --------

def future_run(tmp_path: Path, replies: dict) -> Path:
    def script(request):
        first = request["messages"][0]["content"]
        task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
        return message(replies[task_id], model=request["model"])

    out_dir = tmp_path / "future"
    run.run_benchmark(FakeClient(script), ["claude-sonnet-5-5"], {t: SET[t] for t in replies}, "medium", 1, out_dir,
                      lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="future", prompt_name="full")
    return out_dir


def test_the_scorer_counts_a_program_for_a_probe_as_false_and_a_decline_for_a_control_as_a_refusal(tmp_path):
    sequential = attempt_answer("f_conc_carry_and_drive")
    sequential["rob"] = sequential["rob"].replace("parallel {", "{")  # still not valid: the point is a program at all
    replies = {
        "f_msg_nurse_confirm": decline(),                          # right
        "f_calc_total_weight": decline("infeasible"),              # declined, wrong category
        "f_conc_carry_and_drive": sequential,                      # a program for a probe: false program
        "c_count_two_totes": reference("c_count_two_totes"),       # right
        "c_fries_too_dark": decline(),                             # false refusal of a control
    }
    out_dir = future_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "future" and meta["prompt"] == "full"
    assert by["f_msg_nurse_confirm"]["decision_correct"] and by["f_msg_nurse_confirm"]["category_correct"] is True
    assert by["f_calc_total_weight"]["decision_correct"] and by["f_calc_total_weight"]["category_correct"] is False
    assert by["f_conc_carry_and_drive"]["false_program"] and not by["f_conc_carry_and_drive"]["decision_correct"]
    assert by["c_count_two_totes"]["decision_correct"] and by["c_count_two_totes"]["goal_correct"]
    assert by["c_fries_too_dark"]["false_refusal"] and not by["c_fries_too_dark"]["decision_correct"]
    text = score.score_dir(out_dir)
    assert "Task set future, prompt full." in text
    kinds = text.split("By kind of task:")[1]
    assert "| inexpressible |" in kinds and "| control |" in kinds


def test_the_set_is_in_the_run_script_and_only_runs_on_demand_in_the_workflow():
    assert lib.SETS["future"] == (lib.FUTURE_TASKS_DIR, lib.FUTURE_REFERENCE_DIR)
    workflow = (REPO / ".github" / "workflows" / "benchmark.yml").read_text(encoding="utf-8")
    assert workflow.count("--set future") == 1
    line = next(ln for ln in workflow.splitlines() if "--set future" in ln)
    for part in ("--prompt full", "--repeats 1", "--models claude-sonnet-5-5,claude-haiku-5-5", "--max-usd 3"):
        assert part in line, part
    assert "claude-opus" not in line and "--run-id future" in line
    steps = re.split(r"\n      - ", workflow)
    future_step = next(s for s in steps if "--set future" in s)
    assert "inputs.set_only == 'future'" in future_step and "pull_request" not in future_step
    # the per-pull-request step and the weekly step never run it
    pr_step = next(s for s in steps if s.startswith("name: Run (pull request)"))
    weekly = next(s for s in steps if s.startswith("name: Run (weekly or on demand)"))
    assert "future" not in pr_step and "inputs.set_only == ''" in weekly and "--set future" not in weekly
    assert re.search(r"      set_only:\n        description: .*\n        type: string\n        default: \"\"", workflow)


@pytest.mark.parametrize("task_id", sorted(CLOCKS))
def test_the_1_9_reference_reads_the_clock_exactly_as_asked_on_every_path(task_id):
    """Decision 0042: the relabelled probe is a program with its clock use. Its reference is valid, right on every path
    and `clock_ok`; the same program with the time left out (a fixed message, a guessed `wait for`, nothing) is a
    program that succeeds without it (`time_replaced`)."""
    task = CLOCKS[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 1.9;") == 1 and answer["rob"].isascii()
    checked = lib.check_answer(task, answer)
    assert checked.valid and checked.language_version == "1.9", checked.report
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds and judgement.clock_ok and not judgement.time_replaced
    assert all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths)
    swaps = {  # task: the clock use replaced by a fixed text, or a guessed `wait for` (the cutoff `if` has no
        # such stand-in: test_bench_clock.py scores a program with its time test dropped)
        "f_clock_scan_stamp": (("measure r time as done_at;\n", ""), ("with done_at;", 'with "14:20";')),
        "f_clock_clean_at_2300": (('wait until clock_time >= 23:00 timeout 120 min else { fail "11 pm did not come in '
                                   'time"; }', "wait for 40 min;"),)}
    if task_id not in swaps:
        return
    stripped = answer["rob"]
    for old, new in swaps[task_id]:
        assert old in stripped, (task_id, old)
        stripped = stripped.replace(old, new)
    replaced = lib.judge(task, lib.check_answer(task, dict(answer, rob=stripped)), answer["bindings"])
    assert replaced.state == "SUCCEEDED" and replaced.time_replaced and not replaced.clock_ok
