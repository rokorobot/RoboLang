"""
Benchmark set 8, "clock" (bench/tasks_clock/, decision record 0042, plan docs/plans/bench-8-clock.md): clock readings
(`measure r time as t;`, `measure r elapsed as d;`, `HH:MM[:SS]` literals, `clock_time` / `clock_elapsed` in
`wait until`), pre-registered against RoboLang 1.8 before any code (L3 of iteration I3). `expect` is the 1.8 label (the
baseline), `expect_1_9` the label for acceptance.

10 `clock` requests that become programs in 1.9, 5 `infeasible` in 1.9 (0042 T4, T5, T7), 5 `inexpressible` that stay
out (0042 T4, T11; G-CLOCK, G-LOOP, G-CALC), 4 `control` requests that are programs in both versions. 1.8 declines every
non-control task, and every one has an obvious attempt that 1.8 rejects (`E101`: a time literal, or `E217`: the kind
`time` / `elapsed`). Only the controls have a 1.8 reference; the 1.9 references wait for L4. No network and no SDK; the
model run is the `Benchmark` workflow with `clock: true`.
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
SET = lib.load_set("clock")
REF = lib.CLOCK_REFERENCE_DIR
ATLAS = json.loads((REPO / "bench" / "atlas.json").read_text(encoding="utf-8"))
GAPS = {g["id"]: g for g in ATLAS["gaps"]}
CANNOT = {"decision": "cannot", "category": "inexpressible"}
PROGRAMS = {t: d for t, d in SET.items() if d["group"] == "clock"}
INFEASIBLE = {t: d for t, d in SET.items() if d["group"] == "infeasible"}
INEXPRESSIBLE = {t: d for t, d in SET.items() if d["group"] == "inexpressible"}
CONTROLS = {t: d for t, d in SET.items() if d["group"] == "control"}
DECLINED = {**PROGRAMS, **INFEASIBLE, **INEXPRESSIBLE}  # what 1.8 must decline: an attempt each, a gap each
CLOCK_LINE = re.compile(r"^[^\n]*// @clock\n", re.M)  # an attempt marks every line that is the clock part
WORLDS_NOT_1_5 = ("1.1", "1.2", "1.3", "1.4")


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def reference(task_id: str) -> dict:
    ref = json.loads(read(REF / f"{task_id}.json"))
    rob = read(REF / f"{task_id}.rob")
    return {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob, **ref}


def attempt_answer(task_id: str, strip: bool = False) -> dict:
    """The obvious attempt as a program answer; `strip` removes its clock part (the silent replacement a model may
    write in 1.8: the same program with the time condition, the wait for a time or the reading left out)."""
    stem = REF / f"{task_id}.attempt"
    rob = read(Path(f"{stem}.rob"))
    if strip:
        rob = CLOCK_LINE.sub("", rob)
    return {"decision": "program", "category": "none", "reason": "an attempt", "entities": [], "rob": rob,
            **json.loads(read(Path(f"{stem}.json")))}


def decline(category: str = "inexpressible") -> dict:
    return {"decision": "cannot", "category": category, "reason": "cannot be done", "rob": "", "task": "",
            "bindings": []}


# -------- the set --------

def test_the_mix_of_tasks():
    assert len(SET) == 24
    groups = Counter(d["group"] for d in SET.values())
    assert groups == {"clock": 10, "infeasible": 5, "inexpressible": 5, "control": 4}
    for task_id, task in SET.items():
        assert task["id"] == task_id and (lib.CLOCK_TASKS_DIR / f"{task_id}.json").is_file()
        assert (REPO / task["world"]).is_file() and task["tags"] and task["prompt"].endswith(".")
        assert task["prompt"].isascii() and "\r" not in read(lib.CLOCK_TASKS_DIR / f"{task_id}.json")
    prefixes = {"clock": "k", "infeasible": "i", "inexpressible": "x", "control": "c"}
    for task_id, task in SET.items():
        assert re.fullmatch(prefixes[task["group"]] + r"\d\d_[a-z0-9_]+", task_id), task_id


def test_the_ids_are_new_ones_and_no_prompt_is_a_copy_of_a_standing_task():
    elsewhere = {}
    for name in lib.SETS:
        if name != "clock":
            elsewhere.update(lib.load_set(name))
    assert not set(SET) & set(elsewhere)
    assert not {t["prompt"] for t in SET.values()} & {t["prompt"] for t in elsewhere.values()}


def test_labels_are_1_8_in_expect_and_1_9_in_expect_1_9():
    for task_id, task in SET.items():
        assert task["expect_1_9"]["decision"] in ("program", "cannot"), task_id
        if task["group"] == "control":
            assert task["expect"] == {"decision": "program"} and lib.expected_goals(task), task_id
            assert task["expect_1_9"] == {"decision": "program"} and task["expected_clock"] == []
            assert not {"why", "attempt_fails_with", "gap", "blockers"} & set(task)
            continue
        assert task["expect"] == CANNOT and lib.expectation(task) == ("cannot", "inexpressible"), task_id
        assert len(task["why"]) > 40 and task["attempt_fails_with"] in ("E101", "E217"), task_id
        assert not {"expected_goal", "expected_goals", "expect_1_8", "expect_1_7"} & set(task)
    for task_id, task in PROGRAMS.items():
        later = task["expect_1_9"]
        assert later["decision"] == "program" and later["paths"] and "category" not in later
        assert "world_1_5" in later and task["expected_clock"], task_id
        for path in later["paths"]:
            assert path["when"] and ("goal" in path or path.get("fails")) and "world_patch" in path, task_id
    for task_id, task in INFEASIBLE.items():
        later = task["expect_1_9"]
        assert later["decision"] == "cannot" and later["category"] == "infeasible", task_id
        assert later["because"] in ("0042 T4", "0042 T5", "0042 T7") and task["blockers"], task_id
    for task_id, task in INEXPRESSIBLE.items():
        later = task["expect_1_9"]
        assert {k: later[k] for k in ("decision", "category")} == CANNOT, task_id
        assert later["because"] in ("0042 T4", "0042 T11"), task_id
    reasons = sorted(t["expect_1_9"]["because"] for t in INFEASIBLE.values())
    assert reasons == ["0042 T4", "0042 T4", "0042 T5", "0042 T7", "0042 T7"]


def test_the_clock_tasks_cover_the_cases_the_pre_registration_lists():
    tags = Counter(tag for t in PROGRAMS.values() for tag in t["tags"])
    for tag in ("time_stamp", "cutoff", "bounded_wait", "elapsed", "window", "and", "two_robots", "declared_action",
                "region_limit", "decoy_12_hour", "near_miss_time", "wait_until"):
        assert tags[tag] >= 1, tag
    uses = Counter((c["reading"], c["use"]) for t in PROGRAMS.values() for c in t["expected_clock"])
    assert uses == {("time", "message"): 1, ("time", "if"): 4, ("time", "wait"): 3, ("elapsed", "message"): 2}
    assert len({t["world"] for t in PROGRAMS.values()}) >= 6
    # every path that depends on the time of day is forced by the start of the clock in its world patch
    for task_id, task in PROGRAMS.items():
        timed = any(c["reading"] == "time" for c in task["expected_clock"])
        starts = [p["world_patch"].get("clock", {}).get("start") for p in task["expect_1_9"]["paths"]]
        assert all(starts) if timed else not any(starts), task_id
        for start in starts:
            assert not start or re.fullmatch(r"2026-11-17T\d\d:\d\d:\d\d", start), task_id
    cutoff = SET["k02_shift_cutoff"]["expect_1_9"]["paths"]  # paths that start before and after the cutoff
    assert [bool(p.get("fails")) for p in cutoff] == [False, True]
    assert [p["world_patch"]["clock"]["start"][11:16] for p in cutoff] == ["21:05", "21:50"]
    window = SET["k05_open_hours_window"]["expect_1_9"]["paths"]  # before, inside and after the window
    assert sorted(p["world_patch"]["clock"]["start"][11:16] for p in window) == ["07:10", "09:40", "19:15"]
    assert [bool(p.get("fails")) for p in window] == [False, True, True]
    # the decoy: 11 pm is 23:00, and `11:00` is true at the 22:20 start
    decoy = SET["k10_decoy_11_pm"]
    assert "11 pm" in decoy["prompt"] and decoy["expected_clock"][0]["value"] == "23:00"
    assert decoy["expect_1_9"]["paths"][0]["world_patch"]["clock"]["start"].endswith("T22:20:00")
    # the 1.8 `do` and the 1.7 region limit are kept next to the time condition
    assert SET["k07_time_and_open"]["expected_actions"] == [{"robot": "moxi1", "action": "open",
                                                              "target": "closet_door"}]
    assert SET["k08_time_and_no_go"]["expected_limits"] == [["no_go_region", "wet_floor"]]
    # the cooling time of the standing task, with the wait as the first step: elapsed after the wait
    heat = SET["k09_heating_time"]
    assert heat["expected_clock"][0]["after"] == "wait until" and "which is at the fryer" in heat["prompt"]
    fryer = json.loads(read(REPO / heat["world"]))
    assert fryer["robots"]["flippy1"]["location"] == "fryer" and fryer["objects"]["oil_1"]["location"] == "fryer"


def test_the_infeasible_tasks_each_break_one_rule_of_0042():
    kinds = {t: set(d["tags"]) for t, d in INFEASIBLE.items()}
    assert sorted(k for tags in kinds.values() for k in tags if k not in ("clock_time", "wait_until", "window")) == [
        "no_else", "no_world_clock", "no_world_clock", "time_passed", "time_passed", "time_stamp",
        "timeout_too_short"]
    # no clock: the world file has none, whatever the request is (T7); the same request has a clock in k01
    assert SET["i01_stamp_no_clock"]["expect_1_9"]["world_1_5"]["clock"] is None
    assert SET["i04_wait_for_time_no_clock"]["expect_1_9"]["world_1_5"]["clock"] is None
    assert SET["k01_scan_stamp"]["expect_1_9"]["world_1_5"]["clock"]["start"]
    # the time has passed at the start (T4): the start is after the requested time
    for task_id in ("i02_before_it_has_passed", "i05_window_already_closed"):
        assert SET[task_id]["expect_1_9"]["world_1_5"]["clock"]["start"].endswith("T09:15:00"), task_id
    assert "before 08:00" in SET["i02_before_it_has_passed"]["prompt"]
    assert "between 06:00 and 08:00" in SET["i05_window_already_closed"]["prompt"]
    # the timeout is shorter than the wait (T5): 21:30 to 23:00 is 90 minutes and the request allows 10
    short = SET["i03_timeout_too_short"]
    assert short["expect_1_9"]["world_1_5"]["clock"]["start"].endswith("T21:30:00")
    assert "23:00" in short["prompt"] and "10 minutes" in short["prompt"]
    # a 12-hour request is not infeasible: the author converts it (k10 is a program)
    assert "11 pm" not in " ".join(t["prompt"] for t in INFEASIBLE.values())


def test_the_inexpressible_tasks_each_stay_out_for_one_reason_of_0042_t11():
    reasons = {t: set(d["tags"]) - {"clock_time"} for t, d in INEXPRESSIBLE.items()}
    assert reasons == {"x01_every_night_round": {"every", "mission_trigger"},
                       "x02_tomorrow_at_0100": {"another_day"},
                       "x03_time_between_scans": {"elapsed", "arithmetic"},
                       "x04_pickup_two_hours_later": {"arithmetic"},
                       "x05_on_monday": {"date"}}
    assert [d["gap"] for _, d in sorted(INEXPRESSIBLE.items())] == ["G-LOOP", "G-CLOCK", "G-CALC", "G-CALC", "G-CLOCK"]


def test_no_new_world_1_4_file_and_at_most_two_new_world_files():
    """1.5 facts (`clock`) are recorded in the labels only: L4 writes the world copies. The one new world is a 1.2
    copy of fry_station with flippy1 at the fryer (k09: the wait is the first step)."""
    existing = {p.name for p in (REPO / "bench" / "worlds").glob("*.json")}
    worlds = {Path(t["world"]).name for t in SET.values()}
    new = {"fry_station_at_fryer.json"}
    assert new <= existing and len(new) <= 2
    for name in worlds - new:
        assert name in existing or (REPO / "examples" / name).is_file() or name == "world.json", name
    for task_id, task in SET.items():
        text = read(REPO / task["world"])
        data = json.loads(text)
        assert data["world_schema_version"] in WORLDS_NOT_1_5, task_id
        assert '"clock"' not in text, task_id  # the 1.5 field: no world file has it yet
        assert "world_1_5" not in task and "world_1_5" not in task["expect"], task_id
    base, copy = json.loads(read(REPO / "bench/worlds/fry_station.json")), json.loads(
        read(REPO / "bench/worlds/fry_station_at_fryer.json"))
    copy["robots"]["flippy1"]["location"] = "prep_table"
    assert copy == base
    for task_id, task in DECLINED.items():  # the world note of a 1.9 label: a `clock`, or no clock at all
        note = task["expect_1_9"].get("world_1_5")
        if note is None:
            continue
        clock = note["clock"]
        assert clock is None or (set(clock) == {"start", "zone"} and clock["zone"] == "Europe/Prague"), task_id


def test_every_expected_clock_names_a_robot_that_the_world_has():
    for task_id, task in PROGRAMS.items():
        world = json.loads(read(REPO / task["world"]))
        for entry in task["expected_clock"]:
            assert entry["reading"] in ("time", "elapsed") and entry["use"] in ("message", "if", "wait"), task_id
            robots = entry.get("robot", [])
            for robot in robots if isinstance(robots, list) else [robots]:
                assert robot in world["robots"], (task_id, robot)
            if entry["use"] == "message":
                assert entry["channel"] in task["prompt"], task_id
            if entry["use"] == "wait":
                assert entry["op"] == ">=" and re.fullmatch(r"\d\d:\d\d", entry["value"]) and entry["timeout"]
            if entry["use"] == "if":
                for op, value in entry["conditions"]:
                    assert op in ("<", "<=", ">", ">=") and re.fullmatch(r"\d\d:\d\d", value), task_id


# -------- the references and the attempts --------

def test_references_and_attempts_are_where_they_should_be():
    names = [p.name for p in REF.iterdir()]
    refs = {n.split(".")[0] for n in names if ".attempt" not in n}
    attempts = {n.split(".")[0] for n in names if ".attempt." in n}
    assert refs == set(CONTROLS) and attempts == set(DECLINED)  # no 1.9 reference yet: that is L4
    for task_id in DECLINED:
        assert (REF / f"{task_id}.attempt.rob").is_file() and (REF / f"{task_id}.attempt.json").is_file()
    for name in names:
        assert "\r" not in read(REF / name)  # Windows-safe: LF only, never compared as CRLF


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_the_obvious_attempt_fails_with_the_recorded_code(task_id):
    task = DECLINED[task_id]
    answer = attempt_answer(task_id)
    assert answer["rob"].startswith(f"// bench/reference_clock/{task_id}.attempt.rob: the obvious attempt")
    assert answer["rob"].count("robolang 1.8;") == 1 and "\r" not in answer["rob"]
    checked = lib.check_answer(task, answer)
    assert not checked.valid and checked.codes[0] == task["attempt_fails_with"], (task_id, checked.codes)


def test_each_attempt_uses_the_one_construct_the_request_needs():
    reading = re.compile(r"measure r (time|elapsed) as \w+;|wait until clock_time >= \d\d:\d\d timeout")
    for task_id in DECLINED:
        rob = read(REF / f"{task_id}.attempt.rob")
        assert len(CLOCK_LINE.findall(rob)) >= 1 and reading.search(rob), task_id
        code = DECLINED[task_id]["attempt_fails_with"]
        # E101: a time literal (the lexer stops at `:`); E217: the kind `time` or `elapsed` and no literal
        assert (code == "E101") == bool(re.search(r"\b\d\d:\d\d\b", rob.split("\n", 1)[1])), task_id
    assert "wait until clock_time >= 23:00 timeout 2 h" in read(REF / "k10_decoy_11_pm.attempt.rob")  # 11 pm -> 23:00
    assert "wait until clock_time >= 23:00 timeout 10 min" in read(REF / "i03_timeout_too_short.attempt.rob")
    assert "now >= 08:00 and now < 18:00" in read(REF / "k05_open_hours_window.attempt.rob")
    assert "measure r elapsed as took;" in read(REF / "k04_job_elapsed.attempt.rob")
    assert "do r open door;" in read(REF / "k07_time_and_open.attempt.rob")  # the 1.8 `do` is kept
    assert "@safety no_go_region wet;" in read(REF / "k08_time_and_no_go.attempt.rob")  # the 1.7 limit is kept
    assert "// @clock" not in "".join(read(REF / f"{c}.rob") for c in CONTROLS)


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_no_probe_is_a_bug_in_disguise(task_id):
    """An attempt fails for its clock part: without it, it is a valid 1.8 program in the task's world that ends
    SUCCEEDED. That program is the silent replacement a model may write (a guessed `wait for`, a fixed message, or
    nothing in place of the time), and it is what the baseline counts as a false program (the plan's B8)."""
    answer = attempt_answer(task_id, strip=True)
    assert "clock_time" not in answer["rob"] and "measure r time" not in answer["rob"]
    assert "measure r elapsed" not in answer["rob"]
    checked = lib.check_answer(DECLINED[task_id], answer)
    assert checked.valid, (task_id, checked.report)
    assert checked.language_version == "1.8"
    judgement = lib.judge(dict(DECLINED[task_id], goal_free=True), checked, answer["bindings"])
    assert judgement.plan_succeeds, (task_id, judgement.state, judgement.failure_code)


def test_the_stripped_attempt_of_a_cutoff_task_drops_the_condition_and_the_else_path():
    """k02: with the time left out, the tote is moved on every path, so on the path after the cutoff the run
    succeeds where the label says it must stop: that is the silent replacement of the time condition."""
    answer = attempt_answer("k02_shift_cutoff", strip=True)
    assert "if " not in answer["rob"] and "fail" not in answer["rob"] and "communicate" not in answer["rob"]
    assert "move r to src;" in answer["rob"] and "place r tote at dst;" in answer["rob"]


def test_the_attempt_binds_the_entities_the_request_names():
    for task_id, task in PROGRAMS.items():
        bindings = json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"]
        world = json.loads(read(REPO / task["world"]))
        assert all(v in {*world["robots"], *world["objects"], *world["locations"], *world["sensors"],
                         *world["regions"]} for v in bindings.values()), task_id
        for entry in task["expected_clock"]:
            robots = entry.get("robot", [])
            assert set(robots if isinstance(robots, list) else [robots]) <= set(bindings.values()), task_id
    for task_id in {**INFEASIBLE, **INEXPRESSIBLE}:
        assert json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"]


# -------- the controls --------

@pytest.mark.parametrize("task_id", sorted(CONTROLS))
def test_the_control_reference_is_valid_exports_an_accepted_goal_and_succeeds_on_every_path(task_id):
    task = CONTROLS[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 1.8;") == 1 and answer["rob"].isascii()
    body = answer["rob"].split("\n", 1)[1]
    assert not re.search(r"clock_time|clock_elapsed|measure r (time|elapsed)|\d\d:\d\d", body)
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == "1.8"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert not judgement.half_program
    if "expected_readings" in task:
        assert judgement.value_ok


def test_the_controls_are_a_move_a_wait_for_and_a_measure_and_want_no_clock_reading():
    assert all(t["expected_clock"] == [] for t in CONTROLS.values())
    assert read(REF / "c02_wait_30_seconds.rob").count("wait for 30 s;") == 1
    assert "measure r temperature of pan using probe as t;" in read(REF / "c03_pan_temperature.rob")
    assert CONTROLS["c03_pan_temperature"]["expected_readings"] == [["chef_arm", "temperature", "pan_1"]]
    assert all(lib.task_paths(c) is None for c in CONTROLS.values())
    # the controls use words a model may over-read: "30 seconds", "as soon as possible"
    assert "30 seconds" in CONTROLS["c02_wait_30_seconds"]["prompt"]
    assert "As soon as possible" in CONTROLS["c04_as_soon_as_possible"]["prompt"]
    assert not any(re.search(r"\d\d:\d\d|\bpm\b|\bam\b", c["prompt"]) for c in CONTROLS.values())


# -------- the atlas --------

def test_every_task_1_8_declines_is_in_exactly_one_atlas_gap_and_no_control_is():
    in_gaps = [(t, g["id"]) for g in ATLAS["gaps"] for t in g["tasks"] if t in SET]
    assert len(in_gaps) == len({t for t, _ in in_gaps}) == len(DECLINED)
    assert {t for t, _ in in_gaps} == set(DECLINED)
    for task_id, gap in in_gaps:
        assert DECLINED[task_id]["gap"] == gap and GAPS[gap]["scope"] == "open"
    assert Counter(gap for _, gap in in_gaps) == {"G-CLOCK": 17, "G-LOOP": 1, "G-CALC": 2}
    assert {t["gap"] for t in INEXPRESSIBLE.values()} == {"G-CLOCK", "G-LOOP", "G-CALC"}
    assert {t["gap"] for t in PROGRAMS.values()} | {t["gap"] for t in INFEASIBLE.values()} == {"G-CLOCK"}


# -------- the scorer and the workflow --------

def clock_run(tmp_path: Path, replies: dict) -> Path:
    def script(request):
        first = request["messages"][0]["content"]
        task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
        return message(replies[task_id], model=request["model"])

    out_dir = tmp_path / "clock"
    run.run_benchmark(FakeClient(script), ["claude-sonnet-5-5"], {t: SET[t] for t in replies}, "medium", 1, out_dir,
                      lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="clock", prompt_name="full")
    return out_dir


def test_the_scorer_counts_a_silent_replacement_as_false_and_a_decline_of_a_control_as_a_refusal(tmp_path):
    replies = {
        "k01_scan_stamp": decline(),                                        # right under 1.8
        "i01_stamp_no_clock": decline("infeasible"),                        # declined, category differs (reported only)
        "k03_wait_for_0530": attempt_answer("k03_wait_for_0530", strip=True),  # a move for a wait: a false program
        "c01_plain_two_moves": reference("c01_plain_two_moves"),            # right
        "c02_wait_30_seconds": decline(),                                   # false refusal of a control
    }
    out_dir = clock_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "clock" and meta["prompt"] == "full"
    assert by["k01_scan_stamp"]["decision_correct"] and by["k01_scan_stamp"]["category_correct"]
    declined = by["i01_stamp_no_clock"]
    assert declined["decision_correct"] and declined["category_correct"] is False
    assert by["k03_wait_for_0530"]["false_program"] and not by["k03_wait_for_0530"]["decision_correct"]
    assert by["c01_plain_two_moves"]["decision_correct"] and by["c01_plain_two_moves"]["goal_correct"]
    assert by["c02_wait_30_seconds"]["false_refusal"] and not by["c02_wait_30_seconds"]["decision_correct"]
    text = score.score_dir(out_dir)
    assert "Task set clock, prompt full." in text
    kinds = text.split("By kind of task:")[1]
    for kind in ("clock", "infeasible", "control"):
        assert f"| {kind} |" in kinds


def test_the_set_is_in_the_run_script_and_only_runs_on_demand_in_the_workflow():
    assert lib.SETS["clock"] == (lib.CLOCK_TASKS_DIR, lib.CLOCK_REFERENCE_DIR)
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    assert workflow.count("--set clock") == 2
    lines = [ln for ln in workflow.splitlines() if "--set clock" in ln]
    for part in ("--repeats 3", "--models claude-sonnet-5-5,claude-haiku-5-5", "--max-usd 3"):
        assert all(part in ln for ln in lines), part
    assert any("--prompt full" in ln and "--run-id clock" in ln for ln in lines)
    assert any("--prompt minimal" in ln and "--run-id clock-minimal" in ln for ln in lines)
    assert all("claude-opus" not in ln and "--set clock --prompt" in ln for ln in lines)
    steps = re.split(r"\n      - ", workflow)
    step = next(s for s in steps if s.startswith("name: Run (set clock"))
    assert "inputs.clock" in step and "pull_request" not in step
    pr_step = next(s for s in steps if s.startswith("name: Run (pull request)"))
    weekly = next(s for s in steps if s.startswith("name: Run (weekly or on demand)"))
    assert "clock" not in pr_step and "!inputs.clock" in weekly and "--set clock" not in weekly
    declared = r"      clock:\n        description: .*\n        type: boolean\n        default: false"
    assert re.search(declared, workflow)
    assert "clock clock-minimal" in workflow


def test_the_workflow_stays_within_github_s_limit_of_10_dispatch_inputs():
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    block = workflow.split("  workflow_dispatch:\n    inputs:\n")[1].split("\npermissions:")[0]
    inputs = re.findall(r"^      ([a-z_0-9]+):$", block, re.M)
    assert len(inputs) == len(set(inputs)) and len(inputs) <= 10, inputs
    assert {"set3", "set4", "future", "regions", "actions", "clock"} <= set(inputs)
