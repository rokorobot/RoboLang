"""
Benchmark set 8, "clock" (bench/tasks_clock/, decision record 0042, plan docs/plans/bench-8-clock.md): clock readings
(`measure r time as t;`, `measure r elapsed as d;`, `HH:MM[:SS]` literals, `clock_time` / `clock_elapsed` in
`wait until`), pre-registered against RoboLang 1.8 before any code (L3 of iteration I3) and labelled for 1.9 in L4
(errata docs/evidence/bench-errata-1-9.md). `expect` is the 1.9 label, `expect_1_8` the 1.8 one (the baseline), and
`expect_1_9` keeps the paths of a program and the world 1.5 facts it rests on.

10 `clock` requests that are programs in 1.9, 5 `infeasible` in 1.9 (0042 T4, T5, T7), 5 `inexpressible` that stay
out (0042 T4, T11; G-DATE, G-LOOP, G-CALC), 4 `control` requests that are programs in both versions. 1.8 declines every
non-control task, and every one has an obvious attempt that 1.8 rejects (`E101`: a time literal, or `E217`: the kind
`time` / `elapsed`); those attempts stay as the evidence of the baseline. The 10 programs and the 4 controls have a 1.9
reference. The tasks use world file 1.5 copies. No network and no SDK; the model run is the `Benchmark` workflow with
`clock: true`.
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
COPIES = {"k01": "warehouse_1_5_1420", "k02": "fulfilment_1_5_2105", "k03": "yard_1_5_0430", "k04": "example_1_5",
          "k05": "warehouse_1_5_0940", "k06": "fulfilment_1_5_1930", "k07": "ward_1_5_1630", "k08": "site_1_5_1540",
          "k09": "fry_station_at_fryer_1_5", "k10": "site_1_5_2220", "i01": "lab_1_5", "i02": "warehouse_1_5_0915",
          "i03": "yard_1_5_2130", "i04": "example_1_5", "i05": "warehouse_1_5_0915"}  # task prefix -> its world copy


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


def test_labels_are_1_9_in_expect_and_1_8_in_expect_1_8():
    for task_id, task in SET.items():
        assert task["expect_1_9"]["decision"] in ("program", "cannot"), task_id
        if task["group"] == "control":
            assert task["expect"] == {"decision": "program"} and lib.expected_goals(task), task_id
            assert task["expect_1_9"] == {"decision": "program"} and task["expected_clock"] == []
            assert not {"why", "attempt_fails_with", "gap", "blockers", "expect_1_8"} & set(task)
            continue
        assert task["expect_1_8"] == CANNOT, task_id  # the baseline label, as pre-registered
        later = task["expect_1_9"]
        assert task["expect"] == {k: later[k] for k in ("decision", "category") if k in later}, task_id
        assert len(task["why"]) > 40 and task["attempt_fails_with"] in ("E101", "E217"), task_id
        assert not {"expected_goal", "expected_goals", "expect_1_7"} & set(task)
    for task_id, task in PROGRAMS.items():
        later = task["expect_1_9"]
        assert task["expect"] == {"decision": "program"} and lib.expectation(task) == ("program", "none"), task_id
        assert later["decision"] == "program" and later["paths"] and "category" not in later
        assert lib.task_paths(task) == later["paths"]
        assert "world_1_5" in later and task["expected_clock"], task_id
        for path in later["paths"]:
            has_goal = "goal" in path or "goals" in path  # "goals": defensible readings (erratum K2)
            assert path["when"] and (has_goal or path.get("fails")) and "world_patch" in path, task_id
    for task_id, task in INFEASIBLE.items():
        later = task["expect_1_9"]
        assert task["expect"] == {"decision": "cannot", "category": "infeasible"}, task_id
        assert later["decision"] == "cannot" and later["category"] == "infeasible", task_id
        assert later["because"] in ("0042 T4", "0042 T5", "0042 T7") and task["blockers"], task_id
    for task_id, task in INEXPRESSIBLE.items():
        later = task["expect_1_9"]
        assert task["expect"] == CANNOT and {k: later[k] for k in ("decision", "category")} == CANNOT, task_id
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
    assert [d["gap"] for _, d in sorted(INEXPRESSIBLE.items())] == ["G-LOOP", "G-DATE", "G-CALC", "G-CALC", "G-DATE"]


def test_the_worlds_are_1_5_copies_with_only_the_clock_added():
    """The 1.5 facts of a label (`expect_1_9.world_1_5`) are written as world file 1.5 copies: the base world with the
    version moved to 1.5 and a `clock` (or none), and nothing else. The controls and the inexpressible tasks keep their
    worlds; `k09`'s base is the 1.2 copy of fry_station with flippy1 at the fryer."""
    for task_id, task in SET.items():
        prefix = task_id[:3]
        if prefix not in COPIES:
            assert json.loads(read(REPO / task["world"]))["world_schema_version"] in WORLDS_NOT_1_5, task_id
            assert "world_1_5" not in task["expect_1_9"] or task["group"] == "control", task_id
            continue
        assert task["world"] == f"bench/worlds/{COPIES[prefix]}.json", task_id
        world = json.loads(read(REPO / task["world"]))
        note = task["expect_1_9"]["world_1_5"]
        assert world["world_schema_version"] == "1.5" and world["tasks"] == {}, task_id
        assert world.get("clock") == note["clock"], task_id  # None: the world has no clock
        assert "world_1_5" not in task and "world_1_5" not in task["expect"], task_id
    bases = {"lab_1_5": "bench/worlds/lab.json", "warehouse_1_5_1420": "bench/worlds/warehouse.json",
             "warehouse_1_5_0940": "bench/worlds/warehouse.json", "warehouse_1_5_0915": "bench/worlds/warehouse.json",
             "fulfilment_1_5_2105": "bench/worlds/fulfilment.json",
             "fulfilment_1_5_1930": "bench/worlds/fulfilment.json",
             "yard_1_5_0430": "bench/worlds/yard.json", "yard_1_5_2130": "bench/worlds/yard.json",
             "example_1_5": "examples/world.json", "site_1_5_2220": "bench/worlds/site.json",
             "ward_1_5_1630": "bench/worlds/ward_1_4.json", "site_1_5_1540": "bench/worlds/site_1_3.json",
             "fry_station_at_fryer_1_5": "bench/worlds/fry_station_at_fryer.json"}
    for name, base_file in bases.items():
        copy, base = (json.loads(read(REPO / f"bench/worlds/{name}.json")), json.loads(read(REPO / base_file)))
        copy.pop("clock", None)
        copy["world_schema_version"] = base["world_schema_version"]
        if base_file == "examples/world.json":  # the bench copy carries no task entries (the answer binds)
            base["tasks"] = {}
        assert copy == base, name
    base, copy = json.loads(read(REPO / "bench/worlds/fry_station.json")), json.loads(
        read(REPO / "bench/worlds/fry_station_at_fryer.json"))
    copy["robots"]["flippy1"]["location"] = "prep_table"
    assert copy == base
    for task_id, task in SET.items():  # the world note of a 1.9 label: a `clock`, or no clock at all
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
    assert refs == set(CONTROLS) | set(PROGRAMS) and attempts == set(DECLINED)  # the 1.8 attempts are the baseline
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
    """An attempt fails under 1.8 for its clock part: without it, it is a valid 1.8 program in the task's world that
    ends SUCCEEDED. That program is the silent replacement a model may write (a guessed `wait for`, a fixed message, or
    nothing in place of the time), and the scorer counts it: `time_replaced` for a program task (the plan's B8)."""
    task = DECLINED[task_id]
    answer = attempt_answer(task_id, strip=True)
    assert "clock_time" not in answer["rob"] and "measure r time" not in answer["rob"]
    assert "measure r elapsed" not in answer["rob"]
    checked = lib.check_answer(task, answer)
    assert checked.valid, (task_id, checked.report)
    assert checked.language_version == "1.8"
    judgement = lib.judge(dict(task, goal_free=True), checked, answer["bindings"])
    assert judgement.state == "SUCCEEDED", (task_id, judgement.state, judgement.failure_code)
    if task_id in PROGRAMS:
        assert judgement.time_replaced and judgement.clock_ok is False, task_id


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
    assert answer["rob"].count("robolang 1.9;") == 1 and answer["rob"].isascii()
    body = answer["rob"].split("\n", 1)[1]
    assert not re.search(r"clock_time|clock_elapsed|measure r (time|elapsed)|\d\d:\d\d", body)
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == "1.9"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert not judgement.half_program and judgement.clock_ok is True and judgement.time_replaced is None
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

def test_the_atlas_keeps_what_1_9_still_declines_and_g_clock_is_closed():
    """As for set 7 at 0040 L4: a gap lists the tasks that are still inexpressible, so G-CLOCK, which 1.9 expresses,
    lists none and stays open until the acceptance run (L5) and closes at L6; the five that stay out are in their own
    gaps."""
    in_gaps = [(t, g["id"]) for g in ATLAS["gaps"] for t in g["tasks"] if t in SET]
    assert {t for t, _ in in_gaps} == set(INEXPRESSIBLE) and len(in_gaps) == len(INEXPRESSIBLE)
    for task_id, gap in in_gaps:
        assert INEXPRESSIBLE[task_id]["gap"] == gap and GAPS[gap]["scope"] == "open"
    assert Counter(gap for _, gap in in_gaps) == {"G-DATE": 2, "G-LOOP": 1, "G-CALC": 2}
    assert {t["gap"] for t in PROGRAMS.values()} | {t["gap"] for t in INFEASIBLE.values()} == {"G-CLOCK"}
    assert GAPS["G-CLOCK"]["scope"] == "closed" and GAPS["G-CLOCK"]["closed_in"] == "RoboLang 1.9"  # L6 of I3
    assert not {c for c in CONTROLS} & {t for g in ATLAS["gaps"] for t in g["tasks"]}


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


def test_the_scorer_counts_clock_ok_time_replaced_and_a_decline_of_a_control_as_a_refusal(tmp_path):
    replies = {
        "k01_scan_stamp": reference("k01_scan_stamp"),                       # right: the time is read and sent
        "i01_stamp_no_clock": decline("infeasible"),                         # right: the world has no clock
        "k03_wait_for_0530": attempt_answer("k03_wait_for_0530", strip=True),  # the wait left out: time replaced
        "k02_shift_cutoff": decline(),                                       # a false refusal of a program task
        "c01_plain_two_moves": reference("c01_plain_two_moves"),             # right
        "c02_wait_30_seconds": decline(),                                    # false refusal of a control
    }
    out_dir = clock_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "clock" and meta["prompt"] == "full"
    assert by["k01_scan_stamp"]["decision_correct"] and by["k01_scan_stamp"]["clock_ok"] is True
    assert by["k01_scan_stamp"]["time_replaced"] is False and by["k01_scan_stamp"]["request_satisfied"]
    declined = by["i01_stamp_no_clock"]
    assert declined["decision_correct"] and declined["category_correct"] is True
    assert declined["clock_ok"] is None and declined["time_replaced"] is None  # nothing asked of the clock
    replaced = by["k03_wait_for_0530"]
    assert replaced["clock_ok"] is False and replaced["time_replaced"] is True and not replaced["request_satisfied"]
    assert by["k02_shift_cutoff"]["false_refusal"] and by["k02_shift_cutoff"]["clock_ok"] is False
    assert by["c01_plain_two_moves"]["decision_correct"] and by["c01_plain_two_moves"]["clock_ok"] is True
    assert by["c01_plain_two_moves"]["time_replaced"] is None
    assert by["c02_wait_30_seconds"]["false_refusal"] and not by["c02_wait_30_seconds"]["decision_correct"]
    text = score.score_dir(out_dir)
    assert "Task set clock, prompt full." in text
    assert ("| clock_ok (exactly the clock readings asked, none on a control; all answers to those tasks) "
            "| 2/5 |") in text
    assert ("| time_replaced_silently (valid, SUCCEEDED, an asked time condition or wait missing; all answers to "
            "tasks that ask for one) | 1/3 |") in text
    kinds = text.split("By kind of task:")[1]
    for kind in ("clock", "infeasible", "control"):
        assert f"| {kind} |" in kinds


def judged(task_id: str, rob: str):
    task = SET[task_id]
    answer = reference(task_id)
    answer["rob"] = rob
    checked = lib.check_answer(task, answer)
    assert checked.valid, (task_id, checked.report)
    return lib.judge(task, checked, answer["bindings"])


CUTOFF = read(REF / "k02_shift_cutoff.rob")
SWAPPED = CUTOFF.replace("if now < 21:30 {", "if now >= 21:30 {").replace(
    "            move r to src;\n            grasp r tote;\n            move r to dst;\n"
    "            place r tote at dst;\n        } else {\n"
    '            communicate r to "lead" with "after cutoff";\n            fail "after the cutoff";\n',
    '            communicate r to "lead" with "after cutoff";\n            fail "after the cutoff";\n'
    "        } else {\n"
    "            move r to src;\n            grasp r tote;\n            move r to dst;\n"
    "            place r tote at dst;\n")


def test_clock_ok_accepts_the_test_with_its_branches_swapped_the_paths_decide():
    assert SWAPPED != CUTOFF and "if now >= 21:30 {" in SWAPPED
    for rob in (CUTOFF, SWAPPED):
        j = judged("k02_shift_cutoff", rob)
        assert j.clock_ok and j.goal_correct and j.plan_succeeds and not j.half_program


def test_clock_ok_rejects_another_time_another_operator_a_second_reading_or_the_wrong_robot_channel_or_order():
    assert not judged("k02_shift_cutoff", CUTOFF.replace("21:30", "21:45")).clock_ok
    assert not judged("k02_shift_cutoff", CUTOFF.replace("now < 21:30", "now <= 21:30")).clock_ok
    extra = CUTOFF.replace("measure r time as now;", "measure r time as now;\n        measure r elapsed as spare;")
    assert not judged("k02_shift_cutoff", extra).clock_ok
    scan = read(REF / "k01_scan_stamp.rob")
    assert judged("k01_scan_stamp", scan).clock_ok
    assert not judged("k01_scan_stamp", scan.replace('to "lead"', 'to "ops"')).clock_ok
    assert not judged("k01_scan_stamp", scan.replace("measure r time as now;", "measure r elapsed as now;")).clock_ok
    inspected = "        move r to shelf;\n        inspect r box using camera;\n"
    early = scan.replace(f"{inspected}        measure r time as now;\n", f"        measure r time as now;\n{inspected}")
    assert early != scan and not judged("k01_scan_stamp", early).clock_ok  # `after: inspect`


def test_a_wait_is_compared_by_time_operator_and_timeout_in_durations_an_hour_is_60_min():
    two = read(REF / "k06_two_robots_at_2000.rob")
    assert judged("k06_two_robots_at_2000", two).clock_ok
    assert not judged("k06_two_robots_at_2000", two.replace("timeout 60 min", "timeout 90 min")).clock_ok
    assert not judged("k06_two_robots_at_2000", two.replace(">= 20:00", "> 20:00")).clock_ok
    assert not judged("k06_two_robots_at_2000", two.replace("clock_time >= 20:00", "clock_time >= 20:00:30")).clock_ok
    for name in ("k03_wait_for_0530", "k10_decoy_11_pm"):
        assert judged(name, read(REF / f"{name}.rob")).clock_ok
    assert not judged("k03_wait_for_0530", read(REF / "k03_wait_for_0530.rob").replace("05:30", "11:00")).clock_ok
    # the 11 pm decoy: 23:00, never 11:00. At the 22:20 start 11:00 has passed, so the check before the run (K3,
    # `S010`) stops it: it is not a program that moves at once with no error
    decoy = reference("k10_decoy_11_pm")
    decoy["rob"] = decoy["rob"].replace("23:00", "11:00")
    caught = lib.check_answer(SET["k10_decoy_11_pm"], decoy)
    assert not caught.valid and caught.codes == ("S010",)
    assert lib._ms("1 h") == lib._ms("60 min") == 3600000 and lib._ms("2 h") == lib._ms("120 min")


def test_a_clock_on_a_control_is_not_clock_ok_and_a_relative_wait_is_not_a_clock():
    control = read(REF / "c01_plain_two_moves.rob")
    with_time = control.replace("        move r to a;", "        measure r time as now;\n        move r to a;")
    assert judged("c01_plain_two_moves", control).clock_ok is True
    assert judged("c01_plain_two_moves", with_time).clock_ok is False
    waited = read(REF / "c02_wait_30_seconds.rob")
    assert "wait for 30 s;" in waited and judged("c02_wait_30_seconds", waited).clock_ok is True
    live = waited.replace("wait for 30 s;", 'wait until clock_elapsed >= 30 s timeout 1 min else { fail "x"; }')
    assert live != waited and judged("c02_wait_30_seconds", live).clock_ok is False


def test_time_replaced_is_a_valid_succeeding_program_that_lacks_the_time_asked_b8():
    none = ("robolang 1.9;\n\ntask move_tote(robot r, object tote, location src, location dst) {\n"
            "    goal { object_at tote dst; }\n    plan {\n        move r to src;\n        grasp r tote;\n"
            "        move r to dst;\n        place r tote at dst;\n    }\n}\n")
    j = judged("k02_shift_cutoff", none)
    assert j.time_replaced and not j.clock_ok and j.state == "SUCCEEDED"  # the first path moves the tote
    wait = read(REF / "k03_wait_for_0530.rob")
    guessed = wait.replace('wait until clock_time >= 05:30 timeout 90 min else { fail "05:30 did not come in time"; }',
                           "wait for 60 min;")
    assert guessed != wait and judged("k03_wait_for_0530", guessed).time_replaced
    fixed = read(REF / "k01_scan_stamp.rob").replace("measure r time as now;", "").replace(
        'communicate r to "lead" with now;', 'communicate r to "lead" with "14:20";')
    assert judged("k01_scan_stamp", fixed).time_replaced
    # a right program, or a wrong comparison of a time that is there, is not a replacement
    assert judged("k02_shift_cutoff", CUTOFF).time_replaced is False
    assert judged("k02_shift_cutoff", CUTOFF.replace("21:30", "21:45")).time_replaced is False
    assert judged("c01_plain_two_moves", read(REF / "c01_plain_two_moves.rob")).time_replaced is None


def test_every_1_9_reference_is_valid_exact_and_right_on_every_path_under_world_1_5():
    for task_id, task in PROGRAMS.items():
        answer = reference(task_id)
        assert answer["rob"].count("robolang 1.9;") == 1 and answer["rob"].isascii(), task_id
        body = answer["rob"].split("\n", 1)[1]
        assert "// @clock" not in body and not re.search(r"timeout \d+ h\b", body), task_id
        checked = lib.check_answer(task, answer)
        assert checked.valid and checked.language_version == "1.9", (task_id, checked.report)
        assert json.loads(read(REPO / task["world"]))["world_schema_version"] == "1.5"
        j = lib.judge(task, checked, answer["bindings"])
        assert j.goal_correct and j.plan_succeeds and j.clock_ok and j.time_replaced is False, task_id
        assert len(j.paths) == len(task["expect_1_9"]["paths"])
        for path, expected in zip(j.paths, task["expect_1_9"]["paths"]):
            assert path["goal_ok"] and path["plan_ok"], (task_id, path)
            assert (path["state"] == "FAILED") == bool(expected.get("fails")), (task_id, path)
        assert not j.half_program
        if task_id == "k07_time_and_open":
            assert j.actions_ok
        if task_id == "k08_time_and_no_go":
            assert j.limit_ok


def test_the_model_is_shown_the_clock_start_and_zone_or_that_the_world_has_none_k9():
    shown = lib.user_message(SET["k03_wait_for_0530"])
    assert "clock: the task starts at 2026-11-17 04:30:00 (Europe/Prague)" in shown
    for task_id in ("i01_stamp_no_clock", "i04_wait_for_time_no_clock", "k04_job_elapsed", "k09_heating_time"):
        assert "clock: none (this world has no clock)" in lib.user_message(SET[task_id]), task_id
    for task_id in ("c01_plain_two_moves", "x02_tomorrow_at_0100"):  # a world file older than 1.5 shows no clock line
        assert "clock:" not in lib.user_message(SET[task_id]), task_id
    assert "can do no declared action" in lib.user_message(SET["k03_wait_for_0530"])


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
