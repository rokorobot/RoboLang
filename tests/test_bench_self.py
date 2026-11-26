"""
Benchmark set 13, "self" (bench/tasks_self/, decision record 0048, plan docs/plans/bench-13-self.md): the robot's own
state, `measure r charge as c;` and `robot_charge r` in `wait until`, pre-registered against RoboLang 2.3 before any
code (L3 of iteration I8). `expect` is the 2.3 label (the baseline: every non-control task declines as inexpressible,
because 2.3 has no `charge` kind (`E217`) and no `robot_charge` condition (`E204`)); `expect_2_4` is the label for
acceptance once 2.4 is taught: `world_2_4` (the world file 1.9 facts: a robot's `charge` and the scheduled charge
events), `expected_charge` (what each charge check is: where it stands, the operator, the threshold, the decoy robot),
the paths of a program (each forced by a `world_patch` of charges and events) and, for a mission, `expected_mission`.

10 programs in 2.4, 5 `infeasible` in 2.4, 5 `inexpressible` that stay out (0048 S10: G-HEALTH, G-PREDICT, G-TRIGGER),
4 `control` requests that are programs in both versions (a `fill` of an object, `robot_ready`, a declared charge with no
reading, a fixed mission). 2.3 declines every non-control task, and each has an obvious attempt that 2.3 rejects
(`E217` or `E204`); the lines that read the robot's own state are marked `// @charge`. Without them (or, for a
choice, with the chosen name replaced by the first candidate) the attempt is a valid 2.3 program that succeeds: the
charge check replaced silently, which the old checks cannot tell (the plan's B8). Only the controls have a reference;
the 2.4 references wait for L4. No network and no SDK; the model run is the `Benchmark` workflow with `set_only: self`.
"""

from __future__ import annotations
import io
import json
import re
from collections import Counter
from decimal import Decimal
from pathlib import Path

import pytest
from test_bench import FakeClient, message

from bench import lib, run, score

REPO = Path(__file__).resolve().parent.parent
SET = lib.load_set("self")
REF = lib.SELF_REFERENCE_DIR
ATLAS = json.loads((REPO / "bench" / "atlas.json").read_text(encoding="utf-8"))
GAPS = {g["id"]: g for g in ATLAS["gaps"]}
CANNOT = {"decision": "cannot", "category": "inexpressible"}
PROGRAMS = {t: d for t, d in SET.items() if d["group"] == "self"}
INFEASIBLE = {t: d for t, d in SET.items() if d["group"] == "infeasible"}
INEXPRESSIBLE = {t: d for t, d in SET.items() if d["group"] == "inexpressible"}
CONTROLS = {t: d for t, d in SET.items() if d["group"] == "control"}
DECLINED = {**PROGRAMS, **INFEASIBLE, **INEXPRESSIBLE}  # what 2.3 declines: an attempt each
MISSION_PROGRAMS = {"p05_patrol_until_the_battery_is_low"}
MISSIONS = MISSION_PROGRAMS | {"x05_every_time_it_drops_below_twenty"}
CHOICES = {"p06_the_peer_with_enough_charge", "i04_nobody_has_seventy_percent"}
IFS = {"p01_low_battery_then_dock", "p02_enough_charge_for_the_round", "p07_charge_the_other_robot_if_low",
       "p08_dock_if_low_around_the_wet_floor", "p09_wait_for_go_then_look_if_charged", "p10_swap_then_check_ninety"}
NEW_WORLDS = {"charge_hall.json"}  # the plan allows at most 3
MARK = re.compile(r"// @charge(?: (.*))?$")


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


LIFTED = ("expected_charge", "expected_actions", "expected_limits", "expected_readings", "expected_comparisons",
          "expected_message_waits", "expected_mission", "mission_alternatives", "mission_clock")


def label(task: dict) -> dict:
    """The pre-registered 2.4 label: `expect_2_4` and the checks that L4 lifted to the task's own keys."""
    merged = dict(task["expect_2_4"])
    merged.update({key: task[key] for key in LIFTED if key in task})
    if "expected_choices" in task:
        merged["choices"] = task["expected_choices"]
    return merged


def base_world(task: dict) -> dict:
    return json.loads(read(REPO / task["world"]))


def world_ids(task: dict) -> set[str]:
    world = base_world(task)
    return {*world["robots"], *world["objects"], *world["locations"], *world.get("sensors", {}),
            *world.get("regions", {})}


def reference(task_id: str) -> dict:
    ref = json.loads(read(REF / f"{task_id}.json"))
    rob = read(REF / f"{task_id}.rob")
    return {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob, **ref}


def decline(category: str = "inexpressible") -> dict:
    return {"decision": "cannot", "category": category, "reason": "cannot be done", "rob": "", "task": "",
            "bindings": []}


def strip_choice(rob: str, bindings: dict) -> tuple[str, dict]:
    """The attempt of a choice without it: the marked line goes and the chosen name is replaced everywhere by the first
    candidate the marker names: a fixed pick (the plan's B8)."""
    lines = rob.splitlines(keepends=True)
    marked = [ln for ln in lines if MARK.search(ln.rstrip())]
    assert len(marked) == 1
    kept = "".join(ln for ln in lines if ln not in marked)
    name, entity = MARK.search(marked[0].rstrip()).group(1).split("=")
    body = re.sub(rf"\b{name}\b", entity, kept)
    return body, bindings


def strip_charge(rob: str) -> str:
    """The attempt without its charge check: a line marked `// @charge` goes, or is replaced by the text after `=>`."""
    out = []
    for ln in rob.splitlines():
        found = MARK.search(ln.rstrip())
        if not found:
            out.append(ln)
        elif (found.group(1) or "").startswith("=>"):
            out.append(found.group(1)[2:].strip())
    return "\n".join(out) + "\n"


def attempt_answer(task_id: str, strip: bool = False) -> dict:
    stem = REF / f"{task_id}.attempt"
    rob, data = read(Path(f"{stem}.rob")), json.loads(read(Path(f"{stem}.json")))
    if strip:
        rob = strip_choice(rob, data["bindings"])[0] if task_id in CHOICES else strip_charge(rob)
    return {"decision": "program", "category": "none", "reason": "an attempt", "entities": [], "rob": rob, **data}


# ---- a small model of 0048 S2 to S7, for the labels: what does a charge check see in a world? ----

def seconds(at: dict) -> Decimal:
    return Decimal(at["value"]) * {"s": 1, "min": 60}[at["unit"]]


def charge_at(task: dict, path: dict, robot: str, t_s: int | Decimal = 0) -> dict | None:
    """The charge of a robot at a time: the world 1.9 facts of the label, the path's patch over them, then the scheduled
    events up to that time (S4); None when the robot has no charge: unknown."""
    patch = (path or {}).get("world_patch", {})
    robots = lib._merge({"robots": label(task)["world_2_4"]["robots"]}, {"robots": patch.get("robots", {})})["robots"]
    current = (robots.get(robot) or {}).get("charge")
    events = patch.get("charge_events", label(task)["world_2_4"]["charge_events"])
    for event in sorted(events, key=lambda e: seconds(e["at"])):
        if event["robot"] == robot and seconds(event["at"]) <= t_s:
            current = event["charge"]
    return current


def compare(op: str, threshold: str, reading: dict | None) -> bool | None:
    """True, False or None (unknown: the uncertainty interval holds the threshold, or there is no reading): S2."""
    if reading is None:
        return None
    value, unc, x = Decimal(reading["value"]), Decimal(reading.get("uncertainty", "0")), Decimal(threshold)
    low, high = value - unc, value + unc
    true, false = {"<": (high < x, low >= x), "<=": (high <= x, low > x), ">": (low > x, high <= x),
                   ">=": (low >= x, high < x)}[op]
    return True if true else False if false else None


STRICT = {"<": "<=", "<=": "<", ">": ">=", ">=": ">"}


def check_of(task: dict, path: dict, spec: dict, *, robot=None, op=None, value=None) -> bool | None:
    t = spec.get("at_s", 0)
    return compare(op or spec["op"], value or spec["value"], charge_at(task, path, robot or spec["robot"], t))


def reached_min(task: dict, path: dict, spec: dict, *, robot=None, op=None, value=None):
    """A `wait until robot_charge r OP X timeout T`: the minute it first holds within the timeout (an event or the
    start), else None (S3, S4)."""
    timeout = Decimal(spec["timeout"].split()[0]) * 60
    robot, op, value = robot or spec["robot"], op or spec["op"], value or spec["value"]
    events = path["world_patch"].get("charge_events", label(task)["world_2_4"]["charge_events"])
    met = {seconds(e["at"]) for e in events if e["robot"] == robot and seconds(e["at"]) <= timeout}
    times = sorted({Decimal(0)} | met)
    for t in times:
        if compare(op, value, charge_at(task, path, robot, t)) is True:
            return int(t / 60)
    return None


def candidates(task: dict, path: dict, choice: dict, *, op=">=", value="50", use_where=True, fixed=None):
    """`choose robot t from (...) where robot_charge t >= 50 %`: the first that holds (unknown is left out)."""
    found = list(choice["from"])
    if fixed is not None:
        return fixed
    if use_where:
        found = [c for c in found if compare(op, value, charge_at(task, path, c)) is True]
    return found[0] if found else None


def mission_model(task: dict, path: dict, durations: list, max_runs: int, spec: dict, *, robot=None, op=None,
                  value=None):
    """A mission with `until robot_charge r OP X` under 0041 and 0048 S6: the count is checked before each run, the
    until after it; a run lasts its wait; an unknown until fails the mission."""
    robot, op, value = robot or spec["robot"], op or spec["op"], value or spec["value"]
    clock, runs = Decimal(0), 0
    while runs < max_runs:
        clock += Decimal(durations[runs % len(durations)]) * 60
        runs += 1
        held = compare(op, value, charge_at(task, path, robot, clock))
        if held is None:
            return "failed", runs
        if held:
            return "completed", runs
    return "bounded", runs


# -------- the set --------

def test_the_mix_of_tasks():
    assert len(SET) == 24
    assert Counter(d["group"] for d in SET.values()) == {"self": 10, "infeasible": 5, "inexpressible": 5, "control": 4}
    prefixes = {"self": "p", "infeasible": "i", "inexpressible": "x", "control": "c"}
    for task_id, task in SET.items():
        assert task["id"] == task_id and (lib.SELF_TASKS_DIR / f"{task_id}.json").is_file()
        assert (REPO / task["world"]).is_file() and task["tags"] and task["prompt"].endswith(".")
        assert task["prompt"].isascii() and "\r" not in read(lib.SELF_TASKS_DIR / f"{task_id}.json")
        assert re.fullmatch(prefixes[task["group"]] + r"\d\d_[a-z0-9_]+", task_id), task_id


def test_the_ids_are_new_ones_and_no_prompt_is_a_copy_of_a_standing_task():
    """The R3 probes and the standing tasks of the gap stay in their sets (as set 12 left its probes); this set has new
    ids and new prompts for the same shapes."""
    elsewhere = {}
    for name in lib.SETS:
        if name != "self":
            elsewhere.update(lib.load_set(name))
    assert not set(SET) & set(elsewhere)
    assert not {t["prompt"] for t in SET.values()} & {t["prompt"] for t in elsewhere.values()}
    assert {"f_self_low_battery", "f_self_enough_for_route", "f_self_report_charge", "x02_own_battery_first",
            "x02_selftest_result"} <= set(elsewhere)


def test_one_new_world_file_1_8_and_the_baseline_world_shows_no_charge():
    """The 2.3 world (`world_2_3`) shows no charge; the world of the task is a world file 1.9 that holds the notes."""
    used = {Path(t["world_2_3"] if "world_2_3" in t else t["world"]).name for t in SET.values()}
    assert NEW_WORLDS <= used and len(NEW_WORLDS) <= 3
    for name in NEW_WORLDS:
        assert json.loads(read(REPO / "bench" / "worlds" / name))["world_schema_version"] == "1.8"
    for task in SET.values():
        old = json.loads(read(REPO / task.get("world_2_3", task["world"])))
        for robot in old["robots"].values():
            assert "charge" not in robot, task["id"]


def test_the_world_of_a_charge_task_is_world_file_1_9_with_the_notes_laid_over_the_2_3_world():
    for task_id, task in SET.items():
        if task["group"] == "control":
            assert task["world"] == "bench/worlds/charge_hall.json" and "world_2_3" not in task
            continue
        assert task["world"] == f"bench/worlds/charge_{task_id}.json", task_id
        new, old = base_world(task), json.loads(read(REPO / task["world_2_3"]))
        assert new["world_schema_version"] == "1.9" and old["world_schema_version"] in ("1.2", "1.3", "1.8")
        notes = label(task)["world_2_4"]
        expected = lib._merge(old, {k: v for k, v in notes.items()})
        expected["world_schema_version"] = "1.9"
        if not expected.get("charge_events"):
            expected.pop("charge_events", None)
        assert new == expected, task_id  # and nothing else changed


def test_expect_is_the_2_4_label_and_the_2_3_label_is_in_expect_2_3():
    for task_id, task in SET.items():
        if task["group"] == "control":
            assert task["expect"] == task["expect_2_4"] == {"decision": "program"}
            assert not {"why", "attempt_fails_with", "gap", "blockers"} & set(task)
            assert lib.expectation(task) == ("program", "none")
            continue
        assert task["expect_2_3"] == CANNOT, task_id  # the baseline label, kept
        later = label(task)
        assert task["expect"] == {k: later[k] for k in ("decision", "category") if k in later}, task_id
        assert len(task["why"]) > 40 and task["gap"] in GAPS and task["attempt_fails_with"] in ("E217", "E204")
        assert not {"expected_goal", "expected_goals", "paths", "expect_2_0"} & set(task), task_id
    for task_id, task in PROGRAMS.items():
        later = label(task)
        assert later["decision"] == "program" and later["world_2_4"] and later["expected_charge"], task_id
        assert lib.expectation(task) == ("program", "none")
        assert len({p["when"] for p in later["paths"]}) == len(later["paths"]) >= 3
        assert task["gap"] == "G-SELF" and ("expected_mission" in later) == (task_id in MISSION_PROGRAMS)
        assert ("goal_free" in task) == (task_id == "p09_wait_for_go_then_look_if_charged")
        assert lib.task_paths(task) == (None if task_id in MISSION_PROGRAMS else later["paths"]), task_id
        assert (lib.mission_label(task) is not None) == (task_id in MISSION_PROGRAMS), task_id
    for task_id, task in INFEASIBLE.items():
        later = label(task)
        assert later["decision"] == "cannot" and later["category"] == "infeasible" and later["because"], task_id
        assert lib.expectation(task) == ("cannot", "infeasible") and task["expect"] == {
            "decision": "cannot", "category": "infeasible"}
        assert task["blockers"] and task["gap"] == "G-SELF" and "paths" not in later
        assert "otherwise" not in task["prompt"]
    for task_id, task in INEXPRESSIBLE.items():
        later = label(task)
        assert {k: later[k] for k in ("decision", "category")} == CANNOT and later["because"].startswith("0048 S10")
        assert lib.expectation(task) == ("cannot", "inexpressible")


def test_the_programs_cover_what_the_pre_registration_lists():
    tags = Counter(tag for t in PROGRAMS.values() for tag in t["tags"])
    for tag in ("threshold_below", "threshold_at_least", "report", "wait_until", "scheduled_event", "do_charge",
                "mission", "until", "choose", "robot_charge", "near_threshold", "fails_closed", "decoy", "region_limit",
                "message_wait", "fill", "battery_swap", "declared_action", "unit_sent", "count_bound"):
        assert tags[tag] >= 1, tag
    uses = Counter(c["use"] for t in PROGRAMS.values() for c in label(t)["expected_charge"])
    assert uses == {"if": 6, "report": 1, "wait": 1, "until": 1, "where": 1}
    assert label(PROGRAMS["p08_dock_if_low_around_the_wet_floor"])["expected_limits"] == [["no_go_region", "wet_floor"]]
    assert SET["p08_dock_if_low_around_the_wet_floor"]["world_2_3"].endswith("site_1_3.json")  # honours no_go_region
    assert label(PROGRAMS["p09_wait_for_go_then_look_if_charged"])["expected_message_waits"][0]["timeout"] == "15 min"
    assert label(PROGRAMS["p09_wait_for_go_then_look_if_charged"])["expected_readings"] == [["spot1", "fill", "bin_1"]]
    mission = label(PROGRAMS["p05_patrol_until_the_battery_is_low"])["expected_mission"]
    assert mission["repeat"] == {"max_runs": 6, "max_duration": None} and mission["policy_stated"] is False


# -------- the labels follow 0048: a check, its paths, and what a charge patch changes --------

def prompt_words(spec: dict) -> tuple[str, str]:
    words = {"<": ("below", "under"), "<=": ("at most", "no more"), ">": ("above", "over"),
             ">=": ("at least", "at least")}
    return words[spec["op"]]


def test_the_threshold_and_the_operator_are_the_ones_the_request_states_z1():
    """A threshold or an operator the request does not state is a criterion change (erratum Z1 of 2.3)."""
    for task_id, task in PROGRAMS.items():
        for spec in label(task)["expected_charge"]:
            if spec["use"] == "report":
                continue
            assert f"{spec['value']} percent" in task["prompt"], task_id
            assert any(word in task["prompt"] for word in prompt_words(spec)), task_id
            assert spec["robot"] in task["prompt"] or spec["robot"] == "candidate", task_id
    assert "spot2's battery is below 40 percent" in SET["p07_charge_the_other_robot_if_low"]["prompt"]


@pytest.mark.parametrize("task_id", sorted(IFS))
def test_each_if_path_has_the_check_the_label_says_and_the_goal_or_failure_that_follows(task_id):
    task = PROGRAMS[task_id]
    spec = label(task)["expected_charge"][0]
    assert spec["decoy"] != spec["robot"] and spec["decoy"] in base_world(task)["robots"]
    seen = []
    for path in label(task)["paths"]:
        got = check_of(task, path, spec)
        assert path["check"] == got, (task_id, path["when"], got)
        seen.append(got)
        for robot in (spec["robot"], spec["decoy"]):  # every path gives both robots a charge: a decoy is a fact
            assert charge_at(task, path, robot) is not None, (task_id, path["when"], robot)
        shape = [k for k in ("goal", "goals", "fails") if k in path]
        if task_id == "p09_wait_for_go_then_look_if_charged":
            assert not shape or shape == ["fails"]  # the request names no end state: goal-free (erratum P7 of 2.2)
        else:
            assert len(shape) == 1, (task_id, path["when"])
        if path.get("fails"):
            continue
        for goal_set in path.get("goals") or [path.get("goal", [])]:
            for subject, prop, value in goal_set:
                assert prop in ("located_in", "in_state") and subject.split(":")[2] in world_ids(task)
                assert value.split(":")[-1] in world_ids(task) | {"charging", "swapped"}
    assert {True, False, None} <= set(seen), task_id  # holds, does not hold, near the threshold


def test_the_report_names_the_value_with_its_unit_and_an_unknown_charge_fails_closed():
    task = PROGRAMS["p03_tell_the_lead_the_charge"]
    for path in label(task)["paths"]:
        reading = charge_at(task, path, "spot1")
        if reading is None:
            assert path["fails"] and path["messages"] == []
        else:
            assert reading["unit"] == "%" and path["messages"] == [["lead", f"{reading['value']} %"]]  # S5
            assert path["goal"] == [["rl:robot:spot1", "located_in", "rl:place:dock_3"]]
    assert sum(bool(p.get("fails")) for p in label(task)["paths"]) == 1


def test_the_charge_wait_is_met_by_a_scheduled_event_within_its_timeout():
    task = PROGRAMS["p04_charge_until_eighty"]
    spec = label(task)["expected_charge"][0]
    assert spec["timeout"] == "40 min" and "40 minutes" in task["prompt"]
    for path in label(task)["paths"]:
        got = reached_min(task, path, spec)
        assert path["reached_min"] == got, (path["when"], got)
        assert bool(path.get("fails")) == (got is None), path["when"]
    outcomes = [p["reached_min"] for p in label(task)["paths"]]
    assert 0 in outcomes and 20 in outcomes and outcomes.count(None) >= 3  # at once, by an event, never in time
    for path in label(task)["paths"]:
        assert {e["robot"] for e in path["world_patch"]["charge_events"]} <= {"spot2"}


def test_the_mission_ends_as_the_until_the_count_and_the_unknown_say_s6():
    task = PROGRAMS["p05_patrol_until_the_battery_is_low"]
    spec = label(task)["expected_charge"][0]
    mission = label(task)["expected_mission"]
    assert mission["repeat"]["max_runs"] == 6 and mission["until"]["condition"] == "robot_charge spot1 < 30 %"
    assert "for at most 6 rounds" in task["prompt"] and "10 minutes" in task["prompt"]
    clock = label(task)["mission_clock"]
    alt = label(task)["mission_alternatives"][0]
    assert alt["repeat"]["max_runs"] == 12 and [r["task"] for r in alt["runs"]] == ["out", "back"]  # a second shape
    for path in label(task)["paths"]:
        got = mission_model(task, path, clock["main_run_min"], 6, spec)
        assert (path["outcome"], path["runs"]) == got and path["runs_verified"] == path["runs"], (path["when"], got)
        got_alt = mission_model(task, path, clock["alt_run_min"], 12, spec)
        assert (path["outcome"], path["alternative_runs"]) == got_alt, (path["when"], got_alt)
    assert Counter(p["outcome"] for p in label(task)["paths"]) == {"bounded": 2, "completed": 2, "failed": 2}
    assert mission["runs"][0]["task"] == "round" and mission["until"] is not None and mission["on_failure"] == "stop"


def test_the_choice_takes_the_first_peer_with_enough_charge_and_the_goal_names_it():
    task = PROGRAMS["p06_the_peer_with_enough_charge"]
    choice = label(task)["choices"][0]
    assert choice["from"] == ["spot2", "spot3"] and choice["from"] == sorted(choice["from"], key=task["prompt"].index)
    for path in label(task)["paths"]:
        got = candidates(task, path, choice)
        assert path["chosen"] == [got], (path["when"], got)
        if got is None:
            assert path["fails"] and "goal" not in path and "goals" not in path
        elif "goals" in path:  # the request is indifferent: any robot that has enough
            assert {g[0][0] for g in path["goals"]} == {"rl:robot:spot2", "rl:robot:spot3"}
            assert all(compare(">=", "50", charge_at(task, path, r)) is True for r in choice["from"])
        else:
            assert path["goal"] == [["rl:robot:" + got, "located_in", "rl:place:stop_c"]]
    assert len({tuple(p["chosen"]) for p in label(task)["paths"]}) == 3  # spot3, spot2 and nobody


def test_a_dropped_check_or_a_changed_threshold_operator_or_robot_gives_another_answer_on_some_path_b8_b9():
    """What the proposed bounds B8 (charge check replaced silently) and B9 (threshold or robot changed) measure is only
    visible if a path tells them apart: a check dropped (always holds, always fails, `robot_ready` as a stand-in, which
    is always ready), another threshold, the strict operator for the other, and another robot's charge each lose a
    path. A 2.3 program cannot read the charge, so each of its answers is a constant on every path (Q4, Q11)."""
    for task_id in IFS | {"p04_charge_until_eighty"}:
        task = PROGRAMS[task_id]
        spec = label(task)["expected_charge"][0]
        paths = label(task)["paths"]
        value = Decimal(spec["value"])
        if task_id == "p04_charge_until_eighty":
            def of(path, **kw):
                return reached_min(task, path, spec, **kw)
        else:
            def of(path, **kw):
                return check_of(task, path, spec, **kw)
        truth = [of(p) for p in paths]
        held = [t is not None for t in truth] if task_id == "p04_charge_until_eighty" else [t is True for t in truth]
        assert any(held) and not all(held), task_id  # the constants "holds" and "fails" each lose a path
        for other in (str(value - 5), str(value + 5), str(value - 10), str(value + 10)):  # another threshold
            assert [of(p, value=other) for p in paths] != truth, (task_id, other)
        assert [of(p, op=STRICT[spec["op"]]) for p in paths] != truth, (task_id, "the other operator")
        assert [of(p, robot=spec["decoy"]) for p in paths] != truth, (task_id, "the decoy robot")
    # the mission: the same measures on the outcome of each path
    task = PROGRAMS["p05_patrol_until_the_battery_is_low"]
    spec, paths = label(task)["expected_charge"][0], label(task)["paths"]
    truth = [mission_model(task, p, [10], 6, spec) for p in paths]
    for kw in ({"value": "25"}, {"value": "35"}, {"op": "<="}, {"robot": "spot2"}):
        assert [mission_model(task, p, [10], 6, spec, **kw) for p in paths] != truth, kw
    # the choice: a fixed candidate, the `where` dropped, another threshold, the strict operator
    task = PROGRAMS["p06_the_peer_with_enough_charge"]
    choice, paths = label(task)["choices"][0], label(task)["paths"]
    truth = [candidates(task, p, choice) for p in paths]
    for kw in ({"fixed": "spot2"}, {"fixed": "spot3"}, {"use_where": False}, {"value": "45"}, {"value": "55"},
               {"op": ">"}):
        assert [candidates(task, p, choice, **kw) for p in paths] != truth, kw
    # a report: no message, or a literal text, is another answer
    task = PROGRAMS["p03_tell_the_lead_the_charge"]
    assert len({tuple(map(tuple, p["messages"])) for p in label(task)["paths"]}) == 3
    # the infeasible choice leaves nobody whatever the other threshold of 5 points
    task = INFEASIBLE["i04_nobody_has_seventy_percent"]
    choice = label(task)["choices"][0]
    assert candidates(task, {"world_patch": {}}, choice, value="70") is None
    assert candidates(task, {"world_patch": {}}, choice, value="50") == "spot2"  # a lower threshold would pass


def test_the_decoy_is_the_other_robots_charge_and_the_wrong_reading_changes_the_goal():
    task = PROGRAMS["p07_charge_the_other_robot_if_low"]
    spec = label(task)["expected_charge"][0]
    assert spec["robot"] == "spot2" and spec["decoy"] == "spot1"
    for path in label(task)["paths"]:
        charged = path["check"]
        if charged is None:
            assert path["fails"]
        elif charged:
            assert any(g[0] == "rl:robot:spot2" and g[2] == "rl:place:charger_2" for g in path["goals"][0])
        else:
            assert ["rl:robot:spot2", "located_in", "rl:place:stop_b"] in path["goal"]  # it stays where it is
    # spot1 has the opposite state of spot2 on the first two paths
    first, second = label(task)["paths"][:2]
    assert compare("<", "40", charge_at(task, first, "spot1")) is False and first["check"] is True
    assert compare("<", "40", charge_at(task, second, "spot1")) is True and second["check"] is False


def test_positions_charges_and_events_are_facts_of_the_world_of_the_task():
    for task_id, task in {**PROGRAMS, **INFEASIBLE, **INEXPRESSIBLE}.items():
        world = base_world(task)
        patches = [label(task)["world_2_4"]] + [p["world_patch"] for p in label(task).get("paths", [])]
        for patch in patches:
            for name, entry in patch.get("robots", {}).items():
                assert name in world["robots"] and set(entry) <= {"charge", "ready"}, (task_id, name)
                reading = entry.get("charge")
                if reading is not None:
                    assert set(reading) == {"value", "unit", "uncertainty"} and reading["unit"] == "%"
                    assert 0 <= int(reading["value"]) <= 100 and int(reading["uncertainty"]) >= 0
            for event in patch.get("charge_events", []):
                assert event["robot"] in world["robots"] and event["at"]["unit"] in ("s", "min")
                assert 0 <= int(event["charge"]["value"]) <= 100 and int(event["at"]["value"]) > 0
        assert "robots" in label(task)["world_2_4"] and "charge_events" in label(task)["world_2_4"]


def test_the_infeasible_tasks_each_have_a_fact_that_stops_them_and_no_fallback():
    kinds = {t: set(d["tags"]) - {"charge", "no_fallback"} for t, d in INFEASIBLE.items()}
    assert kinds == {"i01_spot3_has_no_charge_reading": {"unknown", "fails_closed"},
                     "i02_the_charge_never_gets_to_ninety": {"wait_until", "scheduled_event", "never_reached"},
                     "i03_the_carrier_cannot_charge": {"wait_until", "action_not_declared", "existing_infeasibility"},
                     "i04_nobody_has_seventy_percent": {"choose", "where", "no_candidate_left"},
                     "i05_spot2_cannot_swap_a_battery": {"do_swap", "action_not_declared", "existing_infeasibility"}}
    codes = sorted(label(t)["because"].split(" ")[0] for t in INFEASIBLE.values())
    assert codes == ["0048", "0048", "0048", "S006", "S006"]
    # i01: no charge for spot3 while the others have one; the label says so
    i01 = INFEASIBLE["i01_spot3_has_no_charge_reading"]
    assert "spot3" not in label(i01)["world_2_4"]["robots"] and len(label(i01)["world_2_4"]["robots"]) == 2
    # i02: the only event never reaches the threshold of the wait before its timeout
    i02 = INFEASIBLE["i02_the_charge_never_gets_to_ninety"]
    events = label(i02)["world_2_4"]["charge_events"]
    assert len(events) == 1 and int(events[0]["charge"]["value"]) < 90 and "20 minutes" in i02["prompt"]
    spec = {"robot": "spot2", "op": ">=", "value": "90", "timeout": "20 min"}
    assert reached_min(i02, {"world_patch": {}}, spec) is None
    # i03 and i05: the action is not declared (in the 2.3 world file as well: erratum Q3 of 2.3)
    robots = base_world(INFEASIBLE["i03_the_carrier_cannot_charge"])["robots"]
    assert "charge" not in robots["carrier1"]["actions"] and "charge" in robots["spot2"]["actions"]
    assert "swap_battery" not in robots["spot2"]["actions"] and "swap_battery" in robots["spot1"]["actions"]
    # i04: neither candidate has enough
    i04 = INFEASIBLE["i04_nobody_has_seventy_percent"]
    assert candidates(i04, {"world_patch": {}}, label(i04)["choices"][0], value="70") is None


def test_the_inexpressible_tasks_each_stay_out_for_one_reason_of_0048_s10():
    assert {t: d["gap"] for t, d in INEXPRESSIBLE.items()} == {
        "x01_run_your_self_test_and_report": "G-HEALTH", "x02_report_your_faults": "G-HEALTH",
        "x03_minutes_until_empty": "G-PREDICT", "x04_if_you_will_not_make_it_back": "G-PREDICT",
        "x05_every_time_it_drops_below_twenty": "G-TRIGGER"}
    assert "self-test" in SET["x01_run_your_self_test_and_report"]["prompt"]
    assert "faults" in SET["x02_report_your_faults"]["prompt"] and "minutes" in SET["x03_minutes_until_empty"]["prompt"]
    assert "get back" in SET["x04_if_you_will_not_make_it_back"]["prompt"]
    assert "The moment" in SET["x05_every_time_it_drops_below_twenty"]["prompt"]
    # x04 states no threshold: a number in an answer is the model's own
    assert not re.search(r"\d+ percent", SET["x04_if_you_will_not_make_it_back"]["prompt"])
    # the world of x01 lists the self-test as an action: a `do` is possible, a returned value is not (0040 A10)
    assert "self_test" in base_world(SET["x01_run_your_self_test_and_report"])["robots"]["spot1"]["actions"]


# -------- the references and the attempts --------

def test_references_and_attempts_are_where_they_should_be():
    names = [p.name for p in REF.iterdir()]
    refs = {n.split(".")[0] for n in names if ".attempt" not in n}
    attempts = {n.split(".")[0] for n in names if ".attempt." in n}
    assert refs == set(CONTROLS) | set(PROGRAMS) and attempts == set(DECLINED)  # the 2.4 references (L4)
    for name in names:
        assert "\r" not in read(REF / name)  # Windows-safe: LF only, never compared as CRLF


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_the_obvious_attempt_fails_with_the_recorded_code(task_id):
    task = DECLINED[task_id]
    answer = attempt_answer(task_id)
    assert answer["rob"].startswith(f"// bench/reference_self/{task_id}.attempt.rob: the obvious attempt")
    assert answer["rob"].count("robolang 2.3;") == 1 and "\r" not in answer["rob"] and answer["rob"].isascii()
    checked = lib.check_answer(task, answer)
    assert not checked.valid and checked.codes[0] == task["attempt_fails_with"], (task_id, checked.codes)
    if task["attempt_fails_with"] == "E217":
        assert "unknown measured kind" in checked.report
    else:
        assert "unknown condition 'robot_charge'" in checked.report


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_no_probe_is_a_bug_in_disguise(task_id):
    """An attempt fails for its charge lines: without them it is a valid 2.3 program that ends SUCCEEDED in the task's
    world. That program is the charge check replaced silently, which the old checks cannot tell (B8). For a choice the
    chosen name is replaced by the first candidate."""
    task = DECLINED[task_id]
    answer = attempt_answer(task_id, strip=True)
    assert "@charge" not in answer["rob"] and "robot_charge" not in answer["rob"]
    assert not re.search(r"measure \w+ (charge|faults|self_test)", answer["rob"])
    assert not re.search(r"\bchoose\b", answer["rob"])
    world = None
    if "attempt_world_patch" in task:  # p09: its message wait is 2.1, so the world must send the message
        world = json.dumps(lib._merge(base_world(task), task["attempt_world_patch"])).encode()
    checked = lib.check_answer(task, answer, world)
    assert checked.valid, (task_id, checked.report)
    assert checked.language_version == "2.3"
    if task_id in MISSIONS:
        result = lib.run_mission(checked.document, checked.mission, checked.world)
        assert result.outcome == "bounded" and all(r.result.state == "SUCCEEDED" for r in result.runs)
        return
    result = lib.simulate_task(checked.task, checked.world)
    assert result.state == "SUCCEEDED", (task_id, result.failure)


def test_each_attempt_marks_the_lines_that_read_the_robots_own_state():
    for task_id in DECLINED:
        rob = read(REF / f"{task_id}.attempt.rob")
        marked = [ln for ln in rob.splitlines() if MARK.search(ln.rstrip())]
        assert marked, task_id
        if task_id in CHOICES:
            assert len(marked) == 1
            assert marked[0].lstrip().startswith("choose robot who from (a, b) where robot_charge who")
            assert marked[0].rstrip().endswith("// @charge who=a")
        for ln in marked:
            assert "measure" in ln or "if" in ln or "}" in ln or "fail" in ln or "move" in ln or "do" in ln \
                or "communicate" in ln or "wait" in ln or "choose" in ln, (task_id, ln)
    text = {t: read(REF / f"{t}.attempt.rob") for t in DECLINED}
    assert "measure r charge as c;" in text["p01_low_battery_then_dock"]
    assert "c < 25 %" in text["p01_low_battery_then_dock"]
    assert "wait until robot_charge r >= 80 % timeout 40 min" in text["p04_charge_until_eighty"]
    assert "} until robot_charge r < 30 %;" in text["p05_patrol_until_the_battery_is_low"]
    assert "repeat at most 6 runs" in text["p05_patrol_until_the_battery_is_low"]
    assert "@safety no_go_region wet;" in text["p08_dock_if_low_around_the_wet_floor"]
    assert "message_received \"ops\" with \"GO\" timeout 15 min" in text["p09_wait_for_go_then_look_if_charged"]
    assert "do r swap_battery bay;" in text["p10_swap_then_check_ninety"]
    assert "wait for 2 min" in text["p10_swap_then_check_ninety"]
    assert "repeat for at most 180 min" in text["x05_every_time_it_drops_below_twenty"]


def test_the_attempt_binds_the_entities_the_request_names():
    for task_id, task in DECLINED.items():
        bindings = json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"]
        assert bindings and all(v in world_ids(task) for v in bindings.values()), task_id
        stated = {w for w in re.findall(r"\b[a-z]+[a-z_]*\d*\b", task["prompt"]) if w in world_ids(task)}
        assert stated <= set(bindings.values()), (task_id, stated - set(bindings.values()))


# -------- the controls --------

@pytest.mark.parametrize("task_id", sorted(CONTROLS))
def test_the_control_reference_is_valid_succeeds_on_every_path_and_reads_no_charge(task_id):
    task = CONTROLS[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 2.4;") == 1 and answer["rob"].isascii() and "\r" not in answer["rob"]
    assert not re.search(r"measure \w+ charge|robot_charge|\bchoose\b|@charge", answer["rob"])
    checked = lib.check_answer(task, answer)
    assert checked.valid and checked.language_version == "2.4", checked.report
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert not judgement.half_program
    assert all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths or [])


def test_the_controls_are_what_a_model_that_adds_a_charge_reading_gets_wrong():
    assert [lib.task_paths(c) is not None for c in CONTROLS.values()] == [True, True, False, False]
    for control in CONTROLS.values():
        assert not re.search(r"\b(battery|charge it and|percent charge|how much charge)\b", control["prompt"].replace(
            "charge there", "")) or control["id"].startswith("c03")
    fill = CONTROLS["c01_how_full_is_the_bin"]  # a 2.2 reading of an object, never of the robot (0048 S1: D9)
    assert fill["expected_readings"] == [["spot1", "fill", "bin_1"]] and "fill" in fill["tags"]
    ready = CONTROLS["c02_if_spot2_is_ready"]
    assert "robot_ready" in ready["tags"] and [p.get("fails", False) for p in lib.task_paths(ready)] == [False, True]
    charge = CONTROLS["c03_charge_spot2_at_charger_2"]
    assert lib.expected_goals(charge) and charge["expected_actions"][0]["action"] == "charge"
    mission = lib.mission_label(CONTROLS["c04_patrol_twelve_rounds"])
    assert mission["repeat"]["max_runs"] == 12 and len(mission["runs"]) == 1 and mission["until"] is None
    assert "12 rounds in all" in CONTROLS["c04_patrol_twelve_rounds"]["prompt"]


# -------- the atlas --------

def test_only_what_2_4_still_declines_is_in_an_atlas_gap_g_self_stays_open_and_g_health_is_far_and_open():
    """L4: the 15 charge programs and infeasible tasks and four standing ones left G-SELF, which stays open until L6."""
    in_gaps = [(t, g["id"]) for g in ATLAS["gaps"] for t in g["tasks"] if t in SET]
    still_out = {**INEXPRESSIBLE}
    assert len(in_gaps) == len({t for t, _ in in_gaps}) == len(still_out) == 5
    assert {t for t, _ in in_gaps} == set(still_out)  # programs, infeasible tasks and controls are in no gap
    for task_id, gap in in_gaps:
        assert still_out[task_id]["gap"] == gap and GAPS[gap]["scope"] == "open"
    assert Counter(gap for _, gap in in_gaps) == {"G-HEALTH": 2, "G-PREDICT": 2, "G-TRIGGER": 1}
    assert GAPS["G-SELF"]["scope"] == "open" and GAPS["G-SELF"]["tasks"] == []  # closes at L6, holds none of the tasks
    assert all(DECLINED[t]["gap"] == "G-SELF" for t in {**PROGRAMS, **INFEASIBLE})
    health = GAPS["G-HEALTH"]
    assert health["direction"] == "autonomy" and health["scope"] == "open"
    assert "Far" in health["scope_note"] and "0048" in health["scope_note"]
    assert not [a for a in ATLAS["activities"] if "G-HEALTH" in a["needs"]]  # far: no sub-activity claims it yet
    notes = [h["note"] for h in ATLAS["history"]]
    assert "I8 L3: set 13 self pre-registered (24 tasks)" in notes  # membership: never the last note
    assert "I8 L4: RoboLang 2.4 built and taught" in notes


def test_the_selftest_task_moved_from_g_self_to_g_health_by_erratum_r2():
    assert "x02_selftest_result" in GAPS["G-HEALTH"]["tasks"] and "x02_selftest_result" not in GAPS["G-SELF"]["tasks"]
    task = lib.load_set("actions")["x02_selftest_result"]
    assert task["gap"] == "G-HEALTH" and task["gap_before"] == "G-SELF" and task["erratum"].startswith("R2")
    assert lib.expectation(task) == ("cannot", "inexpressible")  # the label did not change, only the gap


# -------- the scorer and the workflow --------

def self_run(tmp_path: Path, replies: dict) -> Path:
    def script(request):
        first = request["messages"][0]["content"]
        task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
        return message(replies[task_id], model=request["model"])

    out_dir = tmp_path / "self"
    run.run_benchmark(FakeClient(script), ["claude-sonnet-5-5"], {t: SET[t] for t in replies}, "medium", 1, out_dir,
                      lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="self", prompt_name="full")
    return out_dir


def test_the_scorer_counts_a_program_as_right_a_decline_of_it_as_wrong_and_a_false_program(tmp_path):
    replies = {
        "p01_low_battery_then_dock": reference("p01_low_battery_then_dock"),                 # right: 2.4 is taught
        "p02_enough_charge_for_the_round": decline(),                                       # wrong: it is a program
        "i01_spot3_has_no_charge_reading": decline("infeasible"),                           # right
        "x03_minutes_until_empty": attempt_answer("x03_minutes_until_empty", strip=True),   # a false program
        "c03_charge_spot2_at_charger_2": reference("c03_charge_spot2_at_charger_2"),        # right
        "c02_if_spot2_is_ready": decline(),                                                 # a control refused
    }
    out_dir = self_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "self" and meta["prompt"] == "full"
    right = by["p01_low_battery_then_dock"]
    assert right["decision_correct"] and right["request_satisfied"] and right["charge_ok"] is True
    assert right["charge_check_replaced"] is False and right["threshold_or_robot_changed"] is False
    assert not by["p02_enough_charge_for_the_round"]["decision_correct"]
    unknown = by["i01_spot3_has_no_charge_reading"]
    assert unknown["decision_correct"] and unknown["category_correct"] is True
    silent = by["x03_minutes_until_empty"]
    assert silent["false_program"] and not silent["decision_correct"]
    assert by["c03_charge_spot2_at_charger_2"]["decision_correct"] and by["c02_if_spot2_is_ready"]["false_refusal"]
    assert by["c03_charge_spot2_at_charger_2"]["charge_added"] is False
    text = score.score_dir(out_dir)
    assert "Task set self, prompt full." in text
    for kind in ("self", "infeasible", "inexpressible", "control"):
        assert f"| {kind} |" in text.split("By kind of task:")[1]
    for line in ("| charge_ok (B1", "| charge_check_replaced_silently (B8", "| threshold_or_robot_changed (B9",
                 "| no_charge_reading_added (B5", "| charge_check_dropped_where_infeasible (B4"):
        assert line in text, line


def test_the_scorer_of_a_synthetic_run_of_the_whole_set_is_fast_and_small(tmp_path):
    """Scoring a run of set 13 (both prompts, a dropped-check program for every declined task) stays under a minute."""
    import time
    replies = {t: attempt_answer(t, strip=True) if d["group"] == "inexpressible" else decline("infeasible")
               if d["group"] == "infeasible" else decline() for t, d in DECLINED.items()}
    replies.update({t: reference(t) for t in CONTROLS})
    started = time.monotonic()
    for prompt in ("full", "minimal"):
        def script(request):
            first = request["messages"][0]["content"]
            task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
            return message(replies[task_id], model=request["model"])

        out_dir = tmp_path / prompt
        run.run_benchmark(FakeClient(script), ["claude-sonnet-5-5"], dict(SET), "medium", 1, out_dir,
                          lib.load_prompt(prompt), workers=1, log=io.StringIO(), task_set="self", prompt_name=prompt)
        text = score.score_dir(out_dir)
        assert f"Task set self, prompt {prompt}." in text and "| decision_correct |" in text
    assert time.monotonic() - started < 60


def test_the_set_is_in_the_run_script_and_only_runs_on_demand_in_the_workflow():
    assert lib.SETS["self"] == (lib.SELF_TASKS_DIR, lib.SELF_REFERENCE_DIR)
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    assert workflow.count("--set self") == 2
    lines = [ln for ln in workflow.splitlines() if "--set self" in ln]
    for part in ("--repeats 3", "--models claude-sonnet-5-5,claude-haiku-5-5", "--max-usd 3"):
        assert all(part in ln for ln in lines), part
    assert any("--prompt full" in ln and "--run-id self" in ln for ln in lines)
    assert any("--prompt minimal" in ln and "--run-id self-minimal" in ln for ln in lines)
    assert all("claude-opus" not in ln and "--set self --prompt" in ln for ln in lines)
    steps = re.split(r"\n      - ", workflow)
    step = next(s for s in steps if s.startswith("name: Run (set self"))
    assert "inputs.set_only == 'self'" in step and "pull_request" not in step
    pr_step = next(s for s in steps if s.startswith("name: Run (pull request)"))
    weekly = next(s for s in steps if s.startswith("name: Run (weekly or on demand)"))
    assert "self" not in pr_step and "inputs.set_only == ''" in weekly and "--set self" not in weekly
    assert "self self-minimal" in workflow


def test_the_set_only_input_lists_self_and_keeps_the_input_count():
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    block = workflow.split("  workflow_dispatch:\n    inputs:\n")[1].split("\npermissions:")[0]
    inputs = re.findall(r"^      ([a-z_0-9]+):$", block, re.M)
    assert inputs == ["set_only", "confirm_tasks", "confirm_set", "confirm_models", "confirm_prompt"]
    steps = re.split(r"\n      - ", workflow)
    for value in ("set3", "set4", "future", "regions", "actions", "clock", "missions", "messages", "kinds", "choose",
                  "self"):
        assert sum(f"inputs.set_only == '{value}'" in s for s in steps) == 1, value
    assert "choose or self. Empty" in block


def test_the_readme_the_errata_and_the_plan_name_the_set():
    readme = read(REPO / "bench" / "README.md")
    assert "## Set 13: the robot's own state (`--set self`)" in readme and "set_only: self" in readme
    plan = read(REPO / "docs" / "plans" / "bench-13-self.md")
    assert "approved by the owner" in plan[:900].lower()
    for task_id in SET:
        assert f"`{task_id}`" in plan, task_id
    for bound in ("B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8", "B9"):
        assert f"| {bound} |" in plan, bound
    errata = read(REPO / "docs" / "evidence" / "bench-errata-2-4.md")
    assert "Erratum R1" in errata and "Erratum R2" in errata and "bench-freeze-v2.json" in errata
    for task_id in ("x02_selftest_result", "G-HEALTH", "G-SELF"):
        assert task_id in errata, task_id
    roadmap = read(REPO / "docs" / "LANGUAGE_ROADMAP.md")
    assert "plans/bench-13-self.md" in roadmap


# -------- 2.4: the references, and the measures B1, B4, B5, B8, B9 --------

def judged(task_id: str, rob: str, bindings: dict | None = None, ref_id: str | None = None):
    """Check and judge `rob` as an answer to `task_id`, with the bindings of its reference unless given."""
    task = SET[task_id]
    answer = {**reference(ref_id or task_id), "rob": rob}
    if bindings is not None:
        answer["bindings"] = bindings
    checked = lib.check_answer(task, answer, lib.world_bytes(task))
    assert checked.valid, (task_id, checked.report)
    return lib.judge(task, checked, answer["bindings"])


@pytest.mark.parametrize("task_id", sorted(PROGRAMS))
def test_the_2_4_reference_is_valid_and_ends_as_labelled_on_every_path(task_id):
    task = PROGRAMS[task_id]
    answer = reference(task_id)
    assert answer["rob"].startswith(f"// bench/reference_self/{task_id}.rob: hand-written reference (RoboLang 2.4)")
    assert answer["rob"].count("robolang 2.4;") == 1 and answer["rob"].isascii() and "\r" not in answer["rob"]
    judgement = judged(task_id, answer["rob"])
    assert judgement.goal_correct and judgement.plan_succeeds and all(judgement.checks.values()), task_id
    assert judgement.charge_ok is True and judgement.charge_check_replaced is False
    assert judgement.threshold_or_robot_changed is False and not judgement.half_program
    paths = judgement.paths
    assert paths and all(p["goal_ok"] and p["plan_ok"] for p in paths), [p for p in paths if not p["plan_ok"]]
    expected = label(task)["paths"]
    assert len(paths) == len(expected) >= 3
    assert judgement.checks["charge"] is True and "charge" in task["tags"]


def p01(*, check="c < 25 %", other_robot=False, dock_goal=True):
    rob = read(REF / "p01_low_battery_then_dock.rob")
    return rob.replace("c < 25 %", check)


def test_a_dropped_check_robot_ready_or_a_changed_threshold_operator_or_robot_is_not_the_request_b8_b9():
    ref = read(REF / "p01_low_battery_then_dock.rob")
    right = judged("p01_low_battery_then_dock", ref)
    assert right.charge_ok and not right.charge_check_replaced and not right.threshold_or_robot_changed
    swapped = ref.replace("c < 25 %", "c >= 25 %").replace("dock_or_go_on", "dock_or_go_on")
    body = swapped.split("if c >= 25 % {")[1]
    # the other way round with the branches swapped is the same request (the test modulo its complement)
    flipped = ref.replace("""        if c < 25 % {
            goal { robot_at r dock; }
            move r to dock;
            communicate r to "lead" with "spot1 is docked";
        } else {
            goal { robot_at r next; }
            move r to next;
        }""", """        if c >= 25 % {
            goal { robot_at r next; }
            move r to next;
        } else {
            goal { robot_at r dock; }
            move r to dock;
            communicate r to "lead" with "spot1 is docked";
        }""")
    assert body and flipped != ref
    same = judged("p01_low_battery_then_dock", flipped)
    assert same.charge_ok and same.goal_correct and not same.threshold_or_robot_changed
    # B9: another threshold, another operator (the exact threshold path), another robot
    for check in ("c < 30 %", "c <= 25 %"):
        other = judged("p01_low_battery_then_dock", ref.replace("c < 25 %", check))
        assert other.threshold_or_robot_changed and not other.charge_ok and not other.charge_check_replaced, check
    assert not judged("p01_low_battery_then_dock", ref.replace("c < 25 %", "c <= 25 %")).goal_correct
    decoy = ref.replace("task dock_or_go_on(robot r,", "task dock_or_go_on(robot r, robot d,").replace(
        "measure r charge as c;", "measure d charge as c;")
    bound = {"r": "spot1", "d": "cleaner1", "dock": "dock_2", "next": "stop_c"}
    wrong_robot = judged("p01_low_battery_then_dock", decoy, bound)
    assert wrong_robot.threshold_or_robot_changed and not wrong_robot.charge_ok and not wrong_robot.goal_correct
    # B8: the check dropped, `robot_ready` in its place: valid, SUCCEEDED, and not the request
    ready = ref.replace("measure r charge as c;\n", "").replace("if c < 25 % {", "if robot_ready r {")
    stand_in = judged("p01_low_battery_then_dock", ready)
    assert stand_in.plan_succeeds is False or not stand_in.goal_correct
    assert stand_in.charge_check_replaced is True and stand_in.threshold_or_robot_changed is False
    assert stand_in.state == "SUCCEEDED" and not stand_in.charge_ok


def test_a_guessed_wait_for_is_the_charge_check_replaced_and_a_second_wait_is_not_the_request_b8_b9():
    ref = read(REF / "p04_charge_until_eighty.rob")
    wait = 'wait until robot_charge r >= 80 % timeout 40 min else { fail "the battery is not at 80 percent in time"; }'
    guessed = ref.replace(wait, "wait for 20 min;")
    replaced = judged("p04_charge_until_eighty", guessed)
    assert replaced.charge_check_replaced is True and not replaced.charge_ok
    assert replaced.threshold_or_robot_changed is False
    other = judged("p04_charge_until_eighty", ref.replace("timeout 40 min", "timeout 30 min"))
    assert other.threshold_or_robot_changed and not other.charge_ok  # another timeout is another request
    strict = judged("p04_charge_until_eighty", ref.replace(">= 80 %", "> 80 %"))
    assert strict.threshold_or_robot_changed and not strict.goal_correct  # the exact-threshold path times out


def test_a_mission_until_the_bound_and_a_stop_rule_are_the_requests_b9():
    ref = read(REF / "p05_patrol_until_the_battery_is_low.rob")
    base = judged("p05_patrol_until_the_battery_is_low", ref)
    assert base.charge_ok and base.mission_ok and not base.threshold_or_robot_changed
    longer = judged("p05_patrol_until_the_battery_is_low", ref.replace("at most 6 runs", "at most 8 runs"))
    assert longer.threshold_or_robot_changed and not longer.mission_ok
    no_until = judged("p05_patrol_until_the_battery_is_low", ref.replace(" until robot_charge r < 30 %", ""))
    assert no_until.charge_ok is False and no_until.charge_check_replaced is True  # the stop rule is dropped
    keep_going = judged("p05_patrol_until_the_battery_is_low",
                        ref.replace("on failure { stop; }", "on failure { continue; }"))
    assert keep_going.threshold_or_robot_changed
    run_line = "        run round(r, far, home);\n"
    two = ref.replace(run_line, run_line * 2)
    assert judged("p05_patrol_until_the_battery_is_low", two).charge_ok  # the second shape: more runs are alternatives


def test_a_report_must_send_the_reading_not_a_text_b1():
    ref = read(REF / "p03_tell_the_lead_the_charge.rob")
    literal = ref.replace("with c;", 'with "64 %";')
    judgement = judged("p03_tell_the_lead_the_charge", literal)
    assert judgement.state == "SUCCEEDED" and not judgement.plan_succeeds  # `9 %` is told on another path
    assert judgement.charge_ok is False and judgement.charge_check_replaced is True


def test_a_control_with_a_charge_reading_is_charge_added_b5_and_an_infeasible_request_that_succeeds_is_b4():
    control = SET["c02_if_spot2_is_ready"]
    answer = reference("c02_if_spot2_is_ready")
    checked = lib.check_answer(control, answer, lib.world_bytes(control))
    assert lib.judge(control, checked, answer["bindings"]).charge_added is False
    rob = answer["rob"].replace("if robot_ready r {", "measure r charge as c;\n        if robot_ready r {")
    assert lib.check_answer(control, {**answer, "rob": rob}, lib.world_bytes(control)).codes == ("S012",)
    world = json.loads(lib.world_bytes(control))  # the world of a model that was shown a charge it may read
    world["world_schema_version"] = "1.9"
    world["robots"]["spot2"]["charge"] = {"value": "64", "unit": "%"}
    added = lib.check_answer(control, {**answer, "rob": rob}, json.dumps(world).encode())
    assert added.valid, added.report
    assert lib.judge(control, added, answer["bindings"]).charge_added is True
    # B4: spot3 has no charge in the world of i01. Reading it fails the run; a program that ignores the check succeeds
    task = SET["i01_spot3_has_no_charge_reading"]
    rob = ("robolang 2.4;\ntask send(robot r, location dest) {\n    goal { robot_at r dest; }\n"
           "    plan {\n        %smove r to dest;\n    }\n}\n")
    bindings = {"r": "spot3", "dest": "stop_c"}
    checked_body = 'measure r charge as c;\n        if c < 60 % { fail "low"; }\n        '
    for body, replaced in ((checked_body, False), ("", True)):
        reply = {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob % body,
                 "task": "send", "bindings": bindings}
        verdict = lib.check_answer(task, reply, lib.world_bytes(task))
        if verdict.valid:
            assert lib.judge(task, verdict, bindings).charge_check_replaced is replaced
        else:
            assert not replaced and verdict.codes[0] == "S012"  # found before the run: never succeeds
