"""
Benchmark set 12, "choose" (bench/tasks_choose/, decision record 0045, plan docs/plans/bench-12-choose.md): choosing
at run time, `choose TYPE NAME from CANDIDATES [where COND] [nearest to X | farthest from X] else { ... }`, pre-
registered against RoboLang 2.2 before any code (L3 of iteration I7) and labelled for 2.3 in L4. `expect` is the 2.3
label (what the acceptance run scores); `expect_2_2` is the 2.2 label (the baseline: every non-control task declined as
inexpressible, because 2.2 has no `choose`: E203, and no position to measure a distance from); `expect_2_3` keeps the
pre-registered label: `world_2_3` (the world file 1.8 facts, now in the world copies `bench/worlds/choose_<id>.json`),
the paths of a program (each forced by a `world_patch` that changes which candidate wins) and, for a mission, the
mission
label. The checks are task keys: `expected_choices`, `expected_limits`, `expected_mission` (erratum Q6).

11 `choose` requests that are programs in 2.3, 5 `infeasible` in 2.3 (erratum Q3: their 2.2 world says the opposite, so
the 2.3 world copy carries the fact), 5 `inexpressible` that stay out (0045 C9: G-EACH, G-PROX, G-DETECT), 4 `control`
requests that are programs in both versions (a fixed carry, two readings compared, a fixed mission, a clock wait). Each
declined task keeps its obvious 2.2 attempt (a `choose` statement; 2.2 rejects it with E203; the line is marked `//
@choice
NAME=ENTITY`) as evidence, and each program has a 2.3 reference, valid and ending as labelled on every path. No network
and no SDK; the model run is the `Benchmark` workflow with `set_only: choose`.
"""

from __future__ import annotations
import io
import json
import re
import sys
from collections import Counter
from pathlib import Path

import pytest
from test_bench import FakeClient, message

from bench import lib, run, score

REPO = Path(__file__).resolve().parent.parent
SET = lib.load_set("choose")
REF = lib.CHOOSE_REFERENCE_DIR
ATLAS = json.loads((REPO / "bench" / "atlas.json").read_text(encoding="utf-8"))
GAPS = {g["id"]: g for g in ATLAS["gaps"]}
CANNOT = {"decision": "cannot", "category": "inexpressible"}
PROGRAMS = {t: d for t, d in SET.items() if d["group"] == "choose"}
INFEASIBLE = {t: d for t, d in SET.items() if d["group"] == "infeasible"}
INEXPRESSIBLE = {t: d for t, d in SET.items() if d["group"] == "inexpressible"}
CONTROLS = {t: d for t, d in SET.items() if d["group"] == "control"}
DECLINED = {**PROGRAMS, **INFEASIBLE, **INEXPRESSIBLE}  # what 2.2 declines: an attempt each
MISSIONS = {"p09_parts_for_six_minutes", "p11_parts_until_five"}
NEW_WORLDS = {"bin_hall.json", "parts_line.json"}  # the plan allows at most 3
MARK = re.compile(r"// @choice (\w+)=(\w+)\s*$")


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def label(task: dict) -> dict:
    return task["expect_2_3"]


def base_world(task: dict) -> dict:
    return json.loads(read(REPO / task["world"]))


def location_ids(world: dict) -> list[str]:
    return [item["id"] if isinstance(item, dict) else item for item in world["locations"]]


def world_ids(task: dict) -> set[str]:
    world = base_world(task)
    return {*world["robots"], *world["objects"], *location_ids(world), *world.get("sensors", {}),
            *world.get("regions", {})}


def choices(task: dict) -> list[dict]:
    return task["expected_choices"]


def world_2_2(task: dict) -> dict:
    """The 2.2 world file the task was pre-registered on (the baseline showed it): no position."""
    return json.loads(read(REPO / task["world_2_2"]))


def reference(task_id: str) -> dict:
    ref = json.loads(read(REF / f"{task_id}.json"))
    rob = read(REF / f"{task_id}.rob")
    return {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob, **ref}


def decline(category: str = "inexpressible") -> dict:
    return {"decision": "cannot", "category": category, "reason": "cannot be done", "rob": "", "task": "",
            "bindings": []}


def strip_choice(rob: str, bindings: dict) -> tuple[str, dict]:
    """The attempt without its choice: the marked line goes, and the chosen name is replaced everywhere by the entity
    the marker names, the first candidate (a parameter of the task, or a world entity that becomes one): a fixed
    pick."""
    lines = rob.splitlines(keepends=True)
    marked = [ln for ln in lines if MARK.search(ln.rstrip())]
    assert len(marked) == 1
    kept = "".join(ln for ln in lines if ln not in marked)
    name, entity = MARK.search(marked[0].rstrip()).groups()
    kind = re.search(rf"choose (\w+) {name}\b", marked[0]).group(1)
    head = re.search(r"^(task \w+)\((.*)\) \{$", kept, re.M)
    params = [p for p in head.group(2).split(", ") if p]
    bindings = dict(bindings)
    body = kept[head.end():]
    if entity not in {p.split()[-1] for p in params}:  # a world entity: it becomes a parameter, bound to itself
        params.append(f"{kind} {entity}")
        bindings[entity] = entity
        if "mission " in body:  # the mission and its run lines carry it too
            task_name = head.group(1).split()[1]
            body = re.sub(r"(mission \w+\([^)]*)\)", rf"\1, {kind} {entity})", body)
            body = re.sub(rf"(run {task_name}\([^)]*)\)", rf"\1, {entity})", body)
    body = re.sub(rf"\b{name}\b", entity, body)
    params = [p for p in params if re.search(rf"\b{p.split()[-1]}\b", body)]
    out = kept[:head.start()] + f"{head.group(1)}({', '.join(params)}) {{" + body
    return out, {k: v for k, v in bindings.items() if k in {p.split()[-1] for p in params}}


def attempt_answer(task_id: str, strip: bool = False) -> dict:
    stem = REF / f"{task_id}.attempt"
    rob, data = read(Path(f"{stem}.rob")), json.loads(read(Path(f"{stem}.json")))
    if strip:
        rob, data["bindings"] = strip_choice(rob, data["bindings"])
    return {"decision": "program", "category": "none", "reason": "an attempt", "entities": [], "rob": rob, **data}


# ---- a small model of 0045 C2 to C5, for the labels: which candidate does a choice take in a world? ----

def positions(task: dict, patch: dict | None = None) -> dict:
    found = {item["id"]: tuple(item["position"]) for item in base_world(task)["locations"]
             if isinstance(item, dict)}  # the world file 1.8 copy carries the notes
    for name, value in ((patch or {}).get("positions") or {}).items():
        if value is None:
            found.pop(name, None)
        else:
            found[name] = tuple(value)
    return found


def world_of(task: dict, patch: dict | None = None) -> dict:
    return lib._merge(base_world(task), {k: v for k, v in (patch or {}).items() if k in ("robots", "objects", "clock")})


def where_is(world: dict, entity: str) -> str:
    if entity in world["robots"]:
        return world["robots"][entity]["location"]
    if entity in world["objects"]:
        return world["objects"][entity]["location"]
    return entity  # a location


def candidates(world: dict, spec) -> list[str]:
    if spec == "robots":
        return list(world["robots"])
    if isinstance(spec, dict):
        return [o for o, v in world["objects"].items()
                if v.get("present", True) and v["location"] == spec["objects_at"]]
    return list(spec)


def choose(task: dict, choice: dict, patch: dict | None = None, *, order: str | None = "as written",
           use_where: bool = True) -> str | None:
    """The first candidate in the order the choice asks for, or None. `order` and `use_where` let a test drop them."""
    world, where = world_of(task, patch), positions(task, patch)
    found = candidates(world, choice["from"])
    if use_where and choice["where"] == "robot_ready":
        found = [c for c in found if world["robots"][c].get("ready") is True]
    asked = choice["order"] if order == "as written" else order
    if not asked:
        return found[0] if found else None
    (kind, ref), = asked.items()
    if where_is(world, ref) not in where and ref not in where:
        return None  # the reference point has no position: unknown, every candidate is left out
    centre = where.get(ref) or where[where_is(world, ref)]
    keyed = []
    for c in found:
        place = where_is(world, c)
        if place in where or c in where:
            x, y = where.get(c) or where[place]
            keyed.append(((x - centre[0]) ** 2 + (y - centre[1]) ** 2, len(keyed), c))
    keyed.sort(key=lambda k: (-k[0], k[1]) if kind == "farthest_from" else (k[0], k[1]))
    return keyed[0][2] if keyed else None


def mission_run(task: dict, path: dict) -> dict:
    """A mission over `objects at L` under 0041 and 0045 C10: the bound is checked before each run, the `until` after
    it, a part that was moved is no longer a candidate, and a run that finds none fails the mission."""
    mission, world = task["expected_mission"], world_of(task, path["world_patch"])
    wait = mission["runs"][0]["wait_min"]
    limit = (mission["repeat"]["max_duration"] or {}).get("value")
    start = world["clock"]["start"][11:16]
    clock = int(start[:2]) * 60 + int(start[3:])
    until = mission["until"] and mission["until"]["condition"].split(">= ")[1]
    elapsed, chosen, runs, outcome = 0, [], 0, None
    while outcome is None:
        if limit is not None and elapsed >= limit:
            outcome = "bounded"
            break
        runs += 1
        pick = choose(task, choices(task)[0], {"objects": {k: v for k, v in world["objects"].items()}})
        chosen.append(pick)
        if pick is None:
            outcome = "failed"
            break
        world["objects"][pick]["location"] = "bin"
        elapsed += wait
        if until and clock + elapsed >= int(until[:2]) * 60 + int(until[3:]):
            outcome = "completed"
    verified = runs - (1 if outcome == "failed" else 0)
    return {"chosen": chosen, "outcome": outcome, "runs": runs, "runs_verified": verified}


# -------- the set --------

def test_the_mix_of_tasks():
    assert len(SET) == 25
    assert Counter(d["group"] for d in SET.values()) == {
        "choose": 11, "infeasible": 5, "inexpressible": 5, "control": 4}
    prefixes = {"choose": "p", "infeasible": "i", "inexpressible": "x", "control": "c"}
    for task_id, task in SET.items():
        assert task["id"] == task_id and (lib.CHOOSE_TASKS_DIR / f"{task_id}.json").is_file()
        assert (REPO / task["world"]).is_file() and task["tags"] and task["prompt"].endswith(".")
        assert task["prompt"].isascii() and "\r" not in read(lib.CHOOSE_TASKS_DIR / f"{task_id}.json")
        assert re.fullmatch(prefixes[task["group"]] + r"\d\d_[a-z0-9_]+", task_id), task_id


def test_the_ids_are_new_ones_and_no_prompt_is_a_copy_of_a_standing_task():
    """The R3 probes and the standing tasks of the gap stay in their sets (as set 11 left its probes); this set has new
    ids and new prompts for the same shapes."""
    elsewhere = {}
    for name in lib.SETS:
        if name != "choose":
            elsewhere.update(lib.load_set(name))
    assert not set(SET) & set(elsewhere)
    assert not {t["prompt"] for t in SET.values()} & {t["prompt"] for t in elsewhere.values()}
    assert {"f_choose_nearest_tugger", "f_choose_nearest_dock", "f_choose_nearer_cart", "x_nearest_robot",
            "x05_every_new_part", "x_shuttle_until_five"} <= set(elsewhere)


def test_the_two_new_1_7_worlds_and_a_world_file_1_8_copy_for_each_program_and_infeasible_task():
    """The plan allows at most 3 new worlds: bin_hall and parts_line (world file 1.7, no position). L4 adds the world
    file 1.8 copies of erratum Q6, which carry the `world_2_3` notes (erratum Q3: what makes i03, i04, i05
    infeasible)."""
    assert len(NEW_WORLDS) <= 3
    for name in NEW_WORLDS:
        world = json.loads(read(REPO / "bench" / "worlds" / name))
        assert world["world_schema_version"] == "1.7" and "position" not in read(REPO / "bench" / "worlds" / name)
    for task_id, task in SET.items():
        if task["group"] in ("choose", "infeasible"):
            assert task["world"] == f"bench/worlds/choose_{task_id}.json", task_id
            world = base_world(task)
            assert world["world_schema_version"] == "1.8" and world["tasks"] == {}
            notes = label(task)["world_2_3"]
            assert positions(task) == {k: tuple(v) for k, v in notes.get("positions", {}).items()}, task_id
            for name, entry in notes.get("robots", {}).items():
                assert all(world["robots"][name][k] == v for k, v in entry.items()), (task_id, name)
            for name, entry in notes.get("objects", {}).items():
                assert all(world["objects"][name][k] == v for k, v in entry.items()), (task_id, name)
            before = world_2_2(task)  # nothing else differs from the 2.2 world the baseline showed
            assert before["world_schema_version"] != "1.8" and location_ids(world) == location_ids(before)
            assert set(world["robots"]) == set(before["robots"]) and set(world["objects"]) == set(before["objects"])
        else:
            assert task["world"].startswith("bench/worlds/") and "world_2_2" not in task
            assert "choose_" not in task["world"], task_id


def test_expect_is_the_2_3_label_the_2_2_label_is_in_expect_2_2_and_the_checks_are_task_keys():
    for task_id, task in SET.items():
        if task["group"] == "control":
            assert task["expect"] == label(task) == {"decision": "program"}
            assert task["expect_2_2"]["decision"] == "program"
            assert not {"why", "attempt_fails_with", "gap", "blockers", "expected_choices"} & set(task)
            assert lib.expectation(task) == ("program", "none")
            continue
        assert task["expect_2_2"] == CANNOT, task_id  # the baseline: 2.2 declines every non-control task
        assert len(task["why"]) > 40 and task["gap"] in GAPS and task["attempt_fails_with"] == "E203"
        assert not {"expected_goal", "expected_goals", "paths"} & set(task), task_id
        assert task["expect"] == {k: label(task)[k] for k in ("decision", "category") if k in label(task)}, task_id
        assert "choices" not in label(task) and "expected_mission" not in label(task), task_id  # moved to task keys
    for task_id, task in PROGRAMS.items():
        later = label(task)
        assert task["expect"] == {"decision": "program"} and lib.expectation(task) == ("program", "none"), task_id
        assert later["decision"] == "program" and choices(task) and later["world_2_3"], task_id
        assert len({p["when"] for p in later["paths"]}) == len(later["paths"]) >= 2
        assert task["gap"] == "G-CHOOSE"
        assert ("expected_mission" in task) == (task_id in MISSIONS)
        assert lib.task_paths(task) == (None if task_id in MISSIONS else later["paths"])
        assert all(c.get("ladder_ok") for c in choices(task)) == (task_id[:3] in ("p05", "p09", "p11"))  # erratum Q4
    for task_id, task in INFEASIBLE.items():
        later = label(task)
        assert later["decision"] == "cannot" and later["category"] == "infeasible", task_id
        assert task["expect"] == {"decision": "cannot", "category": "infeasible"}
        assert task["blockers"] and task["gap"] == "G-CHOOSE" and later["because"] and choices(task), task_id
        assert "paths" not in later and lib.task_paths(task) is None
    for task_id, task in INEXPRESSIBLE.items():
        later = label(task)
        assert {k: later[k] for k in ("decision", "category")} == CANNOT and later["because"].startswith("0045 C9")
        assert task["expect"] == CANNOT and "expected_choices" not in task


def test_the_programs_cover_what_the_pre_registration_lists():
    tags = Counter(tag for t in PROGRAMS.values() for tag in t["tags"])
    for tag in ("list", "locations", "robots", "nearest", "farthest", "where", "robot_ready", "decoy", "objects_at",
                "all_robots", "tie", "order_written", "mission", "duration_bound", "until_clock", "choose_afresh",
                "region_limit", "message", "named_in_goal", "else_runs", "no_position_excluded"):
        assert tags[tag] >= 1, tag
    forms = Counter((c["type"], "list" if isinstance(c["from"], list) else "set")
                    for t in PROGRAMS.values() for c in choices(t))
    assert forms[("robot", "set")] >= 1 and forms[("object", "set")] >= 1 and forms[("location", "list")] >= 1
    assert forms[("robot", "list")] >= 1  # the three forms of 0045 C2: a list, `objects at L`, `robots`
    # the 1.7 limit, the 1.9 clock and a message that go with a choice
    assert SET["p10_dock_near_stop_c_around_the_wet_floor"]["expected_limits"] == [["no_go_region", "wet_floor"]]
    assert label(SET["p11_parts_until_five"])["expected_clock"][0]["value"] == "17:00"
    assert label(SET["p01_nearer_tugger_to_the_trailer"])["paths"][0]["messages"] == [["ops", "tugger sent"]]
    assert SET["p10_dock_near_stop_c_around_the_wet_floor"]["world_2_2"].endswith("site_1_3.json")  # honours it
    # an order or a filter in each choice, and a choice in each program
    for task in PROGRAMS.values():
        for choice in choices(task):
            assert choice["type"] in ("robot", "object", "location") and (choice["order"] or choice["where"]
                                                                          or isinstance(choice["from"], dict))


# -------- the labels follow 0045: a choice, its paths, and what a world patch changes --------

def prompt_order(task: dict, names: list[str]) -> list[str]:
    positions_in_prompt = [task["prompt"].index(n) for n in names]
    return [n for _, n in sorted(zip(positions_in_prompt, names))]


@pytest.mark.parametrize("task_id", sorted(set(PROGRAMS) - MISSIONS))
def test_each_path_chooses_the_candidate_the_label_says_and_the_goal_names_it(task_id):
    task = PROGRAMS[task_id]
    choice = choices(task)[0]
    if isinstance(choice["from"], list):
        assert choice["from"] == prompt_order(task, choice["from"]), task_id  # the list is in the order written
        assert set(choice["from"]) <= world_ids(task)
    for path in label(task)["paths"]:
        got = choose(task, choice, path["world_patch"])
        assert path["chosen"] == [got], (task_id, path["when"], got)
        goals = path.get("goals") or [path.get("goal")]
        if got is None:
            assert path["fails"] and "goal" not in path and "goals" not in path
            continue
        assert not path.get("fails")
        if choice["type"] in ("robot", "location") and any(g[0].startswith("rl:robot:") for g in goals[0]):
            assert (choice["type"] == "robot") == (goals[0][0][0] == f"rl:robot:{got}"), (task_id, path["when"])
        if choice["type"] == "location":
            assert goals[0][0][2] == f"rl:place:{got}", (task_id, path["when"])
        if choice["type"] == "object":
            assert any(g[0][0] == f"rl:thing:{got}" for g in goals), (task_id, path["when"])
        for goal_set in goals:
            for subject, prop, value in goal_set:
                assert prop == "located_in"
                assert {subject.split(":")[2], value.split(":")[2]} <= world_ids(task)
    wins = {tuple(p["chosen"]) for p in label(task)["paths"]}
    assert len(wins) == len(label(task)["paths"])  # a patch changes who wins


@pytest.mark.parametrize("task_id", sorted(MISSIONS))
def test_a_mission_chooses_afresh_and_stops_as_the_bound_or_the_until_says(task_id):
    task = PROGRAMS[task_id]
    mission = task["expected_mission"]
    assert mission["on_failure"] == "stop" and mission["policy_stated"] is True
    assert (mission["until"] is None) != (mission["repeat"]["max_duration"] is None)  # one bound, as stated
    assert mission["repeat"]["max_runs"] is None  # no count of runs is asked for, so none may be invented (0041)
    for path in label(task)["paths"]:
        got = mission_run(task, path)
        want = {k: path[k] for k in ("chosen", "outcome", "runs", "runs_verified")}
        assert got == want, (task_id, path["when"], got)
        assert path["parts_moved"] == len([c for c in got["chosen"] if c])
        assert len(set(c for c in got["chosen"] if c)) == path["parts_moved"]  # a moved part is not chosen again
    outcomes = [p["outcome"] for p in label(task)["paths"]]
    assert "failed" in outcomes and ("bounded" in outcomes or "completed" in outcomes)
    if task_id == "p11_parts_until_five":
        world = base_world(task)
        assert world["clock"]["start"].endswith("16:30:00") and "until" in task["prompt"] and "17:00" in task["prompt"]
    else:
        assert "6 minutes" in task["prompt"] and "2 minutes" in task["prompt"]


def test_a_fixed_candidate_or_a_dropped_part_of_the_choice_gives_another_answer_on_some_path_b8_b9():
    """What the proposed bounds B8 (choice replaced silently) and B9 (criterion changed) measure is only visible if a
    path tells them apart: the first candidate, the order dropped, the `where` dropped and the other order lose a
    path."""
    for task_id, task in PROGRAMS.items():
        if task_id in MISSIONS:
            continue
        choice, paths = choices(task)[0], label(task)["paths"]
        first = candidates(world_of(task), choice["from"])[0]  # the first candidate the world or the request lists
        assert any(p["chosen"] != [first] for p in paths), (task_id, "first candidate")
        if choice["order"]:
            (kind, ref), = choice["order"].items()
            other = {"nearest_to": "farthest_from", "farthest_from": "nearest_to"}[kind]
            assert any(choose(task, choice, p["world_patch"], order={other: ref}) != p["chosen"][0] for p in paths
                       if p["chosen"][0]), (task_id, "the other order")
            assert any(choose(task, choice, p["world_patch"], order=None) != p["chosen"][0] for p in paths
                       if p["chosen"][0]), (task_id, "no order")
        if choice["where"]:
            assert any(choose(task, choice, p["world_patch"], use_where=False) != p["chosen"][0]
                       for p in paths), task_id
    # a tie goes to the first written (0045 C4): the reversed list loses the tie path
    tie = PROGRAMS["p07_equally_near_docks"]
    choice = dict(choices(tie)[0], **{"from": ["dock_1", "dock_3"]})
    assert choose(tie, choice) == "dock_1" and label(tie)["paths"][0]["chosen"] == ["dock_3"]
    assert "equally near" in tie["prompt"]
    # the decoy: the nearest robot is not the ready one
    ready = PROGRAMS["p04_ready_amr_nearer_to_shelf_a"]
    decoy = dict(choices(ready)[0], where=None)
    assert choose(ready, decoy) == "amr2" and label(ready)["paths"][0]["chosen"] == ["amr1"]


def test_positions_are_metres_in_a_plane_and_name_locations_of_the_world():
    for task_id, task in {**PROGRAMS, **INFEASIBLE}.items():
        world = base_world(task)
        ids = location_ids(world)
        notes = [label(task)["world_2_3"].get("positions", {})] + [
            (p["world_patch"].get("positions") or {}) for p in label(task).get("paths", [])]
        for table in notes:
            for name, value in table.items():
                assert name in ids, (task_id, name)
                assert value is None or (len(value) == 2 and all(isinstance(v, int) and v >= 0 for v in value)), task_id
        for patch in [label(task)["world_2_3"]] + [p["world_patch"] for p in label(task).get("paths", [])]:
            for name, entry in patch.get("robots", {}).items():
                assert name in world["robots"] and set(entry) <= {"ready", "location"}, (task_id, name)
                if "location" in entry:
                    assert entry["location"] in ids, (task_id, name)
                    assert entry["location"] in positions(task) or entry["location"] in ("headland", "trolley")
            for name, entry in patch.get("objects", {}).items():
                assert name in world["objects"] and set(entry) == {"location"}
                assert entry["location"] in ids, (task_id, name)


def test_the_infeasible_tasks_each_leave_no_candidate_or_a_candidate_that_cannot_do_it():
    kinds = {t: set(d["tags"]) - {"choose", "no_fallback"} for t, d in INFEASIBLE.items()}
    assert kinds == {"i01_no_tugger_is_ready": {"robots", "where", "no_candidate_left"},
                     "i02_the_wall_has_no_position": {"locations", "nearest", "no_position"},
                     "i03_the_ready_robot_has_no_gripper": {"robots", "where", "no_gripper", "existing_infeasibility"},
                     "i04_nothing_at_the_rework_bench": {"objects_at", "no_candidate_left"},
                     "i05_the_ready_scanbot_has_the_wrong_camera": {"robots", "where", "sensor_elsewhere", "fill",
                                                                    "existing_infeasibility"}}
    assert sorted(label(t)["because"].split(" ")[0] for t in INFEASIBLE.values()) == [
        "0045", "0045", "0045", "S211", "S231"]
    for task in INFEASIBLE.values():
        assert "otherwise" not in task["prompt"] and "stop the job" not in task["prompt"]  # no fallback is named
    for task_id in ("i01_no_tugger_is_ready", "i02_the_wall_has_no_position", "i04_nothing_at_the_rework_bench"):
        task = INFEASIBLE[task_id]
        assert choose(task, choices(task)[0]) is None, task_id  # every candidate is left out: `else` runs
    gripper = INFEASIBLE["i03_the_ready_robot_has_no_gripper"]
    who = choose(gripper, choices(gripper)[0])
    assert who == "tugger_a" and not base_world(gripper)["robots"][who]["gripper"] and base_world(gripper)["robots"][
        "digit2"]["gripper"]
    scan = INFEASIBLE["i05_the_ready_scanbot_has_the_wrong_camera"]
    who = choose(scan, choices(scan)[0])
    assert who == "scanbot1" and base_world(scan)["sensors"]["cam_scan2"]["robot"] == "scanbot2"
    assert label(scan)["world_2_3"]["robots"] == {"scanbot2": {"ready": False}}
    # in the 2.2 world each of these is a request that looks fine (erratum Q3): the decoys are facts of the 2.3 copy
    for task_id in ("i01_no_tugger_is_ready", "i03_the_ready_robot_has_no_gripper"):
        assert all(r["ready"] for r in world_2_2(INFEASIBLE[task_id])["robots"].values())
        assert not all(r["ready"] for r in base_world(INFEASIBLE[task_id])["robots"].values())
    assert world_2_2(INFEASIBLE["i04_nothing_at_the_rework_bench"])["objects"]["p5"]["location"] == "rework"
    assert base_world(INFEASIBLE["i04_nothing_at_the_rework_bench"])["objects"]["p5"]["location"] == "station2"
    assert world_2_2(INFEASIBLE["i05_the_ready_scanbot_has_the_wrong_camera"])["robots"]["scanbot2"]["ready"] is True
    assert location_ids(base_world(INFEASIBLE["i02_the_wall_has_no_position"])).count("level_3_wall") == 1
    assert "level_3_wall" not in positions(INFEASIBLE["i02_the_wall_has_no_position"])  # a location with no position


def test_the_inexpressible_tasks_each_stay_out_for_one_reason_of_0045_c9():
    assert {t: d["gap"] for t, d in INEXPRESSIBLE.items()} == {
        "x01_fullest_bin_on_the_row": "G-EACH", "x02_every_pan_above_75": "G-EACH",
        "x03_carry_as_many_as_there_are_bins": "G-EACH", "x04_the_robot_nearest_the_caller": "G-PROX",
        "x05_an_object_the_list_does_not_know": "G-DETECT"}
    assert {t: set(d["tags"]) for t, d in INEXPRESSIBLE.items()} == {
        "x01_fullest_bin_on_the_row": {"fill", "reading_of_each_candidate", "objects_at", "run_time_set"},
        "x02_every_pan_above_75": {"temperature", "each", "reading_of_each_candidate", "objects_at", "run_time_set"},
        "x03_carry_as_many_as_there_are_bins": {"count", "each", "run_time_number", "pick_place"},
        "x04_the_robot_nearest_the_caller": {"person", "robots", "nearest", "proximity"},
        "x05_an_object_the_list_does_not_know": {"detect", "unlisted_object", "objects_at", "if"}}
    assert "fullest bin" in SET["x01_fullest_bin_on_the_row"]["prompt"] and "come and go" in SET[
        "x02_every_pan_above_75"]["prompt"]
    assert "that many" in SET["x03_carry_as_many_as_there_are_bins"]["prompt"]
    assert "person" in SET["x04_the_robot_nearest_the_caller"]["prompt"]
    assert "does not know" in SET["x05_an_object_the_list_does_not_know"]["prompt"]
    # named bins are a 2.2 program (compare two readings in an `if`): the control is the proof; x01 has a run-time set
    assert "bin_1 and bin_2" in SET["c02_the_fuller_of_two_bins"]["prompt"] and "bin_1" not in SET[
        "x01_fullest_bin_on_the_row"]["prompt"]


# -------- the references and the attempts --------

def test_references_and_attempts_are_where_they_should_be():
    names = [p.name for p in REF.iterdir()]
    refs = {n.split(".")[0] for n in names if ".attempt" not in n}
    attempts = {n.split(".")[0] for n in names if ".attempt." in n}
    # a 2.3 reference for each program and each control; the 2.2 attempts of the declined tasks stay as evidence
    assert refs == set(CONTROLS) | set(PROGRAMS) and attempts == set(DECLINED)
    for name in names:
        assert "\r" not in read(REF / name)  # Windows-safe: LF only, never compared as CRLF


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_the_obvious_attempt_fails_with_the_recorded_code(task_id):
    task = DECLINED[task_id]
    answer = attempt_answer(task_id)
    assert answer["rob"].startswith(f"// bench/reference_choose/{task_id}.attempt.rob: the obvious attempt")
    assert answer["rob"].count("robolang 2.2;") == 1 and "\r" not in answer["rob"] and answer["rob"].isascii()
    checked = lib.check_answer(task, answer)
    assert not checked.valid and checked.codes[0] == task["attempt_fails_with"] == "E203", (task_id, checked.codes)
    assert "unknown statement 'choose'" in checked.report


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_no_probe_is_a_bug_in_disguise(task_id):
    """An attempt fails for its `choose`: with the first candidate in its place it is a valid 2.2 program that ends
    SUCCEEDED in the world the baseline showed. That program is the choice replaced silently, which the old checks
    cannot
    tell (B8). A mission cannot be that silent: a fixed part is gone after the first run, so its second run fails
    (S215).
    In the 2.3 world copy of an infeasible task (erratum Q3) the fixed pick fails: the fact that makes it infeasible."""
    task = DECLINED[task_id]
    answer = attempt_answer(task_id, strip=True)
    assert "@choice" not in answer["rob"] and not re.search(r"\bchoose\b", answer["rob"])
    data = read(REPO / task["world_2_2"]).encode() if "world_2_2" in task else None
    checked = lib.check_answer(task, answer, data)
    assert checked.valid, (task_id, checked.report)
    assert checked.language_version == "2.2"
    if task_id in MISSIONS:
        result = lib.run_mission(checked.document, checked.mission, checked.world)
        assert result.outcome == "failed" and result.runs[0].result.state == "SUCCEEDED"
        assert result.runs[1].result.failure.code == "S215"
        return
    result = lib.simulate_task(checked.task, checked.world)
    assert result.state == "SUCCEEDED", (task_id, result.failure)
    if task_id in INFEASIBLE and task_id != "i02_the_wall_has_no_position":
        # the 2.3 world copy: nothing is ready, nothing is there, the wrong camera
        in_2_3 = lib.check_answer(task, answer)
        if in_2_3.valid:
            assert lib.simulate_task(in_2_3.task, in_2_3.world).state == "FAILED", task_id


def test_each_attempt_has_one_marked_choice_line_in_the_form_of_0045():
    for task_id in DECLINED:
        rob = read(REF / f"{task_id}.attempt.rob")
        marked = [ln for ln in rob.splitlines() if MARK.search(ln.rstrip())]
        assert len(marked) == 1 and marked[0].lstrip().startswith("choose "), task_id
        assert " from " in marked[0] and " else {" in marked[0] and marked[0].count("choose ") == 1, task_id
        others = [ln for ln in rob.splitlines()[2:] if ln not in marked]
        assert not any(re.search(r"\bchoose\b", ln) for ln in others), task_id
    text = {t: read(REF / f"{t}.attempt.rob") for t in DECLINED}
    assert "from (a, b) nearest to spot" in text["p01_nearer_tugger_to_the_trailer"]
    assert "from (d1, d2, d3) nearest to hall" in text["p02_nearest_dock_to_the_hall"]
    assert "from (a, b) nearest to ref" in text["p03_nearer_cart_for_tote_2"]
    assert "where robot_ready who nearest to src" in text["p04_ready_amr_nearer_to_shelf_a"]
    assert "from objects at conveyor" in text["p05_any_part_from_the_conveyor"]
    assert "from robots nearest to spot" in text["p06_closest_robot_to_the_lobby"]
    assert "from (d3, d1) nearest to hall" in text["p07_equally_near_docks"]  # the order written
    assert "farthest from src" in text["p08_farthest_spot_from_the_trailer"]
    assert "@safety no_go_region wet;" in text["p10_dock_near_stop_c_around_the_wet_floor"]
    for task_id in MISSIONS:
        assert "mission move_parts" in text[task_id] and "run move_part(arm, conveyor, bin)" in text[task_id]
    assert "repeat for at most 6 min" in text["p09_parts_for_six_minutes"]
    assert "until clock_time >= 17:00" in text["p11_parts_until_five"]
    assert "wait for 20 min" in text["p11_parts_until_five"]


def test_the_attempt_binds_the_entities_the_request_names():
    for task_id, task in DECLINED.items():
        bindings = json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"]
        assert bindings and all(v in world_ids(task) for v in bindings.values()), task_id
        stated = {w for w in re.findall(r"\b[a-z]+[a-z_]*\d*\b", task["prompt"]) if w in world_ids(task)}
        assert stated <= set(bindings.values()), (task_id, stated - set(bindings.values()))


# -------- the 2.3 references and what the new measures see --------

def judged(task_id: str, rob: str | None = None, patch=None):
    """The judgement of the 2.3 reference of a program, with its source text changed by `patch` (a function)."""
    task = SET[task_id]
    answer = reference(task_id)
    if rob is not None:
        answer = {**answer, "rob": rob}
    if patch is not None:
        answer = {**answer, "rob": patch(answer["rob"])}
    checked = lib.check_answer(task, answer)
    assert checked.valid, (task_id, checked.report)
    return lib.judge(task, checked, answer["bindings"])


@pytest.mark.parametrize("task_id", sorted(PROGRAMS))
def test_the_2_3_reference_is_valid_right_on_every_path_and_makes_the_choices_asked(task_id):
    task = PROGRAMS[task_id]
    answer = reference(task_id)
    assert answer["rob"].startswith(f"// bench/reference_choose/{task_id}.rob: hand-written reference (RoboLang 2.3)")
    assert answer["rob"].count("robolang 2.3;") == 1 and answer["rob"].isascii() and "\r" not in answer["rob"]
    assert "@choice" not in answer["rob"] and len(re.findall(r"\bchoose\b", answer["rob"])) == 1
    checked = lib.check_answer(task, answer)
    assert checked.valid and checked.language_version == "2.3", checked.report
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds and judgement.choice_ok
    assert not judgement.half_program and not judgement.choice_replaced and not judgement.criterion_changed
    assert all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths)
    assert len(judgement.paths) == len(label(task)["paths"])
    if task_id in MISSIONS:
        assert judgement.mission_ok and judgement.bound_dropped_or_invented is False
        assert [p["state"] for p in judgement.paths] == [p["outcome"] for p in label(task)["paths"]]
    else:
        assert [p["state"] == "FAILED" for p in judgement.paths] == [bool(p.get("fails")) for p in label(task)["paths"]]


def test_a_fixed_candidate_a_dropped_order_or_where_and_another_order_are_what_b8_and_b9_see():
    reference_text = {t: read(REF / f"{t}.rob") for t in PROGRAMS}
    tugger = "p01_nearer_tugger_to_the_trailer"
    no_order = judged(tugger, patch=lambda r: r.replace(" nearest to spot", ""))
    assert no_order.plan_succeeds and not no_order.goal_correct and not no_order.choice_ok
    assert no_order.choice_replaced and not no_order.criterion_changed  # B8: the order is dropped, the first is taken
    other = judged(tugger, patch=lambda r: r.replace("nearest to", "farthest from"))
    assert not other.choice_ok and other.criterion_changed and not other.choice_replaced  # B9: the other order
    elsewhere = judged(tugger, patch=lambda r: r.replace("nearest to spot", "nearest to a"))
    assert elsewhere.criterion_changed and not elsewhere.choice_ok  # another reference point
    answer = attempt_answer(tugger, strip=True)  # the obvious attempt with tugger_a fixed in advance
    checked = lib.check_answer(SET[tugger], answer)
    assert checked.valid, checked.report
    fixed = lib.judge(SET[tugger], checked, answer["bindings"])
    assert fixed.plan_succeeds and not fixed.choice_ok and fixed.choice_replaced and not fixed.criterion_changed
    assert not fixed.goal_correct  # a program that fixes tugger_a fails the path where tugger_b is nearer
    first = judged(tugger, patch=lambda r: r.replace("from (a, b)", "from (a)"))
    assert first.choice_replaced and not first.choice_ok  # one candidate fixed in a list
    # the decoy: the nearest is not ready; the `where` dropped fails a path, and invented is B9
    amr = "p04_ready_amr_nearer_to_shelf_a"
    no_where = judged(amr, patch=lambda r: r.replace(" where robot_ready who", ""))
    assert not no_where.plan_succeeds and not no_where.choice_ok and no_where.criterion_changed
    invented = judged(amr, patch=lambda r: r.replace("where robot_ready who",
                                                     "where robot_ready who and not robot_at who dst"))
    assert invented.criterion_changed and not invented.choice_ok
    no_order_amr = judged(amr, patch=lambda r: r.replace(" nearest to src", ""))
    assert no_order_amr.choice_replaced and not no_order_amr.criterion_changed
    # a tie is settled by the order written (p07): the list the other way round is B9, and it loses the tie path
    tie = "p07_equally_near_docks"
    reordered = judged(tie, patch=lambda r: r.replace("(d3, d1)", "(d1, d3)"))
    assert reordered.criterion_changed and not reordered.choice_ok and not reordered.goal_correct
    assert reference_text[tie].count("(d3, d1)") == 1
    # a list in another order is no change where no tie is settled by it
    same = judged(tugger, patch=lambda r: r.replace("(a, b)", "(b, a)"))
    assert same.choice_ok and not same.criterion_changed and same.goal_correct


def test_an_if_ladder_over_the_listed_objects_is_right_where_erratum_q4_says_so_and_a_fixed_part_is_not():
    part = "p05_any_part_from_the_conveyor"
    # a ladder over the parts the world lists: one `goal` per path, the part placed
    ladder = """robolang 2.3;

task one_part(robot arm, location conveyor, location bin, object p1, object p2, object p3, object p4) {
    plan {
        move arm to conveyor;
        if object_at p1 conveyor {
            goal { object_at p1 bin; }
            grasp arm p1; move arm to bin; place arm p1 at bin;
        } else if object_at p2 conveyor {
            goal { object_at p2 bin; }
            grasp arm p2; move arm to bin; place arm p2 at bin;
        } else if object_at p3 conveyor {
            goal { object_at p3 bin; }
            grasp arm p3; move arm to bin; place arm p3 at bin;
        } else if object_at p4 conveyor {
            goal { object_at p4 bin; }
            grasp arm p4; move arm to bin; place arm p4 at bin;
        } else { fail "the conveyor is empty"; }
    }
}
"""
    answer = {**reference(part), "rob": ladder, "task": "one_part",
              "bindings": {"arm": "arm_a", "conveyor": "conveyor_in", "bin": "bin", "p1": "p1", "p2": "p2",
                           "p3": "p3", "p4": "p4"}}
    checked = lib.check_answer(SET[part], answer)
    assert checked.valid, checked.report
    ladder_judged = lib.judge(SET[part], checked, answer["bindings"])
    assert ladder_judged.choice_ok and not ladder_judged.choice_replaced and ladder_judged.goal_correct
    assert ladder_judged.plan_succeeds and not ladder_judged.criterion_changed
    # a fixed part (no `choose`, no ladder) succeeds as given and is the choice replaced silently; it fails the path
    # where that part was taken away
    answer = attempt_answer(part, strip=True)
    checked = lib.check_answer(SET[part], answer)
    assert checked.valid, checked.report
    fixed = lib.judge(SET[part], checked, answer["bindings"])
    assert fixed.choice_replaced and not fixed.choice_ok and not fixed.goal_correct
    # the same ladder where the request states a criterion (p01: nearer) is no right answer: erratum Q4 is for p05,
    # p09, p11
    assert not SET["p01_nearer_tugger_to_the_trailer"]["expected_choices"][0].get("ladder_ok")


def test_a_mission_with_a_bound_or_a_stop_rule_the_request_did_not_state_is_b9():
    parts = "p09_parts_for_six_minutes"
    base = judged(parts)
    assert base.mission_ok and not base.criterion_changed
    longer = judged(parts, patch=lambda r: r.replace("for at most 6 min", "for at most 8 min"))
    assert not longer.mission_ok and longer.criterion_changed
    no_wait = judged(parts, patch=lambda r: r.replace("        wait for 2 min;\n", ""))
    assert no_wait.plan_succeeds is False  # time never advances: the bound is never reached
    keep_going = judged(parts, patch=lambda r: r.replace("on failure { stop; }", "on failure { continue; }"))
    assert keep_going.criterion_changed


def test_an_infeasible_request_answered_with_a_program_that_ends_well_is_the_b4_clause():
    """No valid program that fixes a candidate and succeeds: in the 2.3 world copy the fixed picks fail (erratum Q3),
    so a program that ends well there did something other than the request."""
    for task_id in ("i01_no_tugger_is_ready", "i03_the_ready_robot_has_no_gripper", "i04_nothing_at_the_rework_bench",
                    "i05_the_ready_scanbot_has_the_wrong_camera"):
        task = INFEASIBLE[task_id]
        answer = attempt_answer(task_id, strip=True)
        checked = lib.check_answer(task, answer)
        assert checked.valid, (task_id, checked.report)
        judgement = lib.judge(task, checked, answer["bindings"])
        assert judgement.state == "FAILED" and judgement.choice_replaced is False, task_id  # does not end well
    # the program that does end well in the 2.3 world: it moves a robot that is ready, whatever was asked
    task = INFEASIBLE["i01_no_tugger_is_ready"]
    answer = {**attempt_answer("i01_no_tugger_is_ready", strip=True), "rob": (
        "robolang 2.3;\n\ntask go(robot r, location spot) {\n    goal { robot_at r spot; }\n"
        "    plan { move r to spot; }\n}\n"),
        "task": "go", "bindings": {"r": "tugger_a", "spot": "handoff_point"}}
    checked = lib.check_answer(task, answer, read(REPO / task["world_2_2"]).encode())
    judgement = lib.judge(task, checked, answer["bindings"])
    # in the 2.2 world: ready, so it ends well
    assert judgement.state == "SUCCEEDED" and judgement.choice_replaced is True


def test_a_choose_on_a_control_is_choose_added_b5():
    control = CONTROLS["c02_the_fuller_of_two_bins"]
    answer = reference("c02_the_fuller_of_two_bins")
    checked = lib.check_answer(control, answer)
    assert lib.judge(control, checked, answer["bindings"]).choose_added is False
    added = {**answer, "rob": answer["rob"].replace("robolang 2.2;", "robolang 2.3;")}
    checked = lib.check_answer(control, added)
    assert lib.judge(control, checked, added["bindings"]).choose_added is False  # still no `choose`
    with_choose = {**answer, "rob": answer["rob"].replace("robolang 2.2;", "robolang 2.3;").replace(
        "    plan {", '    plan {\n        choose robot who from (r) else { fail "none"; }', 1)}
    checked = lib.check_answer(control, with_choose)
    assert checked.valid, checked.report
    assert lib.judge(control, checked, with_choose["bindings"]).choose_added is True


def test_the_world_patch_of_a_path_may_set_or_remove_a_position():
    task = PROGRAMS["p02_nearest_dock_to_the_hall"]
    world = lib.patched_world(task, {"positions": {"dock_2": None, "dock_1": [41, 11]}, "robots": {}})
    positions_after = {item["id"]: item.get("position") for item in world["locations"] if isinstance(item, dict)}
    assert "dock_2" not in positions_after and "dock_2" in location_ids(world)  # no position now: a plain ID
    assert positions_after["dock_1"] == [41, 11] and positions_after["hall"] == [40, 10]


# -------- the controls --------

@pytest.mark.parametrize("task_id", sorted(CONTROLS))
def test_the_control_reference_is_valid_succeeds_on_every_path_and_makes_no_choice(task_id):
    task = CONTROLS[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 2.2;") == 1 and answer["rob"].isascii() and "\r" not in answer["rob"]
    assert not re.search(r"\bchoose\b", answer["rob"]) and "@choice" not in answer["rob"]
    checked = lib.check_answer(task, answer)
    assert checked.valid and checked.language_version == "2.2", checked.report
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert not judgement.half_program
    assert all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths or [])


def test_the_controls_name_every_entity_and_a_model_that_adds_a_choice_changes_the_request():
    assert [lib.task_paths(c) is not None for c in CONTROLS.values()] == [False, True, False, True]
    for control in CONTROLS.values():
        assert not re.search(r"\b(whichever|nearest|nearer|closest|closer|farthest|any|every)\b", control["prompt"])
    paths = lib.task_paths(CONTROLS["c02_the_fuller_of_two_bins"])
    assert [p.get("fails", False) for p in paths] == [False, False, True]  # the overlap is an unknown: S261
    assert CONTROLS["c02_the_fuller_of_two_bins"]["goal_free"] is True  # the request names no end state (erratum P7)
    mission = lib.mission_label(CONTROLS["c03_shuttle_p1_twelve_carries"])
    assert mission["repeat"]["max_runs"] == 12 and len(mission["runs"]) == 2 and mission["until"] is None
    carry = CONTROLS["c01_carry_a_tote"]
    assert lib.expected_goals(carry) and "paths" not in carry["expect_2_2"]


# -------- the atlas --------

def test_only_what_2_3_still_declines_is_in_an_atlas_gap_and_the_two_new_gaps_are_far_and_open():
    """After L4 the gap membership is what 2.3 still declines: the programs and the infeasible tasks left, and the
    controls were never in one. G-CHOOSE stays open until L6 and holds none of them."""
    in_gaps = [(t, g["id"]) for g in ATLAS["gaps"] for t in g["tasks"] if t in SET]
    assert len(in_gaps) == len({t for t, _ in in_gaps}) == len(INEXPRESSIBLE) == 5
    assert {t for t, _ in in_gaps} == set(INEXPRESSIBLE)
    for task_id, gap in in_gaps:
        assert INEXPRESSIBLE[task_id]["gap"] == gap and GAPS[gap]["scope"] == "open"
    assert Counter(gap for _, gap in in_gaps) == {"G-EACH": 3, "G-PROX": 1, "G-DETECT": 1}
    assert {t["gap"] for t in PROGRAMS.values()} | {t["gap"] for t in INFEASIBLE.values()} == {"G-CHOOSE"}
    assert GAPS["G-CHOOSE"]["scope"] == "open" and GAPS["G-CHOOSE"]["tasks"] == []
    assert "2.3" in GAPS["G-CHOOSE"]["scope_note"] and "L6" in GAPS["G-CHOOSE"]["scope_note"]
    for name, direction in (("G-EACH", "reasoning"), ("G-DETECT", "perception")):
        assert GAPS[name]["scope"] == "open" and GAPS[name]["direction"] == direction
        assert "Far" in GAPS[name]["scope_note"] and "0045" in GAPS[name]["scope_note"]
        assert not [a for a in ATLAS["activities"] if name in a["needs"]]  # far: no sub-activity claims it yet
    notes = [h["note"] for h in ATLAS["history"]]  # membership: never the last note
    assert "I7 L3: set 12 choose pre-registered (25 tasks)" in notes and "I7 L4: RoboLang 2.3 built and taught" in notes


def test_the_standing_tasks_that_stay_out_of_2_3_are_in_g_each_and_the_six_that_became_programs_left_the_atlas():
    each = GAPS["G-EACH"]["tasks"]
    assert {"x03_any_hot_pan", "x03_carry_as_many_as_counted"} <= set(each) and len(each) == 5
    standing = ("x_nearest_robot", "x05_every_new_part", "x_shuttle_until_five", "f_choose_nearest_tugger",
                "f_choose_nearest_dock", "f_choose_nearer_cart")
    assert not set(standing) & {t for g in ATLAS["gaps"] for t in g["tasks"]}  # erratum Q5: programs in 2.3
    for set_name, task_id in (("values", "x03_any_hot_pan"), ("kinds", "x03_carry_as_many_as_counted")):
        task = lib.load_set(set_name)[task_id]
        assert task["gap"] == "G-EACH" and task["gap_before"] == "G-CHOOSE" and task["erratum"].startswith("Q2")
        assert lib.expectation(task) == ("cannot", "inexpressible")  # the label did not change, only the gap
    for set_name, task_id in (("future", "f_choose_nearest_tugger"), ("future", "f_choose_nearest_dock"),
                              ("future", "f_choose_nearer_cart"), ("branch", "x_nearest_robot"),
                              ("branch", "x_shuttle_until_five"), ("missions", "x05_every_new_part")):
        task = lib.load_set(set_name)[task_id]
        assert lib.expectation(task) == ("program", "none") and task["group"] == "choose" and task["erratum"] == "Q5"
        assert task["gap"] == "G-CHOOSE" and task["expect_2_2"]["category"] == "inexpressible"
        assert task["world"] == f"bench/worlds/choose_{task_id}.json" and task["world_2_2"].startswith("bench/worlds/")


# -------- the scorer and the workflow --------

def choose_run(tmp_path: Path, replies: dict) -> Path:
    def script(request):
        first = request["messages"][0]["content"]
        task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
        return message(replies[task_id], model=request["model"])

    out_dir = tmp_path / "choose"
    run.run_benchmark(FakeClient(script), ["claude-sonnet-5-5"], {t: SET[t] for t in replies}, "medium", 1, out_dir,
                      lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="choose", prompt_name="full")
    return out_dir


def test_the_scorer_counts_a_program_right_a_declined_program_a_false_refusal_and_a_fixed_pick_false(tmp_path):
    replies = {
        "p01_nearer_tugger_to_the_trailer": reference("p01_nearer_tugger_to_the_trailer"),  # right
        "p02_nearest_dock_to_the_hall": decline(),                                         # a false refusal
        "i01_no_tugger_is_ready": decline("infeasible"),                                  # right decision
        "x01_fullest_bin_on_the_row": attempt_answer("x01_fullest_bin_on_the_row", strip=True),  # a false program
        "c01_carry_a_tote": reference("c01_carry_a_tote"),                                  # right
        "c04_carry_at_nine": decline(),                                                     # a control refused
    }
    out_dir = choose_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "choose" and meta["prompt"] == "full"
    right = by["p01_nearer_tugger_to_the_trailer"]
    assert right["decision_correct"] and right["goal_correct"] and right["plan_succeeds"] and right["request_satisfied"]
    assert right["choice_ok"] is True and right["choice_replaced"] is False and right["criterion_changed"] is False
    assert by["p02_nearest_dock_to_the_hall"]["false_refusal"]
    no_tugger = by["i01_no_tugger_is_ready"]
    assert no_tugger["decision_correct"] and no_tugger["category_correct"] is True
    assert no_tugger["choice_replaced"] is False
    silent = by["x01_fullest_bin_on_the_row"]
    assert silent["false_program"] and not silent["decision_correct"] and silent["choice_ok"] is None
    assert by["c01_carry_a_tote"]["decision_correct"] and by["c01_carry_a_tote"]["choose_added"] is False
    assert by["c04_carry_at_nine"]["false_refusal"]
    text = score.score_dir(out_dir)
    assert "Task set choose, prompt full." in text
    for line in ("| choice_ok (B1", "| choice_replaced_silently (B8", "| criterion_changed (B9",
                 "| no_choose_added (B5"):
        assert line in text, line
    for kind in ("infeasible", "inexpressible", "control", "choose"):
        assert f"| {kind} |" in text.split("By kind of task:")[1]


def test_the_scorer_of_a_synthetic_run_of_the_whole_set_is_fast_and_small(tmp_path):
    """Scoring a run of set 12 (both prompts: the 2.3 reference for each program, a fixed-pick program for each task
    that stays out, a decline for each infeasible one) stays under a minute and under 1 GB."""
    import time
    try:
        import resource
    except ImportError:  # Windows has no `resource`
        resource = None
    replies = {t: attempt_answer(t, strip=True) if d["group"] == "inexpressible" else decline("infeasible")
               if d["group"] == "infeasible" else reference(t) for t, d in DECLINED.items()}
    replies.update({t: reference(t) for t in CONTROLS})
    started = time.monotonic()
    for prompt in ("full", "minimal"):
        def script(request):
            first = request["messages"][0]["content"]
            task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
            return message(replies[task_id], model=request["model"])

        out_dir = tmp_path / prompt
        run.run_benchmark(FakeClient(script), ["claude-sonnet-5-5"], dict(SET), "medium", 1, out_dir,
                          lib.load_prompt(prompt), workers=1, log=io.StringIO(), task_set="choose", prompt_name=prompt)
        text = score.score_dir(out_dir)
        assert f"Task set choose, prompt {prompt}." in text and "| decision_correct |" in text
    assert time.monotonic() - started < 60
    if resource is not None and hasattr(resource, "getrusage"):  # ru_maxrss: bytes on macOS, kilobytes elsewhere
        peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        assert peak < ((1 << 30) if sys.platform == "darwin" else (1 << 20))


def test_the_set_is_in_the_run_script_and_only_runs_on_demand_in_the_workflow():
    assert lib.SETS["choose"] == (lib.CHOOSE_TASKS_DIR, lib.CHOOSE_REFERENCE_DIR)
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    assert workflow.count("--set choose") == 2
    lines = [ln for ln in workflow.splitlines() if "--set choose" in ln]
    for part in ("--repeats 3", "--models claude-sonnet-5-5,claude-haiku-5-5", "--max-usd 3"):
        assert all(part in ln for ln in lines), part
    assert any("--prompt full" in ln and "--run-id choose" in ln for ln in lines)
    assert any("--prompt minimal" in ln and "--run-id choose-minimal" in ln for ln in lines)
    assert all("claude-opus" not in ln and "--set choose --prompt" in ln for ln in lines)
    steps = re.split(r"\n      - ", workflow)
    step = next(s for s in steps if s.startswith("name: Run (set choose"))
    assert "inputs.set_only == 'choose'" in step and "pull_request" not in step
    pr_step = next(s for s in steps if s.startswith("name: Run (pull request)"))
    weekly = next(s for s in steps if s.startswith("name: Run (weekly or on demand)"))
    assert "choose" not in pr_step and "inputs.set_only == ''" in weekly and "--set choose" not in weekly
    assert "choose choose-minimal" in workflow


def test_the_set_only_input_lists_choose_and_keeps_the_input_count():
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    block = workflow.split("  workflow_dispatch:\n    inputs:\n")[1].split("\npermissions:")[0]
    inputs = re.findall(r"^      ([a-z_0-9]+):$", block, re.M)
    assert inputs == ["set_only", "confirm_tasks", "confirm_set", "confirm_models", "confirm_prompt"]
    steps = re.split(r"\n      - ", workflow)
    for value in ("set3", "set4", "future", "regions", "actions", "clock", "missions", "messages", "kinds", "choose"):
        assert sum(f"inputs.set_only == '{value}'" in s for s in steps) == 1, value
    assert "kinds or choose. Empty" in block


def test_the_readme_the_errata_and_the_plan_name_the_set():
    readme = read(REPO / "bench" / "README.md")
    assert "## Set 12: choosing at run time (`--set choose`)" in readme and "set_only: choose" in readme
    plan = read(REPO / "docs" / "plans" / "bench-12-choose.md")
    assert "approved by the owner" in plan[:700].lower()
    for task_id in SET:
        assert f"`{task_id}`" in plan, task_id
    for bound in ("B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8", "B9"):
        assert f"| {bound} |" in plan, bound
    errata = read(REPO / "docs" / "evidence" / "bench-errata-2-3.md")
    assert "Erratum Q1" in errata and "Erratum Q2" in errata and "bench-freeze-v2.json" in errata
    for task_id in ("x03_any_hot_pan", "x03_carry_as_many_as_counted", "G-EACH", "G-DETECT"):
        assert task_id in errata, task_id
    roadmap = read(REPO / "docs" / "LANGUAGE_ROADMAP.md")
    assert "plans/bench-12-choose.md" in roadmap
