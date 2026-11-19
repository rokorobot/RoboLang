"""
Benchmark set 12, "choose" (bench/tasks_choose/, decision record 0045, plan docs/plans/bench-12-choose.md): choosing
at run time, `choose TYPE NAME from CANDIDATES [where COND] [nearest to X | farthest from X] else { ... }`, pre-
registered against RoboLang 2.2 before any code (L3 of iteration I7). `expect` is the 2.2 label (the baseline: every
non-control task declines as inexpressible, because 2.2 has no `choose`: E203, and no position to measure a distance
from); `expect_2_3` is the label for acceptance once 2.3 is taught: `world_2_3` (the world file 1.8 facts: positions
in metres, readiness, which objects are where), `choices` (what each choice is: its type, candidates, `where` and
order), the paths of a program (each forced by a `world_patch` that changes which candidate wins) and, for a mission,
`expected_mission`.

11 `choose` requests that are programs in 2.3, 5 `infeasible` in 2.3, 5 `inexpressible` that stay out (0045 C9:
G-EACH, G-PROX, G-DETECT), 4 `control` requests that are programs in both versions (a fixed carry, two readings
compared, a fixed mission, a clock wait). 2.2 declines every non-control task, and each has an obvious attempt (a
`choose` statement, as 0045 shows it) that 2.2 rejects with E203; the lines of the `choose` are marked `// @choice
NAME=ENTITY`. Without it, and with the chosen name replaced by the first candidate (the fixed one the world lists),
the attempt is a valid 2.2 program that succeeds: the choice replaced silently, which the old checks cannot tell (the
plan's B8). Only the controls have a reference; the 2.3 references wait for L4. No network and no SDK; the model run
is the `Benchmark` workflow with `set_only: choose`.
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
    found = {k: tuple(v) for k, v in label(task)["world_2_3"].get("positions", {}).items()}
    for name, value in ((patch or {}).get("positions") or {}).items():
        if value is None:
            found.pop(name, None)
        else:
            found[name] = tuple(value)
    return found


def world_of(task: dict, patch: dict | None = None) -> dict:
    world = lib._merge(base_world(task), {k: v for k, v in label(task)["world_2_3"].items()
                                          if k in ("robots", "objects")})
    return lib._merge(world, {k: v for k, v in (patch or {}).items() if k in ("robots", "objects", "clock")})


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
    mission, world = label(task)["expected_mission"], world_of(task, path["world_patch"])
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
        pick = choose(task, label(task)["choices"][0], {"objects": {k: v for k, v in world["objects"].items()}})
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


def test_at_most_three_new_worlds_and_they_are_world_file_1_7_without_positions():
    used = {Path(t["world"]).name for t in SET.values()}
    assert NEW_WORLDS <= used and len(NEW_WORLDS) <= 3
    for name in NEW_WORLDS:
        world = json.loads(read(REPO / "bench" / "worlds" / name))
        assert world["world_schema_version"] == "1.7"
    for task in SET.values():  # no world of the set has a position: they are in `world_2_3` until world file 1.8 (L4)
        assert "position" not in read(REPO / task["world"]), task["id"]


def test_expect_is_the_2_2_label_and_the_2_3_label_is_in_expect_2_3():
    for task_id, task in SET.items():
        if task["group"] == "control":
            assert task["expect"] == label(task) == {"decision": "program"}
            assert task["expect_2_2"]["decision"] == "program"
            assert not {"why", "attempt_fails_with", "gap", "blockers"} & set(task)
            assert lib.expectation(task) == ("program", "none")
            continue
        assert task["expect"] == CANNOT and lib.expectation(task) == ("cannot", "inexpressible"), task_id
        assert len(task["why"]) > 40 and task["gap"] in GAPS and task["attempt_fails_with"] == "E203"
        assert not {"expected_goal", "expected_goals", "paths", "expect_2_2"} & set(task), task_id
        assert lib.task_paths(task) is None  # the baseline reads no path: every one of these declines
    for task_id, task in PROGRAMS.items():
        later = label(task)
        assert later["decision"] == "program" and later["choices"] and later["world_2_3"], task_id
        assert len({p["when"] for p in later["paths"]}) == len(later["paths"]) >= 2
        assert task["gap"] == "G-CHOOSE"
        assert ("expected_mission" in later) == (task_id in MISSIONS)
    for task_id, task in INFEASIBLE.items():
        later = label(task)
        assert later["decision"] == "cannot" and later["category"] == "infeasible", task_id
        assert task["blockers"] and task["gap"] == "G-CHOOSE" and later["because"] and later["choices"], task_id
        assert "paths" not in later and task["expect"] == CANNOT
    for task_id, task in INEXPRESSIBLE.items():
        later = label(task)
        assert {k: later[k] for k in ("decision", "category")} == CANNOT and later["because"].startswith("0045 C9")


def test_the_programs_cover_what_the_pre_registration_lists():
    tags = Counter(tag for t in PROGRAMS.values() for tag in t["tags"])
    for tag in ("list", "locations", "robots", "nearest", "farthest", "where", "robot_ready", "decoy", "objects_at",
                "all_robots", "tie", "order_written", "mission", "duration_bound", "until_clock", "choose_afresh",
                "region_limit", "message", "named_in_goal", "else_runs", "no_position_excluded"):
        assert tags[tag] >= 1, tag
    forms = Counter((c["type"], "list" if isinstance(c["from"], list) else "set")
                    for t in PROGRAMS.values() for c in label(t)["choices"])
    assert forms[("robot", "set")] >= 1 and forms[("object", "set")] >= 1 and forms[("location", "list")] >= 1
    assert forms[("robot", "list")] >= 1  # the three forms of 0045 C2: a list, `objects at L`, `robots`
    # the 1.7 limit, the 1.9 clock and a message that go with a choice
    assert label(SET["p10_dock_near_stop_c_around_the_wet_floor"])["expected_limits"] == [["no_go_region", "wet_floor"]]
    assert label(SET["p11_parts_until_five"])["expected_clock"][0]["value"] == "17:00"
    assert label(SET["p01_nearer_tugger_to_the_trailer"])["paths"][0]["messages"] == [["ops", "tugger sent"]]
    assert SET["p10_dock_near_stop_c_around_the_wet_floor"]["world"].endswith("site_1_3.json")  # honours no_go_region
    # an order or a filter in each choice, and a choice in each program
    for task in PROGRAMS.values():
        for choice in label(task)["choices"]:
            assert choice["type"] in ("robot", "object", "location") and (choice["order"] or choice["where"]
                                                                          or isinstance(choice["from"], dict))


# -------- the labels follow 0045: a choice, its paths, and what a world patch changes --------

def prompt_order(task: dict, names: list[str]) -> list[str]:
    positions_in_prompt = [task["prompt"].index(n) for n in names]
    return [n for _, n in sorted(zip(positions_in_prompt, names))]


@pytest.mark.parametrize("task_id", sorted(set(PROGRAMS) - MISSIONS))
def test_each_path_chooses_the_candidate_the_label_says_and_the_goal_names_it(task_id):
    task = PROGRAMS[task_id]
    choice = label(task)["choices"][0]
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
    mission = label(task)["expected_mission"]
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
        choice, paths = label(task)["choices"][0], label(task)["paths"]
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
    choice = dict(label(tie)["choices"][0], **{"from": ["dock_1", "dock_3"]})
    assert choose(tie, choice) == "dock_1" and label(tie)["paths"][0]["chosen"] == ["dock_3"]
    assert "equally near" in tie["prompt"]
    # the decoy: the nearest robot is not the ready one
    ready = PROGRAMS["p04_ready_amr_nearer_to_shelf_a"]
    decoy = dict(label(ready)["choices"][0], where=None)
    assert choose(ready, decoy) == "amr2" and label(ready)["paths"][0]["chosen"] == ["amr1"]


def test_positions_are_metres_in_a_plane_and_name_locations_of_the_world():
    for task_id, task in {**PROGRAMS, **INFEASIBLE}.items():
        world = base_world(task)
        notes = [label(task)["world_2_3"].get("positions", {})] + [
            (p["world_patch"].get("positions") or {}) for p in label(task).get("paths", [])]
        for table in notes:
            for name, value in table.items():
                assert name in world["locations"], (task_id, name)
                assert value is None or (len(value) == 2 and all(isinstance(v, int) and v >= 0 for v in value)), task_id
        for patch in [label(task)["world_2_3"]] + [p["world_patch"] for p in label(task).get("paths", [])]:
            for name, entry in patch.get("robots", {}).items():
                assert name in world["robots"] and set(entry) <= {"ready", "location"}, (task_id, name)
                if "location" in entry:
                    assert entry["location"] in world["locations"], (task_id, name)
                    assert entry["location"] in positions(task) or entry["location"] in ("headland", "trolley")
            for name, entry in patch.get("objects", {}).items():
                assert name in world["objects"] and set(entry) == {"location"}
                assert entry["location"] in world["locations"], (task_id, name)


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
        assert choose(task, label(task)["choices"][0]) is None, task_id  # every candidate is left out: `else` runs
    gripper = INFEASIBLE["i03_the_ready_robot_has_no_gripper"]
    who = choose(gripper, label(gripper)["choices"][0])
    assert who == "tugger_a" and not base_world(gripper)["robots"][who]["gripper"] and base_world(gripper)["robots"][
        "digit2"]["gripper"]
    scan = INFEASIBLE["i05_the_ready_scanbot_has_the_wrong_camera"]
    who = choose(scan, label(scan)["choices"][0])
    assert who == "scanbot1" and base_world(scan)["sensors"]["cam_scan2"]["robot"] == "scanbot2"
    assert label(scan)["world_2_3"]["robots"] == {"scanbot2": {"ready": False}}
    # in the 2.2 world each of these is a request that looks fine: the decoys are facts of the 2.3 world only
    for task_id in ("i01_no_tugger_is_ready", "i03_the_ready_robot_has_no_gripper"):
        assert all(r["ready"] for r in base_world(INFEASIBLE[task_id])["robots"].values())
    assert base_world(INFEASIBLE["i02_the_wall_has_no_position"])["locations"].count("level_3_wall") == 1


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
    assert refs == set(CONTROLS) and attempts == set(DECLINED)  # no 2.3 reference yet (L4)
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
    SUCCEEDED in the task's world. That program is the choice replaced silently, which the old checks cannot tell (B8).
    A mission cannot be that silent: a fixed part is gone after the first run, so its second run fails (S215)."""
    task = DECLINED[task_id]
    answer = attempt_answer(task_id, strip=True)
    assert "@choice" not in answer["rob"] and not re.search(r"\bchoose\b", answer["rob"])
    checked = lib.check_answer(task, answer)
    assert checked.valid, (task_id, checked.report)
    assert checked.language_version == "2.2"
    if task_id in MISSIONS:
        result = lib.run_mission(checked.document, checked.mission, checked.world)
        assert result.outcome == "failed" and result.runs[0].result.state == "SUCCEEDED"
        assert result.runs[1].result.failure.code == "S215"
        return
    result = lib.simulate_task(checked.task, checked.world)
    assert result.state == "SUCCEEDED", (task_id, result.failure)


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

def test_what_2_2_declines_is_in_exactly_one_atlas_gap_and_the_two_new_gaps_are_far_and_open():
    in_gaps = [(t, g["id"]) for g in ATLAS["gaps"] for t in g["tasks"] if t in SET]
    assert len(in_gaps) == len({t for t, _ in in_gaps}) == len(DECLINED) == 21
    assert {t for t, _ in in_gaps} == set(DECLINED)  # the controls are in no gap
    for task_id, gap in in_gaps:
        assert DECLINED[task_id]["gap"] == gap and GAPS[gap]["scope"] == "open"
    assert Counter(gap for _, gap in in_gaps) == {"G-CHOOSE": 16, "G-EACH": 3, "G-PROX": 1, "G-DETECT": 1}
    for name, direction in (("G-EACH", "reasoning"), ("G-DETECT", "perception")):
        assert GAPS[name]["scope"] == "open" and GAPS[name]["direction"] == direction
        assert "Far" in GAPS[name]["scope_note"] and "0045" in GAPS[name]["scope_note"]
        assert not [a for a in ATLAS["activities"] if name in a["needs"]]  # far: no sub-activity claims it yet
    notes = [h["note"] for h in ATLAS["history"]]
    assert "I7 L3: set 12 choose pre-registered (25 tasks)" in notes  # membership: never the last note


def test_the_standing_tasks_that_stay_out_of_2_3_moved_to_g_each_by_erratum_q2():
    each = GAPS["G-EACH"]["tasks"]
    assert {"x03_any_hot_pan", "x03_carry_as_many_as_counted"} <= set(each) and len(each) == 5
    assert not {"x03_any_hot_pan", "x03_carry_as_many_as_counted"} & set(GAPS["G-CHOOSE"]["tasks"])
    assert {"x_nearest_robot", "x05_every_new_part", "x_shuttle_until_five", "f_choose_nearest_tugger",
            "f_choose_nearest_dock", "f_choose_nearer_cart"} <= set(GAPS["G-CHOOSE"]["tasks"])
    for set_name, task_id in (("values", "x03_any_hot_pan"), ("kinds", "x03_carry_as_many_as_counted")):
        task = lib.load_set(set_name)[task_id]
        assert task["gap"] == "G-EACH" and task["gap_before"] == "G-CHOOSE" and task["erratum"].startswith("Q2")
        assert lib.expectation(task) == ("cannot", "inexpressible")  # the label did not change, only the gap


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


def test_the_scorer_counts_a_decline_as_right_a_fixed_pick_as_a_false_program_and_a_refused_control(tmp_path):
    replies = {
        "p01_nearer_tugger_to_the_trailer": decline(),                                    # right: 2.2 cannot choose
        "i01_no_tugger_is_ready": decline("infeasible"),                                  # right decision
        "x01_fullest_bin_on_the_row": attempt_answer("x01_fullest_bin_on_the_row", strip=True),  # a false program
        "c01_carry_a_tote": reference("c01_carry_a_tote"),                                  # right
        "c04_carry_at_nine": decline(),                                                     # a control refused
    }
    out_dir = choose_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "choose" and meta["prompt"] == "full"
    assert by["p01_nearer_tugger_to_the_trailer"]["decision_correct"]
    no_tugger = by["i01_no_tugger_is_ready"]
    assert no_tugger["decision_correct"] and no_tugger["category_correct"] is False
    silent = by["x01_fullest_bin_on_the_row"]
    assert silent["false_program"] and not silent["decision_correct"]
    assert by["c01_carry_a_tote"]["decision_correct"] and by["c04_carry_at_nine"]["false_refusal"]
    text = score.score_dir(out_dir)
    assert "Task set choose, prompt full." in text
    for kind in ("infeasible", "inexpressible", "control"):
        assert f"| {kind} |" in text.split("By kind of task:")[1]


def test_the_scorer_of_a_synthetic_run_of_the_whole_set_is_fast_and_small(tmp_path):
    """Scoring a run of set 12 (both prompts, a fixed-pick program for every declined task) stays under a minute."""
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
                          lib.load_prompt(prompt), workers=1, log=io.StringIO(), task_set="choose", prompt_name=prompt)
        text = score.score_dir(out_dir)
        assert f"Task set choose, prompt {prompt}." in text and "| decision_correct |" in text
    assert time.monotonic() - started < 60


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
    assert "APPROVED by the owner" in plan[:700]
    for task_id in SET:
        assert f"`{task_id}`" in plan, task_id
    for bound in ("B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8", "B9"):
        assert f"| {bound} |" in plan, bound
    errata = read(REPO / "docs" / "evidence" / "bench-errata-2-3.md")
    assert "Erratum Q1" in errata and "Erratum Q2" in errata and "bench-freeze-v2.json" in errata
    for task_id in ("x03_any_hot_pan", "x03_carry_as_many_as_counted", "G-EACH", "G-DETECT"):
        assert task_id in errata, task_id
    roadmap = read(REPO / "docs" / "LANGUAGE_ROADMAP.md")
    assert "L3: set 12 pre-registered; baseline next" in roadmap
