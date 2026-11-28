"""The phase 4 scenes (decision 0046 section 3): every scene is a valid site with the reference task's entity IDs, the
family meets the visibility rule, and the acceptance run's judge applies the bar."""

import copy
import json
from pathlib import Path

import pytest
from conftest import needs_assets

from robolang_g1 import scenes
from robolang_g1 import site as S

FAMILY = [  # 0046 section 3, item 1: the 11 tasks of 0029 section 7 and the three mobile-base tasks
    "a_scrap_part", "a_sort_three", "e_box_then_dock", "e_hold_box", "e_pause_home", "e_pick_place", "l_vial_hold",
    "l_vial_to_fridge", "w_part_precheck", "w_pick_from_table", "w_split_boxes", "e_park_amr", "w_carry_hold",
    "w_clear_dock",
]


def test_the_family_is_the_fourteen_tasks_of_0046_and_the_five_cases():
    assert sorted(scenes.TASKS) == sorted(FAMILY)
    assert list(scenes.CASES) == ["fail_object_absent", "fail_object_moved", "fail_out_of_reach",
                                  "fail_camera_covered", "fail_push_fall"]
    assert scenes.names() == [*scenes.TASKS, *scenes.CASES]


@pytest.mark.parametrize("scene", scenes.names())
def test_every_scene_is_a_valid_site_bound_to_the_reference_entities(scene):
    built = scenes.build(scene)
    site = S.load_site(copy.deepcopy(built["site"]))
    S.load_site(copy.deepcopy(built["world"]))
    ref = json.loads((scenes.REFERENCE / f"{built['task_id']}.json").read_text())
    assert built["task"] == ref["task"] and built["bind"] == ref["bindings"]
    assert Path(built["source"]).name == f"{built['task_id']}.rob"
    bound = {**site["bindings"]["robots"], **site["bindings"]["objects"], **site["bindings"]["locations"]}
    assert set(bound) == set(ref["bindings"].values())  # the same entity IDs as the benchmark
    markers = [o["marker"]["id"] for o in site["objects"]] + [s["marker"]["id"] for s in site["stations"]]
    assert len(markers) == len(set(markers))


@pytest.mark.parametrize("scene", scenes.names())
def test_the_reference_task_binds_to_its_scene_unchanged(scene):
    from robolang import ir

    from robolang_g1 import executor

    built = scenes.build(scene)
    site = S.load_site(copy.deepcopy(built["site"]))
    task = ir.compile_source(Path(built["source"]).read_text(), built["source"]).ir.task(built["task"])
    names = executor.refuse(task, built["bind"], site)  # raises if the backend would refuse the task on this site
    assert set(names) == set(built["bind"])


def test_objects_stand_on_their_table_inside_the_region_of_their_location():
    for task, spec in scenes.TASKS.items():
        site = S.load_site(copy.deepcopy(scenes.build(task)["site"]))
        tables = {t["name"]: t for t in site["tables"]}
        for obj in site["objects"]:
            unit, slot = spec["objects"][obj["name"]]
            t = tables[f"{unit}_table"]
            for k in (0, 1):  # the whole box is on the table top
                assert abs(obj["pos"][k] - t["pos"][k]) + obj["half_size"][k] <= t["half_size"][k] + 1e-9, task
            assert obj["pos"][2] - obj["half_size"][2] == pytest.approx(2 * t["half_size"][2])
            # and in the region of the location of that slot, if the task has one there
            for param, (u, s) in spec["locations"].items():
                if u == unit and s in (slot, None, "*"):
                    loc = S.location(site, param)
                    R, p = S.marker_frame(S.station(site, loc["station"])["marker"])
                    q = R.T @ ([obj["pos"][0], obj["pos"][1], obj["pos"][2] + obj["half_size"][2]] - p)
                    assert all(lo < v < hi for lo, v, hi in zip(loc["region"]["min"], q, loc["region"]["max"])), task


def test_slots_of_one_table_do_not_overlap_and_each_hand_has_its_own():
    for group in (("R", "L"), ("R1", "R2", "L"), ("L1", "L2")):  # the slots a table of the family uses together
        xs = sorted((scenes._region(s)["min"][0], scenes._region(s)["max"][0]) for s in group)
        assert all(a[1] < b[0] for a, b in zip(xs, xs[1:])), group
        ys = sorted(scenes.SLOT[s] for s in group)
        gaps = [b - a - 2 * scenes.OBJECT["half_size"][1] for a, b in zip(ys, ys[1:])]
        assert min(gaps) >= 0.05 - 1e-9, group  # 5 cm between neighbouring boxes
    for name, y in scenes.SLOT.items():  # inside what the arm planner takes with the hand of that side
        assert 0.05 <= abs(y) <= 0.20, name
    for task, spec in scenes.TASKS.items():
        for obj in scenes.build(task)["site"]["objects"]:
            slot = spec["objects"][obj["name"]][1]
            assert obj["grasp"]["hand"] == ("left" if scenes.SLOT[slot] > 0 else "right"), (task, obj["name"])


@needs_assets
@pytest.mark.parametrize("task", FAMILY)
def test_the_family_meets_the_visibility_rule(task):
    assert scenes.check_visibility(scenes.build(task)) == []


@needs_assets
def test_a_goal_region_too_far_ahead_breaks_the_rule():
    # the out-of-reach case's site: the destination's region is 0.8 m ahead of where the plan ends
    bad = scenes.check_visibility(scenes.build("fail_out_of_reach"))
    assert len(bad) == 1 and "object_at dst" in bad[0] and "0.80 m ahead" in bad[0]


@needs_assets
def test_the_dock_one_step_further_back_breaks_the_rule():
    built = scenes.build("e_box_then_dock")
    dock = next(s for s in built["site"]["stations"] if s["name"] == "dock")
    dock["stand"][0] += 0.30  # 0.42 m behind the destination's standing place instead of 0.12
    assert any("object_at dst" in p for p in scenes.check_visibility(built))


def test_the_scripted_cases_change_what_they_say():
    base = scenes.build("e_pick_place")
    absent, moved, far = (scenes.build(n) for n in ("fail_object_absent", "fail_object_moved", "fail_out_of_reach"))
    # absent and moved: the executor is told the unedited site; only the world differs
    assert absent["site"] == {**base["site"], "name": absent["site"]["name"]}
    assert moved["site"] == {**base["site"], "name": moved["site"]["name"]}
    assert absent["world"]["objects"][0]["pos"][0] < 0  # behind the start
    assert moved["world"]["objects"][0]["pos"][1] == pytest.approx(scenes.B + scenes.SLOT["M"])
    # out of reach: the site and the world agree (the region really is there), 0.8 m ahead
    assert far["site"]["locations"] == far["world"]["locations"] != base["site"]["locations"]
    for name in ("fail_camera_covered", "fail_push_fall"):
        assert scenes.build(name)["world"] == {**base["site"], "name": name}


# ------------------------------------------------------------------------------------------ the judge (phase4.py)


def _run(task, k, ok=True, counts=True, atoms=((True, "true"),), false_success=False):
    return {"name": f"{task}_{k}", "task": task, "succeeded": ok, "counts": counts, "false_success": false_success,
            "atoms": [{"truth": t, "verdict": v} for t, v in atoms]}


def _cases(ok=True, succeeded=False):
    return {n: {"pass": ok, "succeeded": succeeded} for n in scenes.CASES}


def test_the_judge_passes_a_clean_run_and_needs_every_repeat():
    from robolang_g1.phase4 import judge

    tasks = {t: [_run(t, k) for k in (1, 2, 3)] for t in FAMILY}
    bar = judge(tasks, _cases(), FAMILY, 3)
    assert bar["pass"] and bar["runs_counted"] == 42 and bar["success_rate"] == 1.0 and bar["agreement"] == 1.0
    tasks["e_pick_place"].pop()
    bar = judge(tasks, _cases(), FAMILY, 3)
    assert not bar["pass"] and not bar["complete"]


def test_the_judge_applies_ninety_percent_and_ignores_runs_that_do_not_count():
    from robolang_g1.phase4 import judge

    tasks = {t: [_run(t, k) for k in (1, 2, 3)] for t in FAMILY}
    for t in FAMILY[:4]:  # 4 of 42 failed: 90.5%
        tasks[t][0] = _run(t, 1, ok=False, atoms=((False, "false"),))
    assert judge(tasks, _cases(), FAMILY, 3)["pass"]
    tasks[FAMILY[4]][0] = _run(FAMILY[4], 1, ok=False, atoms=((False, "false"),))  # 5 of 42: 88.1%
    bar = judge(tasks, _cases(), FAMILY, 3)
    assert not bar["pass"] and bar["success_rate"] < 0.9
    # that fifth failure fell behind real time: it does not count, and its repeat does
    tasks[FAMILY[4]][0]["counts"] = False
    tasks[FAMILY[4]].append(_run(FAMILY[4], 4))
    bar = judge(tasks, _cases(), FAMILY, 3)
    assert bar["pass"] and bar["not_counted"] == [f"{FAMILY[4]}_1"] and bar["runs_counted"] == 42


def test_one_false_success_blocks_acceptance_even_in_a_run_that_does_not_count():
    from robolang_g1.phase4 import judge

    tasks = {t: [_run(t, k) for k in (1, 2, 3)] for t in FAMILY}
    tasks["e_hold_box"].append(_run("e_hold_box", 4, counts=False, false_success=True, atoms=((False, "true"),)))
    bar = judge(tasks, _cases(), FAMILY, 3)
    assert not bar["pass"] and bar["false_successes"] == ["e_hold_box_4"]


def test_the_judge_needs_ninety_eight_percent_agreement_and_every_case():
    from robolang_g1.phase4 import judge

    tasks = {t: [_run(t, k) for k in (1, 2, 3)] for t in FAMILY}
    assert judge(tasks, _cases(), FAMILY, 3)["pass"]
    tasks["a_sort_three"][0] = _run("a_sort_three", 1, ok=False, atoms=((True, "unknown"),))  # 41 of 42: 97.6%
    bar = judge(tasks, _cases(), FAMILY, 3)
    assert not bar["pass"] and bar["agreement"] < 0.98 and bar["unknown_where_the_oracle_decides"] == 1
    tasks = {t: [_run(t, k) for k in (1, 2, 3)] for t in FAMILY}
    assert not judge(tasks, _cases(ok=False), FAMILY, 3)["pass"]
    assert not judge(tasks, _cases(succeeded=True), FAMILY, 3)["pass"]
    assert not judge(tasks, {}, FAMILY, 3)["pass"]


def test_a_task_left_out_of_the_family_is_not_judged():
    from robolang_g1.phase4 import judge

    family = [t for t in FAMILY if t not in scenes.WALK_HOLDING]
    tasks = {t: [_run(t, k) for k in (1, 2, 3)] for t in family}
    bar = judge(tasks, _cases(), family, 3)
    assert bar["pass"] and bar["runs_expected"] == 36


@needs_assets
def test_a_plate_whose_line_of_sight_grazes_a_table_breaks_the_rule():
    # run 2: the dock's plate at 0.19 m, its lower edge in line with the table's far edge, read with confidence 0.00
    built = scenes.build("e_box_then_dock")
    dock = next(s for s in built["site"]["stations"] if s["name"] == "dock")
    assert dock["marker"]["pos"][2] == scenes.DOCK_PLATE_Z
    assert scenes.check_visibility(built) == []
    dock["marker"]["pos"][2] = 0.19
    assert scenes.check_visibility(built) == ["goal: a corner of the plate of station dock is hidden by b_table"]


@needs_assets
def test_the_plates_are_checked_against_the_tables_and_a_box_where_the_goal_puts_it(monkeypatch):
    from robolang_g1 import perception

    seen = []

    def blocked(start, end, boxes):
        seen.append(sorted(b["name"] for b in boxes))
        return None

    monkeypatch.setattr(perception, "line_of_sight_blocked", blocked)
    scenes.check_visibility(scenes.build("e_box_then_dock"))
    # (the first calls are perception.visibility_violations' own, against the tables alone)
    assert ["a_table", "b_table", "box at dst"] in seen
    assert len([s for s in seen if "box at dst" in s]) == 8  # four corners of the two plates the goal reads (dst, dock)


@needs_assets
def test_a_plate_hidden_from_where_move_looks_for_it_breaks_the_rule():
    # F2: in acceptance run 3 the wide tables' plates stood 0.45 m to the side at 0.19 m, hidden from the approach point
    built = scenes.build("w_split_boxes")
    assert scenes.check_visibility(built) == []
    for st in built["site"]["stations"]:
        st["marker"]["pos"][1] += scenes.PLATE_LAT - scenes.PLATE_LAT_WIDE
        st["marker"]["pos"][2] = scenes.PLATE["z"]
        st["stand"][1] = scenes.PLATE_LAT
    bad = scenes.check_visibility(built)
    assert "move to a, seen from the approach point: a corner of its plate is hidden by a_table" in bad
    assert "move to b, seen from the approach point: a corner of its plate is hidden by b_table" in bad


def test_a_raised_plate_keeps_its_regions_where_they_were():
    """Regions are in the plate's frame: a plate raised 0.21 m has them 0.21 m lower in it, so in the map every region
    still spans the table top (0.75 m) to 0.2 m above it, on a wide table as on a narrow one."""
    import numpy as np

    for task, name in (("w_split_boxes", "ta"), ("e_pick_place", "dst")):
        site = S.load_site(copy.deepcopy(scenes.build(task)["site"]))
        loc = S.location(site, name)
        R, p = S.marker_frame(S.station(site, loc["station"])["marker"])
        z = sorted(float((R @ np.array(loc["region"][k]) + p)[2]) for k in ("min", "max"))
        assert z == pytest.approx([2 * scenes.TABLE_HALF_Z, 2 * scenes.TABLE_HALF_Z + 0.20]), task
    assert scenes.PLATE_Z_WIDE > scenes.PLATE["z"]
