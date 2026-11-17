"""
Benchmark set 6, "regions" (bench/tasks_regions/, decision record 0039, plan docs/plans/bench-6-regions.md): region
limits on motion (`@safety no_go_region R;`, `@safety in_region R;`), pre-registered against RoboLang 1.6 before any
code (L3 of iteration I1) and labelled for 1.7 in L4 (errata docs/evidence/bench-errata-1-7.md): `expect` is the 1.7
label, `expect_1_6` the 1.6 one (the baseline), `expect_1_7` also holds the paths of a program and the 1.3 world facts.

9 `region` requests that are programs in 1.7, 4 `infeasible` in 1.7 (0039 P5, P6, P8), 5 `inexpressible` that stay
out (0039 P2, P3, P9; G-PROX, G-FORCE), 4 `control` requests that are programs in both versions. Under 1.6 every
non-control task was declined, and every one has an obvious attempt that 1.6 rejects with a named diagnostic. The 9
`region` tasks have a 1.7 reference. The worlds are world file 1.3 copies (C1). No network and no SDK; the model run is
the `Benchmark` workflow with `regions: true`.
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
SET = lib.load_set("regions")
REF = lib.REGIONS_REFERENCE_DIR
ATLAS = json.loads((REPO / "bench" / "atlas.json").read_text(encoding="utf-8"))
GAPS = {g["id"]: g for g in ATLAS["gaps"]}
CANNOT = {"decision": "cannot", "category": "inexpressible"}
REGION = {t: d for t, d in SET.items() if d["group"] == "region"}
INFEASIBLE = {t: d for t, d in SET.items() if d["group"] == "infeasible"}
INEXPRESSIBLE = {t: d for t, d in SET.items() if d["group"] == "inexpressible"}
CONTROLS = {t: d for t, d in SET.items() if d["group"] == "control"}
DECLINED = {**REGION, **INFEASIBLE, **INEXPRESSIBLE}  # what 1.6 declined: each has an attempt, each is in the atlas
OLD = {"decision": "cannot", "category": "inexpressible"}  # the 1.6 label (the baseline)
ANNOTATION = re.compile(r"^\s*@safety (?:no_go_region|in_region|min_distance|max_force)[^\n]*\n", re.M)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def reference(task_id: str) -> dict:
    ref = json.loads(read(REF / f"{task_id}.json"))
    rob = read(REF / f"{task_id}.rob")
    return {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob, **ref}


def attempt_answer(task_id: str, strip: bool = False, version: str = "1.6") -> dict:
    """The obvious attempt as a program answer; `strip` removes its one offending construct (a half program in 1.6)."""
    stem = REF / f"{task_id}.attempt"
    rob = read(Path(f"{stem}.rob")).replace("robolang 1.6;", f"robolang {version};")
    if strip:
        rob = ANNOTATION.sub("", rob).replace(" no_go_region wet;", ";")
    return {"decision": "program", "category": "none", "reason": "an attempt", "entities": [], "rob": rob,
            **json.loads(read(Path(f"{stem}.json")))}


def decline(category: str = "inexpressible") -> dict:
    return {"decision": "cannot", "category": category, "reason": "cannot be done", "rob": "", "task": "",
            "bindings": []}


# -------- the set --------

def test_the_mix_of_tasks():
    assert len(SET) == 22
    assert Counter(d["group"] for d in SET.values()) == {"region": 9, "infeasible": 4, "inexpressible": 5, "control": 4}
    for task_id, task in SET.items():
        assert task["id"] == task_id and (lib.REGIONS_TASKS_DIR / f"{task_id}.json").is_file()
        assert (REPO / task["world"]).is_file() and task["tags"] and task["prompt"].endswith(".")
        assert task["prompt"].isascii() and "\r" not in read(lib.REGIONS_TASKS_DIR / f"{task_id}.json")
    prefixes = {"region": "r", "infeasible": "i", "inexpressible": "x", "control": "c"}
    for task_id, task in SET.items():
        assert re.fullmatch(prefixes[task["group"]] + r"\d\d_[a-z0-9_]+", task_id), task_id


def test_the_ids_are_new_ones_and_no_prompt_is_a_copy_of_a_standing_task():
    elsewhere = {}
    for name in lib.SETS:
        if name != "regions":
            elsewhere.update(lib.load_set(name))
    assert not set(SET) & set(elsewhere)
    assert not {t["prompt"] for t in SET.values()} & {t["prompt"] for t in elsewhere.values()}


def test_labels_are_1_7_in_expect_and_1_6_in_expect_1_6():
    for task_id, task in SET.items():
        assert "expect_1_7" in task and task["expect_1_7"]["decision"] in ("program", "cannot"), task_id
        if task["group"] == "control":
            assert task["expect"] == {"decision": "program"} and lib.expected_goals(task), task_id
            assert task["expect_1_7"] == {"decision": "program"} and task["expected_limits"] == []
            assert not {"why", "attempt_fails_with", "gap"} & set(task)
            continue
        assert len(task["why"]) > 40 and re.fullmatch(r"[ES]\d{3}", task["attempt_fails_with"]), task_id
        assert not {"expected_goal", "expected_goals", "expect_1_5"} & set(task)
    for task_id, task in REGION.items():
        assert task["expect"] == {"decision": "program"} and task["expect_1_6"] == OLD, task_id
        assert lib.expectation(task) == ("program", "none")
        later = task["expect_1_7"]
        assert later["decision"] == "program" and later["paths"] and "category" not in later
        for path in later["paths"]:
            assert path["when"] and path["goal"] and "world_patch" in path
        assert [p["when"] for p in lib.task_paths(task)] == [p["when"] for p in later["paths"]]
        limits = task["expected_limits"]
        assert 1 <= len(limits) <= 8 and len({tuple(x) for x in limits}) == len(limits)  # 0039 P4: at most 8
        assert {kind for kind, _ in limits} <= {"no_go_region", "in_region"}
        assert not {r for k, r in limits if k == "no_go_region"} & {r for k, r in limits if k == "in_region"}  # P4
        world = json.loads(read(REPO / task["world"]))
        assert {region for _, region in limits} <= set(world["regions"]), task_id  # the regions the request names
    for task_id, task in INFEASIBLE.items():
        later = task["expect_1_7"]
        assert task["expect"] == {"decision": "cannot", "category": "infeasible"} and task["expect_1_6"] == OLD
        assert lib.expectation(task) == ("cannot", "infeasible")
        assert later["decision"] == "cannot" and later["category"] == "infeasible", task_id
        assert later["because"] in ("0039 P5", "0039 P6", "0039 P8") and task["blockers"], task_id
        assert ("delegated" in later) == (task_id == "i04_delegated_limit")
    for task_id, task in INEXPRESSIBLE.items():
        assert task["expect"] == CANNOT and task["expect_1_7"] == CANNOT, task_id
        assert lib.expectation(task) == ("cannot", "inexpressible")
    reasons = sorted(t["expect_1_7"]["because"] for t in INFEASIBLE.values())
    assert reasons == ["0039 P5", "0039 P5", "0039 P6", "0039 P8"]


def test_the_region_tasks_cover_the_cases_the_pre_registration_lists():
    tags = Counter(tag for t in REGION.values() for tag in t["tags"])
    assert tags["no_go_region"] >= 7 and tags["in_region"] >= 2 and tags["both_limits"] == 1
    for tag in ("two_robots", "max_speed", "if_region_clear", "carrying", "decoy_names", "two_regions"):
        assert tags[tag] >= 1, tag
    assert {tuple(t["expected_speed"]) for t in REGION.values() if "expected_speed" in t} == {("0.5", "m/s")}
    assert len({(t["world"]) for t in REGION.values()}) >= 5
    # the branching task has two paths, the others one; the decoy world really holds the near-miss names
    assert [len(t["expect_1_7"]["paths"]) for _, t in sorted(REGION.items())].count(2) == 1
    cleanroom = json.loads(read(REPO / "bench/worlds/cleanroom.json"))
    assert {"door_zone", "door_zone2"} <= set(cleanroom["regions"])


def test_the_worlds_are_1_3_copies_made_from_the_facts_recorded_in_the_labels():
    """L4 wrote one world file 1.3 per base world: the base plus the `world_1_3` facts of every task that uses it (and
    `honours` for `straddle1` and `picker1`, which the relabelled `u09_avoid_quay` and `u10_inside_clean_room` need)."""
    both = ["no_go_region", "in_region"]
    extra = {"yard": {"robots": {"straddle1": {"honours": both}}},
             "pharmacy": {"robots": {"picker1": {"honours": both}}}}
    wanted: dict[str, dict] = {}
    for task in SET.values():
        assert task["world"].endswith("_1_3.json") and (REPO / task["world"]).is_file()
        into = wanted.setdefault(Path(task["world"]).stem[:-4], {})
        for kind in ("regions", "robots"):
            for name, fields in task["expect_1_7"].get("world_1_3", {}).get(kind, {}).items():
                into.setdefault(kind, {}).setdefault(name, {}).update(fields)
    assert set(wanted) == {"kitchen", "kitchen_values", "yard", "site", "pharmacy", "cleanroom", "warehouse"}
    for name, facts in wanted.items():
        base = json.loads(read(REPO / f"bench/worlds/{name}.json"))
        made = json.loads(read(REPO / f"bench/worlds/{name}_1_3.json"))
        for patch in (facts, extra.get(name, {})):
            for kind in ("regions", "robots"):
                for entity, fields in patch.get(kind, {}).items():
                    base[kind][entity].update(fields)
        base["world_schema_version"] = "1.3"
        assert made == base, name
        assert '"honours"' not in read(REPO / f"bench/worlds/{name}.json")  # the base worlds are not touched
        assert '"contains"' not in read(REPO / f"bench/worlds/{name}.json")
    site = json.loads(read(REPO / "bench/worlds/site_1_3.json"))
    assert site["regions"]["wet_floor"]["contains"] == ["hall"] and site["robots"]["spot1"]["honours"] == []
    assert site["regions"]["layout_zone"]["contains"] == ["gridline_a", "gridline_c", "level_3_wall"]
    for task_id, task in SET.items():
        has_facts = "world_1_3" in task["expect_1_7"]
        assert "world_1_3" not in task and has_facts == (task["group"] in ("region", "infeasible"))
        facts = task["expect_1_7"].get("world_1_3", {})
        assert set(facts) <= {"regions", "robots"}
        world = json.loads(read(REPO / task["world"]))
        assert set(facts.get("robots", {})) <= set(world["robots"])
        assert set(facts.get("regions", {})) <= set(world["regions"])


def test_the_model_is_shown_what_a_region_holds_and_what_a_robot_honours():
    shown = lib.user_message(INFEASIBLE["i01_hall_is_wet"])
    assert "wet_floor (not clear, contains hall)" in shown
    assert "layout_zone (clear, contains gridline_a, gridline_c" in shown
    assert "spot1 (at stop_a, no gripper, ready, max speed 1.6 m/s, honours no region limit)" in shown
    assert "cleaner1 (" in shown and ", honours no_go_region, in_region)" in shown
    old = lib.user_message({"world": "bench/worlds/site.json", "prompt": "x."})  # a world file 1.2 says nothing of it
    assert "honours" not in old and "contains" not in old


# -------- the references and the attempts --------

def test_references_and_attempts_are_where_they_should_be():
    names = [p.name for p in REF.iterdir()]
    refs = {n.split(".")[0] for n in names if ".attempt" not in n}
    attempts = {n.split(".")[0] for n in names if ".attempt." in n}
    assert refs == set(CONTROLS) | set(REGION) and attempts == set(DECLINED)  # the 1.7 references are the 9 `region`
    for task_id in DECLINED:
        assert (REF / f"{task_id}.attempt.rob").is_file() and (REF / f"{task_id}.attempt.json").is_file()
    for name in names:
        assert "\r" not in read(REF / name)  # Windows-safe: LF only, never compared as CRLF


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_the_obvious_attempt_fails_with_the_recorded_code(task_id):
    task = DECLINED[task_id]
    answer = attempt_answer(task_id)
    assert answer["rob"].startswith(f"// bench/reference_regions/{task_id}.attempt.rob: the obvious attempt")
    assert answer["rob"].count("robolang 1.6;") == 1
    checked = lib.check_answer(task, answer)
    assert not checked.valid and checked.codes[0] == task["attempt_fails_with"], (task_id, checked.codes)


def test_the_attempts_use_each_construct_the_request_needs():
    codes = Counter(t["attempt_fails_with"] for t in DECLINED.values())
    assert codes["E211"] >= 10 and codes["E201"] >= 2 and codes["E202"] == 1 and codes["E207"] == 2
    assert "no_go_region wet;" in read(REF / "x01_first_leg_only.attempt.rob")  # per step: inline on the move
    assert "@safety in_region lane;" in read(REF / "r02_tug_stay_in_lane.attempt.rob")
    assert "@safety min_distance" in read(REF / "x03_person_distance.attempt.rob")
    assert "@safety max_force" in read(REF / "x04_grip_ten_newtons.attempt.rob")


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_no_probe_is_a_bug_in_disguise(task_id):
    """An attempt fails for the construct it uses: without it, it is a valid 1.6 program in the task's world. That
    program is the half program a model may write, and it is what the baseline counts as a false program."""
    answer = attempt_answer(task_id, strip=True)
    assert not re.search(r"no_go_region|in_region|min_distance|max_force", answer["rob"].split("\n", 1)[1])
    checked = lib.check_answer(DECLINED[task_id], answer)
    assert checked.valid, (task_id, checked.report)


def test_the_attempt_binds_the_regions_the_request_names():
    for task_id, task in REGION.items():
        bound = set(json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"].values())
        assert {region for _, region in task["expected_limits"]} <= bound, task_id


@pytest.mark.parametrize("task_id", sorted(REGION))
def test_the_1_7_reference_is_valid_succeeds_on_every_path_and_declares_exactly_its_limits(task_id):
    task = REGION[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 1.7;") == 1 and answer["rob"].isascii()
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report  # valid under world 1.3: `validate --world` finds nothing (S003, S004, S005)
    assert checked.language_version == "1.7"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds and judgement.limit_ok, (judgement.state,)
    assert not judgement.limit_dropped and not judgement.half_program and judgement.checks["limits"]
    assert [p["when"] for p in judgement.paths] == [p["when"] for p in task["expect_1_7"]["paths"]]
    assert all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths), judgement.paths
    bound = answer["bindings"]
    declared = {(s.kind, bound[s.region]) for s in checked.task.region_limits}
    assert declared == {tuple(x) for x in task["expected_limits"]}
    for path in task["expect_1_7"]["paths"]:  # each path, in its patched world, ends SUCCEEDED
        world = lib._merge(json.loads(lib.world_bytes(task)), path["world_patch"])
        loaded = lib.load_world(json.dumps(world).encode(), "w.json", checked.task, bound)
        result = lib.simulate_task(checked.task, loaded)
        assert result.state == "SUCCEEDED"
        statuses = {r.status for r in result.safety_limits if isinstance(r.limit, lib.task_ir.RegionLimit)}
        assert statuses == {"checked_against_world_not_enforced"}


@pytest.mark.parametrize("task_id", sorted(REGION))
def test_the_half_program_reaches_the_goal_on_every_path_and_only_limit_ok_can_tell(task_id):
    """The goals are right for the world: the program without its limit reaches them on every path, so a model that
    drops the limit looks successful to every old check. `limit_dropped` is the B8 measure (decision 0039)."""
    task = REGION[task_id]
    answer = reference(task_id)
    answer["rob"] = ANNOTATION.sub("", answer["rob"]).replace("@safety max_speed 0.5 m/s;\n", "")
    assert "no_go_region" not in answer["rob"] and "in_region" not in answer["rob"]
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds and judgement.state == "SUCCEEDED"
    assert not judgement.limit_ok and judgement.limit_dropped, task_id


def test_limit_ok_wants_the_right_kind_on_the_right_region_and_nothing_extra():
    task = REGION["r03_field_not_road"]  # in_region field_zone and no_go_region road_zone
    good = reference("r03_field_not_road")

    def verdict(rob: str, bindings: dict | None = None) -> tuple[bool | None, bool | None]:
        answer = dict(good, rob=rob, **({"bindings": bindings} if bindings else {}))
        checked = lib.check_answer(task, answer)
        assert checked.valid, checked.report
        judged = lib.judge(task, checked, answer["bindings"])
        return judged.limit_ok, judged.limit_dropped

    assert verdict(good["rob"]) == (True, False)
    swapped = good["rob"].replace("in_region field", "XX").replace("no_go_region road", "in_region road") \
        .replace("XX", "no_go_region field")
    assert verdict(swapped) == (False, True)                                  # the kinds are crossed
    assert verdict(good["rob"].replace("    @safety no_go_region road;\n", "")) == (False, True)   # one is dropped
    extra = good["rob"].replace("region road)", "region road, region pit)").replace(
        "    goal", "    @safety no_go_region pit;\n    goal")
    pit = dict(good["bindings"], pit="wet_floor")
    assert verdict(extra, pit) == (False, False)                              # all asked for, one more: not dropped
    wrong = dict(good["bindings"], road="wet_floor")
    assert verdict(good["rob"], wrong) == (False, True)                       # the right limit on the wrong region
    speed = reference("r05_slow_and_dry")
    task5 = REGION["r05_slow_and_dry"]

    def speed_verdict(rob: str) -> bool:
        checked = lib.check_answer(task5, dict(speed, rob=rob))
        assert checked.valid, checked.report
        return lib.judge(task5, checked, speed["bindings"]).limit_ok

    assert speed_verdict(speed["rob"]) is True
    assert speed_verdict(speed["rob"].replace("0.5 m/s", "0.4 m/s")) is False  # the speed asked for is exact
    assert speed_verdict(speed["rob"].replace("    @safety max_speed 0.5 m/s;\n", "")) is False


def test_a_task_without_expected_limits_has_no_limit_measure():
    plain = lib.load_set("1")
    task = next(iter(plain.values()))
    assert "expected_limits" not in task
    assert lib.limit_ok(task, None, {}) is None and lib.limit_dropped(task, None, {}, "SUCCEEDED") is None


@pytest.mark.parametrize("task_id", ["i01_hall_is_wet", "i02_hall_is_outside", "i03_spot_cannot_honour"])
def test_the_infeasible_attempt_is_refused_before_the_run_in_the_1_3_world(task_id):
    """P5 and P6: the obvious attempt, now a valid 1.7 program, fails `validate --world` (S004, S003) in this world."""
    expected = {"i01_hall_is_wet": "S004", "i02_hall_is_outside": "S004", "i03_spot_cannot_honour": "S003"}[task_id]
    checked = lib.check_answer(INFEASIBLE[task_id], attempt_answer(task_id, version="1.7"))
    assert not checked.valid and checked.codes[0] == expected, (task_id, checked.codes)


def test_the_delegated_limit_is_refused_by_delegate_not_by_the_world():
    task = INFEASIBLE["i04_delegated_limit"]
    answer = attempt_answer("i04_delegated_limit", version="1.7")
    checked = lib.check_answer(task, answer)
    assert checked.valid and task["expect_1_7"]["because"] == "0039 P8"  # the world allows it: Core cannot carry it
    from robolang.export_goal import ExportError, export_goal
    with pytest.raises(ExportError) as caught:
        export_goal(checked.task, checked.world, sender="rl:agent:planner-1", recipient="rl:robot:cleaner1")
    assert caught.value.code == "X605"


def test_the_inexpressible_tasks_stay_out_in_1_7():
    """P2, P3, P9: under 1.7 the per-step limit, the force and the distance limits are still errors; the arm limit and
    the busy areas can only be written as a half program, which is what the scorer counts as a false program."""
    codes = {t: lib.check_answer(INEXPRESSIBLE[t], attempt_answer(t, version="1.7")) for t in INEXPRESSIBLE}
    assert [codes[t].codes[0] for t in ("x01_first_leg_only", "x03_person_distance", "x04_grip_ten_newtons")] == [
        "E202", "E207", "E207"]
    assert codes["x02_arm_over_wet"].valid and codes["x05_avoid_busy_areas"].valid  # valid, and not what was asked


# -------- the controls --------

@pytest.mark.parametrize("task_id", sorted(CONTROLS))
def test_the_control_reference_is_valid_exports_an_accepted_goal_and_succeeds_on_every_path(task_id):
    task = CONTROLS[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 1.6;") == 1 and answer["rob"].isascii()
    assert not re.search(r"no_go_region|in_region", answer["rob"].split("\n", 1)[1])
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == "1.6"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert not judgement.half_program
    paths = lib.task_paths(task) or []
    assert [p["when"] for p in judgement.paths or []] == [p["when"] for p in paths]
    assert all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths or []), judgement.paths


def test_the_controls_need_no_limit_and_two_of_them_stop_where_the_request_says():
    assert "region_clear hot timeout 60 s" in reference("c01_wait_hot_zone_clear")["rob"]  # a wait, not a limit
    assert "@safety max_speed 0.5 m/s;" in reference("c04_speed_only")["rob"]
    for task_id in ("c01_wait_hot_zone_clear", "c03_layout_zone_clear_or_stop"):
        stops = [bool(p.get("fails") or p.get("fail")) for p in lib.task_paths(CONTROLS[task_id])]
        assert stops == [False, True], task_id
    assert lib.task_paths(CONTROLS["c02_plain_move"]) is None and lib.task_paths(CONTROLS["c04_speed_only"]) is None


# -------- the atlas --------

def test_only_the_tasks_1_7_still_declines_as_inexpressible_are_in_the_atlas_gaps():
    """The atlas lists inexpressible tasks. The 9 `region` programs and the 4 `infeasible` tasks left it at L4, and
    G-PATH stays open until the acceptance run (L5) and closes at L6 (C4): what remains of it is x01 and x02."""
    in_gaps = [(t, g["id"]) for g in ATLAS["gaps"] for t in g["tasks"] if t in SET]
    assert len(in_gaps) == len({t for t, _ in in_gaps}) == len(INEXPRESSIBLE)
    assert {t for t, _ in in_gaps} == set(INEXPRESSIBLE)
    for task_id, gap in in_gaps:
        assert INEXPRESSIBLE[task_id]["gap"] == gap and GAPS[gap]["scope"] == "open"
    assert Counter(gap for _, gap in in_gaps) == {"G-PATH": 2, "G-PROX": 2, "G-FORCE": 1}
    assert {t["gap"] for t in INEXPRESSIBLE.values()} == {"G-PATH", "G-PROX", "G-FORCE"}
    assert {t["gap"] for t in REGION.values()} | {t["gap"] for t in INFEASIBLE.values()} == {"G-PATH"}
    assert GAPS["G-PATH"]["scope"] == "open" and "L5" in GAPS["G-PATH"]["scope_note"]
    assert {t for g in ATLAS["gaps"] for t in g["tasks"]} & (set(REGION) | set(INFEASIBLE) | set(CONTROLS)) == set()


# -------- the scorer and the workflow --------

def regions_run(tmp_path: Path, replies: dict) -> Path:
    def script(request):
        first = request["messages"][0]["content"]
        task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
        return message(replies[task_id], model=request["model"])

    out_dir = tmp_path / "regions"
    run.run_benchmark(FakeClient(script), ["claude-sonnet-5-5"], {t: SET[t] for t in replies}, "medium", 1, out_dir,
                      lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="regions", prompt_name="full")
    return out_dir


def test_the_scorer_judges_the_limit_and_counts_a_dropped_one_and_a_refused_control(tmp_path):
    dropped = reference("r05_slow_and_dry")
    dropped["rob"] = dropped["rob"].replace("    @safety no_go_region wet;\n", "")  # the speed stays, the region goes
    replies = {
        "r01_runner_keep_off_hot": reference("r01_runner_keep_off_hot"),     # right: the limit is declared
        "r02_tug_stay_in_lane": decline(),                                   # a decline of a region task: false refusal
        "r05_slow_and_dry": dropped,                                         # B8: valid, succeeding, limit lost
        "i01_hall_is_wet": decline("infeasible"),                            # right, right category
        "i03_spot_cannot_honour": decline(),                                 # declined, category differs
        "x01_first_leg_only": decline(),                                     # right
        "c04_speed_only": reference("c04_speed_only"),                       # right
        "c02_plain_move": decline(),                                         # false refusal of a control
    }
    out_dir = regions_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "regions" and meta["prompt"] == "full"
    right = by["r01_runner_keep_off_hot"]
    assert right["decision_correct"] and right["goal_correct"] and right["limit_ok"] is True
    assert right["limit_dropped"] is False and right["request_satisfied"] and right["checks_ok"]
    assert by["r02_tug_stay_in_lane"]["false_refusal"] and by["r02_tug_stay_in_lane"]["limit_ok"] is False
    assert by["r02_tug_stay_in_lane"]["limit_dropped"] is False and not by["r02_tug_stay_in_lane"]["request_satisfied"]
    lost = by["r05_slow_and_dry"]
    assert lost["decision_correct"] and lost["goal_correct"] and lost["plan_succeeds"]  # every old check passes
    assert lost["limit_ok"] is False and lost["limit_dropped"] is True and not lost["request_satisfied"]
    assert by["i01_hall_is_wet"]["decision_correct"] and by["i01_hall_is_wet"]["category_correct"]
    spot = by["i03_spot_cannot_honour"]
    assert spot["decision_correct"] and spot["category_correct"] is False
    assert by["i01_hall_is_wet"]["limit_ok"] is None and by["x01_first_leg_only"]["limit_dropped"] is None
    control = by["c04_speed_only"]
    assert control["decision_correct"] and control["goal_correct"] and control["limit_ok"] is True
    assert control["limit_dropped"] is None  # a control asks for no region limit
    assert by["c02_plain_move"]["false_refusal"] and by["c02_plain_move"]["limit_ok"] is False
    text = score.score_dir(out_dir)
    assert "Task set regions, prompt full." in text
    kinds = text.split("By kind of task:")[1]
    for kind in ("region", "infeasible", "control"):
        assert f"| {kind} |" in kinds
    assert ("| limit_ok (exactly the region limits asked, none on a control; all answers to those tasks) "
            "| 2/5 | 2/5 (40.0%) |") in text
    assert ("| limit_dropped_silently (valid, SUCCEEDED, an asked region limit missing; all answers to tasks that "
            "ask for one) | 1/3 | 1/3 (33.3%) |") in text


def test_a_control_with_a_region_limit_added_is_not_limit_ok(tmp_path):
    over = reference("c02_plain_move")
    over["rob"] = over["rob"].replace("robolang 1.6;", "robolang 1.7;")
    over["rob"] = over["rob"].replace("location dst", "location dst, region wet")
    over["rob"] = over["rob"].replace("    goal", "    @safety no_go_region wet;\n    goal")
    over["bindings"] = dict(over["bindings"], wet="wet_floor")
    _, rows = score.load_rows(regions_run(tmp_path, {"c02_plain_move": over}))
    row = rows[0]
    assert row["decision_correct"] and row["goal_correct"] and row["limit_ok"] is False
    assert not row["request_satisfied"] and row["limit_dropped"] is None


def test_the_set_is_in_the_run_script_and_only_runs_on_demand_in_the_workflow():
    assert lib.SETS["regions"] == (lib.REGIONS_TASKS_DIR, lib.REGIONS_REFERENCE_DIR)
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    assert workflow.count("--set regions") == 2
    lines = [ln for ln in workflow.splitlines() if "--set regions" in ln]
    for part in ("--repeats 3", "--models claude-sonnet-5-5,claude-haiku-5-5", "--max-usd 3"):
        assert all(part in ln for ln in lines), part
    assert any("--prompt full" in ln and "--run-id regions" in ln for ln in lines)
    assert any("--prompt minimal" in ln and "--run-id regions-minimal" in ln for ln in lines)
    assert all("claude-opus" not in ln and "--set regions --prompt" in ln for ln in lines)
    steps = re.split(r"\n      - ", workflow)
    step = next(s for s in steps if s.startswith("name: Run (set regions"))
    assert "inputs.regions" in step and "pull_request" not in step
    pr_step = next(s for s in steps if s.startswith("name: Run (pull request)"))
    weekly = next(s for s in steps if s.startswith("name: Run (weekly or on demand)"))
    assert "regions" not in pr_step and "!inputs.regions" in weekly and "--set regions" not in weekly
    declared = r"      regions:\n        description: .*\n        type: boolean\n        default: false"
    assert re.search(declared, workflow)
    assert re.search(r"for extra in .*regions regions-minimal.*; do", workflow)
