"""
Benchmark set 6, "regions" (bench/tasks_regions/, decision record 0039, plan docs/plans/bench-6-regions.md): region
limits on motion (`@safety no_go_region R;`, `@safety in_region R;`), pre-registered against RoboLang 1.6 before any
code (L3 of iteration I1). `expect` is the 1.6 label (the baseline), `expect_1_7` the label for acceptance.

9 `region` requests that become programs in 1.7, 4 `infeasible` in 1.7 (0039 P5, P6, P8), 5 `inexpressible` that stay
out (0039 P2, P3, P9; G-PROX, G-FORCE), 4 `control` requests that are programs in both versions. 1.6 declines every
non-control task, and every one has an obvious attempt that 1.6 rejects with a named diagnostic. Only the controls have
a 1.6 reference; the 1.7 references wait for L4. No network and no SDK; the model run is the `Benchmark` workflow with
`regions: true`.
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
DECLINED = {**REGION, **INFEASIBLE, **INEXPRESSIBLE}  # what 1.6 must decline: each has an attempt, each is in the atlas
ANNOTATION = re.compile(r"^\s*@safety (?:no_go_region|in_region|min_distance|max_force)[^\n]*\n", re.M)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def reference(task_id: str) -> dict:
    ref = json.loads(read(REF / f"{task_id}.json"))
    rob = read(REF / f"{task_id}.rob")
    return {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob, **ref}


def attempt_answer(task_id: str, strip: bool = False) -> dict:
    """The obvious attempt as a program answer; `strip` removes its one offending construct (a half program in 1.6)."""
    stem = REF / f"{task_id}.attempt"
    rob = read(Path(f"{stem}.rob"))
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


def test_labels_are_1_6_in_expect_and_1_7_in_expect_1_7():
    for task_id, task in SET.items():
        assert "expect_1_7" in task and task["expect_1_7"]["decision"] in ("program", "cannot"), task_id
        if task["group"] == "control":
            assert task["expect"] == {"decision": "program"} and lib.expected_goals(task), task_id
            assert task["expect_1_7"] == {"decision": "program"}
            assert not {"why", "attempt_fails_with", "gap", "expected_limits"} & set(task)
            continue
        assert task["expect"] == CANNOT and lib.expectation(task) == ("cannot", "inexpressible"), task_id
        assert len(task["why"]) > 40 and re.fullmatch(r"[ES]\d{3}", task["attempt_fails_with"]), task_id
        assert not {"expected_goal", "expected_goals", "expect_1_6", "expect_1_5"} & set(task)
    for task_id, task in REGION.items():
        later = task["expect_1_7"]
        assert later["decision"] == "program" and later["paths"] and "category" not in later
        for path in later["paths"]:
            assert path["when"] and path["goal"] and "world_patch" in path
        limits = task["expected_limits"]
        assert 1 <= len(limits) <= 8 and len({tuple(x) for x in limits}) == len(limits)  # 0039 P4: at most 8
        assert {kind for kind, _ in limits} <= {"no_go_region", "in_region"}
        assert not {r for k, r in limits if k == "no_go_region"} & {r for k, r in limits if k == "in_region"}  # P4
        world = json.loads(read(REPO / task["world"]))
        assert {region for _, region in limits} <= set(world["regions"]), task_id  # the regions the request names
    for task_id, task in INFEASIBLE.items():
        later = task["expect_1_7"]
        assert later["decision"] == "cannot" and later["category"] == "infeasible", task_id
        assert later["because"] in ("0039 P5", "0039 P6", "0039 P8") and task["blockers"], task_id
        assert ("delegated" in later) == (task_id == "i04_delegated_limit")
    for task_id, task in INEXPRESSIBLE.items():
        assert task["expect_1_7"] == CANNOT, task_id
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


def test_no_world_1_3_field_exists_yet_and_no_world_file_is_new():
    """1.3 facts (`contains`, `honours`) are recorded in the labels only: L4 writes the world copies."""
    worlds = {Path(t["world"]).stem for t in SET.values()}
    assert worlds <= {"kitchen", "kitchen_values", "yard", "site", "pharmacy", "cleanroom", "warehouse"}
    for name in worlds:
        text = read(REPO / f"bench/worlds/{name}.json")
        assert '"honours"' not in text and '"contains"' not in text
        assert json.loads(text)["world_schema_version"] in ("1.1", "1.2")
    for task_id, task in SET.items():
        has_facts = "world_1_3" in task["expect_1_7"]
        assert "world_1_3" not in task and has_facts == (task["group"] in ("region", "infeasible"))
    for task in list(REGION.values()) + list(INFEASIBLE.values()):
        facts = task["expect_1_7"]["world_1_3"]
        assert set(facts) <= {"regions", "robots"}
        assert all(set(r) == {"honours"} for r in facts["robots"].values())
        world = json.loads(read(REPO / task["world"]))
        assert set(facts["robots"]) <= set(world["robots"]) and set(facts.get("regions", {})) <= set(world["regions"])


# -------- the references and the attempts --------

def test_references_and_attempts_are_where_they_should_be():
    names = [p.name for p in REF.iterdir()]
    refs = {n.split(".")[0] for n in names if ".attempt" not in n}
    attempts = {n.split(".")[0] for n in names if ".attempt." in n}
    assert refs == set(CONTROLS) and attempts == set(DECLINED)  # no 1.7 reference yet: that is L4
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
def test_the_1_7_goal_of_every_path_is_reached_by_the_program_without_the_limit(task_id):
    """The 1.7 goals are right for the world: the half program (no limit) reaches them on every path, so a model that
    drops the limit looks successful to every old check and only the limit check can tell (plan, acceptance)."""
    task = REGION[task_id]
    answer = attempt_answer(task_id, strip=True)
    checked = lib.check_answer(task, answer)
    labelled = dict(task, expect_1_6=task["expect_1_7"])  # task_paths reads 1.6 paths: L4 will read expect_1_7
    judgement = lib.judge(labelled, checked, answer["bindings"])
    assert [p["when"] for p in judgement.paths] == [p["when"] for p in task["expect_1_7"]["paths"]]
    assert all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths), judgement.paths


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

def test_every_task_1_6_declines_is_in_exactly_one_atlas_gap_and_no_control_is():
    in_gaps = [(t, g["id"]) for g in ATLAS["gaps"] for t in g["tasks"] if t in SET]
    assert len(in_gaps) == len({t for t, _ in in_gaps}) == len(DECLINED)
    assert {t for t, _ in in_gaps} == set(DECLINED)
    for task_id, gap in in_gaps:
        assert DECLINED[task_id]["gap"] == gap and GAPS[gap]["scope"] == "open"
    assert Counter(gap for _, gap in in_gaps) == {"G-PATH": 15, "G-PROX": 1, "G-FORCE": 1, "G-CHOOSE": 1}
    assert {t["gap"] for t in INEXPRESSIBLE.values()} == {"G-PATH", "G-PROX", "G-FORCE", "G-CHOOSE"}
    assert {t["gap"] for t in REGION.values()} | {t["gap"] for t in INFEASIBLE.values()} == {"G-PATH"}


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


def test_the_scorer_counts_a_half_program_as_false_and_a_decline_of_a_control_as_a_refusal(tmp_path):
    replies = {
        "r01_runner_keep_off_hot": decline(),                              # right under 1.6
        "i01_hall_is_wet": decline("infeasible"),                          # declined, category differs (reported only)
        "r05_slow_and_dry": attempt_answer("r05_slow_and_dry", strip=True),  # the speed only: a false program
        "c04_speed_only": reference("c04_speed_only"),                     # right
        "c02_plain_move": decline(),                                       # false refusal of a control
    }
    out_dir = regions_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "regions" and meta["prompt"] == "full"
    assert by["r01_runner_keep_off_hot"]["decision_correct"] and by["r01_runner_keep_off_hot"]["category_correct"]
    assert by["i01_hall_is_wet"]["decision_correct"] and by["i01_hall_is_wet"]["category_correct"] is False
    assert by["r05_slow_and_dry"]["false_program"] and not by["r05_slow_and_dry"]["decision_correct"]
    assert by["c04_speed_only"]["decision_correct"] and by["c04_speed_only"]["goal_correct"]
    assert by["c02_plain_move"]["false_refusal"] and not by["c02_plain_move"]["decision_correct"]
    text = score.score_dir(out_dir)
    assert "Task set regions, prompt full." in text
    kinds = text.split("By kind of task:")[1]
    for kind in ("region", "infeasible", "control"):
        assert f"| {kind} |" in kinds


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
    assert "regions regions-minimal; do" in workflow
