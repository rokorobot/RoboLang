"""
Benchmark set 5, "values" (bench/tasks_values/, decision record 0036, plan docs/plans/bench-5-values.md): measured
values. Pre-registered against RoboLang 1.5 in PR 1 (baseline docs/evidence/bench-set5-baseline.md); labelled for 1.6
in PR 2 (errata docs/evidence/bench-errata-1-6.md): `expect` is the 1.6 label, `expect_1_5` keeps the 1.5 one and
`expect_1_6.paths` holds the world patches that force each path.

15 `value` requests that need a reading: programs, each with paths. 6 `inexpressible` requests that stay
inexpressible in 1.6. 3 `infeasible` requests (inexpressible in 1.5). 2 `control` requests that need no reading.
Every program task has a 1.6 reference that must be right on every path. Also here: the scorer's `value_ok` and
`silent_substitution` (0036 V13). No network and no SDK.
"""

from __future__ import annotations
import io
import json
from collections import Counter
from pathlib import Path

import pytest
from test_bench import FakeClient, message

from bench import lib, run, score

REPO = Path(__file__).resolve().parent.parent
SET = lib.load_set("values")
REF = lib.VALUES_REFERENCE_DIR
CANNOT = {"decision": "cannot", "category": "inexpressible"}
PROGRAMS = sorted(t for t, d in SET.items() if lib.expectation(d)[0] == "program")


def reference(task_id: str, directory: Path = REF) -> dict:
    ref = json.loads((directory / f"{task_id}.json").read_text(encoding="utf-8"))
    rob = (directory / f"{task_id}.rob").read_text(encoding="utf-8")
    return {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob, **ref}


def judged(task_id: str, answer: dict):
    checked = lib.check_answer(SET[task_id], answer)
    assert checked.valid, checked.report
    return checked, lib.judge(SET[task_id], checked, answer["bindings"])


def test_the_mix_of_tasks():
    assert len(SET) == 26
    assert Counter(d["group"] for d in SET.values()) == {"value": 15, "inexpressible": 6, "infeasible": 3, "control": 2}
    for task_id, task in SET.items():
        assert task["id"] == task_id and (REPO / task["world"]).is_file() and task["tags"]
        assert task["prompt"].endswith(".") and "expect_1_6" in task and "expect_1_5" in task


def test_expectations_are_labelled_for_1_6_and_keep_the_1_5_label():
    for task_id, task in SET.items():
        later = task["expect_1_6"]
        assert task["expect_1_5"] == ({"decision": "program"} if task["group"] == "control" else CANNOT), task_id
        assert lib.expectation(task) == (later["decision"], later.get("category", "none")), task_id
        if task["group"] == "control":
            assert later == {"decision": "program"} and lib.expected_goals(task)
            continue
        assert len(task["why"]) > 40
        assert "RoboLang 1.5 cannot read" not in task["why"] and "1.5 also cannot" not in task["why"]
        if task["group"] == "value":
            assert task["expect"] == {"decision": "program"} and lib.task_paths(task) == later["paths"]
            assert "planned_paths" not in later and later["paths"]
        elif task["group"] == "inexpressible":
            assert later == task["expect"] == CANNOT
        else:
            assert later == task["expect"] == {"decision": "cannot", "category": "infeasible"} and task["blockers"]
            assert "entities_1_6" not in task


def test_every_path_has_a_world_patch_and_a_clear_outcome():
    for task_id in (t for t, d in SET.items() if d["group"] == "value"):
        for p in SET[task_id]["expect_1_6"]["paths"]:
            assert p["when"] and "world_patch" in p
            assert sum(k in p for k in ("goal", "goals", "fails")) >= (0 if SET[task_id].get("goal_free") else 1), p


def test_the_world_variant_differs_only_where_it_says():
    base = json.loads((REPO / "bench/worlds/kitchen.json").read_text(encoding="utf-8"))
    variant = json.loads((REPO / "bench/worlds/kitchen_values.json").read_text(encoding="utf-8"))
    assert base["world_schema_version"] == variant["world_schema_version"] == "1.2"
    assert base["objects"]["knife_1"]["held_by"] == "chef_arm" and not base["sensors"]["scale_1"]["available"]
    knife = variant["objects"]["knife_1"]
    assert knife == {"present": True, "location": "prep_1", "class": "knife"}
    assert variant["sensors"]["scale_1"]["available"] is True
    for key in ("objects", "sensors"):
        base[key], variant[key] = dict(base[key]), dict(variant[key])
    base["objects"].pop("knife_1"), variant["objects"].pop("knife_1")
    scale_base, scale_variant = base["sensors"].pop("scale_1"), variant["sensors"].pop("scale_1")
    assert scale_base["measures"] == scale_variant["measures"]
    assert base == variant
    assert SET["i01_scale_offline"]["world"] == "bench/worlds/kitchen.json"


def test_the_worlds_describe_what_sensors_measure_but_never_what_objects_read():
    """The world file 1.2 benchmark worlds: `measures` on the value sensors, `properties` on the objects asked about."""
    for name in ("kitchen", "kitchen_values", "pharmacy", "cleanroom", "warehouse"):
        world = json.loads((REPO / f"bench/worlds/{name}.json").read_text(encoding="utf-8"))
        assert world["world_schema_version"] == "1.2"
        shown = lib.describe_world(world)
        assert "properties" not in shown and "LOT-7731" not in shown and " 55 " not in shown
        assert "measures" in shown  # every one of these worlds has a sensor that measures something
        for sensor in world["sensors"].values():
            assert all(set(m) <= {"unit", "uncertainty", "contact"} for m in sensor.get("measures", {}).values())
    kitchen = json.loads((REPO / "bench/worlds/kitchen.json").read_text(encoding="utf-8"))
    shown = lib.describe_world(kitchen)
    assert "probe_temp (on chef_arm, available; measures temperature in degC +/- 0.5, contact)" in shown
    assert "cam_chef (on chef_arm, available)" in shown  # a camera measures nothing
    pharmacy = json.loads((REPO / "bench/worlds/pharmacy.json").read_text(encoding="utf-8"))
    assert "scan_p1 (on picker1, available; measures label)" in lib.describe_world(pharmacy)


def test_a_property_event_is_not_shown_and_other_worlds_are_described_as_before():
    world = json.loads((REPO / "bench/worlds/kitchen_values.json").read_text(encoding="utf-8"))
    world["events"].append({"at": {"value": "240", "unit": "s"}, "fact": "property", "target": "pan_1",
                            "property": "temperature", "value": {"value": "55", "unit": "degC"}})
    shown = lib.describe_world(world)
    assert "property" not in shown and "pan_1 becomes" not in shown and "scheduled: after 30 s, region_clear" in shown
    old = json.loads((REPO / "bench/worlds/yard.json").read_text(encoding="utf-8"))
    assert "measures" not in lib.describe_world(old) and "; measures" not in lib.describe_world(old)


@pytest.mark.parametrize("task_id", PROGRAMS)
def test_the_reference_is_a_1_6_program_and_is_right_on_every_path(task_id):
    answer = reference(task_id)
    assert answer["rob"].count("robolang 1.6;") == 1 and answer["rob"].isascii()
    checked, judgement = judged(task_id, answer)
    assert checked.language_version == "1.6"
    task = SET[task_id]
    if task["group"] == "control":
        assert "measure" not in answer["rob"].split("\n", 1)[1]  # line 1 is the file comment
        assert judgement.goal_correct and judgement.plan_succeeds
        return
    assert [p["when"] for p in judgement.paths] == [p["when"] for p in task["expect_1_6"]["paths"]]
    assert all(p["goal_ok"] and p["plan_ok"] for p in judgement.paths), judgement.paths
    assert judgement.goal_correct and judgement.plan_succeeds and all(judgement.checks.values())
    assert not judgement.half_program and judgement.silent_substitution in (None, False)
    if lib.has_value_expectations(task):
        assert judgement.value_ok is True
    # a live reading is a comparison in `wait until`, not a `measure`: the two wait tasks have no recorded reading
    assert ("measure" in answer["rob"]) == (task_id != "v03_wait_to_cool")


def test_reference_files_exist_only_for_programs():
    assert {p.name.split(".")[0] for p in REF.iterdir()} == set(PROGRAMS)
    assert len(PROGRAMS) == 17


def test_the_stop_paths_really_stop_and_the_probe_values_are_the_planned_ones():
    paths = {p["when"]: p for p in SET["v02_cool_enough"]["expect_1_6"]["paths"]}
    assert sorted(paths) == ["59.8 +/- 0.5 degC is unknown against 60 degC: the run fails (S261)",
                             "pan 1 at 45 degC", "pan 1 at 80 degC: left on the stove, message sent"]
    answer = reference("v02_cool_enough")
    checked = lib.check_answer(SET["v02_cool_enough"], answer)
    for path in paths.values():
        if path.get("fails"):
            patched = lib._merge(json.loads(lib.world_bytes(SET["v02_cool_enough"])), path["world_patch"])
            world = lib.load_world(json.dumps(patched).encode(), "world.json", checked.task, answer["bindings"])
            result = lib.simulate_task(checked.task, world)
            assert result.state == "FAILED" and result.failure.code == "S261"
    cool = SET["v03_wait_to_cool"]["expect_1_6"]["paths"][0]["world_patch"]["events"]
    assert [e["fact"] for e in cool] == ["region_clear", "property"] and cool[1]["at"] == {"value": "240", "unit": "s"}


def test_u06_style_waits_accept_any_timeout_when_the_pan_is_already_cool():
    """0036 relabels (u06): pan 1 is below 60 degC in the world: a short or a long timeout both pass; else fail."""
    task = SET["v03_wait_to_cool"]
    for timeout in ("1 s", "10 min"):
        answer = reference("v03_wait_to_cool")
        answer["rob"] = answer["rob"].replace("10 min", timeout)
        _, judgement = judged("v03_wait_to_cool", answer)
        assert judgement.paths[1]["goal_ok"], timeout
    assert task["expect_1_6"]["paths"][1]["fails"]


def test_a_retry_in_a_handler_reads_the_same_name_when_the_first_reading_fails():
    """v15 option b (0036 V3): a world patch cannot make only the first try fail; a scripted fault does."""
    task = SET["v15_retry_reading"]
    answer = reference("v15_retry_reading")
    world = json.loads(lib.world_bytes(task))
    world["tasks"] = {answer["task"]: {"bindings": answer["bindings"], "faults": [{"step": 2}]}}  # the first `measure`
    checked = lib.check_answer(task, answer, json.dumps(world).encode())
    assert checked.valid, checked.report
    result = lib.simulate_task(checked.task, checked.world)
    assert result.state == "SUCCEEDED"
    assert [(s.path, s.step.KIND, s.status, s.handled) for s in result.steps] == [
        ("1", "move", "succeeded", None), ("2", "measure", "failed", "S251"),
        ("2.else.1", "measure", "succeeded", None), ("3", "communicate", "succeeded", None)]
    sent = [s.message.value for s in result.steps if s.message and s.message.value]
    assert [(v.subject, v.kind, str(v.value)) for v in sent] == [("pan_1", "temperature", "55")]


# -------- the scorer: value_ok and silent_substitution (0036 V13) --------

def record(task_id: str, answer: dict) -> dict:
    return {"task_id": task_id, "model": "claude-opus-5-5", "repeat": 1,
            "calls": [{"request": {"model": "claude-opus-5-5"}, "response": message(answer)}]}


def row(task_id: str, answer: dict) -> dict:
    return score.evaluate_record(SET[task_id], record(task_id, answer))


def test_the_reference_is_value_ok_and_not_a_silent_substitution():
    for task_id in ("v01_probe_report", "v04_weigh_then_move", "v13_relay_batch"):
        r = row(task_id, reference(task_id))
        assert r["value_ok"] is True and r["silent_substitution"] is False, task_id
        assert r["goal_correct"] and r["plan_succeeds"] and r["request_satisfied"]


def test_the_v04_baseline_answer_is_a_silent_substitution_and_not_value_ok():
    """The set 5 baseline (docs/evidence/bench-set5-baseline.md): `inspect r tray using scale`, then the move."""
    answer = reference("v04_weigh_then_move")
    answer["rob"] = answer["rob"].replace("measure r mass of tray using scale as m;", "inspect r tray using scale;")
    assert "measure" not in answer["rob"].split("\n", 1)[1]
    r = row("v04_weigh_then_move", answer)
    assert r["valid_first"] and r["goal_correct"] and r["plan_succeeds"]  # the old checks cannot tell
    assert r["silent_substitution"] is True and r["value_ok"] is False
    assert r["false_program"] is False  # v04 expects a program in 1.6


def test_a_measure_of_the_wrong_thing_is_not_value_ok_but_is_not_a_substitution_either():
    wrong = reference("v01_probe_report")
    wrong["bindings"]["pan"] = "pan_2"  # reads pan 2 and sends it; the request asked for pan 1
    assert row("v01_probe_report", wrong)["value_ok"] is False
    text = reference("v01_probe_report")
    text["rob"] = text["rob"].replace('communicate r to "pass" with t;', 'communicate r to "pass" with "55";')
    r = row("v01_probe_report", text)
    assert r["goal_correct"] and r["plan_succeeds"] and r["value_ok"] is False  # a literal, not the reading
    assert r["silent_substitution"] is False
    both = reference("v01_probe_report")
    measure = "measure r temperature of pan using probe as t;"
    both["rob"] = both["rob"].replace(measure, "inspect r pan using probe;\n        " + measure)
    assert row("v01_probe_report", both)["silent_substitution"] is False  # it did measure


def test_a_value_sent_on_the_wrong_channel_or_of_the_wrong_kind_is_not_value_ok():
    channel = reference("v09_online_thermometer")
    channel["rob"] = channel["rob"].replace('"lab"', '"ops"')
    assert row("v09_online_thermometer", channel)["value_ok"] is False
    kind = reference("v09_online_thermometer")
    kind["rob"] = kind["rob"].replace("measure r temperature of vial", "measure r mass of vial")
    assert row("v09_online_thermometer", kind)["valid_first"] is False  # thermo_1 does not measure mass: S234


def test_value_ok_is_judged_on_every_path_that_does_not_stop():
    """v07: the temperature goes to qa on the warm path only; a program that never sends it fails that path."""
    silent = reference("v07_cold_chain")
    silent["rob"] = silent["rob"].replace('communicate r to "qa" with t;', 'communicate r to "qa" with "warm";')
    r = row("v07_cold_chain", silent)
    assert r["goal_correct"] and r["value_ok"] is False
    assert [p["value_ok"] for p in r["paths"]] == [True, False]
    ok = row("v07_cold_chain", reference("v07_cold_chain"))
    assert [p["value_ok"] for p in ok["paths"]] == [True, True]


def test_value_ok_and_silent_substitution_are_none_for_tasks_that_ask_for_no_reading():
    r = row("c02_inspect_not_measure", reference("c02_inspect_not_measure"))
    assert r["value_ok"] is None and r["silent_substitution"] is None
    assert row("v03_wait_to_cool", reference("v03_wait_to_cool"))["silent_substitution"] is None  # a live reading
    declined = row("v04_weigh_then_move", {"decision": "cannot", "category": "inexpressible", "reason": "no",
                                           "entities": [], "rob": "", "task": "", "bindings": []})
    assert declined["value_ok"] is False and declined["silent_substitution"] is False and declined["false_refusal"]


def test_other_sets_use_the_same_measures():
    """w_scale_precheck (set 1), d_serum_tube (set 2), f07 (set "ask"): an `inspect` no longer counts as a reading."""
    from test_bench import TASKS, reference_answer
    from test_bench_ask import SET3, answer as ask_answer
    from test_bench_hard import HARD, hard_reference
    cases = [(TASKS["w_scale_precheck"], reference_answer("w_scale_precheck"),
              "measure r mass of item using scale as m;", "inspect r item using scale;"),
             (HARD["d_serum_tube"], hard_reference("d_serum_tube"), "measure r temperature of box using therm as t;",
              "inspect r box using therm;"),
             (SET3["f07_thermo_then_dispatch"], ask_answer("f07_thermo_then_dispatch"),
              "measure r temperature of o using s as t;", "inspect r o using s;")]
    for task, ref, measure, inspect in cases:
        ref = {"decision": "program", "category": "none", "reason": "r", "entities": [], **ref}
        good = score.evaluate_record(task, record(task["id"], ref))
        assert good["value_ok"] is True and good["silent_substitution"] is False, task["id"]
        assert good["request_satisfied"], task["id"]
        bad = dict(ref, rob=ref["rob"].replace(measure, inspect))
        r = score.evaluate_record(task, record(task["id"], bad))
        assert r["valid_first"] and r["silent_substitution"] is True and r["value_ok"] is False, task["id"]


def test_the_report_and_scores_json_show_value_ok_and_silent_substitution(tmp_path):
    good, bad = reference("v04_weigh_then_move"), reference("v04_weigh_then_move")
    bad["rob"] = bad["rob"].replace("measure r mass of tray using scale as m;", "inspect r tray using scale;")
    replies = {"v04_weigh_then_move": [good, bad], "v01_probe_report": [reference("v01_probe_report")] * 2}
    counter = {}

    def script(request):
        first = request["messages"][0]["content"]
        task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
        n = counter[task_id] = counter.get(task_id, -1) + 1
        return message(replies[task_id][n], model=request["model"])

    out_dir = tmp_path / "values"
    run.run_benchmark(FakeClient(script), ["claude-opus-5-5"], {t: SET[t] for t in replies}, "medium", 2, out_dir,
                      lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="values", prompt_name="full")
    text = score.score_dir(out_dir)
    assert "| value_ok " in text and "| silent_substitution " in text
    assert "value_ok (every non-stopping path took the reading and sent the value as asked) | 3/4 " in text
    assert "all answers to those tasks) | 1/4 " in text
    rows = json.loads((out_dir / "scores.json").read_text(encoding="utf-8"))
    assert {r["task_id"] for r in rows} == set(replies)
    assert all("value_ok" in r and "silent_substitution" in r for r in rows)
    assert sorted((r["task_id"], r["value_ok"], r["silent_substitution"]) for r in rows) == [
        ("v01_probe_report", True, False), ("v01_probe_report", True, False),
        ("v04_weigh_then_move", False, True), ("v04_weigh_then_move", True, False)]
