"""
Benchmark set 11, "kinds" (bench/tasks_kinds/, decision record 0044, plan docs/plans/bench-11-kinds.md): four more
measured kinds for the 1.6 `measure`, `fill` and `confidence` (percent, unit token `%`), `pressure` (Pa, kPa, bar) and
`count` (a whole number, no unit, exact, `==` allowed), pre-registered against RoboLang 2.1 before any code (L3 of
iteration I6). `expect` is the 2.1 label (the baseline: every non-control task declines as inexpressible, because
`fill`, `confidence`, `pressure` and `count` are no kind of `measure`: E217, and `%` is no token: E101); `expect_2_2` is
the label for acceptance once 2.2 is taught: the paths of a program (each forced by world file 1.7 property values in
its `world_patch`), `world_2_2` (the sensor `measures` and object `properties` the 1.7 world adds), and the checks a
label asks for (`expected_readings`, `expected_comparisons`, `expected_limits`, `expected_actions`, `expected_clock`,
`expected_message_waits`). They are kept inside `expect_2_2` so that the 2.1 scorer does not read them.

10 `kind` requests that are programs in 2.2, 5 `infeasible` in 2.2, 5 `inexpressible` that stay out (0044 K8:
G-PERCEIVE, G-SELF, G-CHOOSE, G-CALC, G-PROX), 4 `control` requests that are programs in both versions (a plain carry,
a 1.6 `temperature`, `mass` and `label` reading: no new kind). 2.1 declines every non-control task, and every one has an
obvious attempt that 2.1 rejects; those attempts stay as the evidence of the baseline. Only the controls have a
reference; the 2.2 references wait for L4. Erratum K6 (0044) relabels the "ask" tasks `u01_if_dirty` and
`u02_if_damaged`.
No network and no SDK; the model run is the `Benchmark` workflow with `set_only: kinds`.
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
from robolang.measured import Interval, compare_intervals

REPO = Path(__file__).resolve().parent.parent
SET = lib.load_set("kinds")
REF = lib.KINDS_REFERENCE_DIR
ATLAS = json.loads((REPO / "bench" / "atlas.json").read_text(encoding="utf-8"))
GAPS = {g["id"]: g for g in ATLAS["gaps"]}
CANNOT = {"decision": "cannot", "category": "inexpressible"}
PROGRAMS = {t: d for t, d in SET.items() if d["group"] == "kind"}
INFEASIBLE = {t: d for t, d in SET.items() if d["group"] == "infeasible"}
INEXPRESSIBLE = {t: d for t, d in SET.items() if d["group"] == "inexpressible"}
CONTROLS = {t: d for t, d in SET.items() if d["group"] == "control"}
DECLINED = {**PROGRAMS, **INFEASIBLE, **INEXPRESSIBLE}  # what 2.1 declines: an attempt each
NEW_KINDS = ("fill", "confidence", "pressure", "count")
UNITS = {"fill": {"%"}, "confidence": {"%"}, "pressure": {"Pa", "kPa", "bar"}, "count": set()}
FACTOR = {"%": Decimal(1), "Pa": Decimal(1), "kPa": Decimal(1000), "bar": Decimal(100000)}  # 1 bar = 100 kPa (0044 K2)
SECONDS = {"ms": 0.001, "s": 1, "min": 60}
NEW_WORLDS = {"stockroom.json", "plant_room.json"}
THEN = {"k01_restock_under_thirty_percent": "restock shelf_2", "k02_pump_three_over_six_bar": "pump 3 over pressure",
        "k03_shelf_three_out_of_stock": "shelf_3 is out of stock", "k04_weed_and_ninety_percent_sure": "weed confirmed",
        "k05_pump_one_below_450_kpa": "pump 1 low pressure", "k06_tank_close_to_the_mark": "tank_2 is nearly full",
        "k07_around_the_forklift_lane": "shelf_3 is low", "k09_nine_oclock_scan_on_the_signal": "shelf_1 is empty",
        "k10_the_right_camera": "shelf_3 is nearly empty"}
ELSE = {"k02_pump_three_over_six_bar": "pump 3 ok", "k04_weed_and_ninety_percent_sure": "plant_7 skipped"}


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def reference(task_id: str) -> dict:
    ref = json.loads(read(REF / f"{task_id}.json"))
    rob = read(REF / f"{task_id}.rob")
    return {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob, **ref}


def strip_reading(rob: str, bindings: dict) -> tuple[str, dict]:
    """The attempt without its reading: the lines marked `// @reading` go, and so does each parameter (and its
    binding) that nothing else uses (the `person` of `x05`)."""
    kept = "".join(ln for ln in rob.splitlines(keepends=True) if not ln.rstrip().endswith("// @reading"))
    head = re.search(r"^(task \w+)\((.*)\) \{$", kept, re.M)
    body = kept[head.end():]
    params = [p for p in head.group(2).split(", ") if re.search(rf"\b{p.split()[-1]}\b", body)]
    kept = kept[:head.start()] + f"{head.group(1)}({', '.join(params)}) {{" + body
    return kept, {k: v for k, v in bindings.items() if k in {p.split()[-1] for p in params}}


def attempt_answer(task_id: str, strip: bool = False) -> dict:
    stem = REF / f"{task_id}.attempt"
    rob, bindings = read(Path(f"{stem}.rob")), json.loads(read(Path(f"{stem}.json")))
    if strip:
        rob, bindings["bindings"] = strip_reading(rob, bindings["bindings"])
    return {"decision": "program", "category": "none", "reason": "an attempt", "entities": [], "rob": rob, **bindings}


def decline(category: str = "inexpressible") -> dict:
    return {"decision": "cannot", "category": category, "reason": "cannot be done", "rob": "", "task": "",
            "bindings": []}


def base_world(task: dict) -> dict:
    return json.loads(read(REPO / task["world"]))


def world_ids(task: dict) -> set[str]:
    world = base_world(task)
    return {*world["robots"], *world["objects"], *world["locations"], *world.get("sensors", {}),
            *world.get("regions", {})}


def seconds(duration: str | dict) -> float:
    if isinstance(duration, dict):
        return float(duration["value"]) * SECONDS[duration["unit"]]
    value, unit = duration.split()
    return float(value) * SECONDS[unit]


def label_2_2(task: dict) -> dict:
    return task["expect_2_2"]


# -------- the set --------

def test_the_mix_of_tasks():
    assert len(SET) == 24
    assert Counter(d["group"] for d in SET.values()) == {"kind": 10, "infeasible": 5, "inexpressible": 5, "control": 4}
    prefixes = {"kind": "k", "infeasible": "i", "inexpressible": "x", "control": "c"}
    for task_id, task in SET.items():
        assert task["id"] == task_id and (lib.KINDS_TASKS_DIR / f"{task_id}.json").is_file()
        assert (REPO / task["world"]).is_file() and task["tags"] and task["prompt"].endswith(".")
        assert task["prompt"].isascii() and "\r" not in read(lib.KINDS_TASKS_DIR / f"{task_id}.json")
        assert re.fullmatch(prefixes[task["group"]] + r"\d\d_[a-z0-9_]+", task_id), task_id


def test_the_ids_are_new_ones_and_no_prompt_is_a_copy_of_a_standing_task():
    """The R3 probes `f_perceive_shelf_fill` and `f_perceive_weed_confidence` stay in set "future" until L4 relabels
    them; this set has new ids and new prompts for the same shapes."""
    elsewhere = {}
    for name in lib.SETS:
        if name != "kinds":
            elsewhere.update(lib.load_set(name))
    assert not set(SET) & set(elsewhere)
    assert not {t["prompt"] for t in SET.values()} & {t["prompt"] for t in elsewhere.values()}
    assert {"f_perceive_shelf_fill", "f_perceive_weed_confidence"} <= set(elsewhere)


def test_labels_are_2_1_in_expect_and_2_2_in_expect_2_2():
    for task_id, task in SET.items():
        later = label_2_2(task)
        assert later["decision"] in ("program", "cannot"), task_id
        checks = {"expected_readings", "expected_comparisons", "expected_limits", "expected_actions", "expected_clock",
                  "expected_message_waits"}
        assert not checks & set(task) or task["group"] == "control", task_id
        if task["group"] == "control":
            assert task["expect"] == {"decision": "program"} and later == {"decision": "program"}, task_id
            assert lib.expected_goals(task) or lib.task_paths(task), task_id
            assert not {"why", "attempt_fails_with", "gap", "blockers"} & set(task)
            continue
        assert task["expect"] == CANNOT and lib.expectation(task) == ("cannot", "inexpressible"), task_id
        assert len(task["why"]) > 40 and task["gap"] in GAPS and GAPS[task["gap"]]["scope"] in ("open", "out")
        assert not {"expected_goal", "expected_goals", "paths", "expect_2_1", "expect_2_0"} & set(task), task_id
        assert lib.task_paths(task) is None, task_id  # the 2.2 paths are read by the 2.2 scorer (L4), not by 2.1's
    for task_id, task in PROGRAMS.items():
        later = label_2_2(task)
        assert later["decision"] == "program" and later["paths"] and later["expected_readings"], task_id
        assert later["expected_comparisons"] and later["world_2_2"]["sensors"], task_id
        assert len({p["when"] for p in later["paths"]}) == len(later["paths"]) >= 2
        assert task["gap"] == "G-PERCEIVE"
    for task_id, task in INFEASIBLE.items():
        later = label_2_2(task)
        assert later["decision"] == "cannot" and later["category"] == "infeasible", task_id
        assert task["blockers"] and task["gap"] == "G-PERCEIVE" and later["because"][0] == "S", task_id
    for task_id, task in INEXPRESSIBLE.items():
        later = label_2_2(task)
        assert {k: later[k] for k in ("decision", "category")} == CANNOT and later["because"] == "0044 K8", task_id


def test_every_kind_has_a_program_and_the_programs_combine_the_features_the_pre_registration_lists():
    kinds = Counter(r[1] for t in PROGRAMS.values() for r in label_2_2(t)["expected_readings"])
    assert set(kinds) == set(NEW_KINDS) | {"label"} and all(kinds[k] >= 1 for k in NEW_KINDS)
    tags = Counter(tag for t in PROGRAMS.values() for tag in t["tags"])
    for tag in ("conversion", "unknown_fails_closed", "region_limit", "declared_action", "clock_time", "message_wait",
                "decoy", "equals", "two_readings_one_sensor", "if_else", "threshold_above", "threshold_below"):
        assert tags[tag] >= 1, tag
    assert len({t["world"] for t in PROGRAMS.values()}) == 3  # stockroom, plant_room, site
    # the 1.7 limit, the 1.8 `do`, the 1.9 clock and the 2.1 message wait that go with a reading
    assert label_2_2(SET["k07_around_the_forklift_lane"])["expected_limits"] == [["no_go_region", "forklift_lane"]]
    assert label_2_2(SET["k08_open_the_relief_valve"])["expected_actions"] == [
        {"robot": "crawler1", "action": "open", "target": "relief_valve_3"}]
    nine = label_2_2(SET["k09_nine_oclock_scan_on_the_signal"])
    assert nine["expected_clock"][0]["value"] == "09:00" and nine["expected_message_waits"] == [
        {"use": "wait", "channel": "stock", "text": "SCAN NOW", "timeout": "5 min"}]
    # confidence goes with a label reading by the same sensor (0044 K4)
    sure = label_2_2(SET["k04_weed_and_ninety_percent_sure"])
    assert [r[1] for r in sure["expected_readings"]] == ["label", "confidence"]
    assert len({r[0] for r in sure["expected_readings"]}) == 1


def test_each_label_names_what_the_request_names_and_the_sensor_is_on_that_robot():
    for task_id, task in PROGRAMS.items():
        later, world = label_2_2(task), base_world(task)
        sensors = {**world["sensors"]}
        robot = later["expected_readings"][0][0]
        assert task["prompt"].startswith(f"With {robot},"), task_id
        assert {r[0] for r in later["expected_readings"]} == {robot}, task_id
        for name, patch in later["world_2_2"]["sensors"].items():
            assert name in sensors, (task_id, name)  # the sensors are the 2.1 world's: 1.7 adds only what they measure
            for kind, measure in patch["measures"].items():
                assert measure.get("contact") is False  # a camera or a tag reader: read from anywhere
                assert (kind in UNITS and measure.get("unit") in (UNITS[kind] or {None})) or kind == "label"
        for _, _, obj in later["expected_readings"]:
            assert obj in world["objects"], (task_id, obj)
        for comparison in later["expected_comparisons"]:
            assert comparison["kind"] in (*NEW_KINDS, "label") and comparison["op"] in ("<", ">", ">=", "==", "<=")
            if comparison["kind"] in UNITS:
                assert comparison["unit"] in UNITS[comparison["kind"]] if UNITS[comparison["kind"]] else (
                    "unit" not in comparison), task_id
        words = re.findall(r"[a-z]+_[a-z0-9_]+|[a-z]+\d", task["prompt"])
        for word in words:
            assert word in world_ids(task) or word in ("tell_no_one",), (task_id, word)


def value_of(task: dict, path: dict, kind: str, obj: str) -> dict:
    patch = path["world_patch"].get("objects", {}).get(obj, {}).get("properties", {})
    base = label_2_2(task)["world_2_2"]["objects"].get(obj, {}).get("properties", {})
    return patch.get(kind) or base.get(kind) or base_world(task)["objects"][obj].get("properties", {})[kind]


def sensor_measure(task: dict, robot: str, kind: str) -> dict:
    world = base_world(task)
    patched = label_2_2(task)["world_2_2"]["sensors"]
    for name, sensor in world["sensors"].items():
        if sensor["robot"] == robot:
            measures = {**sensor.get("measures", {}), **patched.get(name, {}).get("measures", {})}
            if kind in measures:
                return measures[kind]
    raise AssertionError((task["id"], robot, kind))


def compare(task: dict, path: dict, comparison: dict) -> bool | None:
    """One comparison of a label over a path's reading, in three values (0044 K3, 0036 V7): True, False, or None for
    unknown. A reading is its value plus or minus the sensor's uncertainty; a count and a label are exact."""
    kind = comparison["kind"]
    robot, _, obj = next(r for r in label_2_2(task)["expected_readings"] if r[1] == kind)
    reading = value_of(task, path, kind, obj)
    if kind == "label":
        return (reading["value"] == comparison["value"]) == (comparison["op"] == "==")
    if kind == "count":
        assert comparison["op"] == "==" and "." not in reading["value"] and "unit" not in reading
        return Decimal(reading["value"]) == Decimal(comparison["value"])
    measure = sensor_measure(task, robot, kind)
    spread = Decimal(measure["uncertainty"]) * FACTOR[measure["unit"]]
    centre = Decimal(reading["value"]) * FACTOR[reading["unit"]]
    limit = Decimal(comparison["value"]) * FACTOR[comparison["unit"]]
    return compare_intervals(comparison["op"], Interval(centre - spread, centre + spread), Interval(limit, limit))


def all_of(values: list[bool | None]) -> bool | None:
    """`and` in three values: false if any is false, else unknown if any is unknown."""
    return False if False in values else None if None in values else True


@pytest.mark.parametrize("task_id", sorted(set(PROGRAMS) - {"k08_open_the_relief_valve"}))
def test_the_paths_of_a_reading_follow_the_comparison_in_three_values(task_id):
    task = PROGRAMS[task_id]
    for path in label_2_2(task)["paths"]:
        truth = all_of([compare(task, path, c) for c in label_2_2(task)["expected_comparisons"]])
        if task_id == "k09_nine_oclock_scan_on_the_signal" and path.get("fails"):
            truth = None if "unknown" in path["when"] else truth  # the wait times out before the comparison is reached
            assert "times out" in path["when"] and path["end_ms"] == "900000"
            continue
        if truth is None:  # an unknown `if` fails the run (S261)
            assert path.get("fails") and "S261" in path["when"], (task_id, path["when"])
            continue
        assert not path.get("fails"), (task_id, path["when"])
        sent = [text for _, text in path["messages"]]
        expected = [THEN[task_id]] if truth else [ELSE[task_id]] if task_id in ELSE else []
        assert sent == expected, (task_id, path["when"])


def test_the_unit_conversion_and_the_unknown_reading_are_what_the_labels_say():
    kpa = SET["k05_pump_one_below_450_kpa"]
    paths = label_2_2(kpa)["paths"]
    assert [compare(kpa, p, label_2_2(kpa)["expected_comparisons"][0]) for p in paths] == [True, False]
    assert label_2_2(kpa)["world_2_2"]["sensors"]["cam_crawler2"]["measures"]["pressure"]["unit"] == "bar"
    assert "450 kPa" in kpa["prompt"] and "bar" not in kpa["prompt"]  # the sensor and the request use two units
    tank = SET["k06_tank_close_to_the_mark"]
    third = label_2_2(tank)["paths"][2]
    assert third["fails"] and compare(tank, third, label_2_2(tank)["expected_comparisons"][0]) is None
    assert label_2_2(tank)["paths"][0]["messages"] and label_2_2(tank)["paths"][1]["messages"] == []
    count = SET["k03_shelf_three_out_of_stock"]  # a count is exact: 0 == 0 is true, no uncertainty
    assert [compare(count, p, label_2_2(count)["expected_comparisons"][0]) for p in label_2_2(count)["paths"]] == [
        True, False]
    assert "uncertainty" not in label_2_2(count)["world_2_2"]["sensors"]["tag_scan1"]["measures"]["count"]


def test_the_k08_paths_open_the_valve_only_above_six_bar():
    task = SET["k08_open_the_relief_valve"]
    over, under = label_2_2(task)["paths"]
    assert compare(task, over, label_2_2(task)["expected_comparisons"][0]) is True
    assert compare(task, under, label_2_2(task)["expected_comparisons"][0]) is False
    assert ["rl:thing:relief_valve_3", "in_state", "open"] in over["goal"]
    assert ["rl:thing:relief_valve_3", "in_state", "open"] not in under["goal"]


def test_the_confidence_program_has_a_path_for_each_part_of_the_and():
    task = SET["k04_weed_and_ninety_percent_sure"]
    sure_weed, unsure_weed, sure_grass = label_2_2(task)["paths"]
    props = [p["world_patch"]["objects"]["plant_7"]["properties"] for p in (sure_weed, unsure_weed, sure_grass)]
    assert [(q["label"]["value"], q["confidence"]["value"]) for q in props] == [("weed", "95"), ("weed", "80"),
                                                                                  ("grass", "95")]
    assert [p["messages"][0][1] for p in (sure_weed, unsure_weed, sure_grass)] == [
        "weed confirmed", "plant_7 skipped", "plant_7 skipped"]


def test_property_values_follow_world_file_1_7_k5():
    for task_id, task in {**PROGRAMS, **INFEASIBLE}.items():
        later = label_2_2(task)
        patches = [later["world_2_2"]] + [p["world_patch"] for p in later.get("paths", [])]
        for patch in patches:
            for obj, entry in patch.get("objects", {}).items():
                assert obj in base_world(task)["objects"], (task_id, obj)
                for kind, value in entry["properties"].items():
                    if kind in UNITS:
                        assert value.get("unit") in (UNITS[kind] or {None}), (task_id, kind, value)
                        if kind in ("fill", "confidence"):
                            assert 0 <= Decimal(value["value"]) <= 100, (task_id, kind)
                        if kind == "count":
                            assert re.fullmatch(r"\d+", value["value"]), (task_id, value)
                        if kind == "pressure":
                            assert Decimal(value["value"]) > 0


def test_the_k09_times_follow_the_clock_and_the_message_arrives_when_the_wait_is_reached():
    task = SET["k09_nine_oclock_scan_on_the_signal"]
    world = base_world(task)
    assert world["clock"]["start"].endswith("08:50:00")
    to_nine = 10 * 60  # 08:50 to 09:00
    early, late, never = label_2_2(task)["paths"]
    arrival = lambda p: seconds(p["world_patch"]["messages"][0]["at"])  # noqa: E731
    assert arrival(early) < to_nine and early["end_ms"] == str(to_nine * 1000)  # counted when reached (0043 S2)
    assert to_nine < arrival(late) <= to_nine + 5 * 60 and late["end_ms"] == str(int(arrival(late) * 1000))
    assert never["fails"] and never["world_patch"]["messages"] == []
    assert never["end_ms"] == str((to_nine + 5 * 60) * 1000)
    declared = world["channels"]["stock"]["from"]
    assert all(m["from"] in declared for p in (early, late) for m in p["world_patch"]["messages"])
    assert "20 minutes" in task["prompt"] and "5 minutes" in task["prompt"]


def test_the_decoy_sensors_of_k10_each_fail_for_a_different_reason():
    task = SET["k10_the_right_camera"]
    sensors = base_world(task)["sensors"]
    measures = {n: set(s["measures"]) for n, s in label_2_2(task)["world_2_2"]["sensors"].items()}
    assert measures == {"cam_scan1": {"fill"}, "tag_scan1": {"count"}, "cam_scan2": {"fill"}, "tag_scan2": {"count"}}
    mine = {n for n, s in sensors.items() if s["robot"] == "scanbot2"}
    assert mine == {"cam_scan2", "tag_scan2"} and [n for n in mine if "fill" in measures[n]] == ["cam_scan2"]
    assert "cam_scan1" not in mine and "fill" in measures["cam_scan1"]  # S231: right kind, another robot's sensor
    assert measures["tag_scan2"] == {"count"}  # S234: this robot's sensor, another kind
    assert label_2_2(task)["expected_sensors"] == ["cam_scan2"] and "its own camera" in task["prompt"]


def test_the_infeasible_tasks_each_break_one_rule():
    kinds = {t: set(d["tags"]) - {"no_else_path"} for t, d in INFEASIBLE.items()}
    assert kinds == {"i01_the_pickers_camera_reads_nothing": {"fill", "sensor_does_not_measure_the_kind"},
                     "i02_the_camera_is_on_the_other_robot": {"fill", "sensor_elsewhere"},
                     "i03_the_gauge_camera_is_offline": {"pressure", "sensor_unavailable"},
                     "i04_gauge_nine_is_not_there": {"pressure", "object_not_present"},
                     "i05_count_then_carry_without_a_gripper": {"count", "no_gripper"}}
    assert Counter(label_2_2(t)["because"] for t in INFEASIBLE.values()) == {
        "S234": 1, "S231": 1, "S232": 1, "S213": 1, "S2xx (no gripper)": 1}
    for task in INFEASIBLE.values():
        assert "if it is not" not in task["prompt"] and "otherwise" not in task["prompt"]  # no else path
    world = lambda t: base_world(INFEASIBLE[t])  # noqa: E731
    new = lambda t: label_2_2(INFEASIBLE[t])["world_2_2"]["sensors"]  # noqa: E731
    assert world("i01_the_pickers_camera_reads_nothing")["sensors"]["cam_picker"]["robot"] == "picker_s"
    assert new("i01_the_pickers_camera_reads_nothing")["cam_picker"]["measures"] == {}  # the camera measures nothing
    assert world("i02_the_camera_is_on_the_other_robot")["sensors"]["cam_scan2"]["robot"] == "scanbot2"
    assert INFEASIBLE["i02_the_camera_is_on_the_other_robot"]["prompt"].startswith("With scanbot1")
    assert new("i03_the_gauge_camera_is_offline")["cam_crawler3"]["available"] is False
    assert world("i03_the_gauge_camera_is_offline")["sensors"]["cam_crawler3"]["available"] is False
    assert world("i04_gauge_nine_is_not_there")["objects"]["gauge_p9"]["present"] is False
    assert world("i05_count_then_carry_without_a_gripper")["robots"]["scanbot1"]["gripper"] is False


def test_the_inexpressible_tasks_each_stay_out_for_one_reason_of_0044_k8():
    assert {t: d["gap"] for t, d in INEXPRESSIBLE.items()} == {
        "x01_packaging_looks_worn": "G-PERCEIVE", "x02_own_battery_first": "G-SELF",
        "x03_carry_as_many_as_counted": "G-CHOOSE", "x04_average_fill": "G-CALC", "x05_nearest_person": "G-PROX"}
    assert {t: set(d["tags"]) for t, d in INEXPRESSIBLE.items()} == {
        "x01_packaging_looks_worn": {"wear", "free_kind", "if"},
        "x02_own_battery_first": {"battery", "self_state", "if"},
        "x03_carry_as_many_as_counted": {"count", "run_time_choice", "each"},
        "x04_average_fill": {"fill", "arithmetic", "average"},
        "x05_nearest_person": {"person", "distance", "proximity"}}
    assert "looks worn" in SET["x01_packaging_looks_worn"]["prompt"]
    assert "own battery" in SET["x02_own_battery_first"]["prompt"]
    assert "that many" in SET["x03_carry_as_many_as_counted"]["prompt"]
    assert "the average" in SET["x04_average_fill"]["prompt"]
    assert "nearest person" in SET["x05_nearest_person"]["prompt"]


def test_at_most_three_new_world_files_and_the_2_1_worlds_hold_no_new_kind():
    elsewhere = {Path(t["world"]).name for name in lib.SETS if name != "kinds" for t in lib.load_set(name).values()}
    mine = {Path(t["world"]).name for t in SET.values()}
    assert len(mine - elsewhere) <= 3 and mine - elsewhere == NEW_WORLDS
    for name in mine:
        world = json.loads(read(REPO / "bench" / "worlds" / name))
        assert world["world_schema_version"] in ("1.2", "1.3", "1.4", "1.5", "1.6"), name
        text = json.dumps(world)
        for kind in NEW_KINDS:  # the 1.7 world file is L4's: these files are what 2.1 loads
            assert f'"{kind}"' not in text, (name, kind)
        shown = lib.describe_world(world)
        assert not re.search(r"\b(fill|confidence|pressure|count)\b", shown), name  # nothing hints at the new kinds
    for name in NEW_WORLDS:
        world = json.loads(read(REPO / "bench" / "worlds" / name))
        assert len(world["locations"]) + sum(len(world[k]) for k in ("regions", "robots", "objects", "sensors")) >= 20


# -------- the references and the attempts --------

def test_references_and_attempts_are_where_they_should_be():
    names = [p.name for p in REF.iterdir()]
    refs = {n.split(".")[0] for n in names if ".attempt" not in n}
    attempts = {n.split(".")[0] for n in names if ".attempt." in n}
    assert refs == set(CONTROLS) and attempts == set(DECLINED)  # no 2.2 reference yet: that is L4
    for task_id in DECLINED:
        assert (REF / f"{task_id}.attempt.rob").is_file() and (REF / f"{task_id}.attempt.json").is_file()
    for name in names:
        assert "\r" not in read(REF / name)  # Windows-safe: LF only, never compared as CRLF


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_the_obvious_attempt_fails_with_the_recorded_code(task_id):
    task = DECLINED[task_id]
    answer = attempt_answer(task_id)
    assert answer["rob"].startswith(f"// bench/reference_kinds/{task_id}.attempt.rob: the obvious attempt")
    assert answer["rob"].count("robolang 2.1;") == 1 and "\r" not in answer["rob"]
    checked = lib.check_answer(task, answer)
    assert not checked.valid and checked.codes[0] == task["attempt_fails_with"], (task_id, checked.codes)
    assert task["attempt_fails_with"] in ("E101", "E217", "E505")
    if task_id != "x05_nearest_person":  # a kind 2.1 does not know (E217), or the `%` it does not lex (E101)
        assert "E217" in checked.codes or task["attempt_fails_with"] == "E101"


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_no_probe_is_a_bug_in_disguise(task_id):
    """An attempt fails for its new reading: without it, it is a valid 2.1 program in the task's world that ends
    SUCCEEDED. That program is the reading replaced silently, which the old checks cannot tell (the plan's B8)."""
    task = DECLINED[task_id]
    answer = attempt_answer(task_id, strip=True)
    new_reading = re.compile(rf"measure \w+ ({'|'.join(NEW_KINDS)}|wear) ")
    assert "@reading" not in answer["rob"] and not new_reading.search(answer["rob"])
    world = None
    if "attempt_world_patch" in task:  # k09: its clock and message waits are 2.1, so the world must send the message
        world = json.dumps(lib._merge(base_world(task), task["attempt_world_patch"])).encode()
    checked = lib.check_answer(task, answer, world)
    assert checked.valid, (task_id, checked.report)
    assert checked.language_version == "2.1"
    result = lib.simulate_task(checked.task, checked.world)
    assert result.state == "SUCCEEDED", (task_id, result.failure)


def test_the_attempt_codes_are_the_ones_the_plan_counts():
    codes = Counter(t["attempt_fails_with"] for t in DECLINED.values())
    assert codes == {"E101": 9, "E217": 10, "E505": 1}  # `%` is lexed first (E101); E217 follows in those attempts
    percent = {t for t in DECLINED if " %" in read(REF / f"{t}.attempt.rob")}
    assert {t for t, d in DECLINED.items() if d["attempt_fails_with"] == "E101"} == percent


def test_each_attempt_uses_the_one_construct_the_request_needs():
    for task_id in DECLINED:
        rob = read(REF / f"{task_id}.attempt.rob")
        marked = [ln for ln in rob.splitlines() if ln.rstrip().endswith("// @reading")]
        assert marked and any(ln.lstrip().startswith("measure") for ln in marked), task_id
        others = [ln for ln in rob.splitlines()[2:] if ln not in marked]
        assert not any(re.search(r"\bmeasure\b", ln) for ln in others), task_id  # no other reading in the attempt
    text = {t: read(REF / f"{t}.attempt.rob") for t in DECLINED}
    assert "if f < 30 %" in text["k01_restock_under_thirty_percent"]
    assert "p > 6 bar" in text["k02_pump_three_over_six_bar"] and "else" in text["k02_pump_three_over_six_bar"]
    assert "n == 0" in text["k03_shelf_three_out_of_stock"]
    assert "measure r label" in text["k04_weed_and_ninety_percent_sure"]
    assert "measure r confidence" in text["k04_weed_and_ninety_percent_sure"]
    assert "p < 450 kPa" in text["k05_pump_one_below_450_kpa"] and "f > 80 %" in text["k06_tank_close_to_the_mark"]
    assert "@safety no_go_region lane;" in text["k07_around_the_forklift_lane"]
    assert "do r open valve;" in text["k08_open_the_relief_valve"]
    assert "clock_time >= 09:00" in text["k09_nine_oclock_scan_on_the_signal"]
    assert 'message_received "stock" with "SCAN NOW"' in text["k09_nine_oclock_scan_on_the_signal"]
    assert "measure r fill of r" in text["x02_own_battery_first"]  # the robot as the object of its own reading
    assert "object person" in text["x05_nearest_person"]


def test_the_stripped_attempts_keep_the_rest_of_the_request():
    text = {t: strip_reading(read(REF / f"{t}.attempt.rob"), {})[0] for t in DECLINED}
    assert "@safety no_go_region lane;" in text["k07_around_the_forklift_lane"]
    assert "wait until clock_time >= 09:00" in text["k09_nine_oclock_scan_on_the_signal"]
    assert "wait until message_received" in text["k09_nine_oclock_scan_on_the_signal"]
    assert "move r to pump;" in text["k08_open_the_relief_valve"] and "do r" not in text["k08_open_the_relief_valve"]
    assert "communicate" not in text["k01_restock_under_thirty_percent"]
    assert "move r to charger;" not in text["x02_own_battery_first"]  # the battery branch went with its reading
    assert "move r to pump;" in text["x02_own_battery_first"]


def test_the_attempt_binds_the_entities_the_request_names():
    for task_id, task in DECLINED.items():
        bindings = json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"]
        if task_id == "x05_nearest_person":
            assert bindings["person"] == "person_1" and "person_1" not in world_ids(task)  # nothing to bind (G-PROX)
            continue
        assert bindings and all(v in world_ids(task) for v in bindings.values()), task_id
        stated = {w for w in re.findall(r"\b[a-z]+[a-z_]*\d*\b", task["prompt"]) if w in world_ids(task)}
        assert stated <= set(bindings.values()) | {"charging", "aisle_1"}, (task_id, stated - set(bindings.values()))


# -------- the controls --------

@pytest.mark.parametrize("task_id", sorted(CONTROLS))
def test_the_control_reference_is_valid_exports_an_accepted_goal_and_succeeds_on_every_path(task_id):
    task = CONTROLS[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 2.1;") == 1 and answer["rob"].isascii()
    assert not re.search(rf"\b({'|'.join(NEW_KINDS)})\b", answer["rob"]) and "@reading" not in answer["rob"]
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == "2.1"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert not judgement.half_program and judgement.silent_substitution in (None, False)
    assert all(p["goal_ok"] and p["plan_ok"] and p["value_ok"] in (None, True) for p in judgement.paths or [])
    assert (judgement.paths is not None) == (task_id != "c01_carry_a_carton")


def test_the_controls_are_a_carry_and_the_three_readings_of_1_6_and_want_no_new_kind():
    assert [lib.task_paths(c) is not None for c in CONTROLS.values()] == [False, True, True, True]
    assert SET["c02_probe_the_tank"]["expected_readings"] == [["crawler1", "temperature", "tank_1"]]
    assert SET["c03_weigh_a_carton"]["expected_readings"] == [["picker_s", "mass", "carton_2"]]
    assert SET["c04_label_the_plant"]["expected_readings"] == [["weeder1", "label", "plant_7"]]
    # words a model may over-read as a new kind: "tank", "weigh", "label"; no unit of a new kind, no "full" or "sure"
    for control in CONTROLS.values():
        assert not re.search(r"\b(percent|full|sure|confident|bar|kPa|count)\b", control["prompt"]), control["id"]
    assert "tank_1" in SET["c02_probe_the_tank"]["prompt"] and "degC" in SET["c02_probe_the_tank"]["prompt"]
    assert "cam_weeder1" in SET["c04_label_the_plant"]["prompt"]
    assert "confidence" not in SET["c04_label_the_plant"]["prompt"]


# -------- erratum K6: the stale labels of the "ask" set --------

@pytest.mark.parametrize("task_id", ["u01_if_dirty", "u02_if_damaged"])
def test_erratum_k6_relabels_the_two_ask_tasks_as_infeasible_and_keeps_the_old_label(task_id):
    task = lib.load_set("ask")[task_id]
    assert task["group"] == "infeasible" and lib.expectation(task) == ("cannot", "infeasible")
    assert task["expect_1_3"] == CANNOT and "conditional statement" in task["why_1_3"]  # the history
    assert task["attempt_fails_with"] == "S234" and task["blockers"] and "K6" in task["erratum"]
    sensor = task["blockers"][0]
    assert not base_world(task)["sensors"][sensor].get("measures")  # the camera measures nothing: no label sensor
    assert not any("label" in s.get("measures", {}) for s in base_world(task)["sensors"].values())
    assert not any(task_id in gap["tasks"] for gap in ATLAS["gaps"])  # infeasible tasks are in no gap


# -------- the atlas --------

def test_every_task_2_1_declines_is_in_exactly_one_atlas_gap_and_no_control_is():
    in_gaps = [(t, g["id"]) for g in ATLAS["gaps"] for t in g["tasks"] if t in SET]
    assert len(in_gaps) == len({t for t, _ in in_gaps}) == len(DECLINED) == 20
    assert {t for t, _ in in_gaps} == set(DECLINED)
    for task_id, gap in in_gaps:
        assert DECLINED[task_id]["gap"] == gap and GAPS[gap]["scope"] in ("open", "out")
    assert Counter(gap for _, gap in in_gaps) == {
        "G-PERCEIVE": 16, "G-SELF": 1, "G-CHOOSE": 1, "G-CALC": 1, "G-PROX": 1}
    assert {"f_perceive_shelf_fill", "f_perceive_weed_confidence"} <= set(GAPS["G-PERCEIVE"]["tasks"])
    assert len(GAPS["G-PERCEIVE"]["tasks"]) == 18
    assert "I6 L3: set 11 kinds pre-registered (24 tasks)" in [h["note"] for h in ATLAS["history"]]


# -------- the scorer and the workflow --------

def kinds_run(tmp_path: Path, replies: dict) -> Path:
    def script(request):
        first = request["messages"][0]["content"]
        task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
        return message(replies[task_id], model=request["model"])

    out_dir = tmp_path / "kinds"
    run.run_benchmark(FakeClient(script), ["claude-sonnet-5-5"], {t: SET[t] for t in replies}, "medium", 1, out_dir,
                      lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="kinds", prompt_name="full")
    return out_dir


def test_the_scorer_counts_a_silent_replacement_as_false_and_a_decline_of_a_control_as_a_refusal(tmp_path):
    replies = {
        "k01_restock_under_thirty_percent": decline(),                                       # right under 2.1
        "i01_the_pickers_camera_reads_nothing": decline("infeasible"),                      # category differs
        "k06_tank_close_to_the_mark": attempt_answer("k06_tank_close_to_the_mark", strip=True),  # reading dropped
        "c01_carry_a_carton": reference("c01_carry_a_carton"),                                # right
        "c03_weigh_a_carton": decline(),                                                     # a control refused
    }
    out_dir = kinds_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "kinds" and meta["prompt"] == "full"
    assert by["k01_restock_under_thirty_percent"]["decision_correct"]
    assert by["k01_restock_under_thirty_percent"]["category_correct"]
    declined = by["i01_the_pickers_camera_reads_nothing"]
    assert declined["decision_correct"] and declined["category_correct"] is False
    silent = by["k06_tank_close_to_the_mark"]
    assert silent["false_program"] and not silent["decision_correct"]
    assert by["c01_carry_a_carton"]["decision_correct"] and by["c01_carry_a_carton"]["goal_correct"]
    assert by["c03_weigh_a_carton"]["false_refusal"]
    text = score.score_dir(out_dir)
    assert "Task set kinds, prompt full." in text
    kinds = text.split("By kind of task:")[1]
    for kind in ("kind", "infeasible", "control"):
        assert f"| {kind} |" in kinds


def test_the_set_is_in_the_run_script_and_only_runs_on_demand_in_the_workflow():
    assert lib.SETS["kinds"] == (lib.KINDS_TASKS_DIR, lib.KINDS_REFERENCE_DIR)
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    assert workflow.count("--set kinds") == 2
    lines = [ln for ln in workflow.splitlines() if "--set kinds" in ln]
    for part in ("--repeats 3", "--models claude-sonnet-5-5,claude-haiku-5-5", "--max-usd 3"):
        assert all(part in ln for ln in lines), part
    assert any("--prompt full" in ln and "--run-id kinds" in ln for ln in lines)
    assert any("--prompt minimal" in ln and "--run-id kinds-minimal" in ln for ln in lines)
    assert all("claude-opus" not in ln and "--set kinds --prompt" in ln for ln in lines)
    steps = re.split(r"\n      - ", workflow)
    step = next(s for s in steps if s.startswith("name: Run (set kinds"))
    assert "inputs.set_only == 'kinds'" in step and "pull_request" not in step
    pr_step = next(s for s in steps if s.startswith("name: Run (pull request)"))
    weekly = next(s for s in steps if s.startswith("name: Run (weekly or on demand)"))
    assert "kinds" not in pr_step and "inputs.set_only == ''" in weekly and "--set kinds" not in weekly
    assert "kinds kinds-minimal" in workflow


def test_the_set_only_input_lists_kinds_and_keeps_the_input_count():
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    block = workflow.split("  workflow_dispatch:\n    inputs:\n")[1].split("\npermissions:")[0]
    inputs = re.findall(r"^      ([a-z_0-9]+):$", block, re.M)
    assert inputs == ["set_only", "confirm_tasks", "confirm_set", "confirm_models", "confirm_prompt"]
    steps = re.split(r"\n      - ", workflow)
    for value in ("set3", "set4", "future", "regions", "actions", "clock", "missions", "messages", "kinds"):
        assert sum(f"inputs.set_only == '{value}'" in s for s in steps) == 1, value
    assert "messages or kinds. Empty" in block


def test_the_readme_the_errata_and_the_plan_name_the_set():
    readme = read(REPO / "bench" / "README.md")
    assert "## Set 11: more measured kinds (`--set kinds`)" in readme and "set_only: kinds" in readme
    plan = read(REPO / "docs" / "plans" / "bench-11-kinds.md")
    assert "draft for the owner" in plan[:700]
    for task_id in SET:
        assert f"`{task_id}`" in plan, task_id
    for bound in ("B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8", "B9"):
        assert f"| {bound} |" in plan, bound
    errata = read(REPO / "docs" / "evidence" / "bench-errata-2-2.md")
    for task_id in ("u01_if_dirty", "u02_if_damaged"):
        assert f"`{task_id}`" in errata
    assert "Erratum P1" in errata and "Erratum P2" in errata and "bench-freeze-v2.json" in errata

