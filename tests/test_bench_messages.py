"""
Benchmark set 10, "messages" (bench/tasks_messages/, decision record 0043, plan docs/plans/bench-10-messages.md):
incoming messages for RoboLang 2.1 (a condition `message_received "CH" with "TEXT"` in `pre`, `if`, `wait until` and a
mission's `until`; channels and their allowed senders declared in the world), pre-registered against RoboLang 2.0 before
any code (L3 of iteration I5) and labelled for 2.1 at L4. `expect` is the 2.1 label (the scored one), `expect_2_0` the
2.0 label (the baseline: every non-control task declines), and `expect_2_1` keeps the pre-registered 2.1 label: the
paths of a program (the `messages` events that force each path), `world_2_1` (the declared `channels` and the
scheduled `messages`) and a mission's `expected_mission`. The checks a label asks for (`expected_message_waits`, and a
message program's `expected_clock`, `expected_limits`, `expected_actions`) are task keys, as in sets 6 to 8. The world
file 1.6 copy of each base world is `bench/worlds/msg_<id>.json`.

10 `message` requests that are programs in 2.1, `x05` (`message_goal`, a program by C14: the message part is a final
bounded wait), 5 `infeasible` in 2.1, 4 `inexpressible` that stay out (0043 S3, S7, S10; G-TRIGGER, G-CALC,
G-DELIVERY), 4 `control` requests that are programs in both versions (a plain move, a send, a bounded wait for a
region, a fixed wait: no message wait). 2.0 declines every non-control task, and every one has an obvious attempt that
2.0 rejects (`E211`, `E201`); those attempts stay as the evidence of the baseline. The 2.1 references are the programs
(and the controls). No network and no SDK; the model run is the `Benchmark` workflow with `set_only: messages`.
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
SET = lib.load_set("messages")
REF = lib.MESSAGES_REFERENCE_DIR
ATLAS = json.loads((REPO / "bench" / "atlas.json").read_text(encoding="utf-8"))
GAPS = {g["id"]: g for g in ATLAS["gaps"]}
CANNOT = {"decision": "cannot", "category": "inexpressible"}
PROGRAMS = {t: d for t, d in SET.items() if d["group"] == "message"}
GOAL = {t: d for t, d in SET.items() if d["group"] == "message_goal"}  # x05: a program by C14
INFEASIBLE = {t: d for t, d in SET.items() if d["group"] == "infeasible"}
INEXPRESSIBLE = {t: d for t, d in SET.items() if d["group"] == "inexpressible"}
CONTROLS = {t: d for t, d in SET.items() if d["group"] == "control"}
DECLINED = {**PROGRAMS, **GOAL, **INFEASIBLE, **INEXPRESSIBLE}  # what 2.0 declines: an attempt each
BASE_WORLDS = {  # the 2.0 world file each msg_<id>.json is a copy of
    "c01_plain_carry": "warehouse", "c02_tell_the_ops": "warehouse", "c03_wait_for_the_worker_lane": "fulfilment",
    "c04_wait_twenty_seconds_for_the_lab": "ward", "i01_a_channel_the_world_does_not_have": "pharmacy",
    "i02_the_wrong_sender": "fulfilment", "i03_thirty_seconds_for_a_late_message": "site",
    "i04_no_message_service_in_the_ward": "ward", "i05_only_a_near_miss_arrives": "assembly",
    "m01_place_when_the_tugger_docks": "fulfilment", "m02_confirm_then_go_back": "ward",
    "m03_second_box_if_asked": "warehouse", "m04_rounds_until_shift_end": "yard",
    "m05_release_after_the_clock_wait": "warehouse_1_5_0915", "m06_any_message_from_customs": "yard",
    "m07_lobby_when_it_opens": "site_1_3", "m08_open_the_door_when_unlocked": "ward_1_4",
    "m09_go_and_its_look_alikes": "pharmacy", "m10_door_open_and_lane_clear": "fulfilment",
    "x01_make_sure_it_was_delivered": "ward", "x02_each_time_the_line_says_ready": "fulfilment",
    "x03_forward_the_order_number": "fulfilment", "x04_any_message_containing_ready": "fulfilment",
    "x05_a_message_in_the_goal": "fulfilment"}
SECONDS = {"ms": 0.001, "s": 1, "min": 60}


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def reference(task_id: str) -> dict:
    ref = json.loads(read(REF / f"{task_id}.json"))
    rob = read(REF / f"{task_id}.rob")
    return {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob, **ref}


def strip_message_part(rob: str) -> str:
    """The attempt without its message part: the lines marked `// @message` go; a line marked `// @message-tail` loses
    its ` else ...` or ` until ...` tail (a handler on `communicate`, the `until` of a mission)."""
    kept = []
    for line in rob.splitlines(keepends=True):
        text = line.rstrip()
        if text.endswith("// @message"):
            continue
        if text.endswith("// @message-tail"):
            line = re.sub(r"\s+(else|until)\b.*$", "", text.split("//")[0].rstrip()) + ";\n"
        kept.append(line)
    return "".join(kept)


def attempt_answer(task_id: str, strip: bool = False) -> dict:
    stem = REF / f"{task_id}.attempt"
    rob = read(Path(f"{stem}.rob"))
    return {"decision": "program", "category": "none", "reason": "an attempt", "entities": [],
            "rob": strip_message_part(rob) if strip else rob, **json.loads(read(Path(f"{stem}.json")))}


def decline(category: str = "inexpressible") -> dict:
    return {"decision": "cannot", "category": category, "reason": "cannot be done", "rob": "", "task": "",
            "bindings": []}


def world_ids(task: dict) -> set[str]:
    world = json.loads(read(REPO / task["world"]))
    return {*world["robots"], *world["objects"], *world["locations"], *world.get("sensors", {}),
            *world.get("regions", {})}


def seconds(duration: str | dict) -> float:
    if isinstance(duration, dict):
        return float(duration["value"]) * SECONDS[duration["unit"]]
    value, unit = duration.split()
    return float(value) * SECONDS[unit]


def channels(task: dict) -> dict:
    return task["expect_2_1"].get("world_2_1", {}).get("channels", {})


# -------- the set --------

def test_the_mix_of_tasks():
    assert len(SET) == 24
    groups = Counter(d["group"] for d in SET.values())
    assert groups == {"message": 10, "message_goal": 1, "infeasible": 5, "inexpressible": 4, "control": 4}
    for task_id, task in SET.items():
        assert task["id"] == task_id and (lib.MESSAGES_TASKS_DIR / f"{task_id}.json").is_file()
        assert (REPO / task["world"]).is_file() and task["tags"] and task["prompt"].endswith(".")
        assert task["prompt"].isascii() and "\r" not in read(lib.MESSAGES_TASKS_DIR / f"{task_id}.json")
    prefixes = {"message": "m", "message_goal": "x", "infeasible": "i", "inexpressible": "x", "control": "c"}
    for task_id, task in SET.items():
        assert re.fullmatch(prefixes[task["group"]] + r"\d\d_[a-z0-9_]+", task_id), task_id


def test_the_ids_are_new_ones_and_no_prompt_is_a_copy_of_a_standing_task():
    elsewhere = {}
    for name in lib.SETS:
        if name != "messages":
            elsewhere.update(lib.load_set(name))
    assert not set(SET) & set(elsewhere)
    assert not {t["prompt"] for t in SET.values()} & {t["prompt"] for t in elsewhere.values()}


def test_labels_are_2_1_in_expect_and_2_0_in_expect_2_0():
    for task_id, task in SET.items():
        later = task["expect_2_1"]
        assert later["decision"] in ("program", "cannot"), task_id
        assert not {"expected_message_waits", "expected_clock", "expected_limits", "expected_actions"} & set(later)
        if task["group"] == "control":
            assert task["expect"] == {"decision": "program"} and lib.expected_goals(task), task_id
            assert later["decision"] == "program" and task["expected_message_waits"] == [], task_id
            assert task["expect_2_0"]["decision"] == "program"
            assert not {"why", "attempt_fails_with", "gap", "blockers"} & set(task)
            continue
        assert task["expect_2_0"] == CANNOT, task_id  # the baseline label: 2.0 declines every one of them
        assert len(task["why"]) > 40 and task["attempt_fails_with"] in ("E211", "E201"), task_id
        assert task["gap"] in GAPS and GAPS[task["gap"]]["scope"] in ("open", "out", "closed"), task_id
    for task_id, task in PROGRAMS.items():
        later = task["expect_2_1"]
        assert task["expect"] == {"decision": "program"} and lib.expectation(task) == ("program", "none"), task_id
        assert later["decision"] == "program" and task["expected_message_waits"] and later["paths"], task_id
        assert not {"expected_goal", "expected_goals"} & set(task) and task["gap"] == "G-MSG"
        assert task["attempt_fails_with"] == "E211"
        # a task's own paths are read by the scorer, except a mission's, which are read as set 9's are
        assert (lib.task_paths(task) is None) == (task_id == "m04_rounds_until_shift_end")
    for task_id, task in GOAL.items():  # x05: C14 made it a program; erratum N3 declines it again (S1)
        later = task["expect_2_1"]
        assert task["expect"] == CANNOT and later["because"] == "0043 C14" and "N3" in task["erratum"], task_id
        assert [w["use"] for w in task["expected_message_waits"]] == ["wait"] and len(later["paths"]) == 2
        assert task["gap"] == "G-EVENTGOAL" and task["expect_2_0"] == CANNOT
    for task_id, task in INFEASIBLE.items():
        later = task["expect_2_1"]
        assert task["expect"] == {"decision": "cannot", "category": "infeasible"}, task_id
        assert later["decision"] == "cannot" and later["category"] == "infeasible", task_id
        assert later["because"].startswith("0043 S") and task["blockers"] and task["gap"] == "G-MSG", task_id
        assert task["expected_message_waits"] and task["expected_message_waits"][0]["use"] == "wait", task_id
    for task_id, task in INEXPRESSIBLE.items():
        later = task["expect_2_1"]
        assert task["expect"] == CANNOT and {k: later[k] for k in ("decision", "category")} == CANNOT, task_id
        assert later["because"].startswith("0043 S"), task_id
    assert sorted(t["expect_2_1"]["because"] for t in INFEASIBLE.values()) == [
        "0043 S3", "0043 S4", "0043 S4", "0043 S5", "0043 S6"]
    assert sorted(t["expect_2_1"]["because"] for t in INEXPRESSIBLE.values()) == [
        "0043 S10", "0043 S10", "0043 S3", "0043 S7"]


def test_the_message_programs_cover_the_cases_the_pre_registration_lists():
    tags = Counter(tag for t in PROGRAMS.values() for tag in t["tags"])
    for tag in ("wait_until", "timeout", "message_if", "message_until", "latched", "any_message", "region_limit",
                "declared_action", "clock_time", "decoy", "two_channels", "sender_not_allowed", "exact_text",
                "reply_message"):
        assert tags[tag] >= 1, tag
    uses = Counter(w["use"] for t in PROGRAMS.values() for w in t["expected_message_waits"])
    assert uses == {"wait": 9, "if": 1, "until": 1}
    assert len({t["world"] for t in PROGRAMS.values()}) >= 7
    for task in PROGRAMS.values():  # each program has a path that goes through and a path that does not
        paths = task["expect_2_1"]["paths"]
        assert len(paths) >= 2 and len({p["when"] for p in paths}) == len(paths)
    # the 1.7 limit, the 1.8 `do` and the 1.9 clock that go with a message wait
    assert SET["m07_lobby_when_it_opens"]["expected_limits"] == [["no_go_region", "wet_floor"]]
    assert SET["m08_open_the_door_when_unlocked"]["expected_actions"] == [
        {"robot": "moxi1", "action": "open", "target": "closet_door"}]
    assert SET["m05_release_after_the_clock_wait"]["expected_clock"][0]["value"] == "09:20"
    # a mission whose `until` is a message, judged as set 9's missions are
    mission = SET["m04_rounds_until_shift_end"]["expect_2_1"]
    assert mission["expected_mission"]["until"]["condition"] == 'message_received "ops" with "SHIFT END"'
    assert [p["outcome"] for p in mission["paths"]] == ["completed", "bounded", "bounded"]
    assert [p["runs"] for p in mission["paths"]] == [3, 12, 12]


def test_each_label_names_the_channels_and_texts_the_request_names_and_declares_them_in_the_world():
    for task_id, task in PROGRAMS.items():
        declared = channels(task)
        for wait in task["expected_message_waits"]:
            assert f'"{wait["channel"]}"' in task["prompt"], task_id
            assert wait["channel"] in declared and declared[wait["channel"]]["from"], task_id
            if wait["text"] is None:  # any message on the channel: the request fixes no text
                assert task_id == "m06_any_message_from_customs" and not re.search(
                    r'"customs" (has sent|sends) "', task["prompt"])
            else:
                assert f'"{wait["text"]}"' in task["prompt"], task_id
            if wait["timeout"]:
                minutes = seconds(wait["timeout"]) / 60
                words = f"{minutes:g} minutes" if minutes >= 1 else f"{seconds(wait['timeout']):g} seconds"
                assert words in task["prompt"] or "20 seconds" in task["prompt"], (task_id, words)
        for path in task["expect_2_1"]["paths"]:
            for msg in path["world_patch"]["messages"]:
                assert set(msg) == {"at", "channel", "text", "from"} and msg["from"].startswith("rl:agent:"), task_id
                assert msg["channel"] in declared, (task_id, msg)
    for task_id, task in CONTROLS.items():  # the controls' world declares a channel too, so a wait is possible
        assert channels(task) and "message_received" not in read(REF / f"{task_id}.rob")


def mini_run(task_id: str, path: dict) -> tuple[bool, int]:
    """0043 S2-S4 in a few lines, for tasks that wait for messages one after another: a message counts when its
    sender is allowed on its channel and its text is the one waited for (exact, S3) or any; it counts if it arrived at
    or before the moment the wait is reached (latched, S2). Returns (every wait was satisfied, end of the run in ms)."""
    task = PROGRAMS[task_id]
    declared = channels(task)
    now = {"m05_release_after_the_clock_wait": 300.0}.get(task_id, 0.0)  # 09:15 to 09:20
    for wait in task["expected_message_waits"]:
        arrivals = [seconds(m["at"]) for m in path["world_patch"]["messages"]
                    if m["channel"] == wait["channel"] and m["from"] in declared[m["channel"]]["from"]
                    and wait["text"] in (None, m["text"].strip())]
        limit = now + seconds(wait["timeout"])
        if not arrivals or min(arrivals) > limit:
            return False, round(limit * 1000)
        now = max(now, min(arrivals))
    return True, round(now * 1000)


@pytest.mark.parametrize("task_id", sorted(set(PROGRAMS) - {"m03_second_box_if_asked", "m04_rounds_until_shift_end"}))
def test_the_paths_of_a_wait_follow_0043_s2_to_s4(task_id):
    for path in PROGRAMS[task_id]["expect_2_1"]["paths"]:
        reached, end = mini_run(task_id, path)
        assert reached == (not path.get("fails")), (task_id, path["when"])
        assert path["end_ms"] == str(end), (task_id, path["when"])
        assert ("goal" in path) == reached and bool(path.get("fails")) == (not reached)


def test_the_if_and_the_mission_paths_follow_s2():
    # m03: the `if` is reached when the aisle is clear (8 seconds): a message at 5 seconds counts, one at 30 does not
    paths = {p["when"][:34]: p for p in SET["m03_second_box_if_asked"]["expect_2_1"]["paths"]}
    first, none, late, aisle = SET["m03_second_box_if_asked"]["expect_2_1"]["paths"]
    world = json.loads(read(REPO / SET["m03_second_box_if_asked"]["world"]))
    cleared = seconds(world["events"][0]["at"])
    assert cleared == 8 and seconds(first["world_patch"]["messages"][0]["at"]) < cleared < seconds(
        late["world_patch"]["messages"][0]["at"])
    assert len(first["goal"]) == 2 and len(none["goal"]) == len(late["goal"]) == 1 and paths
    assert aisle["fails"] and aisle["world_patch"]["events"] == [] and aisle["end_ms"] == "20000"
    # m04: a round takes 20 seconds, the until is checked after each run: 50 seconds is seen after run 3 (60 seconds)
    wait = "wait for 20 s;"
    assert wait in read(REF / "m04_rounds_until_shift_end.attempt.rob")
    done, never, ignored = SET["m04_rounds_until_shift_end"]["expect_2_1"]["paths"]
    at = seconds(done["world_patch"]["messages"][0]["at"])
    assert 2 * 20 < at <= 3 * 20 and done["runs"] == 3 and never["world_patch"]["messages"] == []
    sender = ignored["world_patch"]["messages"][0]["from"]
    assert sender not in channels(SET["m04_rounds_until_shift_end"])["ops"]["from"]


def test_the_decoys_of_m09_are_near_misses_in_text_and_in_channel():
    task = SET["m09_go_and_its_look_alikes"]
    real, only_decoys = task["expect_2_1"]["paths"]
    decoys = only_decoys["world_patch"]["messages"]
    assert {(m["channel"], m["text"]) for m in decoys} == {
        ("pharmacy_west", "GO BACK"), ("pharmacy_east", "GO"), ("pharmacy_west", "go")}
    assert real["world_patch"]["messages"][:3] == decoys and real["world_patch"]["messages"][3]["text"] == "GO"
    assert {"pharmacy_west", "pharmacy_east"} == set(channels(task))


def test_a_message_that_arrives_before_the_wait_is_counted_s2():
    first = SET["m05_release_after_the_clock_wait"]["expect_2_1"]["paths"][0]
    assert seconds(first["world_patch"]["messages"][0]["at"]) < 300 and first["end_ms"] == "300000"
    world = json.loads(read(REPO / SET["m05_release_after_the_clock_wait"]["world"]))
    assert world["clock"]["start"].endswith("09:15:00")
    both = SET["m10_door_open_and_lane_clear"]["expect_2_1"]["paths"][0]["world_patch"]["messages"]
    assert [m["channel"] for m in both] == ["safety", "dock"]  # the second wait starts with its message already there


def test_the_infeasible_tasks_each_break_one_rule_of_0043():
    kinds = {t: set(d["tags"]) - {"message_wait", "no_else_path"} for t, d in INFEASIBLE.items()}
    assert kinds == {"i01_a_channel_the_world_does_not_have": {"undeclared_channel", "near_miss_channel"},
                     "i02_the_wrong_sender": {"sender_not_allowed"},
                     "i03_thirty_seconds_for_a_late_message": {"timeout_too_short"},
                     "i04_no_message_service_in_the_ward": {"no_message_service", "no_channels"},
                     "i05_only_a_near_miss_arrives": {"near_miss_text", "exact_text", "case_sensitive"}}

    def world_2_1(task_id):
        return INFEASIBLE[task_id]["expect_2_1"]["world_2_1"]

    # S4: the channel the request names is not declared; the world has a location with that name, no channel
    undeclared = INFEASIBLE["i01_a_channel_the_world_does_not_have"]
    assert '"packing"' in undeclared["prompt"] and "packing" not in world_2_1(undeclared["id"])["channels"]
    assert "packing" in json.loads(read(REPO / undeclared["world"]))["locations"]
    # S4: the only message that would release the tote comes from a sender the channel does not allow
    sender = INFEASIBLE["i02_the_wrong_sender"]
    declared = world_2_1(sender["id"])
    assert [m["from"] for m in declared["messages"]] == ["rl:agent:contractor-3"]
    assert declared["messages"][0]["from"] not in declared["channels"]["line"]["from"]
    assert "stop the job" not in sender["prompt"] and "fail" not in sender["prompt"]
    # the message comes after the time the request allows (S6: the world's schedule decides)
    late = INFEASIBLE["i03_thirty_seconds_for_a_late_message"]
    assert "within 30 seconds" in late["prompt"] and seconds(world_2_1(late["id"])["messages"][0]["at"]) == 300
    assert "timeout 30 s" in read(REF / f"{late['id']}.attempt.rob")
    # S5: no channels at all
    assert "channels" not in world_2_1("i04_no_message_service_in_the_ward")
    assert "channels" not in json.loads(read(REPO / INFEASIBLE["i04_no_message_service_in_the_ward"]["world"]))
    # S3: exact and case-sensitive
    near = INFEASIBLE["i05_only_a_near_miss_arrives"]
    texts = [m["text"] for m in world_2_1(near["id"])["messages"]]
    assert texts == ["release", "RELEASE NOW"] and '"RELEASE"' in near["prompt"] and "RELEASE" not in texts


def test_the_inexpressible_tasks_each_stay_out_for_one_reason_of_0043():
    reasons = {t: set(d["tags"]) for t, d in INEXPRESSIBLE.items()}
    assert reasons == {"x01_make_sure_it_was_delivered": {"message", "delivery", "no_handler_on_communicate"},
                       "x02_each_time_the_line_says_ready": {"message_trigger", "mission", "duration_bound",
                                                             "for_each_message"},
                       "x03_forward_the_order_number": {"message_content", "value_from_message", "forward"},
                       "x04_any_message_containing_ready": {"message_wait", "pattern", "contains", "exact_narrowed"}}
    assert set(GOAL["x05_a_message_in_the_goal"]["tags"]) == {"message_in_goal", "end_state"}  # a program by C14
    assert {t: d["gap"] for t, d in INEXPRESSIBLE.items()} == {
        "x01_make_sure_it_was_delivered": "G-DELIVERY", "x02_each_time_the_line_says_ready": "G-TRIGGER",
        "x03_forward_the_order_number": "G-CALC", "x04_any_message_containing_ready": "G-CALC"}
    assert re.search(r"\b\d+ hours\b", SET["x02_each_time_the_line_says_ready"]["prompt"])  # its bound is stated
    assert "contains the word" in SET["x04_any_message_containing_ready"]["prompt"]
    assert "as the goal" in GOAL["x05_a_message_in_the_goal"]["prompt"]
    assert "make sure the message was delivered" in SET["x01_make_sure_it_was_delivered"]["prompt"]


def test_each_world_is_the_world_file_1_6_copy_of_its_2_0_world_with_the_channels_and_messages_of_its_label():
    """World file 1.6 (L4): `msg_<id>.json` is the 2.0 world of the L3 set, with the `channels` and the scheduled
    `messages` of `world_2_1` and nothing else changed; the shown world lists them (C1, C2)."""
    for task_id, task in SET.items():
        assert task["world"] == f"bench/worlds/msg_{task_id}.json" and (REPO / task["world"]).is_file()
        base = json.loads(read(REPO / "bench" / "worlds" / f"{BASE_WORLDS[task_id]}.json"))
        copy = json.loads(read(REPO / task["world"]))
        assert copy.pop("world_schema_version") == "1.6" and base.pop("world_schema_version") in (
            "1.1", "1.2", "1.3", "1.4", "1.5")
        label = task["expect_2_1"]["world_2_1"]
        assert copy.pop("channels", None) == label.get("channels"), task_id
        assert copy.pop("messages", None) == label.get("messages"), task_id
        assert copy == base, task_id
        shown = lib.describe_world(json.loads(read(REPO / task["world"])))
        if "channels" in label:
            assert "channels: " in shown
            assert all(f"{name} (accepts messages from" in shown for name in label["channels"])
        else:
            assert "channels: none (this world has no message service)" in shown  # i04 (S5, C1)
        for scheduled in label.get("messages", []):  # an infeasible request is decided from what the world shows (C2)
            assert (f'a message "{scheduled["text"]}" arrives on channel {scheduled["channel"]} from '
                    f'{scheduled["from"]}' in shown), task_id
        assert "message_received" not in shown and "world_2_1" not in shown
    for task_id, task in PROGRAMS.items():  # a program's messages come from its paths, never from the world shown
        assert "messages" not in json.loads(read(REPO / task["world"])), task_id


# -------- the references and the attempts --------

def test_references_and_attempts_are_where_they_should_be():
    names = [p.name for p in REF.iterdir()]
    refs = {n.split(".")[0] for n in names if ".attempt" not in n}
    attempts = {n.split(".")[0] for n in names if ".attempt." in n}
    programs = set(CONTROLS) | set(PROGRAMS) | set(GOAL)
    assert refs == programs and attempts == set(DECLINED)  # a 2.1 reference for each program; the 2.0 attempts stay
    for task_id in DECLINED:
        assert (REF / f"{task_id}.attempt.rob").is_file() and (REF / f"{task_id}.attempt.json").is_file()
    for name in names:
        assert "\r" not in read(REF / name)  # Windows-safe: LF only, never compared as CRLF


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_the_obvious_attempt_fails_with_the_recorded_code(task_id):
    task = DECLINED[task_id]
    answer = attempt_answer(task_id)
    assert answer["rob"].startswith(f"// bench/reference_messages/{task_id}.attempt.rob: the obvious attempt")
    assert answer["rob"].count("robolang 2.0;") == 1 and "\r" not in answer["rob"]
    checked = lib.check_answer(task, answer)
    assert not checked.valid and checked.codes[0] == task["attempt_fails_with"], (task_id, checked.codes)


def test_each_attempt_uses_the_one_construct_the_request_needs():
    for task_id in DECLINED:
        rob = read(REF / f"{task_id}.attempt.rob")
        marked = [ln for ln in rob.splitlines() if ln.rstrip().endswith(("// @message", "// @message-tail"))]
        assert marked, task_id
        others = [ln for ln in rob.splitlines()[2:] if ln not in marked]
        assert not any("message" in ln for ln in others), task_id  # nothing else in the attempt is about a message
    text = {t: read(REF / f"{t}.attempt.rob") for t in DECLINED}
    assert 'message_received "line" with "TUGGER DOCKED"' in text["m01_place_when_the_tugger_docks"]
    assert 'if message_received "ops" with "ALSO BOX2"' in text["m03_second_box_if_asked"]
    assert '} until message_received "ops" with "SHIFT END";' in text["m04_rounds_until_shift_end"]
    assert 'message_received "customs" timeout' in text["m06_any_message_from_customs"]  # no text: any message
    assert "@safety no_go_region wet;" in text["m07_lobby_when_it_opens"]
    assert "do r open door;" in text["m08_open_the_door_when_unlocked"]
    assert "wait until clock_time >= 09:20" in text["m05_release_after_the_clock_wait"]
    assert text["m10_door_open_and_lane_clear"].count("message_received") == 2
    assert 'communicate r to "nurse" with "MEDS READY" else' in text["x01_make_sure_it_was_delivered"]
    assert "repeat when message_received" in text["x02_each_time_the_line_says_ready"]
    assert "as n" in text["x03_forward_the_order_number"] and 'with n;' in text["x03_forward_the_order_number"]
    assert 'containing "READY"' in text["x04_any_message_containing_ready"]
    assert 'goal {\n        object_at tote dst;\n        message_received "line"' in text["x05_a_message_in_the_goal"]


@pytest.mark.parametrize("task_id", sorted(DECLINED))
def test_no_probe_is_a_bug_in_disguise(task_id):
    """An attempt fails for its message part: without it, it is a valid 2.0 program in the task's world that ends
    SUCCEEDED (a mission ends `bounded` or `completed`). That program is the message wait replaced silently: the wait
    dropped, which the old checks cannot tell, and what the baseline counts as a false program (the plan's B8)."""
    answer = attempt_answer(task_id, strip=True)
    assert "message_received" not in answer["rob"] and "@message" not in answer["rob"]
    checked = lib.check_answer(DECLINED[task_id], answer)
    assert checked.valid, (task_id, checked.report)
    assert checked.language_version == "2.0"
    judgement = lib.judge(dict(DECLINED[task_id], goal_free=True), checked, answer["bindings"])
    assert judgement.state in ("SUCCEEDED", "bounded"), (task_id, judgement.state, judgement.failure_code)
    if DECLINED[task_id].get("expected_message_waits"):  # B8 (programs) and the B4 clause (infeasible): replaced
        assert judgement.message_wait_replaced is True, task_id


def test_the_stripped_attempts_keep_the_rest_of_the_request():
    """What is left after the message part goes is what the request also asks: the limit, the `do`, the clock."""
    text = {t: strip_message_part(read(REF / f"{t}.attempt.rob")) for t in DECLINED}
    assert "wait until clock_time >= 09:20" in text["m05_release_after_the_clock_wait"]
    assert "@safety no_go_region wet;" in text["m07_lobby_when_it_opens"]
    assert "do r open door;" in text["m08_open_the_door_when_unlocked"]
    assert "wait until region_clear lane" in text["m03_second_box_if_asked"] and "box2" not in text[
        "m03_second_box_if_asked"].split("plan")[1]
    assert "mission rounds" in text["m04_rounds_until_shift_end"] and " until " not in text[
        "m04_rounds_until_shift_end"].split("mission")[1] and "};" in text["m04_rounds_until_shift_end"]
    assert 'communicate r to "nurse" with "MEDS READY";' in text["x01_make_sure_it_was_delivered"]
    assert 'communicate r to "alerts"' not in text["m02_confirm_then_go_back"]


def test_the_attempt_binds_the_entities_the_request_names():
    for task_id, task in DECLINED.items():
        bindings = json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"]
        assert bindings and all(v in world_ids(task) for v in bindings.values()), task_id
    for task_id, task in PROGRAMS.items():
        bindings = json.loads(read(REF / f"{task_id}.attempt.json"))["bindings"]
        stated = {w for w in re.findall(r"\b[a-z]+[a-z_]*\d*\b", task["prompt"]) if w in world_ids(task)}
        assert stated <= set(bindings.values()) | {"aisle", "dock_2"}, task_id  # cleaner1 starts at dock_2


# -------- the controls --------

@pytest.mark.parametrize("task_id", sorted(CONTROLS))
def test_the_control_reference_is_valid_exports_an_accepted_goal_and_succeeds_on_every_path(task_id):
    task = CONTROLS[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 2.0;") == 1 and answer["rob"].isascii()
    assert "message_received" not in answer["rob"] and "// @message" not in answer["rob"]
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == "2.0"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.state, judgement.failure_code)
    assert not judgement.half_program and not judgement.checks or all(judgement.checks.values())


def test_the_controls_are_a_plain_move_a_send_a_region_wait_and_a_fixed_wait_and_want_no_message_wait():
    assert [lib.task_paths(c) is not None for c in CONTROLS.values()] == [False, True, True, False]
    assert "communicate r to \"ops\" with" in read(REF / "c02_tell_the_ops.rob")  # a send, not a wait for a message
    assert "wait until region_clear lane timeout 30 s" in read(REF / "c03_wait_for_the_worker_lane.rob")
    assert "wait for 20 s;" in read(REF / "c04_wait_twenty_seconds_for_the_lab.rob")
    assert CONTROLS["c04_wait_twenty_seconds_for_the_lab"]["expected_end_ms"] == "20000"
    # the words a model may over-read as a message wait: "tell channel", "wait", "ready", "stop the job as failed"
    assert 'tell channel "ops"' in CONTROLS["c02_tell_the_ops"]["prompt"]
    assert "Wait up to 30 seconds" in CONTROLS["c03_wait_for_the_worker_lane"]["prompt"]
    assert "ready" in CONTROLS["c04_wait_twenty_seconds_for_the_lab"]["prompt"]
    assert not any(re.search(r"\b(message|sends?|confirms?|signal)\b", c["prompt"]) for c in CONTROLS.values())


# -------- the standing message tasks, relabelled for 2.1 --------

STANDING = {  # id -> (set, reference directory, the channels the world declares)
    "f_msg_line_docked": ("future", {"line": ["rl:agent:line-ctl"]}),
    "f_msg_nurse_confirm": ("future", {"nurse": ["rl:agent:nurse-1"]}),
    "f_msg_wind_clear": ("future", {"site": ["rl:agent:site-ctl"]}),
    "x_wait_message": ("hard", {"handoff": ["rl:robot:amr1"]}),
    "x_wait_ready_message": ("branch", {"handoff": ["rl:robot:amr1"]}),
    "u11_wait_go": ("ask", {"pharmacy": ["rl:agent:pharmacist-w"]}),
    "u12_radio_clearance": ("ask", {"port_authority": ["rl:agent:port-authority"]}),
}


@pytest.mark.parametrize("task_id", sorted(STANDING))
def test_a_standing_message_task_is_a_2_1_program_with_a_world_file_1_6_a_reference_and_two_paths(task_id):
    set_name, declared = STANDING[task_id]
    task = lib.load_set(set_name)[task_id]
    assert task["group"] == "message" and task["expect"] == {"decision": "program"} and lib.expectation(task) == (
        "program", "none")
    assert task["expect_2_0"] == CANNOT  # the label until L4 stays as the 2.0 label
    assert task["world"] == f"bench/worlds/msg_{task_id}.json"
    world = json.loads(read(REPO / task["world"]))
    assert world["world_schema_version"] == "1.6" and {c: v["from"] for c, v in world["channels"].items()} == declared
    paths = task["expect_2_1"]["paths"]
    assert len(paths) == 2 and sum(1 for p in paths if p.get("fails")) == 1
    assert all(m["from"] in declared[m["channel"]] for p in paths for m in p["world_patch"]["messages"])
    wait = task["expected_message_waits"]
    assert len(wait) == 1 and wait[0]["use"] == "wait" and wait[0]["channel"] in declared and wait[0]["timeout"] is None
    assert task_id not in {t for g in ATLAS["gaps"] for t in g["tasks"]}  # left the atlas at L4
    ref_dir = lib.SETS[set_name][1]
    rob = read(ref_dir / f"{task_id}.rob")
    assert rob.splitlines()[1] == "robolang 2.1;" and "message_received" in rob and rob.isascii()
    answer = {"decision": "program", "category": "none", "reason": "-", "entities": [], "rob": rob,
              **json.loads(read(ref_dir / f"{task_id}.json"))}
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds and judgement.message_ok is True
    assert judgement.message_wait_replaced is False and all(judgement.checks.values())
    assert (ref_dir / f"{task_id}.attempt.rob").is_file()  # the 1.x attempt, the evidence of the R3 probe, stays


def test_the_standing_tasks_that_name_no_text_or_no_channel_follow_c9():
    nurse = lib.load_set("future")["f_msg_nurse_confirm"]
    assert nurse["expected_message_waits"][0]["text"] is None  # "until the nurse confirms": any message on the channel
    assert '"nurse"' in nurse["prompt"] and "confirms" in nurse["prompt"]
    radio = lib.load_set("ask")["u12_radio_clearance"]
    assert radio["expected_message_waits"][0] == {"use": "wait", "channel": "port_authority", "text": None,
                                                  "timeout": None}
    world = json.loads(read(REPO / radio["world"]))
    assert list(world["channels"]) == ["port_authority"]  # no channel named: the one the world declares for that party
    assert "port authority" in radio["prompt"] and "channel" not in radio["prompt"]
    # the sender may be a robot of the world (C6): "until amr1 sends READY"
    for set_name, task_id in (("hard", "x_wait_message"), ("branch", "x_wait_ready_message")):
        task = lib.load_set(set_name)[task_id]
        assert json.loads(read(REPO / task["world"]))["channels"] == {"handoff": {"from": ["rl:robot:amr1"]}}


# -------- the atlas --------

def test_only_what_2_1_still_declines_is_in_an_atlas_gap_and_g_msg_is_closed():
    in_gaps = [(t, g["id"]) for g in ATLAS["gaps"] for t in g["tasks"] if t in SET]
    declined = {**INEXPRESSIBLE, **GOAL}  # x05 is declined again (erratum N3)
    assert {t for t, _ in in_gaps} == set(declined) and len(in_gaps) == len(declined)
    for task_id, gap in in_gaps:
        assert declined[task_id]["gap"] == gap and GAPS[gap]["scope"] in ("open", "out")
    assert Counter(gap for _, gap in in_gaps) == {"G-TRIGGER": 1, "G-CALC": 2, "G-DELIVERY": 1, "G-EVENTGOAL": 1}
    assert GAPS["G-MSG"]["scope"] == "closed" and GAPS["G-MSG"]["closed_in"] == "RoboLang 2.1"  # L6 of I5
    assert GAPS["G-DELIVERY"]["direction"] == "communication" and GAPS["G-DELIVERY"]["scope"] == "out"  # C13
    assert set(GAPS["G-DELIVERY"]["tasks"]) == {"x01_make_sure_it_was_delivered", "f12_message_delivery"}
    notes = [h["note"] for h in ATLAS["history"]]
    assert "I5 L3: set 10 messages pre-registered (24 tasks)" in notes
    assert "I5 L4: RoboLang 2.1 built and taught" in notes
    # the tasks that were in G-MSG before are programs now, and still name the gap (as set 9's tasks name G-LOOP)
    for task_id, task in {**PROGRAMS, **INFEASIBLE}.items():
        assert task["gap"] == "G-MSG" and task_id not in GAPS["G-MSG"]["tasks"]


# -------- the scorer and the workflow --------

def messages_run(tmp_path: Path, replies: dict) -> Path:
    def script(request):
        first = request["messages"][0]["content"]
        task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
        return message(replies[task_id], model=request["model"])

    out_dir = tmp_path / "messages"
    run.run_benchmark(FakeClient(script), ["claude-sonnet-5-5"], {t: SET[t] for t in replies}, "medium", 1, out_dir,
                      lib.load_prompt("full"), workers=1, log=io.StringIO(), task_set="messages", prompt_name="full")
    return out_dir


def changed(task_id: str, old: str, new: str) -> dict:
    answer = reference(task_id)
    assert old in answer["rob"], (task_id, old)
    answer["rob"] = answer["rob"].replace(old, new).replace("robolang 2.0;", "robolang 2.1;")  # a control is 2.0
    return answer


def judged(task_id: str, answer: dict):
    task = SET[task_id]
    checked = lib.check_answer(task, answer)
    assert checked.valid, (task_id, checked.report)
    return lib.judge(task, checked, answer["bindings"])


@pytest.mark.parametrize("task_id", sorted(PROGRAMS))  # x05's reference stays as C14's evidence (erratum N3)
def test_the_reference_of_a_message_program_passes_every_check_on_every_path(task_id):
    task = SET[task_id]
    answer = reference(task_id)
    assert answer["rob"].count("robolang 2.1;") == 1 and answer["rob"].isascii()
    checked = lib.check_answer(task, answer)
    assert checked.valid, checked.report
    assert checked.language_version == "2.1"
    judgement = lib.judge(task, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds, (judgement.paths, judgement.state)
    assert judgement.message_ok is True and judgement.checks["message_waits"] is True and all(
        judgement.checks.values())
    assert judgement.message_wait_replaced is False and judgement.channel_or_text_changed is False
    assert judgement.message_added is None
    assert not judgement.half_program and (judgement.paths is None or all(p["plan_ok"] for p in judgement.paths))


def test_message_ok_needs_the_channel_the_text_the_place_and_the_timeout_asked():
    assert judged("m01_place_when_the_tugger_docks", reference("m01_place_when_the_tugger_docks")).message_ok
    for old, new in (('"TUGGER DOCKED"', '"tugger docked"'), ('"line"', '"ops"'), ('timeout 10 min', 'timeout 5 min'),
                     ('with "TUGGER DOCKED" ', '')):
        answer = changed("m01_place_when_the_tugger_docks", old, new)
        if old == '"line"':
            continue  # an undeclared channel is a world error (S011), covered below
        assert judged("m01_place_when_the_tugger_docks", answer).message_ok is False, (old, new)
    # a space at either end of the text is trimmed by the match, so it is the same condition (C11)
    spaced = changed("m01_place_when_the_tugger_docks", '"TUGGER DOCKED"', '" TUGGER DOCKED "')
    assert judged("m01_place_when_the_tugger_docks", spaced).message_ok is True
    # a wait with no timeout in the label (the request gives none) takes any timeout
    assert judged("x05_a_message_in_the_goal", changed(
        "x05_a_message_in_the_goal", "timeout 10 min", "timeout 3 min")).message_ok is True
    # two waits are one `and` of two atoms, each with the timeout of its wait (m10)
    one = reference("m10_door_open_and_lane_clear")
    one["rob"] = one["rob"].replace(
        'wait until message_received "dock" with "DOOR OPEN" timeout 10 min else { fail "the dock door did not '
        'open"; }\n'
        '        wait until message_received "safety" with "LANE CLEAR" timeout 10 min else { fail "the lane was not '
        'cleared"; }',
        'wait until message_received "dock" with "DOOR OPEN" and message_received "safety" with "LANE CLEAR" '
        'timeout 10 min else { fail "not released"; }')
    assert "and message_received" in one["rob"]
    assert judged("m10_door_open_and_lane_clear", one).message_ok is True


def test_a_message_wait_replaced_silently_is_a_valid_ending_program_with_no_message_condition_b8():
    stripped = attempt_answer("m01_place_when_the_tugger_docks", strip=True)
    judgement = judged("m01_place_when_the_tugger_docks", stripped)
    assert judgement.state == "SUCCEEDED" and judgement.message_wait_replaced is True and not judgement.message_ok
    guessed = changed("m01_place_when_the_tugger_docks", WAIT_M01, 'wait for 90 s;')
    assert judged("m01_place_when_the_tugger_docks", guessed).message_wait_replaced is True  # a guessed wait for
    stand_in = changed("m03_second_box_if_asked", 'message_received "ops" with "ALSO BOX2"', "region_clear lane")
    assert judged("m03_second_box_if_asked", stand_in).message_wait_replaced is True  # a region_clear stand-in
    assert judged("m01_place_when_the_tugger_docks", reference("m01_place_when_the_tugger_docks")
                  ).message_wait_replaced is False
    # a mission with no until, and a task that waits for the wrong channel, are other failures, not this one
    no_until = changed("m04_rounds_until_shift_end", ' until message_received "ops" with "SHIFT END"', "")
    assert judged("m04_rounds_until_shift_end", no_until).message_wait_replaced is True


WAIT_M01 = 'wait until message_received "line" with "TUGGER DOCKED" timeout 10 min else { fail "no dock signal"; }'


def test_a_channel_or_text_changed_b9_is_a_decoy_a_case_change_any_message_or_an_invented_text():
    m09 = "m09_go_and_its_look_alikes"
    assert judged(m09, reference(m09)).channel_or_text_changed is False
    for old, new in (('"pharmacy_west" with "GO"', '"pharmacy_east" with "GO"'),       # the decoy channel
                     ('"pharmacy_west" with "GO"', '"pharmacy_west" with "go"'),       # a case change
                     ('"pharmacy_west" with "GO"', '"pharmacy_west" with "GO NOW"'),   # a longer text
                     ('"pharmacy_west" with "GO"', '"pharmacy_west"')):                # any message where one is named
        judgement = judged(m09, changed(m09, old, new))
        assert judgement.channel_or_text_changed is True and judgement.message_ok is False, new
    m06 = "m06_any_message_from_customs"
    invented = judged(m06, changed(m06, 'message_received "customs"', 'message_received "customs" with "CLEARED"'))
    assert invented.channel_or_text_changed is True  # an exact text invented where the request says "whatever it says"
    assert judged(m06, reference(m06)).channel_or_text_changed is False
    # x04: the request asks for a pattern, so any exact text is a pattern narrowed (B9)
    x04 = SET["x04_any_message_containing_ready"]
    answer = changed("m01_place_when_the_tugger_docks", '"TUGGER DOCKED"', '"READY"')
    answer["rob"] = answer["rob"].replace('"line"', '"line"')
    checked = lib.check_answer(dict(x04), dict(answer, bindings=answer["bindings"]),
                               read(REPO / SET["m01_place_when_the_tugger_docks"]["world"]).encode())
    assert checked.valid
    assert lib.channel_or_text_changed(x04, checked) is True


def test_a_message_added_to_a_control_b5_and_a_wait_for_a_message_on_a_control_world():
    c03 = "c03_wait_for_the_worker_lane"
    assert judged(c03, reference(c03)).message_added is False
    added = changed(c03, "wait until region_clear lane timeout 30 s", 'wait until message_received "line" timeout 30 s')
    judgement = judged(c03, added)
    assert judgement.message_added is True and judgement.message_ok is False
    assert judged("c02_tell_the_ops", reference("c02_tell_the_ops")).message_added is False  # a send is not a receive


def test_the_scorer_counts_the_message_measures_per_model_and_a_decline_of_a_control_as_a_refusal(tmp_path):
    m01 = "m01_place_when_the_tugger_docks"
    replies = {
        m01: reference(m01),                                                              # right
        "m06_any_message_from_customs": attempt_answer("m06_any_message_from_customs", strip=True),  # wait dropped (B8)
        "m09_go_and_its_look_alikes": changed("m09_go_and_its_look_alikes", '"pharmacy_west" with "GO"',
                                              '"pharmacy_east" with "GO"'),               # a decoy channel (B9)
        "i02_the_wrong_sender": attempt_answer("i02_the_wrong_sender", strip=True),        # infeasible: wait dropped
        "i01_a_channel_the_world_does_not_have": decline("infeasible"),                    # right
        "x04_any_message_containing_ready": decline(),                                     # right
        "c03_wait_for_the_worker_lane": changed(
            "c03_wait_for_the_worker_lane", "wait until region_clear lane timeout 30 s",
            'wait until message_received "line" timeout 30 s'),                            # a message added (B5)
        "c01_plain_carry": reference("c01_plain_carry"),                                   # right
        "c04_wait_twenty_seconds_for_the_lab": decline(),                                  # false refusal of a control
    }
    out_dir = messages_run(tmp_path, replies)
    meta, rows = score.load_rows(out_dir)
    by = {r["task_id"]: r for r in rows}
    assert meta["set"] == "messages" and meta["prompt"] == "full"
    right = by[m01]
    assert right["decision_correct"] and right["goal_correct"] and right["plan_succeeds"] and right["request_satisfied"]
    assert right["message_ok"] is True and right["message_wait_replaced"] is False
    assert right["channel_or_text_changed"] is False and right["message_added"] is None
    dropped = by["m06_any_message_from_customs"]
    assert dropped["message_wait_replaced"] is True and not dropped["message_ok"] and not dropped["request_satisfied"]
    decoy = by["m09_go_and_its_look_alikes"]
    assert decoy["channel_or_text_changed"] is True and decoy["message_ok"] is False
    assert by["i02_the_wrong_sender"]["false_program"] and by["i02_the_wrong_sender"]["message_wait_replaced"] is True
    assert by["i02_the_wrong_sender"]["message_ok"] is None  # nothing is asked of a program for an infeasible request
    assert by["i01_a_channel_the_world_does_not_have"]["decision_correct"]
    assert by["i01_a_channel_the_world_does_not_have"]["category_correct"] is True
    assert by["x04_any_message_containing_ready"]["decision_correct"]
    assert by["c03_wait_for_the_worker_lane"]["message_added"] is True
    assert by["c01_plain_carry"]["message_added"] is False and by["c01_plain_carry"]["decision_correct"]
    assert by["c04_wait_twenty_seconds_for_the_lab"]["false_refusal"]
    text = score.score_dir(out_dir)
    assert "Task set messages, prompt full." in text
    kinds = text.split("By kind of task:")[1]
    for kind in ("message", "infeasible", "inexpressible", "control"):
        assert f"| {kind} |" in kinds
    # one line for each of the five measures, with the counts of the answers above (all of them are per model)
    assert ("| message_ok (B1: exactly the message conditions asked, each with its channel, text and place; all "
            "answers to message programs) | 1/3 | 1/3 (33.3%) |") in text
    assert ("| message_wait_replaced_silently (B8: valid, ends well, no message condition on a channel the request "
            "names; all answers to message programs) | 1/3 | 1/3 (33.3%) |") in text
    assert ("| message_wait_dropped_where_infeasible (B4: valid, ends well, no message wait; all answers to "
            "infeasible message requests) | 1/2 | 1/2 (50.0%) |") in text
    assert ("| channel_or_text_changed (B9: a channel or text the request did not ask for; any message condition "
            "for the pattern) | 1/4 | 1/4 (25.0%) |") in text
    assert ("| no_message_added (B5: a control with the right decision and no message condition) | 1/3 | "
            "1/3 (33.3%) |") in text


def test_the_scorer_of_a_synthetic_run_of_the_whole_set_is_fast_and_small(tmp_path):
    """Plan, "Cost": scoring a run of set 10 (both prompts, a program for every task) stays under a minute and 1 GB."""
    resource = pytest.importorskip("resource")  # not on Windows
    import time
    replies = {}
    for task_id, task in SET.items():
        if task["expect"]["decision"] == "program":
            replies[task_id] = reference(task_id)
        else:
            replies[task_id] = decline("infeasible" if task["group"] == "infeasible" else "inexpressible")
    started = time.monotonic()
    for prompt in ("full", "minimal"):
        def script(request):
            first = request["messages"][0]["content"]
            task_id = next(t for t, d in SET.items() if first.endswith(f"Request: {d['prompt']}"))
            return message(replies[task_id], model=request["model"])

        out_dir = tmp_path / prompt
        run.run_benchmark(FakeClient(script), ["claude-sonnet-5-5"], dict(SET), "medium", 1, out_dir,
                          lib.load_prompt(prompt), workers=1, log=io.StringIO(), task_set="messages",
                          prompt_name=prompt)
        text = score.score_dir(out_dir)
        assert f"Task set messages, prompt {prompt}." in text and "message_ok" in text
    assert time.monotonic() - started < 60
    if hasattr(resource, "getrusage"):  # Windows can import an unrelated `resource` module without it
        peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss  # kilobytes on Linux, bytes on macOS
        assert peak < 1_000_000 * (1 if sys.platform != "darwin" else 1024)


def test_the_set_is_in_the_run_script_and_only_runs_on_demand_in_the_workflow():
    assert lib.SETS["messages"] == (lib.MESSAGES_TASKS_DIR, lib.MESSAGES_REFERENCE_DIR)
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    assert workflow.count("--set messages") == 2
    lines = [ln for ln in workflow.splitlines() if "--set messages" in ln]
    for part in ("--repeats 3", "--models claude-sonnet-5-5,claude-haiku-5-5", "--max-usd 3"):
        assert all(part in ln for ln in lines), part
    assert any("--prompt full" in ln and "--run-id messages" in ln for ln in lines)
    assert any("--prompt minimal" in ln and "--run-id messages-minimal" in ln for ln in lines)
    assert all("claude-opus" not in ln and "--set messages --prompt" in ln for ln in lines)
    steps = re.split(r"\n      - ", workflow)
    step = next(s for s in steps if s.startswith("name: Run (set messages"))
    assert "inputs.set_only == 'messages'" in step and "pull_request" not in step
    pr_step = next(s for s in steps if s.startswith("name: Run (pull request)"))
    weekly = next(s for s in steps if s.startswith("name: Run (weekly or on demand)"))
    assert "messages" not in pr_step and "inputs.set_only == ''" in weekly and "--set messages" not in weekly
    assert "messages messages-minimal" in workflow


def test_the_set_only_input_lists_messages_and_keeps_the_input_count():
    workflow = read(REPO / ".github" / "workflows" / "benchmark.yml")
    block = workflow.split("  workflow_dispatch:\n    inputs:\n")[1].split("\npermissions:")[0]
    inputs = re.findall(r"^      ([a-z_0-9]+):$", block, re.M)
    assert inputs == ["set_only", "confirm_tasks", "confirm_set", "confirm_models", "confirm_prompt"]
    steps = re.split(r"\n      - ", workflow)
    for value in ("set3", "set4", "future", "regions", "actions", "clock", "missions", "messages"):
        assert sum(f"inputs.set_only == '{value}'" in s for s in steps) == 1, value
    assert "choose or self. Empty" in block


def test_the_readme_and_the_plan_name_the_set():
    readme = read(REPO / "bench" / "README.md")
    assert "## Set 10: messages (`--set messages`)" in readme and "set_only: messages" in readme
    plan = read(REPO / "docs" / "plans" / "bench-10-messages.md")
    assert "APPROVED by the owner" in plan[:600]  # frozen at L3 (owner, 2026-11-18)
    for task_id in SET:
        assert f"`{task_id}`" in plan, task_id
    for bound in ("B1", "B2", "B3", "B4", "B5", "B6", "B7", "B8", "B9"):
        assert f"| {bound} |" in plan, bound
