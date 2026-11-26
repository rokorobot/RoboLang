"""
RoboLang 2.1: incoming messages (spec/robolang-v2.1.md, decision 0043 S1-S11 and clarifications C1-C14).

Front end, Task IR 1.9, Mission IR 1.1 and the reader, world file 1.6, the simulation (spec/simulation-v1.0.md section
19), the run record 1.9, and the refusals in `delegate` and the ROS 2 backend. No test here needs a model or a network.

To regenerate the IR snapshots after an intended change:

    ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_lang21.py
"""

from __future__ import annotations
import copy
import io
import json
import os
import re
import sys
from pathlib import Path

import pytest

from robolang import RoboLangError, check, check_file
from robolang.cli import main
from robolang.ir import compile_source, to_json
from robolang.ir_reader import IRError, read_ir
from robolang.sim import WorldError, load_world, mission_task, run, run_mission, to_mission_record, to_record
from robolang.sim.check import check_task
from robolang.tokens import FUTURE_RESERVED, RESERVED_BY_VERSION

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "v2.1"
SNAPSHOTS = ROOT / "tests" / "snapshots" / "ir-v2.1"
UPDATE = os.environ.get("ROBOLANG_UPDATE_SNAPSHOTS") == "1"
SPEC = (ROOT / "spec" / "robolang-v2.1.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")

# -------- helpers --------

FEED = """task feed(robot r, object tote, location src, location hold, location line) {
    goal { object_at tote line; }
    plan {
        move r to src; grasp r tote; move r to hold;
        %s
        move r to line; place r tote at line;
    }
}
"""
ROUND = """task round(robot r, location home, location far) {
    goal { robot_at r home; }
    plan { move r to far; wait for 20 s; move r to home; }
}
"""
def pre_task(channel: str, version: str = "2.1") -> str:
    """A task whose `pre` is a message condition."""
    return (f"robolang {version};\ntask t(robot r, location hold) {{\n pre {{ message_received \"{channel}\"; }}\n"
            " goal { robot_at r hold; }\n plan { move r to hold; }\n}\n")


def branch_task(condition: str, extra: str = "") -> str:
    """A task with one `if` on `condition`, both branches moving to `hold`."""
    return ("robolang 2.1;\ntask t(robot r, location hold) {\n plan {\n"
            f" if {condition} {{ goal {{ robot_at r hold; }} move r to hold; }} else {{ goal {{ robot_at r hold; }} "
            f"{extra} move r to hold; }}\n }}\n}}\n")


WAIT = 'wait until message_received "line" with "TUGGER DOCKED" timeout 10 min else { fail "no dock signal"; }'
WORLD = {
    "world_schema": "robolang.sim.world",
    "world_schema_version": "1.6",
    "locations": ["src", "hold", "line", "gate_in", "gate_out"],
    "regions": {},
    "robots": {"digit1": {"ready": True, "location": "src", "gripper": True},
               "tug1": {"ready": True, "location": "gate_in", "gripper": False}},
    "objects": {"tote_1": {"present": True, "location": "src"}},
    "channels": {"line": {"from": ["rl:agent:line-ctl"]}, "ops": {"from": ["rl:agent:ops-ctl", "rl:robot:tug1"]}},
    "events": [],
    "messages": [],
    "tasks": {},
}
BIND = {"r": "digit1", "tote": "tote_1", "src": "src", "hold": "hold", "line": "line"}
MISSION_BIND = {"r": "tug1", "home": "gate_in", "far": "gate_out"}


def feed(body: str = WAIT, version: str = "2.1") -> str:
    return f"robolang {version};\n\n" + FEED % body


def msg(seconds, channel="line", text="TUGGER DOCKED", sender="rl:agent:line-ctl", unit="s") -> dict:
    return {"at": {"value": str(seconds), "unit": unit}, "channel": channel, "text": text, "from": sender}


def world_data(messages=(), patch=None) -> bytes:
    world = copy.deepcopy(WORLD)
    world["messages"] = list(messages)
    if patch is not None:
        patch(world)
    return json.dumps(world).encode()


def codes(source: str) -> list[tuple[str, int]]:
    try:
        check(source)
    except RoboLangError as error:
        return [(d.code, d.line) for d in error.diagnostics]
    return []


def simulate(source: str, messages=(), patch=None, bindings=None, name="feed"):
    task = compile_source(source).ir.task(name)
    world = load_world(world_data(messages, patch), "w.json", task, bindings or BIND)
    return run(task, world)


def mission_run(source: str, messages=(), patch=None, name="shift"):
    document = compile_source(source).ir
    mission = document.mission(name)
    world = load_world(world_data(messages, patch), "w.json", mission_task(mission), MISSION_BIND)
    return run_mission(document, mission, world)


def timed_out(result) -> bool:
    """The wait ran to its timeout (S241): with a failure handler the step's own code is kept as `handled`."""
    return any(step.handled == "S241" for step in result.steps)


def end(result) -> tuple[str, str | None, str]:
    code = None if result.failure is None else result.failure.code
    return result.state, code, str(result.end_t_ms)


# -------- the language --------

def test_2_1_adds_no_reserved_word_and_the_word_was_reserved_since_1_1():
    assert "message_received" in FUTURE_RESERVED
    assert RESERVED_BY_VERSION["2.1"] == RESERVED_BY_VERSION["2.0"]


def test_a_file_below_2_1_still_reads_message_received_as_reserved_e211():
    for version in ("1.5", "1.9", "2.0"):
        assert [code for code, _ in codes(feed(version=version))] == ["E211"], version


def test_the_two_forms_and_their_combinations_validate():
    for condition in ('message_received "ops"', 'message_received "ops" with "GO"',
                      'not message_received "ops" with "STOP"',
                      'message_received "a" or message_received "b" with "X"',
                      '(message_received "a" and not message_received "b") or message_received "c" with "Y"'):
        body = f'wait until {condition} timeout 1 min else {{ fail "no"; }}'
        assert codes(feed(body)) == [], condition


def test_message_received_stands_in_pre_if_wait_and_a_missions_until():
    assert codes(pre_task("ops")) == []
    assert codes(branch_task('message_received "ops"')) == []
    mission = f"robolang 2.1;\n{ROUND}mission shift(robot r, location home, location far) {{\n" \
              '    repeat at most 3 runs { run round(r, home, far); } until message_received "ops" with "END";\n}\n'
    assert codes(mission) == []


def test_a_message_is_never_in_a_goal_e327_however_it_is_nested():
    for goal in ('message_received "line"', 'not message_received "line" with "X"',
                 'object_at tote line and (message_received "line" or robot_at r src)'):
        source = feed().replace("goal { object_at tote line; }", f"goal {{ {goal}; }}")
        assert [code for code, _ in codes(source)] == ["E327"], goal


def test_the_syntax_is_a_channel_and_an_optional_exact_text_in_quotes():
    assert [c for c, _ in codes(feed('wait until message_received timeout 1 min else { fail "n"; }'))] == ["E201"]
    with_nothing = feed('wait until message_received "a" with timeout 1 min else { fail "n"; }')
    assert [c for c, _ in codes(with_nothing)] == ["E201"]
    assert [c for c, _ in codes(feed('wait until message_received "a" with b timeout 1 min else { fail "n"; }'))] == [
        "E201"]
    for bad in ('message_received "" with "x"', 'message_received "a" with ""', 'message_received "a" with "  "'):
        assert [c for c, _ in codes(feed(f'wait until {bad} timeout 1 min else {{ fail "n"; }}'))] == ["E307"], bad


def test_a_wait_for_a_message_still_needs_a_timeout_and_a_handler():
    assert [c for c, _ in codes(feed('wait until message_received "a" else { fail "n"; }'))] == ["E209"]
    assert [c for c, _ in codes(feed('wait until message_received "a" timeout 1 min;'))] == ["E316"]


def test_every_new_message_in_the_error_text_names_what_to_do():
    with pytest.raises(RoboLangError) as caught:
        check(feed().replace("goal { object_at tote line; }", 'goal { message_received "line"; }'))
    assert "wait until" in caught.value.diagnostic.message and "not an end state" in caught.value.diagnostic.message


# -------- fixtures --------

HEADER = re.compile(
    r"\A// RoboLang v2\.1 fixture: (?P<set>valid|invalid)\n"
    r"// expect: (?:(?P<ok>ok)|error (?P<code>E\d{3}) \((?P<stage>lexical|syntax|static)\) line (?P<line>\d+))\n"
    r"// why: .+\n"
)
STAGE_DIGIT = {"lexical": "1", "syntax": "2", "static": "3"}
VALID = sorted((FIXTURES / "valid").glob("*.rob"))
INVALID = sorted((FIXTURES / "invalid").glob("*.rob"))


def header(path: Path) -> dict:
    match = HEADER.match(path.read_text(encoding="utf-8"))
    assert match, f"{path.name}: missing or malformed header"
    return match.groupdict()


@pytest.mark.parametrize("path", VALID, ids=lambda p: p.stem)
def test_valid_fixture_passes_and_matches_its_ir_snapshot(path):
    assert header(path)["ok"] and not check_file(path).warnings
    actual = to_json(compile_source(path.read_text(encoding="utf-8"), str(path)).ir)
    snapshot = SNAPSHOTS / f"{path.stem}.json"
    if UPDATE:
        snapshot.write_text(actual, encoding="utf-8")
    assert actual == snapshot.read_text(encoding="utf-8")
    document = json.loads(actual)
    mission = document["ir_schema"] == "robolang.mission-ir"
    assert (document["ir_schema_version"], document["language_version"]) == ("1.1" if mission else "1.9", "2.1")
    assert document.get("task_ir_schema_version", "1.9") == "1.9"
    assert read_ir(actual.encode()) == compile_source(path.read_text(encoding="utf-8")).ir


@pytest.mark.parametrize("path", INVALID, ids=lambda p: p.stem)
def test_invalid_fixture_is_rejected_with_the_stated_code_and_line_and_makes_no_ir(path):
    stated = header(path)
    assert path.name.startswith(stated["code"].lower()) and stated["code"][1] == STAGE_DIGIT[stated["stage"]]
    with pytest.raises(RoboLangError) as caught:
        compile_source(path.read_text(encoding="utf-8"), str(path))
    assert (caught.value.code, str(caught.value.line)) == (stated["code"], stated["line"])


def test_fixture_sets_are_counted():
    assert len(VALID) == 6 and len(INVALID) == 8


def test_every_new_code_is_in_the_specification_tables():
    table = set(re.findall(r"^\| `((?:[EISX]\d{3})(?:`, `[EISX]\d{3})*)` \|", SPEC, re.MULTILINE))
    listed = {code for row in table for code in row.split("`, `")}
    assert {"E327", "E519", "S011", "X608", "S515"} <= listed
    assert re.search(r"^\| `S011` \|", SIM_SPEC, re.MULTILINE) and re.search(r"^\| `E519` \|", SIM_SPEC, re.MULTILINE)
    assert "| S515 |" in (ROOT / "spec" / "ros2-backend-v0.1.md").read_text(encoding="utf-8")
    assert "`I228`" in (ROOT / "spec" / "ir-reader-v0.1.md").read_text(encoding="utf-8")
    assert "X608" in (ROOT / "spec" / "export-goal-v0.1.md").read_text(encoding="utf-8")
    assert "X608" in (ROOT / "spec" / "delegate-v0.1.md").read_text(encoding="utf-8")


def test_the_grammar_file_says_what_2_1_cannot_say_erratum_e5():
    """The grammar file is the grammar-only prompt: it must say what 2.1 cannot (S7, S10, S3, S4)."""
    grammar = (ROOT / "spec" / "grammar-v2.1.ebnf").read_text(encoding="utf-8")
    for phrase in ("delivery", "make sure it was delivered", "for each message", "the content of a message as a value",
                   "patterns", "containing READY", "inexpressible", "infeasible", "E327", "S011", "unknown",
                   "the sender is the world's", "do not make up a text", "Do not replace a message wait"):
        assert phrase in grammar, phrase
    assert grammar.isascii()


def test_the_safety_case_has_hazard_h23():
    text = (ROOT / "docs" / "safety-case-draft.md").read_text(encoding="utf-8")
    assert "| H23 | A spoofed message changes what a robot does |" in text


# -------- Task IR 1.9, Mission IR 1.1 and the reader --------

def document_of(source: str) -> dict:
    return json.loads(to_json(compile_source(source).ir))


def test_task_ir_1_9_shape_text_is_omitted_for_any_message():
    wait = document_of(feed(WAIT))["tasks"][0]["steps"][3]["condition"]
    assert wait == {"kind": "message_received", "channel": "line", "text": "TUGGER DOCKED"}
    any_message = document_of(feed('wait until message_received "nurse" timeout 1 min else { fail "n"; }'))
    assert any_message["tasks"][0]["steps"][3]["condition"] == {"kind": "message_received", "channel": "nurse"}
    assert any_message["ir_schema_version"] == "1.9"


def test_the_text_is_kept_as_written_the_match_trims():
    wait = document_of(feed('wait until message_received "line" with "  GO " timeout 1 min else { fail "n"; }'))
    assert wait["tasks"][0]["steps"][3]["condition"]["text"] == "  GO "


def test_a_file_with_a_mission_is_mission_ir_1_1_with_task_ir_1_9_tasks():
    source = f"robolang 2.1;\n{ROUND}mission shift(robot r, location home, location far) {{\n" \
             '    repeat at most 3 runs { run round(r, home, far); } until message_received "ops";\n}\n'
    document = document_of(source)
    assert (document["ir_schema"], document["ir_schema_version"], document["task_ir_schema_version"]) == (
        "robolang.mission-ir", "1.1", "1.9")
    assert document["missions"][0]["repeat"]["until"] == {"kind": "message_received", "channel": "ops"}
    assert read_ir(json.dumps(document).encode()) == compile_source(source).ir


def blob(document: dict) -> bytes:
    return json.dumps(document).encode("utf-8")


def problems(document) -> list[tuple[str, str]]:
    with pytest.raises(IRError) as caught:
        read_ir(blob(document), "x.json")
    return [(d.code, d.pointer) for d in caught.value.diagnostics]


def test_the_reader_restates_the_goal_and_empty_rules():
    base = document_of(feed(WAIT))
    bad = copy.deepcopy(base)
    bad["tasks"][0]["goal"].append({"kind": "message_received", "channel": "line"})
    assert problems(bad) == [("I228", "/tasks/0/goal/1")]
    nested = copy.deepcopy(base)
    nested["tasks"][0]["goal"] = [{"kind": "not", "condition": {"kind": "message_received", "channel": "a"}}]
    assert problems(nested) == [("I228", "/tasks/0/goal/0/condition")]
    empty = copy.deepcopy(base)
    empty["tasks"][0]["steps"][3]["condition"]["text"] = "   "
    assert problems(empty) == [("I207", "/tasks/0/steps/3/condition/text")]
    empty["tasks"][0]["steps"][3]["condition"] = {"kind": "message_received", "channel": ""}
    assert problems(empty) == [("I207", "/tasks/0/steps/3/condition/channel")]


def test_the_reader_checks_the_form_and_the_schema_of_the_condition():
    base = document_of(feed(WAIT))
    for change, expected in (({"text": 5}, "I110"), ({"pattern": "x"}, "I112"), ({"channel": None}, "I110")):
        bad = copy.deepcopy(base)
        bad["tasks"][0]["steps"][3]["condition"].update(change)
        assert problems(bad)[0][0] == expected, change
    bad = copy.deepcopy(base)
    del bad["tasks"][0]["steps"][3]["condition"]["channel"]
    assert problems(bad)[0][0] == "I111"
    old = copy.deepcopy(base)
    old.update(ir_schema_version="1.8", language_version="2.0")
    assert problems(old)[0] == ("I115", "/tasks/0/steps/3/condition/kind")


def test_the_reader_checks_the_header_of_both_mission_versions():
    source = f"robolang 2.1;\n{ROUND}mission shift(robot r, location home, location far) {{\n" \
             '    repeat at most 3 runs { run round(r, home, far); } until message_received "ops";\n}\n'
    document = document_of(source)
    for key, value in (("ir_schema_version", "1.5"), ("task_ir_schema_version", "1.8"), ("language_version", "2.0")):
        bad = copy.deepcopy(document)
        bad[key] = value
        assert ("I113", f"/{key}") in problems(bad), key
    bad = copy.deepcopy(document)
    bad["missions"][0]["repeat"]["until"]["channel"] = ""
    assert problems(bad) == [("I207", "/missions/0/repeat/until/channel")]
    mixed = copy.deepcopy(document)  # Mission IR 1.0 holds Task IR 1.8 tasks: a message condition is not in it
    mixed.update(ir_schema_version="1.0", task_ir_schema_version="1.8", language_version="2.0")
    assert ("I115", "/missions/0/repeat/until/kind") in problems(mixed)
    # a 2.0 mission document is still read as before
    old = copy.deepcopy(document)
    old.update(ir_schema_version="1.0", task_ir_schema_version="1.8", language_version="2.0")
    old["missions"][0]["repeat"]["until"] = {"kind": "region_clear", "region": "far"}
    old["missions"][0]["parameters"][2]["type"] = "region"
    old["tasks"][0]["parameters"][2]["type"] = "region"
    old["tasks"][0]["steps"] = [{"kind": "move", "robot": "r", "target": "home"}]
    old["tasks"][0]["goal"] = [{"kind": "robot_at", "robot": "r", "location": "home"}]
    assert read_ir(blob(old)).ir_schema_version == "1.8"


# -------- world file 1.6 --------

def world_error(patch, messages=()) -> WorldError:
    task = compile_source(feed()).ir.task("feed")
    with pytest.raises(WorldError) as caught:
        load_world(world_data(messages, patch), "w.json", task, BIND)
    return caught.value


def test_a_valid_world_has_channels_and_messages_as_events_in_time_order():
    task = compile_source(feed()).ir.task("feed")
    world = load_world(world_data([msg(90), msg(5, text="X", sender="rl:agent:other")]), "w.json", task, BIND)
    assert dict(world.channels) == {"line": ("rl:agent:line-ctl",), "ops": ("rl:agent:ops-ctl", "rl:robot:tug1")}
    assert [(str(e.at_ms), e.fact, e.target, e.value.text) for e in world.events] == [
        ("5000", "message", "line", "X"), ("90000", "message", "line", "TUGGER DOCKED")]


def test_a_message_may_arrive_at_zero_unlike_the_other_events():
    task = compile_source(feed()).ir.task("feed")
    load_world(world_data([msg(0)]), "w.json", task, BIND)  # no error
    assert world_error(None, [msg(-1)]).code == "E504"


def test_world_errors_of_channels_and_messages_are_e519_or_the_usual_forms():
    cases = {
        "empty channels": (lambda w: w.update(channels={}), "E519", "/channels"),
        "channel with no sender": (lambda w: w["channels"]["line"].update({"from": []}), "E519", "/channels/line/from"),
        "a sender listed twice": (lambda w: w["channels"]["line"].update({"from": ["rl:agent:a", "rl:agent:a"]}),
                                  "E519", "/channels/line/from"),
        "a robot the world has": (lambda w: w["channels"]["line"].update({"from": ["rl:robot:nobody"]}), "E519",
                                  "/channels/line/from/0"),
        "a sender of a bad form": (lambda w: w["channels"]["line"].update({"from": ["line-ctl"]}), "E504",
                                   "/channels/line/from/0"),
        "an unknown field of a channel": (lambda w: w["channels"]["line"].update({"to": []}), "E503",
                                          "/channels/line/to"),
    }
    for name, (patch, code, pointer) in cases.items():
        error = world_error(patch)
        assert (error.code, error.pointer) == (code, pointer), name
    for bad, code, pointer in (
            (msg(1, channel="nowhere"), "E519", "/messages/0/channel"),
            (msg(1, text=""), "E519", "/messages/0/text"),
            (msg(1, sender="rl:robot:nobody"), "E519", "/messages/0/from"),
            (msg(1, sender="someone"), "E504", "/messages/0/from")):
        error = world_error(None, [bad])
        assert (error.code, error.pointer) == (code, pointer), bad
    error = world_error(lambda w: w.pop("channels"), [msg(1)])
    assert (error.code, error.pointer) == ("E519", "/messages/0/channel")


def test_world_file_1_5_has_neither_field():
    for field, value in (("channels", WORLD["channels"]), ("messages", [])):
        def patch(w, field=field, value=value):
            w.pop("channels"), w.pop("messages")
            w.update(world_schema_version="1.5", **{field: value})
        error = world_error(patch)
        assert (error.code, error.pointer) == ("E503", f"/{field}")


def test_a_world_with_no_channels_loads_and_has_no_message_service():
    task = compile_source(feed()).ir.task("feed")
    world = load_world(world_data((), lambda w: w.pop("channels")), "w.json", task, BIND)
    assert dict(world.channels) == {}


# -------- the simulation (S2-S5, C1, C4-C8) --------

def test_a_wait_ends_at_the_arrival_of_the_message_s1():
    assert end(simulate(feed(), [msg(90)])) == ("SUCCEEDED", None, "90000")


def test_no_message_in_time_times_out_s241_and_the_handler_decides():
    assert end(simulate(feed(), [])) == ("FAILED", "S262", "600000") and timed_out(simulate(feed(), []))
    handled = feed(WAIT.replace('fail "no dock signal";', 'communicate r to "ops" with "late";'))
    result = simulate(handled, [])
    assert result.state == "SUCCEEDED" and timed_out(result)  # the handler ran, and the plan went on


def test_a_message_at_exactly_the_timeout_counts_c5_and_one_later_does_not():
    assert end(simulate(feed(), [msg(10, unit="min")])) == ("SUCCEEDED", None, "600000")
    assert end(simulate(feed(), [msg(600001, unit="ms")])) == ("FAILED", "S262", "600000")


def test_a_message_that_has_arrived_before_the_wait_counts_at_once_latched_s2_c7():
    source = feed("wait for 60 s; " + WAIT + " " + WAIT)
    result = simulate(source, [msg(30)])
    assert end(result) == ("SUCCEEDED", None, "60000")  # both waits pass at once, at the same time as the wait for 60 s


def test_a_message_at_zero_counts_in_pre_and_in_the_first_step_c4():
    source = ("robolang 2.1;\ntask t(robot r, location src, location hold) {\n pre { message_received \"line\"; }\n"
              " goal { robot_at r hold; }\n plan { move r to hold; }\n}\n")
    bindings = {"r": "digit1", "src": "src", "hold": "hold"}
    assert end(simulate(source, [msg(0)], bindings=bindings, name="t"))[:2] == ("SUCCEEDED", None)
    late = simulate(source, [msg(1)], bindings=bindings, name="t")
    assert (late.state, late.failure.code) == ("FAILED", "S101")  # a declared channel with no message yet is false


def test_the_text_is_exact_and_case_sensitive_but_spaces_are_trimmed_s3_c11():
    assert end(simulate(feed(), [msg(5, text="  TUGGER DOCKED   ")]))[:2] == ("SUCCEEDED", None)
    for text in ("tugger docked", "TUGGER DOCKED NOW", "TUGGER  DOCKED", "TUGGER", ""[:0] + "x"):
        assert end(simulate(feed(), [msg(5, text=text)]))[1] == "S262", text
    spaced = feed(WAIT.replace('"TUGGER DOCKED"', '"  TUGGER DOCKED "'))
    assert end(simulate(spaced, [msg(5)]))[:2] == ("SUCCEEDED", None)


def test_a_message_with_no_text_in_the_condition_means_any_message_on_the_channel_c9():
    source = feed('wait until message_received "line" timeout 1 min else { fail "n"; }')
    assert end(simulate(source, [msg(7, text="whatever it says")]))[:2] == ("SUCCEEDED", None)
    assert end(simulate(source, [msg(7, channel="ops", sender="rl:agent:ops-ctl")]))[1] == "S262"  # another channel


def test_only_a_sender_the_world_allows_counts_and_the_rest_is_recorded_as_ignored_s4():
    result = simulate(feed(), [msg(5, sender="rl:agent:spoofer"), msg(8, sender="rl:agent:line-ctl")])
    assert end(result) == ("SUCCEEDED", None, "8000")
    assert [(str(m.t_ms), m.sender, m.accepted) for m in result.messages] == [
        ("5000", "rl:agent:spoofer", False), ("8000", "rl:agent:line-ctl", True)]
    only_ignored = simulate(feed(), [msg(5, sender="rl:agent:spoofer")])
    assert end(only_ignored) == ("FAILED", "S262", "600000") and not only_ignored.messages[0].accepted


def test_a_world_robot_may_be_a_declared_sender_c6():
    source = feed('wait until message_received "ops" with "GO" timeout 1 min else { fail "n"; }')
    assert end(simulate(source, [msg(4, "ops", "GO", "rl:robot:tug1")]))[:2] == ("SUCCEEDED", None)
    assert end(simulate(source, [msg(4, "ops", "GO", "rl:robot:digit1")]))[1] == "S262"  # not declared for the channel


def test_a_tasks_own_communicate_is_never_a_received_message_c6():
    own = feed('communicate r to "line" with "TUGGER DOCKED"; wait until message_received "line" with "TUGGER DOCKED" '
               'timeout 1 min else { fail "n"; }')
    assert end(simulate(own, []))[1] == "S262"


def test_not_message_received_and_or_combine_c10():
    source = feed('wait until message_received "line" with "A" or message_received "ops" with "B" timeout 1 min else '
                  '{ fail "n"; }')
    assert end(simulate(source, [msg(9, "ops", "B", "rl:agent:ops-ctl")]))[:2] == ("SUCCEEDED", None)
    branch = ("robolang 2.1;\ntask t(robot r, location src, location hold) {\n plan {\n"
              '  if not message_received "line" with "STOP" { goal { robot_at r hold; } move r to hold; } else '
              "{ goal { robot_at r src; } communicate r to \"line\" with \"stopped\"; }\n }\n}\n")
    bindings = {"r": "digit1", "src": "src", "hold": "hold"}
    assert simulate(branch, [], bindings=bindings, name="t").decisions[0].branch == "then"


def test_no_message_service_means_unknown_and_everything_fails_closed_s5_c1_c10():
    def no_channels(w):
        w.pop("channels")
    assert end(simulate(feed(), [], no_channels)) == ("FAILED", "S262", "600000")
    pre = pre_task("line")
    failed = simulate(pre, [], no_channels, {"r": "digit1", "hold": "hold"}, "t")
    assert (failed.state, failed.failure.code) == ("FAILED", "S102")
    branch = branch_task('not message_received "line"')
    unknown = simulate(branch, [], no_channels, {"r": "digit1", "hold": "hold"}, "t")
    assert (unknown.state, unknown.failure.code) == ("FAILED", "S261")  # not of an unknown is unknown


def test_a_world_before_1_6_has_no_message_service_either():
    old = compile_source(feed()).ir.task("feed")
    data = json.loads(world_data())
    data.update(world_schema_version="1.5")
    for key in ("channels", "messages"):
        data.pop(key)
    world = load_world(json.dumps(data).encode(), "w.json", old, BIND)
    assert end(run(old, world))[1] == "S262"


def test_an_undeclared_channel_fails_before_the_run_s011_c1():
    source = feed().replace('"line" with', '"nowhere" with')
    result = simulate(source, [msg(5)])
    assert (result.state, result.failure.code, result.failure.phase) == ("FAILED", "S011", "authorization")
    assert result.states == ("CREATED", "VALIDATED", "FAILED") and result.steps[0].status == "not_run"
    failed = simulate(pre_task("nowhere"), [], None, {"r": "digit1", "hold": "hold"}, "t")
    assert (failed.failure.code, failed.failure.precondition) == ("S011", 1)
    # in a branch that no run may reach, the channel is still a fact of the task and the world
    hidden = branch_task("robot_ready r", 'wait until message_received "nowhere" timeout 1 s else { fail "n"; }')
    assert simulate(hidden, [], None, {"r": "digit1", "hold": "hold"}, "t").failure.code == "S011"


def test_validate_world_reports_s011_at_the_condition_and_not_for_a_world_with_no_channels(tmp_path):
    rob = tmp_path / "t.rob"
    rob.write_text(feed().replace('"line" with', '"nowhere" with'), encoding="utf-8")
    world = tmp_path / "w.json"
    data = json.loads(world_data())
    data["tasks"] = {"feed": {"bindings": BIND}}
    world.write_text(json.dumps(data), encoding="utf-8")
    out, err = io.StringIO(), io.StringIO()
    code = main(["validate", str(rob), "--world", str(world)], out=out, err=err)
    assert code == 1 and "error[S011]" in err.getvalue() and "'nowhere'" in err.getvalue()
    assert f"{rob}:7:" in err.getvalue()  # the line of the wait
    del data["channels"], data["messages"]
    world.write_text(json.dumps(data), encoding="utf-8")
    out, err = io.StringIO(), io.StringIO()
    assert main(["validate", str(rob), "--world", str(world)], out=out, err=err) == 0, err.getvalue()


def test_check_task_lists_the_pre_condition_with_its_index():
    task = compile_source(pre_task("nowhere")).ir.task("t")
    world = load_world(world_data(), "w.json", task, {"r": "digit1", "hold": "hold"})
    assert [(f.code, f.where) for f in check_task(task, world)] == [("S011", ("preconditions", 0))]


def test_the_run_is_a_function_of_its_inputs_and_the_record_lists_the_messages():
    messages = [msg(3, sender="rl:agent:spoofer"), msg(40)]
    first, second = simulate(feed(), messages), simulate(feed(), messages)
    assert first == second
    task = compile_source(feed()).ir.task("feed")
    world = load_world(world_data(messages), "w.json", task, BIND)
    record = to_record(first, task=task, run_id="sim-" + "0" * 64, source_sha256="a" * 64, world_sha256="b" * 64,
                       seed="0", language_version="2.1", world_schema_version="1.6")
    assert (record["run_record_schema_version"], record["inputs"]["ir_schema_version"]) == ("1.9", "1.9")
    assert record["messages"] == [
        {"t_ms": "3000", "channel": "line", "text": "TUGGER DOCKED", "from": "rl:agent:spoofer", "status": "ignored"},
        {"t_ms": "40000", "channel": "line", "text": "TUGGER DOCKED", "from": "rl:agent:line-ctl",
         "status": "accepted"}]
    assert record["events_applied"] == [] and list(record).index("messages") == list(record).index("events_applied") + 1
    assert world.schema_version == "1.6" and json.dumps(record).isascii()


def test_a_2_0_record_is_unchanged_there_is_no_messages_key():
    task = compile_source("robolang 2.0;" + pre_task("line", "2.0").split(";", 1)[1].replace(
        ' pre { message_received "line"; }\n', "")).ir.task("t")
    world = load_world(world_data(), "w.json", task, {"r": "digit1", "hold": "hold"})
    record = to_record(run(task, world), task=task, run_id="sim-" + "0" * 64, source_sha256="a" * 64,
                       world_sha256="b" * 64, seed="0", language_version="2.0", world_schema_version="1.6")
    assert "messages" not in record and record["run_record_schema_version"] == "1.8"


# -------- missions (S2, C8, C12) --------

MISSION = f"""robolang 2.1;

{ROUND}
mission shift(robot r, location home, location far) {{
    repeat at most 12 runs {{ run round(r, home, far); }} until message_received "ops" with "SHIFT END";
}}
"""


def test_a_missions_until_sees_the_messages_since_the_mission_started_c8():
    ops = lambda seconds, text="SHIFT END", sender="rl:agent:ops-ctl": msg(seconds, "ops", text, sender)  # noqa: E731
    done = mission_run(MISSION, [ops(50)])
    assert (done.outcome, done.ended_by, len(done.runs)) == ("completed", "until", 3)  # runs of 20 s: after 60 s
    early = mission_run(MISSION, [ops(5)])  # arrived during run 1: seen by the until after run 1
    assert (early.outcome, len(early.runs)) == ("completed", 1)
    none = mission_run(MISSION, [])
    assert (none.outcome, none.ended_by, len(none.runs)) == ("bounded", "run_bound", 12)
    spoof = mission_run(MISSION, [ops(5, sender="rl:agent:spoofer")])
    assert (spoof.outcome, len(spoof.runs)) == ("bounded", 12)
    assert [(m.accepted, str(m.t_ms)) for m in spoof.messages] == [(False, "5000")]


STEP_MISSION = """robolang 2.1;

task step(robot r, location hold) {
    goal { robot_at r hold; }
    plan {
        move r to hold; wait for 20 s;
        wait until message_received "ops" with "GO" timeout 30 s else { fail "no go"; }
    }
}

mission shift(robot r, location hold) {
    repeat at most 2 runs { run step(r, hold); };
    on failure { continue; }
}
"""


def step_run(seconds: int):
    document = compile_source(STEP_MISSION).ir
    mission = document.mission("shift")
    world = load_world(world_data([msg(seconds, "ops", "GO", "rl:agent:ops-ctl")]), "w.json", mission_task(mission),
                       {"r": "digit1", "hold": "hold"})
    return run_mission(document, mission, world)


def test_each_task_run_starts_fresh_a_message_of_run_1_does_not_satisfy_run_2_c8():
    stale = step_run(2)  # arrived during run 1's own wait for 20 s: counts in run 1 (latched), not in run 2 (20 s on)
    assert [r.result.state for r in stale.runs] == ["SUCCEEDED", "FAILED"]
    assert stale.runs[1].result.failure.code == "S262" and timed_out(stale.runs[1].result)
    assert (stale.outcome, stale.runs[1].t_start_ms) == ("bounded", 20000)
    # a message at exactly the start of run 2 counts in it (C4): run 1 ends at 20 s, run 2 starts at 20 s
    assert [r.result.state for r in step_run(20).runs] == ["SUCCEEDED", "SUCCEEDED"]
    # a message that arrives later, during run 2, counts there (and run 1 failed at 30 s... it did not wait that long)
    late = step_run(25)
    assert [r.result.state for r in late.runs] == ["SUCCEEDED", "SUCCEEDED"]  # run 1 ends at 25 s, 2 starts then


def test_a_mission_until_naming_an_undeclared_channel_fails_before_any_run_s011():
    result = mission_run(MISSION.replace('"ops" with', '"nowhere" with'), [])
    assert (result.outcome, result.ended_by, result.failure.code, len(result.runs)) == (
        "failed", "undeclared_channel", "S011", 0)


def test_a_mission_until_with_no_message_service_is_unknown_and_stops_s402():
    result = mission_run(MISSION, [], lambda w: w.pop("channels"))
    assert (result.outcome, result.ended_by, result.failure.code) == ("failed", "until_unknown", "S402")


def test_the_mission_record_1_1_lists_the_messages_of_the_whole_mission_and_of_each_run():
    document = compile_source(MISSION).ir
    mission = document.mission("shift")
    messages = [msg(50, "ops", "SHIFT END", "rl:agent:ops-ctl")]
    world = load_world(world_data(messages), "w.json", mission_task(mission), MISSION_BIND)
    result = run_mission(document, mission, world)
    record = to_mission_record(result, document=document, run_id="sim-" + "0" * 64, source_sha256="a" * 64,
                               world_sha256="b" * 64, seed="0", world_schema_version="1.6", world_clock=world.clock)
    assert (record["mission_record_schema_version"], record["run_record_schema_version"]) == ("1.1", "1.9")
    assert record["inputs"]["ir_schema_version"] == "1.1" and record["inputs"]["task_ir_schema_version"] == "1.9"
    assert [m["t_ms"] for m in record["messages"]] == ["50000"]
    assert [len(r["messages"]) for r in record["runs"]] == [0, 0, 1]  # it arrived during run 3 (40 to 60 s)
    assert record["messages"][0]["status"] == "accepted"


# -------- the command, and the refusals (S8) --------

def cli(*args: str):
    out, err = io.StringIO(), io.StringIO()
    code = main(list(args), out=out, err=err)
    return code, out.getvalue(), err.getvalue()


@pytest.fixture
def files(tmp_path):
    rob = tmp_path / "feed.rob"
    rob.write_text(feed(), encoding="utf-8")
    data = json.loads(world_data([msg(90)]))
    data["tasks"] = {"feed": {"bindings": BIND}}
    data["objects"]["tote_1"].update({"class": "tote", "mass": {"value": "4", "unit": "kg", "uncertainty": "0.1"}})
    world = tmp_path / "w.json"
    world.write_text(json.dumps(data), encoding="utf-8")
    return str(rob), str(world)


def test_simulate_prints_the_record_with_the_messages_and_exits_by_the_state(files):
    code, out, err = cli("simulate", *files[:1], "--world", files[1])
    assert code == 0, err
    record = json.loads(out)
    assert record["state"] == "SUCCEEDED" and record["end_t_ms"] == "90000" and len(record["messages"]) == 1
    assert out.isascii()


def test_inspect_shows_a_message_condition_in_the_source_form(files):
    code, out, err = cli("inspect", files[0])
    assert code == 0 and 'message_received "line" with "TUGGER DOCKED"' in out, err


def test_ir_input_round_trips_through_every_command(files, tmp_path):
    code, out, _ = cli("inspect", files[0], "--json")
    ir_path = tmp_path / "feed.json"
    ir_path.write_text(out, encoding="utf-8")
    code, out, err = cli("simulate", "--ir", str(ir_path), "--world", files[1])
    assert code == 0 and json.loads(out)["inputs"]["source_kind"] == "ir", err


def test_export_goal_is_unaffected_a_goal_holds_no_message(files):
    code, out, err = cli("export-goal", files[0], "--world", files[1], "--from", "rl:agent:planner", "--core", "0.2")
    assert code == 0, err
    assert "message_received" not in out and json.loads(out)["payload"]["goal"]


def test_delegate_refuses_a_task_with_a_message_x608_and_does_not_use_such_a_library_task(files):
    code, out, err = cli("delegate", files[0], "--world", files[1], "--from", "rl:agent:planner", "--to",
                         "rl:robot:digit1")
    assert (code, out) == (6, "") and "error[X608]" in err and "message_received" in err
    from robolang import delegate
    task = compile_source(feed()).ir.tasks[0]
    assert delegate._consider(None, task, b"", "w.json", "digit1", (), "0.1") == ("message_unsupported", None)


def test_the_ros2_backend_refuses_message_received_with_s515_before_any_motion():
    pytest.importorskip("jsonschema")
    sys.path.insert(0, str(ROOT / "backends" / "ros2"))
    from robolang_ros2.executor import execute
    from test_ros2_executor import FakeOps, fixture
    _, bindings, site = fixture()
    for where in ("pre", "plan"):
        pre = 'pre { message_received "line"; }' if where == "pre" else ""
        wait = 'wait until message_received "line" timeout 1 s else { fail "n"; }' if where == "plan" else ""
        source = ("robolang 2.1;\ntask transfer(robot r, object box, location src, location dst) {\n"
                  f"  {pre} goal {{ robot_at r dst; }}\n  plan {{ {wait} move r to dst; }}\n}}\n")
        ops = FakeOps()
        record = execute(compile_source(source).ir.tasks[0], dict(bindings), site, ops)
        assert record["failure"]["code"] == "S515" and "no message service" in record["failure"]["message"]
        assert ops.calls == [("guard", "ur")]


# -------- Python 3.11 and the economy --------

def test_no_dataclass_of_messages_has_a_mutable_default_python_3_11():
    """PR #151 erratum E4: Python 3.11 rejects an unhashable dataclass default when the module is imported."""
    import dataclasses
    from robolang import nodes
    from robolang.sim import executor, messages, mission, world
    for module in (messages, mission, executor, world, nodes):
        for item in vars(module).values():
            if dataclasses.is_dataclass(item) and isinstance(item, type):
                for f in dataclasses.fields(item):
                    if f.default is not dataclasses.MISSING:
                        hash(f.default)


def test_the_economy_no_statement_one_condition_no_reserved_word_no_declaration():
    from robolang import tokens
    assert tokens.STATEMENTS_V1_8 == tokens.STATEMENTS_V1_8
    assert set(tokens.CONDITIONS_V2_1) - set(tokens.CONDITIONS_V1_8) == {"message_received"}
    assert tokens.RESERVED_V2_0 == RESERVED_BY_VERSION["2.1"]


def test_a_long_wait_over_many_messages_is_fast_and_linear():
    messages = [msg(i, text=f"noise {i}", sender="rl:agent:spoofer") for i in range(1, 590)] + [msg(590)]
    assert end(simulate(feed(), messages))[:2] == ("SUCCEEDED", None)
