"""Tests for RoboLang 1.3: goals, outcome atoms, typed sensors (spec/robolang-v1.3.md, ir-schema-v1.2.md)."""

from __future__ import annotations
import copy
import io
import json
import re
from pathlib import Path

import pytest

from robolang import RoboLangError, TokenKind, check, parse, tokenize
from robolang import nodes as ast
from robolang.cli import main
from robolang.ir import compile_source, to_dict, to_json
from robolang.sim import SimulationInputError, Truth, load_world, run, to_record
from robolang.sim.world import WorldError

ROOT = Path(__file__).resolve().parent.parent
V13 = ROOT / "tests" / "fixtures" / "v1.3" / "simulation"
BASE = json.loads((V13 / "worlds" / "base.json").read_text(encoding="utf-8"))

PARAMS = "robot r, object box, location src, location dst, region cell, sensor cam"


def task(
    pre: str | None = None,
    goal: str | None = "robot_at r dst;",
    plan: str = "move r to dst;",
    version: str | None = "1.3",
    params: str = PARAMS,
) -> str:
    marker = f"robolang {version};\n" if version else ""
    body = f"  pre {{ {pre} }}\n" if pre is not None else ""
    body += f"  goal {{ {goal} }}\n" if goal is not None else ""
    return f"{marker}task t({params}) {{\n{body}  plan {{ {plan} }}\n}}\n"


def error(source: str) -> RoboLangError:
    with pytest.raises(RoboLangError) as raised:
        compile_source(source)
    return raised.value


def where(source: str) -> tuple[str, int, int]:
    e = error(source)
    return e.code, e.line, e.column


def cli(argv):
    out, err = io.StringIO(), io.StringIO()
    code = main(argv, out=out, err=err)
    return code, out.getvalue(), err.getvalue()


# -------- version and reserved words (spec 2) --------

NEW_WORDS = ("goal", "sensor", "object_at", "robot_at", "holding")


@pytest.mark.parametrize(
    ("marker", "kind"),
    [
        ("", TokenKind.IDENT),
        ("robolang 1.1; ", TokenKind.IDENT),
        ("robolang 1.2; ", TokenKind.IDENT),
        ("robolang 1.3; ", TokenKind.KEYWORD),
    ],
)
@pytest.mark.parametrize("word", NEW_WORDS)
def test_new_words_are_names_before_1_3_and_reserved_in_1_3(marker, kind, word):
    (token,) = [t for t in tokenize(marker + word) if t.text == word]
    assert token.kind is kind


@pytest.mark.parametrize("word", NEW_WORDS)
def test_new_words_cannot_be_names_in_1_3(word):
    source = (
        f"robolang 1.3;\ntask t(robot r, location {word}) {{\n"
        f"  goal {{ robot_ready r; }}\n  plan {{ move r to {word}; }}\n}}\n"
    )
    assert where(source)[0] == "E212"


@pytest.mark.parametrize("version", [None, "1.1", "1.2"])
def test_earlier_files_may_use_the_new_words_as_names(version):
    marker = f"robolang {version};\n" if version else ""
    source = (
        f"{marker}task t(robot goal, location sensor, object holding, location object_at, location robot_at) {{\n"
        "  plan { move goal to sensor; move goal to object_at; move goal to robot_at; grasp goal holding; }\n}\n"
    )
    assert compile_source(source).ir.language_version == (version or "1.1")


def test_versions():
    for version, expected in ((None, ("1.1", "1.0")), ("1.1", ("1.1", "1.0")), ("1.2", ("1.2", "1.1"))):
        document = json.loads(
            to_json(compile_source(task(pre=None, goal=None, version=version, params="robot r, location dst")).ir)
        )
        assert (document["language_version"], document["ir_schema_version"]) == expected
    document = json.loads(to_json(compile_source(task()).ir))
    assert (document["language_version"], document["ir_schema_version"]) == ("1.3", "1.2")


def test_unsupported_versions_list_all_supported_ones():
    raised = error("robolang 1.4;\n" + task(version=None))
    assert (raised.code, raised.line, raised.column) == ("E213", 1, 10)
    assert "1.1, 1.2 and 1.3" in str(raised)


# -------- syntax and AST (spec 3, 6) --------


def only_task(source: str) -> ast.Task:
    return parse(source).program.tasks[0]


def test_goal_is_parsed_between_pre_and_plan():
    t = only_task(task(pre="robot_ready r;", goal="object_at box dst; robot_at r dst;"))
    assert isinstance(t.pre, ast.PreBlock) and isinstance(t.goal, ast.GoalBlock)
    assert [type(c) for c in t.goal.conditions] == [ast.ObjectAt, ast.RobotAt]
    assert t.pre.span.end_line <= t.goal.span.line <= t.plan.span.line


def test_goal_without_pre_and_after_safety():
    t = only_task(
        "robolang 1.3;\ntask t(robot r, location dst) {\n @safety max_speed 1 m/s;\n"
        " goal { robot_at r dst; }\n plan { move r to dst; }\n}\n"
    )
    assert t.pre is None and t.goal is not None and len(t.annotations) == 1


def test_a_task_without_a_goal_parses_and_is_rejected_by_the_validator():
    source = task(goal=None)
    assert only_task(source).goal is None
    assert error(source).code == "E309"


def test_pair_atoms_have_two_names_and_a_span():
    t = only_task(task(goal="object_at box dst; robot_at r src; holding r box;"))
    at, robot_at, holding = t.goal.conditions
    assert (at.obj.text, at.location.text) == ("box", "dst")
    assert (robot_at.robot.text, robot_at.location.text) == ("r", "src")
    assert (holding.robot.text, holding.obj.text) == ("r", "box")
    assert (at.span.line, at.span.column, at.span.end_column) == (3, 10, 27)


def test_sensor_ok_and_inspect_take_a_name_in_1_3_and_a_string_before():
    t = only_task(task(pre="sensor_ok cam;", plan="inspect r box using cam;"))
    assert isinstance(t.pre.conditions[0].sensor, ast.Name)
    assert isinstance(t.plan.statements[0].sensor, ast.Name)
    old = only_task(
        task(pre=None, goal=None, version="1.2", params="robot r, object box", plan='inspect r box using "cam";')
    )
    assert isinstance(old.plan.statements[0].sensor, ast.StringLit)


def test_a_string_sensor_is_kept_in_the_tree_and_rejected_by_the_validator():
    t = only_task(task(pre='sensor_ok "camera_top";'))
    assert isinstance(t.pre.conditions[0].sensor, ast.StringLit)
    assert where(task(pre='sensor_ok "camera_top";'))[0] == "E310"


def test_sensor_is_a_parameter_type_in_1_3_only():
    assert only_task(task()).params[-1].type is ast.ParamType.SENSOR
    for version in (None, "1.1", "1.2"):
        raised = error(task(pre=None, goal=None, version=version, params="robot r, sensor s"))
        assert raised.code == "E205" and "'sensor'" not in str(raised).split("expected one of")[1]
    assert "'sensor'" in str(error(task(params="robot r, thing s")))


@pytest.mark.parametrize(
    ("body", "code", "message"),
    [
        ("goal { }\n  plan { move r to dst; }", "E208", "empty goal"),
        ("plan { move r to dst; }\n  goal { robot_at r dst; }", "E208", "cannot follow the plan block"),
        (
            "goal { robot_at r dst; }\n  pre { robot_ready r; }\n  plan { move r to dst; }",
            "E208",
            "pre block must come before",
        ),
        ("goal { robot_at r dst; }\n  goal { robot_ready r; }\n  plan { move r to dst; }", "E208", "at most one goal"),
        ("goal { robot_at r dst; }", "E208", "must have a plan"),
        ("goal robot_at r dst;\n  plan { move r to dst; }", "E201", "'{' after 'goal'"),
        ("goal { robot_at r dst }\n  plan { move r to dst; }", "E202", "missing ';'"),
    ],
)
def test_goal_block_structure_errors(body, code, message):
    raised = error(f"robolang 1.3;\ntask t(robot r, location dst) {{\n  {body}\n}}\n")
    assert raised.code == code and message in str(raised)


@pytest.mark.parametrize(
    ("goal", "message"),
    [
        ("object_at box;", "a location name"),
        ("object_at;", "an object name after 'object_at'"),
        ("holding r;", "an object name"),
        ("robot_at ;", "a robot name after 'robot_at'"),
        ("sensor_ok ;", "a sensor parameter name after 'sensor_ok'"),
        ("sensor_ok 3;", "a sensor parameter name"),
    ],
)
def test_atom_syntax_errors(goal, message):
    raised = error(task(goal=goal))
    assert raised.code == "E201" and message in str(raised)


def test_unknown_condition_lists_the_1_3_atoms_and_1_2_lists_its_own():
    assert "'object_at', 'robot_at', 'holding'" in str(error(task(goal="flying r;")))
    old = str(error(task(goal=None, version="1.2", params="robot r", pre="flying r;")))
    assert "'object_at'" not in old and "expected one of 'robot_ready'" in old


def test_inspect_without_using_names_the_1_3_form():
    raised = error(task(plan="inspect r box;"))
    assert raised.code == "E209" and "'using <sensor>'" in str(raised)
    assert 'using "<sensor>"' in str(
        error(task(pre=None, goal=None, version="1.2", params="robot r, object box", plan="inspect r box;"))
    )


def test_combined_conditions_work_in_goal_and_keep_the_depth_bound():
    t = only_task(task(goal="object_at box dst and not holding r box;"))
    assert isinstance(t.goal.conditions[0], ast.And)
    assert where(task(goal="robot_ready r and robot_at r dst or robot_ready r;"))[0] == "E214"
    assert where(task(goal="not not not not not not not not robot_at r dst and robot_ready r;"))[0] == "E215"


def test_message_received_is_still_deferred():
    assert where(task(goal='message_received "x";'))[0] == "E211"


# -------- static rules (spec 4) --------


@pytest.mark.parametrize(
    ("goal", "code"),
    [
        ("object_at box dst;", None),
        ("object_at r dst;", "E304"),
        ("object_at box box;", "E304"),
        ("object_at box nowhere;", "E303"),
        ("object_at nowhere dst;", "E303"),
        ("robot_at box dst;", "E304"),
        ("robot_at r cell;", "E304"),
        ("holding box r;", "E304"),
        ("holding r dst;", "E304"),
        ("holding r nowhere;", "E303"),
        ("sensor_ok cam;", None),
        ("sensor_ok r;", "E304"),
        ("sensor_ok nowhere;", "E303"),
        ("robot_ready cam;", "E304"),
        ("not (robot_ready r or object_at r dst);", "E304"),
    ],
)
@pytest.mark.parametrize("block", ["pre", "goal", "wait"])
def test_atoms_are_type_checked_in_pre_goal_and_wait_until(block, goal, code):
    kwargs = {"goal": "robot_at r dst;", "plan": "move r to dst;"}
    if block == "pre":
        kwargs["pre"] = goal
    elif block == "goal":
        kwargs["goal"] = goal
    else:
        kwargs["plan"] = f"wait until {goal.rstrip(';')} timeout 1s;"
    if code is None:
        assert compile_source(task(**kwargs)).ir.language_version == "1.3"
    else:
        assert error(task(**kwargs)).code == code


def test_a_sensor_parameter_cannot_be_used_as_anything_else():
    assert where(task(plan="move cam to dst;"))[0] == "E304"
    assert where(task(plan="grasp r cam;"))[0] == "E304"
    assert where(task(plan="inspect r box using r;"))[0] == "E304"
    assert where(task(plan="inspect r box using nowhere;"))[0] == "E303"


@pytest.mark.parametrize(
    "fragment",
    ["pre", "goal", "wait", "inspect"],
)
def test_string_sensor_is_e310_everywhere_and_the_message_names_the_fix(fragment):
    cases = {
        "pre": task(pre='sensor_ok "camera_top";'),
        "goal": task(goal='sensor_ok "camera_top";'),
        "wait": task(plan='wait until not sensor_ok "camera_top" timeout 1s;'),
        "inspect": task(plan='inspect r box using "camera_top";'),
    }
    raised = error(cases[fragment])
    assert raised.code == "E310" and "declare a 'sensor' parameter" in str(raised)


def test_empty_string_sensor_is_e310_not_e307():
    assert where(task(pre='sensor_ok "";'))[0] == "E310"
    assert where(task(goal=None, version="1.2", params="robot r", pre='sensor_ok "";'))[0] == "E307"


def test_missing_goal_is_reported_at_the_task_name_for_every_task():
    source = task(goal="robot_at r dst;", plan="move r to dst;") + task(goal=None, version=None).replace(
        "task t", "task u"
    )
    assert where(source) == ("E309", 6, 6)
    assert "RoboLang 1.3 requires 'goal" in str(error(task(goal=None)))


def test_goal_is_not_required_before_1_3():
    for version in (None, "1.1", "1.2"):
        assert (
            check(task(pre=None, goal=None, version=version, params="robot r, location dst")).program.tasks[0].goal
            is None
        )


def test_contradictory_goals_are_well_formed():
    assert compile_source(task(goal="robot_at r dst; not robot_at r dst; robot_at r dst;")).ir


# -------- IR (ir-schema-v1.2.md) --------


def goal_ir(goal: str) -> list[dict]:
    return to_dict(compile_source(task(goal=goal)).ir.tasks[0].goal)


def test_new_condition_kinds_and_key_order():
    assert goal_ir("object_at box dst;") == [{"kind": "object_at", "object": "box", "location": "dst"}]
    assert goal_ir("robot_at r src;") == [{"kind": "robot_at", "robot": "r", "location": "src"}]
    assert goal_ir("holding r box;") == [{"kind": "holding", "robot": "r", "object": "box"}]
    assert [list(c) for c in goal_ir("object_at box dst; robot_at r src; holding r box; sensor_ok cam;")] == [
        ["kind", "object", "location"],
        ["kind", "robot", "location"],
        ["kind", "robot", "object"],
        ["kind", "sensor"],
    ]


def test_task_key_order_and_goal_placement():
    document = to_dict(compile_source(task(pre="robot_ready r;")).ir)
    assert list(document["tasks"][0]) == ["name", "parameters", "safety", "preconditions", "goal", "steps"]
    assert {"name": "cam", "type": "sensor"} in document["tasks"][0]["parameters"]


@pytest.mark.parametrize("version", [None, "1.1", "1.2"])
def test_earlier_tasks_have_no_goal_key(version):
    document = to_dict(compile_source(task(pre=None, goal=None, version=version, params="robot r, location dst")).ir)
    assert list(document["tasks"][0]) == ["name", "parameters", "safety", "preconditions", "steps"]
    assert '"goal"' not in to_json(
        compile_source(task(pre=None, goal=None, version=version, params="robot r, location dst")).ir
    )


def test_sensor_reference_in_ir_is_the_parameter_name():
    steps = to_dict(compile_source(task(pre="sensor_ok cam;", plan="inspect r box using cam;")).ir.tasks[0])
    assert steps["preconditions"] == [{"kind": "sensor_ok", "sensor": "cam"}]
    assert steps["steps"][0]["sensor"] == "cam"


def test_goal_conditions_are_flattened_like_pre():
    nested = goal_ir("(robot_ready r and object_at box dst) and holding r box;")
    flat = goal_ir("robot_ready r and object_at box dst and holding r box;")
    assert nested == flat and [c["kind"] for c in flat[0]["conditions"]] == ["robot_ready", "object_at", "holding"]


def test_layout_does_not_change_the_ir():
    spaced = task(goal="object_at   box\n dst ;  // comment\n")
    assert to_json(compile_source(spaced).ir) == to_json(compile_source(task(goal="object_at box dst;")).ir)


def test_typed_sensors_flag():
    assert compile_source(task()).ir.tasks[0].typed_sensors is True
    assert (
        compile_source(task(pre=None, goal=None, version="1.2", params="robot r, location dst"))
        .ir.tasks[0]
        .typed_sensors
        is False
    )


# -------- simulation: goals and atoms --------


def sim(source: str, bindings: dict[str, str], edit=None, name: str = "t"):
    """Compile `source`, load the base world with `bindings` for task t, and run it."""
    compiled = compile_source(source).ir.tasks[0]
    world = copy.deepcopy(BASE)
    world["tasks"] = {name: {"bindings": bindings}}
    if edit:
        edit(world)
    return run(compiled, load_world(json.dumps(world).encode(), "w.json", compiled))


PICK = {"r": "arm1", "box": "box1", "src": "shelf", "dst": "table", "home": "home"}
PICK_PARAMS = "robot r, object box, location src, location dst, location home"
PICK_PLAN = "move r to src; grasp r box; move r to dst; place r box at dst;"


def pick(goal: str, plan: str = PICK_PLAN, pre: str | None = None, edit=None, bindings=None):
    return sim(task(pre=pre, goal=goal, plan=plan, params=PICK_PARAMS), bindings or PICK, edit)


def goal_values(result) -> list[str]:
    return [("not_evaluated" if g.result is None else g.result.value) for g in result.goal]


def test_goal_met_succeeds_and_records_the_conditions():
    result = pick("object_at box dst; robot_at r dst;")
    assert result.state == "SUCCEEDED" and result.failure is None
    assert goal_values(result) == ["true", "true"]
    assert [g.index for g in result.goal] == [1, 2]


def test_goal_false_fails_with_s301_in_phase_goal():
    result = pick("object_at box src;")
    assert (result.state, result.failure.code, result.failure.phase) == ("FAILED", "S301", "goal")
    assert (result.failure.step, result.failure.precondition, result.failure.safety_limit) == (None, None, None)
    assert result.states[-2:] == ("RUNNING", "FAILED")
    assert all(s.status == "succeeded" for s in result.steps)


def test_a_held_object_is_not_at_any_location():
    result = pick("object_at box dst;", plan="move r to src; grasp r box; move r to dst;")
    assert result.failure.code == "S301" and goal_values(result) == ["false"]
    assert pick("object_at box src;", plan="move r to src; grasp r box;").failure.code == "S301"
    held = pick("holding r box;", plan="move r to src; grasp r box;")
    assert held.state == "SUCCEEDED"


def test_holding_is_false_after_place_and_object_at_is_true():
    result = pick("not holding r box; object_at box dst;")
    assert result.state == "SUCCEEDED" and goal_values(result) == ["true", "true"]


def test_robot_at_follows_moves():
    assert pick("robot_at r home;", plan="wait for 1 s;").state == "SUCCEEDED"
    assert pick("robot_at r dst;", plan="move r to src;").failure.code == "S301"


def test_unknown_presence_makes_object_at_and_holding_unknown():
    unknown = {"r": "arm1", "box": "ghost", "src": "shelf", "dst": "table", "home": "home"}
    for goal in ("object_at box dst;", "holding r box;"):
        result = pick(goal, plan="move r to dst;", bindings=unknown)
        assert (result.failure.code, goal_values(result)) == ("S302", ["unknown"])
    negated = pick("not object_at box dst;", plan="move r to dst;", bindings=unknown)
    assert negated.failure.code == "S302"


def test_absent_object_is_false_not_unknown():
    absent = lambda world: world["objects"].__setitem__("box1", {"present": False})  # noqa: E731
    for goal in ("object_at box src;", "holding r box;"):
        result = pick(goal, plan="move r to dst;", edit=absent)
        assert (result.failure.code, goal_values(result)) == ("S301", ["false"])


def test_false_beats_unknown_and_unknown_beats_true():
    unknown = {"r": "arm1", "box": "ghost", "src": "shelf", "dst": "table", "home": "home"}
    both = pick("object_at box dst; robot_at r src;", plan="move r to dst;", bindings=unknown)
    assert (both.failure.code, goal_values(both)) == ("S301", ["unknown", "false"])
    mixed = pick("robot_at r dst; object_at box dst;", plan="move r to dst;", bindings=unknown)
    assert (mixed.failure.code, goal_values(mixed)) == ("S302", ["true", "unknown"])


def test_kleene_logic_applies_inside_goal_conditions():
    unknown = {"r": "arm1", "box": "ghost", "src": "shelf", "dst": "table", "home": "home"}
    assert pick("robot_at r dst or object_at box dst;", plan="move r to dst;", bindings=unknown).state == "SUCCEEDED"
    assert pick("robot_at r src and object_at box dst;", plan="move r to dst;", bindings=unknown).failure.code == "S301"


def test_goal_sees_events_applied_by_the_final_clock():
    source = task(goal="region_clear cell;", plan="wait for 3 s;", params="robot r, region cell")
    for at, state in (("2", "SUCCEEDED"), ("5", "FAILED")):

        def edit(world, at=at):
            world["regions"]["cell"] = {"clear": False}
            world["events"] = [
                {"at": {"value": at, "unit": "s"}, "fact": "region_clear", "target": "cell", "value": True}
            ]

        assert sim(source, {"r": "arm1", "cell": "cell"}, edit).state == state


def test_goal_evaluation_changes_neither_clock_nor_events():
    result = pick(
        "robot_at r dst;",
        plan="wait for 3 s;",
        edit=lambda w: w["events"].append(
            {"at": {"value": "10", "unit": "s"}, "fact": "region_clear", "target": "cell", "value": False}
        ),
    )
    assert str(result.end_t_ms) == "3000" and result.events_applied == ()


@pytest.mark.parametrize(
    ("kwargs", "edit", "expected"),
    [
        ({"pre": "robot_at r src;", "goal": "robot_at r dst;"}, None, "S101"),
        ({"plan": "grasp r box;", "goal": "robot_at r dst;"}, None, "S215"),
        ({"goal": "robot_at r dst;"}, lambda w: w["robots"]["arm1"].__setitem__("ready", False), "S201"),
    ],
)
def test_goal_is_not_evaluated_when_the_run_ended_earlier(kwargs, edit, expected):
    params = {"plan": "move r to dst;", **kwargs}
    result = pick(**params, edit=edit)
    assert result.failure.code == expected
    assert goal_values(result) == ["not_evaluated"] and all(g.result is None for g in result.goal)


def test_goal_is_not_evaluated_after_cancellation_or_authorization_failure():
    def cancel(world):
        world["tasks"]["t"]["cancel"] = {"before_step": 2}

    result = pick("robot_at r dst;", plan="move r to dst; wait for 1 s;", edit=cancel)
    assert (result.state, result.failure) == ("CANCELLED", None) and goal_values(result) == ["not_evaluated"]

    limited = sim(
        "robolang 1.3;\ntask t(robot r, location dst) {\n @safety max_speed 5 m/s;\n"
        " goal { robot_at r dst; }\n plan { move r to dst; }\n}\n",
        {"r": "arm1", "dst": "table"},
    )
    assert limited.failure.code == "S001" and goal_values(limited) == ["not_evaluated"]


def test_a_task_without_a_goal_has_no_goal_results():
    compiled = compile_source("robolang 1.2;\ntask t(robot r, location dst) {\n plan { move r to dst; }\n}\n").ir.tasks[
        0
    ]
    world = copy.deepcopy(BASE)
    world["tasks"] = {"t": {"bindings": {"r": "arm1", "dst": "table"}}}
    result = run(compiled, load_world(json.dumps(world).encode(), "w.json", compiled))
    assert result.goal is None and result.state == "SUCCEEDED"


def test_new_atoms_in_pre_and_wait_until():
    assert pick("robot_at r dst;", pre="robot_at r home; object_at box src; not holding r box;").state == "SUCCEEDED"
    failed = pick("robot_at r dst;", pre="object_at box dst; robot_at r home;")
    assert (failed.failure.code, failed.failure.precondition) == ("S101", 1)
    assert [p.result for p in failed.preconditions] == [Truth.FALSE, Truth.TRUE]
    waited = pick("robot_at r dst;", plan="wait until object_at box src timeout 5 s; move r to dst;")
    assert waited.state == "SUCCEEDED"
    timeout = pick("robot_at r dst;", plan="wait until object_at box dst timeout 5 s;")
    assert timeout.failure.code == "S241" and str(timeout.end_t_ms) == "5000"


# -------- simulation: sensor parameters --------

SENSOR_PARAMS = "robot r, object box, location dst, sensor cam"


def sensor_run(plan="inspect r box using cam;", pre=None, goal="object_exists box;", bind="camera_top", edit=None):
    source = task(pre=pre, goal=goal, plan=plan, params=SENSOR_PARAMS)
    return sim(source, {"r": "arm1", "box": "box1", "dst": "table", "cam": bind}, edit)


def test_sensor_parameter_resolves_through_the_binding():
    assert sensor_run().state == "SUCCEEDED"
    assert sensor_run(bind="camera_aux").state == "SUCCEEDED"
    dead = sensor_run(bind="camera_dead")
    assert (dead.failure.code, dead.failure.step) == ("S232", 1)
    assert sensor_run(bind="camera_unknown").failure.code == "S233"
    assert sensor_run(bind="camera_arm2").failure.code == "S231"


def test_sensor_ok_uses_the_bound_sensor():
    assert sensor_run(plan="move r to dst;", pre="sensor_ok cam;").state == "SUCCEEDED"
    assert sensor_run(plan="move r to dst;", pre="sensor_ok cam;", bind="camera_dead").failure.code == "S101"
    assert sensor_run(plan="move r to dst;", pre="sensor_ok cam;", bind="camera_unknown").failure.code == "S102"


def test_sensor_ok_in_wait_until_follows_events():
    def edit(world):
        world["events"] = [
            {"at": {"value": "2", "unit": "s"}, "fact": "sensor_available", "target": "camera_dead", "value": True}
        ]

    result = sensor_run(plan="wait until sensor_ok cam timeout 5 s;", bind="camera_dead", edit=edit)
    assert result.state == "SUCCEEDED" and str(result.end_t_ms) == "2000"


def test_parameter_name_does_not_matter_only_the_binding():
    # `cam` is also a robot's ID in this world; the binding, not the name, picks the sensor.
    def edit(world):
        world["sensors"]["arm1"] = {"robot": "arm1", "available": True}

    assert sensor_run(plan="move r to dst;", pre="sensor_ok cam;", bind="arm1", edit=edit).state == "SUCCEEDED"


@pytest.mark.parametrize(
    ("param", "target", "code"),
    [
        ("cam", "arm1", "E507"),
        ("cam", "table", "E507"),
        ("cam", "box1", "E507"),
        ("cam", "cell", "E507"),
        ("cam", "camera_nowhere", "E505"),
        ("r", "camera_top", "E507"),
        ("dst", "camera_top", "E507"),
        ("box", "camera_top", "E507"),
    ],
)
def test_binding_errors_for_sensor_parameters(param, target, code):
    with pytest.raises(WorldError) as raised:
        sensor_run(bind="camera_top", edit=lambda w: w["tasks"]["t"]["bindings"].__setitem__(param, target))
    assert raised.value.code == code


def test_before_1_3_a_sensor_still_cannot_be_bound():
    source = "robolang 1.2;\ntask t(robot r, location dst) {\n  plan { move r to dst; }\n}\n"
    compiled = compile_source(source).ir.tasks[0]
    world = copy.deepcopy(BASE)
    world["tasks"] = {"t": {"bindings": {"r": "camera_top", "dst": "table"}}}
    with pytest.raises(WorldError) as raised:
        load_world(json.dumps(world).encode(), "w.json", compiled)
    assert raised.value.code == "E505"  # unchanged from 1.1 and 1.2


def test_kind_of_recognises_sensors_after_every_entity_kind():
    compiled = compile_source(task(params=SENSOR_PARAMS, goal="robot_at r dst;")).ir.tasks[0]
    world = copy.deepcopy(BASE)
    world["tasks"] = {"t": {"bindings": {"r": "arm1", "box": "box1", "dst": "table", "cam": "camera_top"}}}
    loaded = load_world(json.dumps(world).encode(), "w.json", compiled)
    assert loaded.kind_of("camera_top") == "sensor" and loaded.kind_of("arm1") == "robot"
    assert loaded.kind_of("nowhere") is None
    assert loaded.binds("sensor", "camera_dead") and not loaded.binds("sensor", "arm1")
    assert loaded.binds("robot", "arm1") and not loaded.binds("robot", "camera_top")


def test_run_rejects_a_world_validated_for_another_task():
    compiled = compile_source(task()).ir.tasks[0]
    other = compile_source(task(goal="robot_at r src;")).ir.tasks[0]
    world = copy.deepcopy(BASE)
    world["tasks"] = {
        "t": {
            "bindings": {
                "r": "arm1",
                "box": "box1",
                "src": "shelf",
                "dst": "table",
                "cell": "cell",
                "cam": "camera_top",
            }
        }
    }
    loaded = load_world(json.dumps(world).encode(), "w.json", compiled)
    assert run(other, loaded).failure.code == "S301"  # same parameters, different goal: it runs, and misses it
    renamed = compile_source(task().replace("task t", "task u")).ir.tasks[0]
    with pytest.raises(SimulationInputError):
        run(renamed, loaded)


# -------- run record 1.2 --------


def record_of(source: str, bindings: dict, edit=None, language="1.3") -> dict:
    compiled = compile_source(source).ir.tasks[0]
    world = copy.deepcopy(BASE)
    world["tasks"] = {compiled.name: {"bindings": bindings}}
    if edit:
        edit(world)
    result = run(compiled, load_world(json.dumps(world).encode(), "w.json", compiled))
    return to_record(
        result, task=compiled, run_id="sim-x", source_sha256="a", world_sha256="b", seed="0", language_version=language
    )


def test_record_1_2_has_a_goal_after_preconditions_and_embeds_ir_1_2():
    record = record_of(task(pre="robot_ready r;", goal="object_at box dst;", plan=PICK_PLAN, params=PICK_PARAMS), PICK)
    assert record["run_record_schema_version"] == "1.2"
    assert (record["inputs"]["language_version"], record["inputs"]["ir_schema_version"]) == ("1.3", "1.2")
    keys = list(record)
    assert keys[keys.index("preconditions") + 1] == "goal"
    assert record["goal"] == [
        {"index": 1, "condition": {"kind": "object_at", "object": "box", "location": "dst"}, "result": "true"}
    ]
    assert list(record["task"]) == ["name", "parameters", "safety", "preconditions", "goal", "steps"]


def test_record_marks_unevaluated_goals_and_the_goal_failure():
    failed = record_of(task(goal="object_at box src;", plan=PICK_PLAN, params=PICK_PARAMS), PICK)
    assert failed["failure"] == {
        "code": "S301",
        "phase": "goal",
        "step": None,
        "precondition": None,
        "safety_limit": None,
        "robot": None,
    }
    assert failed["state"] == "FAILED" and failed["goal"][0]["result"] == "false"
    early = record_of(
        task(pre="robot_at r dst;", goal="robot_at r dst;", plan="move r to dst;", params=PICK_PARAMS), PICK
    )
    assert early["goal"][0]["result"] == "not_evaluated" and early["failure"]["code"] == "S101"


@pytest.mark.parametrize(("version", "schema"), [("1.1", "1.0"), ("1.2", "1.1")])
def test_earlier_records_have_no_goal_and_keep_their_schema(version, schema):
    source = f"robolang {version};\ntask t(robot r, location dst) {{\n plan {{ move r to dst; }}\n}}\n"
    record = record_of(source, {"r": "arm1", "dst": "table"}, language=version)
    assert record["run_record_schema_version"] == schema and "goal" not in record
    assert '"goal"' not in json.dumps(record)


def test_run_id_derivation_is_unchanged_by_the_language():
    from robolang.sim import run_id

    assert run_id(b"s", b"w", "t", "0") == run_id(b"s", b"w", "t", "0")
    assert re.fullmatch(r"sim-[0-9a-f]{64}", run_id(b"s", b"w", "t", "0"))


# -------- command line --------


def test_example_simulates_to_succeeded_with_the_goal_met(tmp_path):
    code, out, err = cli(["simulate", "examples/pick_and_place_v13.rob", "--world", str(V13 / "worlds" / "base.json")])
    assert code == 0 and "SUCCEEDED" in err
    record = json.loads(out)
    assert [g["result"] for g in record["goal"]] == ["true"]
    assert record["task"]["safety"][0]["limit"] == {"value": "0.5", "unit": "m/s"}


def test_simulate_reports_the_goal_failure_code_on_stderr():
    code, _, err = cli(
        ["simulate", str(V13 / "sources" / "goal_held.rob"), "--world", str(V13 / "worlds" / "base.json")]
    )
    assert code == 4 and "FAILED (S301)" in err
    code, _, err = cli(
        ["simulate", str(V13 / "sources" / "goal_unknown.rob"), "--world", str(V13 / "worlds" / "base.json")]
    )
    assert code == 4 and "FAILED (S302)" in err


def test_validate_names_the_version_and_reports_e310(tmp_path):
    code, out, _ = cli(["validate", "examples/pick_and_place_v13.rob"])
    assert code == 0 and "valid RoboLang 1.3 (1 task)" in out
    bad = tmp_path / "old.rob"
    bad.write_text(task(pre='sensor_ok "camera_top";'), encoding="utf-8")
    code, _, err = cli(["validate", str(bad)])
    assert code == 1 and "error[E310]" in err


def test_inspect_renders_goal_and_unquoted_sensors(tmp_path):
    path = tmp_path / "t.rob"
    path.write_text(
        task(
            pre="sensor_ok cam;",
            goal="object_at box dst; holding r box; robot_at r src;",
            plan="inspect r box using cam;",
        ),
        encoding="utf-8",
    )
    _, out, _ = cli(["inspect", str(path)])
    assert "IR schema 1.2" in out
    assert "    sensor_ok cam\n" in out and "using cam" in out and 'using "' not in out
    assert "  goal:\n    object_at box dst\n    holding r box\n    robot_at r src\n" in out
    assert out.index("preconditions:") < out.index("goal:") < out.index("plan:")


def test_inspect_of_earlier_files_is_unchanged(tmp_path):
    path = tmp_path / "t.rob"
    path.write_text(
        "robolang 1.2;\ntask t(robot r, object box) {\n"
        ' pre { sensor_ok "cam"; }\n plan { inspect r box using "cam"; }\n}\n',
        encoding="utf-8",
    )
    _, out, _ = cli(["inspect", str(path)])
    assert 'sensor_ok "cam"' in out and 'using "cam"' in out and "goal" not in out


def test_inspect_json_of_a_1_3_file_has_the_goal():
    _, out, _ = cli(["inspect", "examples/pick_and_place_v13.rob", "--json"])
    document = json.loads(out)
    assert (document["ir_schema_version"], document["language_version"]) == ("1.2", "1.3")
    assert document["tasks"][0]["goal"] == [{"kind": "object_at", "object": "box", "location": "dst"}]


def test_version_lists_all_languages_and_schemas():
    from robolang import cli as cli_module

    assert cli_module.version_text().endswith("(language 1.1, 1.2 and 1.3, IR schema 1.0, 1.1 and 1.2)")
    assert cli_module._joined(["1.1"]) == "1.1" and cli_module._joined(["1.1", "1.2"]) == "1.1 and 1.2"


def test_pick_and_place_example_output_is_unchanged():
    code, out, err = cli(["validate", "examples/pick_and_place.rob"])
    assert code == 0 and out.startswith("examples/pick_and_place.rob: valid RoboLang 1.1 (1 task, 1 warning)")
    assert "W401" in err
