"""Tests for the v1.1 parser and AST (src/robolang/parser.py, src/robolang/nodes.py)."""

from __future__ import annotations
import dataclasses
import random
import re
import subprocess
import sys
from decimal import Decimal
from pathlib import Path

import pytest

from robolang import RoboLangError, parse, parse_file
from robolang import nodes as ast
from robolang.nodes import Span

REPO_ROOT = Path(__file__).resolve().parent.parent
FIXTURES = Path(__file__).parent / "fixtures"

HEAD = "task t(robot r, object box, location src, location dst, region cell) {\n"


def task(plan: str = "move r to src;", pre: str | None = None, safety: str = "") -> str:
    """One task on known lines: line 1 header, then safety, pre and plan one line each."""
    lines = [HEAD.rstrip("\n")]
    if safety:
        lines.append(f"    {safety}")
    if pre is not None:
        lines.append(f"    pre {{ {pre} }}")
    lines.append(f"    plan {{ {plan} }}")
    lines.append("}")
    return "\n".join(lines) + "\n"


def only_task(source: str) -> ast.Task:
    (parsed,) = parse(source).program.tasks
    return parsed


def error(source: str) -> RoboLangError:
    with pytest.raises(RoboLangError) as raised:
        parse(source, "t.rob")
    return raised.value


# -------- AST shape --------

def test_golden_ast_with_spans():
    source = (
        "task go(robot r, location dst) {\n"
        "    plan {\n"
        "        move r to dst;\n"
        "    }\n"
        "}\n"
    )

    expected_task = ast.Task(
        name=ast.Name("go", Span(1, 6, 1, 8)),
        params=(
            ast.Param(ast.ParamType.ROBOT, ast.Name("r", Span(1, 15, 1, 16)), Span(1, 9, 1, 16)),
            ast.Param(ast.ParamType.LOCATION, ast.Name("dst", Span(1, 27, 1, 30)), Span(1, 18, 1, 30)),
        ),
        annotations=(),
        pre=None,
        plan=ast.PlanBlock(
            statements=(
                ast.Move(
                    robot=ast.Name("r", Span(3, 14, 3, 15)),
                    target=ast.Name("dst", Span(3, 19, 3, 22)),
                    span=Span(3, 9, 3, 23),
                ),
            ),
            span=Span(2, 5, 4, 6),
        ),
        span=Span(1, 1, 5, 2),
    )
    result = parse(source)

    assert result.program == ast.Program(None, (expected_task,), Span(1, 1, 5, 2))
    assert result.warnings == ()


def test_every_statement_kind():
    parsed = only_task(task(
        'move r to src; grasp r box; place r box at dst; inspect r box using "camera_top"; '
        'wait for 2.5s; wait until region_clear cell timeout 30s; communicate r to "fleet" with "DONE";'
    ))

    move, grasp, place, inspect, wait_for, wait_until, communicate = parsed.plan.statements
    assert (move.robot.text, move.target.text) == ("r", "src")
    assert (grasp.robot.text, grasp.obj.text) == ("r", "box")
    assert (place.robot.text, place.obj.text, place.target.text) == ("r", "box", "dst")
    assert (inspect.robot.text, inspect.obj.text, inspect.sensor.value) == ("r", "box", "camera_top")
    assert (wait_for.duration.value, wait_for.duration.unit) == (Decimal("2.5"), ast.DurationUnit.S)
    assert isinstance(wait_until.condition, ast.RegionClear)
    assert wait_until.condition.region.text == "cell"
    assert (wait_until.timeout.value, wait_until.timeout.unit) == (Decimal("30"), ast.DurationUnit.S)
    assert (communicate.robot.text, communicate.channel.value, communicate.message.value) == ("r", "fleet", "DONE")
    assert [type(s) for s in parsed.plan.statements] == [
        ast.Move, ast.Grasp, ast.Place, ast.Inspect, ast.WaitFor, ast.WaitUntil, ast.Communicate,
    ]


def test_every_condition_kind():
    parsed = only_task(task(pre='robot_ready r; region_clear cell; object_exists box; sensor_ok "cam";'))

    ready, clear, exists, sensor = parsed.pre.conditions
    assert (type(ready), ready.robot.text) == (ast.RobotReady, "r")
    assert (type(clear), clear.region.text) == (ast.RegionClear, "cell")
    assert (type(exists), exists.obj.text) == (ast.ObjectExists, "box")
    assert (type(sensor), sensor.sensor.value) == (ast.SensorOk, "cam")


def test_parameter_types_and_order():
    parsed = only_task(task())

    assert [(p.type, p.name.text) for p in parsed.params] == [
        (ast.ParamType.ROBOT, "r"),
        (ast.ParamType.OBJECT, "box"),
        (ast.ParamType.LOCATION, "src"),
        (ast.ParamType.LOCATION, "dst"),
        (ast.ParamType.REGION, "cell"),
    ]


def test_task_without_parameters():
    parsed = only_task("task pause() { plan { wait for 1s; } }")
    assert parsed.params == ()


def test_absent_pre_and_empty_pre_are_distinct():
    assert only_task(task()).pre is None
    assert only_task(task(pre="")).pre.conditions == ()


def test_multiple_tasks_keep_source_order():
    program = parse(task() + task().replace("task t(", "task u(")).program
    assert [t.name.text for t in program.tasks] == ["t", "u"]


def test_nodes_are_immutable():
    parsed = only_task(task())
    with pytest.raises(dataclasses.FrozenInstanceError):
        parsed.name = ast.Name("other", parsed.name.span)
    assert isinstance(parsed.params, tuple) and isinstance(parsed.plan.statements, tuple)


def test_statement_span_runs_from_keyword_through_semicolon():
    parsed = only_task(HEAD + "    plan {\n        place r box\n            at dst\n        ;\n    }\n}\n")
    assert parsed.plan.statements[0].span == Span(3, 9, 5, 10)


# -------- numbers, units, strings --------

@pytest.mark.parametrize("text, unit", [("500ms", "MS"), ("2.5s", "S"), ("1 min", "MIN"), ("0.10s", "S")])
def test_duration_keeps_value_and_unit_as_written(text, unit):
    duration = only_task(task(f"wait for {text};")).plan.statements[0].duration

    number = text.rstrip("msin ").strip()
    assert duration.value == Decimal(number)
    assert str(duration.value) == number
    assert duration.unit is ast.DurationUnit[unit]


def test_decimal_duration_is_exact():
    """The v1 runtime truncated 2.5s to 2 seconds."""
    duration = only_task(task("wait for 2.5s;")).plan.statements[0].duration
    assert duration.value == Decimal("2.5")


@pytest.mark.parametrize("text, unit", [("0.5 m/s", ast.SpeedUnit.M_PER_S), ("1.5rad/s", ast.SpeedUnit.RAD_PER_S)])
def test_max_speed_with_unit(text, unit):
    result = parse(task(safety=f"@safety max_speed {text};"))

    (annotation,) = result.program.tasks[0].annotations
    assert annotation.key.text == "max_speed"
    assert (annotation.value.value, annotation.value.unit) == (Decimal(text.rstrip("mrads/ ")), unit)
    assert result.warnings == ()


def test_unitless_max_speed_is_accepted_with_w401_and_no_unit():
    result = parse(task(safety="@safety max_speed 0.5;"), "legacy.rob")

    (annotation,) = result.program.tasks[0].annotations
    assert annotation.value.unit is None
    (warning,) = result.warnings
    assert (warning.code, warning.line, warning.column) == ("W401", 2, 23)
    assert warning.format().startswith("legacy.rob:2:23: warning[W401]: ")


def test_linear_and_angular_limits_parse_in_source_order():
    parsed = only_task(
        HEAD + "    @safety max_speed 1.0 rad/s;\n    @safety max_speed 0.5 m/s;\n    plan { move r to src; }\n}\n"
    )
    assert [a.value.unit for a in parsed.annotations] == [ast.SpeedUnit.RAD_PER_S, ast.SpeedUnit.M_PER_S]


def test_string_literal_value_is_decoded_and_span_covers_source():
    statement = only_task(task(r'communicate r to "fleet" with "said \"done\"\n";')).plan.statements[0]

    assert statement.message.value == 'said "done"\n'
    assert statement.message.span == Span(2, 42, 2, 59)


# -------- version marker --------

def test_version_marker():
    program = parse("// header\nrobolang 1.1;\n" + task()).program
    assert program.version == ast.VersionDecl("1.1", Span(2, 1, 2, 14))


def test_unmarked_file_has_no_version():
    assert parse(task()).program.version is None


@pytest.mark.parametrize("version", ["1.0", "1", "2.0", "1.10", "01.1", "01.2"])
def test_other_versions_are_rejected(version):
    raised = error(f"robolang {version};\n" + task())
    assert (raised.code, raised.line, raised.column) == ("E213", 1, 10)


def test_unsupported_version_stops_before_the_rest_of_the_file():
    """A front end must not guess at the syntax of a version it does not implement."""
    raised = error("robolang 2.0;\n# ~ whatever v2 looks like ~ #\n")
    assert raised.code == "E213"


# -------- diagnostics --------

def test_diagnostic_format():
    raised = error(task("fly r to moon;"))
    assert str(raised) == (
        "t.rob:2:12: error[E203]: unknown statement 'fly'; expected one of "
        "'move', 'grasp', 'place', 'inspect', 'wait', 'communicate'"
    )


def test_missing_semicolon_is_reported_at_the_end_of_the_statement():
    raised = error(HEAD + "    plan {\n        grasp r box\n        move r to dst;\n    }\n}\n")
    assert (raised.code, raised.line, raised.column) == ("E202", 3, 20)


@pytest.mark.parametrize("source, code, line, column", [
    (task("move r src;"), "E201", 2, 19),
    (task("movement r to src;"), "E203", 2, 12),
    (task(pre="battery_ok r;"), "E204", 2, 11),
    (HEAD.replace("object box", "gripper g") + "    plan { move r to src; }\n}\n", "E205", 1, 17),
    (task("wait for 5;"), "E206", 2, 22),
    (task("wait for 5sec;"), "E206", 2, 22),
    (task("wait for 5 m/s;"), "E206", 2, 23),
    (task(safety="@safety max_speed 2 mph;"), "E206", 2, 25),
    (task(safety="@safety max_force 10;"), "E207", 2, 13),
    (HEAD + "    plan { }\n}\n", "E208", 2, 12),
    (HEAD + "}\n", "E208", 2, 1),
    (task("inspect r box;"), "E209", 2, 25),
    (task('inspect r box with "cam";'), "E209", 2, 26),
    (task("wait until region_clear cell;"), "E209", 2, 40),
    ("// empty\n", "E210", 2, 1),
    (task("move r to src in_region cell;"), "E211", 2, 26),
    (task('wait until message_received "X" timeout 5s;'), "E211", 2, 23),
    (task(safety="@safety no_go_region cell;"), "E211", 2, 13),
    (task("move r to src; @safety max_speed 0.2; grasp r box;"), "E211", 2, 27),
    ("task inspect(robot r) { plan { wait for 1s; } }", "E212", 1, 6),
    (task("move r to timeout;"), "E212", 2, 22),
])
def test_error_code_and_position(source, code, line, column):
    raised = error(source)
    assert (raised.code, raised.line, raised.column) == (code, line, column), raised


@pytest.mark.parametrize("plan", [
    "fly r to moon; move r to src;",
    "move r to src; fly r to moon;",
])
def test_unknown_statement_rejects_the_whole_file_wherever_it_is(plan):
    assert error(task(plan)).code == "E203"


def test_missing_semicolon_after_unitless_max_speed_is_e202_not_a_unit_error():
    raised = error(HEAD + "    @safety max_speed 0.5\n    plan { move r to src; }\n}\n")
    assert (raised.code, raised.line) == ("E202", 2)


def test_keywords_are_case_sensitive():
    assert error(task("Move r to src;")).code == "E203"


@pytest.mark.parametrize("source", [
    HEAD,
    HEAD + "    plan {",
    HEAD + "    plan { move r to src;",
    HEAD + "    plan { move r to src; }",
    "task",
    "task t(",
    "task t(robot",
])
def test_truncated_input_is_a_clean_error(source):
    assert error(source).code in ("E201", "E208")


# -------- sources in the repository --------

@pytest.mark.parametrize("relative", ["examples/pick_and_place.rob", "stdlib/robolang_std_v1.rob"])
def test_existing_sources_parse_with_only_the_unit_warning(relative):
    result = parse_file(REPO_ROOT / relative)
    assert [w.code for w in result.warnings] == ["W401"]


@pytest.mark.parametrize("relative", ["examples/handover.rob", "examples/inspect_object.rob"])
def test_empty_placeholder_examples_are_not_programs(relative):
    with pytest.raises(RoboLangError) as raised:
        parse_file(REPO_ROOT / relative)
    assert raised.value.code == "E210"


def test_task_examples_in_the_specification_parse():
    """Every complete task shown in the spec (including the section 8.5 migrations) is valid v1.1."""
    spec = (REPO_ROOT / "spec" / "robolang-v1.1.md").read_text(encoding="utf-8")
    blocks = [b for b in re.findall(r"```rob\n(.*?)```", spec, re.DOTALL) if b.lstrip().startswith("task ")]

    assert len(blocks) >= 4
    for block in blocks:
        assert parse(block).warnings == ()


# -------- robustness and isolation --------

def test_mutated_input_never_crashes_the_parser():
    """Any input either parses or raises RoboLangError; nothing else escapes."""
    rng = random.Random(1101)
    alphabet = ' \n\t;{}(),"\\/@.#-_0123456789abcdefghijklmnopqrstuvwxyzM'
    seeds = [p.read_text(encoding="utf-8") for p in sorted((FIXTURES / "valid").glob("*.rob"))]

    for _ in range(3000):
        text = list(rng.choice(seeds))
        for _ in range(rng.randint(1, 4)):
            position = rng.randrange(len(text) + 1)
            action = rng.random()
            if action < 0.4 and text:
                del text[min(position, len(text) - 1)]
            elif action < 0.8:
                text.insert(position, rng.choice(alphabet))
            else:
                del text[position:]
        try:
            parse("".join(text))
        except RoboLangError as raised:
            assert re.fullmatch(r"E[12]\d\d", raised.code)
            assert raised.line >= 1 and raised.column >= 1


def test_front_end_does_not_load_the_interpreter_or_adapters():
    code = (
        "import sys; sys.path.insert(0, 'src'); import robolang; "
        "robolang.parse_file('examples/pick_and_place.rob'); "
        "loaded = [m for m in sys.modules if m.startswith('robolang_') or m == 'rclpy']; "
        "assert not loaded, loaded"
    )
    result = subprocess.run([sys.executable, "-c", code], cwd=REPO_ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_lexical_error_anywhere_precedes_a_syntax_error():
    """Stages run in order: the '#' on line 3 is reported, not the unknown statement on line 2."""
    raised = error(HEAD + "    plan { fly r to moon; }\n} #\n")
    assert (raised.code, raised.line) == ("E101", 3)


def test_unsupported_version_is_reported_before_a_lexical_error_right_after_it():
    assert error("robolang 2.0 # no semicolon, then junk").code == "E213"
