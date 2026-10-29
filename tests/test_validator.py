"""Tests for static validation (src/robolang/validator.py; spec section 5, E301-E308)."""

from __future__ import annotations
import random
import re
import subprocess
import sys
from pathlib import Path

import pytest

from robolang import RoboLangError, check, check_file, parse, validate

REPO_ROOT = Path(__file__).resolve().parent.parent
FIXTURES = Path(__file__).parent / "fixtures"

PARAMS = "robot r, object box, location src, location dst, region cell"
MARK = "§"


def task(plan: str = "move r to src;", pre: str | None = None, safety: str = "", name: str = "t",
         params: str = PARAMS) -> str:
    lines = [f"task {name}({params}) {{"]
    lines += [f"    {line}" for line in safety.splitlines()]
    if pre is not None:
        lines.append(f"    pre {{ {pre} }}")
    lines.append(f"    plan {{ {plan} }}")
    lines.append("}")
    return "\n".join(lines) + "\n"


def rejected(marked: str) -> RoboLangError:
    """
    `marked` contains one § directly before the token where the error must be
    reported. Checks that position and returns the error.
    """
    before, _, after = marked.partition(MARK)
    assert after, "source has no position marker"
    source = before + after
    line = before.count("\n") + 1
    column = len(before) - (before.rfind("\n") + 1) + 1

    parse(source)  # the source is syntactically valid
    with pytest.raises(RoboLangError) as raised:
        check(source, "t.rob")
    assert (raised.value.line, raised.value.column) == (line, column), raised.value
    return raised.value


def accepted(source: str):
    return check(source)


# -------- E301 duplicate task name --------

def test_e301_duplicate_task_name_is_reported_at_the_later_task():
    error = rejected(task(name="go") + task(name="stop") + task(name=f"{MARK}go"))
    assert error.code == "E301"
    assert "first defined on line 1" in str(error)


def test_task_names_are_case_sensitive():
    accepted(task(name="go") + task(name="Go"))


def test_same_parameter_names_in_different_tasks_are_independent():
    accepted(task(name="a") + task(name="b"))


# -------- E302 duplicate parameter --------

def test_e302_duplicate_parameter_is_reported_at_the_later_one():
    assert rejected(task(params=f"robot r, location dst, location {MARK}dst", plan="move r to dst;")).code == "E302"


def test_e302_applies_across_types():
    assert rejected(task(params=f"robot r, location r2, object {MARK}r", plan="wait for 1s;")).code == "E302"


def test_parameter_may_share_the_task_name():
    accepted(task(name="r"))


def test_unused_parameters_are_allowed():
    accepted(task(plan="wait for 1s;"))


def test_task_without_parameters_or_names():
    accepted("task pause() { plan { wait for 1s; } }")


# -------- E303 undeclared name --------

@pytest.mark.parametrize("plan", [
    f"move {MARK}x to src;",
    f"move r to {MARK}x;",
    f"grasp {MARK}x box;",
    f"grasp r {MARK}x;",
    f"place {MARK}x box at dst;",
    f"place r {MARK}x at dst;",
    f"place r box at {MARK}x;",
    f'inspect {MARK}x box using "cam";',
    f'inspect r {MARK}x using "cam";',
    f'communicate {MARK}x to "fleet" with "DONE";',
    f"wait until robot_ready {MARK}x timeout 5s;",
    f"wait until region_clear {MARK}x timeout 5s;",
    f"wait until object_exists {MARK}x timeout 5s;",
])
def test_e303_undeclared_name_in_plan(plan):
    assert rejected(task(plan)).code == "E303"


@pytest.mark.parametrize("pre", [f"robot_ready {MARK}x;", f"region_clear {MARK}x;", f"object_exists {MARK}x;"])
def test_e303_undeclared_name_in_pre(pre):
    assert rejected(task(pre=pre)).code == "E303"


def test_names_are_local_to_their_task():
    source = task(name="a", params="robot r, location here", plan="move r to here;") + task(
        name="b", params="robot r", plan=f"move r to {MARK}here;")
    assert rejected(source).code == "E303"


def test_a_task_name_is_not_a_usable_name():
    assert rejected(task(name="home", plan=f"move r to {MARK}home;")).code == "E303"


def test_names_are_case_sensitive():
    assert rejected(task(plan=f"move {MARK}R to src;")).code == "E303"


# -------- E304 wrong type --------

@pytest.mark.parametrize("plan", [
    f"move {MARK}box to src;",
    f"move r to {MARK}cell;",
    f"move r to {MARK}box;",
    f"move {MARK}src to r;",
    f"grasp {MARK}src box;",
    f"grasp r {MARK}src;",
    f"grasp r {MARK}r;",
    f"place {MARK}cell box at dst;",
    f"place r {MARK}dst at dst;",
    f"place r box at {MARK}cell;",
    f'inspect {MARK}box box using "cam";',
    f'inspect r {MARK}cell using "cam";',
    f'communicate {MARK}cell to "fleet" with "DONE";',
    f"wait until robot_ready {MARK}box timeout 5s;",
    f"wait until region_clear {MARK}dst timeout 5s;",
    f"wait until object_exists {MARK}r timeout 5s;",
])
def test_e304_wrong_type_in_plan(plan):
    assert rejected(task(plan)).code == "E304"


@pytest.mark.parametrize("pre", [f"robot_ready {MARK}cell;", f"region_clear {MARK}src;", f"object_exists {MARK}dst;"])
def test_e304_wrong_type_in_pre(pre):
    assert rejected(task(pre=pre)).code == "E304"


def test_e304_message_names_both_types():
    error = rejected(task(f"move r to {MARK}box;"))
    assert str(error) == "t.rob:2:22: error[E304]: 'box' is an object, but 'move ... to' expects a location here"


# -------- E305 value out of range --------

@pytest.mark.parametrize("value", ["0", "0.0", "00.000"])
@pytest.mark.parametrize("template", [
    "wait for {};",
    "wait until region_clear cell timeout {};",
])
@pytest.mark.parametrize("unit", ["ms", "s", "min"])
def test_e305_zero_duration(template, value, unit):
    assert rejected(task(template.format(f"{MARK}{value}{unit}"))).code == "E305"


@pytest.mark.parametrize("value", ["0", "0.0"])
@pytest.mark.parametrize("unit", [" m/s", " rad/s", ""])
def test_e305_zero_max_speed(value, unit):
    assert rejected(task(safety=f"@safety max_speed {MARK}{value}{unit};")).code == "E305"


@pytest.mark.parametrize("value", ["0.001", "0.0000001", "1", "3600"])
def test_small_and_large_positive_values_are_accepted(value):
    accepted(task(f"wait for {value}ms; wait until region_clear cell timeout {value}min;",
                  safety=f"@safety max_speed {value} m/s;"))


# -------- E306 / E308 speed limits --------

def test_one_linear_and_one_angular_limit_in_either_order():
    accepted(task(safety="@safety max_speed 0.5 m/s;\n@safety max_speed 1.0 rad/s;"))
    accepted(task(safety="@safety max_speed 1.0 rad/s;\n@safety max_speed 0.5 m/s;"))


@pytest.mark.parametrize("first, second", [
    ("0.5 m/s", "0.5 m/s"),
    ("0.5 m/s", "0.4 m/s"),
    ("1.0 rad/s", "2.0 rad/s"),
    ("0.5", "0.4"),
])
def test_e306_duplicate_limit_is_reported_at_the_later_annotation(first, second):
    error = rejected(task(safety=f"@safety max_speed {first};\n{MARK}@safety max_speed {second};"))
    assert error.code == "E306"
    assert "first declared on line 2" in str(error)


def test_e306_with_another_unit_in_between():
    safety = f"@safety max_speed 1.0 rad/s;\n@safety max_speed 0.5 m/s;\n{MARK}@safety max_speed 2.0 rad/s;"
    assert rejected(task(safety=safety)).code == "E306"


@pytest.mark.parametrize("first, second", [
    ("0.5", "0.5 m/s"),
    ("0.5", "1.0 rad/s"),
    ("0.5 m/s", "0.5"),
    ("1.0 rad/s", "0.5"),
])
def test_e308_unitless_combined_with_explicit_is_reported_at_the_later_annotation(first, second):
    assert rejected(task(safety=f"@safety max_speed {first};\n{MARK}@safety max_speed {second};")).code == "E308"


def test_e308_when_unitless_follows_both_explicit_limits():
    safety = f"@safety max_speed 0.5 m/s;\n@safety max_speed 1.0 rad/s;\n{MARK}@safety max_speed 0.5;"
    assert rejected(task(safety=safety)).code == "E308"


def test_speed_limits_are_per_task():
    accepted(task(name="a", safety="@safety max_speed 0.5 m/s;") + task(name="b", safety="@safety max_speed 0.7 m/s;"))


def test_single_unitless_limit_is_accepted_and_keeps_its_w401_warning():
    result = accepted(task(safety="@safety max_speed 0.5;"))

    assert [(w.code, w.line) for w in result.warnings] == [("W401", 2)]
    assert result.program.tasks[0].annotations[0].value.unit is None


# -------- E307 empty string --------

@pytest.mark.parametrize("plan", [
    f'inspect r box using {MARK}"";',
    f'communicate r to {MARK}"" with "DONE";',
    f'communicate r to "fleet" with {MARK}"";',
    f'wait until sensor_ok {MARK}"" timeout 5s;',
])
def test_e307_empty_string_in_plan(plan):
    assert rejected(task(plan)).code == "E307"


def test_e307_empty_sensor_in_pre():
    assert rejected(task(pre=f'sensor_ok {MARK}"";')).code == "E307"


@pytest.mark.parametrize("text", ['" "', r'"\n"', r'"\t"', r'"\""', r'"\\"'])
def test_whitespace_and_escape_only_strings_are_not_empty(text):
    accepted(task(f'communicate r to {text} with {text}; inspect r box using {text};'))


# -------- ordering --------

def test_static_errors_are_reported_in_source_order():
    """Two violations: the undeclared name on line 2 comes before the zero wait on line 2, later column."""
    assert rejected(task(f"move r to {MARK}nowhere; wait for 0s;")).code == "E303"
    assert rejected(task(f"wait for {MARK}0s; move r to nowhere;")).code == "E305"


def test_error_in_an_earlier_task_body_precedes_a_later_duplicate_task_name():
    assert rejected(task(name="go", plan=f"move r to {MARK}nowhere;") + task(name="go")).code == "E303"


def test_pre_is_checked_before_plan_and_annotations_before_pre():
    assert rejected(task("move r to nowhere;", pre=f"region_clear {MARK}zone;")).code == "E303"
    assert rejected(task(pre="region_clear zone;", safety=f"@safety max_speed {MARK}0 m/s;")).code == "E305"


def test_conflicting_limit_is_reported_before_its_own_zero_value():
    safety = f"@safety max_speed 0.5 m/s;\n{MARK}@safety max_speed 0 m/s;"
    assert rejected(task(safety=safety)).code == "E306"


def test_syntax_errors_precede_static_errors():
    with pytest.raises(RoboLangError) as raised:
        check(task("move r to nowhere; fly r to moon;"))
    assert raised.value.code == "E203"


# -------- API --------

def test_validate_accepts_a_parsed_program_and_returns_nothing():
    assert validate(parse(task()).program) is None


def test_validate_reports_the_given_filename():
    program = parse(task("move r to nowhere;")).program
    with pytest.raises(RoboLangError) as raised:
        validate(program, "plan.rob")
    assert str(raised.value).startswith("plan.rob:2:22: error[E303]: ")


def test_check_returns_the_same_result_as_parse_for_a_valid_program():
    source = task(pre="robot_ready r;", safety="@safety max_speed 0.5;")
    assert check(source) == parse(source)


def test_validation_does_not_change_the_program():
    program = parse(task()).program
    before = repr(program)
    validate(program)
    assert repr(program) == before


# -------- sources in the repository --------

@pytest.mark.parametrize("relative", ["examples/pick_and_place.rob", "legacy/robolang_std_v1.rob"])
def test_existing_sources_pass_with_only_the_unit_warning(relative):
    assert [w.code for w in check_file(REPO_ROOT / relative).warnings] == ["W401"]


def test_task_examples_in_the_specification_pass():
    spec = (REPO_ROOT / "spec" / "robolang-v1.1.md").read_text(encoding="utf-8")
    blocks = [b for b in re.findall(r"```rob\n(.*?)```", spec, re.DOTALL) if b.lstrip().startswith("task ")]

    assert len(blocks) >= 4
    for block in blocks:
        assert check(block).warnings == ()


# -------- robustness and isolation --------

def test_mutated_input_never_crashes_the_front_end():
    """Any input either passes or raises RoboLangError with a known code; nothing else escapes."""
    rng = random.Random(1103)
    names = ["r", "box", "src", "dst", "cell", "x", "0", "0s", '""', "m/s", "rad/s", "task", ";", "robot", "region"]
    seeds = [p.read_text(encoding="utf-8") for p in sorted(FIXTURES.glob("*/*.rob"))]
    codes = set()

    for _ in range(3000):
        words = re.split(r"(\s+)", rng.choice(seeds))
        for _ in range(rng.randint(1, 3)):
            position = rng.randrange(len(words))
            if rng.random() < 0.3:
                del words[position]
            else:
                words[position] = rng.choice(names)
        try:
            check("".join(words))
        except RoboLangError as raised:
            assert re.fullmatch(r"E[123]\d\d", raised.code)
            codes.add(raised.code[:2])
    assert codes == {"E1", "E2", "E3"}


def test_complete_front_end_does_not_load_the_interpreter_or_adapters():
    code = (
        "import sys; sys.path.insert(0, 'src'); import robolang; "
        "robolang.check_file('examples/pick_and_place.rob'); "
        "loaded = [m for m in sys.modules if m.startswith('robolang_') or m == 'rclpy']; "
        "assert not loaded, loaded"
    )
    result = subprocess.run([sys.executable, "-c", code], cwd=REPO_ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
