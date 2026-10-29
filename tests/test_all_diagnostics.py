"""validate reports every error, not just the first (spec/decisions/0015-report-all-diagnostics.md)."""

from __future__ import annotations
import io

import pytest

from robolang import RoboLangError, check, parse
from robolang.cli import EXIT_INVALID, main
from robolang.diagnostics import MAX_ERRORS


def errors(source: str) -> list[tuple[str, int]]:
    with pytest.raises(RoboLangError) as raised:
        check(source)
    return [(d.code, d.line) for d in raised.value.diagnostics]


def cli(source: str, tmp_path, command: str = "validate") -> tuple[int, str, str]:
    path = tmp_path / "t.rob"
    path.write_text(source, encoding="utf-8")
    out, err = io.StringIO(), io.StringIO()
    code = main([command, str(path)], out=out, err=err)
    return code, out.getvalue(), err.getvalue().replace(str(path), "FILE")


THREE = """\
task t(robot r, object o) {
    plan {
        move r to x;
        grasp o r;
        place r o at y;
    }
}
"""


def test_three_errors_in_one_task_are_all_reported():
    assert errors(THREE) == [("E303", 3), ("E304", 4), ("E303", 5)]


def test_diagnostic_is_the_first_and_diagnostics_are_all_in_order():
    with pytest.raises(RoboLangError) as raised:
        check(THREE)
    assert raised.value.diagnostic is raised.value.diagnostics[0]
    assert raised.value.code == "E303" and raised.value.line == 3
    assert not raised.value.truncated


def test_errors_across_tasks_in_source_order():
    source = (
        "task a(robot r) { plan { move r to x; } }\n"
        "task a(robot r) { plan { wait for 0s; } }\n"
        "task c(robot r) { plan { communicate r to \"\" with \"hi\"; } }\n"
    )
    assert errors(source) == [("E303", 1), ("E301", 2), ("E305", 2), ("E307", 3)]


def test_annotation_and_parameter_errors_are_all_reported():
    source = (
        "task t(robot r, robot r) {\n"
        "  @safety max_speed 1 m/s;\n"
        "  @safety max_speed 2 m/s;\n"
        "  @safety max_speed 3;\n"
        "  plan { wait for 1s; }\n"
        "}\n"
    )
    assert errors(source) == [("E302", 1), ("E306", 3), ("E308", 4)]


def test_syntax_errors_in_different_statements_are_all_reported():
    source = "task t(robot r) { plan { fly r; move r; wait for 5; } }\n"
    assert errors(source) == [("E203", 1), ("E201", 1), ("E206", 1)]


def test_syntax_errors_in_different_tasks_are_all_reported():
    source = (
        "task a(robot r) { plan { fly r; } }\n"
        "task b(robot r { plan { wait for 1s; } }\n"
        "task c(robot r) { plan { wait for 1s; } }\n"
        "task d(robot r) { plan { grasp r; } }\n"
    )
    assert errors(source) == [("E203", 1), ("E201", 2), ("E201", 4)]


def test_a_missing_semicolon_does_not_swallow_the_next_statement():
    source = "task t(robot r, object o) { plan { move r to x\n grasp r o\n wait for 1s; } }\n"
    assert errors(source) == [("E202", 1), ("E202", 2)]


def test_syntax_error_in_one_task_and_static_error_in_another():
    source = (
        "task a(robot r) { plan { move r to nowhere; } }\n"
        "task b(robot r) { plan { fly r; } }\n"
    )
    assert errors(source) == [("E303", 1), ("E203", 2)]


def test_a_task_with_a_syntax_error_is_not_validated():
    source = "task a(robot r) { plan { fly r; move r to nowhere; } }\n"
    assert errors(source) == [("E203", 1)]


def test_parse_alone_reports_every_syntax_error():
    with pytest.raises(RoboLangError) as raised:
        parse("task a(robot r) { plan { fly r; hop r; } }\n")
    assert [d.code for d in raised.value.diagnostics] == ["E203", "E203"]


def test_a_lexical_error_is_fatal_and_alone():
    assert errors("task a(robot r) { plan { fly r; # } }\n") == [("E101", 1)]


def test_undeclared_name_does_not_also_report_a_type_error():
    assert errors("task t(robot r) { plan { grasp r missing; } }\n") == [("E303", 1)]


def test_a_duplicate_parameter_does_not_blame_its_uses():
    assert errors("task t(robot r, robot r) { plan { move r to r; } }\n") == [("E302", 1), ("E304", 1)]


def test_one_error_per_statement():
    assert errors('task t(robot r) { plan { communicate r to "" with ""; } }\n') == [("E307", 1)]


def test_errors_are_capped_with_a_too_many_note():
    source = "task t(robot r) { plan {" + " move r to x;" * (MAX_ERRORS + 10) + " } }\n"
    with pytest.raises(RoboLangError) as raised:
        check(source)
    assert len(raised.value.diagnostics) == MAX_ERRORS
    assert raised.value.truncated


def test_exactly_the_cap_is_not_truncated():
    source = "task t(robot r) { plan {" + " move r to x;" * MAX_ERRORS + " } }\n"
    with pytest.raises(RoboLangError) as raised:
        check(source)
    assert len(raised.value.diagnostics) == MAX_ERRORS
    assert not raised.value.truncated


def test_syntax_errors_are_capped_too():
    source = "task t(robot r) { plan {" + " fly r;" * (MAX_ERRORS + 10) + " } }\n"
    with pytest.raises(RoboLangError) as raised:
        parse(source)
    assert len(raised.value.diagnostics) == MAX_ERRORS and raised.value.truncated


# -------- command line --------

def test_validate_prints_every_error_and_a_count(tmp_path):
    code, out, err = cli(THREE, tmp_path)

    assert (code, out) == (EXIT_INVALID, "")
    lines = err.splitlines()
    assert [line.split(": ")[1][:11] for line in lines[:3]] == ["error[E303]", "error[E304]", "error[E303]"]
    assert lines[3] == "FILE: not valid (3 errors)"


def test_validate_counts_warnings_and_prints_them_in_order(tmp_path):
    source = "task t(robot r) {\n  @safety max_speed 1;\n  plan { move r to x; fly r; }\n}\n"
    code, _, err = cli(source, tmp_path)

    assert code == EXIT_INVALID
    assert err.splitlines()[-1] == "FILE: not valid (1 error, 1 warning)"
    assert "warning[W401]" in err.splitlines()[0]


def test_validate_says_when_there_are_too_many_errors(tmp_path):
    source = "task t(robot r) { plan {" + " move r to x;" * (MAX_ERRORS + 5) + " } }\n"
    code, _, err = cli(source, tmp_path)

    assert code == EXIT_INVALID
    lines = err.splitlines()
    assert len(lines) == MAX_ERRORS + 2
    assert lines[-2] == f"FILE: too many errors: stopped after {MAX_ERRORS}"
    assert lines[-1] == f"FILE: not valid ({MAX_ERRORS}+ errors)"


def test_a_single_error_prints_exactly_one_line(tmp_path):
    code, out, err = cli("task t(robot r) { plan { move r to x; } }\n", tmp_path)

    assert (code, out) == (EXIT_INVALID, "")
    assert err.count("\n") == 1 and err.startswith("FILE:1:")


@pytest.mark.parametrize("command", ["inspect", "simulate"])
def test_inspect_and_simulate_print_every_error(command, tmp_path):
    path = tmp_path / "t.rob"
    path.write_text(THREE, encoding="utf-8")
    argv = [command, str(path)] + (["--world", str(path)] if command == "simulate" else [])
    out, err = io.StringIO(), io.StringIO()

    assert main(argv, out=out, err=err) == EXIT_INVALID
    assert out.getvalue() == ""
    assert err.getvalue().count("error[") == 3
    assert "not valid" not in err.getvalue()


def test_a_missing_goal_does_not_stop_the_plan_from_being_checked():
    source = "robolang 1.3;\ntask t(robot r, location l) {\n    plan {\n        move r to x;\n    }\n}\n"
    assert errors(source) == [("E309", 2), ("E303", 4)]
