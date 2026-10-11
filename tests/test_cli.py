"""Tests for the read-only commands (src/robolang/cli.py): validate and inspect."""

from __future__ import annotations
import io
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from robolang.cli import EXIT_INVALID, EXIT_OK, EXIT_USAGE, main

REPO_ROOT = Path(__file__).resolve().parent.parent
FIXTURES = Path(__file__).parent / "fixtures"
SNAPSHOTS = Path(__file__).parent / "snapshots" / "ir"

DEMO = "examples/pick_and_place.rob"
MULTI = "tests/fixtures/valid/multiple_tasks.rob"
MINIMAL = "tests/fixtures/valid/minimal_task.rob"


def cli(*args: str) -> tuple[int, str, str]:
    """Run in-process from the repository root with captured streams."""
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(REPO_ROOT)
    try:
        code = main(list(args), out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


def process(*args: str, launcher: str = "path", encoding: str | None = None) -> subprocess.CompletedProcess:
    """Run as a real process, either from the checkout or with `python -m robolang`."""
    env = dict(os.environ)
    env.pop("PYTHONUTF8", None)
    env.pop("PYTHONIOENCODING", None)
    env.pop("PYTHONPATH", None)
    if encoding:
        env["PYTHONIOENCODING"] = encoding
    if launcher == "path":
        command = [sys.executable, "src/robolang", *args]
    else:
        env["PYTHONPATH"] = str(REPO_ROOT / "src")
        command = [sys.executable, "-m", "robolang", *args]
    return subprocess.run(command, cwd=REPO_ROOT, env=env, capture_output=True, timeout=60)


# -------- validate --------

def test_validate_reports_a_valid_file():
    code, out, err = cli("validate", MINIMAL)

    assert code == EXIT_OK
    assert out.splitlines() == [
        f"{MINIMAL}: valid RoboLang 1.1 (1 task)",
        "Validation covers the text of the program only. It does not make a task safe to run on a robot.",
    ]
    assert err == ""


def test_validate_prints_warnings_to_stderr_and_still_succeeds():
    code, out, err = cli("validate", DEMO)

    assert code == EXIT_OK
    assert out.startswith(f"{DEMO}: valid RoboLang 1.1 (1 task, 1 warning)\n")
    assert err.startswith(f"{DEMO}:5:23: warning[W401]: ")
    assert err.count("\n") == 1


def test_validate_counts_tasks():
    assert cli("validate", MULTI)[1].startswith(f"{MULTI}: valid RoboLang 1.1 (2 tasks)\n")


@pytest.mark.parametrize("fixture, diagnostic", [
    ("e101_unexpected_character", ":7:24: error[E101]: unexpected character '#'"),
    ("e203_unknown_statement", ":8:9: error[E203]: unknown statement 'fly'"),
    ("e303_undeclared_name", ":7:19: error[E303]: undeclared name 'nowhere'"),
])
def test_validate_reports_one_diagnostic_on_stderr_and_fails(fixture, diagnostic):
    path = f"tests/fixtures/invalid/{fixture}.rob"
    code, out, err = cli("validate", path)

    assert code == EXIT_INVALID
    assert out == ""
    assert err.startswith(path + diagnostic)
    assert err.count("\n") == 1


@pytest.mark.parametrize("path", [pytest.param(p, id=p.stem) for p in sorted((FIXTURES / "invalid").glob("*.rob"))])
def test_every_invalid_fixture_fails_both_commands_with_its_code(path):
    expected = path.stem.split("_")[0].upper()
    relative = path.relative_to(REPO_ROOT).as_posix()

    for command in (["validate", relative], ["inspect", relative], ["inspect", relative, "--json"]):
        code, out, err = cli(*command)
        assert (code, out) == (EXIT_INVALID, "")
        assert f"error[{expected}]" in err


@pytest.mark.parametrize("path", [pytest.param(p, id=p.stem) for p in sorted((FIXTURES / "valid").glob("*.rob"))])
def test_every_valid_fixture_passes_both_commands(path):
    relative = path.relative_to(REPO_ROOT).as_posix()

    assert cli("validate", relative)[0] == EXIT_OK
    assert cli("inspect", relative)[0] == EXIT_OK


# -------- inspect --------

def test_inspect_text_summary():
    code, out, err = cli("inspect", DEMO)

    assert code == EXIT_OK
    assert out == f"""\
{DEMO}
RoboLang 1.1 | IR schema 1.0 | 1 task

task demo_pick_and_place
  parameters:
    robot r
    object box
    location src
    location dst
    region cell
  safety:
    max_speed 0.5  (no unit: deprecated; declared only, never treated as an enforced limit)
  preconditions:
    robot_ready r
    region_clear cell
  plan:
    1. move r to src
    2. grasp r box
    3. move r to dst
    4. place r box at dst
    5. communicate r to "fleet" with "TASK_COMPLETE"

This is a description only. Nothing was executed.
"""
    assert "warning[W401]" in err


def test_inspect_marks_empty_sections_and_declared_limits():
    out = cli("inspect", "tests/fixtures/valid/max_speed_linear_and_angular.rob")[1]

    assert "    max_speed 0.5 m/s  (declared only; not enforced by this tool)\n" in out
    assert "    max_speed 1 rad/s  (declared only; not enforced by this tool)\n" in out
    assert "  preconditions: (none)\n" in out


def test_inspect_shows_every_task_when_none_is_selected():
    out = cli("inspect", MULTI)[1]

    assert "| 2 tasks\n" in out
    assert "\ntask handover\n" in out and "\ntask retreat\n" in out


def test_inspect_selected_task_only():
    code, out, _ = cli("inspect", MULTI, "--task", "retreat")

    assert code == EXIT_OK
    assert "| task retreat (1 of 2)\n" in out
    assert "\ntask retreat\n" in out and "task handover" not in out


def test_inspect_unknown_task_is_a_usage_error_listing_the_tasks():
    code, out, err = cli("inspect", MULTI, "--task", "launch")

    assert (code, out) == (EXIT_USAGE, "")
    assert err == f"robolang: error: no task named 'launch' in '{MULTI}'; tasks in this file: handover, retreat\n"


def test_inspect_json_is_the_ir_snapshot():
    code, out, err = cli("inspect", MULTI, "--json")

    assert (code, err) == (EXIT_OK, "")
    assert out == (SNAPSHOTS / "multiple_tasks.json").read_text(encoding="ascii")


def test_inspect_json_of_a_selected_task_keeps_the_document_header():
    document = json.loads(cli("inspect", MULTI, "--task", "retreat", "--json")[1])

    assert list(document) == ["ir_schema", "ir_schema_version", "language_version", "tasks"]
    assert [t["name"] for t in document["tasks"]] == ["retreat"]


def test_inspect_json_keeps_warnings_off_stdout():
    code, out, err = cli("inspect", DEMO, "--json")

    assert code == EXIT_OK
    assert json.loads(out)["tasks"][0]["safety"] == [{"kind": "max_speed", "limit": {"value": "0.5", "unit": None}}]
    assert "warning[W401]" in err


def test_selecting_a_task_still_validates_the_whole_file(tmp_path):
    """An error in another task blocks inspection of the selected one."""
    source = (
        "task good(robot r, location dst) { plan { move r to dst; } }\n"
        "task bad(robot r) { plan { move r to nowhere; } }\n"
    )
    path = tmp_path / "two.rob"
    path.write_text(source, encoding="utf-8")

    for extra in ([], ["--json"]):
        code, out, err = cli("inspect", str(path), "--task", "good", *extra)
        assert (code, out) == (EXIT_INVALID, "")
        assert ":2:38: error[E303]: undeclared name 'nowhere'" in err


def test_unknown_task_in_an_invalid_file_reports_the_invalid_file(tmp_path):
    path = tmp_path / "bad.rob"
    path.write_text("task t() { plan { fly; } }\n", encoding="utf-8")

    assert cli("inspect", str(path), "--task", "missing")[0] == EXIT_INVALID


# -------- input problems --------

def test_missing_file_is_a_usage_error():
    code, out, err = cli("validate", "no/such/file.rob")

    assert (code, out) == (EXIT_USAGE, "")
    assert err.startswith("robolang: error: cannot read 'no/such/file.rob': ")


def test_directory_is_a_usage_error():
    assert cli("validate", "examples")[0] == EXIT_USAGE


def test_file_that_is_not_utf8_is_a_usage_error(tmp_path):
    path = tmp_path / "latin1.rob"
    path.write_bytes(b'task t() { plan { wait for 1s; } } // caf\xe9\n')

    code, out, err = cli("validate", str(path))
    assert (code, out) == (EXIT_USAGE, "")
    assert err.endswith("the file is not valid UTF-8\n")


def test_empty_file_is_invalid_not_a_usage_error(tmp_path):
    path = tmp_path / "empty.rob"
    path.write_bytes(b"")

    code, _, err = cli("validate", str(path))
    assert code == EXIT_INVALID and "error[E210]" in err


def test_crlf_and_bom_input_is_accepted(tmp_path):
    path = tmp_path / "windows.rob"
    path.write_bytes(b"\xef\xbb\xbftask t() {\r\n    plan { wait for 1s; }\r\n}\r\n")

    assert cli("validate", str(path))[0] == EXIT_OK


@pytest.mark.parametrize("args", [[], ["run", DEMO], ["validate"], ["inspect"], ["validate", DEMO, "--task", "x"],
                                  ["inspect", DEMO, "--task"], ["execute", DEMO]])
def test_bad_arguments_are_usage_errors(args, capsys):
    assert cli(*args)[0] == EXIT_USAGE
    assert "usage: robolang" in capsys.readouterr().err


def test_there_is_no_command_that_runs_a_task(capsys):
    cli("--help")
    help_text = capsys.readouterr().out

    assert "validate" in help_text and "inspect" in help_text
    for word in ("run ", "execute", "simulate"):
        assert f"\n    {word}" not in help_text


# -------- help and version --------

@pytest.mark.parametrize("args", [["--help"], ["validate", "--help"], ["inspect", "--help"]])
def test_help_explains_scope_and_exit_status(args, capsys):
    code, _, _ = cli(*args)
    text = capsys.readouterr().out

    assert code == EXIT_OK
    assert "exit status:" in text
    assert "does not make a task safe to run on a robot" in text


def test_inspect_help_documents_its_options(capsys):
    cli("inspect", "--help")
    text = capsys.readouterr().out
    assert "--task NAME" in text and "--json" in text and "Nothing is executed" in text


def test_version_reports_language_and_schema_versions_separately(capsys):
    code, _, _ = cli("--version")
    assert code == EXIT_OK
    assert capsys.readouterr().out == "RoboLang language 1.1, IR schema 1.0\n"


# -------- real processes: launchers, encodings, determinism --------

@pytest.mark.parametrize("launcher", ["path", "module"])
def test_both_launchers_work_without_installation(launcher):
    result = process("validate", MINIMAL, launcher=launcher)

    assert result.returncode == 0, result.stderr.decode(errors="replace")
    assert result.stdout.startswith(f"{MINIMAL}: valid RoboLang 1.1 (1 task)\n".encode())


def test_process_exit_codes():
    assert process("validate", MINIMAL).returncode == 0
    assert process("validate", "tests/fixtures/invalid/e203_unknown_statement.rob").returncode == 1
    assert process("validate", "missing.rob").returncode == 2
    assert process().returncode == 2
    assert process("--help").returncode == 0


def test_process_streams_are_separated():
    result = process("inspect", DEMO, "--json")

    assert json.loads(result.stdout)["ir_schema"] == "robolang.task-ir"
    assert b"warning[W401]" in result.stderr and b"warning" not in result.stdout


def test_process_output_is_byte_identical_to_the_snapshot_on_every_platform():
    """LF line endings and ASCII only, also on Windows."""
    result = process("inspect", MULTI, "--json")

    assert result.returncode == 0
    assert result.stdout == (SNAPSHOTS / "multiple_tasks.json").read_text(encoding="ascii").encode("ascii")


def test_process_output_is_deterministic():
    runs = [process("inspect", MULTI) for _ in range(3)]

    assert {r.returncode for r in runs} == {0}
    assert len({r.stdout for r in runs}) == 1
    assert b"\r" not in runs[0].stdout


@pytest.mark.parametrize("encoding", ["ascii", "cp1252", "utf-8"])
def test_non_ascii_content_never_breaks_the_console(encoding, tmp_path):
    """Regression guard: the v1 demo crashed on consoles that could not encode its output."""
    good = tmp_path / "good.rob"
    good.write_text('task t(robot r) { plan { communicate r to "flotte" with "prêt → ok"; } }\n', encoding="utf-8")
    bad = tmp_path / "bad.rob"
    bad.write_text("task t() { plan { wait for 1s; } } →\n", encoding="utf-8")

    for command in (["inspect", str(good)], ["inspect", str(good), "--json"]):
        result = process(*command, encoding=encoding)
        assert result.returncode == 0, result.stderr.decode(errors="replace")
        result.stdout.decode("ascii")

    result = process("validate", str(bad), encoding=encoding)
    assert result.returncode == 1
    assert b"error[E101]" in result.stderr


def test_commands_do_not_load_the_interpreter_or_adapters():
    code = (
        "import sys; sys.path.insert(0, 'src'); from robolang.cli import main; "
        f"assert main(['inspect', '{DEMO}', '--json'], out=open(__import__('os').devnull, 'w')) == 0; "
        "loaded = [m for m in sys.modules if m.startswith('robolang_') or m == 'rclpy']; "
        "assert not loaded, loaded"
    )
    result = subprocess.run([sys.executable, "-c", code], cwd=REPO_ROOT, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
