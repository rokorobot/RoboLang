"""Tests for the commands in src/robolang/cli.py: validate and inspect (read-only) and the
simulation-only simulate (its record and exit statuses are tested in test_sim_cli.py)."""

from __future__ import annotations
import io
import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

from robolang import ir as task_ir
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


V12_VALID = sorted((FIXTURES / "v1.2" / "valid").glob("*.rob"))


@pytest.mark.parametrize("path", V12_VALID, ids=lambda p: p.name)
def test_inspect_text_describes_every_valid_v12_file(path):
    code, out, err = cli("inspect", str(path.relative_to(REPO_ROOT)))

    assert code == EXIT_OK, err
    assert "RoboLang 1.2 | IR schema 1.1" in out
    assert out.endswith("This is a description only. Nothing was executed.\n")


def _preconditions(path: str) -> list[str]:
    lines = cli("inspect", path)[1].splitlines()
    start = lines.index("  preconditions:") + 1
    return [line.strip() for line in lines[start:lines.index("  plan:")]]


def test_inspect_text_renders_and():
    assert _preconditions("tests/fixtures/v1.2/valid/and_in_pre.rob") == ["robot_ready r and region_clear cell"]


def test_inspect_text_renders_or():
    assert _preconditions("tests/fixtures/v1.2/valid/or_in_pre.rob") == [
        "region_clear cell or region_clear left or region_clear right",
    ]


def test_inspect_text_renders_not_on_its_single_operand():
    assert _preconditions("tests/fixtures/v1.2/valid/not_in_pre.rob") == [
        "not object_exists box",
        'not sensor_ok "camera_top" and robot_ready r',
    ]


def test_inspect_text_parenthesizes_nested_combinations():
    """`and`/`or` never mix unparenthesized, and `not` of a group keeps its parentheses."""
    assert _preconditions("tests/fixtures/v1.2/valid/parenthesized_mix.rob") == [
        '(robot_ready r and region_clear cell) or sensor_ok "camera_top"',
        "robot_ready r and (region_clear left or region_clear right)",
    ]
    assert _preconditions("tests/fixtures/v1.2/valid/not_of_group.rob") == [
        "not (region_clear left or region_clear right)",
        "robot_ready r",
    ]


def test_inspect_text_renders_combined_wait_until():
    out = cli("inspect", "tests/fixtures/v1.2/valid/wait_until_combined.rob")[1]

    assert (
        '    1. wait until (region_clear left or region_clear right) and sensor_ok "camera_top" timeout 30 s\n'
        in out
    )
    assert "    2. wait until not object_exists box timeout 500 ms\n" in out


def _rendered(path: str) -> tuple[list[str], list[str], list[str]]:
    """Parameters, preconditions and `wait until` steps of a one-task file, as text inspect prints them."""
    code, out, err = cli("inspect", path)
    assert code == EXIT_OK, err
    sections: dict[str, list[str]] = {}
    title = ""
    for line in out.splitlines():
        if line.startswith("    "):
            sections[title].append(line.strip())
        elif line.startswith("  "):
            title = line.strip().split(":")[0]
            sections[title] = []
    waits = [re.sub(r"^\d+\. ", "", s) for s in sections["plan"] if re.match(r"\d+\. wait until ", s)]
    return sections["parameters"], sections["preconditions"], waits


def _conditions(document: task_ir.TaskIR) -> tuple[list, list]:
    (task,) = document.tasks
    return list(task.preconditions), [s.condition for s in task.steps if isinstance(s, task_ir.WaitUntil)]


def _assert_round_trip(path: Path) -> None:
    """Text inspect output, written back as 1.2 source, compiles to the same condition IR."""
    parameters, pre, waits = _rendered(str(path))
    rebuilt = "".join(
        [
            f"robolang 1.2;\n\ntask t({', '.join(parameters)}) {{\n    pre {{\n",
            *(f"        {condition};\n" for condition in pre),
            "    }\n    plan {\n",
            *(f"        {wait};\n" for wait in waits),
            "        wait for 1 s;\n    }\n}\n",
        ]
    )
    original = task_ir.compile_source(path.read_text(encoding="utf-8"), str(path))
    result = task_ir.compile_source(rebuilt, "rebuilt.rob")
    assert result.ir is not None, (rebuilt, result.diagnostics)
    assert _conditions(result.ir) == _conditions(original.ir), rebuilt


V12_SOURCES = V12_VALID + sorted((FIXTURES / "v1.2" / "simulation" / "sources").glob("*.rob"))


@pytest.mark.parametrize("path", V12_SOURCES, ids=lambda p: p.name)
def test_inspect_text_round_trips_every_v12_fixture(path):
    _assert_round_trip(path)


ROUND_TRIP_CONDITIONS = [
    "not not not not robot_ready r",
    "(((robot_ready r)))",
    "not (region_clear left and region_clear right)",
    "not (not robot_ready r or region_clear cell)",
    "(robot_ready r or not object_exists box) and region_clear cell",
    'robot_ready r and (region_clear left or (region_clear right and not sensor_ok "cam"))',
    '((robot_ready r and region_clear cell) or not (sensor_ok "cam" or region_clear left)) and object_exists box',
    "(robot_ready r and region_clear cell) and (object_exists box and region_clear left)",
    "region_clear left or (region_clear right or region_clear cell) or not region_clear left",
]


@pytest.mark.parametrize("condition", ROUND_TRIP_CONDITIONS)
def test_inspect_text_round_trips_nested_conditions(condition, tmp_path):
    """Nested and/or/not with parentheses, as a precondition and in `wait until`."""
    path = tmp_path / "nested.rob"
    path.write_text(
        "robolang 1.2;\n\n"
        "task t(robot r, object box, region cell, region left, region right) {\n"
        f"    pre {{\n        {condition};\n    }}\n"
        f"    plan {{\n        wait until {condition} timeout 2 s;\n    }}\n"
        "}\n",
        encoding="utf-8",
    )
    _assert_round_trip(path)


def test_inspect_text_keeps_operator_words_as_names_in_v11_files():
    """In a 1.1 file `and`, `or` and `not` are names, and render as plain atoms."""
    _, pre, _ = _rendered("tests/fixtures/v1.2/compat/operator_words_as_names.rob")

    assert pre == ["robot_ready and", "region_clear or", "object_exists not"]


@pytest.mark.parametrize(
    "condition",
    [object(), task_ir.Not(object()), task_ir.And((task_ir.RobotReady("r"), object()))],
    ids=["bare", "under_not", "under_and"],
)
def test_inspect_text_rejects_an_unknown_condition_kind(condition):
    """An unknown kind is an error, never rendered as another kind (such as sensor_ok)."""
    from robolang import cli as cli_module

    with pytest.raises(TypeError, match="not a condition: object"):
        cli_module._condition(condition)


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
    """The only command that runs a task is the simulation-only `simulate`; none runs or executes one."""
    cli("--help")
    help_text = capsys.readouterr().out

    assert "validate" in help_text and "inspect" in help_text
    for word in ("run ", "execute"):
        assert f"\n    {word}" not in help_text


# -------- help and version --------

@pytest.mark.parametrize("args", [["--help"], ["validate", "--help"], ["inspect", "--help"], ["simulate", "--help"]])
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


def test_version_reports_distribution_language_and_schema_versions_separately(capsys):
    code, _, _ = cli("--version")
    text = capsys.readouterr().out

    assert code == EXIT_OK
    assert re.fullmatch(
        r"robolang (\d+\.\d+\.\d+\S*|unknown \(not installed, running from source\)) "
        r"\(language 1\.1, 1\.2, 1\.3 and 1\.4, IR schema 1\.0, 1\.1, 1\.2 and 1\.3\)\n",
        text,
    ), text


def test_version_text_uses_the_installed_distribution_version(monkeypatch):
    from robolang import cli as cli_module

    monkeypatch.setattr(cli_module.metadata, "version", lambda name: "9.8.7")
    assert cli_module.version_text() == (
        "robolang 9.8.7 (language 1.1, 1.2, 1.3 and 1.4, IR schema 1.0, 1.1, 1.2 and 1.3)"
    )


def test_version_text_when_running_from_source(monkeypatch):
    from robolang import cli as cli_module

    def missing(name):
        raise cli_module.metadata.PackageNotFoundError(name)

    monkeypatch.setattr(cli_module.metadata, "version", missing)
    assert cli_module.version_text() == (
        "robolang unknown (not installed, running from source) "
        "(language 1.1, 1.2, 1.3 and 1.4, IR schema 1.0, 1.1, 1.2 and 1.3)"
    )


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
