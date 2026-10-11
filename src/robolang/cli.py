"""
Read-only developer commands for RoboLang v1.1.

    validate FILE          check a .rob file against the language contract
    inspect  FILE          show what a validated file describes (text or JSON IR)

Neither command executes anything, imports an adapter or contacts a robot.

Exit status:
    0  the file is valid (warnings do not change this)
    1  the file is not valid RoboLang v1.1; diagnostics are on stderr
    2  the command could not be carried out: bad arguments, unreadable file,
       unknown task name
"""

from __future__ import annotations
import argparse
import json
import sys
from pathlib import Path

from . import ir as task_ir
from .diagnostics import RoboLangError

EXIT_OK = 0
EXIT_INVALID = 1
EXIT_USAGE = 2

NOT_A_SAFETY_CHECK = (
    "Validation covers the text of the program only. "
    "It does not make a task safe to run on a robot."
)
NOTHING_EXECUTED = "This is a description only. Nothing was executed."

_DESCRIPTION = f"""\
Read-only tools for RoboLang v{task_ir.LANGUAGE_VERSION} task files.

These commands check and describe .rob files. They never execute a task,
load a robot adapter or connect to ROS 2 or hardware.

{NOT_A_SAFETY_CHECK}
"""

_EPILOG = """\
exit status:
  0  the file is valid (warnings do not change this)
  1  the file is not valid; diagnostics are printed to stderr
  2  bad arguments, unreadable file, or unknown task name
"""


class _CommandError(Exception):
    """The command cannot be carried out (exit status 2)."""


def _plural(count: int, noun: str) -> str:
    return f"{count} {noun}" if count == 1 else f"{count} {noun}s"


def _quoted(text: str) -> str:
    return json.dumps(text, ensure_ascii=True)


def _compile(path: str, err) -> task_ir.CompileResult:
    """Validate the whole file and lower it. Prints warnings; raises on any error."""
    try:
        source = Path(path).read_bytes().decode("utf-8")
    except OSError as error:
        raise _CommandError(f"cannot read '{path}': {error.strerror or error}") from None
    except UnicodeDecodeError:
        raise _CommandError(f"cannot read '{path}': the file is not valid UTF-8") from None
    result = task_ir.compile_source(source, path)
    for warning in result.warnings:
        print(warning.format(), file=err)
    return result


# -------- validate --------

def _validate(args, out, err) -> int:
    result = _compile(args.file, err)
    summary = _plural(len(result.ir.tasks), "task")
    if result.warnings:
        summary += f", {_plural(len(result.warnings), 'warning')}"
    print(f"{args.file}: valid RoboLang {result.ir.language_version} ({summary})", file=out)
    print(NOT_A_SAFETY_CHECK, file=out)
    return EXIT_OK


# -------- inspect --------

def _quantity(quantity: task_ir.Quantity) -> str:
    value = task_ir.decimal_text(quantity.value)
    return f"{value} {quantity.unit}" if quantity.unit else value


def _condition(condition: task_ir.Condition) -> str:
    if isinstance(condition, task_ir.RobotReady):
        return f"robot_ready {condition.robot}"
    if isinstance(condition, task_ir.RegionClear):
        return f"region_clear {condition.region}"
    if isinstance(condition, task_ir.ObjectExists):
        return f"object_exists {condition.obj}"
    return f"sensor_ok {_quoted(condition.sensor)}"


def _step(step: task_ir.Step) -> str:
    if isinstance(step, task_ir.Move):
        return f"move {step.robot} to {step.target}"
    if isinstance(step, task_ir.Grasp):
        return f"grasp {step.robot} {step.obj}"
    if isinstance(step, task_ir.Place):
        return f"place {step.robot} {step.obj} at {step.target}"
    if isinstance(step, task_ir.Inspect):
        return f"inspect {step.robot} {step.obj} using {_quoted(step.sensor)}"
    if isinstance(step, task_ir.WaitFor):
        return f"wait for {_quantity(step.duration)}"
    if isinstance(step, task_ir.WaitUntil):
        return f"wait until {_condition(step.condition)} timeout {_quantity(step.timeout)}"
    return f"communicate {step.robot} to {_quoted(step.channel)} with {_quoted(step.message)}"


def _safety(limit: task_ir.SafetyLimit) -> str:
    text = f"{limit.kind} {_quantity(limit.limit)}"
    if limit.limit.unit is None:
        return f"{text}  (no unit: deprecated; declared only, never treated as an enforced limit)"
    return f"{text}  (declared only; not enforced by this tool)"


def _section(title: str, lines: list[str]) -> list[str]:
    if not lines:
        return [f"  {title}: (none)"]
    return [f"  {title}:"] + [f"    {line}" for line in lines]


def _describe(task: task_ir.Task) -> list[str]:
    lines = [f"task {task.name}"]
    lines += _section("parameters", [f"{p.type} {p.name}" for p in task.parameters])
    lines += _section("safety", [_safety(s) for s in task.safety])
    lines += _section("preconditions", [_condition(c) for c in task.preconditions])
    lines += _section("plan", [f"{i}. {_step(s)}" for i, s in enumerate(task.steps, 1)])
    return lines


def _inspect(args, out, err) -> int:
    result = _compile(args.file, err)
    document = result.ir
    if args.task is not None:
        try:
            document = document.select(args.task)
        except KeyError:
            names = ", ".join(t.name for t in result.ir.tasks)
            raise _CommandError(f"no task named '{args.task}' in '{args.file}'; tasks in this file: {names}") from None

    if args.json:
        out.write(task_ir.to_json(document))
        return EXIT_OK

    shown = _plural(len(document.tasks), "task")
    if args.task is not None:
        shown = f"task {args.task} (1 of {len(result.ir.tasks)})"
    print(args.file, file=out)
    print(
        f"RoboLang {document.language_version} | IR schema {document.ir_schema_version} | {shown}",
        file=out,
    )
    for task in document.tasks:
        print(file=out)
        print("\n".join(_describe(task)), file=out)
    print(file=out)
    print(NOTHING_EXECUTED, file=out)
    return EXIT_OK


# -------- entry point --------

def _parser(prog: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=prog,
        description=_DESCRIPTION,
        epilog=_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"RoboLang language {task_ir.LANGUAGE_VERSION}, IR schema {task_ir.IR_SCHEMA_VERSION}",
    )
    commands = parser.add_subparsers(dest="command", metavar="COMMAND", required=True)

    validate = commands.add_parser(
        "validate",
        help="check a .rob file: syntax, names, types and values",
        description=(
            "Check FILE against the RoboLang language contract: syntax, names, types and values.\n"
            "The whole file is checked. Nothing is executed.\n\n" + NOT_A_SAFETY_CHECK
        ),
        epilog=_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    validate.add_argument("file", metavar="FILE", help="path to a .rob file")
    validate.set_defaults(run=_validate)

    inspect = commands.add_parser(
        "inspect",
        help="show the tasks a valid .rob file describes, as text or JSON",
        description=(
            "Validate FILE, then show the tasks it describes.\n"
            "The whole file is validated first, even when --task selects one task.\n"
            "Nothing is executed: inspect only prints a description.\n\n" + NOT_A_SAFETY_CHECK
        ),
        epilog=_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    inspect.add_argument("file", metavar="FILE", help="path to a .rob file")
    inspect.add_argument("--task", metavar="NAME", help="show only the task with this name")
    inspect.add_argument(
        "--json",
        action="store_true",
        help=f"print the versioned IR (schema {task_ir.IR_SCHEMA_VERSION}) as JSON instead of text",
    )
    inspect.set_defaults(run=_inspect)
    return parser


def main(argv: list[str] | None = None, out=None, err=None, prog: str = "robolang") -> int:
    out = out or sys.stdout
    err = err or sys.stderr
    try:
        args = _parser(prog).parse_args(argv)
    except SystemExit as stop:  # argparse: --help / --version (0) or a usage error (2)
        return stop.code if isinstance(stop.code, int) else EXIT_USAGE
    try:
        return args.run(args, out, err)
    except RoboLangError as error:
        print(error.diagnostic.format(), file=err)
        return EXIT_INVALID
    except _CommandError as error:
        print(f"{prog}: error: {error}", file=err)
        return EXIT_USAGE


def run() -> None:
    """Console entry: fixed LF line endings, and never fail on characters the console cannot encode."""
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(newline="\n", errors="backslashreplace")
    sys.exit(main())
