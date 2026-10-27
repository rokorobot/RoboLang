"""
Developer commands for RoboLang v1.1 and v1.2.

    validate FILE          check a .rob file against the language contract
    inspect  FILE          show what a validated file describes (text or JSON IR)
    simulate FILE --world WORLD
                           run a validated task in a symbolic simulation
    arch02 --scenario SCENARIO --cases CASES --case ID
                           run one case of the RFC-001 reference scenario in simulation

`validate` and `inspect` never execute anything. `simulate` runs only in a symbolic,
deterministic simulation (spec/simulation-v1.0.md): it never loads a robot adapter or connects
to ROS 2 or hardware, and its result is not evidence that a task is safe on a robot. `arch02`
runs the Core 0.1 reference simulation (spec/reference-simulation-v0.1.md) in one process: no
network, no ROS 2, no hardware.

Exit status:
    0  validate/inspect: the file is valid (warnings do not change this);
       simulate: the run ended SUCCEEDED
    1  the file is not valid RoboLang; diagnostics are on stderr
    2  the command could not be carried out: bad arguments or seed, unreadable file,
       unknown task name, several tasks and no --task
    3  simulate: the world file is not valid; arch02: the scenario or cases file is not valid
    4  simulate: the run ended FAILED
    5  simulate: the run ended CANCELLED
"""

from __future__ import annotations
import argparse
import json
import sys
from importlib import metadata
from pathlib import Path

from . import ir as task_ir
from .arch02 import ScenarioError, simulate_case
from .arch02 import to_json as arch02_json
from .core import ReadError
from .diagnostics import RoboLangError
from .sim import DEFAULT_SEED, SEED, WorldError, load_world, run_id, sha256_normalized, to_json, to_record
from .sim import run as simulate_task

EXIT_OK = 0
EXIT_INVALID = 1
EXIT_USAGE = 2
EXIT_WORLD_INVALID = 3
EXIT_FAILED = 4
EXIT_CANCELLED = 5

NOT_A_SAFETY_CHECK = (
    "Validation covers the text of the program only. "
    "It does not make a task safe to run on a robot."
)
NOTHING_EXECUTED = "This is a description only. Nothing was executed."

_DESCRIPTION = f"""\
Tools for RoboLang v1.1 and v1.2 task files.

validate and inspect check and describe .rob files; they never execute anything.
simulate runs a validated task in a symbolic, deterministic simulation only: it never loads a
robot adapter or connects to ROS 2 or hardware, and its result is not evidence that a task is
safe on a robot.

{NOT_A_SAFETY_CHECK}
"""

_EPILOG = """\
exit status:
  0  the file is valid (warnings do not change this); for simulate: the run SUCCEEDED
  1  the file is not valid; diagnostics are printed to stderr
  2  bad arguments, unreadable file, or unknown task name
  3  simulate only: the world file is not valid
  4  simulate only: the run ended FAILED
  5  simulate only: the run ended CANCELLED
"""

_SIMULATE_EPILOG = """\
exit status:
  0  the run ended SUCCEEDED; the run record is on stdout
  1  the source file is not valid RoboLang; nothing is run
  2  bad arguments or seed, unreadable file, unknown task name, or several tasks and no --task
  3  the world file is not valid; diagnostics are printed to stderr
  4  the run ended FAILED; the run record is on stdout
  5  the run ended CANCELLED; the run record is on stdout
"""

_ARCH02_EPILOG = """\
exit status:
  0  the case ran; the run record is on stdout (the planner's recorded status is in it, and on stderr)
  2  bad arguments or seed, unreadable file, or no case with that ID
  3  the scenario or cases file is not valid
"""


def distribution_version() -> str | None:
    """Version of the installed `robolang` distribution, or None when running from a source checkout."""
    try:
        return metadata.version("robolang")
    except metadata.PackageNotFoundError:
        return None


def version_text() -> str:
    """The three independent versions: distribution, language and IR schema."""
    distribution = distribution_version() or "unknown (not installed, running from source)"
    return (
        f"robolang {distribution} "
        f"(language {' and '.join(task_ir.IR_SCHEMA_VERSIONS)}, "
        f"IR schema {' and '.join(task_ir.IR_SCHEMA_VERSIONS.values())})"
    )


class _CommandError(Exception):
    """The command cannot be carried out (exit status 2)."""


def _plural(count: int, noun: str) -> str:
    return f"{count} {noun}" if count == 1 else f"{count} {noun}s"


def _quoted(text: str) -> str:
    return json.dumps(text, ensure_ascii=True)


def _read_bytes(path: str) -> bytes:
    try:
        return Path(path).read_bytes()
    except OSError as error:
        raise _CommandError(f"cannot read '{path}': {error.strerror or error}") from None


def _compile(path: str, err, data: bytes | None = None) -> task_ir.CompileResult:
    """
    Validate the whole file and lower it. Prints warnings; raises on any error.
    `data` is the file's content when the caller has already read it.
    """
    try:
        source = (_read_bytes(path) if data is None else data).decode("utf-8")
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


# -------- simulate --------

_SIMULATE_EXIT = {"SUCCEEDED": EXIT_OK, "FAILED": EXIT_FAILED, "CANCELLED": EXIT_CANCELLED}


def _simulate(args, out, err) -> int:
    seed = args.seed
    if not SEED.fullmatch(seed):
        raise _CommandError("invalid --seed: use 1 to 64 characters from A-Z a-z 0-9 . _ -")
    source_bytes = _read_bytes(args.file)  # read once: the run ID describes exactly what was run
    result = _compile(args.file, err, source_bytes)
    tasks = result.ir.tasks
    if args.task is None:
        if len(tasks) != 1:
            names = ", ".join(t.name for t in tasks)
            raise _CommandError(f"'{args.file}' has several tasks; choose one with --task: {names}")
        task = tasks[0]
    else:
        try:
            task = result.ir.select(args.task).tasks[0]
        except KeyError:
            names = ", ".join(t.name for t in tasks)
            raise _CommandError(f"no task named '{args.task}' in '{args.file}'; tasks in this file: {names}") from None
    world_bytes = _read_bytes(args.world)
    try:
        world = load_world(world_bytes, args.world, task)
    except WorldError as error:
        print(error.format(), file=err)
        return EXIT_WORLD_INVALID
    run_result = simulate_task(task, world)
    identifier = run_id(source_bytes, world_bytes, task.name, seed)
    record = to_record(
        run_result,
        task=task,
        run_id=identifier,
        source_sha256=sha256_normalized(source_bytes),
        world_sha256=sha256_normalized(world_bytes),
        seed=seed,
        language_version=result.ir.language_version,
    )
    out.write(to_json(record))
    summary = f"simulation only: run {identifier}: {run_result.state}"
    if run_result.failure is not None:
        summary += f" ({run_result.failure.code})"
    print(summary, file=err)
    return _SIMULATE_EXIT[run_result.state]


# -------- arch02 --------

def _arch02(args, out, err) -> int:
    if args.seed is not None and not SEED.fullmatch(args.seed):
        raise _CommandError("invalid --seed: use 1 to 64 characters from A-Z a-z 0-9 . _ -")
    scenario = _read_bytes(args.scenario)
    cases_data = _read_bytes(args.cases)
    try:
        cases = json.loads(cases_data.decode("utf-8"))["cases"]
        case = next((c for c in cases if c.get("id") == args.case), None)
    except (ValueError, KeyError, TypeError, AttributeError):
        print(f"{args.cases}: error: not a cases file with a 'cases' list", file=err)
        return EXIT_WORLD_INVALID
    if case is None:
        ids = ", ".join(str(c.get("id")) for c in cases)
        raise _CommandError(f"no case '{args.case}' in '{args.cases}'; cases: {ids}")
    try:
        record = simulate_case(scenario, case, args.seed)
    except (ScenarioError, ReadError, KeyError, TypeError, ValueError) as error:
        print(f"{args.scenario}: error: the scenario or case is not valid ({error})", file=err)
        return EXIT_WORLD_INVALID
    out.write(arch02_json(record))
    status = record["final"]["planner_status"]
    print(f"simulation only: run {record['run_id']}: planner-1 recorded {status}", file=err)
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
        version=version_text(),
        help="show the distribution, language and IR schema versions and exit",
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

    simulate = commands.add_parser(
        "simulate",
        help="run a validated task in a symbolic simulation (no robot, no ROS 2)",
        description=(
            "Validate FILE, then run one task against the world described by WORLD in a symbolic,\n"
            "deterministic simulation and print the run record (JSON) on stdout.\n"
            "Simulation only: nothing is sent to a robot, no adapter is loaded and ROS 2 is not used.\n"
            "The result is not evidence that a task is safe on a robot.\n\n" + NOT_A_SAFETY_CHECK
        ),
        epilog=_SIMULATE_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    simulate.add_argument("file", metavar="FILE", help="path to a .rob file")
    simulate.add_argument("--world", metavar="WORLD", required=True, help="path to a world file (schema 1.0)")
    simulate.add_argument("--task", metavar="NAME", help="the task to run (required when FILE has several tasks)")
    simulate.add_argument(
        "--seed",
        metavar="SEED",
        default=DEFAULT_SEED,
        help=f"1 to 64 characters from A-Z a-z 0-9 . _ - (default {DEFAULT_SEED}); only distinguishes runs",
    )
    simulate.set_defaults(run=_simulate)

    arch02 = commands.add_parser(
        "arch02",
        help="run one case of the RFC-001 reference scenario in simulation (no network, no robot)",
        description=(
            "Run one case of the Core 0.1 reference simulation (spec/reference-simulation-v0.1.md)\n"
            "and print its run record (JSON) on stdout.\n"
            "Simulation only: one process, a symbolic world, no network, no ROS 2, no hardware.\n"
            "The record is not evidence about real robots, networks or signatures."
        ),
        epilog=_ARCH02_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    arch02.add_argument("--scenario", metavar="SCENARIO", required=True, help="path to a scenario file (0.1)")
    arch02.add_argument("--cases", metavar="CASES", required=True, help="path to a cases file")
    arch02.add_argument("--case", metavar="ID", required=True, help="the ID of the case to run")
    arch02.add_argument("--seed", metavar="SEED", help="replaces the case's seed; only distinguishes runs")
    arch02.set_defaults(run=_arch02)
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
