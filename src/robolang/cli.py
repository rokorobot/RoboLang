"""
Developer commands for RoboLang v1.1, v1.2 and v1.3.

    reference              print the shipped RoboLang 1.3 language reference
    validate FILE          check a .rob file against the language contract
    inspect  FILE          show what a validated file describes (text or JSON IR)
    simulate FILE --world WORLD
                           run a validated task in a symbolic simulation
    export-goal FILE --world WORLD --from AGENT_ID
                           print the task's goal as a signed Core 0.1 propose_goal message
    delegate FILE --world WORLD --from AGENT_ID --to ROBOT_ID
                           send the task's goal to a robot that runs its own plan; print the delegate record

validate, inspect, simulate, export-goal and delegate take `--ir IR` instead of FILE: a Task IR JSON
document (schema 1.0 to 1.2), read strictly and re-validated in full (spec/ir-reader-v0.1.md).
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
    1  the file is not valid RoboLang (or, with --ir, not valid Task IR); diagnostics are on stderr
    2  the command could not be carried out: bad arguments or seed, unreadable file,
       unknown task name, several tasks and no --task
    3  simulate and export-goal: the world file is not valid; arch02: the scenario or cases file is not valid
    4  simulate: the run ended FAILED
    5  simulate: the run ended CANCELLED
    6  export-goal and delegate: the goal has no Core 0.1 form, or an object lacks a class or mass
    7  delegate: the robot rejected the goal (or its peer declined), or a participant rejected a message
"""

from __future__ import annotations
import argparse
import json
import re
import sys
from dataclasses import dataclass
from importlib import metadata, resources
from pathlib import Path

from . import bridge
from . import ir as task_ir
from .arch02 import ScenarioError, simulate_case
from .arch02 import to_json as arch02_json
from .core import ReadError
from .diagnostics import MAX_ERRORS, Diagnostic, RoboLangError
from .delegate import Plan, delegate
from .delegate_peer import DEFAULT_LEASE_MS, lease_ms
from .export_goal import ExportError, default_recipient, export_goal, valid_participant
from .ir_reader import IRDiagnostic, IRError, read_ir
from .nodes import Program
from .sim import DEFAULT_SEED, SEED, World, WorldError, load_world, run_id, sha256_normalized, to_json, to_record
from .sim import run as simulate_task
from .sim.check import Finding, check_task

EXIT_OK = 0
EXIT_INVALID = 1
EXIT_USAGE = 2
EXIT_WORLD_INVALID = 3
EXIT_FAILED = 4
EXIT_CANCELLED = 5
EXIT_NOT_EXPORTABLE = 6
EXIT_REJECTED = 7

NOT_A_SAFETY_CHECK = (
    "Validation covers the text of the program only. "
    "It does not make a task safe to run on a robot."
)
NOT_A_SAFETY_CHECK_IR = (
    "Validation covers the IR document only. "
    "It does not make a task safe to run on a robot."
)
NOTHING_EXECUTED = "This is a description only. Nothing was executed."

_DESCRIPTION = f"""\
Tools for RoboLang v1.1, v1.2 and v1.3 task files.

reference prints the shipped 1.3 language reference. validate and inspect check and describe .rob
files; they never execute anything.
simulate runs a validated task in a symbolic, deterministic simulation only: it never loads a
robot adapter or connects to ROS 2 or hardware, and its result is not evidence that a task is
safe on a robot.

{NOT_A_SAFETY_CHECK}
"""

_EPILOG = """\
exit status:
  0  the file is valid (warnings do not change this); for simulate: the run SUCCEEDED
  1  the file is not valid (RoboLang, or with --ir Task IR); diagnostics are printed to stderr
  2  bad arguments, unreadable file, or unknown task name
  3  simulate only: the world file is not valid
  4  simulate only: the run ended FAILED
  5  simulate only: the run ended CANCELLED
  6  export-goal and delegate only: the goal has no Core 0.1 form, or an object lacks a class or mass
  7  delegate only: the robot rejected the goal (or its peer declined), or a participant rejected a message
"""

_SIMULATE_EPILOG = """\
exit status:
  0  the run ended SUCCEEDED; the run record is on stdout
  1  the source file is not valid RoboLang (or the --ir file not valid Task IR); nothing is run
  2  bad arguments or seed, both or neither of FILE and --ir, malformed or repeated --bind, --bind of a
     name that is not a parameter, unreadable file, unknown task name, or several tasks and no --task
  3  the world file is not valid, or a --bind names a missing or wrong-kind entity or leaves a
     parameter unbound; diagnostics are printed to stderr
  4  the run ended FAILED; the run record is on stdout
  5  the run ended CANCELLED; the run record is on stdout
"""

_EXPORT_EPILOG = """\
exit status:
  0  the message is on stdout
  1  the source file is not valid RoboLang (or the --ir file not valid Task IR); nothing is exported
  2  bad arguments, both or neither of FILE and --ir, malformed or repeated --bind, unreadable file,
     unknown task name, several tasks and no --task, an invalid --from, --to or --at, or several robots
     and no --to
  3  the world file is not valid, or a --bind names a missing or wrong-kind entity or leaves a
     parameter unbound; diagnostics are printed to stderr
  6  the goal has no Core 0.1 form (X601), or a bound object has no class or mass (X602)
"""

_DELEGATE_EPILOG = """\
exit status:
  0  the planner recorded a verified `succeeded`; the delegate record is on stdout
  1  the source file, the --ir file or a plan file is not valid; nothing is run
  2  bad arguments, both or neither of FILE and --ir, --ir without --plans, malformed or repeated --bind,
     unreadable file, unknown task name, several tasks and no --task, an invalid --from or --to or --lease,
     or no .rob file under --plans
  3  a world file is not valid, or a --bind names a missing or wrong-kind entity or leaves a
     parameter unbound; diagnostics are printed to stderr
  4  the planner recorded failed, unknown or cancelled; the delegate record is on stdout
  6  the goal has no Core 0.1 form (X601), or a bound object has no class or mass (X602)
  7  the robot rejected the goal (no plan matches, or its peer declined), or a participant
     rejected a message; the delegate record is on stdout
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


def _joined(versions) -> str:
    """"1.1", "1.1 and 1.2", "1.1, 1.2 and 1.3"."""
    versions = list(versions)
    return " and ".join([", ".join(versions[:-1]), versions[-1]] if len(versions) > 2 else versions)


def version_text() -> str:
    """The three independent versions: distribution, language and IR schema."""
    distribution = distribution_version() or "unknown (not installed, running from source)"
    return (
        f"robolang {distribution} "
        f"(language {_joined(task_ir.IR_SCHEMA_VERSIONS)}, "
        f"IR schema {_joined(task_ir.IR_SCHEMA_VERSIONS.values())})"
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
    Validate the whole file and lower it. Prints warnings; raises RoboLangError listing every error.
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


@dataclass(frozen=True)
class _Loaded:
    data: bytes
    document: task_ir.TaskIR
    kind: str  # "source" (a .rob file) or "ir" (a Task IR JSON file, read and re-validated by read_ir)
    warnings: int = 0
    program: Program | None = None  # the syntax tree of a source file (spans for diagnostics); None for IR


def _load(args, err) -> _Loaded:
    """Read the input file once and turn it into a TaskIR. Raises RoboLangError or IRError (exit 1)."""
    data = _read_bytes(args.file)
    if args.ir is not None:
        return _Loaded(data, read_ir(data, args.file), "ir")
    result = _compile(args.file, err, data)
    return _Loaded(data, result.ir, "source", len(result.warnings), result.program)


def _print_ir_errors(error: IRError, path: str, err, summary: bool):
    """Every diagnostic, in document order; a summary line (validate only) when there is more than one."""
    for diagnostic in error.diagnostics:
        print(diagnostic.format(), file=err)
    if error.truncated:
        print(f"{path}: too many errors: stopped after {MAX_ERRORS}", file=err)
    if summary and (len(error.diagnostics) > 1 or error.truncated):
        count = f"{MAX_ERRORS}+ errors" if error.truncated else _plural(len(error.diagnostics), "error")
        print(f"{path}: not valid ({count})", file=err)


def _print_errors(error: RoboLangError, path: str, err, summary: bool):
    """
    Print every diagnostic, in source order. A file with more than one diagnostic then gets a
    summary line (validate only), so a file with a single error prints exactly that error.
    """
    diagnostics = [*error.warnings, *error.diagnostics]
    for diagnostic in sorted(diagnostics, key=lambda d: (d.line, d.column)):
        print(diagnostic.format(), file=err)
    if error.truncated:
        print(f"{path}: too many errors: stopped after {MAX_ERRORS}", file=err)
    if summary and (len(diagnostics) > 1 or error.truncated):
        count = f"{MAX_ERRORS}+ errors" if error.truncated else _plural(len(error.diagnostics), "error")
        if error.warnings:
            count += f", {_plural(len(error.warnings), 'warning')}"
        print(f"{path}: not valid ({count})", file=err)


# -------- validate --------

def _world_diagnostic(args, inputs: _RunInputs, finding: Finding):
    """A world-check finding at its source position, or at its JSON Pointer in a Task IR file."""
    part, index = finding.where
    parts = finding.label.split(".") if finding.label else []  # "3.else.2" -> 3, else, 2
    if inputs.loaded.program is None:
        task_index = inputs.document.tasks.index(inputs.task)
        pointer = f"/tasks/{task_index}/{part}/{index}"
        for side, position in zip(parts[1::2], parts[2::2]):
            pointer += f"/{side}/{int(position) - 1}"
        return IRDiagnostic(finding.code, finding.message, pointer, args.file)
    node = next(t for t in inputs.loaded.program.tasks if t.name.text == inputs.task.name)
    statement = (node.annotations if part == "safety" else node.plan.statements)[index]
    for side, position in zip(parts[1::2], parts[2::2]):  # into the branch, then to the statement in it
        branch = statement.then if side == "then" else statement.orelse
        statement = branch.statements[int(position) - 1]
    span = statement.span
    return Diagnostic(finding.code, finding.message, span, filename=args.file)


def _validate(args, out, err) -> int:
    inputs = _run_inputs(args, err) if args.world is not None else None
    loaded = _load(args, err) if inputs is None else inputs.loaded
    document = loaded.document
    if inputs is not None:
        findings = check_task(inputs.task, inputs.world)
        for finding in findings:
            print(_world_diagnostic(args, inputs, finding).format(), file=err)
        if len(findings) > 1:
            print(f"{args.file}: task '{inputs.task.name}' cannot succeed in {args.world} "
                  f"({_plural(len(findings), 'error')})", file=err)
        if findings:
            return EXIT_INVALID
    summary = _plural(len(document.tasks), "task")
    if loaded.warnings:
        summary += f", {_plural(loaded.warnings, 'warning')}"
    if loaded.kind == "ir":
        print(f"{args.file}: valid Task IR {document.ir_schema_version} "
              f"(RoboLang {document.language_version}, {summary})", file=out)
        print(NOT_A_SAFETY_CHECK_IR, file=out)
    else:
        print(f"{args.file}: valid RoboLang {document.language_version} ({summary})", file=out)
        print(NOT_A_SAFETY_CHECK, file=out)
    if inputs is not None:
        source = "world bindings" if inputs.cli_bindings is None else "--bind"
        paths = " (every path of its branches checked as a straight line)" if inputs.task.branching else ""
        print(f"{args.file}: task '{inputs.task.name}' checked against {args.world} ({source}){paths}: "
              f"no certain failure found. Run `simulate` for the rest", file=out)
    return EXIT_OK


# -------- inspect --------

def _quantity(quantity: task_ir.Quantity) -> str:
    value = task_ir.decimal_text(quantity.value)
    return f"{value} {quantity.unit}" if quantity.unit else value


def _operand(condition: task_ir.Condition, typed: bool) -> str:
    # 1.2 grammar: `not` takes one operand, and `and`/`or` never mix without parentheses.
    text = _condition(condition, typed)
    return f"({text})" if isinstance(condition, (task_ir.And, task_ir.Or)) else text


def _sensor(name: str, typed: bool) -> str:
    """A sensor parameter name in 1.3 (IR schema 1.2), a quoted world sensor ID before."""
    return name if typed else _quoted(name)


def _condition(condition: task_ir.Condition, typed: bool = False) -> str:
    """`typed`: the condition belongs to a RoboLang 1.3 task, where a sensor is a parameter."""
    if isinstance(condition, task_ir.And):
        return " and ".join(_operand(c, typed) for c in condition.conditions)
    if isinstance(condition, task_ir.Or):
        return " or ".join(_operand(c, typed) for c in condition.conditions)
    if isinstance(condition, task_ir.Not):
        return f"not {_operand(condition.condition, typed)}"
    if isinstance(condition, task_ir.RobotReady):
        return f"robot_ready {condition.robot}"
    if isinstance(condition, task_ir.RegionClear):
        return f"region_clear {condition.region}"
    if isinstance(condition, task_ir.ObjectExists):
        return f"object_exists {condition.obj}"
    if isinstance(condition, task_ir.SensorOk):
        return f"sensor_ok {_sensor(condition.sensor, typed)}"
    if isinstance(condition, task_ir.ObjectAt):
        return f"object_at {condition.obj} {condition.location}"
    if isinstance(condition, task_ir.RobotAt):
        return f"robot_at {condition.robot} {condition.location}"
    if isinstance(condition, task_ir.Holding):
        return f"holding {condition.robot} {condition.obj}"
    # A new condition kind must be rendered here explicitly, never shown as another kind.
    raise TypeError(f"not a condition: {type(condition).__name__}")


def _step(step: task_ir.Step, typed: bool = False) -> str:
    if isinstance(step, task_ir.Move):
        return f"move {step.robot} to {step.target}"
    if isinstance(step, task_ir.Grasp):
        return f"grasp {step.robot} {step.obj}"
    if isinstance(step, task_ir.Place):
        return f"place {step.robot} {step.obj} at {step.target}"
    if isinstance(step, task_ir.Inspect):
        return f"inspect {step.robot} {step.obj} using {_sensor(step.sensor, typed)}"
    if isinstance(step, task_ir.WaitFor):
        return f"wait for {_quantity(step.duration)}"
    if isinstance(step, task_ir.WaitUntil):
        return f"wait until {_condition(step.condition, typed)} timeout {_quantity(step.timeout)}"
    if isinstance(step, task_ir.If):
        return f"if {_condition(step.condition, typed)}"
    if isinstance(step, task_ir.Fail):
        return f"fail {_quoted(step.reason)}"
    return f"communicate {step.robot} to {_quoted(step.channel)} with {_quoted(step.message)}"


def _plan_lines(steps, typed: bool, number: str = "") -> list[str]:
    """The steps numbered 1., 2., ...; an `if` is followed by its indented branches, numbered 2.1., 2.2., ..."""
    lines = []
    for i, step in enumerate(steps, 1):
        label = f"{number}{i}."
        lines.append(f"{label} {_step(step, typed)}")
        if isinstance(step, task_ir.If):
            for word, goal, body in (("then", step.goal_then, step.then), ("else", step.goal_else, step.orelse)):
                if not body:
                    continue
                lines.append(f"  {word}:")
                if goal:
                    lines.append(f"    goal: {'; '.join(_condition(c, typed) for c in goal)}")
                lines += [f"    {line}" for line in _plan_lines(body, typed, f"{label[:-1]}.")]
            if not step.orelse:
                lines.append("  else: (nothing; the task goal alone applies)")
    return lines


def _safety(limit: task_ir.SafetyLimit) -> str:
    text = f"{limit.kind} {_quantity(limit.limit)}"
    if limit.limit.unit is None:
        return f"{text}  (no unit: deprecated; declared only, never treated as an enforced limit)"
    return f"{text}  (declared only; not enforced by this tool)"


def _verify_lines(verify: task_ir.Verify) -> list[str]:
    lines = []
    if verify.min_confidence is not None:
        lines.append(f"min_confidence {task_ir.decimal_text(verify.min_confidence)}")
    if verify.max_age is not None:
        lines.append(f"max_age {_quantity(verify.max_age)}")
    return lines


def _section(title: str, lines: list[str]) -> list[str]:
    if not lines:
        return [f"  {title}: (none)"]
    return [f"  {title}:"] + [f"    {line}" for line in lines]


def _describe(task: task_ir.Task) -> list[str]:
    lines = [f"task {task.name}"]
    lines += _section("parameters", [f"{p.type} {p.name}" for p in task.parameters])
    lines += _section("safety", [_safety(s) for s in task.safety])
    if task.verify is not None:
        lines += _section("verify", _verify_lines(task.verify))
    typed = task.typed_sensors
    lines += _section("preconditions", [_condition(c, typed) for c in task.preconditions])
    if task.goal is not None:
        lines += _section("goal", [_condition(c, typed) for c in task.goal])
    lines += _section("plan", _plan_lines(task.steps, typed))
    return lines


def _inspect(args, out, err) -> int:
    whole = document = _load(args, err).document
    if args.task is not None:
        try:
            document = document.select(args.task)
        except KeyError:
            names = ", ".join(t.name for t in whole.tasks)
            raise _CommandError(f"no task named '{args.task}' in '{args.file}'; tasks in this file: {names}") from None

    if args.json:
        out.write(task_ir.to_json(document))
        return EXIT_OK

    shown = _plural(len(document.tasks), "task")
    if args.task is not None:
        shown = f"task {args.task} (1 of {len(whole.tasks)})"
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


_BIND_NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
_BIND_ID = re.compile(r"[A-Za-z][A-Za-z0-9_]*")


def _parse_bindings(groups) -> dict[str, str] | None:
    """`--bind` values (one list per flag) -> parameter name -> entity ID; None when --bind was not given."""
    if not groups:
        return None
    bindings: dict[str, str] = {}
    for item in (item for group in groups for item in group):
        name, equals, target = item.partition("=")
        if not equals or not _BIND_NAME.fullmatch(name) or not _BIND_ID.fullmatch(target):
            raise _CommandError(f"invalid --bind '{item}': use NAME=ID, for example r=arm1")
        if name in bindings:
            raise _CommandError(f"invalid --bind: parameter '{name}' is bound more than once")
        bindings[name] = target
    return bindings


@dataclass(frozen=True)
class _RunInputs:
    source_bytes: bytes
    world_bytes: bytes
    document: task_ir.TaskIR
    task: task_ir.Task
    world: World
    cli_bindings: dict[str, str] | None
    source_kind: str  # "source" or "ir": what source_bytes is
    loaded: _Loaded | None = None


def _run_inputs(args, err) -> _RunInputs:
    """
    What `simulate`, `export-goal` and `delegate` share: validate FILE (or the --ir file), select the task,
    apply `--bind`, load the world. Raises RoboLangError or IRError (exit 1), _CommandError (2) or
    WorldError (3).
    """
    cli_bindings = _parse_bindings(args.bind)
    loaded = _load(args, err)  # read once: the run ID describes exactly what was run
    document = loaded.document
    tasks = document.tasks
    if args.task is None:
        if len(tasks) != 1:
            names = ", ".join(t.name for t in tasks)
            raise _CommandError(f"'{args.file}' has several tasks; choose one with --task: {names}")
        task = tasks[0]
    else:
        try:
            task = document.select(args.task).tasks[0]
        except KeyError:
            names = ", ".join(t.name for t in tasks)
            raise _CommandError(f"no task named '{args.task}' in '{args.file}'; tasks in this file: {names}") from None
    if cli_bindings is not None:
        names = [p.name for p in task.parameters]
        for name in cli_bindings:
            if name not in names:
                listed = ", ".join(names) or "(none)"
                raise _CommandError(f"--bind: '{name}' is not a parameter of task '{task.name}'; parameters: {listed}")
    world_bytes = _read_bytes(args.world)
    world = load_world(world_bytes, args.world, task, cli_bindings)
    return _RunInputs(loaded.data, world_bytes, document, task, world, cli_bindings, loaded.kind, loaded)


def _simulate(args, out, err) -> int:
    seed = args.seed
    if not SEED.fullmatch(seed):
        raise _CommandError("invalid --seed: use 1 to 64 characters from A-Z a-z 0-9 . _ -")
    inputs = _run_inputs(args, err)
    source_bytes, world_bytes, task, world = inputs.source_bytes, inputs.world_bytes, inputs.task, inputs.world
    cli_bindings = inputs.cli_bindings
    run_result = simulate_task(task, world)
    identifier = run_id(source_bytes, world_bytes, task.name, seed, cli_bindings, inputs.source_kind)
    record = to_record(
        run_result,
        task=task,
        run_id=identifier,
        source_sha256=sha256_normalized(source_bytes),
        world_sha256=sha256_normalized(world_bytes),
        seed=seed,
        language_version=inputs.document.language_version,
        bindings_source="world" if cli_bindings is None else "cli",
        world_schema_version=world.schema_version,
        source_kind=inputs.source_kind,
        core=args.core,
    )
    out.write(to_json(record))
    summary = f"simulation only: run {identifier}: {run_result.state}"
    if run_result.failure is not None:
        summary += f" ({run_result.failure.code})"
    print(summary, file=err)
    return _SIMULATE_EXIT[run_result.state]


# -------- export-goal --------

def _export_failure(path: str, task: task_ir.Task, error: Exception, err) -> int:
    """Report a goal that cannot be exported (X601, X602) and give the exit status."""
    if isinstance(error, ExportError):
        print(f"{path}: error[{error.code}]: task '{task.name}': {error.message}", file=err)
        return EXIT_NOT_EXPORTABLE
    if error.code != "X601":
        print(f"{path}: error[{error.code}]: task '{task.name}': {error}", file=err)
        return EXIT_NOT_EXPORTABLE
    shown = f" ({_condition(task.goal[error.index - 1], True)})" if 0 < error.index <= len(task.goal) else ""
    hint = "" if isinstance(error, bridge.NeedsCore02) else "; only object_at, robot_at and holding can be exported"
    print(f"{path}: error[X601]: task '{task.name}': {error}{shown}{hint}", file=err)
    return EXIT_NOT_EXPORTABLE


def _export_goal(args, out, err) -> int:
    for flag, value in (("--from", args.sender), ("--to", args.recipient)):
        if value is not None and not valid_participant(value):
            raise _CommandError(
                f"invalid {flag} '{value}': use a Core identifier without '#', for example rl:agent:planner-1"
            )
    if args.at is not None:
        try:
            bridge.core_time_after(args.at, 0)
        except ValueError:
            example = "2026-10-19T10:14:00.400Z"
            raise _CommandError(f"invalid --at '{args.at}': use a Core time, for example {example}") from None
    inputs = _run_inputs(args, err)
    task, world = inputs.task, inputs.world
    recipient = args.recipient or default_recipient(task, dict(world.task.bindings))
    if recipient is None:
        robots = [p.name for p in task.parameters if p.type == "robot"]
        raise _CommandError(
            f"task '{task.name}' has {_plural(len(robots), 'robot parameter')} ({', '.join(robots) or 'none'}): "
            "choose the recipient with --to"
        )
    try:
        exported = export_goal(task, world, sender=args.sender, recipient=recipient, sent_at=args.at,
                               core=args.core)
    except (bridge.NotExportable, ExportError) as error:
        return _export_failure(args.file, task, error, err)
    for warning in exported.warnings:
        print(f"{args.file}: warning: {warning}", file=err)
    out.write(json.dumps(exported.message, indent=2, ensure_ascii=True) + "\n")
    goal_id, message_id = exported.message["payload"]["goal"]["id"], exported.message["message_id"]
    print(f"simulation only: exported {goal_id} as {message_id}", file=err)
    return EXIT_OK


# -------- delegate --------

class _PlanInvalid(Exception):
    """A plan file is not valid RoboLang; its diagnostics are already printed."""


def _plan_files(paths: list[str]) -> list[str]:
    """The `.rob` files named by --plans: a file as given, a directory's `*.rob` files in name order."""
    files = []
    for path in paths:
        if Path(path).is_dir():
            files += [str(p) for p in sorted(Path(path).glob("*.rob"))]
        else:
            files.append(path)
    if not files:
        raise _CommandError(f"no .rob file under --plans: {', '.join(paths)}")
    return files


def _record_path(path) -> str:
    """A plan path as written in the delegate record: '/' separators on every OS, so records match across platforms."""
    return Path(path).as_posix()


def _load_plans(paths: list[str], err) -> list[Plan]:
    """Validate every plan file; a file that is not valid RoboLang raises RoboLangError (exit 1)."""
    plans = []
    for path in _plan_files(paths):
        data = _read_bytes(path)
        try:
            result = _compile(path, err, data)
        except RoboLangError as error:
            _print_errors(error, path, err, summary=False)
            raise _PlanInvalid from None
        plans.append(Plan(_record_path(path), data, result.ir.tasks, result.ir.language_version))
    return plans


def _delegate(args, out, err) -> int:
    for flag, value in (("--from", args.sender), ("--to", args.recipient)):
        if not valid_participant(value):
            raise _CommandError(
                f"invalid {flag} '{value}': use a Core identifier without '#', for example rl:agent:planner-1"
            )
    if args.ir is not None and not args.plans:
        raise _CommandError("--ir needs --plans: the robot's own plan library is made of .rob files")
    try:
        lease = DEFAULT_LEASE_MS if args.lease is None else lease_ms(args.lease)
    except ValueError as error:
        raise _CommandError(str(error)) from None
    inputs = _run_inputs(args, err)
    task = inputs.task
    if args.plans:
        plans = _load_plans(args.plans, err)
    else:
        plans = [Plan(_record_path(args.file), inputs.source_bytes, inputs.document.tasks,
                      inputs.document.language_version)]
    robot_world_path = args.robot_world or args.world
    robot_world = inputs.world_bytes if robot_world_path == args.world else _read_bytes(robot_world_path)
    try:
        delegation = delegate(task, inputs.world, sender=args.sender, recipient=args.recipient, library=plans,
                              robot_world=robot_world, robot_world_name=robot_world_path, lease_ms=lease,
                              core=args.core)
    except (bridge.NotExportable, ExportError) as error:
        return _export_failure(args.file, task, error, err)
    out.write(json.dumps(delegation.record, indent=2, ensure_ascii=True) + "\n")
    note = "verified" if delegation.verified else "not verified"
    if delegation.rejected:
        reasons = delegation.record["planner_verdict"]["reasons"]
        note = ", ".join(r["code"] for r in reasons)
    print(f"simulation only: delegate {delegation.goal} \u2192 {delegation.robot}: {delegation.status} ({note})",
          file=err)
    if delegation.rejected:
        return EXIT_REJECTED
    return EXIT_OK if delegation.status == "succeeded" and delegation.verified else EXIT_FAILED


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

def _reference(args, out, err) -> int:
    """Print the reference bundled in the installed package, with its final newline unchanged."""
    del args, err
    text = resources.files("robolang").joinpath("reference/language-reference.md").read_text(encoding="utf-8")
    out.write(text)
    return EXIT_OK

def _input_arguments(command: argparse.ArgumentParser, what: str, note: str = ""):
    """The input of a command: FILE, or `--ir` for a Task IR document (exactly one of them)."""
    command.add_argument("file", metavar="FILE", nargs="?", help=f"path to {what}")
    command.add_argument(
        "--ir",
        metavar="IR",
        help=(
            "path to a Task IR JSON file (schema 1.0 to 1.2) instead of FILE. It is read strictly and "
            f"re-validated in full, whatever versions it claims (spec/ir-reader-v0.1.md){note}"
        ),
    )


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

    reference = commands.add_parser(
        "reference",
        help="print the shipped RoboLang 1.3 language reference",
        description="Print the exact RoboLang 1.3 reference shipped with this installation. Nothing is executed.",
    )
    reference.set_defaults(run=_reference)

    validate = commands.add_parser(
        "validate",
        help="check a .rob file: syntax, names, types and values",
        description=(
            "Check FILE against the RoboLang language contract: syntax, names, types and values.\n"
            "The whole file is checked. Nothing is executed.\n"
            "With --world, one task is also checked against a world and its bindings for failures a run\n"
            "would certainly hit (spec/simulation-v1.0.md section 11).\n\n" + NOT_A_SAFETY_CHECK
        ),
        epilog=_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    _input_arguments(validate, "a .rob file")
    validate.add_argument(
        "--world",
        metavar="WORLD",
        help=(
            "also check one task against a world file, without running it: speed limits against the robots' "
            "profiles (S001, S002), grasp without a gripper (S211), a sensor not on the inspecting robot (S231), "
            "and the bindings (E505-E507). Each is a failure the run would certainly hit"
        ),
    )
    validate.add_argument("--task", metavar="NAME",
                          help="with --world: the task to check (required when FILE has several)")
    validate.add_argument(
        "--bind",
        metavar="NAME=ID",
        nargs="+",
        action="append",
        help="with --world: bind task parameters as for simulate, for example --bind r=arm1 box=box1",
    )
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
    _input_arguments(inspect, "a .rob file")
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
    _input_arguments(simulate, "a .rob file")
    simulate.add_argument("--world", metavar="WORLD", required=True, help="path to a world file (schema 1.0)")
    simulate.add_argument("--task", metavar="NAME", help="the task to run (required when FILE has several tasks)")
    simulate.add_argument(
        "--seed",
        metavar="SEED",
        default=DEFAULT_SEED,
        help=f"1 to 64 characters from A-Z a-z 0-9 . _ - (default {DEFAULT_SEED}); only distinguishes runs",
    )
    simulate.add_argument(
        "--bind",
        metavar="NAME=ID",
        nargs="+",
        action="append",
        help=(
            "bind a task parameter to a world entity, for example --bind r=arm1 box=box1 "
            "(repeat the flag or give several pairs; put FILE before --bind). "
            "Every parameter must be bound once. The bindings replace the world file's bindings for "
            "this task (no merging); its faults and cancellation still apply, and the task's entry "
            "may be missing"
        ),
    )
    simulate.add_argument(
        "--core", metavar="VERSION", choices=("0.1", "0.2"), default="0.1",
        help="the Core version of the record's core_outcome (default 0.1; spec/core-v0.2-draft.md)",
    )
    simulate.set_defaults(run=_simulate)

    export = commands.add_parser(
        "export-goal",
        help="print a task's goal as a signed Core 0.1 propose_goal message (simulation only)",
        description=(
            "Validate FILE, then print the goal of one task as a signed Core 0.1 propose_goal message\n"
            "(JSON) on stdout (spec/export-goal-v0.1.md). Bindings come from the world file or --bind.\n"
            "Simulation only: nothing is sent, and the signature is sim-hmac-sha256 under a published\n"
            "key, so it is not an identity proof. The message is not permission to act: the robot decides.\n\n"
            + NOT_A_SAFETY_CHECK
        ),
        epilog=_EXPORT_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    _input_arguments(export, "a .rob file (RoboLang 1.3)")
    export.add_argument("--world", metavar="WORLD", required=True, help="path to a world file (schema 1.1)")
    export.add_argument("--task", metavar="NAME", help="the task to export (required when FILE has several tasks)")
    export.add_argument("--bind", metavar="NAME=ID", nargs="+", action="append",
                        help="bind a task parameter to a world entity, as for simulate")
    export.add_argument("--from", dest="sender", metavar="AGENT_ID", required=True,
                        help="the sending agent, a Core identifier such as rl:agent:planner-1")
    export.add_argument("--to", dest="recipient", metavar="ROBOT_ID",
                        help="the receiving robot (default: the robot bound to the task's only robot parameter)")
    export.add_argument("--at", metavar="TIME",
                        help="sent_at, a Core time (default 2000-01-01T00:00:00.000Z, the simulation epoch)")
    export.add_argument(
        "--core", metavar="VERSION", choices=("0.1", "0.2"), default="0.1",
        help="the Core version: 0.2 exports one alternative per goal path (default 0.1; spec/core-v0.2-draft.md)",
    )
    export.set_defaults(run=_export_goal)

    delegation = commands.add_parser(
        "delegate",
        help="send a task's goal to a robot that runs its own plan and reports a verified outcome (simulation only)",
        description=(
            "Validate FILE, export its goal as a signed Core 0.1 propose_goal (as export-goal does), and\n"
            "let the robot named by --to match it to a task in its own plan library, run that task in\n"
            "its own world, and report the outcome. The planner verifies the outcome. The delegate record\n"
            "(JSON) is on stdout (spec/delegate-v0.1.md). The plan never travels: only the goal does.\n"
            "Simulation only: one process, no network, no ROS 2, no hardware; the signatures are\n"
            "sim-hmac-sha256 under published keys and the verifier is the simulator (ground truth).\n\n"
            + NOT_A_SAFETY_CHECK
        ),
        epilog=_DELEGATE_EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    _input_arguments(delegation, "the AI's .rob file (RoboLang 1.3)", "; --ir then needs --plans")
    delegation.add_argument("--world", metavar="WORLD", required=True,
                            help="path to the world file for the AI's task (schema 1.1)")
    delegation.add_argument("--task", metavar="NAME", help="the AI's task (required when FILE has several tasks)")
    delegation.add_argument("--bind", metavar="NAME=ID", nargs="+", action="append",
                            help="bind a parameter of the AI's task to a world entity, as for simulate")
    delegation.add_argument("--from", dest="sender", metavar="AGENT_ID", required=True,
                            help="the planning agent, a Core identifier such as rl:agent:planner-1")
    delegation.add_argument("--to", dest="recipient", metavar="ROBOT_ID", required=True,
                            help="the robot, a Core identifier such as rl:robot:arm1")
    delegation.add_argument("--plans", metavar="PATH", nargs="+",
                            help="the robot's plan library: .rob files or directories of them (default: FILE)")
    delegation.add_argument("--robot-world", metavar="WORLD",
                            help="the robot's world file, whose tasks.<name>.bindings bind the library "
                                 "(default: --world)")
    delegation.add_argument("--lease", metavar="DURATION",
                            help="the lease a peer robot offers the commitment, such as 60s, 500ms or 2min "
                                 "(default: 60s; used only when the matched plan binds a second robot)")
    delegation.add_argument(
        "--core", metavar="VERSION", choices=("0.1", "0.2"), default="0.1",
        help="the Core version of every message: 0.2 sends alternatives (default 0.1; spec/core-v0.2-draft.md)",
    )
    delegation.set_defaults(run=_delegate)

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
    parser = _parser(prog)
    try:
        args = parser.parse_args(argv)
        if hasattr(args, "ir"):
            if (args.file is None) == (args.ir is None):
                parser.error("give exactly one of FILE (a .rob file) and --ir IR (a Task IR JSON file)")
            args.file = args.ir if args.file is None else args.file
        if args.command == "validate" and args.world is None and (args.task is not None or args.bind):
            parser.error("--task and --bind need --world")
    except SystemExit as stop:  # argparse: --help / --version (0) or a usage error (2)
        return stop.code if isinstance(stop.code, int) else EXIT_USAGE
    try:
        return args.run(args, out, err)
    except RoboLangError as error:
        _print_errors(error, args.file, err, summary=args.command == "validate")
        return EXIT_INVALID
    except IRError as error:
        _print_ir_errors(error, args.file, err, summary=args.command == "validate")
        return EXIT_INVALID
    except WorldError as error:
        print(error.format(), file=err)
        return EXIT_WORLD_INVALID
    except _PlanInvalid:
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
