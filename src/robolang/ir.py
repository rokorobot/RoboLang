"""
Versioned task IR for RoboLang (spec/ir-schema-v1.0.md; schema 1.1 for RoboLang 1.2: ir-schema-v1.1.md;
schema 1.2 for RoboLang 1.3: ir-schema-v1.2.md; schema 1.3 for RoboLang 1.4: ir-schema-v1.3.md; schema 1.4 for
RoboLang 1.5: ir-schema-v1.4.md; schema 1.5 for RoboLang 1.6: ir-schema-v1.5.md; schema 1.6 for
RoboLang 1.7: ir-schema-v1.6.md; schema 1.7 for RoboLang 1.8: ir-schema-v1.7.md; schema 1.8 for RoboLang 1.9:
ir-schema-v1.8.md). RoboLang 2.0 keeps Task IR 1.8 for its tasks and adds the Mission IR 1.0 document
(`robolang.mission-ir`, spec/mission-ir-v1.0.md), which holds the tasks and the missions of a file.

The IR is a description of validated tasks with source details removed:
no spans, comments or formatting. Two sources that differ only in layout
produce identical IR.

IR produced through the compiler API comes from statically validated source.
IR objects constructed directly or received externally carry no such guarantee.
Either way the IR is data, not permission: holding one never authorizes
anything to be executed.
"""

from __future__ import annotations
import dataclasses
import json
from dataclasses import dataclass, field
from decimal import Decimal
from typing import ClassVar, Union

from . import nodes as ast
from .diagnostics import Diagnostic
from .parser import DEFAULT_VERSION
from .tokens import LIVE_CLOCK
from .validator import check, validate

IR_SCHEMA = "robolang.task-ir"
IR_SCHEMA_VERSION = "1.0"
LANGUAGE_VERSION = DEFAULT_VERSION
# Language version -> the IR schema version it compiles to (ir-schema-v1.1.md section 1).
IR_SCHEMA_VERSIONS = {"1.1": "1.0", "1.2": "1.1", "1.3": "1.2", "1.4": "1.3", "1.5": "1.4", "1.6": "1.5", "1.7": "1.6",
                     "1.8": "1.7", "1.9": "1.8", "2.0": "1.8"}  # 2.0 changes no task: its tasks are Task IR 1.8
MISSION_IR_SCHEMA = "robolang.mission-ir"
MISSION_IR_SCHEMA_VERSION = "1.0"


def _key(name: str):
    """Field whose JSON key differs from its Python attribute name."""
    return field(metadata={"key": name})


def _handler():
    """Schema 1.4 (RoboLang 1.5, decision 0033): the steps run when this step fails; None (no key) without one."""
    return field(default=None, kw_only=True, metadata={"key": "else", "omit_none": True})


@dataclass(frozen=True)
class Quantity:
    """A number with the unit written in the source. `unit` is None when none was given."""

    value: Decimal
    unit: str | None


@dataclass(frozen=True)
class Parameter:
    name: str
    type: str


@dataclass(frozen=True)
class SafetyLimit:
    """A declared limit. Declared is not enforced: nothing in this package enforces it."""

    kind: str
    limit: Quantity


@dataclass(frozen=True)
class RegionLimit:
    """
    Schema 1.6 (RoboLang 1.7, decision 0039): the robots of the task must stay out of (`kind` "no_go_region") or
    inside ("in_region") the region parameter `region`, in every motion of the task. A request, as `max_speed` is:
    nothing in this package enforces it.
    """

    kind: str
    region: str


@dataclass(frozen=True)
class Verify:
    """RoboLang 1.3 `@verify`: only the keys the source declares. Parameters of how the goal is checked."""

    min_confidence: Decimal | None = field(metadata={"omit_none": True})
    max_age: Quantity | None = field(metadata={"omit_none": True})


# -------- conditions --------

@dataclass(frozen=True)
class RobotReady:
    KIND: ClassVar[str] = "robot_ready"
    robot: str


@dataclass(frozen=True)
class RegionClear:
    KIND: ClassVar[str] = "region_clear"
    region: str


@dataclass(frozen=True)
class ObjectExists:
    KIND: ClassVar[str] = "object_exists"
    obj: str = _key("object")


@dataclass(frozen=True)
class SensorOk:
    """`sensor` is a world sensor ID in schemas 1.0 and 1.1, the name of a sensor parameter in 1.2."""

    KIND: ClassVar[str] = "sensor_ok"
    sensor: str


@dataclass(frozen=True)
class ObjectAt:
    """Schema 1.2."""

    KIND: ClassVar[str] = "object_at"
    obj: str = _key("object")
    location: str = _key("location")


@dataclass(frozen=True)
class RobotAt:
    """Schema 1.2."""

    KIND: ClassVar[str] = "robot_at"
    robot: str
    location: str


@dataclass(frozen=True)
class Holding:
    """Schema 1.2."""

    KIND: ClassVar[str] = "holding"
    robot: str
    obj: str = _key("object")


@dataclass(frozen=True)
class And:
    """Schema 1.1. Two or more operands; never directly contains an `And` (grouping is flattened)."""

    KIND: ClassVar[str] = "and"
    conditions: tuple[Condition, ...]


@dataclass(frozen=True)
class Or:
    """Schema 1.1. Two or more operands; never directly contains an `Or` (grouping is flattened)."""

    KIND: ClassVar[str] = "or"
    conditions: tuple[Condition, ...]


@dataclass(frozen=True)
class Not:
    """Schema 1.1. Never removed or pushed inward."""

    KIND: ClassVar[str] = "not"
    condition: Condition


@dataclass(frozen=True)
class BindingRef:
    """Schema 1.5: a comparison operand naming the binding of an earlier `measure`."""

    KIND: ClassVar[str] = "binding"
    name: str


@dataclass(frozen=True)
class QuantityLit:
    """Schema 1.5: a comparison operand `60 degC`: the number and unit as written, never converted."""

    KIND: ClassVar[str] = "quantity"
    value: Decimal
    unit: str


@dataclass(frozen=True)
class TextLit:
    """Schema 1.5: a comparison operand that is a string."""

    KIND: ClassVar[str] = "text"
    value: str


@dataclass(frozen=True)
class LiveReading:
    """Schema 1.5: a comparison operand read when the `wait_until` that holds it is evaluated. `reading` is a kind."""

    KIND: ClassVar[str] = "live"
    reading: str
    obj: str = _key("object")
    sensor: str = _key("sensor")


@dataclass(frozen=True)
class TimeLit:
    """Schema 1.8 (RoboLang 1.9, decision 0042 T2): a time of day, always `HH:MM:SS` (`22:00` is `22:00:00`)."""

    KIND: ClassVar[str] = "time"
    value: str


@dataclass(frozen=True)
class LiveClock:
    """Schema 1.8: `clock_time` (`reading` "time") or `clock_elapsed` ("elapsed"), read when the wait is evaluated."""

    KIND: ClassVar[str] = "clock"
    reading: str


Operand = Union[BindingRef, QuantityLit, TextLit, LiveReading, TimeLit, LiveClock]


@dataclass(frozen=True)
class InState:
    """Schema 1.7 (RoboLang 1.8, decision 0040 A2): true when the observed state of `target` is `label`."""

    KIND: ClassVar[str] = "in_state"
    target: str
    label: str


@dataclass(frozen=True)
class Compare:
    """Schema 1.5: `left op right`; `left` is a binding or a live reading, `right` a binding, quantity or text."""

    KIND: ClassVar[str] = "compare"
    left: BindingRef | LiveReading | LiveClock
    op: str
    right: BindingRef | QuantityLit | TextLit | TimeLit


Condition = Union[RobotReady, RegionClear, ObjectExists, SensorOk, ObjectAt, RobotAt, Holding, And, Or, Not, Compare,
                  InState]


# -------- steps --------

@dataclass(frozen=True)
class Move:
    KIND: ClassVar[str] = "move"
    robot: str
    target: str
    orelse: tuple[Step, ...] | None = _handler()


@dataclass(frozen=True)
class Grasp:
    KIND: ClassVar[str] = "grasp"
    robot: str
    obj: str = _key("object")
    orelse: tuple[Step, ...] | None = _handler()


@dataclass(frozen=True)
class Place:
    KIND: ClassVar[str] = "place"
    robot: str
    obj: str = _key("object")
    target: str = _key("target")
    orelse: tuple[Step, ...] | None = _handler()


@dataclass(frozen=True)
class Inspect:
    """`sensor` is a world sensor ID in schemas 1.0 and 1.1, the name of a sensor parameter in 1.2."""

    KIND: ClassVar[str] = "inspect"
    robot: str
    obj: str = _key("object")
    sensor: str = _key("sensor")
    orelse: tuple[Step, ...] | None = _handler()


@dataclass(frozen=True)
class Measure:
    """Schema 1.5: read `reading` (a kind) of `obj` with `sensor`, bind it to `binding`. A failed read has `orelse`."""

    KIND: ClassVar[str] = "measure"
    robot: str
    reading: str
    obj: str = _key("object")
    sensor: str = _key("sensor")
    binding: str = _key("binding")
    orelse: tuple[Step, ...] | None = _handler()


@dataclass(frozen=True)
class ReadClock:
    """
    Schema 1.8 (RoboLang 1.9, decision 0042 T1): read the clock of `robot` (`reading` "time", the time of day, or
    "elapsed", the time since the task started) and bind it to `binding`. It reads no object and no sensor.
    """

    KIND: ClassVar[str] = "read_clock"
    robot: str
    reading: str
    binding: str = _key("binding")
    orelse: tuple[Step, ...] | None = _handler()


@dataclass(frozen=True)
class Do:
    """
    Schema 1.7 (RoboLang 1.8, decision 0040 A1, A5): the robot is asked to do the action its profile declares, on
    `target` (an object or a location), with `value` (a quantity or a text) when given. A request: the effect counts
    only through an `in_state` condition.
    """

    KIND: ClassVar[str] = "do"
    robot: str
    action: str
    target: str
    value: QuantityLit | TextLit | None = field(default=None, metadata={"omit_none": True})
    orelse: tuple[Step, ...] | None = _handler()


@dataclass(frozen=True)
class WaitFor:
    KIND: ClassVar[str] = "wait_for"
    duration: Quantity


@dataclass(frozen=True)
class WaitUntil:
    KIND: ClassVar[str] = "wait_until"
    condition: Condition
    timeout: Quantity
    orelse: tuple[Step, ...] | None = _handler()


@dataclass(frozen=True)
class Communicate:
    """Exactly one of `message` (text) and `binding` (schema 1.5: the typed value of a `measure`) is set."""

    KIND: ClassVar[str] = "communicate"
    robot: str
    channel: str
    message: str | None = field(default=None, metadata={"omit_none": True})
    binding: str | None = field(default=None, kw_only=True, metadata={"omit_none": True})


@dataclass(frozen=True)
class If:
    """
    Schema 1.3. The condition is evaluated once, when the step is reached. `orelse` is empty when the
    source has no `else`. `goal_then` and `goal_else` are the goal conditions of that branch, or None.
    """

    KIND: ClassVar[str] = "if"
    condition: Condition
    then: tuple[Step, ...]
    orelse: tuple[Step, ...] = _key("else")
    goal_then: tuple[Condition, ...] | None
    goal_else: tuple[Condition, ...] | None


@dataclass(frozen=True)
class Fail:
    """Schema 1.3. Ends the run as failed with `reason`; only the last step of a branch."""

    KIND: ClassVar[str] = "fail"
    reason: str


Step = Union[Move, Grasp, Place, Inspect, Measure, ReadClock, Do, WaitFor, WaitUntil, Communicate, If, Fail]
HANDLED = (Move, Grasp, Place, Inspect, Measure, ReadClock, Do, WaitUntil)  # the step kinds with a failure handler


def handler_of(step: Step) -> tuple[Step, ...] | None:
    """The failure handler of a step (schema 1.4), or None."""
    return step.orelse if isinstance(step, HANDLED) else None


# -------- structure --------

@dataclass(frozen=True)
class Task:
    name: str
    parameters: tuple[Parameter, ...]
    safety: tuple[SafetyLimit | RegionLimit, ...]  # in source order; a RegionLimit is schema 1.6
    preconditions: tuple[Condition, ...]
    goal: tuple[Condition, ...] | None = field(metadata={"omit_none": True})  # 1.2 on; () if only branches give goals
    verify: Verify | None = field(default=None, kw_only=True, metadata={"omit_none": True})  # schema 1.2, if declared
    steps: tuple[Step, ...]

    @property
    def speed_limits(self) -> tuple[SafetyLimit, ...]:
        return tuple(s for s in self.safety if isinstance(s, SafetyLimit))

    @property
    def region_limits(self) -> tuple[RegionLimit, ...]:
        return tuple(s for s in self.safety if isinstance(s, RegionLimit))

    @property
    def goal_states(self) -> tuple[InState, ...]:
        """Every `in_state` atom of the goal: the task's and each `if` branch's (RoboLang 1.8)."""
        lists = [self.goal or ()]
        for _, step in walk(self.steps):
            if isinstance(step, If):
                lists += [step.goal_then or (), step.goal_else or ()]
        found: list[InState] = []

        def visit(condition: Condition):
            if isinstance(condition, InState):
                found.append(condition)
            if isinstance(condition, Not):
                visit(condition.condition)
            for inner in getattr(condition, "conditions", ()):
                visit(inner)

        for goals in lists:
            for condition in goals:
                visit(condition)
        return tuple(found)

    @property
    def typed_sensors(self) -> bool:
        """True for a schema 1.2 or 1.3 task (RoboLang 1.3, 1.4), which has a `goal` key: its sensors are parameters."""
        return self.goal is not None

    @property
    def branching(self) -> bool:
        """True when the plan has more than one path: an `if` or a `fail` (schema 1.3), or a failure handler (1.4)."""
        return any(isinstance(step, (If, Fail)) or handler_of(step) is not None for step in self.steps)


def walk(steps: tuple[Step, ...], prefix: str = ""):
    """
    Every step in source order, depth first (an `if`, then its `then` steps, then its `else` steps; a step with a
    failure handler, then the handler's steps), as (path label, step): the label is the step's position in its list,
    joined by `.then.` / `.else.` for a nested one, for example "3", "3.then.1", "3.else.2.then.1" (a handler's steps
    are under `.else.`). The step's index is its 1-based place in this order.
    """
    for position, step in enumerate(steps, start=1):
        label = f"{prefix}{position}"
        yield label, step
        if isinstance(step, If):
            yield from walk(step.then, f"{label}.then.")
            yield from walk(step.orelse, f"{label}.else.")
        elif handler_of(step) is not None:
            yield from walk(step.orelse, f"{label}.else.")


@dataclass(frozen=True)
class Run:
    """Mission IR 1.0 (RoboLang 2.0): run `task` with `arguments`, parameters of the mission, in the task's order."""

    task: str
    arguments: tuple[str, ...]


@dataclass(frozen=True)
class Repeat:
    """
    Mission IR 1.0: the runs repeat until a bound is reached or `until` holds. At least one of `max_runs` (a count),
    `max_duration` and a time-of-day `until` is there (decision 0041 M2).
    """

    max_runs: int | None = field(metadata={"omit_none": True})
    max_duration: Quantity | None = field(metadata={"omit_none": True})
    until: Condition | None = field(metadata={"omit_none": True})


@dataclass(frozen=True)
class OnFailure:
    """Mission IR 1.0: `policy` is "stop", "continue" or "retry"; `retries` (at least 1) is there for "retry" only."""

    policy: str
    retries: int | None = field(default=None, metadata={"omit_none": True})


@dataclass(frozen=True)
class Mission:
    """Mission IR 1.0 (RoboLang 2.0, decision 0041): runs tasks one after another. `repeat` is None for one run."""

    name: str
    parameters: tuple[Parameter, ...]
    repeat: Repeat | None = field(metadata={"omit_none": True})
    runs: tuple[Run, ...]
    on_failure: OnFailure


@dataclass(frozen=True)
class TaskIR:
    """The tasks of a file and, in a 2.0 file that has missions, its missions (the Mission IR document)."""

    tasks: tuple[Task, ...]
    language_version: str = LANGUAGE_VERSION
    ir_schema: str = IR_SCHEMA
    ir_schema_version: str = IR_SCHEMA_VERSION
    missions: tuple[Mission, ...] = ()

    def task(self, name: str) -> Task | None:
        return next((t for t in self.tasks if t.name == name), None)

    def mission(self, name: str) -> Mission | None:
        return next((m for m in self.missions if m.name == name), None)

    def select(self, name: str) -> TaskIR:
        """The same document restricted to one task (and no mission). Raises KeyError for an unknown name."""
        selected = self.task(name)
        if selected is None:
            raise KeyError(name)
        return dataclasses.replace(self, tasks=(selected,), missions=())


@dataclass(frozen=True)
class CompileResult:
    ir: TaskIR
    program: ast.Program
    warnings: tuple[Diagnostic, ...]


# -------- AST -> IR --------

def _quantity(node: ast.Duration | ast.Speed) -> Quantity:
    return Quantity(node.value, node.unit.value if node.unit else None)


def _condition(node: ast.Condition) -> Condition:
    if isinstance(node, (ast.And, ast.Or)):
        kind = And if isinstance(node, ast.And) else Or
        conditions: list[Condition] = []
        for operand in node.operands:
            lowered = _condition(operand)
            conditions.extend(lowered.conditions if isinstance(lowered, kind) else (lowered,))
        return kind(tuple(conditions))  # canonical form: grouping flattened, order kept (section 2.1)
    if isinstance(node, ast.Not):
        return Not(_condition(node.operand))
    if isinstance(node, ast.RobotReady):
        return RobotReady(node.robot.text)
    if isinstance(node, ast.RegionClear):
        return RegionClear(node.region.text)
    if isinstance(node, ast.ObjectExists):
        return ObjectExists(node.obj.text)
    if isinstance(node, ast.ObjectAt):
        return ObjectAt(node.obj.text, node.location.text)
    if isinstance(node, ast.RobotAt):
        return RobotAt(node.robot.text, node.location.text)
    if isinstance(node, ast.Holding):
        return Holding(node.robot.text, node.obj.text)
    if isinstance(node, ast.Compare):
        return Compare(_operand(node.left), node.op, _operand(node.right))
    if isinstance(node, ast.InState):
        return InState(node.target.text, node.label.value)
    return SensorOk(_sensor(node.sensor))


def _operand(node) -> Operand:
    if isinstance(node, ast.Name):
        return BindingRef(node.text)
    if isinstance(node, ast.QuantityLit):
        return QuantityLit(node.value, node.unit)
    if isinstance(node, ast.LiveReading):
        return LiveReading(node.kind.text, node.obj.text, _sensor(node.sensor))
    if isinstance(node, ast.LiveClock):
        return LiveClock(LIVE_CLOCK[node.word.text])
    if isinstance(node, ast.TimeLit):
        return TimeLit(time_text(node.seconds))
    return TextLit(node.value)


def time_text(seconds: int) -> str:
    """The canonical `HH:MM:SS` of a second of the day."""
    return f"{seconds // 3600:02d}:{seconds // 60 % 60:02d}:{seconds % 60:02d}"


def _sensor(node: ast.StringLit | ast.Name) -> str:
    return node.text if isinstance(node, ast.Name) else node.value


def _branch(branch: ast.Branch | None) -> tuple[tuple[Step, ...], tuple[Condition, ...] | None]:
    if branch is None:
        return (), None
    goal = tuple(_condition(c) for c in branch.goal.conditions) if branch.goal else None
    return tuple(_step(s) for s in branch.statements), goal


def _step(node: ast.Statement) -> Step:
    step = _plain_step(node)
    if getattr(node, "handler", None) is None:
        return step
    orelse, _ = _branch(node.handler)  # a handler has no goal block (decision 0034)
    return dataclasses.replace(step, orelse=orelse)


def _plain_step(node: ast.Statement) -> Step:
    if isinstance(node, ast.If):
        then, goal_then = _branch(node.then)
        orelse, goal_else = _branch(node.orelse)
        return If(_condition(node.condition), then, orelse, goal_then, goal_else)
    if isinstance(node, ast.Fail):
        return Fail(node.reason.value)
    if isinstance(node, ast.Move):
        return Move(node.robot.text, node.target.text)
    if isinstance(node, ast.Grasp):
        return Grasp(node.robot.text, node.obj.text)
    if isinstance(node, ast.Place):
        return Place(node.robot.text, node.obj.text, node.target.text)
    if isinstance(node, ast.Inspect):
        return Inspect(node.robot.text, node.obj.text, _sensor(node.sensor))
    if isinstance(node, ast.Measure):
        return Measure(node.robot.text, node.kind.text, node.obj.text, _sensor(node.sensor), node.binding.text)
    if isinstance(node, ast.MeasureClock):
        return ReadClock(node.robot.text, node.kind.text, node.binding.text)
    if isinstance(node, ast.Do):
        return Do(node.robot.text, node.action.text, node.target.text,
                  None if node.value is None else _operand(node.value))
    if isinstance(node, ast.WaitFor):
        return WaitFor(_quantity(node.duration))
    if isinstance(node, ast.WaitUntil):
        return WaitUntil(_condition(node.condition), _quantity(node.timeout))
    if isinstance(node.message, ast.Name):
        return Communicate(node.robot.text, node.channel.value, binding=node.message.text)
    return Communicate(node.robot.text, node.channel.value, node.message.value)


def _verify(node: ast.Task) -> Verify | None:
    declared = {a.key.text: a.value for a in node.verify}
    if not declared:
        return None
    max_age = declared.get("max_age")
    return Verify(declared.get("min_confidence"), _quantity(max_age) if max_age else None)


def _task(node: ast.Task, language: str) -> Task:
    goal = tuple(_condition(c) for c in node.goal.conditions) if node.goal else None
    return Task(
        name=node.name.text,
        parameters=tuple(Parameter(p.name.text, p.type.value) for p in node.params),
        safety=tuple(
            RegionLimit(a.key.text, a.region.text) if isinstance(a, ast.RegionAnnotation)
            else SafetyLimit(a.key.text, _quantity(a.value))
            for a in node.annotations
        ),
        preconditions=tuple(_condition(c) for c in node.pre.conditions) if node.pre else (),
        # 1.4 on: no task goal, the branches give them
        goal=() if goal is None and language in ("1.4", "1.5", "1.6", "1.7", "1.8", "1.9", "2.0") else goal,
        steps=tuple(_step(s) for s in node.plan.statements),
        verify=_verify(node),
    )


def _mission(node: ast.Mission) -> Mission:
    repeat = node.repeat
    failure = node.on_failure
    return Mission(
        name=node.name.text,
        parameters=tuple(Parameter(p.name.text, p.type.value) for p in node.params),
        repeat=None if repeat is None else Repeat(
            None if repeat.max_runs is None else repeat.max_runs.value,
            None if repeat.max_duration is None else _quantity(repeat.max_duration),
            None if repeat.until is None else _condition(repeat.until)),
        runs=tuple(Run(r.task.text, tuple(a.text for a in r.args)) for r in node.runs),
        on_failure=OnFailure("stop") if failure is None else
        OnFailure(failure.policy.text, None if failure.retries is None else failure.retries.value),
    )


def _lower(program: ast.Program) -> TaskIR:
    language = program.version.version if program.version else LANGUAGE_VERSION
    return TaskIR(tuple(_task(t, language) for t in program.tasks), language, IR_SCHEMA, IR_SCHEMA_VERSIONS[language],
                  tuple(_mission(m) for m in program.missions))


def compile_program(program: ast.Program, filename: str = "<string>") -> TaskIR:
    """
    Lower a parsed program to IR. The program is statically validated first, so
    an IR cannot be produced from a program that breaks a static rule.
    """
    validate(program, filename)
    return _lower(program)


def compile_source(source: str, filename: str = "<string>") -> CompileResult:
    """Lex, parse, validate and lower `source`. Raises RoboLangError listing every error."""
    result = check(source, filename)
    return CompileResult(_lower(result.program), result.program, result.warnings)


# -------- IR -> JSON --------

def decimal_text(value: Decimal) -> str:
    """Canonical exact text: plain digits, no exponent, no insignificant zeros ("0.50" -> "0.5")."""
    text = format(value, "f")
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return text


def to_dict(node):
    """JSON-ready form with a fixed key order. Numbers become exact strings, never floats."""
    if isinstance(node, Decimal):
        return decimal_text(node)
    if isinstance(node, tuple):
        return [to_dict(item) for item in node]
    if isinstance(node, TaskIR) and node.missions:  # the Mission IR document (spec/mission-ir-v1.0.md)
        return {
            "ir_schema": MISSION_IR_SCHEMA,
            "ir_schema_version": MISSION_IR_SCHEMA_VERSION,
            "task_ir_schema_version": node.ir_schema_version,
            "language_version": node.language_version,
            "tasks": to_dict(node.tasks),
            "missions": to_dict(node.missions),
        }
    if isinstance(node, TaskIR):
        return {
            "ir_schema": node.ir_schema,
            "ir_schema_version": node.ir_schema_version,
            "language_version": node.language_version,
            "tasks": to_dict(node.tasks),
        }
    if dataclasses.is_dataclass(node):
        out = {"kind": node.KIND} if hasattr(node, "KIND") else {}
        for f in dataclasses.fields(node):
            if f.metadata.get("omit_none") and getattr(node, f.name) is None:
                continue
            out[f.metadata.get("key", f.name)] = to_dict(getattr(node, f.name))
        return out
    return node


def to_json(ir: TaskIR) -> str:
    """Deterministic JSON text: fixed key order, two-space indent, ASCII only, trailing newline."""
    return json.dumps(to_dict(ir), indent=2, ensure_ascii=True) + "\n"
