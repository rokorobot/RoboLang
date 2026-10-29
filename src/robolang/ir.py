"""
Versioned task IR for RoboLang (spec/ir-schema-v1.0.md; schema 1.1 for RoboLang 1.2: ir-schema-v1.1.md;
schema 1.2 for RoboLang 1.3: ir-schema-v1.2.md; schema 1.3 for RoboLang 1.4: ir-schema-v1.3.md).

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
from .validator import check, validate

IR_SCHEMA = "robolang.task-ir"
IR_SCHEMA_VERSION = "1.0"
LANGUAGE_VERSION = DEFAULT_VERSION
# Language version -> the IR schema version it compiles to (ir-schema-v1.1.md section 1).
IR_SCHEMA_VERSIONS = {"1.1": "1.0", "1.2": "1.1", "1.3": "1.2", "1.4": "1.3"}


def _key(name: str):
    """Field whose JSON key differs from its Python attribute name."""
    return field(metadata={"key": name})


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


Condition = Union[RobotReady, RegionClear, ObjectExists, SensorOk, ObjectAt, RobotAt, Holding, And, Or, Not]


# -------- steps --------

@dataclass(frozen=True)
class Move:
    KIND: ClassVar[str] = "move"
    robot: str
    target: str


@dataclass(frozen=True)
class Grasp:
    KIND: ClassVar[str] = "grasp"
    robot: str
    obj: str = _key("object")


@dataclass(frozen=True)
class Place:
    KIND: ClassVar[str] = "place"
    robot: str
    obj: str = _key("object")
    target: str = _key("target")


@dataclass(frozen=True)
class Inspect:
    """`sensor` is a world sensor ID in schemas 1.0 and 1.1, the name of a sensor parameter in 1.2."""

    KIND: ClassVar[str] = "inspect"
    robot: str
    obj: str = _key("object")
    sensor: str = _key("sensor")


@dataclass(frozen=True)
class WaitFor:
    KIND: ClassVar[str] = "wait_for"
    duration: Quantity


@dataclass(frozen=True)
class WaitUntil:
    KIND: ClassVar[str] = "wait_until"
    condition: Condition
    timeout: Quantity


@dataclass(frozen=True)
class Communicate:
    KIND: ClassVar[str] = "communicate"
    robot: str
    channel: str
    message: str


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


Step = Union[Move, Grasp, Place, Inspect, WaitFor, WaitUntil, Communicate, If, Fail]


# -------- structure --------

@dataclass(frozen=True)
class Task:
    name: str
    parameters: tuple[Parameter, ...]
    safety: tuple[SafetyLimit, ...]
    preconditions: tuple[Condition, ...]
    goal: tuple[Condition, ...] | None = field(metadata={"omit_none": True})  # 1.2 on; () if only branches give goals
    verify: Verify | None = field(default=None, kw_only=True, metadata={"omit_none": True})  # schema 1.2, if declared
    steps: tuple[Step, ...]

    @property
    def typed_sensors(self) -> bool:
        """True for a schema 1.2 or 1.3 task (RoboLang 1.3, 1.4), which has a `goal` key: its sensors are parameters."""
        return self.goal is not None

    @property
    def branching(self) -> bool:
        """True when the plan holds an `if` or a `fail` (schema 1.3): only the front end handles such a task yet."""
        return any(isinstance(step, (If, Fail)) for step in self.steps)


def walk(steps: tuple[Step, ...], prefix: str = ""):
    """
    Every step in source order, depth first (an `if`, then its `then` steps, then its `else` steps), as
    (path label, step): the label is the step's position in its list, joined by `.then.` / `.else.` for a
    nested one, for example "3", "3.then.1", "3.else.2.then.1". The step's index is its 1-based place in this order.
    """
    for position, step in enumerate(steps, start=1):
        label = f"{prefix}{position}"
        yield label, step
        if isinstance(step, If):
            yield from walk(step.then, f"{label}.then.")
            yield from walk(step.orelse, f"{label}.else.")


@dataclass(frozen=True)
class TaskIR:
    tasks: tuple[Task, ...]
    language_version: str = LANGUAGE_VERSION
    ir_schema: str = IR_SCHEMA
    ir_schema_version: str = IR_SCHEMA_VERSION

    def task(self, name: str) -> Task | None:
        return next((t for t in self.tasks if t.name == name), None)

    def select(self, name: str) -> TaskIR:
        """The same document restricted to one task. Raises KeyError for an unknown name."""
        selected = self.task(name)
        if selected is None:
            raise KeyError(name)
        return dataclasses.replace(self, tasks=(selected,))


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
    return SensorOk(_sensor(node.sensor))


def _sensor(node: ast.StringLit | ast.Name) -> str:
    return node.text if isinstance(node, ast.Name) else node.value


def _branch(branch: ast.Branch | None) -> tuple[tuple[Step, ...], tuple[Condition, ...] | None]:
    if branch is None:
        return (), None
    goal = tuple(_condition(c) for c in branch.goal.conditions) if branch.goal else None
    return tuple(_step(s) for s in branch.statements), goal


def _step(node: ast.Statement) -> Step:
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
    if isinstance(node, ast.WaitFor):
        return WaitFor(_quantity(node.duration))
    if isinstance(node, ast.WaitUntil):
        return WaitUntil(_condition(node.condition), _quantity(node.timeout))
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
        safety=tuple(SafetyLimit(a.key.text, _quantity(a.value)) for a in node.annotations),
        preconditions=tuple(_condition(c) for c in node.pre.conditions) if node.pre else (),
        goal=() if goal is None and language == "1.4" else goal,  # 1.4: no task goal, the branches give them
        steps=tuple(_step(s) for s in node.plan.statements),
        verify=_verify(node),
    )


def _lower(program: ast.Program) -> TaskIR:
    language = program.version.version if program.version else LANGUAGE_VERSION
    return TaskIR(tuple(_task(t, language) for t in program.tasks), language, IR_SCHEMA, IR_SCHEMA_VERSIONS[language])


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
