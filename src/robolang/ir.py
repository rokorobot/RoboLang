"""
Versioned task IR for RoboLang (spec/ir-schema-v1.0.md).

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
from .parser import SUPPORTED_VERSION
from .validator import check, validate

IR_SCHEMA = "robolang.task-ir"
IR_SCHEMA_VERSION = "1.0"
LANGUAGE_VERSION = SUPPORTED_VERSION


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
    KIND: ClassVar[str] = "sensor_ok"
    sensor: str


Condition = Union[RobotReady, RegionClear, ObjectExists, SensorOk]


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


Step = Union[Move, Grasp, Place, Inspect, WaitFor, WaitUntil, Communicate]


# -------- structure --------

@dataclass(frozen=True)
class Task:
    name: str
    parameters: tuple[Parameter, ...]
    safety: tuple[SafetyLimit, ...]
    preconditions: tuple[Condition, ...]
    steps: tuple[Step, ...]


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
    if isinstance(node, ast.RobotReady):
        return RobotReady(node.robot.text)
    if isinstance(node, ast.RegionClear):
        return RegionClear(node.region.text)
    if isinstance(node, ast.ObjectExists):
        return ObjectExists(node.obj.text)
    return SensorOk(node.sensor.value)


def _step(node: ast.Statement) -> Step:
    if isinstance(node, ast.Move):
        return Move(node.robot.text, node.target.text)
    if isinstance(node, ast.Grasp):
        return Grasp(node.robot.text, node.obj.text)
    if isinstance(node, ast.Place):
        return Place(node.robot.text, node.obj.text, node.target.text)
    if isinstance(node, ast.Inspect):
        return Inspect(node.robot.text, node.obj.text, node.sensor.value)
    if isinstance(node, ast.WaitFor):
        return WaitFor(_quantity(node.duration))
    if isinstance(node, ast.WaitUntil):
        return WaitUntil(_condition(node.condition), _quantity(node.timeout))
    return Communicate(node.robot.text, node.channel.value, node.message.value)


def _task(node: ast.Task) -> Task:
    return Task(
        name=node.name.text,
        parameters=tuple(Parameter(p.name.text, p.type.value) for p in node.params),
        safety=tuple(SafetyLimit(a.key.text, _quantity(a.value)) for a in node.annotations),
        preconditions=tuple(_condition(c) for c in node.pre.conditions) if node.pre else (),
        steps=tuple(_step(s) for s in node.plan.statements),
    )


def _lower(program: ast.Program) -> TaskIR:
    return TaskIR(tuple(_task(t) for t in program.tasks))


def compile_program(program: ast.Program, filename: str = "<string>") -> TaskIR:
    """
    Lower a parsed program to IR. The program is statically validated first, so
    an IR cannot be produced from a program that breaks a static rule.
    """
    validate(program, filename)
    return _lower(program)


def compile_source(source: str, filename: str = "<string>") -> CompileResult:
    """Lex, parse, validate and lower `source`. Raises RoboLangError on the first error."""
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
            out[f.metadata.get("key", f.name)] = to_dict(getattr(node, f.name))
        return out
    return node


def to_json(ir: TaskIR) -> str:
    """Deterministic JSON text: fixed key order, two-space indent, ASCII only, trailing newline."""
    return json.dumps(to_dict(ir), indent=2, ensure_ascii=True) + "\n"
