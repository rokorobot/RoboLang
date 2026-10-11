"""
Static validation for RoboLang v1.1 (spec/robolang-v1.1.md, section 5).

Checks a parsed program for errors that the grammar cannot express: duplicate
and undeclared names, names used with the wrong type, out-of-range values,
conflicting speed limits and empty strings (E301-E308).

`check` and `check_file` are the complete front end: lex, parse, then validate.
They only read source text and never touch an adapter.

A program that passes is well-formed. That is a statement about the text, not
about any robot: it must never be taken to mean the task is safe to execute.
"""

from __future__ import annotations
from decimal import Decimal
from pathlib import Path
from typing import NoReturn

from . import nodes as ast
from .diagnostics import Diagnostic, RoboLangError
from .nodes import ParamType, Span
from .parser import ParseResult, parse


def _article(param_type: ParamType) -> str:
    return f"an {param_type.value}" if param_type is ParamType.OBJECT else f"a {param_type.value}"


class _Validator:
    def __init__(self, filename: str):
        self._filename = filename
        self._params: dict[str, ParamType] = {}

    def _error(self, code: str, message: str, span: Span) -> NoReturn:
        raise RoboLangError(Diagnostic(code, message, span, filename=self._filename))

    # -------- program and task --------

    def program(self, program: ast.Program):
        """Errors are reported in source order; the first one is raised."""
        seen: dict[str, ast.Name] = {}
        for task in program.tasks:
            first = seen.setdefault(task.name.text, task.name)
            if first is not task.name:
                self._error(
                    "E301",
                    f"duplicate task name '{task.name.text}' (first defined on line {first.span.line})",
                    task.name.span,
                )
            self._task(task)

    def _task(self, task: ast.Task):
        self._params = {}
        for param in task.params:
            if param.name.text in self._params:
                self._error(
                    "E302",
                    f"duplicate parameter name '{param.name.text}' in task '{task.name.text}'",
                    param.name.span,
                )
            self._params[param.name.text] = param.type

        self._annotations(task.annotations)
        if task.pre is not None:
            for condition in task.pre.conditions:
                self._condition(condition)
        for statement in task.plan.statements:
            self._statement(statement)

    # -------- @safety --------

    def _annotations(self, annotations: tuple[ast.SafetyAnnotation, ...]):
        earlier: dict[ast.SpeedUnit | None, ast.SafetyAnnotation] = {}
        for annotation in annotations:
            unit = annotation.value.unit
            key = annotation.key.text
            if unit in earlier:
                kind = f"'{unit.value}'" if unit else "unitless"
                self._error(
                    "E306",
                    f"duplicate {kind} {key} limit (first declared on line {earlier[unit].span.line}); "
                    "a task may declare at most one limit per unit type",
                    annotation.span,
                )
            if earlier and (unit is None or None in earlier):
                self._error(
                    "E308",
                    f"a unitless {key} cannot be combined with another {key} limit; "
                    "give every limit a unit ('m/s' or 'rad/s')",
                    annotation.span,
                )
            earlier[unit] = annotation
            self._positive(annotation.value.value, key, annotation.value.span)

    # -------- conditions and statements --------

    def _condition(self, condition: ast.Condition):
        if isinstance(condition, ast.RobotReady):
            self._use(condition.robot, ParamType.ROBOT, "robot_ready")
        elif isinstance(condition, ast.RegionClear):
            self._use(condition.region, ParamType.REGION, "region_clear")
        elif isinstance(condition, ast.ObjectExists):
            self._use(condition.obj, ParamType.OBJECT, "object_exists")
        else:
            self._not_empty(condition.sensor, "sensor name")

    def _statement(self, statement: ast.Statement):
        if isinstance(statement, ast.Move):
            self._use(statement.robot, ParamType.ROBOT, "move")
            self._use(statement.target, ParamType.LOCATION, "move ... to")
        elif isinstance(statement, ast.Grasp):
            self._use(statement.robot, ParamType.ROBOT, "grasp")
            self._use(statement.obj, ParamType.OBJECT, "grasp")
        elif isinstance(statement, ast.Place):
            self._use(statement.robot, ParamType.ROBOT, "place")
            self._use(statement.obj, ParamType.OBJECT, "place")
            self._use(statement.target, ParamType.LOCATION, "place ... at")
        elif isinstance(statement, ast.Inspect):
            self._use(statement.robot, ParamType.ROBOT, "inspect")
            self._use(statement.obj, ParamType.OBJECT, "inspect")
            self._not_empty(statement.sensor, "sensor name")
        elif isinstance(statement, ast.WaitFor):
            self._positive(statement.duration.value, "duration", statement.duration.span)
        elif isinstance(statement, ast.WaitUntil):
            self._condition(statement.condition)
            self._positive(statement.timeout.value, "timeout", statement.timeout.span)
        else:
            self._use(statement.robot, ParamType.ROBOT, "communicate")
            self._not_empty(statement.channel, "channel name")
            self._not_empty(statement.message, "message")

    # -------- individual rules --------

    def _use(self, name: ast.Name, expected: ParamType, context: str):
        declared = self._params.get(name.text)
        if declared is None:
            self._error("E303", f"undeclared name '{name.text}': it is not a parameter of this task", name.span)
        if declared is not expected:
            self._error(
                "E304",
                f"'{name.text}' is {_article(declared)}, but '{context}' expects {_article(expected)} here",
                name.span,
            )

    def _positive(self, value: Decimal, what: str, span: Span):
        if value <= 0:
            self._error("E305", f"{what} must be greater than zero", span)

    def _not_empty(self, string: ast.StringLit, what: str):
        if string.value == "":
            self._error("E307", f"empty string: a {what} must not be empty", string.span)


def validate(program: ast.Program, filename: str = "<string>") -> None:
    """Apply the static rules to a parsed program. Raises RoboLangError on the first violation."""
    _Validator(filename).program(program)


def check(source: str, filename: str = "<string>") -> ParseResult:
    """
    Complete front end: lex, parse and statically validate `source`.

    Returns the AST and any warnings, or raises RoboLangError on the first
    lexical, syntax or static error. Nothing is executed.
    """
    result = parse(source, filename)
    validate(result.program, filename)
    return result


def check_file(path: str | Path) -> ParseResult:
    path = Path(path)
    return check(path.read_text(encoding="utf-8"), str(path))
