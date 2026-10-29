"""
Static validation for RoboLang v1.1 to v1.4 (spec/robolang-v1.1.md, section 5; spec/robolang-v1.3.md, section 4;
spec/robolang-v1.4.md, section 4).

Checks a parsed program for errors that the grammar cannot express: duplicate
and undeclared names, names used with the wrong type, out-of-range values,
conflicting speed limits and empty strings (E301-E308), and in 1.3 a missing goal, a string
where a sensor parameter belongs, and a repeated or out-of-range @verify (E309-E312); in 1.4 a path
with no goal (E309) and a misplaced `fail` (E313).

`check` and `check_file` are the complete front end: lex, parse, then validate.
They only read source text and never touch an adapter.

A program that passes is well-formed. That is a statement about the text, not
about any robot: it must never be taken to mean the task is safe to execute.
"""

from __future__ import annotations
from decimal import Decimal
from pathlib import Path

from . import nodes as ast
from .diagnostics import MAX_ERRORS, Diagnostic, RoboLangError
from .nodes import ParamType, Span
from .paths import goalless_path
from .parser import ParseResult, parse_recovering


def _parts(statement: ast.Statement):
    """The shape `paths.goalless_path` reads: None for a plain statement, "fail", or the two branches of an if."""
    if isinstance(statement, ast.Fail):
        return "fail"
    if not isinstance(statement, ast.If):
        return None
    orelse = statement.orelse
    return (
        statement.then.statements, statement.then.goal is not None,
        orelse.statements if orelse else (), bool(orelse and orelse.goal is not None),
    )


def _article(param_type: ParamType) -> str:
    return f"an {param_type.value}" if param_type is ParamType.OBJECT else f"a {param_type.value}"


class _TooMany(Exception):
    """More errors than are reported: stop validating."""


class _Reported(Exception):
    """An error was recorded for the item being checked (a statement, a condition atom...); skip its other checks."""


class _Validator:
    def __init__(self, filename: str):
        self._filename = filename
        self._params: dict[str, ParamType] = {}
        self.errors: list[Diagnostic] = []
        self._typed_sensors = False  # RoboLang 1.3 and 1.4: sensors are parameters
        self._branching = False  # RoboLang 1.4

    def _report(self, code: str, message: str, span: Span):
        self.errors.append(Diagnostic(code, message, span, filename=self._filename))
        if len(self.errors) > MAX_ERRORS:  # one more than is listed, so "too many" is known
            raise _TooMany

    def _error(self, code: str, message: str, span: Span):
        """
        Record an error and end the check of the current item: one error per statement,
        parameter, annotation or condition atom, so one mistake does not report twice.
        """
        self._report(code, message, span)
        raise _Reported

    def _checked(self, check, *args):
        try:
            check(*args)
        except _Reported:
            pass

    # -------- program and task --------

    def program(self, program: ast.Program):
        """Collect every error, in source order."""
        try:
            self._program(program)
        except _TooMany:
            pass

    def _program(self, program: ast.Program):
        language = program.version.version if program.version is not None else None
        self._typed_sensors = language in ("1.3", "1.4")
        self._branching = language == "1.4"
        seen: dict[str, ast.Name] = {}
        for task in program.tasks:
            first = seen.setdefault(task.name.text, task.name)
            if first is not task.name:
                self._report(
                    "E301",
                    f"duplicate task name '{task.name.text}' (first defined on line {first.span.line})",
                    task.name.span,
                )
            self._task(task)  # a duplicate name does not stop the body from being checked

    def _task(self, task: ast.Task):
        self._params = {}
        for param in task.params:
            if param.name.text in self._params:
                self._report(
                    "E302",
                    f"duplicate parameter name '{param.name.text}' in task '{task.name.text}'",
                    param.name.span,
                )
                continue  # the first declaration stands, so uses are not blamed on the duplicate
            self._params[param.name.text] = param.type

        self._annotations(task.annotations)
        self._verify(task.verify)
        if task.pre is not None:
            for condition in task.pre.conditions:
                self._condition(condition)
        self._goal(task)
        self._statements(task.plan.statements, branch=False)

    def _statements(self, statements: tuple[ast.Statement, ...], branch: bool):
        """`branch`: the statements are the body of an if or else, where `fail` may be the last one (E313)."""
        for i, statement in enumerate(statements):
            if isinstance(statement, ast.Fail) and not (branch and i == len(statements) - 1):
                where = "the last statement of an if or else branch" if branch else "inside an if or else branch"
                self._report("E313", f"'fail' is allowed only as {where}; nothing may follow it", statement.span)
            self._checked(self._statement, statement)

    def _goal(self, task: ast.Task):
        if task.goal is not None:
            for condition in task.goal.conditions:
                self._condition(condition)
        elif self._branching:
            label = goalless_path(task.plan.statements, False, _parts)
            if label is not None:
                where = f"path {label}" if label else "the plan"
                self._report(  # not _error: the plan must still be checked
                    "E309",
                    f"task '{task.name.text}' has no goal on {where}: RoboLang 1.4 needs a task 'goal' block, "
                    "or a 'goal' block in the branch, on every path that does not end in 'fail'",
                    task.name.span,
                )
        elif self._typed_sensors:
            self._report(  # not _error: a missing goal must not stop the plan from being checked
                "E309",
                f"task '{task.name.text}' has no goal block: RoboLang 1.3 requires "
                "'goal { <condition>; ... }' with at least one condition, between pre and plan",
                task.name.span,
            )

    # -------- @safety --------

    def _annotations(self, annotations: tuple[ast.SafetyAnnotation, ...]):
        earlier: dict[ast.SpeedUnit | None, ast.SafetyAnnotation] = {}
        for annotation in annotations:
            self._checked(self._annotation, annotation, earlier)
            earlier.setdefault(annotation.value.unit, annotation)  # the first declaration stands

    def _annotation(self, annotation: ast.SafetyAnnotation, earlier: dict[ast.SpeedUnit | None, ast.SafetyAnnotation]):
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
            return
        if earlier and (unit is None or None in earlier):
            self._error(
                "E308",
                f"a unitless {key} cannot be combined with another {key} limit; "
                "give every limit a unit ('m/s' or 'rad/s')",
                annotation.span,
            )
            return
        self._positive(annotation.value.value, key, annotation.value.span)

    # -------- @verify (RoboLang 1.3) --------

    def _verify(self, annotations: tuple[ast.VerifyAnnotation, ...]):
        earlier: dict[str, ast.VerifyAnnotation] = {}
        for annotation in annotations:
            self._checked(self._verify_annotation, annotation, earlier)
            earlier.setdefault(annotation.key.text, annotation)  # the first declaration stands

    def _verify_annotation(self, annotation: ast.VerifyAnnotation, earlier: dict[str, ast.VerifyAnnotation]):
        key = annotation.key.text
        if key in earlier:
            self._error(
                "E311",
                f"duplicate @verify {key} (first declared on line {earlier[key].span.line}); "
                "a task may declare each @verify key at most once",
                annotation.span,
            )
        if key == "min_confidence":
            if annotation.value > 1:
                self._error("E312", "min_confidence must be between 0 and 1", annotation.value_span)
        else:
            self._positive(annotation.value.value, key, annotation.value_span)

    # -------- conditions and statements --------

    def _condition(self, condition: ast.Condition):
        if isinstance(condition, (ast.And, ast.Or)):
            for operand in condition.operands:
                self._condition(operand)
        elif isinstance(condition, ast.Not):
            self._condition(condition.operand)
        else:
            self._checked(self._atom, condition)

    def _atom(self, condition: ast.Condition):
        if isinstance(condition, ast.RobotReady):
            self._use(condition.robot, ParamType.ROBOT, "robot_ready")
        elif isinstance(condition, ast.RegionClear):
            self._use(condition.region, ParamType.REGION, "region_clear")
        elif isinstance(condition, ast.ObjectExists):
            self._use(condition.obj, ParamType.OBJECT, "object_exists")
        elif isinstance(condition, ast.ObjectAt):
            self._use(condition.obj, ParamType.OBJECT, "object_at")
            self._use(condition.location, ParamType.LOCATION, "object_at")
        elif isinstance(condition, ast.RobotAt):
            self._use(condition.robot, ParamType.ROBOT, "robot_at")
            self._use(condition.location, ParamType.LOCATION, "robot_at")
        elif isinstance(condition, ast.Holding):
            self._use(condition.robot, ParamType.ROBOT, "holding")
            self._use(condition.obj, ParamType.OBJECT, "holding")
        else:
            self._sensor(condition.sensor, "sensor_ok")

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
            self._sensor(statement.sensor, "inspect ... using")
        elif isinstance(statement, ast.WaitFor):
            self._positive(statement.duration.value, "duration", statement.duration.span)
        elif isinstance(statement, ast.WaitUntil):
            self._condition(statement.condition)
            self._positive(statement.timeout.value, "timeout", statement.timeout.span)
        elif isinstance(statement, ast.If):
            self._condition(statement.condition)
            for branch in (statement.then, statement.orelse):
                if branch is not None:
                    if branch.goal is not None:
                        for condition in branch.goal.conditions:
                            self._condition(condition)
                    self._statements(branch.statements, branch=True)
        elif isinstance(statement, ast.Fail):
            self._not_empty(statement.reason, "reason")
        else:
            self._use(statement.robot, ParamType.ROBOT, "communicate")
            self._not_empty(statement.channel, "channel name")
            self._not_empty(statement.message, "message")

    # -------- individual rules --------

    def _sensor(self, sensor: ast.StringLit | ast.Name, context: str):
        if isinstance(sensor, ast.Name):
            self._use(sensor, ParamType.SENSOR, context)
        elif self._typed_sensors:
            self._error(
                "E310",
                f"a sensor is a parameter in RoboLang 1.3, not a string: declare a 'sensor' parameter "
                f"(for example 'sensor cam') and write it without quotes after '{context}'",
                sensor.span,
            )
        else:
            self._not_empty(sensor, "sensor name")

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


def collect(program: ast.Program, filename: str = "<string>") -> list[Diagnostic]:
    """Apply the static rules to a parsed program. Returns every violation, in source order, capped."""
    validator = _Validator(filename)
    validator.program(program)
    return validator.errors


def validate(program: ast.Program, filename: str = "<string>") -> None:
    """Apply the static rules to a parsed program. Raises RoboLangError listing every violation."""
    errors = collect(program, filename)
    if errors:
        raise RoboLangError(errors)


def check(source: str, filename: str = "<string>") -> ParseResult:
    """
    Complete front end: lex, parse and statically validate `source`.

    Returns the AST and any warnings, or raises RoboLangError listing every error found
    (`.diagnostics`, in source order; `.diagnostic` is the first). A lexical error is fatal and
    is the only one reported. A task with a syntax error is not validated; other tasks are.
    Nothing is executed.
    """
    parsed = parse_recovering(source, filename)
    errors = sorted([*parsed.errors, *collect(parsed.program, filename)], key=lambda d: (d.line, d.column))
    if errors:
        raise RoboLangError(errors, warnings=parsed.warnings)
    return ParseResult(parsed.program, parsed.warnings)


def check_file(path: str | Path) -> ParseResult:
    path = Path(path)
    return check(path.read_text(encoding="utf-8"), str(path))
