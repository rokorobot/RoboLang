"""
Static validation for RoboLang v1.1 to v2.1 (spec/robolang-v1.1.md, section 5; spec/robolang-v1.3.md, section 4;
spec/robolang-v1.4.md, section 4; spec/robolang-v1.6.md, section 6 for bindings and kinds).

Checks a parsed program for errors that the grammar cannot express: duplicate
and undeclared names, names used with the wrong type, out-of-range values,
conflicting speed limits and empty strings (E301-E308), and in 1.3 a missing goal, a string
where a sensor parameter belongs, and a repeated or out-of-range @verify (E309-E312); in 1.4 a path
with no goal (E309) and a misplaced `fail` (E313); in every version a grasp of something still held (E315).

`check` and `check_file` are the complete front end: lex, parse, then validate.
They only read source text and never touch an adapter.

A program that passes is well-formed. That is a statement about the text, not
about any robot: it must never be taken to mean the task is safe to execute.
"""

from __future__ import annotations
from contextlib import contextmanager
from decimal import Decimal
from pathlib import Path

from . import nodes as ast
from .diagnostics import MAX_ERRORS, Diagnostic, RoboLangError
from .measured import AREA_KINDS, Atom, Operand, Scopes, View
from .nodes import ParamType, Span
from .paths import double_grasps, goalless_path
from .parser import ParseResult, parse_recovering
from .tokens import CHARGE_KIND, LIVE_CLOCK, MAX_MISSION_RUNS, MAX_REGION_LIMITS, MAX_RETRIES, since


def _parts(statement: ast.Statement):
    """
    The shape `paths.goalless_path` reads: None for a plain statement, "fail", the two branches of an if, or for a
    1.5 step with a failure handler its two outcomes: "ok" (nothing more) and "failed" (the handler).
    """
    if isinstance(statement, ast.Fail):
        return "fail"
    handler = getattr(statement, "handler", None)
    if handler is not None:
        return (), False, handler.statements, False, ("ok", "failed")
    if not isinstance(statement, ast.If):
        return None
    orelse = statement.orelse
    return (
        statement.then.statements, statement.then.goal is not None,
        orelse.statements if orelse else (), bool(orelse and orelse.goal is not None),
    )


def _holding_kinds(statement: ast.Statement):
    """The shape `paths.double_grasps` reads."""
    if isinstance(statement, ast.Fail):
        return "fail"
    if isinstance(statement, ast.If):
        orelse = statement.orelse.statements if statement.orelse else ()
        return "if", statement.then.statements, orelse, _holding_test(statement.condition)
    handler = statement.handler.statements if getattr(statement, "handler", None) is not None else None
    if isinstance(statement, (ast.Grasp, ast.Place)):
        kind = "grasp" if isinstance(statement, ast.Grasp) else "place"
        return kind, statement.robot.text, statement.obj.text, handler
    return None if handler is None else ("step", None, None, handler)


def _holding_test(condition, positive: bool = True):
    """The `test` of `paths.double_grasps` for an `if` condition: a holding or object_at atom, or its negation."""
    if isinstance(condition, ast.Not):
        return _holding_test(condition.operand, not positive)
    if isinstance(condition, ast.Holding):
        return "holding", condition.robot.text, condition.obj.text, positive
    if isinstance(condition, ast.ObjectAt):
        return "object_at", condition.obj.text, positive
    return None


_VALUE_CODES = {"scope": "E317", "clash": "E318", "kind": "E319", "operator": "E320", "goal": "E321", "live": "E322",
                # RoboLang 2.2 (decision 0044): a number needs its unit; a count is a whole number
                "unit": "E206", "count": "E328"}


def _operand(node) -> Operand:
    """One side of an AST comparison, as `measured.Scopes` reads it."""
    if isinstance(node, ast.Name):
        return Operand("binding", node.span, name=node.text)
    if isinstance(node, ast.QuantityLit):
        whole = None if node.unit is not None else node.value == node.value.to_integral_value()
        return Operand("quantity", node.span, unit=node.unit, whole=whole)
    if isinstance(node, ast.LiveReading):
        return Operand("live", node.span, kind=node.kind.text)
    if isinstance(node, ast.LiveClock):  # RoboLang 1.9
        return Operand("clock", node.span, kind=LIVE_CLOCK[node.word.text])
    if isinstance(node, ast.LiveCharge):  # RoboLang 2.4
        return Operand("charge", node.span, kind=CHARGE_KIND)
    if isinstance(node, ast.TimeLit):
        return Operand("time", node.span)
    return Operand("text", node.span)


def _atoms(condition: ast.Condition):
    """The comparisons in a condition, in source order."""
    if isinstance(condition, (ast.And, ast.Or)):
        for operand in condition.operands:
            yield from _atoms(operand)
    elif isinstance(condition, ast.Not):
        yield from _atoms(condition.operand)
    elif isinstance(condition, ast.Compare):
        yield Atom(_operand(condition.left), condition.op, _operand(condition.right), condition.span)


def _view(statement: ast.Statement) -> View:
    """The shape `measured.Scopes` reads from a statement."""
    if isinstance(statement, ast.Fail):
        return View("fail")
    if isinstance(statement, ast.If):
        goals = tuple(c for branch in (statement.then, statement.orelse) if branch is not None and branch.goal
                      for c in branch.goal.conditions)
        return View("if", statement.condition, goals=goals, then=statement.then.statements,
                    orelse=statement.orelse.statements if statement.orelse else ())
    handler = statement.handler.statements if getattr(statement, "handler", None) is not None else None
    if isinstance(statement, (ast.Measure, ast.MeasureClock, ast.MeasureCharge)):
        return View(handler=handler, measure=(statement.binding.text, statement.kind.text, statement.binding.span))
    if isinstance(statement, ast.WaitUntil):
        return View(condition=statement.condition, live_ok=True, handler=handler)
    if isinstance(statement, ast.Communicate) and isinstance(statement.message, ast.Name):
        return View(send=_operand(statement.message))
    return View(handler=handler)


def time_bound(condition: ast.Condition | None) -> bool:
    """Clarification 2 of decision 0041: `clock_time >= T` (or `>`), alone or as one operand of an `or`, is a bound."""
    if isinstance(condition, ast.Or):
        return any(time_bound(operand) for operand in condition.operands)
    return (isinstance(condition, ast.Compare) and isinstance(condition.left, ast.LiveClock)
            and condition.left.word.text == "clock_time" and condition.op in (">", ">=")
            and isinstance(condition.right, ast.TimeLit))


def double_grasp_message(robot: str, obj: str, holder: str, held: str, earlier: str) -> str:
    """E315 and its IR form I216 (decision 0032)."""
    if holder == robot:
        return f"'grasp {robot} {obj}': '{robot}' already holds '{held}' (grasped at {earlier}); place it first"
    return f"'grasp {robot} {obj}': '{obj}' is still held by '{holder}' (grasped at {earlier}); place it first"


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
        self._typed_sensors = False  # RoboLang 1.3 to 1.8: sensors are parameters
        self._branching = False  # RoboLang 1.4 to 1.8
        self._handlers = False  # RoboLang 1.5 to 1.8
        self._values = False  # RoboLang 1.6 to 1.8
        self._kinds = False  # RoboLang 2.2: four more kinds and `%`
        self._language = None
        self._mission_names: set[str] = set()
        self._chosen: set[str] = set()  # RoboLang 2.3: the names a `choose` binds, visible here (also in _params)
        self._measured: set[str] = set()  # the names a `measure` has bound so far in the plan
        self._owner = "task"  # what the names in scope are parameters of: "task", or "mission" (RoboLang 2.0)

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
        self._language = language
        self._typed_sensors = language in since("1.3")
        self._branching = language in since("1.4")
        self._handlers = language in since("1.5")
        self._values = language in since("1.6")
        self._kinds = language in since("2.2")
        self._charging_language = language in since("2.4")
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
        tasks = {t.name.text: t for t in program.tasks}
        self._mission_names = {m.name.text for m in program.missions}
        for mission in program.missions:  # RoboLang 2.0: a name is a task's or a mission's, not both (E301)
            first = seen.setdefault(mission.name.text, mission.name)
            if first is not mission.name:
                self._report("E301", f"duplicate name '{mission.name.text}': a mission and a task, or two missions, "
                                     f"cannot share a name (first defined on line {first.span.line})",
                             mission.name.span)
            self._mission(mission, tasks)

    # -------- mission (RoboLang 2.0, decision 0041 M7) --------

    def _mission(self, mission: ast.Mission, tasks: dict[str, ast.Task]):
        self._params = {}
        self._owner = "mission"
        for param in mission.params:
            if param.name.text in self._params:
                self._report("E302", f"duplicate parameter name '{param.name.text}' in mission "
                                     f"'{mission.name.text}'", param.name.span)
                continue
            self._params[param.name.text] = param.type
        for run in mission.runs:
            self._checked(self._run, run, tasks)
        repeat = mission.repeat
        if repeat is not None:
            self._checked(self._bound, repeat)
            if repeat.until is not None:
                self._condition(repeat.until)
                scopes = Scopes({p: t.value for p, t in self._params.items()},
                                lambda c: View(condition=c, live_ok=True), _atoms,
                                lambda rule, message, at: self._report(_VALUE_CODES[rule], message, at))
                scopes.plan((repeat.until,))  # a live value or the clock; no `measure` binding exists in a mission
        failure = mission.on_failure
        if failure is not None and failure.retries is not None:
            self._checked(self._count, failure.retries, "retry", MAX_RETRIES)

    def _count(self, count: ast.Count, what: str, most: int):
        if count.value <= 0:
            self._error("E305", f"{what} must be greater than zero", count.span)
        if count.value > most:
            self._error("E325", f"{what} is at most {most}; this one is {count.value}", count.span)

    def _bound(self, repeat: ast.Repeat):
        """M2: every repeat has a hard bound: a run count, a duration, or an `until clock_time >= T`."""
        if repeat.max_runs is not None:
            self._count(repeat.max_runs, "the number of runs", MAX_MISSION_RUNS)
        if repeat.max_duration is not None:
            self._positive(repeat.max_duration.value, "the duration bound", repeat.max_duration.span)
        if repeat.max_runs is None and repeat.max_duration is None and not time_bound(repeat.until):
            self._error("E325", "a repeat needs a hard bound (decision 0041 M2): 'at most N runs', 'for at most "
                                "D', or an 'until clock_time >= T' that ends it at a time of day", repeat.span)

    def _run(self, run: ast.RunLine, tasks: dict[str, ast.Task]):
        """E326, E303, E304: a `run` names a task of the file and gives it arguments of the right types."""
        name = run.task.text
        task = tasks.get(name)
        if task is None and name in self._mission_names:
            self._error("E326", f"'run {name}': '{name}' is a mission, and a mission cannot run a mission "
                                "(decision 0041 M7): run a task", run.task.span)
        if task is None:
            names = ", ".join(sorted(tasks)) or "(none)"
            self._error("E326", f"'run {name}': there is no task '{name}' in this file; tasks: {names}", run.task.span)
        if len(run.args) != len(task.params):
            wanted = ", ".join(p.name.text for p in task.params)
            self._error("E326", f"'run {name}' takes {len(task.params)} argument"
                                f"{'' if len(task.params) == 1 else 's'} ({wanted}), found {len(run.args)}", run.span)
        for arg, param in zip(run.args, task.params):
            declared = self._params.get(arg.text)
            if declared is None:
                self._report("E303", f"undeclared name '{arg.text}': it is not a parameter of this mission", arg.span)
            elif declared is not param.type:
                self._report("E304", f"'{arg.text}' is {_article(declared)}, but parameter '{param.name.text}' of task "
                                     f"'{name}' expects {_article(param.type)} here", arg.span)

    def _task(self, task: ast.Task):
        self._params = {}
        self._chosen = set()
        self._measured = set()
        self._owner = "task"
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
        with self._goal_names(task.plan.statements):
            self._goal(task)
        task_params = dict(self._params)  # the parameters alone: a name a `choose` binds is no parameter of the task
        before = len(self.errors)
        self._statements(task.plan.statements, branch=False)
        if self._values:
            self._bindings(task, task_params)
        if len(self.errors) == before:  # names and types are right, so the plan's grasps can be followed
            self._holders(task.plan.statements)

    def _bindings(self, task: ast.Task, params: dict[str, ParamType]):
        """RoboLang 1.6 (decision 0036): bindings, kinds, units and where a comparison may stand."""
        scopes = Scopes({p: t.value for p, t in params.items()}, _view, _atoms,
                        lambda rule, message, at: self._report(_VALUE_CODES[rule], message, at))
        scopes.forbid((c for block in (task.pre, task.goal) if block is not None for c in block.conditions),
                      "a 'pre' or 'goal' block")
        scopes.plan(task.plan.statements)

    def _holders(self, statements: tuple[ast.Statement, ...]):
        """E315 (decision 0032): a grasp that fails whenever its path is taken, as the object or robot is held."""
        for grasp, earlier, holder, held in double_grasps(statements, _holding_kinds):
            at = f"{earlier.span.line}:{earlier.span.column}"
            self._report("E315", double_grasp_message(grasp.robot.text, grasp.obj.text, holder, held, at), grasp.span)

    def _statements(self, statements: tuple[ast.Statement, ...], branch: bool):
        """
        `branch`: the statements are the body of an if or else, where `fail` may be the last one (E313). A name that a
        `choose` binds (RoboLang 2.3) is visible from the end of that statement to the end of its block.
        """
        outer_params, outer_chosen = dict(self._params), set(self._chosen)
        for i, statement in enumerate(statements):
            if isinstance(statement, ast.Fail) and not (branch and i == len(statements) - 1):
                where = "the last statement of an if or else branch" if branch else "inside an if or else branch"
                self._report("E313", f"'fail' is allowed only as {where}; nothing may follow it", statement.span)
            self._checked(self._statement, statement)
            self._handler(statement)
            if isinstance(statement, ast.Choose):
                self._bind_chosen(statement)
        if branch:
            self._params, self._chosen = outer_params, outer_chosen

    @contextmanager
    def _goal_names(self, statements: tuple[ast.Statement, ...]):
        """
        RoboLang 2.3 (decision 0045 C7): a goal block may name what a `choose` at the top level of its own plan or
        branch binds, though the block comes first. Those names are visible to the goal and then taken away again.
        """
        added = {}
        for statement in statements:
            if isinstance(statement, ast.Choose) and statement.name.text not in self._params:
                added[statement.name.text] = ParamType(statement.type.text)
        self._params.update(added)
        try:
            yield
        finally:
            for name in added:
                self._params.pop(name, None)

    def _bind_chosen(self, statement: ast.Choose):
        name = statement.name.text
        if name not in self._params:  # a clash was reported by the statement itself
            self._params[name] = ParamType(statement.type.text)
        self._chosen.add(name)

    def _handler(self, statement: ast.Statement):
        """RoboLang 1.5 (decision 0033): a `wait until` needs a failure handler (E316); a handler is a branch."""
        handler = getattr(statement, "handler", None)
        if self._handlers and isinstance(statement, ast.WaitUntil) and handler is None:
            self._report("E316", f"in RoboLang {self._language} a 'wait until' needs a failure handler: add "
                                 "'else { ... }' to say what happens when it times out", statement.span)
        if handler is not None:  # a branch without a goal block (decision 0034)
            self._statements(handler.statements, branch=True)

    def _goal_condition(self, condition: ast.Condition):
        """A condition of a `goal` block, or of the goal of an `if` branch: never a message (decision 0043 S1)."""
        self._condition(condition)
        stack = [condition]
        while stack:
            node = stack.pop()
            if isinstance(node, (ast.And, ast.Or)):
                stack.extend(node.operands)
            elif isinstance(node, ast.Not):
                stack.append(node.operand)
            elif isinstance(node, ast.MessageReceived):
                self._report("E327", "'message_received' cannot be in a goal: a message is not an end state of the "
                                     "world (decision 0043 S1); wait for it in the plan with 'wait until "
                                     "message_received ...'", node.span)

    def _goal(self, task: ast.Task):
        if task.goal is not None:
            for condition in task.goal.conditions:
                self._goal_condition(condition)
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

    def _annotations(self, annotations: tuple[ast.SafetyAnnotation | ast.RegionAnnotation, ...]):
        earlier: dict[ast.SpeedUnit | None, ast.SafetyAnnotation] = {}
        regions: dict[str, ast.RegionAnnotation] = {}
        for annotation in annotations:
            if isinstance(annotation, ast.RegionAnnotation):
                self._checked(self._region_limit, annotation, regions)
                continue
            self._checked(self._annotation, annotation, earlier)
            earlier.setdefault(annotation.value.unit, annotation)  # the first declaration stands

    def _region_limit(self, annotation: ast.RegionAnnotation, earlier: dict[str, ast.RegionAnnotation]):
        """RoboLang 1.7 (decision 0039 P4): a region parameter (E303, E304), named once (E323), at most 8 (E324)."""
        region = annotation.region
        self._use(region, ParamType.REGION, annotation.key.text)
        first = earlier.get(region.text)
        if first is not None:
            same = first.key.text == annotation.key.text
            what = f"is already a {first.key.text}"
            if not same:
                what += f", and cannot also be a {annotation.key.text}"
            self._error("E323", f"region '{region.text}' {what} (line {first.span.line}); "
                                "a task names a region in one region limit only", annotation.span)
        if len(earlier) >= MAX_REGION_LIMITS:
            self._error("E324", f"a task may declare at most {MAX_REGION_LIMITS} region limits", annotation.span)
        earlier[region.text] = annotation

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
        elif isinstance(condition, ast.MessageReceived):  # RoboLang 2.1 (decision 0043 S1, C11)
            self._not_empty(condition.channel, "channel name")
            if condition.text is not None and condition.text.value.strip(" ") == "":
                self._error("E307", "empty string: a message text must not be empty, or only spaces, which are "
                                    "trimmed (decision 0043 S3)", condition.text.span)
        elif isinstance(condition, ast.InState):  # RoboLang 1.8 (decision 0040 A6)
            self._use_any(condition.target, "in_state")
            self._not_empty(condition.label, "state label")
        elif isinstance(condition, ast.Compare):
            if isinstance(condition.left, ast.LiveReading):  # a binding is checked by `_bindings`
                self._live(condition.left)
            elif isinstance(condition.left, ast.LiveCharge):  # RoboLang 2.4 (decision 0048 S3): a robot parameter
                self._use(condition.left.robot, ParamType.ROBOT, "robot_charge")
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
        elif isinstance(statement, ast.Choose):
            self._choose(statement)
        elif isinstance(statement, ast.Measure):
            self._binding_is_new(statement.binding)
            self._measured.add(statement.binding.text)
            self._use(statement.robot, ParamType.ROBOT, "measure")
            if statement.kind.text in AREA_KINDS and self._kinds:  # RoboLang 2.2 (decision 0044 D1): or a location
                self._use_any(statement.obj, f"measure {statement.kind.text}")
            else:
                self._use(statement.obj, ParamType.OBJECT, "measure")
            self._sensor(statement.sensor, "measure ... using")
        elif isinstance(statement, ast.MeasureClock):  # RoboLang 1.9 (decision 0042 T1): the robot is recorded
            self._binding_is_new(statement.binding)
            self._measured.add(statement.binding.text)
            self._use(statement.robot, ParamType.ROBOT, "measure")
        elif isinstance(statement, ast.MeasureCharge):  # RoboLang 2.4 (decision 0048 S1, point 4): any robot parameter
            self._binding_is_new(statement.binding)
            self._measured.add(statement.binding.text)
            self._use(statement.robot, ParamType.ROBOT, "measure")
        elif isinstance(statement, ast.Do):  # RoboLang 1.8 (decision 0040 A6): the action is checked against the world
            self._use(statement.robot, ParamType.ROBOT, "do")
            self._use_any(statement.target, f"do ... {statement.action.text}")
            if isinstance(statement.value, ast.StringLit):
                self._not_empty(statement.value, "value")
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
                        with self._goal_names(branch.statements):
                            for condition in branch.goal.conditions:
                                self._goal_condition(condition)
                    self._statements(branch.statements, branch=True)
        elif isinstance(statement, ast.Fail):
            self._not_empty(statement.reason, "reason")
            problem = fail_reason_problem(statement.reason.value)
            if problem:
                self._error("E314", problem, statement.reason.span)
        else:
            self._use(statement.robot, ParamType.ROBOT, "communicate")
            self._not_empty(statement.channel, "channel name")
            if isinstance(statement.message, ast.StringLit):  # a Name is a binding, checked by `_bindings`
                self._not_empty(statement.message, "message")

    # -------- choose (RoboLang 2.3, decision 0045) --------

    def _candidate(self, name: ast.Name, expected: ParamType, context: str):
        """A candidate, the place of `objects at` or the reference of an order: a parameter, never an earlier choice."""
        if name.text in self._chosen:
            self._error("E331", f"'{name.text}' was chosen earlier: a choice cannot use another choice, as several "
                                "choices that depend on each other are not in RoboLang 2.3 (decision 0045 C9); "
                                "use a parameter", name.span)
        self._use(name, expected, context)

    def _binding_is_new(self, binding: ast.Name):
        """E318: a binding of `measure` must not share a name with an entity a `choose` bound (RoboLang 2.3)."""
        if binding.text in self._chosen:
            self._error("E318", f"binding '{binding.text}' has the name of {_article(self._params[binding.text])} "
                                "that a 'choose' bound; a binding must not share a name with an entity", binding.span)

    def _choose(self, statement: ast.Choose):
        """C1 to C7: the type, the candidates, `where` (world facts only, E330), the order and the `else` (E329)."""
        kind = ParamType(statement.type.text)
        name = statement.name.text
        if name in self._params or name in self._measured:
            what = "a parameter or an earlier choice" if name in self._params else "the binding of a 'measure'"
            self._error("E302", f"'{name}' is already {what}: a 'choose' binds a name of its own", statement.name.span)
        source = statement.source
        if isinstance(source, ast.ChooseList):
            for candidate in source.names:
                self._candidate(candidate, kind, f"a candidate of a choice of {_article(kind)}")
        elif isinstance(source, ast.ObjectsAt):
            if kind is not ParamType.OBJECT:
                self._error("E304", f"'objects at {source.location.text}' gives objects, but this choice is "
                                    f"{_article(kind)}: write 'choose object {name} from objects at ...'", source.span)
            self._candidate(source.location, ParamType.LOCATION, "objects at")
        elif kind is not ParamType.ROBOT:
            self._error("E304", f"'robots' gives robots, but this choice is {_article(kind)}: write "
                                f"'choose robot {name} from robots'", source.span)
        order = statement.order
        if order is not None:
            reference = order.reference
            if reference.text in self._chosen:
                self._candidate(reference, ParamType.LOCATION, "an order")  # reports E331
            declared = self._params.get(reference.text)
            if declared is None:
                self._undeclared(reference)
            if declared not in (ParamType.ROBOT, ParamType.OBJECT, ParamType.LOCATION):
                self._error("E304", f"'{reference.text}' is {_article(declared)}, but '{order.word.text}' expects a "
                                    "location, a robot or an object here", reference.span)
        if statement.where is not None:
            self._where(statement.where, name, kind)
        handler = statement.handler
        if handler is not None and not (handler.statements and isinstance(handler.statements[-1], ast.Fail)):
            self._report("E329", "the 'else' of a 'choose' must end in 'fail' (decision 0045 C6, point 1): with no "
                                 "candidate left the name stays unbound, so the plan cannot go on; a message may "
                                 "come before the 'fail'", handler.span)

    def _where(self, condition: ast.Condition, name: str, kind: ParamType):
        """C3: a condition over the chosen name that states world facts only: no reading, no clock, no message."""
        names, stack = False, [condition]
        while stack:
            node = stack.pop()
            if isinstance(node, (ast.And, ast.Or)):
                stack.extend(node.operands)
            elif isinstance(node, ast.Not):
                stack.append(node.operand)
            elif isinstance(node, ast.Compare) and self._charge_test(node, name, kind):
                names = True  # RoboLang 2.4 (decision 0048 S7): the chosen robot's charge is a world fact
            elif isinstance(node, (ast.Compare, ast.MessageReceived)):
                what = "a message" if isinstance(node, ast.MessageReceived) else "a reading or the clock"
                self._error("E330", f"'where' states world facts only, not {what}: a reading is a sequence of robot "
                                    "actions, not a choice (decision 0045 C3, C9); choose by the facts, then "
                                    "'measure' the entity chosen", node.span)
            else:
                names = names or any(getattr(node, f, None) is not None and getattr(node, f).text == name
                                     for f in ("robot", "obj", "location", "target"))
        if not names:
            self._error("E330", f"'where' must be a condition over the chosen name '{name}': none of its conditions "
                                f"names it", condition.span)
        self._params[name] = kind  # the name is visible inside its own `where`
        try:
            self._condition(condition)
        finally:
            del self._params[name]

    def _charge_test(self, node: ast.Compare, name: str, kind: ParamType) -> bool:
        """
        RoboLang 2.4 (decision 0048 S7, point 5): `robot_charge t >= 50 %` in the `where` of a choice of a robot `t`
        is a world fact that the fleet manager reports: the one reading a `where` may state. It names the chosen robot
        and compares with a literal (no binding exists there); anything else keeps E330.
        """
        left, right = node.left, node.right
        if not (self._charging_language and kind is ParamType.ROBOT and isinstance(left, ast.LiveCharge)
                and left.robot.text == name and isinstance(right, ast.QuantityLit)):
            return False
        if right.unit is None:  # the rules of `measured.Scopes`, which do not read a `where`
            self._report("E206", "a charge needs its unit: write '50 %'", right.span)
        elif right.unit != "%":
            self._report("E319", f"'{right.unit}' is not a unit of charge (charge is in %)", right.span)
        if node.op not in ("<", "<=", ">", ">="):
            self._report("E320", f"charge is compared with '<', '<=', '>' or '>=', not '{node.op}': a reading has an "
                                 "uncertainty, so equality is almost never decidable", node.span)
        return True

    # -------- individual rules --------

    def _live(self, reading: ast.LiveReading):
        if reading.kind.text in AREA_KINDS and self._kinds:  # RoboLang 2.2 (decision 0044 D1): or a location
            self._use_any(reading.obj, f"a live {reading.kind.text} reading")
        else:
            self._use(reading.obj, ParamType.OBJECT, "a live reading")
        self._sensor(reading.sensor, "a live reading ... using")

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

    def _undeclared(self, name: ast.Name):
        self._error("E303", f"undeclared name '{name.text}': it is not a parameter of this {self._owner}", name.span)

    def _use(self, name: ast.Name, expected: ParamType, context: str):
        declared = self._params.get(name.text)
        if declared is None:
            self._undeclared(name)
        if declared is not expected:
            self._error(
                "E304",
                f"'{name.text}' is {_article(declared)}, but '{context}' expects {_article(expected)} here",
                name.span,
            )

    def _use_any(self, name: ast.Name, context: str):
        """RoboLang 1.8: the target of `do` or `in_state` is an object or a location parameter (E303, E304)."""
        declared = self._params.get(name.text)
        if declared is None:
            self._undeclared(name)
        if declared not in (ParamType.OBJECT, ParamType.LOCATION):
            self._error(
                "E304",
                f"'{name.text}' is {_article(declared)}, but '{context}' expects an object or a location here",
                name.span,
            )

    def _positive(self, value: Decimal, what: str, span: Span):
        if value <= 0:
            self._error("E305", f"{what} must be greater than zero", span)

    def _not_empty(self, string: ast.StringLit, what: str):
        if string.value == "":
            self._error("E307", f"empty string: a {what} must not be empty", string.span)


FAIL_REASON_MAX = 200  # characters; Core 0.2 `declared` text has the same limit (decision 0028 Q1)


def fail_reason_problem(reason: str) -> str | None:
    """Why a `fail` reason cannot be Core 0.2 `declared` text, or None: at most 200 characters, no control chars."""
    if len(reason) > FAIL_REASON_MAX:
        return f"a 'fail' reason has at most {FAIL_REASON_MAX} characters; this one has {len(reason)}"
    if any(ord(c) < 0x20 or ord(c) == 0x7F for c in reason):
        return "a 'fail' reason must not contain control characters such as \\n or \\t"
    return None


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
