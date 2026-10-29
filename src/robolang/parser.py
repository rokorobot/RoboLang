"""
Recursive-descent parser for RoboLang v1.1 (spec/grammar.ebnf), v1.2 (spec/grammar-v1.2.ebnf) and
v1.3 (spec/grammar-v1.3.ebnf).

Produces the AST of `nodes.py` or raises RoboLangError for the first error.
Stages run in order: a lexical error anywhere in the file is reported before
any syntax error. It performs no name binding, type checking or range checking
(RL-11C) and never touches an adapter: parsing a file executes nothing.

A program accepted here is syntactically valid. That says nothing about
whether it is safe to run on a robot.
"""

from __future__ import annotations
from collections import deque
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path
from typing import NoReturn

from . import nodes as ast
from .diagnostics import Diagnostic, RoboLangError, Severity
from .lexer import Lexer
from .nodes import Span
from .tokens import (
    CONDITIONS,
    CONDITIONS_V1_3,
    DURATION_UNITS,
    PARAM_TYPES,
    PARAM_TYPES_V1_3,
    RESERVED_BY_VERSION,
    SAFETY_KEYS,
    SPEED_UNITS,
    STATEMENTS,
    Token,
    TokenKind,
)

DEFAULT_VERSION = "1.1"  # the version of a file without a marker
SUPPORTED_VERSIONS = ("1.1", "1.2", "1.3")
SUPPORTED_VERSION = DEFAULT_VERSION  # kept for existing imports
MAX_CONDITION_DEPTH = 8  # spec/robolang-v1.2.md section 3.3

_NOT_IN_V1_1 = "is documented for v1 but is not part of v1.1"


def _listed(words) -> str:
    return ", ".join(f"'{w}'" for w in words)


# 1.3 atoms with two names: word -> (node, what the first name is, what the second name is)
_PAIR_ATOMS = {
    "object_at": (ast.ObjectAt, "an object name", "a location name"),
    "robot_at": (ast.RobotAt, "a robot name", "a location name"),
    "holding": (ast.Holding, "a robot name", "an object name"),
}


def _versions_text() -> str:
    return f"{', '.join(SUPPORTED_VERSIONS[:-1])} and {SUPPORTED_VERSIONS[-1]}"


@dataclass(frozen=True)
class ParseResult:
    program: ast.Program
    warnings: tuple[Diagnostic, ...]


class _Parser:
    def __init__(self, lexer: Lexer, filename: str):
        self._lexer = lexer
        self._filename = filename
        self._warnings: list[Diagnostic] = []
        self._prev: Token | None = None
        self._buffer: deque[Token] = deque()
        self._language = DEFAULT_VERSION
        self._tok = lexer.next_token()

    # -------- helpers --------

    def _error(self, code: str, message: str, span: Span | None = None) -> NoReturn:
        raise RoboLangError(Diagnostic(code, message, span or self._tok.span, filename=self._filename))

    def _warn(self, code: str, message: str, span: Span):
        self._warnings.append(Diagnostic(code, message, span, Severity.WARNING, self._filename))

    def _advance(self) -> Token:
        token = self._tok
        self._prev = token
        self._tok = self._buffer.popleft() if self._buffer else self._lexer.next_token()
        return token

    def _lex_remaining(self):
        """Lex the rest of the file now, so a lexical error anywhere precedes any syntax error."""
        last = self._buffer[-1] if self._buffer else self._tok
        while last.kind is not TokenKind.EOF:
            last = self._lexer.next_token()
            self._buffer.append(last)

    def _found(self) -> str:
        if self._tok.kind is TokenKind.EOF:
            return "end of file"
        return f"'{self._tok.text}'"

    def _at(self, kind: TokenKind) -> bool:
        return self._tok.kind is kind

    def _at_keyword(self, *words: str) -> bool:
        return self._tok.kind is TokenKind.KEYWORD and self._tok.text in words

    def _expect(self, kind: TokenKind, what: str) -> Token:
        if not self._at(kind):
            self._error("E201", f"expected {what}, found {self._found()}")
        return self._advance()

    def _expect_keyword(self, word: str, context: str) -> Token:
        if not self._at_keyword(word):
            self._error("E201", f"expected '{word}' {context}, found {self._found()}")
        return self._advance()

    def _semicolon(self, after: str) -> Token:
        if not self._at(TokenKind.SEMICOLON):
            end = self._prev.span
            here = Span(end.end_line, end.end_column, end.end_line, end.end_column)
            self._error("E202", f"missing ';' after {after}", here)
        return self._advance()

    def _name(self, what: str) -> ast.Name:
        if self._at(TokenKind.KEYWORD):
            self._error("E212", f"'{self._tok.text}' is a reserved word and cannot be used as {what}")
        token = self._expect(TokenKind.IDENT, what)
        return ast.Name(token.text, token.span)

    def _string(self, what: str) -> ast.StringLit:
        token = self._expect(TokenKind.STRING, what)
        return ast.StringLit(token.value, token.span)

    # -------- program --------

    def parse_program(self) -> ParseResult:
        start = self._tok.span
        # The version marker is read token by token: an unsupported version (E213)
        # must be reported without interpreting the rest of the file.
        version = self._version() if self._at_keyword("robolang") else None
        self._lex_remaining()
        tasks = []
        while not self._at(TokenKind.EOF):
            if self._at_keyword("robolang"):
                self._error("E201", "the version marker may appear only once, before the first task")
            if not self._at_keyword("task"):
                self._error("E201", f"expected 'task', found {self._found()}")
            tasks.append(self._task())
        if not tasks:
            self._error("E210", "no task in file: a program must define at least one task")
        return ParseResult(
            ast.Program(version, tuple(tasks), start.to(self._prev.span)),
            tuple(self._warnings),
        )

    def _version(self) -> ast.VersionDecl:
        start = self._advance()
        if self._at(TokenKind.NUMBER) and self._tok.text not in SUPPORTED_VERSIONS:
            self._error(
                "E213",
                f"unsupported language version {self._tok.text}; "
                f"this front end implements {_versions_text()}",
            )
        number = self._expect(TokenKind.NUMBER, f"a version number after 'robolang' (for example {DEFAULT_VERSION})")
        self._language = number.text
        if self._language in RESERVED_BY_VERSION:
            self._lexer.reserved = RESERVED_BY_VERSION[self._language]  # the token after ';' is not lexed yet
        end = self._semicolon("the version marker")
        return ast.VersionDecl(number.text, start.span.to(end.span))

    # -------- task --------

    def _task(self) -> ast.Task:
        start = self._advance()
        name = self._name("a task name")
        self._expect(TokenKind.LPAREN, "'(' after the task name")
        params = []
        if not self._at(TokenKind.RPAREN):
            params.append(self._param())
            while self._at(TokenKind.COMMA):
                self._advance()
                params.append(self._param())
        self._expect(TokenKind.RPAREN, "',' or ')' in the parameter list")
        self._expect(TokenKind.LBRACE, "'{' to open the task body")

        annotations = []
        while self._at(TokenKind.AT_SAFETY):
            annotations.append(self._annotation())
        pre = self._pre() if self._at_keyword("pre") else None
        goal = self._goal() if self._language == "1.3" and self._at_keyword("goal") else None

        if not self._at_keyword("plan"):
            if self._at(TokenKind.AT_SAFETY):
                self._error("E208", "@safety annotations must come before the pre block")
            if self._at_keyword("pre"):
                if goal is not None:
                    self._error("E208", "the pre block must come before the goal block")
                self._error("E208", "a task has at most one pre block")
            if self._at_keyword("goal"):
                self._error("E208", "a task has at most one goal block")
            if self._at(TokenKind.RBRACE):
                self._error("E208", "a task must have a plan block")
            self._error("E201", f"expected 'plan', found {self._found()}")
        plan = self._plan()

        blocks = ("pre", "goal", "plan") if self._language == "1.3" else ("pre", "plan")
        if self._at(TokenKind.AT_SAFETY) or self._at_keyword(*blocks):
            order = "pre, goal, then one plan" if self._language == "1.3" else "pre, then one plan"
            self._error(
                "E208",
                f"{self._found()} cannot follow the plan block; a task is: @safety annotations, {order}",
            )
        end = self._expect(TokenKind.RBRACE, "'}' to close the task")
        return ast.Task(name, tuple(params), tuple(annotations), pre, plan, start.span.to(end.span), goal)

    def _param(self) -> ast.Param:
        token = self._tok
        types = PARAM_TYPES_V1_3 if self._language == "1.3" else PARAM_TYPES
        if self._at_keyword("scalar", "duration"):
            self._error("E211", f"parameter type '{token.text}' {_NOT_IN_V1_1}")
        if self._at(TokenKind.IDENT):
            self._error("E205", f"unknown parameter type '{token.text}'; expected one of {_listed(types)}")
        if not self._at_keyword(*types):
            self._error("E201", f"expected a parameter type ({_listed(types)}), found {self._found()}")
        self._advance()
        name = self._name("a parameter name")
        return ast.Param(ast.ParamType(token.text), name, token.span.to(name.span))

    def _annotation(self) -> ast.SafetyAnnotation:
        start = self._advance()
        key = self._tok
        if self._at_keyword("no_go_region", "require_guard"):
            self._error("E211", f"'@safety {key.text}' {_NOT_IN_V1_1}")
        if not self._at(TokenKind.IDENT):
            self._error("E201", f"expected a @safety key ({_listed(SAFETY_KEYS)}), found {self._found()}")
        if key.text not in SAFETY_KEYS:
            self._error("E207", f"unknown @safety key '{key.text}'; expected one of {_listed(SAFETY_KEYS)}")
        self._advance()

        number = self._expect(TokenKind.NUMBER, f"a number after '{key.text}'")
        if self._at(TokenKind.SPEED_UNIT):
            unit = self._advance()
            speed = ast.Speed(Decimal(number.text), ast.SpeedUnit(unit.text), number.span.to(unit.span))
        elif self._at(TokenKind.IDENT):
            self._error("E206", f"unknown speed unit '{self._tok.text}'; expected one of {_listed(SPEED_UNITS)}")
        else:
            speed = ast.Speed(Decimal(number.text), None, number.span)
            self._warn(
                "W401",
                f"'{key.text}' has no unit; write 'm/s' or 'rad/s'. "
                "A unitless limit is deprecated and is never treated as an enforced limit",
                number.span,
            )
        end = self._semicolon("the @safety annotation")
        return ast.SafetyAnnotation(ast.Name(key.text, key.span), speed, start.span.to(end.span))

    def _pre(self) -> ast.PreBlock:
        start = self._advance()
        self._expect(TokenKind.LBRACE, "'{' after 'pre'")
        conditions = []
        while not self._at(TokenKind.RBRACE):
            conditions.append(self._condition("a condition or '}'"))
            self._semicolon("the condition")
        end = self._advance()
        return ast.PreBlock(tuple(conditions), start.span.to(end.span))

    def _goal(self) -> ast.GoalBlock:
        start = self._advance()
        self._expect(TokenKind.LBRACE, "'{' after 'goal'")
        conditions = []
        while not self._at(TokenKind.RBRACE):
            conditions.append(self._condition("a condition or '}'"))
            self._semicolon("the condition")
        if not conditions:
            self._error("E208", "empty goal: a goal block must contain at least one condition")
        end = self._advance()
        return ast.GoalBlock(tuple(conditions), start.span.to(end.span))

    def _plan(self) -> ast.PlanBlock:
        start = self._advance()
        self._expect(TokenKind.LBRACE, "'{' after 'plan'")
        statements = []
        while not self._at(TokenKind.RBRACE):
            statements.append(self._statement())
        if not statements:
            self._error("E208", "empty plan: a plan must contain at least one statement")
        end = self._advance()
        return ast.PlanBlock(tuple(statements), start.span.to(end.span))

    # -------- conditions --------

    def _condition(self, expected: str) -> ast.Condition:
        """One atom in 1.1; a combined condition in 1.2 and 1.3 (spec/robolang-v1.2.md section 3)."""
        if self._language == "1.1":
            return self._atom(expected)
        start = self._tok
        condition, depth = self._combined(expected, start, 0)
        if depth > MAX_CONDITION_DEPTH:
            self._too_deep(start)
        return condition

    def _too_deep(self, start: Token) -> NoReturn:
        self._error(
            "E215",
            f"condition is deeper than {MAX_CONDITION_DEPTH} levels; "
            "each 'not', each '( )' and each and/or chain is a level",
            start.span,
        )

    def _combined(self, expected: str, start: Token, groups: int) -> tuple[ast.Condition, int]:
        """Operands joined by one operator word, and the depth of the result (section 3.3)."""
        first, depth = self._operand(expected, start, groups)
        if not self._at_keyword("and", "or"):
            return first, depth
        word = self._tok.text
        operands = [first]
        while self._at_keyword(word):
            self._advance()
            operand, operand_depth = self._operand(f"a condition after '{word}'", start, groups)
            operands.append(operand)
            depth = max(depth, operand_depth)
        if self._at_keyword("and", "or"):
            other = self._tok.text
            self._error(
                "E214",
                f"'{other}' cannot follow '{word}' at the same level; "
                f"group with parentheses, for example '(a {word} b) {other} c'",
            )
        node = ast.And if word == "and" else ast.Or
        return node(tuple(operands), operands[0].span.to(operands[-1].span)), depth + 1

    def _operand(self, expected: str, start: Token, groups: int) -> tuple[ast.Condition, int]:
        """`not` operand, `( condition )` or an atom. `groups` counts the open `not` and `(`."""
        if not (self._at_keyword("not") or self._at(TokenKind.LPAREN)):
            return self._atom(expected), 0
        if groups == MAX_CONDITION_DEPTH:
            self._too_deep(start)  # depth is at least the number of open groups
        token = self._advance()
        if token.kind is TokenKind.LPAREN:
            inner, depth = self._combined("a condition after '('", start, groups + 1)
            self._expect(TokenKind.RPAREN, "')' to close the group")
            return inner, depth + 1
        operand, depth = self._operand("a condition after 'not'", start, groups + 1)
        return ast.Not(operand, token.span.to(operand.span)), depth + 1

    def _atom(self, expected: str) -> ast.Condition:
        token = self._tok
        atoms = CONDITIONS_V1_3 if self._language == "1.3" else CONDITIONS
        if self._at_keyword("message_received"):
            self._error("E211", f"'message_received' {_NOT_IN_V1_1}")
        if self._at(TokenKind.IDENT):
            self._error("E204", f"unknown condition '{token.text}'; expected one of {_listed(atoms)}")
        if not self._at_keyword(*atoms):
            self._error("E201", f"expected {expected}, found {self._found()}")
        self._advance()

        if token.text == "sensor_ok":
            sensor = self._sensor("sensor_ok")
            return ast.SensorOk(sensor, token.span.to(sensor.span))
        if token.text in _PAIR_ATOMS:
            node, first, second = _PAIR_ATOMS[token.text]
            one = self._name(f"{first} after '{token.text}'")
            other = self._name(second)
            return node(one, other, token.span.to(other.span))
        name = self._name(f"a name after '{token.text}'")
        node = {"robot_ready": ast.RobotReady, "region_clear": ast.RegionClear, "object_exists": ast.ObjectExists}
        return node[token.text](name, token.span.to(name.span))

    def _sensor(self, after: str) -> ast.StringLit | ast.Name:
        """A quoted sensor name in 1.1 and 1.2; a sensor parameter in 1.3 (a string is read, then E310)."""
        if self._language != "1.3":
            return self._string(f"a sensor name in quotes after '{after}'")
        if self._at(TokenKind.STRING):
            return self._string("a sensor parameter")
        return self._name(f"a sensor parameter name after '{after}'")

    # -------- statements --------

    def _statement(self) -> ast.Statement:
        token = self._tok
        if self._at(TokenKind.AT_SAFETY):
            self._error("E211", f"'@safety' between statements {_NOT_IN_V1_1}; annotate the task instead")
        if self._at(TokenKind.IDENT):
            self._error("E203", f"unknown statement '{token.text}'; expected one of {_listed(STATEMENTS)}")
        if not self._at_keyword(*STATEMENTS):
            self._error("E201", f"expected a statement or '}}', found {self._found()}")
        self._advance()
        return getattr(self, f"_{token.text}")(token)

    def _move(self, start: Token) -> ast.Move:
        robot = self._name("a robot name after 'move'")
        self._expect_keyword("to", "after the robot name")
        target = self._name("a location name after 'to'")
        if self._at_keyword("with_approach", "in_region"):
            self._error("E211", f"'{self._tok.text}' {_NOT_IN_V1_1}")
        end = self._semicolon("the move statement")
        return ast.Move(robot, target, start.span.to(end.span))

    def _grasp(self, start: Token) -> ast.Grasp:
        robot = self._name("a robot name after 'grasp'")
        obj = self._name("an object name")
        end = self._semicolon("the grasp statement")
        return ast.Grasp(robot, obj, start.span.to(end.span))

    def _place(self, start: Token) -> ast.Place:
        robot = self._name("a robot name after 'place'")
        obj = self._name("an object name")
        self._expect_keyword("at", "after the object name")
        target = self._name("a location name after 'at'")
        end = self._semicolon("the place statement")
        return ast.Place(robot, obj, target, start.span.to(end.span))

    def _inspect(self, start: Token) -> ast.Inspect:
        robot = self._name("a robot name after 'inspect'")
        obj = self._name("an object name")
        if not self._at_keyword("using"):
            hint = " ('with' is not accepted here)" if self._at_keyword("with") else ""
            shown = "using <sensor>" if self._language == "1.3" else 'using "<sensor>"'
            self._error("E209", f"inspect must name its sensor: expected '{shown}', found {self._found()}{hint}")
        self._advance()
        sensor = self._sensor("using")
        end = self._semicolon("the inspect statement")
        return ast.Inspect(robot, obj, sensor, start.span.to(end.span))

    def _wait(self, start: Token) -> ast.WaitFor | ast.WaitUntil:
        if self._at_keyword("for"):
            self._advance()
            duration = self._duration("after 'wait for'")
            end = self._semicolon("the wait statement")
            return ast.WaitFor(duration, start.span.to(end.span))
        if self._at_keyword("until"):
            self._advance()
            condition = self._condition("a condition after 'wait until'")
            if not self._at_keyword("timeout"):
                self._error(
                    "E209",
                    f"wait until needs an explicit timeout: expected 'timeout <duration>', found {self._found()}",
                )
            self._advance()
            timeout = self._duration("after 'timeout'")
            end = self._semicolon("the wait statement")
            return ast.WaitUntil(condition, timeout, start.span.to(end.span))
        self._error("E201", f"expected 'for' or 'until' after 'wait', found {self._found()}")

    def _communicate(self, start: Token) -> ast.Communicate:
        robot = self._name("a robot name after 'communicate'")
        self._expect_keyword("to", "after the robot name")
        channel = self._string("a channel name in quotes after 'to'")
        self._expect_keyword("with", "after the channel")
        message = self._string("a message in quotes after 'with'")
        end = self._semicolon("the communicate statement")
        return ast.Communicate(robot, channel, message, start.span.to(end.span))

    def _duration(self, where: str) -> ast.Duration:
        number = self._expect(TokenKind.NUMBER, f"a duration {where}")
        units = _listed(DURATION_UNITS)
        if self._at(TokenKind.IDENT) and self._tok.text in DURATION_UNITS:
            unit = self._advance()
            return ast.Duration(Decimal(number.text), ast.DurationUnit(unit.text), number.span.to(unit.span))
        if self._at(TokenKind.IDENT) or self._at(TokenKind.SPEED_UNIT):
            self._error("E206", f"unknown duration unit '{self._tok.text}'; expected one of {units}")
        end = number.span
        self._error(
            "E206",
            f"duration '{number.text}' has no unit; expected one of {units}",
            Span(end.end_line, end.end_column, end.end_line, end.end_column),
        )


def parse(source: str, filename: str = "<string>") -> ParseResult:
    """Parse RoboLang v1.1, v1.2 or v1.3 source. Raises RoboLangError on the first lexical or syntax error."""
    return _Parser(Lexer(source, filename), filename).parse_program()


def parse_file(path: str | Path) -> ParseResult:
    path = Path(path)
    return parse(path.read_text(encoding="utf-8"), str(path))
