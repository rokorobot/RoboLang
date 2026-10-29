"""Diagnostics for the RoboLang v1.1 front end (spec/robolang-v1.1.md, section 6)."""

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum

from .nodes import Span


class Severity(Enum):
    ERROR = "error"
    WARNING = "warning"


@dataclass(frozen=True)
class Diagnostic:
    code: str
    message: str
    span: Span
    severity: Severity = Severity.ERROR
    filename: str = "<string>"

    @property
    def line(self) -> int:
        return self.span.line

    @property
    def column(self) -> int:
        return self.span.column

    def format(self) -> str:
        return f"{self.filename}:{self.line}:{self.column}: {self.severity.value}[{self.code}]: {self.message}"

    def __str__(self) -> str:
        return self.format()


MAX_ERRORS = 50  # a front end reports at most this many errors, then says there are more


class RoboLangError(Exception):
    """
    Raised when a source file has at least one error.

    `diagnostic` is the first error (what a front end that stops at the first error reports);
    `diagnostics` is every error found, in source order, at most `MAX_ERRORS`. `truncated` is
    true when more errors were found than are listed. `warnings` are the warnings found before
    the file was rejected.
    """

    def __init__(
        self,
        diagnostic: Diagnostic | list[Diagnostic] | tuple[Diagnostic, ...],
        *,
        truncated: bool = False,
        warnings: tuple[Diagnostic, ...] = (),
    ):
        diagnostics = (diagnostic,) if isinstance(diagnostic, Diagnostic) else tuple(diagnostic)
        if len(diagnostics) > MAX_ERRORS:
            diagnostics, truncated = diagnostics[:MAX_ERRORS], True
        super().__init__(diagnostics[0].format())
        self.diagnostic = diagnostics[0]
        self.diagnostics = diagnostics
        self.truncated = truncated
        self.warnings = warnings

    @property
    def code(self) -> str:
        return self.diagnostic.code

    @property
    def line(self) -> int:
        return self.diagnostic.line

    @property
    def column(self) -> int:
        return self.diagnostic.column
