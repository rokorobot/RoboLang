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


class RoboLangError(Exception):
    """Raised for the first lexical or syntax error in a source file."""

    def __init__(self, diagnostic: Diagnostic):
        super().__init__(diagnostic.format())
        self.diagnostic = diagnostic

    @property
    def code(self) -> str:
        return self.diagnostic.code

    @property
    def line(self) -> int:
        return self.diagnostic.line

    @property
    def column(self) -> int:
        return self.diagnostic.column
