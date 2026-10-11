"""
RoboLang v1.1 front end: lexer, parser, AST (RL-11B), static validation (RL-11C)
and the versioned task IR with read-only commands (RL-11D).

`check` / `check_file` run the complete front end. This package only reads
source text: it is not connected to the v1 interpreter (`robolang_runtime`) or
to any adapter. A program that passes is well-formed, not safe to execute.
"""

from . import ir, nodes
from .diagnostics import Diagnostic, RoboLangError, Severity
from .lexer import Lexer, tokenize
from .parser import SUPPORTED_VERSION, ParseResult, parse, parse_file
from .tokens import Token, TokenKind
from .validator import check, check_file, validate

__all__ = [
    "Diagnostic",
    "Lexer",
    "ParseResult",
    "RoboLangError",
    "Severity",
    "SUPPORTED_VERSION",
    "Token",
    "TokenKind",
    "check",
    "check_file",
    "ir",
    "nodes",
    "parse",
    "parse_file",
    "tokenize",
    "validate",
]
