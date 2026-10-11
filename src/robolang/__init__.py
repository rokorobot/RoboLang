"""
RoboLang v1.1 front end: lexer, parser and AST (RL-11B).

This package only reads source text. It is not connected to the v1 interpreter
(`robolang_runtime`) or to any adapter, and it does not perform the static
checks of the specification (RL-11C).
"""

from . import nodes
from .diagnostics import Diagnostic, RoboLangError, Severity
from .lexer import Lexer, tokenize
from .parser import SUPPORTED_VERSION, ParseResult, parse, parse_file
from .tokens import Token, TokenKind

__all__ = [
    "Diagnostic",
    "Lexer",
    "ParseResult",
    "RoboLangError",
    "Severity",
    "SUPPORTED_VERSION",
    "Token",
    "TokenKind",
    "nodes",
    "parse",
    "parse_file",
    "tokenize",
]
