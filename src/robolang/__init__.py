"""
RoboLang front end for languages 1.1, 1.2 and 1.3: lexer, parser, AST (RL-11B), static
validation (RL-11C), the versioned task IR (RL-11D; schema 1.1 for 1.2 sources, LANG-1b;
schema 1.2 for 1.3 sources),
and `robolang.sim`, the symbolic, simulation-only executor (RL-BASIC-01).

`check` / `check_file` run the complete front end and never execute anything. The package
is not connected to the v1 interpreter (`robolang_runtime`), to any adapter, ROS 2 or
hardware. A program that passes is well-formed, not safe to execute on a robot.
"""

from . import ir, nodes
from .diagnostics import Diagnostic, RoboLangError, Severity
from .lexer import Lexer, tokenize
from .parser import DEFAULT_VERSION, SUPPORTED_VERSION, SUPPORTED_VERSIONS, ParseResult, parse, parse_file
from .tokens import Token, TokenKind
from .validator import check, check_file, validate

__all__ = [
    "DEFAULT_VERSION",
    "Diagnostic",
    "Lexer",
    "ParseResult",
    "RoboLangError",
    "Severity",
    "SUPPORTED_VERSION",
    "SUPPORTED_VERSIONS",
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
