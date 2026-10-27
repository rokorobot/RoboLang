"""Token definitions and word sets for RoboLang v1.1 and v1.2 (robolang-v1.1.md section 3, robolang-v1.2.md 2.2)."""

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum

from .nodes import Span


class TokenKind(Enum):
    IDENT = "identifier"
    KEYWORD = "keyword"
    NUMBER = "number"
    STRING = "string"
    SPEED_UNIT = "speed unit"
    AT_SAFETY = "@safety"
    LPAREN = "("
    RPAREN = ")"
    LBRACE = "{"
    RBRACE = "}"
    COMMA = ","
    SEMICOLON = ";"
    EOF = "end of file"


@dataclass(frozen=True)
class Token:
    kind: TokenKind
    text: str                  # exact source text
    span: Span
    value: str | None = None   # decoded content, for STRING tokens


PARAM_TYPES = ("robot", "object", "location", "region")
STATEMENTS = ("move", "grasp", "place", "inspect", "wait", "communicate")
CONDITIONS = ("robot_ready", "region_clear", "object_exists", "sensor_ok")

KEYWORDS = frozenset(
    ("robolang", "task", "pre", "plan", "to", "at", "using", "with", "for", "until", "timeout")
    + PARAM_TYPES + STATEMENTS + CONDITIONS
)

# Words of constructs documented for v1 but not part of v1.1 (spec section 8.2).
FUTURE_RESERVED = frozenset(
    ("with_approach", "in_region", "no_go_region", "require_guard", "message_received", "scalar", "duration")
)

RESERVED = KEYWORDS | FUTURE_RESERVED

# Reserved in RoboLang 1.2 files only; in 1.1 files they are names (spec/robolang-v1.2.md section 2.2).
OPERATORS = ("and", "or", "not")
RESERVED_V1_2 = RESERVED | frozenset(OPERATORS)

# Recognized only in their grammatical position; not reserved.
SAFETY_KEYS = ("max_speed",)
SPEED_UNITS = ("m/s", "rad/s")
DURATION_UNITS = ("ms", "s", "min")

ESCAPES = {'"': '"', "\\": "\\", "n": "\n", "t": "\t"}
