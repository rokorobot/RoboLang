"""Token definitions and word sets for RoboLang v1.1 to v1.8 (robolang-v1.1.md section 3, v1.2 2.2, v1.3 2.2)."""

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
    OPERATOR = "comparison operator"  # RoboLang 1.6 files only
    AT_SAFETY = "@safety"
    AT_VERIFY = "@verify"  # RoboLang 1.3 files only
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
STATEMENTS_V1_6 = STATEMENTS + ("measure",)
STATEMENTS_V1_8 = STATEMENTS_V1_6 + ("do",)  # RoboLang 1.8: a declared action (decision 0040 A1)
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

# RoboLang 1.3 adds the goal block, three outcome atoms and the sensor parameter type
# (spec/robolang-v1.3.md sections 2.2 and 3).
PARAM_TYPES_V1_3 = PARAM_TYPES + ("sensor",)
CONDITIONS_V1_3 = CONDITIONS + ("object_at", "robot_at", "holding")
CONDITIONS_V1_8 = CONDITIONS_V1_3 + ("in_state",)  # RoboLang 1.8 (decision 0040 A2)
RESERVED_V1_3 = RESERVED_V1_2 | frozenset(("goal", "sensor", "object_at", "robot_at", "holding"))

# RoboLang 1.4 adds bounded branching (spec/robolang-v1.4.md, decision 0027 B9).
RESERVED_V1_4 = RESERVED_V1_3 | frozenset(("if", "else", "fail"))

# RoboLang 1.5 adds step failure handlers (`<step> else { ... }`, decision 0033) and no new words.
# RoboLang 1.6 adds measured values (decision 0036 V10): `measure` and `as` are reserved. Kind names, `of` and units
# are recognised by position and are not reserved.
RESERVED_V1_6 = RESERVED_V1_4 | frozenset(("measure", "as"))
# RoboLang 1.7 adds region limits (decision 0039 P1): `@safety no_go_region R;` and `@safety in_region R;`. Both words
# have been reserved since 1.1, so there is no new word.
# RoboLang 1.8 adds declared actions (decision 0040 A1, A2): `do` and `in_state` are reserved. An action name is a
# name that the robot's profile declares; it is recognised by position and is not reserved.
RESERVED_V1_8 = RESERVED_V1_6 | frozenset(("do", "in_state"))
# Language version -> its reserved words. Versions not listed read as 1.1.
RESERVED_BY_VERSION = {
    "1.2": RESERVED_V1_2, "1.3": RESERVED_V1_3, "1.4": RESERVED_V1_4, "1.5": RESERVED_V1_4, "1.6": RESERVED_V1_6,
    "1.7": RESERVED_V1_6, "1.8": RESERVED_V1_8,
}

# Recognized only in their grammatical position; not reserved.
SAFETY_KEYS = ("max_speed",)
REGION_LIMITS = ("no_go_region", "in_region")  # RoboLang 1.7 (decision 0039): the keys of a region limit
MAX_REGION_LIMITS = 8  # decision 0039 P4
VERIFY_KEYS = ("min_confidence", "max_age")  # RoboLang 1.3
SPEED_UNITS = ("m/s", "rad/s")
DURATION_UNITS = ("ms", "s", "min")

# RoboLang 1.6 measured values (decision 0036 V5): kind -> its units, in the order the spec lists them.
KIND_UNITS = {"temperature": ("degC", "K"), "mass": ("g", "kg"), "distance": ("mm", "cm", "m"), "label": ()}
COMPARISON_OPERATORS = ("<", "<=", ">", ">=", "==", "!=")

ESCAPES = {'"': '"', "\\": "\\", "n": "\n", "t": "\t"}
