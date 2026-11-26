"""Token definitions and word sets for RoboLang v1.1 to v2.0 (robolang-v1.1.md section 3, v1.2 2.2, v1.3 2.2)."""

from __future__ import annotations
from dataclasses import dataclass
from enum import Enum

from .nodes import Span


class TokenKind(Enum):
    IDENT = "identifier"
    KEYWORD = "keyword"
    NUMBER = "number"
    TIME = "time of day"  # RoboLang 1.9 files only: 22:00 or 22:00:00
    STRING = "string"
    SPEED_UNIT = "speed unit"
    PERCENT = "%"  # RoboLang 2.2 files only: the unit of a `fill` or `confidence` (decision 0044 K2)
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
# RoboLang 1.9 adds clock readings (decision 0042 T1, T6): `clock_time` and `clock_elapsed` are reserved. The kind names
# `time` and `elapsed` are recognised by position (after `measure r`) and are not reserved.
CLOCK_KINDS = ("time", "elapsed")
LIVE_CLOCK = {"clock_time": "time", "clock_elapsed": "elapsed"}  # the live names (T6) -> the kind they read
RESERVED_V1_9 = RESERVED_V1_8 | frozenset(LIVE_CLOCK)
# RoboLang 2.0 adds the mission layer (decision 0041 M1, M10): seven reserved words. `at`, `for`, `until` and `else`
# are reserved already; `most`, `on` and `failure` are recognised by position and are not reserved.
MISSION_WORDS = ("mission", "repeat", "run", "runs", "stop", "continue", "retry")
RESERVED_V2_0 = RESERVED_V1_9 | frozenset(MISSION_WORDS)
MAX_MISSION_RUNS = 1000  # a mission's run count, and the most runs any mission makes (spec/robolang-v2.0.md section 3)
MAX_RETRIES = 100  # `retry N`
# RoboLang 2.1 adds incoming messages (decision 0043 S1): the condition `message_received "CH" with "TEXT"`. The word
# has been reserved since 1.1 and `with` is a keyword already, so there is no new reserved word.
CONDITIONS_V2_1 = CONDITIONS_V1_8 + ("message_received",)
# RoboLang 2.2 adds four measured kinds (decision 0044 K1) and the unit token `%` (K2). The kind names are recognised by
# position (after `measure r` and before `of`) and are not reserved (D10): `count` may still be a name.
NEW_KINDS_V2_2 = ("fill", "confidence", "pressure", "count")
# RoboLang 2.3 adds the statement `choose` (decision 0045 C1, point 10). `choose`, `from`, `where`, `nearest`,
# `farthest`, `objects` and `robots` are recognised by position and are not reserved: a 2.2 program that uses them as
# names still parses, and in a file below 2.3 `choose` is an unknown statement (E203).
CHOOSE_TYPES = ("robot", "object", "location")
# RoboLang 2.4 adds the robot's own charge (decision 0048 S1, S3): the kind `charge` (after `measure r`) and the live
# `robot_charge r` (in a `wait until` and a mission's `until`). Both are recognised by position and are not reserved
# (S3): `charge` is already a world action name, and a 2.3 program may use either word as a name.
CHARGE_KIND = "charge"
LIVE_CHARGE = "robot_charge"
# Every language version, oldest first. `since("1.6")` is the versions from 1.6 on.
VERSIONS = ("1.1", "1.2", "1.3", "1.4", "1.5", "1.6", "1.7", "1.8", "1.9", "2.0", "2.1", "2.2", "2.3", "2.4")


def since(first: str) -> tuple[str, ...]:
    """The language versions from `first` on (a feature added in `first` is in each of them)."""
    return VERSIONS[VERSIONS.index(first):]


# Language version -> its reserved words. Versions not listed read as 1.1.
RESERVED_BY_VERSION = {
    "1.2": RESERVED_V1_2, "1.3": RESERVED_V1_3, "1.4": RESERVED_V1_4, "1.5": RESERVED_V1_4, "1.6": RESERVED_V1_6,
    "1.7": RESERVED_V1_6, "1.8": RESERVED_V1_8, "1.9": RESERVED_V1_9, "2.0": RESERVED_V2_0,
    "2.1": RESERVED_V2_0, "2.2": RESERVED_V2_0, "2.3": RESERVED_V2_0, "2.4": RESERVED_V2_0,
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
# RoboLang 2.2 (decision 0044 K1, K2): `%` for fill and confidence (0 to 100), Pa, kPa and bar for pressure; a count has
# no unit. The 1.6 mapping above stays for files below 2.2 and for the kinds a world action takes.
KIND_UNITS_V2_2 = {**KIND_UNITS, "fill": ("%",), "confidence": ("%",), "pressure": ("Pa", "kPa", "bar"), "count": ()}
# RoboLang 2.4 (decision 0048 S1): the robot's own charge, in `%` (0 to 100) like fill and confidence.
KIND_UNITS_V2_4 = {**KIND_UNITS_V2_2, CHARGE_KIND: ("%",)}
COMPARISON_OPERATORS = ("<", "<=", ">", ">=", "==", "!=")

ESCAPES = {'"': '"', "\\": "\\", "n": "\n", "t": "\t"}
