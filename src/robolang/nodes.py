"""
Abstract syntax tree for RoboLang v1.1 to v1.3 (robolang-v1.1.md section 7, v1.2 section 6, v1.3 section 6).

Nodes are immutable and record the source faithfully: names are unresolved,
units are kept as written and nothing is normalized. Binding and type checks
(RL-11C) operate on this tree.
"""

from __future__ import annotations
from dataclasses import dataclass
from decimal import Decimal
from enum import Enum
from typing import Union


@dataclass(frozen=True)
class Span:
    """Source range. Lines and columns are 1-based; the end is exclusive."""

    line: int
    column: int
    end_line: int
    end_column: int

    def to(self, other: Span) -> Span:
        """Span from the start of this one to the end of `other`."""
        return Span(self.line, self.column, other.end_line, other.end_column)


class ParamType(Enum):
    ROBOT = "robot"
    OBJECT = "object"
    LOCATION = "location"
    REGION = "region"
    SENSOR = "sensor"  # RoboLang 1.3


class SpeedUnit(Enum):
    M_PER_S = "m/s"
    RAD_PER_S = "rad/s"


class DurationUnit(Enum):
    MS = "ms"
    S = "s"
    MIN = "min"


# -------- leaves --------

@dataclass(frozen=True)
class Name:
    """A use or declaration of an identifier."""

    text: str
    span: Span


@dataclass(frozen=True)
class StringLit:
    """`value` is the decoded text: no quotes, escapes resolved."""

    value: str
    span: Span


@dataclass(frozen=True)
class VersionDecl:
    version: str
    span: Span


@dataclass(frozen=True)
class Duration:
    value: Decimal
    unit: DurationUnit
    span: Span


@dataclass(frozen=True)
class Speed:
    """`unit` is None for the deprecated unitless form (W401)."""

    value: Decimal
    unit: SpeedUnit | None
    span: Span


# -------- conditions --------

@dataclass(frozen=True)
class RobotReady:
    robot: Name
    span: Span


@dataclass(frozen=True)
class RegionClear:
    region: Name
    span: Span


@dataclass(frozen=True)
class ObjectExists:
    obj: Name
    span: Span


@dataclass(frozen=True)
class SensorOk:
    """`sensor` is a StringLit in 1.1 and 1.2, a Name (a sensor parameter) in 1.3; a 1.3 StringLit is E310."""

    sensor: StringLit | Name
    span: Span


@dataclass(frozen=True)
class ObjectAt:
    """RoboLang 1.3. The object is present, not held, and at the location."""

    obj: Name
    location: Name
    span: Span


@dataclass(frozen=True)
class RobotAt:
    """RoboLang 1.3."""

    robot: Name
    location: Name
    span: Span


@dataclass(frozen=True)
class Holding:
    """RoboLang 1.3."""

    robot: Name
    obj: Name
    span: Span


@dataclass(frozen=True)
class And:
    """RoboLang 1.2. Two or more operands in source order; grouping is kept as written."""

    operands: tuple[Condition, ...]
    span: Span


@dataclass(frozen=True)
class Or:
    """RoboLang 1.2. Two or more operands in source order; grouping is kept as written."""

    operands: tuple[Condition, ...]
    span: Span


@dataclass(frozen=True)
class Not:
    """RoboLang 1.2."""

    operand: Condition
    span: Span


Condition = Union[RobotReady, RegionClear, ObjectExists, SensorOk, ObjectAt, RobotAt, Holding, And, Or, Not]


# -------- statements --------

@dataclass(frozen=True)
class Move:
    robot: Name
    target: Name
    span: Span


@dataclass(frozen=True)
class Grasp:
    robot: Name
    obj: Name
    span: Span


@dataclass(frozen=True)
class Place:
    robot: Name
    obj: Name
    target: Name
    span: Span


@dataclass(frozen=True)
class Inspect:
    robot: Name
    obj: Name
    sensor: StringLit | Name  # a Name in 1.3, as in SensorOk
    span: Span


@dataclass(frozen=True)
class WaitFor:
    duration: Duration
    span: Span


@dataclass(frozen=True)
class WaitUntil:
    condition: Condition
    timeout: Duration
    span: Span


@dataclass(frozen=True)
class Communicate:
    robot: Name
    channel: StringLit
    message: StringLit
    span: Span


@dataclass(frozen=True)
class Fail:
    """RoboLang 1.4: end the run as FAILED with `reason`. Only the last statement of a branch (E313)."""

    reason: StringLit
    span: Span


@dataclass(frozen=True)
class Branch:
    """RoboLang 1.4: one side of an `if`. `goal` holds the conditions this path adds to the task goal."""

    goal: GoalBlock | None
    statements: tuple[Statement, ...]
    span: Span


@dataclass(frozen=True)
class If:
    """
    RoboLang 1.4. `orelse` is None when there is no `else`. `else if` is an `orelse` branch with
    no goal whose only statement is the nested `If`.
    """

    condition: Condition
    then: Branch
    orelse: Branch | None
    span: Span


Statement = Union[Move, Grasp, Place, Inspect, WaitFor, WaitUntil, Communicate, If, Fail]


# -------- structure --------

@dataclass(frozen=True)
class Param:
    type: ParamType
    name: Name
    span: Span


@dataclass(frozen=True)
class SafetyAnnotation:
    """`key.text` is "max_speed" in v1.1."""

    key: Name
    value: Speed
    span: Span


@dataclass(frozen=True)
class VerifyAnnotation:
    """RoboLang 1.3: `key.text` is "min_confidence" (`value` a Decimal) or "max_age" (`value` a Duration)."""

    key: Name
    value: Decimal | Duration
    value_span: Span
    span: Span


@dataclass(frozen=True)
class PreBlock:
    conditions: tuple[Condition, ...]
    span: Span


@dataclass(frozen=True)
class GoalBlock:
    """RoboLang 1.3: the conditions that must hold at the end for the task to have succeeded."""

    conditions: tuple[Condition, ...]
    span: Span


@dataclass(frozen=True)
class PlanBlock:
    statements: tuple[Statement, ...]
    span: Span


@dataclass(frozen=True)
class Task:
    name: Name
    params: tuple[Param, ...]
    annotations: tuple[SafetyAnnotation, ...]
    pre: PreBlock | None
    plan: PlanBlock
    span: Span
    goal: GoalBlock | None = None  # required in 1.3 (E309), absent before
    verify: tuple[VerifyAnnotation, ...] = ()  # 1.3 only


@dataclass(frozen=True)
class Program:
    """`version` is None for a file without a version marker."""

    version: VersionDecl | None
    tasks: tuple[Task, ...]
    span: Span
