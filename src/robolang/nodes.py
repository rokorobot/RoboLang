"""
Abstract syntax tree for RoboLang v1.1 to v2.0 (robolang-v1.1.md section 7, v1.2 section 6, v1.3 section 6).

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


@dataclass(frozen=True)
class QuantityLit:
    """RoboLang 1.6: `60 degC`. The unit is kept as written; whether it belongs to the compared kind is static."""

    value: Decimal
    unit: str
    span: Span


@dataclass(frozen=True)
class LiveReading:
    """RoboLang 1.6: `temperature of pan using probe`, read at every evaluation of a `wait until` (decision 0036 V4)."""

    kind: Name
    obj: Name
    sensor: Name | StringLit  # a StringLit is E310, as for `inspect`
    span: Span


@dataclass(frozen=True)
class TimeLit:
    """RoboLang 1.9 (decision 0042 T2): `22:00` or `22:00:00`, a time of day. `seconds` is the second of the day."""

    seconds: int
    span: Span


@dataclass(frozen=True)
class LiveClock:
    """RoboLang 1.9 (decision 0042 T6): `clock_time` or `clock_elapsed`, read when its `wait until` is evaluated."""

    word: Name
    span: Span


@dataclass(frozen=True)
class Compare:
    """
    RoboLang 1.6: `t < 60 degC`, `batch == "LOT-7"`, `a <= b`, `temperature of pan using probe < 60 degC`
    (decision 0036 V6). `left` is a binding or a live reading; `right` a quantity, a string or a binding. RoboLang 1.9
    adds the live clock on the left and a time of day on the right (decision 0042 T3, T6).
    """

    left: Name | LiveReading | LiveClock
    op: str
    right: Name | QuantityLit | StringLit | TimeLit
    span: Span


@dataclass(frozen=True)
class InState:
    """RoboLang 1.8 (decision 0040 A2): `in_state closet "open"`: the entity's observed state is the label."""

    target: Name
    label: StringLit
    span: Span


Condition = Union[RobotReady, RegionClear, ObjectExists, SensorOk, ObjectAt, RobotAt, Holding, And, Or, Not, Compare,
                  InState]


# -------- statements --------

@dataclass(frozen=True)
class Move:
    robot: Name
    target: Name
    span: Span
    handler: Branch | None = None  # RoboLang 1.5: `else { ... }`, run when the step fails (decision 0033)


@dataclass(frozen=True)
class Grasp:
    robot: Name
    obj: Name
    span: Span
    handler: Branch | None = None  # RoboLang 1.5: `else { ... }`, run when the step fails (decision 0033)


@dataclass(frozen=True)
class Place:
    robot: Name
    obj: Name
    target: Name
    span: Span
    handler: Branch | None = None  # RoboLang 1.5: `else { ... }`, run when the step fails (decision 0033)


@dataclass(frozen=True)
class Inspect:
    robot: Name
    obj: Name
    sensor: StringLit | Name  # a Name in 1.3, as in SensorOk
    span: Span
    handler: Branch | None = None  # RoboLang 1.5: `else { ... }`, run when the step fails (decision 0033)


@dataclass(frozen=True)
class Measure:
    """RoboLang 1.6: `measure r temperature of pan using probe as t;`. `kind` is a Name (kinds are not reserved)."""

    robot: Name
    kind: Name
    obj: Name
    sensor: Name | StringLit  # a StringLit is E310, as for `inspect`
    binding: Name
    span: Span
    handler: Branch | None = None


@dataclass(frozen=True)
class MeasureClock:
    """RoboLang 1.9 (decision 0042 T1): `measure r time as t;` or `measure r elapsed as d;`. No object, no sensor."""

    robot: Name
    kind: Name
    binding: Name
    span: Span
    handler: Branch | None = None


@dataclass(frozen=True)
class Do:
    """
    RoboLang 1.8 (decision 0040 A1, A5): `do r open closet;` or `do r set fryer to 175 degC;`. `action` is a name the
    robot's profile declares; `value` is a quantity or a string, or None.
    """

    robot: Name
    action: Name
    target: Name
    value: QuantityLit | StringLit | None
    span: Span
    handler: Branch | None = None


@dataclass(frozen=True)
class WaitFor:
    duration: Duration
    span: Span


@dataclass(frozen=True)
class WaitUntil:
    condition: Condition
    timeout: Duration
    span: Span
    handler: Branch | None = None  # RoboLang 1.5: `else { ... }`, run when the step fails (decision 0033)


@dataclass(frozen=True)
class Communicate:
    """`message` is a Name (a binding) in RoboLang 1.6 when a measured value is sent (decision 0036 V8)."""

    robot: Name
    channel: StringLit
    message: StringLit | Name
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


Statement = Union[Move, Grasp, Place, Inspect, Measure, MeasureClock, Do, WaitFor, WaitUntil, Communicate, If, Fail]
HANDLED = (Move, Grasp, Place, Inspect, Measure, MeasureClock, Do, WaitUntil)  # the statements with a failure handler


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
class RegionAnnotation:
    """RoboLang 1.7 (decision 0039): `@safety no_go_region R;` or `@safety in_region R;`. `key.text` is the kind."""

    key: Name
    region: Name
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
    annotations: tuple[SafetyAnnotation | RegionAnnotation, ...]
    pre: PreBlock | None
    plan: PlanBlock
    span: Span
    goal: GoalBlock | None = None  # required in 1.3 (E309), absent before
    verify: tuple[VerifyAnnotation, ...] = ()  # 1.3 only


@dataclass(frozen=True)
class Count:
    """RoboLang 2.0: a whole number in a mission, a run count or a number of retries."""

    value: int
    span: Span


@dataclass(frozen=True)
class RunLine:
    """RoboLang 2.0 (decision 0041): `run task(a, b);` in a mission. The arguments are parameters of the mission."""

    task: Name
    args: tuple[Name, ...]
    span: Span


@dataclass(frozen=True)
class Repeat:
    """
    RoboLang 2.0: `repeat at most N runs, for at most D { run ...; } until CONDITION;`. Both bounds and `until` are
    optional here; a mission needs a hard bound (E325).
    """

    max_runs: Count | None
    max_duration: Duration | None
    until: Condition | None
    span: Span


@dataclass(frozen=True)
class OnFailure:
    """RoboLang 2.0: `on failure { stop; }`, `{ continue; }` or `{ retry N; }`. `policy` is the word; `retries` is N."""

    policy: Name
    retries: Count | None
    span: Span


@dataclass(frozen=True)
class Mission:
    """RoboLang 2.0 (decision 0041 M1): runs tasks, one run after another. `repeat` is None for a single run."""

    name: Name
    params: tuple[Param, ...]
    runs: tuple[RunLine, ...]
    repeat: Repeat | None
    on_failure: OnFailure | None
    span: Span


@dataclass(frozen=True)
class Program:
    """`version` is None for a file without a version marker. `missions` is empty before 2.0."""

    version: VersionDecl | None
    tasks: tuple[Task, ...]
    span: Span
    missions: tuple[Mission, ...] = ()
