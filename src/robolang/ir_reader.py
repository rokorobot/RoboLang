"""
Reader and re-validator for externally supplied Task IR (spec/ir-reader-v0.1.md; CORE-1).

`read_ir(data, filename)` turns the bytes of an IR JSON document into a `TaskIR`, or raises `IRError`
listing every problem found (at most `MAX_ERRORS`), each located by a JSON Pointer. It is the consumer
of ir-schema-v1.0.md section 2 (duties 1 and 2): strict JSON, the schema of the claimed version, then
the static rules of the compiler again. The version fields are claims, never evidence.

Codes: I1xx are form errors (the bytes, the shape, the vocabulary of the claimed schema version),
I2xx are meaning errors (the compiler's static rules E301 to E316, restated on IR). A task with a
form error is not checked for meaning; other tasks are. Nothing here executes anything.
"""

from __future__ import annotations
import re
import dataclasses
from contextlib import contextmanager
from dataclasses import dataclass, fields
from decimal import Decimal

from . import ir
from .core import ReadError, read_document
from .diagnostics import MAX_ERRORS
from .measured import AREA_KINDS, CHARGE, KINDS, KINDS_V2_2, Atom, Operand, Scopes, View
from .parser import MAX_CONDITION_DEPTH, MAX_IF_DEPTH
from .paths import double_grasps, goalless_path
from .tokens import (
    CHOOSE_TYPES,
    CLOCK_KINDS,
    COMPARISON_OPERATORS,
    DURATION_UNITS,
    KIND_UNITS,
    KIND_UNITS_V2_2,
    MAX_MISSION_RUNS,
    MAX_RETRIES,
    RESERVED,
    RESERVED_BY_VERSION,
    MAX_REGION_LIMITS,
    REGION_LIMITS,
    SPEED_UNITS,
    VERIFY_KEYS,
)
from .validator import double_grasp_message, fail_reason_problem

SCHEMA_VERSIONS = tuple(dict.fromkeys(ir.IR_SCHEMA_VERSIONS.values()))  # "1.0" to "1.8": the index is the schema's rank
LANGUAGES_OF: dict[str, list[str]] = {}  # schema -> the languages that compile to it (1.8: RoboLang 1.9 and 2.0)
for _language, _schema in ir.IR_SCHEMA_VERSIONS.items():
    LANGUAGES_OF.setdefault(_schema, []).append(_language)
LANGUAGE_OF = {schema: languages[0] for schema, languages in LANGUAGES_OF.items()}  # the language it began in
SAFETY_KINDS = ("max_speed", "no_go_region", "in_region")
_REGION_RANK = 6  # the first schema with region limits (`no_go_region`, `in_region`)
_ACTIONS_RANK = 7  # the first schema with the step `do` and the condition `in_state` (RoboLang 1.8)
_CLOCK_RANK = 8  # the first schema with the step `read_clock`, `time` and `clock` operands (RoboLang 1.9)
_MESSAGE_RANK = 9  # the first schema with the condition `message_received` (RoboLang 2.1)
_KINDS_RANK = 10  # the first schema with the kinds fill, confidence, pressure, count and a unitless count literal (2.2)
_CHOOSE_RANK = 11  # the first schema with the step `choose` (RoboLang 2.3, decision 0045)
_CHARGE_RANK = 12  # the first schema with the step `read_charge` and the `charge` operand (RoboLang 2.4, decision 0048)
MAX_DECIMAL_CHARS = 64
_POLICIES = ("stop", "continue", "retry")  # a mission's failure policy (Mission IR 1.0)

_READ_CODES = {"encoding": "I101", "json": "I102", "duplicate_key": "I103", "number": "I104", "depth": "I105"}
_NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
_DECIMAL = re.compile(r"[0-9]+(\.[0-9]+)?")
_TIME_TEXT = re.compile(r"([01][0-9]|2[0-3]):[0-5][0-9]:[0-5][0-9]")  # a canonical time of day (schema 1.8)
_SURROGATE = re.compile("[\ud800-\udfff]")

# kind -> (first schema rank that has it, the string fields that name parameters)
_ATOMS = {
    "robot_ready": (0, ("robot",)),
    "region_clear": (0, ("region",)),
    "object_exists": (0, ("object",)),
    "sensor_ok": (0, ("sensor",)),
    "object_at": (2, ("object", "location")),
    "robot_at": (2, ("robot", "location")),
    "holding": (2, ("robot", "object")),
    "in_state": (_ACTIONS_RANK, ("target", "label")),
}
_MESSAGE_KEYS = ("kind", "channel")  # `message_received`: a channel, and optionally the exact `text`
_COMPOUND = {"and": 1, "or": 1, "not": 1}
_STEPS = {
    "move": ("robot", "target"),
    "grasp": ("robot", "object"),
    "place": ("robot", "object", "target"),
    "inspect": ("robot", "object", "sensor"),
    "communicate": ("robot", "channel", "message"),
}
_MEASURE_KEYS = ("robot", "reading", "object", "sensor", "binding")  # schema 1.5 (rank 5)
_VALUES_RANK = 5  # the first schema with `measure`, `compare` and a `binding` in `communicate`
_OPERAND_KEYS = {"binding": ("name",), "quantity": ("value", "unit"), "text": ("value",),
                 "live": ("reading", "object", "sensor"), "time": ("value",), "clock": ("reading",),
                 "charge": ("robot",)}
_LEFT_OPERANDS, _RIGHT_OPERANDS = ("binding", "live"), ("binding", "quantity", "text")
_QUANTITY_UNITS_V1_6 = tuple(unit for units in KIND_UNITS.values() for unit in units)
_QUANTITY_UNITS = tuple(dict.fromkeys(unit for units in KIND_UNITS_V2_2.values() for unit in units))  # schema 1.10
_CLOCK_LEFT, _CLOCK_RIGHT = ("clock",), ("time",)  # the operands schema 1.8 adds to a comparison
_CHARGE_LEFT = ("charge",)  # the operand schema 1.12 adds to a comparison (RoboLang 2.4)
_CLOCK_UNITS = ("ms", "s", "min")  # a duration an `elapsed` value compares with (schema 1.8)
_VALUE_CODES = {"scope": "I218", "clash": "I219", "kind": "I220", "operator": "I221", "goal": "I222", "live": "I223",
                "unit": "I117", "count": "I229"}  # schema 1.10: a quantity with no unit is a count's, and whole
# kind -> (field, parameter type it must name): the uses E303 and E304 check
_USES = {
    "sensor_ok": (),
    "live": (("object", "object"),),
    "robot_ready": (("robot", "robot"),),
    "region_clear": (("region", "region"),),
    "object_exists": (("object", "object"),),
    "object_at": (("object", "object"), ("location", "location")),
    "robot_at": (("robot", "robot"), ("location", "location")),
    "holding": (("robot", "robot"), ("object", "object")),
    "move": (("robot", "robot"), ("target", "location")),
    "grasp": (("robot", "robot"), ("object", "object")),
    "place": (("robot", "robot"), ("object", "object"), ("target", "location")),
    "inspect": (("robot", "robot"), ("object", "object")),
    "measure": (("robot", "robot"), ("object", "object")),
    "read_clock": (("robot", "robot"),),
    "read_charge": (("robot", "robot"),),
    "charge": (("robot", "robot"),),
    "communicate": (("robot", "robot"),),
    "do": (("robot", "robot"), ("target", ("object", "location"))),
    "in_state": (("target", ("object", "location")),),
    "message_received": (),
}
_PARAM_TYPES = {"robot": 0, "object": 0, "location": 0, "region": 0, "sensor": 2}
_BRANCH_STEPS = {"if": 3, "fail": 3}  # kind -> first schema rank that has it
# a failure handler: schema 1.4 (rank 4)
_HANDLED = ("move", "grasp", "place", "inspect", "measure", "read_clock", "read_charge", "do", "wait_until",
            "choose")
_CHOOSE_SOURCES = {"list": ("candidates",), "objects_at": ("location",), "robots": ()}  # kind -> its other keys
_HANDLER_KEYS = ("else",)  # a handler has no goal (decision 0034)


@dataclass(frozen=True)
class IRDiagnostic:
    code: str
    message: str
    pointer: str  # RFC 6901 JSON Pointer to the value at fault; "" is the whole document
    filename: str = "<string>"

    def format(self) -> str:
        return f"{self.filename}: {self.pointer or '(document)'}: error[{self.code}]: {self.message}"

    def __str__(self) -> str:
        return self.format()


class IRError(Exception):
    """
    An IR document is not valid. `diagnostics` is every problem found, in document order, at most
    `MAX_ERRORS`; `truncated` is true when more were found than are listed; `diagnostic` is the first.
    """

    def __init__(self, diagnostics, *, truncated: bool = False):
        diagnostics = tuple(diagnostics)
        if len(diagnostics) > MAX_ERRORS:
            diagnostics, truncated = diagnostics[:MAX_ERRORS], True
        super().__init__(diagnostics[0].format())
        self.diagnostics = diagnostics
        self.diagnostic = diagnostics[0]
        self.truncated = truncated

    @property
    def code(self) -> str:
        return self.diagnostic.code


class _TooMany(Exception):
    """More errors than are reported: stop reading."""


def _pointer(base: str, part) -> str:
    return f"{base}/{str(part).replace('~', '~0').replace('/', '~1')}"


def _what(value) -> str:
    if isinstance(value, dict):
        return "an object"
    if isinstance(value, list):
        return "an array"
    if isinstance(value, str):
        return "a string"
    if value is None:
        return "null"
    return "a boolean" if isinstance(value, bool) else "a number"


def _article(param_type: str) -> str:
    return f"an {param_type}" if param_type == "object" else f"a {param_type}"


class _Reader:
    def __init__(self, filename: str):
        self._filename = filename
        self.errors: list[IRDiagnostic] = []
        self._rank = 0
        self._schema = SCHEMA_VERSIONS[0]
        self._reserved = RESERVED
        self._mission_names: set = set()
        self._chosen: set[str] = set()  # schema 1.11: the names a `choose` binds, visible here (also in _params)
        self._measured: set[str] = set()  # the names a `measure` has bound so far in the task
        self._owner = "task"  # what the names in scope are parameters of: "task", or "mission" (Mission IR)

    def error(self, code: str, message: str, pointer: str):
        self.errors.append(IRDiagnostic(code, message, pointer, self._filename))
        if len(self.errors) > MAX_ERRORS:  # one more than is listed, so "too many" is known
            raise _TooMany

    # -------- primitives (form) --------

    def string(self, value, ptr: str) -> bool:
        if not isinstance(value, str):
            self.error("I110", f"expected a string, found {_what(value)}", ptr)
            return False
        if _SURROGATE.search(value):
            self.error("I119", "the string holds an unpaired surrogate, which is not text", ptr)
            return False
        return True

    def array(self, value, ptr: str) -> bool:
        if not isinstance(value, list):
            self.error("I110", f"expected an array, found {_what(value)}", ptr)
            return False
        return True

    def shape(self, value, ptr: str, required=(), optional=()) -> bool:
        """`value` is an object with every `required` key and no key outside `required` and `optional`."""
        if not isinstance(value, dict):
            self.error("I110", f"expected an object, found {_what(value)}", ptr)
            return False
        ok = True
        for key in value:
            if key not in required and key not in optional:
                self.error("I112", f"unknown field '{key}' for IR schema {self._schema}", _pointer(ptr, key))
                ok = False
        for key in required:
            if key not in value:
                self.error("I111", f"missing field '{key}'", ptr)
                ok = False
        return ok

    def _needs(self, name: str, since: int, ptr: str) -> bool:
        if self._rank >= since:
            return True
        self.error("I115", f"'{name}' is not part of IR schema {self._schema} (it needs {SCHEMA_VERSIONS[since]})", ptr)
        return False

    def name(self, value, ptr: str) -> bool:
        """A name being declared: a task or parameter name, which must be a legal identifier."""
        if not self.string(value, ptr):
            return False
        if not _NAME.fullmatch(value):
            self.error("I118", f"'{value}' is not a name: use a letter, then letters, digits or '_'", ptr)
        elif value in self._reserved:
            self.error("I118", f"'{value}' is a reserved word and cannot be used as a name", ptr)
        else:
            return True
        return False

    def decimal(self, value, ptr: str) -> bool:
        if not self.string(value, ptr):
            return False
        if len(value) > MAX_DECIMAL_CHARS or not _DECIMAL.fullmatch(value):
            self.error("I116", "expected an exact decimal in a string: digits with an optional fraction, "
                               f"no sign or exponent, at most {MAX_DECIMAL_CHARS} characters", ptr)
            return False
        return True

    def quantity(self, value, ptr: str, units: tuple[str, ...], unitless: bool) -> bool:
        ok = self.shape(value, ptr, ("value", "unit"))
        if isinstance(value, dict):
            if "value" in value:
                ok = self.decimal(value["value"], _pointer(ptr, "value")) and ok
            if "unit" in value:
                unit, at = value["unit"], _pointer(ptr, "unit")
                if unit is None and unitless:
                    pass
                elif isinstance(unit, str) and unit in units:
                    pass
                else:
                    listed = ", ".join(f"'{u}'" for u in units) + (" or null" if unitless else "")
                    self.error("I117", f"unit not allowed here; expected one of {listed}", at)
                    ok = False
        return ok

    # -------- conditions and steps (form) --------

    def condition(self, node, ptr: str, levels: int = 0) -> bool:
        """`levels`: and/or/not objects around this one. Past MAX_CONDITION_DEPTH the node is not read."""
        kind = self.kind(node, ptr)
        if kind is None:
            return False
        if kind == "message_received":
            if not self._needs(f"condition kind '{kind}'", _MESSAGE_RANK, _pointer(ptr, "kind")):
                return False
            return self.strings(node, ptr, _MESSAGE_KEYS, ("channel", "text"), ("text",))
        if kind in _ATOMS:
            since, keys = _ATOMS[kind]
            if not self._needs(f"condition kind '{kind}'", since, _pointer(ptr, "kind")):
                return False
            return self.strings(node, ptr, ("kind", *keys), keys)
        if kind == "compare":
            return self._needs("condition kind 'compare'", _VALUES_RANK, _pointer(ptr, "kind")) \
                and self.compare(node, ptr)
        if kind in _COMPOUND:
            if not self._needs(f"condition kind '{kind}'", _COMPOUND[kind], _pointer(ptr, "kind")):
                return False
            if levels >= MAX_CONDITION_DEPTH:
                self.error("I121", f"condition nested deeper than {MAX_CONDITION_DEPTH} levels "
                                   "(each and/or/not is one level)", ptr)
                return False
            if kind == "not":
                ok = self.shape(node, ptr, ("kind", "condition"))
                if "condition" in node:
                    ok = self.condition(node["condition"], _pointer(ptr, "condition"), levels + 1) and ok
                return ok
            return self.operands(node, ptr, kind, levels)
        self.error("I114", f"unknown condition kind '{kind}'", _pointer(ptr, "kind"))
        return False

    def compare(self, node, ptr: str) -> bool:
        """Schema 1.5: `left`, `op`, `right`. The forms of the operands and the operator are checked here."""
        ok = self.shape(node, ptr, ("kind", "left", "op", "right"))
        if "op" in node and self.string(node["op"], _pointer(ptr, "op")):
            if node["op"] not in COMPARISON_OPERATORS:
                self.error("I124", "unknown comparison operator; expected one of "
                                   + ", ".join(f"'{o}'" for o in COMPARISON_OPERATORS), _pointer(ptr, "op"))
                ok = False
        elif "op" in node:
            ok = False
        clock = self._rank >= _CLOCK_RANK
        charge = self._rank >= _CHARGE_RANK
        left = _LEFT_OPERANDS + (_CLOCK_LEFT if clock else ()) + (_CHARGE_LEFT if charge else ())
        for side, allowed in (("left", left),
                              ("right", _RIGHT_OPERANDS + (_CLOCK_RIGHT if clock else ()))):
            if side in node:
                ok = self.operand(node[side], _pointer(ptr, side), allowed, durations=clock) and ok
        return ok

    def operand(self, node, ptr: str, allowed: tuple[str, ...], durations: bool = False) -> bool:
        kind = self.kind(node, ptr)
        if kind is None:
            return False
        if kind not in allowed:
            self.error("I114", f"unknown operand kind '{kind}' here; expected one of "
                               + ", ".join(f"'{k}'" for k in allowed), _pointer(ptr, "kind"))
            return False
        keys = _OPERAND_KEYS[kind]
        if kind == "quantity" and self._rank >= _KINDS_RANK:  # a count literal has no unit (schema 1.10)
            ok = self.shape(node, ptr, ("kind", "value"), ("unit",))
        else:
            ok = self.shape(node, ptr, ("kind", *keys))
        for key in keys:
            if key not in node:
                continue
            at = _pointer(ptr, key)
            if kind == "binding" and key == "name":
                ok = self.name(node[key], at) and ok
            elif key == "value" and kind == "quantity":
                ok = self.decimal(node[key], at) and ok
            elif key == "unit":
                units = _QUANTITY_UNITS if self._rank >= _KINDS_RANK else _QUANTITY_UNITS_V1_6
                unit, units = node[key], units + (_CLOCK_UNITS if durations else ())
                if not (isinstance(unit, str) and unit in units):
                    listed = ", ".join(f"'{u}'" for u in units)
                    self.error("I117", f"unit not allowed here; expected one of {listed}", at)
                    ok = False
            elif kind == "time" and key == "value":
                ok = self.time_text(node[key], at) and ok
            elif kind == "clock" and key == "reading":
                ok = self.reading(node[key], at, clock=True) and ok
            elif key == "reading":
                ok = self.reading(node[key], at) and ok
            else:
                ok = self.string(node[key], at) and ok
        return ok

    def reading(self, value, ptr: str, clock: bool = False) -> bool:
        """A measured kind: `temperature`, `mass`, `distance` or `label`; for a clock reading `time` or `elapsed`."""
        if not self.string(value, ptr):
            return False
        kinds = CLOCK_KINDS if clock else KINDS_V2_2 if self._rank >= _KINDS_RANK else KINDS
        if value not in kinds:
            self.error("I125", f"unknown {'clock' if clock else 'measured'} kind '" + value + "'; expected one of "
                       + ", ".join(f"'{k}'" for k in kinds), ptr)
            return False
        return True

    def time_text(self, value, ptr: str) -> bool:
        """Schema 1.8: a time of day is the text `HH:MM:SS`, 24-hour (E105)."""
        if not self.string(value, ptr):
            return False
        if not _TIME_TEXT.fullmatch(value):
            self.error("I126", f"not a time of day: {value!r}; the form is HH:MM:SS, 24-hour, hours 00 to 23", ptr)
            return False
        return True

    def kind(self, node, ptr: str) -> str | None:
        """The `kind` string of a condition or step object, or None after reporting why there is none."""
        if not isinstance(node, dict):
            self.error("I110", f"expected an object, found {_what(node)}", ptr)
        elif "kind" not in node:
            self.error("I111", "missing field 'kind'", ptr)
        elif not self.string(node["kind"], _pointer(ptr, "kind")):
            pass
        else:
            return node["kind"]
        return None

    def strings(self, node, ptr: str, required, string_keys, optional=()) -> bool:
        ok = self.shape(node, ptr, required, optional)
        for key in string_keys:
            if key in node:
                ok = self.string(node[key], _pointer(ptr, key)) and ok
        return ok

    def operands(self, node, ptr: str, kind: str, levels: int) -> bool:
        ok = self.shape(node, ptr, ("kind", "conditions"))
        if "conditions" not in node or not self.array(node["conditions"], _pointer(ptr, "conditions")):
            return False
        at = _pointer(ptr, "conditions")
        if len(node["conditions"]) < 2:
            self.error("I120", f"'{kind}' needs two or more conditions", at)
            ok = False
        for i, operand in enumerate(node["conditions"]):
            ok = self.condition(operand, _pointer(at, i), levels + 1) and ok
            if isinstance(operand, dict) and operand.get("kind") == kind:
                self.error("I122", f"'{kind}' directly inside '{kind}': grouping must be flattened "
                                   "(schema 1.1 section 2.1)", _pointer(at, i))
                ok = False
        return ok

    def conditions(self, value, ptr: str) -> bool:
        if not self.array(value, ptr):
            return False
        ok = True
        for i, node in enumerate(value):
            ok = self.condition(node, _pointer(ptr, i)) and ok
        return ok

    def steps(self, value, ptr: str, ifs: int, branch: bool = False) -> bool:
        """A non-empty array of steps. `ifs`: the `if` steps around it. A branch of an `if` may be empty."""
        if not self.array(value, ptr):
            return False
        ok = True
        if not value and not branch:
            what = "a task" if ifs == 0 else "a branch"
            self.error("I120", f"{what} needs at least one step", ptr)
            ok = False
        for i, step in enumerate(value):
            ok = self.step(step, _pointer(ptr, i), ifs) and ok
        return ok

    def branch(self, node: dict, ptr: str, side: str, ifs: int) -> bool:
        """The `then` or `else` steps and the `goal_then` or `goal_else` of an `if`. Only `else` may be empty."""
        ok = True
        goal, body = node.get(f"goal_{side}"), node.get("then" if side == "then" else "else")
        if f"goal_{side}" in node and goal is not None:
            at = _pointer(ptr, f"goal_{side}")
            ok = self.conditions(goal, at)
            if isinstance(goal, list) and not goal:
                self.error("I120", "a branch goal needs at least one condition (use null for none)", at)
                ok = False
        key = "then" if side == "then" else "else"
        if key in node:
            ok = self.steps(body, _pointer(ptr, key), ifs + 1, branch=side == "else") and ok
            if side == "else" and body == [] and goal is not None:
                self.error("I120", "'goal_else' needs a non-empty 'else'", _pointer(ptr, "goal_else"))
                ok = False
        return ok

    def step(self, node, ptr: str, ifs: int = 0) -> bool:
        kind = self.kind(node, ptr)
        if kind is None:
            return False
        if kind in _BRANCH_STEPS and not self._needs(f"step kind '{kind}'", _BRANCH_STEPS[kind], _pointer(ptr, "kind")):
            return False
        if kind == "fail":
            return self.strings(node, ptr, ("kind", "reason"), ("reason",))
        if kind == "if":
            if ifs >= MAX_IF_DEPTH:
                self.error("I123", f"'if' nested deeper than {MAX_IF_DEPTH} levels", ptr)
                return False
            ok = self.shape(node, ptr, ("kind", "condition", "then", "else", "goal_then", "goal_else"))
            if "condition" in node:
                ok = self.condition(node["condition"], _pointer(ptr, "condition")) and ok
            return self.branch(node, ptr, "then", ifs) and self.branch(node, ptr, "else", ifs) and ok
        handled = _HANDLER_KEYS if kind in _HANDLED and self._rank >= 4 else ()
        if kind == "measure":
            if not self._needs("step kind 'measure'", _VALUES_RANK, _pointer(ptr, "kind")):
                return False
            ok = self.strings(node, ptr, ("kind", *_MEASURE_KEYS), ("robot", "object", "sensor"), handled)
            if "reading" in node:
                ok = self.reading(node["reading"], _pointer(ptr, "reading")) and ok
            if "binding" in node:
                ok = self.name(node["binding"], _pointer(ptr, "binding")) and ok
            return self.handler(node, ptr, ifs) and ok
        if kind == "read_clock":
            if not self._needs("step kind 'read_clock'", _CLOCK_RANK, _pointer(ptr, "kind")):
                return False
            ok = self.strings(node, ptr, ("kind", "robot", "reading", "binding"), ("robot", "binding"), handled)
            if "reading" in node:
                ok = self.reading(node["reading"], _pointer(ptr, "reading"), clock=True) and ok
            if "binding" in node:
                ok = self.name(node["binding"], _pointer(ptr, "binding")) and ok
            return self.handler(node, ptr, ifs) and ok
        if kind == "read_charge":
            if not self._needs("step kind 'read_charge'", _CHARGE_RANK, _pointer(ptr, "kind")):
                return False
            ok = self.strings(node, ptr, ("kind", "robot", "binding"), ("robot", "binding"), handled)
            if "binding" in node:
                ok = self.name(node["binding"], _pointer(ptr, "binding")) and ok
            return self.handler(node, ptr, ifs) and ok
        if kind == "do":
            known = self._needs("step kind 'do'", _ACTIONS_RANK, _pointer(ptr, "kind"))
            return known and self.do(node, ptr, ifs, handled)
        if kind == "choose":
            known = self._needs("step kind 'choose'", _CHOOSE_RANK, _pointer(ptr, "kind"))
            return known and self.choose(node, ptr, ifs)
        if kind == "communicate" and self._rank >= _VALUES_RANK:
            return self.communicate(node, ptr)
        if kind in _STEPS:
            keys = _STEPS[kind]
            return self.strings(node, ptr, ("kind", *keys), keys, handled) and self.handler(node, ptr, ifs)
        if kind == "wait_for":
            ok = self.shape(node, ptr, ("kind", "duration"))
            if "duration" in node:
                ok = self.quantity(node["duration"], _pointer(ptr, "duration"), DURATION_UNITS, False) and ok
            return ok
        if kind == "wait_until":
            ok = self.shape(node, ptr, ("kind", "condition", "timeout"), handled)
            if "condition" in node:
                ok = self.condition(node["condition"], _pointer(ptr, "condition")) and ok
            if "timeout" in node:
                ok = self.quantity(node["timeout"], _pointer(ptr, "timeout"), DURATION_UNITS, False) and ok
            return self.handler(node, ptr, ifs) and ok
        self.error("I114", f"unknown step kind '{kind}'", _pointer(ptr, "kind"))
        return False

    def choose(self, node, ptr: str, ifs: int) -> bool:
        """Schema 1.11: `type`, `name`, `source`, an optional `where` and `order`, and the `else` always there."""
        ok = self.shape(node, ptr, ("kind", "type", "name", "source", "else"), ("where", "order"))
        if not isinstance(node, dict):
            return False
        if "type" in node and self.string(node["type"], _pointer(ptr, "type")):
            if node["type"] not in CHOOSE_TYPES:
                self.error("I114", f"unknown type '{node['type']}' of a choice; expected one of "
                                   f"{', '.join(CHOOSE_TYPES)}", _pointer(ptr, "type"))
                ok = False
        elif "type" in node:
            ok = False
        if "name" in node:
            ok = self.name(node["name"], _pointer(ptr, "name")) and ok
        if "source" in node:
            ok = self.source(node["source"], _pointer(ptr, "source")) and ok
        if "where" in node:
            ok = self.condition(node["where"], _pointer(ptr, "where")) and ok
        if "order" in node:
            at, order = _pointer(ptr, "order"), node["order"]
            ok = self.shape(order, at, ("mode", "reference")) and ok
            if isinstance(order, dict):
                if "mode" in order and self.string(order["mode"], _pointer(at, "mode")):
                    if order["mode"] not in ("nearest", "farthest"):
                        self.error("I114", f"unknown order '{order['mode']}'; expected nearest or farthest",
                                   _pointer(at, "mode"))
                        ok = False
                elif "mode" in order:
                    ok = False
                if "reference" in order:
                    ok = self.string(order["reference"], _pointer(at, "reference")) and ok
        return self.handler(node, ptr, ifs) and ok

    def source(self, node, ptr: str) -> bool:
        kind = self.kind(node, ptr)
        if kind is None:
            return False
        if kind not in _CHOOSE_SOURCES:
            self.error("I114", f"unknown source '{kind}' of a choice; expected one of "
                               f"{', '.join(_CHOOSE_SOURCES)}", _pointer(ptr, "kind"))
            return False
        keys = _CHOOSE_SOURCES[kind]
        ok = self.shape(node, ptr, ("kind", *keys))
        if kind == "objects_at" and "location" in node:
            ok = self.string(node["location"], _pointer(ptr, "location")) and ok
        if kind == "list" and "candidates" in node:
            at = _pointer(ptr, "candidates")
            if not self.array(node["candidates"], at):
                return False
            if not node["candidates"]:
                self.error("I120", "a list of candidates needs at least one name", at)
                ok = False
            for i, name in enumerate(node["candidates"]):
                ok = self.string(name, _pointer(at, i)) and ok
        return ok

    def do(self, node, ptr: str, ifs: int, handled) -> bool:
        """Schema 1.7: `robot`, `action` and `target` names, an optional `value` (a quantity or a text), a handler."""
        ok = self.strings(node, ptr, ("kind", "robot", "action", "target"), ("robot", "target"), ("value", *handled))
        if "action" in node:
            ok = self.name(node["action"], _pointer(ptr, "action")) and ok
        if "value" in node:
            ok = self.operand(node["value"], _pointer(ptr, "value"), ("quantity", "text")) and ok
            value = node["value"]
            if isinstance(value, dict) and value.get("kind") == "quantity" and "unit" not in value:  # schema 1.10
                self.error("I111", "missing field 'unit'", _pointer(ptr, "value"))
                ok = False
        return self.handler(node, ptr, ifs) and ok

    def communicate(self, node, ptr: str) -> bool:
        """Schema 1.5: `message` (text) or `binding` (the name of a `measure`), exactly one."""
        ok = self.strings(node, ptr, ("kind", "robot", "channel"), ("robot", "channel", "message"),
                          ("message", "binding"))
        if "binding" in node:
            ok = self.name(node["binding"], _pointer(ptr, "binding")) and ok
        if isinstance(node, dict):
            if "message" in node and "binding" in node:
                self.error("I112", "a communicate has 'message' or 'binding', not both", ptr)
                ok = False
            elif "message" not in node and "binding" not in node:
                self.error("I111", "a communicate needs 'message' (text) or 'binding' (a measured value)", ptr)
                ok = False
        return ok

    def handler(self, node: dict, ptr: str, ifs: int) -> bool:
        """Schema 1.4 (decisions 0033, 0034): `else`, the non-empty steps run when the step fails. No goal."""
        if not isinstance(node, dict) or self._rank < 4 or "else" not in node:
            return True
        if ifs >= MAX_IF_DEPTH:
            self.error("I123", f"failure handler nested deeper than {MAX_IF_DEPTH} levels (an 'if' and a handler "
                               "each count as one)", ptr)
            return False
        return self.steps(node["else"], _pointer(ptr, "else"), ifs + 1)

    # -------- task (form) --------

    def task_form(self, task, ptr: str) -> bool:
        optional = ("goal", "verify") if self._rank >= 2 else ()
        ok = self.shape(task, ptr, ("name", "parameters", "safety", "preconditions", "steps"), optional)
        if not isinstance(task, dict):
            return False
        if "name" in task:
            ok = self.name(task["name"], _pointer(ptr, "name")) and ok
        if "parameters" in task:
            ok = self.parameters(task["parameters"], _pointer(ptr, "parameters")) and ok
        if "safety" in task:
            ok = self.safety(task["safety"], _pointer(ptr, "safety")) and ok
        if "preconditions" in task:
            ok = self.conditions(task["preconditions"], _pointer(ptr, "preconditions")) and ok
        if self._rank >= 2:
            ok = self.goal(task, ptr) and ok
            if "verify" in task:
                ok = self.verify(task["verify"], _pointer(ptr, "verify")) and ok
        if "steps" in task:
            ok = self.steps(task["steps"], _pointer(ptr, "steps"), 0) and ok
        return ok

    def goal(self, task: dict, ptr: str) -> bool:
        """
        Schema 1.2 requires a non-empty goal; schema 1.3 requires the key but may leave it empty (the branches then
        give every path its goal: checked in `task_meaning`). A missing or empty goal is I209 (E309) and does not
        stop the rest.
        """
        at = _pointer(ptr, "goal")
        if "goal" not in task or (task["goal"] == [] and self._rank < 3):
            self.error("I209", f"this task has no goal: IR schema {self._schema} requires 'goal' with at least one "
                               "condition" if self._rank < 3 else "this task has no 'goal' key: IR schema 1.3 "
                               "requires it, as an empty array when only the branches give goals",
                       at if "goal" in task else ptr)
            return True
        return self.conditions(task["goal"], at)

    def parameters(self, value, ptr: str) -> bool:
        if not self.array(value, ptr):
            return False
        ok = True
        for i, param in enumerate(value):
            at = _pointer(ptr, i)
            ok = self.shape(param, at, ("name", "type")) and ok
            if not isinstance(param, dict):
                continue
            if "name" in param:
                ok = self.name(param["name"], _pointer(at, "name")) and ok
            if "type" in param and self.string(param["type"], _pointer(at, "type")):
                kind = param["type"]
                if kind not in _PARAM_TYPES:
                    self.error("I114", f"unknown parameter type '{kind}'", _pointer(at, "type"))
                    ok = False
                else:
                    ok = self._needs(f"parameter type '{kind}'", _PARAM_TYPES[kind], _pointer(at, "type")) and ok
            elif "type" in param:
                ok = False
        return ok

    def safety(self, value, ptr: str) -> bool:
        if not self.array(value, ptr):
            return False
        ok = True
        for i, limit in enumerate(value):
            at = _pointer(ptr, i)
            region = isinstance(limit, dict) and limit.get("kind") in REGION_LIMITS
            ok = self.shape(limit, at, ("kind", "region") if region else ("kind", "limit")) and ok
            if not isinstance(limit, dict):
                continue
            if "kind" in limit and self.string(limit["kind"], _pointer(at, "kind")):
                if limit["kind"] not in SAFETY_KINDS:
                    self.error("I114", f"unknown safety kind '{limit['kind']}'", _pointer(at, "kind"))
                    ok = False
                elif region:
                    ok = self._needs(f"safety kind '{limit['kind']}'", _REGION_RANK, _pointer(at, "kind")) and ok
            elif "kind" in limit:
                ok = False
            if "limit" in limit:
                ok = self.quantity(limit["limit"], _pointer(at, "limit"), SPEED_UNITS, True) and ok
            if "region" in limit:
                ok = self.string(limit["region"], _pointer(at, "region")) and ok
        return ok

    def verify(self, value, ptr: str) -> bool:
        ok = self.shape(value, ptr, (), VERIFY_KEYS)
        if isinstance(value, dict):
            if "min_confidence" in value:
                ok = self.decimal(value["min_confidence"], _pointer(ptr, "min_confidence")) and ok
            if "max_age" in value:
                ok = self.quantity(value["max_age"], _pointer(ptr, "max_age"), DURATION_UNITS, False) and ok
        return ok

    # -------- meaning (I2xx): the compiler's static rules, restated on IR --------

    def task_meaning(self, task: dict, ptr: str):
        self._params: dict[str, str] = {}
        for i, param in enumerate(task["parameters"]):
            if param["name"] in self._params:
                self.error("I202", f"duplicate parameter name '{param['name']}' in task '{task['name']}'",
                           _pointer(_pointer(_pointer(ptr, "parameters"), i), "name"))
                continue  # the first declaration stands
            self._params[param["name"]] = param["type"]
        self.limits(task["safety"], _pointer(ptr, "safety"))
        if "verify" in task:
            self.verify_meaning(task["verify"], _pointer(ptr, "verify"))
        self._chosen, self._measured = set(), set()
        self.condition_list(task["preconditions"], _pointer(ptr, "preconditions"))
        if self._rank >= 2:
            with self.goal_names(task["steps"]):
                self.goal_list(task.get("goal", []), _pointer(ptr, "goal"))
        params = dict(self._params)  # the parameters alone: a name a `choose` binds is no parameter of the task
        before = len(self.errors)
        self.steps_meaning(task["steps"], _pointer(ptr, "steps"), branch=False)
        if self._rank >= _VALUES_RANK:
            self.bindings(task, ptr, params)
        if len(self.errors) == before:  # names and types are right, so the grasps can be followed
            self.holders(task["steps"], _pointer(ptr, "steps"))
        if self._rank >= 3 and "goal" in task and not task["goal"]:  # a missing key was reported as I209 above
            label = goalless_path(task["steps"], False, _parts)
            if label is not None:
                where = f"path {label}" if label else "the plan"
                self.error("I209", f"no goal on {where}: the task 'goal' is empty and a path that does not end in "
                                   "'fail' has no branch goal", _pointer(ptr, "goal"))

    def bindings(self, task: dict, ptr: str, params: dict[str, str]):
        """Schema 1.5: bindings, kinds, units and where a comparison may stand (E317 to E322, as I218 to I223)."""
        def report(rule: str, message: str, at: str):
            self.error(_VALUE_CODES[rule], message, at)
        scopes = Scopes(params, _view, _atoms, report)
        scopes.forbid(_handles(task["preconditions"], _pointer(ptr, "preconditions")), "a 'pre' or 'goal' block")
        scopes.forbid(_handles(task.get("goal", []), _pointer(ptr, "goal")), "a 'pre' or 'goal' block")
        scopes.plan(_handles(task["steps"], _pointer(ptr, "steps")))

    def holders(self, steps: list, ptr: str):
        """E315 (decision 0032): a grasp that fails whenever its path is taken, as the object or robot is held."""
        pointers: dict[int, str] = {}

        def locate(steps: list, at: str):
            for i, step in enumerate(steps):
                pointers[id(step)] = _pointer(at, i)
                if step["kind"] == "if":
                    locate(step["then"], _pointer(_pointer(at, i), "then"))
                if "else" in step:
                    locate(step["else"], _pointer(_pointer(at, i), "else"))

        locate(steps, ptr)
        for grasp, earlier, holder, held in double_grasps(steps, _holding_kinds):
            message = double_grasp_message(grasp["robot"], grasp["object"], holder, held, pointers[id(earlier)])
            self.error("I216", message, pointers[id(grasp)])

    def steps_meaning(self, steps: list, ptr: str, branch: bool):
        """
        `branch`: the steps are an `if` or `else` body, where a `fail` may be the last step (I214). A name that a
        `choose` binds (schema 1.11) is visible from the end of that step to the end of its block.
        """
        outer_params, outer_chosen = dict(self._params), set(self._chosen)
        for i, step in enumerate(steps):
            at = _pointer(ptr, i)
            if step["kind"] == "fail" and not (branch and i == len(steps) - 1):
                self.error("I214", "'fail' is allowed only as the last step of an if or else branch", at)
            self.step_meaning(step, at)
            if step["kind"] == "choose" and step["name"] not in self._params:
                self._params[step["name"]] = step["type"]
            if step["kind"] == "choose":
                self._chosen.add(step["name"])
        if branch:
            self._params, self._chosen = outer_params, outer_chosen

    @contextmanager
    def goal_names(self, steps: list):
        """Decision 0045 C7: a goal may name what a `choose` at the top level of its plan or branch binds."""
        added = {s["name"]: s["type"] for s in steps if s["kind"] == "choose" and s["name"] not in self._params}
        self._params.update(added)
        try:
            yield
        finally:
            for name in added:
                self._params.pop(name, None)

    def region_limits(self, entry: dict, at: str, earlier: dict[str, tuple[int, str]], index: int):
        """Schema 1.6 (E303, E304, E323, E324): a region parameter, named by one region limit only, at most 8."""
        region = entry["region"]
        declared = self._params.get(region)
        here = _pointer(at, "region")
        if declared is None:
            self.error("I203", f"undeclared name '{region}': it is not a parameter of this task", here)
            return
        if declared != "region":
            self.error("I204", f"'{region}' is {_article(declared)}, but '{entry['kind']}' expects a region here", here)
            return
        if region in earlier:
            first, kind = earlier[region]
            what = f"is already a {kind}" if kind == entry["kind"] else f"is already a {kind}, and cannot also be a " \
                                                                        f"{entry['kind']}"
            self.error("I224", f"region '{region}' {what} (at safety entry {first}); a task names a region in one "
                               "region limit only", at)
            return
        if len(earlier) >= MAX_REGION_LIMITS:
            self.error("I225", f"a task may declare at most {MAX_REGION_LIMITS} region limits", at)
            return
        earlier[region] = (index, entry["kind"])

    def limits(self, limits: list, ptr: str):
        earlier: dict[str | None, int] = {}
        regions: dict[str, tuple[int, str]] = {}
        for i, entry in enumerate(limits):
            at = _pointer(ptr, i)
            if entry["kind"] in REGION_LIMITS:
                self.region_limits(entry, at, regions, i)
                continue
            unit, value = entry["limit"]["unit"], entry["limit"]["value"]
            if unit in earlier:
                kind = f"'{unit}'" if unit else "unitless"
                self.error("I206", f"duplicate {kind} max_speed limit (first at {_pointer(ptr, earlier[unit])}); "
                                   "a task may declare at most one limit per unit type", at)
            elif earlier and (unit is None or None in earlier):
                self.error("I208", "a unitless max_speed cannot be combined with another max_speed limit; "
                                   "give every limit a unit ('m/s' or 'rad/s')", at)
            else:
                self.positive(value, "max_speed", _pointer(_pointer(at, "limit"), "value"))
            earlier.setdefault(unit, i)  # the first declaration stands

    def verify_meaning(self, verify: dict, ptr: str):
        if not verify:
            self.error("I213", "an empty 'verify': leave it out, or give 'min_confidence' or 'max_age'", ptr)
        if "min_confidence" in verify and Decimal(verify["min_confidence"]) > 1:
            self.error("I212", "min_confidence must be between 0 and 1", _pointer(ptr, "min_confidence"))
        if "max_age" in verify:
            self.positive(verify["max_age"]["value"], "max_age", _pointer(_pointer(ptr, "max_age"), "value"))

    def condition_list(self, conditions: list, ptr: str):
        for i, condition in enumerate(conditions):
            self.condition_meaning(condition, _pointer(ptr, i))

    def goal_list(self, conditions: list, ptr: str):
        """A goal (of a task or of an `if` branch): its conditions, and never a message (I228, the form of E327)."""
        self.condition_list(conditions, ptr)
        stack = [(c, _pointer(ptr, i)) for i, c in enumerate(conditions)]
        while stack:
            node, at = stack.pop()
            if node["kind"] in ("and", "or"):
                stack.extend((c, _pointer(_pointer(at, "conditions"), i)) for i, c in enumerate(node["conditions"]))
            elif node["kind"] == "not":
                stack.append((node["condition"], _pointer(at, "condition")))
            elif node["kind"] == "message_received":
                self.error("I228", "'message_received' cannot be in a goal: a message is not an end state of the "
                                   "world (decision 0043 S1)", at)

    def condition_meaning(self, node: dict, ptr: str):
        kind = node["kind"]
        if kind in ("and", "or"):
            self.condition_list(node["conditions"], _pointer(ptr, "conditions"))
        elif kind == "not":
            self.condition_meaning(node["condition"], _pointer(ptr, "condition"))
        elif kind == "compare":
            if node["left"]["kind"] == "live":
                self.live(node["left"], _pointer(ptr, "left"))
            elif node["left"]["kind"] == "charge":
                self.uses(node["left"], _pointer(ptr, "left"))
        elif kind == "message_received":  # schema 1.9 (E307 for an empty channel or text)
            self.not_empty(node["channel"], "channel name", _pointer(ptr, "channel"))
            if "text" in node and node["text"].strip(" ") == "":
                self.error("I207", "empty string: a message text must not be empty, or only spaces, which are "
                                   "trimmed (decision 0043 S3)", _pointer(ptr, "text"))
        else:
            self.uses(node, ptr)
            if kind == "sensor_ok":
                self.sensor(node["sensor"], _pointer(ptr, "sensor"), "sensor_ok")
            elif kind == "in_state":
                self.not_empty(node["label"], "state label", _pointer(ptr, "label"))

    def candidate(self, name: str, expected: str, context: str, ptr: str):
        """A candidate, the place of `objects at` or the reference of an order: a parameter, never an earlier choice."""
        if name in self._chosen:
            self.error("I232", f"'{name}' was chosen earlier: a choice cannot use another choice, as several choices "
                               "that depend on each other are not in RoboLang 2.3 (decision 0045 C9)", ptr)
            return
        declared = self._params.get(name)
        if declared is None:
            self.error("I203", f"undeclared name '{name}': it is not a parameter of this {self._owner}", ptr)
        elif declared not in ((expected,) if isinstance(expected, str) else expected):
            wanted = _article(expected) if isinstance(expected, str) else ", ".join(map(_article, expected))
            self.error("I204", f"'{name}' is {_article(declared)}, but {context} expects {wanted} here", ptr)

    def choose_meaning(self, node: dict, ptr: str):
        """Schema 1.11, the rules of E302, E303, E304, E329, E330 and E331 on a `choose` step object."""
        kind, name = node["type"], node["name"]
        if name in self._params or name in self._measured:
            what = "a parameter or an earlier choice" if name in self._params else "the binding of a 'measure'"
            self.error("I202", f"'{name}' is already {what}: a 'choose' binds a name of its own", _pointer(ptr, "name"))
            return
        source, at = node["source"], _pointer(ptr, "source")
        if source["kind"] == "list":
            for i, candidate in enumerate(source["candidates"]):
                self.candidate(candidate, kind, f"a candidate of a choice of {_article(kind)}",
                               _pointer(_pointer(at, "candidates"), i))
        elif source["kind"] == "objects_at":
            if kind != "object":
                self.error("I204", f"'objects at' gives objects, but this choice is {_article(kind)}", at)
            self.candidate(source["location"], "location", "'objects at'", _pointer(at, "location"))
        elif kind != "robot":
            self.error("I204", f"'robots' gives robots, but this choice is {_article(kind)}", at)
        order = node.get("order")
        if order is not None:
            self.candidate(order["reference"], ("robot", "object", "location"), f"'{order['mode']}'",
                           _pointer(_pointer(ptr, "order"), "reference"))
        if "where" in node:
            self.where_meaning(node["where"], _pointer(ptr, "where"), name, kind)
        body = node["else"]
        if not body or body[-1]["kind"] != "fail":
            self.error("I230", "the 'else' of a 'choose' must end in 'fail' (decision 0045 C6, point 1)",
                       _pointer(ptr, "else"))

    def where_meaning(self, condition: dict, ptr: str, name: str, kind: str):
        """C3 (E330): world facts only, and over the chosen name; the name is visible inside its own `where`."""
        names, stack = False, [(condition, ptr)]
        while stack:
            node, at = stack.pop()
            if node["kind"] in ("and", "or"):
                stack.extend((c, _pointer(_pointer(at, "conditions"), i)) for i, c in enumerate(node["conditions"]))
            elif node["kind"] == "not":
                stack.append((node["condition"], _pointer(at, "condition")))
            elif node["kind"] == "compare" and self.charge_test(node, at, name, kind):
                names = True  # schema 1.12 (RoboLang 2.4, decision 0048 S7): the chosen robot's charge
            elif node["kind"] in ("compare", "message_received"):
                self.error("I231", "'where' states world facts only: not a reading, the clock or a message "
                                   "(decision 0045 C3, C9)", at)
                return
            else:
                names = names or name in (node.get(f) for f in ("robot", "object", "location", "target"))
        if not names:
            self.error("I231", f"'where' must be a condition over the chosen name '{name}'", ptr)
            return
        self._params[name] = kind
        try:
            self.condition_meaning(condition, ptr)
        finally:
            del self._params[name]

    def charge_test(self, node: dict, ptr: str, name: str, kind: str) -> bool:
        """The one reading a `where` may state (E330 narrowed, decision 0048 S7): `robot_charge t >= 50 %`, t chosen."""
        left, right = node["left"], node["right"]
        if not (self._rank >= _CHARGE_RANK and kind == "robot" and left["kind"] == "charge" and left["robot"] == name
                and right["kind"] == "quantity"):
            return False
        if right.get("unit") != "%":
            self.error("I220", "a charge is compared with a quantity in % (decision 0048 S1)", _pointer(ptr, "right"))
        if node["op"] not in ("<", "<=", ">", ">="):
            self.error("I221", "charge is compared with '<', '<=', '>' or '>=', not '" + node["op"] + "'",
                       _pointer(ptr, "op"))
        return True

    def live(self, node: dict, ptr: str):
        """The object and sensor of a live reading are parameters of the right types (E303, E304, E310)."""
        self.uses({"kind": "live", "object": node["object"], "reading": node["reading"]}, ptr)
        self.sensor(node["sensor"], _pointer(ptr, "sensor"), "a live reading")

    def step_meaning(self, node: dict, ptr: str):
        self.plain_step_meaning(node, ptr)
        if "else" in node and node["kind"] != "if":  # a failure handler is a branch without a goal (0033, 0034)
            self.steps_meaning(node["else"], _pointer(ptr, "else"), branch=True)

    def plain_step_meaning(self, node: dict, ptr: str):
        kind = node["kind"]
        if kind == "if":
            self.condition_meaning(node["condition"], _pointer(ptr, "condition"))
            for side, key in (("then", "then"), ("else", "else")):
                if node[f"goal_{side}"] is not None:
                    with self.goal_names(node[key]):
                        self.goal_list(node[f"goal_{side}"], _pointer(ptr, f"goal_{side}"))
                self.steps_meaning(node[key], _pointer(ptr, key), branch=True)
        elif kind == "choose":
            self.choose_meaning(node, ptr)
        elif kind == "fail":
            self.not_empty(node["reason"], "reason", _pointer(ptr, "reason"))
            problem = fail_reason_problem(node["reason"])
            if problem:
                self.error("I215", problem, _pointer(ptr, "reason"))
        elif kind == "wait_for":
            self.positive(node["duration"]["value"], "duration", _pointer(_pointer(ptr, "duration"), "value"))
        elif kind == "wait_until":
            self.condition_meaning(node["condition"], _pointer(ptr, "condition"))
            self.positive(node["timeout"]["value"], "timeout", _pointer(_pointer(ptr, "timeout"), "value"))
            if self._rank >= 4 and "else" not in node:
                self.error("I217", "in IR schema 1.4 a 'wait_until' needs a failure handler ('else') (E316)", ptr)
        else:
            self.uses(node, ptr)
            if kind in ("measure", "read_clock", "read_charge"):
                if node["binding"] in self._chosen:
                    taken = _article(self._params[node["binding"]])
                    self.error("I219", f"binding '{node['binding']}' has the name of {taken} that a 'choose' bound",
                               _pointer(ptr, "binding"))
                self._measured.add(node["binding"])
            if kind in ("inspect", "measure"):
                self.sensor(node["sensor"], _pointer(ptr, "sensor"), kind)
            elif kind == "do":
                value = node.get("value")
                if isinstance(value, dict) and value["kind"] == "text":
                    self.not_empty(value["value"], "value", _pointer(_pointer(ptr, "value"), "value"))
            elif kind == "communicate":
                for key, what in (("channel", "channel name"), ("message", "message")):
                    if key in node:  # schema 1.5 may send a binding instead of a message
                        self.not_empty(node[key], what, _pointer(ptr, key))

    def uses(self, node: dict, ptr: str):
        """E303 and E304: every parameter a condition or step names is declared, with the type it needs."""
        uses = _USES[node["kind"]]
        if self._rank >= _KINDS_RANK and node.get("reading") in AREA_KINDS:  # schema 1.10: a fill or count of a place
            uses = tuple((key, ("object", "location") if key == "object" else expected) for key, expected in uses)
        for key, expected in uses:
            name, at = node[key], _pointer(ptr, key)
            declared = self._params.get(name)
            if declared is None:
                self.error("I203", f"undeclared name '{name}': it is not a parameter of this {self._owner}", at)
            elif declared not in ((expected,) if isinstance(expected, str) else expected):
                wanted = _article(expected) if isinstance(expected, str) else " or ".join(map(_article, expected))
                self.error("I204", f"'{name}' is {_article(declared)}, but '{node['kind']}' expects {wanted} here", at)

    def sensor(self, name: str, ptr: str, context: str):
        """E307, E310 and the sensor case of E303/E304: a world sensor ID before schema 1.2, a parameter name in it."""
        if self._rank < 2:
            self.not_empty(name, "sensor name", ptr)
            return
        declared = self._params.get(name)
        if declared is None:
            self.error("I210", f"'{name}' is not a sensor parameter of this task: in IR schema 1.2 a sensor is "
                               f"the name of a 'sensor' parameter, not a world sensor ID ('{context}')", ptr)
        elif declared != "sensor":
            self.error("I204", f"'{name}' is {_article(declared)}, but '{context}' expects a sensor here", ptr)

    def positive(self, value: str, what: str, ptr: str):
        if Decimal(value) <= 0:
            self.error("I205", f"{what} must be greater than zero", ptr)

    def not_empty(self, value: str, what: str, ptr: str):
        if value == "":
            self.error("I207", f"empty string: a {what} must not be empty", ptr)

    # -------- missions (Mission IR 1.0, RoboLang 2.0) --------

    def missions(self, document: dict) -> bool:
        """Form, then meaning, of each mission; a mission with a form error is not checked for meaning."""
        missions = document.get("missions")
        if "missions" not in document or not self.array(missions, "/missions"):
            return False
        if not missions:
            self.error("I120", "a Mission IR document needs at least one mission", "/missions")
            return False
        tasks = _task_signatures(document.get("tasks"))
        self._mission_names = {m.get("name") for m in missions if isinstance(m, dict)}
        seen = {name: _pointer("/tasks", i) for i, name in enumerate(tasks_names(document.get("tasks")))}
        for i, mission in enumerate(missions):
            ptr = _pointer("/missions", i)
            name = mission.get("name") if isinstance(mission, dict) else None
            if isinstance(name, str):
                if name in seen:
                    self.error("I201", f"duplicate name '{name}': a mission and a task, or two missions, cannot share "
                                       f"a name (first defined at {seen[name]})", _pointer(ptr, "name"))
                seen.setdefault(name, ptr)
            if self.mission_form(mission, ptr):
                self.mission_meaning(mission, ptr, tasks)
        return True

    def count(self, value, ptr: str) -> bool:
        if isinstance(value, int) and not isinstance(value, bool):
            return True
        self.error("I110", f"expected a whole number, found {_what(value)}", ptr)
        return False

    def mission_form(self, mission, ptr: str) -> bool:
        ok = self.shape(mission, ptr, ("name", "parameters", "runs", "on_failure"), ("repeat",))
        if not isinstance(mission, dict):
            return False
        if "name" in mission:
            ok = self.name(mission["name"], _pointer(ptr, "name")) and ok
        if "parameters" in mission:
            ok = self.parameters(mission["parameters"], _pointer(ptr, "parameters")) and ok
        if "repeat" in mission:
            ok = self.repeat_form(mission["repeat"], _pointer(ptr, "repeat")) and ok
        if "runs" in mission:
            ok = self.runs_form(mission["runs"], _pointer(ptr, "runs")) and ok
        if "on_failure" in mission:
            ok = self.on_failure_form(mission["on_failure"], _pointer(ptr, "on_failure")) and ok
        return ok

    def repeat_form(self, repeat, ptr: str) -> bool:
        ok = self.shape(repeat, ptr, (), ("max_runs", "max_duration", "until"))
        if not isinstance(repeat, dict):
            return False
        if "max_runs" in repeat:
            ok = self.count(repeat["max_runs"], _pointer(ptr, "max_runs")) and ok
        if "max_duration" in repeat:
            ok = self.quantity(repeat["max_duration"], _pointer(ptr, "max_duration"), DURATION_UNITS, False) and ok
        if "until" in repeat:
            ok = self.condition(repeat["until"], _pointer(ptr, "until")) and ok
        return ok

    def runs_form(self, runs, ptr: str) -> bool:
        if not self.array(runs, ptr):
            return False
        if not runs:
            self.error("I120", "a mission needs at least one run", ptr)
            return False
        ok = True
        for i, run in enumerate(runs):
            at = _pointer(ptr, i)
            ok = self.shape(run, at, ("task", "arguments")) and ok
            if not isinstance(run, dict):
                continue
            if "task" in run:
                ok = self.string(run["task"], _pointer(at, "task")) and ok
            if "arguments" in run and self.array(run["arguments"], _pointer(at, "arguments")):
                for j, argument in enumerate(run["arguments"]):
                    ok = self.string(argument, _pointer(_pointer(at, "arguments"), j)) and ok
            elif "arguments" in run:
                ok = False
        return ok

    def on_failure_form(self, failure, ptr: str) -> bool:
        ok = self.shape(failure, ptr, ("policy",), ("retries",))
        if not isinstance(failure, dict):
            return False
        policy = failure.get("policy")
        if "policy" in failure and self.string(policy, _pointer(ptr, "policy")):
            if policy not in _POLICIES:
                self.error("I114", f"unknown failure policy '{policy}'; expected one of "
                                   + ", ".join(f"'{p}'" for p in _POLICIES), _pointer(ptr, "policy"))
                ok = False
        elif "policy" in failure:
            ok = False
        if "retries" in failure:
            ok = self.count(failure["retries"], _pointer(ptr, "retries")) and ok
            if policy in _POLICIES and policy != "retry":
                self.error("I112", f"unknown field 'retries' for policy '{policy}': only 'retry' has a count",
                           _pointer(ptr, "retries"))
                ok = False
        elif policy == "retry":
            self.error("I111", "missing field 'retries': 'retry' needs a count", ptr)
            ok = False
        return ok

    def mission_meaning(self, mission: dict, ptr: str, tasks: dict):
        """The compiler's rules for a mission (E302, E303, E304, E305, E317 to E320, E325, E326) on IR."""
        self._owner = "mission"
        self._params = {}
        for i, param in enumerate(mission["parameters"]):
            if param["name"] in self._params:
                self.error("I202", f"duplicate parameter name '{param['name']}' in mission '{mission['name']}'",
                           _pointer(_pointer(_pointer(ptr, "parameters"), i), "name"))
                continue
            self._params[param["name"]] = param["type"]
        for i, run in enumerate(mission["runs"]):
            self.run_meaning(run, _pointer(_pointer(ptr, "runs"), i), tasks)
        repeat = mission.get("repeat")
        if repeat is not None:
            self.repeat_meaning(repeat, _pointer(ptr, "repeat"))
        failure = mission["on_failure"]
        if "retries" in failure:
            self.counted(failure["retries"], "retry", MAX_RETRIES, _pointer(_pointer(ptr, "on_failure"), "retries"))
        self._owner = "task"

    def counted(self, value: int, what: str, most: int, ptr: str):
        if value <= 0:
            self.error("I205", f"{what} must be greater than zero", ptr)
        elif value > most:
            self.error("I226", f"{what} is at most {most}; this one is {value}", ptr)

    def repeat_meaning(self, repeat: dict, ptr: str):
        if "max_runs" in repeat:
            self.counted(repeat["max_runs"], "the number of runs", MAX_MISSION_RUNS, _pointer(ptr, "max_runs"))
        if "max_duration" in repeat:
            self.positive(repeat["max_duration"]["value"], "the duration bound",
                          _pointer(_pointer(ptr, "max_duration"), "value"))
        until = repeat.get("until")
        if until is not None:
            at = _pointer(ptr, "until")
            self.condition_meaning(until, at)
            scopes = Scopes(self._params, lambda handle: View(condition=handle, live_ok=True), _atoms,
                            lambda rule, message, where: self.error(_VALUE_CODES[rule], message, where))
            scopes.plan(((until, at),))
        if "max_runs" not in repeat and "max_duration" not in repeat and not time_bound(until):
            self.error("I226", "a repeat needs a hard bound (decision 0041 M2): 'max_runs', 'max_duration', or an "
                               "'until' of the form clock_time >= T that ends it at a time of day", ptr)

    def run_meaning(self, run: dict, ptr: str, tasks: dict):
        """I227 (E326), I203, I204: the run names a task of the document and gives it arguments of the right types."""
        name = run["task"]
        if name not in tasks and name in self._mission_names:
            self.error("I227", f"'run {name}': '{name}' is a mission, and a mission cannot run a mission "
                               "(decision 0041 M7): run a task", _pointer(ptr, "task"))
            return
        if name not in tasks:
            listed = ", ".join(sorted(tasks)) or "(none)"
            self.error("I227", f"'run {name}': there is no task '{name}' in this document; tasks: {listed}",
                       _pointer(ptr, "task"))
            return
        types = tasks[name]
        if types is None:  # the task has a form error, reported already
            return
        if len(run["arguments"]) != len(types):
            self.error("I227", f"'run {name}' takes {len(types)} argument{'' if len(types) == 1 else 's'}, found "
                               f"{len(run['arguments'])}", _pointer(ptr, "arguments"))
            return
        for i, (argument, wanted) in enumerate(zip(run["arguments"], types)):
            at = _pointer(_pointer(ptr, "arguments"), i)
            declared = self._params.get(argument)
            if declared is None:
                self.error("I203", f"undeclared name '{argument}': it is not a parameter of this mission", at)
            elif declared != wanted:
                self.error("I204", f"'{argument}' is {_article(declared)}, but task '{name}' expects "
                                   f"{_article(wanted)} here", at)

    # -------- document --------

    def document(self, document) -> bool:
        """Check the claims of the document header. False when the schema cannot be determined."""
        if isinstance(document, dict) and document.get("ir_schema") == ir.MISSION_IR_SCHEMA:
            return self.mission_header(document)
        if not self.shape(document, "", ("ir_schema", "ir_schema_version", "language_version", "tasks")):
            if not isinstance(document, dict) or "ir_schema_version" not in document:
                return False
        if document.get("ir_schema") != ir.IR_SCHEMA:
            self.error("I113", f"not a Task IR or Mission IR document: 'ir_schema' must be '{ir.IR_SCHEMA}' or "
                               f"'{ir.MISSION_IR_SCHEMA}'", "/ir_schema")
        version = document.get("ir_schema_version")
        if not isinstance(version, str) or version not in SCHEMA_VERSIONS:
            known = ", ".join(f"'{v}'" for v in SCHEMA_VERSIONS)
            self.error("I113", f"unsupported ir_schema_version; this reader knows {known}", "/ir_schema_version")
            return False
        self._task_header(document, version)
        return True

    def _task_header(self, document: dict, version: str):
        """The schema, its rank and the reserved words of the language the document claims (it must fit the schema)."""
        self._schema, self._rank = version, SCHEMA_VERSIONS.index(version)
        languages = LANGUAGES_OF[version]
        claimed = document.get("language_version")
        language = claimed if claimed in languages else languages[0]
        self._reserved = RESERVED_BY_VERSION.get(language, RESERVED)
        if claimed not in languages and "language_version" in document:
            wanted = " or ".join(f"'{lang}'" for lang in languages)
            self.error("I113", f"language_version must be {wanted} for ir_schema_version '{version}'",
                       "/language_version")

    def mission_header(self, document: dict) -> bool:
        """
        Mission IR 1.0 (RoboLang 2.0: Task IR 1.8 tasks) or 1.1 (RoboLang 2.1: Task IR 1.9 tasks): its own version, the
        Task IR version of its tasks, and the language.
        """
        self.shape(document, "", ("ir_schema", "ir_schema_version", "task_ir_schema_version", "language_version",
                                  "tasks", "missions"))
        known = {schema: language for language, schema in ir.MISSION_IR_SCHEMA_VERSIONS.items()}
        if document.get("ir_schema_version") not in known:
            listed = " and ".join(f"'{v}'" for v in known)
            self.error("I113", f"unsupported ir_schema_version for a Mission IR document; this reader knows {listed}",
                       "/ir_schema_version")
            return False
        language = known[document["ir_schema_version"]]
        version = ir.IR_SCHEMA_VERSIONS[language]
        if document.get("task_ir_schema_version") != version:
            self.error("I113", f"task_ir_schema_version must be '{version}': the tasks of Mission IR "
                               f"{document['ir_schema_version']} are Task IR {version}", "/task_ir_schema_version")
        if document.get("language_version") != language:
            self.error("I113", f"language_version must be '{language}' for a Mission IR document "
                               f"{document['ir_schema_version']}", "/language_version")
        self._task_header({"language_version": language}, version)
        return True


def time_bound(node: dict | None) -> bool:
    """Clarification 2 of decision 0041 on a condition object: `clock_time >= T` (or `>`), alone or in an `or`."""
    if node is None:
        return False
    if node["kind"] == "or":
        return any(time_bound(c) for c in node["conditions"])
    return (node["kind"] == "compare" and node["left"]["kind"] == "clock" and node["left"]["reading"] == "time"
            and node["op"] in (">", ">=") and node["right"]["kind"] == "time")


def _task_signatures(tasks) -> dict[str, list[str] | None]:
    """The tasks of a document as name -> parameter types, or None for a task whose parameters cannot be read."""
    found: dict[str, list[str] | None] = {}
    for task in tasks if isinstance(tasks, list) else ():
        name = task.get("name") if isinstance(task, dict) else None
        if not isinstance(name, str):
            continue
        params = task.get("parameters")
        ok = isinstance(params, list) and all(isinstance(p, dict) and isinstance(p.get("type"), str) for p in params)
        found.setdefault(name, [p["type"] for p in params] if ok else None)
    return found


def tasks_names(tasks) -> list[str | None]:
    """The names of the tasks, in order (None where there is none), as the positions in `/tasks`."""
    return [t.get("name") if isinstance(t, dict) and isinstance(t.get("name"), str) else None
            for t in tasks] if isinstance(tasks, list) else []


def _handles(items: list, ptr: str) -> tuple:
    """Steps or conditions with their JSON Pointers, the handles `measured.Scopes` reads."""
    return tuple((item, _pointer(ptr, i)) for i, item in enumerate(items))


def _whole(value: str) -> bool:
    return Decimal(value) == Decimal(value).to_integral_value()


def _operand_of(node: dict, ptr: str) -> Operand:
    kind = node["kind"]
    if kind == "binding":
        return Operand("binding", ptr, name=node["name"])
    if kind == "quantity":
        unit = node.get("unit")
        return Operand("quantity", ptr, unit=unit, whole=None if unit is not None else _whole(node["value"]))
    if kind == "live":
        return Operand("live", ptr, kind=node["reading"])
    if kind == "clock":
        return Operand("clock", ptr, kind=node["reading"])
    if kind == "charge":
        return Operand("charge", ptr, kind=CHARGE)
    if kind == "time":
        return Operand("time", ptr)
    return Operand("text", ptr)


def _atoms(handle: tuple):
    """The comparisons in a condition, in document order."""
    node, ptr = handle
    if node["kind"] in ("and", "or"):
        for atom in _handles(node["conditions"], _pointer(ptr, "conditions")):
            yield from _atoms(atom)
    elif node["kind"] == "not":
        yield from _atoms((node["condition"], _pointer(ptr, "condition")))
    elif node["kind"] == "compare":
        yield Atom(_operand_of(node["left"], _pointer(ptr, "left")), node["op"],
                   _operand_of(node["right"], _pointer(ptr, "right")), ptr)


def _view(handle: tuple) -> View:
    """The shape `measured.Scopes` reads from a step object."""
    step, ptr = handle
    kind = step["kind"]
    if kind == "fail":
        return View("fail")
    if kind == "if":
        goals = tuple(g for side in ("goal_then", "goal_else") if step[side] is not None
                      for g in _handles(step[side], _pointer(ptr, side)))
        return View("if", (step["condition"], _pointer(ptr, "condition")), goals=goals,
                    then=_handles(step["then"], _pointer(ptr, "then")),
                    orelse=_handles(step["else"], _pointer(ptr, "else")))
    handler = _handles(step["else"], _pointer(ptr, "else")) if "else" in step else None
    if kind in ("measure", "read_clock"):
        return View(handler=handler, measure=(step["binding"], step["reading"], _pointer(ptr, "binding")))
    if kind == "read_charge":
        return View(handler=handler, measure=(step["binding"], CHARGE, _pointer(ptr, "binding")))
    if kind == "wait_until":
        return View(condition=(step["condition"], _pointer(ptr, "condition")), live_ok=True, handler=handler)
    if kind == "communicate" and "binding" in step:
        return View(send=Operand("binding", _pointer(ptr, "binding"), name=step["binding"]))
    return View(handler=handler)


def _quantity(node: dict) -> ir.Quantity:
    return ir.Quantity(Decimal(node["value"]), node["unit"])


def _mission(node: dict) -> ir.Mission:
    repeat = node.get("repeat")
    failure = node["on_failure"]
    return ir.Mission(
        name=node["name"],
        parameters=tuple(ir.Parameter(p["name"], p["type"]) for p in node["parameters"]),
        repeat=None if repeat is None else ir.Repeat(
            repeat.get("max_runs"),
            _quantity(repeat["max_duration"]) if "max_duration" in repeat else None,
            _condition(repeat["until"]) if "until" in repeat else None),
        runs=tuple(ir.Run(r["task"], tuple(r["arguments"])) for r in node["runs"]),
        on_failure=ir.OnFailure(failure["policy"], failure.get("retries")),
    )


def _condition(node: dict) -> ir.Condition:
    kind = node["kind"]
    if kind in ("and", "or"):
        return (ir.And if kind == "and" else ir.Or)(tuple(_condition(c) for c in node["conditions"]))
    if kind == "not":
        return ir.Not(_condition(node["condition"]))
    if kind == "compare":
        return ir.Compare(_operand(node["left"]), node["op"], _operand(node["right"]))
    if kind == "in_state":
        return ir.InState(node["target"], node["label"])
    if kind == "message_received":
        return ir.MessageReceived(node["channel"], node.get("text"))
    return _of(next(c for c in _CONDITIONS if c.KIND == kind), node)


def _operand(node: dict) -> ir.Operand:
    kind = node["kind"]
    if kind == "quantity":
        return ir.QuantityLit(Decimal(node["value"]), node.get("unit"))
    classes = {"binding": ir.BindingRef, "text": ir.TextLit, "live": ir.LiveReading, "time": ir.TimeLit,
               "clock": ir.LiveClock, "charge": ir.LiveCharge}
    return _of(classes[kind], node)


def _of(cls, node: dict):
    """A dataclass with only string fields, read from the JSON keys the schema gives them."""
    return cls(**{f.name: node[f.metadata.get("key", f.name)] for f in fields(cls) if not f.kw_only})


_CONDITIONS = (ir.RobotReady, ir.RegionClear, ir.ObjectExists, ir.SensorOk, ir.ObjectAt, ir.RobotAt, ir.Holding)
_STEP_TYPES = {cls.KIND: cls for cls in (ir.Move, ir.Grasp, ir.Place, ir.Inspect, ir.Measure, ir.ReadClock,
                                         ir.ReadCharge)}


def _goal(node: list | None) -> tuple[ir.Condition, ...] | None:
    return None if node is None else tuple(_condition(c) for c in node)


def _parts(step: dict):
    """The shape `paths.goalless_path` reads, from a step object."""
    if step["kind"] == "fail":
        return "fail"
    if step["kind"] != "if" and "else" in step:
        return (), False, step["else"], False, ("ok", "failed")
    if step["kind"] != "if":
        return None
    return step["then"], step["goal_then"] is not None, step["else"], step["goal_else"] is not None


def _holding_kinds(step: dict):
    """The shape `paths.double_grasps` reads, from a step object."""
    kind = step["kind"]
    if kind == "fail":
        return "fail"
    if kind == "if":
        return "if", step["then"], step["else"], _holding_test(step["condition"])
    handler = step.get("else")
    if kind in ("grasp", "place"):
        return kind, step["robot"], step["object"], handler
    return None if handler is None else ("step", None, None, handler)


def _holding_test(condition: dict, positive: bool = True):
    """The `test` of `paths.double_grasps` for an `if` condition: a holding or object_at atom, or its negation."""
    if condition["kind"] == "not":
        return _holding_test(condition["condition"], not positive)
    if condition["kind"] == "holding":
        return "holding", condition["robot"], condition["object"], positive
    if condition["kind"] == "object_at":
        return "object_at", condition["object"], positive
    return None


def _step(node: dict) -> ir.Step:
    step = _plain_step(node)
    if node["kind"] == "if" or "else" not in node:
        return step
    return dataclasses.replace(step, orelse=tuple(_step(s) for s in node["else"]))


def _plain_step(node: dict) -> ir.Step:
    kind = node["kind"]
    if kind == "do":
        return ir.Do(node["robot"], node["action"], node["target"],
                     None if "value" not in node else _operand(node["value"]))
    if kind == "if":
        return ir.If(_condition(node["condition"]), tuple(_step(s) for s in node["then"]),
                     tuple(_step(s) for s in node["else"]), _goal(node["goal_then"]), _goal(node["goal_else"]))
    if kind == "fail":
        return ir.Fail(node["reason"])
    if kind == "choose":
        source = node["source"]
        order = node.get("order")
        return ir.Choose(
            node["type"], node["name"],
            ir.ChooseList(tuple(source["candidates"])) if source["kind"] == "list"
            else ir.ObjectsAt(source["location"]) if source["kind"] == "objects_at" else ir.AllRobots(),
            _condition(node["where"]) if "where" in node else None,
            None if order is None else ir.Order(order["mode"], order["reference"]), ())
    if kind == "wait_for":
        return ir.WaitFor(_quantity(node["duration"]))
    if kind == "wait_until":
        return ir.WaitUntil(_condition(node["condition"]), _quantity(node["timeout"]))
    if kind == "communicate":
        return ir.Communicate(node["robot"], node["channel"], node.get("message"), binding=node.get("binding"))
    return _of(_STEP_TYPES[kind], node)


def _verify(node: dict | None) -> ir.Verify | None:
    if node is None:
        return None
    confidence, max_age = node.get("min_confidence"), node.get("max_age")
    return ir.Verify(None if confidence is None else Decimal(confidence),
                     None if max_age is None else _quantity(max_age))


def _task(node: dict, rank: int) -> ir.Task:
    return ir.Task(
        name=node["name"],
        parameters=tuple(ir.Parameter(p["name"], p["type"]) for p in node["parameters"]),
        safety=tuple(
            ir.RegionLimit(s["kind"], s["region"]) if s["kind"] in REGION_LIMITS
            else ir.SafetyLimit(s["kind"], _quantity(s["limit"]))
            for s in node["safety"]
        ),
        preconditions=tuple(_condition(c) for c in node["preconditions"]),
        goal=tuple(_condition(c) for c in node["goal"]) if rank >= 2 else None,
        verify=_verify(node.get("verify")),
        steps=tuple(_step(s) for s in node["steps"]),
    )


def read_ir(data: bytes, filename: str = "<string>") -> ir.TaskIR:
    """
    Read and re-validate an IR JSON document. Raises IRError listing every problem found.

    The result equals the TaskIR the compiler produces for any source with the same meaning, and
    `ir.to_json` of it is the canonical text of the document. Reading is not permission to run.
    """
    try:
        document = read_document(data)
    except ReadError as error:
        raise IRError([IRDiagnostic(_READ_CODES[error.reason], str(error), "", filename)]) from None
    reader = _Reader(filename)
    clean = False
    try:
        if reader.document(document):
            clean = _tasks(reader, document)
            if document.get("ir_schema") == ir.MISSION_IR_SCHEMA:
                clean = reader.missions(document) and clean
    except _TooMany:
        pass
    if reader.errors:
        raise IRError(reader.errors)
    assert clean
    if document["ir_schema"] == ir.MISSION_IR_SCHEMA:
        schema, language = document["task_ir_schema_version"], document["language_version"]
    else:
        schema, language = document["ir_schema_version"], document["language_version"]
    rank = SCHEMA_VERSIONS.index(schema)
    missions = tuple(_mission(m) for m in document["missions"]) if "missions" in document else ()
    return ir.TaskIR(tuple(_task(t, rank) for t in document["tasks"]), language, ir.IR_SCHEMA, schema, missions)


def _tasks(reader: _Reader, document: dict) -> bool:
    tasks = document.get("tasks")
    if "tasks" not in document or not reader.array(tasks, "/tasks"):
        return False
    if not tasks:
        reader.error("I120", "a document needs at least one task", "/tasks")
        return False
    seen: dict[str, int] = {}
    for i, task in enumerate(tasks):
        ptr = _pointer("/tasks", i)
        name = task.get("name") if isinstance(task, dict) else None
        if isinstance(name, str):
            if name in seen:
                first = _pointer("/tasks", seen[name])
                reader.error("I201", f"duplicate task name '{name}' (first defined at {first})", _pointer(ptr, "name"))
            seen.setdefault(name, i)
        if reader.task_form(task, ptr):  # a task with a form error is not checked for meaning; other tasks are
            reader.task_meaning(task, ptr)
    return True
