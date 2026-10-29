"""
Reader and re-validator for externally supplied Task IR (spec/ir-reader-v0.1.md; CORE-1).

`read_ir(data, filename)` turns the bytes of an IR JSON document into a `TaskIR`, or raises `IRError`
listing every problem found (at most `MAX_ERRORS`), each located by a JSON Pointer. It is the consumer
of ir-schema-v1.0.md section 2 (duties 1 and 2): strict JSON, the schema of the claimed version, then
the static rules of the compiler again. The version fields are claims, never evidence.

Codes: I1xx are form errors (the bytes, the shape, the vocabulary of the claimed schema version),
I2xx are meaning errors (the compiler's static rules E301 to E312, restated on IR). A task with a
form error is not checked for meaning; other tasks are. Nothing here executes anything.
"""

from __future__ import annotations
import re
from dataclasses import dataclass, fields
from decimal import Decimal

from . import ir
from .core import ReadError, read_document
from .diagnostics import MAX_ERRORS
from .parser import MAX_CONDITION_DEPTH, MAX_IF_DEPTH
from .paths import goalless_path
from .tokens import DURATION_UNITS, RESERVED, RESERVED_BY_VERSION, SPEED_UNITS, VERIFY_KEYS

SCHEMA_VERSIONS = tuple(ir.IR_SCHEMA_VERSIONS.values())  # "1.0" to "1.3": the index is the schema's rank
LANGUAGE_OF = {schema: language for language, schema in ir.IR_SCHEMA_VERSIONS.items()}
SAFETY_KINDS = ("max_speed",)
MAX_DECIMAL_CHARS = 64

_READ_CODES = {"encoding": "I101", "json": "I102", "duplicate_key": "I103", "number": "I104", "depth": "I105"}
_NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*")
_DECIMAL = re.compile(r"[0-9]+(\.[0-9]+)?")
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
}
_COMPOUND = {"and": 1, "or": 1, "not": 1}
_STEPS = {
    "move": ("robot", "target"),
    "grasp": ("robot", "object"),
    "place": ("robot", "object", "target"),
    "inspect": ("robot", "object", "sensor"),
    "communicate": ("robot", "channel", "message"),
}
# kind -> (field, parameter type it must name): the uses E303 and E304 check
_USES = {
    "sensor_ok": (),
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
    "communicate": (("robot", "robot"),),
}
_PARAM_TYPES = {"robot": 0, "object": 0, "location": 0, "region": 0, "sensor": 2}
_BRANCH_STEPS = {"if": 3, "fail": 3}  # kind -> first schema rank that has it


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
        if kind in _ATOMS:
            since, keys = _ATOMS[kind]
            if not self._needs(f"condition kind '{kind}'", since, _pointer(ptr, "kind")):
                return False
            return self.strings(node, ptr, ("kind", *keys), keys)
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

    def strings(self, node, ptr: str, required, string_keys) -> bool:
        ok = self.shape(node, ptr, required)
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
        if kind in _STEPS:
            keys = _STEPS[kind]
            return self.strings(node, ptr, ("kind", *keys), keys)
        if kind == "wait_for":
            ok = self.shape(node, ptr, ("kind", "duration"))
            if "duration" in node:
                ok = self.quantity(node["duration"], _pointer(ptr, "duration"), DURATION_UNITS, False) and ok
            return ok
        if kind == "wait_until":
            ok = self.shape(node, ptr, ("kind", "condition", "timeout"))
            if "condition" in node:
                ok = self.condition(node["condition"], _pointer(ptr, "condition")) and ok
            if "timeout" in node:
                ok = self.quantity(node["timeout"], _pointer(ptr, "timeout"), DURATION_UNITS, False) and ok
            return ok
        self.error("I114", f"unknown step kind '{kind}'", _pointer(ptr, "kind"))
        return False

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
            ok = self.shape(limit, at, ("kind", "limit")) and ok
            if not isinstance(limit, dict):
                continue
            if "kind" in limit and self.string(limit["kind"], _pointer(at, "kind")):
                if limit["kind"] not in SAFETY_KINDS:
                    self.error("I114", f"unknown safety kind '{limit['kind']}'", _pointer(at, "kind"))
                    ok = False
            elif "kind" in limit:
                ok = False
            if "limit" in limit:
                ok = self.quantity(limit["limit"], _pointer(at, "limit"), SPEED_UNITS, True) and ok
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
        self.condition_list(task["preconditions"], _pointer(ptr, "preconditions"))
        if self._rank >= 2:
            self.condition_list(task.get("goal", []), _pointer(ptr, "goal"))
        self.steps_meaning(task["steps"], _pointer(ptr, "steps"), branch=False)
        if self._rank >= 3 and not task["goal"]:
            label = goalless_path(task["steps"], False, _parts)
            if label is not None:
                where = f"path {label}" if label else "the plan"
                self.error("I209", f"no goal on {where}: the task 'goal' is empty and a path that does not end in "
                                   "'fail' has no branch goal", _pointer(ptr, "goal"))

    def steps_meaning(self, steps: list, ptr: str, branch: bool):
        """`branch`: the steps are an `if` or `else` body, where a `fail` may be the last step (I214)."""
        for i, step in enumerate(steps):
            at = _pointer(ptr, i)
            if step["kind"] == "fail" and not (branch and i == len(steps) - 1):
                self.error("I214", "'fail' is allowed only as the last step of an if or else branch", at)
            self.step_meaning(step, at)

    def limits(self, limits: list, ptr: str):
        earlier: dict[str | None, int] = {}
        for i, entry in enumerate(limits):
            at = _pointer(ptr, i)
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

    def condition_meaning(self, node: dict, ptr: str):
        kind = node["kind"]
        if kind in ("and", "or"):
            self.condition_list(node["conditions"], _pointer(ptr, "conditions"))
        elif kind == "not":
            self.condition_meaning(node["condition"], _pointer(ptr, "condition"))
        else:
            self.uses(node, ptr)
            if kind == "sensor_ok":
                self.sensor(node["sensor"], _pointer(ptr, "sensor"), "sensor_ok")

    def step_meaning(self, node: dict, ptr: str):
        kind = node["kind"]
        if kind == "if":
            self.condition_meaning(node["condition"], _pointer(ptr, "condition"))
            for side, key in (("then", "then"), ("else", "else")):
                if node[f"goal_{side}"] is not None:
                    self.condition_list(node[f"goal_{side}"], _pointer(ptr, f"goal_{side}"))
                self.steps_meaning(node[key], _pointer(ptr, key), branch=True)
        elif kind == "fail":
            self.not_empty(node["reason"], "reason", _pointer(ptr, "reason"))
        elif kind == "wait_for":
            self.positive(node["duration"]["value"], "duration", _pointer(_pointer(ptr, "duration"), "value"))
        elif kind == "wait_until":
            self.condition_meaning(node["condition"], _pointer(ptr, "condition"))
            self.positive(node["timeout"]["value"], "timeout", _pointer(_pointer(ptr, "timeout"), "value"))
        else:
            self.uses(node, ptr)
            if kind == "inspect":
                self.sensor(node["sensor"], _pointer(ptr, "sensor"), "inspect")
            elif kind == "communicate":
                for key, what in (("channel", "channel name"), ("message", "message")):
                    self.not_empty(node[key], what, _pointer(ptr, key))

    def uses(self, node: dict, ptr: str):
        """E303 and E304: every parameter a condition or step names is declared, with the type it needs."""
        for key, expected in _USES[node["kind"]]:
            name, at = node[key], _pointer(ptr, key)
            declared = self._params.get(name)
            if declared is None:
                self.error("I203", f"undeclared name '{name}': it is not a parameter of this task", at)
            elif declared != expected:
                self.error("I204", f"'{name}' is {_article(declared)}, but '{node['kind']}' expects "
                                   f"{_article(expected)} here", at)

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

    # -------- document --------

    def document(self, document) -> bool:
        """Check the claims of the document header. False when the schema cannot be determined."""
        if not self.shape(document, "", ("ir_schema", "ir_schema_version", "language_version", "tasks")):
            if not isinstance(document, dict) or "ir_schema_version" not in document:
                return False
        if document.get("ir_schema") != ir.IR_SCHEMA:
            self.error("I113", f"not a Task IR document: 'ir_schema' must be '{ir.IR_SCHEMA}'", "/ir_schema")
        version = document.get("ir_schema_version")
        if not isinstance(version, str) or version not in SCHEMA_VERSIONS:
            known = ", ".join(f"'{v}'" for v in SCHEMA_VERSIONS)
            self.error("I113", f"unsupported ir_schema_version; this reader knows {known}", "/ir_schema_version")
            return False
        self._schema, self._rank = version, SCHEMA_VERSIONS.index(version)
        language = LANGUAGE_OF[version]
        self._reserved = RESERVED_BY_VERSION.get(language, RESERVED)
        if document.get("language_version") != language and "language_version" in document:
            self.error("I113", f"language_version must be '{language}' for ir_schema_version '{version}'",
                       "/language_version")
        return True


def _quantity(node: dict) -> ir.Quantity:
    return ir.Quantity(Decimal(node["value"]), node["unit"])


def _condition(node: dict) -> ir.Condition:
    kind = node["kind"]
    if kind in ("and", "or"):
        return (ir.And if kind == "and" else ir.Or)(tuple(_condition(c) for c in node["conditions"]))
    if kind == "not":
        return ir.Not(_condition(node["condition"]))
    return _of(next(c for c in _CONDITIONS if c.KIND == kind), node)


def _of(cls, node: dict):
    """A dataclass with only string fields, read from the JSON keys the schema gives them."""
    return cls(**{f.name: node[f.metadata.get("key", f.name)] for f in fields(cls)})


_CONDITIONS = (ir.RobotReady, ir.RegionClear, ir.ObjectExists, ir.SensorOk, ir.ObjectAt, ir.RobotAt, ir.Holding)
_STEP_TYPES = {cls.KIND: cls for cls in (ir.Move, ir.Grasp, ir.Place, ir.Inspect, ir.Communicate)}


def _goal(node: list | None) -> tuple[ir.Condition, ...] | None:
    return None if node is None else tuple(_condition(c) for c in node)


def _parts(step: dict):
    """The shape `paths.goalless_path` reads, from a step object."""
    if step["kind"] == "fail":
        return "fail"
    if step["kind"] != "if":
        return None
    return step["then"], step["goal_then"] is not None, step["else"], step["goal_else"] is not None


def _step(node: dict) -> ir.Step:
    kind = node["kind"]
    if kind == "if":
        return ir.If(_condition(node["condition"]), tuple(_step(s) for s in node["then"]),
                     tuple(_step(s) for s in node["else"]), _goal(node["goal_then"]), _goal(node["goal_else"]))
    if kind == "fail":
        return ir.Fail(node["reason"])
    if kind == "wait_for":
        return ir.WaitFor(_quantity(node["duration"]))
    if kind == "wait_until":
        return ir.WaitUntil(_condition(node["condition"]), _quantity(node["timeout"]))
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
        safety=tuple(ir.SafetyLimit(s["kind"], _quantity(s["limit"])) for s in node["safety"]),
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
    except _TooMany:
        pass
    if reader.errors:
        raise IRError(reader.errors)
    assert clean
    schema = document["ir_schema_version"]
    rank = SCHEMA_VERSIONS.index(schema)
    return ir.TaskIR(tuple(_task(t, rank) for t in document["tasks"]), LANGUAGE_OF[schema], ir.IR_SCHEMA, schema)


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
