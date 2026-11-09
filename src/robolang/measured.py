"""
Measured values (RoboLang 1.6, spec/robolang-v1.6.md, decision 0036): kinds, units, exact comparison, and the static
rules for bindings. Nothing here executes anything.

Two parts, both used by the compiler's validator and by the IR reader so that the two cannot disagree:

- units and comparison: exact decimal conversion within a kind, and the three-valued comparison of two readings
  `v +- u` (V5, V6, V7);
- `Scopes`, the binding and kind rules (V1, V3, V4, V5, V6): a name is bound by `measure`, visible after it in its
  block and nested blocks, and every comparison and every sent value is checked against what is visible there.
  It reads steps and conditions through small adapters, as `paths.py` does, so it works on the AST (source
  positions) and on IR dictionaries (JSON pointers) alike.
"""

from __future__ import annotations
from dataclasses import dataclass
from decimal import Context, Decimal, localcontext
from typing import Callable, Iterable, NamedTuple

from .tokens import KIND_UNITS

KINDS = tuple(KIND_UNITS)
LABEL = "label"
ORDERING = ("<", "<=", ">", ">=")  # for quantities; labels take == and != (V6)

# unit -> (kind, factor to the kind's base unit, offset). Base units: degC, g, mm. K = degC + 273.15 exactly.
_UNITS = {
    "degC": ("temperature", Decimal(1), Decimal(0)),
    "K": ("temperature", Decimal(1), Decimal("-273.15")),
    "g": ("mass", Decimal(1), Decimal(0)),
    "kg": ("mass", Decimal(1000), Decimal(0)),
    "mm": ("distance", Decimal(1), Decimal(0)),
    "cm": ("distance", Decimal(10), Decimal(0)),
    "m": ("distance", Decimal(1000), Decimal(0)),
}
assert {u for units in KIND_UNITS.values() for u in units} == set(_UNITS)
_EXACT = Context(prec=200)  # world and source numbers have at most a few dozen digits: arithmetic here is exact


def unit_kind(unit: str) -> str | None:
    """The kind a unit belongs to, or None for a word that is not a unit."""
    return _UNITS[unit][0] if unit in _UNITS else None


def to_base(value: Decimal, unit: str) -> Decimal:
    """`value` in `unit` as an exact decimal in the kind's base unit (degC, g or mm)."""
    _, factor, offset = _UNITS[unit]
    with localcontext(_EXACT):
        return value * factor + offset


def spread_to_base(uncertainty: Decimal, unit: str) -> Decimal:
    """An uncertainty is a difference, so it scales but does not shift (1 K = 1 degC)."""
    with localcontext(_EXACT):
        return uncertainty * _UNITS[unit][1]


def convert(value: Decimal, unit: str, target: str) -> Decimal:
    """`value` in `unit` expressed in `target`, exactly. Both units must be of one kind."""
    _, factor, offset = _UNITS[target]
    with localcontext(_EXACT):
        return (to_base(value, unit) - offset) / factor


class Interval(NamedTuple):
    """The values a reading may have: `lo` to `hi`, both included. A literal is a point (lo == hi)."""

    lo: Decimal
    hi: Decimal


def reading_interval(value: Decimal, uncertainty: Decimal, unit: str) -> Interval:
    centre, spread = to_base(value, unit), spread_to_base(uncertainty, unit)
    with localcontext(_EXACT):
        return Interval(centre - spread, centre + spread)


def compare_intervals(op: str, a: Interval, b: Interval) -> bool | None:
    """
    `a OP b` over every value in the two intervals (V7): True if every pair of values satisfies it, False if none
    does, None (unknown) otherwise. With uncertainty 0 on both sides this is the ordinary comparison.
    """
    if op == "<":
        return True if a.hi < b.lo else False if a.lo >= b.hi else None
    if op == "<=":
        return True if a.hi <= b.lo else False if a.lo > b.hi else None
    if op == ">":
        return True if a.lo > b.hi else False if a.hi <= b.lo else None
    if op == ">=":
        return True if a.lo >= b.hi else False if a.hi < b.lo else None
    raise ValueError(f"not an ordering operator: {op!r}")


# -------- binding and kind rules --------

@dataclass(frozen=True)
class Operand:
    """One side of a comparison, as the rules see it. `at` is whatever locates it in a diagnostic."""

    form: str  # "binding", "quantity", "text" or "live"
    at: object
    name: str | None = None  # binding: its name
    unit: str | None = None  # quantity: its unit
    kind: str | None = None  # live: the measured kind


@dataclass(frozen=True)
class Atom:
    """One comparison `left op right` found in a condition."""

    left: Operand
    op: str
    right: Operand
    at: object


class View(NamedTuple):
    """What the rules read from one step. Every body is a sequence of step handles the adapter understands."""

    shape: str = "step"  # "step", "if" or "fail"
    condition: object = None  # the condition an `if` or `wait until` evaluates, as a condition handle
    live_ok: bool = False  # a live reading is allowed in `condition` (a `wait until`)
    goals: tuple = ()  # the goal conditions of an `if`'s branches, as condition handles
    then: tuple = ()
    orelse: tuple = ()
    handler: tuple | None = None  # the failure handler's steps, or None
    measure: tuple | None = None  # (binding name, kind, where) of a `measure`
    send: Operand | None = None  # the binding a `communicate` sends


def article(param_type: str) -> str:
    return f"an {param_type}" if param_type == "object" else f"a {param_type}"


class Scopes:
    """
    The rules of decision 0036 V1, V3, V4, V5 and V6 over a task's plan.

    `report(rule, message, at)` receives each violation; `rule` is one of "scope", "clash", "kind", "operator",
    "goal" and "live" (E317 to E322, I218 to I223). `view(step)` gives a `View`; `atoms(condition)` the comparisons
    in a condition handle, in order.
    """

    def __init__(self, params: dict[str, str], view: Callable[[object], View],
                 atoms: Callable[[object], Iterable[Atom]], report: Callable[[str, str, object], None]):
        self._params = params
        self._view = view
        self._atoms = atoms
        self._report = report
        self._declared: dict[str, str] = {}  # every binding of the task -> its kind (the first declaration)

    # -- entry points --

    def forbid(self, conditions: Iterable[object], where: str):
        """V1: a `pre` or `goal` block states facts about the world, never a comparison of values."""
        for condition in conditions:
            for atom in self._atoms(condition):
                self._report("goal", f"a comparison of measured values cannot appear in {where}: goals and "
                                     "preconditions state facts about the world, not values (decision 0036 V1); "
                                     "compare in an 'if' or a 'wait until'", atom.at)

    def plan(self, steps: tuple):
        self._block(steps, {}, frozenset())

    # -- blocks --

    def _block(self, steps, scope: dict[str, str], retry: frozenset[str]) -> dict[str, str] | None:
        """
        Check `steps` with the bindings visible at their start. Returns the bindings visible at their end, or None
        when the block always ends in `fail` (it has no normal exit). A binding made in an `if` branch or a failure
        handler is not visible after it (V3), except as `_handled_measure` says.
        """
        scope = dict(scope)
        for step in steps:
            view = self._view(step)
            if view.shape == "fail":
                return None
            if view.condition is not None:
                self._condition(view.condition, scope, view.live_ok)
            if view.send is not None:
                self._use(view.send.name, view.send.at, scope, "a value to send")
            if view.shape == "if":
                self.forbid(view.goals, "a branch 'goal' block")
                self._block(view.then, scope, retry)
                self._block(view.orelse, scope, retry)
            elif view.measure is not None:
                self._handled_measure(view, scope, retry)
            elif view.handler is not None:
                self._block(view.handler, scope, retry)
        return scope

    def _handled_measure(self, view: View, scope: dict[str, str], retry: frozenset[str]):
        name, kind, at = view.measure
        self._declare(name, kind, at, retry)
        if view.handler is None:
            scope[name] = kind
            return
        # In its own handler the name is not visible, and a `measure` there may bind it again (a retry, V3 b).
        end = self._block(view.handler, scope, retry | {name})
        if end is None or end.get(name) == kind:  # every normal exit of the handler also binds it (or has none)
            scope[name] = kind

    def _declare(self, name: str, kind: str, at: object, retry: frozenset[str]):
        if name in self._params:
            self._report("clash", f"binding '{name}' has the name of {article(self._params[name])} parameter; a "
                                  "binding must not share a name with a parameter", at)
        elif name not in self._declared:
            self._declared[name] = kind
        elif name not in retry:
            self._report("clash", f"binding '{name}' is already bound in this task; a name is bound once, except "
                                  "that a 'measure' in the handler of a 'measure' may bind the same name again "
                                  "(a retry)", at)
        elif self._declared[name] != kind:
            self._report("kind", f"the retry binds '{name}' as {self._declared[name]} first and {kind} here; a "
                                 "retry reads the same kind", at)

    # -- uses --

    def _use(self, name: str, at: object, scope: dict[str, str], what: str) -> str | None:
        """The kind of the visible binding `name`, or None after reporting why it cannot be used."""
        if name in scope:
            return scope[name]
        if name in self._params:
            self._report("scope", f"'{name}' is {article(self._params[name])}, not a measured value: {what} must "
                                  "be a binding made by 'measure'", at)
        else:
            self._report("scope", f"'{name}' is not a visible binding here: a 'measure ... as {name}' must come "
                                  "before this point in this block or an enclosing one, and a binding made "
                                  "inside an 'if' branch or a failure handler is not visible after it", at)
        return None

    def _condition(self, condition: object, scope: dict[str, str], live_ok: bool):
        for atom in self._atoms(condition):
            self._atom(atom, scope, live_ok)

    def _atom(self, atom: Atom, scope: dict[str, str], live_ok: bool):
        left, right = atom.left, atom.right
        if left.form == "live":
            if not live_ok:
                self._report("live", f"a live reading ('{left.kind} of ... using ...') is allowed only in the "
                                     "condition of a 'wait until'; elsewhere 'measure' it and compare the binding",
                             left.at)
            kind = left.kind
        else:
            kind = self._use(left.name, left.at, scope, "the left side of a comparison")
        if right.form == "binding":
            other = self._use(right.name, right.at, scope, "the right side of a comparison")
        elif right.form == "text":
            other = LABEL
        else:
            other = unit_kind(right.unit)
        if kind is None or other is None:
            return
        if right.form == "quantity" and kind != LABEL and other != kind:
            self._report("kind", f"'{right.unit}' is a unit of {other}, but the left side is {kind} "
                                 f"(units of {kind}: {', '.join(KIND_UNITS[kind])})", right.at)
        elif other != kind:
            what = "a string" if right.form == "text" else "a quantity" if right.form == "quantity" \
                else f"'{right.name}' ({other})"
            self._report("kind", f"cannot compare {kind} with {what}: both sides must be the same kind "
                                 "(a label is compared with a string or another label)", right.at)
        elif kind == LABEL and atom.op in ORDERING:
            self._report("operator", f"a label is compared with '==' or '!=', not '{atom.op}': labels have no order",
                         atom.at)
        elif kind != LABEL and atom.op not in ORDERING:
            self._report("operator", f"{kind} is compared with '<', '<=', '>' or '>=', not '{atom.op}': a reading "
                                     "has an uncertainty, so equality is almost never decidable", atom.at)
