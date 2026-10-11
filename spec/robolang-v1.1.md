# RoboLang v1.1 Language Specification

**Status:** Draft for owner review (RL-11A). **Nothing in this document is implemented.**
The repository still runs the v1 line-based interpreter described in the README.
**Syntax version:** 1.1
**Grammar:** [`grammar.ebnf`](grammar.ebnf)
**Fixtures:** [`tests/fixtures/`](../tests/fixtures/README.md)
**Decision record:** [`decisions/0001-rl-11a-language-decisions.md`](decisions/0001-rl-11a-language-decisions.md)

This specification defines what a v1.1 `.rob` file *is*: its tokens, its grammar, its
abstract syntax tree and the errors a conforming front end must report. It does not define
how a task is executed. Execution semantics belong to v1.2 (see [Deferred semantics](#9-deferred-semantics)).

> RoboLang v1.1 is a task description language. It is not a safety system. A file that
> validates is well-formed; it is not thereby safe to run. Physical robot execution is not
> validated and is outside the approved scope of this project.

---

## 1. Core invariant

**The complete source file must be lexed, parsed and statically validated before any adapter
action can occur.**

- A file with any error (lexical, syntactic or static) produces diagnostics and **zero**
  adapter calls. This holds wherever the error is: an unknown statement on the last line
  blocks the first statement too.
- This applies to the whole file, not only to the task selected for execution. An error in
  one task blocks every task in that file.
- There is no "skip and continue". The v1 behavior of printing `(ignored line)` and carrying
  on is removed.
- Warnings do not block.

## 2. Source text

- Encoding is UTF-8. A leading byte order mark is ignored.
- Line terminators are LF or CRLF. Lines and columns in diagnostics are 1-based; a column
  counts Unicode code points.
- The language is case-sensitive. `Move` is not `move`.
- Whitespace and line breaks only separate tokens. A statement may span lines, and several
  statements may share a line.
- `//` starts a comment that runs to the end of the line. There are no block comments.
- Characters outside the token set (for example `#`, `-`, `/*`) are errors, except inside
  strings and comments.

## 3. Tokens

| Token | Form | Notes |
|---|---|---|
| `IDENT` | letter, then letters, digits or `_` | ASCII only. Must start with a letter. Must not be a reserved word. |
| `NUMBER` | `digits` or `digits.digits` | Unsigned decimal. No sign, exponent, leading or trailing `.`. |
| `STRING` | `"…"` | No escape sequences. May not contain `"` or a line break. |
| `SPEED_UNIT` | `m/s`, `rad/s` | |
| `DURATION_UNIT` | `ms`, `s`, `min` | |
| punctuation | `( ) { } , ;` and `@safety` | |

Whitespace between a number and its unit is optional: `5s` and `5 s` are equivalent, as are
`0.5m/s` and `0.5 m/s`.

**Reserved words** cannot be used as names:

```text
task pre plan robot object location region
move grasp place inspect wait communicate
to at using with for until timeout
robot_ready region_clear object_exists sensor_ok
```

These are also reserved, for constructs that the v1 documents describe but v1.1 does not
include (section 8):

```text
with_approach in_region no_go_region require_guard message_received scalar duration
```

`max_speed` and the unit words `ms`, `s`, `min` are **not** reserved; they are recognized only
in their grammatical position.

## 4. Grammar

The normative grammar is [`grammar.ebnf`](grammar.ebnf). In summary:

```text
program     = task+
task        = "task" NAME "(" params? ")" "{" annotation* pre? plan "}"
params      = type NAME ("," type NAME)*          type = robot | object | location | region
annotation  = "@safety" "max_speed" NUMBER SPEED_UNIT? ";"
pre         = "pre" "{" (condition ";")* "}"
plan        = "plan" "{" (statement ";")+ "}"
```

### 4.1 Task structure

- A file contains one or more tasks. A file with no task is an error.
- Inside a task the order is fixed: `@safety` annotations, then an optional `pre` block, then
  exactly one `plan` block. Nothing follows `plan`.
- `pre` may be omitted or empty. `plan` must contain at least one statement.
- A task may have zero parameters.
- Blocks do not nest.

### 4.2 Parameter types

`robot`, `object`, `location`, `region`. A parameter is a symbolic name. How a name is bound
to a real robot, pose or zone is not part of the language (section 9).

### 4.3 Safety annotations

```rob
@safety max_speed 0.5 m/s;
@safety max_speed 1.5 rad/s;
```

- `max_speed` is the only key in v1.1. Any other key is an error.
- The unit is mandatory: `m/s` (linear) or `rad/s` (angular).
- **Deprecated form:** `@safety max_speed 0.5;` without a unit is accepted with warning
  `W401`, so that existing v1 files stay valid. Its unit is recorded as *unspecified*. A front
  end must not assume or convert a unit for it, and no execution path may treat an
  unspecified-unit limit as an enforced physical limit.
- The value must be greater than zero, and a key may appear once per task.
- In v1.1 an annotation is a declaration with a static contract. Nothing enforces it at run
  time until v1.2.

### 4.4 Conditions

| Condition | Argument | Kind |
|---|---|---|
| `robot_ready r` | name | `robot` |
| `region_clear cell` | name | `region` |
| `object_exists box` | name | `object` |
| `sensor_ok "camera_top"` | string | sensor name |

Conditions appear in `pre` blocks and in `wait until`. In v1.1 they are parsed and statically
checked only; their evaluation is defined in v1.2.

### 4.5 Statements

| Statement | Form | Argument kinds |
|---|---|---|
| move | `move r to dst;` | robot, location |
| grasp | `grasp r box;` | robot, object |
| place | `place r box at dst;` | robot, object, location |
| inspect | `inspect r box using "camera_top";` | robot, object, sensor string |
| wait (time) | `wait for 2.5s;` | duration |
| wait (condition) | `wait until region_clear cell timeout 30s;` | condition, duration |
| communicate | `communicate r to "fleet" with "TASK_COMPLETE";` | robot, channel string, message string |

- Every statement ends with `;`.
- `inspect` must name its sensor. No default sensor is ever selected.
- `wait until` must carry an explicit `timeout`. What happens when the timeout expires is a
  v1.2 question; the syntax only guarantees that a bound is always written down.
- A duration is a number followed by `ms`, `s` or `min`, and must be greater than zero.
  A number without a unit is an error. Decimal values are allowed (`2.5s`).
- Statement keywords match whole words. `movement r to src;` is an unknown statement, not a
  `move`.
- Sensor, channel and message strings must not be empty.

## 5. Static rules

Static validation runs after a successful parse and before anything executes. It is specified
here and implemented in RL-11C.

1. Task names are unique within a file.
2. Parameter names are unique within a task.
3. Every name used in `pre` or `plan` is a parameter of the enclosing task. Names are local to
   their task; there are no globals and no shadowing.
4. Every name is used where its declared type is expected (tables in 4.4 and 4.5).
5. `max_speed` and every duration are greater than zero.
6. A `@safety` key appears at most once per task.
7. Strings naming a sensor, channel or message are not empty.

An unused parameter is not an error.

## 6. Diagnostics

Format:

```text
<file>:<line>:<column>: error[E203]: unknown statement 'fly'
<file>:<line>:<column>: warning[W401]: max_speed has no unit; write 'm/s' or 'rad/s'
```

Every diagnostic carries a stable code, the position of the offending token and a reason.
Where a fixed set was expected, the message lists it. A front end may stop at the first error.

| Code | Stage | Meaning |
|---|---|---|
| `E101` | lexical | Unexpected character |
| `E102` | lexical | Unterminated string |
| `E103` | lexical | Malformed number |
| `E201` | syntax | Unexpected token (general case; message states what was expected) |
| `E202` | syntax | Missing `;` (reported at the end of the unterminated construct) |
| `E203` | syntax | Unknown statement |
| `E204` | syntax | Unknown condition |
| `E205` | syntax | Unknown parameter type |
| `E206` | syntax | Missing or unknown unit |
| `E207` | syntax | Unknown `@safety` key |
| `E208` | syntax | Invalid task structure: missing, empty, duplicated or misordered block |
| `E209` | syntax | Missing required clause: `using` on `inspect`, `timeout` on `wait until` |
| `E210` | syntax | No task in file |
| `E211` | syntax | Construct documented for v1 but not part of v1.1 (section 8) |
| `E212` | syntax | Reserved word used as a name |
| `E301` | static | Duplicate task name |
| `E302` | static | Duplicate parameter name |
| `E303` | static | Undeclared name |
| `E304` | static | Name used with the wrong type |
| `E305` | static | Value out of range |
| `E306` | static | Duplicate `@safety` key |
| `E307` | static | Empty string |
| `W401` | warning | `max_speed` without a unit (deprecated) |

When several codes could apply, the more specific wins: `E211` over `E203`/`E204`/`E205`/`E207`,
and `E202`, `E206`, `E209` over `E201`.

Exit status (for the RL-11D command line): zero when there are no errors, non-zero otherwise.
Warnings alone do not change the exit status.

## 7. Abstract syntax tree

Nodes are immutable (frozen dataclasses). Every node carries a `Span`. The AST is a faithful
record of the source: names are unresolved, units are kept as written, and nothing is
normalized. Binding and type checks (section 5) operate on this tree in RL-11C; conversion to
a versioned IR happens in RL-11D.

```python
@dataclass(frozen=True)
class Span:
    line: int          # 1-based, start
    column: int        # 1-based, start
    end_line: int
    end_column: int    # exclusive

class ParamType(Enum):     ROBOT, OBJECT, LOCATION, REGION
class SpeedUnit(Enum):     M_PER_S = "m/s"; RAD_PER_S = "rad/s"
class DurationUnit(Enum):  MS = "ms"; S = "s"; MIN = "min"

Name(text: str, span)                       # a use or declaration of an identifier
StringLit(value: str, span)                 # value excludes the quotes
Duration(value: Decimal, unit: DurationUnit, span)
Speed(value: Decimal, unit: SpeedUnit | None, span)    # None = deprecated unitless form

Program(tasks: tuple[Task, ...], span)
Task(name: Name, params: tuple[Param, ...], annotations: tuple[SafetyAnnotation, ...],
     pre: PreBlock | None, plan: PlanBlock, span)
Param(type: ParamType, name: Name, span)
SafetyAnnotation(key: Name, value: Speed, span)        # key.text == "max_speed" in v1.1
PreBlock(conditions: tuple[Condition, ...], span)
PlanBlock(statements: tuple[Statement, ...], span)

# Condition =
RobotReady(robot: Name, span)
RegionClear(region: Name, span)
ObjectExists(obj: Name, span)
SensorOk(sensor: StringLit, span)

# Statement =
Move(robot: Name, target: Name, span)
Grasp(robot: Name, obj: Name, span)
Place(robot: Name, obj: Name, target: Name, span)
Inspect(robot: Name, obj: Name, sensor: StringLit, span)
WaitFor(duration: Duration, span)
WaitUntil(condition: Condition, timeout: Duration, span)
Communicate(robot: Name, channel: StringLit, message: StringLit, span)
```

Notes:

- Numeric values are `Decimal`, parsed from the source digits, so `2.5s` is exactly 2.5.
- `pre: None` (block absent) and an empty `PreBlock` are distinct in the AST and equivalent in
  meaning.
- Comments and whitespace are not represented.
- A statement's span runs from its keyword through its `;`.

## 8. Backward compatibility

### 8.1 Existing sources

| Source | v1.1 result |
|---|---|
| `examples/pick_and_place.rob` | Valid, with `W401` (unitless `max_speed`) |
| `stdlib/robolang_std_v1.rob` | Valid, with `W401` |
| `examples/handover.rob`, `examples/inspect_object.rob` | Empty files: `E210`. They are placeholders, not programs. |
| README and `docs/index.md` example | Valid, with `W401` |
| Manual ch. 3 `pick_and_place` | Valid |
| Manual ch. 9 `handover` | Valid |
| Library guide `pick_and_place` | Valid |
| Developer guide `store_box` | Valid |
| Manual ch. 1 `store_red_box` | Migration required: `with_approach`, `in_region` |
| Manual ch. 5 `inspect` | Migration required: task is named `inspect` (reserved), uses `in_region` |
| Manual ch. 9 `receive_box` | Migration required: `message_received`, no `timeout`, `box` undeclared, `region_clear` applied to a location |
| Library guide `inspect_object` | Migration required: `scalar` parameter, `wait for dwell_s` |

"Manual", "Library guide" and "Developer guide" are the PDFs in `docs/`.

### 8.2 Documented v1 constructs not in v1.1

These appear in the v1 PDFs. The v1 runtime never implemented any of them (it ignores
everything after the statement keyword). v1.1 rejects each with `E211` instead of guessing a
meaning. Their words stay reserved so they can be added later without breaking files.

| Construct | Where documented | Why deferred |
|---|---|---|
| `move … with_approach 0.1` | Manual ch. 1, 6 | Unitless distance; meaning undefined |
| `move … in_region cell` | Manual ch. 1, 5, 6 | Runtime constraint; needs v1.2 semantics |
| `@safety no_go_region zone;` | Manual ch. 7, 11 | Needs v1.2 enforcement model |
| `@safety require_guard "…";` | Manual ch. 7 | Needs v1.2 enforcement model |
| `@safety` between statements | Manual ch. 7 | Scope of an action-level annotation undefined |
| `message_received "…"` | Manual ch. 9 | Depends on the v1.3 message protocol |
| `scalar` / `duration` parameters, `wait for <name>` | Manual ch. 5, Library guide | Needs value parameters and their binding |

### 8.3 Behavior that changes relative to the v1 runtime

Files that the v1 interpreter accepted and v1.1 rejects:

| v1 runtime | v1.1 |
|---|---|
| Unknown statement printed as ignored; execution continues | `E203`; nothing executes |
| Statements matched by prefix (`movement` runs as `move`) | `E203` |
| `;` optional | `E202` |
| `wait for 5` treated as 5 seconds | `E206` |
| `wait for 2.5s` truncated to 2 seconds | Parsed as exactly 2.5 seconds |
| `inspect r box;` uses sensor `camera_top` | `E209` |
| `wait until …` silently ignored | Parsed; requires `timeout` |
| `plan` ends at the first `}`; later statements dropped | Blocks are parsed; a nested `{` is `E201` |
| Everything outside `plan` ignored (types, `pre`, `@safety`, typos) | All of it is parsed and checked |
| Every `plan` block in a file executes | Execution requires explicit task selection (section 9) |

### 8.4 Deprecation policy

- A deprecated form stays valid and produces a `W4xx` warning.
- A deprecated form is not removed within the 1.x series. Removal needs an owner decision and
  migration notes.
- `W401` (unitless `max_speed`) is the only deprecation in v1.1.

## 9. Deferred semantics

Defined here as syntax and static contract only:

| Topic | Decided in |
|---|---|
| Evaluating `pre` conditions; freshness and failure behavior | v1.2 (RL-12B) |
| Enforcing `max_speed`; meaning for a robot with both linear and angular limits | v1.2 (RL-12B) |
| `wait until`: polling, cancellation, what happens on timeout | v1.2 (RL-12A/B) |
| Resolving a `location` to a motion target | v1.2 (RL-12C) |
| Binding task parameters to real robots, objects, poses and regions | RL-11C/RL-11D |
| Sensor and channel names: which exist, capability checks | v1.2 / v1.3 |
| Structured messages instead of free-text strings | v1.3 (RL-13C) |
| Versioned IR, `validate` / `inspect` commands | RL-11D |

**Task selection.** A file may hold several tasks. Execution must name the task to run. When a
file holds exactly one task, that task may be selected implicitly, which keeps the current
demo command working. Running "all tasks in the file" is never a default. The selection
mechanism is part of RL-11D.

## 10. Open questions for the owner

1. **`inspect` keyword.** This draft uses `using "sensor"`, the form in the v1 manual and
   library guide. The owner decision of 2026-10-12 was written with `with "sensor"`, based on
   a proposal made before the PDFs were checked. `using` keeps the published examples valid;
   `with` would break them. Confirm `using`, or choose `with` (a one-word change here).
2. **`object_exists`.** Added beyond the approved condition list because most documented
   `pre` blocks use it. Keep or remove?
3. **Deferred v1 constructs (8.2).** All seven are rejected in v1.1. Should any of them
   (`in_region` is the simplest) be brought into v1.1 instead?
4. **Reserved statement words as task names.** `task inspect(…)` from the manual is invalid
   because `inspect` is reserved. Acceptable, or should keywords be contextual?
5. **Version marker.** Files carry no syntax version; a v1.1 front end treats every file as
   v1.1. Should a marker (for example a first-line `robolang 1.1;`) be introduced now or with
   the IR in RL-11D?
6. **String escapes.** None in v1.1, so a message cannot contain `"`. Enough for now?
7. **Empty `plan`.** Rejected (`E208`). Empty `pre` is allowed. Agreed?
8. **`max_speed` kind.** One `max_speed` per task means a task declares either a linear or an
   angular limit, not both. Should both be expressible (for example separate keys)?
