# RoboLang v1.1 Language Specification

**Status:** RL-11A specification, approved by the owner on 2026-10-12.
**Implementation:** the lexer, parser and AST (RL-11B) and the static rules of section 5
(RL-11C) are implemented in `src/robolang/`; see section 12. Validated tasks run only in the
symbolic simulation of [`simulation-v1.0.md`](simulation-v1.0.md) (`robolang simulate`);
physical execution is not implemented. The v1 line-based interpreter is kept as history
([`docs/legacy-v1.md`](../docs/legacy-v1.md)).
**Syntax version:** 1.1
**Grammar:** [`grammar.ebnf`](grammar.ebnf)
**Fixtures:** [`tests/fixtures/`](../tests/fixtures/README.md)
**Decision record:** [`decisions/0001-rl-11a-language-decisions.md`](decisions/0001-rl-11a-language-decisions.md)

This specification defines what a v1.1 `.rob` file *is*: its tokens, its grammar, its
abstract syntax tree and the errors a conforming front end must report. It does not define
how a task is executed. Execution semantics belong to v1.2 (see [Deferred semantics](#9-deferred-semantics)).

> **Three separate questions.** RoboLang keeps these apart, and an answer to one is never an
> answer to the next:
>
> 1. **Valid syntax** — the file matches the grammar (sections 2 to 4).
> 2. **Valid task semantics** — names, types and values are consistent (section 5).
> 3. **Safe physical execution** — a property of a specific robot, cell, controller and risk
>    assessment. No part of this language decides it.
>
> A program accepted by the parser and the static checker is well-formed. It must never be
> treated, automatically or by default, as safe to run on real hardware. Physical robot
> execution is not validated and is outside the approved scope of this project.

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
| `STRING` | `"…"` | May not contain a line break. Escapes: section 3.1. |
| `SPEED_UNIT` | `m/s`, `rad/s` | |
| `DURATION_UNIT` | `ms`, `s`, `min` | |
| punctuation | `( ) { } , ;` and `@safety` | |

Whitespace between a number and its unit is optional: `5s` and `5 s` are equivalent, as are
`0.5m/s` and `0.5 m/s`.

### 3.1 String escapes

Inside a string, `\` starts an escape sequence. Exactly four exist:

| Written | Character |
|---|---|
| `\"` | `"` |
| `\\` | `\` |
| `\n` | line feed |
| `\t` | horizontal tab |

Any other character after `\` is an error (`E104`); a backslash is never passed through
literally. A string cannot span lines: a line break before the closing `"` is `E102`, also
directly after a `\`. The value of a string is its content after escapes are decoded.

### 3.2 Reserved words

**Reserved words** cannot be used as names:

```text
robolang task pre plan robot object location region
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
program     = version? task+
version     = "robolang" "1.1" ";"
task        = "task" NAME "(" params? ")" "{" annotation* pre? plan "}"
params      = type NAME ("," type NAME)*          type = robot | object | location | region
annotation  = "@safety" "max_speed" NUMBER SPEED_UNIT? ";"
pre         = "pre" "{" (condition ";")* "}"
plan        = "plan" "{" (statement ";")+ "}"
```

### 4.1 Version marker

```rob
robolang 1.1;
```

- The marker is optional. A file without one is read as v1.1, so existing files stay valid.
- If present it must be the first construct in the file (comments may precede it) and may
  appear only once.
- The version is written exactly `1.1`. Any other version, older or newer, is rejected with
  `E213`, and the rest of the file is not interpreted: a front end must not guess at the
  syntax of a version it does not implement.

### 4.2 Task structure

- A file contains one or more tasks. A file with no task is an error.
- Inside a task the order is fixed: `@safety` annotations, then an optional `pre` block, then
  exactly one `plan` block. Nothing follows `plan`.
- `pre` may be omitted or empty. `plan` must contain at least one statement.
- A task may have zero parameters.
- Blocks do not nest.

### 4.3 Parameter types

`robot`, `object`, `location`, `region`. A parameter is a symbolic name. How a name is bound
to a real robot, pose or zone is not part of the language (section 9).

### 4.4 Safety annotations

```rob
@safety max_speed 0.5 m/s;
@safety max_speed 1.5 rad/s;
```

- `max_speed` is the only key in v1.1. Any other key is an error.
- The unit is mandatory: `m/s` (linear) or `rad/s` (angular).
- A task may declare both a linear and an angular limit, in either order:

  ```rob
  @safety max_speed 0.5 m/s;
  @safety max_speed 1.0 rad/s;
  ```

  Each unit type may appear **at most once** per task. A second limit of the same unit type
  is an error (`E306`).
- **Deprecated form:** `@safety max_speed 0.5;` without a unit is accepted with warning
  `W401`, so that existing v1 files stay valid.
  - Its unit is recorded as *unspecified*. A front end must not assume or convert a unit for
    it, and no execution path may treat an unspecified-unit limit as an enforced physical
    limit.
  - It is allowed only as the task's single `max_speed` annotation. Combining it with any
    explicit-unit limit is an error (`E308`); two unitless limits are a duplicate (`E306`).
- The value must be greater than zero.
- In v1.1 an annotation is a declaration with a static contract. Nothing enforces it at run
  time. Enforcement, and how declared limits relate to what a given robot can do, belong to
  v1.2.

### 4.5 Conditions

| Condition | Argument | Kind |
|---|---|---|
| `robot_ready r` | name | `robot` |
| `region_clear cell` | name | `region` |
| `object_exists box` | name | `object` |
| `sensor_ok "camera_top"` | string | sensor name |

Conditions appear in `pre` blocks and in `wait until`. In v1.1 they are parsed and statically
checked only; their evaluation is defined in v1.2.

### 4.6 Statements

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
- Strings may use the escapes of section 3.1, for example
  `communicate r to "fleet" with "said \"done\"";`.

## 5. Static rules

Static validation runs after a successful parse and before anything executes.

1. Task names are unique within a file.
2. Parameter names are unique within a task.
3. Every name used in `pre` or `plan` is a parameter of the enclosing task. Names are local to
   their task; there are no globals and no shadowing.
4. Every name is used where its declared type is expected (tables in 4.5 and 4.6).
5. `max_speed` and every duration are greater than zero.
6. A task has at most one `max_speed` limit per unit type (`m/s`, `rad/s`).
7. A unitless `max_speed` is the only `max_speed` annotation of its task.
8. Strings naming a sensor, channel or message are not empty after escapes are decoded.

For rules 6 and 7 the diagnostic is reported at the later of the conflicting annotations.

Static errors are reported in source order, by the position at which each is reported. A
front end reports every static error (section 6), at most one for each statement, parameter,
`@safety` annotation and condition atom: once one rule fails for an item, its other rules are
not checked, so one mistake is not reported twice. A name that is undeclared (`E303`) is never
also reported as having the wrong type (`E304`).

An unused parameter is not an error.

## 6. Diagnostics

Format:

```text
<file>:<line>:<column>: error[E203]: unknown statement 'fly'
<file>:<line>:<column>: warning[W401]: max_speed has no unit; write 'm/s' or 'rad/s'
```

Every diagnostic carries a stable code, the position of the offending token and a reason.
Where a fixed set was expected, the message lists it.

A front end reports every error it finds, in source order, not only the first
([decision 0015](decisions/0015-report-all-diagnostics.md)). To do so after a syntax error it
resumes after the next `;`, at the next `}`, or at the next `task` (a missing `;` skips nothing
when the next token already starts a statement or condition). A task with a syntax error is not
statically validated, so its syntax error is not followed by errors it caused; every other task
is. A lexical error, and an error in the version marker, stop the front end: they are reported
alone. At most 50 errors are reported, followed by a note that there are more. The first error
reported is the one a front end that stops at the first error would report, except that a static
error in an earlier task now comes before a syntax error in a later one.

Stages run in order: lexical, then syntax, then static. A lexical error anywhere in the file
is reported before any syntax error, even one that appears earlier in the text. The one
exception is the version marker: an unsupported version (`E213`) is reported without
examining anything after it (section 4.1).

| Code | Stage | Meaning |
|---|---|---|
| `E101` | lexical | Unexpected character |
| `E102` | lexical | Unterminated string |
| `E103` | lexical | Malformed number |
| `E104` | lexical | Unknown escape sequence in a string |
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
| `E213` | syntax | Unsupported language version in the version marker |
| `E301` | static | Duplicate task name |
| `E302` | static | Duplicate parameter name |
| `E303` | static | Undeclared name |
| `E304` | static | Name used with the wrong type |
| `E305` | static | Value out of range |
| `E306` | static | Duplicate `max_speed` limit for the same unit type (including two unitless limits) |
| `E307` | static | Empty string |
| `E308` | static | Unitless `max_speed` combined with an explicit-unit `max_speed` |
| `W401` | warning | `max_speed` without a unit (deprecated) |

When several codes could apply, the more specific wins: `E211` over `E203`/`E204`/`E205`/`E207`,
and `E202`, `E206`, `E209` over `E201`.

Exit status of the command line: zero when there are no errors, non-zero otherwise.
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
StringLit(value: str, span)                 # decoded value: no quotes, escapes resolved
VersionDecl(version: str, span)             # "1.1"
Duration(value: Decimal, unit: DurationUnit, span)
Speed(value: Decimal, unit: SpeedUnit | None, span)    # None = deprecated unitless form

Program(version: VersionDecl | None, tasks: tuple[Task, ...], span)   # None = unmarked file
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
- `Task.annotations` holds zero, one or two `SafetyAnnotation` nodes in source order.
- The span of a `StringLit` covers the source text including quotes and escapes; its `value`
  is the decoded text.
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
| README and `docs/index.md` example | Valid. Since RL-DOC-README-01 both write `max_speed 0.5 m/s`, so no warning; earlier versions gave `W401` |
| Manual ch. 3 `pick_and_place` | Valid |
| Manual ch. 9 `handover` | Valid |
| Library guide `pick_and_place` | Valid |
| Developer guide `store_box` | Valid |
| Manual ch. 1 `store_red_box` | Migration required: `with_approach`, `in_region` |
| Manual ch. 5 `inspect` | Migration required: task is named `inspect` (reserved), uses `in_region` |
| Manual ch. 9 `receive_box` | No equivalent in v1.1: `message_received` (also no `timeout`, `box` undeclared, `region_clear` applied to a location). Needs a redesign and manual review. |
| Library guide `inspect_object` | Migration required: `scalar` parameter, `wait for dwell_s` |

"Manual", "Library guide" and "Developer guide" are the PDFs in `docs/`. The four affected
examples are worked through in section 8.5.

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

### 8.5 Migrating the affected documented examples

Each task below is valid v1.1. Where a v1 construct has no v1.1 equivalent, the migration
**drops** it; it does not emulate it. What is lost is stated for each case. None of these is
an automatic rewrite: each needs its author's review, and `receive_box` in particular is a
redesign, not a migration.

**Manual ch. 1 — `store_red_box`.** Remove `with_approach` and `in_region`; give `max_speed`
a unit.

```rob
task store_red_box(robot r, object box, location conveyor, location shelf, region cell) {
    @safety max_speed 0.5 m/s;
    pre {
        object_exists box;
        robot_ready r;
    }
    plan {
        move r to conveyor;
        grasp r box;
        move r to shelf;
        place r box at shelf;
    }
}
```

Lost: the 0.1 approach offset and the restriction of the first move to `cell`. Neither was
ever applied by the v1 runtime. Whether 0.5 meant m/s is the author's call; the unit must be
chosen, not assumed.

**Manual ch. 5 — `inspect`.** Rename the task (`inspect` is reserved) and remove `in_region`.

```rob
task inspect_object(robot r, object obj, location cam_pos, region zone) {
    pre { object_exists obj; }
    plan {
        move r to cam_pos;
        inspect r obj using "camera_top";
    }
}
```

Lost: the restriction of the move to `zone`. The `zone` parameter is now unused, which is
allowed.

**Manual ch. 9 — `receive_box` — non-equivalent illustrative redesign; requires manual
workflow review.** `message_received` has no v1.1 equivalent, so this example **cannot be
migrated**. The task below is a different workflow that happens to be expressible in v1.1: it
follows the library guide, which shows the receiver waiting on a sensor signal. It is not an
automatic or behavior-preserving migration. The redesign also declares `box`, gives
`wait until` a timeout, and applies `region_clear` to a region rather than to the `handoff`
location.

```rob
task receive_box(robot mobile, object box, location handoff, region handoff_zone) {
    pre { region_clear handoff_zone; }
    plan {
        wait until sensor_ok "handoff_signal" timeout 60s;
        move mobile to handoff;
        grasp mobile box;
    }
}
```

Not equivalent: `sensor_ok "handoff_signal"` reports the health of a sensor. It does not mean
that a `HANDOFF_READY` message was received, and it must not be read as a handoff event.
Whether a sensor-based trigger is acceptable for a given cell is a workflow decision for a
person to make. Message-driven coordination remains deferred to v1.3. The 60s timeout is an
example value.

**Library guide — `inspect_object`.** Remove the `scalar dwell_s` parameter and write the
dwell time as a literal duration.

```rob
task inspect_object(robot r, object obj, location cam_pos, region cell) {
    plan {
        move r to cam_pos;
        inspect r obj using "camera_top";
        wait for 2s;
    }
}
```

Lost: passing the dwell time as an argument. The 2s value is an example.

The PDFs themselves are not changed by this specification and still show the v1 forms.

## 9. Deferred semantics

Defined here as syntax and static contract only:

| Topic | Decided in |
|---|---|
| Evaluating `pre` conditions; freshness and failure behavior | v1.2 (RL-12B) |
| Enforcing `max_speed`; how declared linear and angular limits relate to a robot's capabilities | v1.2 (RL-12B) |
| `wait until`: polling, cancellation, what happens on timeout | v1.2 (RL-12A/B) |
| Resolving a `location` to a motion target | v1.2 (RL-12C) |
| Binding task parameters to real robots, objects, poses and regions | Not defined yet; v1.2 (RL-12C) |
| Sensor and channel names: which exist, capability checks | v1.2 / v1.3 |
| Structured messages instead of free-text strings | v1.3 (RL-13C) |
| Versioned IR, `validate` / `inspect` commands | Done in RL-11D: [`ir-schema-v1.0.md`](ir-schema-v1.0.md) |

**Task selection.** A file may hold several tasks. Execution must name the task to run. When a
file holds exactly one task, that task may be selected implicitly, which keeps the current
demo command working. Running "all tasks in the file" is never a default. RL-11D
provides selection for inspection only (`inspect --task NAME`); selecting a task for execution
belongs to the executor, which does not exist yet.

## 10. Resolved questions

Decided by the owner on 2026-10-12; recorded in
[decision record 0001](decisions/0001-rl-11a-language-decisions.md).

| Question | Decision |
|---|---|
| `inspect` sensor clause keyword | `using`, as documented in the v1 manual. The sensor stays mandatory. |
| `object_exists` condition | Included. Runtime evaluation deferred. |
| Seven documented v1 constructs (8.2) | Stay deferred; each rejected with `E211`. |
| Statement keywords as task names | Keywords stay reserved. `task inspect(…)` is invalid. |
| Version marker | Optional `robolang 1.1;`. Unmarked files are v1.1. Other versions fail (`E213`). |
| String escapes | `\"`, `\\`, `\n`, `\t`. Unknown escapes are rejected (`E104`). |
| Empty `plan` | Rejected (`E208`). Empty `pre` is valid. |
| Linear and angular `max_speed` | Both allowed, at most once per unit type. Unitless form only as a single legacy annotation. |
| Position of `E306` / `E308` | The later annotation. A future improvement may also reference the original one. |
| `robolang 1.0;` | Rejected with `E213`. An explicitly versioned 1.0 file is never silently read as 1.1. |
| Escapes in sensor and channel names | Accepted lexically. Whether the resulting name is supported belongs to later capability and semantic validation. |
| PDFs in `docs/` | Left unchanged as historical v1 documentation. Aligning them is a separate task. |

## 11. Remaining questions

None.

## 12. Implementation status

| Part of this specification | Status |
|---|---|
| Lexer, parser and AST (sections 2 to 4 and 7; `E1xx`, `E2xx`, `W401`) | Implemented in `src/robolang/` (RL-11B) |
| Static rules (section 5; `E301`–`E308`) | Implemented in `src/robolang/validator.py` (RL-11C) |
| Versioned IR ([`ir-schema-v1.0.md`](ir-schema-v1.0.md)), `validate` / `inspect` commands, task selection for inspection | Implemented in `src/robolang/ir.py` and `cli.py` (RL-11D) |
| Installable package and `robolang` console command | Implemented in `pyproject.toml` (RL-11E); installed from a clone, not published |
| Execution of parsed tasks | Simulation only: `robolang simulate` in `src/robolang/sim/` (RL-BASIC-01, [`simulation-v1.0.md`](simulation-v1.0.md)). Physical execution is not implemented |

Conformance checked by CI (`tests/test_fixtures.py`): all 76 fixtures.

- The 15 valid fixtures must pass the complete front end with exactly the warnings they state.
- The 47 lexical and syntax fixtures must be rejected by the parser with the stated code and
  line.
- The 14 static fixtures must be syntactically valid and then rejected by static validation
  with the stated code and line.

Passing every fixture shows agreement with these 76 cases. It is not a proof that the
implementation matches the specification everywhere, and it says nothing about whether any
accepted task is safe to execute.

Follow-up outside this specification: the PDFs in `docs/` describe v1, including the deferred
constructs and unitless forms. They are kept as historical documentation; aligning them is a
separate task.
