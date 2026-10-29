# RoboLang v1.2 Language Specification — LANG-1 combined conditions

**Status:** LANG-1 specification, **approved by the owner on 2026-10-27**. Every choice is a row
in [decision record 0006](decisions/0006-lang-1-combined-conditions.md) (L1–L10).
**Work item:** LANG-1 of Track B (Foundation 1 Part B language extensions,
[RL-FOUNDATION-02 section 5.2](architecture/rl-foundation-02-domain-contracts.md#52-track-b--foundation-1-part-b-language-extensions-independent-owner-selected)).
Domain 09, Logic & Control Flow (contract 4.9).
**Implementation:** LANG-1b, in `src/robolang/` (section 12). Authorized by the owner on 2026-10-27.
**Syntax version:** 1.2
**Builds on:** [RoboLang 1.1](robolang-v1.1.md), unchanged; [Task IR 1.0](ir-schema-v1.0.md),
unchanged; [simulation 1.0](simulation-v1.0.md), unchanged
**Grammar:** [`grammar-v1.2.ebnf`](grammar-v1.2.ebnf)
**IR:** [Task IR schema 1.1](ir-schema-v1.1.md)
**Fixtures:** [`tests/fixtures/v1.2/`](../tests/fixtures/v1.2/README.md)

RoboLang 1.2 is RoboLang 1.1 plus one construct: conditions combined with `and`, `or` and
`not`, evaluated in three-valued logic. This document states only what differs from 1.1.
Everything it does not mention is exactly as in `robolang-v1.1.md`, including the core
invariant (1.1 section 1) and the separation of valid syntax, valid task semantics and safe
physical execution.

---

## 1. Why this construct, and why first

Today a `pre` block is an implicit conjunction and `wait until` takes one condition. A task
cannot say "either region is clear", "the object is no longer present", or wait for two facts
at once. Domain contract 4.9 lists combining conditions as the first missing piece of Logic &
Control Flow, and decision G6 puts it first in Track B because the later items build on it:
branching (LANG-2) needs a condition with a defined unknown, and comparisons (LANG-4) are new
operands for the same operators.

The construct adds no statement, no action and no new fact about the world. It only combines
the four existing conditions, so it can be specified, compiled and simulated without any
change to the world file.

## 2. Version and compatibility

### 2.1 Version marker (L1)

```rob
robolang 1.2;
```

| File starts with | Read as | Combined conditions |
|---|---|---|
| no marker | 1.1, exactly as today | Not available |
| `robolang 1.1;` | 1.1, exactly as today | Not available |
| `robolang 1.2;` | 1.2 | Available |
| any other version | rejected with `E213`, as today | — |

The new syntax requires the marker. A file is never read as 1.2 by default, so every existing
file keeps its meaning, its diagnostics and its IR **byte for byte**: the 76 RoboLang 1.1
fixtures, the golden IR snapshots and the simulation run records do not change.

A RoboLang 1.1 front end rejects every 1.2 file with `E213` at the marker, before it reads
anything else (1.1 section 4.1). A 1.2 file is therefore never misread by an older front end.

### 2.2 Reserved words (L2)

In a 1.2 file, three more words are reserved:

```text
and or not
```

In a 1.1 file (marked or unmarked) they remain ordinary names. `task t(robot and) { … }` stays
valid 1.1. Moving such a file to 1.2 is a **breaking change for that file**: its author renames
the parameter and adds the marker. No file in this repository uses these words as names.

Reserved-word sets therefore depend on the version marker. The lexer already reads the marker
before the rest of the file (1.1 section 4.1, `E213`), so the set is known before any name is
lexed.

## 3. Syntax

### 3.1 Grammar (L3)

The normative grammar is [`grammar-v1.2.ebnf`](grammar-v1.2.ebnf). The change to 1.1 is one
rule: `condition` becomes

```text
condition   = operand ( ("and" operand)+ | ("or" operand)+ )?
operand     = "not" operand | "(" condition ")" | atom
atom        = robot_ready IDENT | region_clear IDENT | object_exists IDENT | sensor_ok STRING
```

- `and`, `or` and `not` are the only operators. There are no symbols such as `&&` or `!`.
- `not` applies to the single operand directly after it: `not robot_ready r and region_clear
  c` means `(not robot_ready r) and region_clear c`.
- **`and` and `or` are never mixed without parentheses.** Within one level, every binary
  operator is the same word. `a and b or c` is an error (`E214`); write `(a and b) or c` or
  `a and (b or c)`. There is no precedence between `and` and `or` to remember or to get wrong.
  `E214` is reported at the first operator word of a level that differs from that level's
  first one.
- Parentheses group. A parenthesized single condition is allowed: `(robot_ready r)`.
- An operator and its operands may span lines.

### 3.2 Where a combined condition may appear (L4)

Everywhere a condition may appear in 1.1, and nowhere else:

```rob
robolang 1.2;

task deliver(robot r, object box, location src, location dst, region left, region right) {
    pre {
        robot_ready r and object_exists box;
        region_clear left or region_clear right;
    }
    plan {
        move r to src;
        grasp r box;
        wait until (region_clear left or region_clear right) and sensor_ok "camera_top" timeout 30s;
        move r to dst;
        place r box at dst;
    }
}
```

- **`pre` block:** each entry is one condition, combined or not, ending with `;`. The entries
  are still an implicit conjunction, evaluated one by one and recorded one by one (section 6).
- **`wait until … timeout …`:** the condition is one condition, combined or not. `timeout`
  ends it.

### 3.3 Nesting bound (L5)

A condition has a **depth** of at most 8:

| Form | Depth |
|---|---|
| atom | 0 |
| `not x` | depth of `x` + 1 |
| `( x )` | depth of `x` + 1 |
| `x₁ and … and xₙ`, `x₁ or … or xₙ` | largest depth of its operands + 1 |

Redundant parentheses count. A `pre` entry or `wait until` condition deeper than 8 is `E215`,
reported at its first token.

| Condition | Depth |
|---|---|
| `robot_ready r and region_clear c and sensor_ok "s"` | 1 |
| `not (robot_ready r or not region_clear c)` | 4 |
| `((((not not not not robot_ready r))))` | 8, valid |
| `not not not not not not not robot_ready r and region_clear c` | 8, valid |
| `not not not not not not not not robot_ready r and region_clear c` | 9, `E215` |

**Source depth bounds IR depth.** Every IR `not` comes from a source `not`, and every IR `and`
or `or` from a source chain of the same operator; parentheses and flattening only remove
levels. The IR depth of a compiled condition (IR schema 1.1 section 4) is therefore never
greater than its source depth, so a compiler that enforces this bound never produces IR that a
conforming IR consumer must reject.

The bound keeps conditions reviewable and the parser's recursion bounded (AF3): each `not` and
each `(` already adds a level, so a parser can stop at the ninth open group.

## 4. Static rules

No new rule. Rules 3, 4 and 8 of 1.1 section 5 apply to every atom, at any depth: each name is
a parameter of the task and of the type the atom expects, and each sensor string is non-empty.
`E303`, `E304` and `E307` are reported at the offending atom.

Duplicate or contradictory operands (`robot_ready r and robot_ready r`,
`region_clear c and not region_clear c`) are not errors. They are well-formed; whether they
make sense is the author's concern.

## 5. Diagnostics

1.1 section 6 applies unchanged, with these additions:

| Code | Stage | Meaning |
|---|---|---|
| `E214` | syntax | `and` and `or` mixed at one level without parentheses |
| `E215` | syntax | Condition deeper than 8 (section 3.3) |

And these clarifications, all with existing codes:

| Input | Code |
|---|---|
| Missing operand: `not ;`, `robot_ready r and ;`, `( )` | `E201`, expected a condition |
| Unknown word as operand: `robot_ready r and flying r` | `E204` |
| `message_received` or another 8.2 construct as operand | `E211` |
| `(` without `)` | `E201`, expected `)` |
| `and`, `or` or `not` used as a name in a 1.2 file | `E212` |
| `robolang 1.3;` and every other unsupported version | `E213`; the message lists `1.1` and `1.2` |

(1.3 note: since [RoboLang 1.3](robolang-v1.3.md), `robolang 1.3;` is supported and the message lists `1.1, 1.2 and 1.3`; `1.2` files are unaffected.)

When several codes could apply, the 1.1 order holds, and `E214` and `E215` take precedence
over `E201`.

## 6. Abstract syntax tree

Three node types are added to the `Condition` union of 1.1 section 7:

```python
And(operands: tuple[Condition, ...], span)   # two or more, in source order
Or(operands: tuple[Condition, ...], span)    # two or more, in source order
Not(operand: Condition, span)
```

- The AST stays a faithful record of the source: `(a and b) and c` is `And(And(a, b), c)`.
  Flattening happens in the IR (IR schema 1.1 section 2).
- Parentheses are not a node. The span of `(a)` as an operand is the span of `a`.
- The span of `And` and `Or` runs from the first operand to the last; the span of `Not` from
  `not` to the end of its operand.
- `Program` records the language version: `VersionDecl.version` is `"1.1"` or `"1.2"`; an
  unmarked file has `version: None` and is 1.1, as today.

## 7. Task IR

A 1.2 source compiles to **Task IR schema 1.1**, which adds the condition kinds `and`, `or`
and `not`: see [`ir-schema-v1.1.md`](ir-schema-v1.1.md). The IR document records
`"language_version": "1.2"` and `"ir_schema_version": "1.1"`, whether or not the file uses a
combined condition.

A 1.1 source still compiles to Task IR 1.0, byte-identical to today (L6).

## 8. Simulation semantics

The [simulation specification](simulation-v1.0.md) applies to 1.2 sources unchanged, except
for what this section adds. The world file stays at schema 1.0.

### 8.1 Three-valued evaluation (L7)

Combined conditions use Kleene's strong three-valued logic. Atoms evaluate as in simulation
section 4.4.

| `not a` | |
|---|---|
| `a` = `true` | `false` |
| `a` = `false` | `true` |
| `a` = `unknown` | `unknown` |

| `a and b` | `b` = `true` | `b` = `false` | `b` = `unknown` |
|---|---|---|---|
| `a` = `true` | `true` | `false` | `unknown` |
| `a` = `false` | `false` | `false` | `false` |
| `a` = `unknown` | `unknown` | `false` | `unknown` |

| `a or b` | `b` = `true` | `b` = `false` | `b` = `unknown` |
|---|---|---|---|
| `a` = `true` | `true` | `true` | `true` |
| `a` = `false` | `true` | `false` | `unknown` |
| `a` = `unknown` | `true` | `unknown` | `unknown` |

For more than two operands: `and` is `false` if any operand is `false`, otherwise `unknown` if
any is `unknown`, otherwise `true`. `or` is `true` if any operand is `true`, otherwise
`unknown` if any is `unknown`, otherwise `false`.

Conditions have no effects, so the value does not depend on the order in which operands are
evaluated. An implementation may stop early or evaluate every operand; the result and the run
record are the same.

**What this means for safety.** Only `true` lets a run continue (simulation 4.4), and that is
unchanged. A combined condition is `true` only when its known operands alone make it true:
`region_clear left or region_clear right` with `left` unknown and `right` clear is `true`
because `right` is known to be clear, not because `left` is assumed to be. Negation does not
turn the unknown into the known: `not region_clear c` with `c` unknown is `unknown`, and fails
closed (MC3).

### 8.2 `pre` and `wait until` (L8)

- **`pre` block:** each entry is evaluated to one value and recorded as one entry. The run
  fails before the first step with `S101` if the first entry that is not `true` is `false`,
  and with `S102` if it is `unknown`, as in simulation 4.4.
- **`wait until c timeout d`:** exactly the algorithm of simulation 4.5, with `c` the combined
  condition. Timeout is `S241`. The last value of `c` is recorded.

No run failure code is added.

### 8.3 Run record (L9)

A run of a 1.2 source writes **run record schema 1.1**. It is run record 1.0 with one
difference: the IR forms it embeds (`task`, `preconditions[].condition`, `steps[].step`) are
Task IR 1.1, and `inputs.ir_schema_version` is `"1.1"`. Its `inputs.language_version` is
`"1.2"`. No field is added, removed or redefined.

- A combined `pre` entry is one entry of `preconditions`; its `condition` is the whole IR
  condition and its `result` is the combined value.
- A combined `wait until` records the combined value in `condition_result`.
- Per-operand values are not recorded. A run is a function of its inputs (simulation 4.7), so
  the value of every operand at the time of evaluation can be recomputed from the world file
  and the applied events.

A run of a 1.1 source still writes run record 1.0, byte-identical to today, with the same run
ID. The run-ID derivation (simulation section 6) is unchanged for both.

## 9. What LANG-1 does not do

- No comparisons of values, no quantities in conditions (LANG-4).
- No branching or repetition (LANG-2, LANG-3).
- No new atoms. `message_received` stays deferred (`E211`).
- No change to statements, `@safety`, parameters, the world file or the run-ID derivation.
- No change to RoboLang 1.1, Task IR 1.0, run record 1.0 or any existing fixture.
- No Core schema. A combined condition is Plan data (home *Plan*, contracts 2.1); mapping
  conditions onto Core goal criteria is F2-1.

## 10. Fixtures and acceptance

[`tests/fixtures/v1.2/`](../tests/fixtures/v1.2/README.md) holds:

| Set | Content | Count |
|---|---|---|
| `valid/` | 1.2 files a conforming front end must accept | 10 |
| `invalid/` | 1.2 files it must reject, with code, stage and line | 15 |
| `compat/` | 1.1 files whose result must not change | 3 |
| `simulation/` | Sources, worlds and cases for section 8, in the format of `tests/fixtures/simulation/` | 12 cases |

Acceptance tests for LANG-1b:

| # | Test | Expected |
|---|---|---|
| L-T1 | Every `valid/` fixture | Passes the complete front end with no diagnostics; IR golden snapshot with `language_version` `1.2` and `ir_schema_version` `1.1` |
| L-T2 | Every `invalid/` fixture | Rejected with the stated code and line; no IR |
| L-T3 | Every `compat/` fixture | The stated 1.1 result, identical before and after LANG-1b |
| L-T4 | IR canonical form | `flatten_grouping.rob`: its three conditions produce the same IR; operand order kept; `not` never removed |
| L-T5 | Every `simulation/` case | The stated exit status and run-record fields, as for the RL-BASIC-01 cases |
| L-T6 | Kleene logic | `l1_and_false_unknown` fails with `S101`, not `S102`; `l1_or_true_unknown` succeeds |
| L-T7 | Regression | The 76 RoboLang 1.1 fixtures, every golden IR snapshot, every RL-BASIC-01 simulation case and run-ID vector, and the full suite pass unchanged |

`tests/test_v1_2_fixtures.py` runs L-T1 to L-T6 against these fixtures, with golden IR snapshots
in `tests/snapshots/ir-v1.2/`; `tests/test_lang1.py` adds unit tests for each section. L-T7 is
the existing suite, unchanged.

## 11. Delivery

1. **LANG-1a — this specification.** Documentation and fixtures only. No change under `src/`.
2. **LANG-1b — implementation.** Lexer (version-dependent reserved words), parser (section 3,
   `E214`, `E215`), AST (section 6), validator (atoms at any depth), IR 1.1 (`compile_source`,
   flattening, golden snapshots), executor and run record 1.1 (section 8), the `validate`
   message naming the version read, README and tests L-T1 to L-T7. Starts only after LANG-1a
   is approved and implementation is separately authorized.

## 12. Implementation status

| Part | Status |
|---|---|
| Version marker and reserved words (section 2) | Implemented in `lexer.py`, `parser.py` (LANG-1b) |
| Syntax, `E214`, `E215`, AST (sections 3, 5, 6) | Implemented in `parser.py`, `nodes.py` |
| Static rules at any depth (section 4) | Implemented in `validator.py` |
| Task IR 1.1 (section 7) | Implemented in `ir.py` |
| Three-valued evaluation, run record 1.1 (section 8) | Implemented in `sim/values.py`, `sim/executor.py`, `sim/record.py` |
| Acceptance tests L-T1 to L-T7 (section 10) | Pass in CI |
