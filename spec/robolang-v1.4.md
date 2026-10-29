# RoboLang 1.4 (delta over 1.3)

**Status:** complete in code (front end, simulation, `validate --world` paths, reference); the acceptance run is pending. Decision: [0027](decisions/0027-bounded-branching.md) (B1 to B9). Grammar:
[grammar-v1.4.ebnf](grammar-v1.4.ebnf). IR: [schema 1.3](ir-schema-v1.3.md).
Everything not stated here is exactly as in [RoboLang 1.3](robolang-v1.3.md) (and 1.2, 1.1 below it).

## 1. Version

`robolang 1.4;` selects 1.4. A file with this marker is read with the 1.3 rules, plus this document. A 1.3 front end
rejects it with `E213`. Files with no marker or with `1.1`, `1.2` or `1.3` are read, validated and compiled exactly as
before: same diagnostics, same IR, same run records.

**Simulation:** `simulate`, `export-goal` and `delegate` accept a branching task
([simulation 12](simulation-v1.0.md)). `validate --world` checks every path of it as a straight line
([simulation 11](simulation-v1.0.md)). A task of 1.4 without `if` or `fail` behaves as in 1.3.

## 2. Reserved words

`if`, `else` and `fail` are reserved in a 1.4 file (`E212` when used as a name). In 1.1 to 1.3 files they stay
ordinary names, so no earlier file changes meaning.

## 3. Statements

### 3.1 `if`

```
if <condition> {
    [goal { <condition>; ... }]
    <statement> ...
} [else {
    [goal { <condition>; ... }]
    <statement> ...
}]
```

- The condition is any condition of 1.3 (atoms, `and`, `or`, `not`, depth at most 8) and is evaluated once, when
  the `if` is reached.
- A branch has at least one statement (`E208`). `else` is optional.
- `else if C { ... }` is shorthand for `else { if C { ... } }`.
- Statements after an `if` follow it on every path that does not end in `fail`.
- An `if` cannot appear in `pre`, or in a `wait until` condition: those hold conditions, not statements.

### 3.2 Branch `goal`

A `goal { ... }` block may appear only as the first item of an `if` or `else` branch (`E208` anywhere else, also at
plan level). It adds goal conditions to the paths through that branch.

### 3.3 Nesting

`if` is nested at most 4 deep, counting `else if` (`E216`). Sequential `if` statements are not limited.

### 3.4 `fail`

`fail "reason";` ends the run as failed. It is allowed only as the last statement of an `if` or `else` branch;
anywhere else (not last, or at plan level) is `E313`. The reason is a non-empty string (`E307`).

## 4. Goals (B2)

A *path* is one choice of `then` or `else` at each `if` it reaches. The goal of a path is the task `goal` plus the
branch `goal` blocks the path passes through. In 1.4 the task `goal` block is optional, but **every path must have at
least one goal condition, or end in `fail`**. Otherwise `E309`, reported at the task name and naming one such path:
`path then`, `path else/then`, or `the plan` for a plan with no `if`. An `if` with no `else` has an `else` path
with no statements.

## 5. Diagnostics added or changed

| Code | Meaning |
|---|---|
| `E216` | `if` nested deeper than 4 levels (syntax) |
| `E309` | In 1.4: a path with no goal condition that does not end in `fail`; the message names the path (static) |
| `E313` | `fail` that is not the last statement of an `if` or `else` branch (static) |
| `E208` | Also: an empty branch, a misplaced `goal` block (syntax) |
| `E212` | Also: `if`, `else` or `fail` used as a name in a 1.4 file |

## 6. Compatibility

Nothing changes for 1.1, 1.2 and 1.3 files. `E213` now lists `1.1, 1.2, 1.3 and 1.4`.
