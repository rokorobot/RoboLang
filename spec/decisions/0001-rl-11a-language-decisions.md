# Decision record 0001 — RL-11A language decisions

**Date:** 2026-10-12
**Status:** Sections 1 and 2 decided by the owner. Section 3 is a proposal awaiting decision.
**Applies to:** [`spec/robolang-v1.1.md`](../robolang-v1.1.md), [`spec/grammar.ebnf`](../grammar.ebnf)

## 1. Owner decisions

| # | Topic | Decision |
|---|---|---|
| 1 | `max_speed` | Explicit unit required (`m/s` or `rad/s`). The existing unitless form stays accepted as deprecated, with a warning. No implicit unit conversion or assumption for physical execution. |
| 2 | `inspect` | The sensor must be named explicitly. No default sensor is selected. Sensor capability validation comes later. |
| 3 | `wait until` | In the v1.1 grammar, with a mandatory `timeout <duration>`. Execution, cancellation and timeout handling belong to v1.2. |
| 4 | Multiple tasks per file | Allowed. Execution requires explicit task selection; all tasks are never executed automatically. |
| 5 | Strict syntax | Missing semicolons, unitless durations, unknown statements, prefix matches and malformed constructs are rejected with useful diagnostics. |
| 6 | AST | Frozen dataclasses with source spans. Name binding and type validation are deferred to RL-11C. |
| 7 | Invariant | The complete program is parsed and validated before any adapter action can occur. |

## 2. Where the specification departs from a decision as written

**`inspect` keyword.** Decision 2 was written as `inspect r box with "camera_top";`. That form
came from a proposal made before the v1 PDFs were read. The manual and the library guide both
document `inspect r box using "camera_top";`. The draft specification uses **`using`** so the
published examples stay valid, and reports `with` in that position as `E209`. The substance of
the decision (explicit sensor, no default) is unchanged. This needs the owner's confirmation.

## 3. Proposal: parser tooling for RL-11B

The development plan (A6) allows a parser generator such as Lark "after a small spike".

| Option | For | Against |
|---|---|---|
| **Hand-written recursive descent** (recommended) | No runtime dependency. The grammar is LL(1): every statement starts with a distinct keyword. Full control over the specific diagnostics the spec requires (`E202`, `E206`, `E209`, `E211`) and over spans. | The grammar file is documentation, not the executable source of truth; fixtures must keep the two in step. |
| Lark | Grammar file is executable. Less parsing code. | Adds a runtime dependency. Mapping generic parse errors to the specific codes above takes custom handling per case. |

Recommendation: hand-written recursive descent, with the fixtures in `tests/fixtures/` as the
conformance suite. To be decided by the owner before RL-11B starts.

## 4. Syntax compatibility

Set out in section 8 of the specification: existing repository sources stay valid (with
`W401`), seven documented-but-unimplemented v1 constructs are rejected with `E211`, and the
permissive behaviors of the v1 runtime listed in 8.3 become errors.
