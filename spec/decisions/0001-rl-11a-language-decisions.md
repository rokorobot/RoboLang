# Decision record 0001 — RL-11A language decisions

**Date:** 2026-10-12
**Status:** Decided by the owner. No open proposals.
**Applies to:** [`spec/robolang-v1.1.md`](../robolang-v1.1.md), [`spec/grammar.ebnf`](../grammar.ebnf)

## 1. Guiding principle

Valid syntax, valid task semantics and safe physical execution are separate concepts. A
program accepted by the parser must never automatically be considered safe to run on real
hardware.

## 2. First round of decisions

| # | Topic | Decision |
|---|---|---|
| 1 | `max_speed` | Explicit unit required (`m/s` or `rad/s`). The existing unitless form stays accepted as deprecated, with a warning. No implicit unit conversion or assumption for physical execution. |
| 2 | `inspect` | The sensor must be named explicitly. No default sensor is selected. Sensor capability validation comes later. |
| 3 | `wait until` | In the v1.1 grammar, with a mandatory `timeout <duration>`. Execution, cancellation and timeout handling belong to v1.2. |
| 4 | Multiple tasks per file | Allowed. Execution requires explicit task selection; all tasks are never executed automatically. |
| 5 | Strict syntax | Missing semicolons, unitless durations, unknown statements, prefix matches and malformed constructs are rejected with useful diagnostics. |
| 6 | AST | Frozen dataclasses with source spans. Name binding and type validation are deferred to RL-11C. |
| 7 | Invariant | The complete program is parsed and validated before any adapter action can occur. |

## 3. Second round: review of the draft specification

| # | Question | Decision |
|---|---|---|
| 1 | `using` vs `with` on `inspect` | **`using`** is the canonical syntax, as documented in the v1 manual. This supersedes the `with` wording of decision 2 above; the sensor remains mandatory. |
| 2 | `object_exists` | Approved as a condition. Runtime evaluation remains deferred. |
| 3 | Seven documented v1 constructs | Stay deferred. Each is rejected explicitly with `E211` rather than silently accepted with unsupported safety or execution semantics. |
| 4 | Reserved keywords as task names | Keywords stay reserved. `task inspect(...)` is invalid; use a name such as `inspect_object`. |
| 5 | Version marker | Optional `robolang 1.1;`. Unmarked files remain accepted as v1.1. Unsupported versions fail validation. |
| 6 | String escapes | `\"`, `\\`, `\n`, `\t`. Unknown escapes are rejected. |
| 7 | Empty `plan` | Rejected with `E208`. An empty `pre` remains valid. |
| 8 | Linear and angular `max_speed` | Both may be declared, at most once per unit type within a task. Duplicate limits of the same unit type are errors. The unitless form is acceptable only as a single legacy annotation with `W401`; it must not be combined with explicit limits or treated as an enforced physical limit. Enforcement and the relationship to robot capabilities belong to v1.2. |

The seven deferred constructs are `with_approach`, `in_region`, `@safety no_go_region`,
`@safety require_guard`, action-level `@safety`, `message_received`, and `scalar` / `duration`
parameters.

## 4. Parser approach for RL-11B

**Decided: hand-written recursive descent.**

- The grammar is small and LL(1): every statement starts with a distinct keyword.
- No runtime dependency is added.
- It gives direct control over the diagnostic codes and source spans the specification
  requires.
- The implementation must have a proper lexer and build the AST of section 7 of the
  specification. It must not be another collection of line-based regular expressions.
- The fixtures in `tests/fixtures/` are the conformance suite, since the grammar file is
  documentation rather than executable input.

Considered and not chosen: Lark (executable grammar, but adds a dependency and needs custom
handling to map generic parse errors to the specific codes).

## 5. How the specification implements round two

| Decision | In the specification |
|---|---|
| Version marker | Grammar rule `version_decl`; section 4.1; `robolang` reserved; new code `E213` |
| String escapes | Section 3.1; new lexical code `E104` |
| Dual speed limits | Section 4.4; static rules 6 and 7; `E306` redefined per unit type; new code `E308` for unitless combined with explicit |
| Migration of documented examples | Section 8.5 |

## 6. Syntax compatibility

Set out in section 8 of the specification: existing repository sources stay valid (with
`W401`), the seven deferred constructs are rejected with `E211`, the permissive behaviors of
the v1 runtime listed in 8.3 become errors, and the four affected documented examples have
written migrations in 8.5.
