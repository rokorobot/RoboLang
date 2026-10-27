# Decision record 0006 — LANG-1 combined conditions (RoboLang 1.2)

**Date:** 2026-10-27
**Status:** **All ten decisions APPROVED — 2026-10-27**, as revised after review (final approval
given on the text at commit `b576a8e`). LANG-1b implementation authorized on 2026-10-27.
**Applies to:** [`spec/robolang-v1.2.md`](../robolang-v1.2.md),
[`spec/grammar-v1.2.ebnf`](../grammar-v1.2.ebnf), [`spec/ir-schema-v1.1.md`](../ir-schema-v1.1.md),
[`tests/fixtures/v1.2/`](../../tests/fixtures/v1.2/README.md)
**Work item:** LANG-1 (Track B, decision G6); domain 09, Logic & Control Flow

## 1. What this record does and does not do

It proposes the choices that define LANG-1. Each row names a recommendation and the
alternative that was considered. On approval, the status column of each row is updated with
its date; an approved row is not rewritten later.

It does not:

- change RoboLang 1.1, Task IR 1.0, simulation 1.0, run record 1.0 or any existing fixture;
- amend any approved decision (D1–D9, F1–F5, B1–B16, G1–G7). LANG-1 follows kernel rules 1–5
  (RL-FOUNDATION-01 section 4) and charter AF1 (the IR form is specified together with the
  syntax);
- authorize LANG-1b by itself. The owner authorized it separately on 2026-10-27: strictly to
  this specification, with acceptance tests L-T1 to L-T7, simulation only, no further language
  features, and independent of RL-ARCH-02.

## 2. Decisions

| # | Topic | Proposal | Alternative considered | Spec | Status |
|---|---|---|---|---|---|
| L1 | Language version | LANG-1 is **RoboLang 1.2**. The new syntax requires `robolang 1.2;`. Unmarked and `robolang 1.1;` files stay 1.1, with identical meaning, diagnostics and IR. Each later Part B item that is approved becomes the next minor version unless the owner bundles items | Read unmarked files as 1.2: every golden IR snapshot changes its `language_version`, and files that use `and`/`or`/`not` as names break | 2.1 | APPROVED — 2026-10-27 |
| L2 | Reserved words | `and`, `or`, `not` are reserved in 1.2 files only; in 1.1 files they stay names. Moving such a file to 1.2 is listed as breaking for that file (kernel rule 2). No file in the repository is affected | Reserve them in every file: breaks valid 1.1 files | 2.2 | APPROVED — 2026-10-27 |
| L3 | Operators and grouping | Words `and`, `or`, `not`; parentheses group; `not` applies to one operand; **`and` and `or` are never mixed at one level without parentheses** (`E214`) | Conventional precedence (`not` > `and` > `or`): shorter, but a frequent source of human and model error, and a convention inherited rather than needed (AF10) | 3.1 | APPROVED — 2026-10-27 |
| L4 | Where allowed | Wherever a condition is allowed in 1.1: each `pre` entry and the condition of `wait until`. `pre` entries remain an implicit conjunction, recorded one by one | Only in `pre`; or a single combined `pre` expression | 3.2 | APPROVED — 2026-10-27 |
| L5 | Nesting bound | Depth at most 8 in the source: each `not`, each pair of parentheses and each `and`/`or` chain adds a level (`E215`, syntax). Depth at most 8 in the IR, counted on `and`/`or`/`not` objects. Source depth is never smaller than IR depth, so compiled IR always satisfies the IR bound | No bound (unbounded recursion in every consumer); a source bound counting only `not` and parentheses (first draft: eight `not` inside an `and` passed it but gave IR of depth 9, which a consumer must reject); a bound counted after flattening (a static check, so the parser's own recursion stays unbounded) | 3.3; IR 4 | APPROVED — 2026-10-27 |
| L6 | IR | Task IR schema **1.1**, additive: kinds `and` and `or` with `conditions` (two or more), `not` with `condition`. Canonical form flattens grouping only; operand order and every `not` are kept. A 1.2 source always produces IR 1.1; a 1.1 source still produces IR 1.0 byte-identical | IR 2.0; binary `and`/`or` nodes that keep the source grouping (several IR forms for one meaning, against MC1) | 7; IR 1–4 | APPROVED — 2026-10-27 |
| L7 | Three-valued logic | Kleene's strong logic: `false and unknown` = `false`, `true or unknown` = `true`, `not unknown` = `unknown`. Only `true` lets a run continue, as today | Strict logic, where any `unknown` operand makes the result `unknown`: also fails closed, but reports `S102` for conditions that are known to be false, and rejects an `or` that a known operand already satisfies | 8.1 | APPROVED — 2026-10-27 |
| L8 | Failure codes | No new run failure code. A combined `pre` entry fails with `S101` (false) or `S102` (unknown); `wait until` times out with `S241` | Separate codes for combined conditions | 8.2 | APPROVED — 2026-10-27 |
| L9 | Run record | Run record schema **1.1** for runs of 1.2 sources: identical in shape to 1.0, embedding IR 1.1. One value per `pre` entry and per `wait until`; per-operand values are not recorded, because they can be recomputed from the deterministic inputs. Runs of 1.1 sources keep run record 1.0 and their run IDs | Keep 1.0 and rely on `inputs.ir_schema_version`; or record a value for every operand (more evidence, larger records, a new field) | 8.3 | APPROVED — 2026-10-27 |
| L10 | Delivery | Two PRs, as RL-11A and RL-BASIC-01: LANG-1a (this specification, documentation and fixtures only) and LANG-1b (implementation, tests L-T1 to L-T7), after separate authorization | One PR for both | 11 | APPROVED — 2026-10-27 |

## 3. Relationship to the domain contracts

LANG-1 implements the first item of contract 4.9 (combining conditions) as Plan data, with
the three-valued truth of building block K1. Contract 4.9's other items — comparisons,
branching, bounded repetition and failure-handling branches — remain LANG-2 to LANG-4. No
other domain contract changes.

## 4. Revision history

| Date | Change |
|---|---|
| 2026-10-27 | First draft: L1–L10 proposed |
| 2026-10-27 | Review of PR #22: the source and IR nesting bounds disagreed. L5 revised so that `and`/`or` chains count in the source depth, which is then never smaller than the IR depth; fixtures `nesting_limit_with_conjunction.rob` and `e215_not_chain_in_conjunction.rob` added for the boundary |
| 2026-10-27 | Final owner approval of L1–L10 as revised, on the text at commit `b576a8e`; LANG-1b implementation authorized. Status updated to APPROVED; no change to the decision text |
