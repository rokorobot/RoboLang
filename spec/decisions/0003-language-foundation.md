# Decision record 0003 — RoboLang Language Foundation (RL-FOUNDATION-01)

**Date:** 2026-10-26
**Status:** **All five decisions APPROVED — 2026-10-26**, as revised after owner review
(final approval given on the text at commit `4f351be`). The semantic choices in section 3
remain open until RL-BASIC-01a.
**Applies to:** [`spec/architecture/rl-foundation-01-language-foundation.md`](../architecture/rl-foundation-01-language-foundation.md)

## 1. What this record does and does not do

It records the owner's decisions on the Language Foundation document. The rows below are
approved; a later change to one of them is recorded as a new row or a new decision record,
with its date, and the approved rows are not rewritten.

It does not:

- change RoboLang 1.1, its grammar, parser, validator, IR schema 1.0, CLI or v1 runtime;
- change RFC-001 or the approved rows of
  [decision record 0002](0002-ai-native-architecture.md). In particular, D8's requirement of a
  separate explicit owner authorization before RL-ARCH-02 is implemented stays in force;
- change [`docs/DEVELOPMENT_PLAN.md`](../../docs/DEVELOPMENT_PLAN.md). The plan's milestone
  order is updated to reflect F5 in a separate PR;
- authorize implementation of RL-BASIC-01 or RL-ARCH-02.

## 2. Decisions

| # | Topic | Decision (as revised by the owner on 2026-10-26) | Status |
|---|---|---|---|
| F1 | Thirteen domains | Approve the thirteen-domain taxonomy in four families (section 3) as RoboLang's architectural taxonomy. The domains place work; they are not modules and not a requirement to cover every domain before RL-BASIC-01 | APPROVED — 2026-10-26 |
| F2 | Language Kernel | Approve the existing RoboLang 1.1 front end (lexer, parser, typed AST, validator, Task IR 1.0, CLI) as the single `.rob` language engine. New `.rob` features extend it under the rules of section 4. Core JSON documents use their own schema and semantic validation path, and neither path bypasses local authorization (section 2) | APPROVED — 2026-10-26 |
| F3 | Foundations 1–3 | Approve the three development stages of section 5. Foundation 1 Part A is RL-BASIC-01, and the Foundation 1 exit gate is RL-BASIC-01 passing its approved acceptance tests with owner sign-off. Part B extensions (variables, expressions, structured data beyond RoboLang 1.1) are scheduled independently, each needing its own owner decision, and are not a prerequisite for the exit gate or for RL-ARCH-02 | APPROVED — 2026-10-26 |
| F4 | RL-BASIC-01 | Approve RL-BASIC-01 (section 6) as the planned simulation-only milestone, delivered as a specification PR (RL-BASIC-01a) and a separate implementation PR (RL-BASIC-01b). This does not authorize implementation. The simulation semantics of section 6.3 are decided in RL-BASIC-01a, not by this row | APPROVED — 2026-10-26 |
| F5 | D8 scheduling amendment | Amend D8's order to: RL-BASIC-01a specification → separately authorized RL-BASIC-01b implementation and acceptance → RL-ARCH-02, subject to its existing separate implementation authorization. Foundation 1 Part B extensions must not delay RL-ARCH-02. The developer-preview pilot stays independent | APPROVED — 2026-10-26 |

Section numbers refer to the Language Foundation document.

## 3. Semantic choices left to RL-BASIC-01a

Approving F4 does not settle these. They are proposed in section 6.3 and decided when the
RL-BASIC-01 specification is reviewed:

1. `place r o at dst` requires the robot to be at `dst`, rather than moving it there.
2. A condition the world does not declare is unknown and fails closed.
3. `wait until` that times out ends the run `FAILED`.
4. A `max_speed` with a unit above the robot profile's maximum fails the run before the first
   step, as a configuration check. No limit is enforced in simulation, and a unitless limit
   is recorded as "unit unspecified, not enforced".
5. `communicate` records sending only, not delivery.

RL-BASIC-01a must also define how the deterministic run ID is derived (section 6.2).

## 4. Relationship to open questions

None of RFC-001's open questions (decision record 0002, section 4) is resolved here. In
particular, open question 8 (whether goals and outcomes get `.rob` syntax) stays open:
RL-BASIC-01 produces a run record, not new syntax.

## 5. Revision history

| Date | Change |
|---|---|
| 2026-10-26 | First draft: F1–F5 proposed |
| 2026-10-26 | Owner review. F2 clarified (Core JSON keeps its own validation path; no bypass of authorization). F3 amended (the Foundation 1 exit gate is RL-BASIC-01 acceptance; Part B is independent of RL-ARCH-02). F4 states that it does not authorize implementation. F5 given the exact order RL-BASIC-01a → RL-BASIC-01b → RL-ARCH-02. Document corrections: the layer diagram shows Core JSON validation and local authorization before execution; RL-BASIC-01 run IDs are deterministic; simulated movement is symbolic and no limit is enforced in simulation |
| 2026-10-26 | Final owner approval of F1–F5 as revised, on the text at commit `4f351be`. Status updated to APPROVED; no change to the decision text |
