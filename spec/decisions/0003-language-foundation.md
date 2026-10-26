# Decision record 0003 — RoboLang Language Foundation (RL-FOUNDATION-01)

**Date:** 2026-10-26
**Status:** **Proposed. No decision has been made.**
**Applies to:** [`spec/architecture/rl-foundation-01-language-foundation.md`](../architecture/rl-foundation-01-language-foundation.md)

## 1. What this record does and does not do

It lists the owner decisions the Language Foundation document needs. Until the owner
approves a row, that row has no effect.

It does not:

- change RoboLang 1.1, its grammar, parser, validator, IR schema 1.0, CLI or v1 runtime;
- change RFC-001 or the approved rows of
  [decision record 0002](0002-ai-native-architecture.md);
- change [`docs/DEVELOPMENT_PLAN.md`](../../docs/DEVELOPMENT_PLAN.md). If F5 is approved, the
  plan is updated in a separate PR;
- authorize implementation of RL-BASIC-01 or RL-ARCH-02.

## 2. Proposed decisions

| # | Topic | Proposal | Status |
|---|---|---|---|
| F1 | Thirteen domains | Adopt the thirteen domains in four families (section 3) as RoboLang's architectural taxonomy. They place work; they are not modules and not a requirement to cover every domain before RL-BASIC-01 | PROPOSED |
| F2 | Language Kernel | The existing RoboLang 1.1 front end (lexer, parser, typed AST, validator, Task IR 1.0, CLI) is the Language Kernel. New features extend it under the five rules of section 4; no second language engine is started | PROPOSED |
| F3 | Foundations 1–3 | Adopt the three development stages of section 5. Foundation 1 Part A is RL-BASIC-01; Part B language extensions (variables, structured data, expressions) are separate versioned proposals, each needing its own owner decision | PROPOSED |
| F4 | RL-BASIC-01 | Adopt RL-BASIC-01 as defined in section 6: simulation-only execution of RoboLang 1.1 tasks, delivered as a specification PR (RL-BASIC-01a) and then an implementation PR (RL-BASIC-01b). The simulation semantics of section 6.3 are decided in RL-BASIC-01a, not by this row | PROPOSED |
| F5 | D8 scheduling amendment | Prioritize Foundation 1, including RL-BASIC-01, before RL-ARCH-02. RL-ARCH-02 stays approved as a milestone, still needs separate explicit authorization, and is scheduled after the Foundation 1 exit gate. The developer-preview pilot stays independent | PROPOSED |

Section numbers refer to the Language Foundation document.

## 3. Semantic choices left to RL-BASIC-01a

Approving F4 does not settle these. They are proposed in section 6.3 and decided when the
RL-BASIC-01 specification is reviewed:

1. `place r o at dst` requires the robot to be at `dst`, rather than moving it there.
2. A condition the world does not declare is unknown and fails closed.
3. `wait until` that times out ends the run `FAILED`.
4. A `max_speed` with a unit above the robot profile's maximum fails the run before the first
   step; a unitless limit is recorded as not enforced.
5. `communicate` records sending only, not delivery.

## 4. Relationship to open questions

None of RFC-001's open questions (decision record 0002, section 4) is resolved here. In
particular, open question 8 (whether goals and outcomes get `.rob` syntax) stays open:
RL-BASIC-01 produces a run record, not new syntax.
