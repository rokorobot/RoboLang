# Decision record 0005 — AI-first charter and domain contracts (RL-FOUNDATION-02)

**Date:** 2026-10-27
**Status:** **All seven decisions APPROVED — 2026-10-27**, as revised after owner review
(final approval given on the text at commit `6890d4d`). No implementation is authorized.
**Applies to:** [`spec/architecture/ai-first-language-charter.md`](../architecture/ai-first-language-charter.md),
[`spec/architecture/rl-foundation-02-domain-contracts.md`](../architecture/rl-foundation-02-domain-contracts.md)

## 1. What this record does and does not do

It records the owner's decisions on the AI-first charter and the domain contracts. A later
change to an approved row is recorded as a new row or a new decision record, with its date;
the approved rows are not rewritten.

It does not:

- change RoboLang 1.1, Task IR 1.0, the CLI, the v1 runtime or the simulation specification;
- amend any approved decision: D1–D9 (RFC-001, decision record 0002), F1–F5 (decision
  record 0003) or B1–B16 (decision record 0004). Applying the charter needed none;
- create Core schema files (RL-ARCH-02, RFC-001 section 19);
- authorize any implementation. **Approving G7 approves the scope of the first coding slice;
  starting it still needs the separate RL-BASIC-01b implementation authorization** that F4
  and F5 require.

## 2. Decisions

| # | Topic | Decision | Applies to | Status |
|---|---|---|---|---|
| G1 | AI-first charter | Adopt principles AF1–AF10: RoboLang is designed primarily for AI systems to generate, read, exchange, validate and act on, building on RFC-001 P1–P10 | Charter sections 1–3 | APPROVED — 2026-10-27 |
| G2 | Measurable criteria | Adopt criteria MC1–MC8, each demonstrated by a named artifact, as the acceptance basis for domain designs and implementation slices | Charter section 4 | APPROVED — 2026-10-27 |
| G3 | Concepts as data first | A new domain concept is defined first as Core or Task IR data. `.rob` syntax for it is added only for a demonstrated authoring need, through its own versioned language specification and owner decision. Goals and outcomes in `.rob` remain RFC-001 open question 8 | Charter AF1, AF2; contracts section 4 | APPROVED — 2026-10-27 |
| G4 | Four homes and the matrix | Place every domain concept in Plan, Core, Sim or Ext, and adopt the development matrix and the shared building blocks K1–K8 | Contracts sections 2–3 | APPROVED — 2026-10-27 |
| G5 | Domain contracts | Adopt the thirteen contracts as the engineering baseline. They are not normative specifications: a slice that adds `.rob` syntax, an IR kind or a Core schema still needs its own specification. Reasoning and memory are external interfaces with validated results; RoboLang embeds no model, database or planner | Contracts section 4 | APPROVED — 2026-10-27 |
| G6 | Work breakdown | Adopt Tracks A–E and their order, consistent with F3 and F5: BASIC-1 to BASIC-3, then the Foundation 1 exit gate; LANG-1 to LANG-6 scheduled independently; CORE-1 and RL-ARCH-02 after BASIC-3 under separate authorization; Foundation 2 and 3 items after their gates. Add **CORE-1, an IR reader and re-validator**, as a new planned work item | Contracts section 5 | APPROVED — 2026-10-27 |
| G7 | First coding slice | Approve the scope of BASIC-1 (RL-BASIC-01b-1): `robolang.sim` values and world file 1.0 reading and validation, with the files, exclusions and verification commands of contracts section 6. Coding starts only after the separate RL-BASIC-01b implementation authorization | Contracts section 6 | APPROVED — 2026-10-27 |

## 3. Relationship to open questions

None of RFC-001's open questions is resolved. The contracts name the ones each domain depends
on: 1 (ontology) for Memory & Knowledge, 2 (world-model depth) for Data & World Model, 3
(time) for Time & Coordination, 8 (text syntax for goals and outcomes) for Goals & Tasks and
Outcomes & Recovery, 11 (policy language) for Safety & Authorization, and 12 (skills) for
Goals & Tasks and Capabilities & Resources.

## 4. Revision history

| Date | Change |
|---|---|
| 2026-10-27 | First draft: G1–G7 proposed |
| 2026-10-27 | Owner refinements before final review; documentation only, G1–G7 and the work breakdown unchanged. Charter: the central abstraction (a verifiable transition from an observed world state toward a desired outcome) at the start of section 1; a comparison with traditional languages (1.1); MC1 limited to the same inputs in the same canonical representation; the thirteen domains described as one semantic language, not thirteen implementations. Contracts: trajectories and trajectory evidence (01), hypotheses distinct from observations (05), temporal relationships (10), capability prerequisites (11). Correction to 11: RL-BASIC-01 checks capabilities in stages (speed maxima before authorization, gripper and sensors at their steps), and a comprehensive preflight is planned later; the approved simulator behavior and fixtures are unchanged |
| 2026-10-27 | Final owner approval of G1–G7 as revised, on the text at commit `6890d4d`. Status updated to APPROVED; no change to the decision text |
