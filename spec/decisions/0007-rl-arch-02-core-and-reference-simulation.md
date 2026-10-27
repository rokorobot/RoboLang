# Decision record 0007 — RL-ARCH-02a: Core 0.1 and the reference simulation

**Date:** 2026-10-27
**Status:** **All fourteen decisions APPROVED — 2026-10-27**, including R11 (final approval given
on the text at commit `bad9969`). No implementation is authorized: ARCH-1 to ARCH-3 still need
the separate authorization D8 requires.
**Applies to:** [`spec/core-v0.1.md`](../core-v0.1.md), [`spec/core/`](../core/),
[`spec/reference-simulation-v0.1.md`](../reference-simulation-v0.1.md),
[`tests/fixtures/core/`](../../tests/fixtures/core/README.md),
[`tests/fixtures/arch02/`](../../tests/fixtures/arch02/README.md)
**Milestone:** RL-ARCH-02 (RFC-001 section 19; D8)

## 1. What this record does and does not do

It proposes the choices that turn RFC-001's description of RL-ARCH-02 into a checkable
specification. Each row names a recommendation and the alternative that was considered. On
approval, each row's status is updated with its date; an approved row is not rewritten later.

It does not:

- change RFC-001 or any approved decision (D1–D9, F1–F5, B1–B16, G1–G7, L1–L10). Where this
  specification adds something RFC-001 does not name (R4) or departs from an RFC-001 example
  (R11, and `keys` in Core 5.10), the row says so;
- resolve any RFC-001 open question. Rows R5, R6, R7 and R9 state which ones they touch and
  leave them open;
- change RoboLang 1.1 or 1.2, Task IR, the simulation specification, the CLI or any code;
- authorize implementation. ARCH-1 starts only after approval and the separate
  authorization D8 requires.

## 2. Decisions

| # | Topic | Proposal | Alternative considered | Spec | Status |
|---|---|---|---|---|---|
| R1 | Core 0.1 scope | Define exactly the 20 message types the RFC-001 §18 scenario and its variants use. The other 10 names of RFC-001 9.1 and 10.1 are reserved and rejected (`C106`), so the vocabulary is closed (AF3) | Define payloads for every RFC type now: larger, and untested by the scenario | Core 1, 4 | APPROVED — 2026-10-27 |
| R2 | Document rules and codes | Every field always present (nullable rather than optional); integers only for counters; no extension points; unknown fields rejected; stable rejection codes `C101`–`C404` in a fixed check order, signature before content | Optional fields and free extension points: more flexible, less predictable to generate and validate | Core 2.3, 8 | APPROVED — 2026-10-27 |
| R3 | No plan exchange in 0.1 | No message carries a plan; robots run their own local plans. RL-ARCH-02 therefore does not depend on CORE-1 (IR reader) | Carry Task IR in `request_assistance`: needs CORE-1 first and is not required by §18 | Core 10; sim 6 | APPROVED — 2026-10-27 |
| R4 | Replies RFC-001 implies but does not name | Add `acknowledge`, `authorization_decision` and `reservation_decision`; `coordinate` phases `ready_for_handover` and `placed`; assessment result `feasible_with_assistance`. §18 step 6 becomes `reserve` and `reservation_decision` | Encode replies as other types with a status field (for example `reserve` with `granted`): fewer types, but one type with two meanings | Core 4.1 | APPROVED — 2026-10-27 |
| R5 | Signatures | JCS (RFC 8785) canonical form. `ed25519` defined but not implemented in the reference; the reference uses `sim-hmac-sha256` (standard library only), which is simulation only and rejected in physical mode (`C204`). Keys are deployment configuration; open question 5 (key management) stays open | Implement `ed25519` now: adds a third-party runtime dependency to a dependency-free package; or no signatures: leaves the "wrongly signed `commit`" variant untestable | Core 7 | APPROVED — 2026-10-27 |
| R6 | Policy as data | Local policy, standing-authorization evaluation and the authority are scenario data and a scripted stub. A goal's class is decided locally and is not a Core field. Open question 11 (policy language) stays open | Add `goal_class` to the Core goal; or adopt a policy language now | Sim 7 | APPROVED — 2026-10-27 |
| R7 | Simulated world | A symbolic world of Core triples specific to the scenario, with carried things and custody. Not world file 1.0, which is unchanged; open question 2 (world-model depth) stays open | Extend world file 1.0 with carrying and custody: reopens approved B12 and B13 for a scenario that does not need `.rob` tasks | Sim 4 | APPROVED — 2026-10-27 |
| R8 | "No motion" is observable | Every action emits a motion event; acceptance tests count them per participant | Infer motion from final world state: misses motion that was undone or never completed | Sim 4.2 | APPROVED — 2026-10-27 |
| R9 | Determinism | Derived message IDs (SHA-256 of sender, conversation and sequence), one shared simulated clock with the clock tolerance still applied, fixed latency, events ordered by time then send order. Run ID from scenario hash, case and seed. Open question 3 (time) stays open | Random ULIDs and wall-clock time, as a real deployment would use: runs would not be reproducible (MC1, MC8) | Sim 3, 5, 9 | APPROVED — 2026-10-27 |
| R10 | Exact expected results | Where RFC-001 §18 allows "`failed` or `unknown`", fix one: `failed` when the robot can observe that the goal was not reached (V4, V5); `unknown` when custody or evidence is unknown (V6, V8, V9) | Accept either value: the tests would not distinguish correct behavior from a lucky one | Sim 8 | APPROVED — 2026-10-27 |
| R11 | V11 deviation from RFC-001 | In the 35 kg variant, raise arm-02's grasp limit to 40 kg, because otherwise arm-02's own 30 kg limit stops the run before amr-07's standing authorization is reached, and the variant would not test what RFC-001 says it tests | Keep 30 kg: V11 then duplicates V13 (assessment fails) and leaves the standing-authorization limit untested | Sim 8 | APPROVED — 2026-10-27 |
| R12 | Schema validation in tests | ARCH-1 adds `jsonschema` as a **test-only** dependency (in `requirements-dev.txt`) to check that the schemas are valid draft 2020-12 and that the fixtures validate. The `robolang` package keeps no runtime dependencies: its own Core checks are written by hand | No schema validator: the schema files would never be checked against the fixtures; or a runtime dependency | Sim 12 (A-T17) | APPROVED — 2026-10-27 |
| R13 | Package placement | New subpackages `robolang.core` and `robolang.arch02` beside `robolang.sim`. They import nothing from the 1.x parser, IR or `robolang.sim`, and change none of it | A separate package or repository: cleaner for gate 2, premature before Core is stable | Sim 13 | APPROVED — 2026-10-27 |
| R14 | Delivery | This specification PR (RL-ARCH-02a), then three implementation slices ARCH-1 (Core checks), ARCH-2 (world and participants), ARCH-3 (record, entry point, README), one PR each, after the D8 authorization | One implementation PR: too large to review | Sim 13 | APPROVED — 2026-10-27 |

## 3. Relationship to the domain contracts

RL-ARCH-02 is the Core path of the domain contracts (RL-FOUNDATION-02, homes *Core*): it
delivers the message envelope and A2R/R2R operations of domain 08, capability descriptors of
domain 11, authorization of domain 12 and outcomes with evidence of domain 13, in the
simulation. Goals (domain 04) appear as Core documents; their `.rob` syntax remains RFC-001
open question 8. Building blocks K2, K4 and K5 are reused as designs; K7 (executor states)
is reused as the goal lifecycle's design, not as code (R13).

## 4. Revision history

| Date | Change |
|---|---|
| 2026-10-27 | First draft: R1–R14 proposed |
| 2026-10-27 | Final owner approval of R1–R14 as proposed, including R11, on the text at commit `bad9969`. Status updated to APPROVED; no change to the decision text. Implementation not authorized |
