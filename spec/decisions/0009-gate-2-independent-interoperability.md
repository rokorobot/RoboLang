# Decision record 0009 — Gate 2: independent interoperability (RL-INTEROP-01a)

**Date:** 2026-10-27
**Status:** **All twelve decisions APPROVED — 2026-10-27**, as revised after review (final approval
given on the text at commit `508f5f9`). No implementation is authorized: INTEROP-1 to INTEROP-4
each need their own authorization, and INTEROP-3 needs the owner to name the author.
**Applies to:** [`spec/interop-gate-2-v0.1.md`](../interop-gate-2-v0.1.md)
**Gate:** RFC-001 section 1.4, gate 2 (decision D9)

## 1. What this record does and does not do

It proposes how gate 2 is run and judged. Each row names a recommendation and the alternative
considered. On approval, each row's status is updated with its date; an approved row is not
rewritten later.

It does not:

- change RFC-001, Core 0.1, the reference simulation, the run-record schema, or any approved
  decision (D1–D9, F1–F5, B1–B16, G1–G7, L1–L10, R1–R14, Q1–Q9);
- resolve RFC-001 open question 7 (local transport) or 13 (conformance programme). The test
  binding of I6 is for gate 2 only;
- authorize any slice (INTEROP-1 to INTEROP-4) or name the independent author.

## 2. Decisions

| # | Topic | Proposal | Alternative considered | Spec | Status |
|---|---|---|---|---|---|
| I1 | Independence level | Gate 2 passes only at **level 1**: a different author works from the frozen bundle alone, with a logged Q&A. Isolation must be **auditable**, and the evidence states its form: a recruited human or team (statement of conditions), or a clean-room AI session (fresh context, no RoboLang memory or instructions, no repository or network access, empty working directory; model, prompt, configuration and transcript recorded). A level 2 run (same author, other language, spec-only rule) may precede it as a rehearsal that does not pass gate 2 | Accept level 2 as gate 2: cheaper, but shared hidden assumptions would go undetected, which is what RFC-001 1.4 asks to exclude | 2 | APPROVED — 2026-10-27 |
| I2 | Role | The independent implementation plays **amr-07**; planner-1, arm-02 and the authority stay the reference | planner-1: A2R only, much less Core surface (no commitments, reservations, custody or R2R sequencing) | 3 | APPROVED — 2026-10-27 |
| I3 | Bundle | A fixed list of specification, schema and Core fixture files and an example amr-07 view, at one frozen commit, with a SHA-256 manifest; no source, records or golden hashes, and no harness-private data (`scenario.json`, `cases.json`, `sequencing_cases.json`: other participants' configuration, fault scripts, expected outcomes). Changes during gate 2 are errata with a new freeze; Q&A answers that add rules become errata | Give access to the repository read-only: makes the barrier unverifiable | 4 | APPROVED — 2026-10-27 |
| I4 | Dependencies | Standard library (JSON, SHA-256, HMAC), optionally a JSON Schema validator and a published JCS library; no RoboLang package or code from this repository; no network, filesystem or clock beyond the binding during a run | Allow any library: risks reusing a library built from this repository's behavior | 5 | APPROVED — 2026-10-27 |
| I5 | Language | Not Python; proposed **TypeScript on Node.js**, standard library only | Python: the same language as the reference hides language-specific assumptions | 5 | APPROVED — 2026-10-27 |
| I6 | Test binding | **`robolang-sim-stdio` 0.1**, published in the bundle: newline-delimited JSON frames over the child process's standard input and output, lock-step, harness-owned clock; carries Core documents unchanged and adds only simulation-only frames for clock, sensors, actions, receipts and claims. The participant receives a **filtered view** (its own configuration, public deployment facts, and only the HMAC secrets it needs to verify), never the case, faults, expectations or other participants' private configuration. **Lock-step rules** fix sensing (one `sense` outstanding, answered by one `sensed` with the same `request` before anything else, never `idle` while outstanding) and a 10 s wall-clock limit per expected frame; a breach is a binding failure. Shared HMAC secrets cannot prove participant identity and gate 2 does not test it. Not a deployment transport; open question 7 stays open | A real network transport (sockets or HTTP): non-deterministic timing, flaky fault injection, and it would decide open question 7 by accident | 6 | APPROVED — 2026-10-27 |
| I7 | Success criteria | Required per case: every `expect` field holds, evaluated from harness-observed facts where they exist; every amr-07 receipt equals the reference's; the sent-message sequence (type, sender, recipient, time) equals the reference's; **every participant claim is corroborated** by the harness (motion only from executed actions; stage 1–3 receipts recomputed with `robolang.core`; authorization before motion established from the wire or the standing authorization; the commit point from amr-07's acknowledgement of a release; local-state claims consistent with the wire); determinism. An inconsistent claim fails the case. Measured and reported: JCS identity of each message. Byte-identical records are not a cross-implementation criterion | Require byte-identical records: fails on key order, which Core does not fix. Trust the participant's event frames: a faulty participant could report the right states without performing the transitions | 9.1, 9.1.1, 9.2 | APPROVED — 2026-10-27 |
| I8 | Findings | Every difference is classified first as a specification gap or ambiguity (erratum decision, new freeze, both retested), then as a reference defect, then as an independent defect | Treat every difference as the independent side's bug: hides specification gaps, which gate 2 exists to find | 9.3 | APPROVED — 2026-10-27 |
| I9 | Run identity | A gate 2 run has the reference run's run ID (same inputs). The implementation used for amr-07, its version and the bundle manifest are recorded in the gate 2 evidence; the run-record schema (0007) is unchanged | Add an implementation field to the run record: a record-schema change under 0007 for information that belongs to the evidence | 10 | APPROVED — 2026-10-27 |
| I10 | Delivery | INTEROP-1 (binding self-test: the reference amr-07 through the binding reproduces the F1 golden records) → INTEROP-2 (completeness review of amr-07's rules against the bundle, then the frozen bundle) → INTEROP-3 (independent participant, by the author the owner names) → INTEROP-4 (gate 2 run and evidence). Each separately authorized | Start with the independent participant: a binding or specification defect would be found during gate 2 and cost a restart | 11 | APPROVED — 2026-10-27 |
| I11 | Scope | Excluded: real transports, A2A, `ed25519` and key management, more than one independent participant, multiple conversations, physical execution, the `.rob` language | Include a real local transport: decides open question 7 within a test gate | 12 | APPROVED — 2026-10-27 |
| I12 | Conformance prerequisite | The 30 Core fixtures run against the independent receiver in a `check` mode (G-T0) before any scenario run | Scenario runs only: receiver disagreements would surface late and mixed with behavior differences | 7 | APPROVED — 2026-10-27 |

## 3. Revision history

| Date | Change |
|---|---|
| 2026-10-27 | First draft: I1–I12 proposed |
| 2026-10-27 | Revision after review of PR #34 at `1fbd443`; all rows still proposed. I1: auditable isolation, with the form (human or team, or clean-room AI) and its conditions recorded. I3: harness-private data (scenario, case scripts, expected outcomes) removed from the bundle; an example view added. I6: filtered participant view instead of the whole scenario, the case no longer named in `start`, lock-step rules for sensing and wall-clock limits, the limits of shared HMAC secrets stated. I7: participant claims corroborated against harness-observed facts |
| 2026-10-27 | Final owner approval of I1–I12 as revised, on the text at commit `508f5f9`. Status updated to APPROVED; no change to the decision text. No slice authorized; no author named |
