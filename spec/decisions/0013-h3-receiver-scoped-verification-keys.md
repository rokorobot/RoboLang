# Decision record 0013 — Receiver-scoped verification keys (H3)

**Date:** 2026-10-28
**Status:** **APPROVED — 2026-10-28.** H3, option A, approved by the owner. Applied to the
reference simulation (5.3), the reference receiver and the INTEROP harness in one PR, authorized
2026-10-28. No change to Core 0.1, the schemas, the gate 2 participant view, the fixtures or the
golden records.
**Applies to:** [`spec/reference-simulation-v0.1.md`](../reference-simulation-v0.1.md) 5.3
**Builds on:** [`spec/core-v0.1.md`](../core-v0.1.md) 7 ("in 0.1 the deployment's configuration
lists the keys"); [`spec/interop-gate-2-v0.1.md`](../interop-gate-2-v0.1.md) 6.4 (the participant
view) and 9.1.1 (stages 1–3 recomputed by the harness); finding H3 of the
[discrepancy register](../../docs/interop/spec-implementation-discrepancies.md), section 2

## 1. Why

Reference simulation 5.3 says that the scenario file gives each participant one key and that every
receiver verifies every signature. The reference read this as "every receiver knows every key". Gate
2 6.4 gives amr-07 only its own key and the verification keys of the participants whose messages it
must verify: arm-02 and its authority. A correctly signed message to amr-07 from a sender outside its
view (planner-1) was therefore accepted at stage 2 by the reference amr-07, and by the harness's
recomputation, but rejected `C203` by an amr-07 built from the bundle. Both behaviors followed from a
text: a specification gap (gate 2 9.3, class 1), found in INTEROP-2e. None of the 24 cases reaches it.

## 2. Decision

| # | Topic | Rule | Alternatives considered | Spec | Status |
|---|---|---|---|---|---|
| H3 | The keys a receiver verifies with | **Option A.** Each receiver verifies signatures with the keys of its configured trust context only. For amr-07 that context is its own key and the verification keys of its gate 2 6.4 view. A signature that needs a key outside that context is rejected `C203`, even if the scenario holds that key elsewhere. Other reference participants keep their existing verification keys. The stage-2 order is unchanged: `C204`, `C203`, `C202` (0012 P3) | **(B)** Keep the reference as it is and have the harness skip comparing `C203` for senders outside the view: it would keep a permitted difference between reference and independent receipts, against the purpose of gate 2 | Sim 5.3 | APPROVED 2026-10-28 |

**Principle.** Trust belongs to the receiving robot's configuration, not to the global knowledge of
the simulation.

**Consistency.** Core 7 already leaves key lists to the deployment's configuration; this record
states what that configuration is for amr-07. Core 0.1 signature semantics and codes are unchanged.
amr-07 gets no additional key, and the view's structure is unchanged. Key distribution and revocation
stay RFC-001 open question 5.

## 3. Normative wording

**Reference simulation 5.3, after "…so a corrupted or missing signature is caught where Core 8 says (R5).":**

> *Erratum ([decision record 0013](decisions/0013-h3-receiver-scoped-verification-keys.md), H3).*
> Each receiver verifies signatures using only the keys available in its configured trust context.
> For amr-07, that context consists of its own key and the verification keys supplied in its gate 2
> 6.4 participant view. A signature requiring a key unavailable in that context is rejected with
> `C203`, even if the scenario contains that key elsewhere. Other reference participants retain their
> existing configured verification keys.

## 4. Implementation and impact

| | Change |
|---|---|
| Reference receiver | The engine's receiver context for amr-07 holds only the keys of its trust context. The key references come from one function, used by both the engine and the gate 2 view builder, so the two cannot drift |
| Harness | It recomputes stages 1–3 with that same context (it already used the engine's receiver context), so its recomputation, the reference amr-07 and an amr-07 built from the view agree |
| Golden records | **Unchanged.** In the 24 cases only arm-02 and the authority send to amr-07 |
| Fixtures | None added (INTEROP-2f) |

## 5. Revision history

| Date | Change |
|---|---|
| 2026-10-28 | H3 option A approved by the owner and recorded; applied to Sim 5.3, the reference receiver and the harness |
