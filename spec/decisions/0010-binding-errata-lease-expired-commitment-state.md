# Decision record 0010 — `robolang-sim-stdio` 0.1 errata: `lease_expired` and commitment state (INTEROP-1 findings E1, E4)

**Date:** 2026-10-27
**Status:** **Both decisions APPROVED — 2026-10-27**, with the verification safeguards below as
conditions of the approval. Implemented in INTEROP-1 (PR #36). This record does not authorize a
merge or any other slice.
**Applies to:** [`spec/interop-gate-2-v0.1.md`](../interop-gate-2-v0.1.md) sections 6.3 and 9.1.1
**Amends:** the frame list of the binding approved as I6 in
[decision record 0009](0009-gate-2-independent-interoperability.md). Decision record 0009 and its
rows I6 and I7 are not changed; this record adds two narrowly scoped errata (spec 9.3, class 1).

## 1. The problem

The INTEROP-1 self-test ran the reference amr-07 as an external process through the binding.
Two things the run record requires could not cross the binding as written:

- **E1.** The participant `event` kinds of 6.3 (`state`, `authorization`, `escalated`,
  `custody_committed`) have no `lease_expired`. The reference amr-07 records one when its
  commitment lease expires, and the run record carries it. With the kinds of 6.3 alone,
  `v05_stale_confirmation` and `v06_partition_before_release` fail with a binding failure.
- **E4.** 6.3 says `state` claims cover "goal, commitment and custody state changes", but does
  not say which claims become run-record events. The reference records no event for commitment
  state changes; the state appears only in the record's final section
  (`final.participants[…].commitment_state`). Without a rule, that state is lost.

## 2. Decisions

| # | Topic | Decision | Safeguard (condition of approval) | Spec | Status |
|---|---|---|---|---|---|
| B1 | `lease_expired` claim (E1) | Add `lease_expired` to the participant `event` kinds, with detail `lease: "commitment"` and `commitment` (the commitment ID). It is a claim and is recorded as a run-record event, as the reference records it | The harness verifies that the named commitment arrived in a `commit` the participant accepted, and that its lease has expired (`now ≥ lease.expires_at`). Otherwise the claim is inconsistent and the run fails | 6.3, 9.1.1 | APPROVED — 2026-10-27 |
| B2 | Commitment state claim (E4) | A `state` claim with `record: "commitment"` and `state` (Core 5.8 local state: `active`, `fulfilled`, `failed`, `lapsed`) sets the participant's final commitment state. It is **not** a run-record event, because the reference records none | The harness accepts only supported transitions: `active` first, then one of `fulfilled`, `failed` or `lapsed`. Every claim needs an accepted `commit` whose debtor is the participant. `fulfilled` needs a sent `outcome` of `succeeded` whose evidence consists of observations the harness itself made; `failed` needs a sent `outcome` other than `succeeded`; `lapsed` needs the lease to have expired. Otherwise the claim is inconsistent and the run fails | 6.3, 9.1.1 | APPROVED — 2026-10-27 |

## 3. Not decided here

- **E2** (the view's sensor characteristics) and **E3** (who sends the reply to a duplicate
  again) are documented before the independent bundle is frozen (INTEROP-2).
- **E5** (hold timers of a silent participant) must be resolved before the gate 2 run
  (INTEROP-4).
- INTEROP-2 to INTEROP-4 remain unauthorized; PR #36 is not authorized to merge.

## 4. Revision history

| Date | Change |
|---|---|
| 2026-10-27 | B1 and B2 recorded as approved by the owner in the review of PR #36 at `50d982c`, with the safeguards as conditions |
