# Specification–implementation discrepancy register (INTEROP-2b-SPEC; updated by INTEROP-2d)

**Date:** 2026-10-28. **Specifications:** as amended by
[decision record 0011](../../spec/decisions/0011-interop-2a-bundle-errata.md) (N1–N14, approved
2026-10-28) and [decision record 0012](../../spec/decisions/0012-interop-2c-remaining-normative-decisions.md)
(P1–P3, approved 2026-10-28). **Code compared:** `main` at `6d22085`; section 1 updated by INTEROP-2d, which corrects D1–D7, and section 2 by INTEROP-2e, which corrects N3, H1, H2 and N13(c).

The specifications now state the approved behavior. Where the reference implementation
(`robolang.arch02`) or the INTEROP-1 harness (`robolang.interop`) does not yet follow them, the
difference is listed here, so that it is visible rather than hidden. **Nothing here is fixed by
this register**: each fix needs its own approved scope (0011, section 4). None of these
differences is reached by the 24 cases, so the golden records do not show them, and every fix
must leave the 24 golden records unchanged.

The register is reviewer material and cites `src/`. It is not part of the gate 2 bundle.

## 1. Reference defects (0011, section 4; spec 9.3, class 2)

**All seven corrected by INTEROP-2d** (authorized 2026-10-28), with regression tests in
`tests/test_arch02_corrections.py`. None was reached by the 24 cases, and the 24 golden records
are unchanged. The rows record what the reference did before the correction.

| # | Specification says | The reference did | Correction (INTEROP-2d) | Reached by the 24 cases |
|---|---|---|---|---|
| D1 | A `contradicted` criterion gives an outcome `failed` with empty `reasons` (reference simulation 6.3, N11) | `reasons: [{"code": null}]`, an invalid payload, in amr-07's and arm-02's outcomes. planner-1 records `reason: null` in its `planner_goal` event | **Corrected:** empty `reasons` in amr-07's and arm-02's outcomes. planner-1's record is not a message, and is unchanged | No |
| D2 | A `custody_query` before any `request_assistance` is rejected `C306` (reference simulation 6.4, N8) | Raised an error: no thing was known yet | **Corrected:** `C306` when no sub-goal exists | No |
| D3 | An action in progress at lease expiry completes; when it completes after the lapse, amr-07 changes no state, senses nothing and sends nothing (reference simulation 6.2 rule 3 and 6.4, 0012 P1) | Arrival set the commitment `fulfilled` over `lapsed` and sent a second outcome | **Corrected:** after either move, amr-07 takes no follow-up step once its commitment has lapsed | No |
| D4 | Capability failure codes: `capability_unsupported` for an absent, non-current or invalidly signed descriptor, or a kind absent or listed unsupported; otherwise `capability_insufficient` (reference simulation 6.4, N10; Core 5.11) | `capability_insufficient` in every case | **Corrected:** two codes, as the specification says | No |
| D5 | Before every move: authorization still established; goal `RUNNING`; commitment `active` with its lease unexpired; a granted reservation unexpired; judged in the safe direction (reference simulation 6.2 rule 2 and 6.4) | Checked only the goal state and the commitment state | **Corrected:** all four checks at the moment of each move. The standing authorization is checked again (7.2); a granted decision by its lease | No (the latest amr-07 move starts at 10:14:51.200, before `reservation_end` 10:20) |
| D6 | A different message with the sequence of a message held in the same stream is rejected `C306`; the held message keeps its place and gets one terminal release receipt (Core 8.4, 0012 P2) | The second replaced the held one, and the first never got a release receipt | **Corrected:** `C306`; the held message is untouched | No |
| D7 | `C401`, `C402` (unmet criterion) and `C404` are applied after duplicate and order handling (Core 8.1, N13) | Applied together with stages 1–3, before stage 4 | **Corrected:** the reference receiver applies them after stage 4 (`check_message(..., meaning=False)`, then `check_meaning`). `check_message` alone still gives the stage-5 codes for a stateless conformance check | No (the only such rejection is v10's `C401`, and nothing follows it in that stream) |

## 2. INTEROP-1 harness (0011, section 6; recognized follow-up work)

**All four corrected by INTEROP-2e** (authorized 2026-10-28), with regression tests in
`tests/test_interop_harness.py`. The 24 golden records are unchanged and the INTEROP-1 self-test
still reproduces them. The rows record what the harness did before the correction.

| # | Specification says | The harness did | Correction (INTEROP-2e) |
|---|---|---|---|
| N3 | Silence: the harness takes over open holds through a hold ledger, with exactly one terminal release receipt per held message (gate 2 6.2) | The participant's hold timers were owned by the participant and skipped while it was silent; there was no ledger or takeover | **Corrected:** a `held` receipt opens a ledger entry with its stream (sender, `conversation_id`), sequence, expiry and delivery time; the harness checks it at both Core 8.4 instants and, while the participant is silent, closes it (`C301` at expiry, which wins a tie; `not_handled` at `max_hold`), then releases the entries of that stream that are now next. The recommended `seq_silent_while_held` golden case is INTEROP-2f |
| H1 | Presented observations are compared **in meaning**, with a valid signature; formatting alone is not a failure (gate 2 9.1.1, N5) | Compared the whole document, signature value included: stricter than specified | **Corrected:** same fields, `id`, `kind`, `subject`, `property`, `value` and `source`; the same instant, duration and decimal; a `sim-hmac-sha256` signature under the participant's key that verifies over the observation as presented. The four INTEROP-1 provenance tests still fail as intended |
| H2 | Diagnostics are optional, but each must be consistent with the same cycle; any other `record` value is a binding failure (gate 2 6.3, 9.1.1, N7) | Recorded every `state` claim without checking diagnostics or unknown `record` values | **Corrected:** each of the five diagnostics is checked against the same cycle (accepted messages, `act` frames, null readings, the commit point), with exactly its listed fields; any other `record` is a binding failure |
| N13(c) | Only stage 1–3 codes are compared with the harness's recomputation (gate 2 9.1.1) | Compared every code that `check_message` gave, including the stage-5 `C401`, `C402` and `C404` | **Corrected:** the harness recomputes stages 1–3 only (`check_message(..., meaning=False)`); stage-5 codes are judged against the reference run, like `C306` and `C403` |

Already conforming: the duplicate-reply resend (N2); unique names per kind and one receipt per
delivery, first in its cycle (N14); and commitment claims within the cycle (N7).

## 3. Resolved normative gaps and remaining conformance coverage

These came up while applying N1–N14. None of the approved rows covers them, so no normative text
was written. Each needs a decision before the bundle is frozen. The first three were decided by
decision record 0012 (INTEROP-2c) and are now in the specifications.

| Topic | Status | Note |
|---|---|---|
| D3: a move in progress when the lease expires | Decided by 0012 P1 and written in reference simulation 6.2 rule 3 and 6.4 | Corrected in the reference (D3, INTEROP-2d) |
| D6: two different messages with the same sequence | Decided by 0012 P2 and written in Core 8.2 and 8.4 | Corrected in the reference (D6, INTEROP-2d) |
| Stage 2: the order of `C204`, `C203` and `C202` for one signed object | Decided by 0012 P3 and written in Core 8.1. No fixture has more than one of these faults | The reference already checks `C204`, then `C203`, then `C202` (`core/receiver.py`, `_signature`). Multi-fault fixtures: as for stage 1 |
| Stage 1: three orderings without a fixture | Decided by 0011 N13 and written in Core 8.1. No fixture pins them: rank 2 before ranks 3–6; the order among ranks 3–6; `C103` before `C104` when both apply | Adding multi-fault fixtures to the conformance kit would make them testable (test data; separately authorized) |
| Stage 3: the order `C302`, `C301`, `C303`, `C307` | Decided by 0011 N13 and written in Core 8.1. Each stage-3 fixture has a single fault | As for stage 1 |
