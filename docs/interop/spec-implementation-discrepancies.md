# Specification–implementation discrepancy register (INTEROP-2b-SPEC)

**Date:** 2026-10-28. **Specifications:** as amended by
[decision record 0011](../../spec/decisions/0011-interop-2a-bundle-errata.md) (N1–N14, approved
2026-10-28) and [decision record 0012](../../spec/decisions/0012-interop-2c-remaining-normative-decisions.md)
(P1–P3, approved 2026-10-28). **Code compared:** `main` at `6d22085`.

The specifications now state the approved behavior. Where the reference implementation
(`robolang.arch02`) or the INTEROP-1 harness (`robolang.interop`) does not yet follow them, the
difference is listed here, so that it is visible rather than hidden. **Nothing here is fixed by
this register**: each fix needs its own approved scope (0011, section 4). None of these
differences is reached by the 24 cases, so the golden records do not show them, and every fix
must leave the 24 golden records unchanged.

The register is reviewer material and cites `src/`. It is not part of the gate 2 bundle.

## 1. Reference defects (0011, section 4; spec 9.3, class 2)

| # | Specification now says | The reference currently does | Expected correction | Reached by the 24 cases |
|---|---|---|---|---|
| D1 | A `contradicted` criterion gives an outcome `failed` with empty `reasons` (reference simulation 6.3, N11) | `reasons: [{"code": null}]`, an invalid payload. amr-07: `participants.py:619`. arm-02 does the same in its own outcome (`:428`). planner-1 sends no message here, but records `reason: null` in its `planner_goal` event (`:157`) | Empty `reasons` in amr-07's and arm-02's outcomes. planner-1's record is not a message, and is noted only | No |
| D2 | A `custody_query` before any `request_assistance` is rejected `C306` (reference simulation 6.4, N8) | Raises an error: no thing is known yet (`participants.py:621`) | Test that a sub-goal exists first; `C306` | No |
| D3 | An action in progress at lease expiry completes; when it completes after the lapse, amr-07 changes no state, senses nothing and sends nothing (reference simulation 6.4, 0012 P1) | If the commitment lease expires during the final move, arrival sets the commitment `fulfilled` over `lapsed` and sends a second outcome (`participants.py:603-619`) | As the specification says (INTEROP-2d) | No |
| D4 | Capability failure codes: `capability_unsupported` for an absent, non-current or invalidly signed descriptor, or a kind absent or listed unsupported; otherwise `capability_insufficient` (reference simulation 6.4, N10; Core 5.11) | `capability_insufficient` in every case (`participants.py:494`) | Two codes, as the specification says | No (only V1's policy decline is exercised) |
| D5 | Before every move: authorization still established; goal `RUNNING`; commitment `active` with its lease unexpired; a granted reservation unexpired; judged in the safe direction (reference simulation 6.2 rule 2 and 6.4) | Checks only the goal state and the commitment state (`participants.py:558`) | All four checks at the moment of the `act` | No (measured: the latest amr-07 move starts at 10:14:51.200, before `reservation_end` 10:20) |
| D6 | A different message with the sequence of a message held in the same stream is rejected `C306`; the held message keeps its place and gets one terminal release receipt (Core 8.4, 0012 P2) | A second, different message with the same sequence as a held one replaces it, and the first never gets a release receipt (`engine.py:167`) | As the specification says (INTEROP-2d) | No |
| D7 | `C401`, `C402` (unmet criterion) and `C404` are applied after duplicate and order handling (Core 8.1, N13) | Applied together with stages 1–3, before stage 4 (`core/receiver.py:230`, used by the reference inbox) | Apply them after stage 4. The conformance kit is unaffected. Regression tests: a stage-5 rejection after a sequence gap, and a duplicate of a stage-5-rejected message | No (measured: the only such rejection is v10's `C401`, and nothing follows it in that stream) |

## 2. INTEROP-1 harness (0011, section 6; recognized follow-up work)

| # | Specification now says | The harness currently does | Expected correction |
|---|---|---|---|
| N3 | Silence: the harness takes over open holds through a hold ledger, with exactly one terminal release receipt per held message (gate 2 6.2) | The participant's hold timers are owned by the participant and skipped while it is silent; there is no ledger or takeover | Hold ledger and harness-side hold checks; a recommended new case, `seq_silent_while_held` (an **added** golden record) |
| H1 | Presented observations are compared **in meaning**, with a valid signature; formatting alone is not a failure (gate 2 9.1.1, N5) | Compares the whole document, signature value included (`harness.py:296`): stricter than specified, so it is safe for the self-test but would fail a correct implementation that formats times differently | Comparison by meaning plus signature verification; the four INTEROP-1 provenance tests must still fail |
| H2 | Diagnostics are optional, but each must be consistent with the same cycle; any other `record` value is a binding failure (gate 2 6.3, 9.1.1, N7) | Records every `state` claim without checking diagnostics or unknown `record` values (`harness.py:253`) | Check the five diagnostics; reject unknown records |
| N13(c) | Only stage 1–3 codes are compared with the harness's recomputation (gate 2 9.1.1) | Compares every code that `check_message` gives, including the stage-5 `C401`, `C402` and `C404` (`harness.py:200`) | Compare by recomputed stage; together with D7 |

Already conforming: the duplicate-reply resend (N2); unique names per kind and one receipt per
delivery, first in its cycle (N14); and commitment claims within the cycle (N7).

## 3. Rules the specifications do not yet state

These came up while applying N1–N14. None of the approved rows covers them, so no normative text
was written. Each needs a decision before the bundle is frozen. The first three were decided by
decision record 0012 (INTEROP-2c) and are now in the specifications.

| Topic | Status | Note |
|---|---|---|
| D3: a move in progress when the lease expires | Decided by 0012 P1 and written in reference simulation 6.2 rule 3 and 6.4 | The reference correction is D3 in section 1 |
| D6: two different messages with the same sequence | Decided by 0012 P2 and written in Core 8.2 and 8.4 | The reference correction is D6 in section 1 |
| Stage 2: the order of `C204`, `C203` and `C202` for one signed object | Decided by 0012 P3 and written in Core 8.1. No fixture has more than one of these faults | The reference already checks `C204`, then `C203`, then `C202` (`core/receiver.py`, `_signature`). Multi-fault fixtures: as for stage 1 |
| Stage 1: three orderings without a fixture | Decided by 0011 N13 and written in Core 8.1. No fixture pins them: rank 2 before ranks 3–6; the order among ranks 3–6; `C103` before `C104` when both apply | Adding multi-fault fixtures to the conformance kit would make them testable (test data; separately authorized) |
| Stage 3: the order `C302`, `C301`, `C303`, `C307` | Decided by 0011 N13 and written in Core 8.1. Each stage-3 fixture has a single fault | As for stage 1 |
