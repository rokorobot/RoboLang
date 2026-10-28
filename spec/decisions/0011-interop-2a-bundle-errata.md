# Decision record 0011 — Gate 2 bundle errata: E2, E3, E5 and the amr-07 traceability findings (INTEROP-2a)

**Date:** 2026-10-28
**Status:** **PROPOSED.** Fourteen decisions (N1–N14) and seven reference-defect proposals (D1–D7)
for the owner. No implementation is authorized, no golden record changes, and no bundle is
frozen.
**Applies to:** [`spec/interop-gate-2-v0.1.md`](../interop-gate-2-v0.1.md) sections 3, 6.3–6.5;
[`spec/reference-simulation-v0.1.md`](../reference-simulation-v0.1.md) sections 4.3, 5, 6–8;
[`spec/core-v0.1.md`](../core-v0.1.md) sections 8.1 and 8.4 (clarifications only)
**Amends:** nothing yet. If these are approved, the specification text changes as each row says,
with an erratum note pointing here. Decision records 0007–0010 and their rows are not changed.
**Evidence:** [`docs/interop/amr-07-traceability-audit.md`](../../docs/interop/amr-07-traceability-audit.md)
(144 rows: 59 traced, 67 ambiguous, 18 missing). Row IDs below (M, R, H, L, C, S, T) refer to it.

## 1. Why

Gate 2 asks an independent author to write amr-07 from the bundle alone (0009, I1 and I3). The
INTEROP-1 self-test showed that the binding carries everything the reference needs. It did not
show that the specification says everything an author needs. The audit traced every behavior of
the reference amr-07 that shows on the wire, in receipts, in claims or in timing:

- **Missing:** 18 rules have no specification text.
- **Ambiguous:** 67 allow more than one behavior, or rest on a value, order or format the text
  does not fix.

An independent author would meet each of these as a question or a finding after the freeze. That
is the risk INTEROP-2 exists to remove.

**Principle used for every proposal:**

- **The reference is not a defect:** the erratum writes down what it does, so the 24 golden
  records stay valid.
- **The reference is a defect:** it contradicts approved text, or produces an invalid message.
  The defect is listed in section 4 for a fix under its own authorization (spec 9.3, class 2).
  It is never written into the specification.

## 2. Findings carried over from INTEROP-1

### 2.1 E2: the participant's sensors in the view

Spec 6.4 lists "its sensors (name and scope)". An observation (Core 5.6) also states
`valid_for`, `confidence` and the source's `method`, so a participant with only the name and
scope cannot build one. The INTEROP-1 view already gives the participant its own full sensor
records, minus the `participant` field.

### 2.2 E3: who sends the reply to a duplicate again

Core 8.4 (0008, Q5) says that when a duplicate arrives, "the reply sent to the original, if any, is
sent again". The binding does not say who sends it. Inside one process, the reference engine
re-delivers the stored reply documents itself:

- **What:** every message with `in_reply_to` equal to the duplicate's ID, including replies that
  were dropped the first time.
- **How:** byte-identical to the original.
- **When:** after one `latency`.
- **Recorded as:** a delivery of kind `resend`.
- **Faults:** none apply. The scripted message faults match the original send, and resends are not
  checked against partitions.

The INTEROP-1 harness does the same when the participant's receipt is `duplicate`.

### 2.3 E5: hold timers of a silent participant

| Path | What happens to a message the participant holds when it goes silent |
|---|---|
| Reference (one process) | The hold checks are engine timers with no owner, so they still fire during silence. At `expires_at − clock_tolerance`, the message is recorded `released`, `rejected C301`. At `max_hold`, it is recorded `released`, `not_handled`, the stream advances, and later held messages that become next are also released `not_handled` |
| External (INTEROP-1 binding) | The hold checks are the participant's own `set_timer` frames, owned by the participant. A silent participant's timers do not fire, so the hold never ends: no release receipt is recorded |

Neither the 24 cases nor any fixture puts a participant into silence while it holds a message,
so the golden records do not show the difference. Reference simulation 10 says a silent
participant "handles no message, sends nothing and starts no action". It does not say whether
the protocol's hold expiry continues.

**Alternatives:**

| | Rule | Consequences |
|---|---|---|
| **A (recommended; the owner's stated preference)** | Hold expiry is **protocol timing owned by the harness**, and it continues during silence. For every message the participant reported `held` before going silent, the harness ends the hold on the Core 8.4 schedule exactly as the reference does: `rejected C301` at expiry, or `not_handled` at `max_hold`, then later held messages of that stream released `not_handled` in sequence order. The silent participant receives no frame | Reference engine and golden records unchanged. The record shows every hold ending, consistent with 8.4's liveness bound. The harness must track held messages by sender and sequence from the participant's receipts and keep its own hold checks, which do nothing while the participant is not silent. No new frame |
| B | Silence stops everything, including hold expiry: a silent participant's holds never end | The reference engine changes (its hold checks would belong to the receiver). The 24 golden records would not change, since no case exercises this, but held messages could stay unresolved in a record forever, against the liveness purpose of `max_hold` (0008, Q4) |
| C | The harness keeps sending hold timers to a silent participant, which then releases them itself | Needs a new `set_timer` field to mark hold timers, and contradicts silence ("handles no message"). Rejected |

## 3. Decisions

| # | Topic | Proposal | Alternative considered | Spec | Golden records | Status |
|---|---|---|---|---|---|---|
| N1 | Sensors in the view (E2; M24, S2) | Spec 6.4, "own configuration": for each of its own sensors, `sensor` (name), `scope` (`own_deck` or `places`), `places` (list or `null`), `valid_for`, `confidence`, `method`. Never the lag or unavailability of a sensor (they are faults, applied by the harness), and never another participant's sensors. The INTEROP-1 view already does this | Give only name and scope, and publish the characteristics in the simulation spec: the view would then depend on a document the participant does not receive | G2 6.4 | Unchanged | PROPOSED |
| N2 | Reply to a duplicate (E3; R10) | Spec 6.3, `receipt` row. On `duplicate`, the participant sends nothing. **The harness repeats the original reply**: every message the participant sent with `in_reply_to` equal to the duplicate's ID, byte-identical, one `latency` after the duplicate, recorded as a delivery of kind `resend`. Faults are not applied again. The resent copy keeps its original ID and sequence, so it cannot create a new effect or a gap at its receiver (Core 8.4 idempotency is preserved). Core 8.4 gets a note: in the reference simulation and the binding, the transport repeats the reply | The participant sends the reply again itself: a new `send` would need a new sequence number, which breaks byte identity and creates a gap at the receiver | G2 6.3; Core 8.4 note | Unchanged | PROPOSED |
| N3 | Silence and hold timers (E5; R16) | **Alternative A of section 2.3.** Hold expiry is harness-owned protocol timing and continues during silence. The binding also states that a participant requests a hold end with `set_timer` at both instants of Core 8.4 (`expires_at − clock_tolerance` and delivery + `max_hold`), and answers that `timer` frame with the release receipt | B and C (section 2.3) | G2 6.2, 6.3; Sim 10 `silence` row | Unchanged. A new case (`seq_silent_while_held`) is recommended to exercise the rule; it would **add** a golden record, not change one | PROPOSED |
| N4 | `in_reply_to` per type (M13a–e) | Sim 6.2, new table. `offer_assistance` and `decline` reply to `request_assistance`. `acknowledge` replies to the message it acknowledges. `custody_state` replies to `custody_query`. `authorization_request`, `reserve`, `coordinate`, `custody_confirmed` and `outcome` carry `null`. N2 resends by this field, so it is not cosmetic | Leave it free: an author's choice would change which replies are resent | Sim 6.2 | Unchanged | PROPOSED |
| N5 | Formats and fixed values (M1, M4, M9, M16, M20, M29, M30) | **Message IDs:** amr-07 derives `message_id` by the formula of Sim 5.2. G2 3 adds Sim 5 to the sections that define amr-07's behavior. **Times:** every time amr-07 writes has exactly three fractional digits and `Z`. **Fixed values:** the `outcome` profile is `r2r`; every lease amr-07 proposes has `granted_at` equal to the send time; `evaluated_at` and `reported_at` equal the send time. **Outcome contents:** a `succeeded` outcome has empty `reasons`; any other outcome has `verification: null` and `evidence: []` | Leave the formats to Core 2.4 (one to three digits): equal content, but different signatures and a different observation signature, which the INTEROP-1 provenance check compares | G2 3; Sim 5, 6.1, 6.3 | Unchanged | PROPOSED |
| N6 | Observations (S4, M26) | Sim 4.3: a `sensed` with `value: null` does not use up an observation number. Readings are numbered in order, whether or not the observation is ever sent. amr-07 senses `carried_by` with `deck_camera` on `coordinate(placed)`, after acknowledging a release (rule 9) and on every `custody_query`, and `located_in` on arrival at the target. Nowhere else | — | Sim 4.3, 6.1 | Unchanged | PROPOSED |
| N7 | Claim schema (C2–C5, C8, C9, H35, L3, L11) | **Detail keys:** G2 6.3 fixes the `detail` of each event kind. `state` has `record` (`goal`, `custody`, `commitment`) and `state`. `authorization` has `decision` and `basis` (the standing authorization's ID or the decision's message ID). `custody_committed` has `thing` and `release`. `escalated` has `reason`. `lease_expired` is as in 0010. **When to claim:** every transition is claimed when it happens, in the order it happens; the commitment state is claimed at least at each change (0010, B2). **Diagnostic records:** the five `state` records `peer_ready`, `motion_refused`, `no_confirmation_evidence`, `physical_custody_unknown` and `reservation_denied` are listed as **optional diagnostics**. They are recorded, but neither required nor compared (9.1) | Require the diagnostics: it would turn reference debugging aids into protocol obligations | G2 6.3, 9.1 | Unchanged | PROPOSED |
| N8 | amr-07 input table (H1, H9, H17, H20, H21, H25–H27, H30, H32, H37, H40, R18, R19) | Sim 6.2, new table (section 5 below): for each input, the local state it needs, the code when it does not hold (`C403` is tested before `C306`), and the reaction. amr-07 never replies to a message it rejects. A message type with no row is `C306` | Leave this to RFC-001 10.4 and Core 8.2: neither fixes the order of checks or the silent reactions, so a reasonable author would differ in receipts or in sends | Sim 6.2 | Unchanged | PROPOSED |
| N9 | Rule 1 against V5 (H28, H36) | Sim 6.2: amr-07 sends `custody_confirmed` with its reading **without testing its freshness or the `criteria`**; the giver judges (V5 depends on it). For rule 9, a "fresh" observation means any reading the sensor returns. This is recorded as a simulation simplification of RFC-001 10.4 rules 1 and 9, not a change to them | Make amr-07 test its own evidence first: V5 would no longer show the giver's check, and its golden record would change | Sim 6.2, V5 | Unchanged | PROPOSED |
| N10 | Values derived from configuration (H2–H4, H7, H13–H15, H44) | **The sub-goal:** the thing and the target are the first triple of `desired_state`. **The hand-off place:** the key of `arbiters` whose value is the requester. **Mass:** compare the stated mass value only (`uncertainty` is not added); every limit is inclusive (`≤`). **Places involved:** the hand-off place and the target, each in scope if it, or its direct `within`, is listed. **The counterpart:** the sender of `request_assistance`. **`any_participant_in_domain`:** the participants whose `authority_domain` is the evaluator's. **Capability test:** the descriptor is current, its signature is valid, and `transport_payload` and `dock_at` are present and not unsupported; mass ≤ `max_mass`; both places are in `dock_at.places`. **Standing authorization:** checked as in Sim 7.2, with `granted_at ≤ now < expires_at − clock_tolerance` | — | Sim 6.2, 7.2; Core 5.3 | Unchanged | PROPOSED |
| N11 | Evaluation and result mapping (H43, H45) | Sim 6.3 gets the algorithm, in this order for each criterion. **Relevant observations:** those from an accepted source, with the criterion's subject and property; none gives `missing`. **Fresh enough:** age = `now − observed_at + clock_tolerance`, which must be ≤ both `max_age` and `valid_for`; no relevant observation fresh enough gives `stale`. **Met:** a fresh-enough observation with the criterion's value and `confidence` ≥ `min_confidence`; otherwise `contradicted` (a right value at too low a confidence counts as contradicted). **Mapping:** all met gives `succeeded`; the first unmet criterion gives `unknown` (`evidence_missing`) for missing, `unknown` (`evidence_stale`) for stale, and `failed` for contradicted (reasons: see D1) | — | Sim 6.3 | Unchanged | PROPOSED |
| N12 | Lease expiry (L1–L8, L10) | Sim 6.2 rule 3, new table. **The timer:** amr-07's commitment-lease timer fires at exactly `lease.expires_at` (the clock tolerance is not applied to one's own timer; RFC-001 11.4 applies it to judging others' times). **When nothing happens:** the timer does nothing if the commitment has already ended. **The branches:** custody `ACKNOWLEDGED`, `ACCEPTED` or `CONFIRMATION_SENT` leads to custody `TIMED_OUT`, `escalated` (`custody_unknown`), goal `FAILED` and outcome `failed` (`lease_expired`, `custody_unknown`); this is V5, and also V6 for amr-07, which Sim V6 gets a sentence for. `HOLDING` leads to goal `UNKNOWN` and outcome `unknown` (`lease_expired`), with no escalation. Any other state leads to goal `FAILED` and outcome `failed` (`lease_expired`). **No reservation claim:** amr-07 claims `lease_expired` for its commitment only | Apply the tolerance early: the INTEROP-1 harness would reject the `lease_expired` claim (0010, B1), and V5 and V6 would change | Sim 6.2, V5, V6; G2 6.3 | Unchanged | PROPOSED |
| N13 | Receiver details (R2–R6, R9, R14, R20) | **Order of checks:** Core 8.1 states the exact order within stages 1 and 3 as the conformance kit applies it: within stage 3, `C302`, `C301`, `C303`, `C307`. **Stage 2:** checks each embedded observation's signature after the message's. **Expiry:** a message is expired when `now ≥ expires_at − clock_tolerance`, and in the future when `sent_at > now + clock_tolerance`. **Duplicates:** "processed" means handed over and accepted, so a message rejected at stage 5 does not make a later copy a duplicate. **Ties:** expiry wins a tie with `max_hold`. **Conversation:** in 0.1 a receiver does not compare `conversation_id` | — | Core 8.1, 8.4 | Unchanged | PROPOSED |
| N14 | Binding mechanics (R17, T1, T7) | **Receipts:** exactly one `receipt` per `deliver`, and it is the first `receipt` of that cycle (receipts for released messages follow it). `code` is `null` unless `rejected`. For data that cannot be decoded, `message_id` and `sender` are `null`. **Names:** `timer`, `action` and `request` names are unique within their kind for the run. **Order:** in a cycle, only these orders are compared: the delivered message's receipt before any release receipt, sends in the order written, and claims in the order of N7 | — | G2 6.3, 6.5 | Unchanged | PROPOSED |

## 4. Reference defects (spec 9.3, class 2)

The reference does something the specification forbids, or could not reasonably intend. None of
them is reached by the 24 cases, so the proposal is to fix each in the reference under a separate
authorization. The golden test then shows the records unchanged. Each fix would add its own case,
as an **added** golden record.

| # | Defect (audit row) | Proposed fix | Golden records |
|---|---|---|---|
| D1 | A `contradicted` criterion produces an outcome `failed` with `reasons: [{"code": null}]`, an invalid payload (H46) | `failed` from a contradicted criterion has empty `reasons` (Core 5.9: "may be empty"); Core 5.11 is not extended | Unchanged (not reached) |
| D2 | A `custody_query` before any `request_assistance` raises an error instead of a receipt (H41) | `C306` | Unchanged (not reached) |
| D3 | If the lease expires during the final move, arrival sets the commitment `fulfilled` over `lapsed`, and the outcome follows the lapse outcome (H47) | An ended commitment stays ended; arrival after expiry sends nothing | Unchanged (not reached) |
| D4 | Any capability failure is declined `capability_insufficient`, where Core 5.11 says `capability_unsupported` when the capability is unsupported or absent (M18) | Use `capability_unsupported` in that case | Unchanged (not reached) |
| D5 | Before a move, Sim 6.2 rule 2 requires an unexpired lease and an unexpired reservation; the reference checks only that the goal is `RUNNING` and the commitment `active` (H23) | Check both expiries at the moment of the `act` | Expected unchanged: the latest amr-07 move in the 24 cases starts at 10:14:51.200 (measured), well before `reservation_end` (10:20) and the commitment lease end (10:25). To be confirmed by the golden test |
| D6 | A second, different message with the same sequence as a held one silently replaces it, and the first never gets a release receipt (R15) | The second is rejected `C306` (an equivocating message) | Unchanged (not reached) |
| D7 | `C401`, `C402` and `C404` are applied before duplicate and order handling, contrary to Core 8.1 (stage 5 after stage 4) and 0008, Q2 (R7) | Apply them after stage 4, as Core 8.1 says; the conformance kit (stateless) is unaffected | Unchanged: the only such rejection in the 24 cases is v10's `C401`, and nothing follows it in that stream (measured) |

## 5. Proposed amr-07 input table (N8)

| Input | Required local state | Otherwise | Reaction |
|---|---|---|---|
| `request_assistance` | no sub-goal yet | `C306` | `ASSESSING`; policy, then capability (N10); `offer_assistance` or `decline` |
| `commit` | goal `ASSESSING`; debtor is amr-07; obligation names the sub-goal | `C306` | commitment `active`; goal `ACCEPTED`, then `AUTHORIZING`; lease timer (N12); standing authorization, or `authorization_request` |
| `authorization_decision` | goal `AUTHORIZING`; payload names the sub-goal | `C306` (`C401` from anyone but its authority, Core) | `granted`: `AUTHORIZED`, then `reserve`. `denied`: `DENIED`, commitment `failed`, outcome `rejected` (`authorization_denied`) |
| `reservation_decision` | goal `AUTHORIZED`; resource is the hand-off place | `C306` | `granted`: `RUNNING`, then move to the hand-off place. `denied`: diagnostic `reservation_denied` only; nothing is sent; the lease expiry decides (N12) |
| `coordinate` `ready_for_handover` | any | — | diagnostic `peer_ready` only; nothing is sent |
| `coordinate` `placed` | custody `ACCEPTED` | `C306` | sense `carried_by`. A reading: `custody_confirmed`, then `CONFIRMATION_SENT`. `null`: diagnostic `no_confirmation_evidence` only |
| `transfer_custody` | goal `RUNNING`; payload names the commitment and the thing; custody not yet set | `C306` | `ACKNOWLEDGED`, then `acknowledge` (`accepted: true`), then `ACCEPTED` |
| `custody_released` | names amr-07's `custody_confirmed` and the thing (else `C403`, tested first); custody `CONFIRMATION_SENT` | `C306` | `HOLDING`, then `custody_committed`, then `acknowledge`; sense `carried_by` (rule 9). A reading: move to the target. `null`: diagnostic `physical_custody_unknown`; no motion |
| `custody_query` | a sub-goal exists (D2); payload names the thing | `C306` | sense `carried_by`; `custody_state` (accepted release if `HOLDING`, observations if any) |
| any other type | — | `C306` | — |

## 6. Consequences

- **Golden records:**
  - N1–N14 write down the reference's behavior: **no golden record changes**.
  - N3, and each fix in section 4, would **add** a case and its golden record. The 24 existing
    records stay byte-identical (for D5 and D7 measured, to be confirmed by the golden test).
- **INTEROP-1 harness:**
  - N3 needs harness-side hold checks for a silent participant.
  - N14's name uniqueness per kind is already implemented.
  - Nothing else changes.
- **Bundle:** after approval, the specification text is updated per row (erratum notes pointing
  here), and the frozen bundle, manifest, example view and conformance instructions follow as the
  rest of INTEROP-2, under its own authorization.
- **Residual risk:** the audit traced the reference amr-07 only. planner-1, arm-02 and the
  authority stay reference-only in gate 2 (0009, I11), so their rules are not part of the
  bundle's obligations.

## 7. Revision history

| Date | Change |
|---|---|
| 2026-10-28 | First draft: N1–N14 and D1–D7 proposed (INTEROP-2a) |
