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
| N3 | Silence and hold timers (E5; R16) | **Alternative A of section 2.3**, with the ownership rules of section 3.1. Hold expiry is protocol timing; the participant owns it while it is live, and the harness takes it over when the participant goes silent. Each held message gets exactly one terminal release receipt, and expiry wins a tie | B and C (section 2.3) | G2 6.2, 6.3; Sim 10 `silence` row | Unchanged. A new case (`seq_silent_while_held`) is recommended to exercise the rule; it would **add** a golden record, not change one | PROPOSED (revised) |
| N4 | `in_reply_to` per type (M13a–e) | Sim 6.2, new table. `offer_assistance` and `decline` reply to `request_assistance`. `acknowledge` replies to the message it acknowledges. `custody_state` replies to `custody_query`. `authorization_request`, `reserve`, `coordinate`, `custody_confirmed` and `outcome` carry `null`. N2 resends by this field, so it is not cosmetic | Leave it free: an author's choice would change which replies are resent | Sim 6.2 | Unchanged | PROPOSED |
| N5 | Values and formats (M1, M4, M9, M16, M20, M29, M30) | Split into **normative values**, required of any implementation, and **reference serialization**, which is not required and is measured under 9.2. Details in section 3.2. The harness's comparison of observations moves from document equality to equality of meaning plus a valid signature: a harness refinement (H1), separately authorized | Require exactly three fractional digits in the binding (the first draft): it would make gate 2 test the reference's serialization, not interoperability | G2 3, 9.1.1, 9.2; Sim 5, 6.1, 6.3 | Unchanged | PROPOSED (revised) |
| N6 | Observations (S4, M26) | Sim 4.3: a `sensed` with `value: null` does not use up an observation number. Readings are numbered in order, whether or not the observation is ever sent. amr-07 senses `carried_by` with `deck_camera` on `coordinate(placed)`, after acknowledging a release (rule 9) and on every `custody_query`, and `located_in` on arrival at the target. Nowhere else | — | Sim 4.3, 6.1 | Unchanged | PROPOSED |
| N7 | Claim schema (C2–C5, C8, C9, H35, L3, L11) | Three distinct variants of the `state` claim, each with its own `detail` and timing: **transition claims** (goal, custody), **commitment claims** and **diagnostics**. The other kinds (`authorization`, `custody_committed`, `escalated`, `lease_expired`) get fixed `detail` keys. Optional diagnostics are not required or compared, but must be consistent with what the harness observed. Details in section 3.3 | Keep the diagnostics as ordinary `state` records (the first draft): one schema would cover two different things. Or require the diagnostics: reference debugging aids would become protocol obligations | G2 6.3, 9.1, 9.1.1 | Unchanged | PROPOSED (revised) |
| N8 | amr-07 input table (H1, H9, H17, H20, H21, H25–H27, H30, H32, H37, H40, R18, R19) | Sim 6.2, new table (section 5 below): for each input, the local state it needs, the code when it does not hold (`C403` is tested before `C306`), and the reaction. amr-07 never replies to a message it rejects. A message type with no row is `C306` | Leave this to RFC-001 10.4 and Core 8.2: neither fixes the order of checks or the silent reactions, so a reasonable author would differ in receipts or in sends | Sim 6.2 | Unchanged | PROPOSED |
| N9 | Rule 1 against V5 (H28, H36) | A **simulation-specific forwarding rule**, worded in section 3.4. In the reference simulation and gate 2 only, amr-07 forwards its sensor reading without a local freshness or criteria check, and the giver performs the verification RFC-001 10.4 specifies. The rule does not define evidence freshness, does not amend RFC-001 10.4 rules 1 or 9, and is not a pattern for physical deployments | Make amr-07 test its own evidence first: V5 would no longer show the giver's check, and its golden record would change. Or call any returned reading "fresh" (the first draft): it reads as a general freshness rule | Sim 6.2, V5 | Unchanged | PROPOSED (revised) |
| N10 | Values derived from configuration (H2–H4, H7, H13–H15, H44) | **The sub-goal:** the thing and the target are the first triple of `desired_state`. **The hand-off place:** the key of `arbiters` whose value is the requester. **Mass:** compare the stated mass value only (`uncertainty` is not added); every limit is inclusive (`≤`). **Places involved:** the hand-off place and the target, each in scope if it, or its direct `within`, is listed. **The counterpart:** the sender of `request_assistance`. **`any_participant_in_domain`:** the participants whose `authority_domain` is the evaluator's. **Capability test:** the descriptor is current, its signature is valid, and `transport_payload` and `dock_at` are present and not unsupported; mass ≤ `max_mass`; both places are in `dock_at.places`. **Standing authorization:** checked as in Sim 7.2, with `granted_at ≤ now < expires_at − clock_tolerance` | — | Sim 6.2, 7.2; Core 5.3 | Unchanged | PROPOSED |
| N11 | Evaluation and result mapping (H43, H45) | Sim 6.3 gets the algorithm, in this order for each criterion. **Relevant observations:** those from an accepted source, with the criterion's subject and property; none gives `missing`. **Fresh enough:** age = `now − observed_at + clock_tolerance`, which must be ≤ both `max_age` and `valid_for`; no relevant observation fresh enough gives `stale`. **Met:** a fresh-enough observation with the criterion's value and `confidence` ≥ `min_confidence`; otherwise `contradicted` (a right value at too low a confidence counts as contradicted). **Mapping:** all met gives `succeeded`; the first unmet criterion gives `unknown` (`evidence_missing`) for missing, `unknown` (`evidence_stale`) for stale, and `failed` for contradicted (reasons: see D1) | — | Sim 6.3 | Unchanged | PROPOSED |
| N12 | Lease expiry (L1–L8, L10) | Sim 6.2 rule 3, new table. **The timer:** amr-07's commitment-lease timer fires at exactly `lease.expires_at` (the clock tolerance is not applied to one's own timer; RFC-001 11.4 applies it to judging others' times). **When nothing happens:** the timer does nothing if the commitment has already ended. **The branches:** custody `ACKNOWLEDGED`, `ACCEPTED` or `CONFIRMATION_SENT` leads to custody `TIMED_OUT`, `escalated` (`custody_unknown`), goal `FAILED` and outcome `failed` (`lease_expired`, `custody_unknown`); this is V5, and also V6 for amr-07, which Sim V6 gets a sentence for. `HOLDING` leads to goal `UNKNOWN` and outcome `unknown` (`lease_expired`), with no escalation. Any other state leads to goal `FAILED` and outcome `failed` (`lease_expired`). **No reservation claim:** amr-07 claims `lease_expired` for its commitment only | Apply the tolerance early: the INTEROP-1 harness would reject the `lease_expired` claim (0010, B1), and V5 and V6 would change | Sim 6.2, V5, V6; G2 6.3 | Unchanged | PROPOSED |
| N13 | Receiver details (R2–R7, R9, R14, R20) | Kept in three separate parts (section 3.5). **(a) The normative stage order of Core 8.1 is unchanged:** stages 1–3, then duplicate and order, then meaning, including `C401`, `C402` (the stage-5 case) and `C404`. The reference's earlier application of those codes is defect D7 and is not made normative. **(b) Clarifications within stages and of 8.4.** **(c) What the harness compares:** stage 1–3 codes only | Write down the reference's order (the first draft's risk): it would make D7 normative by accident | Core 8.1, 8.4; G2 9.1.1 | Unchanged | PROPOSED (revised) |
| N14 | Binding mechanics (R17, T1, T7) | **Receipts:** exactly one `receipt` per `deliver`, and it is the first `receipt` of that cycle (receipts for released messages follow it). `code` is `null` unless `rejected`. For data that cannot be decoded, `message_id` and `sender` are `null`. **Names:** `timer`, `action` and `request` names are unique within their kind for the run. **Order:** in a cycle, only these orders are compared: the delivered message's receipt before any release receipt, sends in the order written, and claims in the order of N7 | — | G2 6.3, 6.5 | Unchanged | PROPOSED |

### 3.1 N3: ownership of hold expiry

1. **While the participant is live,** it owns its holds: it requests a `set_timer` at both
   Core 8.4 instants of each held message (`expires_at − clock_tolerance` and delivery +
   `max_hold`), and answers the `timer` frame with the release receipt, if any is due.
2. **The harness keeps a hold ledger** from the participant's receipts. A `held` receipt opens an
   entry (sender, sequence, message ID, `expires_at`, delivery time). The entry is **closed** by the
   first terminal release receipt for that message: `accepted`, `rejected` (with its code) or
   `not_handled`. For each entry the harness also schedules its own two checks at the same
   instants. While the participant is live, these checks do nothing.
3. **Takeover.** Silence starts at the delivery named by the `silence` fault, including it
   (Sim 10). From that moment:
   - the participant gets no frame, and none of its timers fires, hold timers included;
   - the harness's checks act on every entry still open. At `expires_at − clock_tolerance`, the
     entry is closed with `released`, `rejected C301`. At delivery + `max_hold`, it is closed with
     `released`, `not_handled`, and the missing sequence numbers are skipped;
   - after either, open entries of the same stream that are now next are closed `released`,
     `not_handled`, in sequence order, at the same instant. This is Core 8.4's ordering, applied
     by the harness to the ledger exactly as the reference receiver applies it.
4. **Exactly one terminal receipt.** A closed entry is removed, so a later check finds nothing.
   A participant release receipt for a message with no open entry is an inconsistent claim
   (9.1.1), as INTEROP-1 already checks.
5. **Ties.** When the expiry instant and the `max_hold` instant coincide, expiry wins (`C301`),
   as in N13(b).
6. **No handback.** Silence has no end in 0.1 (Sim 10 gives only `from`), so the harness never
   hands holds back. A later version that ends silence needs its own rule.

### 3.2 N5: normative values and reference serialization

**Normative**, required of any implementation and checked by the harness:

- **Fixed values:**
  - the `outcome` profile is `r2r`;
  - every lease amr-07 proposes has `granted_at` at the send time;
  - `evaluated_at` and `reported_at` are the send time.
- **Outcome contents:** a `succeeded` outcome has empty `reasons`; any other outcome has
  `verification: null` and `evidence: []`.
- **Times are compared as instants:** `…T10:14:00.1Z` and `…T10:14:00.100Z` are the same time.

**Reference serialization**, not required, measured and reported under 9.2 (JCS identity):

- the reference writes every time with three fractional digits and `Z`;
- it derives `message_id` with the formula of Sim 5.2.

Core requires only that an ID is unique per sender (Core 3), and the harness's criteria (9.1)
never compare IDs. An implementation that formats or numbers differently is interoperable;
the difference is a 9.2 finding, not a failure.

**Harness refinement H1** (separately authorized, before INTEROP-4). The INTEROP-1 provenance check
compares a presented observation with the harness's copy as documents. H1 replaces that with a
comparison by meaning, which a semantically equal observation always passes:

- the same `id`, `subject`, `property` and `value`;
- the same instant for `observed_at`;
- the same `valid_for` as a duration;
- the same `confidence` as a decimal;
- the same `source`;
- a `sim-hmac-sha256` signature that verifies under the participant's key over the observation as
  presented.

The four INTEROP-1 negative tests keep failing, because each alters a value. Formatting
alone stops being a failure. Until H1 is in place, the INTEROP-1 check stays stricter than this
decision, which is safe for the self-test.

### 3.3 N7: claim variants

| Variant | `kind` / `record` | `detail` | When | Required |
|---|---|---|---|---|
| Goal transition | `state` / `goal` | `record`, `state` (RFC-001 9.2 name) | At the moment of the transition, in the order the transitions happen | Yes: every transition |
| Custody transition | `state` / `custody` | `record`, `state` (RFC-001 10.4 name) | Same | Yes: every transition |
| Commitment state | `state` / `commitment` | `record`, `state` (Core 5.8 local state) | Any time within the cycle in which the change happened, before `idle`. The reference claims it at the end of the cycle | Yes: every change (0010, B2) |
| Diagnostic | `state` / one of the five names below | `record`, plus the listed fields | When the participant notes it | **No**: optional, not compared |
| Authorization | `authorization` | `decision` (`granted` or `denied`), `basis` (the standing authorization's ID or the decision's message ID) | When decided | Yes |
| Commit point | `custody_committed` | `thing`, `release` (the `custody_released` message ID) | Before the `acknowledge` (RFC-001 10.4 rule 3) | Yes |
| Escalation | `escalated` | `reason` (a Core 5.11 code) | When escalating | Yes |
| Lease expiry | `lease_expired` | as in 0010, B1 | At expiry | Yes |

**The diagnostics.** An optional diagnostic is never required, but what it asserts must hold, or
the claim is inconsistent (9.1.1). Each one is checked against the cycle it is written in:

| `record` | Fields | Must be consistent with |
|---|---|---|
| `peer_ready` | — | An accepted `coordinate` `ready_for_handover` |
| `reservation_denied` | — | An accepted `reservation_decision` with `denied` |
| `motion_refused` | `to` | No `act` |
| `no_confirmation_evidence` | — | A `sensed` with `value: null` |
| `physical_custody_unknown` | — | A `sensed` with `value: null` after the commit point |

A `state` claim with any other `record` is a binding failure. Checking diagnostics against the
cycle is a harness refinement (**H2**), separately authorized; INTEROP-1 records them without
checking.

### 3.4 N9: the forwarding rule (proposed text for Sim 6.2)

> **Simulation-specific forwarding (reference simulation and gate 2 only).** amr-07 forwards its
> sensor reading in `custody_confirmed` without checking its freshness or the transfer's
> `criteria` itself; the giver verifies it as RFC-001 10.4 specifies (V5 tests this). After
> accepting a release, amr-07 proceeds on any reading its sensor returns for rule 9. This rule
> does not define evidence freshness, does not amend RFC-001 10.4 rules 1 and 9, and is not a
> pattern for physical deployments, where those rules apply as written.

### 3.5 N13: receiver stages, clarifications and harness comparison

**(a) Normative stage order.** Core 8.1, unchanged:

1. form;
2. signature;
3. freshness, then recipient;
4. duplicate and order;
5. meaning: `C306`, `C401`, `C402` for an outcome whose criteria are not all met, `C403` and `C404`.

The reference applies `C401`, `C402` and `C404` before stage 4: that is defect **D7**, to be fixed
in the reference, and it is not normative. The form-stage `C402` that Core 8.1 names (an outcome
`succeeded` with `verification: null`) stays in stage 1.

**(b) Clarifications.** These are new normative text, chosen where Core is silent, not copied from
the reference by default.

- **Order within a stage:**
  - Core 8.1 lists codes, not an order inside a stage. The conformance kit (G-T0) compares exact
    codes, so the order is fixed.
  - Stage 3: `C302` (lifetime), `C301` (expired), `C303` (future), then `C307`.
  - Stage 1: the rank listed in the audit, row R3.
- **Stage 2:** checks each embedded observation's signature after the message's.
- **Time comparisons:** a message is expired when `now ≥ expires_at − clock_tolerance`, and in
  the future when `sent_at > now + clock_tolerance`.
- **Core 8.4:**
  - "processed" means handed over and accepted, so a message rejected at stage 5 does not make a
    later copy a duplicate;
  - expiry wins a tie with `max_hold`;
  - in 0.1 a receiver does not compare `conversation_id`.

**(c) What the harness compares.** 9.1.1 recomputes stages 1–3 only. The INTEROP-1 harness uses the
reference's `check_message`, which also applies the stateless stage-5 codes. After D7, it must
compare only receipts whose recomputed stage is form, signature or freshness. Otherwise it
would demand `C401` where a correct receiver reports `duplicate` first. This is part of the D7 fix.

## 4. Reference defects (spec 9.3, class 2)

The reference does something the specification forbids, or could not reasonably intend. None of
them is reached by the 24 cases. Each is a **candidate** for a fix in the reference, and each fix
needs its own approved scope; none is authorized by this record. Within such a scope, the golden
test must show the 24 records unchanged, and any new case is an **added** golden record. The
behavior of a defect is never written into the specification.

| # | Defect (audit row) | Proposed fix | Golden records |
|---|---|---|---|
| D1 | A `contradicted` criterion produces an outcome `failed` with `reasons: [{"code": null}]`, an invalid payload (H46) | `failed` from a contradicted criterion has empty `reasons` (Core 5.9: "may be empty"); Core 5.11 is not extended | Unchanged (not reached) |
| D2 | A `custody_query` before any `request_assistance` raises an error instead of a receipt (H41) | `C306` | Unchanged (not reached) |
| D3 | If the lease expires during the final move, arrival sets the commitment `fulfilled` over `lapsed`, and the outcome follows the lapse outcome (H47) | **A move already in progress at expiry.** The simulation does not model stopping a robot, so the harness completes the move, and its motion and effect facts stand. On arrival, amr-07 changes no state and sends nothing: the commitment stays `lapsed`, and the goal and outcome stay as the expiry branch set them (N12, HOLDING: `unknown`). **No new move** starts after expiry (D5). The alternative is to stop the move at expiry, which would need a new world action and is not proposed for 0.1 | Unchanged (not reached) |
| D4 | Any capability failure is declined `capability_insufficient`, where Core 5.11 says `capability_unsupported` when the capability is unsupported or absent (M18) | Two codes. **`capability_unsupported`:** the descriptor is absent, not current or not validly signed, or a needed kind (`transport_payload`, `dock_at`) is listed as unsupported or not at all (Core 5.11). **`capability_insufficient`:** the kinds are supported, but the mass is over `max_mass` or a place is outside `dock_at.places` | Unchanged (not reached) |
| D5 | Before a move, Sim 6.2 rule 2 requires an unexpired lease and an unexpired reservation; the reference checks only that the goal is `RUNNING` and the commitment `active` (H23) | At the moment of each `act`, all of these must hold, or the move is refused: the authorization is still established (the standing authorization's lease is unexpired, or a granted decision was accepted); the goal is `RUNNING`; the commitment is `active` and its lease unexpired; and a granted reservation is unexpired. The judgement is made in the safe direction (`now < expires_at − clock_tolerance`) | Expected unchanged: the latest amr-07 move in the 24 cases starts at 10:14:51.200 (measured), well before `reservation_end` (10:20) and the commitment lease end (10:25). To be confirmed by the golden test |
| D6 | A second, different message with the same sequence as a held one silently replaces it, and the first never gets a release receipt (R15) | The second is rejected `C306` (an equivocating message) | Unchanged (not reached) |
| D7 | `C401`, `C402` and `C404` are applied before duplicate and order handling, contrary to Core 8.1 (stage 5 after stage 4) and 0008, Q2 (R7) | Apply them after stage 4, as Core 8.1 says (N13(a)). The conformance kit is stateless and unaffected. The harness compares stage 1–3 codes only (N13(c)). Regression tests are needed for a stage-5 rejection with a sequence gap, and for a duplicate of a stage-5-rejected message | Unchanged: the only such rejection in the 24 cases is v10's `C401`, and nothing follows it in that stream (measured) |

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
- **INTEROP-1 harness**, each change separately authorized:
  - N3: harness-side hold checks for a silent participant.
  - **H1** (N5): observations compared by meaning and signature.
  - **H2** (N7): diagnostics checked against the cycle, and unknown `record` values rejected.
  - N13(c): only stage 1–3 codes compared, with D7.
  - N14's name uniqueness per kind is already implemented.
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
| 2026-10-28 | Revision after the owner's review of PR #38 at `2eaddbc`; all rows still proposed. **N3:** ownership, takeover, the hold ledger, exactly one terminal receipt, expiry wins a tie (3.1). **N5:** normative values split from reference serialization; harness refinement H1 (3.2). **N7:** three `state` variants; diagnostics optional but corroborated; H2 (3.3). **N9:** reworded as a simulation-only forwarding rule (3.4). **N13:** normative stage order kept apart from D7; clarifications; the harness compares stages 1–3 only (3.5). **D3, D4, D5, D7:** fixes made precise. D1–D7 are candidates only: none authorized, no golden record added |
