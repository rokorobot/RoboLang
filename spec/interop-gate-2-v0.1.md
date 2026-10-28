# RL-INTEROP-01 — Gate 2: independent interoperability, version 0.1 (draft)

**Status:** specification, **approved by the owner on 2026-10-27**. Every choice is a row in
[decision record 0009](decisions/0009-gate-2-independent-interoperability.md) (I1–I12),
approved 2026-10-27. Each slice of section 11 needs its own authorization; section 13 gives
their status.
**Gate:** Gate 2 — independent implementation ([RFC-001 section 1.4](architecture/rfc-001-ai-native-architecture.md#14-how-the-ambition-is-tested);
decision D9).
**Builds on:** [Core 0.1](core-v0.1.md), [reference simulation 0.1](reference-simulation-v0.1.md),
decision records [0007](decisions/0007-rl-arch-02-core-and-reference-simulation.md) and
[0008](decisions/0008-core-sequencing-per-recipient.md); gate 1 passed 2026-10-27
([evidence](../docs/evidence/rl-arch-02-exit-gate.md)).
**Errata:** [decision record 0010](decisions/0010-binding-errata-lease-expired-commitment-state.md)
(B1, B2, approved 2026-10-27) and [decision record 0011](decisions/0011-interop-2a-bundle-errata.md) (N1–N14, approved 2026-10-28) amend sections 3, 6.2–6.5,
9.1.1 and 9.2. The amended text is marked. Where the reference or the INTEROP-1 harness does
not yet follow it, the [discrepancy register](../docs/interop/spec-implementation-discrepancies.md) lists the difference.

## 1. What gate 2 proves, and what it does not

RFC-001 1.4 defines gate 2: *a participant implemented independently, in a different codebase,
communicates with the reference implementation using only the published schemas and this
specification, with no private integration logic.* It is what distinguishes an open standard
from a product: gate 1 showed that one team's implementation is consistent with itself;
gate 2 shows that someone else can implement Core 0.1 from the documents alone.

Gate 2 **proves**, to the independence level chosen (I1): that Core 0.1 and the reference
simulation's behavior rules are complete and unambiguous enough for an independent
implementation of one participant to produce the same protocol outcomes as the reference,
across the happy path and the failure variants; and that Core 0.1 is implementable without
the RoboLang Python package (P10).

Gate 2 **does not prove**: interoperability over a real network or binding (RFC-001 open
question 7 stays open); production authentication (`sim-hmac-sha256` stays; `ed25519` is out);
anything about real robots; conformance of whole products (open question 13).

## 2. Independence (I1)

| Level | Who writes the participant | Information barrier | What a pass proves |
|---|---|---|---|
| **1 — clean-room** | A different author: a person, another team, or a separate AI session | Works in an empty repository with only the frozen bundle (section 4); no access to this repository; questions only through the logged Q&A (4.3) | Core 0.1 is independently implementable from the documents. This is RFC-001's gate 2 |
| **2 — spec-only, same author** | The reference's authors | Separate repository and language; the rule that only the bundle may be read, enforced by not checking out `src/` | The documents are sufficient for *someone who already knows the design*; hidden assumptions shared by both sides are not detected |
| **3 — conformance kit only** | Anyone | None | The receiver checks agree on the Core fixtures (G-T0). A prerequisite, not gate 2 |

**Proposed:** gate 2 is passed only at level 1. A level 2 run may be held first as a
rehearsal; it is reported, but it does not pass gate 2.

### 2.1 Auditable isolation

Level 1 must be demonstrable, not asserted. Two forms qualify, and the evidence always states
which one was used, because they are not the same kind of independence:

| Form | Isolation conditions | Recorded in the evidence |
|---|---|---|
| **Human or team** recruited for gate 2 | Has not worked on this repository or seen its source, records or earlier discussions; receives only the bundle | Who they are (or an anonymised identifier), a signed statement of the conditions, and the Q&A log |
| **Clean-room AI implementation** | A new session with no earlier RoboLang conversation, no shared project memory or custom instructions that mention RoboLang, no access to this repository or its history, no tools that can reach it (no network access to GitHub), and no hints derived from the implementation; it starts in an empty working directory containing only the bundle | The model and provider, the complete initial prompt, the session's configuration showing the absence of memory and repository access, the full transcript, and the Q&A log |

A pass by a clean-room AI shows that the documents suffice for an implementer without the
authors' knowledge; it does not show what a different organisation would build. A later pass by
a human team is stronger evidence, and the record of which form was used lets readers weigh it.

## 3. Roles (I2)

| Participant | Implemented by |
|---|---|
| `rl:robot:amr-07` | **The independent implementation** |
| `rl:agent:planner-1`, `rl:robot:arm-02`, `rl:human:shift-lead-3` | The reference (`robolang.arch02`) |
| World, clock, sensors, actions, faults, record | The reference harness (section 6) |

amr-07 exercises the most Core surface: R2R requests, offers and commitments, reservations,
local authorization under a standing authorization and by explicit request, the custody
transfer and its commit rule, sequencing (0008), signatures, observations and outcomes with
evidence. **amr-07's required behavior is the reference simulation's**, sections 6 to 8 and
the 6.1 timeline, with Core 0.1 for everything on the wire. *Erratum (0011, N5–N12):* that
includes reference simulation 4.3 (observations) and 6.4 (amr-07 in detail). Of section 5,
only the delivery rules (5.1) bind amr-07: the message-ID formula of 5.2 is reference
serialization (9.2). This specification does not restate
it; a gap or ambiguity found there is handled as a finding (section 9.3).

## 4. The specification bundle (I3)

### 4.1 Contents

The independent author receives exactly these files, taken at a frozen commit of this
repository, with a manifest of SHA-256 values:

| File | Why |
|---|---|
| `spec/architecture/rfc-001-ai-native-architecture.md` | Semantics and authority rules |
| `spec/core-v0.1.md`, `spec/core/*.schema.json` | The contract |
| `spec/reference-simulation-v0.1.md` | amr-07's behavior, the scenario and the faults |
| `spec/decisions/0007-…`, `spec/decisions/0008-…` | Choices and their reasons |
| `spec/interop-gate-2-v0.1.md` (this document) | The binding and the tests |
| `tests/fixtures/core/` | Conformance fixtures (G-T0) |
| An example `view` for amr-07 in the base scenario (6.4), produced in INTEROP-2 | The exact shape of what the participant receives |

Nothing else: no source code, no run records, no `golden_records.json`, no test code, and no
harness-private data. `scenario.json`, `cases.json` and `sequencing_cases.json` stay with the
harness: they hold the other participants' configuration, the fault scripts and the expected
outcomes. The cases are described in prose in the reference simulation (sections 8 and 8.1),
which is how the author learns what amr-07 must do; the scripts and expectations are not given,
so that behavior cannot be fitted to them.

### 4.2 Freeze

The bundle is frozen at one commit for the whole gate 2 run. A change found necessary during
gate 2 is an erratum (9.3): it gets its own decision, a new frozen bundle, and both
implementations are retested against it.

### 4.3 Questions

The independent author's questions and the answers are kept in a written log, published with
the evidence. An answer that adds or changes a rule is an erratum (9.3), never a private
clarification: what the author needs to know must end up in the bundle.

## 5. Language and dependencies (I4, I5)

- **Language:** proposed TypeScript on Node.js (standard library only), or another language
  that is not Python. A different language also shows that no Python behavior (number
  formatting, dictionary order, JSON details) leaked into the contract.
- **Allowed:** the language's standard library, including JSON, SHA-256 and HMAC; optionally a
  JSON Schema validator. JCS (RFC 8785) is implemented from Core 7, or taken from a published
  third-party library.
- **Not allowed:** the RoboLang Python package, any code or generated code from this
  repository, and any network, filesystem or clock access during a run beyond the binding.

## 6. The test binding `robolang-sim-stdio` 0.1 (I6)

### 6.1 What it is

A **test binding**, published in the bundle: it carries Core 0.1 documents unchanged between
the reference harness and an external participant process (RFC-001 14: a binding adds no
meaning and removes none). It adds a few **simulation-only frames** that stand in for the
robot's own clock, sensors and actuators, which in the reference live inside one process.
It is not a transport for real deployments and does not answer RFC-001 open question 7.

### 6.2 Framing and determinism

- The harness starts the participant as a child process. Frames are UTF-8 JSON objects, one per
  line, on the participant's standard input (harness → participant) and standard output
  (participant → harness). Standard error is for diagnostics only.
- **Lock-step.** After every harness frame the participant processes it completely, writes all
  its frames, and ends with `{"frame": "idle"}`. The harness advances the clock only after
  `idle`. The precise rules, including sensing and time limits, are in 6.5.
- **The harness owns time.** Every harness frame carries `now_ms` (milliseconds after the
  scenario epoch). The participant has no other clock, no randomness and no I/O besides the
  binding.
- *Erratum (0011, N3).* **Silence and holds.** While the participant is live, it owns its holds
  (6.3, `set_timer`). The harness keeps a hold ledger from the participant's receipts: a `held`
  receipt opens an entry, and the first terminal release receipt for that message (`accepted`,
  `rejected` or `not_handled`) closes it. When silence starts (reference simulation 10), the
  participant gets no further frame and none of its timers fires. At the Core 8.4 instants the
  harness closes each open entry itself:
  - `released`, `rejected C301` at `expires_at − clock_tolerance`;
  - `released`, `not_handled` at delivery + `max_hold`, skipping the missing numbers;
  - then open entries of the same stream that are now next, `released`, `not_handled`, in
    sequence order, at the same instant.

  A closed entry is removed, so each held message gets **exactly one** terminal release
  receipt. A participant release receipt for a message with no open entry is an inconsistent
  claim (9.1.1). Expiry wins a tie. Silence has no end in 0.1, so holds are never handed back.
- **Faults are harness-side.** Drops, delays, duplicates, replays, altered signatures,
  partitions and silence (reference simulation 10) are applied by the harness; the participant
  is not told about them.

### 6.3 Frames

**Harness → participant**

| `frame` | Fields | Meaning |
|---|---|---|
| `start` | `now_ms`, `participant`, `view` (6.4) | Begin a run. The frame does not name the case |
| `deliver` | `now_ms`, `data` (the message text exactly as delivered, which may be altered) | A message arrives; the participant applies Core 8 and its rules |
| `timer` | `now_ms`, `timer` | A timer the participant set has fired |
| `sensed` | `now_ms`, `request`, `value` (or `null`), `observed_at_ms` | Answer to `sense`: the sensor's reading, already lagged; `null` if the sensor is unavailable or cannot see the thing |
| `action_done` | `now_ms`, `action` | An action the participant started has completed |
| `end` | — | The run is over; the participant exits |

**Participant → harness**

| `frame` | Fields | Meaning |
|---|---|---|
| `send` | `message` (a Core 0.1 message object) | Send a message; the harness serializes it, applies faults and delivers it |
| `receipt` | `message_id`, `sender`, `receipt`, `code` | The participant's receipt for a delivered message (Core 8: `accepted`, `duplicate`, `held`, `rejected` with a code, or for a released held message a second receipt). *Erratum (0011, N14):* exactly one per `deliver`, and the first `receipt` of that cycle; receipts for released messages follow it. `code` is `null` unless `rejected`. For data that cannot be decoded, `message_id` and `sender` are `null`. *(N2):* on `duplicate` the participant sends nothing: **the harness repeats the original reply**, meaning every message the participant sent with `in_reply_to` equal to the duplicate's ID, byte-identical, one `latency` later, delivered as kind `resend`, with no fault applied again. It keeps its ID and sequence, so it causes no second effect and no gap |
| `set_timer` | `timer`, `at_ms` | Ask for a `timer` frame at `at_ms`. *Erratum (0011, N3):* for each held message the participant requests a timer at both Core 8.4 instants (`expires_at − clock_tolerance` and delivery + `max_hold`), and answers the `timer` frame with the release receipt, if one is due |
| `sense` | `request`, `sensor`, `thing`, `property` | Read one of its own sensors now; answered by one `sensed` with the same `request` (6.5) |
| `act` | `action`, `kind` (`move`), `to` | Start an action; the harness records a motion event now and sends `action_done` after the scenario's duration |
| `event` | `kind` (`state`, `authorization`, `escalated`, `custody_committed`, `lease_expired`†), `detail` | The participant's **claims** about its internal state: goal, commitment and custody state changes, authorization decisions and their basis, escalations, the commit point. Claims are recorded, but never accepted as evidence on their own (9.1.1) |
| `idle` | — | Done with the current harness frame |

The participant builds and signs its own observations (Core 5.6, 7) from `sensed`: it is the
source, and its key signs them.

**Erratum ([decision record 0010](decisions/0010-binding-errata-lease-expired-commitment-state.md)).**
† `lease_expired` (detail `lease: "commitment"`, `commitment`) claims that the participant's
commitment lease expired; it is recorded as a run-record event (B1). A `state` claim with
`record: "commitment"` and a Core 5.8 local `state` sets the participant's final commitment
state and is not a run-record event (B2). Both are corroborated as 9.1.1 states.

**Erratum ([decision record 0011](decisions/0011-interop-2a-bundle-errata.md), N7): claim variants.** A `state` claim takes one of three forms, distinguished
by its `record`:

| Variant | `kind` / `record` | `detail` | When | Required |
|---|---|---|---|---|
| Goal transition | `state` / `goal` | `record`, `state` (RFC-001 9.2 name) | At the moment of the transition, in the order the transitions happen | Yes: every transition |
| Custody transition | `state` / `custody` | `record`, `state` (RFC-001 10.4 name) | Same | Yes: every transition |
| Commitment state | `state` / `commitment` | `record`, `state` (Core 5.8 local state) | Any time within the cycle in which the change happened, before `idle` | Yes: every change (B2) |
| Diagnostic | `state` / one of the five names below | `record`, plus the listed fields | When the participant notes it | **No**: optional, not compared |
| Authorization | `authorization` | `decision` (`granted` or `denied`), `basis` (the standing authorization's ID or the decision's message ID) | When decided | Yes |
| Commit point | `custody_committed` | `thing`, `release` (the `custody_released` message ID) | Before the `acknowledge` (RFC-001 10.4 rule 3) | Yes |
| Escalation | `escalated` | `reason` (a Core 5.11 code) | When escalating | Yes |
| Lease expiry | `lease_expired` | as in B1, for the commitment only | At expiry | Yes |

The five diagnostics, each optional but, when written, consistent with what the harness saw in
the same cycle (9.1.1):

| `record` | Fields | Must be consistent with |
|---|---|---|
| `peer_ready` | — | An accepted `coordinate` `ready_for_handover` |
| `reservation_denied` | — | An accepted `reservation_decision` with `denied` |
| `motion_refused` | `to` | No `act` |
| `no_confirmation_evidence` | — | A `sensed` with `value: null` |
| `physical_custody_unknown` | — | A `sensed` with `value: null` after the commit point |

A `state` claim with any other `record` is a binding failure.

### 6.4 The participant view

`start` carries a **view**: only what amr-07 would hold as its own configuration, plus the
public material it needs. The harness builds it from the scenario after the case's overrides
and keeps everything else private.

| In the view | Content |
|---|---|
| Identity | `participant` (its `rl:robot:…` ID), its key reference and that key's `sim-hmac-sha256` secret |
| Own configuration | Its capability descriptor, its standing authorization, its policy (`accept_assistance`), its sensors, its initial location. *Erratum (0011, N1):* for each of its own sensors, `sensor` (name), `scope` (`own_deck` or `places`), `places` (a list, or `null`), `valid_for`, `confidence` and `method`. Never a sensor's lag or unavailability (faults), and never another participant's sensors |
| Deployment parameters | `epoch`, `message_ttl`, `max_message_lifetime`, `clock_tolerance`, `max_hold`, `offer_lease_end`, `reservation_end`, the `conversation_id` |
| Public deployment facts | The participant records (7.2 form) of the participants it deals with, the places and their `within` relations, the `arbiters` of shared resources, and its own authority (`authorities` for itself) |
| Verification material | The `sim-hmac-sha256` secrets of the keys of participants whose messages it must verify: arm-02 and the authority. These are inherently shared in this scheme (Core 7) |

**Never in the view:** other participants' policies, standing authorizations, sensors and
private configuration; the planner's goal; the authority's decision table; durations (the
harness applies them); faults, the case ID and the expected results.

The shared HMAC secrets are acceptable only in this test environment: whoever holds a secret can
produce valid signatures for that key, so a valid `sim-hmac-sha256` signature here never proves
which participant produced a message. Gate 2 does not test participant identity.

### 6.5 Lock-step rules

1. After writing a harness frame, the harness reads the participant's frames in order until
   `idle`.
2. **Sensing.** When the harness reads a `sense` frame, it writes exactly one `sensed` frame with
   the same `request` and the same `now_ms`, before reading anything further. A participant has
   at most one `sense` outstanding: it waits for the `sensed` reply before writing any other
   frame, including `idle`. `request` values are unique within a run. *Erratum (0011, N14):* so
   are `timer` and `action` names, each within its own kind; reuse is a binding failure.
3. A participant never writes `idle` while a `sense` is outstanding, and never writes frames
   after `idle` until the next harness frame.
4. **Time limits.** Simulated time does not pass while the harness waits. In wall-clock time the
   harness waits at most 10 s for each frame it expects (a reply to `start`, `deliver`, `timer`,
   `action_done` or `sensed`, or the next frame before `idle`). Exceeding it, invalid JSON, an
   unknown `frame`, an answer to a request that was not made, or output after `end` is a
   **binding failure**: the run fails (G-T5) and is reported, not retried.
5. **Order.** Frames from one `idle` cycle take effect in the order written. Messages sent in one
   cycle are delivered in that order (reference simulation 5.1). *Erratum (0011, N14):* within a
   cycle, only these orders are compared:
   - the delivered message's receipt before any release receipt;
   - sends in the order written;
   - claims in the order 6.3 requires.

## 7. Prerequisite: conformance kit (G-T0, I12)

Before any scenario run, the independent implementation's receiver checks must give the stated
receipt for all 47 Core fixtures (A-T16), 17 of them multi-fault precedence fixtures (INTEROP-2f): a `check` mode reads one fixture and the context of
`tests/fixtures/core/context.json` and prints the code and stage, or `accepted`. This is
level 3 of section 2 and the first thing the independent author can test alone.

## 8. Interoperability tests

| # | Test | Cases | amr-07 role |
|---|---|---|---|
| G-T0 | Conformance kit | 47 Core fixtures | Receiver checks |
| G-T1 | Happy path | `happy_path`, `happy_path_seed_1` | The full role |
| G-T2 | amr-07 behavior under faults | `v01_declines`, `v02_duplicate_commit`, `v03_replayed_commit`, `v05_stale_confirmation`, `v06_partition_before_release`, `v07_release_ack_lost`, `v08_arrived_without_evidence`, `v11_payload_beyond_standing_authorization`, `v12a_unsigned_commit`, `v12b_wrongly_signed_commit` | Exercised directly |
| G-T3 | Sequencing (0008) | All 9 `seq_*` cases | Exercised directly where amr-07 sends or receives the affected stream (`seq_replay_while_fresh`, `seq_lost_ack_holds_next`, and `seq_silent_while_held`, where amr-07 holds a message and the harness closes the hold, 6.2); otherwise a regression check, since amr-07 numbers and receives its own streams in every case |
| G-T4 | Regression | `v04_assistant_silent`, `v09_stale_reverification`, `v10_requester_authorizes`, `v13_no_grasp` | Must still behave correctly although the fault is elsewhere |
| G-T5 | Determinism | Every case run twice | The independent run is repeatable: byte-identical gate 2 records |

## 9. Success criteria, failure conditions and findings (I7, I8)

### 9.1 Required for each case

1. Every `expect` field of the case (reference simulation 11) holds for the run, evaluated from
   harness-observed facts wherever 9.1.1 provides them.
2. Every receipt the independent amr-07 reports equals the receipt the reference amr-07 gives
   for the same delivery (same `receipt` and `code`).
3. The sequence of sent messages, by type, sender, recipient and send time, equals the
   reference run's.
4. Every claim the participant makes is consistent with the harness's own observations
   (9.1.1). An inconsistent claim fails the case even if every expectation holds.
5. Lock-step and determinism hold (G-T5); the participant never times out or breaks framing.

#### 9.1.1 Corroboration of claims

The harness does not take the participant's word for anything it can observe. Facts come from
the harness; the participant's `event` and `receipt` frames are claims, checked against them.

| What | Source of truth | Claims checked against it |
|---|---|---|
| Motion and the world | The harness: a motion event exists only for an `act` it executed, and the world changes only by its own effects | Any claim of having moved or of a thing's location |
| Messages | The harness's message log: what was sent, delivered, dropped or altered | Receipts must refer to a delivery that happened; a reply to a message must follow its delivery |
| Receipts, stages 1–3 (Core 8: form, signature, freshness, recipient) | The harness recomputes them with `robolang.core` on the exact delivered bytes | The participant's `receipt` code for those stages. *Erratum (0011, N13):* only codes whose recomputed stage is form, signature or freshness are compared. The stage-5 codes come after duplicate and order handling (Core 8.1), and are judged with stages 4–5 |
| Receipts, stages 4–5 (duplicates, order, `C306`, `C403`) | The reference run of the same case | The participant's receipts (9.1, item 2) |
| Authorization before motion | Wire evidence: a granted `authorization_decision` delivered to amr-07 and accepted, or a standing authorization that covers the sub-goal under reference simulation 7.2, evaluated by the harness from the scenario | Every `act` must follow an authorization the harness can establish this way; an `authorization` claim the harness cannot establish is inconsistent |
| Commit point and designated custody | Wire evidence: amr-07's `acknowledge` with `accepted: true` of a `custody_released` it received (RFC-001 10.4 rules 3–4) | The `custody_committed` claim, and any later `HOLDING` custody claim, must match it |
| Commitment lease and commitment state (erratum 0010) | Wire evidence: the `commit` the participant accepted, its lease, and the participant's sent `outcome` with the harness's own observations as evidence | A `lease_expired` claim needs an accepted commitment of that ID whose lease has expired. Commitment state claims go `active`, then one of `fulfilled` (a sent `outcome` of `succeeded` whose evidence the harness observed), `failed` (a sent `outcome` other than `succeeded`) or `lapsed` (lease expired), and each needs an accepted `commit` whose debtor is the participant |
| Observations a participant presents (erratum 0011, N5) | The harness's own reading for each `sense` | Every observation in a sent message must equal **in meaning** one the participant could build from a harness reading: the same `id`, `subject`, `property`, `value` and `source`; the same instant for `observed_at`; the same `valid_for` duration and `confidence` decimal. Its `sim-hmac-sha256` signature must verify under the participant's key. Formatting alone is not a failure |
| Diagnostics (erratum 0011, N7) | The harness's observations in the same cycle | An optional diagnostic must be consistent with them (6.3) |
| Local states (goal, commitment, custody), escalation | Claims only | Must be consistent with the wire: for example, no `SUCCEEDED` goal claim without a sent `outcome` of `succeeded`; no escalation claim unless a lease expired with custody unresolved. Expectations about these states (`custody` local states, `escalated`) are evaluated from claims only after these checks pass |

### 9.2 Measured and reported, not required

- **JCS identity:** for each message, whether its JCS form (Core 7) equals the reference's.
  Differences are listed and classified (9.3); identity is the expected result where the
  specification fixes every field.
- *Erratum (0011, N5).* **Reference serialization:** times written with three fractional digits
  and `Z`, and message IDs derived by reference simulation 5.2. They are measured and reported
  here; a difference is a finding of 9.3, not a failure.
- Byte-identical run records are **not** a criterion across implementations (key order in
  documents may differ); `golden_records.json` applies to the reference only.

### 9.3 Findings

Any difference between the independent and the reference run is a **finding**, classified in
this order:

1. **Specification gap or ambiguity** — the bundle allows both behaviors, or says nothing.
   Resolved by an erratum decision (as 0008 was), a new frozen bundle, and both implementations
   retested.
2. **Reference defect** — the bundle is clear and the reference disagrees with it. Fixed in the
   reference, under its own authorization; both retested.
3. **Independent defect** — the bundle is clear and the independent implementation disagrees.
   Fixed by its author from the bundle alone.

### 9.4 Gate 2 passes when

G-T0 to G-T5 meet 9.1 for every case at one frozen bundle, at the independence level of I1;
every finding is resolved and recorded; the evidence (section 11, INTEROP-4) is complete; and
the owner signs off.

### 9.5 Gate 2 fails, or is suspended, when

the barrier of the chosen level is broken (for example, the independent author reads `src/`);
a required criterion cannot be met without a change to the specification that the owner does
not approve; or the reference cannot pass its own self-test (INTEROP-1).

## 10. Run identity (I9)

A gate 2 run has the same inputs as the reference run of that case, so it has the **same run
ID** (reference simulation 9.1). Which implementation played amr-07, its version and the bundle
manifest are recorded in the gate 2 evidence, not in the run record, whose schema (0007) is
unchanged.

## 11. Delivery (I10)

| Slice | Content | Done when |
|---|---|---|
| **RL-INTEROP-01a** — this specification | Decision record 0009 | Approved |
| **INTEROP-1** — binding self-test | The harness side of `robolang-sim-stdio` in this repository, and the reference amr-07 run *as an external process* through it | All cases pass (24 when INTEROP-1 was done; 25 since INTEROP-2f added `seq_silent_while_held`) with amr-07 receiving only the filtered view (6.4) and every claim corroborated (9.1.1), and the records are identical to the F1 golden records. Shows that the binding is lossless, the view is sufficient and the harness checks work before anyone else depends on them |
| **INTEROP-2** — bundle | A completeness review of amr-07's behavior: every rule the reference code applies is traced to a section of the bundle; gaps become errata first. Then the frozen bundle, manifest and conformance kit instructions | The review finds no untraced rule; bundle published |
| **INTEROP-3** — independent participant | Written by the chosen author (I1) in a separate repository, from the bundle | The author declares it ready |
| **INTEROP-4** — gate 2 run and evidence | G-T0 to G-T5, findings log, Q&A log, evidence document | Section 9.4; owner sign-off |

Each slice needs its own authorization. INTEROP-3 also needs the owner to name the author.

## 12. Not in gate 2 (I11)

A real network or local peer-to-peer transport; A2A; `ed25519` and key distribution; more than
one independent participant (planner-1 or arm-02 could follow); multiple conversations;
physical execution; the language front end (`.rob`, Task IR).

## 13. Status

| Part | Status |
|---|---|
| This specification, decision record 0009 | Approved 2026-10-27 (on the text at `508f5f9`) |
| Erratum, decision record 0010 (B1, B2) | Approved 2026-10-27 |
| Erratum, decision record 0011 (N1–N14) | Approved 2026-10-28; applied to this specification, Core 0.1 and the reference simulation by INTEROP-2b-SPEC |
| INTEROP-1 | Done 2026-10-28 (PR #36) |
| INTEROP-2a | Done 2026-10-28 (PR #38) |
| INTEROP-2b-SPEC: N1–N14 applied to the specifications | Done 2026-10-28 (PR #39) |
| INTEROP-2c: decision record 0012 (P1–P3), and its application | Approved and applied 2026-10-28 (PRs #40, #41) |
| INTEROP-2d: reference corrections D1–D7 | Done 2026-10-28 (PR #42) |
| INTEROP-2e: harness corrections N3, H1, H2, N13(c) | Done 2026-10-28 (PR #43) |
| H3: decision record 0013, receiver-scoped verification keys | Approved and applied 2026-10-28 (PR #44) |
| INTEROP-2f: precedence fixtures (47 Core fixtures) and `seq_silent_while_held` (25 cases) | Done 2026-10-28 (PR #45) |
| INTEROP-2g: final bundle audit and freeze preparation | Authorized 2026-10-28; the freeze itself needs owner approval |
| INTEROP-3, INTEROP-4 | Not authorized |
