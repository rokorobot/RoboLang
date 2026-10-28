# RL-ARCH-02 — Reference simulation of the RFC-001 scenario, version 0.1 (draft)

**Status:** RL-ARCH-02a specification, **approved by the owner on 2026-10-27**. Every choice is a row in
[decision record 0007](decisions/0007-rl-arch-02-core-and-reference-simulation.md) (R1–R14),
approved 2026-10-27.
**Milestone:** RL-ARCH-02 (RFC-001 section 19; decision D8). Gate 1 of RFC-001 section 1.4.
**Core:** [Core 0.1](core-v0.1.md)
**Erratum (approved 2026-10-27):** sections 3, 5.2 and 8 to 13 are amended by
[decision record 0008](decisions/0008-core-sequencing-per-recipient.md) (Q1–Q9, approved 2026-10-27):
per-recipient sequencing, holding on gaps, and the sequencing cases (A-T20). The amended text
is marked.
**Erratum (approved 2026-10-28):** sections 4.3, 5.2, 6.2, 6.3, 8 (V6) and 10 are amended,
and section 6.4 added, by [decision record 0011](decisions/0011-interop-2a-bundle-errata.md)
(N3–N12): what gate 2 needs to implement amr-07. The amended text is marked; where the reference
does not yet follow it, the [discrepancy register](../docs/interop/spec-implementation-discrepancies.md)
lists the difference.
**Erratum (approved 2026-10-28):** sections 6.2 (rule 3) and 6.4 (lease expiry) are amended by
[decision record 0012](decisions/0012-interop-2c-remaining-normative-decisions.md) (P1): an
action in progress at lease expiry. The amended text is marked.
**Erratum (approved 2026-10-28):** section 5.3 is amended by
[decision record 0013](decisions/0013-h3-receiver-scoped-verification-keys.md) (H3): the keys a
receiver verifies with. The amended text is marked.
**Fixtures:** [`tests/fixtures/arch02/`](../tests/fixtures/arch02/README.md)
**Implementation:** none. RL-ARCH-02b implements it only after this specification is approved
and the owner separately authorizes implementation (D8).

> **Simulation only.** Everything here runs in one Python process against a symbolic world.
> No ROS 2, no network, no AI model, no hardware. A run that ends `succeeded` says that the
> participants followed Core 0.1 and RFC-001 sections 9–13 consistently with a declared world.
> It is not evidence about real robots, real networks or real cryptography.

---

## 1. Purpose and acceptance

RFC-001 section 19 fixes what RL-ARCH-02 must show:

> the happy path of section 18 ends in a verified `succeeded` outcome; every failure variant
> ends in the stated result with no simulated motion where none is allowed; every run
> produces a reconstructable message and event log.

This document makes that checkable: it fixes the participants, the world, how each
participant behaves, how failures are injected, what is recorded, and one acceptance test per
case (section 12). It is the reference behavior for gate 1; it is not part of the Core
contract, and a gate-2 implementation needs only [Core 0.1](core-v0.1.md).

## 2. Participants

| Participant | Kind | Role | Can | Cannot |
|---|---|---|---|---|
| `rl:agent:planner-1` | agent | Requests the goal; re-verifies outcomes | Send A2R messages | Act physically; authorize anything |
| `rl:robot:arm-02` | robot | Fixed manipulator. Receives the goal, coordinates, gives custody | `grasp` up to 30 kg within reach of `cell-a2` and `handoff-h1`; `inspect` with `cell_camera` at both | Travel |
| `rl:robot:amr-07` | robot | Mobile robot. Takes on the sub-goal, receives custody | `transport_payload` up to 40 kg; `dock_at` `dock-d1`, `handoff-h1`, `station-s3`; `inspect` with `deck_camera` | `grasp` (listed unsupported) |
| `rl:human:shift-lead-3` | human_interface | Authority of the domain `rl:domain:warehouse-north`. A scripted stub (7.3) | Send `authorization_decision` | Anything else |

All four belong to `rl:domain:warehouse-north`. Each robot's capability descriptor is a Core
0.1 document in the scenario file. Each robot holds one standing authorization issued by
`shift-lead-3`: profile `r2r`, goal classes `transport_payload` and `custody_transfer`, places
within `warehouse-north-floor`, counterparts in the domain, at most 30 kg, mode `simulation`
(RFC-001 13.3 and 18).

Place identifiers are written in this document without their `rl:place:` prefix: `cell-a2`,
`handoff-h1`, `station-s3`, `dock-d1` and `warehouse-north-floor`; `C42` is
`rl:thing:container-c42`, 25 kg, class `container`.

## 3. Deployment parameters

From the scenario file (section 11), with these values in the base scenario:

| Parameter | Base value | Use |
|---|---|---|
| `epoch` | `2026-10-19T10:14:00.000Z` | Simulated time 0 |
| `latency` | 100 ms | Every message is delivered this long after it is sent |
| `message_ttl` | 30 s | Every sender sets `expires_at = sent_at + message_ttl` |
| `max_message_lifetime` | 60 s | `C302` |
| `clock_tolerance` | 500 ms | Applied in the safe direction (Core 8.1) |
| `ack_timeout` | 5 s | A giver without an `acknowledge` of its `custody_released` enters `IN_DOUBT` |
| `query_interval` | 30 s | A giver in doubt repeats `custody_query` at this interval |
| `max_hold` | 5 s | The longest a message after a sequence gap is held (Core 8.4; 0008, Q4) |
| `authority_delay` | 20 s | The authority stub answers this long after receiving a request |
| `offer_lease_end` | `10:25:00.000Z` | amr-07 proposes this lease end in `offer_assistance` |
| `reservation_end` | `10:20:00.000Z` | amr-07 asks for H1 until then |
| `durations` | `dock-d1 → handoff-h1` 30 s, `handoff-h1 → station-s3` 60 s, `handover_place` 20 s | Simulated action durations |

Clocks are perfect and shared: every participant reads the same simulated time. The tolerance
is still applied, so the rules that depend on it are exercised.

## 4. The symbolic world

### 4.1 Facts

The world is a set of facts `(subject, property) → value` using the Core 0.1 properties
(Core 5.2). Initially:

| Subject | Property | Value |
|---|---|---|
| C42 | `located_in` | `cell-a2` |
| arm-02 | `located_in` | `cell-a2` |
| amr-07 | `located_in` | `dock-d1` |

A thing that is `carried_by` a robot is `located_in` that robot's place; the world derives
it. Places `cell-a2`, `handoff-h1`, `station-s3` and `dock-d1` are within
`warehouse-north-floor`.

This world is specific to the scenario. It is not world file 1.0 of
[`simulation-v1.0.md`](simulation-v1.0.md), which has no carried objects and no custody, and
it does not answer RFC-001 open question 2 (world-model depth, F2-6) (R7).

### 4.2 Actions and motion events

| Action | By | Requires | Effect | Duration |
|---|---|---|---|---|
| `move(to)` | amr-07 | amr-07 can dock at `to`; it is authorized for the work | amr-07 `located_in` `to`; anything it carries follows | From `durations` |
| `handover_place(thing, onto)` | arm-02 | The thing is within arm-02's reach; `onto` is `located_in` `handoff-h1`; arm-02 is authorized | The thing is `carried_by` `onto` | `handover_place` |

Each action emits one **motion event** when it starts, `{ t_ms, participant, action, args }`,
and its effect when it ends. A motion event is the only way the world changes after time 0.
"No simulated motion" in an acceptance test means no motion event, for the participant and
interval it names (R8).

### 4.3 Sensors and observations

| Sensor | On | Observes |
|---|---|---|
| `cell_camera` | arm-02 | `located_in` and `carried_by` of things at `cell-a2` and `handoff-h1` |
| `deck_camera` | amr-07 | `carried_by amr-07` for things on its deck, and their `located_in` |

In the scenario file a sensor's `scope` is either `places` (it observes things at the listed
`places`) or `own_deck` (it observes what its robot carries, wherever the robot is; `places`
is then `null`).

An observation is the ground truth at the moment it is made: `observed_at` is the current
time (minus any scripted sensor lag, 10), `valid_for` 30 s, `confidence` `0.95`, `method`
`vision`, signed by the observing robot. A sensor that is scripted unavailable makes no
observation. Observation IDs are `rl:obs:<robot>:<n>`, numbered from 1 per robot.

*Erratum (0011, N6).* `<robot>` is the last `:`-separated part of the robot's ID (`amr-07`).
Numbers count only readings that returned a value, in order, whether or not the observation
is ever sent: a reading of `null` does not use up a number. amr-07 senses `carried_by` with
`deck_camera` on `coordinate` `placed`, after acknowledging a release (rule 9) and on every
`custody_query`, and `located_in` on arrival at the target; nowhere else.

## 5. Messages

### 5.1 Delivery

Messages are delivered in simulated time, `latency` after they are sent, in the order they
were sent when several are due at the same moment. Delivery is reliable unless a fault
(section 10) changes it. A participant handles a delivered message at once; handling takes
no simulated time. Everything a participant sends in response is sent at that same moment,
in the order this document lists.

### 5.2 Identifiers

Message IDs are derived, so that runs are reproducible (R9, amended by 0008, Q7). Sequence
numbers count per sender, recipient and conversation (Core 3):

```text
message_id = "m-" + first 32 hex digits of SHA-256( "robolang-arch02-msg/0.1-seq\n"
                                                   + sender + "\n" + recipient + "\n"
                                                   + conversation_id + "\n" + decimal sequence )
```

This is a property of the reference simulation, not a Core rule: Core 0.1 requires only that
message IDs are unique per sender. *Erratum (0011, N5):* for gate 2 the formula is reference
serialization: an independent participant may number differently, and the difference is
reported, not failed ([`interop-gate-2-v0.1.md`](interop-gate-2-v0.1.md) 9.2). The conversation is `rl:conv:c42-delivery`. Other
identifiers are fixed: goal `rl:goal:c42-to-s3`, sub-goal `rl:goal:c42-to-s3-sub-1`,
commitment `rl:commit:c42-transport`.

### 5.3 Signatures

Every message, observation and descriptor is signed with `sim-hmac-sha256` (Core 7). The
scenario file gives each participant one key and its secret. Every receiver verifies every
signature, so a corrupted or missing signature is caught where Core 8 says (R5).

*Erratum ([decision record 0013](decisions/0013-h3-receiver-scoped-verification-keys.md), H3).*
Each receiver verifies signatures using only the keys available in its configured trust context.
For amr-07, that context consists of its own key and the verification keys supplied in its gate 2
6.4 participant view. A signature requiring a key unavailable in that context is rejected with
`C203`, even if the scenario contains that key elsewhere. Other reference participants retain their
existing configured verification keys.

## 6. Behavior

Each participant is a deterministic state machine. Goal states are those of RFC-001 9.2;
custody states those of 10.4; commitment states those of Core 5.8.

### 6.1 Happy path

The base scenario produces exactly these 20 messages, in this order. Times are simulated
UTC on 2026-10-19; "→" is send and the receiver handles it `latency` later.

| # | RFC-001 §18 | Time sent | From → to | Type | Content and effect |
|---|---|---|---|---|---|
| 1 | 1 | 10:14:00.000 | planner-1 → arm-02 | `propose_goal` | Goal: C42 `located_in` `station-s3`; criterion `observed`, min confidence 0.9, max age 10 s, `any_participant_in_domain`; deadline 10:30:00 |
| 2 | 2 | 10:14:00.100 | arm-02 → planner-1 | `assessment` | `feasible_with_assistance`, `assistance_required`: arm-02 can grasp C42 but cannot travel |
| 3 | 3 | 10:14:00.100 | arm-02 → amr-07 | `request_assistance` | Sub-goal: C42 `located_in` `station-s3`, same criterion; C42 25 kg |
| 4 | 4 | 10:14:00.200 | amr-07 → arm-02 | `offer_assistance` | Condition `resource_held handoff-h1`; lease to `offer_lease_end` |
| 5 | 5 | 10:14:00.300 | arm-02 → amr-07 | `commit` | Commitment `rl:commit:c42-transport`, active |
| 6 | 6 | 10:14:00.400 | amr-07 → arm-02 | `reserve` | amr-07's sub-goal is inside its standing authorization: authorized locally first (step 8 of §18). Asks for `handoff-h1` until `reservation_end` |
| 7 | 6 | 10:14:00.500 | arm-02 → amr-07 | `reservation_decision` | `granted`: arm-02 arbitrates `handoff-h1` (7.5) |
| 8 | 7 | 10:14:00.500 | arm-02 → planner-1 | `accept` | Assistance and the handover region are secured |
| 9 | 8 | 10:14:00.500 | arm-02 → shift-lead-3 | `authorization_request` | The goal is A2R, outside arm-02's `r2r` standing authorization: `outside_standing_authorization` |
| 10 | 8 | 10:14:20.600 | shift-lead-3 → arm-02 | `authorization_decision` | `granted`, mode `simulation`. arm-02's goal is `AUTHORIZED` |
| 11 | 9 | 10:14:30.600 | amr-07 → arm-02 | `coordinate` | `ready_for_handover`: amr-07 started `move(handoff-h1)` at 10:14:00.600 and arrived |
| 12 | 9 | 10:14:30.700 | arm-02 → amr-07 | `coordinate` | `ready_for_handover` |
| 13 | 10 | 10:14:30.700 | arm-02 → amr-07 | `transfer_custody` | C42; criteria: C42 `carried_by amr-07`, min confidence 0.9, max age 2 s, source amr-07. arm-02 `OFFERED` |
| 14 | 10 | 10:14:30.800 | amr-07 → arm-02 | `acknowledge` | `accepted: true`. amr-07 `ACCEPTED`; arm-02 starts `handover_place` at 10:14:30.900, `AWAITING_CONFIRMATION` |
| 15 | 10 | 10:14:50.900 | arm-02 → amr-07 | `coordinate` | `placed` |
| 16 | 11 | 10:14:51.000 | amr-07 → arm-02 | `custody_confirmed` | amr-07's own observation `rl:obs:amr-07:1`: C42 `carried_by amr-07`. amr-07 `CONFIRMATION_SENT` |
| 17 | 12 | 10:14:51.100 | arm-02 → amr-07 | `custody_released` | arm-02 checked the evidence against the criteria and its own `cell_camera` observation. `RELEASE_SENT` |
| 18 | 12 | 10:14:51.200 | amr-07 → arm-02 | `acknowledge` | amr-07 durably accepted the release first: the commit point. amr-07 `HOLDING`; it observes C42 on its deck (`rl:obs:amr-07:2`) and starts `move(station-s3)`. arm-02 `RELEASED` on receipt |
| 19 | 13 | 10:15:51.200 | amr-07 → arm-02 | `outcome` | Sub-goal `succeeded`, `self_reported`: the only evidence, `rl:obs:amr-07:3` (C42 `located_in station-s3`), is amr-07's own |
| 20 | 14 | 10:15:51.300 | arm-02 → planner-1 | `outcome` | Goal `succeeded`, `criteria_evaluated` by arm-02, citing amr-07's observation. planner-1 re-verifies it at 10:15:51.400 (age 0.2 s): `succeeded` |

Final state: C42 `located_in station-s3`, `carried_by amr-07`; designated custody amr-07;
commitment `fulfilled`; motion events: amr-07 two moves, arm-02 one `handover_place`.

### 6.2 Rules every participant follows

1. **Every receipt is checked as Core 8 says**, and the result is recorded (section 9).
2. **Before every action**, a robot checks all of these:
   - its goal or sub-goal has an **established authorization that is still valid** (7.2: a
     covering standing authorization, or a granted decision from its authority);
   - the goal is in the state its role requires for that action;
   - the commitment it depends on is active and its lease unexpired;
   - for motion involving the handover region, it holds a granted, unexpired reservation or
     arbitrates the region.

   *Erratum (0011, N8; consistency correction).* "Authorized" here means the authorization has
   been established, not the goal state `AUTHORIZED`. A goal moves on from `AUTHORIZED` to
   `RUNNING` when work starts (RFC-001 9.2), and amr-07 starts both of its moves in `RUNNING`.
   The exact checks for amr-07 are in 6.4.
3. **On lease expiry**, without waiting for anyone (RFC-001 11.1): the debtor starts no new
   work and stays in its safe state; a reservation holder does not enter the region; the
   creditor treats the obligation as unfulfilled. In 0.1 a robot's safe state is "no further
   motion": it starts no new motion. In the symbolic simulation this does not interrupt an
   action already in progress, which completes as 6.4 says (decision record 0012, P1). RFC-001
   open question 10 stays open. amr-07's expiry table is in 6.4 (erratum 0011).
4. **Custody** follows RFC-001 10.4 rules 1–13. In particular, the receiver commits durably
   before it acknowledges; a giver without an acknowledgement after `ack_timeout` enters
   `IN_DOUBT`, takes no action involving the thing, and sends `custody_query` at once and
   then every `query_interval` until it is answered or the lease expires.
5. **Reporting.** A robot that cannot reach its goal sends an `outcome` (or, before it has
   accepted, an `assessment` `infeasible` and a `reject`) with the reasons of Core 5.11.
   `unknown` is used whenever the robot does not know the physical result or custody. A
   robot that reaches rule 11 of 10.4 records an `escalated` event (section 9).
6. **No retries in 0.1.** A participant does not repeat a message except `custody_query`
   (rule 4) and replies to duplicates (Core 8.4).

### 6.3 Evidence and verification

- A robot evaluates a criterion against observations whose source the criterion accepts,
  that are within both their `valid_for` and the criterion's `max_age` at evaluation time,
  with confidence at least `min_confidence`. Otherwise the criterion is not met.
- The outcome is `succeeded` only if every criterion is met; `unknown` if no observation
  bears on a criterion; `failed` if observations show it is not met.
- planner-1's policy re-verifies every outcome it receives, at receipt (RFC-001 11.3). If
  re-verification fails, it records the goal as `unknown`.

*Erratum ([decision record 0011](decisions/0011-interop-2a-bundle-errata.md), N11).* The evaluation
of one criterion, in this order:

1. **Relevant observations:** those from an accepted source, with the criterion's subject and
   property. If there are none, the result is `missing`.
2. **Fresh enough:** age = `now − observed_at + clock_tolerance`, which must be ≤ both
   `max_age` and the observation's `valid_for`. If no relevant observation is fresh enough, the
   result is `stale`.
3. **Met:** a fresh-enough observation with the criterion's value and a `confidence` of at least
   `min_confidence`. Otherwise the result is `contradicted`: a right value at too low a
   confidence counts as contradicted.

The outcome follows from the results:

- every criterion met: `succeeded`;
- the first criterion not met is `missing`: `unknown` (`evidence_missing`);
- it is `stale`: `unknown` (`evidence_stale`);
- it is `contradicted`: `failed`, with empty `reasons`.

### 6.4 amr-07 in detail

*Erratum ([decision record 0011](decisions/0011-interop-2a-bundle-errata.md), N4–N12).* Sections
6.1–6.3 describe the scenario. This section fixes what they leave open for amr-07, the participant
that gate 2 asks an independent author to implement ([`interop-gate-2-v0.1.md`](interop-gate-2-v0.1.md)
section 3). Where the reference does not yet do what this section says, the difference is listed
in the [discrepancy register](../docs/interop/spec-implementation-discrepancies.md) and marked
*known reference discrepancy* below.

**Values (N5).** These are normative:

- the `outcome` profile is `r2r`;
- every lease amr-07 proposes (`offer_assistance`, `reserve`) has `granted_at` equal to the send
  time;
- `evaluated_at` and `reported_at` are the send time;
- a `succeeded` outcome has empty `reasons`;
- any other outcome has `verification: null` and `evidence: []`;
- times are compared as instants.

Two things are **reference serialization**, not required: writing every time with three
fractional digits and `Z`, and deriving `message_id` by the formula of 5.2. Differences in them
are reported under gate 2 section 9.2, not failures.

**Replies (N4).**

| amr-07 sends | `in_reply_to` |
|---|---|
| `offer_assistance`, `decline` | The `request_assistance` |
| `acknowledge` | The message it acknowledges (`transfer_custody` or `custody_released`) |
| `custody_state` | The `custody_query` |
| `authorization_request`, `reserve`, `coordinate`, `custody_confirmed`, `outcome` | `null` |

**Derived values (N10).**

- **Sub-goal:** the thing and the target are the subject and value of the first triple of the
  sub-goal's `desired_state`.
- **Requester:** the sender of `request_assistance`; it is also the counterpart (7.2).
- **Hand-off place:** the key of `arbiters` whose value is the requester.
- **Mass:** the stated `mass.value` of the thing in the request's `things` (`uncertainty` is not
  added). Every limit is inclusive (`≤`).
- **Places involved (7.2):** the hand-off place and the target. Each is in scope if it, or its
  direct `within`, is listed.
- **`any_participant_in_domain`** (Core 5.3): the participants whose `authority_domain` is the
  evaluator's.
- **Capability test**, in this order:
  1. the descriptor is current and validly signed;
  2. `transport_payload` and `dock_at` are present and not listed unsupported;
  3. mass ≤ `transport_payload.max_mass`;
  4. both places are in `dock_at.places`.

  If 1 or 2 fails, the request is declined `capability_unsupported`; if 3 or 4 fails,
  `capability_insufficient` (Core 5.11).
- **Standing authorization:** checked as in 7.2, with
  `granted_at ≤ now < expires_at − clock_tolerance`.

**Inputs (N8).** For each input, amr-07 checks the required local state. If it does not hold,
the message is rejected with the code shown. amr-07 never replies to a message it rejects.

| Input | Required local state | Otherwise | Reaction |
|---|---|---|---|
| `request_assistance` | No sub-goal yet | `C306` | `ASSESSING`. Policy, then the capability test. `offer_assistance` (condition `resource_held` on the hand-off place, lease to `offer_lease_end`) or `decline` |
| `commit` | Goal `ASSESSING`; the debtor is amr-07; the obligation names the sub-goal | `C306` | Commitment `active`; goal `ACCEPTED`, then `AUTHORIZING`; lease timer (below). Then the standing authorization, or an `authorization_request` (the sub-goal and things as received, mode `simulation`, reason `outside_standing_authorization`) |
| `authorization_decision` | Goal `AUTHORIZING`; the payload names the sub-goal | `C306` (`C401` from anyone but its authority, Core 8.2) | `granted`: `AUTHORIZED`, then `reserve`. `denied`: `DENIED`, commitment `failed`, outcome `rejected` (`authorization_denied`) |
| `reservation_decision` | Goal `AUTHORIZED`; the resource is the hand-off place | `C306` | `granted`: `RUNNING`, then move to the hand-off place. `denied`: the diagnostic `reservation_denied` only. Nothing is sent; the lease expiry decides |
| `coordinate` `ready_for_handover` | Any | — | The diagnostic `peer_ready` only. Nothing is sent |
| `coordinate` `placed` | Custody `ACCEPTED` | `C306` | Sense `carried_by`. With a reading: `custody_confirmed`, then `CONFIRMATION_SENT`. With `null`: the diagnostic `no_confirmation_evidence` only |
| `transfer_custody` | Goal `RUNNING`; the payload names the commitment and the thing; custody not yet set | `C306` | `ACKNOWLEDGED`, then `acknowledge` (`accepted: true`), then `ACCEPTED` |
| `custody_released` | It names amr-07's `custody_confirmed` and the thing (else `C403`, tested first); custody `CONFIRMATION_SENT` | `C306` | `HOLDING`, then the `custody_committed` claim, then `acknowledge`. Sense `carried_by` (rule 9). With a reading: move to the target. With `null`: the diagnostic `physical_custody_unknown`, and no motion |
| `custody_query` | A sub-goal exists; the payload names the thing | `C306` | Sense `carried_by`. `custody_state`: the accepted release if `HOLDING`, else `null`; the observation if any |
| Any other type | — | `C306` | — |

Before every move (6.2 rule 2), all of these must hold:

- the authorization is still established;
- the goal is `RUNNING`;
- the commitment is `active` and its lease unexpired;
- a granted reservation is unexpired.

Each is judged in the safe direction (`now < expires_at − clock_tolerance`). Otherwise amr-07
claims the diagnostic `motion_refused` and does not move.

**Forwarding (N9).**

> **Simulation-specific forwarding (reference simulation and gate 2 only).** amr-07 forwards its
> sensor reading in `custody_confirmed` without checking its freshness or the transfer's
> `criteria` itself; the giver verifies it as RFC-001 10.4 specifies (V5 tests this). After
> accepting a release, amr-07 proceeds on any reading its sensor returns for rule 9. This rule
> does not define evidence freshness, does not amend RFC-001 10.4 rules 1 and 9, and is not a
> pattern for physical deployments, where those rules apply as written.

**Lease expiry (N12).** amr-07 sets one timer, when it accepts `commit`. The timer fires at exactly
the commitment's `lease.expires_at`: the clock tolerance applies to judging others' times (Core 8.1),
not to its own timer. If the commitment has already ended (`fulfilled` or `failed`), the timer does
nothing. Otherwise the commitment becomes `lapsed`, amr-07 claims `lease_expired`, and then:

| Custody at expiry | Result |
|---|---|
| `ACKNOWLEDGED`, `ACCEPTED` or `CONFIRMATION_SENT` | Custody `TIMED_OUT`; `escalated` (`custody_unknown`); goal `FAILED`; outcome `failed` (`lease_expired`, `custody_unknown`). This is V5, and amr-07's part of V6 |
| `HOLDING` | Goal `UNKNOWN`; outcome `unknown` (`lease_expired`); no escalation |
| None (before the hand-over) | Goal `FAILED`; outcome `failed` (`lease_expired`) |

**An action in progress at expiry** (decision record 0012, P1). RoboLang 0.1 does not model
interrupting an action. An action that has started completes when its duration ends; its
effect on the symbolic world applies, and its motion and effect events stand. Expiry prevents
new actions (6.2 rule 2). When an action of amr-07 completes after its commitment has lapsed,
amr-07 takes none of the action's follow-up steps: it changes no local state (the commitment
stays `lapsed`, and the goal keeps the terminal state the expiry set), senses nothing and
sends nothing. **This is a property of the simulation only.** It is not a stopping policy and
not a safety guarantee for physical execution: what a robot must do with an action in
progress when its authority lapses is RFC-001 open question 10, which stays open.

amr-07 claims `lease_expired` for its commitment only, never for its reservation.

## 7. Policy

Local policy is scenario data, not a policy language (RFC-001 open question 11) (R6).

### 7.1 Goal classes

A robot classifies a goal by what it must do for it: `custody_transfer` if it gives or
receives custody, `transport_payload` if it carries the thing. The profile of the goal is the
profile of the message that brought it.

### 7.2 Standing authorization

A goal is covered when its profile, all its classes, all places involved, the counterpart's
domain and the thing's mass are within the scope, the lease is unexpired at the moment
`AUTHORIZING` is entered, and the mode is `simulation`. Otherwise the robot sends
`authorization_request` to its authority and waits. Every decision is recorded with the
standing authorization's ID or the authority's message ID.

### 7.3 The authority stub

`shift-lead-3` answers each `authorization_request` after `authority_delay`, from a table in
the scenario: `(grantee, goal) → granted | denied`. A granted decision has mode `simulation`
and a lease to 11:00:00. A request not in the table is denied.

### 7.4 Authority checks

A robot accepts an `authorization_decision` only from its own authority. One from any other
sender, and in particular from the participant that requested the goal, is rejected with
`C401` and changes nothing.

### 7.5 Resource arbitration

`handoff-h1` is arbitrated by arm-02 (scenario `arbiters`). arm-02 grants a reservation that
does not overlap another and ends no later than the commitment's lease. RFC-001 open question
6 (more than two robots) stays open.

## 8. Failure variants

Each variant is the base scenario with the faults or overrides of section 10. The expected
results are those of RFC-001 section 18, made exact; where RFC-001 allows "`failed` or
`unknown`", this table fixes one and says why (R10).

| # | RFC-001 §18 variant | Faults and overrides | Expected |
|---|---|---|---|
| V1 | amr-07 declines | amr-07's policy declines assistance (`participant_busy`) | amr-07 sends `decline`. arm-02 sends `assessment` `infeasible` and `reject` (`assistance_unavailable`). planner-1 records `rejected`. No motion |
| V2 | Step 5 delivered twice | Duplicate `commit` 1 s after the original | amr-07 records receipt `duplicate` and no second effect; one commitment. `commit` has no reply, so none is repeated. Ends as the happy path |
| V3 | Step 5 replayed after its expiry | Replay `commit` at 10:16:00 | amr-07 rejects it with `C301`; one commitment. Ends as the happy path |
| V4 | amr-07 goes silent after step 6 | amr-07 is silent from the delivery of `reservation_decision` | amr-07 sends nothing and does not move. At 10:25:00 the commitment lapses; arm-02 never placed C42, sees it at `cell-a2`, and reports `failed` (`lease_expired`). planner-1 records `failed`. No motion by either robot |
| V5 | `custody_confirmed` with a 40 s old observation | amr-07's `deck_camera` lags 40 s | arm-02 finds the evidence stale and sends no `custody_released`; designated custody stays with arm-02. amr-07 does not move after the placement (10.4 rule 10). At 10:25:00 the lease expires: amr-07, which never accepted a release, stays in its safe state, reports `failed` (`lease_expired`, `custody_unknown`) and escalates (rule 11). arm-02 is not in doubt, because it sent no release; it reports `failed` (`evidence_stale`, `lease_expired`): it sees C42 on amr-07's deck at H1, not at S3. planner-1 records `failed` |
| V6 | Connection lost between steps 11 and 12 | arm-02 and amr-07 cannot reach each other from the delivery of `custody_confirmed` until 10:26:00 | arm-02's `custody_released` is lost: amr-07 never reaches the commit point and does not move C42. arm-02 enters `IN_DOUBT` at 10:14:56.100; its `custody_query` messages are lost. At 10:25:00 rule 11: both escalate. arm-02 reports `unknown` (`custody_unknown`, `lease_expired`). *Erratum (0011, N12):* amr-07, never past the commit point, reports `failed` (`lease_expired`, `custody_unknown`), as in V5. planner-1 records `unknown`. Designated custody: arm-02 |
| V7 | Acknowledgement of step 12 lost | Drop amr-07's second `acknowledge` | amr-07 proceeds to S3 as in the happy path. arm-02 enters `IN_DOUBT` at 10:14:56.100 and sends one `custody_query`; amr-07 answers `custody_state` naming the accepted release, with a fresh observation; arm-02 resolves to `RELEASED`. planner-1 records `succeeded` |
| V8 | amr-07 reports "arrived" without an observation | amr-07's `deck_camera` unavailable from 10:15:00 | amr-07 reaches S3 (the world has C42 there) but has no evidence: sub-goal `unknown` (`evidence_missing`). arm-02 reports `unknown`. planner-1 records `unknown` |
| V9 | planner-1 re-verifies with too old evidence | Delay arm-02's `outcome` to planner-1 by 15 s | arm-02's goal is `SUCCEEDED`. planner-1 re-verifies at 10:16:06.400; the evidence is 15.2 s old against a 10 s maximum: planner-1 records `unknown` (`evidence_stale`) |
| V10 | planner-1 tries to authorize the goal itself | planner-1 sends arm-02 an `authorization_decision` `granted` at 10:14:01.000 | arm-02 rejects it with `C401` and keeps waiting for its authority. Ends as the happy path; arm-02's authorization record names shift-lead-3's decision |
| V11 | amr-07's standing authorization and a 35 kg payload | C42 is 35 kg; arm-02's grasp limit is raised to 40 kg (see R11); the authority denies amr-07's request | amr-07 commits, finds the goal outside its standing authorization (30 kg), sends `authorization_request` and does not move. It is denied: its sub-goal outcome is `rejected` (`authorization_denied`). arm-02, not yet accepted, sends `assessment` `infeasible` and `reject` (`assistance_unavailable`). planner-1 records `rejected`. No motion |
| V12a | Unsigned `commit` | Remove the signature of `commit` | amr-07 rejects it with `C201` and does not reply. No reservation is requested. At 10:25:00 arm-02's commitment lapses: `assessment` `infeasible` and `reject` (`assistance_unavailable`, `lease_expired`). planner-1 records `rejected`. No motion |
| V12b | Wrongly signed `commit` | Corrupt the signature of `commit` | As V12a, with `C202` |
| V13 | arm-02's descriptor lists no grasp | arm-02's descriptor lists `grasp` as unsupported | arm-02 sends `assessment` `infeasible` (`capability_unsupported`) and `reject`. No R2R message is sent. planner-1 records `rejected`. No motion |

V11 needs one change RFC-001 does not mention: in the reference scenario a 35 kg container is
already beyond arm-02's 30 kg grasp limit, so the run would stop at arm-02's assessment and
never test amr-07's standing authorization. The variant therefore raises arm-02's limit to
40 kg. R11 asks the owner to confirm this.

### 8.1 Sequencing cases

*Added by decision record 0008 (Q9).* Each case is the base scenario with the faults given,
in `tests/fixtures/arch02/sequencing_cases.json`. Sequence numbers are per stream (Core 3), so
arm-02's stream to planner-1 is `assessment` 1, `accept` 2, `outcome` 3, and amr-07's stream
to arm-02 runs from `offer_assistance` 1 to `outcome` 8. In every case a held message has no
effect until it is released (Core 8.4), and every case ends with planner-1 recording
`succeeded`.

| Case | Faults | Expected |
|---|---|---|
| `seq_gap_filled` | Delay arm-02's first `assessment` by 1 s | planner-1 holds `accept` (2) from 10:14:00.600. The assessment (1) arrives at 10:14:01.200; both are processed in order then. No rejection |
| `seq_gap_abandoned` | Delay the `assessment` by 40 s | `accept` is held from 10:14:00.600 and processed at 10:14:05.600, after `max_hold`; sequence 1 is skipped. The assessment arrives at 10:14:40.200 expired: `C301` |
| `seq_held_message_expires` | Delay the `assessment` by 40 s and `accept` by 27 s | `accept` is held from 10:14:27.600 and expires at 10:14:30.000 (safe direction) before `max_hold`: `C301` at that moment, and the stream advances past it. The assessment arrives expired: `C301`. The `outcome` (3) is processed normally |
| `seq_duplicate_while_held` | Delay the `assessment` by 2 s; duplicate `accept` after 0.5 s | The copy at 10:14:01.100 is a duplicate of the held message. `accept` is processed once, at 10:14:02.200 |
| `seq_replay_while_fresh` | Replay `commit` at 10:14:10.000 | amr-07 records a duplicate and no second commitment. (Replayed after its expiry it is `C301`: V3) |
| `seq_stale_nonduplicate` | Inject at 10:14:05.000 another `assessment` from arm-02 to planner-1 with sequence 1 and a different message ID | Stale and not a duplicate: `C306` |
| `seq_unsigned_not_held` | Remove the signature of `accept`; delay the `assessment` by 1 s | `accept` is rejected at once with `C201`: never held, not counted. The `outcome` (3) is held from 10:15:51.400 and processed at 10:15:56.400; planner-1's re-verification finds the evidence 5.2 s old, within its 10 s limit |
| `seq_lost_ack_holds_next` | Drop amr-07's second `acknowledge` (as V7) | amr-07's `custody_state` (7) is held at arm-02 from 10:14:56.300 and processed at 10:15:01.300, when arm-02 resolves to `RELEASED` |

## 9. Run record

A run writes one JSON document, `robolang.arch02.run` version `0.1`, with the IR determinism
rules (two-space indentation, ASCII, LF, one trailing newline). Times (`*_ms`) are integer
milliseconds after the scenario's `epoch`:

| Field | Content |
|---|---|
| `run_record_schema`, `run_record_schema_version` | `"robolang.arch02.run"`, `"0.1"` |
| `mode` | Always `"simulation"` |
| `run_id` | Section 9.1 |
| `inputs` | `scenario_sha256`, `case`, `seed`, `core_version` (`"0.1"`) |
| `messages` | Every message sent, in send order: `t_sent_ms`, `t_delivered_ms` (or `null` if lost), `fate` (`delivered`, `dropped`, `duplicated`, `replayed`, `injected`), the full Core document (`document`), and `receipts`: one entry per delivery attempt, in order, each `{ t_ms, receiver, kind, receipt, code }` with `kind` `original`, `duplicate`, `replay`, `resend` or `released` and `receipt` `accepted`, `duplicate`, `held`, `rejected` (with its Core code in `code`) or `not_handled` (a silenced receiver). A held message gets a second receipt, `accepted` or `rejected`, when it is released (0008) |
| `events` | Every event in time order: `motion`, `effect`, `observation`, `state` (goal, commitment, custody or reservation state changes), `authorization` (decision and its basis), `lease_expired`, `escalated` |
| `final` | `planner_status`, the planner's recorded goal status; and `participants`: per robot, `goal_state`, `commitment_state` and `custody` (local custody state) |
| `designated_custody` | Who is responsible for each thing under the commit rule (RFC-001 10.4 rule 3), as the harness computes it from the record |
| `final_world` | All facts, sorted |
| `end_t_ms` | Time of the last event |

The record holds codes and structured values only; it does not change when wording changes.
It is enough to reconstruct the run: every message with its fate and receipt, and every
event (§19 "reconstructable message and event log").

### 9.1 Run ID

```text
run_id = "arch02-" + lowercase hex SHA-256( "robolang-arch02-run/0.1\n" + scenario_sha256 + "\n"
                                            + case + "\n" + seed + "\n" )
```

`scenario_sha256` is taken of the scenario file with a leading BOM removed and CR LF read as
LF, as in [simulation 1.0](simulation-v1.0.md) section 6. The simulation has no random
behavior; the seed only distinguishes runs of identical inputs.

## 10. Faults and overrides

A case applies a list of these to the base scenario. Messages are selected by
`{ type, sender, n }`: the n-th message of that type from that sender.

| `kind` | Fields | Effect |
|---|---|---|
| `drop` | `match` | The message is never delivered |
| `delay` | `match`, `by` (duration) | Delivered `by` later than normal |
| `duplicate` | `match`, `after` (duration) | A byte-identical copy is delivered `after` the original |
| `replay` | `match`, `at` (time) | A byte-identical copy is delivered at `at` |
| `strip_signature` | `match` | The `signature` field is removed in transit |
| `corrupt_signature` | `match` | The signature value is altered in transit |
| `inject` | `at`, `message` | A message built from the given fields is signed by its sender and sent at `at`. `message` may set `sequence` and `message_id` explicitly, to test stale and equivocating messages (0008, Q9); otherwise they follow 5.2 |
| `silence` | `participant`, `from` (a message delivery) | From that delivery on, including it, the participant handles no message, sends nothing and starts no action. *Erratum (0011, N3):* hold expiry is protocol timing and continues. A message the participant holds is still released on the Core 8.4 schedule: `rejected C301` at expiry, or `not_handled` at `max_hold`, followed by later held messages of its stream that are now next. Each held message gets exactly one release receipt (gate 2 6.3) |
| `partition` | `between` (two participants), `from` (a message delivery), `until` (time) | Starting just after that delivery and until `until`, every message between the two is dropped |
| `sensor_lag` | `participant`, `sensor`, `lag` | Observations are dated `lag` earlier than the moment they are made |
| `sensor_unavailable` | `participant`, `sensor`, `from` (time) | No observations from then on |
| `override` | `path`, `value` | Replaces one value in the scenario before the run (thing mass, a descriptor capability, a policy flag, an authority answer) |

`path` is a JSON Pointer (RFC 6901) into the scenario file. A descriptor or standing
authorization changed by an `override` is signed again by its signer before the run, so that
an override changes content, not signature validity.

Faults change transport, sensing and scenario data only; they never change a participant's
rules.

## 11. Fixture files

[`tests/fixtures/arch02/`](../tests/fixtures/arch02/README.md):

- `scenario.json` — `robolang.arch02.scenario` version `0.1`: the deployment parameters
  (section 3, `parameters`), `conversation_id`, `participants`, `keys`, `authorities`, the
  capability `descriptors`, `standing_authorizations`, `places`, `things`, `initial_facts`,
  `initial_custody`, `sensors`, `durations`, `arbiters` (7.5), `peers` (whom a robot may ask
  for assistance), `policies`, the `authority_table` (7.3), the `goal` planner-1 proposes,
  its `goal_recipient`, and the `handover_criteria` arm-02 puts in `transfer_custody`.
- `cases.json` — one case per scenario acceptance test, A-T1 to A-T15 (section 12): `id`,
  `tests`, `seed`, `faults` and `expect`. A-T16 to A-T19 use the Core fixtures, the schema
  files and the existing suite instead.
- `sequencing_cases.json` — the cases of 8.1, in the same form, for A-T20 (0008, Q9).

`expect` fields (absent means not checked):

| Field | Checked against |
|---|---|
| `planner_status` | planner-1's recorded status for the goal |
| `arm02_goal_state` | arm-02's final state for the goal (RFC-001 9.2) |
| `message_types` | The `(type, sender, recipient)` of every sent message, in order (happy path) |
| `messages_present`, `messages_absent` | `(type, sender)` pairs that must, or must not, appear |
| `rejections` | `(type, receiver, code)` for every rejected receipt, in order |
| `duplicates` | `(type, receiver)` for every duplicate receipt |
| `motion` | Motion event count per participant |
| `custody` | `designated` holder of C42, and each robot's final local custody state |
| `reasons` | Reason codes of the outcome or reject that planner-1 receives last |
| `escalated` | Participants that recorded `escalated` |
| `final_world` | A subset of the final facts |
| `held` | `(type, receiver, held at, released at, result)` for every held message, in order; result `accepted` or a code (0008) |
| `processed_by` | Per participant, the types of the messages it processed (accepted receipts), in processing order (0008) |

## 12. Acceptance tests

| # | Test | Case(s) | Expected |
|---|---|---|---|
| A-T1 | Happy path | `happy_path` | The 20 messages of 6.1 in order; planner-1 records `succeeded` with verification; designated custody amr-07; C42 at S3 |
| A-T2 … A-T14 | Failure variants V1 … V13 | `v01_declines` … `v13_no_grasp` (14 cases: V12 has two) | Section 8 |
| A-T15 | Determinism | Every case, run twice; and `happy_path` with seeds `0` and `1` | Byte-identical records with the same run ID; a different seed gives a different run ID and an otherwise identical record |
| A-T16 | Core fixtures | `tests/fixtures/core/` | Every valid document is accepted by the Core 0.1 checks; every invalid one is rejected with its stated code |
| A-T17 | Schemas | `spec/core/*.schema.json` | Both are valid JSON Schema 2020-12; every valid Core fixture validates; every schema-stage invalid fixture fails (R12) |
| A-T18 | Isolation and mode | All runs | The package imports no `rclpy`, `robolang_adapter*`, `socket`, `http` or `urllib` module; every record says `"mode": "simulation"`; a standing authorization or decision with mode `physical` is rejected (`C404`) |
| A-T19 | Regression | — | The RoboLang 1.1 and 1.2 suites, every IR snapshot, every RL-BASIC-01 and LANG-1 simulation case and run-ID vector pass unchanged |
| A-T20 | Sequencing (0008) | `sequencing_cases.json` (8 cases) | Section 8.1; a held message has no effect before it is released |

**Exit gate (gate 1 of RFC-001 1.4):** A-T1 to A-T20 pass (A-T20 added by 0008, approved 2026-10-27) in CI on every supported platform;
the README states that the reference simulation is simulation only; owner approval.

## 13. Delivery

| Slice | Content | Tests |
|---|---|---|
| **RL-ARCH-02a** — this specification | Core 0.1, the two schemas, this document, decision record 0007, fixtures and a consistency test that executes nothing | Fixture consistency |
| **ARCH-1** | `robolang.core`: strict JSON reading, Core 0.1 schema and rule checks, JCS canonical form, `sim-hmac-sha256`, receipt codes | A-T16, A-T17 |
| **ARCH-2** | `robolang.arch02`: symbolic world, sensors, scheduler and delivery, faults, participants and policy | A-T1, A-T2 to A-T14 in progress |
| **ARCH-SEQ** | Decision record 0008 implemented in `robolang.arch02` (approved and authorized 2026-10-27) | A-T20, and A-T1 to A-T15 unchanged |
| **ARCH-3** | Run record and run ID, a `robolang arch02` command or test entry point, README; isolation | A-T1 to A-T20 |

Each slice is one pull request and needs owner approval to merge. ARCH-1 starts only after
RL-ARCH-02a is approved and implementation is authorized (D8).

**Package placement.** `robolang.core` and `robolang.arch02` are new subpackages next to
`robolang.sim`. They do not import the RoboLang 1.x parser, IR or `robolang.sim`, and change
none of them (R13).

## 14. What the reference simulation does not model

Geometry, frames, kinematics and collision; real time; clock skew; transport behavior beyond
latency and the scripted faults; key distribution and revocation; crash and restart from
durable state (RFC-001 12, last rows); more than one conversation; any binding, including
A2A; any AI model; physical execution.

## 15. Implementation status

| Part | Status |
|---|---|
| Core 0.1 per-message checks, schemas, canonical form, `sim-hmac-sha256` (ARCH-1) | Implemented in `src/robolang/core/`; A-T16 and A-T17 pass (`tests/test_core.py`) |
| World, participants, delivery, faults, conversation-state checks (ARCH-2) | Implemented in `src/robolang/arch02/`; A-T1 to A-T14 pass, A-T15 in memory (`tests/test_arch02.py`) |
| Sequencing per recipient (decision record 0008) | Implemented in `src/robolang/arch02/engine.py` (ARCH-SEQ); A-T20 passes, and A-T1 to A-T15 are unchanged (`tests/test_arch02.py`) |
| Run record, run ID, entry point, README (ARCH-3) | Implemented in `src/robolang/arch02/record.py` and `robolang arch02` (`cli.py`); A-T15 at full strength and the record shape pass (`tests/test_arch02_record.py`) |
| Scenario fixtures (section 11) | Run by `tests/test_arch02.py`; consistency also checked by `tests/test_arch02_fixtures.py` |
