# RL-INTEROP-01 — Gate 2: independent interoperability, version 0.1 (draft)

**Status:** specification, **proposed for owner review**. Every choice is a row in
[decision record 0009](decisions/0009-gate-2-independent-interoperability.md) (I1–I12),
status PROPOSED. **No work under it is authorized**: each slice of section 11 needs its own
authorization.
**Gate:** Gate 2 — independent implementation ([RFC-001 section 1.4](architecture/rfc-001-ai-native-architecture.md#14-how-the-ambition-is-tested);
decision D9).
**Builds on:** [Core 0.1](core-v0.1.md), [reference simulation 0.1](reference-simulation-v0.1.md),
decision records [0007](decisions/0007-rl-arch-02-core-and-reference-simulation.md) and
[0008](decisions/0008-core-sequencing-per-recipient.md); gate 1 passed 2026-10-27
([evidence](../docs/evidence/rl-arch-02-exit-gate.md)).

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
the 6.1 timeline, with Core 0.1 for everything on the wire. This specification does not restate
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
| `tests/fixtures/arch02/scenario.json`, `cases.json`, `sequencing_cases.json` | Configuration and cases |

Nothing else: no source code, no run records, no `golden_records.json`, no test code.

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
  `idle`.
- **The harness owns time.** Every harness frame carries `now_ms` (milliseconds after the
  scenario epoch). The participant has no other clock, no randomness and no I/O besides the
  binding.
- **Faults are harness-side.** Drops, delays, duplicates, replays, altered signatures,
  partitions and silence (reference simulation 10) are applied by the harness; the participant
  is not told about them.

### 6.3 Frames

**Harness → participant**

| `frame` | Fields | Meaning |
|---|---|---|
| `start` | `now_ms`, `participant`, `scenario` (the scenario document after the case's overrides), `case` | Begin a run; the participant reads its keys, descriptor, standing authorization and policy from `scenario` |
| `deliver` | `now_ms`, `data` (the message text exactly as delivered, which may be altered) | A message arrives; the participant applies Core 8 and its rules |
| `timer` | `now_ms`, `timer` | A timer the participant set has fired |
| `sensed` | `now_ms`, `request`, `value` (or `null`), `observed_at_ms` | Answer to `sense`: the sensor's reading, already lagged; `null` if the sensor is unavailable or cannot see the thing |
| `action_done` | `now_ms`, `action` | An action the participant started has completed |
| `end` | — | The run is over; the participant exits |

**Participant → harness**

| `frame` | Fields | Meaning |
|---|---|---|
| `send` | `message` (a Core 0.1 message object) | Send a message; the harness serializes it, applies faults and delivers it |
| `receipt` | `message_id`, `sender`, `receipt`, `code` | The participant's receipt for a delivered message (Core 8: `accepted`, `duplicate`, `held`, `rejected` with a code, or for a released held message a second receipt) |
| `set_timer` | `timer`, `at_ms` | Ask for a `timer` frame at `at_ms` |
| `sense` | `request`, `sensor`, `thing`, `property` | Read one of its own sensors now; answered by `sensed` before the next frame |
| `act` | `action`, `kind` (`move`), `to` | Start an action; the harness records a motion event now and sends `action_done` after the scenario's duration |
| `event` | `kind` (`state`, `authorization`, `escalated`, `custody_committed`), `detail` | Facts the run record and expectations need (reference simulation 9, 11): goal, commitment and custody state changes, authorization decisions and their basis, escalations, the commit point |
| `idle` | — | Done with the current harness frame |

The participant builds and signs its own observations (Core 5.6, 7) from `sensed`: it is the
source, and its key signs them.

## 7. Prerequisite: conformance kit (G-T0, I12)

Before any scenario run, the independent implementation's receiver checks must give the stated
receipt for all 30 Core fixtures (A-T16): a `check` mode reads one fixture and the context of
`tests/fixtures/core/context.json` and prints the code and stage, or `accepted`. This is
level 3 of section 2 and the first thing the independent author can test alone.

## 8. Interoperability tests

| # | Test | Cases | amr-07 role |
|---|---|---|---|
| G-T0 | Conformance kit | 30 Core fixtures | Receiver checks |
| G-T1 | Happy path | `happy_path`, `happy_path_seed_1` | The full role |
| G-T2 | amr-07 behavior under faults | `v01_declines`, `v02_duplicate_commit`, `v03_replayed_commit`, `v05_stale_confirmation`, `v06_partition_before_release`, `v07_release_ack_lost`, `v08_arrived_without_evidence`, `v11_payload_beyond_standing_authorization`, `v12a_unsigned_commit`, `v12b_wrongly_signed_commit` | Exercised directly |
| G-T3 | Sequencing (0008) | All 8 `seq_*` cases | Exercised directly where amr-07 sends or receives the affected stream (`seq_replay_while_fresh`, `seq_lost_ack_holds_next`); otherwise a regression check, since amr-07 numbers and receives its own streams in every case |
| G-T4 | Regression | `v04_assistant_silent`, `v09_stale_reverification`, `v10_requester_authorizes`, `v13_no_grasp` | Must still behave correctly although the fault is elsewhere |
| G-T5 | Determinism | Every case run twice | The independent run is repeatable: byte-identical gate 2 records |

## 9. Success criteria, failure conditions and findings (I7, I8)

### 9.1 Required for each case

1. Every `expect` field of the case (reference simulation 11) holds for the run.
2. Every receipt the independent amr-07 reports equals the receipt the reference amr-07 gives
   for the same delivery (same `receipt` and `code`).
3. The sequence of sent messages, by type, sender, recipient and send time, equals the
   reference run's.
4. Lock-step and determinism hold (G-T5); the participant never times out or breaks framing.

### 9.2 Measured and reported, not required

- **JCS identity:** for each message, whether its JCS form (Core 7) equals the reference's.
  Differences are listed and classified (9.3); identity is the expected result where the
  specification fixes every field.
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
| **INTEROP-1** — binding self-test | The harness side of `robolang-sim-stdio` in this repository, and the reference amr-07 run *as an external process* through it | All 24 cases pass, and the records are identical to the F1 golden records. Shows the binding is lossless and the harness is correct before anyone else depends on it |
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
| This specification, decision record 0009 | Proposed; not approved |
| INTEROP-1 to INTEROP-4 | Not started; not authorized |
