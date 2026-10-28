# RoboLang Core — version 0.1 (draft)

**Status:** RL-ARCH-02a specification, **approved by the owner on 2026-10-27**. Every choice is a row in
[decision record 0007](decisions/0007-rl-arch-02-core-and-reference-simulation.md) (R1–R14),
approved 2026-10-27.
**Version:** `0.1`. Experimental: until Core 1.0 any change may be breaking (RFC-001 6.2).
**Erratum (approved 2026-10-27):** sections 3 and 8 are amended by
[decision record 0008](decisions/0008-core-sequencing-per-recipient.md) (Q1–Q9, approved 2026-10-27):
sequence numbers per sender and recipient, and holding on gaps. The amended text is marked.
**Erratum (approved 2026-10-28):** sections 8.1 and 8.4 are clarified by
[decision record 0011](decisions/0011-interop-2a-bundle-errata.md) (N2, N13): precedence within
stages, exact time comparisons, ties, and the repeated reply. The amended text is marked.
**Erratum (approved 2026-10-28):** sections 8.1, 8.2 and 8.4 are amended by
[decision record 0012](decisions/0012-interop-2c-remaining-normative-decisions.md) (P2, P3): the
stage-2 order for one signed object, and a sequence that conflicts with a held message. The
amended text is marked.
**Builds on:** [RFC-001](architecture/rfc-001-ai-native-architecture.md) sections 6–13
(decisions D1–D9), unchanged.
**Schemas:** [`core/message-0.1.schema.json`](core/message-0.1.schema.json),
[`core/capability-descriptor-0.1.schema.json`](core/capability-descriptor-0.1.schema.json)
(JSON Schema draft 2020-12)
**Fixtures:** [`tests/fixtures/core/`](../tests/fixtures/core/README.md)
**Implementation:** the per-message checks of sections 2 to 8 are implemented in
`src/robolang/core/` (ARCH-1, authorized 2026-10-27); duplicates, ordering, `C306` and `C403`
need conversation state and come with the participants (ARCH-2).

This document is the RoboLang Core contract at version 0.1: the documents two participants
exchange, what every field means, and what a receiver must check. It is written so that a
participant can be implemented from it and the two schema files alone, in any language,
without the RoboLang Python package (RFC-001 P10, gate 2). The reference simulation that
exercises it is specified separately, in
[`reference-simulation-v0.1.md`](reference-simulation-v0.1.md).

> **A valid Core document is never permission to act.** Schema validity, a correct signature
> and a known sender make a message well-formed and attributable. Whether a robot acts is
> decided on the robot, under its local policy (RFC-001 13.2). Core 0.1 is exercised only in
> simulation; nothing in it enables physical execution (plan decision A5).

---

## 1. Scope

Core 0.1 defines exactly what the RFC-001 section 18 reference scenario and its failure
variants need (R1):

- the message envelope (section 3);
- 20 message types with defined payloads, and 10 reserved type names (section 4);
- the object kinds those payloads carry (section 5);
- the capability descriptor (section 6);
- signatures (section 7);
- what a receiver checks, in what order, with stable codes (section 8).

Not in 0.1 (section 10): discovery, subscription, negotiation, progress, cancellation, lease
renewal and early release, observation sharing, plan exchange, multi-party coordination, and
any binding or transport.

## 2. Documents and common forms

### 2.1 Two document types

| `schema` | `schema_version` | Document | Schema file |
|---|---|---|---|
| `robolang.core.message` | `0.1` | A message: envelope and payload (section 3) | `core/message-0.1.schema.json` |
| `robolang.core.capability-descriptor` | `0.1` | A robot's capability descriptor (section 6) | `core/capability-descriptor-0.1.schema.json` |

Objects inside a document are identified by `kind` and defined by the enclosing document's
schema version. There is no separate version per object kind (RFC-001 6.1).

### 2.2 Versioning

As RFC-001 6.2. A receiver checks `schema` and `schema_version` first and rejects anything it
does not support (`C102`). Correct version identifiers are claims, not evidence.

### 2.3 JSON

- UTF-8, RFC 8259. A leading byte order mark is rejected.
- Duplicate keys in any object are rejected (`C101`).
- **Numbers are integers only**, and appear only where a field table says "integer"
  (`sequence`, `renewal`, `criterion`). Every measured value is an exact decimal string. A
  JSON number with a fraction or exponent is rejected (`C101`). The JSON reader enforces this,
  not the schema files: JSON Schema's `integer` type accepts `1.0`, so a receiver must not
  rely on schema validation alone for this rule.
- Unknown fields are rejected at every level (`C103`). Core 0.1 has no extension points.
- Every field listed for an object is required. Where a value may be absent, the table says
  "or `null`", and the field is still present. One document shape per object keeps
  generation and validation unambiguous (charter AF3, MC1).

### 2.4 Common value forms

| Form | Rule | Example |
|---|---|---|
| Identifier | `rl:` + a lowercase type word + `:` + 1 to 128 characters from `A–Z a–z 0–9 . _ : @ # -`, starting with a letter or digit. Opaque: no meaning may be derived from the spelling (RFC-001 7.1) | `rl:robot:amr-07`, `rl:capdesc:amr-07@3` |
| Key reference | An identifier with a `#` fragment naming one key of a participant | `rl:robot:amr-07#k1` |
| Message ID | 1 to 128 characters from `A–Z a–z 0–9 . _ -`. Opaque; unique per sender | `m-3f9a…` |
| Time | UTC, RFC 3339, `YYYY-MM-DDTHH:MM:SS` with an optional fraction of 1 to 3 digits, and `Z` | `2026-10-19T10:14:05.004Z` |
| Decimal | Digits with an optional fractional part; no sign, exponent or spaces (IR schema 1.0, 3.5) | `"25"`, `"0.93"` |
| Quantity | `{ "value": <decimal>, "unit": <unit> }`. Units: `ms`, `s`, `min` (duration); `kg` (mass); `m/s` (speed). Values are greater than zero | `{ "value": "30", "unit": "s" }` |
| Confidence | A decimal from 0 to 1 inclusive | `"0.95"` |
| Reason | `{ "code": <reason code> }`; the codes are listed in section 5.11 | `{ "code": "assistance_required" }` |

## 3. Message envelope

A `robolang.core.message` document. Every field is required.

| Field | Content |
|---|---|
| `schema`, `schema_version` | `"robolang.core.message"`, `"0.1"` |
| `message_id` | Unique per sender; makes handling idempotent (8.4) |
| `conversation_id` | Identifier of the conversation. In 0.1 a conversation is one top-level goal and everything done for it |
| `sequence` | Integer ≥ 1. Counts per sender, recipient and conversation: the first message a sender sends to a recipient in a conversation has sequence 1, and each further one to the same recipient increases it by one (amended by 0008, Q1) |
| `in_reply_to` | The `message_id` this message answers, or `null` |
| `profile` | `"a2r"` or `"r2r"`, as section 4 fixes for the type (`C107`) |
| `type` | A defined type (section 4.1). A reserved name is `C106`, anything else `C105` |
| `sender`, `recipient` | Participant identifiers |
| `sent_at`, `expires_at` | Times. `expires_at` is after `sent_at` and at most the deployment's maximum message lifetime after it (`C302`) |
| `payload` | The object section 4 defines for `type` |
| `signature` | Section 7 |

The envelope is the same over every binding (RFC-001 14).

## 4. Message types

### 4.1 Defined in 0.1

| Type | Profile | From → to | Payload fields | Meaning |
|---|---|---|---|---|
| `propose_goal` | a2r | agent → robot | `goal`, `things` | Ask the robot to bring about the goal |
| `assessment` | a2r | robot → agent | `goal`, `result`, `reasons` | `result`: `feasible`, `feasible_with_assistance`, `feasible_with_changes`, `infeasible`. A later assessment for the same goal supersedes an earlier one |
| `accept` | a2r | robot → agent | `goal` | The robot takes on the goal. Accepting is not authorizing |
| `reject` | a2r | robot → agent | `goal`, `reasons` | The robot declines the goal; terminal |
| `authorization_request` | a2r | robot → its authority | `goal`, `things`, `mode`, `reasons` | No standing authorization covers the goal (RFC-001 13.3) |
| `authorization_decision` | a2r | authority → robot | `goal`, `decision`, `mode`, `constraints`, `lease` | `decision`: `granted` or `denied`. Valid only from the recipient's authority (`C401`) |
| `request_assistance` | r2r | robot → robot | `goal`, `things`, `parent_goal` | Ask a peer to take on a sub-goal |
| `offer_assistance` | r2r | robot → robot | `goal`, `conditions`, `lease` | Offer to take it on, with conditions and a proposed lease |
| `decline` | r2r | robot → robot | `goal`, `reasons` | Refuse the request |
| `commit` | r2r | robot → robot | `commitment` | Turn the offer into a commitment, on the offered conditions and lease |
| `reserve` | r2r | robot → resource arbiter | `resource`, `commitment`, `lease` | Request a leased hold on a shared resource |
| `reservation_decision` | r2r | arbiter → robot | `resource`, `decision`, `lease` | `granted` or `denied`. A reservation without a granted lease is not a reservation (RFC-001 10.3) |
| `coordinate` | r2r | robot → robot | `commitment`, `phase` | `phase`: `ready_for_handover` or `placed` |
| `transfer_custody` | r2r | giver → receiver | `thing`, `commitment`, `criteria` | Offer custody of a thing; `criteria` are what the receiver's confirming observation must meet |
| `custody_confirmed` | r2r | receiver → giver | `thing`, `observations` | The receiver's own fresh observations of the thing (RFC-001 10.4 rule 1) |
| `custody_released` | r2r | giver → receiver | `thing`, `confirmation` | Release, naming the `custody_confirmed` message it rests on. Durable acceptance by the receiver is the commit point (rule 3) |
| `acknowledge` | r2r | either | `acknowledged`, `accepted` | Answers the message named in `acknowledged` (also given in `in_reply_to`); `accepted` is `true` or `false` |
| `custody_query` | r2r | giver → receiver | `thing` | A giver in doubt asks which release, if any, the receiver accepted (rule 8) |
| `custody_state` | r2r | receiver → giver | `thing`, `accepted_release`, `observations` | `accepted_release`: the accepted `custody_released` message ID, or `null` |
| `outcome` | a2r or r2r | robot → requester | `outcome`, `evidence` | Final outcome of a goal or sub-goal, with the observations its verification cites |

`acknowledge`, `authorization_decision` and `reservation_decision` are not named in RFC-001
9.1 and 10.1. They are the replies those sections imply but do not name (R4).

### 4.2 Reserved in 0.1

These names are listed in RFC-001 9.1 and 10.1 and keep their meaning there. Their payloads
are not defined in 0.1, and a message using one is rejected with `C106`:

```text
discover observe subscribe negotiate progress cancel advertise renew release share_observation
```

## 5. Object kinds

### 5.1 Goal

| Field | Content |
|---|---|
| `kind` | `"goal"` |
| `id` | Identifier |
| `requested_by` | The requesting participant |
| `desired_state` | At least one triple (5.2) |
| `success_criteria` | At least one criterion (5.3). A goal without criteria is invalid (RFC-001 P5) |
| `constraints` | Constraints (5.4); may be empty |
| `priority` | `low`, `normal` or `high` |

A goal says what must be true and how it is checked, never how (RFC-001 7.7). Which class of
goal it is, for standing authorizations (7.6 of the simulation), is decided by the robot's
local policy and is not a Core field (R6).

### 5.2 Triple and properties

`{ "subject": <identifier>, "property": <property>, "value": <identifier> }`. Core 0.1
properties:

| `property` | `subject` | `value` |
|---|---|---|
| `located_in` | A thing or robot | A place |
| `carried_by` | A thing | A robot |

### 5.3 Success criterion

| Field | Content |
|---|---|
| `kind` | `"observed"` |
| `subject`, `property`, `value` | The triple that must be observed |
| `min_confidence` | Confidence |
| `max_age` | Duration quantity |
| `accepted_sources` | Non-empty list of participant identifiers or the word `any_participant_in_domain` |

### 5.4 Constraints

| `kind` | Other fields |
|---|---|
| `deadline` | `at`: time |
| `max_speed` | `limit`: speed quantity |
| `avoid_place` | `place`: identifier |

### 5.5 Thing

`{ "kind": "thing", "id", "class", "mass" }`. `class` is a lowercase word; `mass` is a mass
quantity with an additional `uncertainty` decimal in the same unit.

### 5.6 Observation

| Field | Content |
|---|---|
| `kind` | `"observation"` |
| `id` | Identifier |
| `subject`, `property`, `value` | What is believed (5.2) |
| `observed_at` | Time |
| `valid_for` | Duration quantity |
| `confidence` | Confidence |
| `source` | `{ "participant", "sensor", "method" }`; `sensor` and `method` are non-empty strings |
| `signature` | Section 7, by `source.participant` |

An observation past `observed_at + valid_for` is stale and counts as unknown. A received
observation is evidence of its source's belief, not a fact (RFC-001 7.6, 11.3).

### 5.7 Lease

`{ "granted_at", "expires_at", "renewal" }`: times and an integer ≥ 0 (0 in Core 0.1, which
defines no renewal). Never open-ended (RFC-001 11.1).

### 5.8 Commitment and condition

| Field | Content |
|---|---|
| `kind` | `"commitment"` |
| `id` | Identifier |
| `debtor`, `creditor` | Participants |
| `obligation` | `{ "goal": <identifier> }` |
| `conditions` | Conditions; in 0.1 the only kind is `{ "kind": "resource_held", "resource": <identifier> }` |
| `lease` | Lease (5.7) |
| `state` | `active` in a `commit` message. Local records also use `fulfilled`, `lapsed` and `failed` |

### 5.9 Outcome and verification

| Field | Content |
|---|---|
| `kind` | `"outcome"` |
| `goal` | Identifier of the goal or sub-goal |
| `status` | `succeeded`, `failed`, `cancelled`, `timed_out`, `rejected` or `unknown` |
| `verification` | The object below, or `null` |
| `reasons` | Reasons; may be empty |
| `reported_by`, `reported_at` | Participant and time |

Verification: `{ "verified_by", "method", "evaluated_at", "criteria_results" }`. `method` is
`criteria_evaluated` or `self_reported` (RFC-001 11.3). Each criterion result is
`{ "criterion": <integer index from 0>, "met": true | false, "evidence": [<observation id>…] }`.

`succeeded` requires a verification in which every criterion is met (`C402`). Every
observation ID a verification cites is carried in the `outcome` message's `evidence` list,
so the receiver can re-evaluate it.

### 5.10 Standing authorization and participant

Neither is sent in a 0.1 message; both are configuration a deployment installs. Their forms
are those of RFC-001 13.3 and 7.2 and are defined in the message schema's `$defs`, so that
configuration can be validated with the same schema. `mode` is `simulation` or `physical`;
Core 0.1 participants reject `physical` (`C404`). One change from the RFC-001 7.2 example: a
participant lists `keys` (key references) instead of one `public_key`, because key material
depends on the signature algorithm (section 7) and is part of the deployment's configuration.

### 5.11 Reason codes

| Code | Used in |
|---|---|
| `capability_unsupported` | The descriptor lists the needed capability as unsupported, or not at all |
| `capability_insufficient` | Supported, but outside its limits (mass, place) |
| `assistance_required` | Feasible only with a peer's help |
| `assistance_unavailable` | The peer declined, or no commitment was reached |
| `participant_busy` | The participant declines because it is committed elsewhere |
| `outside_standing_authorization` | No standing authorization covers the goal |
| `authorization_denied` | The authority denied the goal |
| `lease_expired` | A commitment or reservation lapsed |
| `evidence_missing` | No observation supports a criterion |
| `evidence_stale` | Supporting observations are older than allowed |
| `custody_unknown` | Custody of a thing cannot be established |
| `deadline_passed` | A deadline constraint expired |

## 6. Capability descriptor

A `robolang.core.capability-descriptor` document (RFC-001 7.3).

| Field | Content |
|---|---|
| `schema`, `schema_version` | `"robolang.core.capability-descriptor"`, `"0.1"` |
| `id` | Identifier, conventionally with a version suffix (`@3`) |
| `robot` | The robot it describes |
| `capabilities` | List of capabilities below |
| `unsupported` | Capability kinds known not to work. "Not listed" and "unsupported" differ |
| `issued_at`, `valid_until` | Times. A descriptor past `valid_until` is treated as absent |
| `signature` | Section 7, by the robot |

| Capability `kind` | Fields |
|---|---|
| `transport_payload` | `max_mass` (mass), `max_speed` (speed), `operating_area` (place) |
| `dock_at` | `places`: places the robot can dock at |
| `grasp` | `max_mass` (mass), `reach`: places within reach |
| `inspect` | `sensor`, `properties`: properties it can observe, `places` |

A capability is declared, not proven (RFC-001 7.3). A kind appears at most once, either in
`capabilities` or in `unsupported`.

## 7. Signatures

`{ "alg", "key", "value" }`. The signature covers the document or object it is in, without
its `signature` field, in the **canonical form of RFC 8785 (JCS)**. Because Core documents
contain no non-integer numbers, JCS here is: object keys sorted by code point, no
whitespace, strings as JSON escapes them, UTF-8.

| `alg` | Meaning | Status in 0.1 |
|---|---|---|
| `ed25519` | Ed25519 (RFC 8032) over the canonical bytes; `value` is the unpadded base64url signature | Defined, for gate 2 and later. **Not implemented by the reference simulation** |
| `sim-hmac-sha256` | HMAC-SHA-256 over the canonical bytes with a secret the deployment assigns to the key; `value` is lowercase hex | **Simulation only.** It shows that a check happens and that a wrong key fails; it is not an identity proof, because the secret is shared with the verifier. A participant in physical mode rejects it (`C204`) |

The key named in `key` must belong to the participant that signs: the `sender` for a message,
`source.participant` for an observation, `robot` for a descriptor, `issued_by` for a standing
authorization (`C203`). Revocation and key distribution are RFC-001 open question 5; in 0.1
the deployment's configuration lists the keys (R5).

## 8. What a receiver checks

### 8.1 Order

A receiver handles each message in this order and stops at the first failure:

1. **Form:** JSON rules (`C101`), version (`C102`), schema (`C103`–`C107`).
2. **Signature** (`C201`–`C204`), before anything else about the content is considered
   (RFC-001 11.2).
3. **Freshness:** expiry, lifetime and clock (`C301`–`C303`), then **recipient** (`C307`).
4. **Duplicate and order** (8.4). Only a message that passed stages 1 to 3 reaches this stage:
   a rejected message is never held and does not count in its stream (0008, Q2).
5. **Meaning:** authority, outcome and state rules (`C306`, `C401`–`C404`).

Two failures that the schema also detects are reported with their specific code, not the
general `C104`, as in RoboLang 1.1 (`robolang-v1.1.md` section 6): a message without a
`signature` field is `C201`, and an outcome `succeeded` whose `verification` is `null` is
`C402`.

Clock comparisons apply the deployment's clock tolerance in the safe direction (RFC-001
11.4): a message is treated as expiring earlier, as sent later and an observation as older.

*Erratum ([decision record 0011](decisions/0011-interop-2a-bundle-errata.md), N13).* The order of
the five stages above is normative and unchanged. The rules below fix what Core 0.1 left open
inside a stage.

**Stage 1, precedence.** A receiver applies these checks in this order and reports the first that
fails:

| Rank | Check | Code | Fixture evidence |
|---|---|---|---|
| 1 | The bytes are not a JSON document under 2.3 | `C101` | `c101_*` (nothing else can be checked) |
| 2 | `schema` and `schema_version` | `C102` | `c102_unsupported_version` also fails the schema (rank 7) |
| 3 | `type` is reserved (4.2) | `C106` | `c106_reserved_type` also fails rank 7 |
| 3 | `type` is neither defined nor reserved | `C105` | `c105_unknown_type` also fails rank 7. `C105` and `C106` cannot both apply |
| 4 | No `signature` field | `C201` | `c201_unsigned` also fails rank 7 |
| 5 | `profile` not allowed for `type` (4.1) | `C107` | `c107_profile_mismatch` also fails rank 7 |
| 6 | An `outcome` `succeeded` with `verification: null` | `C402` | `c402_succeeded_without_verification` also fails rank 7 |
| 7 | The schema: `C103` if any field is unknown, otherwise `C104` | `C103` / `C104` | `c103_unknown_field`, `c104_*` |
| 8 | A time value that does not parse (2.4) | `C104` | — |
| 9 | A lease whose `expires_at` is not after its `granted_at` (5.7) | `C104` | — |

Ranks 3 to 6 apply to messages only. The fixtures establish only that ranks 2 to 6 come before
rank 7, and that rank 1 comes first. Three other orderings are fixed by decision record 0011
(N13) but have **no fixture yet**:

- rank 2 before ranks 3 to 6;
- the order among ranks 3 to 6;
- `C103` before `C104` when a document has both an unknown field and another schema error.

Ranks 7 to 9 give the same code, so their order changes only the JSON Pointer of the receipt,
not the code.

**Stage 2.** The message's own signature is checked first, then the signature of each
observation it carries in `payload.observations` and then `payload.evidence`, in array order.
Each observation's key must belong to its `source.participant` (`C203`), and a failure of any of
them rejects the message. For each signed object, the checks apply in this order and the first
failure is reported (decision record 0012, P3): `C204`, the algorithm is not accepted by this receiver in its mode;
`C203`, the key is unknown, revoked, or not the signer's; `C202`, the signature does not
verify.

**Stage 3, precedence.** In this order:

1. `C302`: `expires_at` is not after `sent_at`, or it is beyond the maximum lifetime.
2. `C301`: expired.
3. `C303`: `sent_at` is in the future.
4. `C307`: the wrong recipient.

The comparisons are exact:

- a message is expired when `now ≥ expires_at − clock_tolerance`;
- it is in the future when `sent_at > now + clock_tolerance`.

Each stage-3 fixture has one fault only, so this order rests on decision record 0011 alone.

**Stage 5.** The meaning codes (`C306`, `C401`, `C402` for a verification with an unmet criterion,
`C403`, `C404`) are applied **after** duplicate and order handling (stage 4). A conformance check
of a single message, with no conversation state, gives the same code. *Known reference
discrepancy D7:* the reference receiver currently applies `C401`, `C402` and `C404` before
stage 4 ([register](../docs/interop/spec-implementation-discrepancies.md)).

### 8.2 Rejection codes

| Code | Stage | Meaning |
|---|---|---|
| `C101` | form | Not UTF-8 JSON, a byte order mark, a duplicate key, or a non-integer number |
| `C102` | form | `schema` or `schema_version` missing or not supported |
| `C103` | form | Unknown field |
| `C104` | form | A required field is missing or a value has the wrong form, including an empty `success_criteria` |
| `C105` | form | Unknown message type |
| `C106` | form | Reserved message type, not defined in 0.1 |
| `C107` | form | `profile` does not match `type` |
| `C201` | signature | No signature |
| `C202` | signature | The signature does not verify |
| `C203` | signature | The key is unknown, revoked, or not the signer's |
| `C204` | signature | The algorithm is not accepted by this participant in its mode |
| `C301` | freshness | Expired: `expires_at` is past |
| `C302` | freshness | `expires_at` is not after `sent_at`, or is beyond the maximum lifetime |
| `C303` | freshness | `sent_at` is in the future beyond the clock tolerance |
| `C306` | meaning | The message would cause an impossible state transition; or it is stale: at or below the last processed or skipped sequence of its stream, and not a duplicate (8.4); or its sequence conflicts with a message currently held in its stream (8.4) |
| `C307` | freshness | `recipient` is not this participant |
| `C401` | meaning | Authority: a decision not from the recipient's authority, or a requester authorizing its own goal |
| `C402` | meaning | `succeeded` without a verification in which every criterion is met |
| `C403` | meaning | A `custody_released` that does not name a `custody_confirmed` this receiver sent for the same thing |
| `C404` | meaning | `physical` mode requested, granted or configured |

A message rejected at stages 1 to 3 gets no reply: an unauthenticated sender is not answered.
A message rejected at stage 5 may be answered under local policy (for example `reject` or
`acknowledge` with `accepted: false`). Every rejection is recorded by the receiver.

### 8.3 Evaluation is not rejection

Evaluating evidence against a criterion (stale, wrong source, low confidence) is not a
message rejection. It makes the criterion not met and the outcome `failed` or `unknown`
(RFC-001 11.3, 12).

### 8.4 Duplicates and order

*Amended by decision record 0008 (Q2–Q6).* A **stream** is the messages one sender sends to
one recipient in one conversation (section 3).

- **Duplicates.** A receiver keeps the message IDs it has processed or is holding, per
  sender, until each message's `expires_at` plus the clock tolerance. A repeated message ID
  is not processed again and is not held again; the reply sent to the original, if any, is
  sent again.
- **Order.** The receiver processes each stream in `sequence` order, starting at 1.
- **Gaps.** A message whose sequence is above the next expected one is **held**. A held
  message has no effect of any kind and gets no reply until it becomes eligible. It becomes
  eligible when the missing messages have been processed, or when the deployment's
  `max_hold` has passed since its delivery; in that case the missing sequence numbers are
  skipped. Held messages that become eligible together are processed in sequence order.
- **Expiry while held.** A held message that expires before it becomes eligible (judged in
  the safe direction, 8.1) is rejected with `C301` at that moment, and its stream advances
  past it.
- **Stale.** A message at or below the last processed or skipped sequence of its stream that
  is not a duplicate is rejected with `C306`.
- **Conflicting sequence** (decision record 0012, P2). A message whose `sequence` equals that
  of a message currently held in the same stream (same sender, recipient and
  `conversation_id`), but whose `message_id` differs, is rejected with `C306`. The original
  held message is unaffected and receives exactly one terminal release receipt.

`max_hold` is a deployment parameter shorter than the message lifetime. It bounds how long
one lost message can delay the messages after it.

*Erratum ([decision record 0011](decisions/0011-interop-2a-bundle-errata.md), N13, N2).*

- **Processed** means handed to the participant and accepted. A message rejected at stage 5 is
  not remembered: a later copy of it is judged again (and is stale, `C306`, if its stream has
  passed it), not treated as a duplicate.
- **Ties.** When a held message's expiry and its `max_hold` fall at the same instant, expiry
  wins (`C301`).
- **Conversation.** In 0.1 a receiver does not compare `conversation_id` with its own.
- **The repeated reply.** In the reference simulation and the gate 2 binding, the reply to a
  duplicate is repeated by the transport, not by the participant: the stored reply, unchanged,
  with its original ID and sequence
  ([`interop-gate-2-v0.1.md`](interop-gate-2-v0.1.md) 6.3). Its receiver therefore sees a
  duplicate, or the first copy if the original was lost, and never a new sequence number.

## 9. Conformance of documents

A document conforms to Core 0.1 when it validates against the schema file for its `schema`
and satisfies the rules of sections 2 to 7 that a schema cannot express: signature checks,
key ownership, time ordering (`expires_at` after `sent_at`, `valid_until` after `issued_at`,
lease end after start), the integer-only rule, uniqueness of capability kinds, and the
`succeeded` rule. `tests/fixtures/core/` holds valid documents for the message types listed in
its `cases.json` and both capability descriptors, and at least one invalid document for every
code a receiver can check on a single message. `C306` and `C403` depend on conversation state
and are covered by the reference scenario instead.

Conformance of a participant (what it does with a valid message) is tested by the reference
scenario, not by this document. A conformance programme is RFC-001 open question 13.

## 10. Not in Core 0.1

| Left out | Where it is decided |
|---|---|
| Discovery and version negotiation (`discover`, `advertise`) | A later Core version. 0.1 participants are configured with each other's versions and descriptors |
| Plans in messages (`plan_kind`, Task IR documents) | CORE-1 (IR reader) and a later Core version. 0.1 robots execute their own local plans (R3) |
| `negotiate`, `progress`, `cancel`, `renew`, `release`, `share_observation`, `observe`, `subscribe` | Reserved (4.2) |
| Places with geometry and frames (RFC-001 7.5) | Open question 2. 0.1 places are identifiers resolved by each robot |
| Multi-party commitments | D5: bilateral only in version 0 |
| `ed25519` in the reference implementation; key distribution and revocation | Open question 5 |
| Bindings and transports | RFC-001 14; open question 7 |
