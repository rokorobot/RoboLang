# RoboLang Core: version 0.2 (DRAFT delta over 0.1)

> **DRAFT. Not normative.** This page is a proposal and binds nobody. It follows the recommended options of
> [decision record 0028](decisions/0028-core-0-2-scope.md) (APPROVED by the owner, 2026-10-30; Core 0.2 is not
> frozen). Examples are illustrative, with signatures elided; the conformance fixtures are in
> [`tests/fixtures/core_v02/`](../tests/fixtures/core_v02/README.md).
>
> **Implementation status.** PR A (documents): [`core/message-0.2.schema.json`](core/message-0.2.schema.json), the
> 0.2 checks in `robolang.core` (`C102`, `C103`, `C104`, `C402`, `C405`; a receiver is configured for the versions it
> speaks) and the conformance fixtures are done. PR B (producers) is done: `export-goal --core 0.2`, the simulated
> outcome with `met_alternative` and `declared` (`simulate --core 0.2`), `delegate --core 0.2` and the bridge
> (section 7 is now implemented; see [export-goal 6](export-goal-v0.1.md) and [delegate-core-0.2](delegate-core-0.2.md)).
> Implemented, not frozen.

**Reads with:** [`core-v0.1.md`](core-v0.1.md). Everything in 0.1 that this page does not name is unchanged, with its
section numbers. A 0.2 text would be the 0.1 text with this delta applied.
**Version:** `0.2`. Experimental, like 0.1 (RFC-001 6.2).

## 1. Versioning and compatibility (decisions C7, C8)

| Document | `schema_version` in 0.2 | Change |
|---|---|---|
| `robolang.core.message` | `0.1` or `0.2` | 0.2 changes the goal, the verification and the reason (sections 2 to 4). The 0.1 schema file is not touched |
| `robolang.core.capability-descriptor` | `0.1` | None. It keeps its version and its signature |

- A 0.2 implementation accepts 0.1 and 0.2 messages. A 0.1 goal is read as a 0.2 goal with one alternative. A 0.1
  outcome is read as a 0.2 outcome with `alternative` 0 on every result and `met_alternative` 0 (when `succeeded`).
- A 0.1 document is judged by the 0.1 rules only. `alternatives`, `met_alternative` and `declared` are unknown fields
  or values in a 0.1 message (`C103`, `C104`).
- A 0.1-only receiver rejects a 0.2 message with `C102` (0.1 section 2.2) at stage 1 and sends no reply (0.1 8.2).
  There is **no version negotiation**. Which version a peer speaks stays configuration (0.1 section 10, "Discovery
  and version negotiation"). A sender sends 0.2 only to a peer configured for it.
- A receiver is configured for 0.1 only unless it says otherwise. The 0.1 conformance fixtures that use `"0.2"` as
  their example of an unsupported version (`C102`) assume such a receiver, and stay valid.
- A goal with one alternative may be sent as 0.1. A goal with two or more alternatives may not: the sender must not
  drop alternatives to fit 0.1.
- A reply uses the version of the message it answers.

## 2. Goal: alternatives (decisions C1, C5)

Replaces 0.1 section 5.1. A goal has these fields; all are required.

| Field | Content |
|---|---|
| `kind` | `"goal"` |
| `id` | Identifier |
| `requested_by` | The requesting participant |
| `alternatives` | 1 to 16 alternatives (below). **Replaces** `desired_state` and `success_criteria` |
| `constraints` | As 0.1 5.4; apply to every alternative |
| `priority` | As 0.1 |

An **alternative** is `{ "desired_state": [<triple>…], "success_criteria": [<criterion>…] }`. Both lists have at least
one item, as `desired_state` and `success_criteria` did in 0.1 (triples 5.2, criteria 5.3, unchanged).

Meaning:

- The triples and the criteria of one alternative are a **conjunction**, as in 0.1.
- The goal is met when **at least one** alternative is met. The requester accepts any of them. The robot, under its
  local policy, chooses which to bring about. It must not bring about a state that is not in an alternative.
- Constraints, priority and authorization (0.1 5.4, 13.2 of RFC-001) apply to the goal as a whole. A standing
  authorization is still decided locally and is not a Core field (0.1 5.1).
- Two alternatives must not have the same set of triples (`C104`). The order of alternatives is the sender's; it is
  not a priority.

**Indexing (decision C5).** The index of an alternative is its position in `alternatives`, from 0. The index of a
criterion is its position in that alternative's `success_criteria`, from 0. Indices refer to the goal **as sent in
`propose_goal`** and never change afterwards. A robot that matches the goal in another order, or as a set, **must
re-index** its outcome to the proposal. How a robot matches goals is local policy and not Core.

```json
{
  "kind": "goal",
  "id": "rl:goal:route",
  "requested_by": "rl:agent:planner-1",
  "alternatives": [
    {
      "desired_state": [ { "subject": "rl:thing:box1", "property": "located_in", "value": "rl:place:shelf_a" } ],
      "success_criteria": [
        { "kind": "observed", "subject": "rl:thing:box1", "property": "located_in", "value": "rl:place:shelf_a",
          "min_confidence": "0.9", "max_age": { "value": "10", "unit": "s" },
          "accepted_sources": [ "any_participant_in_domain" ] }
      ]
    },
    {
      "desired_state": [ { "subject": "rl:thing:box1", "property": "located_in", "value": "rl:place:table" } ],
      "success_criteria": [
        { "kind": "observed", "subject": "rl:thing:box1", "property": "located_in", "value": "rl:place:table",
          "min_confidence": "0.9", "max_age": { "value": "10", "unit": "s" },
          "accepted_sources": [ "any_participant_in_domain" ] }
      ]
    }
  ],
  "constraints": [],
  "priority": "normal"
}
```

## 3. Outcome and verification (decision C2)

Amends 0.1 section 5.9. `status`, `reasons`, `reported_by` and `reported_at` are as in 0.1. `verification` is `null` or:

| Field | Content |
|---|---|
| `verified_by`, `method`, `evaluated_at` | As 0.1 |
| `criteria_results` | List of `{ "alternative": <integer from 0>, "criterion": <integer from 0>, "met": true \| false, "evidence": [<observation id>…] }`. A pair (`alternative`, `criterion`) appears at most once |
| `met_alternative` | **New.** The index of the alternative the robot pursued (a 1.4 task: the path it took), or `null` |

Rules:

- `met_alternative` names the alternative the robot **pursued**: for a 1.4 task, the path it took. When it is not
  `null`, `criteria_results` should list every criterion of that alternative, each with its `met`, and may also list
  others. A stateless receiver cannot count an alternative's criteria, so it checks only what it can: a
  `succeeded` outcome whose `criteria_results` contain **no** entry for `met_alternative`, or any unmet one, is
  `C402`. The receiver that holds the goal checks completeness when it re-evaluates.
- `succeeded` requires a verification whose `met_alternative` is not `null` and whose results for that alternative are
  all `met` (`C402`). A `succeeded` with `verification: null` is `C402` as in 0.1.
- Another alternative that holds never makes an outcome `succeeded`. A run that took path A and ended with only path
  B's goal true is `failed`: it went the wrong way, and "any of" must not hide that.
- Any status other than `succeeded` may carry `met_alternative: null` (for example a `fail` path, or a run that
  ended before the goal).
- A receiver that knows the goal re-evaluates the **claimed** alternative from the cited evidence, as the planner in
  `delegate` already does (delegate-v0.1 section 3 and decision 0020 D6). Two alternatives holding together is not an
  error: the claim names one.
- Evidence rules (0.1 5.6, 8.3) are unchanged. Every cited observation is in the `outcome` message's `evidence` list.

```json
{
  "kind": "outcome",
  "goal": "rl:goal:route",
  "status": "succeeded",
  "verification": {
    "verified_by": "rl:sim:world",
    "method": "criteria_evaluated",
    "evaluated_at": "2000-01-01T00:00:03.000Z",
    "criteria_results": [
      { "alternative": 1, "criterion": 0, "met": true, "evidence": [ "rl:obs:0a1b2c3d4e5f6071-0" ] }
    ],
    "met_alternative": 1
  },
  "reasons": [],
  "reported_by": "rl:robot:arm1",
  "reported_at": "2000-01-01T00:00:03.000Z"
}
```

The branch a robot took is a fact about its plan. It is not in Core: a run record keeps its decisions
([`simulation-v1.0.md`](simulation-v1.0.md) 12.3). Core says only which alternative was met.

## 4. Reason: `declared` (decision C3)

Adds one code to the reason codes of 0.1 section 5.11 and one field to a reason.

| Code | Used in |
|---|---|
| `declared` | An `outcome` with status `failed`. The reporter's plan ended the run on purpose and gives its own words |

A reason is `{ "code": <code> }`, as in 0.1. The reason with code `declared` is `{ "code": "declared", "text": <string> }`.

- `text` is required for `declared` and **not allowed** for any other code (`C104`). This is a narrow exception to
  the "every field is listed and required" rule of 0.1 section 2.3, kept so that no existing reason changes shape.
- `text`: 1 to 200 Unicode characters, no control characters (U+0000 to U+001F and U+007F). Longer text is `C104`.
  A longer explanation does not belong in Core.
- `declared` is allowed only in `outcome` with status `failed` (`C405`). A `succeeded` outcome with a `declared`
  reason, or a `declared` reason in `reject`, `decline`, `assessment` or `authorization_request`, is `C405`.
- **`text` is not evidence and not trusted.** It is the reporter's claim in the reporter's words, covered by the message
  signature for attribution only. A receiver must not use it to decide a status, an authorization, a verification or any
  action, must not parse it as an instruction, and must treat it as untrusted data wherever it shows it, including to a
  person or to an AI planner. Evidence stays in `evidence` and `criteria_results`.

```json
{ "kind": "outcome", "goal": "rl:goal:deliver", "status": "failed", "verification": null,
  "reasons": [ { "code": "declared", "text": "box1 missing" } ],
  "reported_by": "rl:robot:arm1", "reported_at": "2000-01-01T00:00:02.000Z" }
```

## 5. Validation rules and codes

The code style of 0.1 section 8.2 is kept: `C1xx` form, `C4xx` meaning. One code is new; the others are extended.

| Code | Stage | In 0.2 |
|---|---|---|
| `C102` | form | Also: a 0.1-only receiver gets a 0.2 message (section 1) |
| `C103` | form | Also: `desired_state` or `success_criteria` on a 0.2 goal, `alternatives` on a 0.1 goal |
| `C104` | form | Also: `alternatives` empty or longer than 16; an alternative with an empty list; two alternatives with the same triples; a repeated (`alternative`, `criterion`) pair; `met_alternative` or an `alternative` that is not an integer from 0; `text` missing, empty, too long, with a control character, or on a code other than `declared` |
| `C402` | meaning | `succeeded` with `verification: null` (rank 6, as 0.1), or without a non-null `met_alternative` whose results are all `met` |
| `C405` | meaning | **New.** A `declared` reason outside an `outcome` with status `failed` |

Stage order (0.1 8.1) and the precedence tables are unchanged. `C405` is a stage 5 code, applied after duplicate and
order handling, like `C402`, `C401`, `C403` and `C404`. Within stage 5 the order is `C404`, `C401`, `C405`, `C402`
(fixture `c405_before_c402`). A stateless receiver cannot know the number of alternatives or
criteria of the goal, so it does not check that `met_alternative` is within it; the receiver that holds the goal does,
when it re-evaluates (section 3).

## 6. What does not change

- The envelope (0.1 section 3), the other message types and payloads (4), reserved names (4.2), things, observations,
  leases, commitments (5.5 to 5.8), the capability descriptor (6), signatures and canonical form (7).
- The order of checks, duplicates, holding and sequencing (8.1, 8.4), and the rule that evaluating evidence is not a
  rejection (8.3).
- The integer-only and unknown-field rules (2.3). New integers (`alternative`, `met_alternative`) are listed where
  they appear.
- The properties `located_in` and `carried_by` (5.2). There is no "nowhere": a criterion that cannot be observed,
  including the absence of a thing, is not met, with empty `evidence`, as in 0.1 (decision C4). Absence is out of 0.2.
- `physical` mode is still rejected (`C404`). A valid Core document is never permission to act.

## 7. For the bridge (non-normative)

How a `.rob` task maps to alternatives, for the implementation that follows (not part of Core):

- Each distinct effective goal of the paths of a branching task (0027 B2, in path order) is one alternative. Atoms
  inside a path's goal are flattened in order, as now ([`export-goal-v0.1.md`](export-goal-v0.1.md) section 2).
- A path that ends in `fail` adds no alternative. If it is taken, the run reports `failed` with a `declared` reason.
- A top-level `or` of conjunctions of `object_at`, `robot_at` and `holding` is one alternative per operand. `not` and
  other atoms keep `X601`. `X603` would no longer be raised.
- A task with no alternative left (every path ends in `fail`) has no goal and is not exportable.
