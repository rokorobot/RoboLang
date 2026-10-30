# `robolang delegate --core 0.2`: alternatives and re-indexing

**Status:** implemented in `src/robolang/delegate.py` (PR B of N5); decision [0028](decisions/0028-core-0-2-scope.md) C5, C8.
**Builds on:** [delegate v0.1](delegate-v0.1.md) and [v0.2](delegate-v0.2.md) (read them first; this adds to them),
[Core 0.2 draft](core-v0.2.md), [export-goal 6](export-goal-v0.1.md).

**Why a separate file.** The delegate *record* has its own version (`0.1`, or `0.2` when a peer took part, v0.2 above). Core
0.2 is a different axis: the version of the messages. Calling this "delegate v0.3" would say the record changed
version, and it did not. Without `--core 0.2` nothing in delegate changes.

## 1. Behaviour

`robolang delegate … --core 0.2` (function argument `core="0.2"`):

- The proposal is the `export-goal --core 0.2` message. Every later message of the exchange (assessment, accept,
  outcome, and the peer's) has `schema_version` `0.2` (a reply uses the version of the message it answers). Receivers
  are configured for that version only. A goal that needs two or more alternatives with the default `--core 0.1` is
  not delegated: exit 6, saying to use `--core 0.2`.
- **Matching (the robot).** A library task matches when its alternatives, exported as in export-goal 6, equal the
  proposal's **as sets of alternatives, each alternative a set of triples** (same count, none repeated, no triple
  repeated inside an alternative). Order of alternatives and of criteria is free. Otherwise `goal_differs`, and the
  robot rejects with `capability_unsupported` as in 0.1. The other checks (robot bound, `robots_unsupported`) are
  unchanged: **a branching plan that binds two or more robots stays `robots_unsupported`** (lifting it is not trivial,
  because the custody point is found on a straight plan only).
- **Re-indexing (C5).** The robot's outcome indexes alternatives and criteria as the **proposal** did:
  `alternative` is the proposal's position of the matched local alternative, `criterion` the proposal's position of the
  triple inside it.
- **Outcome.** As [simulation 12.4](simulation-v1.0.md): `met_alternative` is the alternative pursued (the path
  taken); a run ended by `fail` is `failed` with a `declared` reason and no verification.
- **Verification (the planner)** (`verify_outcome`, draft section 3). The planner re-evaluates the *claimed*
  alternative from the cited evidence, as in v0.1 (signature, triple, confidence, age). The verdict is `succeeded` only if
  the outcome claims `succeeded`, `met_alternative` is a valid alternative, **every** criterion of it has a result
  (completeness) and each re-evaluates as met, and nothing else is wrong. Results for other alternatives are
  evaluated and listed too, but an unmet one there does not matter. A claim the evidence does not support, an
  alternative or criterion the goal does not have, or a missing result for the claimed alternative is
  `claim_not_supported` (with `alternative` and `criterion`); the status is then `unknown`. Each entry of
  `planner_verdict.criteria` has an `alternative`, and the verdict has `met_alternative` (the verified one, or `null`).
  The `declared` text is never read by the planner (draft section 4); nothing feeds it to a model.

## 2. What does not change

Record shape and versions, event names, exit codes (a failed run is exit 4, a rejection exit 7), the peer
protocol, custody, leases, and every Core 0.1 exchange.
