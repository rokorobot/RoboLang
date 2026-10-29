# `robolang delegate` v0.2 — a peer robot: commitment, lease and custody transfer

**Status:** implemented in `src/robolang/delegate.py` and `delegate_peer.py`; decision
[0022](decisions/0022-delegate-custody-and-leases.md).
**Builds on:** [delegate v0.1](delegate-v0.1.md) (read it first; this document only adds to it),
[Core 0.1](core-v0.1.md) sections 4.1 (r2r messages), 5.7 (lease), 5.8 (commitment) and 8, RFC-001 10.4
(custody rules), [reference simulation](reference-simulation-v0.1.md) (the order of arch02's messages, which this mirrors).

> **Simulation only.** Same limits as v0.1. The peer is a third participant of the same process with its
> own published simulation key; it senses nothing, and its observation is the simulator's ground truth
> signed with its key. `arch02`, its fixtures, the gate 1 evidence and the frozen gate 2 bundle are untouched.

## 1. When a peer takes part

After the robot has matched a plan (v0.1 section 3), it looks at the robots the task binds in the robot
world. If the task binds exactly one robot other than `--to`, that robot is the **peer**
(`rl:robot:<id>`, for example `arm1` served by `amr1` in `examples/handover.rob`). A task that binds
only `--to` has no peer and runs exactly as v0.1: same messages, same bytes, record version `0.1`.
A task that binds **two or more** other robots is not served: its `plans_considered` result is
`robots_unsupported` (a task that is not a candidate, like `robot_not_bound`).

## 2. The exchange

Times in ms; every message takes 50 ms to arrive and a participant acts 50 ms after its previous action
(as in v0.1). `R` is the robot, `P` the peer, `N` the planner.

| t | From → to | Message | Content |
|---|---|---|---|
| 0 | N → R | `propose_goal` | as v0.1 |
| 50 | R → N | `assessment` | `feasible_with_assistance`, reason `assistance_required` |
| 100 | R → P | `request_assistance` | `goal` = the sub-goal (the goal with ID `<goal>-sub-1`, `requested_by` R), `things`, `parent_goal` |
| 150 | P → R | `offer_assistance` | `goal` = the sub-goal, `conditions` `[]`, `lease` (section 3) |
| 200 | R → P | `commit` | the commitment: debtor P, creditor R, obligation the sub-goal, the offered conditions and lease, `state` `active` |
| 250 | R → N | `accept` | standing authorization (simulation) at both robots: P at 250, R at 300 (`authorization` events, as v0.1 section 5) |
| 300 | | | R runs its plan (section 4) |
| `T` | R → P | `transfer_custody` | `thing`, `commitment`, `criteria` = the thing `located_in` the handoff place, from P, at least 0.9 confident, at most 10 s old |
| `T`+50 | P → R | `custody_confirmed` | `thing`, `observations`: P's own signed observation of the thing |
| `T`+100 | R → P | `custody_released` | `thing`, `confirmation` = the `custody_confirmed` message ID |
| `T`+150 | P → R | `acknowledge` | `accepted: true`, `acknowledged` = the release |
| end | R → N | `outcome` | as v0.1; the planner verifies it as v0.1 section 6 |

`in_reply_to`: `offer_assistance` and `decline` reply to the request, `custody_released` to the
confirmation, `acknowledge` to the release; `commit`, `transfer_custody` and `custody_confirmed` start a
step and reply to nothing, as in arch02. Sequences count per sender, recipient and conversation (0008);
every message is checked by its receiver with `check_message` and the order check (`C306`).
`custody_released` is also checked by the peer for `C403`: it must name the `custody_confirmed` the peer
sent for the same thing. P accepts the release durably (`custody_committed` event) before it acknowledges (rule 3).

**Trust.** Each receiver verifies with the keys of its counterparts only (0013): P holds R's key; R holds
N's and P's; N holds R's and the simulator's. A message to P signed by N is `C203`.

## 3. The lease

The lease is the one in `offer_assistance` and in the commitment (Core 5.7): `granted_at` is when P
offers (150 ms), `expires_at` is `granted_at` + the lease duration, `renewal` 0. The default is **60 s**;
`--lease DURATION` sets it (`60s`, `500ms`, `2min`, `0.5s`: a number and `ms`, `s` or `min`, whole
milliseconds, from 1 ms to 24 h; otherwise exit 2). `--lease` has no effect without a peer.

**Lease rule.** The lease must hold for every act of the peer, judged in the safe direction (the clock
tolerance is 500 ms):

- at the custody point `T`, the robot starts the exchange only if it ends inside the lease, that is
  `T` + 200 ms < `expires_at` − 500 ms;
- every plan step of the peer must start before `expires_at` − 500 ms (a step's start includes the
  time the exchange took, if it comes after the custody point).

Otherwise the commitment is `lapsed` and the run is cancelled before the first step that fails the
rule (section 5). Amended 2026-10-29: before, only the exchange was checked, and the peer's later
steps were not timed against the lease.

## 4. The custody point and how the plan runs

The **custody point** is the first plan step where the peer grasps an object that the robot placed earlier
in the plan (`bridge`-bound: robot and peer are the world IDs the robot world binds). The **handoff place**
is where the robot placed it. A plan without such a step has a commitment but no custody exchange: the
commitment is `fulfilled` when the run succeeds, `failed` otherwise.

`robolang.sim` is not changed. The robot runs the plan in two segments:

1. **Probe.** The task runs once with the world script set to `cancel_before_step` = the custody step
   (`dataclasses.replace` on the loaded world, not written to any file). If the run fails or is cancelled
   earlier (a precondition, a fault, an earlier scripted cancel), that run is the final run and there is no exchange.
2. **Exchange** at `T` = the run's start + the probe's `end_t_ms`, with the world as the probe ended it
   (the peer's observation is the thing's place in that final world).
3. **Final run.** If the exchange succeeds, the whole task runs again in the unchanged robot world; that
   normal run record (`robot_run`) is the one reported. Steps after the custody point are shifted by the
   exchange's 200 ms in the exchange's own timeline (times in the outcome and `run_ended`), not in the
   run record. If the exchange fails, the probe is the final run: a normal run record in state
   `CANCELLED` with `cancelled_before_step` = the custody step.

## 5. Failures

| Case | What happens | Reasons | Outcome / verdict | Commitment | Exit |
|---|---|---|---|---|---|
| Peer not ready in the robot world (`robots.<peer>.ready` not `true`) | P → R `decline` (`participant_busy`); R → N `assessment` `infeasible` and `reject` | `assistance_unavailable` | no outcome; planner `rejected` | none (`commitment` null) | 7 |
| Lease ends before a later step of the peer (after a successful custody exchange) | run cancelled before that step; R reports `failed` at the lease end | `lease_expired` | `failed`; planner `failed` | `lapsed` | 4 |
| Lease ends before the custody exchange can finish | no `transfer_custody`; run cancelled before the peer's step; R reports `failed` at the later of `T` and the lease end | `lease_expired` | `failed`, verification null; planner `failed` | `lapsed` (`lease_expired` event at the lease end) | 4 |
| The confirmation does not meet the criteria (thing observed elsewhere, too old, not confident, not P's own, missing) | no `custody_released`; R's custody returns to `HOLDING`; run cancelled before the peer's step; `confirmation_not_accepted` event with the criterion reason | `custody_unknown` | `failed`; planner `failed` | `failed` | 4 |
| The run fails or is cancelled before the custody point, or fails after it | as a v0.1 failed or cancelled run | as v0.1 | as v0.1 | `failed` | 4 |
| A message fails `check_message` or the order check at its receiver, including a `custody_released` that names no confirmation (`C403`) | the exchange stops; the receipt and the Core code are recorded | `C202`, `C203`, `C403`, … | planner `unknown`, reasons with code, pointer, message and receiver | as last set | 7 |
| The task binds two or more other robots | the plan is not a candidate (`robots_unsupported`) | `capability_unsupported` if nothing else matches | `rejected` | none | 7 |

Both the lease and the confirmation failure are reported as `failed`, not `cancelled`, as arch02 does when
the thing never left the giver; the cancelled run record is the mechanism, not the verdict.

## 6. The record (version 0.2)

A record of a delegation with a peer has `"version": "0.2"` and two more top-level fields; a record
without a peer is unchanged (`"0.1"`, neither field).

```json
{
  "delegate_record_schema": "robolang.delegate",
  "version": "0.2",
  "robot": "rl:robot:arm1",
  "peer": "rl:robot:amr1",
  "messages": [ "... as v0.1, including the r2r messages between the robots ..." ],
  "events": [ "..." ],
  "robot_run": { "...": "a normal run record" },
  "commitment": { "kind": "commitment", "...": "the commit message's commitment, with the final local state: fulfilled, lapsed or failed" },
  "planner_verdict": { "...": "as v0.1" }
}
```

New `events`: `peer_selected` (`peer`); `commitment` (`commitment`, `state`: `active`, then `fulfilled`,
`failed` or `lapsed`); `lease_expired`; `custody_point` (`step`, `thing`, `place`); `custody` (`thing`,
`state`; actor R: `OFFERED`, `RELEASE_SENT`, `RELEASED`, or back to `HOLDING` when nothing is released; actor
P: `CONFIRMATION_SENT`, `HOLDING`); `custody_committed` (`thing`, `release`); `confirmation_not_accepted`
(`thing`, `reason`); `no_confirmation_evidence`. Custody state names are arch02's. The `authorization`
event is recorded for both robots (P's names the sub-goal).

## 7. What is not shown

Everything in v0.1 section 8, and: **no reservation** of the handoff place (no arbiter; `conditions` is
empty), **no `coordinate` messages**, **no peer `outcome`** (the peer's steps are part of R's run and R
reports for both), **no physical placement or sensing** (the peer sees ground truth; R does not cross-check
it with a sensor of its own), **no `custody_query` recovery** (an unacknowledged release cannot happen
without a transport), **no lease checks after the custody exchange**, and **one peer at most**.
