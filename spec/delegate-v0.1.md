# `robolang delegate` v0.1 — a goal crosses the wire, a robot runs its own plan, a verified outcome comes back

**Status:** implemented in `src/robolang/delegate.py`; decision [0020](decisions/0020-delegate.md).
A task that binds a second robot also has a peer, with commitment, lease and custody:
[delegate v0.2](delegate-v0.2.md), decision [0022](decisions/0022-delegate-custody-and-leases.md). This document
describes the delegation of a task with one robot, whose records stay version 0.1.
**Plan:** [`docs/plans/goals-and-core-bridge.md`](../docs/plans/goals-and-core-bridge.md), phases 4 and 5.
**Builds on:** [export-goal](export-goal-v0.1.md), [simulation](simulation-v1.0.md) sections 3 and 10,
[Core 0.1](core-v0.1.md) sections 3, 4, 5.9, 5.10 and 8, decisions R3 (plans never travel) and
[0008](decisions/0008-core-sequencing-per-recipient.md) (sequence per sender, recipient, conversation).

> **Simulation only.** One process, no threads, no network, no ROS 2, no hardware. The signatures
> are `sim-hmac-sha256` under keys anyone can compute, and the verifier is the simulator. A verified
> `succeeded` here shows that the exchange and the checks work, not that a robot did anything.

## 1. Command

```text
robolang delegate FILE --world WORLD [--task NAME] [--bind NAME=ID ...]
                  --from AGENT_ID --to ROBOT_ID [--plans PATH ...] [--robot-world WORLD2] [--lease DURATION]
```

| Option | Meaning |
|---|---|
| `FILE`, `--world`, `--task`, `--bind` | The AI's task. Its goal is exported exactly as `export-goal` does (same message, same codes `X601`, `X602`) |
| `--from`, `--to` | Planner and robot, Core identifiers without `#`. Both are required |
| `--plans` | The robot's plan library: `.rob` files, or directories whose `*.rob` files are read in name order. Default: `FILE` |
| `--robot-world` | The robot's world. Its `tasks.<name>.bindings` bind the library tasks (the robot binds its own plans to its own world). Default: `--world` |
| `--lease` | The lease a peer robot offers (default 60 s). Only a task that binds a second robot uses it: [v0.2](delegate-v0.2.md) |

Output: one line on stderr, `simulation only: delegate <goal> → <robot>: <status> (<note>)` where the
note is `verified`, `not verified`, or the reason codes of a rejection; then the delegate record
(section 4) as JSON on stdout (two-space indent, ASCII, trailing newline). Identical inputs give
identical bytes.

## 2. The exchange

Every message is a full Core 0.1 document, signed with the sender's `<participant>#k1` simulation key
(`export_goal.simulation_key`), and every delivery is checked by the receiver with
`robolang.core.check_message` (all stages, including meaning, so `C402`) under a `Context` that trusts
both participants' keys and the simulator's (`rl:sim:world#sim-1`, which signs evidence). A rejected
delivery stops the exchange. Sequence numbers count per sender, recipient and conversation (0008); a
receiver also rejects a sequence that is not the next one (`C306`). Time is `bridge.core_time`; every
message takes 50 ms to arrive and a participant acts 50 ms after its previous action.

| t (ms) | Step |
|---|---|
| 0 | planner → robot `propose_goal` (from `export_goal`), sequence 1 |
| 50 | the robot matches the goal (section 3) |
| 50 | no match: robot → planner `reject`, reason `capability_unsupported`; the planner records `rejected` |
| 50, 100 | match: robot → planner `assessment` (`feasible`), then `accept` |
| 150 | authorization (section 5); the robot runs the matched task with the symbolic executor in its world |
| end of run | robot → planner `outcome` (`outcome` and `evidence`); the planner verifies it (section 6) |

The outcome payload is the run's `core_outcome` (simulation 10.5) adapted by parameters of
`sim.outcome.core_outcome`: `goal` is the **proposed goal's ID** (not `rl:goal:<local task name>`),
`reported_by` is the **robot**, times count from the start of the run at 150 ms, and `verified_by`
stays `rl:sim:world` with method `criteria_evaluated`. The robot's run record keeps the run's own
view (`rl:goal:<task name>`, reporter `rl:sim:world`, times from 0).

## 3. How the robot matches a goal

The robot reads its library in order (files in the order given, tasks in file order) and takes the
first task for which all of these hold:

1. the task has a goal (RoboLang 1.3);
2. the robot world has bindings for it (`tasks.<name>.bindings`; a missing or invalid entry makes the
   task a non-candidate, it is not an error);
3. `bridge.goal_triples(task, bindings)` holds **the same triples as the proposed `desired_state`, in
   any order** (no duplicates, nothing more, nothing less). The robot re-indexes its outcome to the
   proposal: each criterion result names the index of the proposed criterion, results and evidence are
   listed in that order, and each cited observation states the proposed criterion's subject, so the
   planner checks every criterion against its own goal (amended 2026-10-29, decision record 0020 D2);
4. `--to`'s entity (`rl:robot:arm1` → `arm1`) is bound to one of the task's robot parameters.

The record lists every task considered with one result: `matched`, `no_goal`, `world_E5xx`,
`no_core_form`, `goal_differs`, `robot_not_bound` or `robots_unsupported` (v0.2: the task binds two or more other
robots). Nothing else is compared: safety limits and
preconditions are the plan's own business, and a failing precondition is an outcome, not a rejection.

## 4. The delegate record

```json
{
  "delegate_record_schema": "robolang.delegate",
  "version": "0.1",
  "mode": "simulation",
  "goal": "rl:goal:handover",
  "planner": "rl:agent:planner-1",
  "robot": "rl:robot:arm1",
  "messages": [ { "t_ms": 50, "document": { "...a Core message...": "" }, "receipt": { "status": "accepted", "code": null } } ],
  "events": [ { "t_ms": 50, "actor": "rl:robot:arm1", "event": "plan_matched", "file": "...", "task": "..." } ],
  "robot_run": { "...the run record (run record 1.2) of the robot's run, or null...": "" },
  "planner_verdict": { "status": "succeeded", "criteria": [ { "criterion": 0, "met": true, "evidence": ["..."], "reason": null } ], "reasons": [] }
}
```

- `messages`: in delivery order. `t_ms` is the delivery time; `document` is what the receiver got
  (after any change in flight); `receipt` is its `check_message` receipt.
- `events` (stable order by `t_ms`): `plans_considered`, `plan_matched` or `no_plan_matched`,
  `authorization`, `run_started`, `run_ended`. Plan paths are shown as given on the command line.
- `robot_run`: null when the robot rejected the goal or the exchange stopped before the run.

## 5. Authorization

The robot holds a **standing authorization in `simulation` mode** (Core 5.10): local configuration, no
message. It covers any goal the robot matched to a local plan, and is recorded as the `authorization`
event (`basis` `standing_authorization`, `mode` `simulation`). A mode other than `simulation` is never
accepted: the API raises `DelegateError` `D701` before anything is sent, and Core 0.1 receivers reject
a physical `mode` in a payload (`C404`). The command has no way to ask for it.

## 6. The planner's verdict

The planner accepts the outcome only after `check_message` accepts it. It then verifies each evidence
observation's signature against the keys it trusts, and re-evaluates each of its own success
criteria against the observations the outcome cites for that criterion: the observed subject, property
and value equal the criterion's; `confidence` ≥ `min_confidence`; and the age at the verification's
`evaluated_at` is within `max_age` and the observation's own `valid_for` (an observation after
`evaluated_at` is stale). `accepted_sources` is satisfied by any participant whose key the planner
holds. Per-criterion `reason`: `evidence_missing`, `signature_invalid`, `criterion_not_met`,
`confidence_too_low` or `evidence_stale`.

`status`:

- `succeeded` only if the robot claimed `succeeded`, the goal ID and reporter are right, every
  criterion is met by the planner's own evaluation, and the robot's per-criterion `met` agrees;
- if the robot claimed `succeeded` and any of that fails: `unknown`, with `reasons` `goal_mismatch`,
  `reporter_mismatch` or `claim_not_supported` (per criterion);
- otherwise the robot's own `failed`, `unknown` or `cancelled`;
- `rejected` when the robot rejected the goal;
- `unknown` with the Core code, stage and pointer when the planner (or the robot) rejected a message.

## 7. Exit status

| Code | Meaning |
|---|---|
| `0` | The planner recorded a verified `succeeded` |
| `1` | FILE or a plan file is not valid RoboLang |
| `2` | Usage: bad arguments, `--bind` or `--lease`, unreadable file, unknown task, several tasks and no `--task`, invalid `--from` or `--to`, no `.rob` file under `--plans` |
| `3` | A world file is not valid, or a `--bind` is wrong |
| `4` | The planner recorded `failed`, `unknown` or `cancelled` |
| `6` | The goal has no Core form (`X601`) or an object lacks class or mass (`X602`) |
| `7` | The robot rejected the goal (no plan matches, or its peer declined), or a participant rejected a message (record on stdout) |

## 8. What is not shown

- **No transport.** Delivery is a function call with a fixed latency: no loss, reordering,
  duplication or timing faults. The sequence checks are applied but never triggered.
- **Simulation-only signatures.** Anyone can compute every key, so a valid signature shows that a
  check happened, never who signed.
- **Ground-truth verification.** The evidence is the simulator's view of the robot's own world (plan
  decision Q1, `rl:sim:world`), not an independent sensor. The robot's world is also the world it
  runs in, so the evidence cannot disagree with the run.
- **The plan's safety limits are not matched to the goal's constraints**, and nothing is enforced.
- **No descriptors or authority messages.** Those are in `arch02`. Assistance, commitment, lease and custody
  are in [v0.2](delegate-v0.2.md), for a task that binds a second robot; a task with one robot has none of them.
- Ordered goal matching only; goals with `or` or `not` have no Core form and are not delegated.
