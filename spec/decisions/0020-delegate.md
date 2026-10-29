# Decision record 0020 — `delegate`: a goal out, a local plan run, a verified outcome back

**Date:** 2026-10-29
**Status:** **IMPLEMENTED, owner approval pending.** Phases 4 and 5 of
[`docs/plans/goals-and-core-bridge.md`](../../docs/plans/goals-and-core-bridge.md); not merged.
**Applies to:** [`delegate-v0.1.md`](../delegate-v0.1.md), [export-goal](../export-goal-v0.1.md),
[simulation](../simulation-v1.0.md) section 10
**Builds on:** plan decisions Q1 and Q4 (approved 2026-10-29); [0008](0008-core-sequencing-per-recipient.md),
[0018](0018-core-shaped-outcomes.md), [0019](0019-export-goal.md)

## 1. Why

The seam of the plan is: the goal crosses, the plan stays, the outcome returns as evidence.
`delegate` runs that in one process so the whole path can be tested and shown.

## 2. Decisions

| # | Topic | Rule | Alternative considered |
|---|---|---|---|
| D1 | A new mode (Q4) | A new command and module. `arch02`, its fixtures, the gate 1 evidence and the frozen gate 2 bundle are untouched | Replace the scripted motion in the arch02 stubs: invalidates the frozen bundle |
| D2 | Matching | The first library task whose goal triples equal the proposed `desired_state` as an **ordered list**, under the robot world's bindings, and whose robot parameter is bound to `--to` | A set match: criterion *i* of the outcome is goal condition *i*, so a reordered plan would need re-indexing |
| D3 | Rejection | No match, or `--to` not a robot of the matching task: `reject` with `capability_unsupported`; the planner records `rejected` (exit 7) | `capability_insufficient`: the robot has no such capability, it is not out of limits |
| D4 | Authorization | A standing authorization in `simulation` mode, local, recorded as an event. Another mode raises `D701` before anything is sent | An `authorization_request` round trip: it needs an authority participant, which `arch02` already shows |
| D5 | The outcome | The run's `core_outcome` with parameters on `sim.outcome.core_outcome`: the proposed goal's ID, the robot as `reported_by`, times from the run's start. `verified_by` stays `rl:sim:world` (Q1) | Re-building the outcome in `delegate`: two builders to keep equal |
| D6 | The planner | After `check_message` it verifies each evidence signature and re-evaluates each criterion itself. `succeeded` needs the robot's claim and its own agreement; a claim it cannot support is `unknown`, with reasons | Trust `check_message` alone: C402 only checks the robot's own `met` flags |
| D7 | Checks | Every delivery goes through `check_message` with a `Context` trusting both participants' keys and the simulator's; a rejection stops the exchange and is recorded. No reply is sent to a rejected message | Skip the checks in process: the exchange would test nothing |
| D8 | Time and order | Fixed 50 ms latency, deterministic Core times, sequence per sender, recipient and conversation (0008) with a `C306` check on receipt | Wall clock: not reproducible |
| D9 | Exit 7 | New: the robot rejected the goal, or the planner rejected a message. 4 stays "the outcome is not a verified success", also when the planner's re-evaluation disagrees | Reuse 4 for rejection: a caller could not tell a refusal from a failed run |
| D10 | Library | `--plans` files or directories (`*.rob`, name order), default FILE; the robot world's `tasks.<name>.bindings` bind them; a task without bindings is skipped, not an error | One bindings file per plan: more files for no gain |

## 3. Consequences

- `examples/plans/` holds two robot plans (`relocate_box`, `park_robot`) and `examples/world.json`
  gains their bindings, so run IDs of the examples change (the world's bytes changed).
- `core_outcome` has three keyword parameters; defaults give exactly the previous output.
- The in-flight `intercept` argument of `delegate()` is a test seam for tampering, not a CLI option.
- Not shown: a real transport, real signatures, an independent verifier (spec section 8).

## Amendment — 2026-10-29

D2 now matches the goal as a **set**: the same triples in any order, no duplicates. The robot maps each
of its goal conditions to the index of the proposed criterion (`core_outcome(criterion_of=...)`), so
criterion *i* of the outcome is criterion *i* of the proposal, and the planner's check is unchanged.
