# Decision record 0022 — `delegate` with a peer robot: commitment, lease and custody

**Date:** 2026-10-29
**Status:** **IMPLEMENTED, owner approval pending.** Not merged.
**Applies to:** [`delegate-v0.2.md`](../delegate-v0.2.md), [`delegate-v0.1.md`](../delegate-v0.1.md)
**Builds on:** [0020](0020-delegate.md), [0008](0008-core-sequencing-per-recipient.md),
[0013](0013-h3-receiver-scoped-verification-keys.md); Core 0.1 4.1, 5.7, 5.8, RFC-001 10.4; the message order of `arch02`

## 1. Why

The robot's plan may bind a second robot (`examples/handover.rob`), but in v0.1 that robot never took part: no
commitment, no lease, no custody. `delegate` now runs the Core r2r exchange `arch02` shows, for any matched plan.
`arch02`, its fixtures, the gate 1 evidence and the frozen gate 2 bundle are untouched.

## 2. Decisions

| # | Topic | Rule | Alternative considered |
|---|---|---|---|
| P1 | Who is the peer | The one other robot the matched task binds in the robot world, a third participant with its own simulation key. Two or more others: not a candidate (`robots_unsupported`) | A peer chosen from the world or by flag: the plan already says who helps |
| P2 | Message order | Mirrors `arch02`: `assessment` `feasible_with_assistance` → `request_assistance` → `offer_assistance` (or `decline`) → `commit` → `accept` → custody → `outcome`. The peer declines when its robot is not `ready` in the robot world (`participant_busy`); the robot then withdraws with `assessment` `infeasible` and `reject` `assistance_unavailable` | A richer peer policy: nothing in the world describes one |
| P3 | Lease | The peer offers it: 60 s by default, `--lease DURATION` (1 ms to 24 h). Checked once, at the custody point: the exchange must end before `expires_at` − the 500 ms tolerance (`arch02`'s safe direction). Expired: commitment `lapsed`, outcome `failed` with `lease_expired` (as `arch02` for a thing that never left the giver) | `cancelled`: the cancelled run is the mechanism, not the verdict. Checking every peer step: needs timed steps per robot |
| P4 | Custody point | Found by reading the plan: the first step where the peer grasps an object the robot placed; the handoff place is where it was placed. The exchange comes just before that step | A new plan statement: a language change (1.3 is released) |
| P5 | Keeping `robolang.sim` pure | Two runs: a probe with `cancel_before_step` = the custody step (`dataclasses.replace` on the loaded world), the exchange on the probe's final world, then the whole task again as the final run. A failed exchange keeps the probe as the final run. Both are normal run records | One run in two segments: the executor would need to pause and resume |
| P6 | Custody checks | The confirmation must be the peer's own signed observation and meet the criteria (the planner's rules: triple, confidence ≥ 0.9, age ≤ 10 s and ≤ `valid_for`). Otherwise no release, `confirmation_not_accepted`, outcome `failed` with `custody_unknown`. The peer rejects a release that names no confirmation it sent (`C403`) | Cross-checking with a robot sensor of its own: the world has none that sees the box |
| P7 | Trust | Each receiver verifies with its counterparts' keys only (0013); a rejected message ends the exchange, planner `unknown`, exit 7 | One shared key set: it would hide a missing key |
| P8 | Record | With a peer: version **0.2**, plus `peer` and `commitment` (final local state) and new events. Without a peer: version 0.1 and bytes unchanged | Optional fields under 0.1: a consumer could not tell the exchange it describes |

## 3. Consequences

- `sim.outcome.object_triple` is public (the old private code, moved); `core_outcome` is unchanged.
- The peer's `ready` is read from the robot world; a `participant_busy` decline is only that.
- Not shown (spec 7): reservation of the handoff place, `coordinate`, a peer `outcome`, real sensing, `custody_query`
  recovery, lease checks after the exchange, more than one peer.

## Amendment — 2026-10-29

The lease is checked at **every act of the peer**, not only at the custody exchange, as the arch02
harness does since H4 (PR #49). A peer step that would start when the lease no longer holds is not
run: the run is cancelled before it, the commitment is `lapsed`, and the outcome is `failed` with
`lease_expired` (spec/delegate-v0.2.md section 3).
