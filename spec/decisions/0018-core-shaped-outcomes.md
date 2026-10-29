# Decision record 0018 — Core-shaped outcomes in run records

**Date:** 2026-10-29
**Status:** IMPLEMENTED (phase 2 of [the plan](../../docs/plans/goals-and-core-bridge.md)); owner approval pending
**Applies to:** [`spec/simulation-v1.0.md`](../simulation-v1.0.md) sections 10.4 and 10.5 (run record 1.2, unreleased, amended in place)
**Builds on:** plan decision Q1 (simulator ground truth, labelled `rl:sim:world`), Q3, Q5

## Decisions

| # | Topic | Rule | Alternative considered |
|---|---|---|---|
| O1 | Field | `core_outcome`, last key of a 1.3 record: `{outcome, evidence}`, the payload of a Core `outcome` message. 1.1 and 1.2 records are unchanged | Embed in `goal` entries: mixes the language's three-valued results with Core's two-valued `met` |
| O2 | Not exportable | `core_outcome: null` when `bridge.goal_triples` raises `NotExportable` (Q3). The key is present, so a reader can tell "no Core form" from "old record" | Omit the key |
| O3 | Status | `SUCCEEDED` → `succeeded`; `S302` → `unknown` (reason `evidence_missing`); other `FAILED` → `failed`; `CANCELLED` → `cancelled` | `S302` as `failed`: hides that nothing was disproved |
| O4 | Verification | `null` when the goal was not evaluated; else `criteria_evaluated` by `rl:sim:world`, one result per goal condition, `met` = the condition is `true` | |
| O5 | Evidence | One observation per criterion whose subject is known, stating what is actually true (`carried_by` its holder, or `located_in` its place), not the criterion. `met` is then exactly "observed triple equals criterion triple". Unknown or absent subject: no observation, `evidence: []`. Goal not evaluated: no evidence | Observe only the criterion triple: it would be evidence for the claim, not of the world |
| O6 | Identity and signing | Observation ID `rl:obs:<first 16 hex digits of the run ID>-<i>`; `confidence` `"1"`, `valid_for` 10 s; `sim-hmac-sha256` with a fixed, published, simulation-only key (`SIM_KEY`) | Per-run random key: breaks determinism |
| O7 | Run ID | Unchanged. It is derived from the inputs only | |

## Consequences

- Same inputs → byte-identical record, outcome and signatures.
- `C402` holds by construction (`SUCCEEDED` needs every goal condition `true`) and is tested on every 1.3 fixture.
- The signatures prove nothing about identity (the key is public); they let a Core receiver run its checks on simulator evidence. Robot-observation-only verification stays Foundation 2 (Q1).
- An object absent from the world, or one with unknown presence, has no Core observation: Core 0.1 has no "nowhere" value. Its criterion is simply not met.
