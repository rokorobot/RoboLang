# Decision record 0023 — Resume INTEROP-3 on the frozen gate 2 bundle

**Date:** 2026-10-29
**Status:** PROPOSED — owner approval pending. Launch also needs the steps of the
[INTEROP-3 protocol](../../docs/interop/interop-3-protocol.md) (model ID verified, operator attestation, launch authorization).
**Amends:** [0017](0017-reprioritise-goals-before-interop-3.md) R1 and R3

## Context

0017 deferred INTEROP-3 until `.rob` tasks compile to Core. That condition now holds: goals
map to Core triples (`bridge.py`), `export-goal` emits Core `propose_goal` messages, `delegate`
runs a goal end to end through Core messages (with a peer, a lease and custody), run records
carry Core outcomes, and CORE-1 reads Task IR JSON.

None of that changed Core 0.1. The gate 2 bundle (`e6c9bb75…d104`, frozen at `28c69fe`) still
describes the Core receiver and amr-07 exactly; `tests/test_interop_bundle.py` rebuilds it
byte-identically on every run.

## Decision

| # | Decision |
|---|---|
| I1 | Lift the 0017 deferral. INTEROP-3 proceeds under its approved protocol, on the **existing** frozen bundle. |
| I2 | Do not re-freeze the bundle to include the language bridge. Gate 2 tests what is hardest to get right independently, the Core 0.1 receiver; the bridge is a thin layer on Core and is tested by the reference implementation. |
| I3 | Independent implementation of the bridge (`export-goal` mapping, `delegate` verdict rules) is a candidate for a later gate, once `delegate-v0.2` has settled. |

## Not changed

Physical execution remains unauthorized. INTEROP-4 still needs separate authorization.
