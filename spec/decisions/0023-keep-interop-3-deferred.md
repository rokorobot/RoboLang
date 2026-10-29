# Decision record 0023 — Keep INTEROP-3 deferred until Core 0.2 or a second implementer

**Date:** 2026-10-29
**Status:** APPROVED by the owner — 2026-10-29
**Amends:** [0017](0017-reprioritise-goals-before-interop-3.md) R3 (the condition for reconsidering INTEROP-3)

## Context

0017 deferred INTEROP-3 until `.rob` tasks compile to Core. That now holds: `export-goal`,
`delegate` (with a peer, a lease and custody), Core outcomes in run records, and CORE-1. The frozen
gate 2 bundle (`e6c9bb75…d104`) is unchanged and still describes Core 0.1 exactly.

## Decision

| # | Decision |
|---|---|
| K1 | INTEROP-3 and INTEROP-4 **stay deferred**. The bundle stays frozen; the [INTEROP-3 protocol](../../docs/interop/interop-3-protocol.md) stays approved for preparation. |
| K2 | Reconsider INTEROP-3 when **either** Core reaches 0.2, **or** a real second implementer or adopter needs auditable interoperability evidence. |

## Reasons

1. **Core 0.1 is still moving.** Building the bridge found gaps: no way to say a thing is
   nowhere (absent), outcomes renumbered when the goal order differs, and reservation,
   `coordinate` and custody recovery still unused in practice. An independent implementation of a
   draft that will change mostly produces errata, a re-freeze and a second run.
2. **The cost is high for one owner.** The protocol needs an isolated environment, a separate
   model session, a signed operator attestation, a Q&A intermediary and a full audit trail, and
   nobody is waiting for its result.
3. **A bigger risk is untested.** RoboLang's central claim is that AI systems write it well. The
   v2.0 benchmark of model-written tasks has not started; it is the next priority.

## Not changed

Physical execution remains unauthorized. A cheap, informal check of the spec (an independent
implementation written from the bundle, labelled "not gate 2 evidence") may run at any time, and
its findings feed Core 0.2.
