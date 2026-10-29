# Decision record 0017 — Goals and usability before INTEROP-3

**Date:** 2026-10-29
**Status:** APPROVED by the owner — 2026-10-29
**Applies to:** [`docs/DEVELOPMENT_PLAN.md`](../../docs/DEVELOPMENT_PLAN.md), Track C (INTEROP-3, INTEROP-4)

## Context

The `.rob` language and Core 0.1 are not connected yet: `.rob` does not compile to Core, and
the reference simulation (`robolang arch02`) does not run `.rob`. Gate 2 (independent
interoperability) would test a protocol that nothing in the language produces yet.

The charter makes success a verified outcome, but before RoboLang 1.3 a `.rob` task could not
declare one, and `simulate` reported `SUCCEEDED` when the steps had only run.

## Decision

| # | Decision |
|---|---|
| R1 | INTEROP-3 and INTEROP-4 are **deferred**. The gate 2 bundle stays frozen as recorded in the [freeze record](../../docs/interop/gate2-bundle-freeze.md); nothing in it changes. |
| R2 | Work now goes, in order, to: declared goals that decide `SUCCEEDED` (RoboLang 1.3, [0014](0014-lang-2-goals-and-typed-sensors.md)); reporting every diagnostic at once ([0015](0015-report-all-diagnostics.md)); task bindings on the command line ([0016](0016-cli-bindings.md)); typed sensor parameters (part of 1.3); repository and README cleanup. |
| R3 | INTEROP-3 is reconsidered once `.rob` tasks compile to Core documents (CORE-1 or its successor), so that gate 2 exercises the language end to end. |

## Not changed

Safety posture: simulation only; physical execution remains unauthorized.
