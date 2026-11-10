# Decision record 0037: Gate 2 errata F1 and F2 (from INTEROP-4 run 1)

**Date:** 2026-11-10
**Status:** **APPROVED by the owner, 2026-11-10**: F1 as recommended, and F2 in the same erratum.
**Evidence:** [INTEROP-4 run 1](../../docs/interop/interop-4-run-1.md): 24 of 25 gate 2 cases pass with the independent
amr-07; finding F1 is the only difference. F2 comes from the author's question Q1 (no behaviour difference).
**Applies to:** [gate 2 specification](../interop-gate-2-v0.1.md) 6.3, [Core 0.1](../core-v0.1.md) 5.9,
[Core 0.2](../core-v0.2.md) 3; a new frozen bundle. Under gate 2 9.3 both are class 1 (specification gap).

| # | Gap | Decision | Reference |
|---|---|---|---|
| F1 | The `sensed` frame lists `observed_at_ms` but does not say what it is when `value` is `null` | `observed_at_ms` is `null` exactly when `value` is `null`: no reading, no reading time | Already so (`interop/harness.py`); no change |
| F2 | Core requires every cited observation ID in the `outcome`'s `evidence` list but defines no code for a breach | Not a rejection: no stage of Core 8 checks it. A re-evaluating receiver finds that criterion without evidence (`evidence_missing`), so re-evaluation fails and the outcome is treated as `unknown`, not `succeeded` (RFC-001 11.3) | Already so (`core/receiver.py` has no such check; `delegate.py` treats a missing cited ID as `evidence_missing`); no change |

Alternative for F1, rejected: the harness sends `now_ms` as `observed_at_ms`, inventing a time for a reading that
did not happen.

## Consequences

- A new gate 2 bundle is built and frozen from the commit that carries this record; its freeze record names it.
- The new archive goes to the author session as an erratum delivery (protocol 2.4); the author fixes its
  implementation from the bundle alone; the transcript is audited again; INTEROP-4 run 2 follows.
- Core 0.1 and 0.2 change by clarification only: no message, schema, code or receipt changes.
