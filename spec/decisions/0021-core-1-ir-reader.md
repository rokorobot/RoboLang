# Decision record 0021 — CORE-1: the Task IR reader and re-validator

**Date:** 2026-10-29
**Status:** **IMPLEMENTED, owner approval pending.** Work item CORE-1 ([contracts](../architecture/rl-foundation-02-domain-contracts.md) section on work items; plan [goals-and-core-bridge.md](../../docs/plans/goals-and-core-bridge.md)); not merged.
**Applies to:** [`ir-reader-v0.1.md`](../ir-reader-v0.1.md), [IR schemas](../ir-schema-v1.0.md) 1.0 to 1.2, [simulation](../simulation-v1.0.md) sections 5 and 6
**Builds on:** kernel rules K4, K5; [0015](0015-report-all-diagnostics.md)

## 1. Why

An AI can now write a goal in `.rob`, but it can also write the Task IR directly. The IR trust boundary
(schema 1.0 section 2) said a consumer must check the schema and re-establish the static rules itself.
`read_ir` does, so IR becomes a second input to the tools without a second, weaker path.

## 2. Decisions

| # | Topic | Rule | Alternative considered |
|---|---|---|---|
| R1 | Claims are not trusted | The version fields select a vocabulary; every kind, field and type is checked against it, then the static rules E301–E312 are re-applied | Trust a matching version pair: a 1.0 document using `and` would pass |
| R2 | Strict JSON | Reuse `core.read_document`; `ReadError` gains a `reason` so I101–I105 are distinct. Messages are unchanged | A second JSON loader |
| R3 | Codes | `I1xx` form, `I2xx` meaning, numbered like E3xx where a rule is the same (I201 = E301); located by JSON Pointer; all errors, cap 50 | Reuse E-codes: they locate by line and column |
| R4 | Form errors hide meaning errors per task | A task with a form error is not checked for meaning, other tasks are; a missing goal (I209) does not hide the rest | Two passes over the whole document: a form error anywhere would hide every meaning error |
| R5 | Canonical, not literal | Key order, white space and insignificant zeros (`"0.50"`) are accepted and not kept; `to_json(read_ir(x))` is canonical. Non-flattened `and`/`or` is rejected (I122), as 1.1 section 4 requires | Reject `"0.50"`: stricter than the schema's own wording ("digits with a fraction"), no safety gain |
| R6 | Sensors in schema 1.2 | A reference that is not a declared parameter is I210 (the IR form of E310, which the source reports for a string); a declared non-sensor is I204 | I203 for both: loses the hint that a world sensor ID was written |
| R7 | Run records | `inputs.source_kind: "ir"` only for IR input; `source_sha256` is the hash of the IR bytes; the run ID preimage gains `source_kind ir` after its first line, so a source and an IR file never share an ID. Source records and IDs are byte-identical (additive, optional, record schema versions unchanged) | A new record schema version: a note suffices for an optional key |
| R8 | CLI | `--ir IR` replaces the positional `FILE` (exactly one of them); same exit statuses; `delegate --ir` requires `--plans` | A new `read-ir` command: every command would need it |
| R9 | No new trust | `run()` still takes a `TaskIR`; reading is not authorization; duties 3 to 5 are not implemented | |

## 3. Consequences

- `core/reader.py` changes only by an attribute on `ReadError` and a `RecursionError` guard (C101 behaviour is the same).
- The `run()` docstring no longer refuses IR from `read_ir`; a `Task` built by hand is still unsupported.
- Not done: IR in Core messages (`plan_kind`), a JSON Schema file for the IR, a size limit on documents.
