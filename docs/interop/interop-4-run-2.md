# INTEROP-4, run 2: gate 2 with the independent amr-07, after erratum 0037 — 2026-11-10

Follows [run 1](interop-4-run-1.md). Decision record [0037](../../spec/decisions/0037-gate-2-errata-f1-f2.md) (F1, F2,
approved by the owner 2026-11-10) resolved run 1's findings; the bundle was re-frozen
([freeze record](gate2-bundle-freeze-erratum-0037.md)) and delivered to the author session as an erratum.
**Result: every gate 2 criterion of section 9.1 is met for every case. Gate 2 passes on the owner's sign-off (9.4).**

## 1. Identity

| Item | Value |
|---|---|
| Frozen bundle | `3a33dce09fd3be38deef191fe4c2c662c1f4094848c7f38aa5c53ecc6cc112a7` (122 files), source commit `05859a6`; archive SHA-256 `16a97331…7194`, verified in the author's environment (hash, 122 files, manifest, `sha256sum -c`) |
| Independent author | Unchanged: OpenAI `gpt-6-astra`, Codex CLI 0.167.0 via Cloudroom, same session; option A′ deviation and audit as in run 1 |
| Deliverables | `impl-run2.tar.gz`, `codex-sessions-run2.tar.gz` ([hashes](interop-4/run-2-deliverables.sha256)), draft release `interop-3-author-kit` |

## 2. Transcript audit (erratum turn onward)

The only network command after run 1 is the operator's download of the new bundle (record 355), exactly as pasted.
The author's work on the erratum is 9 tool calls: reading the marked erratum text, editing its own files with local
Node scripts, `npx --offline tsc`, its tests, a local `git` diff and commit, and a `tar` of its package. No web search
record exists in the whole session. **Result: clean.**

## 3. Results

- G-T0, re-run here from the delivered code against bundle `3a33dce0`: **109/109**
  ([results](interop-4/run-2-g-t0-conformance.jsonl)). The author reports 31/31 of its own tests.
- Runner self-test with the reference amr-07: 25/25 ([self-test](interop-4/run-2-reference-selftest.json)).

| Test | Cases | Passed |
|---|---|---:|
| G-T1 Happy path | 2 | 2 |
| G-T2 amr-07 under faults | 10 | **10** (`v08_arrived_without_evidence` now passes) |
| G-T3 Sequencing | 9 | 9 |
| G-T4 Regression | 4 | 4 |
| G-T5 Determinism | 25 | 25 |

Measured, not required (9.2): **442 of 442** messages the independent amr-07 sent have the same JCS form as the
reference's. [Full results](interop-4/run-2-results.json).

## 4. Findings

| # | Status |
|---|---|
| F1 `observed_at_ms` with a `null` reading | Resolved by erratum 0037; the author implemented it from the bundle alone; `v08` passes |
| F2 missing cited evidence | Resolved by erratum 0037 (clarification; no behaviour difference) |

No new finding.

## 5. Gate 2 verdict (9.4)

| Condition | State |
|---|---|
| G-T0 to G-T5 meet 9.1 for every case at one frozen bundle | **Met** (bundle `3a33dce0`) |
| At the independence level of I1 | **Met, with the recorded deviation:** clean-room AI; network and external search not technically blocked; full transcript audit found neither used (runbook section 8) |
| Every finding resolved and recorded | **Met** (F1, F2: decision 0037), once PR #120 is merged |
| Evidence complete (section 11) | **Met:** findings log, Q&A log, both runs, audits, deliverable hashes |
| Owner sign-off | **Pending** |

With the owner's sign-off, gate 2 passes and STRATEGY H3's second exit criterion ("independent Core 0.2 implementation
interoperates") is met. A protocol-compliant run (technically isolated, or a human team) remains possible later and
would be stronger evidence of the same claim.
