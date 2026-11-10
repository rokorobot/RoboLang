# INTEROP-4, run 1: gate 2 with an independent amr-07 — 2026-11-10

Decision record [0030](../../spec/decisions/0030-relaunch-interop-3-on-core-0-2.md) J6 steps 5 and 6;
[gate 2 specification](../../spec/interop-gate-2-v0.1.md) sections 8 and 9. **Verdict: gate 2 not yet passed.** 24 of 25
cases meet section 9.1; one case fails on a **specification gap** (finding F1), which section 9.4 requires to be
resolved by an erratum before gate 2 can pass. Owner sign-off pending.

## 1. Identity

| Item | Value |
|---|---|
| Frozen bundle | `f3164c5afc54893be232433c785f11f765dae780483a3e8d7ba28178247c7e22`, source commit `39bc07a`; archive `ba874251…da70`, rebuilt and re-verified 2026-11-10 |
| Independence (gate 2 2.1) | Clean-room AI: **OpenAI `gpt-6-astra`** (session log), Codex CLI 0.167.0 via Cloudroom, reasoning effort medium; new Cloudroom project containing only the four kit files; Codex memory off |
| Deviation (owner, 2026-11-10, option A′) | Network and external search **not technically blocked** (the Cloudroom sandbox allows no firewall, Codex has no search switch); forbidden in the prompt and **verified by full transcript audit** ([runbook](interop-3-cloudroom-runbook.md) section 8) |
| Operator and intermediary | The owner. The RoboLang session prepared the kit and audit and took no role in the author session |
| Deliverables | `impl.tar.gz` and the Codex session log `codex-sessions.tar.gz` ([hashes](interop-4/run-1-deliverables.sha256)), attached to the draft release `interop-3-author-kit` |

## 2. Transcript audit (INTEROP-3)

The session log has five turns: environment check (operator), launch authorization (operator), the task, one answer
(intermediary), delivery (operator). In the task and answer turns the author used one tool, `exec`, 32 times; every
command is local (compile, run, tests, a local `git` commit with no remote). No web search record exists in the
session, and no search tool was in the tool list offered (`exec`, agent and messaging tools only). No sub-agent was
used. No network API (`http`, `https`, `net`, `fetch`) appears in the commands or the implementation; `child_process`
appears once, in a test that starts the implementation itself. The operator's environment check and delivery were the
only network use, outside the work. **Result: clean.**

## 3. Q&A log

| # | Question (author) | Answer (intermediary, from the bundle only) |
|---|---|---|
| Q1 | What should a receiver do when an observation ID cited by verification is missing from the outcome's `evidence` list: reject at stateless checking (code, stage, precedence) or only at re-evaluation? | The bundle specifies no rejection code, stage or precedence for it (Core 0.1 8.2 and the Core 0.2 table name none). Quoted: Core 0.1 5.9, Core 0.2 3, RFC-001 11.3 ("If re-evaluation fails, it treats the outcome as `unknown`"). Do not add a code the bundle does not define. |

## 4. Results

G-T0 (author, and re-run here from the delivered code against the frozen bundle):
**109/109** ([results](interop-4/run-1-g-t0-conformance.jsonl)): 47 Core 0.1 fixtures and 62 Core 0.2 entries.

G-T1 to G-T5, by [`tools/interop_gate2_run.py`](../../tools/interop_gate2_run.py) (section 9.1 items 1–5). The runner
first passed the reference amr-07 through the same binding: 25/25, all deterministic
([self-test](interop-4/run-1-reference-selftest.json)). Independent run ([results](interop-4/run-1-results.json)):

| Test | Cases | Passed | Notes |
|---|---|---:|---|
| G-T1 Happy path | 2 | 2 | |
| G-T2 amr-07 under faults | 10 | **9** | `v08_arrived_without_evidence`: binding failure (F1) |
| G-T3 Sequencing | 9 | 9 | |
| G-T4 Regression | 4 | 4 | |
| G-T5 Determinism | 25 | 24 | every completed case byte-identical on a second run |

Measured, not required (9.2): in all 24 completed cases **every message the independent amr-07 sent has the same JCS
form as the reference's** (100%).

## 5. Findings (9.3)

| # | Class | Finding | Blocks gate 2 |
|---|---|---|---|
| F1 | **1, specification gap** | Gate 2 spec 6.3 lists `sensed` as `now_ms`, `request`, `value` (or `null`), `observed_at_ms`, and does not say what `observed_at_ms` is when `value` is `null`. The reference harness sends `null`; the independent amr-07 requires an integer and closes the binding ("Unexpected sensed reply") in `v08_arrived_without_evidence`, where amr-07's camera is unavailable. Both readings follow from the text. Diagnostic only (not evidence): with that one check relaxed in a local copy, all 25 cases pass and are deterministic, so F1 is the only difference. | **Yes** |
| F2 | 1, specification gap | From Q1: Core 0.1 5.9 and Core 0.2 3 require every cited observation in the `evidence` list but define no rejection code for a breach. No run showed a behaviour difference. | No (record for a later erratum) |

## 6. What closes gate 2

1. **Erratum decision for F1** (owner). Recommended: gate 2 spec 6.3 states that `observed_at_ms` is `null` exactly
   when `value` is `null` (the reference's behaviour, and the only one that carries no invented time). Alternative: the
   harness sends `now_ms` there; it invents a reading time for a reading that did not happen.
2. A new frozen bundle with the erratum (and, if the owner wishes, F2), and the reference re-checked against it.
3. The new bundle delivered to the author session as an erratum, the author fixes its implementation **from the bundle
   alone**, and the transcript is audited again.
4. INTEROP-4 run 2 with the same runner; gate 2 passes when every case meets 9.1, every finding is resolved and
   recorded, and the owner signs off (9.4).
