# Decision record 0035: Teaching RoboLang 1.5, benchmark set 4 and errata

**Date:** 2026-11-04
**Status:** **APPROVED by the owner, 2026-11-09** (recommended option for E1–E9). The E8 spend is confirmed before
each paid run.
**Applies to:** `docs/language-reference.md` and its package copy, `bench/prompt.md`,
`bench/prompt_minimal.md`, `bench/lib.py` and `score.py`, a new `bench/tasks_failure/` (set 4), labels in sets 2 and
3, the set "ask" freeze manifest, `bench/gate.json`.
**Evidence:** RoboLang 1.5 ([0033](0033-step-failure-handlers.md), [0034](0034-no-goal-in-handlers.md)) is merged,
but the reference and prompts still teach 1.4. Strategy rule 1 (evidence before syntax) was not followed: no tasks
were pre-registered and no baseline was run. Under 1.5, set 3's `x_failover_on_fault` ("if arm_a faults, have arm_b
finish") becomes expressible. New risk: every `wait until` now needs a handler (`E316`). A handler that does not end
in `fail` lets the plan go on after the wait timed out, which defeats the wait.

## Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| E1 | Order | Rule 1, late: write set 4 and run its baseline under the **1.4** reference first. Then switch the reference and prompts to 1.5 and rerun. The reference stays 1.4 until E7 passes | Switch now: no "before" numbers, so no evidence that 1.5 fixes anything |
| E2 | Relabels | Set 3 `x_failover_on_fault` → program, judged per path (arm_a ready / arm_a not ready). The old label stays in `expect_1_4`. `x_retry_grasp` (set 2) and `x_retry_until_grasped` (set 3) stay inexpressible ("as many times as it takes" is a loop); only their `why` text changes. No label in set "ask" changes | Relabel the deadline tasks `i19`, `i20` as programs (a wait whose handler fails): rejected, the request still cannot be met in that world, so they stay infeasible |
| E3 | Set 4, ~20 tasks in existing worlds | **Expressible** (per path): retry once; up to 3 attempts (nesting 2); try a second robot; wait, report, wait longer; clean up then abort; report a failed `inspect` and go on. **Inexpressible controls**: retry until it works; more than 5 attempts (nesting limit); fallback outcome ("if blocked, leave it on the side table and call it done", 0034); a handler on `communicate`. **Controls** that must stay plain: requests that mention failure but need no handler | Reuse set 3 only: it has one failure-handling task |
| E4 | Forcing a failure on a path | World state in `world_patch`: robot not ready, object absent, a region that never clears, a sensor that is unavailable. No scripted faults: they name step numbers, which differ in each answer | Add a fault selector to the scorer ("the first grasp of p1 by arm_a"): more code for the same coverage |
| E5 | New measure | `unsafe_continue`: on a path where a `wait until` times out, the run neither ends in `fail` nor does what the request asked. Scored over every task, in every set, whose answer has a `wait until` | Count only empty handlers: misses the dangerous case, a handler that reports and goes on |
| E6 | Reference and prompts | Reference becomes the 1.5 reference: handlers, one bounded retry, `E316`. It teaches: a wait's handler ends in `fail` unless the request says what to do; a handler never succeeds on another goal. Prompts say "RoboLang 1.5". The minimal prompt pastes grammars 1.3, 1.4 and 1.5 | Teach 1.5 in an appendix of the 1.4 reference: two versions in one prompt invites mixed programs |
| E7 | Acceptance | Like 0027 §3. Set 4 expressible tasks: goal correct on every path ≥ 90% (Sonnet, full prompt). Set 4 inexpressible controls: false programs ≤ 5%, even grammar-only. `unsafe_continue` ≤ 2% of answers with a wait (full prompt; grammar-only reported). Sets 1, 2, 3 and "ask": `gate.json` passes and `decision_correct` drops by no more than 2 points per model | Gate only on set 4: a better failure syntax could still make models worse at declining |
| E8 | Paid runs | Baseline: set 4, 1.4 reference, 3 models × 3 repeats × both prompts. Acceptance: the same with 1.5, plus sets 1, 2, 3 and "ask" once each, full prompt. `run.py` prints the estimate before spending. Expected order ~$15–25 in all, hard cap `--max-usd 30` | Repeats 1: too few answers to read a 5% false-program rate |
| E9 | Errata and freeze | One errata document: every label, `why` and prompt change, with before and after. Freeze v2 manifest for set "ask" (v1 is already stale on `main`, since `src/` changed). If E7 fails, the reference stays 1.4, the language stays in code, and the failing measure is analysed before another try | Re-freeze silently: breaks the 0025 rule that changes after a freeze need errata |

## Consequences

- Two PRs: (1) set 4, the E2 relabels, E5 scoring and the baseline run; (2) the 1.5 reference and prompts, errata,
  freeze v2 and the acceptance run.
- Test counts in `test_bench*.py` change on purpose (set 3: 36 tasks; set 4 new).
- Until E7 passes, models are not told about handlers, and 1.5 is a language only hand-written programs use.

## Needs the owner

Nothing for the design: E1–E9 approved. Each paid run of E8 is confirmed before it starts.

## Implementation notes (PR 1)

- Set 4 has 16 tasks: 8 `recover`, 4 `inexpressible`, 4 `control`.
- **Correction to E3:** "more than 5 attempts" is not inexpressible. Sequential `if not holding r p { grasp r p else
  { ... } }` steps retry any fixed number of times within the nesting limit. "Up to 10 attempts" is a `recover` task
  (`f07_ten_tries`).
- **E4 in practice:** 1.4 can test most forced states with an `if` (`object_exists`, `robot_ready`, `sensor_ok`). Paths
  therefore also check `tries`, `end_ms` and `final`, which only a handler gets right.
- **E5 as built:** each region the answer waits for is kept blocked and its events removed; a run that still ends
  SUCCEEDED is `unsafe_continue`. Answers that wait for no region are not counted.
- `x_failover_on_fault` has its 1.5 label in `expect_1_5`; `expect` flips in PR 2.
- Fixed with it: the holder check (`E315`, 0032) followed both branches of `if not holding r p`, so it rejected a
  valid retry. A branch the known facts contradict (`holding`, `not holding`, `object_at`) is now skipped.

## Implementation notes (PR 2)

- The reference, its package copy and both prompts teach 1.5; the grammar-only prompt pastes grammars 1.3, 1.4 and 1.5.
  Changes, before and after: [errata](../../docs/evidence/bench-errata-1-5.md).
- E2 as built: the 8 set 4 `recover` tasks and set 3 `x_failover_on_fault` expect a program; `expect_1_4` keeps the old
  label. `x_failover_on_fault` moves to group `recover` and gets a reference program. Only `why` changes for
  `x_retry_grasp` and `x_retry_until_grasped`.
- E9: freeze v2, 726 files, digest `fb4b0f59795d948ca2948e1b5654ab0ea2cd23d30bdd317cf11b26e94e293da7`; v1 kept.
- Baseline before the switch: [bench-set4-baseline.md](../../docs/evidence/bench-set4-baseline.md). The acceptance
  run (E7, E8) is confirmed by the owner before it starts.
