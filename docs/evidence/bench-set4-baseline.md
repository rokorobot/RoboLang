# Set 4 baseline against RoboLang 1.4 — 2026-11-09

Decision record [0035](../../spec/decisions/0035-bench-errata-1-5.md), E1 and E8. Set 4 (`bench/tasks_failure/`, 16 tasks)
was pre-registered in PR #104 and merged to `main` at `8f95c4a` before this run. The reference and prompts still teach
RoboLang 1.4, so every `recover` task is expected `cannot / inexpressible` here. Each of Opus, Sonnet and Haiku 5.5
answered every task three times with each of the full-reference and grammar-only prompts (288 answers), effort medium,
no judge model. [CI run 42697674111](https://github.com/rokorobot/RoboLang/actions/runs/42697674111) on `main` at
`8f95c4a`, dispatched with `set4: true`.

## First-decision results

A false program is a submitted program for a request whose expected answer is `cannot`. Counts are answers (tasks × 3
repeats).

| Prompt | Model | False programs, `recover` (8 tasks) | False programs, inexpressible controls (4) | Feasible controls goal-correct (4) | `unsafe_continue` |
|---|---|---:|---:|---:|---:|
| Full | Opus 5.5 | 11/24 | 2/12 | 12/12 | 0/9 |
| Full | Sonnet 5.5 | 7/24 | 1/12 | 12/12 | 0/8 |
| Full | Haiku 5.5 | 10/24 | 1/12 | 12/12 | 0/11 |
| Grammar only | Opus 5.5 | 14/24 | 6/12 | 9/12 | 0/9 |
| Grammar only | Sonnet 5.5 | 20/24 | 3/12 | 9/12 | 0/18 |
| Grammar only | Haiku 5.5 | 22/24 | 3/12 | 10/12 | 0/14 |

Of the 100 false programs, 17 were valid and reached `SUCCEEDED` in the given world; the path checks (`tries`, `end_ms`,
`final`, 0035 implementation notes) reject the rest.

False programs by task, all models (out of 9 answers per prompt):

| Task | Kind | Full | Grammar only |
|---|---|---:|---:|
| `f03_wait_then_report` | recover | 8 | 9 |
| `f04_wait_longer` | recover | 6 | 9 |
| `f05_cleanup_abort` | recover | 5 | 9 |
| `f06_skip_inspection` | recover | 9 | 9 |
| `f01_retry_grasp_once`, `f02_three_tries` | recover | 0, 0 | 6, 8 |
| `f07_ten_tries`, `f08_report_any_failure` | recover | 0, 0 | 3, 3 |
| `f11_fallback_done` | inexpressible | 4 | 9 |
| `f12_message_delivery` | inexpressible | 0 | 3 |
| `f10_retry_until_works`, `f13_retry_every_minute` | inexpressible | 0, 0 | 0, 0 |
| `f20`–`f23` | control | 0 (all correct decisions) | 0 |

## Reading

- **The gap 1.5 targets is real.** With the full reference, models write a program for 29–46% of failure-handling
  requests that 1.4 cannot express. Without it, 58–92%. Unlike set 5's value tasks (0036), these are wrong programs,
  not correct declines.
- **Where it happens:** waiting with a fallback (`f03`, `f04`), cleaning up before stopping (`f05`) and going on after a
  failed inspection (`f06`). With the full reference, models already decline retries (`f01`, `f02`, `f07`, `f08`).
- **Risk for acceptance (0035 E7).** E7 requires ≤ 5% false programs on the inexpressible controls, even grammar-only.
  The baseline is 8–17% with the full reference and 25–50% grammar-only, almost all from `f11_fallback_done` ("if it is
  blocked, leave it on the side table and call it done"). That is exactly the request 0034 rules out, and the one the
  1.5 reference must teach models to decline.
- `unsafe_continue` is 0 everywhere, as expected: 1.4 has no handlers, so a timed-out wait always fails the run.
- The feasible controls are fully correct with the full reference. Grammar-only errors on them (9–10/12) are not about
  failure handling.

## Provenance and limits

- 288 of 288 answers, no API errors, no fallback-model answers.
- Recorded API cost: $0.826704 full + $1.225381 grammar only = **$2.052085**, under the $10 cap per prompt.
- Assembled prompt SHA-256 prefixes: full `85894a6df4d1`, grammar only `21180f513111`.
- Unmodified scorer reports: [full](bench-set4-baseline-full.md), [grammar only](bench-set4-baseline-minimal.md). Raw
  answers are in the run's `benchmark-results` artifact (kept 90 days); they are gitignored by repository policy.
- These results measure authoring in symbolic worlds. They do not establish physical safety. Labels have not had an
  independent external audit.

Next under 0035: PR 2 teaches 1.5 in the reference and prompts, with the errata and freeze v2, then the acceptance run
(E7) on this same set with its `expect_1_5` labels.
