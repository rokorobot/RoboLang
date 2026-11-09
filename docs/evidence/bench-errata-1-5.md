# Benchmark errata for RoboLang 1.5 — 2026-11-09

Decision record [0035](../../spec/decisions/0035-bench-errata-1-5.md), PR 2 (E2, E6, E9). From this change on, the
reference and both prompts teach RoboLang 1.5. Every benchmark input that changed is listed here, with its old and new
form. The set 4 baseline before these changes is [bench-set4-baseline.md](bench-set4-baseline.md) (1.4, `main` at
`8f95c4a`).

## Labels (E2)

The old label is kept in each task as `expect_1_4`.

| Set | Task | Before (`expect`) | After (`expect`) | Paths |
|---|---|---|---|---|
| 4 | `f01_retry_grasp_once` … `f08_report_any_failure` (the 8 `recover` tasks) | cannot / inexpressible | program | from `expect_1_5`, unchanged since PR 1 |
| 3 | `x_failover_on_fault` | cannot / inexpressible, group `inexpressible` | program, group **`recover`** | from `expect_1_5` (arm_a ready / not ready); new reference `bench/reference_branch/x_failover_on_fault.rob` |

Unchanged labels, `why` text only:

| Set | Task | Before | After |
|---|---|---|---|
| 2 | `x_retry_grasp` | "Reacting to a failed step needs error handling and a loop. A failing step ends the run; there is no retry, catch or loop construct." | "\"As many times as it takes\" is a loop. RoboLang 1.5 failure handlers allow a fixed number of retries (decision 0033 F5), but no loop and no unbounded retry." |
| 3 | `x_retry_until_grasped` | "Reacting to a failed step needs error handling and a loop. 1.4 branches on world conditions only (decision 0027); a failing step still ends the run." | "\"Until it works\" is a loop. RoboLang 1.5 failure handlers allow a fixed number of retries (decision 0033 F5), but no loop and no unbounded retry; 1.4 branches on world conditions only (decision 0027)." |
| 3 | `x_failover_on_fault` | "… In 1.3 and 1.4 a failing step ends the run." | "… In 1.3 and 1.4 a failing step ends the run; RoboLang 1.5 failure handlers (decision 0033) express it." |

No label in set "ask" (`bench/tasks_ask/`) or set 1 changes. Set 3 now has 7 `inexpressible` tasks and 1 `recover`.

## Reference (E6)

`docs/language-reference.md` and its package copy `src/robolang/reference/language-reference.md` (identical):

| Before (1.4) | After (1.5) |
|---|---|
| Title "RoboLang 1.4 language reference"; files start `robolang 1.4;` | "RoboLang 1.5 language reference"; `robolang 1.5;` |
| "no loops, retry, error handling, counts, …" | step failure handlers exist; "no loops ("until it works"), counts, …" |
| `wait until region_clear cell timeout 30 s;` | `wait until … timeout 30 s else { fail "cell not clear"; }`; `wait until` needs a handler (`E316`) |
| `if` bullet: "nothing is retried … a retry … cannot be represented" | an `if` is decided by the world; to react to a failed step use a handler; loops still cannot be represented |
| — | New section "Failure handlers": which steps take one (`E201` for the others); one handler per step, none for a group; a failed step changes nothing; **a wait's handler is `fail` unless the request says what to do**, and ends in `fail` unless the request says carry on; **a handler has no goal and never changes what success means**, so "otherwise … and call it done" cannot be represented; a fixed number of retries by nesting (at most 4 levels, `E216`) or by `if not holding` steps; "until it works" is a loop; paths include each handled step (`E309`) |
| Diagnostics table | adds `E201`, `E316`; `E208`, `E216`, `E313` extended to handlers |
| Two complete examples (1.4) | both 1.5, the first with a wait handler; a third example retries a grasp once, then reports and stops |

## Prompts

| File | Before | After |
|---|---|---|
| `bench/prompt.md` | "You write RoboLang 1.4 programs", "when RoboLang 1.4 cannot say it"; example `robolang 1.4;` with `wait until … timeout 20 s;` | 1.5 in both places; example `robolang 1.5;` with `wait until … timeout 20 s else { fail "the cell did not clear"; }` |
| `bench/prompt_minimal.md` | 1.4; pastes `grammar-v1.3.ebnf`, `grammar-v1.4.ebnf` | 1.5; pastes `grammar-v1.3.ebnf`, `grammar-v1.4.ebnf`, `grammar-v1.5.ebnf`, in that order (`bench/lib.py` `GRAMMAR_FILES`) |

The assembled prompt hashes change; each run records its own.

## Other changes in the same PR

- `robolang reference` and the CLI help said "1.3" (stale since 1.4); they now say 1.5, and the CLI help says it reads
  1.1 to 1.5 files. README and `bench/README.md` updated to match.
- Tests that pinned the 1.4 labels, prompts and reference now pin the 1.5 ones (`tests/test_reference.py`,
  `tests/test_bench.py`, `tests/test_bench_branch.py`, `tests/test_bench_failure.py`).

## Freeze v2 (E9)

`tools/bench_freeze.py` now writes and checks `docs/evidence/bench-freeze-v2.json` (schema `robolang.bench.freeze.v2`)
and covers every `spec/grammar-v1.*.ebnf`. The v1 manifest (`bench2-freeze.json`, decision 0025) is kept unchanged as
history; it no longer matches `main`. Freeze v2: **726 files**, digest
`fb4b0f59795d948ca2948e1b5654ab0ea2cd23d30bdd317cf11b26e94e293da7`. Run
`python tools/bench_freeze.py --check` before a paid run; any later change to a covered file needs a new erratum.

## What happens next (E7)

The acceptance run: set 4 under 1.5 with both prompts and 3 repeats, plus sets 1, 2, 3 and "ask" once each with the
full prompt. If E7 fails, the reference goes back to 1.4 and the failing measure is analysed before another try.
