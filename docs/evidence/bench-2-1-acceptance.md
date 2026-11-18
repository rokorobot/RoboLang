# RoboLang 2.1 acceptance (I5, step L5)

**Run:** 2026-11-18, `Benchmark` with `set_only: messages` on the branch of PR #163,
[run 44363305012](https://github.com/rokorobot/RoboLang/actions/runs/44363305012). Set 10 under 2.1, Sonnet 5.5 and
Haiku 5.5, full and grammar-only prompts, 3 repeats. **Cost $1.37.** Bounds B1–B9 were approved before the run
([plan](../plans/bench-10-messages.md)); the baseline is in [bench-set10-baseline.md](bench-set10-baseline.md).
B6 comes from the per-PR run on PR #163 (sets 1 and 2, 1 repeat).

| # | Measure (full prompt unless stated) | Bound (per model) | Sonnet | Haiku | Verdict |
|---|---|---|---|---|---|
| B1 | message programs: right on every path, `message_ok` | ≥ 27/30 | **30/30** | **30/30** | met |
| B2 | the same, grammar-only (information) | — | 28/30 | 26/30 | — |
| B3 | inexpressible: false programs, full and grammar-only (`x01`–`x04`) | ≤ 1 per prompt | 0/12, 0/12 | 0/12, 0/12 | met |
| B4 | infeasible: `cannot`; no program that drops the wait and succeeds | ≥ 14/15 and 0 | 15/15, 0 | 15/15, 0 | met (both prompts) |
| B5 | controls: right, no message wait added | ≥ 11/12 | 12/12 | 12/12 | met |
| B6 | sets 1 and 2: `gate.json` | the gate passes | 40/40, 39/40; false programs 0/13 | 40/40, 39/40; 0/13 | met |
| B7 | economy | 0 statements, 0 new reserved words, 1 condition | 0, 0, 1; 4 user-facing codes (`E327`, `E519`, `S011`, `X608`) plus `S515` and `I228` | | met |
| B8 | message wait replaced silently | ≤ 1/30 | 0/30 | 0/30 | met |
| B9 | channel or text changed | ≤ 1/30 | 0 | 0 | met |

## Notes

- **`x05_a_message_in_the_goal`, relabelled a program by C14.** Both models declined it in 5 of 6 full-prompt
  answers. The request says "state both as the goal". 2.1 forbids a message in a goal (S1, `E327`), and the
  models read the request literally. No bound covers this task: it is a false refusal for information only, and
  it suggests that C14 was the wrong call. Recommended follow-up: revert `x05` to `cannot / inexpressible` by
  erratum, with no rerun needed.
- **Grammar-only B2:** the misses are `m08_open_the_door_when_unlocked`, where the goal leaves out the
  `in_state` result, and one `m03`. For information.
- PR #163's Windows tests failed on the new scorer-speed test: Windows imported an unrelated `resource` module, so
  memory is now measured only where `getrusage` exists. Nothing in the language was affected.

## Verdict

**ACCEPTED by the owner, 2026-11-18.** Every bound is met. `x05` is declined again (erratum N3). **Paid cost of I5:** baseline $0.99, acceptance $1.37.
