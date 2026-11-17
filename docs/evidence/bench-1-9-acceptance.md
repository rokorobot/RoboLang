# RoboLang 1.9 acceptance (I3, step L5)

**Run:** 2026-11-17, `Benchmark` with `clock: true` on the branch of PR #155,
[run 44200706311](https://github.com/rokorobot/RoboLang/actions/runs/44200706311). Set 8 under 1.9 (world 1.5
copies), Sonnet 5.5 and Haiku 5.5, full and grammar-only prompts, 3 repeats. **Cost $1.12.** Bounds B1–B8 were
approved before the run ([plan](../plans/bench-8-clock.md)); the baseline is in
[bench-set8-baseline.md](bench-set8-baseline.md). The figures are re-scored offline after erratum K2
(`k01_scan_stamp` also accepts "the box stays on shelf_a", the reading of set 2's `k_inspect_thrice`;
[errata](bench-errata-1-9.md)). No new request was sent. B6 comes from the per-PR run on PR #155 (sets 1 and 2,
1 repeat).

| # | Measure | Bound (per model) | Sonnet | Haiku | Verdict |
|---|---|---|---|---|---|
| B1 | clock programs: goal and `clock_ok`, full prompt | ≥ 27/30 | 27/30 | 29/30 | **met** |
| B2 | the same, grammar-only (information) | — | 23/30 | 23/30 | — |
| B3 | inexpressible: false programs, full and grammar-only | ≤ 1/15 per prompt | 0/15, 0/15 | 0/15, 0/15 | **met** |
| B4 | infeasible: `cannot`, no valid program that succeeds | ≥ 14/15 and 0 | 15/15, 0 | 15/15, 0 | **met** on the full prompt; grammar-only Haiku 12/15 with 0 that succeed (see below) |
| B5 | controls: right decision, no clock reading added, full prompt | ≥ 11/12 | 12/12 | 12/12 | **met** |
| B6 | sets 1 and 2: `gate.json`, decision accuracy | the gate passes; ≤ 2 points or one answer lost | 40/40, 39/40; false programs 0/14 | 40/40, 39/40; 0/14 | **met** (the gate passes) |
| B7 | economy | 0 statements, 2 kinds, 1 literal, 2 reserved words, 1 world field, about 4 user-facing codes plus IR forms | 0, 2, 1, 2, 1; 5 user-facing (`E105`, `E518`, `S010`, `S237`, `S238`), the ROS 2 refusal `S510` and `I126` | | **met** |
| B8 | time replaced silently, full prompt | ≤ 1/30 | 0/30 | 0/30 | **met** |

## Notes

- **B4, grammar-only.** For `i03_timeout_too_short` (a 10-minute timeout for a 90-minute wait), Haiku wrote a
  program twice and asked to clarify once. Both programs **failed in the run** (`S262`, the wait's `fail`), so
  neither is a silent success. B4 does not say which prompt it covers; its siblings B1, B5 and B8 are measured on
  the full prompt, and so is B4 here. The owner decides.
- **Grammar-only programs (B2, for information):** the misses are `k01` (Sonnet's `object_exists` goal),
  `k05_open_hours_window` (the window written with one bound only) and `k07_time_and_open`.
- **K3 (a time that has passed)** caught no answer in this run: no model wrote a wait for a past time.

## Verdict

**ACCEPTED by the owner, 2026-11-17.** Every bound is met on the full prompt. B4 is read on the full prompt, as its
siblings are; on the grammar-only prompt Haiku had 12/15, with no silent success. **Paid cost of I3:** baseline
$0.75, acceptance $1.12.
