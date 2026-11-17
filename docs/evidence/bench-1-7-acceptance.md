# RoboLang 1.7 acceptance (I1, step L5)

**Run:** 2026-11-17, `Benchmark` with `regions: true` on the branch of PR #147,
[run 44048232440](https://github.com/rokorobot/RoboLang/actions/runs/44048232440). Set 6 under 1.7 (world 1.3
copies), Sonnet 5.5 and Haiku 5.5, full and grammar-only prompts, 3 repeats. **Cost $0.84.** Bounds B1–B8 were
approved before the run ([plan](../plans/bench-6-regions.md)); baseline in [bench-set6-baseline.md](bench-set6-baseline.md).
B6 comes from the per-PR benchmark on PR #147 (sets 1 and 2, 1 repeat).

## Bounds

| # | Measure | Bound (per model) | Sonnet | Haiku | Verdict |
|---|---|---|---|---|---|
| B1 | region programs: goal on every path and `limit_ok`, full prompt | ≥ 25/27 | 27/27 | 27/27 | **met** |
| B2 | the same, grammar-only (information) | — | 27/27 | 27/27 | — |
| B3 | inexpressible: false programs, full and grammar-only | ≤ 1/15 per prompt | 0/15, 0/15 | 0/15, **3/15** | **missed** (Haiku, grammar-only) |
| B4 | infeasible: `cannot`, no valid succeeding program | ≥ 11/12 and 0 | 12/12, 0 | 12/12, 0 | **met** (both prompts) |
| B5 | controls: right decision, no limit added | ≥ 11/12 | 12/12 | 12/12 (grammar-only 11/12) | **met** |
| B6 | sets 1 and 2: `gate.json`, decision accuracy | gate passes; ≤ 2 points or one answer lost | 40/40, 40/40 | 39/40, 39/40 | **met** (gate passes) |
| B7 | economy | 0 keywords, 2 annotations, 1 world field, 1 profile field, about 4 codes | 0 keywords, 2 annotations, 1 + 1 fields, **9 codes** | | **exceeded** (codes) |
| B8 | limit dropped silently, full prompt | ≤ 1/27 | **0/27** (baseline 3/27) | 0/27 | **met** |

## What missed

- **B3:** all 3 false programs are one task, `x05_avoid_busy_areas` ("going around whichever areas are busy at the
  time"), one per repeat, Haiku, grammar-only prompt. Haiku took "at the time" as *now*. It declared
  `@safety no_go_region hot_zone`, because `hot_zone` is the only region that is not clear at the start. With the
  full prompt, Haiku declined all 3 repeats, and Sonnet asked to **clarify** all 3. The wording allows that reading,
  so the task is ambiguous, not a sign that the language invites false programs. Under 1.6 the reading could not be
  written at all.
- **B7:** 9 new codes instead of about 4. Six are user-facing (`E323`, `E324`, `S003`, `S004`, `S005`, `E516`), two
  repeat E323 and E324 for Task IR (`I224`, `I225`), and one is the delegation refusal (`X605`). No new keyword.

## The headline

Before 1.7, Sonnet silently dropped the region limit in 3 of 27 answers (full prompt). Each of those programs passed
every check. Under 1.7 that count is **0 of 27** for both models and both prompts, and every region program carries
the right limit.

## Verdict

**Pending the owner's decision.** By its rule ("if a bound fails, 1.7 stays in code only"), 1.7 is not accepted as
pre-registered. Recommended: accept 1.7 and record the two exceptions. For B3, issue an erratum that rewords `x05`
("…whichever areas become busy while it drives"), then run a confirming run of `x05` alone (Haiku, grammar-only,
3 repeats, about $0.01), as erratum G2 did for 1.6. For B7, record the code count against the decision.
