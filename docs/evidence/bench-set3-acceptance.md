# Benchmark set 3: RoboLang 1.4 acceptance (2026-10-29)

The acceptance run of decision [0027](../../spec/decisions/0027-bounded-branching.md), section 3, after N4 PRs
1–3 (#70, #72, #73). It ran on `main` ([Actions run 40974978677](https://github.com/rokorobot/RoboLang/actions/runs/40974978677))
with 3 repeats, full and grammar-only prompts, 3 models, effort medium, and cost about $4.10. Raw reports:
[full](bench-set3-acceptance-full-report.md), [minimal](bench-set3-acceptance-minimal-report.md).
Baseline (1.3, same tasks): [bench-set3-baseline.md](bench-set3-baseline.md).

## Against the acceptance criteria

| Criterion (0027 §3, horizon H2) | Result | |
|---|---|---|
| Set 3 goal correctness ≥ 90% (Sonnet) | 84/84 full (100%); 80/84 grammar-only (95%) | met |
| False programs on still-inexpressible tasks ≤ 5%, even grammar-only | full 0/72; grammar-only 2/72 (2.8%) | met |
| No regression on sets 1 and 2 | #73 PR run: gate pass, 0/15 false programs, half_program 0/3 | met |

**1.4 is accepted.**

## Before and after (branching requests, 60 answers per model)

| | Opus full | Sonnet full | Haiku full | Opus minimal | Sonnet minimal | Haiku minimal |
|---|---|---|---|---|---|---|
| Silent half-programs, 1.3 baseline | 6 | 7 | 17 | 29 | 30 | 31 |
| Half-programs, 1.4 | 0 | 0 | 0 | 0 | 1 | 5 |
| Right on every path, 1.4 | 60 | 60 | 60 | 60 | 59 | 53 |

With the reference, every branching answer from every model was right on every path. The 8 conditional-sounding
controls stayed straight-line programs with no false refusals. That includes `c_whether_or_not`, reworded after the
baseline.

## What is left (follow-up, not blocking)

- **`x_nearest_robot` label.** 14 of 18 answers asked a question instead of declining: "the world gives no
  distances, which robot?" That is safe and arguably correct. Either accept `clarify` for this task or give the
  world distances. One grammar-only Sonnet answer guessed "arm1 is nearest" and wrote a program: a real false program.
- **`x_count_and_send` label.** One grammar-only Opus answer counted the 4 known parts with nested `if`s and sent
  the exact count. That is legitimate in 1.4 for a known, fixed set of parts. The task should say the parts are
  not known in advance (new ones arrive), or it should be relabelled.
- **Grammar-only Haiku** still writes a few half-programs (5/60, mostly robot choice) and misses `c_only_if_cell`.
  The reference fixes both, so the reference is the product, as F4 already said.
- **Set 3 is saturated with the reference.** Under strategy rule 5 it becomes a regression set; the next hard set
  should probe what 1.4 cannot yet catch.
