# RSI-01: plan and status of the first language-evolution cycle

**Status:** **APPROVED by the owner, 2026-11-16** (with the strategy amendment below), extended 2026-11-17 (autonomy,
horizon scan, language phase). Level 3 of the [document map](../LANGUAGE_EVOLUTION.md#document-map): the principles
are in the [language evolution strategy](../LANGUAGE_EVOLUTION.md), the binding rules in
[LANGUAGE_EVOLUTION_RULES.md](../LANGUAGE_EVOLUTION_RULES.md). This page holds the steps and their state.

## Data phase

| Step | What | Cost | State |
|---|---|---|---|
| R1 | Atlas [`bench/atlas.json`](../../bench/atlas.json): gaps seeded from the 39 `inexpressible` benchmark tasks, test that each belongs to one gap; generated plan [humanoid-activities.md](humanoid-activities.md) with priorities and development log | $0 | **Done** 2026-11-16/17. Economy counter still open |
| R2 | First present scan (A1): activities with sources, maturity, forecasts | ~$5–15 | **Done** 2026-11-17: 25 activities, gaps G-PROX, G-PREDICT, G-TEACH ([report](../evidence/rsi-r2-foresight-pass-1.md)); autonomy added by hand: 3 activities, gap G-SELF |
| A2 | First horizon scan: leading signals for far activities, `concept` activities, R2's holes | ~$5–15 | **Done** 2026-11-17: 35 leading signals, 7 `concept` activities, machine tending, weeding and solar installation added ([report](../evidence/rsi-a2-horizon-scan-1.md)) |
| R3 | Future tasks (`bench/tasks_future/`) for the near activities: an attempt that fails with a named diagnostic per claimed gap, a reference for what 1.6 can do; then a model run through the `Benchmark` workflow | ~$2 | Next |
| R4 | Weekly Cloudroom Loop: A1 and A2, atlas PR with a development log entry; monthly a ranked proposal for L1 | ~$1/week | After R3 |
| R5 | Quarterly calibration (loop C): score due forecasts, adjust sources and weights | ~$1/quarter | First forecasts due 2027-03 |

## Language phase, first iteration

Starts after R3 ([strategy section 6](../LANGUAGE_EVOLUTION.md#6-phases)): L1 choose → L2 decision record → L3
pre-register → L4 build → L5 accept → L6 release.

**Candidates** on 2026-11-17, near gaps by priority, to be confirmed or refuted by R3: G-PERCEIVE (conditions on
perceived properties), G-CLOCK (clock readings), G-CHOOSE (choosing objects at run time), G-ACT (new actions),
G-SELF (the robot's own state). G-PATH and G-FORCE are the closest to the H4 safety case. G-LOOP ranks first but is
out of scope (N4); it enters L1 only through its own decision record, with a mission layer above tasks as the
recommended design.

## Strategy amendment (approved 2026-11-16, now in [STRATEGY.md](../STRATEGY.md))

- **Rule 1, added:** a future task, derived from an atlas activity of maturity `pilot` or `deployment`, counts as a
  benchmark task that fails without the feature.
- **D5 replaces D4:** new language work follows the ranked atlas; near gaps needed by the current horizon first.
- **Pillar P1:** the requests of today's robot activities and of those expected within three years.

## History

| Date | Change |
|---|---|
| 2026-11-16 | Proposed; seeded from the benchmark's inexpressible tasks; owner: activities drive the language; approved with the strategy amendment; R1 and R2 started |
| 2026-11-17 | Eight directions, then autonomy as the ninth; generated plan with development log; horizon scan A2 done; language phase L1–L6; strategy and rules split into their own documents |
