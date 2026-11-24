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
| R3 | Future tasks (`bench/tasks_future/`) for the near activities: an attempt that fails with a named diagnostic per claimed gap, a reference for what 1.6 can do; then a model run through the `Benchmark` workflow | ~$2 | **Probes done** 2026-11-17: 25 probes confirm the nine near gaps, 3 claims refuted, 8 controls ([report](../evidence/rsi-r3-gap-verification.md)); model run 2026-11-17: 0/25 false programs (Sonnet 25/25 declines; Haiku 23/25 plus 2 `clarify`), $0.20; the ACT-MFG-001 G-CHOOSE refutation withdrawn; follow-up: teach fixed counts and bounded retries in the reference (F01) |
| R3b | Verify the gaps proposed for I11–I20 against the atlas, with probes for the ones that hold (yardstick 2.2) | $0 | **Done** 2026-11-24: G-TRIGGER and G-DATE near again (needs dropped at L6 of I4), 4 probes; G-SHIFT is G-DATE; G-CATEGORY, G-NEGOTIATE, G-SHARED refuted ([report](../evidence/rsi-r3b-new-gaps.md)); model run 2026-11-24: 8/8 declines on the new probes, 0/15 false programs, $0.47 |
| R4 | Weekly Cloudroom Loop: A1 and A2, atlas PR with a development log entry; monthly a ranked proposal for L1 | ~$1/week | **Running** since 2026-11-17: Cloudroom Loop "Atlas weekly (Mondays 08:00)", Europe/Prague, Sonnet; first test run 2026-11-17 |
| R5 | Quarterly calibration (loop C): score due forecasts, adjust sources and weights | ~$1/quarter | First forecasts due 2027-03 |

## Language phase, first iteration

Starts after R3 ([strategy section 6](../LANGUAGE_EVOLUTION.md#6-phases)): L1 choose → L2 decision record → L3
pre-register → L4 build → L5 accept → L6 release. The iterations and their order: [language roadmap](../LANGUAGE_ROADMAP.md).

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
