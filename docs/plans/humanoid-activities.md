# Humanoid activities and the direction of RoboLang

**Generated** from [`bench/atlas.json`](../../bench/atlas.json) by `python bench/atlas_report.py`; do not edit by
hand. Strategy: [language evolution](../LANGUAGE_EVOLUTION.md); rules: [LE-1 to LE-15](../LANGUAGE_EVOLUTION_RULES.md).
Status: [RSI-01](rsi-01.md). Sub-activities are added to the atlas as the foresight loop finds them; the
priorities below follow from them.

<p align="center"><img src="capability-directions.svg" width="420"
alt="The directions of the RoboLang Capability Atlas (eight in the diagram; autonomy, the ninth, added later)"/></p>

**Priorities.** A sub-activity's priority is `p × nearness` (`p`: its forecast's probability; nearness: now 1,
1–3y 0.7, 3–7y 0.4, 7y+ 0.1). A direction's priority is the sum of its sub-activities' priorities, each split
evenly over the directions it names. A language gap's priority is the sum over the sub-activities that need it;
it is **near** when one of them is a pilot or deployment within three years, and only near gaps are eligible
for new syntax (strategy rule 1, D5). Out-of-scope gaps are listed but never proposed without a decision record.

## Directions by priority

*Priority* comes from the sub-activities (what robots will do). *Language need* is the sum of the priorities
of the direction's open gaps (what RoboLang lacks there); the two differ when sub-activities of one direction
need gaps of another, e.g. tote handling (action) needs clock readings (reasoning). *Future need* is the sum of
`p` over the direction's far sub-activities (concept or research, or 3+ years away): where the language is
headed, and what a near design must not rule out. It never gives syntax by itself.

| # | Direction | Priority | Language need | Future need | Sub-activities | Near gaps |
|---|---|---|---|---|---|---|
| 1 | action | 4.52 | 0.00 | 1.01 | 21 | — |
| 2 | perception | 2.78 | 5.23 | 0.45 | 11 | G-CHOOSE |
| 3 | cooperation | 1.07 | 1.00 | 1.81 | 13 | G-CONC |
| 4 | communication | 0.95 | 0.00 | 0.52 | 6 | — |
| 5 | learning | 0.75 | 0.65 | 0.70 | 6 | — |
| 6 | reasoning | 0.68 | 6.43 | 0.88 | 7 | G-TRIGGER, G-DATE, G-CALC |
| 7 | autonomy | 0.67 | 1.50 | 0.38 | 4 | G-SELF |
| 8 | safety | 0.49 | 1.95 | 0.74 | 5 | — |
| 9 | prediction | 0.25 | 0.50 | 0.68 | 4 | — |

## Suggested order of language work

Near gaps in the order that unlocks the most: *Unlocked* is the summed priority of the sub-activities
whose needs are all closed once this gap and those above it are. Gaps the current horizon needs come
first (strategy D5). An `out` gap enters only through its own decision record (LE-9). The roadmap
([LANGUAGE_ROADMAP.md](../LANGUAGE_ROADMAP.md)) turns this order into versions.

| # | Gap | Direction | Scope | Priority | Unlocked | Why here |
|---|---|---|---|---|---|---|
| 1 | `G-CHOOSE` | perception | open | 5.23 | 3.50 | unlocks the most |
| 2 | `G-SELF` | autonomy | open | 1.50 | 5.00 | unlocks the most |
| 3 | `G-TRIGGER` | reasoning | open | 3.11 | 6.35 | unlocks the most |
| 4 | `G-CALC` | reasoning | open | 1.55 | 7.52 | unlocks the most |
| 5 | `G-DATE` | reasoning | open | 1.76 | 8.62 | unlocks the most |
| 6 | `G-CONC` | cooperation | open | 1.00 | 9.62 | unlocks the most |

## Future direction

Far sub-activities by `p`: what is coming, and the signals that point to it. Leading signals: research,
roadmap, funding, standard, demand, forecast.

| Sub-activity | Directions | Maturity | Horizon | p | Needs | Leading signals |
|---|---|---|---|---|---|---|
| `ACT-SAFE-001` Fence-free humanoid work beside people | safety, cooperation | demo | 3-7y | 0.65 | G-PROX, G-FORCE | [standard](https://www.din.de/en/getting-involved/standards-committees/nam/projects/wdc-proj:din21:400666335) [roadmap](https://letsdatascience.com/news/agility-robotics-previews-cage-free-digit-v5-847ce6a1) |
| `ACT-AGR-001` Greenhouse fruit harvesting | perception, action | pilot | 3-7y | 0.60 | G-CHOOSE, G-FORCE, G-TRIGGER, G-DATE | [roadmap](https://www.freshplaza.com/europe/article/9842754/japanese-agri-tech-startup-puts-cherry-tomato-harvesting-robot-into-routine-production-use/) [funding](https://agfundernews.com/🎥-carbon-robotics-100m-revenue-new-machine-and-a-path-to-ipo) |
| `ACT-AUTO-003` Self-diagnosis, safe degradation and service requests | autonomy, safety | research | 3-7y | 0.45 | G-SELF | [research](https://arxiv.org/abs/2602.01515) [roadmap](https://en.edaily.co.kr/news/eda202606185436/) |
| `ACT-PRED-002` Checking a plan with a world model before acting | prediction, learning | research | 3-7y | 0.45 | G-PREDICT, G-CHOOSE | [research](https://arxiv.org/abs/2606.13672) |
| `ACT-HOME-002` General household chores by a home humanoid | action, reasoning | demo | 3-7y | 0.40 | G-CHOOSE | [research](https://arxiv.org/abs/2604.15483) [forecast](https://humanoid.guide/goldman-raises-2035-humanoid-robot-forecast-to-6-5-million/) |
| `ACT-HRC-002` Following ambiguous and corrective instructions during shared work | reasoning, cooperation | research | 3-7y | 0.40 | G-CHOOSE | [research](https://arxiv.org/abs/2602.14551) |
| `ACT-HRC-003` Handing a stuck job to a remote operator and taking it back | cooperation, communication | concept | 3-7y | 0.40 | G-SELF | [roadmap](https://roadtovr.com/helper-robot-neo-vr-telepresence/) [roadmap](https://sacra.com/chat/h/93dc70de-7775-4bc2-b477-ce4cd741da03/) [research](https://arxiv.org/abs/2605.00416) |
| `ACT-PRED-001` Anticipating a coworker's next movement | prediction, cooperation | research | 3-7y | 0.40 | G-PREDICT, G-PROX | [research](https://arxiv.org/abs/2604.03065) |
| `ACT-HRC-001` Task-oriented object handover to a person | cooperation, communication | research | 3-7y | 0.35 | G-CHOOSE | [research](https://arxiv.org/abs/2602.05760v1) |
| `ACT-LRN-001` Learning a new task from half an hour of a worker's video | learning, action | concept | 3-7y | 0.35 | G-TEACH | [research](https://arxiv.org/abs/2605.24934) [funding](https://dealroom.co/news/127095-robot-ai-startup-physical-intelligence-seeks-1b-at-11b-valuation/) |
| `ACT-MRC-001` Mixed robot teams on one workflow | cooperation, reasoning | demo | 3-7y | 0.35 | G-CONC, G-CHOOSE | [roadmap](https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/) [standard](https://www.therobotreport.com/new-iso-interoperability-standard-covers-collaboration-among-mobile-robots-from-multiple-vendors/) |
| `ACT-PRED-003` Forecasting its own failure and asking before it happens | prediction, reasoning | concept | 3-7y | 0.35 | G-PREDICT | [research](https://arxiv.org/abs/2604.16677) [roadmap](https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/) |
| `ACT-AUTO-004` Mid-shift relief: a low-battery robot hands its job to a peer | autonomy, cooperation | concept | 3-7y | 0.30 | G-SELF, G-CHOOSE | [research](https://arxiv.org/abs/2603.22731) [roadmap](https://newatlas.com/ai-humanoids/walker-s2-humanoid-robot-swaps-batteries) [standard](https://www.therobotreport.com/new-iso-interoperability-standard-covers-collaboration-among-mobile-robots-from-multiple-vendors/) |
| `ACT-ELD-001` Companion robot in dementia and memory care | communication, perception | pilot | 3-7y | 0.30 | — | [demand](https://e27.co/when-the-carer-has-dementia-too-japan-turns-to-physical-ai-to-rescue-eldercare-20261007/) [roadmap](https://e27.co/when-the-carer-has-dementia-too-japan-turns-to-physical-ai-to-rescue-eldercare-20261007/) |
| `ACT-HOME-001` Laundry folding in the home | learning, action | research | 3-7y | 0.30 | G-CHOOSE, G-CALC | [research](https://arxiv.org/abs/2604.15483) [funding](https://dealroom.co/news/127095-robot-ai-startup-physical-intelligence-seeks-1b-at-11b-valuation/) |
| `ACT-LRN-002` One taught skill, any robot on the site | learning, cooperation | concept | 3-7y | 0.30 | G-TEACH, G-CHOOSE | [roadmap](https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/) [research](https://arxiv.org/abs/2605.24934) |
| `ACT-SAFE-002` A safety layer under a learned policy that vetoes unsafe moves | safety, action | concept | 3-7y | 0.30 | G-PROX, G-FORCE | [research](https://arxiv.org/abs/2606.09749) [standard](https://www.din.de/en/getting-involved/standards-committees/nam/projects/wdc-proj:din21:400666335) [roadmap](https://letsdatascience.com/news/agility-robotics-previews-cage-free-digit-v5-847ce6a1) |
| `ACT-ASM-001` Robot-team structural assembly with symbolic replanning | reasoning, cooperation | research | 7y+ | 0.25 | G-CONC | [research](https://arxiv.org/abs/2601.00978v1) [demand](https://ecmweb.com/construction/news/55344103/construction-industry-must-attract-349000-workers-in-2026) |
| `ACT-PRED-004` Helping before being asked: acting on a predicted need | prediction, cooperation | concept | 7y+ | 0.15 | G-PREDICT | [research](https://arxiv.org/abs/2603.23950) [demand](https://e27.co/when-the-carer-has-dementia-too-japan-turns-to-physical-ai-to-rescue-eldercare-20261007/) |
| `ACT-CARE-001` Physical help with standing and transfers in care homes | safety, action, cooperation | concept | 7y+ | 0.12 | G-FORCE, G-PROX | [demand](https://e27.co/when-the-carer-has-dementia-too-japan-turns-to-physical-ai-to-rescue-eldercare-20261007/) [roadmap](https://e27.co/when-the-carer-has-dementia-too-japan-turns-to-physical-ai-to-rescue-eldercare-20261007/) [research](https://arxiv.org/abs/2508.19816) |

## Development log

Newest first. Ranking: the directions in order of priority, with the change of rank since the entry
before.

| Date | Note | Added | Removed | Changed | Ranking |
|---|---|---|---|---|---|
| 2026-11-24 | R3b: G-TRIGGER and G-DATE near (needs dropped when G-LOOP closed, now claimed again with probes); G-CATEGORY, G-NEGOTIATE and G-SHARED refuted as new gaps | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-18 | L6 of I6: G-PERCEIVE closed by RoboLang 2.2 measured kinds | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-18 | I6 L4: RoboLang 2.2 built and taught | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-18 | I6 L3: set 11 kinds pre-registered (24 tasks) | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-18 | L6 of I5: G-MSG closed by RoboLang 2.1 messages | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-18 | I5 L5: 2.1 accepted; x05 declined again (erratum N3, G-EVENTGOAL out) | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-18 | I5 L4: RoboLang 2.1 built and taught | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-18 | I5 L3 done: set 10 frozen; bounds and clarifications approved; G-DELIVERY out (C13) | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-18 | I5 L3: set 10 messages pre-registered (24 tasks) | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-18 | L6 of I4: G-LOOP closed by RoboLang 2.0 missions; G-UNBOUNDED (out, 0041 M2/M7) split off | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-18 | I4 L5: 2.0 accepted with one exception; G-OBSERVE opened | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-18 | I4 L4: RoboLang 2.0 built and taught | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-18 | I4 L3: set 9 missions pre-registered (26 tasks) | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | L6 of I3: G-CLOCK closed by RoboLang 1.9 clock readings | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | I3 L4: RoboLang 1.9 built and taught | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | I3 L3 done: set 8 frozen; bounds B1-B8 and clarifications K1-K11 approved; G-DATE opened (K11) | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | I3 L3: set 8 clock pre-registered (24 tasks) | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | L6 of I2: G-ACT closed by RoboLang 1.8 declared actions | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | I2 L4: RoboLang 1.8 built and taught | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | I2 L3 done: set 7 frozen; bounds B1-B8 and clarifications C1-C15 approved; x02 to G-SELF (C9) | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | I2 L3: set 7 actions pre-registered (24 tasks) | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | 0040 approved (I2 G-ACT); 0041 approved: G-LOOP reopened with the mission layer (LE-9) | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | L6 of I1: G-PATH closed by RoboLang 1.7; G-STEPLIMIT and G-WORKSPACE opened (0039 C4) | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | I1 L4: RoboLang 1.7 built and taught | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | 0039 clarifications C1-C7 (owner); x05 moved from G-CHOOSE to G-PROX | — | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | Weekly A1/A2: ACT-MFG-004 (Hexagon AEON at BMW Leipzig, demo) and ACT-HRC-003 (remote takeover and handback, concept); no maturity or horizon changes, no gap changes | ACT-HRC-003, ACT-MFG-004 | — | — | 1. action 4.52 · 2. perception 2.78 · 3. cooperation 1.07 · 4. communication 0.95 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | I1 L3: set 6 regions pre-registered (22 tasks) | — | — | — | 1. action 4.17 · 2. perception 2.78 · 3. cooperation 0.99 · 4. communication 0.87 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | R3 model run: 0/25 false programs (Sonnet 25/25 declines, Haiku 23/25 plus 2 clarify); ACT-MFG-001 G-CHOOSE refutation withdrawn | — | — | — | 1. action 4.17 · 2. perception 2.78 · 3. cooperation 0.99 · 4. communication 0.87 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | R3: 25 probes confirm G-CLOCK, G-CALC, G-MSG, G-ACT, G-PATH, G-CONC, G-CHOOSE, G-PERCEIVE, G-SELF; refuted: G-PERCEIVE for ACT-HOS-001 and ACT-SOL-001, G-CHOOSE for ACT-MFG-001 | — | — | — | 1. action 4.17 · 2. perception 2.78 · 3. cooperation 0.99 · 4. communication 0.87 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | Orchestrator spot-check of A2: ACT-MFG-003 pilot announced, not running; ACT-SOL-001 basis corrected | — | — | ACT-MFG-003: pilot 1-3y → demo 1-3y | 1. action 4.17 · 2. perception 2.78 · 3. cooperation 0.99 · 4. communication 0.87 · 5. learning 0.75 · 6. reasoning 0.68 · 7. autonomy 0.67 · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | Horizon scan 1 (A2): 35 leading signals (18 on 11 far activities, 17 on the new ones), 7 concept activities, 3 R2 holes filled (machine tending, weeding, solar); no changes to existing activities, no new gaps | ACT-AGR-002, ACT-AUTO-004, ACT-CARE-001, ACT-LRN-001, ACT-LRN-002, ACT-MFG-003, ACT-PRED-003, ACT-PRED-004, ACT-SAFE-002, ACT-SOL-001 | — | — | 1. action 4.17 · 2. perception 2.78 · 3. cooperation 0.99 (+1) · 4. communication 0.87 (-1) · 5. learning 0.75 · 6. reasoning 0.68 (+1) · 7. autonomy 0.67 (-1) · 8. safety 0.49 · 9. prediction 0.25 |
| 2026-11-17 | Ninth direction autonomy (owner): 3 sub-activities, new gap G-SELF | ACT-AUTO-001, ACT-AUTO-002, ACT-AUTO-003 | — | — | 1. action 3.43 · 2. perception 2.33 · 3. communication 0.87 · 4. cooperation 0.86 · 5. learning 0.62 · 6. autonomy 0.61 (+3) · 7. reasoning 0.45 (-1) · 8. safety 0.43 (-1) · 9. prediction 0.17 (-1) |
| 2026-11-17 | Foresight pass 1 (R2): 25 sub-activities, 3 new gaps (G-PROX, G-PREDICT, G-TEACH) | ACT-AGR-001, ACT-ASM-001, ACT-CLN-001, ACT-CON-001, ACT-ELD-001, ACT-EV-001, ACT-HLTH-001, ACT-HOME-001, ACT-HOME-002, ACT-HOS-001, ACT-HRC-001, ACT-HRC-002, ACT-INSP-001, ACT-INSP-002, ACT-LOG-001, ACT-LOG-002, ACT-MFG-001, ACT-MFG-002, ACT-MRC-001, ACT-MRC-002, ACT-PRED-001, ACT-PRED-002, ACT-RET-001, ACT-SAFE-001, ACT-SEC-001 | — | — | 1. action 2.91 · 2. perception 2.33 · 3. communication 0.87 · 4. cooperation 0.86 · 5. learning 0.62 · 6. reasoning 0.45 · 7. safety 0.34 · 8. prediction 0.17 · others 0 |

## Action

| Sub-activity | Maturity | Horizon | p | Priority | Needs | Sources |
|---|---|---|---|---|---|---|
| `ACT-AUTO-001` Self-charging on long inspection missions | deployment | now | 0.65 | 0.65 | G-SELF, G-CHOOSE | [1](https://bostondynamics.com/blog/making-every-step-count-with-spot/) |
| `ACT-LOG-001` Humanoid tote handling in fulfilment centres | deployment | now | 0.60 | 0.60 | G-CALC, G-CHOOSE | [1](https://www.dcvelocity.com/material-handling/robotics/gxo-to-expand-use-of-humanoid-robots-from-agility) |
| `ACT-CLN-001` Scheduled autonomous floor cleaning | deployment | now | 0.55 | 0.55 | G-TRIGGER, G-DATE | [1](https://www.roboticstomorrow.com/news/2026/04/07/brain-corp-launches-brainos-clean-20-with-selfpath-ai-advancing-adaptive-autonomy-for-tennant-company-floor-cleaning-robots/26371/) |
| `ACT-HLTH-001` Hospital delivery and supply runs for nurses and pharmacy | deployment | now | 0.55 | 0.55 | G-TRIGGER | [1](https://www.therobotreport.com/five-years-operation-shape-diligent-robotics-rollout-moxi-2-0/) |
| `ACT-SOL-001` Robotic solar module installation | deployment | now | 0.50 | 0.50 | — | [1](https://www.eweek.com/news/robots-install-100mw-solar-power-maximo/) |
| `ACT-MFG-002` Humanoid-to-tugger handoff of totes in a car plant | pilot | 1-3y | 0.65 | 0.45 | G-CONC, G-CHOOSE | [1](https://agilityrobotics.com/content/agility-robotics-announces-commercial-agreement-with-toyota-motor-manufacturing-canada) [2](https://www.therobotreport.com/bmw-group-deploys-figure-03-humanoid-after-tests-previous-version/) |
| `ACT-CON-001` Construction-site layout printing | deployment | now | 0.45 | 0.45 | — | [1](https://www.dustyrobotics.com/discover/construction-robot) |
| `ACT-HOS-001` Fry-station cooking in restaurant kitchens | deployment | now | 0.45 | 0.45 | — | [1](https://www.fastcasual.com/news/miso-robotics-flippy-fry-station-robot-surpasses-5m-baskets) |
| `ACT-LOG-002` Autonomous trailer and container unloading | deployment | now | 0.45 | 0.45 | G-CALC | [1](https://www.therobotreport.com/boston-dynamics-built-stretch-gen-2-for-reliable-consistent-performance/) |
| `ACT-MFG-001` Parts sequencing for the assembly line | pilot | 1-3y | 0.60 | 0.42 | G-CHOOSE | [1](https://www.therobotreport.com/bmw-group-deploys-figure-03-humanoid-after-tests-previous-version/) |
| `ACT-AGR-002` Autonomous laser weeding between crop rows | deployment | now | 0.40 | 0.40 | G-CHOOSE | [1](https://agfundernews.com/🎥-carbon-robotics-100m-revenue-new-machine-and-a-path-to-ipo) |
| `ACT-AUTO-002` Battery swapping for whole-shift humanoid work | demo | 1-3y | 0.55 | 0.39 | G-SELF | [1](https://newatlas.com/ai-humanoids/walker-s2-humanoid-robot-swaps-batteries) |
| `ACT-EV-001` Robotic EV charging in garages and depots | pilot | 1-3y | 0.55 | 0.39 | G-CHOOSE | [1](https://electrek.co/2026/02/23/china-overhead-rail-ev-charging-robots-parking-garages/) |
| `ACT-MFG-004` Humanoid assembly and component work in a car plant | demo | 1-3y | 0.50 | 0.35 | G-FORCE | [1](https://robotics.hexagon.com/?p=720) [2](https://www.artificialintelligence-news.com/news/bmw-humanoid-robots-manufacturing-europe-leipzig/) |
| `ACT-MFG-003` Humanoid machine tending at a machine builder's customer site | demo | 1-3y | 0.45 | 0.32 | — | [1](https://roboticsandautomationnews.com/2026/05/15/hexagon-and-fill-maschinenbau-partner-to-advance-manufacturing-autonomy-using-humanoids/101578/) |
| `ACT-AGR-001` Greenhouse fruit harvesting | pilot | 3-7y | 0.60 | 0.24 | G-CHOOSE, G-FORCE, G-TRIGGER, G-DATE | [1](https://www.freshplaza.com/europe/article/9842754/japanese-agri-tech-startup-puts-cherry-tomato-harvesting-robot-into-routine-production-use/) |
| `ACT-HOME-002` General household chores by a home humanoid | demo | 3-7y | 0.40 | 0.16 | G-CHOOSE | [1](https://heise.de/-11287205) |
| `ACT-LRN-001` Learning a new task from half an hour of a worker's video | concept | 3-7y | 0.35 | 0.14 | G-TEACH | [1](https://arxiv.org/abs/2605.24934) [2](https://dealroom.co/news/127095-robot-ai-startup-physical-intelligence-seeks-1b-at-11b-valuation/) |
| `ACT-HOME-001` Laundry folding in the home | research | 3-7y | 0.30 | 0.12 | G-CHOOSE, G-CALC | [1](https://arxiv.org/abs/2511.14759) |
| `ACT-SAFE-002` A safety layer under a learned policy that vetoes unsafe moves | concept | 3-7y | 0.30 | 0.12 | G-PROX, G-FORCE | [1](https://arxiv.org/abs/2606.09749) [2](https://www.din.de/en/getting-involved/standards-committees/nam/projects/wdc-proj:din21:400666335) [3](https://letsdatascience.com/news/agility-robotics-previews-cage-free-digit-v5-847ce6a1) |
| `ACT-CARE-001` Physical help with standing and transfers in care homes | concept | 7y+ | 0.12 | 0.01 | G-FORCE, G-PROX | [1](https://e27.co/when-the-carer-has-dementia-too-japan-turns-to-physical-ai-to-rescue-eldercare-20261007/) [2](https://arxiv.org/abs/2508.19816) |

| Language gap | Scope | Priority | Near | Benchmark tasks |
|---|---|---|---|---|
| `G-ACT` Actions beyond move, grasp, place, inspect, wait, communicate, measure: open, charge, set a device | closed | 0.00 | — | 0 |

## Perception

| Sub-activity | Maturity | Horizon | p | Priority | Needs | Sources |
|---|---|---|---|---|---|---|
| `ACT-INSP-001` Autonomous gauge and thermal inspection rounds | demo | now | 0.80 | 0.80 | G-TRIGGER | [1](https://www.therobotreport.com/boston-dynamics-releases-spot-and-orbit-5-1-with-new-spot-cam/) |
| `ACT-RET-001` Store shelf scanning and out-of-stock reporting | deployment | now | 0.55 | 0.55 | G-CHOOSE, G-TRIGGER, G-DATE | [1](https://theshelbyreport.com/2026/09/21/simbe-surpasses-3000-units-in-shelf-intelligence-fleet/) |
| `ACT-SOL-001` Robotic solar module installation | deployment | now | 0.50 | 0.50 | — | [1](https://www.eweek.com/news/robots-install-100mw-solar-power-maximo/) |
| `ACT-CON-001` Construction-site layout printing | deployment | now | 0.45 | 0.45 | — | [1](https://www.dustyrobotics.com/discover/construction-robot) |
| `ACT-HOS-001` Fry-station cooking in restaurant kitchens | deployment | now | 0.45 | 0.45 | — | [1](https://www.fastcasual.com/news/miso-robotics-flippy-fry-station-robot-surpasses-5m-baskets) |
| `ACT-LOG-002` Autonomous trailer and container unloading | deployment | now | 0.45 | 0.45 | G-CALC | [1](https://www.therobotreport.com/boston-dynamics-built-stretch-gen-2-for-reliable-consistent-performance/) |
| `ACT-SEC-001` Security patrol that reacts to people | demo | 1-3y | 0.60 | 0.42 | G-PROX, G-TRIGGER, G-DATE | [1](https://www.therobotreport.com/boston-dynamics-releases-spot-and-orbit-5-1-with-new-spot-cam/) |
| `ACT-AGR-002` Autonomous laser weeding between crop rows | deployment | now | 0.40 | 0.40 | G-CHOOSE | [1](https://agfundernews.com/🎥-carbon-robotics-100m-revenue-new-machine-and-a-path-to-ipo) |
| `ACT-INSP-002` Inspection that learns the facility | demo | 1-3y | 0.55 | 0.39 | G-TEACH, G-CALC | [1](https://bostondynamics.com/blog/aivi-learning-now-powered-google-gemini-robotics/) |
| `ACT-AGR-001` Greenhouse fruit harvesting | pilot | 3-7y | 0.60 | 0.24 | G-CHOOSE, G-FORCE, G-TRIGGER, G-DATE | [1](https://www.freshplaza.com/europe/article/9842754/japanese-agri-tech-startup-puts-cherry-tomato-harvesting-robot-into-routine-production-use/) |
| `ACT-ELD-001` Companion robot in dementia and memory care | pilot | 3-7y | 0.30 | 0.12 | — | [1](https://health.ucdavis.edu/medical-center/news/headlines/uc-davis-launches-first-long-term-us-study-led-by-nurses-on-humanoid-robots-in-dementia-care/2026/05) |

| Language gap | Scope | Priority | Near | Benchmark tasks |
|---|---|---|---|---|
| `G-CHOOSE` Choosing objects or robots at run time | open | 5.23 | yes | 8 |
| `G-OBSERVE` Tasks that only observe (inspect, scan, wait) and ask for no end state, so a goal must not be invented | open | 0.00 | — | 0 |
| `G-PERCEIVE` Conditions on perceived properties (dirty, damaged) | closed | 0.00 | — | 0 |
| `G-QUALITY` Judgements of appearance or quality that no sensor kind declares (worn, dirty, damaged, burnt) | open | 0.00 | — | 1 |

## Cooperation

| Sub-activity | Maturity | Horizon | p | Priority | Needs | Sources |
|---|---|---|---|---|---|---|
| `ACT-MFG-002` Humanoid-to-tugger handoff of totes in a car plant | pilot | 1-3y | 0.65 | 0.45 | G-CONC, G-CHOOSE | [1](https://agilityrobotics.com/content/agility-robotics-announces-commercial-agreement-with-toyota-motor-manufacturing-canada) [2](https://www.therobotreport.com/bmw-group-deploys-figure-03-humanoid-after-tests-previous-version/) |
| `ACT-MRC-002` Multi-vendor fleet interoperability | demo | 1-3y | 0.55 | 0.39 | G-CHOOSE, G-CONC | [1](https://www.therobotreport.com/new-iso-interoperability-standard-covers-collaboration-among-mobile-robots-from-multiple-vendors/) |
| `ACT-SAFE-001` Fence-free humanoid work beside people | demo | 3-7y | 0.65 | 0.26 | G-PROX, G-FORCE | [1](https://letsdatascience.com/news/agility-robotics-previews-cage-free-digit-v5-847ce6a1) [2](https://www.i-scoop.eu/iso-25785-1-explained-and-what-it-means-for-humanoid-robot-safety/) |
| `ACT-HRC-002` Following ambiguous and corrective instructions during shared work | research | 3-7y | 0.40 | 0.16 | G-CHOOSE | [1](https://arxiv.org/abs/2602.14551) |
| `ACT-HRC-003` Handing a stuck job to a remote operator and taking it back | concept | 3-7y | 0.40 | 0.16 | G-SELF | [1](https://roadtovr.com/helper-robot-neo-vr-telepresence/) [2](https://sacra.com/chat/h/93dc70de-7775-4bc2-b477-ce4cd741da03/) [3](https://arxiv.org/abs/2605.00416) |
| `ACT-PRED-001` Anticipating a coworker's next movement | research | 3-7y | 0.40 | 0.16 | G-PREDICT, G-PROX | [1](https://arxiv.org/abs/2604.03065) |
| `ACT-HRC-001` Task-oriented object handover to a person | research | 3-7y | 0.35 | 0.14 | G-CHOOSE | [1](https://arxiv.org/abs/2602.05760v1) |
| `ACT-MRC-001` Mixed robot teams on one workflow | demo | 3-7y | 0.35 | 0.14 | G-CONC, G-CHOOSE | [1](https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/) |
| `ACT-AUTO-004` Mid-shift relief: a low-battery robot hands its job to a peer | concept | 3-7y | 0.30 | 0.12 | G-SELF, G-CHOOSE | [1](https://arxiv.org/abs/2603.22731) [2](https://newatlas.com/ai-humanoids/walker-s2-humanoid-robot-swaps-batteries) [3](https://www.therobotreport.com/new-iso-interoperability-standard-covers-collaboration-among-mobile-robots-from-multiple-vendors/) |
| `ACT-LRN-002` One taught skill, any robot on the site | concept | 3-7y | 0.30 | 0.12 | G-TEACH, G-CHOOSE | [1](https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/) [2](https://arxiv.org/abs/2605.24934) |
| `ACT-ASM-001` Robot-team structural assembly with symbolic replanning | research | 7y+ | 0.25 | 0.03 | G-CONC | [1](https://arxiv.org/abs/2601.00978v1) |
| `ACT-PRED-004` Helping before being asked: acting on a predicted need | concept | 7y+ | 0.15 | 0.01 | G-PREDICT | [1](https://arxiv.org/abs/2603.23950) [2](https://e27.co/when-the-carer-has-dementia-too-japan-turns-to-physical-ai-to-rescue-eldercare-20261007/) |
| `ACT-CARE-001` Physical help with standing and transfers in care homes | concept | 7y+ | 0.12 | 0.01 | G-FORCE, G-PROX | [1](https://e27.co/when-the-carer-has-dementia-too-japan-turns-to-physical-ai-to-rescue-eldercare-20261007/) [2](https://arxiv.org/abs/2508.19816) |

| Language gap | Scope | Priority | Near | Benchmark tasks |
|---|---|---|---|---|
| `G-CONC` Several robots acting at the same time | open | 1.00 | yes | 4 |

## Communication

| Sub-activity | Maturity | Horizon | p | Priority | Needs | Sources |
|---|---|---|---|---|---|---|
| `ACT-HLTH-001` Hospital delivery and supply runs for nurses and pharmacy | deployment | now | 0.55 | 0.55 | G-TRIGGER | [1](https://www.therobotreport.com/five-years-operation-shape-diligent-robotics-rollout-moxi-2-0/) |
| `ACT-RET-001` Store shelf scanning and out-of-stock reporting | deployment | now | 0.55 | 0.55 | G-CHOOSE, G-TRIGGER, G-DATE | [1](https://theshelbyreport.com/2026/09/21/simbe-surpasses-3000-units-in-shelf-intelligence-fleet/) |
| `ACT-MRC-002` Multi-vendor fleet interoperability | demo | 1-3y | 0.55 | 0.39 | G-CHOOSE, G-CONC | [1](https://www.therobotreport.com/new-iso-interoperability-standard-covers-collaboration-among-mobile-robots-from-multiple-vendors/) |
| `ACT-HRC-003` Handing a stuck job to a remote operator and taking it back | concept | 3-7y | 0.40 | 0.16 | G-SELF | [1](https://roadtovr.com/helper-robot-neo-vr-telepresence/) [2](https://sacra.com/chat/h/93dc70de-7775-4bc2-b477-ce4cd741da03/) [3](https://arxiv.org/abs/2605.00416) |
| `ACT-HRC-001` Task-oriented object handover to a person | research | 3-7y | 0.35 | 0.14 | G-CHOOSE | [1](https://arxiv.org/abs/2602.05760v1) |
| `ACT-ELD-001` Companion robot in dementia and memory care | pilot | 3-7y | 0.30 | 0.12 | — | [1](https://health.ucdavis.edu/medical-center/news/headlines/uc-davis-launches-first-long-term-us-study-led-by-nurses-on-humanoid-robots-in-dementia-care/2026/05) |

| Language gap | Scope | Priority | Near | Benchmark tasks |
|---|---|---|---|---|
| `G-DELIVERY` Confirming that a message the robot sent was delivered | out | 0.00 | — | 2 |
| `G-EVENTGOAL` An event, such as a received message, stated as an end state of a task | out | 0.00 | — | 1 |
| `G-MSG` Waiting for an incoming message, or confirming delivery | closed | 0.00 | — | 0 |

## Learning

| Sub-activity | Maturity | Horizon | p | Priority | Needs | Sources |
|---|---|---|---|---|---|---|
| `ACT-CLN-001` Scheduled autonomous floor cleaning | deployment | now | 0.55 | 0.55 | G-TRIGGER, G-DATE | [1](https://www.roboticstomorrow.com/news/2026/04/07/brain-corp-launches-brainos-clean-20-with-selfpath-ai-advancing-adaptive-autonomy-for-tennant-company-floor-cleaning-robots/26371/) |
| `ACT-INSP-002` Inspection that learns the facility | demo | 1-3y | 0.55 | 0.39 | G-TEACH, G-CALC | [1](https://bostondynamics.com/blog/aivi-learning-now-powered-google-gemini-robotics/) |
| `ACT-PRED-002` Checking a plan with a world model before acting | research | 3-7y | 0.45 | 0.18 | G-PREDICT, G-CHOOSE | [1](https://arxiv.org/abs/2606.13672) |
| `ACT-LRN-001` Learning a new task from half an hour of a worker's video | concept | 3-7y | 0.35 | 0.14 | G-TEACH | [1](https://arxiv.org/abs/2605.24934) [2](https://dealroom.co/news/127095-robot-ai-startup-physical-intelligence-seeks-1b-at-11b-valuation/) |
| `ACT-HOME-001` Laundry folding in the home | research | 3-7y | 0.30 | 0.12 | G-CHOOSE, G-CALC | [1](https://arxiv.org/abs/2511.14759) |
| `ACT-LRN-002` One taught skill, any robot on the site | concept | 3-7y | 0.30 | 0.12 | G-TEACH, G-CHOOSE | [1](https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/) [2](https://arxiv.org/abs/2605.24934) |

| Language gap | Scope | Priority | Near | Benchmark tasks |
|---|---|---|---|---|
| `G-TEACH` Teaching the robot a new skill, object or route by demonstration or correction, then using it by name | open | 0.65 | — | 1 |

## Reasoning

| Sub-activity | Maturity | Horizon | p | Priority | Needs | Sources |
|---|---|---|---|---|---|---|
| `ACT-MFG-001` Parts sequencing for the assembly line | pilot | 1-3y | 0.60 | 0.42 | G-CHOOSE | [1](https://www.therobotreport.com/bmw-group-deploys-figure-03-humanoid-after-tests-previous-version/) |
| `ACT-MFG-003` Humanoid machine tending at a machine builder's customer site | demo | 1-3y | 0.45 | 0.32 | — | [1](https://roboticsandautomationnews.com/2026/05/15/hexagon-and-fill-maschinenbau-partner-to-advance-manufacturing-autonomy-using-humanoids/101578/) |
| `ACT-HOME-002` General household chores by a home humanoid | demo | 3-7y | 0.40 | 0.16 | G-CHOOSE | [1](https://heise.de/-11287205) |
| `ACT-HRC-002` Following ambiguous and corrective instructions during shared work | research | 3-7y | 0.40 | 0.16 | G-CHOOSE | [1](https://arxiv.org/abs/2602.14551) |
| `ACT-MRC-001` Mixed robot teams on one workflow | demo | 3-7y | 0.35 | 0.14 | G-CONC, G-CHOOSE | [1](https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/) |
| `ACT-PRED-003` Forecasting its own failure and asking before it happens | concept | 3-7y | 0.35 | 0.14 | G-PREDICT | [1](https://arxiv.org/abs/2604.16677) [2](https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/) |
| `ACT-ASM-001` Robot-team structural assembly with symbolic replanning | research | 7y+ | 0.25 | 0.03 | G-CONC | [1](https://arxiv.org/abs/2601.00978v1) |

| Language gap | Scope | Priority | Near | Benchmark tasks |
|---|---|---|---|---|
| `G-TRIGGER` Triggers on missions: every hour, at a time, on a message or an event, on the robot's own state | open | 3.11 | yes | 13 |
| `G-DATE` Dates and times on another day: tomorrow, a weekday, a calendar date | open | 1.76 | yes | 4 |
| `G-CALC` Arithmetic, counting and text built from values | open | 1.55 | yes | 11 |
| `G-CLOCK` Clock readings and times of day | closed | 0.00 | — | 0 |
| `G-FALLBACK` A different end state counts as done | out | 0.00 | — | 1 |
| `G-LOOP` Repetition without a fixed bound: loops, retry until it works, schedules, event streams | closed | 0.00 | — | 0 |
| `G-UNBOUNDED` Repetition with no bound (forever, until it works with no limit) and nested repetition | out | 0.00 | — | 8 |

## Autonomy

| Sub-activity | Maturity | Horizon | p | Priority | Needs | Sources |
|---|---|---|---|---|---|---|
| `ACT-AUTO-001` Self-charging on long inspection missions | deployment | now | 0.65 | 0.65 | G-SELF, G-CHOOSE | [1](https://bostondynamics.com/blog/making-every-step-count-with-spot/) |
| `ACT-AUTO-002` Battery swapping for whole-shift humanoid work | demo | 1-3y | 0.55 | 0.39 | G-SELF | [1](https://newatlas.com/ai-humanoids/walker-s2-humanoid-robot-swaps-batteries) |
| `ACT-AUTO-003` Self-diagnosis, safe degradation and service requests | research | 3-7y | 0.45 | 0.18 | G-SELF | [1](https://en.edaily.co.kr/news/eda202606185436/) [2](https://arxiv.org/abs/2602.01515) |
| `ACT-AUTO-004` Mid-shift relief: a low-battery robot hands its job to a peer | concept | 3-7y | 0.30 | 0.12 | G-SELF, G-CHOOSE | [1](https://arxiv.org/abs/2603.22731) [2](https://newatlas.com/ai-humanoids/walker-s2-humanoid-robot-swaps-batteries) [3](https://www.therobotreport.com/new-iso-interoperability-standard-covers-collaboration-among-mobile-robots-from-multiple-vendors/) |

| Language gap | Scope | Priority | Near | Benchmark tasks |
|---|---|---|---|---|
| `G-SELF` The robot's own state as readings and conditions: battery level, health, faults; acting on them and asking for service | open | 1.50 | yes | 5 |

## Safety

| Sub-activity | Maturity | Horizon | p | Priority | Needs | Sources |
|---|---|---|---|---|---|---|
| `ACT-SEC-001` Security patrol that reacts to people | demo | 1-3y | 0.60 | 0.42 | G-PROX, G-TRIGGER, G-DATE | [1](https://www.therobotreport.com/boston-dynamics-releases-spot-and-orbit-5-1-with-new-spot-cam/) |
| `ACT-SAFE-001` Fence-free humanoid work beside people | demo | 3-7y | 0.65 | 0.26 | G-PROX, G-FORCE | [1](https://letsdatascience.com/news/agility-robotics-previews-cage-free-digit-v5-847ce6a1) [2](https://www.i-scoop.eu/iso-25785-1-explained-and-what-it-means-for-humanoid-robot-safety/) |
| `ACT-AUTO-003` Self-diagnosis, safe degradation and service requests | research | 3-7y | 0.45 | 0.18 | G-SELF | [1](https://en.edaily.co.kr/news/eda202606185436/) [2](https://arxiv.org/abs/2602.01515) |
| `ACT-SAFE-002` A safety layer under a learned policy that vetoes unsafe moves | concept | 3-7y | 0.30 | 0.12 | G-PROX, G-FORCE | [1](https://arxiv.org/abs/2606.09749) [2](https://www.din.de/en/getting-involved/standards-committees/nam/projects/wdc-proj:din21:400666335) [3](https://letsdatascience.com/news/agility-robotics-previews-cage-free-digit-v5-847ce6a1) |
| `ACT-CARE-001` Physical help with standing and transfers in care homes | concept | 7y+ | 0.12 | 0.01 | G-FORCE, G-PROX | [1](https://e27.co/when-the-carer-has-dementia-too-japan-turns-to-physical-ai-to-rescue-eldercare-20261007/) [2](https://arxiv.org/abs/2508.19816) |

| Language gap | Scope | Priority | Near | Benchmark tasks |
|---|---|---|---|---|
| `G-FORCE` Force limits and gentle handling | open | 0.98 | — | 2 |
| `G-PROX` Limits and reactions that depend on people nearby: slow, stop or yield when a person is within a distance | open | 0.97 | — | 4 |
| `G-PATH` Staying inside or outside a region while moving | closed | 0.00 | — | 0 |
| `G-STEPLIMIT` A region or speed limit on one step or one stretch of a route, not the whole task | open | 0.00 | — | 1 |
| `G-WORKSPACE` Limits on the arm's workspace, not only on where the robot drives | open | 0.00 | — | 1 |

## Prediction

| Sub-activity | Maturity | Horizon | p | Priority | Needs | Sources |
|---|---|---|---|---|---|---|
| `ACT-PRED-002` Checking a plan with a world model before acting | research | 3-7y | 0.45 | 0.18 | G-PREDICT, G-CHOOSE | [1](https://arxiv.org/abs/2606.13672) |
| `ACT-PRED-001` Anticipating a coworker's next movement | research | 3-7y | 0.40 | 0.16 | G-PREDICT, G-PROX | [1](https://arxiv.org/abs/2604.03065) |
| `ACT-PRED-003` Forecasting its own failure and asking before it happens | concept | 3-7y | 0.35 | 0.14 | G-PREDICT | [1](https://arxiv.org/abs/2604.16677) [2](https://deepmind.google/blog/gemini-robotics-2-brings-whole-body-intelligence-to-robots/) |
| `ACT-PRED-004` Helping before being asked: acting on a predicted need | concept | 7y+ | 0.15 | 0.01 | G-PREDICT | [1](https://arxiv.org/abs/2603.23950) [2](https://e27.co/when-the-carer-has-dementia-too-japan-turns-to-physical-ai-to-rescue-eldercare-20261007/) |

| Language gap | Scope | Priority | Near | Benchmark tasks |
|---|---|---|---|---|
| `G-PREDICT` Acting on a predicted future: a person's next move, the outcome of a plan, time to failure | open | 0.50 | — | 0 |
