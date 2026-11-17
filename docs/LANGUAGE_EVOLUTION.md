# RoboLang language evolution strategy

**Status:** **APPROVED by the owner**, 2026-11-16 (forward-looking evolution, strategy amendment: rule 1, D5, P1) and
2026-11-17 (nine directions with autonomy; the language development phase). Part of the [strategy](STRATEGY.md),
which keeps the north star, the pillars and the horizons. This page says **how the language evolves**; the
[rules](LANGUAGE_EVOLUTION_RULES.md) say what must hold while it does.

## Document map

| Level | Document | Holds | Changes |
|---|---|---|---|
| 1 Direction | [STRATEGY.md](STRATEGY.md) | North star, pillars, horizons, decision rules 1–6, D1–D5 | Owner, monthly review |
| 2 Language strategy | this page | Principle, directions, loops, atlas, ranking, phases | Owner |
| 2 Rules | [LANGUAGE_EVOLUTION_RULES.md](LANGUAGE_EVOLUTION_RULES.md) | Rules LE-1 to LE-15, binding for people and automation | Owner only |
| 3 Plan and status | [plans/rsi-01.md](plans/rsi-01.md) | Steps R1–R5 and their state | Each step |
| 3 Language roadmap | [LANGUAGE_ROADMAP.md](LANGUAGE_ROADMAP.md) | The iterations I1–I10: which gap, which version, in what order, designed for which far needs | After R3, then monthly; dated snapshots with a git tag in [roadmap-snapshots/](roadmap-snapshots/README.md) |
| 4 Data | [`bench/atlas.json`](../bench/atlas.json) → [plans/humanoid-activities.md](plans/humanoid-activities.md) (generated) | Activities, gaps, priorities, development log | Loops A1, A2 (weekly PR) |
| 4 Evidence | [`docs/evidence/rsi-*.md`](evidence/), [`spec/decisions/`](../spec/decisions/) | One report per research pass; one decision record per language change | Each pass, each change |

Lower levels follow higher ones. When they disagree, the higher level wins and the lower one is corrected.

## 1. Principle

RoboLang does not grow by copying what other programming languages contain. It grows with **what robots and
humanoids will need to do in the real world**. Activities decide which capabilities matter; capabilities decide which
language constructs are worth their cost. RoboLang 1.4–1.6 grew from today's benchmark failures; that loop stays, and
a forward-looking one is added in front of it, so the language evolves with robotics instead of behind it.

The risk of looking forward is building a large language for a future that never arrives. Two answers: **every
forecast is falsifiable and scored**, and **only near activities get syntax** (section 5).

## 2. Nine directions

Activities and language gaps are grouped by nine directions (owner's diagram,
[capability-directions.svg](plans/capability-directions.svg); autonomy added 2026-11-17):

| Direction | Covers |
|---|---|
| perception | recognising objects, properties, events, people, uncertainty |
| reasoning | conditions, choices, time, counting, planning within a task |
| prediction | acting on an expected future: people's movements, outcomes, time to failure |
| action | manipulation, tools, devices, locomotion |
| learning | skills, objects or routes taught by demonstration or correction |
| cooperation | robots and people working together: handover, simultaneous work, delegation |
| safety | zones, limits, reactions to people nearby; always as requests checked by the controller |
| communication | messages to and from people and systems, confirmations |
| autonomy | the robot's own energy, health, faults and service requests |

## 3. Loops

| Loop | Question | Input | Output | Cadence |
|---|---|---|---|---|
| A1 Present scan | What do robots do now? | Vendor deployments and pilots, SDK releases (ROS 2, Isaac, LeRobot), HumanoidOnline | New activities; maturity changes | Weekly |
| A2 Horizon scan | Where is it heading? | Leading signals (below); the atlas's own history | Emerging activities with signals and forecasts | Weekly, deeper monthly |
| B Language | Can RoboLang express it? | Activities → operator requests → benchmark tasks | Verified gaps, ranked | Weekly; proposal monthly |
| C Calibration | Are A and B right? | Forecasts that came due; accepted and rejected proposals | Changes to sources and weights | Quarterly |

**Leading signals (A2)** come years before a deployment: `research` (CoRL, RSS, ICRA, arXiv cs.RO; robot foundation
models), `roadmap` (vendors' and integrators' plans), `funding` (rounds and programmes with a named use case),
`standard` (standards in drafting, regulation), `demand` (labour shortages, ageing), `forecast` (published
forecasts, for calibration). A2 uses three methods: **combination** (capabilities shown separately that together
enable an activity), **extrapolation** (how fast activities moved from research to pilot, from the development log),
**demand pull** (what sectors short of workers will ask for).

## 4. The activity atlas

[`bench/atlas.json`](../bench/atlas.json) has two levels:

- **Activities** (sub-activities of a direction): name, domain, 1–3 directions, maturity (`concept`, `research`,
  `demo`, `pilot`, `deployment`), horizon (`now`, `1-3y`, `3-7y`, `7y+`), a basis, 2–4 operator requests, the gaps
  they need, a dated forecast with a probability, opened sources and, for far activities, leading signals.
- **Gaps**: language capabilities RoboLang lacks, each with one direction, a scope (`open` or `out`, a dated
  decision) and the benchmark tasks that prove it.

The plan by direction, [humanoid-activities.md](plans/humanoid-activities.md), is generated from it by
`python bench/atlas_report.py`, with a dated development log for the owner's review.

## 5. Priorities and two speeds

- **Sub-activity priority** = `p × nearness` (now 1, 1–3y 0.7, 3–7y 0.4, 7y+ 0.1).
- **Direction priority** = sum of its sub-activities' priorities, each split over the directions it names.
- **Language need** of a direction = sum of its open gaps' priorities. **Future need** = sum of `p` over its far
  sub-activities.
- **Gap priority** = sum of the priorities of the sub-activities that need it.

**Near** gaps (needed by a `pilot` or `deployment` within three years) are eligible for syntax. **Far** gaps get a
design sketch only, and every near design checks that it does not close a door a far one needs. **Build next,
design for.**

## 6. Phases

**Data phase (R1–R4):** the atlas (R1), a present scan (R2), a horizon scan (A2), verification of gaps by tests (R3),
the weekly loop (R4). Its purpose is trustworthy data for language work.

**Language phase (L1–L6)**, starting after R3 and running alongside R4. Each iteration closes one gap and ends in a
release:

| Step | What | Owner |
|---|---|---|
| L1 Choose | The top near gap, verified in R3 | Approves the choice |
| L2 Design | One-page decision record: smallest construct, alternatives, the far needs it must not rule out | Approves |
| L3 Pre-register | The gap's future tasks plus controls; acceptance bounds against the baseline, before any code | Confirms the paid run |
| L4 Build | Spec, grammar, validator, simulation, Task IR, reference, examples | Reviews |
| L5 Accept | The gap's tasks pass; false programs and decision accuracy no worse; Economy recorded | Accepts or rejects |
| L6 Release | New version on PyPI; the gap closes in the atlas; a log entry | Publishes |

## 7. Measuring the language

| Dimension | Measure |
|---|---|
| Coverage | open gaps expressible, weighted by priority |
| Fidelity | `goal_correct`, `decision_correct` |
| Safety to say no | `false_program`: a hard gate, never traded |
| Verifiability | goal verified at the end; diagnostics static |
| Economy | grammar productions, keywords, reference tokens |
| Foresight quality | Brier score of due forecasts; share of closed gaps whose activities arrived |

A version is better only when Coverage or Economy improves and Fidelity, Safety and Verifiability are no worse.

## 8. Long-term arc

Instructions (what to do) → intent (what to achieve; goals since 1.3) → cooperation (Core, delegation) → adaptation
(branching 1.4, failure handlers 1.5, measured values 1.6) → prediction and learning → a language that evolves with
robotics under control. Each step is earned by evidence, as above.
