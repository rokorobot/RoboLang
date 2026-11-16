# RSI-01: a language that evolves with future robot activities

**Status:** **APPROVED by the owner, 2026-11-16**, with the strategy amendment of section 8 (now in the
[strategy](../STRATEGY.md): rule 1, D5, P1). R1 and R2 started 2026-11-16; R3–R5 not started.

## 1. The shift

RoboLang 1.4–1.6 grew from **today's** failures: a benchmark task that a model could not express was evidence for a
feature. That loop exists and works (decision records 0027, 0033, 0036). It can only see requests someone has
already thought of.

RSI-01 adds a loop that looks **forward**: it tries to discover which activities humanoids and other robots will
perform, derives the requests operators will give them, and finds what the language will need before the need is
urgent. The language then evolves with robotics instead of behind it.

The risk of looking forward is building for an imagined future: a large language for activities that never arrive.
RSI-01 handles it with two rules: **every forecast is falsifiable and scored**, and **syntax changes only for
activities that are close** (section 5).

## 2. Three loops

| Loop | Question | Input | Output | Cadence |
|---|---|---|---|---|
| A Foresight | What will robots do? | Vendor deployments and pilots, research (e.g. arXiv cs.RO), SDK releases (ROS 2, Isaac, LeRobot), standards, HumanoidOnline | Activity entries with dated, checkable forecasts | Weekly |
| B Language | Can RoboLang express it? | Activities → operator requests → benchmark tasks | Gaps, ranked; one proposal | Weekly gaps, monthly proposal |
| C Calibration (meta) | Is A right, is B worth it? | Forecasts that came due; accepted and rejected proposals | Source and weight changes for A and B | Quarterly |

Loop B is the existing benchmark machinery. Loop A and C are new.

## 3. Activity atlas

The atlas has two levels. **Activities** are what robots will do; **gaps** are the language capabilities those
activities need. One entry per activity:

```json
{
  "id": "ACT-LOG-004",
  "name": "Machine tending for a whole shift",
  "domain": "logistics",
  "maturity": "pilot",
  "horizon": "1-3y",
  "requests": ["Keep the press fed until the shift ends; call me if a part is rejected twice."],
  "needs": ["G-LOOP", "G-CLOCK", "G-MSG"],
  "forecast": {"claim": "At least two humanoid vendors publish a paid machine-tending deployment", "due": "2027-06-30", "p": 0.7},
  "sources": ["https://…"]
}
```

`maturity` is one of `research`, `demo`, `pilot`, `deployment`. `forecast` is a claim that a reviewer can mark true
or false on its due date, with the probability the loop gave it. Loop C scores these (Brier score), so the foresight
loop is measured, not believed.

**Today's gaps, seeded from evidence.** The 39 `inexpressible` benchmark tasks give the first gap list:

| Gap | Tasks | Scope today (a dated decision, not a law) |
|---|---|---|
| G-LOOP repetition, retry-until, schedules | 12 | Out since N4 (2026-10-29). First to revisit: continuous work is repetition |
| G-CLOCK, G-MSG clock readings; waiting for messages | 5, 5 | Open (`message_received` reserved, E211) |
| G-CALC arithmetic, counting, formatting | 4 | Open (0036 V1) |
| G-ACT new actions: open, charge, set a device | 3 | Open |
| G-PATH, G-FORCE stay in or out of a region; force limits | 2, 1 | Open; needed by H4 (safety case G2) |
| G-CONC, G-CHOOSE, G-PERCEIVE concurrency; choosing objects at run time; perceived properties | 2, 2, 2 | Open; G-CHOOSE deferred by D4 |
| G-FALLBACK a different end state counts as done | 1 | Out since 0034 |

Scope moves only by a decision record, which the loop may propose but never makes. Coverage counts open gaps, so the
metric cannot rise by quietly accepting what the current scope declines (promise 2: safe to say no).

## 4. From a forecast to evidence

A forecast is not evidence for syntax; a failing task is. Each activity becomes **future tasks**: 3–5 operator
requests with a world, written like today's benchmark tasks and marked `"activity": "ACT-…"`. They live in a new
set (`bench/tasks_future/`), run weekly with the others but outside the PR gate. A future task that models must
decline today is a measured gap with a known activity behind it.

## 5. Ranking gaps, and two speeds

A gap's priority is the sum over the activities that need it of `p(activity arrives) × nearness × safety relevance`,
divided by its estimated grammar cost. Then:

- **Near** (maturity `pilot` or `deployment`, horizon ≤ 3 years): eligible for a language proposal, run as 1.4–1.6
  were: decision record, pre-registered set, implementation, acceptance run, owner.
- **Far** (`research`, `demo`, or > 3 years): a design sketch only, no syntax. It is promoted when its activity's
  maturity changes.

This keeps the language small while the atlas stays large.

## 6. Measuring the language

| Dimension | Measure | New work |
|---|---|---|
| Coverage | open gaps expressible, weighted by activity priority | atlas |
| Fidelity | `goal_correct`, `decision_correct` | none |
| Safety to say no | `false_program`; **a hard gate, never traded** | none |
| Verifiability | goal verified at the end; diagnostics static | none |
| Economy | grammar productions, keywords, reference tokens | small counter |
| Foresight quality | Brier score of due forecasts; share of proposals whose activity arrived | loop C |

A candidate is better only when Coverage or Economy improves and Fidelity, Safety and Verifiability are no worse
than the baseline (rule 6).

## 7. Steps

| Step | What | Cost | Needs owner |
|---|---|---|---|
| R1 | Atlas file with the gap list above; test that every `inexpressible` task belongs to a gap; Economy counter | $0 | Approve this plan |
| R2 | First foresight pass, by hand-supervised agent: 15–25 activities with sources, maturity and forecasts | ~$5–15, once | Approve the spend |
| R3 | Future tasks for the near activities (`bench/tasks_future/`), run once to measure today's gaps | ~$2 | No |
| R4 | Cloudroom Loop for loop A (weekly): new sources, maturity changes, new activities, as a PR | ~$1/week | Approve the loop |
| R5 | Monthly ranked proposal (section 5); quarterly calibration (loop C) | ~$1/month | Each proposal |

## 8. Proposed strategy amendment

- **Rule 1, added:** "A future task, derived from an atlas activity of maturity `pilot` or `deployment`, counts as a
  benchmark task that fails without the feature."
- **D4, replaced:** "New language work follows the ranked atlas. Near gaps needed by the current horizon come first."
- **Pillar P1:** "Everyday requests" becomes "the requests of today's and the next three years' robot activities".

## 9. Not now

- **Probabilistic prediction in `.rob`** (`PREDICT …`, `collision_probability > 0.05`): collision avoidance is the
  motion planner's and safety controller's job (safety case C1, H06). The task language asks for constraints the
  backend must guarantee (G-PATH, G-FORCE). 1.6 already models uncertainty where tasks need it.
- **Auto-implementation of language changes.** An implementation agent may help after a proposal is approved, in its
  own branch, with the acceptance set frozen before it starts. Never auto-merge.
