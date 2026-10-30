# RoboLang strategy

**Status:** PROPOSED, 2026-10-29. Owner approval pending.
**Reviewed:** every month (section 5). The [development plan](DEVELOPMENT_PLAN.md) holds the detail;
this page holds the direction.

## 1. Where we are going

**North star:** an AI system turns an operator's intent into a robot outcome that is **verified, never
assumed, and safe to refuse**: across vendors, through a shared Core, without a person writing code.

Two measurable promises follow from it, and every release is judged against them:

1. **Faithful:** what the AI writes means what the operator asked: right goal, right entities, and the
   plan reaches it.
2. **Safe to say no:** when a request cannot be expressed or done, the AI declines instead of writing
   something that looks right. A wrong program that passes every check is the worst failure we have.

## 2. Five pillars (stable for 12–18 months)

| Pillar | What "done" looks like | Today |
|---|---|---|
| P1 **Expressive language** | Operators' everyday requests fit RoboLang without silent loss: goals, bounded branching, typed values. Unbounded loops and schedules stay with the orchestrator | Goals and conditions (1.3). No branching |
| P2 **Safety by construction** | Errors are caught while the task is written, not when it runs. Authority is local and explicit, and success requires evidence | Validator, simulation-time checks, verified outcomes |
| P3 **Interoperability through Core** | A stable Core version that an independent party implements from the spec alone | Core 0.1 draft; bridge from `.rob` built; INTEROP-3 deferred |
| P4 **Path to real execution** | The same task runs in the symbolic simulator, then in a physics simulator through ROS 2, then on hardware behind a safety case | Symbolic simulation only |
| P5 **Adoption** | Someone outside this repo installs it from PyPI, reads the reference, and gets a verified run in 10 minutes | Alpha, installed from a clone |

## 3. Horizons and their exit criteria

A horizon ends when its exit criteria hold, not on a date. Dates are targets.

| Horizon | Target | Main work | Exit criteria |
|---|---|---|---|
| **H1 Trust the basics** | ~Nov 2026 | Development plan section 0, N1–N3 and N6. First PyPI release (0.2.0) | Benchmark sets 1+2: decision accuracy ≥ 97% for every model with the reference. Reference shipped and tested. `pip install robolang` works |
| **H2 Express real requests** | ~Q1 2027 | RoboLang 1.4 (bounded branching, N4); `validate --world` (N2); Core 0.2 spec (N5 and known gaps); benchmark set 3 | Set 3 (branching-heavy) goal correctness ≥ 90% (Sonnet); false programs on inexpressible tasks ≤ 5% even with the grammar only; Core 0.2 frozen |
| **H3 Leave the symbolic world** | ~Q2 2027 | ROS 2 backend in a physics simulator (Gazebo or Isaac): same `.rob`, same goals, verified through sensor observations, not ground truth. INTEROP-3 relaunch on Core 0.2 | One benchmark task family passes end to end in physics simulation. Independent Core 0.2 implementation interoperates (gate 2) |
| **H4 Earn hardware** | ~H2 2027 | Safety case, a hardware pilot with an external partner, physical-mode authorization design | Owner authorization for physical mode, given on a written safety case. Never before H3 is met |

## 4. How decisions get made

1. **Evidence before syntax.** A new language feature needs (a) benchmark tasks that fail without it,
   and (b) a re-run showing that it fixes them without raising false programs.
2. **No regression on the two promises.** A change that lowers decision accuracy or raises false
   programs on the standing benchmark is not merged without an explicit owner decision.
3. **Horizons limit scope.** Work outside the current horizon waits unless it unblocks it. At most two
   large items are in flight at once.
4. **Process stays light.** Decision records are one page. A gate exists only where an outside party or
   a safety risk needs it.
5. **Saturated measures get replaced.** When every model scores ≥ 98% on a set, it becomes a regression
   set, and a harder set takes its place.

## 5. Cadence

Development moves in days, so measurement does too.

| When | What | Output | Cost |
|---|---|---|---|
| Every PR | CI (tests, lint, Windows, packaging) | Green or blocked | – |
| Every PR that changes the language, Core, the bridge, the reference or the benchmark | GitHub Actions `Benchmark`: sets 1+2, full prompt, 1 repeat, Sonnet and Haiku (Opus runs weekly; owner decision 2026-10-30), plus the Haiku→Sonnet cascade for information. The scores are posted on the PR; a floor in `bench/gate.json` fails the job (rule 2) | PR comment | ~$0.40 |
| Weekly (Monday 06:00 UTC) | GitHub Actions `Benchmark`: sets 1+2, 3 repeats, plus set 2 with the grammar-only prompt for information | Artifact for 90 days | ~$4–6 |
| Weekly (Monday, after the run) | A Cloudroom Loop reads the weekly results and the failed answers, adds a row to the [benchmark history](evidence/benchmark-history.md) and opens a PR with a short note. It stays open for the owner only when something regressed or a new failure pattern appears | PR, or nothing | – |
| Monthly (first Monday) | Strategy review: pillars, horizon progress, kill or continue, dates | This page, with a dated note in section 6 | – |
| On demand | Before a release, and when a new model comes out (`workflow_dispatch`) | As weekly | ~$1–5 |

The per-PR gate checks the two safety decisions—decision accuracy and false programs—for every
requested model separately. Goal correctness and plan success are pooled across the requested models:
with only 22 feasible hard tasks in a one-repeat PR run, one model's single extra ambiguous reading
moves its rate by 4.5 points and is not evidence of a product regression. The weekly three-repeat run
is where per-model drift is interpreted.

The API key lives in one place: the repository secret `ANTHROPIC_API_KEY`. Without it the benchmark job
is skipped and nothing is spent.

## 6. Review log

| Date | Note |
|---|---|
| 2026-10-29 | First version. Basis: benchmark sets 1 and 2 ([analysis](evidence/bench-set2-analysis.md)). H1 started |
