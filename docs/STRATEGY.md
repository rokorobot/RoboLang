# RoboLang strategy

**Status:** PROPOSED, 2026-10-29. Owner approval pending.
**Reviewed:** every quarter (section 5). The [development plan](DEVELOPMENT_PLAN.md) holds the detail;
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

| When | What | Output |
|---|---|---|
| Every PR | CI (tests, lint, Windows, packaging) | Green or blocked |
| Every PR that changes the language, Core, prompt or reference | Benchmark sets 1+2, full prompt, 1 repeat (~$1) | Rates in the PR description; rule 2 applies |
| Monthly (first Monday) | Full benchmark: both prompts, 3 repeats, current models (~$4–6). Add 10 new tasks from that month's work | Row added to [benchmark history](evidence/benchmark-history.md) |
| Quarterly | Strategy review: pillars, horizon progress, kill or continue, dates | This page updated, with a dated note in section 6 |

The monthly run can be a scheduled Cloudroom Loop once the owner approves its cost and the API key is
available to it.

## 6. Review log

| Date | Note |
|---|---|
| 2026-10-29 | First version. Basis: benchmark sets 1 and 2 ([analysis](evidence/bench-set2-analysis.md)). H1 started |
