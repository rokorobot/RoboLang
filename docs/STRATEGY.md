# RoboLang strategy

**Status:** **APPROVED by the owner, 2026-11-11**, with decisions D1–D4 and rule 6 as below. First proposed 2026-10-29;
revised 2026-11-10/11 (state as of 2026-11-11).
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
| P1 **Expressive language** | Operators' everyday requests fit RoboLang without silent loss: goals, bounded branching, typed values. Unbounded loops and schedules stay with the orchestrator | RoboLang 1.6: goals, bounded branching (1.4), step failure handlers (1.5), measured values with units and uncertainty (1.6). Not yet: choosing objects at run time (`detect`), arithmetic, structured data |
| P2 **Safety by construction** | Errors are caught while the task is written, not when it runs. Authority is local and explicit, and success requires evidence | Validator with all diagnostics, `validate --world`, holder check, verified outcomes; uncertain readings and unknown conditions fail closed. The benchmark measures silent failures (`half_program`, `unsafe_continue`, `silent_substitution`) |
| P3 **Interoperability through Core** | A stable Core version that an independent party implements from the spec alone | Core 0.2 frozen; gate 2 passed 2026-11-10: an independent Core 0.2 amr-07, written by a clean-room AI session from the frozen bundle, passes 25/25 cases ([evidence](interop/interop-4-run-2.md)). Network and search were not technically blocked. No human or outside implementer yet |
| P4 **Path to real execution** | The same task runs in the symbolic simulator, then in a physics simulator through ROS 2, then on hardware behind a safety case | Symbolic simulation, plus a ROS 2 backend in Gazebo (UR5e, MoveIt 2) verified from sensor observations: 11 tasks × 3 runs pass. `robot_ready` from ROS 2 merged (#100). No hardware |
| P5 **Adoption** | Someone outside this repo installs it from PyPI, reads the reference, and gets a verified run in 10 minutes | `pip install robolang` works (first release: 0.2.0, language 1.1–1.4). 0.3.0 with 1.5 and 1.6 is tagged (`v0.3.0`); PyPI gets it when the owner publishes the GitHub release (D3). No outside users known |

## 3. Horizons and their exit criteria

A horizon ends when its exit criteria hold, not on a date. Dates are targets.

| Horizon | Target | Main work | Exit criteria | State on 2026-11-10 |
|---|---|---|---|---|
| **H1 Trust the basics** | ~Nov 2026 | Development plan section 0, N1–N3 and N6. First PyPI release (0.2.0) | Benchmark sets 1+2: decision accuracy ≥ 97% for every model with the reference. Reference shipped and tested. `pip install robolang` works | **Nearly met.** PyPI and the reference: done. Accuracy: 97.5–100% for every model in the latest single-repeat runs; the weekly three-repeat run of 2026-11-16 decides (the 2026-11-09 run failed: no API credit) |
| **H2 Express real requests** | ~Q1 2027 | RoboLang 1.4 (bounded branching, N4); `validate --world` (N2); Core 0.2 spec (N5 and known gaps); benchmark set 3. Since then also 1.5 (failure handlers, [0033](../spec/decisions/0033-step-failure-handlers.md), [0035](../spec/decisions/0035-bench-errata-1-5.md)) and 1.6 (measured values, [0036](../spec/decisions/0036-lang-4-measured-values.md)) | Set 3 (branching-heavy) goal correctness ≥ 90% (Sonnet); false programs on inexpressible tasks ≤ 5% even with the grammar only; Core 0.2 frozen | **Met 2026-10-30.** 1.5 and 1.6 accepted on 2026-11-09 and 2026-11-10 against their own pre-registered sets |
| **H3 Leave the symbolic world** | ~Q2 2027 | ROS 2 backend in a physics simulator (Gazebo or Isaac): same `.rob`, same goals, verified through sensor observations, not ground truth. INTEROP-3 relaunch on Core 0.2 | One benchmark task family passes end to end in physics simulation. Independent Core 0.2 implementation interoperates (gate 2) | **Met 2026-11-10.** Criterion 1 met 2026-11-01. Criterion 2: gate 2 passed, signed off by the owner 2026-11-10 (independent amr-07 by `gpt-6-astra`, 25/25 cases after erratum [0037](../spec/decisions/0037-gate-2-errata-f1-f2.md); [evidence](interop/interop-4-run-2.md)). Caveat recorded: network and search not technically blocked, audit found neither used |
| **H4 Earn hardware** | ~H2 2027 | Safety case, a hardware pilot with an external partner, physical-mode authorization design | Owner authorization for physical mode, given on a written safety case. Never before H3 is met | Not started |

**Decided with this revision (owner, 2026-11-11):**

- **D1. Dates.** H1 closes with the 2026-11-16 weekly run (target kept). H2 and H3 are done. H4 keeps ~H2 2027,
  with an intermediate target: a draft safety case by Q1 2027.
- **D2. H4 starts.** H4 is the current horizon. Work that moves nothing starts now: a robot-independent draft safety
  case, a hazard list (including the 0.05 rad MoveIt start tolerance of the Gazebo backend, #99) and the
  physical-mode authorization design. Physical execution stays unauthorized until the owner signs a written safety
  case (unchanged).
- **D3. Release 0.3.0** with languages 1.5 and 1.6, first: P5's "installs it from PyPI" covers the current language.
  The owner publishes the GitHub release, which runs the PyPI workflow.
- **D4. No new language work until H4 needs it.** The `detect` evidence set (set 5 `x03` is its first evidence) is
  deferred: it lies outside the current horizon (rule 3) and its evidence is thin.
- **Follow-up, off the critical path:** a stronger gate 2 run, technically isolated or by a human team, would remove
  the recorded caveat of the 2026-11-10 pass (gate 2 spec section 2).

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
6. **Acceptance bounds are set with the baseline in hand** (approved 2026-11-11; learned in 0035 and 0036). A bound the baseline
   already misses, or one finer than a single answer of a small set, decides nothing; write it as "no worse than the
   baseline" or "N points or one answer, whichever is larger" before the run.

## 5. Cadence

Development moves in days, so measurement does too.

| When | What | Output | Cost |
|---|---|---|---|
| Every PR | CI (tests, lint, Windows, packaging) | Green or blocked | – |
| Every PR that changes the language, Core, the bridge, the reference or the benchmark | GitHub Actions `Benchmark`: sets 1+2, full prompt, 1 repeat, Sonnet and Haiku (Opus runs weekly; owner decision 2026-10-30), plus the Haiku→Sonnet cascade for information. The scores are posted on the PR; a floor in `bench/gate.json` fails the job (rule 2) | PR comment | ~$0.50 (measured) |
| Weekly (Monday 06:00 UTC) | GitHub Actions `Benchmark`: sets 1+2, 3 repeats, plus set 2 with the grammar-only prompt and set 5 (measured values) with both prompts, for information | Artifact for 90 days | ~$8–10 |
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
| 2026-10-30 | **H2 exit criteria met:** RoboLang 1.4 accepted (set 3 goal correctness 100%/95% Sonnet full/grammar-only; false programs on inexpressible tasks 2.8% grammar-only; [evidence](evidence/bench-set3-acceptance.md)), `validate --world` shipped, Core 0.2 frozen ([core-v0.2.md](../spec/core-v0.2.md)). H1 still open: `pip install robolang` (0.2.0 publish, owner) and decision accuracy ≥ 97% for every model (Haiku 38/40 on set 2 in single PR runs; the weekly three-repeat run decides). H3 started: [0029](../spec/decisions/0029-physics-simulation-backend.md) approved. Per-PR benchmark without Opus, plus an offline Haiku→Sonnet cascade |
| 2026-11-01 | **H3 criterion 1 met:** one benchmark task family passes end to end in physics simulation. 11 single-arm tasks of set 1 × 3 runs in Gazebo all `SUCCEEDED`, verified from camera, TF and gripper observations; 51/51 goal checks agree with the ground-truth oracle, no false success, 4/4 scripted failure cases fail with their codes ([evidence](evidence/h3-phase4-acceptance.md), scope in [0029](../spec/decisions/0029-physics-simulation-backend.md) section 7). H3 still open: an independent Core 0.2 implementation interoperating (gate 2, INTEROP-3 relaunch; [0023](../spec/decisions/0023-keep-interop-3-deferred.md) K2 now holds) |
| 2026-11-10 | **Revision proposed** (approved 2026-11-11, row below). Since 11-01: RoboLang 1.5 (failure handlers) and 1.6 (measured values) built, taught and accepted on pre-registered benchmark sets 4 and 5 ([1.5](evidence/bench-1-5-acceptance.md), [1.6](evidence/bench-1-6-acceptance.md)); the 1.6 baseline showed models silently substituting `inspect` for a reading, now 0. Weekly run adds set 5. Benchmark spend 2026-11-09/10: about $24 in local runs plus about $4 of PR checks. H1 decided by the 11-16 weekly run; H3 met 2026-11-10 (gate 2 passed); D1–D4 above for the owner |
| 2026-11-10 | **H3 met.** Gate 2 passed and signed off by the owner: an independent Core 0.2 amr-07, written in TypeScript by a clean-room `gpt-6-astra` session from the frozen bundle alone, passes all 25 gate 2 cases with 442/442 messages identical to the reference, after erratum [0037](../spec/decisions/0037-gate-2-errata-f1-f2.md) ([run 1](interop/interop-4-run-1.md), [run 2](interop/interop-4-run-2.md)). Caveat: network and search were not technically blocked; the transcript audit found neither used. H4 is next; it still needs a written safety case and the owner's authorization |
| 2026-11-11 | **Approved by the owner** with D1–D4 and rule 6 as amended after H3: H4 is the current horizon; order: release 0.3.0, start H4 (safety case draft, hazard list, authorization design), confirm H1 on 2026-11-16 |
