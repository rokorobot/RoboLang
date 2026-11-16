# RSI-01: the first automated improvement loop for RoboLang

**Status:** PROPOSED, 2026-11-16. Not approved. Nothing here is built or scheduled.
**Scope:** automate the loop RoboLang already runs by hand. No new subsystem, no new language feature, no new
dependency. Strategy D4 (no new language work until H4 needs it) and section 4 rules 1, 2 and 6 hold unchanged.

## 1. The loop already exists, by hand

RoboLang has finished three improvement iterations with the same steps:

| Step | 1.4 branching | 1.5 failure handlers | 1.6 measured values | Automated today? |
|---|---|---|---|---|
| Evaluate | set 2: silent half-programs (F3) | set 4 baseline | set 5 baseline: silent `inspect` substitution | **Yes:** per-PR and weekly `Benchmark` |
| Diagnose | [bench-set2-analysis](../evidence/bench-set2-analysis.md) | [0033](../../spec/decisions/0033-step-failure-handlers.md) | [0036](../../spec/decisions/0036-lang-4-measured-values.md) evidence | Partly: the weekly Loop writes the history row and a note |
| Propose | N4 | 0033 | 0036 | No |
| Pre-register | set 3 bounds | set 4 bounds | set 5 bounds (rule 6) | No |
| Implement + verify | #70–#73, acceptance run | acceptance run | acceptance run | No |
| Promote | owner | owner | owner | Must stay manual |

RSI-01 automates Diagnose and Propose, and keeps the rest as it is. Language changes stay a human decision.

## 2. Capability atlas, seeded from evidence

The 39 `inexpressible` tasks across sets 2, ask, 3, 4 and 5 are the first atlas: each is a request that RoboLang
cannot express today, with a written reason. Grouped by the capability they need (one primary group per task):

| Gap | Tasks | Examples | Status by design |
|---|---|---|---|
| G-LOOP unbounded repetition, retry-until, schedules | 12 | `x_every_hour`, `f10_retry_until_works` | **Out of scope:** belongs to the orchestrator (N4). A decline is correct |
| G-CLOCK clock readings, times of day | 5 | `u17_at_1400`, `x05_cooling_time` | Open |
| G-MSG waiting for or confirming a message | 5 | `u11_wait_go`, `f12_message_delivery` | Open (`message_received` reserved, E211) |
| G-CALC arithmetic, counting, formatting | 4 | `x01_average`, `x_count_and_send` | Open (0036 V1 excluded it) |
| G-ACT new actions: open, charge, set a device | 3 | `u15_open_oven`, `x06_set_oven` | Open |
| G-PATH stay inside / outside a region while moving | 2 | `u09_avoid_quay`, `u10_inside_clean_room` | Open (`in_region`, `no_go_region` reserved, E211) |
| G-CONC two robots at the same time | 2 | `u13_simultaneous_lift`, `u14_while_carrying` | Open |
| G-CHOOSE choose an object at run time | 2 | `x03_any_hot_pan`, `x_nearest_robot` | Deferred by D4 |
| G-PERCEIVE conditions on perceived properties | 2 | `u01_if_dirty`, `u02_if_damaged` | Open |
| G-FORCE force limits, gentle handling | 1 | `a15_nice_and_slow` | Open |
| G-FALLBACK a different end state counts as done | 1 | `f11_fallback_done` | Out of scope: success means the stated goal |

Two consequences for measuring:

1. **Coverage is not "more is better".** 13 of 39 gaps are out of scope on purpose. Coverage counts only gaps marked
   open, and an out-of-scope gap that becomes expressible is a regression of promise 2 (safe to say no).
2. **New atlas entries need a task, not just a description.** An entry without a benchmark task that fails without
   it is a wish, not evidence (rule 1). Capabilities from robotics research (humanoids, bimanual work, tool use)
   enter the atlas as new `inexpressible` tasks.

## 3. Quality, measured with what exists

| Dimension | Measure | Source | New work |
|---|---|---|---|
| Coverage | open gaps expressible / open gaps | atlas file + task groups | atlas file |
| Fidelity | `goal_correct`, `decision_correct` | `bench/score.py` | none |
| Safety to say no | `false_program`, false programs valid and `SUCCEEDED` | `bench/score.py`, `gate.json` | none: **a hard gate, never traded** |
| Verifiability | `plan_succeeds` with goal verified; every diagnostic static, not at run time | scorer, validator | none |
| Economy | grammar productions, keywords, tokens of `language-reference.md` | `spec/grammar-v1.x.ebnf`, reference | one small deterministic counter |
| Cost | dollars per accepted change | run reports | none |

A candidate is better only when Coverage or Economy improves and Fidelity, Safety and Verifiability are no worse
than the baseline (rule 6 wording). Economy guards against growing a language of a thousand special commands.

## 4. RSI-01 in three steps

| Step | What | Cost | Needs owner |
|---|---|---|---|
| R1 | `bench/atlas.json`: the table above, one entry per gap with its task IDs and status. A test fails if an `inexpressible` task is in no entry, or if a task's ID is missing. A deterministic counter adds Economy to the score report | $0 | Approve this plan |
| R2 | The existing weekly Loop also classifies each failed answer: F1 the model misused the language (fix: reference), F2 the language cannot express it (atlas), F3 the language is ambiguous (spec), F4 a tool bug (code), F5 a correct decline. The note lists new F2 and F3 cases and which gap they hit | a few cents per week | No |
| R3 | Monthly, with the strategy review: a ranked one-page proposal of the next gap to close (most failing tasks, horizon fit, smallest grammar change), with draft pre-registered bounds. **Proposal only:** no branch, no code, no auto-merge | ~$1 | Each proposal |

An approved proposal then runs as 1.4–1.6 did: decision record, pre-registered set, implementation, acceptance run,
owner. That is where an implementation agent could later help, in its own branch, with the acceptance set frozen
before it starts.

## 5. Not now

- **Meta-RSI** (improving the improver): after three hand iterations there is too little data. Revisit after five
  RSI-01 proposals, with dollars and owner rejections per proposal as the measure.
- **Probabilistic prediction in `.rob`** (`PREDICT …`, `collision_probability > 0.05`): collision avoidance is the
  motion planner's and the safety controller's job (safety case C1, H06). The task language can ask for constraints
  the backend must guarantee (G-PATH, G-FORCE); it does not compute them. 1.6 already models uncertainty where the
  task needs it: readings with `±` that fail closed when unknown.
- **Automated research of robotics trends.** Useful input for new atlas tasks, but it produces descriptions, not
  evidence. Add it after R2 shows the loop can absorb new tasks.

## 6. A first candidate for the owner, when H4 asks

G-PATH and G-FORCE are the gaps closest to the current horizon: safety case gap G2 needs "a declared workspace it
cannot leave" and checked speed and force limits (hazards H05, H06). Expressing them as requests checked against
the controller's envelope, never enforced by RoboLang, would close three standing tasks and serve H4. It would also
show whether the loop works on a safety-relevant change before it is used on anything larger.
