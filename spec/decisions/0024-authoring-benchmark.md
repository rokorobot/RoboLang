# Decision record 0024 — A benchmark of model-written RoboLang (`bench/`)

**Date:** 2026-10-29
**Status:** Set 1: APPROVED by the owner as a **baseline** — 2026-10-29 (see [Owner review](#owner-review--2026-10-29)). Set 2 (below): PROPOSED — for the owner's review
**Follows:** [0023](0023-keep-interop-3-deferred.md) reason 3 (the central claim, that AI systems write RoboLang well, is untested)

## What is measured

40 operator-style requests (`bench/tasks/`), four small worlds, one system prompt (`bench/prompt.md`). A model
answers with `{rob, task, bindings}`. If the answer is invalid it gets **one** repair round with the exact tool
output. Scores come from the installed `robolang` package only; there is no judge model.

| # | Decision |
|---|---|
| A1 | Four rates per model: `valid_first`, `valid_after_repair`, `goal_correct`, `plan_succeeds`. **Valid** means what `robolang simulate` needs to start: the source passes `robolang validate`, the named task exists, and the bindings are complete and well typed. |
| A2 | `goal_correct`: the goal exported under the model's bindings (`robolang.bridge`) equals the task's expected **set** of Core triples. `plan_succeeds`: the simulation under those bindings ends `SUCCEEDED`. Both are judged on the final answer and are false when it is not valid. |
| A3 | A fifth figure, `checks_ok`, covers what a goal and a run cannot see: safety limits stated in words, a message sent, a pause of exact length, and **probe worlds** (a changed world in which a correct plan must *not* succeed, for "only once the cell is clear"). It is reported next to the four rates, never mixed into them. |
| A4 | Every task has a hand-written reference (`bench/reference/`). `tests/test_bench.py` requires all 40 to be valid, to export exactly `expected_goal`, to reach `SUCCEEDED` and to pass their checks. |
| A5 | Refusal fallbacks (`fallbacks: "default"`) are on for the models that have them (Claude Opus 5.5, Sonnet 5.5), off for Haiku 5.5. The model that actually served each answer is recorded; **fallback-served answers are scored in their own table** and never count for the requested model. |
| A6 | `run.py` prints an estimated cost and sends nothing without `--yes`. Saved requests and responses never contain the key (it is redacted before writing). |
| A7 | The `anthropic` SDK is a benchmark-only dependency (`bench/requirements.txt`), imported lazily by `run.py` alone. The `robolang` package and its tests do not need it. |

## Limits of what this shows

- The model is **given** the world (entity IDs and state) in the request. This measures writing RoboLang, not
  finding out what is in a cell.
- 40 tasks and one repeat are a small sample: differences of a few points between models are noise. Use
  `--repeats` before drawing a conclusion.
- `prompt.md` is part of the measurement. Its hash is stored with every run; change it and the numbers are not comparable.
- The tasks, worlds and references are public, so a later model may have seen them.
- Success is success in the symbolic simulation. It says nothing about a real robot.
- The cost estimate is a guess (characters per token, output tokens by effort); the report has the real figures.

## Not changed

RoboLang 1.3, IR 1.2, run record 1.2 and world 1.1 keep their meaning. No file of the `robolang` package changed.

## First run — 2026-10-29 (`run3`, effort medium, 1 repeat)

| Model | valid first | valid after repair | goal correct | plan succeeds | optional checks | cost |
|---|---|---|---|---|---|---|
| claude-opus-5-5 | 40/40 | 40/40 | 40/40 | 40/40 | 20/20 | $0.29 |
| claude-sonnet-5-5 | 40/40 | 40/40 | 40/40 | 40/40 | 20/20 | $0.15 |
| claude-haiku-5-5 | 40/40 | 40/40 | 40/40 | 40/40 | 19/20 | $0.02 |

Full report: [docs/evidence/bench-run3-report.md](../../docs/evidence/bench-run3-report.md). No repair
round was needed. Haiku's one optional-check miss (`w_two_limits`) chose stricter limits than the
reference for "under 80 cm/s" (0.75 rather than 0.8 m/s): a wording ambiguity in the task.

**Reading.** With a 2,000-token language reference, current models write valid RoboLang 1.3 with
correct goals for every task in this set; even the smallest model does, at about $0.02 for 40
tasks. The set is **saturated**: it no longer separates models. A harder second set is needed
(requests the language cannot express, infeasible requests, larger worlds, less guidance).


## Set 2 — "hard": robustness, not syntax (`bench/tasks_hard/`, `--set hard`)

Set 1 is saturated, so set 2 asks something else: **does the model know what RoboLang and this world cannot do, and
does it bind the right entity in a crowded world?** 40 tasks, same JSON shape plus `group`, `expect` and, for a decline,
`why`. All of A1–A7 hold; these are added.

| # | Decision |
|---|---|
| H1 | **Five kinds.** 10 *inexpressible* (branching on world state, unbounded or recurring repetition, a choice made at run time, counting or measuring, waiting for a message, retry on failure); 8 *infeasible* (no gripper, unknown place or object, a sensor on another robot, an absent object, a speed limit above the robot's maximum or in a unit it does not declare, a robot that is never ready); 8 *decoy* (two 40+ entity worlds, near-miss names such as `box_1`/`box1`/`box01`, the request names things by description); 6 *ambiguous* (`expected_goals`: a list of acceptable goal sets); 8 *control* (tricky but expressible: unrolled repetition, `wait until … timeout`, `and`/`or`/`not` preconditions, a swap). 22 of 40 (55%) expect a program, so declining everything scores badly. |
| H2 | **Answer format.** `{"decision": "program"\|"cannot", "category": "inexpressible"\|"infeasible"\|"none", "reason", "rob", "task", "bindings"}`, all six required, closed object; a decline has `rob`/`task` empty and `bindings` `[]`. An older answer with only `rob`, `task`, `bindings` is read as `decision: "program"`, so set 1 results re-score unchanged. `prompt.md` gains one general paragraph on declining (and on ambiguity: answer the sensible reading); it names no task. **Its hash changed, so run3 (set 1) was made with the previous prompt and is not directly comparable to a new set 1 run.** |
| H3 | **Scoring.** The decision is judged on the **first** answer, before any tool output. `false_program` (a program for a task that expects a decline) is the headline error; `false_refusal` (a decline of a task that expects a program); `decision_correct`; `category_correct` (among correct declines). Only programs that are invalid get a repair round, as before; a decline is final. If that round turns a program into a decline, it is reported separately (`false_program after repair`), and so is how many false programs were **valid and ended `SUCCEEDED`**, i.e. nothing in the toolchain would have caught them. The four set 1 rates, `goal_correct` etc., are computed over the tasks that expect a program; a decline there counts as no program. Per-kind table in the report. |
| H4 | **Why a task is a decline.** Each has a `why`, and an "obvious attempt" in `bench/reference_hard/<id>.attempt.rob` that tests prove fails with the recorded code: for inexpressible tasks `robolang validate` rejects the natural syntax (`E203`, `E209`, `E211`, …) and the grammar has none of the constructs; for infeasible ones the world check (`E505`) or the simulation (`S201`, `S211`, `S213`, `S231`, `S001`, `S002`) fails it. Every feasible task has a reference that is valid, reaches an accepted goal and `SUCCEEDED`, and passes its probes (a changed world in which a correct plan must not succeed). |
| H5 | **Descriptions** of entities come from `bench/worlds/<world>.notes.json` and are printed under the world listing. World files 1.1 are unchanged (no new field); two new worlds, `depot` (42 entities) and `cleanroom` (46), were added. An absent object is listed as `not present`. |
| H6 | **`--prompt minimal`** sends only the grammar (`spec/grammar-v1.3.ebnf`, pasted in) and the answer format, with the same paragraph on declining, and no language guidance, example or checker hints. It measures how much the result depends on `prompt.md`. Set and prompt are stored in `run.json`; a run is resumed only with the same ones. |

**Limits of set 2.** The split between *inexpressible* and *infeasible* and the accepted readings of *ambiguous* tasks are
judgement calls, written down in each task's `why` and goals. A request such as "never give up" is judged inexpressible
because every `wait until` has a timeout; a model that writes a very long timeout counts as a false program. The cannot-tasks
state that the world may differ at run time where the listing alone would let a model pick one branch. Nothing was run
against a model for this record; there are no set 2 results yet. No file of the `robolang` package changed.

## Owner review — 2026-10-29

Set 1 is approved as the **baseline** benchmark, not as evidence that models author RoboLang robustly. The 40/40
results hold for that set only. Its limits, besides those above:

- **Coverage is narrow.** 21 of 40 tasks are tagged `pick_place`; `or`, `not`, `communicate` and `long_plan` have one
  task each.
- **Guidance is generous.** The prompt carries a full language reference and a worked example close to many tasks.
- **Every request is solvable.** Set 1 cannot see a model that should have declined or asked (sets 2 and 3 can).
- **One run per task.** Repeatability is unknown.
- **Same family.** The tasks and references were written with a Claude model; only Claude models were run.
- **run3's cost estimate was about 12× too high** ($5.5 estimated, $0.46 spent).

Set 1's tasks, worlds and references stay unchanged for comparison. Its prompt changed with set 2 and with
[0025](0025-bench-2.md) (the `clarify` answer), so a new set 1 run is not directly comparable with run3.
