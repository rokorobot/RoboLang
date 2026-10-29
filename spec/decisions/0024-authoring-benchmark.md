# Decision record 0024 — A benchmark of model-written RoboLang (`bench/`)

**Date:** 2026-10-29
**Status:** PROPOSED — for the owner's review
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
