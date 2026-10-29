# RoboLang authoring benchmark

Can AI models write RoboLang well? 40 plain-English requests, four small simulated worlds, a stable system prompt,
and an offline scorer. Decision record: [0024](../spec/decisions/0024-authoring-benchmark.md). This directory is not
part of the installed `robolang` package; the package never imports it or the Anthropic SDK.

## Run it

```bash
pip install -r requirements-dev.txt -e .      # the repo's own setup
pip install -r bench/requirements.txt         # the Anthropic SDK, for run.py only
echo 'ANTHROPIC_API_KEY=sk-ant-...' > bench/.env    # or export it; bench/.env is gitignored

python bench/run.py                           # prints the estimated cost and stops (exit 2)
python bench/run.py --yes                     # spends it
python bench/score.py bench/results/<run-id>  # offline: writes report.md and scores.json there
```

Options: `--models claude-opus-5-5,claude-sonnet-5-5,claude-haiku-5-5` (default), `--effort medium` (`low` to `max`),
`--repeats 1`, `--tasks id,id` (default all 40), `--workers 4`, `--run-id NAME` (an existing run is resumed: saved
answers are not asked again), `--no-fallbacks`, `--out DIR`.

**Cost.** The default run (3 models, 40 tasks, effort `medium`, one repeat) is estimated at about **$5.5**
(Opus ~$3.6, Sonnet ~$1.8, Haiku ~$0.1); `--effort low` about $2.5; `--repeats 3 --effort high` about $37. The estimate
is printed before anything is sent and is a guess: about 3.5 characters per token, a fixed number of output tokens per
request by effort (thinking is the bulk), a quarter of the answers needing a repair round, and the ~2,000-token system
prompt written to the cache once per model and read after that. `report.md` has the real tokens and dollars. Prices are
in one dict, `PRICES` in `lib.py`.

## What happens to one task

1. The model gets `prompt.md` (cached) and a message with the world (entity IDs and state) and the request, and answers
   `{"rob", "task", "bindings"}` (structured output, `output_config.effort`).
2. The answer is checked like `robolang validate` + `robolang simulate` would: source, task name, bindings.
3. If invalid, the model gets its own answer back with the exact tool output, once, and answers again.
4. Every request and response is saved to `bench/results/<run-id>/<model>/<task>.<repeat>.json` with token usage and
   which model served it. The API key is redacted from anything written.

Opus 5.5 and Sonnet 5.5 are sent with server-side refusal fallbacks (`fallbacks: "default"`, beta
`server-side-fallback-2026-07-01`); Haiku 5.5 has none. Errors: the SDK retries 429 and 5xx; a request that still fails
is saved as a failed answer; 401, 403, 404 (and a 400 on a model's first request) stop the run, keeping what was saved.

## Reading the report

Per model, one table of rates (k/n answers, answers served by that model only):

| Rate | Meaning |
|---|---|
| `valid_first` | the first answer validates, names an existing task, and its bindings are accepted |
| `valid_after_repair` | the same, after at most one repair round (so it is never below `valid_first`) |
| `goal_correct` | the goal, exported under the bindings, is exactly the expected set of Core triples |
| `plan_succeeds` | the simulation under the bindings ends `SUCCEEDED` |
| `checks_ok` | the optional checks of the tasks that have them: safety limits, a message, an exact pause, probe worlds |

`goal_correct`, `plan_succeeds` and `checks_ok` look at the final answer, and are false when it is still invalid. Read them
together: `plan_succeeds` without `goal_correct` is a program that runs but declares or reaches the wrong thing;
`goal_correct` without `plan_succeeds` is the right goal with a plan that cannot reach it. Refusals and failed requests
count as failed answers (`B003`, `B004`; `B001` is an unreadable reply, `B002` an unknown task name; the other codes are
`robolang` diagnostics). Then tokens and cost, the ten most common diagnostic codes, and a breakdown by feature tag.
Answers a fallback model produced are in a separate table under the model and are not in its numbers.

Do not read much into a few points of difference: 40 tasks is a small sample. Use `--repeats`.

## Files

| Path | What |
|---|---|
| `tasks/<id>.json` | `id`, `prompt`, `world`, `expected_goal` (set of Core triples), `tags`; optional `expected_safety`, `expected_messages`, `expected_end_ms`, `probes` |
| `worlds/*.json`, `../examples/world.json` | the worlds (world file 1.1) |
| `reference/<id>.rob`, `<id>.json` | a hand-written solution and its task name and bindings; CI runs all 40 |
| `prompt.md` | the system prompt (keep it stable; its hash is stored with each run) |
| `lib.py`, `run.py`, `score.py` | shared checks and prices; the runner; the scorer |

To add a task: write `tasks/<id>.json`, `reference/<id>.rob` and `reference/<id>.json`, then run `pytest tests/test_bench.py`
(the count in that test is 40; change it on purpose).
