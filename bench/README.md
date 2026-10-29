# RoboLang authoring benchmark

Can AI models write RoboLang well? 40 plain-English requests, four small simulated worlds, a stable system prompt,
and an offline scorer. Set 1 (the default) measures syntax and is saturated; **set 2, `--set hard`**, measures robustness: 40
requests of which 18 cannot be done (inexpressible in RoboLang 1.3, or infeasible in the world), plus decoy worlds with
near-miss names, ambiguous requests and tricky controls (see "Set 2" below). Decision record: [0024](../spec/decisions/0024-authoring-benchmark.md). This directory is not
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

Options: `--set 1|hard` (default 1), `--prompt full|minimal` (default full), `--models claude-opus-5-5,claude-sonnet-5-5,claude-haiku-5-5` (default), `--effort medium` (`low` to `max`),
`--repeats 1`, `--tasks id,id` (default the whole set), `--workers 4`, `--run-id NAME` (an existing run is resumed: saved
answers are not asked again), `--no-fallbacks`, `--out DIR`.

**Cost.** The default run (3 models, 40 tasks, effort `medium`, one repeat) is estimated at about **$5.5**
(Opus ~$3.6, Sonnet ~$1.8, Haiku ~$0.1); `--effort low` about $2.5; `--repeats 3 --effort high` about $37. The estimate
is printed before anything is sent and is a guess: about 3.5 characters per token, a fixed number of output tokens per
request by effort (thinking is the bulk), a quarter of the answers needing a repair round, and the ~2,000-token system
prompt written to the cache once per model and read after that. `report.md` has the real tokens and dollars. Prices are
in one dict, `PRICES` in `lib.py`.

## What happens to one task

1. The model gets `prompt.md` with the shipped `docs/language-reference.md` inserted (cached), plus a message with the world (entity IDs and state) and the request, and answers
   `{"decision", "category", "reason", "rob", "task", "bindings"}` (structured output, `output_config.effort`);
   `decision` is `program` or `cannot`. Set 1 answers saved before this have only the last three keys and are read as programs.
2. The answer is checked like `robolang validate` + `robolang simulate` would: source, task name, bindings.
3. If it is an invalid program, the model gets its own answer back with the exact tool output, once, and answers again.
   A `cannot` is final and never repaired.
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
| `goal_correct` | the goal means what an expected reading means: under the bindings it holds in exactly the same states as one expected set of Core triples, or as "any of" the readings. Checked by enumerating every placement of the things and robots the goals mention, so `and`, `or`, `not`, order and duplicates do not matter |
| `goal_exact` | the measure before N6: the goal, exported under the bindings, is exactly an expected set of Core triples. Kept in the report for comparison with older runs |
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
| `prompt.md`, `prompt_minimal.md` | stable prompt wrappers (the hash of the assembled prompt is stored with each run); the full prompt inserts `docs/language-reference.md`, while the minimal prompt inserts the grammar |
| `tasks_hard/`, `reference_hard/`, `worlds/*.notes.json` | set 2: tasks, references and attempts, entity descriptions |
| `lib.py`, `run.py`, `score.py` | shared checks and prices; the runner; the scorer |

To add a task: write `tasks/<id>.json`, `reference/<id>.rob` and `reference/<id>.json`, then run `pytest tests/test_bench.py`
(the count in that test is 40; change it on purpose).

## Set 2: robustness (`--set hard`)

```bash
python bench/run.py --set hard                  # estimate only; nothing is spent
python bench/run.py --set hard --prompt minimal --yes   # only the grammar and the answer format as the prompt
python bench/score.py bench/results/<run-id>
```

Tasks are in `tasks_hard/`, references in `reference_hard/`, in five kinds (`group` in each task): `inexpressible` (10),
`infeasible` (8), `decoy` (8), `ambiguous` (5, `expected_goals` lists the acceptable goal sets) and `control` (9).
A task that expects a decline has `expect: {"decision": "cannot", "category": ...}`, a `why`, and an obvious attempt
(`<id>.attempt.rob`) that the tests show fails with `attempt_fails_with`. Decoy worlds `depot` and `cleanroom`
have a `<world>.notes.json` of entity descriptions, printed under the world listing.

Reading the report: the four rates are over the tasks that expect a program. Then, per model:

| Figure | Meaning |
|---|---|
| `decision_correct` | the first answer's decision (program or cannot) is the expected one |
| `false_program` | a program for a task that expects a decline: the dangerous error |
| `false_program after repair` / `of which valid and SUCCEEDED` | still a program after the repair round; and valid and `SUCCEEDED`, so no tool would catch it |
| `false_refusal` | `cannot` on a task that expects a program |
| `category_correct (of declines)` | of the correct declines, the category matches |

and a table per kind of task. To add a set 2 task, write `tasks_hard/<id>.json` and either `reference_hard/<id>.rob` +
`<id>.json` (feasible) or `<id>.attempt.rob` (+ `<id>.attempt.json` when the attempt parses) with `attempt_fails_with`,
then run `pytest tests/test_bench_hard.py` (it counts 40 tasks and the five kinds; change the counts on purpose).

## Set "ask": asking as well as declining (`--set ask`)

```bash
python bench/run.py --set ask                                 # estimate only
python bench/run.py --set ask --yes --max-usd 10              # with a hard cap on what the run may spend
```

80 tasks in three new worlds (`pharmacy`, `yard`, `kitchen`), decision record [0025](../spec/decisions/0025-bench-2.md),
design [docs/plans/bench-2.md](../docs/plans/bench-2.md). After the independent review, five kinds (`group`):
`feasible` (12), `decoy` (11), `infeasible` (19), `inexpressible` (22) and `clarify` (16). The set is **not frozen**:
the clarification coverage is below the approved minimum. See the
[review response](../docs/evidence/bench2-label-review-response.md) and the preserved
[original packet](../docs/plans/bench-2-label-review-baseline.md). A `decoy` looks impossible or unclear but is feasible. A `clarify` task
expects `"decision": "clarify"`: the request could mean several entities (`candidates`) or omits a value, and the world
does not decide. A request that leaves the choice to the model ("a shelf", "one of the vials", set 2's `ambiguous`
kind) still expects a program.

Every answer has an `entities` list: the blockers of a decline, the candidates of a question. Extra figures:

| Figure | Meaning |
|---|---|
| `silent_guess` | legacy name: a program on entity ambiguity; free text is unscored, so disclosure is not inferred |
| `unjustified_default` | a program that assumes a required value instead of asking; a lesser error |
| `request_satisfied` | correct goal, successful simulation and every procedural check together |
| `entities_correct` | of the right declines and questions, `entities` names a blocker / exactly the candidates |

Files: `tasks_ask/<id>.json`; in `reference_ask/`, `<id>.rob` + `.json` (feasible), `<id>.attempt.rob` + `.json`
(the faithful attempt of an infeasible task, and the rejected attempt of some inexpressible ones) and
`<id>.reading<n>.rob` + `.json` (one succeeding program per reading of a clarify task). An infeasible task also has
`blockers`, and a `fix_patch` under which its attempt succeeds (or a `why` saying no world can fix it).
`tests/test_bench_ask.py` checks all of it. Set "ask" is not in the CI benchmark workflow: its runs need the owner's
authorization (0025 B8).

Procedure checks use `expected_action_sequences`: each entry has a `steps` list of patterns over successful executed
actions (`kind`, world IDs in `robot`/`obj`/`sensor`/`target`, literal `channel`/`message`, normalized `duration_ms`).
Patterns match in order while allowing unrelated intervening steps; `adjacent: true` requires consecutive actions.
Separate sequences let independent deliveries occur in any order. `required_entities` requires use in successful
actions, rather than merely listing an unused binding. `speed_below` accepts any positive declared max speed strictly
below the recorded bound. These feed `checks_ok` and `request_satisfied`; individual results appear in `scores.json`.
For missing entities (`failure_kind: missing_entity`), a decline must have an empty `entities` list. The answer category
remains `infeasible`. Sufficient timeouts based on guaranteed scheduled events are implementation choices.
