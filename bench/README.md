# RoboLang authoring benchmark

Can AI models write RoboLang well? 40 plain-English requests, four small simulated worlds, a stable system prompt,
and an offline scorer. Set 1 (the default) measures syntax and is saturated; **set 2, `--set hard`**, measures robustness: 40
requests of which 18 cannot be done (inexpressible, or infeasible in the world; three of the first became expressible in 1.4 and are branching tasks now), plus decoy worlds with
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

Tasks are in `tasks_hard/`, references in `reference_hard/`, in six kinds (`group` in each task): `inexpressible` (7), `branching` (3: relabelled for 1.4, a program judged per path, see
set 3), `infeasible` (8), `decoy` (8), `ambiguous` (5, `expected_goals` lists the acceptable goal sets) and `control` (9).
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
`feasible` (12), `decoy` (11), `value` (4: `u05`, `u06`, `u19`, `u20`, programs that read a value since 0036 PR 2), `infeasible` (20, now with `i12_scale_down`), `inexpressible` (15) and `clarify` (18).
The owner approved the replacement plan, freeze, paid validation and merge. See the
[review response](../docs/evidence/bench2-label-review-response.md),
[original packet](../docs/plans/bench-2-label-review-baseline.md) and the freeze hash in decision 0025.
Run `python tools/bench_freeze.py --check` to verify the recorded inputs against freeze v2
(`docs/evidence/bench-freeze-v2.json`, decision 0035 E9; v1 is kept as history). Changes after the freeze require errata
([errata for 1.5](../docs/evidence/bench-errata-1-5.md), [errata for 1.6](../docs/evidence/bench-errata-1-6.md),
[errata for 1.7](../docs/evidence/bench-errata-1-7.md), [errata for 1.8](../docs/evidence/bench-errata-1-8.md), [errata for 1.9](../docs/evidence/bench-errata-1-9.md), [errata for 2.0](../docs/evidence/bench-errata-2-0.md), [errata for 2.1](../docs/evidence/bench-errata-2-1.md), [errata for 2.2](../docs/evidence/bench-errata-2-2.md)).
A `decoy` looks impossible or unclear but is feasible. A `clarify` task
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

## A Haiku-first cascade (`bench/cascade.py`)

`python bench/cascade.py bench/results/<run-id>` scores, offline and from the saved answers, a cascade in which
Claude Haiku 5.5 answers first and Claude Sonnet 5.5 answers only when Haiku's final answer is not a valid program or
is a decline or a question. That is what a deployed system can see without labels. It needs a run in which both
models answered. The output is the gate table for both models and the cascade, the escalation rate and the cost per
answer, written to `cascade.json`. It is for information: no floor applies.

On the set 3 acceptance run with the full reference, the cascade matched Sonnet: 108/108 decisions, 0 false
programs, 84/84 goals. It escalated 22% of tasks and cost $1.35 per 1,000 answers, against $5.53 for Sonnet alone and
$0.59 for Haiku alone. With the grammar only it could not catch half-programs, because they are valid programs and
are never escalated. The reference is the fix there, not the cascade.

## Set 3: branching (`--set branch`)

Pre-registered for RoboLang 1.4 (decision record [0027](../spec/decisions/0027-bounded-branching.md), section 3),
before any 1.4 code, and run against 1.3 for the [baseline](../docs/evidence/bench-set3-baseline.md). Since N4 PR 3 it is
labelled for 1.4. The tasks are in `tasks_branch/` and the programs in `reference_branch/`. There are 36 tasks:

| group | tasks | `expect` (1.4) | `expect_1_3` (kept for history) |
|---|---:|---|---|
| `branching` | 20 | a program with `if`; `expect_1_4.paths` has the goal of every path, or `fail "reason"` with its messages | decline, inexpressible |
| `inexpressible` | 8 | still a decline: retry, failover, loops, counts, clocks, message waits, distances | (unchanged) |
| `control` | 8 | a program with `pre` or `wait until`; no `if` is needed | (unchanged) |

A `branching` task has `expect: {"decision": "program"}` and `expect_1_3` with the old label. Its reference is
`<id>.rob` + `<id>.json`, the 1.4 program (it was `<id>.attempt.rob` while 1.3 could not read it). `inexpressible` tasks
keep their `<id>.attempt.rob`, which still fails.

**Paths.** Each entry of `expect_1_4.paths` has `when` (a condition, for reading), `goal` (Core triples) or `fail`
(the reason) with optional `messages`, and a `world_patch`: the probe-patch format of `probes` (merged over the world
file, objects key by key; `null` removes a key, e.g. an absent object's location), with `"events": []` so that nothing
changes between the start and the `if`. A message `[channel, null]` accepts any text on that channel, for requests that
do not fix the wording. A 1.4 answer to a task without paths (set 1) is judged on the goal of the path it took. The
scorer runs the answer once per path in its patched world:

- `goal_correct`: on every path the run got through its plan to the goal check, and the effective goal of the path
  it took (`goal_means`) means the path's goal; for a `fail` path, the run ended in `fail` (`S262`);
- `plan_succeeds`: every goal path ends `SUCCEEDED` and every `fail` path ends `FAILED` `S262`, with the path's messages
  sent (the reason text is not compared);
- `scores.json` has `paths`: for each path `goal_ok`, `plan_ok`, `state`, `failure_code`; and `has_paths`.

**`half_program`** (informational, no floor): the answer is valid and `SUCCEEDED` in the world as given, but is wrong
on another path (G1: the model reads the condition off the description and writes one branch, with or without `pre`).
It is in `report.md` (the rates table, over branching tasks) and in the `gate.py` table. The acceptance run (0027
section 3: set 3 goal correctness >= 90% for Sonnet, false programs on the controls <= 5% with the grammar only) is
read from these numbers.

Set 2 has the same for its three branching tasks (`x_route_by_cell`, `x_whichever_free`, `x_missing_box_branch`),
group `branching`. The grammar-only prompt pastes `spec/grammar-v1.3.ebnf`, `spec/grammar-v1.4.ebnf`,
`spec/grammar-v1.5.ebnf`, `spec/grammar-v1.6.ebnf`, `spec/grammar-v1.7.ebnf`, `spec/grammar-v1.8.ebnf`, `spec/grammar-v1.9.ebnf`, `spec/grammar-v2.0.ebnf`, `spec/grammar-v2.1.ebnf` and `spec/grammar-v2.2.ebnf`, each a delta
over the one before. Since 0044 L4 the prompts teach RoboLang 2.2 (0043 L4 taught 2.1, 0041 L4 taught 2.0, 0042 L4 taught 1.9, 0040 L4 taught 1.8, 0039 L4 taught 1.7, 0036 PR 2 taught 1.6, 0035 PR 2 taught 1.5). In set 3,
`x_failover_on_fault` is now a program judged per path (group `recover`; `expect_1_4` keeps the old label).

Run it on demand from GitHub Actions: `Benchmark`, `set_only: set3`. That runs full and grammar-only prompts with 3
repeats, for information only and not gated (~$3-4). Locally: `python bench/run.py --set branch --repeats 3`.

## Set 4: failure handling (`--set failure`)

16 tasks in `tasks_failure/`, references in `reference_failure/`, decision record
[0035](../spec/decisions/0035-bench-errata-1-5.md). Pre-registered against RoboLang 1.4 and labelled for 1.5 in 0035 PR 2:
`expect` is the 1.5 label, `expect_1_4` keeps the 1.4 one, `expect_1_5` holds the paths. Baseline:
[bench-set4-baseline.md](../docs/evidence/bench-set4-baseline.md). Groups: `recover` (8: retry once, three and ten attempts, report and stop after a wait, wait longer,
clean up then abort, skip an inspection, report any failure; inexpressible in 1.4, a program in 1.5), `inexpressible`
(4: retry until it works, a fallback outcome, message delivery, retry every minute) and `control` (4: no handler
needed). A `recover` task's attempt is its 1.5 answer marked 1.4 (`E202`).

Failures are forced through the world (`world_patch`: an object missing, a robot not ready, a region or sensor that
never becomes available), not by scripted faults, whose step numbers differ in each answer. Since 1.4 can test most
of these states with an `if`, paths also check what only a handler gets right: `tries` (how many steps of a kind ran),
`end_ms` (a wait that ends as soon as its condition holds) and `final` (where things end up on a failing path).
`fails: true` accepts any failed run; `fail` needs the `fail` statement (`S262`).

`unsafe_continue` (every set) keeps each region an answer waits for blocked and flags a run that still ends
SUCCEEDED: a handler let the plan go on after the wait timed out. A task whose request says to go on sets
`continue_ok`.

Run it on demand from GitHub Actions: `Benchmark`, `set_only: set4`: full and grammar-only prompts, 3 repeats, not gated,
each capped at $10. Locally: `python bench/run.py --set failure --repeats 3`.

## Set 5: measured values (`--set values`)

26 tasks in `tasks_values/`, references in `reference_values/` (17 programs), decision record
[0036](../spec/decisions/0036-lang-4-measured-values.md), plan and approved labels in
[bench-5-values.md](../docs/plans/bench-5-values.md), baseline under 1.5 in
[bench-set5-baseline.md](../docs/evidence/bench-set5-baseline.md). Pre-registered against RoboLang 1.5 and labelled for 1.6 in
0036 PR 2 ([errata](../docs/evidence/bench-errata-1-6.md)): `expect` is the 1.6 label, `expect_1_5` keeps the 1.5 one
(`cannot / inexpressible` for all but the two controls) and `expect_1_6.paths` holds the world patches that force each
path (15 `value` programs, 6 `inexpressible`, 3 `infeasible`, 2 `control`). Kitchen tasks use `worlds/kitchen_values.json`.
The worlds that serve it are world file 1.2: sensors have `measures`, objects have `properties`, and a `property` event
changes one over time. What a sensor measures is shown to the model; **what an object reads never is**: the program must
measure it.

Two measures on top of the usual ones (0036 V13), in `report.md` and `scores.json`: `value_ok` (a task with
`expected_readings: [[robot, kind, object], ...]` or a `{"channel", "value_of": [object, kind]}` message in
`expected_messages` or in a path's `messages`: on every path that is not a stop, each reading was taken by a `measure`
of that robot and each such message was sent as a value, not a text) and `silent_substitution` (a valid program with
an `inspect` of the object and no `measure` of an expected kind of it: `inspect` never reads a value). A path may carry
`expected_readings` of its own (a path where the probe is down reads nothing), `goals` (several acceptable end states)
and `fails: true` together with `goals` ("stop" may also be "leave it"). A task with `goal_free: true` names no end
state ("send the batch number"): any goal that the run reaches is right.
Locally: `python bench/run.py --set values --repeats 3` (a paid run: confirm it first, 0036 V12).

## Set "future": probes of the atlas's near gaps (`--set future`)

33 tasks in `tasks_future/`, references and attempts in `reference_future/`, evidence in
[rsi-r3-gap-verification.md](../docs/evidence/rsi-r3-gap-verification.md) (RSI-01 step R3, rule LE-7). 25 `inexpressible`
**probes**, each needing exactly one open gap that a near activity of `atlas.json` claims (`activity`, `gap`), with an
obvious attempt (`<id>.attempt.rob`) that RoboLang 1.6 rejects and `attempt_fails_with`, the first code `check_answer`
reports (since 0042 L4: 16 probes; the three `G-PATH` probes are `region` programs with a 1.7 reference, the three `G-ACT`
probes `action` programs with a 1.8 reference and the three `G-CLOCK` probes `clock` programs with a 1.9 reference); 8 `control` requests from near activities that 1.6 can express (`<id>.rob`, `<id>.json`), three of which
record an atlas claim that R3 refuted. Four new worlds (`worlds/fulfilment.json`, `site.json`, `ward.json`,
`fry_station.json`) and `worlds/warehouse.json`. A probe belongs to exactly one gap in `atlas.json`;
`tests/test_bench_future.py` checks the counts per group and per gap.

Run it on demand from GitHub Actions: `Benchmark`, `set_only: future` (full prompt, 1 repeat, Sonnet and Haiku, `--max-usd 3`,
not gated). It is not in the per-PR or weekly runs. Locally: `python bench/run.py --set future` (a paid run: confirm it first).

## Set 6: region limits (`--set regions`)

22 tasks in `tasks_regions/`, references (the 9 `region` programs and the 4 controls) and the obvious 1.6 attempts in
`reference_regions/`, decision record [0039](../spec/decisions/0039-region-limits.md), plan and approved bounds in
[bench-6-regions.md](../docs/plans/bench-6-regions.md), baseline under 1.6 in
[bench-set6-baseline.md](../docs/evidence/bench-set6-baseline.md). Pre-registered against 1.6 (iteration I1, step L3) and
labelled for 1.7 in L4 ([errata](../docs/evidence/bench-errata-1-7.md)): `expect` is the 1.7 label, `expect_1_6` keeps the
1.6 one (`cannot / inexpressible` for every non-control), and `expect_1_7` holds the paths of a program and the 1.3 world
facts it rests on (9 `region` programs, 4 `infeasible`, 5 `inexpressible`, 4 `control` programs). The tasks use world file
1.3 copies (`worlds/*_1_3.json`, built from those facts); the prompt shows, for a 1.3 world, the locations a region
`contains` and what a robot `honours`. Three older probes were relabelled the same way: `f_path_avoid_wet_floor`,
`f_path_stay_in_zone`, `f_path_keep_off_road` (set future) and `u09_avoid_quay`, `u10_inside_clean_room` (set ask), group
`region`.

Two measures on top of the usual ones (0039 B1, B5, B8), in `report.md` and `scores.json`, per model:
`limit_ok` (a task with `expected_limits: [[kind, region ID], ...]`, an empty list on a control: the valid program
declares exactly those region limits, the right kind on the right region, none more and none fewer; with
`expected_speed: [value, unit]` also exactly that `max_speed`; it is one of the task's checks, so `request_satisfied`
needs it) and `limit_dropped_silently` (a valid program whose run ends SUCCEEDED but that lacks a region limit the
request asked for, so no other check notices; counted over every answer to those tasks, like `silent_substitution`).
`tests/test_bench_regions.py` checks the counts, the labels, the worlds, the references (valid, SUCCEEDED on every path,
exactly their limits), the scorer and the controls. On demand from GitHub Actions: `Benchmark`, `set_only: regions` (full and
minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated, not in the per-PR or weekly runs). Locally:
`python bench/run.py --set regions --repeats 3` (a paid run: confirm it first; estimate ~$3.3 per prompt).

## Set 7: declared actions (`--set actions`)

24 tasks in `tasks_actions/`, references (the 10 `action` programs and the 4 controls) and the obvious 1.7 attempts in
`reference_actions/`, decision record [0040](../spec/decisions/0040-declared-actions.md), plan and approved bounds in
[bench-7-actions.md](../docs/plans/bench-7-actions.md), baseline under 1.7 in
[bench-set7-baseline.md](../docs/evidence/bench-set7-baseline.md). Pre-registered against 1.7 (iteration I2, step L3) and
labelled for 1.8 in L4 ([errata](../docs/evidence/bench-errata-1-8.md)): `expect` is the 1.8 label, `expect_1_7` keeps the
1.7 one (`cannot / inexpressible` for every non-control, each with an attempt that fails with `E203`), and `expect_1_8`
holds the paths of a program and the world 1.4 facts it rests on (10 `action` programs, 5 `infeasible`, 5
`inexpressible`, 4 `control` programs). `expected_actions` is the calls a 1.8 program must make (`[]` on a control). The
tasks use world file 1.4 copies (`worlds/*_1_4.json`, built from those facts, one file for each profile a request
needs); the prompt shows, for a 1.4 world, each robot's declared actions with target, result and value kind, each
object's state and the location states (0040 C12). Four older probes were relabelled the same way: `f_act_set_fryer`,
`f_act_open_closet`, `f_act_print_layout` (set future), `u15_open_oven`, `u16_charge_drone` (set ask) and `x06_set_oven`
(set values), group `action`. A path may script a failing `do` (`scripted_fault: {step: "do", action, target}`): the
benchmark writes the world's fault for each `do` of that action on that entity (0040 C6).

Two measures on top of the usual ones (0040 B1, B5, B8), in `report.md` and `scores.json`, per model: `actions_ok` (a
task with `expected_actions: [{robot, action, target[, value]}, ...]`, an empty list on a control: the valid program has
exactly those `do` steps, the right action by the right robot on the right target with the right value, a quantity
compared in the base unit, none more and none fewer; it is one of the task's checks, so `request_satisfied` needs it) and
`action_replaced_silently` (a valid program whose run ends SUCCEEDED but that lacks an action the request asked for, as when a
`move`, an `inspect` or nothing stands in its place, so no other check notices; counted over every answer to those tasks,
like `limit_dropped_silently`). A goal `in_state` is compared by meaning as a fact `[X, "in_state", label]` about the
state of `X`, separate from where `X` is (0040 C1). `tests/test_bench_actions.py` checks the counts, the labels, the
worlds, the references (valid, SUCCEEDED on every path, exactly their actions), the 1.7 attempts, the scorer and the
controls. On demand from GitHub Actions: `Benchmark`, `set_only: actions` (full and minimal prompts, 3 repeats, Sonnet and
Haiku, `--max-usd 3` each, not gated, not in the per-PR or weekly runs). Locally: `python bench/run.py --set actions
--repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5` (a paid run: confirm it first; estimate ~$3.6 per prompt).

## Set 8: clock readings (`--set clock`)

24 tasks in `tasks_clock/`, controls' references and the obvious attempts in `reference_clock/`, decision record
[0042](../spec/decisions/0042-clock-readings.md), plan and approved bounds in
[bench-8-clock.md](../docs/plans/bench-8-clock.md), baseline under 1.8 in
[bench-set8-baseline.md](../docs/evidence/bench-set8-baseline.md). Pre-registered against 1.8 (iteration I3, step L3) and
labelled for 1.9 in L4 ([errata](../docs/evidence/bench-errata-1-9.md)): `expect` is the 1.9 label, `expect_1_8` keeps the
1.8 one (`cannot / inexpressible` for every non-control, each with an attempt that fails with `E101`, a time literal, or
`E217`, the kind `time` or `elapsed`), and `expect_1_9` holds the paths of a program and the world 1.5 facts it rests on
(10 `clock` programs, 5 `infeasible`, 5 `inexpressible`, 4 `control` programs). `expected_clock` is the clock use a 1.9
program must make (`[]` on a control). The tasks use world file 1.5 copies (`worlds/*_1_5*.json`: the base world with the
version moved to 1.5 and a `clock`, or none; a path may set another start with `world_patch.clock`); the prompt shows,
for a 1.5 world, `clock: the task starts at 2026-11-17 21:30:00 (Europe/Prague)` or `clock: none (this world has no
clock)` (0042 K9). Seven older probes were relabelled the same way: `f_clock_scan_stamp`, `f_clock_shift_cutoff`,
`f_clock_clean_at_2300` (set future), `x_elapsed_time` (set hard), `x_report_duration` (set branch), `u17_at_1400` (set
ask) and `x05_open_at_six` (set actions), group `clock`. The 1.8 attempts mark the clock part with `// @clock`: without
those lines every attempt is a valid 1.8 program that ends SUCCEEDED, which is the silent replacement of the time that
B8 counts.

Two measures on top of the usual ones (0042 B1, B5, B8), in `report.md` and `scores.json`, per model: `clock_ok` (a task
with `expected_clock: [{reading, use, ...}, ...]`, an empty list on a control: the valid program reads the clock exactly
as asked, the right reading sent to the right channel, a comparison with the operators and times asked (an `if` with its
branches swapped is the same test: the paths decide), a wait for the time asked with the timeout asked (compared as
durations, so 60 min is 1 h), none more, none fewer, and none on a control; it is one of the task's checks, so
`request_satisfied` needs it) and `time_replaced_silently` (a valid program whose run ends SUCCEEDED but that lacks a time
condition or a wait for a time the request asked for, as when a guessed `wait for`, a fixed message or nothing stands in
its place; counted over every answer to those tasks, like `action_replaced_silently`). `tests/test_bench_clock.py`
checks the counts, the labels, the worlds, the references (valid, SUCCEEDED on every path, exactly their clock use), the
1.8 attempts, the scorer and the controls. On demand from GitHub Actions: `Benchmark`, `set_only: clock` (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not
gated, not in the per-PR or weekly runs; the workflow is at GitHub's limit of 10 dispatch inputs). Locally: `python
bench/run.py --set clock --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5` (a paid run: confirm it first;
estimate ~$3.7 for the full prompt and ~$3.5 for the grammar-only one).

## Set 9: missions (`--set missions`)

26 tasks in `tasks_missions/`, the 2.0 references and the obvious 1.9 attempts in `reference_missions/`, decision record
[0041](../spec/decisions/0041-mission-layer.md), plan, bounds and open points in
[bench-9-missions.md](../docs/plans/bench-9-missions.md) (bounds B1–B9 approved 2026-11-18). Pre-registered against 1.9
(L3) and labelled for 2.0 at L4 ([errata](../docs/evidence/bench-errata-2-0.md)): `expect` is the 2.0 label (12
`program`: the 10 `mission` tasks and `x06`, `x07`, group `mission_until`, which became programs when the owner approved
clarification 2; 5 `infeasible`; 5 `inexpressible`; 4 `control`), `expect_1_9` the 1.9 label (every non-control
`cannot / inexpressible`, each with an attempt that fails with `E201`), `expect_2_0` the pre-registered 2.0 label.
`expected_mission` is `{runs, repeat, until, on_failure, policy_stated}`: the task runs with their entities and goal (and
`limits`, `actions`, a value `messages`), the bound (`max_runs` and/or `max_duration`), the `until` condition (a string)
and the failure policy (`stop`, `continue`, `{"retry": N}`); a path forces its outcome with a `world_patch` (events,
objects) and `run_faults` (task run k fails: a test form, the world file has no such field). `c01` also has an
`accepted_mission` (a mission with `at most 3 runs` is as good as the task with six moves, clarification 10).

**The answer.** A mission answer names the mission in an optional key `mission` (clarification 4); `bindings` then bind the
mission's parameters and `task` is empty. **Scoring** (`mission_ok`, `mission_paths_ok`, `repetition_replaced_silently`,
`bound_dropped_or_invented`, `no_mission_added`, `policy_invented` in `bench/score.py`): `mission_ok` is the runs (entities,
goal, limits, actions), the bound, the `until` and the policy asked, none more and none fewer; `mission_paths_ok` adds that
every path (a patched world, with its faulted runs) ends as the label says (outcome, task runs, verified runs, failed runs);
B8 counts a valid, succeeding answer with no mission where one is needed; B9 a stated bound missing or an unstated one
added (any mission for "forever", `x01`); B5 a control with the right decision and no mission added (`c01` may be either).
A policy the request does not state is `stop`; another one is recorded in `policy_invented`, never a failure.

`tests/test_bench_missions.py` checks the counts, the labels, the bounds against the prompts, the paths, that every 2.0
reference is valid and ends as its label says on every path, the scorer, the controls, the atlas and the workflow. On
demand from GitHub Actions: `Benchmark`, `set_only: missions` (full and minimal prompts, 3 repeats, Sonnet and Haiku,
`--max-usd 3` each, not gated, not in the per-PR or weekly runs; one string input `set_only` replaces the per-set booleans,
because the workflow had reached GitHub's limit of 10 dispatch inputs). Locally: `python bench/run.py --set missions
--repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5` (a paid run: confirm it first; estimate ~$4.0 for the full prompt
and ~$3.9 for the grammar-only one).

## Set 10: messages (`--set messages`)

24 tasks in `tasks_messages/`, the 2.1 references of the programs and the 2.0 references of the controls, and the obvious 2.0
attempts, in `reference_messages/`; decision record [0043](../spec/decisions/0043-incoming-messages.md), plan, bounds and open
points in [bench-10-messages.md](../docs/plans/bench-10-messages.md) (approved by the owner, 2026-11-18). Pre-registered against 2.0
(L3 of I5) and **labelled for 2.1 at L4**: `expect` is the 2.1 label (10 `message` programs, `x05` as the program `message_goal`
(clarification C14: the message part is a final bounded wait, the world part is the goal), 5 `infeasible`, 4 `inexpressible`, 4
`control`), `expect_2_0` the 2.0 label (every non-control task `cannot / inexpressible`, each with an attempt that fails with `E211`
or `E201`), `expect_2_1` the pre-registered 2.1 label: `world_2_1` (the `channels` the world declares, each with its allowed
senders; an infeasible task also the `messages` the world holds), the `paths` of a program, each forced by a `world_patch` of
incoming `messages` (`{at, channel, text, from}`) and ending at `end_ms`, and a mission's `expected_mission`. The checks a label
asks for are task keys, as in sets 6 to 8: `expected_message_waits` (a list of `{use, channel, text, timeout}`: `use` is `wait`,
`if` or `until`, `text` is `null` for any message on the channel, `timeout` `null` where the request gives none; empty on a
control), and a program's `expected_clock`, `expected_limits` and `expected_actions`. The world shown to the model is the **world
file 1.6 copy** `worlds/msg_<id>.json` of the base world: it lists `channels: line (accepts messages from rl:agent:line-ctl)` (or
`channels: none (this world has no message service)`) and the scheduled messages with channel, text and sender (C1, C2).

Scorer (`lib.py`, `score.py`): `message_ok` (exactly the message conditions asked, each with its channel, exact text, place and
timeout, none more, none fewer; one of the task's checks, so `request_satisfied` needs it; B1), **`message_wait_replaced`** (a valid
program whose run ends well with no message condition on a channel the request names: a dropped wait, a guessed `wait for`, a
`region_clear` or `sensor_ok` stand-in; counted over the programs for B8 and over the infeasible requests for the B4 clause),
**`channel_or_text_changed`** (B9: a decoy channel or text, a case change, any message where a text is named, a text invented where
the request says "whatever it says"; for `x04`, the pattern, any message condition), and `message_added` (B5: a message condition on
a control; right is the right decision and none, `no_message_added`). All are per model, like `time_replaced`. `lib.task_paths`
reads `expect_2_1.paths` of a program that is no mission, `lib.mission_label` `expect_2_1.expected_mission` (`m04`). The standing
message tasks are relabelled the same way (`f_msg_line_docked`, `f_msg_nurse_confirm`, `f_msg_wind_clear`, `x_wait_message`,
`u11_wait_go`, `u12_radio_clearance`, `x_wait_ready_message`: group `message`, world copies, references, `expect_2_0` the old
label); see [bench-errata-2-1.md](../docs/evidence/bench-errata-2-1.md).

`tests/test_bench_messages.py` checks the counts, the labels, the world copies, the channels and texts against the prompts, that every
path follows 0043 S2–S4 (a small model of it in the test), the infeasible and inexpressible reasons, that every attempt fails with
its code and every stripped attempt succeeds (and is a replaced wait), that every 2.1 reference passes every check on every path, the
measures above, the standing tasks, the controls, the atlas, the scorer (a synthetic run of both prompts in under a minute and 1 GB)
and the workflow. On demand from GitHub
Actions: `Benchmark`, `set_only: messages` (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not
gated, not in the per-PR or weekly runs). Locally: `python bench/run.py --set messages --repeats 3 --models
claude-sonnet-5-5,claude-haiku-5-5` (a paid run: confirm it first; estimate ~$3.7 for the full prompt and ~$3.6 for the
grammar-only one). The acceptance run (L5) is next.

## Set 11: more measured kinds (`--set kinds`)

24 tasks in `tasks_kinds/`, the 2.2 references of the 10 programs and of the controls (the controls' are 2.1) and the obvious 2.1
attempts in `reference_kinds/`, decision record [0044](../spec/decisions/0044-more-measured-kinds.md), plan, bounds B1-B9 and
clarifications D1-D14 in [bench-11-kinds.md](../docs/plans/bench-11-kinds.md) (approved by the owner, 2026-11-18). Pre-registered
against 2.1 (L3 of I6) and **labelled for 2.2 at L4**: `expect` is the 2.2 label (10 `kind` programs, 5 `infeasible`, 5
`inexpressible`, 4 `control`), `expect_2_1` the 2.1 label (every non-control task `cannot / inexpressible`, each with an attempt
that fails with `E101`, `E217` or `E505`), `expect_2_2` the pre-registered 2.2 label: `world_2_2` (the sensor `measures` and the
object `properties` that world file 1.7 adds, with unit and uncertainty) and the `paths` of a program (each forced by a `world_patch`
of property values; a reading near the threshold is a path that fails, `S261`). The checks a label asks for are task keys, as in sets
6 to 10: `expected_readings` (`[robot, kind, object]`), `expected_comparisons` (`{kind, op, value, unit}`; a count has no unit) and,
where the program combines a reading with an older feature, `expected_limits`, `expected_actions`, `expected_clock`,
`expected_message_waits`, and `expected_sensors` (the decoy). The world shown to the model is the **world file 1.7 copy**
`worlds/kinds_<id>.json` of the 2.1 world (15 of them: the 10 programs and the 5 infeasible tasks; the other tasks keep their 2.1
worlds): it lists which sensor measures which new kind with its unit (`cam_scan1 (on scanbot1, available; measures fill in % +/- 2)`),
and never a property value. The attempts mark the reading lines with `// @reading`: without them each is a valid 2.1 program that
succeeds, the reading replaced silently (the plan's B8; `k09` needs `attempt_world_patch`, a message in its world).

Scorer (`lib.py`, `score.py`): **`kind_ok`** (exactly the readings of `expected_readings`, each the asked kind of the asked object by
the asked robot, and exactly the comparisons of `expected_comparisons`: the kind, the test modulo its complement, and the limit exact
after unit conversion, so `450 kPa` is `4.5 bar` and `450 bar` is not; with `expected_sensors`, only those sensors; one of the task's
checks, so `request_satisfied` needs it; B1), **`reading_replaced`** (B8, and the B4 clause for the infeasible requests: a valid
program whose run ends SUCCEEDED with no `measure` of the new kind the request asks for: an `inspect`, a `label` stand-in or nothing
in its place), **`kind_or_threshold_changed`** (B9: a reading of another kind or object, a `confidence` without its `label`, a test or
limit that differs after conversion: `>=` for `>`, a factor off, the limit moved to avoid an unknown, another sensor), and `kind_added`
(B5: a new-kind reading on a control; right is the right decision and none, `no_kind_added`). All are per model, like
`message_wait_replaced`. `lib.task_paths` reads `expect_2_2.paths`.

Erratum P2 relabels `u01_if_dirty` and `u02_if_damaged` (set "ask") as `cannot / infeasible` (their worlds have no label sensor,
`S234`), and erratum P3 the four standing tasks that 2.2 changes: the R3 probes `f_perceive_shelf_fill` (set "future"; `shelf_a` is a
location: D1) and `f_perceive_weed_confidence`, and the counting requests `x_count_and_send` (set 3) and `x_count_report` (set 2):
group `kind`, programs with a world file 1.7 copy, a reference and two paths, `expect_2_1` the old label
([bench-errata-2-2.md](../docs/evidence/bench-errata-2-2.md)).

`tests/test_bench_kinds.py` checks the counts, the labels, the sensors and objects against the prompts and the world, that every
path follows the comparison (a small model of it in the test: three values, the sensor's uncertainty, exact conversion of kPa
and bar), the infeasible and inexpressible reasons, the world copies and what the model is shown, that every attempt fails with its
code and every stripped attempt succeeds, that every 2.2 reference validates and ends as labelled on every path, the scorer measures
on variants, the standing relabels, the atlas, the cost (scoring a synthetic run: under a minute and 1 GB) and the workflow. On
demand from GitHub Actions: `Benchmark`, `set_only: kinds` (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3`
each, not gated, not in the per-PR or weekly runs).
Locally: `python bench/run.py --set kinds --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5` (a paid run: confirm it
first; estimate ~$3.8 for the full prompt and ~$3.6 for the grammar-only one).

## Set 12: choosing at run time (`--set choose`)

25 tasks in `tasks_choose/`, the references of the controls and the obvious 2.2 attempts in `reference_choose/`, decision record
[0045](../spec/decisions/0045-choosing-at-run-time.md), plan, bounds and open points in
[bench-12-choose.md](../docs/plans/bench-12-choose.md) (draft for the owner: the bounds B1–B9 are placeholders). Pre-registered
against 2.2 (L3 of I7), not yet labelled for 2.3: `expect` is the 2.2 label (every non-control task `cannot / inexpressible`, each
with an attempt that fails with `E203`, a `choose` statement; 11 `choose` programs in 2.3, 5 `infeasible`, 5 `inexpressible` that
stay out, 4 `control` programs), `expect_2_3` the label for acceptance. A 2.3 program carries, inside `expect_2_3`, `world_2_3` (the
world file 1.8 facts: a `position` `[x, y]` in metres per location, the readiness of robots and the places of objects that differ from
the 2.2 world), `choices` (type, candidates in the order written, `where`, `nearest_to` or `farthest_from`), its `paths` (each forced
by a `world_patch` that changes which candidate wins; a path where no candidate is left is `fails`, `else` runs) and, for a mission,
`expected_mission` (the bound the request states, `on_failure`). The goal of a path names the entity chosen (0045 C7). The attempts
mark the choice line with `// @choice NAME=ENTITY`: with that line removed and the name replaced by the first candidate, each is a
valid 2.2 program that succeeds, the choice replaced silently (the plan's B8; a mission fails on its second run, `S215`). The worlds are
existing ones and two new world files (1.7), `bin_hall` and `parts_line`; no world of the set has a position. The new scorer measures
(`choice_ok`, choice replaced silently, criterion changed) are L4's.

The R3 probes and the standing tasks of G-CHOOSE stay in their sets. Erratum Q2 moves `x03_any_hot_pan` and
`x03_carry_as_many_as_counted` from G-CHOOSE to the new far gap G-EACH ([bench-errata-2-3.md](../docs/evidence/bench-errata-2-3.md)).

`tests/test_bench_choose.py` checks the counts, the labels, a small model of 0045 C2 to C5 over every path (which candidate wins
in each patched world, that a fixed pick, another order, no order and no `where` each lose a path), the mission paths, the
infeasible and inexpressible reasons, that every attempt fails with `E203` and every stripped attempt succeeds, the controls, the
atlas, the scorer and the workflow. On demand from GitHub Actions: `Benchmark`, `set_only: choose` (full and minimal prompts, 3
repeats, Sonnet and Haiku, `--max-usd 3` each, not gated, not in the per-PR or weekly runs). Locally: `python bench/run.py --set choose
--repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5` (a paid run: confirm it first; estimate ~$4.0 for the full prompt and ~$3.8 for
the grammar-only one).
