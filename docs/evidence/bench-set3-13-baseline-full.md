# RoboLang authoring benchmark: set3-13-baseline-full

Effort medium, 3 repeat(s), 36 task(s), prompt sha256 b7b19bc11b80, refusal fallbacks enabled (Claude Haiku 5.5 has none).
Task set branch, prompt full.
Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.

## Summary (answers served by the requested model)

| model | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | cost |
|---|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 108 | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | $0.0354 |
| claude-opus-5-5 | 108 | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | $0.8262 |
| claude-sonnet-5-5 | 108 | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | $0.4189 |

The four rates are over the 24 task(s) that expect a program (the others expect a decline). Robustness, per model:

| model | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 108 | 91/108 (84.3%) | 14/84 (16.7%) | 14/84 | 14/84 | 3/24 (12.5%) | 70/70 (100.0%) |
| claude-opus-5-5 | 108 | 103/108 (95.4%) | 2/84 (2.4%) | 2/84 | 2/84 | 3/24 (12.5%) | 81/82 (98.8%) |
| claude-sonnet-5-5 | 108 | 96/108 (88.9%) | 12/84 (14.3%) | 12/84 | 12/84 | 0/24 (0.0%) | 72/72 (100.0%) |

## claude-haiku-5-5

108 answer(s) served by claude-haiku-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 21/24 | 21/24 (87.5%) |
| valid_after_repair | 21/24 | 21/24 (87.5%) |
| goal_correct | 21/24 | 21/24 (87.5%) |
| plan_succeeds | 21/24 | 21/24 (87.5%) |
| goal_exact (triples equal, before N6) | 21/24 | 21/24 (87.5%) |
| checks_ok (tasks with optional checks) | 21/21 | 21/21 (100.0%) |

Robustness (decision judged on the first answer; the four rates above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 91/108 (84.3%) | 14/84 (16.7%) | 14/84 | 14/84 | 3/24 (12.5%) | 70/70 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 46/60 (76.7%) | 14/60 (23.3%) | 14/60 | 14/60 | - | 46/46 (100.0%) | - | - |
| control | 24 | 21/24 (87.5%) | - | - | - | 3/24 (12.5%) | - | 21/24 (87.5%) | 21/24 (87.5%) |
| inexpressible | 24 | 24/24 (100.0%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 24/24 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 34,473 | 4,236 | 453,252 | 53,726 | $0.0354 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|
| conditional_wording | 24 | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) |
| no_branch_needed | 3 | 0/3 (0.0%) | 0/3 (0.0%) | 0/3 (0.0%) | 0/3 (0.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| pre | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| region | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| wait_until | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |

## claude-opus-5-5

108 answer(s) served by claude-opus-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 21/24 | 21/24 (87.5%) |
| valid_after_repair | 21/24 | 21/24 (87.5%) |
| goal_correct | 21/24 | 21/24 (87.5%) |
| plan_succeeds | 21/24 | 21/24 (87.5%) |
| goal_exact (triples equal, before N6) | 21/24 | 21/24 (87.5%) |
| checks_ok (tasks with optional checks) | 21/21 | 21/21 (100.0%) |

Robustness (decision judged on the first answer; the four rates above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 103/108 (95.4%) | 2/84 (2.4%) | 2/84 | 2/84 | 3/24 (12.5%) | 81/82 (98.8%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 58/60 (96.7%) | 2/60 (3.3%) | 2/60 | 2/60 | - | 58/58 (100.0%) | - | - |
| control | 24 | 21/24 (87.5%) | - | - | - | 3/24 (12.5%) | - | 21/24 (87.5%) | 21/24 (87.5%) |
| inexpressible | 24 | 24/24 (100.0%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 23/24 (95.8%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 34,473 | 4,236 | 453,252 | 28,825 | $0.8262 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|
| conditional_wording | 24 | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) |
| no_branch_needed | 3 | 0/3 (0.0%) | 0/3 (0.0%) | 0/3 (0.0%) | 0/3 (0.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| pre | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| region | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| wait_until | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |

## claude-sonnet-5-5

108 answer(s) served by claude-sonnet-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 24/24 | 24/24 (100.0%) |
| valid_after_repair | 24/24 | 24/24 (100.0%) |
| goal_correct | 24/24 | 24/24 (100.0%) |
| plan_succeeds | 24/24 | 24/24 (100.0%) |
| goal_exact (triples equal, before N6) | 24/24 | 24/24 (100.0%) |
| checks_ok (tasks with optional checks) | 20/21 | 20/21 (95.2%) |

Robustness (decision judged on the first answer; the four rates above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 96/108 (88.9%) | 12/84 (14.3%) | 12/84 | 12/84 | 0/24 (0.0%) | 72/72 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 51/60 (85.0%) | 9/60 (15.0%) | 9/60 | 9/60 | - | 51/51 (100.0%) | - | - |
| control | 24 | 24/24 (100.0%) | - | - | - | 0/24 (0.0%) | - | 24/24 (100.0%) | 24/24 (100.0%) |
| inexpressible | 24 | 21/24 (87.5%) | 3/24 (12.5%) | 3/24 | 3/24 | - | 21/21 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 34,473 | 4,236 | 453,252 | 24,876 | $0.4189 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|
| conditional_wording | 24 | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) |
| no_branch_needed | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| pre | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| region | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| wait_until | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |
