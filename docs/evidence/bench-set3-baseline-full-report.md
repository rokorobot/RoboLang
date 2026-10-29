# RoboLang authoring benchmark: set3

Effort medium, 3 repeat(s), 36 task(s), prompt sha256 744874a8696f, refusal fallbacks enabled (Claude Haiku 5.5 has none).
Task set branch, prompt full.
Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.

## Summary (answers served by the requested model)

| model | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied | cost |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 108 | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | $0.0433 |
| claude-opus-5-5 | 108 | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | $0.9583 |
| claude-sonnet-5-5 | 108 | 22/24 (91.7%) | 22/24 (91.7%) | 22/24 (91.7%) | 22/24 (91.7%) | 22/24 (91.7%) | $0.3962 |

The program measures are over the 24 task(s) that expect a program (the others expect a decline or a question). Robustness, per model:

| model | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 108 | 85/108 (78.7%) | 17/84 (20.2%) | 17/84 | 17/84 | 3/24 (12.5%) | 64/64 (100.0%) |
| claude-opus-5-5 | 108 | 96/108 (88.9%) | 6/84 (7.1%) | 6/84 | 6/84 | 3/24 (12.5%) | 75/75 (100.0%) |
| claude-sonnet-5-5 | 108 | 96/108 (88.9%) | 7/84 (8.3%) | 7/84 | 7/84 | 2/24 (8.3%) | 74/74 (100.0%) |

## claude-haiku-5-5

108 answer(s) served by claude-haiku-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 21/24 | 21/24 (87.5%) |
| valid_after_repair | 21/24 | 21/24 (87.5%) |
| goal_correct | 21/24 | 21/24 (87.5%) |
| plan_succeeds | 21/24 | 21/24 (87.5%) |
| request_satisfied | 21/24 | 21/24 (87.5%) |
| goal_exact (triples equal, before N6) | 21/24 | 21/24 (87.5%) |
| checks_ok (tasks with optional checks) | 21/21 | 21/21 (100.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 85/108 (78.7%) | 17/84 (20.2%) | 17/84 | 17/84 | 3/24 (12.5%) | 64/64 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 43/60 (71.7%) | 17/60 (28.3%) | 17/60 | 17/60 | - | 43/43 (100.0%) | - | - |
| control | 24 | 21/24 (87.5%) | - | - | - | 3/24 (12.5%) | - | 21/24 (87.5%) | 21/24 (87.5%) |
| inexpressible | 24 | 21/24 (87.5%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 21/21 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 35,066 | 4,619 | 498,852 | 68,441 | $0.0433 |

Most common diagnostic codes (all rounds):

| code | seen |
|---|---:|
| E303 | 1 |

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| conditional_wording | 24 | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) |
| no_branch_needed | 3 | 0/3 (0.0%) | 0/3 (0.0%) | 0/3 (0.0%) | 0/3 (0.0%) | 0/3 (0.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| pre | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| region | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| wait_until | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |

## claude-opus-5-5

108 answer(s) served by claude-opus-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 21/24 | 21/24 (87.5%) |
| valid_after_repair | 21/24 | 21/24 (87.5%) |
| goal_correct | 21/24 | 21/24 (87.5%) |
| plan_succeeds | 21/24 | 21/24 (87.5%) |
| request_satisfied | 21/24 | 21/24 (87.5%) |
| goal_exact (triples equal, before N6) | 21/24 | 21/24 (87.5%) |
| checks_ok (tasks with optional checks) | 21/21 | 21/21 (100.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 96/108 (88.9%) | 6/84 (7.1%) | 6/84 | 6/84 | 3/24 (12.5%) | 75/75 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 54/60 (90.0%) | 6/60 (10.0%) | 6/60 | 6/60 | - | 54/54 (100.0%) | - | - |
| control | 24 | 21/24 (87.5%) | - | - | - | 3/24 (12.5%) | - | 21/24 (87.5%) | 21/24 (87.5%) |
| inexpressible | 24 | 21/24 (87.5%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 21/21 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 34,473 | 4,619 | 494,233 | 34,924 | $0.9583 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| conditional_wording | 24 | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) |
| no_branch_needed | 3 | 0/3 (0.0%) | 0/3 (0.0%) | 0/3 (0.0%) | 0/3 (0.0%) | 0/3 (0.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| pre | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| region | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| wait_until | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |

## claude-sonnet-5-5

108 answer(s) served by claude-sonnet-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 22/24 | 22/24 (91.7%) |
| valid_after_repair | 22/24 | 22/24 (91.7%) |
| goal_correct | 22/24 | 22/24 (91.7%) |
| plan_succeeds | 22/24 | 22/24 (91.7%) |
| request_satisfied | 22/24 | 22/24 (91.7%) |
| goal_exact (triples equal, before N6) | 22/24 | 22/24 (91.7%) |
| checks_ok (tasks with optional checks) | 21/21 | 21/21 (100.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 96/108 (88.9%) | 7/84 (8.3%) | 7/84 | 7/84 | 2/24 (8.3%) | 74/74 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 53/60 (88.3%) | 7/60 (11.7%) | 7/60 | 7/60 | - | 53/53 (100.0%) | - | - |
| control | 24 | 22/24 (91.7%) | - | - | - | 2/24 (8.3%) | - | 22/24 (91.7%) | 22/24 (91.7%) |
| inexpressible | 24 | 21/24 (87.5%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 21/21 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 34,473 | 4,619 | 494,233 | 21,687 | $0.3962 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| conditional_wording | 24 | 22/24 (91.7%) | 22/24 (91.7%) | 22/24 (91.7%) | 22/24 (91.7%) | 22/24 (91.7%) |
| no_branch_needed | 3 | 1/3 (33.3%) | 1/3 (33.3%) | 1/3 (33.3%) | 1/3 (33.3%) | 1/3 (33.3%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| pre | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| region | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| wait_until | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |
