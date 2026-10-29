# RoboLang authoring benchmark: set3-13-baseline-minimal

Effort medium, 3 repeat(s), 36 task(s), prompt sha256 4e297a05f5be, refusal fallbacks enabled (Claude Haiku 5.5 has none).
Task set branch, prompt minimal.
Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.

## Summary (answers served by the requested model)

| model | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | cost |
|---|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 108 | 23/24 (95.8%) | 23/24 (95.8%) | 23/24 (95.8%) | 23/24 (95.8%) | $0.0558 |
| claude-opus-5-5 | 108 | 22/24 (91.7%) | 22/24 (91.7%) | 22/24 (91.7%) | 22/24 (91.7%) | $1.1344 |
| claude-sonnet-5-5 | 108 | 21/24 (87.5%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | $0.6511 |

The four rates are over the 24 task(s) that expect a program (the others expect a decline). Robustness, per model:

| model | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 108 | 92/108 (85.2%) | 15/84 (17.9%) | 15/84 | 15/84 | 1/24 (4.2%) | 69/69 (100.0%) |
| claude-opus-5-5 | 108 | 94/108 (87.0%) | 12/84 (14.3%) | 12/84 | 12/84 | 2/24 (8.3%) | 72/72 (100.0%) |
| claude-sonnet-5-5 | 108 | 65/108 (60.2%) | 43/84 (51.2%) | 43/84 | 43/84 | 0/24 (0.0%) | 41/41 (100.0%) |

## claude-haiku-5-5

108 answer(s) served by claude-haiku-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 23/24 | 23/24 (95.8%) |
| valid_after_repair | 23/24 | 23/24 (95.8%) |
| goal_correct | 23/24 | 23/24 (95.8%) |
| plan_succeeds | 23/24 | 23/24 (95.8%) |
| goal_exact (triples equal, before N6) | 23/24 | 23/24 (95.8%) |
| checks_ok (tasks with optional checks) | 20/21 | 20/21 (95.2%) |

Robustness (decision judged on the first answer; the four rates above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 92/108 (85.2%) | 15/84 (17.9%) | 15/84 | 15/84 | 1/24 (4.2%) | 69/69 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 46/60 (76.7%) | 14/60 (23.3%) | 14/60 | 14/60 | - | 46/46 (100.0%) | - | - |
| control | 24 | 23/24 (95.8%) | - | - | - | 1/24 (4.2%) | - | 23/24 (95.8%) | 23/24 (95.8%) |
| inexpressible | 24 | 23/24 (95.8%) | 1/24 (4.2%) | 1/24 | 1/24 | - | 23/23 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 35,101 | 12,688 | 333,060 | 94,692 | $0.0558 |

Most common diagnostic codes (all rounds):

| code | seen |
|---|---:|
| E303 | 2 |

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|
| conditional_wording | 24 | 23/24 (95.8%) | 23/24 (95.8%) | 23/24 (95.8%) | 23/24 (95.8%) |
| no_branch_needed | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| pre | 12 | 11/12 (91.7%) | 11/12 (91.7%) | 11/12 (91.7%) | 11/12 (91.7%) |
| region | 12 | 11/12 (91.7%) | 11/12 (91.7%) | 11/12 (91.7%) | 11/12 (91.7%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| wait_until | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |

## claude-opus-5-5

108 answer(s) served by claude-opus-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 22/24 | 22/24 (91.7%) |
| valid_after_repair | 22/24 | 22/24 (91.7%) |
| goal_correct | 22/24 | 22/24 (91.7%) |
| plan_succeeds | 22/24 | 22/24 (91.7%) |
| goal_exact (triples equal, before N6) | 22/24 | 22/24 (91.7%) |
| checks_ok (tasks with optional checks) | 21/21 | 21/21 (100.0%) |

Robustness (decision judged on the first answer; the four rates above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 94/108 (87.0%) | 12/84 (14.3%) | 12/84 | 12/84 | 2/24 (8.3%) | 72/72 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 48/60 (80.0%) | 12/60 (20.0%) | 12/60 | 12/60 | - | 48/48 (100.0%) | - | - |
| control | 24 | 22/24 (91.7%) | - | - | - | 2/24 (8.3%) | - | 22/24 (91.7%) | 22/24 (91.7%) |
| inexpressible | 24 | 24/24 (100.0%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 24/24 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 34,473 | 3,172 | 339,404 | 45,639 | $1.1344 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|
| conditional_wording | 24 | 22/24 (91.7%) | 22/24 (91.7%) | 22/24 (91.7%) | 22/24 (91.7%) |
| no_branch_needed | 3 | 1/3 (33.3%) | 1/3 (33.3%) | 1/3 (33.3%) | 1/3 (33.3%) |
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
| valid_first | 21/24 | 21/24 (87.5%) |
| valid_after_repair | 24/24 | 24/24 (100.0%) |
| goal_correct | 24/24 | 24/24 (100.0%) |
| plan_succeeds | 24/24 | 24/24 (100.0%) |
| goal_exact (triples equal, before N6) | 24/24 | 24/24 (100.0%) |
| checks_ok (tasks with optional checks) | 21/21 | 21/21 (100.0%) |

Robustness (decision judged on the first answer; the four rates above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 65/108 (60.2%) | 43/84 (51.2%) | 43/84 | 43/84 | 0/24 (0.0%) | 41/41 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 20/60 (33.3%) | 40/60 (66.7%) | 40/60 | 40/60 | - | 20/20 (100.0%) | - | - |
| control | 24 | 24/24 (100.0%) | - | - | - | 0/24 (0.0%) | - | 24/24 (100.0%) | 24/24 (100.0%) |
| inexpressible | 24 | 21/24 (87.5%) | 3/24 (12.5%) | 3/24 | 3/24 | - | 21/21 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 37,653 | 3,172 | 352,092 | 49,748 | $0.6511 |

Most common diagnostic codes (all rounds):

| code | seen |
|---|---:|
| E212 | 3 |
| E304 | 1 |

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|
| conditional_wording | 24 | 21/24 (87.5%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) |
| no_branch_needed | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| pre | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| region | 12 | 9/12 (75.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| wait_until | 9 | 6/9 (66.7%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |
