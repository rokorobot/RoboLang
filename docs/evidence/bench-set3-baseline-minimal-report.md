# RoboLang authoring benchmark: set3-minimal

Effort medium, 3 repeat(s), 36 task(s), prompt sha256 c4ad189b8fe5, refusal fallbacks enabled (Claude Haiku 5.5 has none).
Task set branch, prompt minimal.
Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.

## Summary (answers served by the requested model)

| model | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied | cost |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 108 | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | $0.0669 |
| claude-opus-5-5 | 108 | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | $1.3062 |
| claude-sonnet-5-5 | 108 | 19/24 (79.2%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | $0.5084 |

The program measures are over the 24 task(s) that expect a program (the others expect a decline or a question). Robustness, per model:

| model | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 108 | 72/108 (66.7%) | 33/84 (39.3%) | 33/84 | 31/84 | 0/24 (0.0%) | 48/48 (100.0%) |
| claude-opus-5-5 | 108 | 76/108 (70.4%) | 29/84 (34.5%) | 29/84 | 29/84 | 0/24 (0.0%) | 52/52 (100.0%) |
| claude-sonnet-5-5 | 108 | 72/108 (66.7%) | 30/84 (35.7%) | 30/84 | 30/84 | 3/24 (12.5%) | 51/51 (100.0%) |

## claude-haiku-5-5

108 answer(s) served by claude-haiku-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 24/24 | 24/24 (100.0%) |
| valid_after_repair | 24/24 | 24/24 (100.0%) |
| goal_correct | 24/24 | 24/24 (100.0%) |
| plan_succeeds | 24/24 | 24/24 (100.0%) |
| request_satisfied | 24/24 | 24/24 (100.0%) |
| goal_exact (triples equal, before N6) | 24/24 | 24/24 (100.0%) |
| checks_ok (tasks with optional checks) | 21/21 | 21/21 (100.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 72/108 (66.7%) | 33/84 (39.3%) | 33/84 | 31/84 | 0/24 (0.0%) | 48/48 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 27/60 (45.0%) | 33/60 (55.0%) | 33/60 | 31/60 | - | 27/27 (100.0%) | - | - |
| control | 24 | 24/24 (100.0%) | - | - | - | 0/24 (0.0%) | - | 24/24 (100.0%) | 24/24 (100.0%) |
| inexpressible | 24 | 21/24 (87.5%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 21/21 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 35,038 | 3,549 | 383,292 | 118,323 | $0.0669 |

Most common diagnostic codes (all rounds):

| code | seen |
|---|---:|
| E303 | 1 |

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| conditional_wording | 24 | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) |
| no_branch_needed | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
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
| valid_first | 24/24 | 24/24 (100.0%) |
| valid_after_repair | 24/24 | 24/24 (100.0%) |
| goal_correct | 24/24 | 24/24 (100.0%) |
| plan_succeeds | 24/24 | 24/24 (100.0%) |
| request_satisfied | 24/24 | 24/24 (100.0%) |
| goal_exact (triples equal, before N6) | 24/24 | 24/24 (100.0%) |
| checks_ok (tasks with optional checks) | 21/21 | 21/21 (100.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 76/108 (70.4%) | 29/84 (34.5%) | 29/84 | 29/84 | 0/24 (0.0%) | 52/52 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 31/60 (51.7%) | 29/60 (48.3%) | 29/60 | 29/60 | - | 31/31 (100.0%) | - | - |
| control | 24 | 24/24 (100.0%) | - | - | - | 0/24 (0.0%) | - | 24/24 (100.0%) | 24/24 (100.0%) |
| inexpressible | 24 | 21/24 (87.5%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 21/21 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 34,473 | 3,549 | 379,743 | 53,730 | $1.3062 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| conditional_wording | 24 | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) |
| no_branch_needed | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
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
| valid_first | 19/24 | 19/24 (79.2%) |
| valid_after_repair | 21/24 | 21/24 (87.5%) |
| goal_correct | 21/24 | 21/24 (87.5%) |
| plan_succeeds | 21/24 | 21/24 (87.5%) |
| request_satisfied | 21/24 | 21/24 (87.5%) |
| goal_exact (triples equal, before N6) | 21/24 | 21/24 (87.5%) |
| checks_ok (tasks with optional checks) | 18/21 | 18/21 (85.7%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 72/108 (66.7%) | 30/84 (35.7%) | 30/84 | 30/84 | 3/24 (12.5%) | 51/51 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 30/60 (50.0%) | 30/60 (50.0%) | 30/60 | 30/60 | - | 30/30 (100.0%) | - | - |
| control | 24 | 21/24 (87.5%) | - | - | - | 3/24 (12.5%) | - | 21/24 (87.5%) | 21/24 (87.5%) |
| inexpressible | 24 | 21/24 (87.5%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 21/21 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 36,707 | 3,549 | 390,390 | 34,803 | $0.5084 |

Most common diagnostic codes (all rounds):

| code | seen |
|---|---:|
| E212 | 3 |

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| conditional_wording | 24 | 19/24 (79.2%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) | 21/24 (87.5%) |
| no_branch_needed | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| pre | 12 | 9/12 (75.0%) | 9/12 (75.0%) | 9/12 (75.0%) | 9/12 (75.0%) | 9/12 (75.0%) |
| region | 12 | 7/12 (58.3%) | 9/12 (75.0%) | 9/12 (75.0%) | 9/12 (75.0%) | 9/12 (75.0%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| wait_until | 9 | 7/9 (77.8%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |
