# RoboLang authoring benchmark: set3

Effort medium, 3 repeat(s), 36 task(s), prompt sha256 c6424efa664b, refusal fallbacks enabled (Claude Haiku 5.5 has none).
Task set branch, prompt full.
Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.

## Summary (answers served by the requested model)

| model | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied | cost |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 108 | 83/84 (98.8%) | 84/84 (100.0%) | 84/84 (100.0%) | 84/84 (100.0%) | 84/84 (100.0%) | $0.0633 |
| claude-opus-5-5 | 108 | 84/84 (100.0%) | 84/84 (100.0%) | 84/84 (100.0%) | 84/84 (100.0%) | 84/84 (100.0%) | $1.0601 |
| claude-sonnet-5-5 | 108 | 84/84 (100.0%) | 84/84 (100.0%) | 84/84 (100.0%) | 84/84 (100.0%) | 84/84 (100.0%) | $0.5976 |

The program measures are over the 84 task(s) that expect a program (the others expect a decline or a question). Robustness, per model:

| model | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 108 | 105/108 (97.2%) | 0/24 (0.0%) | 0/24 | 0/24 | 0/84 (0.0%) | 21/21 (100.0%) |
| claude-opus-5-5 | 108 | 105/108 (97.2%) | 0/24 (0.0%) | 0/24 | 0/24 | 0/84 (0.0%) | 21/21 (100.0%) |
| claude-sonnet-5-5 | 108 | 108/108 (100.0%) | 0/24 (0.0%) | 0/24 | 0/24 | 0/84 (0.0%) | 24/24 (100.0%) |

## claude-haiku-5-5

108 answer(s) served by claude-haiku-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 83/84 | 83/84 (98.8%) |
| valid_after_repair | 84/84 | 84/84 (100.0%) |
| goal_correct | 84/84 | 84/84 (100.0%) |
| plan_succeeds | 84/84 | 84/84 (100.0%) |
| request_satisfied | 84/84 | 84/84 (100.0%) |
| goal_exact (triples equal, before N6) | 17/84 | 17/84 (20.2%) |
| checks_ok (tasks with optional checks) | 21/21 | 21/21 (100.0%) |
| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) | 0/60 | 0/60 (0.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 105/108 (97.2%) | 0/24 (0.0%) | 0/24 | 0/24 | 0/84 (0.0%) | 21/21 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 60/60 (100.0%) | - | - | - | 0/60 (0.0%) | - | 60/60 (100.0%) | 60/60 (100.0%) |
| control | 24 | 24/24 (100.0%) | - | - | - | 0/24 (0.0%) | - | 24/24 (100.0%) | 24/24 (100.0%) |
| inexpressible | 24 | 21/24 (87.5%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 21/21 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 35,079 | 5,828 | 629,424 | 105,444 | $0.0633 |

Most common diagnostic codes (all rounds):

| code | seen |
|---|---:|
| E201 | 1 |

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| and | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| branching | 60 | 59/60 (98.3%) | 60/60 (100.0%) | 60/60 (100.0%) | 60/60 (100.0%) | 60/60 (100.0%) |
| communicate | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| conditional_wording | 24 | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) |
| handover | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| holding | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| inspect | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| nested | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| no_branch_needed | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object | 12 | 11/12 (91.7%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object_choice | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
| pre | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| region | 27 | 27/27 (100.0%) | 27/27 (100.0%) | 27/27 (100.0%) | 27/27 (100.0%) | 27/27 (100.0%) |
| report_and_stop | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_at | 12 | 11/12 (91.7%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_choice | 15 | 15/15 (100.0%) | 15/15 (100.0%) | 15/15 (100.0%) | 15/15 (100.0%) | 15/15 (100.0%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| route | 30 | 29/30 (96.7%) | 30/30 (100.0%) | 30/30 (100.0%) | 30/30 (100.0%) | 30/30 (100.0%) |
| sensor | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |
| wait_until | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |

## claude-opus-5-5

108 answer(s) served by claude-opus-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 84/84 | 84/84 (100.0%) |
| valid_after_repair | 84/84 | 84/84 (100.0%) |
| goal_correct | 84/84 | 84/84 (100.0%) |
| plan_succeeds | 84/84 | 84/84 (100.0%) |
| request_satisfied | 84/84 | 84/84 (100.0%) |
| goal_exact (triples equal, before N6) | 21/84 | 21/84 (25.0%) |
| checks_ok (tasks with optional checks) | 21/21 | 21/21 (100.0%) |
| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) | 0/60 | 0/60 (0.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 105/108 (97.2%) | 0/24 (0.0%) | 0/24 | 0/24 | 0/84 (0.0%) | 21/21 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 60/60 (100.0%) | - | - | - | 0/60 (0.0%) | - | 60/60 (100.0%) | 60/60 (100.0%) |
| control | 24 | 24/24 (100.0%) | - | - | - | 0/24 (0.0%) | - | 24/24 (100.0%) | 24/24 (100.0%) |
| inexpressible | 24 | 21/24 (87.5%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 21/21 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 34,452 | 5,828 | 623,596 | 38,420 | $1.0601 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| and | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| branching | 60 | 60/60 (100.0%) | 60/60 (100.0%) | 60/60 (100.0%) | 60/60 (100.0%) | 60/60 (100.0%) |
| communicate | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| conditional_wording | 24 | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) |
| handover | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| holding | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| inspect | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| nested | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| no_branch_needed | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object_choice | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
| pre | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| region | 27 | 27/27 (100.0%) | 27/27 (100.0%) | 27/27 (100.0%) | 27/27 (100.0%) | 27/27 (100.0%) |
| report_and_stop | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_at | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_choice | 15 | 15/15 (100.0%) | 15/15 (100.0%) | 15/15 (100.0%) | 15/15 (100.0%) | 15/15 (100.0%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| route | 30 | 30/30 (100.0%) | 30/30 (100.0%) | 30/30 (100.0%) | 30/30 (100.0%) | 30/30 (100.0%) |
| sensor | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |
| wait_until | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |

## claude-sonnet-5-5

108 answer(s) served by claude-sonnet-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 84/84 | 84/84 (100.0%) |
| valid_after_repair | 84/84 | 84/84 (100.0%) |
| goal_correct | 84/84 | 84/84 (100.0%) |
| plan_succeeds | 84/84 | 84/84 (100.0%) |
| request_satisfied | 84/84 | 84/84 (100.0%) |
| goal_exact (triples equal, before N6) | 15/84 | 15/84 (17.9%) |
| checks_ok (tasks with optional checks) | 21/21 | 21/21 (100.0%) |
| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) | 0/60 | 0/60 (0.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 108/108 (100.0%) | 0/24 (0.0%) | 0/24 | 0/24 | 0/84 (0.0%) | 24/24 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 60/60 (100.0%) | - | - | - | 0/60 (0.0%) | - | 60/60 (100.0%) | 60/60 (100.0%) |
| control | 24 | 24/24 (100.0%) | - | - | - | 0/24 (0.0%) | - | 24/24 (100.0%) | 24/24 (100.0%) |
| inexpressible | 24 | 24/24 (100.0%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 24/24 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 34,452 | 5,828 | 623,596 | 38,943 | $0.5976 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| and | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| branching | 60 | 60/60 (100.0%) | 60/60 (100.0%) | 60/60 (100.0%) | 60/60 (100.0%) | 60/60 (100.0%) |
| communicate | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| conditional_wording | 24 | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) |
| handover | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| holding | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| inspect | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| nested | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| no_branch_needed | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object_choice | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
| pre | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| region | 27 | 27/27 (100.0%) | 27/27 (100.0%) | 27/27 (100.0%) | 27/27 (100.0%) | 27/27 (100.0%) |
| report_and_stop | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_at | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_choice | 15 | 15/15 (100.0%) | 15/15 (100.0%) | 15/15 (100.0%) | 15/15 (100.0%) | 15/15 (100.0%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| route | 30 | 30/30 (100.0%) | 30/30 (100.0%) | 30/30 (100.0%) | 30/30 (100.0%) | 30/30 (100.0%) |
| sensor | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |
| wait_until | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |
