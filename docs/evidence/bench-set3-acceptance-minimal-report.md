# RoboLang authoring benchmark: set3-minimal

Effort medium, 3 repeat(s), 36 task(s), prompt sha256 21180f513111, refusal fallbacks enabled (Claude Haiku 5.5 has none).
Task set branch, prompt minimal.
Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.

## Summary (answers served by the requested model)

| model | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied | cost |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 108 | 79/84 (94.0%) | 84/84 (100.0%) | 75/84 (89.3%) | 80/84 (95.2%) | 72/84 (85.7%) | $0.0886 |
| claude-opus-5-5 | 108 | 84/84 (100.0%) | 84/84 (100.0%) | 84/84 (100.0%) | 84/84 (100.0%) | 78/84 (92.9%) | $1.5025 |
| claude-sonnet-5-5 | 108 | 82/84 (97.6%) | 84/84 (100.0%) | 80/84 (95.2%) | 84/84 (100.0%) | 80/84 (95.2%) | $0.7785 |

The program measures are over the 84 task(s) that expect a program (the others expect a decline or a question). Robustness, per model:

| model | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 108 | 105/108 (97.2%) | 0/24 (0.0%) | 0/24 | 0/24 | 0/84 (0.0%) | 21/21 (100.0%) |
| claude-opus-5-5 | 108 | 104/108 (96.3%) | 1/24 (4.2%) | 1/24 | 1/24 | 0/84 (0.0%) | 20/20 (100.0%) |
| claude-sonnet-5-5 | 108 | 105/108 (97.2%) | 1/24 (4.2%) | 1/24 | 1/24 | 0/84 (0.0%) | 21/21 (100.0%) |

## claude-haiku-5-5

108 answer(s) served by claude-haiku-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 79/84 | 79/84 (94.0%) |
| valid_after_repair | 84/84 | 84/84 (100.0%) |
| goal_correct | 75/84 | 75/84 (89.3%) |
| plan_succeeds | 80/84 | 80/84 (95.2%) |
| request_satisfied | 72/84 | 72/84 (85.7%) |
| goal_exact (triples equal, before N6) | 17/84 | 17/84 (20.2%) |
| checks_ok (tasks with optional checks) | 16/21 | 16/21 (76.2%) |
| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) | 5/60 | 5/60 (8.3%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 105/108 (97.2%) | 0/24 (0.0%) | 0/24 | 0/24 | 0/84 (0.0%) | 21/21 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 60/60 (100.0%) | - | - | - | 0/60 (0.0%) | - | 53/60 (88.3%) | 56/60 (93.3%) |
| control | 24 | 24/24 (100.0%) | - | - | - | 0/24 (0.0%) | - | 22/24 (91.7%) | 24/24 (100.0%) |
| inexpressible | 24 | 21/24 (87.5%) | 0/24 (0.0%) | 0/24 | 0/24 | - | 21/21 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 38,241 | 4,223 | 472,976 | 159,003 | $0.0886 |

Most common diagnostic codes (all rounds):

| code | seen |
|---|---:|
| E201 | 3 |
| E303 | 2 |

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| and | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| branching | 60 | 57/60 (95.0%) | 60/60 (100.0%) | 53/60 (88.3%) | 56/60 (93.3%) | 53/60 (88.3%) |
| communicate | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 1/3 (33.3%) | 3/3 (100.0%) | 1/3 (33.3%) |
| conditional_wording | 24 | 22/24 (91.7%) | 24/24 (100.0%) | 22/24 (91.7%) | 24/24 (100.0%) | 19/24 (79.2%) |
| handover | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| holding | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| inspect | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 1/3 (33.3%) | 1/3 (33.3%) | 1/3 (33.3%) |
| nested | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| no_branch_needed | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object_choice | 6 | 5/6 (83.3%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
| pre | 12 | 11/12 (91.7%) | 12/12 (100.0%) | 10/12 (83.3%) | 12/12 (100.0%) | 7/12 (58.3%) |
| region | 27 | 24/27 (88.9%) | 27/27 (100.0%) | 23/27 (85.2%) | 27/27 (100.0%) | 20/27 (74.1%) |
| report_and_stop | 12 | 11/12 (91.7%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_at | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 11/12 (91.7%) | 12/12 (100.0%) | 11/12 (91.7%) |
| robot_choice | 15 | 15/15 (100.0%) | 15/15 (100.0%) | 12/15 (80.0%) | 13/15 (86.7%) | 12/15 (80.0%) |
| robot_ready | 3 | 2/3 (66.7%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| route | 30 | 29/30 (96.7%) | 30/30 (100.0%) | 26/30 (86.7%) | 28/30 (93.3%) | 26/30 (86.7%) |
| sensor | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 7/9 (77.8%) | 7/9 (77.8%) | 7/9 (77.8%) |
| wait_until | 9 | 8/9 (88.9%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |

## claude-opus-5-5

108 answer(s) served by claude-opus-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 84/84 | 84/84 (100.0%) |
| valid_after_repair | 84/84 | 84/84 (100.0%) |
| goal_correct | 84/84 | 84/84 (100.0%) |
| plan_succeeds | 84/84 | 84/84 (100.0%) |
| request_satisfied | 78/84 | 78/84 (92.9%) |
| goal_exact (triples equal, before N6) | 13/84 | 13/84 (15.5%) |
| checks_ok (tasks with optional checks) | 15/21 | 15/21 (71.4%) |
| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) | 0/60 | 0/60 (0.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 104/108 (96.3%) | 1/24 (4.2%) | 1/24 | 1/24 | 0/84 (0.0%) | 20/20 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 60/60 (100.0%) | - | - | - | 0/60 (0.0%) | - | 60/60 (100.0%) | 60/60 (100.0%) |
| control | 24 | 24/24 (100.0%) | - | - | - | 0/24 (0.0%) | - | 24/24 (100.0%) | 24/24 (100.0%) |
| inexpressible | 24 | 20/24 (83.3%) | 1/24 (4.2%) | 1/24 | 1/24 | - | 20/20 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 34,452 | 4,223 | 451,861 | 62,659 | $1.5025 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| and | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| branching | 60 | 60/60 (100.0%) | 60/60 (100.0%) | 60/60 (100.0%) | 60/60 (100.0%) | 60/60 (100.0%) |
| communicate | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| conditional_wording | 24 | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 24/24 (100.0%) | 18/24 (75.0%) |
| handover | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| holding | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| inspect | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| nested | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| no_branch_needed | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object_choice | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
| pre | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 6/12 (50.0%) |
| region | 27 | 27/27 (100.0%) | 27/27 (100.0%) | 27/27 (100.0%) | 27/27 (100.0%) | 21/27 (77.8%) |
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
| valid_first | 82/84 | 82/84 (97.6%) |
| valid_after_repair | 84/84 | 84/84 (100.0%) |
| goal_correct | 80/84 | 80/84 (95.2%) |
| plan_succeeds | 84/84 | 84/84 (100.0%) |
| request_satisfied | 80/84 | 80/84 (95.2%) |
| goal_exact (triples equal, before N6) | 12/84 | 12/84 (14.3%) |
| checks_ok (tasks with optional checks) | 18/21 | 18/21 (85.7%) |
| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) | 1/60 | 1/60 (1.7%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 105/108 (97.2%) | 1/24 (4.2%) | 1/24 | 1/24 | 0/84 (0.0%) | 21/21 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| branching | 60 | 60/60 (100.0%) | - | - | - | 0/60 (0.0%) | - | 59/60 (98.3%) | 60/60 (100.0%) |
| control | 24 | 24/24 (100.0%) | - | - | - | 0/24 (0.0%) | - | 21/24 (87.5%) | 24/24 (100.0%) |
| inexpressible | 24 | 21/24 (87.5%) | 1/24 (4.2%) | 1/24 | 1/24 | - | 21/21 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 36,017 | 4,223 | 460,307 | 60,382 | $0.7785 |

Most common diagnostic codes (all rounds):

| code | seen |
|---|---:|
| E212 | 2 |

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| and | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| branching | 60 | 60/60 (100.0%) | 60/60 (100.0%) | 59/60 (98.3%) | 60/60 (100.0%) | 59/60 (98.3%) |
| communicate | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| conditional_wording | 24 | 22/24 (91.7%) | 24/24 (100.0%) | 21/24 (87.5%) | 24/24 (100.0%) | 21/24 (87.5%) |
| handover | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| holding | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| inspect | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 2/3 (66.7%) | 3/3 (100.0%) | 2/3 (66.7%) |
| nested | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| no_branch_needed | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| object_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| object_choice | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
| pre | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 9/12 (75.0%) | 12/12 (100.0%) | 9/12 (75.0%) |
| region | 27 | 25/27 (92.6%) | 27/27 (100.0%) | 24/27 (88.9%) | 27/27 (100.0%) | 24/27 (88.9%) |
| report_and_stop | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_at | 12 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) |
| robot_choice | 15 | 15/15 (100.0%) | 15/15 (100.0%) | 15/15 (100.0%) | 15/15 (100.0%) | 15/15 (100.0%) |
| robot_ready | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| route | 30 | 30/30 (100.0%) | 30/30 (100.0%) | 29/30 (96.7%) | 30/30 (100.0%) | 29/30 (96.7%) |
| sensor | 9 | 9/9 (100.0%) | 9/9 (100.0%) | 8/9 (88.9%) | 9/9 (100.0%) | 8/9 (88.9%) |
| wait_until | 9 | 7/9 (77.8%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) | 9/9 (100.0%) |
