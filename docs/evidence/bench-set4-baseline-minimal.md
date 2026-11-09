# RoboLang authoring benchmark: set4-minimal

Effort medium, 3 repeat(s), 16 task(s), prompt sha256 21180f513111, refusal fallbacks enabled (Claude Haiku 5.5 has none).
Task set failure, prompt minimal.
Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.

## Summary (answers served by the requested model)

| model | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied | cost |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 48 | 11/12 (91.7%) | 12/12 (100.0%) | 10/12 (83.3%) | 7/12 (58.3%) | 7/12 (58.3%) | $0.0516 |
| claude-opus-5-5 | 48 | 12/12 (100.0%) | 12/12 (100.0%) | 9/12 (75.0%) | 6/12 (50.0%) | 6/12 (50.0%) | $0.7911 |
| claude-sonnet-5-5 | 48 | 11/12 (91.7%) | 12/12 (100.0%) | 9/12 (75.0%) | 7/12 (58.3%) | 7/12 (58.3%) | $0.3827 |

The program measures are over the 12 task(s) that expect a program (the others expect a decline or a question). Robustness, per model:

| model | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 48 | 21/48 (43.8%) | 25/36 (69.4%) | 25/36 | 4/36 | 0/12 (0.0%) | 9/9 (100.0%) |
| claude-opus-5-5 | 48 | 28/48 (58.3%) | 20/36 (55.6%) | 20/36 | 6/36 | 0/12 (0.0%) | 16/16 (100.0%) |
| claude-sonnet-5-5 | 48 | 25/48 (52.1%) | 23/36 (63.9%) | 23/36 | 3/36 | 0/12 (0.0%) | 13/13 (100.0%) |

## claude-haiku-5-5

48 answer(s) served by claude-haiku-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 11/12 | 11/12 (91.7%) |
| valid_after_repair | 12/12 | 12/12 (100.0%) |
| goal_correct | 10/12 | 10/12 (83.3%) |
| plan_succeeds | 7/12 | 7/12 (58.3%) |
| request_satisfied | 7/12 | 7/12 (58.3%) |
| goal_exact (triples equal, before N6) | 0/12 | 0/12 (0.0%) |
| checks_ok (tasks with optional checks) | 0/0 | - |
| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) | 5/12 | 5/12 (41.7%) |
| unsafe_continue (a wait forced to time out, still SUCCEEDED; answers that wait) | 0/14 | 0/14 (0.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 21/48 (43.8%) | 25/36 (69.4%) | 25/36 | 4/36 | 0/12 (0.0%) | 9/9 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| control | 12 | 12/12 (100.0%) | - | - | - | 0/12 (0.0%) | - | 10/12 (83.3%) | 7/12 (58.3%) |
| inexpressible | 12 | 7/12 (58.3%) | 3/12 (25.0%) | 3/12 | 3/12 | - | 7/7 (100.0%) | - | - |
| recover | 24 | 2/24 (8.3%) | 22/24 (91.7%) | 22/24 | 1/24 | - | 2/2 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 18,906 | 4,223 | 206,927 | 94,211 | $0.0516 |

Most common diagnostic codes (all rounds):

| code | seen |
|---|---:|
| E303 | 2 |
| E305 | 1 |

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| grasp | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 0/3 (0.0%) | 0/3 (0.0%) |
| precondition | 3 | 2/3 (66.7%) | 3/3 (100.0%) | 1/3 (33.3%) | 1/3 (33.3%) | 1/3 (33.3%) |
| wait | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |

## claude-opus-5-5

48 answer(s) served by claude-opus-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 12/12 | 12/12 (100.0%) |
| valid_after_repair | 12/12 | 12/12 (100.0%) |
| goal_correct | 9/12 | 9/12 (75.0%) |
| plan_succeeds | 6/12 | 6/12 (50.0%) |
| request_satisfied | 6/12 | 6/12 (50.0%) |
| goal_exact (triples equal, before N6) | 0/12 | 0/12 (0.0%) |
| checks_ok (tasks with optional checks) | 0/0 | - |
| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) | 6/12 | 6/12 (50.0%) |
| unsafe_continue (a wait forced to time out, still SUCCEEDED; answers that wait) | 0/9 | 0/9 (0.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 28/48 (58.3%) | 20/36 (55.6%) | 20/36 | 6/36 | 0/12 (0.0%) | 16/16 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| control | 12 | 12/12 (100.0%) | - | - | - | 0/12 (0.0%) | - | 9/12 (75.0%) | 6/12 (50.0%) |
| inexpressible | 12 | 6/12 (50.0%) | 6/12 (50.0%) | 6/12 | 6/12 | - | 6/6 (100.0%) | - | - |
| recover | 24 | 10/24 (41.7%) | 14/24 (58.3%) | 14/24 | 0/24 | - | 10/10 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 18,259 | 4,223 | 202,704 | 32,820 | $0.7911 |

Most common diagnostic codes (all rounds):

| code | seen |
|---|---:|
| E303 | 2 |

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| grasp | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 0/3 (0.0%) | 0/3 (0.0%) |
| precondition | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 0/3 (0.0%) | 0/3 (0.0%) | 0/3 (0.0%) |
| wait | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |

## claude-sonnet-5-5

48 answer(s) served by claude-sonnet-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 11/12 | 11/12 (91.7%) |
| valid_after_repair | 12/12 | 12/12 (100.0%) |
| goal_correct | 9/12 | 9/12 (75.0%) |
| plan_succeeds | 7/12 | 7/12 (58.3%) |
| request_satisfied | 7/12 | 7/12 (58.3%) |
| goal_exact (triples equal, before N6) | 0/12 | 0/12 (0.0%) |
| checks_ok (tasks with optional checks) | 0/0 | - |
| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) | 5/12 | 5/12 (41.7%) |
| unsafe_continue (a wait forced to time out, still SUCCEEDED; answers that wait) | 0/18 | 0/18 (0.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 25/48 (52.1%) | 23/36 (63.9%) | 23/36 | 3/36 | 0/12 (0.0%) | 13/13 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| control | 12 | 12/12 (100.0%) | - | - | - | 0/12 (0.0%) | - | 9/12 (75.0%) | 7/12 (58.3%) |
| inexpressible | 12 | 9/12 (75.0%) | 3/12 (25.0%) | 3/12 | 3/12 | - | 9/9 (100.0%) | - | - |
| recover | 24 | 4/24 (16.7%) | 20/24 (83.3%) | 20/24 | 0/24 | - | 4/4 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 18,074 | 4,223 | 202,704 | 29,542 | $0.3827 |

Most common diagnostic codes (all rounds):

| code | seen |
|---|---:|
| E212 | 1 |

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| grasp | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 1/3 (33.3%) | 1/3 (33.3%) |
| precondition | 3 | 2/3 (66.7%) | 3/3 (100.0%) | 0/3 (0.0%) | 0/3 (0.0%) | 0/3 (0.0%) |
| wait | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
