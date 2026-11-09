# RoboLang authoring benchmark: set4

Effort medium, 3 repeat(s), 16 task(s), prompt sha256 85894a6df4d1, refusal fallbacks enabled (Claude Haiku 5.5 has none).
Task set failure, prompt full.
Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.

## Summary (answers served by the requested model)

| model | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied | cost |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 48 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | $0.0248 |
| claude-opus-5-5 | 48 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | $0.5141 |
| claude-sonnet-5-5 | 48 | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | 12/12 (100.0%) | $0.2878 |

The program measures are over the 12 task(s) that expect a program (the others expect a decline or a question). Robustness, per model:

| model | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 48 | 37/48 (77.1%) | 11/36 (30.6%) | 11/36 | 1/36 | 0/12 (0.0%) | 25/25 (100.0%) |
| claude-opus-5-5 | 48 | 35/48 (72.9%) | 13/36 (36.1%) | 13/36 | 2/36 | 0/12 (0.0%) | 23/23 (100.0%) |
| claude-sonnet-5-5 | 48 | 40/48 (83.3%) | 8/36 (22.2%) | 8/36 | 1/36 | 0/12 (0.0%) | 28/28 (100.0%) |

## claude-haiku-5-5

48 answer(s) served by claude-haiku-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 12/12 | 12/12 (100.0%) |
| valid_after_repair | 12/12 | 12/12 (100.0%) |
| goal_correct | 12/12 | 12/12 (100.0%) |
| plan_succeeds | 12/12 | 12/12 (100.0%) |
| request_satisfied | 12/12 | 12/12 (100.0%) |
| goal_exact (triples equal, before N6) | 0/12 | 0/12 (0.0%) |
| checks_ok (tasks with optional checks) | 0/0 | - |
| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) | 0/12 | 0/12 (0.0%) |
| unsafe_continue (a wait forced to time out, still SUCCEEDED; answers that wait) | 0/11 | 0/11 (0.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 37/48 (77.1%) | 11/36 (30.6%) | 11/36 | 1/36 | 0/12 (0.0%) | 25/25 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| control | 12 | 12/12 (100.0%) | - | - | - | 0/12 (0.0%) | - | 12/12 (100.0%) | 12/12 (100.0%) |
| inexpressible | 12 | 11/12 (91.7%) | 1/12 (8.3%) | 1/12 | 1/12 | - | 11/11 (100.0%) | - | - |
| recover | 24 | 14/24 (58.3%) | 10/24 (41.7%) | 10/24 | 0/24 | - | 14/14 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 17,328 | 0 | 285,072 | 40,453 | $0.0248 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| grasp | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| precondition | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| wait | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |

## claude-opus-5-5

48 answer(s) served by claude-opus-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 12/12 | 12/12 (100.0%) |
| valid_after_repair | 12/12 | 12/12 (100.0%) |
| goal_correct | 12/12 | 12/12 (100.0%) |
| plan_succeeds | 12/12 | 12/12 (100.0%) |
| request_satisfied | 12/12 | 12/12 (100.0%) |
| goal_exact (triples equal, before N6) | 0/12 | 0/12 (0.0%) |
| checks_ok (tasks with optional checks) | 0/0 | - |
| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) | 0/12 | 0/12 (0.0%) |
| unsafe_continue (a wait forced to time out, still SUCCEEDED; answers that wait) | 0/9 | 0/9 (0.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 35/48 (72.9%) | 13/36 (36.1%) | 13/36 | 2/36 | 0/12 (0.0%) | 23/23 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| control | 12 | 12/12 (100.0%) | - | - | - | 0/12 (0.0%) | - | 12/12 (100.0%) | 12/12 (100.0%) |
| inexpressible | 12 | 10/12 (83.3%) | 2/12 (16.7%) | 2/12 | 2/12 | - | 10/10 (100.0%) | - | - |
| recover | 24 | 13/24 (54.2%) | 11/24 (45.8%) | 11/24 | 0/24 | - | 13/13 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 17,328 | 5,939 | 279,133 | 17,965 | $0.5141 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| grasp | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| precondition | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| wait | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |

## claude-sonnet-5-5

48 answer(s) served by claude-sonnet-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 12/12 | 12/12 (100.0%) |
| valid_after_repair | 12/12 | 12/12 (100.0%) |
| goal_correct | 12/12 | 12/12 (100.0%) |
| plan_succeeds | 12/12 | 12/12 (100.0%) |
| request_satisfied | 12/12 | 12/12 (100.0%) |
| goal_exact (triples equal, before N6) | 0/12 | 0/12 (0.0%) |
| checks_ok (tasks with optional checks) | 0/0 | - |
| half_program (valid, SUCCEEDED as given, wrong on another path; branching tasks) | 0/12 | 0/12 (0.0%) |
| unsafe_continue (a wait forced to time out, still SUCCEEDED; answers that wait) | 0/8 | 0/8 (0.0%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 40/48 (83.3%) | 8/36 (22.2%) | 8/36 | 1/36 | 0/12 (0.0%) | 28/28 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| control | 12 | 12/12 (100.0%) | - | - | - | 0/12 (0.0%) | - | 12/12 (100.0%) | 12/12 (100.0%) |
| inexpressible | 12 | 11/12 (91.7%) | 1/12 (8.3%) | 1/12 | 1/12 | - | 11/11 (100.0%) | - | - |
| recover | 24 | 17/24 (70.8%) | 7/24 (29.2%) | 7/24 | 0/24 | - | 17/17 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 17,328 | 0 | 285,072 | 19,611 | $0.2878 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| grasp | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| precondition | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| wait | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
