# RoboLang authoring benchmark: ask-frozen-validation-20261029-funded

Effort medium, 1 repeat(s), 80 task(s), prompt sha256 744874a8696f, refusal fallbacks enabled (Claude Haiku 5.5 has none).
Task set ask, prompt full.
Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.

## Summary (answers served by the requested model)

| model | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied | cost |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 80 | 22/23 (95.7%) | 22/23 (95.7%) | 22/23 (95.7%) | 22/23 (95.7%) | 22/23 (95.7%) | $0.0289 |
| claude-opus-5-5 | 80 | 21/23 (91.3%) | 21/23 (91.3%) | 21/23 (91.3%) | 21/23 (91.3%) | 21/23 (91.3%) | $0.6376 |
| claude-sonnet-5-5 | 80 | 21/23 (91.3%) | 21/23 (91.3%) | 21/23 (91.3%) | 21/23 (91.3%) | 21/23 (91.3%) | $0.3138 |

The program measures are over the 23 task(s) that expect a program (the others expect a decline or a question). Robustness, per model:

| model | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | silent_guess | unjustified_default | entities_correct |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 80 | 77/80 (96.2%) | 0/39 (0.0%) | 0/39 | 0/39 | 1/23 (4.3%) | 38/39 (97.4%) | 1/18 (5.6%) | 1/18 (5.6%) | 30/30 (100.0%) |
| claude-opus-5-5 | 80 | 78/80 (97.5%) | 0/39 (0.0%) | 0/39 | 0/39 | 2/23 (8.7%) | 38/39 (97.4%) | 0/18 (0.0%) | 0/18 (0.0%) | 31/31 (100.0%) |
| claude-sonnet-5-5 | 80 | 78/80 (97.5%) | 0/39 (0.0%) | 0/39 | 0/39 | 2/23 (8.7%) | 38/39 (97.4%) | 0/18 (0.0%) | 0/18 (0.0%) | 31/31 (100.0%) |

## claude-haiku-5-5

80 answer(s) served by claude-haiku-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 22/23 | 22/23 (95.7%) |
| valid_after_repair | 22/23 | 22/23 (95.7%) |
| goal_correct | 22/23 | 22/23 (95.7%) |
| plan_succeeds | 22/23 | 22/23 (95.7%) |
| request_satisfied | 22/23 | 22/23 (95.7%) |
| goal_exact (triples equal, before N6) | 22/23 | 22/23 (95.7%) |
| checks_ok (tasks with optional checks) | 22/23 | 22/23 (95.7%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | silent_guess | unjustified_default | entities_correct |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 77/80 (96.2%) | 0/39 (0.0%) | 0/39 | 0/39 | 1/23 (4.3%) | 38/39 (97.4%) | 1/18 (5.6%) | 1/18 (5.6%) | 30/30 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | silent_guess | unjustified_default | entities_correct | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| clarify | 18 | 16/18 (88.9%) | - | - | - | - | - | 1/18 (5.6%) | 1/18 (5.6%) | 11/11 (100.0%) | - | - |
| decoy | 11 | 10/11 (90.9%) | - | - | - | 1/11 (9.1%) | - | - | - | - | 10/11 (90.9%) | 10/11 (90.9%) |
| feasible | 12 | 12/12 (100.0%) | - | - | - | 0/12 (0.0%) | - | - | - | - | 12/12 (100.0%) | 12/12 (100.0%) |
| inexpressible | 20 | 20/20 (100.0%) | 0/20 (0.0%) | 0/20 | 0/20 | - | 19/20 (95.0%) | - | - | - | - | - |
| infeasible | 19 | 19/19 (100.0%) | 0/19 (0.0%) | 0/19 | 0/19 | - | 19/19 (100.0%) | - | - | 19/19 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 38,127 | 4,619 | 364,901 | 41,629 | $0.0289 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| and | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| capability | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| communicate | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| handover | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| holding | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| inspect | 4 | 4/4 (100.0%) | 4/4 (100.0%) | 4/4 (100.0%) | 4/4 (100.0%) | 4/4 (100.0%) |
| long_plan | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| looks_ambiguous | 5 | 4/5 (80.0%) | 4/5 (80.0%) | 4/5 (80.0%) | 4/5 (80.0%) | 4/5 (80.0%) |
| looks_infeasible | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| looks_unsupported | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| multiple_objects | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| or | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| pick_place | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| precondition | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| referent_by_location | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| region | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| relative_speed | 1 | 0/1 (0.0%) | 0/1 (0.0%) | 0/1 (0.0%) | 0/1 (0.0%) | 0/1 (0.0%) |
| robot_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| safety | 3 | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) |
| scheduled_clearance | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| sensor_ok | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| timeout | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| two_robot | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| units_in_words | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| wait_for | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| wait_until | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |

## claude-opus-5-5

80 answer(s) served by claude-opus-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 21/23 | 21/23 (91.3%) |
| valid_after_repair | 21/23 | 21/23 (91.3%) |
| goal_correct | 21/23 | 21/23 (91.3%) |
| plan_succeeds | 21/23 | 21/23 (91.3%) |
| request_satisfied | 21/23 | 21/23 (91.3%) |
| goal_exact (triples equal, before N6) | 21/23 | 21/23 (91.3%) |
| checks_ok (tasks with optional checks) | 21/23 | 21/23 (91.3%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | silent_guess | unjustified_default | entities_correct |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 78/80 (97.5%) | 0/39 (0.0%) | 0/39 | 0/39 | 2/23 (8.7%) | 38/39 (97.4%) | 0/18 (0.0%) | 0/18 (0.0%) | 31/31 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | silent_guess | unjustified_default | entities_correct | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| clarify | 18 | 18/18 (100.0%) | - | - | - | - | - | 0/18 (0.0%) | 0/18 (0.0%) | 12/12 (100.0%) | - | - |
| decoy | 11 | 9/11 (81.8%) | - | - | - | 2/11 (18.2%) | - | - | - | - | 9/11 (81.8%) | 9/11 (81.8%) |
| feasible | 12 | 12/12 (100.0%) | - | - | - | 0/12 (0.0%) | - | - | - | - | 12/12 (100.0%) | 12/12 (100.0%) |
| inexpressible | 20 | 20/20 (100.0%) | 0/20 (0.0%) | 0/20 | 0/20 | - | 19/20 (95.0%) | - | - | - | - | - |
| infeasible | 19 | 19/19 (100.0%) | 0/19 (0.0%) | 0/19 | 0/19 | - | 19/19 (100.0%) | - | - | 19/19 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 38,127 | 4,619 | 364,901 | 19,449 | $0.6376 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| and | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| capability | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| communicate | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| handover | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| holding | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| inspect | 4 | 4/4 (100.0%) | 4/4 (100.0%) | 4/4 (100.0%) | 4/4 (100.0%) | 4/4 (100.0%) |
| long_plan | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| looks_ambiguous | 5 | 4/5 (80.0%) | 4/5 (80.0%) | 4/5 (80.0%) | 4/5 (80.0%) | 4/5 (80.0%) |
| looks_infeasible | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| looks_unsupported | 3 | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) |
| multiple_objects | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| or | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| pick_place | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| precondition | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| referent_by_location | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| region | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| relative_speed | 1 | 0/1 (0.0%) | 0/1 (0.0%) | 0/1 (0.0%) | 0/1 (0.0%) | 0/1 (0.0%) |
| robot_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| safety | 3 | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) |
| scheduled_clearance | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| sensor_ok | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| timeout | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| two_robot | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| units_in_words | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| wait_for | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| wait_until | 6 | 5/6 (83.3%) | 5/6 (83.3%) | 5/6 (83.3%) | 5/6 (83.3%) | 5/6 (83.3%) |

## claude-sonnet-5-5

80 answer(s) served by claude-sonnet-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 21/23 | 21/23 (91.3%) |
| valid_after_repair | 21/23 | 21/23 (91.3%) |
| goal_correct | 21/23 | 21/23 (91.3%) |
| plan_succeeds | 21/23 | 21/23 (91.3%) |
| request_satisfied | 21/23 | 21/23 (91.3%) |
| goal_exact (triples equal, before N6) | 21/23 | 21/23 (91.3%) |
| checks_ok (tasks with optional checks) | 21/23 | 21/23 (91.3%) |

Robustness (decision judged on the first answer; the program measures above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | silent_guess | unjustified_default | entities_correct |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 78/80 (97.5%) | 0/39 (0.0%) | 0/39 | 0/39 | 2/23 (8.7%) | 38/39 (97.4%) | 0/18 (0.0%) | 0/18 (0.0%) | 31/31 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | silent_guess | unjustified_default | entities_correct | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| clarify | 18 | 18/18 (100.0%) | - | - | - | - | - | 0/18 (0.0%) | 0/18 (0.0%) | 12/12 (100.0%) | - | - |
| decoy | 11 | 9/11 (81.8%) | - | - | - | 2/11 (18.2%) | - | - | - | - | 9/11 (81.8%) | 9/11 (81.8%) |
| feasible | 12 | 12/12 (100.0%) | - | - | - | 0/12 (0.0%) | - | - | - | - | 12/12 (100.0%) | 12/12 (100.0%) |
| inexpressible | 20 | 20/20 (100.0%) | 0/20 (0.0%) | 0/20 | 0/20 | - | 19/20 (95.0%) | - | - | - | - | - |
| infeasible | 19 | 19/19 (100.0%) | 0/19 (0.0%) | 0/19 | 0/19 | - | 19/19 (100.0%) | - | - | 19/19 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 38,127 | 4,619 | 364,901 | 15,305 | $0.3138 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | request_satisfied |
|---|---:|---:|---:|---:|---:|---:|
| and | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| capability | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| communicate | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| handover | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| holding | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| inspect | 4 | 4/4 (100.0%) | 4/4 (100.0%) | 4/4 (100.0%) | 4/4 (100.0%) | 4/4 (100.0%) |
| long_plan | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| looks_ambiguous | 5 | 4/5 (80.0%) | 4/5 (80.0%) | 4/5 (80.0%) | 4/5 (80.0%) | 4/5 (80.0%) |
| looks_infeasible | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| looks_unsupported | 3 | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) |
| multiple_objects | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| or | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| pick_place | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| precondition | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| referent_by_location | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| region | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| relative_speed | 1 | 0/1 (0.0%) | 0/1 (0.0%) | 0/1 (0.0%) | 0/1 (0.0%) | 0/1 (0.0%) |
| robot_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| safety | 3 | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) |
| scheduled_clearance | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| sensor_ok | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| timeout | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| two_robot | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| units_in_words | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| wait_for | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| wait_until | 6 | 5/6 (83.3%) | 5/6 (83.3%) | 5/6 (83.3%) | 5/6 (83.3%) | 5/6 (83.3%) |
