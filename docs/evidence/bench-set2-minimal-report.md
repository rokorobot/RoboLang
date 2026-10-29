# RoboLang authoring benchmark: hard-minimal

Effort medium, 1 repeat(s), 40 task(s), prompt sha256 4e297a05f5be, refusal fallbacks enabled (Claude Haiku 5.5 has none).
Task set hard, prompt minimal.
Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.

## Summary (answers served by the requested model)

| model | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | cost |
|---|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 40 | 21/22 (95.5%) | 21/22 (95.5%) | 20/22 (90.9%) | 21/22 (95.5%) | $0.0225 |
| claude-opus-5-5 | 40 | 22/22 (100.0%) | 22/22 (100.0%) | 19/22 (86.4%) | 22/22 (100.0%) | $0.4759 |
| claude-sonnet-5-5 | 40 | 22/22 (100.0%) | 22/22 (100.0%) | 20/22 (90.9%) | 22/22 (100.0%) | $0.2282 |

The four rates are over the 22 task(s) that expect a program (the others expect a decline). Robustness, per model:

| model | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 40 | 38/40 (95.0%) | 1/18 (5.6%) | 1/18 | 0/18 | 1/22 (4.5%) | 17/17 (100.0%) |
| claude-opus-5-5 | 40 | 38/40 (95.0%) | 2/18 (11.1%) | 2/18 | 0/18 | 0/22 (0.0%) | 16/16 (100.0%) |
| claude-sonnet-5-5 | 40 | 39/40 (97.5%) | 1/18 (5.6%) | 1/18 | 0/18 | 0/22 (0.0%) | 16/17 (94.1%) |

## claude-haiku-5-5

40 answer(s) served by claude-haiku-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 21/22 | 21/22 (95.5%) |
| valid_after_repair | 21/22 | 21/22 (95.5%) |
| goal_correct | 20/22 | 20/22 (90.9%) |
| plan_succeeds | 21/22 | 21/22 (95.5%) |
| checks_ok (tasks with optional checks) | 10/10 | 10/10 (100.0%) |

Robustness (decision judged on the first answer; the four rates above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 38/40 (95.0%) | 1/18 (5.6%) | 1/18 | 0/18 | 1/22 (4.5%) | 17/17 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ambiguous | 6 | 6/6 (100.0%) | - | - | - | 0/6 (0.0%) | - | 5/6 (83.3%) | 6/6 (100.0%) |
| control | 8 | 8/8 (100.0%) | - | - | - | 0/8 (0.0%) | - | 8/8 (100.0%) | 8/8 (100.0%) |
| decoy | 8 | 7/8 (87.5%) | - | - | - | 1/8 (12.5%) | - | 7/8 (87.5%) | 7/8 (87.5%) |
| inexpressible | 10 | 10/10 (100.0%) | 0/10 (0.0%) | 0/10 | 0/10 | - | 10/10 (100.0%) | - | - |
| infeasible | 8 | 7/8 (87.5%) | 1/8 (12.5%) | 1/8 | 0/8 | - | 7/7 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 21,424 | 3,172 | 123,708 | 37,425 | $0.0225 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|
| ambiguous | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 5/6 (83.3%) | 6/6 (100.0%) |
| and | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| communicate | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| control | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) |
| decoy_names | 8 | 7/8 (87.5%) | 7/8 (87.5%) | 7/8 (87.5%) | 7/8 (87.5%) |
| description | 8 | 7/8 (87.5%) | 7/8 (87.5%) | 7/8 (87.5%) | 7/8 (87.5%) |
| handover | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| holding | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| inspect | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| location_choice | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 2/3 (66.7%) | 3/3 (100.0%) |
| long_plan | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| multi_condition | 3 | 2/3 (66.7%) | 2/3 (66.7%) | 1/3 (33.3%) | 2/3 (66.7%) |
| multiple_objects | 3 | 2/3 (66.7%) | 2/3 (66.7%) | 1/3 (33.3%) | 2/3 (66.7%) |
| not | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| object_choice | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| or | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| pick_place | 13 | 12/13 (92.3%) | 12/13 (92.3%) | 12/13 (92.3%) | 12/13 (92.3%) |
| precondition | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| region | 5 | 5/5 (100.0%) | 5/5 (100.0%) | 5/5 (100.0%) | 5/5 (100.0%) |
| robot_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor_ok | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor_param | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| timeout | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
| two_robot | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| unrolled_repetition | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| wait_for | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| wait_until | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |

## claude-opus-5-5

40 answer(s) served by claude-opus-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 22/22 | 22/22 (100.0%) |
| valid_after_repair | 22/22 | 22/22 (100.0%) |
| goal_correct | 19/22 | 19/22 (86.4%) |
| plan_succeeds | 22/22 | 22/22 (100.0%) |
| checks_ok (tasks with optional checks) | 10/10 | 10/10 (100.0%) |

Robustness (decision judged on the first answer; the four rates above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 38/40 (95.0%) | 2/18 (11.1%) | 2/18 | 0/18 | 0/22 (0.0%) | 16/16 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ambiguous | 6 | 6/6 (100.0%) | - | - | - | 0/6 (0.0%) | - | 5/6 (83.3%) | 6/6 (100.0%) |
| control | 8 | 8/8 (100.0%) | - | - | - | 0/8 (0.0%) | - | 7/8 (87.5%) | 8/8 (100.0%) |
| decoy | 8 | 8/8 (100.0%) | - | - | - | 0/8 (0.0%) | - | 7/8 (87.5%) | 8/8 (100.0%) |
| inexpressible | 10 | 10/10 (100.0%) | 0/10 (0.0%) | 0/10 | 0/10 | - | 10/10 (100.0%) | - | - |
| infeasible | 8 | 6/8 (75.0%) | 2/8 (25.0%) | 2/8 | 0/8 | - | 6/6 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 21,424 | 3,172 | 123,708 | 17,479 | $0.4759 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|
| ambiguous | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 5/6 (83.3%) | 6/6 (100.0%) |
| and | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| communicate | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| control | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 7/8 (87.5%) | 8/8 (100.0%) |
| decoy_names | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 7/8 (87.5%) | 8/8 (100.0%) |
| description | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 7/8 (87.5%) | 8/8 (100.0%) |
| handover | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| holding | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| inspect | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| location_choice | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 2/3 (66.7%) | 3/3 (100.0%) |
| long_plan | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 0/1 (0.0%) | 1/1 (100.0%) |
| multi_condition | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 0/3 (0.0%) | 3/3 (100.0%) |
| multiple_objects | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 0/3 (0.0%) | 3/3 (100.0%) |
| not | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| object_choice | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| or | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| pick_place | 13 | 13/13 (100.0%) | 13/13 (100.0%) | 11/13 (84.6%) | 13/13 (100.0%) |
| precondition | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| region | 5 | 5/5 (100.0%) | 5/5 (100.0%) | 5/5 (100.0%) | 5/5 (100.0%) |
| robot_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor_ok | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor_param | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| timeout | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
| two_robot | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| unrolled_repetition | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| wait_for | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| wait_until | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |

## claude-sonnet-5-5

40 answer(s) served by claude-sonnet-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 22/22 | 22/22 (100.0%) |
| valid_after_repair | 22/22 | 22/22 (100.0%) |
| goal_correct | 20/22 | 20/22 (90.9%) |
| plan_succeeds | 22/22 | 22/22 (100.0%) |
| checks_ok (tasks with optional checks) | 10/10 | 10/10 (100.0%) |

Robustness (decision judged on the first answer; the four rates above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 39/40 (97.5%) | 1/18 (5.6%) | 1/18 | 0/18 | 0/22 (0.0%) | 16/17 (94.1%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ambiguous | 6 | 6/6 (100.0%) | - | - | - | 0/6 (0.0%) | - | 6/6 (100.0%) | 6/6 (100.0%) |
| control | 8 | 8/8 (100.0%) | - | - | - | 0/8 (0.0%) | - | 6/8 (75.0%) | 8/8 (100.0%) |
| decoy | 8 | 8/8 (100.0%) | - | - | - | 0/8 (0.0%) | - | 8/8 (100.0%) | 8/8 (100.0%) |
| inexpressible | 10 | 10/10 (100.0%) | 0/10 (0.0%) | 0/10 | 0/10 | - | 10/10 (100.0%) | - | - |
| infeasible | 8 | 7/8 (87.5%) | 1/8 (12.5%) | 1/8 | 0/8 | - | 6/7 (85.7%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 21,424 | 3,172 | 123,708 | 15,264 | $0.2282 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|
| ambiguous | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
| and | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| communicate | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 1/2 (50.0%) | 2/2 (100.0%) |
| control | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 6/8 (75.0%) | 8/8 (100.0%) |
| decoy_names | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) |
| description | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) |
| handover | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| holding | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| inspect | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| location_choice | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| long_plan | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 0/1 (0.0%) | 1/1 (100.0%) |
| multi_condition | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 2/3 (66.7%) | 3/3 (100.0%) |
| multiple_objects | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 2/3 (66.7%) | 3/3 (100.0%) |
| not | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| object_choice | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| or | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| pick_place | 13 | 13/13 (100.0%) | 13/13 (100.0%) | 11/13 (84.6%) | 13/13 (100.0%) |
| precondition | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| region | 5 | 5/5 (100.0%) | 5/5 (100.0%) | 5/5 (100.0%) | 5/5 (100.0%) |
| robot_at | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor_ok | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor_param | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| timeout | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
| two_robot | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| unrolled_repetition | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 1/2 (50.0%) | 2/2 (100.0%) |
| wait_for | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| wait_until | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
