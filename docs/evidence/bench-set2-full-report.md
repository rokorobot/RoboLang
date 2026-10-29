# RoboLang authoring benchmark: hard-full

Effort medium, 1 repeat(s), 40 task(s), prompt sha256 3776dd1df690, refusal fallbacks enabled (Claude Haiku 5.5 has none).
Task set hard, prompt full.
Scored offline by bench/score.py from the saved answers; no judge model. Rates are k/n answers.

## Summary (answers served by the requested model)

| model | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds | cost |
|---|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 40 | 22/22 (100.0%) | 22/22 (100.0%) | 22/22 (100.0%) | 22/22 (100.0%) | $0.0172 |
| claude-opus-5-5 | 40 | 22/22 (100.0%) | 22/22 (100.0%) | 20/22 (90.9%) | 22/22 (100.0%) | $0.3691 |
| claude-sonnet-5-5 | 40 | 21/22 (95.5%) | 21/22 (95.5%) | 21/22 (95.5%) | 20/22 (90.9%) | $0.1879 |

The four rates are over the 22 task(s) that expect a program (the others expect a decline). Robustness, per model:

| model | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---|---:|---:|---:|---:|---:|---:|---:|
| claude-haiku-5-5 | 40 | 39/40 (97.5%) | 1/18 (5.6%) | 1/18 | 1/18 | 0/22 (0.0%) | 17/17 (100.0%) |
| claude-opus-5-5 | 40 | 40/40 (100.0%) | 0/18 (0.0%) | 0/18 | 0/18 | 0/22 (0.0%) | 18/18 (100.0%) |
| claude-sonnet-5-5 | 40 | 39/40 (97.5%) | 0/18 (0.0%) | 0/18 | 0/18 | 1/22 (4.5%) | 18/18 (100.0%) |

## claude-haiku-5-5

40 answer(s) served by claude-haiku-5-5.

| measure | answers | rate |
|---|---:|---:|
| valid_first | 22/22 | 22/22 (100.0%) |
| valid_after_repair | 22/22 | 22/22 (100.0%) |
| goal_correct | 22/22 | 22/22 (100.0%) |
| plan_succeeds | 22/22 | 22/22 (100.0%) |
| checks_ok (tasks with optional checks) | 10/10 | 10/10 (100.0%) |

Robustness (decision judged on the first answer; the four rates above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 39/40 (97.5%) | 1/18 (5.6%) | 1/18 | 1/18 | 0/22 (0.0%) | 17/17 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ambiguous | 6 | 6/6 (100.0%) | - | - | - | 0/6 (0.0%) | - | 6/6 (100.0%) | 6/6 (100.0%) |
| control | 8 | 8/8 (100.0%) | - | - | - | 0/8 (0.0%) | - | 8/8 (100.0%) | 8/8 (100.0%) |
| decoy | 8 | 8/8 (100.0%) | - | - | - | 0/8 (0.0%) | - | 8/8 (100.0%) | 8/8 (100.0%) |
| inexpressible | 10 | 9/10 (90.0%) | 1/10 (10.0%) | 1/10 | 1/10 | - | 9/9 (100.0%) | - | - |
| infeasible | 8 | 8/8 (100.0%) | 0/8 (0.0%) | 0/8 | 0/8 | - | 8/8 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 22,222 | 3,602 | 144,080 | 26,201 | $0.0172 |

Most common diagnostic codes (all rounds):

| code | seen |
|---|---:|
| E212 | 1 |

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|
| ambiguous | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
| and | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| communicate | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| control | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) |
| decoy_names | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) |
| description | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) |
| handover | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| holding | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| inspect | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| location_choice | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| long_plan | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| multi_condition | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| multiple_objects | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| not | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| object_choice | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| or | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| pick_place | 13 | 13/13 (100.0%) | 13/13 (100.0%) | 13/13 (100.0%) | 13/13 (100.0%) |
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
| goal_correct | 20/22 | 20/22 (90.9%) |
| plan_succeeds | 22/22 | 22/22 (100.0%) |
| checks_ok (tasks with optional checks) | 10/10 | 10/10 (100.0%) |

Robustness (decision judged on the first answer; the four rates above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 40/40 (100.0%) | 0/18 (0.0%) | 0/18 | 0/18 | 0/22 (0.0%) | 18/18 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ambiguous | 6 | 6/6 (100.0%) | - | - | - | 0/6 (0.0%) | - | 4/6 (66.7%) | 6/6 (100.0%) |
| control | 8 | 8/8 (100.0%) | - | - | - | 0/8 (0.0%) | - | 8/8 (100.0%) | 8/8 (100.0%) |
| decoy | 8 | 8/8 (100.0%) | - | - | - | 0/8 (0.0%) | - | 8/8 (100.0%) | 8/8 (100.0%) |
| inexpressible | 10 | 10/10 (100.0%) | 0/10 (0.0%) | 0/10 | 0/10 | - | 10/10 (100.0%) | - | - |
| infeasible | 8 | 8/8 (100.0%) | 0/8 (0.0%) | 0/8 | 0/8 | - | 8/8 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 21,424 | 3,602 | 140,478 | 11,865 | $0.3691 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|
| ambiguous | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 4/6 (66.7%) | 6/6 (100.0%) |
| and | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| communicate | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| control | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) |
| decoy_names | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) |
| description | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) |
| handover | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| holding | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| inspect | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| location_choice | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 2/3 (66.7%) | 3/3 (100.0%) |
| long_plan | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| multi_condition | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 2/3 (66.7%) | 3/3 (100.0%) |
| multiple_objects | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 2/3 (66.7%) | 3/3 (100.0%) |
| not | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| object_choice | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 0/1 (0.0%) | 1/1 (100.0%) |
| or | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| pick_place | 13 | 13/13 (100.0%) | 13/13 (100.0%) | 12/13 (92.3%) | 13/13 (100.0%) |
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
| valid_first | 21/22 | 21/22 (95.5%) |
| valid_after_repair | 21/22 | 21/22 (95.5%) |
| goal_correct | 21/22 | 21/22 (95.5%) |
| plan_succeeds | 20/22 | 20/22 (90.9%) |
| checks_ok (tasks with optional checks) | 9/10 | 9/10 (90.0%) |

Robustness (decision judged on the first answer; the four rates above are over the tasks that expect a program):

| decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) |
|---:|---:|---:|---:|---:|---:|
| 39/40 (97.5%) | 0/18 (0.0%) | 0/18 | 0/18 | 1/22 (4.5%) | 18/18 (100.0%) |

By kind of task:

| kind | answers | decision_correct | false_program | false_program after repair | of which valid and SUCCEEDED | false_refusal | category_correct (of declines) | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| ambiguous | 6 | 5/6 (83.3%) | - | - | - | 1/6 (16.7%) | - | 5/6 (83.3%) | 5/6 (83.3%) |
| control | 8 | 8/8 (100.0%) | - | - | - | 0/8 (0.0%) | - | 8/8 (100.0%) | 7/8 (87.5%) |
| decoy | 8 | 8/8 (100.0%) | - | - | - | 0/8 (0.0%) | - | 8/8 (100.0%) | 8/8 (100.0%) |
| inexpressible | 10 | 10/10 (100.0%) | 0/10 (0.0%) | 0/10 | 0/10 | - | 10/10 (100.0%) | - | - |
| infeasible | 8 | 8/8 (100.0%) | 0/8 (0.0%) | 0/8 | 0/8 | - | 8/8 (100.0%) | - | - |

Tokens and cost of all requests for these answers:

| input (uncached) | cache write | cache read | output | cost |
|---:|---:|---:|---:|---:|
| 21,424 | 3,602 | 140,478 | 10,798 | $0.1879 |

Most common diagnostic codes (all rounds):

No diagnostics.

By tag (tasks that expect a program):

| tag | answers | valid_first | valid_after_repair | goal_correct | plan_succeeds |
|---|---:|---:|---:|---:|---:|
| ambiguous | 6 | 5/6 (83.3%) | 5/6 (83.3%) | 5/6 (83.3%) | 5/6 (83.3%) |
| and | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| communicate | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 1/2 (50.0%) |
| control | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) | 7/8 (87.5%) |
| decoy_names | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) |
| description | 8 | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) | 8/8 (100.0%) |
| handover | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| holding | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| inspect | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| location_choice | 3 | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) |
| long_plan | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| multi_condition | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| multiple_objects | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| not | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| object_choice | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| or | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| pick_place | 13 | 13/13 (100.0%) | 13/13 (100.0%) | 13/13 (100.0%) | 12/13 (92.3%) |
| precondition | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| region | 5 | 5/5 (100.0%) | 5/5 (100.0%) | 5/5 (100.0%) | 5/5 (100.0%) |
| robot_at | 3 | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) | 2/3 (66.7%) |
| sensor_ok | 3 | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) | 3/3 (100.0%) |
| sensor_param | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| timeout | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
| two_robot | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) |
| unrolled_repetition | 2 | 2/2 (100.0%) | 2/2 (100.0%) | 2/2 (100.0%) | 1/2 (50.0%) |
| wait_for | 1 | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) | 1/1 (100.0%) |
| wait_until | 6 | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) | 6/6 (100.0%) |
