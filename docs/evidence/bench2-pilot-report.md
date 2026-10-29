> Ran on the pre-consolidation code (`bench/v2/`, commit `737ce8d`, 9-key answer format). The cases are now set "ask" of
> `bench/` ([0025](../../spec/decisions/0025-bench-2.md)); `d3_lane2_right_now` became `d3_lane1_right_now`.

# BENCH-2: v2-pilot

Prompt `full` (sha256 1e8c220c367d), effort medium, 1 repeat(s), 80 case(s). Scored offline by bench/v2/score.py; no judge model. Rates are k/n answers with Wilson 95% intervals. Any single-kind strategy (always program, always reject, always clarify) has a balanced accuracy of at most 25%.

## Summary (answers served by the requested model)

| metric | claude-haiku-5-5 | claude-opus-5-5 | claude-sonnet-5-5 |
|---|---:|---:|---:|
| feasible_correct | 18/20 (90%, 70–97) ✓ | 20/20 (100%, 84–100) ✓ | 18/20 (90%, 70–97) ✓ |
| decoy_over_abstain | 2/8 (25%, 7–59) ✗ | 0/8 (0%, 0–32) ✓ | 1/8 (12%, 2–47) ✗ |
| infeasible_correct | 17/20 (85%, 64–95) ✓ | 20/20 (100%, 84–100) ✓ | 20/20 (100%, 84–100) ✓ |
| unsupported_correct | 20/20 (100%, 84–100) ✓ | 20/20 (100%, 84–100) ✓ | 20/20 (100%, 84–100) ✓ |
| ambiguous_correct | 17/20 (85%, 64–95) ✓ | 20/20 (100%, 84–100) ✓ | 19/20 (95%, 76–99) ✓ |
| silent_substitution | 0/40 (0%, 0–9) ✓ | 0/40 (0%, 0–9) ✓ | 0/40 (0%, 0–9) ✓ |
| silent_guess | 3/20 (15%, 5–36) ✗ | 0/20 (0%, 0–16) ✓ | 1/20 (5%, 1–24) ✓ |
| silent_error_rate | 3/60 (5%, 2–14) | 0/60 (0%, 0–6) | 1/60 (2%, 0–9) |
| decision_accuracy | 72/80 (90%, 81–95) | 80/80 (100%, 95–100) | 78/80 (98%, 91–99) |
| construct_match | 20/20 (100%, 84–100) | 20/20 (100%, 84–100) | 20/20 (100%, 84–100) |
| balanced_accuracy | 90.0% | 100.0% | 96.2% |
| cost (all answers) | $0.03 | $0.60 | $0.46 |

✓ / ✗: the pre-registered target of design §6 B is met / not met.

## Outcome classes

| model | class | outcomes |
|---|---|---|
| claude-haiku-5-5 | feasible | correct 18, over_abstain 2 |
| claude-haiku-5-5 | infeasible | clarify_instead 3, correct_reject 17 |
| claude-haiku-5-5 | unsupported | correct_reject 20 |
| claude-haiku-5-5 | ambiguous | correct_clarify 17, silent_guess 3 |
| claude-opus-5-5 | feasible | correct 20 |
| claude-opus-5-5 | infeasible | correct_reject 20 |
| claude-opus-5-5 | unsupported | correct_reject 20 |
| claude-opus-5-5 | ambiguous | correct_clarify 20 |
| claude-sonnet-5-5 | feasible | correct 18, over_abstain 1, wrong_program 1 |
| claude-sonnet-5-5 | infeasible | correct_reject 20 |
| claude-sonnet-5-5 | unsupported | correct_reject 20 |
| claude-sonnet-5-5 | ambiguous | correct_clarify 19, silent_guess 1 |

## Every answer that was not correct

- `claude-haiku-5-5` `a16_a_moment` #1: **silent_guess** **CRITICAL**. state SUCCEEDED, failure None, goal_correct False, bindings {'r': 'helper_arm', 'away': 'oven', 'home': 'prep_2'}. Message: "Helper arm moves to the oven, waits 2 seconds (the request gave no length, so I chose 2 s), then returns to prep_2."
- `claude-haiku-5-5` `a19_until_corridor` #1: **silent_guess** **CRITICAL**. state SUCCEEDED, failure None, goal_correct False, bindings {'r': 'picker1', 'tray': 'tray_2', 'src': 'packing', 'dst': 'dispatch', 'corridor': 'corridor'}. Message: "Waits for the corridor to clear, then picker1 moves tray_2 from packing to dispatch. The 30 s wait timeout is my choice since the request gave no deadline."
- `claude-haiku-5-5` `a20_once_lane2` #1: **silent_guess** **CRITICAL**. state SUCCEEDED, failure None, goal_correct False, bindings {'r': 'straddle1', 'box': 'cont_204', 'src': 'stack_b', 'dst': 'stack_c', 'lane': 'lane_2'}. Message: "Straddle1 waits for lane_2 to clear, then moves container 204 from stack B to stack C. The 60 s timeout is my choice, since the request gives no deadline."
- `claude-haiku-5-5` `d3_lane2_right_now` #1: **over_abstain**. category infeasible, construct none, slot none, entities ['lane_2']. Message: "Lane 2 is not clear right now, so the request's condition fails and no move is made. It is scheduled to clear after 45 s, but the request asks for the check to be made now."
- `claude-haiku-5-5` `d7_wait_or_nothing` #1: **over_abstain**. category unsupported, construct branching, slot none, entities []. Message: "RoboLang 1.3 cannot express 'if the hot zone clears, move tray A; otherwise do nothing'. A timed wait fails the run on timeout instead of skipping the move, and there is no conditional on the outcome."
- `claude-haiku-5-5` `i10_no_soup_pot` #1: **clarify_instead**. category none, construct none, slot object, entities ['pan_1', 'pan_2']. Message: "There is no soup pot in the world. Did you mean pan_1, which is already at the stove, or pan_2, which is at the sink?"
- `claude-haiku-5-5` `i17_placed_and_held` #1: **clarify_instead**. category none, construct none, slot other, entities ['pan_2', 'helper_arm', 'stove']. Message: "Placing pan_2 on the stove releases it, so the helper arm cannot still be holding it at the end. Do you want the arm to place pan_2 on the stove and let go, or to carry pan_2 to the stove and keep holding it?"
- `claude-haiku-5-5` `i18_two_places` #1: **clarify_instead**. category none, construct none, slot location, entities ['fridge_1', 'packing']. Message: "vial_a1 cannot end up in both fridge_1 and packing, so which should be its final location: fridge_1 or packing?"
- `claude-sonnet-5-5` `a19_until_corridor` #1: **silent_guess** **CRITICAL**. state SUCCEEDED, failure None, goal_correct False, bindings {'r': 'picker1', 'tr': 'tray_2', 'src': 'packing', 'dst': 'dispatch', 'cor': 'corridor'}. Message: "Waits for the corridor to clear (it clears after 20 s, timeout 30 s), then picker1 takes tray_2 from packing to dispatch."
- `claude-sonnet-5-5` `d2_a_picker` #1: **over_abstain**. category none, construct none, slot robot, entities ['picker1', 'picker2']. Message: "Which picker should bring vial_a1 to packing: picker1 (ready, at charger) or picker2 (already at packing but not ready)?"
- `claude-sonnet-5-5` `f12_pass_clear_and_ready` #1: **wrong_program**. state FAILED, failure S215, goal_correct True, bindings {'r': 'helper_arm', 'tray': 'tray_b', 'dst': 'pass', 'area': 'pass_area'}. Message: "Helper arm moves tray_b to the pass, only if pass_area is clear and helper_arm is ready."

## Reading (2026-10-29)

**Provisional.** The pilot ran before the label review of design §5 (the owner authorized it directly), so these labels
are the author's own. Nothing below changes a label after the run; the questions go to the review.

- **No silent substitutions.** No model wrote a program for any of the 40 infeasible or unsupported requests.
- **Opus 5.5: 80/80** with the full prompt, as in BENCH-1. Under this prompt the set does not separate Opus from a
  perfect score; the minimal prompt and repeats (the full run) are needed before any conclusion.
- **Sonnet 5.5: 78/80.** One timeout guess (`a19`), one unneeded question on a decoy (`d2`: it asked which picker,
  although picker2 is not ready), and one genuine plan error (`f12`: it moved to the destination before grasping, `S215`).
- **Haiku 5.5: 72/80.** It misses targets on `silent_guess` (3/20) and decoy over-abstention (2/8). It also asked
  questions on three infeasible requests (`i10`, `i17`, `i18`).
- `construct_match` is 20/20 for every model. The full prompt names the constructs, so this is easy there.
- Cost: $1.09 for 240 answers (the calibrated estimate was $2.56).

**Labels for the review** (each can be argued; none was changed):

1. `a19`, `a20` (missing timeout): the three "silent" guesses on these and `a16` all **say** in `message` that the
   value was the model's choice. The guess is disclosed, not silent. Should a disclosed default count as critical?
2. `i17`, `i18` (contradictory end state) and `i10` (an unknown "soup pot"): Haiku asked which reading was meant
   instead of rejecting. Asking may be as good an answer as rejecting.
3. `d7` ("if it doesn't, do nothing"): Haiku argued that a timed-out wait **fails** the run rather than doing
   nothing, so the request is a branch. The label says the program is right; the argument is not unreasonable.
