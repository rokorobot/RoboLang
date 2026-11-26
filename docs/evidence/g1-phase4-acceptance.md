# G1 phase 4: the acceptance run (run 1)

Date: 2026-11-26. Code: [`backends/unitree_g1/`](../../backends/unitree_g1/) (`scenes.py`, `phase4.py`), decision
[0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md) section 3; contract in
[`g1-backend-v0.1.md`](../../spec/g1-backend-v0.1.md), "Acceptance scenes and the acceptance run". Simulation only.
Result file: [`g1-phase4/phase4-run1.json`](g1-phase4/phase4-run1.json); every run's record, result, site and scene:
[`g1-phase4/run1-records.tar.gz`](g1-phase4/run1-records.tar.gz).

**Result: the bar is not met. 37 of 42 runs ended `SUCCEEDED` (88.1%); the bar is 90%, which is 38.** The other three
parts of the bar are met: no false success, 54 of 55 atoms agree with the oracle (98.2%, bar 98%), and all five scripted
failure cases end `FAILED` with their codes. Every one of the five failed runs is a refusal or an `unknown`: the robot
declined or could not confirm, and never reported a goal that was not met. Eleven of the 14 tasks pass 3 of 3.

| Bar (0046 section 3, item 3) | Required | Run 1 | |
|---|---|---|---|
| Runs `SUCCEEDED`, from observations | ≥ 90% | 37 of 42, 88.1% | **not met, by one run** |
| Atoms agreeing with the oracle | ≥ 98% | 54 of 55, 98.2% | met |
| False successes | 0 | 0 | met |
| Scripted failure cases `FAILED` with their code | 5 of 5 | 5 of 5 | met |

Host: a Cloudroom sandbox, x86_64, 4 vCPUs, no GPU; stance `free`, arm work `supported` (0046 open point 1). The run
took 3 h 4 min. Versions are in the result file.

## 1. The tasks

Each task ran unchanged from `bench/reference/`, three times, each on a fresh stand-in through `robolang_g1.cli`.

| Task | Runs that count | What happened |
|---|---|---|
| `a_scrap_part` | 3 of 3 | |
| `a_sort_three` | **1 of 3** | 2 × S502 at the fourth `place` |
| `e_box_then_dock` | **1 of 3** | 1 × S504 and 1 × S302, both at the dock |
| `e_hold_box` | 3 of 3 | |
| `e_pause_home` | 3 of 3 | |
| `e_pick_place` | 3 of 3 | |
| `l_vial_hold` | **2 of 3** | 1 × S502 at the `place` |
| `l_vial_to_fridge` | 3 of 3 | |
| `w_part_precheck` | 3 of 3 | |
| `w_pick_from_table` | 3 of 3 | |
| `w_split_boxes` | 3 of 3 | one box in each hand |
| `e_park_amr` | 3 of 3 | |
| `w_carry_hold` | 3 of 3 | walks holding the crate (0046 U6) |
| `w_clear_dock` | 3 of 3 | walks holding the crate (0046 U6) |

The visibility rule holds for all 14 scenes, checked from the geometry before the run. No task left the family.

## 2. The five failed runs

| Run | Code | What the record says | Cause |
|---|---|---|---|
| `a_sort_three_2`, `_5` | S502 | The fourth `place` (part y in the outer right slot): no inverse-kinematics solution, the target 0.22–0.25 m to the right of the pelvis | The robot stood 2–5 cm to the left of the station, inside the 5 cm tolerance. The planner places a held box up to about 0.21 m to the side. Three slots on one table leave the outer one 2 cm to spare |
| `e_box_then_dock_2` | S504 | `move` to the dock did not converge in time | The dock is 12 cm behind the destination's standing place. `move` walks to an approach point 0.5 m short of a station and comes in again, even for a 12 cm step; in this run its pose estimate then jumped by 0.5 m and 0.6 rad between looks |
| `e_box_then_dock_3` | S302 | `robot_at r dock` is `unknown`: offset 4.94 cm against a tolerance of 5 cm, "within the margin of the tolerance". The oracle says the robot was at the dock | `move` arrived inside its 4.2 cm; the robot then stood 4.9 cm off when the goal was observed. A value within 5 mm of the tolerance is never decided in favour of `true` |
| `l_vial_hold_2` | S502 | The `place` at the spinner: the held box came 0.4 mm inside the table's 3 cm margin on the way out of the carry pose | The bench and the spinner share one station, 1.30 m from the plate. Phase 3 found the same for places from 1.30 m and moved the place stations to 1.35 m; this scene grasps there too, so it stayed at 1.30 m |

All five end with the arm and the legs stopped as the backend's stop prescribes, and none moved an object it should not
have. Two of the causes are scene choices made in this phase (`a_sort_three`'s three slots, `l_vial_hold`'s shared
station). Two are the backend's (`move` over a very short distance; standing 4.9 cm off after arriving inside 4.2 cm).

## 3. Runs that did not count

Two runs of `a_sort_three` (`_1`: S517, `_4`: S502) fell behind real time in one second each (ratio 0.60 and 0.76, with
the loop neither computing nor waiting for a CPU: the virtual machine paused). They are harness failures under 0046
U13 and were repeated. Neither ended `SUCCEEDED`, so neither could hide a false success. Over the 42 runs that count,
the worst second was 0.989 of real time.

## 4. The scripted failure cases

All on `e_pick_place`, once each.

| Case | Code | Also checked |
|---|---|---|
| The box is not in the scene | S506 | no arm, hand or attach command was sent |
| The box is on the destination's table | S506 | no arm, hand or attach command; the box did not move |
| The destination's region is 0.8 m ahead | S502, at the `place` | the box is still in the hand; nothing was detached |
| The head camera is covered from the start | S517 | no arm or hand command; the box did not move |
| A push on the torso while walking | S516 | the oracle confirms the fall; no command of any kind after it |

## 5. What this run does and does not show

- **It shows** that the executor, the observations and the verdicts hold over 14 tasks: 54 atoms decided `true` and
  true, one `unknown`, none wrong, no false success in 44 task runs.
- **Every atom the oracle judged was true.** The family's tasks have goals that the plan achieves, so this run has no
  `false` verdict in it. Those are in phase 3 (O1, O2, O4), which passed on x86_64 on the same code.
- **The walking stand-in is not Unitree's controller** (0046 section 5). Nothing here says how the real G1 walks.
- **Grasps are welds** (`grasp_model: weld_after_hand_close`, 0046 U7). Nothing here says the hand holds a box.
- **The left hand is new.** `w_split_boxes`, `a_sort_three` and `l_vial_hold` are its first runs; phases 2 and 3 use
  the right hand only.

## 6. Before the run

The scenes were built and tried before the run started, and changed while they were tried. For the record:

- `w_split_boxes` first had both boxes for the right hand, 18 cm apart: S502 at the first grasp. The planner's reach to
  the side was then measured on the robot's own model, and the scenes give each side of a table its own hand.
- A table 5 cm wider on the right than the verified one: S502 at a grasp that works on the narrow table. Wide tables
  now reach only as far right as the narrow one.
- Trial runs with the final layout: `w_split_boxes`, `e_box_then_dock` and `l_vial_hold` each succeeded once;
  `a_sort_three` failed twice (S502); the five failure cases passed. So `a_sort_three` was known to be marginal when
  the run started, and it ran as it was.
- x86_64 CI on pull request #192 passed P1–P6, E1–E7 and O1–O7 before the run ([prerequisites](g1-phase4-prereqs.md)).

No scene and no code changed during the run.

## 7. For the owner

The bar is missed by one run. Two ways on, and one thing not to do:

1. **Fix and run again (recommended).** Three changes, none of them to the judge: `l_vial_hold`'s shared station at a
   distance that suits both the grasp and the place; `move` going straight to a station it is already beside, and the
   goal observed only when the robot stands inside the sure band; and either two slots less far out for
   `a_sort_three` or that task leaving the family. The backend changes need phases 2 and 3 again (about an hour of
   CI), then the acceptance run again (about three hours).
2. **Accept with a recorded exception.** Without `a_sort_three` the same runs give 36 of 39 (92.3%) and 51 of 52 atoms
   (98.1%), with no false success: the bar is met by a family of 13. 0046 lets a task leave the family for the
   visibility rule or for walking while holding; a task leaving because three parts at one station are beyond the arm
   planner is a new reason, so it needs the owner's decision. H3 left `w_two_boxes` out for a like reason (0029
   section 7).
3. **Not this:** counting runs again until 38 succeed. A run counts or does not by the rule written before it.

## 8. Reproduce

```sh
bash backends/unitree_g1/scripts/fetch_assets.sh
python -m pip install -e . -e "backends/unitree_g1[test]"
MUJOCO_GL=egl python -m robolang_g1.phase4 --out phase4.json     # about 3 hours; resumes if interrupted
python -m robolang_g1.scenes a_sort_three --out /tmp/scene       # one scene: site.json, scene.json, the visibility rule
```

On GitHub: the workflow `G1 simulation harness`, started by hand with `acceptance: true`.
