# G1 phase 4: the acceptance run

Date: 2026-11-26. Code: [`backends/unitree_g1/`](../../backends/unitree_g1/) (`scenes.py`, `phase4.py`), decision
[0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md) section 3; contract in
[`g1-backend-v0.1.md`](../../spec/g1-backend-v0.1.md), "Acceptance scenes and the acceptance run". Simulation only.

**Result: run 3 meets the bar, with no margin.** 38 of 42 runs `SUCCEEDED` (90.5%; 90% is 38), 57 of 57 atoms agree
with the oracle, no false success, 5 of 5 scripted failure cases. Runs 1 and 2 did not meet it (37 of 42; then 39 of
42 but 96.8% agreement). **No run of any of the three ended `SUCCEEDED` with its goal not met.** Run 3 found a fault in
`move` that the bar does not measure: an arrival confirmed without a valid marker fix (section 9, follow-up F1).

| Bar (0046 section 3, item 3) | Required | Run 1 | Run 2 | Run 3 |
|---|---|---|---|---|
| Runs `SUCCEEDED`, from observations | ≥ 90% (38 of 42) | 37, 88.1% | 39, 92.9% | **38, 90.5%** |
| Atoms agreeing with the oracle | ≥ 98% | 54 of 55, 98.2% | 61 of 63, 96.8% | **57 of 57, 100%** |
| False successes | 0 | 0 | 0 | **0** |
| Scripted failure cases `FAILED` with their code | 5 of 5 | 5 | 5 | **5** |
| Bar | | not met | not met | **met** |

Host for all three: a Cloudroom sandbox, x86_64, 4 vCPUs, no GPU; stance `free`, arm work `supported` (0046 open point
1). Sections 1–7 are run 1, section 8 run 2, section 9 run 3. Results and every run's record, result, site and scene
are in [`g1-phase4/`](g1-phase4/).

## 1. Run 1: the tasks

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

## 2. Run 1: the five failed runs

| Run | Code | What the record says | Cause |
|---|---|---|---|
| `a_sort_three_2`, `_5` | S502 | The fourth `place` (part y in the outer right slot): no inverse-kinematics solution, the target 0.22–0.25 m to the right of the pelvis | The robot stood 2–5 cm to the left of the station, inside the 5 cm tolerance. The planner places a held box up to about 0.21 m to the side. Three slots on one table leave the outer one 2 cm to spare |
| `e_box_then_dock_2` | S504 | `move` to the dock did not converge in time | The dock is 12 cm behind the destination's standing place. `move` walks to an approach point 0.5 m short of a station and comes in again, even for a 12 cm step; in this run its pose estimate then jumped by 0.5 m and 0.6 rad between looks |
| `e_box_then_dock_3` | S302 | `robot_at r dock` is `unknown`: offset 4.94 cm against a tolerance of 5 cm, "within the margin of the tolerance". The oracle says the robot was at the dock | `move` arrived inside its 4.2 cm; the robot then stood 4.9 cm off when the goal was observed. A value within 5 mm of the tolerance is never decided in favour of `true` |
| `l_vial_hold_2` | S502 | The `place` at the spinner: the held box came 0.4 mm inside the table's 3 cm margin on the way out of the carry pose | The bench and the spinner share one station, 1.30 m from the plate. Phase 3 found the same for places from 1.30 m and moved the place stations to 1.35 m; this scene grasps there too, so it stayed at 1.30 m |

All five end with the arm and the legs stopped as the backend's stop prescribes, and none moved an object it should not
have. Two of the causes are scene choices made in this phase (`a_sort_three`'s three slots, `l_vial_hold`'s shared
station). Two are the backend's (`move` over a very short distance; standing 4.9 cm off after arriving inside 4.2 cm).

## 3. Run 1: runs that did not count

Two runs of `a_sort_three` (`_1`: S517, `_4`: S502) fell behind real time in one second each (ratio 0.60 and 0.76, with
the loop neither computing nor waiting for a CPU: the virtual machine paused). They are harness failures under 0046
U13 and were repeated. Neither ended `SUCCEEDED`, so neither could hide a false success. Over the 42 runs that count,
the worst second was 0.989 of real time.

## 4. Run 1: the scripted failure cases

All on `e_pick_place`, once each.

| Case | Code | Also checked |
|---|---|---|
| The box is not in the scene | S506 | no arm, hand or attach command was sent |
| The box is on the destination's table | S506 | no arm, hand or attach command; the box did not move |
| The destination's region is 0.8 m ahead | S502, at the `place` | the box is still in the hand; nothing was detached |
| The head camera is covered from the start | S517 | no arm or hand command; the box did not move |
| A push on the torso while walking | S516 | the oracle confirms the fall; no command of any kind after it |

## 5. What the acceptance run does and does not show

- **It shows** that the executor, the observations and the verdicts hold over 14 tasks: 54 atoms decided `true` and
  true, one `unknown`, none wrong, no false success in 44 task runs.
- **Every atom the oracle judged was true.** The family's tasks have goals that the plan achieves, so this run has no
  `false` verdict in it. Those are in phase 3 (O1, O2, O4), which passed on x86_64 on the same code.
- **The walking stand-in is not Unitree's controller** (0046 section 5). Nothing here says how the real G1 walks.
- **Grasps are welds** (`grasp_model: weld_after_hand_close`, 0046 U7). Nothing here says the hand holds a box.
- **The left hand is new.** `w_split_boxes`, `a_sort_three` and `l_vial_hold` are its first runs; phases 2 and 3 use
  the right hand only.

## 6. Before run 1

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

## 7. Run 1: for the owner

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

## 8. Run 2 (2026-11-26): the fixes of section 7, and the bar missed on agreement

The owner chose "fix and run again" (section 7, option 1). Code `650cd48`:

- **`move` to a station the robot already stands beside is the servo alone** ("Steps" in the spec): no walk out to the
  approach point and back for `l_vial_hold`'s bench and spinner (one station) or the dock 12 cm behind a destination.
- **A place out of reach at the region's centre aims 1.5 cm, then 3 cm, nearer the robot**, inside the region and at
  least 3 cm from its edge.
- **The dock's plate 0.45 m to the side**, not 0.6 m: from 0.6 m it was read 24° off its axis, and `robot_at` was 1.6 cm
  off what the arrival had measured two seconds before.

Result ([`phase4-run2.json`](g1-phase4/phase4-run2.json), [records](g1-phase4/run2-records.tar.gz)): **39 of 42 runs
`SUCCEEDED` (92.9%, met); no false success; 5 of 5 failure cases; but 61 of 63 atoms agree with the oracle (96.8%,
bar 98%): not met.** All 42 runs counted (worst second 0.988 of real time). Run time 2 h 37 min.

| Task | Run 1 | Run 2 |
|---|---|---|
| `a_sort_three` | 1 of 3 | 2 of 3 (S502 at the second grasp: no carry pose clears the box, the robot 3.9 cm off) |
| `e_box_then_dock` | 1 of 3 | **1 of 3** (S302 twice: `robot_at r dock` unknown) |
| `l_vial_hold` | 2 of 3 | **3 of 3** |
| the other 11 | 33 of 33 | 33 of 33 |

The direct move worked at the dock in all three runs (arrival 1.2–3.2 cm). The place's shift was never needed. **The
two `unknown`s were a fault of the move of the plate:** at its new place the line from the camera to the plate's lower
edge crossed the table's far edge, the plate's pose jittered by more than 1 cm from frame to frame, and its confidence
was 0.00 in two runs of three (the destination's plate in the same snapshots, 3 cm clear of the table, read 1.00). The
visibility check of the scenes looked only at the plate's centre, so it did not see this.

**After run 2** (code `e01b9a8`): the dock's plate is raised to 0.40 m, and the visibility rule checks every corner of
each plate the goal reads, white border included, 2 cm clear of every table and of a box where the goal puts it. It
fails the run-2 dock and passes the new one; two trial runs of `e_box_then_dock` (not counted) read the dock's plate at
confidence 1.00 and succeeded.

**Also changed between the runs, from x86 CI:** E5 (phase 2) failed twice in CI on a check nothing in phase 4 had
touched: a stop confirmed from `lowstate` (every joint under 0.09 rad/s) while the robot's body still slid 1.6 cm in
the window, against the check's 1.5 cm. The stop is now confirmed only when the legs and waist have also travelled at
most 0.01 rad over the window. With it, P1–P6, E1–E7 and O1–O7 pass on x86_64 (CI run of `fd71842`). This changes no
outcome of a task run: it only decides when a stop counts as confirmed.

## 9. Run 3 (2026-11-26): the bar met

Code `e01b9a8` (run 2's code, the dock's plate raised to 0.40 m, the plate check of the visibility rule, and the stop
confirmed only when the legs have not travelled). [`phase4-run3.json`](g1-phase4/phase4-run3.json),
[records](g1-phase4/run3-records.tar.gz). All 42 runs counted (worst second 0.971 of real time). Run time 2 h 32 min.

| Task | Run 1 | Run 2 | Run 3 |
|---|---|---|---|
| `a_sort_three` | 1 of 3 | 2 of 3 | 2 of 3 (S502 at a grasp) |
| `e_box_then_dock` | 1 of 3 | 1 of 3 | **3 of 3** (the dock's plate read at confidence 1.00) |
| `l_vial_hold` | 2 of 3 | 3 of 3 | 2 of 3 (S502 at the place, same station: the held box 1.1 cm inside the table's margin) |
| `l_vial_to_fridge` | 3 of 3 | 3 of 3 | 2 of 3 (S517: see F1) |
| `w_split_boxes` | 3 of 3 | 3 of 3 | 2 of 3 (S517: see F2) |
| the other 9 | 27 of 27 | 27 of 27 | 27 of 27 |

Over the three runs 114 of 126 task runs succeeded (90.5%), and every one of the 12 that did not was a refusal (S502),
a lost marker (S517), a timeout (S504) or an `unknown` (S302): never a wrong verdict.

### Follow-ups found by run 3 (not part of the bar)

**F1. `move` confirmed an arrival without a valid marker fix (`l_vial_to_fridge_2`).** The record says the robot
arrived within 3.5 cm of the destination's station; the oracle had it 0.45 m away. At the station every reading of
its plate was **rejected** by the localization (its vertical 0.07–0.08 rad from the IMU's: the wrong one of the two
poses a flat marker allows), so the belief kept its dead reckoning. `_confirm_pose` checks only that the station's
marker is among the readings, not that the look fixed the pose from it, and so confirmed the stale belief. The look
before the place found the plate's confidence too low and ended the run with S517 before the arm moved. **The verdicts
are not affected:** `robot_at` and `object_at` count only frames whose vertical agrees with the IMU, so such a reading
gives `unknown`, never `true`. Proposed fix (one condition, not made: it changes the code this run accepted):
`_confirm_pose` also requires `self.last_look[1]`, a fix from this look; then phases 2 and 3 in CI.

**F2. A marker fix from far off the plate's axis (`w_split_boxes_3`).** Walking between the tables with a box in each
hand, the robot fixed its pose from the first station's plate seen 2.7 m away and about 39° off its axis; the fix put
it 1 m from where it was, and it then looked for the next plate where it was not (S517 after a 90 s scan). Proposed: a
station marker fixes the pose only within an angle of its axis (to be measured), as it already does only within 2.9 m.

**F3. Three slots and one shared station are still marginal.** `a_sort_three` failed one run in each of the three
runs, and `l_vial_hold`'s place from its shared station failed once in runs 1 and 3: the arm planner's margins, not the
executor's logic. Two hand-sized slots per table and a station each would remove them; that changes the scenes, and
the family's tasks run unchanged either way.

**F4. Phase 2's E2 had its own scene fault, found and fixed in CI during these runs** (pull request #192): its
out-of-reach box was enlarged so that its marker would be read, but the executor was told the original 8 cm box, read
the 10.7 cm marker as 5.7 cm, put the box at half its distance and reached for it. The scene is now also the
executor's site. Phase 2 is not part of the acceptance run.

## 10. Reproduce

```sh
bash backends/unitree_g1/scripts/fetch_assets.sh
python -m pip install -e . -e "backends/unitree_g1[test]"
MUJOCO_GL=egl python -m robolang_g1.phase4 --out phase4.json     # about 3 hours; resumes if interrupted
python -m robolang_g1.scenes a_sort_three --out /tmp/scene       # one scene: site.json, scene.json, the visibility rule
```

On GitHub: the workflow `G1 simulation harness`, started by hand with `acceptance: true`.
