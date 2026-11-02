# Decision record 0031: Observing `region_clear` and `robot_ready` in physics simulation

**Date:** 2026-11-02
**Status:** **APPROVED by the owner, 2026-11-02**, with the recommended option for R1–R7. Design only: each delivery
step of R7 is its own PR.
**Applies to (when decided):** [`ros2-backend-v0.1.md`](../ros2-backend-v0.1.md) "Observations and verdicts", the site
file, the H3 scenes, and the acceptance family of [0029](0029-physics-simulation-backend.md) section 7.
**Evidence:** 0029 section 7 deferred these two conditions. Five single-arm tasks of benchmark set 1 wait on them:
`e_pick_place_cell_clear` (`region_clear cell`), `e_pre_and` (`robot_ready r and region_clear cell`),
`e_pre_not_occupied` (`not region_clear zone`), `e_pre_or` (`region_clear cell or region_clear zone`) and
`a_pre_conjunction` (`object_at`, `robot_ready` and `region_clear`). Today the backend rejects them with `S508`.

## 1. The problem

The symbolic simulator reads both conditions from facts ([`simulation-v1.0.md`](../simulation-v1.0.md) 4.4). The
backend must decide them from observations only (0029 X6). `robot_ready` is easy: the robot reports its own state.
`region_clear` is not. The fiducial cameras see only marked objects, so "no marker in the region" would call a region
clear when an unmarked box or a person stands in it. That is a false `true` in a condition people use to guard motion,
the same failure a false success is for a goal.

## 2. Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| R1 | Meaning of `robot_ready r` | `true` when, at observation time: every ros2_control hardware component of the robot is `active`; the arm and gripper trajectory controllers are `active`; and `/joint_states` is fresh (within `max_age`). `false` when a component or controller is reported inactive, unconfigured or in error. `unknown` when the controller manager does not answer or the joint states are stale. Confidence 1 | (b) Also require MoveIt to be ready: MoveIt is the planner, not the robot, and a plan failure already fails the step (`S502`). (c) A protective-stop or e-stop state: Gazebo has none. It is a hardware question and belongs with H4 |
| R2 | How `region_clear c` is observed | A **depth camera** that sees the region, and a voxel test. The region is a `base_link` box. Each depth pixel is a ray: voxels it passes through are *seen free*, and the voxel it ends in is *occupied*. Points on the robot's own links (its collision geometry from TF, inflated 2 cm) are removed first. `false`: at least 3 occupied voxels inside the region. `true`: every voxel of the region is seen free. `unknown` otherwise, including any voxel that is hidden or outside the view. Confidence: the fraction of voxels seen | (b) "No marker in the region": a false `true` for any unmarked thing. (c) Gazebo's logical camera or contact sensors: they report model poses, which is ground truth (X6). (d) Background subtraction on the RGB image: fails on lighting and colour, and gives no 3D extent |
| R3 | What counts as occupying a region | Anything except the robot itself, including the task's own objects. A region the plan needs empty is empty, whatever is in it | Count only things that are not task objects: a box left in a guarded cell would then count as clear |
| R4 | Site file | A new key `zones`: region ID → `{frame, min, max, sensor}`, where `sensor` names a depth camera in `cameras` (which gains `type`: `rgb` or `depth`). Zones are separate from location `regions`, as `region` and `location` are separate types in `.rob`. Voxel size 2 cm, fixed in the spec | Reuse location regions: `.rob` keeps the two types apart, and a guarded cell is not a place an object is put |
| R5 | `wait until region_clear c timeout d` | Out of scope here. The executor still rejects `wait until` (`S508`). It needs repeated observation on simulation time, and its own short record | Include it: it doubles this record's scope, and no task in the R6 family needs it |
| R6 | Acceptance | The five tasks above join the family: three runs each in the existing workflow, the same bar (≥ 90% `SUCCEEDED`, ≥ 98% oracle agreement, no false success). The test oracle decides `region_clear` from Gazebo model poses (a model whose box overlaps the zone; robot excluded) and `robot_ready` from the controller manager. New scripted cases: an **unmarked** box in a zone the task needs clear (must not be `true`); a zone half hidden behind a wall (must be `unknown`); the arm controller stopped before the run (`robot_ready` `false`) | Test only the five tasks: none of them would show that an unmarked obstacle is caught, which is the point of R2 |
| R7 | Delivery | (1) `robot_ready` with its tests (small PR). (2) The depth camera, zones and `region_clear`, with unit tests on synthetic depth images. (3) The scenes, scripted cases and an acceptance run, with an evidence document. Each PR separate | One PR: a depth-sensing defect would hold back `robot_ready` |

## 3. Limits stated now

- An observed `region_clear` is evidence for a RoboLang condition. It is **never a safety function** (RFC-001 13.6).
  A real deployment needs certified safety sensors and stops that work with RoboLang unavailable. Physical
  execution stays unauthorized (strategy H4).
- A region can only be called clear where the camera sees. A zone partly hidden by the arm itself is `unknown`, and a
  task that guards a zone the arm covers will fail rather than pass.
- Simulated depth has no noise unless we add it. The acceptance run adds Gaussian noise (1 cm at 1 m) so the
  3-voxel threshold is tested against something like a real sensor.

## 4. Needs the owner

R1 (what "ready" means without a safety controller), R2 and R3 (the occupancy rule and that task objects count), and
R6 (the scripted cases).
