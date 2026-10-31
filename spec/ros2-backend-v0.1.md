# RoboLang ROS 2 / Gazebo backend 0.1

**Status:** phase 2 executor verified in the [integration run](../docs/evidence/h3-phase2-executor.md); phase 3 added goal and precondition verdicts from sensor observations; phase 4 adds several cameras, a grasp check, approach moves and the benchmark acceptance run. Decision [0029](decisions/0029-physics-simulation-backend.md) approved by the owner.

## Scope and safety boundary

The optional `robolang-ros2` ament Python package executes a validated, single-robot task against the declared Gazebo model. The core `robolang` wheel does not import ROS 2. There is no physical-mode option. Before creating MoveIt or gripper clients, the backend requires an advancing `/clock`, `use_sim_time=true`, the declared model in `gz model --list`, active controller hardware components matching `gz_ros2_control/GazeboSimSystem` declarations in the published robot description, and the declared Gazebo detachable-joint topics. It requests initial detachment before motion. Failure is `S501`, with no motion command sent. The backend does not treat language safety limits as hardware safety functions.

The site file maps IDs in `.rob` bindings to a Gazebo model and `base_link` poses. It has exactly `gazebo_model`, `robot`, `objects` (object ID to Gazebo model name), `poses` (location ID to `{frame, position, orientation}`), `grasps` (object ID to approach pose), `markers` (object ID to a distinct ArUco 4x4_50 ID), `regions` (location ID to a `base_link` box `{frame, min, max}` that the object's marker must be inside), `cameras` (a nonempty list, each with a unique `name`, an image `topic`, the camera link's `base_link` pose, `horizontal_fov`, image `width` and `height`, and printed `marker_length` in metres), `obstacles` (`base_link` boxes added to MoveIt's planning scene) and `approach_height` (metres, from 0 to 0.5). Camera poses are calibrations, not observations. Quaternions must be normalized and all coordinates finite. No extra keys, including a physical-mode key, are accepted.

## Execution

`move r to L` gives MoveIt 2 a pose goal from the site file and executes the resulting trajectory through `joint_trajectory_controller`. A motion step passes only when MoveIt reports completed execution, not goal acceptance. `grasp r O` moves through MoveIt to `approach_height` above the site grasp pose, then down to it. It then takes an observation snapshot: the object's marker must be seen under the gripper (as for `holding`, below) or the step fails with S506 before the gripper closes. It closes the gripper action, waits for its final reached-goal result, attaches the declared object with Gazebo's detachable joint, and moves back up. `place r O at L` moves to the site pose and `approach_height` down from it, opens the gripper, detaches, then moves back up. With `approach_height` 0 there are no extra moves. `wait for` advances on `/clock`. The backend stops at the first failure. Other steps and declared speed limits are rejected before executing the plan.

The detachable joint is a simulation expedient: one per object, with topics `/model/<robot model>/detachable_joint/<object model>/{attach,detach,state}`. The installed Jetty build starts each attached despite an `initial_attach=false` request, so the harness detaches every object before anything moves and the executor repeats the release during its guard. In the acceptance scenes the 0.1 m boxes are wider than the gripper's 85 mm opening, so the grasp pose keeps the fingertips above the box and the joint carries it; the fingers never touch it. A record says `grasp_model: detachable_joint_after_gripper_close`; it does not claim friction or contact physics has been validated.

## Observations and verdicts

Preconditions are evaluated from one observation snapshot before the first step; the goal from one snapshot after the last step succeeds. Conditions are `object_at`, `robot_at` and `holding`, combined with `and`, `or` and `not` in three-valued logic as in the symbolic simulator. Any other condition, or an atom naming an object without a marker or an `object_at` location without a region, fails with S508 or S507 before the first motion. Gazebo ground truth is never read.

| Reading | Source | Confidence |
|---|---|---|
| Marker position | for each site camera, ten frames stamped at least 500 ms of simulation time after the request; each detected marker's pose from `solvePnP` (subpixel corners) with ideal pinhole intrinsics from the site camera | per camera, the median position with confidence = the fraction of the ten frames whose detection lies within 1 cm of the median; the reading kept is the most confident camera's, the earlier camera in the site on a tie |
| Tool pose | latest `base_link` to `tool0` transform | 1 |
| Gripper | latest `robotiq_85_left_knuckle_joint` position | 1 |

A reading is usable only when its confidence is at least `min_confidence` and its age (snapshot time minus reading stamp, in simulation time) is at most `max_age`, from `@verify` or the RoboLang 1.3 defaults 0.9 and 10 s. An atom with an unusable reading is `unknown`.

| Atom | `true` | `false` |
|---|---|---|
| `object_at o l` | the marker of `o` is inside the region of `l` | the marker is seen anywhere else |
| `robot_at r l` | `tool0` is within 2 cm and 0.05 rad of the site pose of `l` | it is not |
| `holding r o` | the gripper is closed (within 0.05 rad of 0.8) and the marker of `o` is under `tool0`: within 10 cm horizontally and 5 to 35 cm below it | the gripper is open, or the marker is seen elsewhere |

A closed gripper whose object marker no camera sees gives `unknown`. Results follow the symbolic simulator: a false precondition is S101, an unknown one S102; a false goal condition is S301, an unknown one S302, each with `step: null`.

## Failure codes

| Code | Meaning |
|---|---|
| S501 | Simulation-only graph guard failed |
| S502 | MoveIt initialization or planning failed |
| S503 | MoveIt/controller execution failed or aborted |
| S504 | Motion, gripper or simulated wait timed out |
| S505 | Gripper controller rejected, failed or could not reach its goal |
| S506 | Gazebo attachment/detachment failed, or place has no matching grasp |
| S507 | Site or binding is invalid or incomplete |
| S508 | Task feature has no observation or is unsupported by this backend |

S101, S102, S301 and S302 have their [simulation](simulation-v1.0.md) meanings, decided from observations.

## Record

Each attempt writes a JSON record, schema `robolang.ros2.run` 0.2, with `mode: simulation`, `backend.kind: ros2-gazebo`, resolved package versions, Task IR, bindings, the `verify` thresholds used, each precondition, ordered step results, each goal condition (`true`, `false`, `unknown` or `not_evaluated`), every observation snapshot with its readings, and `core_outcome`. `state` is `SUCCEEDED` only when every goal condition is `true`; otherwise `FAILED` with a code. The CLI exits 0 for `SUCCEEDED`, 3 for S302 and 1 otherwise.

`core_outcome` is the Core 0.1 `outcome` payload, or `null` when the goal has no Core form. The verifier and reporter are `rl:sim:gazebo`. Each evidence observation states what the readings support (for example `located_in` the location whose region holds the marker), with the reading's stamp, `valid_for` equal to `max_age`, its confidence rounded down to two places, and the sensor and method. Observations are signed with a fixed, published, simulation-only key, like the symbolic simulator's. A criterion is met only when its verdict is `true`. The symbolic simulator's records remain byte-identical.

The Lyrical MoveItPy 2.15.2 binding can crash during Python interpreter teardown after successful execution. The dedicated CLI process writes and closes its record, flushes output, then exits directly with the documented status; it does not destroy MoveItPy in the same process.

## Verification

Unit tests use fake operations and snapshots to check guard ordering, failure propagation, thresholds, the grasp check and verdicts, and validate every generated scene's site. The on-demand/nightly ROS 2 workflow is the acceptance run of 0029 sections 3 and 7: `scripts/scenes.py` generates a Gazebo world and site per benchmark task with the same entity IDs, each task runs three times in a fresh container (`scripts/run_task.sh`), the four failure cases once each, and a test-only oracle (`scripts/oracle.py`) compares every evaluated goal condition with Gazebo's box and wrist poses and the simulated gripper joint. `scripts/acceptance.py` applies the pass bar.
