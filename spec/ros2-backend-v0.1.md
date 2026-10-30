# RoboLang ROS 2 / Gazebo backend 0.1

**Status:** phase 2 implementation in progress; decision [0029](decisions/0029-physics-simulation-backend.md) approved by the owner. This specification covers the executor, not the phase 3 sensor verdicts.

## Scope and safety boundary

The optional `robolang-ros2` ament Python package executes a validated, single-robot task against the declared Gazebo model. The core `robolang` wheel does not import ROS 2. There is no physical-mode option. Before creating MoveIt or gripper clients, the backend requires an advancing `/clock`, `use_sim_time=true`, the declared model in `gz model --list`, and a controller manager whose hardware plugin is `gz_ros2_control/GazeboSimSystem`. Failure is `S501`, with no motion command sent. The backend does not treat language safety limits as hardware safety functions.

The site file maps IDs in `.rob` bindings to a Gazebo model and `base_link` poses. It has exactly `gazebo_model`, `robot`, `objects` (object ID to Gazebo model name), `poses` (location ID to `{frame, position, orientation}`), and `grasps` (object ID to approach pose). Quaternions must be normalized and all coordinates finite. No extra keys, including a physical-mode key, are accepted.

## Execution

`move r to L` gives MoveIt 2 a pose goal from the site file and executes the resulting trajectory through `joint_trajectory_controller`. A motion step passes only when MoveIt reports completed execution, not goal acceptance. `grasp r O` moves through MoveIt to the site grasp pose, closes the gripper action, waits for its final reached-goal result, then attaches the declared object with Gazebo's detachable joint. `place r O at L` moves to the site pose, opens the gripper, then detaches. `wait for` advances on `/clock`. The backend stops at the first failure. Other steps, preconditions and declared speed limits are rejected before executing the plan until they have a defined observation implementation.

The detachable joint is a simulation expedient. A record says `grasp_model: detachable_joint_after_gripper_close`; it does not claim friction or contact physics has been validated.

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
| S508 | Task feature needs an observation adapter or is unsupported by this backend |

## Record

Each attempt writes a JSON record with `mode: simulation`, `backend.kind: ros2-gazebo`, resolved package versions, bindings, ordered step results, failure code, and a goal status. Until phase 3 can evaluate goals from sensor observations, completed steps produce `state: UNKNOWN`, `goal: not_evaluated`, and no successful Core outcome. A backend failure produces `state: FAILED`. The CLI exits 3 for `UNKNOWN` and 1 for `FAILED`; it never reports unverified completion as exit 0. The symbolic simulator's records remain byte-identical. Phase 3 will add the observation verdict and the full Core outcome without using Gazebo ground truth as evidence.

## Verification

Unit tests use fake operations to check guard ordering and failure propagation. The on-demand/nightly ROS 2 workflow checks real MoveIt, gripper, and attachment behavior in the Lyrical Gazebo container. Phase 4 has the benchmark acceptance threshold from 0029 section 3.
