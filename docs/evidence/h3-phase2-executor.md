# H3 phase 2: ROS 2 executor findings

Date: 2026-10-30. Decision: [0029](../../spec/decisions/0029-physics-simulation-backend.md), delivery item 2. Code: [`backends/ros2/`](../../backends/ros2/README.md). [Green integration run](https://github.com/rokorobot/RoboLang/actions/runs/41063857929) (on-demand workflow, amd64 GitHub runner); its `ros2-sim-results` artifact includes `executor.json`, `smoke.json`, controller and Gazebo logs, and resolved versions.

## Result

A validated [`transfer.rob`](../../backends/ros2/examples/transfer.rob) ran in the Lyrical/Jetty container with the combined UR5e and Robotiq gripper model. The MoveIt 2 OMPL planner and `joint_trajectory_controller` executed all four task steps: move to pickup, approach and grasp, move to drop, and place. The gripper trajectory controller reported reached goals for close and open. Gazebo logged a detachable-joint attach after close and a detach after open. The executor record lists all four steps as `succeeded`, `backend.kind: ros2-gazebo`, package versions, and `mode: simulation`.

The record state is **`UNKNOWN`**, with the declared `object_at box drop` goal marked `not_evaluated` and `core_outcome: null`. The CLI exits 3 on this path. Phase 2 proves execution and attachment; phase 3 will decide task success from camera and joint observations. The arm smoke test also passed (maximum joint error 9.0e-13 rad against 0.05 rad), and the camera detected marker 7 in a nonempty 640×480 frame.

| Measurement | Green run |
|---|---:|
| Image build | 287 s |
| `move` to pickup | 0.706 s simulation time |
| `grasp` including MoveIt approach, close and attach | 5.036 s |
| `move` to drop | 0.628 s |
| `place` including MoveIt motion, open and detach | 4.571 s |

## Safety boundary and failure behavior

The executor has no physical-mode option. Before constructing MoveIt or gripper clients it requires an advancing `/clock`, `use_sim_time=true`, the declared Gazebo model, active controller hardware components matching `GazeboSimSystem` declarations in the live robot description, and the detachable-joint topics. It requests detachment before motion. Invalid bindings and sites, unsupported steps, preconditions, and declared speed limits fail closed before the first task motion. Backend failures use S501–S508 as specified in [`ros2-backend-v0.1.md`](../../spec/ros2-backend-v0.1.md).

The `robolang-ros2` wheel is separate from the core `robolang` wheel. The symbolic simulator's record format and behavior are unchanged.

## Integration details and limits

- UR's released simulation xacro has no gripper mount hook. The backend wraps it with the released Robotiq macro, a Gazebo gripper hardware component, and a gripper trajectory controller. The physical wrist link is the detachable joint parent because Gazebo reduces the gripper's fixed base link.
- The installed Jetty detachable-joint system starts attached despite an `initial_attach=false` request. The harness releases the box before its smoke motion, and the executor repeats a release request before MoveIt starts. This release path is covered by the green run.
- MoveIt needed explicit OMPL pipeline parameters and collision allowances for contacts among the Robotiq's own links. It still checks arm-to-gripper collision. Site poses and planning time were tuned against the measured UR5e tool transform.
- MoveItPy 2.15.2 segfaulted during Python interpreter teardown in CI even after explicit object cleanup. The dedicated CLI writes and closes its record, flushes output, then exits directly with its status. The green run verifies a clean process exit and an inspectable record.
- This is one green end-to-end run after several diagnostic revisions, not a reliability rate. The detachable joint is a simulation shortcut; this run does not prove contact grasp physics or observation-based goal success. The camera pose adapter, oracle agreement, and repeated acceptance threshold are phase 3 and 4 work under 0029.

Local verification at this revision: 2,994 tests passed, 2 skipped, 6 expected failures; Ruff clean; optional backend wheel built.
