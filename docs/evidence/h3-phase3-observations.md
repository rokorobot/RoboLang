# H3 phase 3: observation-based verdicts findings

Date: 2026-10-31. Decision: [0029](../../spec/decisions/0029-physics-simulation-backend.md), delivery item 3. Contract: [`ros2-backend-v0.1.md`](../../spec/ros2-backend-v0.1.md), section "Observations and verdicts". [Green integration run](https://github.com/rokorobot/RoboLang/actions/runs/41214856256); its `ros2-sim-results` artifact includes `executor.json` (with every reading), `oracle.json` and the logs.

## Result

The unchanged [`transfer.rob`](../../backends/ros2/examples/transfer.rob) ran all four steps, then the executor fused ten overhead camera frames, the `tool0` transform and the gripper joint state. Marker 7 was seen in all ten frames (confidence 1.0) inside the `drop` region, so `object_at box drop` was `true` and the record ended **`SUCCEEDED`**, with a Core outcome `succeeded` citing one signed camera observation. The CLI exited 0. Gazebo ground truth was read afterwards only by the test oracle, which agreed with the verdict; no false success.

| Marker face position (m, `base_link`) | x | y | z |
|---|---:|---:|---:|
| Observed (camera) | 0.1223 | 0.5067 | 0.3149 |
| Oracle (Gazebo box pose) | 0.1173 | 0.5105 | 0.3505 |
| Error | +0.005 | −0.004 | −0.036 |

Step durations in simulation time: move 0.65 s, grasp 5.02 s, move 0.61 s, place 4.57 s. Image build 250 s.

## Findings

- **The first run failed correctly.** In [run 41214165226](https://github.com/rokorobot/RoboLang/actions/runs/41214165226) the box landed at x = 0.109, outside the first drop region (x ≥ 0.12). The verdict was `false` (S301) and the oracle agreed. The box lands about 9 cm short of the drop pose's x because of the grasp offset, so regions are calibrated to where `place` actually leaves the box.
- **Depth is the weak axis.** With the marker about 40 px wide at 1.05 m, the camera underestimates its height: by 4.6 cm without and 3.6 cm with subpixel corner refinement. Horizontal error is under 1 cm. Region heights (0.27–0.40 m) leave margin for this bias while excluding a box still held at the drop pose (true marker height about 0.46 m).
- **`holding` is often `unknown` from this camera**, because the closed gripper covers the box's top marker. That is the intended fail-closed result; `transfer.rob` does not use it.
- Confidence is a detection rate (agreeing frames over ten), not a calibrated probability. The tool and gripper readings have confidence 1 because they come from the simulated joint encoders.

## Limits

One green run after one calibration, not a reliability rate. Agreement over repeats, the 98% oracle bar and the scripted failure cases (object absent, moved, out of reach, camera covered) are phase 4.
