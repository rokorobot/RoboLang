# RoboLang Unitree G1 backend 0.1 (MuJoCo, simulation only)

**Status:** phase 1 of decision [0046](decisions/0046-humanoid-backend-unitree-g1.md): the stand-in, the guard and the
harness checks P1–P6, passing on aarch64 ([evidence](../docs/evidence/g1-phase1-harness.md)) and x86_64
([CI run](https://github.com/rokorobot/RoboLang/actions/runs/44559535665)). The executor (`move`,
`grasp`, `place`, `wait for`), the observations and the run records are phases 2 and 3; their sections below are the
contract they must meet. Code: [`backends/unitree_g1/`](../backends/unitree_g1/).

## Scope and safety boundary

The optional `robolang-g1` package runs a validated task on a simulated Unitree G1 (29 DoF, Dex3-1 hands) in MuJoCo.
The core `robolang` wheel does not import it. It talks to the robot **only through the interfaces the real G1 offers**:
Unitree's SDK over CycloneDDS (`LocoClient` on the RPC service `sport`, `rt/arm_sdk`, `rt/dex3/*`, `rt/lowstate`) and
the head camera, which on the robot is read directly, not through the SDK. In simulation a **stand-in** process answers
on those interfaces; it is described in full in the docstring of
[`robolang_g1/sim/standin.py`](../backends/unitree_g1/robolang_g1/sim/standin.py). The stand-in's walking controller is
Unitree's pretrained whole-body policy, not the robot's own controller, so nothing here says how the real G1 walks.

**Guard (S501).** Before any command the backend requires, in order: a DDS domain other than 0 and the loopback
interface, checked before any DDS participant exists; an answer from the stand-in's simulation-identity API (service
`robolang_sim`, which no real robot has); `simulation: true` on the same domain; and a simulation clock that advances.
There is no flag, variable or argument that skips a step ([`guard.py`](../backends/unitree_g1/robolang_g1/guard.py)).

**Never evidence.** MuJoCo state and `SportModeState` are never read by the backend. With `--oracle` only, the
stand-in writes ground truth for the test harness; nothing else reads it.

## Time

Physics steps every 2 ms against the wall clock, the joint PD law is evaluated every step (500 Hz) and the policy
every 10th step (50 Hz). A 5 ms step is not allowed: MuJoCo integrates the PD torque explicitly, which is stable only
for dt·kd/I < 2, and the arms reach 4.5 at 5 ms (1.8 at 2 ms). Simulation time is in `LowState_.tick` (ms), in the
identity API and in every camera frame. The SDK's timeouts are wall-clock, as on the robot. The stand-in reports its
real-time ratio. A run in which any second falls more than 5% behind real time does not count towards acceptance.

## Stance and stop

`stance_model` is `free` by default: the policy alone, which comes to rest after a walk (about 0.13 m and 2 s of coast
at 0.5 m/s, then about 1 cm of torso sway). `free` takes a recovery step of 0.2–0.35 m about every 45 s while standing,
so arm work longer than about 40 s uses `supported`: the torso welded to a mocap body (the fallback the owner approved,
0046 open point 1). Every record states the stance model.

The stand-in never collapses the robot: `SetFsmId` accepts only 500 (standing or walking) and refuses every other id
with code 7400, `Damp` included. The backend's stop (phase 2) is zero velocity, the arms held through `arm_sdk`, the
hands unchanged, the robot kept standing. It never calls `Damp()`, zero torque or `SwitchToUserCtrl`, and never writes
`rt/lowcmd` (safety case H25).

## Steps, observations and records (phases 2 and 3)

As decided in 0046 U5–U8 and U11. Arm motions are ramped: a sudden reach moved a free-standing robot about 12 cm.
Before a `grasp` or `place`, the executor re-observes the station marker and repeats the approach if the robot has
moved (the recovery step above).

## Failure codes

The codes and meanings of the [ROS 2 backend](ros2-backend-v0.1.md#failure-codes) (S501–S515), plus:

| Code | Meaning |
|---|---|
| S516 | The robot fell or lost balance (IMU tilt beyond the limit, or the walking service left its standing and walking states). The run stops; no further command |
| S517 | Localization lost: a station marker that `move` or an observation needs was not seen within its timeout |

## Where it runs

`.github/workflows/g1-sim.yml`: on demand, nightly, and on pull requests that change `backends/unitree_g1/` or the
workflow itself (0046 U12 said on demand and nightly; the pull-request trigger is limited to the backend's own files so
that a backend change is checked on x86_64 before it merges). Plain x86_64 runner, no GPU, no Docker; the assets and the
CycloneDDS build are cached.
