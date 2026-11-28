# robolang-g1

The Unitree G1 humanoid in MuJoCo, behind Unitree's own SDK. **Simulation only.** Decision record
[0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md): phase 1 is the stand-in and the harness (P1-P6);
phase 2 is the **executor** (`move`, `grasp`, `place`, `wait for`, the biped's stop, run records; checks E1-E7), phase 3
the **verdicts**: preconditions and the goal decided from observations (S101, S102, S301, S302, `SUCCEEDED`; checks
O1-O7). Two prerequisites of the phase 4 acceptance run: `move`'s heading between marker fixes comes from the IMU, and
the stand-in keeps pace with real time on a loaded 4-CPU host
([evidence](../../docs/evidence/g1-phase4-prereqs.md)). Nothing here can reach a real robot: the guard refuses DDS domain 0, any interface
but loopback, and anything that does not answer as the simulator.

## Run it

```sh
python -m pip install -e .                                   # robolang (repo root)
bash backends/unitree_g1/scripts/fetch_assets.sh             # models, policy, SDK at pinned commits; builds CycloneDDS
python -m pip install -e "backends/unitree_g1[test]"         # mujoco, numpy, opencv (headless), onnxruntime, cyclonedds
python -m pytest -q backends/unitree_g1/tests
MUJOCO_GL=egl python -m robolang_g1.harness --out harness.json    # P1-P6; exit 0 = go; takes about 2 minutes
MUJOCO_GL=egl python -m robolang_g1.phase2 --out phase2.json      # E1-E7; exit 0 = all pass; about half an hour
MUJOCO_GL=egl python -m robolang_g1.phase3 --out phase3.json      # O1-O7; verdicts against the oracle; about half an hour
MUJOCO_GL=egl python -m robolang_g1.phase2 --only move-stress --out stress.json   # 20 shelf arrivals; add --carry for the box in hand; about an hour
MUJOCO_GL=egl python -m robolang_g1.phase4 --out phase4.json      # the acceptance run: 14 tasks x 3, 5 failure cases; about 4 hours; can be resumed
python -m robolang_g1.scenes --list                               # the acceptance scenes; `scenes NAME --out DIR` writes one
```

Run a task (the stand-in starts for it; `python -m robolang_g1.cli` runs one against a stand-in that is already up):

```sh
MUJOCO_GL=egl python -m robolang_g1.run_task bench/reference/e_pick_place.rob --task pick_place \
    --site backends/unitree_g1/robolang_g1/sites/pick_place.json \
    --bind r=robot_1 --bind box=box_1 --bind src=loc_a --bind dst=loc_b --out record.json   # exit 0: SUCCEEDED
```

**One stand-in at a time**: two on one DDS domain mix their `lowstate`, frames and commands. The stand-in is paced to real
time, so **start the executor at a lower priority** (`nice -n 10`; `run_task` and the checks do) and keep the host's other
work off the stand-in's CPUs: a run is `realtime_valid` only if every second is within 5% of real time.

Assets go to `${G1_ASSETS:-$HOME/.cache/robolang-g1}` (about 200 MB with the CycloneDDS build), each with its licence.
The fetch script is idempotent and prints a versions JSON. Use a virtual environment; set `PYTHON` to its interpreter
if you do not activate it. It needs `git`, `cmake`, a C compiler and `python3-dev`. Software rendering needs EGL
(`MUJOCO_GL=egl`, Mesa) or OSMesa.

## What is in it

| Path | |
|---|---|
| `scripts/fetch_assets.sh` | pinned fetch of unitree_mujoco, unitree_rl_lab, mujoco_menagerie, unitree_rl_gym, unitree_sdk2_python; CycloneDDS build |
| `robolang_g1/sim/standin.py` | the stand-in process: MuJoCo paced to real time, `rt/lowstate`, `rt/arm_sdk`, `rt/dex3/*`, RPC `sport` and `robolang_sim`, head camera, oracle |
| `robolang_g1/sim/policy.py` | the whole-body walking policy (ONNX, 50 Hz) |
| `robolang_g1/sim/imu.py` | the IMU `rt/lowstate` carries: MuJoCo's exact sensor plus a gyro bias (turn-on and random walk), noise and a drifting yaw; `--imu-seed`, `--imu-bias` |
| `robolang_g1/sim/fastdds.py` | `rt/lowstate` and the hand states published from pre-serialized byte buffers (the same bytes as the SDK writes, 17 times cheaper) |
| `robolang_g1/sim/scene.py` | the scene from a **site file** (floor, tables, objects with ArUco markers, stations) and the head camera |
| `robolang_g1/sim/stance.py` | what the robot does when it stands still: `free` (default) or `supported` |
| `robolang_g1/guard.py` | the simulation guard (S501) |
| `robolang_g1/harness.py` | checks P1-P6 through the real SDK clients, writes `harness.json` |
| `robolang_g1/robot.py` | the SDK clients (`LocoClient`, `rt/arm_sdk`, `rt/dex3/*`, `rt/lowstate`) and the camera source, shared by the harness and the executor |
| `robolang_g1/site.py` | the site file, `robolang-g1-site/2` (`/1` still loads) |
| `robolang_g1/kinematics.py`, `perception.py` | the robot's own kinematic model, damped-least-squares IK; ArUco poses in the pelvis frame |
| `robolang_g1/ops.py`, `executor.py`, `cli.py` | what is done to the robot (and the observation snapshots); the run, the stop, the verdicts and the record; the command line |
| `robolang_g1/heading.py` | the belief heading between station-marker fixes: the last fix's plus the IMU's change in yaw, with the gyro bias learned from pairs of fixes (pure: no DDS, no ground truth) |
| `robolang_g1/response.py` | what each burst of velocity command does to a standing robot (measured, empty and loaded) and `plan_correction`, which `move`'s last centimetres use (pure: no DDS, no ground truth) |
| `robolang_g1/observe.py` | verdicts from observations: `object_at`, `holding`, `robot_at`, `robot_ready`, three-valued logic, `@verify`, the Core outcome (pure: no DDS, no ground truth) |
| `robolang_g1/phase2.py`, `phase3.py`, `run_task.py` | checks E1-E7 and O1-O7 (they read the oracle; the executor never does); start a stand-in and run one task |
| `robolang_g1/scenes.py`, `phase4.py` | phase 4: a site for each of the 14 tasks of the acceptance family and for the 5 scripted failure cases, with the visibility rule checked from the geometry; the acceptance run and its judge (0046 section 3). `phase4.py` reads the oracle, the executor never does |
| `robolang_g1/sites/` | `phase1.json` (the harness's, `/1`), `pick_place.json`, `carry_hold.json` and `part_precheck.json` (for `bench/reference/e_pick_place.rob`, `w_carry_hold.rob` and `w_part_precheck.rob`) |

## The stand-in

`python -m robolang_g1.sim.standin --site SITE --run-dir DIR [--oracle PATH] [--stance free|supported]`, on DDS domain
1, interface `lo`, both fixed. The module docstring is the interface description; the short version:

* `rt/lowstate` at 250 Hz. **Its IMU is not ideal**: the gyro has a bias (drawn at start-up, about 0.11 deg/s one sigma)
  that wanders, and noise, and the quaternion's yaw drifts with them, as a MEMS IMU's does with no magnetometer
  (`sim/imu.py`; roll and pitch stay right). **Simulation time** is in `tick` (ms), in the identity API (`sim_time`) and
  in the camera frame's JSON. `LocoClient` works: `SetVelocity` (with `duration`), `GetFsmId`, `SetFsmId`. **`Damp`, and every other
  state change except Start (500), is refused with code 7400**: the stand-in never collapses the robot.
* `rt/arm_sdk` blends joints 12-28 over the walking controller by the weight in `motor_cmd[29].q`; the Dex3 hands take
  `rt/dex3/{left,right}/cmd` and publish `.../state`. `robolang_sim` also attaches an object to a hand (a MuJoCo weld,
  U7), switches the stance (`SetStance`), and counts the commands it receives (`status`). Started with `--test-hooks` it
  also answers a push on the torso, for the fall test; the executor never calls it.
* The head camera (640x480, 5 Hz) is rendered on its own thread (nice 19: it yields to the physics loop) from a copy of
  the state. Frames: `DIR/head/latest.png` and `latest.json`. Read the JSON, then the PNG, then the JSON again; keep the frame if both name the same `frame`.
* `--oracle PATH` writes ground truth (poses, marker positions, a fallen flag) for the harness. Nothing else reads
  MuJoCo state; the backend never will.

**Physics step: 0.002 s, not the 0.005 s of 0046.** The PD law runs on every physics step, and at 0.005 s the arm
joints are numerically unstable (see the evidence document). Do not raise it.

## Stances

`free` is the policy alone: after walking it coasts about 0.12 m, comes to rest, and stands still, but it takes a
recovery step of about 0.3 m every 45 s or so. `supported` welds the pelvis to a mocap body while the robot stands, so
it never steps; it stands in for the balance the robot's own controller gives on hardware (0046 open point 1). Every
record carries its `stance_model`. A new stance implements the small `Stance` interface in `stance.py`.

## Evidence and limits

[docs/evidence/g1-phase1-harness.md](../../docs/evidence/g1-phase1-harness.md): what ran, `harness.json`, the hand-model
decision, the timestep finding, and everything that differs from 0046. The stand-in is **not** Unitree's controller:
gait, speed and drift on the robot will differ, so an executor must close every motion on observation. CI is
`.github/workflows/g1-sim.yml` (on demand and nightly, and on pull requests that change the backend; x86_64).
Phase 2: [docs/evidence/g1-phase2-executor.md](../../docs/evidence/g1-phase2-executor.md).
