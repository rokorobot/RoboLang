# G1 phase 1: the stand-in and the harness (checks P1-P6)

Date: 2026-11-19. Code: [`backends/unitree_g1/`](../../backends/unitree_g1/) (package `robolang-g1`), decision
[0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md) section 7. Simulation only.

**Result: go, on aarch64.** `python -m robolang_g1.harness` exits 0 with P1-P6 all passing, three times: twice with the
`free` stance and once with `--stance supported`. CI (x86_64) is the reference and has not run yet (the workflow only
exists on this branch).

| | free, run 1 | free, run 2 | supported |
|---|---|---|---|
| P1 guard (4 refusals, the stand-in accepted, `Damp` refused) | pass | pass | pass |
| P2 walking, forward / sideways in the start heading (limits 1.6-2.4 m / 0.15 m) | 1.75 / 0.08, 1.74 / 0.02, 1.73 / 0.02 | 1.75 / 0.08, 1.73 / 0.02, 1.74 / 0.02 | 1.79 / 0.09, 1.78 / 0.11, 1.77 / 0.12 |
| P3 coast after the stop (time, distance) | 2.1-2.4 s, 0.12-0.13 m | 2.1-2.4 s, 0.11-0.13 m | 2.0-2.2 s, 0.14-0.15 m |
| P3 torso sway over 10 s at rest (limit 2 cm) | 1.1 cm | 1.1 cm | 0.2 cm |
| P4 arm tracking (limit 0.05 rad) / carry walk with 0.3 kg | 0.029 rad / 2.2 m, no fall | 0.031 rad / 2.1 m, no fall | 0.038 rad / 2.0 m, no fall |
| P5 marker position error at 0.9 m (limit 2 cm) | 0.4 cm median, 0.9 max | 0.5 cm, 0.8 | 0.5 cm, 0.6 |
| P6 simulation / real time, worst second, camera | 1.000, 0.999, 4.99 Hz | 1.000, 1.000, 5.0 Hz | 1.000, 1.000, 5.0 Hz |
| wall time | 91 s | 90 s | 100 s |

The full results are [`harness-free-1.json`](g1-phase1/harness-free-1.json), [`harness-free-2.json`](g1-phase1/harness-free-2.json)
and [`harness-supported.json`](g1-phase1/harness-supported.json) (`stance_model`, `versions`, `arch` and every check's
details are in them). `harness-free-1.json` is reproduced at the end. A fourth run with the final code and the default
stance, the verification run, also exited 0. The simulation is deterministic: the two `free` runs differ only by the
timing of when a command reaches the simulation (P2 run 1 is identical, runs 2 and 3 differ by about 1 cm).

## The timestep finding (this decides U13's step size)

**0046 U13 says physics at dt 0.005. That is numerically unstable for the G1's arm joints. The stand-in runs at dt
0.002 (500 Hz, the robot's own low-level rate), with the PD law evaluated on every physics step and the policy on every
10th step (50 Hz).**

MuJoCo integrates a motor torque explicitly. A velocity feedback gain kd on a joint of effective inertia I is then
stable only if **dt x kd / I < 2**. The policy's `deploy.yaml` gives the arm and wrist joints kd = 10. In the model, the
effective inertia of the arm joints (the mass-matrix diagonal, armature included, standing pose) runs from **0.011** to
0.325 kg m^2 (smallest on the shoulder yaw and the wrists). So:

| dt | worst dt x kd / I over the 14 arm joints | arm torque saturated, 3 s at rest | pelvis drift in 3 s at rest | 4 s walk at 0.5 m/s (forward, sideways) | after the walk, zero command |
|---|---|---|---|---|---|
| 0.005 | **4.53** (unstable) | **67%** of arm control steps at the torque limit | **39 cm** | 2.63 m, 0.04 m | never at rest in 20 s, still moving 0.27 m in the last 10 s |
| 0.002 | **1.81** (stable) | 0% | 0.5 cm | 1.78 m, -0.02 m | at rest after 2.2 s, coast 0.135 m, then 1.0 cm torso sway in 10 s, no fall |

(`docs/evidence/g1-phase1/timestep_check.py`; the dt 0.005 row is chaotic, other runs gave 64% / 49 cm and a fall.) At
0.005 the arm torques chatter bang-bang between +-25 N m, the shoulders sit 0.6 rad off their targets, and everything
downstream is irreproducible. This is the cause of three earlier results that were wrong:

* the spike's and the first phase 1 experiments' conclusion that **the policy never comes to rest after walking** (it
  creeps at about 0.15 m/s): at dt 0.002 it comes to rest by itself (next section);
* the "10 cm drift while standing" and the larger x86/aarch64 differences seen at 0.005: standing drift is 0.5 cm at
  0.002, and a walk is repeatable to the centimetre between runs;
* my own first diagnosis in this phase, that the Dex3 finger meshes touch the thighs and spoil walking. It was the same
  chatter: at dt 0.002 the models with and without finger collisions walk alike (hand decision below).

The margin at 0.002 is not large (1.81 against 2): a client that gives these joints kd above about 11 through `arm_sdk`
would reintroduce it. The harness uses kd 5 for the arms and 2 for the wrists (Unitree's `g1_arm7_sdk_dds_example.py` uses kp 60 / kd 1.5).
Halving dt again (0.001) would double the margin at about twice the CPU.

## Standing free after walking, and what `free` costs

With dt 0.002 the whole-body policy needs no support:

* After the 4 s walk at 0.5 m/s it **coasts 0.11-0.13 m over 2.1-2.4 s** (P3, measured separately), then stands with
  **about 1 cm of torso sway over 10 s** (1.1 cm in the harness runs, 0.6-0.8 cm in standalone ones). From 0.3 m/s it coasts 0.05 m.
* **It does not stand forever.** At zero command it drifts for 40-50 s and then takes one **recovery step of 0.26-0.34
  m**, then stands again, repeating about every 45 s (200 s at zero command, `free_stance_long.py`: from rest steps at
  50, 95, 145 and 190 s; after a 4 s walk at 45, 90, 135/140 and 185 s). It never fell. P3 (a 10 s window at rest)
  passes, but a free-standing robot is not a fixed base for arm work longer than about 40 s.
* `supported` (the approved fallback, 0046 open point 1) welds the pelvis to a mocap body while the robot stands, as the
  parent's spike recipe (`free_stance.py`, mode `weld`): 2 s of policy at zero command, weld at the current pose, the
  mocap moved to upright over 1 s, legs eased to the standing pose over 3 s, held. It never steps (sway 0.2 cm) and
  releases on the next non-zero velocity with a clean policy history. A command arriving in the middle of the settle
  releases it too (tested from every phase: no fall, the walk is the same).

`free` is the default; `--stance supported` selects the fallback. The harness records the stance as `stance_model` and
the go rule uses the one it actually ran with.

## What was run

* This sandbox: aarch64 Debian 12, 4 vCPU shared with other jobs (load average about 18), Python 3.11.2, MuJoCo 4.0.1,
  onnxruntime 1.32.0, numpy 2.4.6, opencv-python-headless 5.1.0.95, cyclonedds 0.10.2 against the C library 0.10.5
  (`abb064e`), unitree_sdk2py 1.0.1 (`814556d`). No GPU: EGL on Mesa llvmpipe. Pins are in
  `scripts/fetch_assets.sh` and in each `harness.json`.
* `bash backends/unitree_g1/scripts/fetch_assets.sh` (exit 0, versions JSON; idempotent; also run from nothing in a fresh
  virtual environment and a fresh assets directory: about 30 s, 199 MB), `python -m pytest -q backends/unitree_g1/tests`
  (34 passed with `MUJOCO_GL=egl`; 33 passed and one skipped without), `MUJOCO_GL=egl python -m robolang_g1.harness --out
  harness.json` (and `--stance supported`), `python -m ruff check .` (clean).
* Experiments, each a script here, reading MuJoCo state (they are experiments, not the backend):
  `g1-phase1/timestep_check.py`, `g1-phase1/hand_model_check.py`, `g1-phase1/free_stance_long.py`.

## U1: the hand model

**Chosen: unitree_rl_gym `g1_29dof_with_hand.xml`** (`276801e`, BSD-3), rebuilt by `scene.py` into one layout: 43 torque
actuators, the 29 body joints in SDK order 0-28, then left hand (7), right hand (7), each in Dex3 order. Candidates and
evidence:

| | unitree_rl_gym `g1_29dof_with_hand.xml` | mujoco_menagerie `g1_with_hands.xml` |
|---|---|---|
| Body | the same as `unitree_mujoco` `g1_29dof.xml`, the model the policy is deployed on in simulation: 35.11 kg without hands, 36.17 kg with (+1.06 kg for both hands); same joint ranges and force limits (e.g. left hip roll +-88 N m: `g1_29dof_with_hand.xml:72`, `g1_29dof.xml:85-86`) | `g1_29dof_with_hand_rev_1_0` (line 1): the rev 1.0 waist and torso meshes (`torso_link_rev_1_0.STL`, line 43), 34.39 kg: a different body than the policy's |
| Actuators | torque `<motor>`s, no gains (lines 341-385), as `unitree_mujoco` (`g1_29dof.xml:387-421`) | position actuators, `kp=500` (line 10), `integrator implicitfast` (line 4), hip-roll force range 139 (line 419) against 88 in Unitree's: re-tuned, would need converting to torques |
| Joint order | **hands are interleaved with the arms**: left arm 15-21, left hand 22-28, right arm 29-35, right hand (actuators 357-384). Body joints 0-28 are not contiguous | same interleaving (lines 417-464) |
| Joint dynamics | none (no armature, damping or friction): added from `unitree_mujoco` (damping 0.05, armature 0.01, friction 0.2, wrists 0.1: `g1_29dof.xml:4-18`) | armature 0.01, friction 0.3 |
| Hands | Dex3-1: palm in the wrist link, thumb 0-2, index 0-1, middle 0-1 per hand (joints 202-323) | the same hand, same meshes |
| Camera mount | `g1_29dof_with_hand.urdf:623-624`: `d435_joint` on `torso_link`, xyz 0.0576235 0.01753 0.41987, rpy 0 0.8307767 0 | none |

Walking with hands (P2's walk, 4 s at 0.5 m/s after 3 s standing; `hand_model_check.py`, dt 0.002, hands held by PD):

| model | forward | sideways | heading | rest drift 3 s |
|---|---|---|---|---|
| unitree_mujoco `g1_29dof` (no hands) | 1.78 m | -0.08 m | -3.1 deg | 0.6 cm |
| unitree_rl_gym with hands | 1.75 m | -0.08 m | -1.5 deg | 0.6 cm |
| unitree_rl_gym with hands, fingers not colliding with the robot | 1.79 m | -0.08 m | -2.2 deg | 0.5 cm |
| menagerie with hands (converted to torque motors) | 1.78 m | -0.09 m | -2.7 deg | 0.6 cm |
| menagerie with hands, fingers not colliding | 1.80 m | -0.09 m | -2.7 deg | 0.6 cm |

**Walking still passes with hands, with either model; the hands cost nothing at dt 0.002.** (At dt 0.005 the same
script gives 1.13-1.52 m and 26-31 cm of drift for the hand models, and 64-70% saturated arm torque for all five: the
timestep finding above, not the hands.) I chose the rl_gym model because it is the body the policy is deployed on plus
the hands, with torque motors and Unitree's own limits, so nothing but the hands differs; the menagerie model would
have meant converting position actuators and a different waist.

What `scene.py` changes in the model, and why:

* actuators rebuilt in SDK order by joint name; every index is looked up by name (`Robot`), none assumed;
* joint armature, damping and friction as in `unitree_mujoco` (above); hand joints armature 0.005, damping 0.02 (the Dex3
  PD, kp 1.5 / kd 0.2 as in Unitree's hand examples, needs the armature to be stable);
* the 14 **finger links are drawn and collide as capsules**, not their meshes (joints, masses and inertias are
  Unitree's): the finger meshes are 240k triangles and cost 120 ms of the 250 ms a head-camera frame took on the CPU,
  which would break the 5 Hz camera. The palm keeps its mesh. Parts the head camera cannot see (legs, pelvis, waist,
  torso) and the duplicate collision meshes are left out of the camera's render (geom groups), which brings a frame to
  about 70 ms;
* a mocap body `support` welded (inactive) to the pelvis for the supported stance, and an inactive weld per object and
  hand for `attach`;
* `statistic extent` 4 and `znear` 0.002: the camera sits inside the head link, and with the default near plane (a
  fraction of the model's extent) a black band cut across every frame;
* the camera: Unitree's mount (above), pitched down 0.83 rad; the field of view (58 degrees vertical, 640x480) is the
  spike's choice and is not checked against a D435.

## Differences from 0046

1. **dt 0.002, not 0.005** (U13), and the policy on every 10th step. See above. 0046 and the spike evidence should be
   corrected; this is the parent's to do (`spec/decisions/` and `docs/evidence/g1-spike*` were not touched).
2. **`free` stands, and is the default; `supported` is the fallback** (U4 and open point 1). Both stances implement the
   `Stance` interface (`sim/stance.py`). `free` takes a recovery step of about 0.3 m every 45 s.
3. **Walking is measured as a stand-in's walk, not Unitree's**: the P2 window (1.6-2.4 m) is met at 1.73-1.79 m, in the
   lower half; the sideways drift is measured in the robot's heading at the start of each walk, as asked.
4. **`robolang_sim` has more than the identity API**: `status` (real-time ratio, worst second, camera rate, stance) and
   `attach`/`detach` (a MuJoCo weld of an object to a hand, `grasp_model: weld_after_hand_close`, U7). Simulation time is
   in `LowState_.tick` (ms), the identity, the status and every camera frame's JSON. The identity API is required to
   report the domain it is on.
5. **`rt/arm_sdk` blend**: torque = (1-w) x the stance's own PD + w x the user's PD, joints 12-28; the last command stays
   in force if the publisher stops. The arm weight is `motor_cmd[29].q`. Hands: `HandCmd_` PD, default kp 1.5 / kd 0.2.
6. **`sport` service**: `SetVelocity` (clipped to the policy's trained range, `duration` honoured in simulation time,
   `duration <= 0` refused with 3204), `GetFsmId` (always 500), `SetFsmId` (500 accepted, every other id refused with
   **7400**, `Damp` included). `LocoClient.Damp()` in this SDK version swallows the return code, so the harness checks
   `SetFsmId(1)`. Everything else is "not implemented" (3203), `SwitchToUserCtrl` included.
7. **Guard (S501)**: domain 0 or any interface but `lo` are refused before any DDS participant exists (the harness checks
   `ChannelFactory` was never initialised); then the identity API must answer in time, say `simulation: true`, name the
   same domain, and the simulation clock must advance. No flag skips a step. Missing identity: the harness runs a robot
   that answers `sport` and counts its requests: 0 were received.
8. **P1 also checks** that the guard accepts the stand-in and that `Damp` is refused with the robot still standing. **P6
   also checks** the `lowstate` rate (250 Hz, limit 100 Hz) and that `tick` follows simulation time.
9. **P3 is measured at rest**: the coast (time and distance from the stop to rest) is reported on its own; rest means at
   most 1 cm of pelvis travel in 1 s, with a 30 s timeout that fails P3; the 10 s window starts then.
10. **P4's gains are the harness's**: arm_sdk with waist kp 500 / kd 10, arms 200 / 5, wrists 60 / 2. The shoulder
    holds a forward reach against about 7 N m of gravity, so tracking within 0.05 rad needs kp above about 150; the
    Unitree example's kp 60 would give 0.12 rad.
11. **P5 walks to the marker first**: the robot's walk drifts and turns, so the harness steers from the oracle
    (forward, sideways and turn commands through `LocoClient`) to a place 0.8 m in front of the marker's face, and measures
    there at 0.89-0.93 m. The harness uses the oracle for this and for judging P2-P5; the backend never will.
12. **Site**: `scene.py` defines `robolang-g1-site/1` (floor, tables as boxes, objects with a size and an ArUco 4x4_50
    marker on a chosen face, stations with their marker panels, the robot's start pose). The phase 1 site is
    `sites/phase1.json`: a 16 m floor, the robot starting 9.5 m from the table (the three P2 walks, the carry walk and
    the approach are about 10 m), a table, a 12 cm box with marker 7 and a 8 cm payload with marker 8.
13. **Dependencies**: `pyproject.toml` lists U10's packages but **not `unitree_sdk2py`**: `fetch_assets.sh` installs it from
    the pinned checkout (`--no-deps`, with the SDK's `opencv-python` requirement changed to the headless build in the
    fetched `setup.py`, the only edit to a fetched file), because a direct reference makes pip fetch it again and pull in
    the GUI OpenCV next to the headless one. The `sdk` extra records the pin. `deploy.yaml` is read with regular
    expressions, not PyYAML (U10: no other dependencies). The CycloneDDS C library is built from `releases/0.10.x`
    (0.10.5, `abb064e`) and the Python binding is 0.10.2 (what the SDK pins), as in the spike.
14. `unitree_rl_gym` is fetched sparse (its `g1_description` only), for the hand model and the camera mount.

## Open issues

* **Not run on x86_64.** CI is the reference; the numbers above are aarch64. P2 passes with margin on forward distance
  (1.73-1.79 against 1.6) and on sideways drift (0.02-0.12 against 0.15) but the `supported` runs sit closest to the
  sideways limit; P3's `free` sway (1.1 cm against 2 cm) is the other tight one.
* **The workflow has not run.** `g1-sim.yml` is untested (a workflow can only be dispatched once it is on the default
  branch). EGL on `ubuntu-latest` and the CycloneDDS multicast-off fallback on `lo` ("selected interface lo is not
  multicast-capable: disabling multicast") worked here and are assumed there.
* **`free` steps about every 45 s.** The executor's `grasp` and `place` must expect it, or the arm work should run in
  `supported`. A step while an object is welded to a hand is not tested.
* **Headroom of the 5 Hz camera on slower machines**: a frame takes 60-70 ms here with the load above, but about 250 ms
  before the render was trimmed, and the stand-in drops frames rather than slowing physics. CI should show `camera.hz`.
* The camera's field of view is not checked against a D435. The marker error (0.4-0.9 cm) is for a 8.6 cm marker at 0.9
  m, straight on.
* `GetFsmId` always answers 500; the standing/walking distinction of U8's `robot_ready` is in the oracle and the
  `status` API, not in `sport`.

## x86_64 on the pull request (#174)

- [Run 1](https://github.com/rokorobot/RoboLang/actions/runs/44559535665) passed, with the same results as aarch64
  within a few centimetres: P2 1.72–1.74 m forward and at most 8 cm sideways; P3 coast 0.11–0.16 m, then sway at
  most 1.12 cm; P5 0.49 cm median and 0.94 cm maximum marker error at 0.91 m; P6 ratio 1.000.
- [Run 2](https://github.com/rokorobot/RoboLang/actions/runs/44561181964), on a commit that changed only documents,
  **failed P1**: `simulation time does not advance (0.640 then 0.738)`. The stand-in had just started and was still
  running slowly (0.1 simulated s in at least 0.4 s of real time). The guard demanded a quarter of real time within
  0.4 s, so it confused a slow start with a frozen clock. **Fixed:** the guard now requires the clock to advance at
  least 0.1 simulated s before its timeout, and refuses a frozen clock. How fast the simulation runs is P6's
  question, not the guard's. Two unit tests cover the fix: a slow but moving clock is accepted, and a frozen one is
  refused.

## `harness-free-1.json`

```json
{
  "checks": {
    "P1": {
      "pass": true,
      "details": {
        "domain_0": {
          "refused": true,
          "code": "S501",
          "reason": "DDS domain 0 is the real robot's domain; the simulation backend never uses it",
          "dds_participant_created": false
        },
        "interface_eth0": {
          "refused": true,
          "code": "S501",
          "reason": "network interface 'eth0' is not the loopback interface 'lo'",
          "dds_participant_created": false
        },
        "identity_missing": {
          "refused": true,
          "code": "S501",
          "reason": "no answer from the simulation identity API (robolang_sim) within 3.0 s: this is not the simulator",
          "requests_received_by_the_robot": 0
        },
        "stand_in_accepted": {
          "simulation": true,
          "domain": 1,
          "interface": "lo",
          "sim_time": 1.1040000000000008,
          "stance_model": "free",
          "grasp_model": "weld_after_hand_close"
        },
        "damp_refused": {
          "code": 7400,
          "still_standing": true,
          "fsm_id_after": [
            0,
            500
          ]
        }
      }
    },
    "P2": {
      "pass": true,
      "details": {
        "runs": [
          {
            "set_velocity_code": 0,
            "forward_m": 1.754,
            "sideways_m": -0.08,
            "heading_change_deg": -2.6,
            "fallen": false,
            "walked_s": 4.0,
            "pass": true
          },
          {
            "set_velocity_code": 0,
            "forward_m": 1.735,
            "sideways_m": -0.02,
            "heading_change_deg": -1.3,
            "fallen": false,
            "walked_s": 4.0,
            "pass": true
          },
          {
            "set_velocity_code": 0,
            "forward_m": 1.728,
            "sideways_m": -0.017,
            "heading_change_deg": -1.0,
            "fallen": false,
            "walked_s": 3.98,
            "pass": true
          }
        ],
        "command": "SetVelocity(0.5, 0, 0, 4.0)",
        "frame": "robot heading at the start of each walk"
      }
    },
    "P3": {
      "pass": true,
      "details": {
        "runs": [
          {
            "stance_model": "free",
            "settled": true,
            "pass": true,
            "coast_s": 2.13,
            "coast_m": 0.134,
            "torso_sway_cm_xyz": [
              1.14,
              0.1,
              0.1
            ],
            "window_samples": 501,
            "window_start_after_stop_s": 2.13
          },
          {
            "stance_model": "free",
            "settled": true,
            "pass": true,
            "coast_s": 2.35,
            "coast_m": 0.115,
            "torso_sway_cm_xyz": [
              1.08,
              0.05,
              0.11
            ],
            "window_samples": 500,
            "window_start_after_stop_s": 2.35
          },
          {
            "stance_model": "free",
            "settled": true,
            "pass": true,
            "coast_s": 2.33,
            "coast_m": 0.124,
            "torso_sway_cm_xyz": [
              1.07,
              0.04,
              0.11
            ],
            "window_samples": 500,
            "window_start_after_stop_s": 2.33
          }
        ],
        "stance_model": "free",
        "settled": "pelvis path <= 1 cm in 1 s, within 30 s of the stop; the sway window starts then"
      }
    },
    "P4": {
      "pass": true,
      "details": {
        "tracking_max_error_rad": 0.0288,
        "tracking_worst_joint": 22,
        "tracking_tolerance_rad": 0.05,
        "hand_tracking_max_error_rad": 0.0114,
        "attach": {
          "code": 0,
          "object": "payload",
          "hand": "right",
          "relative_position": [
            0.0901,
            0.0,
            -0.0
          ]
        },
        "carry_walk": {
          "walked_m": 2.16,
          "payload_kg": 0.3,
          "fallen": false,
          "payload_attached": true,
          "stood_again": true,
          "arm_error_after_rad": 0.008
        }
      }
    },
    "P5": {
      "pass": true,
      "details": {
        "approach": [
          {
            "range_m": 2.87,
            "ahead_m": 1.59,
            "left_m": -1.36,
            "turn_deg": -23.5
          },
          {
            "range_m": 1.21,
            "ahead_m": 0.38,
            "left_m": -0.05,
            "turn_deg": 1.0
          },
          {
            "range_m": 0.89,
            "ahead_m": 0.03,
            "left_m": 0.01,
            "turn_deg": 1.2
          }
        ],
        "frames": [
          {
            "frame": 399,
            "sim_time": 79.8,
            "ids": [
              7
            ],
            "error_m": 0.0092,
            "range_m": 0.89
          },
          {
            "frame": 400,
            "sim_time": 80.0,
            "ids": [
              7
            ],
            "error_m": 0.0057,
            "range_m": 0.89
          },
          {
            "frame": 401,
            "sim_time": 80.2,
            "ids": [
              7
            ],
            "error_m": 0.0039,
            "range_m": 0.889
          },
          {
            "frame": 402,
            "sim_time": 80.4,
            "ids": [
              7
            ],
            "error_m": 0.0032,
            "range_m": 0.889
          },
          {
            "frame": 403,
            "sim_time": 80.6,
            "ids": [
              7
            ],
            "error_m": 0.0033,
            "range_m": 0.89
          },
          {
            "frame": 404,
            "sim_time": 80.8,
            "ids": [
              7
            ],
            "error_m": 0.0035,
            "range_m": 0.89
          }
        ],
        "range_m": 0.89,
        "detected_frames": 6,
        "median_error_m": 0.0037,
        "max_error_m": 0.0092
      }
    },
    "P6": {
      "pass": true,
      "details": {
        "harness_ratio": 1.0,
        "harness_wall_s": 76.5,
        "lowstate_hz": 250.0,
        "lowstate_tick_s_minus_sim_time_s": 0.01,
        "stand_in": {
          "sim_time": 80.88799999999289,
          "wall_elapsed": 80.887,
          "realtime_ratio": 1.0,
          "worst_second_ratio": 0.9988,
          "stalls": 0,
          "slow_windows": [],
          "camera": {
            "frames": 404,
            "dropped": 0,
            "render_ms": 67.9,
            "hz": 4.99
          },
          "stance": {
            "model": "free",
            "phase": "standing"
          },
          "velocity_command": [
            0.0,
            0.0,
            0.0
          ],
          "fell": false
        }
      }
    }
  },
  "stance_model": "free",
  "versions": {
    "arch": "aarch64",
    "python": "3.11.2",
    "assets": {
      "unitree_mujoco": "1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d",
      "unitree_rl_lab": "4960b84732b0c2ec593dccbfe963fda1bcd7b1e3",
      "mujoco_menagerie": "370c43965323c0944b433b202aac5058b5f907a9",
      "unitree_rl_gym": "276801e46c5d433564f24658bac64f254b7d2d4b",
      "unitree_sdk2_python": "814556d15970dd2ecf1c9984e845ca02ab07e206",
      "cyclonedds_c": "abb064e35b6fbc79554c255ff8b371333e87b029"
    },
    "packages": {
      "mujoco": "4.0.1",
      "onnxruntime": "1.32.0",
      "numpy": "2.4.6",
      "opencv-python-headless": "5.1.0.95",
      "cyclonedds": "0.10.2",
      "unitree_sdk2py": "1.0.1"
    }
  },
  "arch": "aarch64",
  "stand_in_ready": {
    "ready": true,
    "pid": 20678,
    "domain": 1,
    "interface": "lo",
    "stance_model": "free",
    "run_dir": "/tmp/ev/free1/harness.run",
    "versions": {
      "arch": "aarch64",
      "python": "3.11.2",
      "assets": {
        "unitree_mujoco": "1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d",
        "unitree_rl_lab": "4960b84732b0c2ec593dccbfe963fda1bcd7b1e3",
        "mujoco_menagerie": "370c43965323c0944b433b202aac5058b5f907a9",
        "unitree_rl_gym": "276801e46c5d433564f24658bac64f254b7d2d4b",
        "unitree_sdk2_python": "814556d15970dd2ecf1c9984e845ca02ab07e206",
        "cyclonedds_c": "abb064e35b6fbc79554c255ff8b371333e87b029"
      },
      "packages": {
        "mujoco": "4.0.1",
        "onnxruntime": "1.32.0",
        "numpy": "2.4.6",
        "opencv-python-headless": "5.1.0.95",
        "cyclonedds": "0.10.2",
        "unitree_sdk2py": "1.0.1"
      }
    }
  },
  "stand_in_final": {
    "stopped": true,
    "sim_time": 80.89199999999288,
    "wall_elapsed": 81.079,
    "realtime_ratio": 0.9977,
    "worst_second_ratio": 0.9988,
    "stalls": 0,
    "slow_windows": [],
    "camera": {
      "frames": 404,
      "dropped": 0,
      "render_ms": 67.9,
      "hz": 4.99
    },
    "stance": {
      "model": "free",
      "phase": "standing"
    },
    "velocity_command": [
      0.0,
      0.0,
      0.0
    ],
    "fell": false
  },
  "go": true,
  "elapsed_s": 90.7
}
```
