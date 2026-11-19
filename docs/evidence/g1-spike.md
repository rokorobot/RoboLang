# G1 feasibility spike: MuJoCo, the Unitree SDK and a head camera on CPU

Date: 2026-11-19, two rounds: the first for the draft of decision
[0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md), the second during its review. Throwaway code, not a
backend: the scripts are in [`g1-spike/`](g1-spike/) so the numbers can be reproduced. They read MuJoCo state, but
only as an oracle to judge the spike itself, never as evidence a backend would use.

**Machine:** the Cloudroom sandbox: x86_64, 4 vCPU, 7 GB RAM, **no GPU**, Debian 12, Python 3.11.2. MuJoCo 4.0.1
rendering offscreen through EGL in software (OSMesa also worked).

**Sources, at these commits (all BSD-3-Clause except `unitree_rl_lab`, Apache-2.0):**
`unitree_mujoco` 1eb6642 (2026-09-07; G1 29-DoF MJCF), `unitree_rl_gym` 276801e (2025-07-25; pretrained G1 walking
policy), `unitree_rl_lab` 4960b84 (2025-11-19; whole-body G1 velocity policy `velocity/v0`), `unitree_sdk2_python`
814556d (2026-09-21; installs as `unitree_sdk2py` 1.0.1), `unitree_sdk2` aaf69cb, `mujoco_menagerie` 370c439
(`unitree_g1`, model with hands).

## Results

| Question | Result |
|---|---|
| Does the G1 walk in MuJoCo on this CPU? | **Yes, with either Unitree policy.** `unitree_rl_lab`'s whole-body policy (all 29 joints, ONNX): from rest it stands with 1 cm torso sway, then walks **2.52 m** in 5 s at 0.5 m/s with 2.6 cm sideways drift; 32 s of simulation in 0.48 s (**67× real time**). `unitree_rl_gym`'s legs-only policy (waist and arms held by PD): walked 2.17 m, ≤ 4 cm drift, 18–24× real time |
| Do arm commands upset walking? | **Not with the whole-body policy.** It never fell in 32 s that included a 5 s right-arm reach while standing and 9 s of walking and stopping with both arms overridden to a carry pose (as `rt/arm_sdk` would). The `rl_gym` policy is sensitive: with raised forearms it drifted about 1 m backwards and stopped following the command |
| Can it stand still after walking? | **No: the main open risk.** At zero command after any walk (0.2, 0.3 or 0.5 m/s), the whole-body policy keeps walking forward at **about 0.15 m/s**, turning about 1.5°/s, for at least 15 s. Clearing its history, a reverse pulse (0.6 s at −0.3 m/s), and an untuned supported settle (a spring on the pelvis while the joints ease to the default pose, then release) did not stop it; the last one fell on release. The `rl_gym` policy keeps stepping too (33–49 cm sway over 4 s), and PD holding of the legs fell, from rest and after settling |
| Can the head camera see and identify an object? | **Yes.** A camera at the G1's head camera mount, pitched down, 640×480: **113–148 ms per frame** in software. After the walk it saw a 9 cm ArUco marker (4×4_50, id 7) on a box on the table, and OpenCV detected it ([frame](g1-spike-head.png)) |
| Does the real SDK interface work against a simulated robot? | **Yes.** Unitree's own, unmodified `LocoClient` called a sim-side `sport` RPC service, built on the SDK's `Server` class, over CycloneDDS on loopback (domain 1): in one process (0.6–0.9 ms) and **from a separate process** (0.4–1.3 ms). `SetVelocity(0.4, 0, 0, 2.0)` returned code 0 and the service received `{"velocity": [0.4, 0, 0], "duration": 2.0}`; `GetFsmId` answered too |
| Does the backend need PyTorch? | **No.** The whole-body policy is ONNX and runs with `onnxruntime`. (The `rl_gym` policy, an LSTM 47→64 plus an MLP 64→32→12 in 145 KB, could run in numpy.) |

![G1 after the walk, at the table](g1-spike-overview.png)

## Porting the whole-body policy

In `deploy.yaml`, `stiffness` and `damping` are in **motor (SDK) order**: `State_RLBase.h` sets `motor_cmd[i].kp`
from `joint_stiffness[i]`. `default_joint_pos`, the actions and the observations are in **policy order**, mapped by
`joint_ids_map` (policy index i → motor `joint_ids_map[i]`). Observations are 5-step histories per term, oldest first,
terms concatenated (480 inputs). The first round applied the gains in policy order, which gave the ankles, hip rolls
and waist the wrong stiffness, and the robot fell within 1.5 s; the second round fixed it.

## Setup friction

- `cyclonedds==0.10.2` has no wheel for this Python: the CycloneDDS C library (`releases/0.10.x`, 0.10.5) was built
  from source (about 2 minutes on 4 cores) and needed `python3.11-dev`. `unitree_sdk2py` then installed from its
  repository. Phase 1 should cache this build in CI.
- `unitree_mujoco` simulates only the low-level messages (`LowCmd`, `LowState`, `SportModeState`, IMU); the real G1's
  built-in walking service (`sport`) and the `rt/arm_sdk` blending are **not simulated**. A backend has to provide
  stand-ins for both, which is what the RPC test above shows is possible.
- `SportModeState` carries the robot's position in simulation, but the real robot does not publish it once its
  built-in motion service is off (`unitree_mujoco` README). It must never count as evidence.
- The SDK has camera clients for Go2 and B2, none for G1: on the robot the head camera is read directly.

## Reproduce

```sh
python3 -m venv venv && venv/bin/pip install mujoco numpy pyyaml opencv-python-headless onnxruntime
venv/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu   # only for the rl_gym .pt policy
# CycloneDDS: build releases/0.10.x into $CYCLONEDDS_HOME, then: pip install cyclonedds==0.10.2 ./unitree_sdk2_python
M=unitree_mujoco/unitree_robots/g1/scene_29dof.xml; P=unitree_rl_gym/deploy/pre_train/g1/motion.pt
D=unitree_rl_lab/deploy/robots/g1_29dof/config/policy/velocity/v0
MUJOCO_GL=egl venv/bin/python -I g1-spike/whole_body_policy.py $M $D/exported/policy.onnx $D/params/deploy.yaml
for s in none reset reverse support; do
  venv/bin/python -I g1-spike/settle_after_walk.py $M $D/exported/policy.onnx $D/params/deploy.yaml $s
done
ARMS=zero MUJOCO_GL=egl venv/bin/python -I g1-spike/walk_and_look.py $M $P out/   # rl_gym walk + head camera
MUJOCO_GL=egl venv/bin/python -I g1-spike/stand_still.py $M $P                     # rl_gym standing
venv/bin/python -I g1-spike/loco_rpc.py                                            # LocoClient → sim service
venv/bin/python -I g1-spike/loco_rpc.py serve & venv/bin/python -I g1-spike/loco_rpc.py call   # two processes
```
