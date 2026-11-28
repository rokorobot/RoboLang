# G1 phase 4 prerequisites: the heading from the IMU, and keeping pace with real time

Date: 2026-11-23 (the work), written up 2026-11-26. Code: [`backends/unitree_g1/`](../../backends/unitree_g1/), decision
[0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md); contract in
[`g1-backend-v0.1.md`](../../spec/g1-backend-v0.1.md), "The IMU" and "Time". Simulation only.

**Status: the work was stopped by the owner on 2026-11-23 and is taken as it stands** (pull request #181). This page is
written from that pull request's results, its commits and the measurements in [`g1-phase4-prereqs/`](g1-phase4-prereqs/);
nothing here was measured again for this page, except the x86 results at the end.

## 1. Heading between marker fixes

`move` used to dead-reckon its heading from the yaw rates it commanded. Under the stand-in's heading creep that belief
was up to 0.4 rad out before the next station marker came into view, and the marker was then not found (S517).

- `move` now takes heading changes from the IMU's yaw in `rt/lowstate`, with a bias it learns while the robot is known
  to be still ([`heading.py`](../../backends/unitree_g1/robolang_g1/heading.py)).
- The stand-in's IMU has bias, noise and yaw drift ([`sim/imu.py`](../../backends/unitree_g1/robolang_g1/sim/imu.py)), so
  the change is not judged on a perfect sensor.
- `move` aims against the heading creep the IMU reads, and walks a leg that is mostly sideways as two diagonals: the
  walking policy answers a pure sidestep unpredictably.

## 2. Keeping pace with real time

- `lowstate` and the hand states are published from patched byte buffers
  ([`sim/fastdds.py`](../../backends/unitree_g1/robolang_g1/sim/fastdds.py)), and the camera thread's priority is
  lowered, so the stand-in does less work in each 2 ms step.
- Every slow second is reported with the loop's CPU time and its wait for a CPU, in the run records and the CI summary.

## 3. Results (aarch64, limited to 4 CPUs, judged by the oracle)

| Move stress, 20 arrivals each | Shelf | Bench |
|---|---|---|
| Carrying the box, emulated creep, before | 15/20 (5 × S517) | 19/20 (1 × S504) |
| Carrying the box, emulated creep, after | 19/20 (1 × S504) | 19/20 (1 × S504) |
| Carrying the box, no extra creep, after | 20/20, ≤ 4.0 cm, ≤ 0.045 rad | 20/20, ≤ 4.2 cm, ≤ 0.051 rad |

Phase 1 (P1–P6) and phase 2 (E1–E7) passed, with every phase 2 run real-time valid. Data:
[`g1-phase4-prereqs/data/`](g1-phase4-prereqs/data/).

## 4. What was open when the work stopped, and what happened to it

| Open on 2026-11-23 | State on 2026-11-26 |
|---|---|
| No phase 3 run on the final code | **Done on x86_64:** P1–P6, E1–E7 and O1–O7 pass in one CI run of 56 minutes (pull request #192, job `harness`) |
| The real-time change unproven on x86 | **Done:** the same run; the phase 4 acceptance run reports each run's real time ([evidence](g1-phase4-acceptance.md)) |
| x86 CI of #181 failed E2 | **Fixed in the check, not the backend:** the out-of-reach scene's 8 cm box, 0.61 m ahead, was read with confidence 0.4 when the robot stopped 3 cm short, so the grasp ended S506 (not seen) where the scene is there to check S502 (out of reach). Both refuse before the arm moves. The scene now uses a 15 cm box |
| Under emulated creep, 1 arrival in 20 times out (S504) | **Still open.** The acceptance run counts such a run as failed |
| No evidence write-up | This page |
