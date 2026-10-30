# Decision record 0029: H3, a ROS 2 execution backend in physics simulation

**Date:** 2026-10-30
**Status:** **PROPOSED.** Design only, for owner decision. No code. **Physical execution stays unauthorized**
(development plan A5; strategy H4).
**Applies to (when decided):** a new optional package `robolang-ros2` (not part of `pip install robolang`), a new
`spec/ros2-backend-v0.1.md`, the run record (a `backend` field), a CI workflow, and benchmark acceptance.
**Evidence and direction:** [STRATEGY](../../docs/STRATEGY.md) pillar P4 and horizon H3 ("one benchmark task family
passes end to end in physics simulation, verified through sensor observations, not ground truth"). RFC-001 sections
5 and 14 (ROS 2 is an *execution backend*), 11.3 (evidence) and 13.6 (RoboLang limits are not safety functions).
Today every run is symbolic (`simulation-v1.0.md`), and its evidence is simulator ground truth (section 10.5).

## 1. What changes

The same `.rob` task, unchanged, with the same goal and bindings, runs on a simulated robot arm in Gazebo through
ROS 2. Success is decided from what the robot's **sensors observed**, through the same goal evaluation and
`@verify` rules as today. The symbolic simulator stays the fast, deterministic reference. The physics backend is
the first step towards real execution, and still moves nothing real.

```
.rob ──validate──▶ Task IR ──▶ robolang-ros2 executor ──▶ MoveIt 2 / ros2_control ──▶ Gazebo (physics)
                                     ▲                                                  │
                                     └──── observations (camera, joint states) ◀────────┘
                                     goal evaluated from observations → run record + Core outcome
```

## 2. Decisions proposed (recommendation first; owner picks)

| # | Question | Recommended | Alternatives |
|---|---|---|---|
| X1 | Simulator | **Gazebo Jetty** (LTS to 2031) through `ros_gz` | Isaac Sim: needs an NVIDIA GPU (none in CI or the sandbox) and has a heavier licence. MuJoCo: light and fast, but not ROS-native, so the backend would test less of what real robots run |
| X2 | ROS 2 version | **Lyrical Luth** (LTS to May 2031), the distribution `ros_gz` pairs with Jetty in binaries. It runs in the official `ros:lyrical` container on GitHub Actions (Ubuntu 26.04, amd64) | Jazzy with Gazebo Harmonic (LTS to 2029): the fallback if a needed package is missing for Lyrical. Checked in phase 1 before any code depends on it |
| X3 | Robot | **One arm with a gripper**, from a maintained Gazebo and MoveIt 2 configuration (a Universal Robots arm with `ur_simulation_gz`, or Franka). Chosen in phase 1 by which has working Lyrical/Jetty packages. Mobile bases (Nav2) and two robots come later | Write our own robot model: no. A mobile base first: navigation adds a whole stack before the first pick and place |
| X4 | How a plan step runs | `move r to L`: MoveIt 2 to the pose a **site file** gives for location `L`. `grasp`: approach, close the gripper, then attach. `place`: move, release, detach. `inspect`: read the bound camera. `wait for`/`wait until`: on the simulation clock. `communicate`: publish on a topic. A step fails with the existing `S2xx` codes, plus new `S5xx` codes for backend failures (no motion plan, controller error, timeout) | One ROS 2 action per RoboLang statement written by us: more code, the same result. Calling Gazebo directly: skips the ROS 2 stack the backend is supposed to exercise |
| X5 | Grasping | **Attach on grasp** (Gazebo detachable joint) after the gripper closes on the object. The limitation is stated in every record: contact physics of the grasp is not what is being tested | Pure friction grasping: notoriously unreliable in Gazebo, and would make the acceptance run measure grasp tuning instead of RoboLang |
| X6 | Evidence (the point of H3) | Goal atoms are evaluated from **observations only**: `object_at` from an overhead camera and **fiducial markers** on objects (observed pose inside the location's region, from the site file); `holding` from the gripper state together with the camera; `robot_at` from joint states and TF. Each observation has a confidence and an age, so `@verify min_confidence` and `max_age` apply unchanged. Gazebo ground truth is used **only in tests**, as an oracle to measure agreement, and never as evidence | Ground truth as evidence: that is the symbolic simulator again. Learned object detection: more realistic, but adds a model to the trust chain before we need one |
| X7 | Safety mode | The backend runs only in **simulation mode**. It refuses to start unless the ROS graph runs on simulated time and the robot is a declared Gazebo model. There is no flag that turns this off. The record says `"mode": "simulation"` | A configuration switch for physical mode: not before H4 and a written safety case (strategy rule; plan A5) |
| X8 | Packaging | A separate package **`robolang-ros2`** (ament_python) in `backends/ros2/` in this repo, depending on `robolang`. `robolang` itself stays dependency-free and never imports it | Put it inside `robolang`: it would pull ROS 2 into the installable language package, which `pyproject.toml` explicitly forbids today |
| X9 | Records and determinism | Physics is not bit-for-bit repeatable. Run records gain `backend: {kind: "ros2-gazebo", versions…}` and keep the same goal, outcome and Core shape. Physics runs are compared by outcome, not byte for byte. Symbolic runs keep byte-identical records | Force determinism (fixed seeds, single thread): helps, so we do it, but it cannot be promised |
| X10 | Where it runs | A container image built from `ros:lyrical`, run in a separate GitHub Actions workflow on demand and nightly, headless, rendering in software for the camera. Not on every PR | Every PR: too slow (minutes per task). This sandbox: Debian 12 arm64 without Docker, so it is not a supported ROS 2 platform |

## 3. How it is judged

**Acceptance (the H3 exit criterion for this part):** one benchmark task family passes end to end.

1. **The family.** The pick-and-place and precondition tasks of benchmark set 1 that fit one arm. Each is rewritten
   only in its world (a Gazebo scene and site file with the same entity IDs), never in its `.rob`.
2. **Same program.** Each reference `.rob` validates and passes the symbolic simulator, then runs unchanged in
   physics.
3. **Pass bar:**
   - at least 90% of the family's runs end `SUCCEEDED`, verified from observations, over 3 repeats;
   - observed goal verdicts agree with the ground-truth oracle on at least 98% of goal checks;
   - **0** runs report `SUCCEEDED` when the oracle says the goal does not hold. A false success is the one result
     that blocks acceptance outright.
4. **Failure cases** (scripted): object absent, object moved before grasp, a location out of reach, a camera covered.
   Each must end FAILED or UNKNOWN with a code, never SUCCEEDED.

## 4. Delivery (after approval; each its own PR)

1. **Harness:** the container image, Lyrical/Jetty and the robot packages verified (X2, X3 decided here), a scene with
   a table, a shelf, a box and a camera, and a CI job that starts it headless and moves the arm once.
2. **Executor:** `move`, `grasp`, `place` and `wait` through MoveIt 2, with the `S5xx` codes, the simulation-only
   guard and run records with `backend`.
3. **Observations:** fiducial-based `object_at`, `holding` and `robot_at`, with confidence and age, plus the
   ground-truth oracle used in tests.
4. **Acceptance run** and its evidence document.

## 5. Consequences and risks

- **This is the largest dependency step so far.** It brings in ROS 2, Gazebo, MoveIt 2 and a robot description. The
  language package stays untouched and dependency-free (X8).
- **Headless rendering in CI may be slow or fragile.** Phase 1 measures it before anything depends on the camera.
  If it fails, the fallback is a simulated pose sensor with added noise. That keeps observation-based verification,
  and the evidence document must state the downgrade.
- **Package gaps on a new LTS.** If Lyrical lacks a robot or MoveIt package, X2 falls back to Jazzy/Harmonic. That is
  recorded, not hidden.
- **INTEROP-3** relaunches on frozen Core 0.2 within H3 ([0023](0023-keep-interop-3-deferred.md)). It is not planned
  here and needs an outside implementer.
- **Physical mode** remains out of scope until H4. This record adds no way to reach hardware.
