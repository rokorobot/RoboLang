# Decision record 0048: NVIDIA Isaac OS as an execution backend (design only)

**Date:** 2026-11-25
**Status:** **APPROVED by the owner, 2026-11-25**: V1–V10 as recommended, and open points 1–4 as recommended
(section 7). Proposed the same day. **Nothing is built yet** (V10): the next step is the spike of section 5, after the G1
acceptance run. Design only: no code, no new dependency, nothing here enables physical
execution ([0038](0038-physical-mode-authorization-stop-approval.md); strategy H4).
**Applies to:** a possible third optional backend package, `robolang-isaac` in `backends/isaac_os/`; one
new hazard in the [safety case draft](../../docs/safety-case-draft.md); a spike on a rented GPU machine.
**Builds on:** [0029](0029-physics-simulation-backend.md) (ROS 2 backend) and
[0046](0046-humanoid-backend-unitree-g1.md) (Unitree G1 backend), unchanged.
**Trigger:** an owner brief of 2026-11-25 on Isaac OS, which NVIDIA announced on 2026-10-21. The brief was checked
against NVIDIA's documentation and source at tag `v0.11.0` (sources in section 8). **Nothing was built or run**: every
statement below about Isaac OS is read from its documentation or source, not measured.

## 1. What Isaac OS is: the brief, checked

| Claim in the brief | Verdict | What the sources say |
|---|---|---|
| Announced 2026-10-21 at GTC Berlin | **Confirmed** | NVIDIA press release of that date |
| Current version 0.11.0 | **Confirmed** | The only tag is `v0.11.0`. Repository created 2026-09-21, last push 2026-10-30, 25 stars |
| "(Beta)" | **Not found** | The documentation, the README and the press release never say beta. The release calls Isaac Lab 3.0 "generally available" and says nothing about Isaac OS's status. It is a 0.x first public tag; treat it as early |
| Not an operating system: a runtime and middleware on NVIDIA Holoscan | **Confirmed, with a precision** | Applications are Holoscan SDK 5.0 graphs. The runtime is the Holoscan SDK; the Isaac OS repository holds "reference applications and developer resources" (operators, libraries, message schemas) |
| Open source | **Confirmed for the repository** (Apache-2.0) | A build needs a **prebuilt** Holoscan SDK 5.0, CUDA 13 and TensorRT. The licence of Holoscan SDK 5.0 was not checked |
| Compiles and validates the graph into an immutable execution plan before running | **Confirmed** | `holoscan::compile(graph)` checks connections, timing rules and resource bounds and returns an immutable `ExecutionPlan` with diagnostics; the application checks `plan.ok()`, then `run_async(plan)` |
| Deterministic real-time control beside AI inference | **Claimed by NVIDIA, not reproduced** | Benchmarks on Jetson AGX Thor only (for example whole-body control p99.9 latency 2.57 → 0.37 ms with GPU partitioning) |
| Whole-body control of humanoids | **Confirmed** | Reference applications for the **Unitree G1**: `wbc_g1` (Agile WBC) and `gr00t_n1_7_wbc_g1` (GR00T N1.7 for arms and hands over the WBC) |
| ROS 2 bridge | **Confirmed, narrower than it sounds** | A C++ library, `ros2_bridge`: **topics only**, same host, eight built-in message pairs (`JointState`, `Imu`, `Pose`, `Twist`, `Transform`, `Odometry`, `CameraInfo`, `Image`). No services or actions are documented |
| Moves applications from simulation to the robot | **Partly** | Modes `mock`, `sim` (Isaac Sim over UCX), `unitree_sim` (Unitree MuJoCo over DDS) and `real`. The documentation warns that mode names "do not guarantee the same graph, model, configuration or timing" |
| Oriented to NVIDIA hardware | **Confirmed, and stronger** | Validated on **Jetson AGX Thor only**. IGX support expected Q1 2027; Jetson AGX Orin and x86_64 are "untested". CUDA is required even to compile a graph without running it. Isaac Sim needs x86_64 and an RTX GPU |
| Validates task conditions "partly" | **Misleading** | The compile step validates the **graph** (wiring, timing, resources). Nothing in Isaac OS knows a task, a goal, a precondition or an authorization |
| Robot-to-robot communication as technical infrastructure | **Not found** | Every documented application is one process on one host. No multi-robot feature is documented |
| AI-to-robot communication at the runtime level | **Not as a task interface** | In 0.11.0 the GR00T operator's inputs are a camera image and joint state. There is **no instruction input**: the task ("apple to plate") is fixed by the checkpoint |

## 2. Three facts the brief missed, which shape the design

1. **Isaac OS has no task-level interface at all.** What goes in is a walking velocity (keyboard or gamepad) or a
   policy fixed to one task. So it does not compete with RoboLang: it is a controller that needs something above it to
   say what to do, to refuse, and to check the result. That is RoboLang's whole job.
2. **On the G1, Isaac OS replaces the robot's own controller.** Its publisher calls
   `MotionSwitcherClient::ReleaseMode()` and then writes `rt/lowcmd` and `rt/dex3/*/cmd` itself. That is exactly what
   `robolang-g1` never does (0046 U9, hazard H25): our G1 backend talks to Unitree's walking service and leaves balance
   to the robot. So Isaac OS is **not a drop-in under the existing G1 backend**. It is a different robot stack: same
   body, NVIDIA's whole-body controller, and a user process that is the balance.
3. **It does not document how to stop.** The real-robot pages say Ctrl+C "is not a documented physical robot stopping
   procedure", that enable keys and upper-body hold "are not an emergency stop", that the `real` deployment
   instructions are "incomplete" and that the `unitree_sim` run procedure is "not documented".

One coincidence helps: Isaac OS's `unitree_sim` mode uses Unitree MuJoCo on **DDS domain 1**, the simulator family and
the domain convention 0046 already chose (U2, U3).

## 3. Decisions (approved as recommended; the alternatives are kept for the record)

| # | Question | Recommended | Alternatives |
|---|---|---|---|
| V1 | Role | **A controller under RoboLang: one execution backend among three** (ROS 2, Unitree's SDK, Isaac OS). The core `robolang` wheel never depends on it (0029 X8). The same `.rob` runs unchanged | Build RoboLang's execution on Holoscan: ties the project to one vendor's GPUs, against the north star ("across vendors"). Ignore Isaac OS: gives up the stack NVIDIA's humanoid partners are likely to deploy |
| V2 | Seam | **ROS 2 topics through Isaac OS's `ros2_bridge`.** The executor stays a Python process, as in both existing backends. It publishes `geometry_msgs/Twist` (walking velocity) and `sensor_msgs/JointState` (arm and hand targets) and subscribes to `JointState`, `Imu`, `Image` and `CameraInfo`. All have built-in converters | A Holoscan operator that reads Task IR inside the graph: C++20, CMake 4 and CUDA 13 in our build, and verification would live inside the vendor's runtime. The plan is immutable once compiled, so plan steps cannot be graph structure anyway |
| V3 | The graph | **One small C++ application in `backends/isaac_os/`**: Isaac OS's own `wbc_g1` graph plus bridge ports, with Isaac OS fetched by CMake `FetchContent` (the documented way; Apache-2.0, not copied) from the owner's fork [`rokorobot/isaac-os`](https://github.com/rokorobot/isaac-os), pinned to commit `a4ab23f` (upstream `v0.11.0`; the fork has no tags). **The fork is an unpatched mirror**: it protects the pin if upstream rewrites or removes it, and it moves only by a deliberate sync. This is the only C++ in the repository | Fetch from upstream directly: one repository fewer, but the pin depends on NVIDIA keeping it. Patch Isaac OS's reference application in the fork: a diverging fork to maintain against a 0.x project |
| V4 | Robot and simulator | **Unitree G1, 29 DoF, Dex3-1 hands** (as 0046 U1; Isaac OS's simulation configuration is the same 29-joint model). Simulator: **`unitree_sim` (Unitree MuJoCo, DDS domain 1) first**, because it needs no RTX GPU. Isaac Sim (`sim`) later, for camera realism | Isaac Sim first: photorealistic, but x86_64 with an RTX GPU, and its own UCX transport |
| V5 | How a plan step runs | `move`: a velocity command to the whole-body controller, closed on the station marker (as 0046 U5). `grasp`, `place`: arm targets from our inverse kinematics, **if** Isaac OS's joint-command combiner accepts them from outside the graph (the spike checks it). **Learned policies are not in the first version** | Drive the arms with GR00T N1.7 from day one: the policy is fixed to one task and takes no instruction, so RoboLang could only start it and watch |
| V6 | Learned policies, later | **A policy is a declared action** (`do`, RoboLang 1.8, [0040](0040-declared-actions.md) A9: "a request to the robot's own controller"). RoboLang starts it, bounds it in time, and decides success **from its own observations**, never from the policy. This is where RoboLang adds the most to a GR00T stack: a policy that reports nothing becomes a step with a verified outcome | A new language construct for policies: no evidence it is needed (strategy rule 1) |
| V7 | Evidence | **Unchanged: observations only** (0046 U8). `plan.ok()` and the plan's diagnostics are a readiness check before the first step, recorded in the run record. They are not evidence of a task outcome. Simulator ground truth is never read | Trust the runtime's own status: it has none about tasks |
| V8 | Stop | **Zero velocity, arms held, hands unchanged, and the graph keeps running.** RoboLang never stops the run session and never ends the controller process while the robot stands: under Isaac OS that process is the balance. RoboLang's executor still never writes `rt/lowcmd` itself | Stop the graph as the stop: for a biped that is a fall (H25) |
| V9 | Where it runs | **Not in CI and not in a Cloudroom sandbox**: CUDA and TensorRT are required. The spike and any acceptance run use a rented GPU machine, with the owner's approval of the spend. The guard (S501) follows 0046 U9: domain other than 0, loopback, an advancing simulation clock | Wait for a CPU path: none is announced |
| V10 | When | **Nothing is built now.** Order: the G1 acceptance run (0046 phase 4), then the spike of section 5, then a decision to build. Two large items are already in flight or proposed (H4-1 and [0047](0047-low-energy-bench-pilot.md); strategy rule 3), and Isaac OS is one month old | Start now beside them: three large items, on an interface that will move (a `holoscan-robotics` package is announced for Q1 2027 and would replace `FetchContent`) |

```
AI ─▶ .rob ─validate─▶ Task IR ─▶ authorization (0038)
                          │
        ┌─────────────────┼──────────────────────┐
        ▼                 ▼                      ▼
   robolang-ros2     robolang-g1          robolang-isaac (this record)
   MoveIt, Gazebo    Unitree SDK, DDS     ROS 2 topics ─▶ ros2_bridge ─▶ Isaac OS graph
                     robot keeps balance                  (NVIDIA WBC writes rt/lowcmd)
        └─────────────────┴──────────────────────┘
          goal and preconditions decided from observations, in every backend
```

## 4. What RoboLang gains, and what it does not

- **Gains:** a second, independent humanoid stack under the same `.rob` and the same verdict rules, which is the claim
  of pillar P4 and of the north star. A concrete answer to "what does RoboLang add to GR00T": refusal, authorization
  and verified outcomes around policies that have none (V6).
- **Does not gain:** hardware independence (this backend is NVIDIA-only by construction, which is why it is one of
  three), a path to hardware (physical mode stays unauthorized), or anything for Core and robot-to-robot work.
- **Shared idea, different level:** Isaac OS checks a computation graph before it runs; RoboLang checks a task before
  it runs. The two do not overlap, and neither replaces the other.

## 5. The spike: go or no-go, two days at most

On one rented x86_64 machine with an NVIDIA GPU (an untested platform for Isaac OS, so the first check is real):

| # | Check | Passes when |
|---|---|---|
| K1 | Build `wbc_g1` at `v0.11.0` | It builds from the repository's Dockerfile |
| K2 | `./isaac_os run wbc_g1 mock -- --message-count=4` | It exits 0 |
| K3 | `unitree_sim` against Unitree MuJoCo (the procedure is undocumented) | The G1 stands for 60 s |
| K4 | A Python ROS 2 node publishes `Twist` through `ros2_bridge` | The robot walks 2 m and joint states come back |
| K5 | Arm targets from outside the graph, through the joint-command combiner | One arm reaches a pose while the robot stands |
| K6 | The stop of V8, and what happens when the controller process is killed | Zero velocity leaves it standing; the kill is measured and recorded for the new hazard |

**No-go** if K1–K4 do not pass in two days: the record is closed as "revisit at the Q1 2027 package". K5 failing
alone means walking only, and `grasp` and `place` are refused with S508.

## 6. Risks

- **A moving target.** One tag, one month old, 0.x, with a packaging change announced for Q1 2027. Pinning the tag
  contains it; it does not remove the upkeep.
- **New hazard for the safety case, H28:** *the balance controller is a user process; its
  exit, stall or missed deadline drops the robot.* With Unitree's own controller the robot keeps standing if our
  process dies. With Isaac OS it does not. Control: V8, and a check like H18's that the robot is confirmed standing.
- **No stop procedure from the vendor** (section 2, point 3). A pilot on this stack would need one written by whoever
  owns the cell (0038 P6); RoboLang's stop is not a safety function.
- **Cost and reach.** No CI, no sandbox runs: every check costs GPU time, and the nightly run that both existing
  backends have is not free here.
- **Vendor concentration.** If this backend got ahead of the other two, RoboLang would look like an NVIDIA add-on.
  V10's order keeps the vendor-neutral backends first.

## 7. Open points for the owner

1. **Role and seam (V1, V2).** Isaac OS as one optional backend behind ROS 2 topics, never a dependency of the
   language? I recommend **yes**.
2. **The spike (section 5).** Two days on a rented GPU machine, after the G1 acceptance run? I recommend **yes**. It is
   the cheapest way to learn whether sections 3–4 hold; today they rest on reading alone.
3. **G7 (the hardware pilot robot).** Should "G1 on Isaac OS" count as a candidate stack for G7? I recommend **not
   yet**: no documented stop procedure, and the validated computer is the Jetson AGX Thor only.
4. **Contact with NVIDIA.** An issue or discussion upstream (for example on a documented way to feed commands from
   outside a graph) would be public. I recommend **after the spike**, and only by the owner.

**Decided by the owner, 2026-11-25:** "V1–V10 and points 1–4 as recommended". So: Isaac OS is one optional backend
behind ROS 2 topics (1); the spike runs after the G1 acceptance run, two days at most on a rented GPU machine (2);
"G1 on Isaac OS" is not a G7 candidate for now (3); no contact with NVIDIA before the spike, and then only by the
owner (4).

## 8. Sources (read 2026-11-25)

- NVIDIA press release, 2026-10-21:
  <https://nvidianews.nvidia.com/news/nvidia-advances-isaac-platform-for-general-purpose-robotics-with-new-reference-humanoid-frameworks-simulation-pipelines-and-data>
- Documentation, v0.11.0: <https://docs.nvidia.com/isaac-os/v0-11-0/llms.txt> (overview; Isaac OS concepts; Isaac OS
  and ROS 2; application modes; integrate in your project; exchange messages with ROS 2; support matrix; components;
  Unitree G1 WBC; GR00T N1.7 + WBC; real-robot tutorial)
- Source, tag `v0.11.0` (commit `a4ab23f`): <https://github.com/nvidia-isaac/isaac-os/tree/v0.11.0>:
  `operators/g1_low_level/g1_low_level_joint_command_publisher.cc` (`ReleaseMode`, `rt/lowcmd`),
  `operators/agile_wbc/agile_wbc_op.h` (inputs: joint state, IMU, velocity), `operators/gr00t_n1_7/gr00t_n1_7_fused_op.h`
  (inputs: joint state, image), `common/ros2_bridge/README.md` (built-in converters), `applications/wbc_g1/`
- The owner's fork, created 2026-11-25: <https://github.com/rokorobot/isaac-os>. Its `main` is commit `a4ab23f`, identical
  to upstream `v0.11.0`, so everything above holds for it unchanged
