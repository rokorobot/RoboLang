# Safety case draft and hazard list (H4)

**Status:** draft 0.1, 2026-11-12. Robot-independent (STRATEGY D2). Not a safety case yet: nothing here authorizes
physical execution. Physical mode stays unauthorized until the owner signs a written safety case for a named robot
and cell.

## 1. The position this draft takes

RoboLang is **not a safety function** and must never become one. Speed, force, workspace and stop functions on a real
robot belong to the robot controller's certified safety configuration and the cell's safeguarding (the integrator's
risk assessment under ISO 12100 and ISO 10218-2). RoboLang's job is narrower:

1. it can only ask for what the integrator's envelope allows, and it checks that before anything moves;
2. it fails closed: an unknown fact, a stale reading or a failed step stops the task, never continues it;
3. a person approves what runs, and success is claimed only from observations.

This keeps RoboLang out of tool qualification (it is not in the safety chain), and makes every hazard below either
"covered by RoboLang", "covered by the cell", or a gap.

## 2. Hazard list

Columns: what can go wrong, how, what exists today, and what is missing before hardware. "Today" names code or
evidence that exists on `main`; "gap" items are collected and ranked in section 4.

### Task content

| # | Hazard | Cause | Today | Gap |
|---|---|---|---|---|
| H01 | The task does something other than the operator asked | An LLM misreads the request | Static validation with every diagnostic; benchmark decision accuracy 97.5–100% (H1); goal verified at the end | No recorded human approval of a task before it runs (G6) |
| H02 | Part of the request is silently dropped or substituted | Model writes `half_program`, `silent_substitution` | Benchmark measures both; inexpressible requests must be refused (H2 bound: false programs ≤ 5%) | Rates are low, not zero: human review (G6) is the control |
| H03 | The right task acts on the wrong object or place | Wrong binding, or the site file maps an ID to the wrong physical thing | Typed parameters, world-kind checks (`E5xx`), `--bind` checks; ROS 2 site file strictly read (`S507`) | Nothing checks the site file against the real cell (G5) |
| H04 | A retry or failure handler repeats a hazardous action | Handler logic | No loops; retries only by bounded nesting (1.5); every `wait until` needs a handler (`E316`) | None for RoboLang; the cell must tolerate one repeated step |

### Motion and handling

| # | Hazard | Cause | Today | Gap |
|---|---|---|---|---|
| H05 | Excess speed or force | Wrong limit, or none | `@safety max_speed` is **recorded, not enforced**; the ROS 2 backend refuses tasks that declare limits | The controller must enforce; RoboLang must check the task against the controller's configured limits (G2) |
| H06 | Collision with a person, the cell or an object | Motion through occupied space | MoveIt plans around static boxes from the site file; no human detection | Safeguarding is the cell's (fences, scanners, ISO/TS 15066 if collaborative); RoboLang needs a declared workspace it cannot leave (G2) |
| H07 | Jump at the start of a motion | Start state differs from the planned one; MoveIt `allowed_start_tolerance` raised to 0.05 rad (#99) | Arm-settle wait in the Gazebo harness | 0.05 rad on a ~0.85 m reach is about 4 cm. Hardware needs a justified tolerance or a re-plan from the measured state (G5) |
| H08 | Dropped load | Grasp fails, payload too heavy, placed at the wrong pose | Marker under the gripper checked before closing (`S506`); `holding` verified from cameras; mass in world files 1.1 | No payload check against the robot's rating; grasping in Gazebo is a detachable joint, not validated contact physics (G2, G5) |
| H09 | Robot holds or moves a load longer than expected | Timeout, lost operator | `wait until` timeouts fail with `S241` | Safe state after a failure is not defined for hardware (G3) |

### Observation

| # | Hazard | Cause | Today | Gap |
|---|---|---|---|---|
| H10 | False success or false precondition | Stale or wrong reading, calibration drift, marker misread | `min_confidence`, `max_age`; unknown fails closed (`S102`, `S302`); 51/51 goal checks agreed with ground truth in Gazebo, no false success | Camera poses are calibrations, not observations; no recalibration check on hardware (G5) |

### Authority and communication

| # | Hazard | Cause | Today | Gap |
|---|---|---|---|---|
| H11 | Someone not entitled commands the robot | Spoofed or forged goal | Core signatures exist but use **published, simulation-only keys**: no identity | Real keys and an authorization design (G1, G4) |
| H12 | A goal runs twice or out of date | Replay, duplicate delivery | Per-recipient sequencing (0008), leases (0022) | No transport exists yet; covered once G4 is built |
| H13 | Handover between two robots drops the object | Custody conflict | Custody transfer with leases in `delegate` (0022) | Physical timing of a handover is not modelled; out of scope for a single-robot pilot |
| H14 | Communication lost mid-task | Network, crash | Leases expire in Core | What the robot does when a lease expires is not defined for hardware (G3) |
| H15 | The operator cannot stop the robot | Software holds the motion | Cancellation exists only as a simulation script | E-stop is hardware and must never pass through RoboLang; RoboLang's cancel must map to a protective stop (G3) |

### Mode and tooling

| # | Hazard | Cause | Today | Gap |
|---|---|---|---|---|
| H16 | A simulation result is taken as proof of safety, or a task meant for simulation reaches hardware | Mode confusion | "Simulation only" in every output; no physical-mode option or key is accepted; the ROS 2 backend refuses to start without simulated time and the Gazebo model (`S501`) | A physical mode that can only be entered by an explicit, recorded authorization (G1) |
| H17 | A bug in the validator or executor | Software defect | ~3,400 tests on Linux and Windows; deterministic, byte-identical run records | Not qualified as safety software. Acceptable only while section 1 holds: RoboLang never carries a safety function |

## 3. Safety case outline

Top claim, for one named robot and cell: **running a validated RoboLang task adds no risk beyond the cell's own
risk assessment.**

| Claim | Argument | Evidence today | Missing |
|---|---|---|---|
| C1 RoboLang carries no safety function | Safety functions are in the controller and cell; RoboLang limits are requests | ROS 2 backend spec, "Scope and safety boundary"; limits recorded, not enforced | Cell risk assessment by the partner (G7) |
| C2 A task can only ask for what the envelope allows | Checked before the run against the integrator's declared envelope | `validate --world` checks against world profiles | Envelope from the controller's configuration (G2) |
| C3 A task fails closed | Unknown and stale facts stop the task | Simulation spec 4.4; `S102`, `S302`; Gazebo acceptance run | Defined safe state on hardware (G3) |
| C4 Only an authorized person can start a physical run, of a reviewed task | Physical mode requires a recorded authorization naming the task | None | G1, G4, G6 |
| C5 Success is claimed only from observations | Goal verified from sensors, not from completed steps | H3 acceptance: 51/51, no false success | Calibration procedure on hardware (G5) |

## 4. Gaps, ranked for the owner

Ranked by how much of the safety case each one unblocks. Each is a design item; none is built yet.

1. **G1 Physical-mode authorization.** Who may authorize a physical run, what the authorization names (task IR
   hash, robot, cell, expiry), where it is recorded, and why no config edit can enter physical mode without it.
   Unblocks C4, H16. Named in STRATEGY D2.
2. **G2 Envelope from the controller.** Read the integrator's limits (speed, workspace, payload) and refuse, before
   motion, any task that asks for more. `max_speed` moves from "recorded" to "checked against the controller".
   Unblocks C2, H05, H06, H08.
3. **G3 Stop and safe state.** Map RoboLang cancel, step failure and lease expiry to a protective stop and a defined
   safe state; state that e-stop never passes through RoboLang. Unblocks C3, H09, H14, H15.
4. **G4 Real identity in Core.** Replace the simulation-only keys: key provisioning, rotation, revocation. Needed
   once a planner and a robot are separate machines. Unblocks H11, H12.
5. **G5 Commissioning checks.** A procedure, run per cell, that checks the site file, camera calibration and start
   tolerance against reality (includes #99). Unblocks H03, H07, H10.
6. **G6 Human approval record.** A reviewer signs the task's IR hash; physical mode accepts only approved IR.
   Unblocks H01, H02; part of C4.
7. **G7 External partner and cell risk assessment.** Owner action: choose the pilot partner and robot, whose risk
   assessment C1 rests on.

**Recommendation:** design G1, G3 and G6 together first (proposed in decision
[0038](../spec/decisions/0038-physical-mode-authorization-stop-approval.md)). They are robot-independent, small, and are what an
external partner will ask about first. G2 and G5 need a named robot (G7).
