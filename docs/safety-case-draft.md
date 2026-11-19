# Safety case draft and hazard list (H4)

**Status:** draft 0.1, 2026-11-12. H18 and H19 come from the earlier drafts of PR #122 (HZ6 and HZ15), closed in
favour of this one. Robot-independent (STRATEGY D2). Not a safety case yet: nothing here authorizes
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
| H15 | The operator cannot stop the robot | Software holds the motion | Gazebo backend: cancel, deadline and revoke stop a run even during a motion and hold the arm (0038 step 3) | E-stop is hardware and must never pass through RoboLang; RoboLang's cancel must map to a protective stop (G3) |

### Mode and tooling

| # | Hazard | Cause | Today | Gap |
|---|---|---|---|---|
| H16 | A simulation result is taken as proof of safety, or a task meant for simulation reaches hardware | Mode confusion | "Simulation only" in every output; no physical-mode option or key is accepted; the ROS 2 backend refuses to start without simulated time and the Gazebo model (`S501`) | A physical mode that can only be entered by an explicit, recorded authorization (G1) |
| H17 | A bug in the validator or executor | Software defect | ~3,400 tests on Linux and Windows; deterministic, byte-identical run records | Not qualified as safety software. Acceptable only while section 1 holds: RoboLang never carries a safety function |
| H18 | A stop is recorded as done while the robot is still moving | The hold is sent but not confirmed | **Gazebo backend, 2026-11-13:** a stop is confirmed only when fresh joint states show no arm joint moving more than 0.001 rad over 500 ms; the record says whether it was. Both Gazebo stop cases confirmed, and the oracle agreed (0.0 m drift) | On hardware, confirm from the controller's own state too; the Gazebo stop cases both came while the arm was already still, so stopping a fast motion is tested only in unit tests (G3) |
| H19 | Unvalidated code drives a robot | The v1 prototype in `legacy/` could drive ROS 2 without the validator | **Closed 2026-11-13 (G8):** its ROS 2 adapter and entry point are removed; what remains only prints. A test fails if any `legacy/` file imports ROS 2 | None |
| H20 | A declared action does more than its name says | RoboLang does not know what `open`, `print` or `dock` does physically: an action is a **request to the robot's own controller** (decision 0040 A9, RoboLang 1.8), and a profile entry named `open` could also move the arm or release a brake | **Controls (2026-11-17, built in L4 of I2):** the integrator's profile entry in the world file (`actions`) is the only description of an action, and the integrator is accountable for it. The signed task approval (G6) names the task, the robot and the world that declares its actions, so an approver sees every `do`, its target and its value kind. Before the run the robot must declare the action, the target and the value must be of the declared kinds (`S006` to `S008`), and a goal state no declared action produces fails at once (`S009`). A run never claims an action took effect: the record says `requested`, and success needs the goal's `in_state`, observed (unknown fails closed). The region and speed limits of 1.7 cover every motion, actions included (`S004` for a location target). The ROS 2 backend refuses `do` and `in_state` (`S508`); `export-goal` and `delegate` refuse a goal with `in_state` (`X606`) | RoboLang cannot check that a declared action does what its name says, or nothing more: that is the integrator's declaration, reviewed at approval (G6). A misleading profile entry cannot be detected. Nothing here runs a physical action |
| H21 | A wrong clock makes a time condition decide wrongly | A task that waits for or compares the time of day (RoboLang 1.9, decision 0042) acts early, late or on the wrong day because the clock it reads is wrong, unsynchronised or on another day than the task assumed | **Controls (2026-11-17, built in L4 of I3):** a `time` reading fails the step when the world has no clock (`S237`, fail closed: never the host's time), and when the clock has reached the end of the day the run started on (`S238`: one day only, T4); a wait for a time that has already passed fails before the run (`S010`, K3: "at 09:00" never silently means "now"); readings are exact in the simulation and a wait ends at the exact instant (K5). A wait for a time keeps the robot idle and holding its state, the timeout bounds it, and a pending stop (0038) ends it (T10). The ROS 2 backend refuses `measure r time` and `measure r elapsed` (`S510`). On hardware the clock source must be trustworthy (NTP or the controller's clock), which is commissioning (G5) | RoboLang cannot check a real clock: the simulation's clock is exact by construction and says nothing about a robot's. A daylight-saving change during a run is not modelled. A wrong clock on site is not detected by RoboLang; it is commissioned (G5) and the approver sees every time literal in the signed task (G6) |
| H22 | A mission runs longer or more often than approved | A mission (RoboLang 2.0, decision 0041) repeats task runs, so a missing or too wide bound, a failed run that is skipped or retried without limit, or a condition that is wrongly read as met makes the robot work past what the approver signed for | **Controls (2026-11-18, built in L4 of I4), M9:** every repeat has a hard bound (M2): a run count (at most 1000), a duration, or a time of day to end at; a mission with none is `E325` (`I226` in the IR reader), and there is no way to write "forever". The **bound and the stop are part of the signed approval** (0038): an approval names the mission, its bound and its failure policy, so a mission cannot run past what was approved. Each task run keeps its own `pre` checks, limits (1.7) and verified goal (M9), and a failed run is never counted as done: the mission record lists every run with its outcome and ends `completed`, `bounded`, `failed` or `stopped`, never claiming more than its runs verified (M5). `retry N` is at most 100 and its retries count against the bound (clarification 7); `continue` that fails every run ends `failed`, not `bounded` (clarification 6); the `until` is read live and an unknown value stops the mission (`S402`, fail closed); the duration is checked before each run, a started run is never cut, and a task that cannot fit even once fails before any run (`S401`). A mission runs on the planner's side and a robot is never sent one: `export-goal` and `delegate` refuse it (`X607`) and the ROS 2 backend too (`S514`), so the robot only ever sees one bounded, verified task at a time. An operator stop and a lease end (0038) end a mission at any time | The approval must name the bound: a signature on the tasks alone does not limit a mission (G6, the approval format must carry the mission and its bound; the simulator does not yet issue or check an approval of a mission). The bound limits runs and simulated time, not what one run does: a task approved for one run is approved for N. The simulation's duration is the time its waits take; real task durations are not modelled, so a duration bound is checked on hardware by the controller's clock (commissioning, G5). A trigger (every hour, on a message) is not in 2.0 (M6), so no mission starts by itself |
| H23 | A spoofed message changes what a robot does | A task can wait for, or branch on, an incoming message (RoboLang 2.1, decision 0043), so a forged or mistaken "WIND CLEAR" or "DOOR OPEN" from the wrong sender could release a load, open a gate or start a motion that the real source never allowed | **Controls (2026-11-18, built in L4 of I5), S4, S9:** the sender of a message is never chosen by the task: each channel and the senders it accepts are declared in the world (`channels`, world file 1.6), which is the integrator's configuration, as a robot's abilities are; a message from anyone else is ignored and the run record lists it as `ignored`, so a spoof attempt is visible. A world that declares no channel has no message service, so every message condition is unknown and fails closed; a task that names an undeclared channel fails before the run (`S011`). A message never enters a goal (`E327`), so `export-goal` and Core never carry one. A robot is never delegated a task that waits for a message (`X608`) and the ROS 2 backend refuses it (`S515`): neither has the channel's trust configuration. A task's own `communicate` is never a received message. **Simulation only in 2.1:** nothing in this package authenticates a sender; a simulated message is a world event, and in physical mode only authenticated senders may be accepted (real identity in Core, G4), which is not built | Authentication of the sender (G4): until it exists, 2.1 must not be run against physical hardware on the strength of a message |
| H24 | A mistaken reading, unit or count changes what a robot does | A task can compare a fill, a pressure, a count or a detector's confidence (RoboLang 2.2, decision 0044), so a misread gauge, a unit off by a factor (`kPa` for `bar`), a count taken for a fill, or a detection acted on without its confidence could open a relief valve, restock the wrong shelf or skip a hazard | **Controls (2026-11-18, built in L4 of I6), K2, K3, K4:** every kind has its own units and a comparison between kinds is a static error (`E319`); a pressure converts between `Pa`, `kPa` and `bar` in exact decimal arithmetic, never in floats; a unit is always required (`E206`), so a bare fraction or `percent` cannot be misread; a reading carries the sensor's uncertainty and a comparison it cannot decide is unknown, which fails closed (`S261`); a count is exact and takes a whole number; `confidence` is the sensor's confidence in the label it reports, read with the same sensor as the label; a reading never enters a goal (`E321`), so `export-goal` and Core never carry one; `delegate` and the ROS 2 backend refuse every `measure` (`S509`). **Simulation only:** a simulated reading is the world file's value; nothing here calibrates a real gauge or camera | Calibration and the sensor's real uncertainty are the integrator's (G4): until a real sensor's error is declared in the world, a threshold in a 2.2 task says nothing about a physical machine |
| H25 | A stale or wrong world model makes a choice wrong | A task can choose at run time (RoboLang 2.3, decision 0045), so a robot, an object or a place is picked from the world model as it is when the choice is made: a stale position, a robot marked ready that is not, or a part that is no longer on the conveyor sends a different robot, or picks a different object, from what the reviewer expected | **Controls (2026-11-19, built in L4 of I7), C11:** the candidates are finite and appear in the task (a list, `objects at L` or `robots`), so an approval (0038) names them and the reviewer sees every entity that can be chosen; the order is a plain rule (straight-line distance between positions, a tie goes to the first written), the same in every run, so a choice can be reproduced; the choice is recorded with its reasons: the candidates, each one excluded and why, the order and the entity chosen (run record 1.11 `choices`); an unknown fact excludes a candidate (`where_unknown`, `no_position`: it fails closed) and a choice with no candidate left runs its `else`, which must end in `fail` (`E329`), so a task never goes on with a name unbound; `where` and the order use world facts only (`E330`), never a reading; a goal is checked for the entity chosen, never for some entity (C7); a step that uses a chosen robot or place is checked when it runs (`S211`, `S231`, region limits `S003` and `S004`); a place whose position is unknown fails before the run (`S273`). The robot never chooses: Core 0.2 carries only ground goals, so `export-goal` and `delegate` accept a choice only before the first action, the planner makes it from its world model and sends the goal with it filled in, and anything else is refused (`X609`); the ROS 2 backend refuses `choose` (`S508`). **Simulation only:** positions and states are the world file's | In physical mode the positions and states must come from the fleet manager and be current (G4, commissioning): until they do, a choice in a 2.3 task says nothing about the real site. A choice is as wrong as the world model it is made from; RoboLang does not check it against reality |

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
   safe state; state that e-stop never passes through RoboLang. Unblocks C3, H09, H14, H15, H18. Built for the
   Gazebo backend (0038 step 3); confirming a stop from the controller (H18) is still open.
4. **G4 Real identity in Core.** Replace the simulation-only keys: key provisioning, rotation, revocation. Needed
   once a planner and a robot are separate machines. Unblocks H11, H12.
5. **G5 Commissioning checks.** A procedure, run per cell, that checks the site file, camera calibration and start
   tolerance against reality (includes #99). Unblocks H03, H07, H10.
6. **G6 Human approval record.** A reviewer signs the task's IR hash; physical mode accepts only approved IR.
   Unblocks H01, H02; part of C4.
7. **G7 External partner and cell risk assessment.** Owner action: choose the pilot partner and robot, whose risk
   assessment C1 rests on.
8. **G8 Quarantine `legacy/`.** **Done 2026-11-13:** the ROS 2 adapter and entry point are removed; the print-only
   v1 interpreter stays as the compatibility fixture of development plan A7. Closes H19.

**Recommendation:** design G1, G3 and G6 together first (proposed in decision
[0038](../spec/decisions/0038-physical-mode-authorization-stop-approval.md)). They are robot-independent, small, and are what an
external partner will ask about first. G2 and G5 need a named robot (G7).
