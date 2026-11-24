# Decision record 0047: a low-energy bench pilot, the first physical runs (H4, before G7)

**Date:** 2026-11-24
**Status:** **PROPOSED**, for the owner; reviewed 2026-11-24 (section 10). Design only: no code, and nothing here enables physical execution. Physical mode
stays disabled (`A731`) until the owner approves this record, signs the bench safety case (B9) and signs a cell
configuration for one named robot (B3).
**Applies to (when decided):** [safety case draft](../../docs/safety-case-draft.md) gaps G2, G3, G5 and G7 for one class
of robot only; [authorization 0.1](../authorization-v0.1.md) (a new field, B6); the ROS 2 backend
([ros2-backend-v0.1.md](../ros2-backend-v0.1.md)) or a small new backend package (B7); strategy H4.
**Builds on:** [0038](0038-physical-mode-authorization-stop-approval.md) P1–P9, unchanged: a signed cell configuration,
a signed task approval of the IR hash, a run grant by a person for every run, RoboLang's own stop and the safe state.
**Not decided here:** the external partner pilot (G7 as written), industrial arms, cobots, humanoids, real keys in Core
beyond 0038 (G4), standing authorizations.

## 1. The problem

Every claim RoboLang makes about hardware is still argued from simulation. The symbolic simulator and two physics
simulators (Gazebo, MuJoCo) show that a validated task reaches a robot unchanged and that success is judged from
observations. They cannot show that real sensors, real servo errors and real timing leave those claims standing.

The planned route to hardware is a pilot with an external partner on an industrial or collaborative robot (G7). That
route is slow, and its hazards (speed, force, payload, a person in the cell) are severe. Most of the evidence RoboLang
is missing does not need that energy. A robot too weak to injure a person, on a bench, with the owner present, can
answer the same questions about the task chain at a small fraction of the risk.

The idea in one line: **limit the hazard by the hardware's own energy, not by RoboLang.** RoboLang stays outside the
safety chain (safety case C1). The bench limits (section 3) hold even if every RoboLang component, model and network
misbehaves.

## 2. Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| B1 | Is a low-energy bench pilot allowed before the partner pilot (G7)? | **Yes, as a separate, earlier step.** It does not meet the H4 exit criterion and does not replace G7: it produces evidence for the task chain, not for industrial safety | (b) Keep G7 as the first hardware: no physical evidence for months. (c) Treat the bench pilot as meeting H4: wrong, an industrial cell has hazards a bench robot cannot show |
| B2 | Which robots qualify | Only robots inside **every** limit of section 3, **measured** on the actual unit (B8). Datasheet values are not enough | (b) Accept datasheet values: they are often optimistic and differ by firmware. (c) Name a robot model instead of limits: a model can be configured above the limits |
| B3 | How many robots and cells | **One** robot, **one** bench, named in the cell configuration. Adding another unit or another robot model is a new cell configuration with its own measurements | (b) A class approval for any qualifying robot: easier, but nobody has measured the second unit |
| B4 | Who may be present | The operator, who also grants every run (0038 P2, P5), within reach of the power cut (E7). Nobody else inside the robot's reach during a run. No children and no pets in the room | (b) Allow visitors behind a line: later, after the first sessions |
| B5 | Runs without a person | **Never** in the bench pilot. Every run is granted by the operator at the bench (0038 P2, P3), and the operator watches it | (b) Standing authorizations for repeated missions: right later (0038 P2 (b)), not in a first pilot |
| B6 | How physical mode is switched on | Only for a cell configuration that also declares `energy_class: "bench-low"` and the hash of the measurement sheet (B8). Authorization 0.2 adds both fields; a cell configuration without them keeps ending in `A731`. Physical mode lives only in the robot backend package; the core `robolang` wheel never gains it (0038 P1) | (b) A separate "bench mode" flag: a second switch nobody signs |
| B7 | Which backend | **Extend the ROS 2 backend** if the chosen robot has ROS 2 and MoveIt support (section 5 recommends one that does). The same `.rob` file then runs in Gazebo and on the bench, which is the comparison that matters. Its simulation guards (advancing `/clock`, the Gazebo model and plugins, `S501`) stay for every cell without a `bench-low` cell configuration; the site file gains the bench fields only behind that configuration. Gazebo's attach and detach have no bench equivalent: holding is observed, never assumed. Otherwise a small new backend package, as 0046 did for the G1 | (b) A new backend for every robot: more code, and the Gazebo comparison is lost |
| B8 | What proves a unit is inside the limits | A **measurement sheet**, part of the commissioning record (G5): supply voltage and current limit, moving mass, top speed at the tool, gripper force, reach, the power-cut test, the servo torque and speed limits as configured, and photos. Re-measured after any firmware or configuration change | (b) The integrator's declaration: there is no integrator here; the owner is both |
| B9 | The safety case for the bench | A short **bench safety case**: this record, the hazard table of section 4 and the measurement sheet, signed by the owner. Its version and hash go into the cell configuration (0038 P1) | (b) The full robot-independent safety case first: it is written for industrial cells and would hold the bench back for nothing |
| B10 | What RoboLang checks | Before motion, the backend refuses a task whose `@safety max_speed` is above the measured top speed, or whose regions reach outside the bench (G2, for this cell only); limits it cannot pass to the planner stay refused (`S508`). During a run, a servo's own error (overload, overheating, voltage) fails the step and stops as in 0038 P7. These checks are a convenience: the limits are enforced by the hardware and the servo configuration, never by RoboLang (C1) | (b) Rely on the servo limits alone: correct for safety, but a task that asks for more than the cell allows should fail before it moves, with a clear reason |
| B11 | How goals are observed | As in the Gazebo backend: one or more fixed USB cameras and printed ArUco markers on every object, calibrated at commissioning (G5). Goals and preconditions are judged from these observations and the servos' own readings, never from what the plan sent. Without a calibrated camera the backend refuses the cell | (b) Trust the servos' positions alone: says where the arm is, not where the object is; `object_at` and `holding` need the camera |

## 3. The low-energy limits (owner confirms the values)

A robot qualifies only if it is inside every limit, measured on the unit (B8). The values are proposals, chosen
conservatively. **They have not been checked against the standards text.** Before signing, the owner checks them against
ISO/TS 15066 (body-region limits for contact with people) and ISO 10218-1 (reduced speed, 250 mm/s), and lowers any
value the standards put lower.

| # | Limit | Proposed value | Why |
|---|---|---|---|
| E1 | Actuator power | **≤ 12 V DC**, from a current-limited bench supply set to the lowest current the robot runs at, or a battery pack with its own protection circuit. No mains-powered motors | Low voltage and a current limit bound the force every motor can produce at once |
| E2 | Moving mass | Arm: **≤ 1.5 kg** moving, payload **≤ 0.5 kg**. Mobile robot: **≤ 2 kg** in total | Impact energy grows with mass; a falling arm or a dropped object must not injure |
| E3 | Speed | **≤ 0.25 m/s** at the tool or the robot's body, as configured in the servos or the controller, and measured | ISO 10218-1 uses 250 mm/s as its reduced speed; a person can pull a hand away at this speed |
| E4 | Gripper and pinch force | **≤ 20 N** at the fingertips, measured with a force gauge, and the servo torque limit set as low as the payload allows | Fingers are the most exposed body part at a bench; to be checked against the hand and finger limits of ISO/TS 15066 |
| E5 | Reach and workspace | Arm reach **≤ 0.5 m**. The robot cannot reach past the marked bench area or off the bench edge; a mobile robot runs inside a walled area or on the floor | Bounds where a fault can take it |
| E6 | Kind of robot | Desktop arms and small wheeled robots only. **No humanoids, legged robots, drones or tools** (blades, heat, lasers, needles) | Falls, flight and tools add hazards the limits above do not cover |
| E7 | Power cut | A hardware switch that cuts actuator power, independent of all software, within the operator's reach, and tested at the start of every session | The stop that works when everything else has failed (0038 P6); servos go limp, which E2 makes acceptable |
| E8 | Wireless | Wireless off; connected by USB or wired Ethernet on an isolated network. Firmware version recorded | Safety case H26: cheap robots ship with weak wireless setups too |
| E9 | People | The operator present for every run (B4, B5), wearing safety glasses when within reach | Eyes are at bench height |
| E10 | Batteries | Mains bench supply preferred. If a battery is used: charged in a fire-safe bag, never unattended, removed when not in use | A LiPo fire is the most severe hazard left at this energy |

## 4. Hazards that remain at bench scale

Rows refer to the [safety case draft](../../docs/safety-case-draft.md).

| Hazard | At bench scale | Control |
|---|---|---|
| H05 Excess speed or force | Bounded by E1, E3, E4 even if the task or the software is wrong | Servo and controller configuration (B8); B10 refuses tasks that ask for more |
| H06 Collision with a person | A touch, not an injury, within E2–E4 | E5, B4, E9 |
| Pinching | Fingers in the gripper or a joint | E4; hands out of reach during a run |
| Runaway motion | A bug keeps it moving | 0038 P7 stop, grant expiry, E7 power cut |
| H18 Stop recorded but not done | On hardware, confirm from the servos' own position readings | Confirmed stop as in the Gazebo backend, plus the servo readings |
| Fire | Battery or a stalled servo overheating | E1 current limit, E10; never leave it powered and unattended |
| H08 Dropped load | A power cut (E7) or a fault drops the object | E2 payload limit; foam blocks in the first sessions (section 7) |
| Falls | The robot or the object falls off the bench | E5 |
| H26 Compromised wireless | Someone else commands the robot | E8 |
| H01, H02 Wrong or partial task | The task does something other than intended | 0038 P4 task approval of the IR hash; the limits make a wrong task harmless, not correct |

## 5. Candidate robots (to confirm at selection)

Values below are from public product information and must be measured on the unit (B8) before signing. A robot that
cannot be configured inside section 3 is not used.

| Robot | Kind | ROS 2 | Fit |
|---|---|---|---|
| **ROBOTIS OpenManipulator-X** | 4-axis desktop arm with gripper, Dynamixel servos on 12 V, reach about 0.38 m, payload about 0.5 kg | Official packages, MoveIt and a Gazebo model | **Recommended first.** Closest to the current ROS 2 backend, so B7 is smallest and the same task runs in Gazebo and on the bench. Dynamixel X-series servos keep velocity, current and PWM limits in their own memory and report overload and overheating (B8, B10) |
| SO-101 (open-source arm, LeRobot) | 6-axis desktop arm on serial bus servos, low cost | Community packages only | Good second: cheap and widely used for learning; needs more backend work |
| Elephant Robotics myCobot 280 | 6-axis desktop arm, payload about 0.25 kg | Vendor packages | Use a wired variant only (E8); closed firmware |
| ROBOTIS TurtleBot3 Burger | Small wheeled robot, about 1 kg, top speed about 0.22 m/s | Official packages, Nav2, Gazebo model | For a later mobile session; battery rules E10 apply |

Excluded: the Unitree G1 and every humanoid (E6; 0046 keeps it a simulation target), industrial arms and cobots (they
belong to G7), anything with mains-powered motors (E1).

## 6. What it proves, and what it does not

- **Proves**, on real hardware: a validated task reaches a robot unchanged; 0038's chain (approval of the IR hash,
  grant, stop, safe state) works end to end; goals are judged from real observations, with real sensor noise and
  servo error; the same `.rob` file gives the same verdict in Gazebo and on the bench, or the record shows why not.
- **Does not prove** that anything is safe in an industrial cell, with a cobot or with a humanoid. H4's exit
  criterion (a partner pilot behind a written safety case) is unchanged.

## 7. How it is judged

The bar of the Gazebo acceptance (0029), at bench scale. Proposed; the owner confirms it with B1.

1. **Tasks:** a single `move`; `grasp` and `place` of a foam block; and the same pick and place with the block
   missing, so the run must fail. The `.rob` files are the ones that pass in Gazebo, **unchanged**.
2. **Runs:** each task 10 times, each run approved and granted on its own (0038 P2, P4).
3. **Oracle:** after every run the operator records, with a photo, where the block and the arm are, before seeing
   RoboLang's verdict. RoboLang's verdict must agree with it.
4. **Pass:** no false success (a `SUCCEEDED` the operator's check contradicts), every expected failure fails with its
   code, every stop is confirmed from the servos' readings, and the same verdict as in Gazebo or a recorded reason why not.
5. **Stop rule:** any measured breach of a section 3 limit, any injury however small, or any motion after a stop ends the
   pilot. Nothing resumes without a new decision by the owner.

## 8. Delivery, once approved (each its own PR)

1. Bench safety case (B9) and an empty measurement sheet template. No code.
2. Authorization 0.2: `energy_class` and the measurement sheet's hash in the cell configuration (B6). Simulation
   only: `A731` stays for every cell until step 4.
3. Backend support for the chosen robot (B7), first against its Gazebo model, with B10's checks.
4. Commissioning: measure the unit (B8), sign the bench safety case and the cell configuration. Only now does the
   backend accept `mode: physical`, and only for that cell.
5. The acceptance of section 7, starting with a single `move`. Evidence in `docs/evidence/`.

## 9. Needs the owner

B1 (a bench pilot before G7, not instead of it), the values of E1–E10 after checking them against the standards, B3 (one robot, one bench), B4 and B5 (who may
be present; no unattended runs), B6 (a new authorization field), B11 (cameras and markers), the acceptance bar
(section 7), the robot (section 5; OpenManipulator-X recommended) and its purchase.

## 10. Review (2026-11-24)

Reviewed before merging as a proposal. Corrections made in the record:

- **Observation was missing.** The draft said goals are judged from real observations but named no sensor. B11 adds
  fixed cameras and ArUco markers, as in the Gazebo backend, calibrated at commissioning.
- **An unverified claim.** The draft called 20 N "well below" the ISO/TS 15066 hand limits without checking the text.
  Section 3 now says the values are unchecked, and the owner checks them before signing.
- **No acceptance bar.** Section 7 adds one, with an operator's photo as the oracle, no false success, and a stop rule.
- **Backend details.** B7 keeps the Gazebo backend's simulation guards for every other cell and drops the Gazebo
  attach/detach shortcut on the bench. B10 adds servo errors as a stop condition and keeps `S508` for limits the
  backend cannot pass on.

Checked and unchanged: the record builds on 0038 without changing it; the core wheel still never gains a physical
mode; `A731` stays for every cell until delivery step 4; H4's exit criterion and G7 are unchanged.
