# Hazard log (robot-independent) — DRAFT

**Status:** DRAFT, 2026-11-11, for the owner's review. STRATEGY H4, decision D2. Physical execution is **not
authorized**; nothing here enables it. Companion documents: [safety case draft](safety-case-draft.md),
[physical-mode authorization design](physical-mode-authorization.md).

**Scope.** Hazards that RoboLang itself can introduce or fail to stop when it commands a robot: the language, AI
authoring, the validator and simulator, Core messages between participants, and an execution backend. The robot's own
hazards (pinch points, payload, stored energy) belong to the robot's risk assessment, which a robot-specific annex will
add. **Premise (RFC-001 13.6):** RoboLang constraints are not safety functions. Safety-rated stops, speed and
separation monitoring and the robot's risk assessment are outside RoboLang and must work with every RoboLang
component, AI model and network unavailable.

Status values: **controlled** (control exists and is tested in simulation), **partial** (control exists, physical
evidence missing), **open** (no control yet).

| # | Hazard | Cause | Existing controls (evidence) | Gap before physical mode | Status |
|---|---|---|---|---|---|
| HZ1 | A program looks right but does less than asked | AI author drops part of a request (one-branch "half-program", `inspect` used instead of a reading) | Validator; required `goal` checked after the run; benchmark measures `half_program` and `silent_substitution` (1.4 and 1.6 acceptance: 0 with the full reference) | Requests outside the benchmark's coverage; grammar-only prompting is weaker | partial |
| HZ2 | A run counts a different end state as success | A fallback "otherwise leave it there and call it done" | A handler has no goal (0034); the benchmark's inexpressible controls (`f11`) | Grammar-only answers still write it (25–50% in the 1.5 baseline) | partial |
| HZ3 | A run carries on after a wait timed out | A handler that reports and goes on | Every `wait until` needs a handler (`E316`); `unsafe_continue` measure (0 with the full reference) | Physical waits depend on real sensors | partial |
| HZ4 | Stale, uncertain or missing data treated as true | Old observation, sensor offline, borderline reading | Three-valued conditions, unknown fails closed (`S261`); uncertainty intervals (0036 V7); `@verify max_age`; Gazebo verdicts from observations, never ground truth | Real sensor calibration and failure modes; time sync between robots | partial |
| HZ5 | Motion without authority | A requester authorizing itself; a grant outside its scope; a forged grant | Authorization is local (RFC-001 13.2); `C401`; standing authorizations are bounded, leased, not transferable; physical mode rejected (`C404`) | No physical-mode authorization path exists yet (by design) | controlled in simulation |
| HZ6 | A stop, timeout or cancel is taken to mean the robot stopped | Software state diverges from controller state | Safety semantics 4 (development plan section 5): request a controller stop and confirm it | Not implemented against a real controller | open |
| HZ7 | A RoboLang speed limit is relied on as a safety limit | Misreading `@safety max_speed` | RFC-001 13.6; the Gazebo backend rejects declared speed limits rather than pretend to enforce them | Physical mode must use the robot's safety-rated speed monitoring | partial |
| HZ8 | Collision with a person or object | RoboLang has no collision prevention | MoveIt planning scene in simulation only; `region_clear` observed, unknown fails closed | Entirely the robot's safety functions and the site risk assessment | open (outside RoboLang) |
| HZ9 | Unexpected arm state at the start of motion is accepted | MoveIt start tolerance raised to 0.05 rad for Gazebo settling (#99) | Simulation only | Re-assess per robot; the default 0.01 rad or lower for physical runs | open |
| HZ10 | A grasp is reported held when it is not, and the object drops | Gazebo detachable joint instead of contact physics; grasp checked by a marker under the gripper | `holding` decided from observations; the grasp check fails before closing (`S506`) | Real gripper force and slip sensing; drop zones | partial |
| HZ11 | The wrong object, place or robot is bound | Decoy names; perception confusion | `validate --world`; decoy tasks in the benchmark; distinct marker IDs per object | Real perception misidentification | partial |
| HZ12 | Two robots disagree about who holds an object | R2R handover faults, lost messages | Core custody, commit point and leases; gate 2 passed with an independent implementation (25/25) | Physical handover timing and a both-holding state | partial |
| HZ13 | A forged or replayed message moves a robot | Spoofing on the network | Signatures, freshness, per-recipient sequencing (Core 0.1 8) | Simulation signatures use a published HMAC key: **not secure**. Physical mode needs Ed25519 with real keys (the independent implementation has none) | open |
| HZ14 | The AI model or network fails mid-task | Outage, provider change | Leases expire to a local safe state (RFC-001 13.3); weekly benchmark per model | "Local safe state" must be defined per robot | partial |
| HZ15 | Unvalidated code drives a robot | The v1 prototype in `legacy/` can drive ROS 2 without the validator | Not installed (packaging test); README warning | Remove or quarantine `legacy/` before any physical pilot | open |
| HZ16 | An ambiguous request is guessed | Several matching entities, missing values | `clarify` decisions in the benchmark (set "ask") | Operator workflow for answering questions | partial |

## Review

Each row is reviewed when the robot-specific annex is written, and again before the first physical run. A row moves to
**controlled** only with physical evidence named in the safety case.
