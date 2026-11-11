# Safety case for physical mode (robot-independent) — DRAFT

**Status:** DRAFT, 2026-11-11, for the owner's review. STRATEGY H4 (exit criterion: the owner authorizes physical mode on
a written safety case), decision D2; target for a reviewed draft: Q1 2027. **Physical execution is not authorized.**
This is not a certification and does not claim conformity with any standard. Hazards: [hazard log](hazard-log.md).
Authorization path: [physical-mode authorization design](physical-mode-authorization.md).

## 1. Top claim

> **G0.** Commanding a robot through RoboLang in physical mode does not reduce the protection given by the robot's own
> safety functions, and RoboLang refuses, stops or asks rather than act on unauthorized, uncertain or unverified
> information.

RoboLang never claims to *provide* safety. It claims to stay out of the safety functions' way and to fail closed.

## 2. Boundary and assumptions

| # | Assumption (must hold for any physical deployment) |
|---|---|
| A1 | The robot has safety-rated stops (emergency and protective), speed and separation or force limiting as its risk assessment requires, independent of RoboLang |
| A2 | A robot-specific risk assessment exists for the cell (for example along ISO 12100, and ISO 10218-1/-2 for industrial robots); its measures stay in force |
| A3 | The first pilots run in a fenced cell with no person inside during motion, at reduced speed |
| A4 | A named, trained operator can stop the robot at any time by means outside RoboLang |
| A5 | RoboLang runs only through the validated path: no `legacy/` code, no direct controller access by an AI model |

## 3. Argument

| Claim | Argument | Evidence today | Evidence still needed |
|---|---|---|---|
| **G1** RoboLang cannot disable or bypass a safety function | It has no command that reaches safety configuration; constraints are passed to the backend, not enforced as safety (RFC-001 13.6); backends reject what they cannot honour | RFC-001 13.6; Gazebo backend rejects declared speed limits and unsupported steps before motion (`S507`, `S508`, `S509`) | Per-robot check that the backend's controller interface has no write access to safety parameters |
| **G2** Nothing moves without a valid, local authorization | Authorization is decided on the robot; requesters never authorize themselves; standing authorizations are bounded, leased and mode-explicit | `C401`, `C404`; reference simulation cases `v10_requester_authorizes`, `v11_payload_beyond_standing_authorization` | The physical-mode enablement path ([design](physical-mode-authorization.md)) implemented and tested |
| **G3** Uncertain or missing information stops or holds the task | Three-valued logic; unknown fails closed; uncertainty intervals; observation freshness | Simulation tests (`S261`, V7), Gazebo verdicts from observations with an independent oracle (51/51 agreement, 0 false success) | Real-sensor failure injection on the pilot robot (HZ4) |
| **G4** A software stop means a confirmed controller stop | Every terminal failure requests a controller stop and confirms it (development plan safety semantics 4) | None | Implementation and test on the pilot controller (HZ6) |
| **G5** AI-written programs are checked before they run, and their failure modes are measured | Validator, `validate --world`, required goals; the benchmark tracks silent failures per model, weekly | 1.4 to 1.6 acceptance evidence (`half_program`, `unsafe_continue`, `silent_substitution` at 0 with the full reference) | A pilot task set run first in simulation with the same site and robot profile (HZ1–HZ3) |
| **G6** Messages between participants cannot be forged or replayed | Signatures, freshness, sequencing | Core conformance kits; gate 2 passed | Ed25519 with managed keys in physical mode; simulation HMAC refused there (HZ13) |
| **G7** Failures of the network or AI leave the robot in a safe state | Leases expire to a local safe state; safety functions are independent (A1) | Lease and escalation cases in the reference simulation | A defined local safe state per robot, tested (HZ14) |

## 4. What a robot-specific annex adds

The robot and its controller; the cell layout and fencing; the risk assessment reference; the local safe state; the
mapping of RoboLang steps to controller commands; sensor sources and their calibration; the stop-confirmation interface;
the hazard log rows re-assessed for that robot; the pilot task list.

## 5. Next steps (H4)

1. Owner review of this draft, the hazard log and the authorization design.
2. Choose the pilot robot and partner; write the annex.
3. Close the "open" hazard rows that are RoboLang's to close: HZ6, HZ9, HZ13, HZ15.
4. A reviewed draft by Q1 2027; the owner's physical-mode authorization only on the completed case (STRATEGY H4).
