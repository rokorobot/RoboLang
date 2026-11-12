# Decision record 0038: Physical-mode authorization, stop and safe state, task approval (H4 G1, G3, G6)

**Date:** 2026-11-12
**Status:** **APPROVED by the owner, 2026-11-12**, with the recommended option for P1–P9, including the `ed25519`
dependency confined to an optional extra or the robot backend package (never the core wheel). The maximum grant
lease is set in the cell configuration, which the owner signs (P1, P2). Design only: no code, and nothing here
enables physical execution. Each delivery step of section 4 is its own PR.
Physical mode stays unauthorized until the owner signs a written safety case for a named robot and cell.
**Applies to (when decided):** [safety case draft](../../docs/safety-case-draft.md) gaps G1, G3 and G6; claims C3
and C4. Builds on RFC-001 13.2–13.6 (authorization is local, standing authorizations, `mode`, safety), Core 0.1
`authorization_decision` and `C404`, and development plan A5.
**Not decided here:** limits read from the controller (G2), commissioning checks (G5) and real keys in Core (G4),
beyond what P3 needs. G2 and G5 need a named robot (G7).

## 1. The problem

RFC-001 already says who decides (the robot's local authority, never the requester) and that `mode` is explicit.
It leaves open three things a hardware partner will ask first: how physical mode is switched on at all, what
happens when something stops the robot, and how a person's review of an AI-written task is bound to what runs.

## 2. Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| P1 | Where physical mode can be enabled | Only by a **cell configuration** installed on the robot side: robot ID, cell ID, the signed safety case's version and hash, the commissioning record, an expiry, signed by the owner's key. Without a valid one, every component stays simulation only and `physical` keeps failing with `C404`. The core `robolang` wheel never gains a physical mode; it lives only in a robot backend package | (b) A CLI flag with a confirmation prompt: any script or AI agent can type it. (c) An environment variable: the same, and invisible in records |
| P2 | What each physical run needs | A Core `authorization_decision` with `mode: physical` from the robot's authority, naming the goal and the approved IR hash (P4), with a lease no longer than the cell configuration allows. No standing authorizations in physical mode during the pilot: every run is granted by a person | (b) Standing physical authorizations (RFC-001 13.3) from the start: right later, but in a first pilot every run should have a person behind it |
| P3 | Who may grant | A named person holding an `ed25519` key listed in the cell configuration, present at the cell. An AI agent is never an authority (RFC-001 13.2). `sim-hmac-sha256` is already rejected in physical mode (`C204`) | (b) A policy service: needs G4 and a policy language (RFC-001 open question 11); later |
| P4 | Task approval (G6) | A **task approval** signed by a reviewer: the Task IR's SHA-256, the task name, language version, robot and cell, reviewer ID and time. The reviewer signs after reading `robolang inspect` of that IR. Any change to the IR invalidates it. The author (person or AI) cannot approve its own task | (b) Approve the `.rob` source: the IR is what runs, and the reader re-validates it in full. (c) No separate approval, the grant is enough: then nobody has to read the task |
| P5 | The reviewer and the granting operator | May be the same person in the pilot; both roles are recorded separately | (b) Two people required: safer, but heavy for a one-cell pilot. The owner may require it |
| P6 | Safety stops | E-stop and protective stops are the cell's: hardware, independent, working with every RoboLang component, model and network off (RFC-001 13.6). RoboLang never resets or acknowledges a safety stop. After any safety stop the run ends `FAILED`, and continuing needs a new grant (P2) | (b) Let RoboLang resume after a protective stop clears: a resume is a new motion nobody granted |
| P7 | RoboLang's own stop | On a failed step, an unknown precondition, a stale reading, cancel, revocation, or expiry of the grant or a commitment lease, the executor sends no further motion, stops the active trajectory as a controlled stop (power kept, IEC 60204-1 category 2), and enters the safe state (P8). Expiry during a motion stops it; there is no automatic renewal | (b) Finish the current step first: a step can be long, and expiry exists to bound the run |
| P8 | Safe state (RFC-001 open question 10) | Declared per cell in the cell configuration. Default: hold position, **gripper unchanged** (never open a gripper that may hold something), motion disabled until a new grant. No automatic retreat moves in the pilot | (b) Retreat to a home pose: an unplanned motion after a fault. (c) Open the gripper: drops the load (H08) |
| P9 | Records | A physical run record carries the hashes of the cell configuration, the approval and the grant; `mode: physical`; the stop that ended it and why. New failure codes for P6 and P7 are allocated with the first implementation | Log files only: a run must be attributable from its record alone (RFC-001 13.3 "Recorded") |

## 3. Limits stated now

- None of this makes RoboLang a safety function. P6–P8 decide what RoboLang does; the cell's safeguarding decides
  whether people are safe.
- P1–P4 stop an unauthorized or unreviewed task from running. They do not make a reviewed task correct: a person can
  approve a wrong task. Goal verification and the cell's limits remain the controls for that.
- Keys: P3 needs `ed25519` signatures. Core specifies them, but the reference does not implement them, on purpose:
  [0007](0007-rl-arch-02-core-and-reference-simulation.md) R5 keeps the core wheel free of third-party runtime
  dependencies. P3 therefore needs a new dependency (for example `cryptography`), confined to the robot backend
  package or an optional extra, never the core wheel. That is the first code item, a slice of G4, and needs the
  owner's approval as a new dependency.

## 4. Delivery, once approved

1. `ed25519` signing and verification in an optional extra or the backend package (not the core wheel), with key
   references in the cell configuration and tests. Simulation only.
   **Done 2026-11-12:** extra `robolang[ed25519]` (`cryptography>=42`). A receiver accepts `ed25519` only when its
   context lists it; the default receiver, every fixture and the frozen gate 2 cases are unchanged (`C204`). Physical
   mode still rejects every algorithm. The Core 0.1 and 0.2 texts are frozen and keep "not implemented by the reference
   simulation", which remains true of the default install.
2. Cell configuration, task approval and grant documents: schemas, a reader, and checks, all exercised in the
   symbolic simulator with `mode: physical` still rejected.
   **Done 2026-11-12:** [authorization-v0.1.md](../authorization-v0.1.md) and `robolang.authorization`: the three
   documents, a strict reader and the chain check (`A701`–`A726`); a valid chain still ends in `A731`. The grant is
   its own document, `robolang.run_grant`, not a Core `authorization_decision`: that payload has no field for the
   approval's hash, and Core 0.1 and 0.2 are frozen. A later Core version can carry it.
3. Stop and safe-state behaviour in the Gazebo backend: cancel, expiry and revocation tested in simulation.
4. Physical mode itself: only with G2, G5, G7 and the owner's signature on the safety case.

## 5. Needs the owner

The new `ed25519` dependency and where it lives (section 3), P1 (enablement only by a signed cell configuration), P2 (no standing physical authorizations in the pilot; who sets
the maximum lease), P5 (one person may review and grant, or two are required), and P8 (default safe state: hold,
gripper unchanged).
