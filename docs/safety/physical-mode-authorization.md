# Physical-mode authorization design — DRAFT

**Status:** DRAFT design, 2026-11-11, for the owner's review. STRATEGY H4, decision D2. **Design only: nothing is
implemented and physical execution stays unauthorized.** Implementing any part needs its own decision record and the
owner's approval. Builds on RFC-001 13.2–13.3 (authorization is local; mode is explicit), development plan decision A5
(physical execution disabled by default) and Domain 12 (an authorization gate between validation and execution).

## 1. Principles

1. **Off by default, on by record.** Physical mode exists only where a signed enablement record names the site, the
   robot, the backend and the safety case version. No flag, environment variable or message can enable it.
2. **Every layer can only narrow.** The enablement record bounds the site configuration, which bounds standing
   authorizations, which bound each goal's grant. Nothing downstream widens what is upstream.
3. **Fail closed.** Any missing, expired, unverifiable or unknown item means no motion.
4. **Simulation first.** A task runs physically only after the same task, site and robot profile ran in simulation.
5. **RoboLang stops; the robot protects.** A RoboLang stop is a request that must be confirmed; protection comes from
   the robot's safety functions (safety case A1).

## 2. Enablement levels

| Level | What moves | Gate |
|---|---|---|
| L0 Simulation | Nothing physical (today) | None beyond today's rules |
| L1 Connected dry run | Nothing: real controller and sensors connected **read-only**; plans validated and rehearsed; observations recorded | Enablement record for L1 |
| L2 Fenced physical | The robot, in a fenced cell, no person inside during motion, reduced speed | Enablement record for L2, completed robot annex, L1 evidence |
| L3 Shared space | Out of scope for this design | A separate safety case |

## 3. The chain of authority

| Layer | Issued by | States | Checked where |
|---|---|---|---|
| Enablement record | The owner, on the safety case and robot annex | Site, robot, backend, level, safety case version, expiry | On the robot, before any physical session |
| Site configuration | Site safety lead | Robot profile with the controller's safety-rated limits, cell boundaries, sensor sources, local safe state | On the robot; its hash must match the enablement record |
| Standing authorization (`mode: physical`) | A named human of the robot's authority domain (RFC-001 13.3) | Goal classes, places, counterparts, limits, lease | On the robot, per goal |
| Per-goal grant | Policy, or a named human where policy requires | One goal | On the robot, when `AUTHORIZING` is entered |

A requester never authorizes itself (`C401`); an AI model is never an issuer.

## 4. Gates before the first motion of a run (all must pass)

1. A valid enablement record for this site, robot, backend and level; not expired.
2. The site configuration hash matches the record.
3. The task validates, `validate --world` is clean against the site, and a simulation run of the same task, site and
   robot profile succeeded within the configured window.
4. A standing authorization or grant in `mode: physical` covers the goal; signatures are Ed25519 with managed keys
   (simulation HMAC is refused in physical mode).
5. The controller reports: emergency stop released, no protective stop, the expected operating mode (reduced speed for
   L2), and a start state within the configured tolerance (default 0.01 rad; see hazard HZ9).
6. Every observation used is fresh under `max_age`; any unknown fails closed.
7. An operator with an independent stop is recorded as present.

## 5. During and after a run

- Any failure, unknown condition, expired lease or lost connection: request a controller stop, **confirm** the
  controller reports stopped, record both, then end the run as failed (hazard HZ6).
- No retries beyond those written in the program.
- The physical run record adds the enablement record ID, the authorization chain, controller safety status at start,
  at every stop and at the end, and the operator ID.
- Revocation of any layer stops new work immediately; work in progress goes to the local safe state.

## 6. Changes this would need (each separately authorized)

`C404` accepts `physical` only with a matching enablement record; a physical flag in the backend that is absent from
every simulation path; Ed25519 keys and key management; the stop-confirmation interface per controller; the L1
read-only mode; quarantine or removal of `legacy/` (hazard HZ15); tests that a missing or wrong item at each gate
blocks motion.

## 7. Questions for the owner

1. The pilot robot and partner (the annex depends on it).
2. Who signs the site configuration and standing authorizations at the pilot site.
3. Whether L1 (connected dry run) is the first milestone after the safety case review. Recommended: yes.
4. Which standards the robot annex references (for example ISO 12100, ISO 10218-1/-2), and whether an external
   reviewer reads the case before L2.
