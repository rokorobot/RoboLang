# RL-FOUNDATION-02 — Engineering contracts for the thirteen domains

**Date:** 2026-10-27
**Status:** **Approved by the owner on 2026-10-27** (decisions G3–G7 in
[decision record 0005](../decisions/0005-ai-first-charter-and-domain-contracts.md)). No
implementation is authorized by it: BASIC-1 still needs the separate RL-BASIC-01b
implementation authorization.
**Builds on:** [RFC-001](rfc-001-ai-native-architecture.md) (D1–D9),
[RL-FOUNDATION-01](rl-foundation-01-language-foundation.md) (F1–F5),
[`simulation-v1.0.md`](../simulation-v1.0.md) (B1–B16), the
[AI-First Language Design Charter](ai-first-language-charter.md) (G1–G3)
**Scope:** Documentation only. No change to RoboLang 1.1, Task IR 1.0, the CLI, the
simulation specification, RFC-001 or any approved decision.

## 1. Summary

This document turns the thirteen domains of RL-FOUNDATION-01 into engineering contracts. They
are designed together, so that they share one set of types and concepts, and implemented in
small, independently testable slices.

- **Section 2** places every domain's concepts in one of four homes and gives the development
  matrix.
- **Section 3** names the building blocks that several domains share, and the order they are
  built in.
- **Section 4** is the contract for each domain, in one fixed template.
- **Section 5** is the work breakdown, in five tracks that respect the approved staging (F3)
  and order (F5).
- **Section 6** is the exact first coding slice, presented for review and for the separate
  RL-BASIC-01b implementation authorization.

The contracts are an engineering baseline, not normative specifications. Any slice that adds
`.rob` syntax, a Task IR kind or a Core schema still needs its own versioned specification,
fixtures and owner decision, as RL-11A and RL-BASIC-01a had.

## 2. Where each domain lives

### 2.1 Four homes

| Home | Meaning | Validated by | Versioned by |
|---|---|---|---|
| **Plan** | Steps, conditions and values inside a task: `.rob` source compiled to Task IR | The Language Kernel (F2) | Language version and IR schema version |
| **Core** | Documents exchanged between participants over A2R and R2R: goals, plans, observations, commitments, outcomes, capability descriptors | The Core path: schema validation and semantic checks, separate from the kernel (F2) | Core schema version (RFC-001 6.2) |
| **Sim** | Inputs and results of the simulation runtime: world file, run record | The world-file validator and the executor (RL-BASIC-01) | World file and run-record schema versions |
| **Ext** | Contracts for components RoboLang does not implement: reasoners, knowledge stores, perception and motion backends | The caller validates every result before use (AF6) | The interface contract's version |

### 2.2 Development matrix

| # | Domain | Today (1.1 and RL-BASIC-01a) | Homes now → later | Stage (F3) | Next slice | Gate |
|---|---|---|---|---|---|---|
| 01 | Motion & Navigation | `move`, `location`; symbolic simulated move | Plan, Sim → Core (places 7.5) | F1 | BASIC-1, BASIC-2 | RL-BASIC-01b authorization |
| 02 | Physical Manipulation | `grasp`, `place`, `object`; gripper profile | Plan, Sim → Core (things 7.4) | F1 | BASIC-1, BASIC-2 | RL-BASIC-01b authorization |
| 03 | Sensing & Perception | `inspect`, `sensor_ok`, `object_exists`; simulated observations | Plan, Sim → Core (observations 7.6), Ext | F1; F2 for Core observations | BASIC-1, BASIC-2 | RL-BASIC-01b authorization |
| 04 | Abstract Goals & Tasks | `task`, parameters, `pre`, `plan`; task selection | Plan → Core (goals 7.7, plans 7.8) | F1 tasks; F2 goals | F2-1 | Core schemas (RL-ARCH-02) |
| 05 | Thinking & Reasoning | None | Ext, Core | F2 | F2-4 | Foundation 1 exit gate |
| 06 | Memory & Knowledge | None (run records are episodic evidence) | Ext, Core | F2 | F2-3 | Foundation 1 exit gate |
| 07 | Data & World Model | Four parameter types; exact quantities; world file 1.0 | Plan (Part B), Sim → Core | F1 | **BASIC-1** | RL-BASIC-01b authorization |
| 08 | Communication | `communicate` (recorded only) | Plan, Sim → Core (envelope 8, A2R 9, R2R 10) | F1 local; F3 A2R/R2R | BASIC-2; RL-ARCH-02 | RL-ARCH-02 authorization |
| 09 | Logic & Control Flow | Sequential `plan`, `pre`, `wait until` | Plan (Part B), Sim | F2 | LANG-1 | Owner selection of Part B |
| 10 | Time & Coordination | `wait for`, `wait until … timeout`; exact simulated clock | Plan, Sim → Core (deadlines, leases 11.1) | F1 clock; F3 concurrency | BASIC-1, BASIC-2 | RL-BASIC-01b authorization |
| 11 | Capabilities & Resources | Simulated robot profile (B13) | Sim → Core (descriptor 7.3, resources 10.3) | F1 profile; F3 resources | BASIC-1, BASIC-2; RL-ARCH-02 | RL-BASIC-01b authorization |
| 12 | Safety & Authorization | `@safety max_speed` (checked, not enforced), `pre`; simulation-only authorization | Plan, Sim → Core (13) | F1 | BASIC-2 | RL-BASIC-01b authorization |
| 13 | Outcomes & Recovery | Run states, failure codes, run record | Sim → Core (outcomes 7.10, failure model 12) | F1 outcomes; F2 recovery | BASIC-3 | RL-BASIC-01b authorization |

Slice identifiers refer to section 5.

## 3. Shared building blocks

These are the pieces that more than one domain needs. Building them once, in the simulation
runtime first, is what keeps thirteen domains one language.

| # | Building block | Domains | First built in | Later reused by |
|---|---|---|---|---|
| K1 | Three-valued truth: `true`, `false`, `unknown` | 03, 04, 09, 12, 13 | BASIC-1 | Conditions (BASIC-2), branching (LANG-2), goal criteria (F2-1) |
| K2 | Exact decimal quantities with units; exact duration arithmetic in milliseconds | 01, 10, 11, 12 | BASIC-1 | Clock (BASIC-2), deadlines and leases (RL-ARCH-02) |
| K3 | Typed entity references: robot, object, location, region; sensors by name | 01, 02, 03, 07, 11 | BASIC-1 | Bindings in Core plans (7.8) |
| K4 | Strict JSON reading: UTF-8, duplicate keys rejected, unknown fields rejected, no floating-point values | 07, and every JSON input | BASIC-1 | IR reader (CORE-1), Core documents (RL-ARCH-02) |
| K5 | Diagnostics located by JSON Pointer, with stable codes | Every JSON input | BASIC-1 | CORE-1, RL-ARCH-02 |
| K6 | World state with unknown facts | 03, 07, 09, 12 | BASIC-1 | Observations (F2-2), memory (F2-3) |
| K7 | Executor state machine and per-step results (RL-12A) | 01, 02, 03, 08, 10, 12, 13 | BASIC-2 | Recovery (F2-5), ROS 2 backend (F3-4) |
| K8 | Deterministic run record and run ID | 13, 03, 08 | BASIC-3 | Outcome evidence (F2-1), memory (F2-3) |

K1–K6 are the first code that serves several domains at once, and they are exactly what the
first coding slice builds (section 6).

## 4. Domain contracts

Each contract uses the same nine aspects. "Sketch" marks a non-normative illustration of
possible future `.rob` syntax; it is not a proposal to add that syntax. Section numbers in the
form "7.6" refer to RFC-001.

### 4.1 Domain 01 — Motion & Navigation

| Aspect | Contract |
|---|---|
| Reuse | `move r to dst` (grammar, static rules, IR kind `move`); `location` type; simulated `move` (B1, B7, B8); RL-12C target resolution; places 7.5 |
| Missing | Geometric places and frames; poses and orientation; approach and reach; motion constraints (speed per goal, places to avoid); trajectories, and trajectory evidence (the motion actually performed); real motion backends |
| Representation | Core: places 7.5 (`pose`, `region`, `named_place`), goal constraints `max_speed` and `avoid_place` (7.7). IR: `move` unchanged; any new motion kind is an additive IR 1.x kind. `.rob`: no new syntax proposed |
| Types and rules | A frame is always named; geometry in different frames is compared only through an explicit transform, otherwise the comparison fails (7.5). SI units as exact decimals. A trajectory is a plan for motion in a named frame; it is never evidence that the motion happened. Only trajectory evidence, recorded as observations of the motion actually performed, can support an outcome. Simulated positions are symbolic and have no trajectory |
| IR and compatibility | IR 1.0 `move` keeps its meaning. New kinds are added, never folded into `move` |
| Validation and authorization | Static: `location` typing. Runtime: readiness (B8), reach and operating area from the capability descriptor (7.3), speed limits against the profile (B4). Authorization is local to the robot (13.2); physical mode disabled by default (A5) |
| Simulation and backends | Simulation: symbolic relocation in zero time. Backend contract: a motion request returns a final, verified result; acceptance of a goal is never success (RL-12D) |
| Acceptance tests | Positive: T1 move effects; later, one named place resolved differently on two profiles (RL-13B). Negative: `S201`/`S202`; unbound or undeclared place (`E505`–`E507`); geometry in an unrelated frame rejected; a planned trajectory offered as evidence of completed motion is rejected; limit above profile (`S001`) |
| Dependencies and priority | Needs 07, 11, 12. **Priority 1** (Foundation 1 through RL-BASIC-01b); geometry with RL-ARCH-02 and Foundation 3 |

### 4.2 Domain 02 — Physical Manipulation

| Aspect | Contract |
|---|---|
| Reuse | `grasp`, `place`; `object` type; simulated semantics (B1); gripper in the profile (B13); things 7.4; RL-12C |
| Missing | Release, push, pull and tool use; contact and grasp state as observable facts; payload limits; partial-grasp evidence |
| Representation | Core: things 7.4 (class, mass with uncertainty), capability kinds for grasping and payload (7.3). IR: additive kinds only, for example a future `release`. `.rob`: no new syntax proposed |
| Types and rules | An object is held by at most one robot; a held object has no location; payload never exceeds the declared capability; grasp state is an observation, not an assumption |
| IR and compatibility | `grasp` and `place` keep their meaning |
| Validation and authorization | Gripper capability (`S211`), empty gripper (`S212`), presence (`S213`/`S214`), co-location (`S215`), holding and position for `place` (`S221`/`S222`); payload against the descriptor later |
| Simulation and backends | Simulation: state changes of B1. Backend contract: grasp and place report a verified gripper result (RL-12D) |
| Acceptance tests | Positive: T1, T7 (`place_elsewhere` reaching `place`). Negative: `S211`–`S215`, `S221`, `S222`, T15 fault |
| Dependencies and priority | Needs 01, 07, 11. **Priority 1** |

### 4.3 Domain 03 — Sensing & Perception

| Aspect | Contract |
|---|---|
| Reuse | `inspect … using` with a mandatory sensor; `sensor_ok`, `object_exists`; simulated observations (B9); observations 7.6 |
| Missing | Confidence, validity and staleness; provenance; measured values with units; detection and identification of unexpected objects; observations as evidence for outcomes |
| Representation | Core: observation 7.6 (`subject`, `property`, `value`, `observed_at`, `valid_for`, `confidence`, `source`). The run record's observation (B9) is mapped onto it in F2-2. IR: `inspect` unchanged. `.rob`: no new syntax proposed |
| Types and rules | An observation is a belief, not a fact. Past `observed_at + valid_for` it is stale and counts as unknown. Confidence is an exact decimal in [0, 1]. A peer's observation is evidence of the peer's belief (13.5) |
| IR and compatibility | Unchanged |
| Validation and authorization | Sensor declared and mounted (`S231`), available (`S232`/`S233`); freshness rules of RL-12B; peer observations under 11.3 and 13.5 |
| Simulation and backends | Simulation: ground-truth observation at the moment of `inspect`. Backend contract: an observation always carries source, time, validity and confidence |
| Acceptance tests | Positive: T8 `t08_inspect_ok`; later, a fresh observation satisfies a goal criterion. Negative: `S231`–`S233`; a stale observation evaluates to unknown; confidence below `min_confidence` fails the criterion |
| Dependencies and priority | Needs 07, 10, 11. **Priority 1** in simulation; Core observations in Foundation 2 (F2-2) |

### 4.4 Domain 04 — Abstract Goals & Tasks

| Aspect | Contract |
|---|---|
| Reuse | `task` with typed parameters, `pre`, `plan`; task selection; IR 1.0 tasks; goals 7.7; plans 7.8 with `plan_kind: robolang.task-ir` |
| Missing | Goals on the executable path; binding a goal to a plan; decomposition into subgoals; skills (RFC-001 open question 12); evaluation of success criteria |
| Representation | Core: goal 7.7 and plan 7.8. IR: unchanged; a plan binds a task to a goal and to entities. `.rob`: **none** — goals and outcomes in `.rob` are RFC-001 open question 8, which stays open |
| Types and rules | A goal states a desired state as subject–property–value triples and must have success criteria (P5). Tasks are means; goals are ends |
| IR and compatibility | Unchanged |
| Validation and authorization | Goal documents through the Core path (F2); plans re-validated under the IR trust rules (CORE-1); a requester is never its own authority (D6) |
| Simulation and backends | Simulation: success criteria evaluated against the final world and observations (F2-1) |
| Acceptance tests | Positive: a goal reached by a plan whose criteria are met ends `succeeded` with evidence. Negative: a goal without success criteria is rejected; a plan with incomplete bindings is rejected; success claimed without evidence is rejected |
| Dependencies and priority | Needs 03, 07, 13 and Core schemas (RL-ARCH-02). **Priority 2** |

### 4.5 Domain 05 — Thinking & Reasoning

| Aspect | Contract |
|---|---|
| Reuse | AI content is untrusted (13.4); RoboLang components stay deterministic (A4); provider-neutral generation and repair loop (RL-20A, RL-20B); machine-actionable diagnostics (AF8) |
| Missing | A proposal contract; hypotheses; alternatives and predictions; rationale and provenance; deterministic selection among alternatives |
| Representation | Ext: a reasoner returns proposals. A proposal is a Core goal or plan with provenance (7.11: provider, model, time), an optional predicted outcome and a confidence. Core: proposals travel as goals and plans with `requested_by`. `.rob`: none |
| Types and rules | A proposal never carries authority. A **hypothesis** is a candidate belief produced by reasoning, kept distinct from a verified observation: it carries the reasoner's provenance, never counts as a fact or as evidence, and can only be confirmed or refuted by an observation. A predicted outcome is not an observation. Confidence never replaces evidence (AF5) |
| IR and compatibility | Unchanged; proposed plans are ordinary plans |
| Validation and authorization | Every proposal passes the Core path or the kernel, the capability check and local authorization. Attempts are bounded (RL-20B). Selection among valid proposals uses a declared, deterministic policy |
| Simulation and backends | A deterministic stub reasoner with fixed proposals for tests. Real reasoners (language models, planners) are external, behind RL-20A |
| Acceptance tests | Positive: a valid proposal becomes a validated plan that waits for authorization. Negative: a malformed proposal is rejected with diagnostics; a proposal claiming authority is rejected; a proposal needing an unsupported capability is rejected deterministically; a hypothesis offered as evidence for a success criterion is rejected; an unavailable reasoner causes no action |
| Dependencies and priority | Needs 04, 06, 07, 12, 13. **Priority 3** (Foundation 2, interfaces only) |

### 4.6 Domain 06 — Memory & Knowledge

| Aspect | Contract |
|---|---|
| Reuse | Run records as episodic evidence (B14); observations 7.6; provenance 7.11 |
| Missing | Knowledge records with validity; episodic, semantic and procedural memory (procedural ties to open question 12); query; persistence; expiry |
| Representation | Ext: a knowledge store appends records and answers queries by subject, property and time, returning records with validity and provenance. Records are Core observations and outcomes. `.rob`: none |
| Types and rules | A memory entry is a belief with a source and an age. Recall returns evidence, never a fact. An expired record counts as unknown (P4) |
| IR and compatibility | Unchanged |
| Validation and authorization | Memory never authorizes anything. Recalled content is subject to the freshness rules. Writing to memory follows a declared policy |
| Simulation and backends | An in-memory reference store for tests; persistent stores are replaceable backends |
| Acceptance tests | Positive: recall returns the latest valid record with its provenance. Negative: an expired record yields unknown; a record without provenance is rejected; recalled content cannot satisfy an authorization check |
| Dependencies and priority | Needs 03, 07, 10, 13. **Priority 3** (Foundation 2). Vocabulary alignment is RFC-001 open question 1 |

### 4.7 Domain 07 — Data & World Model

| Aspect | Contract |
|---|---|
| Reuse | Four parameter types; exact decimal quantities (IR 3.5); world file 1.0 (B12); things, places and observations (7.4–7.6) |
| Missing | Part B extensions (F3): typed values and comparisons, local bindings, structured data; entity properties and relations; entity classes; world-model depth (open question 2) |
| Representation | Plan: each Part B extension is defined first as an IR form (AF1), then given `.rob` syntax through its own versioned language specification. Core: subject–property–value triples (7.6, 7.7). Sim: world file 1.0 |
| Types and rules | No implicit conversions; exact decimals; units mandatory; three-valued truth (K1); typed entity references (K3) |
| IR and compatibility | Existing files keep their meaning. New value and expression forms are additive IR 1.x changes where an IR 1.0 consumer would reject them safely, otherwise IR 2.0 (`ir-schema-v1.0.md` section 1) |
| Validation and authorization | Static typing in the kernel; world-file validation (`E501`–`E511`) |
| Simulation and backends | World file 1.0 is the simulation's world model; a Core world model follows open question 2 |
| Acceptance tests | Positive: every valid world fixture loads into typed values. Negative: one fixture per code `E501`–`E511`; each Part B extension brings its own fixtures |
| Dependencies and priority | None: the foundation for the rest. **Priority 1 — first coding slice (BASIC-1)** |

### 4.8 Domain 08 — Communication

| Aspect | Contract |
|---|---|
| Reuse | `communicate` (free text, recorded only: B5); message envelope 8; A2R 9; R2R 10 (RL-13C and RL-13D superseded) |
| Missing | Typed intents; request and response; acknowledgement; correlation; delivery evidence; waiting for a message (`message_received`, deferred in 1.1 with `E211`) |
| Representation | Core: envelope and A2R/R2R operations. `.rob`: `communicate` stays free text; typed messaging is Core-only for now |
| Types and rules | Sent, delivered and accepted are different (P5). A message never grants authority (P3) |
| IR and compatibility | `communicate` unchanged |
| Validation and authorization | Envelope schema validation; replay and duplicate handling (11.2); expiry (11.1) |
| Simulation and backends | Simulation: recorded only. RL-ARCH-02: in-process A2R and R2R |
| Acceptance tests | Positive: T10 (message recorded); the RFC-001 section 18 scenario in RL-ARCH-02. Negative: a message alone moves no robot; a duplicate is handled idempotently; an expired message is rejected |
| Dependencies and priority | Needs 10, 12 and Core schemas. **Priority 2** (RL-ARCH-02) |

### 4.9 Domain 09 — Logic & Control Flow

| Aspect | Contract |
|---|---|
| Reuse | Sequential `plan`; `pre`; `wait until`; the four conditions; three-valued evaluation (B2, K1) |
| Missing | Combining conditions (and, or, not); comparisons of values; branching; bounded repetition; failure-handling branches |
| Representation | Plan: Part B extensions, each defined first as an IR form, then `.rob` syntax in its own versioned specification. Sketch only: `if region_clear cell { … } else { … }` |
| Types and rules | Conditions are three-valued. A branch whose condition is unknown fails the run rather than taking either branch. Every repetition has a static bound; no unbounded loops and no recursion |
| IR and compatibility | Nested step lists are new IR kinds; an IR 1.0 consumer rejects them, so they can be an additive IR 1.x change |
| Validation and authorization | Static: bound present and within a total step budget. Runtime: unknown condition fails closed |
| Simulation and backends | Deterministic evaluation in the executor (K7) |
| Acceptance tests | Positive: the taken branch is recorded. Negative: an unknown branch condition fails closed; repetition without a bound is rejected statically; nesting beyond the limit is rejected |
| Dependencies and priority | Needs 07 (values). **Priority 2** (Part B, scheduled independently) |

### 4.10 Domain 10 — Time & Coordination

| Aspect | Contract |
|---|---|
| Reuse | `wait for`, `wait until … timeout`; exact simulated clock (B7); deadlines (7.7); leases (11.1); time assumptions (11.4) |
| Missing | Task deadlines; temporal relationships between events, goals and actions (before, after, during, within a bound); scheduling; synchronization between robots; parallel branches; clock synchronization (open question 3) |
| Representation | Core: deadlines, leases, and temporal relationships stated between identified events, goals and actions. Plan: parallel blocks later (Foundation 3). `.rob`: no new syntax proposed now |
| Types and rules | Durations are exact (K2). Absolute times are UTC. A temporal relationship names both sides and an explicit bound; one whose truth cannot be established (for example, a missing timestamp) is unknown and fails closed. An expired lease fails closed |
| IR and compatibility | Unchanged until parallel blocks |
| Validation and authorization | A deadline already in the past is rejected; waits always carry a timeout (already in 1.1) |
| Simulation and backends | The simulated clock; concurrency semantics with Foundation 3 |
| Acceptance tests | Positive: T9 cases; later, a relationship such as "action after observation, within 10 s" holds on recorded times. Negative: `S241`; an expired lease blocks the dependent action (RL-ARCH-02); a relationship referring to an unknown event is rejected |
| Dependencies and priority | Needs 07, 09. **Priority 1** for the clock; concurrency is Foundation 3 |

### 4.11 Domain 11 — Capabilities & Resources

| Aspect | Contract |
|---|---|
| Reuse | Simulated robot profile (B13); capability descriptor 7.3 (RL-13A); resources 10.3 |
| Missing | The descriptor schema; discovery; dependencies and prerequisites between capabilities; a task's derived requirements; a comprehensive capability preflight; reservation and allocation of resources; expiry |
| Representation | Core: capability descriptor (schema drafted in RL-ARCH-02). Sim: the profile is a subset of a descriptor. Plan: a task's requirements are derived from its IR (which actions and sensors it uses), never declared separately |
| Types and rules | A capability is declared, not proven. `unsupported` is explicit. A capability may declare prerequisites (placing requires grasping; inspecting requires a mounted, available sensor); a capability whose prerequisite is unsupported or unavailable counts as unavailable. A stale descriptor counts as absent |
| IR and compatibility | Unchanged; requirements are computed from IR 1.0 |
| Validation and authorization | **Staged.** In RL-BASIC-01, as approved (B4, B13; `simulation-v1.0.md` 4.3 and 4.5), capabilities are checked at two points: speed maxima before `AUTHORIZED` (`S001`, `S002`), and the gripper and sensors at the step that needs them (`S211`, `S231`–`S233`), so earlier steps may already have run. A **comprehensive preflight**, deriving all of a plan's requirements and their prerequisites from the IR and checking them before `RUNNING`, is planned later, with capability descriptors (RL-ARCH-02), and needs its own specification. It does not change the approved simulator behavior or fixtures |
| Simulation and backends | Profile checks (`S211`, `S231`, `S002`) |
| Acceptance tests | Positive: the requirements derived from `pick_and_place.rob` are movement, a gripper and placing. Negative, RL-BASIC-01: a robot without a gripper fails at `grasp` (`S211`) after its first `move` has run (`t05_no_gripper`). Negative, with the later preflight: the same plan is rejected before `RUNNING`; a capability whose prerequisite is missing is unavailable; a stale descriptor is treated as absent |
| Dependencies and priority | Needs 07. **Priority 1** for the profile; descriptors with RL-ARCH-02; resources Foundation 3 |

### 4.12 Domain 12 — Safety & Authorization

| Aspect | Contract |
|---|---|
| Reuse | `@safety max_speed` (checked against the profile, not enforced: B4); `pre`; fail closed (A2); simulation-first (A5); trust and authority (13); simulation-only authorization of the simulation specification (4.3) |
| Missing | A policy language (open question 11); standing authorizations with an explicit mode (13.3); constraints beyond speed (`no_go_region` is deferred in 1.1 with `E211`); revocation; authorization records |
| Representation | Core: authorization and standing authorization (13.3). `.rob`: further `@safety` keys only through a versioned specification |
| Types and rules | Authority is local to the robot. Mode is `simulation` or `physical`, always explicit; physical is disabled by default |
| IR and compatibility | Unchanged |
| Validation and authorization | An authorization gate sits between validation and execution on every path (RL-FOUNDATION-01 section 2) |
| Simulation and backends | Simulation mode only |
| Acceptance tests | Positive: an authorized simulation run. Negative: no authorization means no execution; a request for physical mode is rejected; a requester authorizing itself is rejected (D6); a revoked authorization stops further steps |
| Dependencies and priority | Needs 10, 11, 13. **Priority 1** for the simulation gate; policy and standing authorizations later |

### 4.13 Domain 13 — Outcomes & Recovery

| Aspect | Contract |
|---|---|
| Reuse | Run states, run record and failure codes (B14, B16); RL-12A; outcomes 7.10, including `unknown`; failure model 12 |
| Missing | Outcomes verified against goal criteria; partial completion; retry and recovery policies; reporting an unknown outcome after a crash; reporting outcomes to the requester |
| Representation | Core: outcome 7.10. Sim: the run record, mapped onto Core outcomes later. `.rob`: none (open question 8) |
| Types and rules | Success requires evidence. `unknown` is a first-class outcome. Retries are bounded and idempotent |
| IR and compatibility | Unchanged |
| Validation and authorization | An outcome without verification is rejected. A recovery action is authorized like any other action |
| Simulation and backends | Simulation: T15 and T16; recovery in Foundation 2 (F2-5) |
| Acceptance tests | Positive: run records with every terminal state. Negative: success without evidence is rejected; a retry beyond its bound fails; an unauthorized recovery step does not run |
| Dependencies and priority | Needs 03, 04, 12. **Priority 1** for run outcomes; recovery Foundation 2 |

## 5. Work breakdown

Five tracks. Each item is one pull request, or one specification PR followed by one
implementation PR. Every implementation item needs owner authorization before coding starts;
every item that adds `.rob` syntax, an IR kind or a Core schema needs its own versioned
specification first.

### 5.1 Track A — Foundation 1: RL-BASIC-01b (approved specification, authorization pending)

RL-BASIC-01b is split into three slices so that no PR is large. Together they meet the
Foundation 1 exit gate (F3).

| Slice | Content | Domains | Verified by |
|---|---|---|---|
| **BASIC-1** | Shared values (K1, K2, K3) and world file 1.0 reading and validation (K4, K5, K6). Section 6 | 07, 01, 02, 03, 10, 11, 12 | World fixtures, `E501`–`E511` |
| BASIC-2 | Executor state machine (K7): authorization check, `pre`, the six statements, simulated clock, events, faults and cancellation, producing an in-memory run result | 01, 02, 03, 08, 10, 11, 12, 13 | Run cases T1, T3–T10, T15–T17, compared field by field |
| BASIC-3 | Run record 1.0, run ID (K8), `robolang simulate`, exit statuses, README; isolation test (T18) | 13, 08, 03 | T2, T11–T14, T18, T19 |

### 5.2 Track B — Foundation 1 Part B language extensions (independent, owner-selected)

Each is a versioned language specification (the RL-11A pattern) followed by an
implementation, defined first as IR forms (AF1). They do not block Track A or RL-ARCH-02
(F3, F5). Order (G6):

| Item | Content | Domains |
|---|---|---|
| LANG-1 | Combined conditions: `and`, `or`, `not` over the existing conditions, three-valued | 09 |
| LANG-2 | Conditional branching with fail-closed unknown | 09 |
| LANG-3 | Bounded repetition | 09, 10 |
| LANG-4 | Typed values and comparisons (quantities with units) | 07, 09 |
| LANG-5 | Local bindings | 07 |
| LANG-6 | Structured data | 07 |

### 5.3 Track C — Core (after BASIC-3, subject to RL-ARCH-02 authorization)

| Item | Content | Domains |
|---|---|---|
| CORE-1 | **IR reader and re-validator**: reads an externally supplied Task IR 1.0 document and performs the consumer duties of `ir-schema-v1.0.md` section 2. Lets an AI submit Task IR JSON directly (AF2) and is needed by any robot receiving a plan in a Core message (RFC-001 6.3). Specification, then code | 04, 05, 07, 12 |
| RL-ARCH-02 | As approved (D8, F5): draft Core schemas for the message and the capability descriptor; in-process simulation of the RFC-001 section 18 scenario. Reuses the BASIC-2 executor design | 04, 08, 10, 11, 12, 13 |

### 5.4 Track D — Foundation 2 (after the Foundation 1 exit gate)

| Item | Content | Domains |
|---|---|---|
| F2-1 | Goal execution in simulation: a goal bound to a plan, success criteria evaluated with evidence | 04, 13, 03 |
| F2-2 | Core-shaped observations in simulation: validity, staleness, provenance, confidence | 03, 06, 10 |
| F2-3 | Knowledge-store interface and in-memory reference store | 06 |
| F2-4 | Reasoner proposal interface and deterministic stub reasoner | 05 |
| F2-5 | Bounded recovery and retry policies | 13, 09 |
| F2-6 | Decision record on world-model depth (RFC-001 open question 2) | 07 |

### 5.5 Track E — Foundation 3 (once the basic semantics are proven)

| Item | Content | Domains |
|---|---|---|
| F3-1 | Concurrency and parallel blocks | 10, 09 |
| F3-2 | Resources and reservations (R2R 10.3) | 11, 08 |
| F3-3 | Portable skills (open question 12) | 04, 11 |
| F3-4 | ROS 2 backend against fake action servers (RL-12D) | 01, 02, 03 |
| F3-5 | Two-profile portability (RL-13A, RL-13B, RL-13E) | 01, 02, 11 |
| F3-6 | A2A binding | 08 |

### 5.6 Order

```text
BASIC-1 → BASIC-2 → BASIC-3 ──(Foundation 1 exit gate)──> F2-1 … F2-6 ──> F3-1 … F3-6
                         │
                         └──> CORE-1, RL-ARCH-02   (separate authorization)

LANG-1 → LANG-2 → … → LANG-6   (independent; the owner selects which and when)
```

## 6. First coding slice: BASIC-1 (RL-BASIC-01b-1)

**Purpose.** Build the shared values and the world-file reader that seven domains depend on,
exactly as `simulation-v1.0.md` section 3 defines them. Nothing executes yet.

**Status.** Fully covered by the approved RL-BASIC-01a specification (B12, B16). Coding starts
only after the owner's separate RL-BASIC-01b implementation authorization (F4, F5). This
document does not grant it.

**Files added**

| File | Content |
|---|---|
| `src/robolang/sim/__init__.py` | Package marker; states that the package is simulation-only and imports no adapter or ROS 2 module |
| `src/robolang/sim/values.py` | `Truth` (`TRUE`, `FALSE`, `UNKNOWN`); exact decimal parsing per `ir-schema-v1.0.md` 3.5; durations converted exactly to milliseconds (`ms` × 1, `s` × 1000, `min` × 60000) with canonical text; speed values with unit |
| `src/robolang/sim/world.py` | `load_world(data: bytes, filename: str, task: ir.Task) -> World`: strict JSON reading (duplicate keys, floating-point values), world file 1.0 validation (sections 3.1–3.8) and the bindings of the selected task. Frozen dataclasses for the world. `WorldError` with code `E501`–`E511`, JSON Pointer and message, formatted as section 2.4 shows |
| `tests/test_sim_values.py` | Unit tests for `Truth`, decimal parsing and duration conversion (`2.5 s` = `2500`, `0.5 ms` = `0.5`, `1 min` = `60000`; equal times in different units compare equal) |
| `tests/test_sim_world.py` | Every world fixture loads, or fails with the code its case states (T13, T14); at least one inline negative test per code `E501`–`E511`; the loaded world matches `base.json`; importing `robolang.sim` loads no `rclpy` and no `robolang_adapter*` module (part of T18) |

**Files changed:** `pyproject.toml` only, to add `robolang.sim` to the explicit `packages`
list so that it is installed.

**Not in BASIC-1:** condition evaluation, the executor, the clock, the run record, the run ID, the
`simulate` command, any change to the lexer, parser, validator, IR, CLI or v1 runtime.

**Verification**

```bash
python -m pytest -q tests/test_sim_values.py tests/test_sim_world.py tests/test_simulation_fixtures.py
python -m pytest -q
python -m ruff check .
```

Expected: the new tests all pass; the full suite passes with the existing 606 tests unchanged
and 6 expected failures; ruff reports no issues; CI passes on Ubuntu and Windows with Python
3.11 and 3.13.

**Size.** About 400–500 lines of code and 300 lines of tests. One PR.

## 7. Decisions

The charter, the four homes, these contracts, the work breakdown, CORE-1 and the scope of BASIC-1 were
approved on 2026-10-27 and are recorded in [decision record 0005](../decisions/0005-ai-first-charter-and-domain-contracts.md).
No approved decision is amended.
