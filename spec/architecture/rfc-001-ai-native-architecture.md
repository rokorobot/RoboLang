# RFC-001 — RoboLang AI-native semantic and communication architecture

**Status:** Architecture approved by the owner on 2026-10-19: decisions D1–D9, with
qualifications on D5 and D8, recorded in [decision record 0002](../decisions/0002-ai-native-architecture.md).
The decisions were taken on revision 6; revision 7 records them and changes no content.
**Date:** 2026-10-19 (revision 7)
**Milestone:** RL-ARCH-01
**Changes to RoboLang 1.1:** None. The language, the grammar, IR schema 1.0, the `robolang`
command and every other implemented behavior are unchanged by this RFC.

> The architecture in this RFC is approved. It defines no implemented behavior. Every
> message format and schema below is a **draft (Core version 0.x)**, to be validated by
> RL-ARCH-02, not a published protocol, and nothing here describes a capability RoboLang
> has today.

---

## 1. Summary

RoboLang 1.1 is a validated task-description language for a single robot: a typed task is
written, checked and turned into a versioned intermediate representation (IR). It does not
execute anything and it does not communicate.

This RFC establishes that RoboLang's long-term identity is **RoboLang Core: a shared,
transport-independent semantic foundation for intelligent machines working in the physical
world**, used by two interaction profiles:

| Profile | Between | Purpose |
|---|---|---|
| **A2R** | AI agents and robots | An agent proposes a physical outcome; a robot assesses, negotiates, executes under its own authority and reports evidence |
| **R2R** | Robots and robots | Embodied participants request help, offer it, commit to responsibilities, share resources and hand over custody |

A2R and R2R are two profiles of **one** model: the same definitions of goals, capabilities,
places, observations, commitments, outcomes and evidence. Neither profile defines a concept
of its own that the other could interpret differently.

The interoperability contract is **a set of versioned Core JSON schemas** (section 6). The
existing RoboLang 1.1 task IR stays a supported plan representation inside that contract, and
`.rob` stays an authoring format. Nothing built so far is discarded.

Agent-to-agent communication is already covered by A2A. RoboLang treats A2A as a first-class
interoperability target, but not as a required transport: Core messages are meaningful
without it, and two robots must be able to cooperate with no A2A service, no cloud and no AI
model available.

The owner approved nine decisions (section 20) on 2026-10-19. The next planned milestone is
one small executable proof (section 19), before any protocol is formalized; its
implementation requires separate owner authorization.

### 1.1 Long-term ambition

The mission, approved as decision D1, is:

> RoboLang is an open, AI-native interoperability platform for embodied intelligence. It
> defines shared semantic contracts, A2R and R2R communication profiles, portable robotic
> capabilities, and verifiable cooperation across heterogeneous robots, AI systems and
> execution technologies.
>
> RoboLang is designed to support a vendor-neutral ecosystem of interoperable
> implementations, reference runtimes, developer tools and reusable skills, without
> requiring one operating system, middleware, controller, AI model or cloud provider.

Internally this has been described as "the Linux of robotics". The comparison is about the
**ecosystem**, not the software: Linux succeeded because many independent parties could build
on shared, stable interfaces. RoboLang is not an operating system kernel and does not
compete with ROS 2 by recreating what ROS 2 does. Its distinguishing aim is a common
semantic and cooperation layer that works across ROS-based and non-ROS-based machines, and
lets newer architectures such as embodied foundation models and learned controllers take
part without pretending every capability is a conventional motion command.

| Linux ecosystem concept | RoboLang counterpart (target, not current) |
|---|---|
| Kernel and common system interfaces | RoboLang Core semantic model and schemas |
| Hardware drivers | Capability adapters and manufacturer interfaces (execution backends) |
| Interprocess communication | A2R and R2R |
| System services | Task lifecycle, coordination, authorization and recovery |
| Software packages; package managers | Portable, versioned robot skills; a future skill registry |
| Distributions | Manufacturer- or application-specific RoboLang implementations |
| Development tools | Compiler, validator, debugger, simulator, protocol inspector |
| Open-source community | Public specifications, conformance tests and independent implementations |

The test of the ambition is **interchangeability**: an organization can change its AI
provider, robot platform, controller or transport without redesigning the meaning of its
tasks.

"The Linux of robotics" is a description of intent for internal use. It is not a claim the
project can make publicly until independent implementations interoperate (section 1.4).

### 1.2 Five pillars

| Pillar | Meaning | Where in this RFC |
|---|---|---|
| **Core — shared understanding** | One representation of the physical world, capabilities, intentions, constraints, observations and verified outcomes, shared by every implementation | Sections 6 and 7 |
| **A2R and R2R — communication** | AI agents propose goals to robots; robots negotiate, commit, share knowledge and coordinate directly. A2A through a first-class binding, not as the only mechanism | Sections 8 to 11, 14 |
| **Portable skills** | Reusable capabilities such as `deliver_object` or `handover_payload`, each declaring required capabilities, preconditions, constraints, expected outcomes, compatible execution interfaces and test evidence. A skill is portable only when a receiving robot can demonstrably satisfy its requirements, not because it recognizes the skill's name | Future plan representation (6.3); open question 12 |
| **Trustworthy local autonomy** | Robots keep local authority; messages never confer permission to move; offline operation, signed identities, leased commitments, uncertainty, recovery and auditable outcomes; safety-rated controls stay independent | Sections 11 to 13 |
| **Open developer ecosystem** | Public specifications, stable versioning, SDKs, conformance suites, documentation, skill registries and vendor-neutral governance | P10; open questions 13 and 14 |

### 1.3 What is proposed for standardization and what is experimental

| Proposed for standardization (once stable) | Experimental or implementation-specific |
|---|---|
| Core schemas: message and capability descriptor (section 6) | The Python reference implementation and any reference runtime |
| Meaning of Core object kinds and states (sections 7 to 10) | Execution backends and their adapters |
| Lease, replay, evidence and failure semantics (sections 11 and 12) | Local policy engines and how policies are written |
| Authority rules (section 13) | Simulator, debugger and developer tools |
| Binding rules: what any binding must preserve (section 14) | Particular bindings beyond A2A and the local transport |
| Conformance tests for all of the above | Skill registry, packaging and distribution |

The architecture is approved, but no part of it is a standard yet. "Proposed for standardization" means the part
other implementations would have to follow; it becomes binding only after review, a stable
version and passing interoperability gates.

### 1.4 How the ambition is tested

The first measure of success is interoperability, not the number of features.

| Gate | Requirement | Milestone |
|---|---|---|
| **Gate 1 — reproducible reference** | A reproducible simulation of the reference scenario (section 18) shows correct messaging, local authorization, failure handling and evidence-based completion | RL-ARCH-02 |
| **Gate 2 — independent implementation** | A participant implemented independently, in a different codebase, communicates with the reference implementation using only the published schemas and this specification, with no private integration logic | Later milestone, not yet scoped |

Gate 2 is what distinguishes an open standard from a product: a system built by one team
against its own components has not shown that others can implement it.

## 2. Why this RFC, and how it relates to the existing plan

The owner's direction (2026-10-18/19) is that RoboLang should be future-first and centered
on communication between AI and robots and between robots, implementing existing systems such
as ROS 2 where useful but not letting their abstractions define the language.

Much of what this needs is already in [`docs/DEVELOPMENT_PLAN.md`](../../docs/DEVELOPMENT_PLAN.md),
scheduled late and framed around a single-robot executor:

| Existing plan item | Concept in this RFC |
|---|---|
| RL-12A executor states `CREATED → VALIDATED → AUTHORIZED → RUNNING → …` | Goal lifecycle (9.2) |
| RL-12B evaluated preconditions, freshness, fail-closed | Observations (7.6); evidence and freshness (11) |
| RL-12C capabilities registry, target resolution | Capability descriptors (7.3); places with frames (7.5) |
| RL-13A versioned capability descriptor schema | Capability descriptors (7.3) |
| RL-13C typed envelope: sender, recipient, correlation ID, expiry | Message envelope (8) |
| RL-13D handoff `OFFERED → ACKNOWLEDGED → ACCEPTED → COMPLETED` | Custody transfer (10.4) |
| RL-20C provenance, review gate, audit trail | Provenance (7.11); authority (13) |
| Mission: "delegate execution to established ROS 2 controllers" | Execution backends behind bindings (14) |

What changes is the **order and the center of gravity**:

- Communication and the shared semantic model move from v1.3/v2.0 to the next milestone.
- A single-robot executor is still needed (a robot cannot report a verified outcome without
  one), but it is built as the execution side of A2R, not as the end goal.
- ROS 2, MoveIt and vendor SDKs become execution backends behind bindings, chosen per robot,
  instead of the model the language is shaped after.

Section 17 lists where the plan and this RFC disagree. The decisions are taken; the plan is
updated in a separate, owner-approved change.

## 3. Scope and non-goals

**In scope:** RoboLang Core (semantic model and schemas), the A2R and R2R interaction
profiles, lifecycle, lease, replay, evidence and failure semantics, trust and authority
rules, and the boundaries to bindings and execution backends.

**Out of scope for RoboLang Core and the initial A2R and R2R profiles.** These capabilities
may be provided through independent modules, execution backends or future extensions,
without becoming mandatory Core requirements:

- Real-time or closed-loop control. A2R and R2R carry goals, commitments and evidence, not
  servo set-points or control loops.
- Motion planning, kinematics, perception and learned policies. Today these are execution
  backends; how they are described and exchanged (for example as capabilities or portable
  skills) is an open question, not an exclusion (section 21).
- A new network stack where existing transports suffice.
- Multi-party consensus among many robots. Version 0 defines **bilateral** commitments only
  (section 10); multi-party coordination may be added by a later extension.

**Outside the authority of the Core protocol.** Independent safety-rated functions
(emergency stops, protective stops, speed and separation monitoring) and risk assessment stay
in independent, safety-rated systems. No RoboLang message, profile or extension can command,
override or disable them.

RoboLang does not claim to be the first robot communication protocol. Related work exists
(section 15).

## 4. Principles

| # | Principle | Consequence |
|---|---|---|
| P1 | **Semantics before syntax** | Concepts are defined as data with schemas first. Text syntax for them comes later, if at all |
| P2 | **The Core schemas are the contract** | Interoperability is defined on versioned JSON documents (section 6). `.rob` text is one authoring surface |
| P3 | **A message never grants authority** | No message, however well-formed or well-signed, can by itself make a robot move. The robot decides under its local policy (section 13) |
| P4 | **Fail closed** | Unknown, missing, stale, unverifiable or contradictory information blocks the affected action. Inherited from the plan's decision A2 |
| P5 | **Outcomes, not dispatch** | "Message sent" and "command accepted" are never success. Success is a verified outcome with evidence |
| P6 | **Local first** | Two robots must be able to cooperate with no cloud, no A2A service and no AI model reachable. Loss of connectivity has defined behavior |
| P7 | **Independence** | No dependency on one AI provider, robot vendor, operating system, middleware or transport |
| P8 | **Provenance everywhere** | Every assertion carries who made it, when, from what source and how confident it is |
| P9 | **Integrate, do not let integration define** | ROS 2, MoveIt, A2A, MCP and vendor SDKs are used through bindings. If an existing system's abstraction conflicts with the model, the model wins and the binding adapts |
| P10 | **Ecosystem before implementation** | No part of RoboLang Core is defined by what the Python reference implementation happens to do. Every published schema and protocol rule must be implementable independently, in any language and on embedded targets, from the specification alone. Conforming to RoboLang never requires installing the RoboLang Python package |

## 5. Layers

```text
  AI agents, planners, embodied models, humans
                    |
                    |  A2R
                    v
  +-----------------------------------------------+
  | RoboLang Core                                 |      R2R
  | versioned schemas and semantics: participants,| <---------> other robots
  | capabilities, things, places, observations,   |
  | goals, plans, commitments, outcomes, evidence |
  +-----------------------------------------------+
                    |
          bindings (transport and ecosystem)
          A2A  |  local peer-to-peer  |  ROS 2  |  MCP
                    |
          execution backends (per robot)
          ROS 2 actions, MoveIt, Nav2, vendor SDKs, learned policies, simulator
                    |
          independent controllers and safety-rated systems
```

- **RoboLang Core** is transport-independent and backend-independent. A Core message means
  the same thing whichever binding carried it.
- **Bindings** carry Core messages over a transport and map identities. They add no meaning
  and remove none.
- **Execution backends** turn an authorized plan into motion on one specific robot and report
  what happened. They are chosen per robot and are invisible to the other side of A2R or R2R.

**A2R or R2R is decided by the interaction, not by the endpoint.** A robot may run an AI model
internally. When a participant asks a robot to bring about a physical outcome that the
participant has no authority to carry out itself, that is A2R. When embodied participants
coordinate their own physical actions, resources or custody, that is R2R, whatever software
they run.

## 6. The Core contract: schemas, versions and representations

### 6.1 What the contract is

The interoperability contract is a set of **JSON Schema (draft 2020-12) documents** published
in this repository under `spec/core/` (proposed location, not yet created). Two participants
interoperate if they exchange documents that validate against the same schema versions and
give them the meaning this RFC defines. Nothing else is part of the contract: not a
programming language, not a library, not a transport.

There are two top-level document types:

| Schema identifier | Document | Notes |
|---|---|---|
| `robolang.core.message` | A message (section 8), including its payload | Every Core object kind (7.2 to 7.11) is defined as part of this schema |
| `robolang.core.capability-descriptor` | A robot's capability descriptor (7.3) | Separate because it is published and cached independently of any conversation |

Every top-level document carries `schema` and `schema_version`. Objects inside a document are
identified by `kind`, and their definitions are those of the enclosing document's schema
version. There is no separate version per object kind.

### 6.2 Versioning

The rule is the one IR schema 1.0 already uses (IR schema 1.0, section 1):

- The **first** number changes when an existing consumer could misread a document: a field
  removed, renamed or redefined, or a state or outcome value added that a consumer must
  handle.
- The **second** number changes when fields or object kinds are only added and an existing
  consumer can safely reject or ignore them as specified.
- A consumer checks `schema` and `schema_version` first and rejects versions it does not
  support. Correct version identifiers are claims, not evidence (IR schema 1.0, section 2).
- Unknown fields are rejected, not ignored, unless a schema explicitly marks an extension
  point.
- Version `0.x` is experimental. Until 1.0, any change may be breaking.

Version negotiation between two participants is a field of discovery (`discover`,
`advertise`): each side states the Core versions it supports, and a conversation uses one
version both support, or does not start.

### 6.3 Plan representations

A plan (7.8) names its representation with `plan_kind` and `plan_kind_version`. The Core
schema lists the representations it admits:

| `plan_kind` | Version | Status |
|---|---|---|
| `robolang.task-ir` | `1.0` | Supported: [IR schema 1.0](../ir-schema-v1.0.md), unchanged |
| others (for example behavior trees, calls to a robot's own skills) | — | Future, each added by a new Core schema version |

A plan representation is a payload. Its own schema and trust rules still apply; in
particular, an executing robot re-validates a task IR document and never trusts it because it
arrived inside a valid Core message (IR schema 1.0, section 2).

### 6.4 Authoring formats

`.rob` source is an **authoring format**: humans and AI models write it, and the RoboLang 1.1
compiler turns it into task IR. It is not itself exchanged as part of the contract. Other
authoring formats (a graphical editor, an AI model producing Core JSON directly) are equally
valid if they produce valid Core documents.

## 7. Core semantic model

Every example below is a **proposed** JSON shape. Field names follow the IR's conventions:
snake_case, `kind` first where an object has variants, and quantities as exact decimal
strings with an explicit unit (IR schema 1.0, section 3.5). Units are SI. Coordinate frame and
axis conventions follow ROS REP-103 and REP-105 unless a robot's descriptor states otherwise,
and a frame is always named, never implied.

### 7.1 Identifiers and references

Every entity has a URI-like identifier that is unique within a deployment, for example
`rl:robot:amr-07`, `rl:agent:planner-1`, `rl:thing:container-C42`, `rl:place:station-S3`.
Identifiers are opaque: no meaning may be derived from their spelling.

### 7.2 Participants

A participant is anything that sends or receives RoboLang messages.

```json
{
  "kind": "robot",
  "id": "rl:robot:amr-07",
  "display_name": "AMR 07",
  "public_key": "ed25519:…",
  "capability_descriptor": "rl:capdesc:amr-07@3",
  "authority_domain": "rl:domain:warehouse-north",
  "core_versions": ["0.1"]
}
```

`kind` is `robot`, `agent` or `human_interface`. `authority_domain` names whose local policy
governs the participant (section 13).

### 7.3 Capability descriptors

What a robot can do, what it cannot, and under what limits. This is RL-13A, generalized.

```json
{
  "schema": "robolang.core.capability-descriptor",
  "schema_version": "0.1",
  "id": "rl:capdesc:amr-07@3",
  "robot": "rl:robot:amr-07",
  "capabilities": [
    {
      "kind": "transport_payload",
      "max_mass": { "value": "40", "unit": "kg" },
      "max_speed": { "value": "1.2", "unit": "m/s" },
      "operating_area": "rl:place:warehouse-north-floor"
    },
    { "kind": "dock_at", "places": ["rl:place:station-S3", "rl:place:handoff-H1"] }
  ],
  "unsupported": ["grasp", "inspect"],
  "issued_at": "2026-10-19T06:00:00Z",
  "valid_until": "2026-10-19T18:00:00Z",
  "signature": { "alg": "ed25519", "key": "rl:robot:amr-07#k1", "value": "…" }
}
```

- A capability is **declared**, not proven. Declarations are inputs to assessment, never a
  guarantee (P3, P4).
- `unsupported` is explicit, so "not listed" and "known not to work" are different.
- Descriptors expire. A stale descriptor is treated as absent.
- Alignment with existing ontologies (IEEE 1872 family, OMG RoSO) is an open question
  (section 21).

### 7.4 Things

Physical objects a task refers to.

```json
{
  "kind": "thing",
  "id": "rl:thing:container-C42",
  "class": "container",
  "mass": { "value": "25", "unit": "kg", "uncertainty": "2" }
}
```

### 7.5 Places

A RoboLang 1.1 `location` or `region` is a name only. The Core gives places geometry with an
explicit frame.

```json
{
  "kind": "pose",
  "id": "rl:place:station-S3",
  "frame": "map",
  "position": { "x": "12.40", "y": "3.05", "z": "0", "unit": "m" },
  "orientation": { "yaw": "1.5708", "unit": "rad" }
}
```

```json
{
  "kind": "region",
  "id": "rl:place:handoff-H1",
  "frame": "map",
  "polygon": [["10","2"],["11","2"],["11","3"],["10","3"]],
  "unit": "m"
}
```

Places may also be **symbolic** (`"kind": "named_place"`), resolved by the robot that
executes. Two participants may use different frames; a shared frame or an explicit transform
must be established before geometry is compared, otherwise comparison fails (P4).

### 7.6 Observations

What a participant believes about the world, with its age, source and confidence.

```json
{
  "kind": "observation",
  "id": "rl:obs:amr-07:000193",
  "subject": "rl:thing:container-C42",
  "property": "located_in",
  "value": "rl:place:handoff-H1",
  "observed_at": "2026-10-19T10:14:03.120Z",
  "valid_for": { "value": "30", "unit": "s" },
  "confidence": "0.93",
  "source": { "participant": "rl:robot:amr-07", "sensor": "front_camera", "method": "fiducial" },
  "signature": { "alg": "ed25519", "key": "rl:robot:amr-07#k1", "value": "…" }
}
```

- An observation past `observed_at + valid_for` is **stale** and counts as unknown (plan
  safety semantics 1 and 2).
- An observation is signed by the participant that made it, so it can be forwarded without
  losing its origin.
- An observation received from another participant is **evidence about that participant's
  belief**, not a fact. Section 11.3 says when it may be relied on.

### 7.7 Goals

A desired state of the world, with success criteria and constraints. This is the main thing
A2R carries.

```json
{
  "kind": "goal",
  "id": "rl:goal:7f3c",
  "requested_by": "rl:agent:planner-1",
  "desired_state": [
    { "subject": "rl:thing:container-C42", "property": "located_in", "value": "rl:place:station-S3" }
  ],
  "success_criteria": [
    {
      "kind": "observed",
      "subject": "rl:thing:container-C42",
      "property": "located_in",
      "value": "rl:place:station-S3",
      "min_confidence": "0.9",
      "max_age": { "value": "10", "unit": "s" },
      "accepted_sources": ["any_participant_in_domain"]
    }
  ],
  "constraints": [
    { "kind": "deadline", "at": "2026-10-19T10:30:00Z" },
    { "kind": "max_speed", "limit": { "value": "0.8", "unit": "m/s" } },
    { "kind": "avoid_place", "place": "rl:place:aisle-4" }
  ],
  "priority": "normal"
}
```

A goal says **what** must be true and how that will be checked. It does not say how. Success
criteria are mandatory: a goal without them is rejected (P5). `accepted_sources` states whose
evidence the requester will accept (section 11.3).

### 7.8 Plans

How a participant intends to reach a goal. The first supported representation is the
existing task IR (6.3).

```json
{
  "kind": "plan",
  "plan_kind": "robolang.task-ir",
  "plan_kind_version": "1.0",
  "task": "deliver_container",
  "bindings": {
    "r": "rl:robot:amr-07",
    "box": "rl:thing:container-C42",
    "dst": "rl:place:station-S3"
  },
  "document": { "ir_schema": "robolang.task-ir", "ir_schema_version": "1.0", "language_version": "1.1", "tasks": ["…"] }
}
```

`bindings` supplies what IR schema 1.0 deliberately leaves out: which real robot, thing and
place each task parameter stands for. A plan is a proposal; an executing robot re-validates
it and may refuse it.

### 7.9 Commitments

An acknowledged agreement that a participant is responsible for something, under conditions,
for a bounded time.

```json
{
  "kind": "commitment",
  "id": "rl:commit:91aa",
  "debtor": "rl:robot:amr-07",
  "creditor": "rl:robot:arm-02",
  "obligation": { "goal": "rl:goal:7f3c-sub-1" },
  "conditions": [
    { "kind": "resource_held", "resource": "rl:place:handoff-H1" }
  ],
  "lease": { "granted_at": "2026-10-19T10:14:06Z", "expires_at": "2026-10-19T10:25:00Z", "renewal": 0 },
  "state": "active"
}
```

A commitment always has a lease. Section 11.1 defines what happens when it ends.

### 7.10 Outcomes

What actually happened, with evidence.

```json
{
  "kind": "outcome",
  "goal": "rl:goal:7f3c",
  "status": "succeeded",
  "verification": {
    "verified_by": "rl:robot:arm-02",
    "method": "criteria_evaluated",
    "evaluated_at": "2026-10-19T10:22:40Z",
    "criteria_results": [{ "criterion": 0, "met": true, "evidence": ["rl:obs:amr-07:000207"] }]
  },
  "reported_by": "rl:robot:arm-02",
  "reported_at": "2026-10-19T10:22:41Z"
}
```

`status` is one of `succeeded`, `failed`, `cancelled`, `timed_out`, `rejected` or
**`unknown`**. `unknown` is a first-class outcome: after a lost connection or a crash, a
robot must be able to say that it does not know whether the physical result was achieved,
instead of guessing either way. `succeeded` is only valid with a `verification` block
(section 11.3).

### 7.11 Provenance

Every message and every assertion carries its origin: participant, time, software and
versions, and, for AI-originated content, the provider and model identifier. Provenance is
evidence for audit, not a grant of trust.

## 8. Message envelope

All A2R and R2R messages share one envelope, extending RL-13C. It is a
`robolang.core.message` document.

```json
{
  "schema": "robolang.core.message",
  "schema_version": "0.1",
  "message_id": "01J9ZK8Q3D7X2M",
  "conversation_id": "rl:conv:7f3c",
  "sequence": 6,
  "in_reply_to": "01J9ZK7W0B1F9A",
  "profile": "r2r",
  "type": "offer_assistance",
  "sender": "rl:robot:amr-07",
  "recipient": "rl:robot:arm-02",
  "sent_at": "2026-10-19T10:14:05.004Z",
  "expires_at": "2026-10-19T10:14:35Z",
  "payload": { "…": "…" },
  "signature": { "alg": "ed25519", "key": "rl:robot:amr-07#k1", "value": "…" }
}
```

- `message_id` is unique per sender and makes handling idempotent (section 11.2).
- `sequence` increases by one per message a sender sends within a conversation.
- `expires_at` is mandatory and bounded (section 11.2).
- The signature covers every field except itself.
- Unknown schema versions, unknown `type`s and unexpected fields are rejected (plan security
  baseline).
- The envelope is identical over every binding (section 14).

## 9. A2R — AI agents and robots

### 9.1 Operations

| Type | From → to | Meaning |
|---|---|---|
| `discover` | agent → robot | Request identity, supported Core versions and capability descriptor |
| `observe` / `subscribe` | agent → robot | Request or subscribe to observations |
| `propose_goal` | agent → robot | Ask the robot to bring about a goal |
| `assessment` | robot → agent | Feasible, infeasible or feasible with changes, with reasons |
| `negotiate` | both ways | Clarify or counter-propose constraints, deadline or plan |
| `accept` / `reject` | robot → agent | Robot takes on, or declines, the goal |
| `authorization_request` | robot → its authority | Only when no standing authorization covers the goal (13.3) |
| `progress` | robot → agent | State changes and intermediate observations |
| `cancel` | agent → robot | Request cancellation; the robot acknowledges with the state actually reached |
| `outcome` | robot → agent | Final outcome with verification |

### 9.2 Goal lifecycle (at the robot)

```text
PROPOSED → ASSESSING → REJECTED
                     → NEGOTIATING → ASSESSING
                     → ACCEPTED → AUTHORIZING → DENIED
                                              → AUTHORIZED → RUNNING → SUCCEEDED (verified)
                                                                     → FAILED
                                                                     → CANCELLED
                                                                     → TIMED_OUT
                                                                     → UNKNOWN
```

- **Accepting is not authorizing.** A robot may accept a goal and still be denied.
- `AUTHORIZING` is a local decision. It may complete immediately under a standing
  authorization (13.3) or wait for an explicit grant.
- Terminal states are final. A new attempt is a new goal.
- `SUCCEEDED` requires every success criterion to be met by verified evidence (11.3).
  Otherwise the outcome is `FAILED` or `UNKNOWN`.
- `CANCELLED` and `TIMED_OUT` report the physical state reached. They never imply that motion
  stopped unless the backend confirmed it (plan safety semantics 4).

## 10. R2R — robots and robots

### 10.1 Operations

| Type | Meaning |
|---|---|
| `advertise` | Announce identity, supported Core versions and capability descriptor to peers |
| `request_assistance` | Ask a peer to take on a sub-goal |
| `offer_assistance` | Offer to take it on, with conditions and a proposed lease |
| `commit` / `decline` | Turn an offer into a commitment, or refuse it |
| `renew` / `release` | Extend a lease before it expires, or end a commitment or reservation early |
| `share_observation` | Share signed observations, with full provenance |
| `reserve` | Request a leased hold on a shared resource such as a handoff region |
| `coordinate` | Agree on phase changes in a joint activity (for example "ready to hand over") |
| `transfer_custody`, `custody_confirmed`, `custody_released` | Hand responsibility for a thing to a peer (10.4) |
| `custody_query`, `custody_state` | Reconcile custody after a lost acknowledgement or a partition (10.4) |
| `outcome` | Report the result of a commitment, with verification |

### 10.2 Commitments

`request_assistance → offer_assistance → commit` produces a commitment (7.9) held by both
sides. A commitment is bilateral and leased. Version 0 has no multi-party commitments and no
voting; a robot needing several helpers makes several bilateral commitments, each with its
own lease.

### 10.3 Resources

A shared physical resource (a region, a doorway, a charging dock) is held through a leased
`reserve`. A reservation without a granted lease is not a reservation. Lease rules (11.1)
apply unchanged. Who arbitrates a resource (a peer, a fixed owner or a fleet system) is part
of the deployment's configuration and is an open question for settings with more than two
robots (section 21).

### 10.4 Custody transfer

Custody is responsibility for a thing: knowing where it is and keeping it safe. Over an
unreliable network two robots cannot always know that they agree, so the rules separate three
things:

| Term | Meaning | Who can know it |
|---|---|---|
| **Designated responsibility** | Who is responsible according to the protocol's commit rule (rule 3 below) | Defined at every moment, but a participant may not be able to tell which side of the commit point the transfer is on |
| **Local custody state** | What a participant's own records say | Each participant, about itself. The two local states may disagree for a while |
| **Verified physical custody** | Where the thing actually is, as shown by a fresh observation (11.3) | Only a participant that can observe the thing now |

States, extending the RL-13D handoff states:

```text
giver:    HOLDING → OFFERED → AWAITING_CONFIRMATION → RELEASE_SENT → RELEASED     (release acknowledged)
                                                                   → IN_DOUBT     (no acknowledgement)
                                                                       → reconcile → RELEASED | HOLDING
receiver:           ACKNOWLEDGED → ACCEPTED → CONFIRMATION_SENT → HOLDING        (release durably accepted: commit point)
either side, before the commit point: REJECTED | TIMED_OUT → giver keeps designated responsibility
```

1. **Receiver confirms with its own evidence.** The receiver sends `custody_confirmed` only
   with a fresh observation of the thing made by its own sensors, meeting the criteria agreed
   for the handover (for example "C42 seated on my deck, confidence ≥ 0.9, age ≤ 2 s").
2. **Giver verifies before releasing.** The giver checks the receiver's evidence, and its own
   observation where it has one, and only then sends `custody_released`.
3. **Commit rule.** Designated responsibility passes to the receiver **at the moment the
   receiver durably accepts a valid `custody_released`**: it records the acceptance in durable
   state, so that it survives a crash, before acknowledging. Before that moment responsibility
   is the giver's. There is therefore exactly one designated responsible participant at every
   moment.
4. **The receiver always knows the designation.** It either has durably accepted the release
   or it has not. It acknowledges an accepted release; the acknowledgement is the giver's
   record.
5. **Only the giver can be in doubt.** A giver that sent `custody_released` and has no
   acknowledgement cannot tell whether the commit point was reached. Its local state is
   `IN_DOUBT`.
6. **The receiver proceeds after the commit point.** Once it has durably accepted the release,
   the receiver is the responsible participant. It may continue work involving the thing
   without waiting for the giver, but only:
   - under its own local authorization (13.2, 13.3);
   - after verifying the current physical state with a fresh observation of the thing (rule 9);
   - within its constraints and safety limits.
   The giver's doubt does not restrict the receiver, because the giver is required not to
   interfere (rule 7).
7. **The giver does not interfere while in doubt.** A giver in `IN_DOUBT` does not touch, move
   or re-grasp the thing, starts no new transfer of it, makes no commitment that depends on
   holding it and keeps clear of the receiver's handling of it. It stays in its locally defined
   safe state for the activity (11.1).
8. **The giver reconciles before acting as holder.** A giver in doubt sends `custody_query`.
   The receiver answers with a signed `custody_state` that names the release message it
   accepted, or states that it accepted none, together with a fresh observation of the thing
   where it can make one. The giver resolves to `RELEASED` or back to `HOLDING` from that
   answer. Until it has, the giver takes no action that depends on retaining custody.
9. **Verify physical custody before acting.** The responsible participant makes a fresh
   observation of the thing before moving it: the receiver after the commit point, or the
   giver after reconciling back to `HOLDING`. If it cannot, physical custody is `unknown` and
   it does not move the thing.
10. **Before the commit point, the receiver does not act as holder.** A receiver that has not
    accepted a release does not move the thing out of the handover position, does not hand it
    on and makes no commitment that depends on it. The giver remains responsible.
11. **No reconciliation before the lease expires.** A giver still in doubt, and a receiver that
    never accepted a release, stay in their safe states, report custody as `unknown` to their
    authorities and to the requester, and escalate under local policy (for example to a named
    human). Neither may resolve the doubt on its own by assuming the other has the thing. A
    receiver that did accept the release keeps designated responsibility (rule 13).
12. **Physical action and custody are separate.** A gripper opening, or a robot driving away,
    is a physical event. It does not by itself transfer custody, and custody records never
    claim a physical state that was not observed.

13. **Expiry does not reverse custody, and custody does not extend permission.** Expiry of a
    commitment does not reverse a completed custody transfer: the receiver retains designated
    responsibility for the thing. It must still follow its local expiry and safe-state policy
    (11.1), and any further work that required the expired commitment, such as continuing
    to transport the thing, must be separately authorized under its local policy (13.2,
    13.3) and, where it serves another participant, covered by a new commitment.

A message sent is never a handover (plan v1.3 exit gate).

## 11. Leases, replay and evidence

### 11.1 Leases and expiry

Commitments, reservations and standing authorizations (13.3) all use the same lease rules.

- A lease has an absolute `expires_at`. It is never open-ended.
- Either side may `renew` before expiry. A renewal is a new message with an increased
  `renewal` count; it must be accepted by the other side to take effect. A renewal received
  after `expires_at` (judged by the receiver's clock, 11.4) is rejected.
- **On expiry, each side acts on its own**, without waiting for the other:
  - the debtor stops starting new work under the commitment and brings any work in progress
    to a **locally defined safe state** for that activity;
  - a reservation holder must leave, or not enter, the reserved resource;
  - the creditor treats the obligation as unfulfilled.
- A lapsed commitment cannot be revived. Continuing requires a new commitment.
- Expiry ends the permission to work under the commitment, not responsibility for things
  already in a participant's custody. A completed custody transfer stays completed (10.4
  rule 13).
- When contact returns, each side reports the state it reached, using `unknown` where it does
  not know the physical result.
- Leases should be short relative to how long a robot can safely continue without contact.
  Choosing them is a deployment policy decision, informed by the clock tolerance (11.4).

### 11.2 Replay and duplicates

- **Duplicates.** A receiver keeps the `message_id`s it has processed, per sender, at least
  until each message's `expires_at` plus the clock tolerance. A repeated `message_id` is not
  processed again; the original reply is sent again.
- **Replay after the window.** A message whose `expires_at` has passed is rejected. Because
  the duplicate window lasts until expiry, an old message can be neither reprocessed nor
  replayed later.
- **Bounded lifetime.** A message whose `expires_at` is further than a configured maximum
  after its `sent_at`, or whose `sent_at` is further in the future than the clock tolerance,
  is rejected. This prevents a recorded message with a distant expiry from being replayed.
- **Ordering.** Within a conversation, a receiver processes messages from each sender in
  `sequence` order. A gap is held until the missing message arrives or expires; a message
  that would cause an impossible state transition is rejected.
- **Signatures.** A message whose signature fails, or whose key is unknown or revoked, is
  rejected before any of the rules above are applied to its content.

### 11.3 Evidence and verification

- **Evidence is observations.** Evidence for a criterion is one or more signed observations
  (7.6). A statement such as "done" or "arrived" is not evidence.
- **Freshness at evaluation.** An observation counts only if it is within both its own
  `valid_for` and the criterion's `max_age` at the time the criterion is evaluated.
- **Accepted sources.** A criterion states whose observations count (`accepted_sources`). The
  evaluating participant applies that and its own policy. A participant's own sensors are
  always an accepted source for that participant's decisions.
- **Who verifies.** The participant reporting an outcome evaluates the criteria and records
  how in the `verification` block. A participant **receiving** an outcome may re-evaluate
  the evidence, and must do so if its policy requires it. If re-evaluation fails, it treats
  the outcome as `unknown`, not `succeeded`.
- **Self-reported evidence is marked.** When the only evidence comes from the participant
  that performed the action, `verification.method` says so (`self_reported`). Policies may
  refuse to accept such outcomes for some goals.
- **Forwarded evidence keeps its origin.** An observation passed on by a third participant
  is still attributed to, and signed by, the participant that made it.
- **No success without verification.** An outcome of `succeeded` without a `verification`
  block is invalid and rejected.

### 11.4 Time

Leases, expiry, freshness and replay all depend on clocks. Each deployment states a clock
tolerance (the maximum assumed difference between participants' clocks). Every time
comparison applies it in the safe direction: a lease is treated as expiring early, an
observation as older, a message as less fresh. How clocks are synchronized is an open
question (section 21).

## 12. Failure model

| Situation | Required behavior |
|---|---|
| Duplicate message | Recognized by `message_id`; original reply repeated; no second effect (11.2) |
| Messages out of order | Processed in `sequence` order per sender; impossible transitions rejected (11.2) |
| Expired or replayed message | Rejected (11.2) |
| Invalid signature, unknown or revoked key | Rejected before content is considered (11.2) |
| Peer silent, lease running | Wait; start no new physical work that depends on the peer |
| Lease expired | Commitment or reservation lapses; each side moves to its local safe state and later reports what it reached (11.1) |
| Network partition during motion | The robot's own policy and safety systems govern. No remote party is assumed to be able to stop it |
| Custody transfer interrupted before the commit point | Giver keeps designated responsibility; the receiver does not act as holder; the giver reports physical custody `unknown` if it cannot observe the thing (10.4 rules 3 and 10) |
| Acknowledgement of `custody_released` lost | The receiver has durably accepted the release, is responsible and may proceed under its local authorization, a fresh observation and its constraints. The giver is `IN_DOUBT`, does not interfere and reconciles with `custody_query` before any action that depends on retaining custody (10.4 rules 5 to 9) |
| Partition outlasts the lease during a custody transfer | A giver in doubt and a receiver without an accepted release stay in safe states, report custody `unknown` and escalate; neither assumes the other has the thing. A receiver that accepted the release keeps designated responsibility, follows its local expiry and safe-state policy, and needs separate authorization for further work under the expired commitment (10.4 rules 11 and 13) |
| Stale observation | Treated as unknown; criteria depending on it are not met (11.3) |
| Conflicting observations | Not resolved by voting. For decisions about its own motion a robot relies on its own sensing; otherwise it reports the conflict and the affected outcome is `unknown` |
| Backend reports success without evidence | Outcome is `unknown`, not `succeeded` |
| Outcome fails re-verification by the receiver | Receiver treats it as `unknown` (11.3) |
| Crash and restart | Goals, commitments, custody and leases are restored from durable state and re-checked against the clock, or reported as `unknown`; never silently dropped or silently resumed |
| Unsupported Core version | Conversation does not start (6.2) |

## 13. Trust, authority and safety

### 13.1 Identity

Each participant has a key pair. Messages, observations and capability descriptors are signed.
Identity is bound to keys through the deployment's configuration or through a binding's
mechanism (for example signed A2A Agent Cards). Unsigned messages may be accepted only for
read-only operations, if the deployment allows it.

### 13.2 Authorization is local

Authentication says who sent a message. **Authorization decides whether the robot acts, and
it is decided on the robot, under its local policy.**

- Every robot belongs to an `authority_domain`. The domain's policy is installed on the robot
  and evaluated there.
- A policy decides, per goal, whether to act and which constraints apply. It may require a
  grant from a named human, from a policy service, or both.
- The requesting participant is **never** its own authority. An AI agent cannot authorize its
  own goal, and a robot asking a peer for help cannot authorize the peer's actions.
- A grant is evaluated on the robot. A grant arriving from elsewhere is input to the local
  policy, not an override of it.
- A physical-execution policy is disabled by default, consistent with the plan's decision A5.

### 13.3 Standing authorization for autonomous action

Robots must be able to act on their own, including in R2R exchanges with no human in the
loop. That is done with **standing authorizations**: grants issued in advance by the robot's
authority for a bounded class of goals.

```json
{
  "kind": "standing_authorization",
  "id": "rl:auth:wn-handoff-2026-10-19",
  "issued_by": "rl:human:shift-lead-3",
  "authority_domain": "rl:domain:warehouse-north",
  "grantee": "rl:robot:amr-07",
  "scope": {
    "profiles": ["r2r"],
    "goal_classes": ["transport_payload", "custody_transfer"],
    "places": ["rl:place:warehouse-north-floor"],
    "counterparts": ["rl:domain:warehouse-north"],
    "max_mass": { "value": "30", "unit": "kg" }
  },
  "constraints": [
    { "kind": "max_speed", "limit": { "value": "1.0", "unit": "m/s" } }
  ],
  "lease": { "granted_at": "2026-10-19T06:00:00Z", "expires_at": "2026-10-19T18:00:00Z", "renewal": 0 },
  "mode": "simulation",
  "signature": { "alg": "ed25519", "key": "rl:human:shift-lead-3#k1", "value": "…" }
}
```

- **Bounded.** A standing authorization states its scope (profiles, goal classes, places,
  counterparts, limits) and constraints. A goal outside the scope needs an explicit grant.
- **Leased and revocable.** It has a lease (11.1) and can be revoked at any time. Revocation
  stops new work under it; work in progress goes to its local safe state.
- **Evaluated per goal.** Each goal is still checked against the policy and the standing
  authorization at the moment `AUTHORIZING` is entered. A standing authorization does not
  bypass capability checks, freshness rules or constraints.
- **Not transferable.** Only its grantee may use it. It cannot be delegated to a peer or to a
  requesting agent.
- **Recorded.** Every decision taken under it is logged with its identifier, so autonomous
  actions remain attributable to the authority that permitted them.
- **Mode is explicit.** `mode` is `simulation` or `physical`. Physical mode requires the
  separate enablement process of the plan's decision A5; nothing in this RFC enables it.

### 13.4 AI-originated content

Everything produced by a model (goals, plans, observations, negotiation) is untrusted input.
It passes the same schema validation, plan validation and policy as any other input, with
provenance recording the provider and model (plan RL-20C). No provider receives special
privileges. A robot that runs a model internally applies the same rule to its own model's
output.

### 13.5 Peer-originated observations

A peer's observation may inform planning and may serve as evidence where a criterion accepts
it (11.3). It must not be the only basis for a decision that affects the safety of a robot's
own motion; for those, the robot relies on its own sensing or on infrastructure it trusts
under its policy. This limits the damage from a faulty, compromised or spoofed peer.

### 13.6 Safety

RoboLang constraints (`max_speed`, `avoid_place`) are requirements passed to the execution
backend and checked by the executor. They are not safety functions. Safety-rated stops,
monitoring and risk assessment are outside RoboLang and must keep working with every
RoboLang component, AI model and network unavailable.

## 14. Bindings

A binding carries the Core envelope unchanged and maps identities. It may not add or drop
meaning. **No binding is mandatory**: a deployment uses the bindings it needs, and Core
semantics do not depend on any of them.

| Binding | Role | Notes |
|---|---|---|
| **A2A** | First-class interoperability target, not a required transport | A2R and R2R as an A2A extension; the Agent Card advertises RoboLang support, Core versions and the capability descriptor. The way RoboLang participants reach the wider agent ecosystem. Designed in from the start and kept conformant, but a deployment without A2A loses nothing in Core meaning |
| **Local peer-to-peer** | Required to satisfy P6 | A minimal transport for robots with no shared middleware and no cloud. To be selected in the proof |
| **ROS 2** | Execution backend; optionally a local transport | Plans run through ROS 2 actions where a robot uses ROS 2. As a transport, DDS QoS (reliability, deadline, liveliness) can carry the envelope on a local network |
| **MCP** | Optional, for AI model clients | A robot or gateway may expose A2R operations as MCP tools. Calling a tool is a `propose_goal`; authorization is unchanged |

## 15. Related work

Checked on 2026-10-19. Status notes are as found then.

| Work | Status | Relationship |
|---|---|---|
| A2A (Linux Foundation) | v1.0 in 2026; signed Agent Cards, extensions, several transport bindings | Agent-to-agent layer. First-class binding (14) |
| Model Context Protocol | Revision 2026-07-28; sessionless, formal extensions framework, Tasks | AI model to tool layer. Optional binding (14) |
| 3GPP TR 22.916, Release 19 | Study on Network of Service Robots with Ambient Intelligence; robot-to-robot as one scenario | Network-level requirements, not task semantics |
| OMG RoSO | 1.0 formal (April 2026); **1.1 formal, adopted June 2026** | Vocabulary for robotic functions and services; candidate for capability alignment (21) |
| IEEE 1872 family; P1872.3 | 1872-2015 and 1872.2-2021 published; P1872.3 (multi-robot reasoning) is an active project | Ontology alignment candidate (21) |
| VDA 5050 | Fleet control of mobile robots | Possible binding for mobile fleets; centralized model, different scope |
| Robot Context Protocol (arXiv 2506.11650) | 2025 preprint; read/write/execute/subscribe over HTTP/WebSocket, MCP and A2A adapters | Closest prior work for A2R; operation-level rather than goal- and evidence-level |
| `r2r-protocol` (Tech-Parivartan) | Open-source project | Uses the name "R2R" |
| `r2r_spl` (ROS Sports) | ROS 2 package for RoboCup SPL team messages | Uses the name "R2R" |

**Naming.** "R2R" is already used by at least two projects and by 3GPP for a scenario. This
RFC therefore uses A2R and R2R only as names of **RoboLang interaction profiles**
("RoboLang R2R"), not as standalone standard names (decision D3).

## 16. Compatibility with RoboLang 1.1

- RoboLang 1.1, its grammar, IR schema 1.0 and the `validate` and `inspect` commands are
  unchanged.
- Task IR 1.0 is the first supported plan representation (6.3). Validated 1.1 tasks can be
  carried inside A2R without change. `.rob` remains an authoring format (6.4).
- The 1.1 `communicate` statement stays a free-text step inside a plan. It is not R2R.
  Structured messaging is defined in the Core, not by changing that statement.
- New language constructs (for example goals, outcomes or skill calls in `.rob` text) would
  come in a later language version, derived from the Core schemas, and follow the existing
  compatibility policy.
- The deferred semantics in [`robolang-v1.1.md`](../robolang-v1.1.md) section 9 (parameter
  binding, target resolution, condition evaluation) are addressed by 7.5, 7.6 and 7.8.

## 17. Consistency with the development plan

Checked against [`docs/DEVELOPMENT_PLAN.md`](../../docs/DEVELOPMENT_PLAN.md) as of this
revision. The plan is **not** changed by this RFC. With decisions D1–D9 approved, these are the places
the plan needs to change, in a separate, owner-approved update:

| Plan location | Plan says | This RFC (approved) | Related decision |
|---|---|---|---|
| Mission statement | A task specification and coordination language that delegates execution to ROS 2 controllers | An open, AI-native interoperability platform for embodied intelligence (section 1.1), with execution through interchangeable backends | D1 |
| Decision A6 | Python 3.11+ for language tooling | Compatible: Python remains the reference implementation language, but P10 requires the Core to be implementable without it | D9 |
| Section 1, milestone order | v1.2 execution, then v1.3 interoperability, then v2.0 AI-native | Core and the A2R/R2R proof next; executor work re-framed as the execution side of A2R | D8 |
| Section 1, non-goals | "Distributed consensus" | Compatible: bilateral commitments only, no consensus | D5 |
| Section 1, non-goals | "Inventing an emergent robot language" | Compatible if read as a language robots develop themselves; A2R and R2R are designed and specified. The wording should say so | D7 |
| Section 3, diagram and decision A4 | AI proposes `.rob` source | AI may propose Core JSON directly; `.rob` is one authoring format | D2 |
| Decision A4 | The compiler, validator and executor do not invoke an LLM at runtime | Compatible. A robot may run a model for its own decisions (5), but RoboLang components stay deterministic and model output is untrusted (13.4) | — |
| Decision A5 | Physical execution disabled by default | Compatible and restated: standing authorizations carry an explicit `mode` (13.3) | D6 |
| RL-13C, RL-13D | Typed envelope; handoff `OFFERED → ACKNOWLEDGED → ACCEPTED → COMPLETED` | Superseded by the Core envelope (8) and custody rules (10.4) | D8 |
| RL-20B | Generation prompted with the `.rob` language spec | Would extend to generating Core documents | D2 |
| Section 10, first slice | Describes RL-11A as the first authorized slice | Historical; not affected | — |

No other conflicts were found. The plan's safety semantics (section 5), security baseline
and risk register are consistent with this RFC and are referenced by it.

## 18. Reference scenario

**An AI agent asks for a container to be delivered. The robot it asks cannot carry it alone.**

Participants: `planner-1` (agent), `arm-02` (fixed manipulator, can grasp up to 30 kg, cannot
travel), `amr-07` (mobile robot, can carry 40 kg, cannot grasp). Both robots hold standing
authorizations for R2R transport and custody transfer in their domain, in simulation mode.

| # | Profile | From → to | Type | Content |
|---|---|---|---|---|
| 1 | A2R | planner-1 → arm-02 | `propose_goal` | C42 located in S3, verified by observation, deadline 10:30 |
| 2 | A2R | arm-02 → planner-1 | `assessment` | Feasible with assistance: arm-02 cannot reach S3 |
| 3 | R2R | arm-02 → amr-07 | `request_assistance` | Sub-goal: C42 from H1 to S3, 25 kg |
| 4 | R2R | amr-07 → arm-02 | `offer_assistance` | Accepts subject to holding H1 from 10:15 to 10:20; proposed lease to 10:25 |
| 5 | R2R | arm-02 → amr-07 | `commit` | Commitment `91aa`, lease to 10:25 |
| 6 | R2R | amr-07 → arm-02 | `reserve` granted | H1 leased to amr-07 |
| 7 | A2R | arm-02 → planner-1 | `accept` | Goal accepted |
| 8 | – | local | authorization | The planner's goal is outside arm-02's standing authorization, so arm-02 requests an explicit grant from its authority. amr-07's sub-goal is inside its standing authorization and is authorized locally |
| 9 | R2R | arm-02 ↔ amr-07 | `coordinate` | Both ready at H1 |
| 10 | R2R | arm-02 → amr-07 | `transfer_custody` | C42 placed on amr-07's deck |
| 11 | R2R | amr-07 → arm-02 | `custody_confirmed` | With amr-07's own observation: C42 on deck, age 1 s |
| 12 | R2R | arm-02 → amr-07 | `custody_released` | After checking the evidence. amr-07 durably accepts it (commit point) and acknowledges |
| 13 | R2R | amr-07 → arm-02 | `outcome` | Sub-goal succeeded; evidence: amr-07's observation of C42 in S3 |
| 14 | A2R | arm-02 → planner-1 | `outcome` | Goal `succeeded`; verification by arm-02, evidence attached, marked as observed by amr-07 |

**Failure variants the proof must cover:**

| Variant | Expected result |
|---|---|
| amr-07 declines | arm-02 reports `assessment: infeasible`; no motion |
| Step 5 delivered twice | Second copy recognized; one commitment |
| Step 5 replayed after its expiry | Rejected |
| amr-07 goes silent after step 6 | Lease lapses at 10:25; arm-02 keeps custody, moves to its safe state, reports `failed` or `unknown` |
| `custody_confirmed` arrives with a 40 s old observation | Stale; arm-02 does not release; custody stays with arm-02 |
| Connection lost between steps 11 and 12 | Commit point not reached: arm-02 keeps designated responsibility; amr-07 does not move C42 out of the handover position. The transfer continues only if arm-02 sends a release after contact returns; otherwise 10.4 rule 11 applies at lease expiry |
| Acknowledgement of step 12 lost | amr-07 has durably accepted the release and is responsible: it verifies C42 on its deck and continues to S3 under its standing authorization. arm-02 is `IN_DOUBT`, does not touch C42 or the handover region, and resolves to `RELEASED` with `custody_query` before taking on other work involving C42 |
| amr-07 reports "arrived" without an observation of C42 in S3 | Sub-goal outcome `unknown`; goal not `succeeded` |
| planner-1 re-verifies the outcome and the evidence is too old | planner-1 treats the goal as `unknown` |
| planner-1 tries to authorize the goal itself | Rejected: the requester is not an authority |
| amr-07 tries to use its standing authorization for a 35 kg payload | Outside scope (max 30 kg); explicit grant required |
| Unsigned or wrongly signed `commit` | Rejected |
| arm-02's descriptor lists no grasp capability | Assessment fails before any commitment |

## 19. Next steps

| Step | Deliverable | Boundaries |
|---|---|---|
| **RL-ARCH-01** (this RFC) | Owner decisions on section 20: approved 2026-10-19, recorded in decision record 0002 | Documentation only |
| **RL-ARCH-02 — executable proof** | Draft Core JSON Schemas (message, capability descriptor). An in-process Python simulation of section 18 with one agent stub and two robot stubs, each with a minimal executor, local policy and a simulated world. Tests for every failure variant in section 18 | No ROS 2, no network transport, no AI model calls, no hardware. Simulation only. Does not change the 1.1 parser or IR. **Not started.** Approved as the next planned milestone (D8); implementation requires separate explicit owner authorization |
| **Independent-implementation gate** | A participant written independently, in a different codebase, interoperates with the reference using only the published schemas and this specification (gate 2, section 1.4) | Later; not yet scoped or approved. Requires stable Core schemas first |
| **Re-scope RL-12 and RL-13** | Revised development plan (section 17) | Separate change; owner approval required |
| **Later** | A2A binding, local transport, ROS 2 backend, ontology alignment, portable skills, conformance suite, language constructs | Each its own milestone |

Not proposed now: a RoboLang distribution, package manager or skill marketplace, a runtime
for every manufacturer, or a robotics operating system. Ecosystem components are expected to
grow around a stable, independently implementable Core, not ahead of it.

**Acceptance for RL-ARCH-02 (proposed):** the happy path of section 18 ends in a verified
`succeeded` outcome; every failure variant ends in the stated result with no simulated
motion where none is allowed; every run produces a reconstructable message and event log.

The developer-preview pilot of RoboLang 1.1 is independent of this RFC and stays the
immediate product-validation step.

## 20. Owner decisions

All nine were **approved by the owner on 2026-10-19**, with qualifications on D5 and D8. They
are recorded in
[decision record 0002](../decisions/0002-ai-native-architecture.md).

| # | Decision | Content | Status |
|---|---|---|---|
| D1 | Revise the mission statement of the development plan | From "task specification language that delegates execution to ROS 2" to the mission in section 1.1: "an open, AI-native interoperability platform for embodied intelligence", supporting a vendor-neutral ecosystem of interoperable implementations | APPROVED — 2026-10-19 |
| D2 | Core schemas are the contract | Versioned Core JSON schemas define interoperability; task IR 1.0 is one supported plan representation; `.rob` is an authoring format | APPROVED — 2026-10-19 |
| D3 | Naming | Use "RoboLang A2R" and "RoboLang R2R" as profile names; do not present "R2R" as a standalone standard | APPROVED — 2026-10-19 |
| D4 | A2A | First-class interoperability target from the start, implemented after the proof; not a mandatory transport | APPROVED — 2026-10-19 |
| D5 | Commitments | Bilateral and leased only in version 0; no multi-party consensus | APPROVED — 2026-10-19, for protocol version 0 only. Future multi-party coordination remains open |
| D6 | Authority | Adopt section 13: authorization is local; a requester is never its own authority; autonomous action through bounded, leased, revocable standing authorizations | APPROVED — 2026-10-19 |
| D7 | Non-goal wording in the plan | Keep "distributed consensus" as a non-goal. Clarify that "inventing an emergent robot language" means a language robots develop themselves, which A2R and R2R are not | APPROVED — 2026-10-19 |
| D8 | Next milestone | Approve RL-ARCH-02 as defined in section 19, after or in parallel with the pilot | APPROVED — 2026-10-19 as the next planned milestone. Implementation requires separate explicit owner authorization |
| D9 | Ecosystem before implementation | Adopt principle P10 and the two interoperability gates of section 1.4: the Core must be implementable from the specification alone, and the ambition is measured by an independent implementation interoperating with the reference (gate 2) | APPROVED — 2026-10-19 |

## 21. Open questions

1. **Ontology alignment.** Adopt IEEE 1872 / OMG RoSO terms for capabilities and things, map
   to them, or define RoboLang's own vocabulary with a published mapping?
2. **World model depth.** How much geometry and state belongs in the Core, and how much stays
   private to each robot?
3. **Time.** What clock synchronization is assumed, what clock tolerance is typical, and how
   is it configured and expressed (11.4)?
4. **Evidence formats.** What counts as evidence for common criteria, and can evidence refer
   to sensor data without moving large payloads?
5. **Key management.** How keys are issued, rotated and revoked in a deployment without a
   cloud, and how revocation reaches robots that are offline.
6. **Resource arbitration.** Who arbitrates a shared resource when more than two robots are
   involved?
7. **Local transport.** Which transport to use for P6 in the proof and beyond.
8. **Text syntax.** Whether, and when, goals and outcomes get `.rob` syntax.
9. **Learned policies.** How a learned or vision-language-action policy is described as a
   capability, given that its behavior cannot be enumerated.
10. **Safe states.** How a robot's locally defined safe state for an activity (11.1) is
    declared, and whether peers need to know it.
11. **Policy language.** How local policies and standing authorizations are expressed and
    distributed, and whether an existing policy language is reused.
12. **Skill model.** How a portable skill declares its requirements, expected outcomes and
    test evidence, and how a robot demonstrates that it satisfies them.
13. **Conformance.** How conformance is tested and claimed: test suites, profiles (for
    example A2R-only, R2R-only) and what an implementation must pass to call itself
    conforming.
14. **Governance.** Where the specifications are maintained, under what licence, how changes
    are proposed and decided, and whether and when to move to vendor-neutral governance.

## References

- [A2A protocol](https://a2a-protocol.org/)
- [Model Context Protocol, 2026-07-28 changes (Agentic AI Foundation)](https://aaif.io/blog/mcp-2026-07-28-whats-changing-and-how-to-migrate)
- [3GPP TR 22.916](https://www.3gpp.org/DynaReport/22916.htm)
- [OMG Robotic Service Ontology](https://www.omg.org/spec/ROSO)
- [IEEE P1872.3](https://standards.ieee.org/ieee/1872.3/11037)
- [Robot Context Protocol, arXiv 2506.11650](https://arxiv.org/abs/2506.11650)
- [JSON Schema draft 2020-12](https://json-schema.org/draft/2020-12)
- [ROS REP-103: Standard units and coordinate conventions](https://www.ros.org/reps/rep-0103.html)
- [ROS REP-105: Coordinate frames for mobile platforms](https://www.ros.org/reps/rep-0105.html)
- [RoboLang development plan](../../docs/DEVELOPMENT_PLAN.md)
- [RoboLang 1.1 specification](../robolang-v1.1.md)
- [RoboLang IR schema 1.0](../ir-schema-v1.0.md)
