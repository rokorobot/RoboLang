# RFC-001 — RoboLang AI-native semantic and communication architecture

**Status:** Draft for owner review. Nothing in this document is approved.
**Date:** 2026-10-19
**Milestone:** RL-ARCH-01
**Changes to RoboLang 1.1:** None. The language, the grammar, IR schema 1.0 and the
`robolang` command are unchanged by this RFC.

> This RFC proposes a direction and the decisions needed to follow it. It defines no
> implemented behavior. All message formats below are **proposals for discussion**, not a
> published protocol, and nothing here describes a capability RoboLang has today.

---

## 1. Summary

RoboLang 1.1 is a validated task-description language for a single robot: a typed task is
written, checked and turned into a versioned intermediate representation (IR). It does not
execute anything and it does not communicate.

This RFC proposes that RoboLang's long-term identity is a **shared semantic model for
intelligent machines working in the physical world**, used by two interaction profiles:

| Profile | Between | Purpose |
|---|---|---|
| **A2R** | AI agents and robots | An agent proposes a physical outcome; a robot assesses, negotiates, executes under its own authority and reports evidence |
| **R2R** | Robots and robots | Embodied participants request help, offer it, commit to responsibilities, share resources and hand over custody |

A2R and R2R are two profiles of **one** model: the same definitions of goals, capabilities,
places, observations, commitments and outcomes. Agent-to-agent communication is already
covered by A2A, which RoboLang binds to rather than duplicates.

The central proposal is that **the versioned, schema-defined IR is the interoperability
contract**, and `.rob` text is one way of authoring it. RoboLang 1.1 task IR becomes one kind
of plan carried inside the new messages, so nothing built so far is discarded.

The RFC asks the owner for eight decisions (section 17) and proposes one small executable
proof (section 16) before any protocol is formalized.

## 2. Why this RFC, and how it relates to the existing plan

The owner's direction (2026-10-18/19) is that RoboLang should be future-first and centered
on communication between AI and robots and between robots, implementing existing systems such
as ROS 2 where useful but not letting their abstractions define the language.

Much of what this needs is already in [`docs/DEVELOPMENT_PLAN.md`](../../docs/DEVELOPMENT_PLAN.md),
scheduled late and framed around a single-robot executor:

| Existing plan item | Concept in this RFC |
|---|---|
| RL-12A executor states `CREATED → VALIDATED → AUTHORIZED → RUNNING → …` | Goal lifecycle (section 8.2) |
| RL-12B evaluated preconditions, freshness, fail-closed | Observations with time and uncertainty (6.6); failure model (10) |
| RL-12C capabilities registry, target resolution | Capability descriptors (6.3); places with frames (6.5) |
| RL-13A versioned capability descriptor schema | Capability descriptors (6.3) |
| RL-13C typed envelope: sender, recipient, correlation ID, expiry | Message envelope (7) |
| RL-13D handoff `OFFERED → ACKNOWLEDGED → ACCEPTED → COMPLETED` | R2R custody transfer (9.4) |
| RL-20C provenance, review gate, audit trail | Trust, authority and provenance (11) |
| Mission: "delegate execution to established ROS 2 controllers" | Execution backends behind bindings (12) |

What changes is the **order and the center of gravity**:

- Communication and the shared semantic model move from v1.3/v2.0 to the next milestone.
- A single-robot executor is still needed (a robot cannot report a verified outcome without
  one) but it is built as the execution side of A2R, not as the end goal.
- ROS 2, MoveIt and vendor SDKs become execution backends behind bindings, chosen per robot,
  instead of the model the language is shaped after.

## 3. Scope and non-goals

**In scope:** the semantic model, the A2R and R2R interaction profiles, lifecycle and failure
semantics, trust and authority rules, and the boundaries to bindings and execution backends.

**Not in scope, now or later:**

- Real-time or closed-loop control. A2R and R2R carry goals, commitments and evidence, not
  servo set-points or control loops.
- Safety-rated functions. Emergency stops, protective stops, speed and separation monitoring
  and risk assessment stay in independent, safety-rated systems. Nothing in RoboLang replaces
  them.
- Motion planning, kinematics, perception and learned policies. These are execution
  backends.
- A new network stack where existing transports suffice.
- Group consensus among many robots. Version 0 defines **bilateral** commitments only
  (section 9). Multi-party coordination is a later question.
- Any claim to be the first robot communication protocol. Related work exists (section 13).

## 4. Principles

| # | Principle | Consequence |
|---|---|---|
| P1 | **Semantics before syntax** | Concepts are defined as data with schemas first. Text syntax for them comes later, if at all |
| P2 | **The IR is the contract** | Interoperability is defined on versioned JSON documents. `.rob` text is one authoring surface |
| P3 | **A message never grants authority** | No message, however well-formed or well-signed, can by itself make a robot move. The robot decides under its local policy (section 11) |
| P4 | **Fail closed** | Unknown, missing, stale, unverifiable or contradictory information blocks the affected action. Inherited from the plan's decision A2 |
| P5 | **Outcomes, not dispatch** | "Message sent" and "command accepted" are never success. Success is a verified outcome with evidence |
| P6 | **Local first** | Two robots must be able to cooperate with no cloud and no AI service reachable. Loss of connectivity has defined behavior |
| P7 | **Independence** | No dependency on one AI provider, robot vendor, operating system, middleware or transport |
| P8 | **Provenance everywhere** | Every assertion carries who made it, when, from what source and how confident it is |
| P9 | **Integrate, do not let integration define** | ROS 2, MoveIt, A2A, MCP and vendor SDKs are used through bindings. If an existing system's abstraction conflicts with the model, the model wins and the binding adapts |

## 5. Layers

```text
  AI agents, planners, embodied models, humans
                    |
                    |  A2R
                    v
  +-----------------------------------------------+
  | RoboLang Core semantic model                  |      R2R
  | participants, capabilities, things, places,   | <---------> other robots
  | observations, goals, plans, commitments,      |
  | outcomes, provenance                          |
  +-----------------------------------------------+
                    |
          bindings (transport and ecosystem)
          A2A  |  MCP  |  ROS 2  |  local peer-to-peer
                    |
          execution backends (per robot)
          ROS 2 actions, MoveIt, Nav2, vendor SDKs, learned policies, simulator
                    |
          independent controllers and safety-rated systems
```

- The **Core** is transport-free and backend-free.
- **Bindings** carry Core messages over a transport and map identities. They add no meaning.
- **Execution backends** turn an authorized plan into motion on one specific robot and report
  what happened. They are chosen per robot and are invisible to the other side of A2R or R2R.

**A2R or R2R is decided by the interaction, not by the endpoint.** A robot may run an LLM
internally. When a participant asks a robot to bring about a physical outcome that the
participant has no authority to carry out itself, that is A2R. When embodied participants
coordinate their own physical actions, resources or custody, that is R2R, whatever software
they run.

## 6. Core semantic model

Every example below is a **proposed** JSON shape. Field names follow the IR's conventions:
snake_case, a `kind` field first where an object has variants, and quantities as exact
decimal strings with an explicit unit (IR schema 1.0, section 3.5). Units are SI. Coordinate
frame and axis conventions follow ROS REP-103 and REP-105 unless a robot's descriptor states
otherwise, and a frame is always named, never implied.

### 6.1 Identifiers and references

Every entity has a URI-like identifier that is unique within a deployment, for example
`rl:robot:amr-07`, `rl:agent:planner-1`, `rl:thing:container-C42`, `rl:place:station-S3`.
Identifiers are opaque: no meaning may be derived from their spelling.

### 6.2 Participants

A participant is anything that sends or receives RoboLang messages.

```json
{
  "kind": "robot",
  "id": "rl:robot:amr-07",
  "display_name": "AMR 07",
  "public_key": "ed25519:…",
  "capability_descriptor": "rl:capdesc:amr-07@3",
  "authority_domain": "rl:domain:warehouse-north"
}
```

`kind` is `robot`, `agent` or `human_interface`. `authority_domain` names whose local policy
governs the participant (section 11).

### 6.3 Capability descriptors

What a robot can do, what it cannot, and under what limits. This is RL-13A, generalized.

```json
{
  "schema": "robolang.capability-descriptor",
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
  "valid_until": "2026-10-19T18:00:00Z"
}
```

- A capability is **declared**, not proven. Declarations are inputs to assessment, never a
  guarantee (P3, P4).
- `unsupported` is explicit, so "not listed" and "known not to work" are different.
- Descriptors expire. A stale descriptor is treated as absent.
- Alignment with existing ontologies (IEEE 1872 family, OMG RoSO) is an open question
  (section 18).

### 6.4 Things

Physical objects a task refers to.

```json
{
  "kind": "thing",
  "id": "rl:thing:container-C42",
  "class": "container",
  "mass": { "value": "25", "unit": "kg", "uncertainty": "2" }
}
```

### 6.5 Places

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

### 6.6 Observations

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
  "source": { "participant": "rl:robot:amr-07", "sensor": "front_camera", "method": "fiducial" }
}
```

- An observation past `observed_at + valid_for` is **stale** and counts as unknown (plan
  safety semantics 1 and 2).
- An observation received from another participant is **evidence about that participant's
  belief**, not a fact. A robot may use it for planning, but never as the only basis for a
  safety-relevant decision about its own motion (section 11.4).

### 6.7 Goals

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
      "max_age": { "value": "10", "unit": "s" }
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
criteria are mandatory: a goal without them is rejected (P5).

### 6.8 Plans

How a participant intends to reach a goal. The Core admits several plan kinds; the first is
the existing one.

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
it (IR schema 1.0, section 2) and may refuse it.

Later plan kinds may include behavior trees or calls to a robot's own skills. All of them go
through the same assessment and authorization.

### 6.9 Commitments

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
  "lease_until": "2026-10-19T10:25:00Z",
  "state": "active"
}
```

A commitment always has a lease. If it is not renewed or completed before `lease_until`, it
lapses (section 10).

### 6.10 Outcomes

What actually happened, with evidence.

```json
{
  "kind": "outcome",
  "goal": "rl:goal:7f3c",
  "status": "succeeded",
  "verified": true,
  "evidence": ["rl:obs:arm-02:000552"],
  "criteria_results": [{ "criterion": 0, "met": true }],
  "reported_by": "rl:robot:amr-07",
  "reported_at": "2026-10-19T10:22:41Z"
}
```

`status` is one of `succeeded`, `failed`, `cancelled`, `timed_out`, `rejected` or
**`unknown`**. `unknown` is a first-class outcome: after a lost connection or a crash, a robot
must be able to say that it does not know whether the physical result was achieved, instead
of guessing either way.

### 6.11 Provenance

Every message and every assertion carries its origin: participant, time, software and
versions, and, for AI-originated content, the provider and model identifier. Provenance is
evidence for audit, not a grant of trust.

## 7. Message envelope

All A2R and R2R messages share one envelope, extending RL-13C.

```json
{
  "rl_message": "0.1",
  "message_id": "01J9ZK8Q3D7X2M",
  "conversation_id": "rl:conv:7f3c",
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

- `message_id` makes handling **idempotent**: a duplicate is recognized and answered with the
  original reply, never processed twice.
- `expires_at` is mandatory. An expired message is discarded and, where a reply is expected,
  answered with `expired`.
- Unknown `rl_message` versions, unknown `type`s and unexpected fields are rejected (plan
  security baseline).
- The envelope is the same over every binding (section 12).

## 8. A2R — AI agents and robots

### 8.1 Operations

| Type | From → to | Meaning |
|---|---|---|
| `discover` | agent → robot | Request identity and capability descriptor |
| `observe` / `subscribe` | agent → robot | Request or subscribe to observations |
| `propose_goal` | agent → robot | Ask the robot to bring about a goal |
| `assessment` | robot → agent | Feasible, infeasible or feasible with changes, with reasons |
| `negotiate` | both ways | Clarify or counter-propose constraints, deadline or plan |
| `accept` / `reject` | robot → agent | Robot takes on, or declines, the goal |
| `authorization_request` | robot → its authority | Robot asks its configured authority (section 11.2) |
| `progress` | robot → agent | State changes and intermediate observations |
| `cancel` | agent → robot | Request cancellation; the robot acknowledges with the state actually reached |
| `outcome` | robot → agent | Final outcome with evidence |

### 8.2 Goal lifecycle (at the robot)

```text
PROPOSED → ASSESSING → REJECTED
                     → NEGOTIATING → ASSESSING
                     → ACCEPTED → AWAITING_AUTHORIZATION → DENIED
                                                         → AUTHORIZED → RUNNING → SUCCEEDED (verified)
                                                                                → FAILED
                                                                                → CANCELLED
                                                                                → TIMED_OUT
                                                                                → UNKNOWN
```

- **Accepting is not authorizing.** A robot may accept a goal and still be denied by its
  authority.
- Terminal states are final. A new attempt is a new goal.
- `SUCCEEDED` requires every success criterion to be met by fresh evidence. Otherwise the
  outcome is `FAILED` or `UNKNOWN`.
- `CANCELLED` and `TIMED_OUT` report the physical state reached. They never imply that motion
  stopped unless the backend confirmed it (plan safety semantics 4).

## 9. R2R — robots and robots

### 9.1 Operations

| Type | Meaning |
|---|---|
| `advertise` | Announce identity and capability descriptor to peers |
| `request_assistance` | Ask a peer to take on a sub-goal |
| `offer_assistance` | Offer to take it on, with conditions and a lease |
| `commit` / `decline` | Turn an offer into a commitment, or refuse it |
| `share_observation` | Share observations, with full provenance |
| `reserve` / `release` | Lease a shared resource such as a handoff region |
| `coordinate` | Agree on phase changes in a joint activity (for example "ready to hand over") |
| `transfer_custody` | Hand responsibility for a thing to a peer (9.4) |
| `outcome` | Report the result of a commitment, with evidence |

### 9.2 Commitments

`request_assistance → offer_assistance → commit` produces a commitment (6.9) held by both
sides. A commitment is bilateral and leased. Version 0 has no multi-party commitments and no
voting; a robot needing several helpers makes several bilateral commitments.

### 9.3 Resources

A shared physical resource (a region, a doorway, a charging dock) is held through a leased
`reserve`. A reservation without a granted lease is not a reservation. Who arbitrates a
resource (a peer, a fixed owner or a fleet system) is part of the deployment's configuration
and is an open question for multi-robot settings (section 18).

### 9.4 Custody transfer

Extends the RL-13D handoff states with evidence:

```text
OFFERED → ACKNOWLEDGED → ACCEPTED → COMPLETED (both sides hold evidence)
        → REJECTED     → TIMED_OUT
```

Custody passes only when the receiver reports `COMPLETED` with its own fresh observation of
the thing. Until then the giver remains responsible. A sent message is never a handover
(plan v1.3 exit gate).

## 10. Failure model

| Situation | Required behavior |
|---|---|
| Duplicate message | Recognized by `message_id`; original reply repeated; no second effect |
| Messages out of order | Processed by `in_reply_to` and state, not arrival order; impossible transitions rejected |
| Expired message | Discarded; `expired` reply where one is expected |
| Peer silent, lease running | Wait; no new physical commitments that depend on the peer |
| Lease expired | Commitment lapses. Each side moves to a **locally defined safe state** for that activity and reports `unknown` or `failed` when contact returns |
| Network partition during motion | The robot's own policy and safety systems govern. No remote party is assumed to be able to stop it |
| Stale observation | Treated as unknown (6.6); criteria depending on it are not met |
| Conflicting observations | Not resolved by voting. The deciding robot uses its own sensors where safety is concerned, otherwise reports the conflict |
| Backend reports success without evidence | Outcome is `unknown`, not `succeeded` |
| Crash and restart | Commitments and goals restored from durable state, or reported as `unknown`; never silently dropped or silently resumed |

## 11. Trust, authority and safety

### 11.1 Identity

Each participant has a key pair. Messages are signed (7). Identity is bound to keys through
the deployment's configuration or through a binding's mechanism (for example signed A2A Agent
Cards). Unsigned messages may be accepted only for read-only operations, if the deployment
allows it.

### 11.2 Authority

Authentication says who sent a message. **Authorization decides whether the robot acts, and
it is local.**

- Every robot belongs to an `authority_domain` with a local policy.
- A policy decides, per goal, whether to act and which constraints apply. It may require
  approval by a named human, a policy service or both.
- The requesting participant is **never** its own authority. An AI agent cannot authorize its
  own goal.
- A physical-execution policy is disabled by default, consistent with the plan's decision A5.

### 11.3 AI-originated content

Everything produced by a model (goals, plans, observations, negotiation) is untrusted input.
It passes the same schema validation, plan validation and policy as any other input, with
provenance recording the provider and model (plan RL-20C). No provider receives special
privileges.

### 11.4 Peer-originated observations

A peer's observation may inform planning. It must not be the only basis for a decision that
affects the safety of a robot's own motion; for those, the robot relies on its own sensing or
on infrastructure it trusts under its policy. This limits the damage from a faulty,
compromised or spoofed peer.

### 11.5 Safety

RoboLang constraints (`max_speed`, `avoid_place`) are requirements passed to the execution
backend and checked by the executor. They are not safety functions. Safety-rated stops,
monitoring and risk assessment are outside RoboLang and must keep working with every
RoboLang component, LLM and network unavailable.

## 12. Bindings

A binding carries the envelope unchanged and maps identities. It may not add or drop meaning.

| Binding | Use | Notes |
|---|---|---|
| **A2A** | First-class. A2R and R2R as an A2A extension | Agent Card advertises RoboLang support and points to the capability descriptor. A2A tasks carry RoboLang conversations. Gives reach into the agent ecosystem |
| **MCP** | Optional, for LLM clients | A robot (or gateway) may expose A2R operations as MCP tools. Calling a tool is a `propose_goal`; authorization is unchanged |
| **ROS 2** | Execution backend; optionally a local transport | Plans run through ROS 2 actions where a robot uses ROS 2. As a transport, DDS QoS (reliability, deadline, liveliness) can carry the envelope on a local network |
| **Local peer-to-peer** | Required for P6 | A minimal transport for robots with no shared middleware and no cloud. To be selected in the proof |

## 13. Related work

Checked on 2026-10-19. Status notes are as found then.

| Work | Status | Relationship |
|---|---|---|
| A2A (Linux Foundation) | v1.0 in 2026; signed Agent Cards, extensions, several transport bindings | Agent-to-agent layer. RoboLang binds to it (12) |
| Model Context Protocol | Revision 2026-07-28; sessionless, formal extensions framework, Tasks | LLM-to-tool layer. Optional binding (12) |
| 3GPP TR 22.916, Release 19 | Study on Network of Service Robots with Ambient Intelligence; robot-to-robot as one scenario | Network-level requirements, not task semantics |
| OMG RoSO | 1.0 formal (April 2026); 1.1 in beta | Vocabulary for robotic functions; candidate for capability alignment (18) |
| IEEE 1872 family; P1872.3 | 1872-2015 and 1872.2-2021 published; P1872.3 (multi-robot reasoning) is an active project | Ontology alignment candidate (18) |
| VDA 5050 | Fleet control of mobile robots | Possible binding for mobile fleets; centralized model, different scope |
| Robot Context Protocol (arXiv 2506.11650) | 2025 preprint; read/write/execute/subscribe over HTTP/WebSocket, MCP and A2A adapters | Closest prior work for A2R; operation-level rather than goal- and evidence-level |
| `r2r-protocol` (Tech-Parivartan) | Open-source project | Uses the name "R2R" |
| `r2r_spl` (ROS Sports) | ROS 2 package for RoboCup SPL team messages | Uses the name "R2R" |

**Naming.** "R2R" is already used by at least two projects and by 3GPP for a scenario. This
RFC therefore uses A2R and R2R only as names of **RoboLang interaction profiles**
("RoboLang R2R"), not as standalone standard names (decision D3).

## 14. Compatibility with RoboLang 1.1

- RoboLang 1.1, its grammar, IR schema 1.0 and the `validate` and `inspect` commands are
  unchanged.
- A 1.1 task IR is the first plan kind (6.8). Validated 1.1 tasks can be carried and executed
  inside A2R without change.
- The 1.1 `communicate` statement stays a free-text step inside a plan. It is not R2R.
  Structured messaging is defined at the Core level, not by changing that statement.
- New language constructs (for example goals, outcomes or skill calls in `.rob` text) would
  come in a later language version, derived from the Core schemas, and follow the existing
  compatibility policy.
- The deferred semantics in [`robolang-v1.1.md`](../robolang-v1.1.md) section 9 (parameter
  binding, target resolution, condition evaluation) are answered by 6.5, 6.6 and 6.8.

## 15. Reference scenario

**An AI agent asks for a container to be delivered. The robot it asks cannot carry it alone.**

Participants: `planner-1` (agent), `arm-02` (fixed manipulator, can grasp up to 30 kg, cannot
travel), `amr-07` (mobile robot, can carry 40 kg, cannot grasp).

| # | Profile | From → to | Type | Content |
|---|---|---|---|---|
| 1 | A2R | planner-1 → arm-02 | `propose_goal` | C42 located in S3, verified by observation, deadline 10:30 |
| 2 | A2R | arm-02 → planner-1 | `assessment` | Feasible with assistance: arm-02 cannot reach S3 |
| 3 | R2R | arm-02 → amr-07 | `request_assistance` | Sub-goal: C42 from H1 to S3, 25 kg |
| 4 | R2R | amr-07 → arm-02 | `offer_assistance` | Accepts subject to holding H1 from 10:15 to 10:20; lease to 10:25 |
| 5 | R2R | arm-02 → amr-07 | `commit` | Commitment `91aa` |
| 6 | R2R | amr-07 → arm-02 | `reserve` granted | H1 leased |
| 7 | A2R | arm-02 → planner-1 | `accept` | Then requests authorization from its authority domain |
| 8 | – | authority → arm-02, amr-07 | authorization | Each robot authorized under its own policy (simulation only) |
| 9 | R2R | arm-02 ↔ amr-07 | `coordinate`, `transfer_custody` | C42 placed on amr-07 in H1; amr-07 observes it; custody `COMPLETED` |
| 10 | R2R | amr-07 → arm-02 | `outcome` | Sub-goal succeeded; evidence: observation of C42 in S3 |
| 11 | A2R | arm-02 → planner-1 | `outcome` | Goal `succeeded`, `verified: true`, evidence attached |

**Failure variants the proof must cover:**

| Variant | Expected result |
|---|---|
| amr-07 declines | arm-02 reports `assessment: infeasible`; no motion |
| Step 5 delivered twice | Second copy recognized; one commitment |
| amr-07 goes silent after step 6 | Lease lapses at 10:25; arm-02 keeps custody, moves to its safe state, reports `failed` or `unknown` |
| Observation of C42 in H1 is 40 s old | Stale; handover does not start |
| amr-07 reports "arrived" without an observation of C42 in S3 | Sub-goal outcome `unknown`; goal not `succeeded` |
| planner-1 tries to authorize the goal itself | Rejected: the requester is not an authority |
| Unsigned or wrongly signed `commit` | Rejected |
| arm-02's descriptor lists no grasp capability | Assessment fails before any commitment |

## 16. Proposed next steps

| Step | Deliverable | Boundaries |
|---|---|---|
| **RL-ARCH-01** (this RFC) | Owner decisions on section 17; then decision record 0002 | Documentation only |
| **RL-ARCH-02 — executable proof** | JSON Schemas for the envelope, goal, capability descriptor, observation, commitment and outcome. An in-process Python simulation of section 15 with one agent stub and two robot stubs, each with a minimal executor and a simulated world. Tests for every failure variant in section 15 | No ROS 2, no network transport, no LLM calls, no hardware. Simulation only. Does not change the 1.1 parser or IR |
| **Re-scope RL-12 and RL-13** | Revised development plan: the executor, condition evaluation and capability work re-framed as the execution side of A2R; RL-13C/D replaced by the R2R profile | Owner approval required |
| **Later** | A2A binding, local transport, ROS 2 backend, ontology alignment, language constructs | Each its own milestone |

**Acceptance for RL-ARCH-02:** the happy path of section 15 ends in a verified `succeeded`
outcome; every failure variant ends in the stated result with no simulated motion where none
is allowed; every run produces a reconstructable message and event log.

The developer-preview pilot of RoboLang 1.1 is independent of this RFC and stays the
immediate product-validation step.

## 17. Decisions requested from the owner

| # | Decision | Proposal |
|---|---|---|
| D1 | Revise the mission statement of the development plan | From "task specification language that delegates execution to ROS 2" to "shared semantic model and interaction profiles for AI-to-robot and robot-to-robot cooperation, with execution through interchangeable backends" |
| D2 | The IR is the primary contract | Versioned JSON schemas define interoperability; `.rob` text is one authoring surface |
| D3 | Naming | Use "RoboLang A2R" and "RoboLang R2R" as profile names; do not present "R2R" as a standalone standard |
| D4 | A2A binding | First-class binding from the start, implemented after the proof |
| D5 | Commitments | Bilateral and leased only in version 0; no multi-party consensus |
| D6 | Authority | Adopt section 11.2: authorization is local; a requester is never its own authority |
| D7 | Non-goals in the plan | Keep "distributed consensus" as a non-goal (consistent with D5). Clarify that "inventing an emergent robot language" means a language robots develop themselves, which A2R and R2R are not: they are designed and specified |
| D8 | Next milestone | Approve RL-ARCH-02 as defined in section 16, after or in parallel with the pilot |

Once decided, the decisions are recorded as `spec/decisions/0002-…` and this RFC's status is
updated. Rejected proposals are recorded too.

## 18. Open questions

1. **Ontology alignment.** Adopt IEEE 1872 / OMG RoSO terms for capabilities and things, map
   to them, or define RoboLang's own vocabulary with a published mapping?
2. **World model depth.** How much geometry and state belongs in the Core, and how much stays
   private to each robot?
3. **Time.** Freshness and leases need comparable clocks. What clock synchronization is
   assumed, and how is clock uncertainty expressed?
4. **Evidence formats.** What counts as evidence for common criteria, and can evidence include
   sensor data references without moving large payloads?
5. **Key management.** How keys are issued, rotated and revoked in a deployment without a
   cloud.
6. **Resource arbitration.** Who arbitrates a shared resource when more than two robots are
   involved?
7. **Local transport.** Which transport to use for P6 in the proof and beyond.
8. **Text syntax.** Whether, and when, goals and outcomes get `.rob` syntax.
9. **Learned policies.** How a learned or vision-language-action policy is described as a
   capability, given that its behavior cannot be enumerated.

## References

- [A2A protocol](https://a2a-protocol.org/)
- [Model Context Protocol, 2026-07-28 changes (Agentic AI Foundation)](https://aaif.io/blog/mcp-2026-07-28-whats-changing-and-how-to-migrate)
- [3GPP TR 22.916](https://www.3gpp.org/DynaReport/22916.htm)
- [OMG Robotic Service Ontology](https://www.omg.org/spec/ROSO)
- [IEEE P1872.3](https://standards.ieee.org/ieee/1872.3/11037)
- [Robot Context Protocol, arXiv 2506.11650](https://arxiv.org/abs/2506.11650)
- [ROS REP-103: Standard units and coordinate conventions](https://www.ros.org/reps/rep-0103.html)
- [ROS REP-105: Coordinate frames for mobile platforms](https://www.ros.org/reps/rep-0105.html)
- [RoboLang development plan](../../docs/DEVELOPMENT_PLAN.md)
- [RoboLang 1.1 specification](../robolang-v1.1.md)
- [RoboLang IR schema 1.0](../ir-schema-v1.0.md)
