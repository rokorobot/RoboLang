# AI-First Language Design Charter

**Date:** 2026-10-27
**Status:** **Approved by the owner on 2026-10-27** (decisions G1–G3 in
[decision record 0005](../decisions/0005-ai-first-charter-and-domain-contracts.md)). It
authorizes no implementation.
**Builds on:** [RFC-001](rfc-001-ai-native-architecture.md) (D1–D9),
[RL-FOUNDATION-01](rl-foundation-01-language-foundation.md) (F1–F5),
[`simulation-v1.0.md`](../simulation-v1.0.md) (B1–B16)
**Applied by:** [RL-FOUNDATION-02 domain contracts](rl-foundation-02-domain-contracts.md)

## 1. Purpose

RoboLang is written primarily for AI systems to generate, read, exchange, validate and act on.
Humans design, review and authorize it. This charter states what that means for the design of
every one of the thirteen domains, and how to check that a design meets it.

> **Central abstraction.** RoboLang's fundamental abstraction is a **verifiable transition from
> an observed world state toward a desired outcome**. Actions, plans and communication are
> means to achieve that outcome; execution alone does not establish success.

In RFC-001 terms, the observed state is made of observations (7.6), the desired outcome is a
goal with success criteria (7.7), the means are plans (7.8) and commitments (7.9), and the
transition is established only by an outcome verified against evidence (7.10, 11.3).

The direction is already largely approved. RFC-001 puts semantics before syntax (P1), makes
versioned JSON schemas the interoperability contract (P2), and defines observations with
confidence and provenance (section 7.6), goals with success criteria (7.7) and outcomes that
include `unknown` (7.10). This charter does not restate those decisions. It turns them into
design rules for the language and adds **measurable criteria**, each tied to the artifact that
demonstrates it.

**No approved decision is amended by this charter.** Decision F2 makes the existing front end
the single *`.rob`* engine; decision D2 already lets an AI produce Core JSON directly, without
`.rob`. If applying the charter to a domain ever needs a change to D1–D9, F1–F5 or B1–B16, that
change is proposed as its own row in a decision record, never made in passing.

### 1.1 How RoboLang differs from a traditional language

| Traditional programming language | AI-native RoboLang |
|---|---|
| Humans write sequences of instructions | AI systems express goals, plans, constraints and actions |
| Syntax is organized around human readability | The representation is organized around reliable machine generation and validation |
| Variables and functions dominate | Entities, capabilities, relationships and state transitions are first-class |
| The programmer controls most behavior directly | AI proposes behavior; it runs only within authority granted by the robot's local policy |
| Errors are mostly programming mistakes | Uncertainty, stale observations and unsuccessful actions are normal outcomes, with defined handling |
| Execution usually assumes one computer | Execution may span several robots and AI systems |
| Success is inferred from a function returning | Success requires explicit, verified outcome evidence |

This is why RoboLang puts goals, typed semantics, uncertainty, authorization and verified
outcomes first, and treats a sequence of instructions as one possible means, not the
language's centre. It keeps what has proven itself regardless of origin: static typing,
units, deterministic validation and explicit execution authority (AF10).

## 2. Three parts, two representations, one meaning

| Part | What it is | Where it is defined |
|---|---|---|
| **Semantic Core** | The authoritative meaning of goals, plans, observations, capabilities, commitments, outcomes and the rest, as versioned JSON documents | RoboLang Core, RFC-001 sections 6–7 (schemas drafted in RL-ARCH-02) |
| **Language front end** | `.rob` authoring and the Language Kernel: lexer, parser, validator, Task IR | RL-FOUNDATION-01 section 4; `robolang-v1.1.md`; `ir-schema-v1.0.md` |
| **Runtime** | Validated, authorized execution and verified outcomes | `simulation-v1.0.md` (RL-BASIC-01); RL-12A–E; execution backends |

There is no third representation. An AI system can produce:

1. **Core JSON documents** (goals, plans, observations, outcomes) — validated by the Core path,
   separate from the kernel (F2);
2. **Task IR JSON** inside a plan (RFC-001 section 6.3) — re-validated by an IR reader under
   the IR trust rules (`ir-schema-v1.0.md` section 2; work item CORE-1 in the domain contracts);
3. **`.rob` source** — compiled by the kernel.

All three must mean the same thing for the same task, and none of them bypasses validation or
local authorization.

## 3. Design principles

| # | Principle | Rule for every domain | Builds on |
|---|---|---|---|
| AF1 | **Semantics before syntax** | A domain concept is defined first as typed data: its fields, values, units and rules. `.rob` syntax for it comes later, only for a demonstrated authoring need, and only through a versioned language specification | P1, F3 |
| AF2 | **Structured generation is the primary interface** | Every capability an AI can request is expressible as a Core or Task IR JSON document. `.rob` is an equal but optional authoring surface, never the only way in | P2, D2, RFC-001 6.4 |
| AF3 | **Closed, versioned vocabularies** | Every kind, property, unit, state and code is enumerated in a versioned specification. Unknown values are rejected, not guessed. A generator can know the complete set of valid outputs | RFC-001 6.2, A2 |
| AF4 | **Uncertainty is first-class** | Facts, observations and the unknown are kept distinct. Truth is three-valued (`true`, `false`, `unknown`). Every belief carries its age, source and confidence. Unknown or stale never counts as true | P4, P8, 7.6, B2 |
| AF5 | **Proposed, authorized, executed and verified are distinct states** | No representation lets a proposal stand for an authorization, an executed step stand for an achieved outcome, or confidence stand for evidence | P3, P5, 7.10, 13.4 |
| AF6 | **Intelligence is composed through interfaces** | Reasoning, memory, perception and planning are external components with typed contracts. RoboLang defines what they must return and validates it; it does not embed a model, database or planner | A4, 13.4 |
| AF7 | **RoboLang components are deterministic** | The same inputs give the same validation result, the same authorization decision and the same record | A4, B14 |
| AF8 | **Diagnostics are machine-actionable** | Every rejection has a stable code, a structured location (line and column, or JSON Pointer) and, where a fixed set was expected, the set. An AI can repair its output from the diagnostic alone | `robolang-v1.1.md` 6, B16, RL-20B |
| AF9 | **Plans name capabilities, not machines** | Tasks and goals refer to entities and capabilities; robot-specific resolution belongs to the executing robot | A3, 7.3, RL-13B |
| AF10 | **No convention by inheritance** | A construct exists because robotics or AI coordination needs it, not because Python, C++, ROS 2 or another DSL has it. Proven properties are kept on purpose: static typing, units, deterministic validation and explicit execution authority | — |

## 4. Measurable criteria

A domain design or implementation slice meets the charter when the criteria that apply to it
hold. Each criterion names the artifact that demonstrates it.

| # | Criterion | Measure | Demonstrated by | Available |
|---|---|---|---|---|
| MC1 | Unambiguous typed representation | Every supported construct has exactly one canonical JSON form. The same inputs in the same canonical representation always produce byte-identical output. Equivalent content written in different authoring formats (for example `.rob` and Core JSON) must have the same meaning, not necessarily identical bytes | IR golden snapshots (`tests/snapshots/ir/`); run-record determinism (T11) | IR now; run record in RL-BASIC-01b |
| MC2 | Deterministic rejection | 100% of a published negative corpus is rejected with the stated code, and no rejected input reaches execution | The 76 v1.1 fixtures; world-file fixtures `E5xx`; each new slice adds its own | Now, extended per slice |
| MC3 | Unknown never counts as true | Every fixture where a required fact is unknown or stale ends without the dependent action | T4, T8 (`S102`, `S233`); staleness fixtures with Core observations | RL-BASIC-01b; Foundation 2 |
| MC4 | Proposal, authorization, execution and outcome are distinguishable | A record or document always states which of the four it is; a success without evidence is rejected | Run-record states (B14); Core outcome rules in RL-ARCH-02 tests | RL-BASIC-01b; RL-ARCH-02 |
| MC5 | Portability | The same plan runs on two compatible robot profiles without edits; an incompatible profile is rejected before execution | RL-13A/B fixtures | Foundation 3 |
| MC6 | Repairability | Every diagnostic carries code, location and expected set; repair success within a bounded number of attempts is measured | Diagnostic format tests; RL-20D benchmark | Codes now; benchmark in v2.0 |
| MC7 | Generation reliability | Valid-output rate on a preregistered held-out task set meets a target approved before benchmarking | RL-20D | v2.0 |
| MC8 | Reproducibility | Identical inputs give identical records on every supported platform | T11 on Ubuntu and Windows CI | RL-BASIC-01b |

## 5. Consequences for the thirteen domains

The thirteen domains describe everything RoboLang is meant to express. They are not thirteen
implementations: what unifies them, and what is new, is one AI-native semantic language with
shared types, truth values, evidence and authority rules.

The [domain contracts](rl-foundation-02-domain-contracts.md) apply this charter to each
domain. In particular:

- Every contract states where the domain's concepts live: in Task IR plan steps, in Core
  documents, in the simulation runtime, or behind an external interface.
- Every contract defines its concepts as data first (AF1). Any `.rob` syntax it mentions is a
  non-normative sketch for a future versioned specification.
- Goals and outcomes in `.rob` remain RFC-001 open question 8.
- Thinking & Reasoning and Memory & Knowledge are interfaces with verifiable results (AF6).
  Their negative tests check that unverified or unauthorized proposals are rejected, not that
  reasoning is good.

## 6. What this charter does not do

- It does not change RoboLang 1.1, Task IR 1.0, the CLI, RFC-001, or any approved decision.
- It does not create Core schema files. Drafting them is RL-ARCH-02 (RFC-001 section 19).
- It does not authorize implementation of any slice.
