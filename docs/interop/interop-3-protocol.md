# INTEROP-3 — Independent author protocol

**Status:** **APPROVED FOR PREPARATION — 2026-10-28.** Launch is **not** authorized. The archive is
not delivered and no implementation starts until (1) the exact model ID and API access are verified
outside the RoboLang sessions, (2) the environment operator signs the attestation of section 5, and
(3) the owner authorizes launch.

**For the owner and the environment operator only.** This document is never placed in the author's
environment. The author receives the frozen archive and the initial prompt of section 4, nothing else.

**Frozen inputs:** source commit `28c69feece0de14b1537a4d29e0cd4b700cff920`, bundle ID
`e6c9bb751503d52a16aa7f6aa0c42c9fe54726027ae5ff6ef229e566df22d104`
([freeze record](gate2-bundle-freeze.md)); archive `robolang-gate2-bundle-e6c9bb75.zip`, SHA-256
`02bcc7d490532081eb32c70cf3bc9ef8665d94f6e1264f885ebf24fb17860ba1`.

## 1. Decisions

| Decision | Requirement |
|---|---|
| Independent author | A new OpenAI API agent session (Gate 2 2.1, clean-room AI form) |
| Model | GPT-6 family preferred; the exact API identifier is verified externally before launch |
| Language | TypeScript on Node.js (Gate 2 5) |
| Environment | A new isolated VM or container |
| Source access | The frozen archive only |
| Repository access | None: no GitHub or other access to the RoboLang repository |
| Memory | No earlier conversation, shared memory or RoboLang-specific custom instructions |
| Network | Only the required OpenAI API endpoint |
| Scope | The Core 0.1 receiver and an independent amr-07 participant |
| Tests | G-T0 by the author; G-T1 to G-T5 belong to INTEROP-4 |
| Evidence | Full transcript, environment configuration, Q&A log, implementation commits |

## 2. Safeguards

1. **Environment.** Provision a clean environment with the runtime and compiler installed
   beforehand. The unpacked bundle is read-only. The implementation goes in a separate, initially
   empty directory. No reference repository, harness, golden records or other project file is
   mounted.
2. **Archive verification, before the author starts.**
   - The archive's SHA-256 is `02bcc7d4…860ba1`.
   - After extraction, there are exactly 61 files.
   - The SHA-256 of `MANIFEST.sha256` is `e6c9bb75…d104`.
   - `sha256sum -c MANIFEST.sha256` passes.
3. **Tools.**
   - Permitted: local editing, compiling, deterministic testing, and standard-library cryptography.
   - JSON Schema and JCS libraries are permitted (Gate 2 5) if they are provisioned independently
     and contain no RoboLang implementation material.
   - Blocked: external search, access to the reference repository, and unrestricted network calls.
4. **Questions.**
   - Questions go through a written Q&A channel, administered outside the author session.
   - The intermediary answers only from the bundle and gives no implementation hints or private
     case expectations.
   - An answer that adds or changes a rule is an erratum (Gate 2 4.3, 9.3): a decision, a new
     frozen bundle, and both implementations retested. It is never an informal explanation.
5. **Audit trail.** Record:
   - the complete API conversation and every tool operation;
   - the model identifier, configuration and initial prompt;
   - every implementation change and local test result;
   - the Q&A log.

   Preserve the final code at an identifiable commit.
6. **Completion.** INTEROP-3 ends when the author declares the implementation ready, supplies
   reproducible build and run instructions, and reports its own G-T0 results. That is not a gate 2
   pass (Gate 2 9.4).

## 3. Consistency with the frozen Gate 2 specification

Checked against the frozen `spec/interop-gate-2-v0.1.md`. The attestation (section 5) must confirm
each point.

| Gate 2 | Requirement | How the protocol meets it |
|---|---|---|
| 2.1 | A new session with no earlier RoboLang conversation, no shared memory, no custom instructions that mention RoboLang | A new API session with no memory features. The only RoboLang text the model receives is the task prompt (section 4) and the bundle; no system or custom instructions beyond them |
| 2.1 | No access to the repository or its history, and no tool that can reach it | Network limited to the API endpoint; no GitHub; nothing from this repository is mounted, **including this document** |
| 2.1 | Starts in an empty working directory containing only the bundle | The workspace root holds only the read-only bundle and the empty implementation directory |
| 2.1 | Evidence: model and provider, complete initial prompt, configuration showing no memory and no repository access, full transcript, Q&A log | Section 2.5 and the attestation. Record the model ID **as returned by the API in each response**, not only as requested |
| 4.3 | Questions logged; a rule-adding answer is an erratum | Section 2.4 |
| 5 | Not Python; standard library; optional JSON Schema validator; JCS from Core 7 or a published library; nothing from this repository; no network, filesystem or clock access during a run beyond the binding | TypeScript on Node.js. Libraries are pre-installed (network is closed), each recorded with name, version and integrity hash. The run-time restriction binds the author's code and is exercised by G-T5 |
| 7 | G-T0: the stated receipt for all 47 fixtures | The author's step 3. The check-mode output format is the author's choice (audit risk R-2), recorded with the results |
| 11 | INTEROP-3 is done when the author declares the implementation ready | Section 2.6 |

**Who may operate the environment:** the RoboLang team may provision it. Nobody who worked on the
reference, this session included, may act as the author or pass reference knowledge to it, and
answers given as the Q&A intermediary come only from the bundle.

## 4. Initial prompt (verbatim, as approved)

```text
You are an independent software engineer implementing the RoboLang Gate 2 participant `rl:robot:amr-07`.
You have received a frozen specification bundle. It is your sole source of RoboLang requirements.
Implement the participant independently in TypeScript, using the published Core 0.1 specification, reference simulation requirements, JSON schemas, conformance fixtures, and `robolang-sim-stdio` 0.1 binding.
Work in this order:

1. Study the bundle and identify the normative requirements.
2. Implement Core 0.1 validation and the required receiver behavior.
3. Implement the conformance `check` mode and evaluate all 47 supplied fixtures.
4. Implement the `amr-07` participant, its local state transitions, authorization, commitments, custody, sequencing, sensing, timing, and messages.
5. Implement the published standard-input/standard-output binding.
6. Add independent tests and reproducible build/run instructions.
7. Declare the implementation ready when your work is complete.

Do not access any RoboLang reference repository, existing implementation, private test cases, or previous project discussions.
Do not invent missing normative requirements. Log any ambiguity as a question for the designated intermediary.
Do not attempt to access or reconstruct hidden tests.
Use only the permitted runtime, libraries, filesystem locations and API access.
When finished, provide the complete implementation, dependency information, test results, known limitations, and a signed-off readiness declaration.
Your work will subsequently be assessed by a separate harness. You are not responsible for operating that harness or judging whether Gate 2 passes.
```

The prompt adds nothing beyond the bundle: it names the bundle's own parts and the order of work.
The operator should add only environment facts: where the bundle and the implementation directory
are, how to submit a question, and which tools exist. Those additions are recorded with the prompt.

## 5. Pre-launch attestation (to be completed by the environment operator)

| Item | Value (operator fills in) |
|---|---|
| Operator (name or anonymised ID), date | |
| Provider; exact model ID as requested; model ID as returned by a test call | |
| API account or project: new, no memory or stored-conversation features, no custom instructions | |
| Agent runtime and its configuration (tools offered, limits) | |
| VM or container image and its digest | |
| Node.js and TypeScript versions | |
| Pre-installed libraries: name, version, integrity hash, source | |
| Network egress policy: only the API endpoint (attach the rule set) | |
| Filesystem layout: read-only bundle path, empty implementation path, nothing else mounted | |
| Archive SHA-256 computed in the environment (must be `02bcc7d4…860ba1`) | |
| Files after extraction (must be 61); SHA-256 of `MANIFEST.sha256` (must be `e6c9bb75…d104`); `sha256sum -c` result | |
| Transcript capture: mechanism, storage location, confirmed active | |
| Q&A channel and intermediary | |
| Confirmation that nothing from the RoboLang repository, including this document, is in the environment | |
| Operator signature | |

**Launch** needs the completed attestation and the owner's authorization, recorded here or in a
follow-up record, together with the delivery entry of the archive's delivery receipt.
