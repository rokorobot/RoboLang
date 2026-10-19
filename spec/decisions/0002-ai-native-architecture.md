# Decision record 0002 — AI-native architecture (RL-ARCH-01)

**Date:** 2026-10-19
**Status:** Scope approved. **All nine substantive decisions are pending owner approval.**
**Applies to:** [`spec/architecture/rfc-001-ai-native-architecture.md`](../architecture/rfc-001-ai-native-architecture.md)

## 1. What the owner has approved

The owner approved the **documentation-only scope** of RL-ARCH-01: drafting and refining
RFC-001 and this decision record on a draft pull request.

That approval does **not** cover any of the decisions in section 2. Until each is decided:

- RoboLang 1.1, its grammar, parser, validator, IR schema 1.0, CLI and runtime stay exactly
  as they are.
- [`docs/DEVELOPMENT_PLAN.md`](../../docs/DEVELOPMENT_PLAN.md) is not changed. RFC-001
  section 17 lists where it would need to change.
- RL-ARCH-02 (the executable proof) is not started.

## 2. Decisions

| # | Topic | Proposal (RFC-001 section 20) | Status |
|---|---|---|---|
| D1 | Mission statement | Change the plan's mission from "task specification language that delegates execution to ROS 2" to "an open, AI-native interoperability platform for embodied intelligence", supporting a vendor-neutral ecosystem of interoperable implementations (RFC-001 section 1.1) | PROPOSED — OWNER APPROVAL PENDING |
| D2 | Interoperability contract | Versioned Core JSON schemas define interoperability. Task IR 1.0 stays one supported plan representation; `.rob` stays an authoring format | PROPOSED — OWNER APPROVAL PENDING |
| D3 | Naming | Use "RoboLang A2R" and "RoboLang R2R" as interaction-profile names; do not present "R2R" as a standalone standard | PROPOSED — OWNER APPROVAL PENDING |
| D4 | A2A | First-class interoperability target from the start, implemented after the proof; not a mandatory transport | PROPOSED — OWNER APPROVAL PENDING |
| D5 | Commitments | Bilateral and leased only in version 0; no multi-party consensus | PROPOSED — OWNER APPROVAL PENDING |
| D6 | Authority | Authorization is local to each robot; a requester is never its own authority; autonomous action through bounded, leased, revocable standing authorizations (RFC-001 section 13) | PROPOSED — OWNER APPROVAL PENDING |
| D7 | Non-goal wording | Keep "distributed consensus" as a non-goal; clarify that "inventing an emergent robot language" means a language robots develop themselves, which A2R and R2R are not | PROPOSED — OWNER APPROVAL PENDING |
| D8 | Next milestone | Approve RL-ARCH-02, an in-process Python simulation of RFC-001 section 18 with its failure variants, after or in parallel with the developer-preview pilot | PROPOSED — OWNER APPROVAL PENDING |
| D9 | Ecosystem before implementation | Adopt principle P10 (the Core must be implementable from the specification alone, without the Python package) and the two interoperability gates of RFC-001 section 1.4, including an independent implementation interoperating with the reference | PROPOSED — OWNER APPROVAL PENDING |

When a decision is taken, its row is updated with the outcome, the date and any change to the
proposal. A rejected proposal stays in the table, marked rejected.

## 3. Long-term ambition

RFC-001 revision 3 records the long-term ambition (sections 1.1 to 1.4): an open,
vendor-neutral interoperability platform for embodied intelligence, described internally as
"the Linux of robotics". It is recorded as **ambition, not commitment**. It does not widen
the documentation-only scope of RL-ARCH-01 or the proposed scope of RL-ARCH-02, and it does
not approve a distribution, package manager, skill registry, manufacturer runtimes or a
robotics operating system.

## 4. Open architectural questions

Carried from RFC-001 section 21. None is decided.

1. **Ontology alignment.** Adopt IEEE 1872 / OMG RoSO terms for capabilities and things, map
   to them, or define RoboLang's own vocabulary with a published mapping?
2. **World model depth.** How much geometry and state belongs in the Core, and how much stays
   private to each robot?
3. **Time.** What clock synchronization is assumed, what clock tolerance is typical, and how
   is it configured and expressed?
4. **Evidence formats.** What counts as evidence for common criteria, and can evidence refer
   to sensor data without moving large payloads?
5. **Key management.** How keys are issued, rotated and revoked in a deployment without a
   cloud, and how revocation reaches robots that are offline.
6. **Resource arbitration.** Who arbitrates a shared resource when more than two robots are
   involved?
7. **Local transport.** Which transport to use for local-first operation in the proof and
   beyond.
8. **Text syntax.** Whether, and when, goals and outcomes get `.rob` syntax.
9. **Learned policies.** How a learned or vision-language-action policy is described as a
   capability, given that its behavior cannot be enumerated.
10. **Safe states.** How a robot's locally defined safe state for an activity is declared,
    and whether peers need to know it.
11. **Policy language.** How local policies and standing authorizations are expressed and
    distributed, and whether an existing policy language is reused.
12. **Skill model.** How a portable skill declares its requirements, expected outcomes and
    test evidence, and how a robot demonstrates that it satisfies them.
13. **Conformance.** How conformance is tested and claimed: test suites, profiles and what an implementation must pass to call itself
    conforming.
14. **Governance.** Where the specifications are maintained, under what licence, how changes
    are proposed and decided, and whether and when to move to vendor-neutral governance.
