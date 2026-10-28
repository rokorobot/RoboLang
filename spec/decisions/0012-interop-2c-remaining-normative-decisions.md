# Decision record 0012 — Remaining normative decisions before the reference corrections (INTEROP-2c)

**Date:** 2026-10-28
**Status:** **APPROVED — 2026-10-28.** P1, P2 and P3 approved by the owner. Applied to the
specifications by INTEROP-2c-SPEC (authorized 2026-10-28), with the wording of section 4. No
code, test, fixture, schema or golden record changes under this record: the reference
corrections are INTEROP-2d, and the added fixtures INTEROP-2f, each separately authorized.
**Applies to:** [`spec/reference-simulation-v0.1.md`](../reference-simulation-v0.1.md) 6.2 rule 3
(safe state) and 6.4 (lease expiry); [`spec/core-v0.1.md`](../core-v0.1.md) 8.1 (stage 2), 8.2
(the `C306` definition) and 8.4
**Builds on:** [decision record 0011](0011-interop-2a-bundle-errata.md), section 4 (defects D3 and
D6, whose correction approaches were accepted but which no N row made normative), and the
[discrepancy register](../../docs/interop/spec-implementation-discrepancies.md), section 3

## 1. Why

INTEROP-2b-SPEC applied N1–N14 and left three rules unstated, because no approved row covered
them:

- **D3:** what happens to an action already in progress when the commitment lapses;
- **D6:** two different messages with the same sequence number while one is held;
- **S2:** the order of the stage-2 codes for one signed object.

Each must be normative before the reference is corrected (INTEROP-2d). Otherwise the
correction would create the semantics instead of implementing them.

## 2. Decisions

| # | Topic | Proposed rule | Alternatives considered | Spec | Status |
|---|---|---|---|---|---|
| P1 | D3: lease expiry during an action | An action already in progress completes, and its simulated world effect applies. Expiry prevents new actions. When the action completes after the commitment lapsed, amr-07 takes none of its follow-up steps: it changes no local state (the commitment stays `lapsed`, the goal keeps the terminal state the expiry set), senses nothing and sends nothing. **Simulation only:** not a stopping policy and not a physical safety guarantee (section 3) | **(a)** A new world action that interrupts the motion at expiry: more realistic, but it adds world semantics 0.1 does not model (partial positions, a carried thing between places), and RFC-001 open question 10 is still open. **(b)** Keep the reference's behavior, where arrival evaluates and sends a second outcome: two outcomes for one sub-goal, and `fulfilled` over `lapsed` (D3) | Sim 6.2, 6.4 | APPROVED 2026-10-28 |
| P2 | D6: conflicting sequence number | A message whose sequence equals that of a message currently **held in the same stream** (same sender, recipient and `conversation_id`, Core 3 and 8.4), but whose message ID differs, is rejected with `C306`. The held message is unaffected: it keeps its place and gets exactly one terminal release receipt. A copy with the same ID stays a duplicate (8.4). The rule never applies across streams | **(a)** Reject both: an attacker or a faulty sender could then cancel a legitimate held message by sending a conflicting one. **(b)** Treat the second as a duplicate, keyed by sequence: it hides a conflicting content behind a `duplicate` receipt. **(c)** Replace the held message with the second (the reference today): the first never gets a receipt, against 0011 N3's "exactly one terminal release receipt" | Core 8.2, 8.4 | APPROVED 2026-10-28 |
| P3 | S2: stage-2 precedence | For each signed object: `C204` (the algorithm is not accepted by the receiver in its mode), then `C203` (the key is unknown, revoked, or not the signer's), then `C202` (the signature does not verify). The first failure is reported. The message's own signature comes before the observations it carries (already in Core 8.1) | **(a)** `C202` first: verifying under an algorithm or key the receiver does not accept for that signer proves nothing, and would mean computing with untrusted material. **(b)** Leave the order open: the conformance kit (G-T0) compares exact codes, so a multi-fault message would give a finding with no rule to resolve it | Core 8.1 | APPROVED 2026-10-28 |

## 3. The D3 safety boundary

P1 is acceptable only because the 0.1 world is symbolic: a move is two events, a start and an
effect, with nothing in between to interrupt. Continuing a physical movement after its
authorization or commitment has expired could be unsafe. The rule therefore states:

- **No interruption in 0.1:** RoboLang 0.1 does not model interrupting an action.
- **Not a physical guarantee:** the behavior is a property of the simulation, not a stopping
  policy, and not a safety guarantee for physical execution.
- **Open question 10 stays open:** what a robot must do with an action in progress when its
  authority lapses is RFC-001 open question 10 (a robot's safe state). A physical binding must
  answer it; this record does not.

## 4. Normative wording (approved and applied)

**P1: reference simulation 6.2 rule 3, replacing the sentence "In 0.1 a robot's safe state is
"no further motion"; RFC-001 open question 10 stays open.":**

> In 0.1 a robot's safe state is "no further motion": it starts no new motion. In the symbolic
> simulation this does not interrupt an action already in progress, which completes as 6.4 says
> (decision record 0012, P1). RFC-001 open question 10 stays open.

**P1: reference simulation 6.4, after the lease-expiry table:**

> **An action in progress at expiry** (decision record 0012, P1). RoboLang 0.1 does not model
> interrupting an action. An action that has started completes when its duration ends; its
> effect on the symbolic world applies, and its motion and effect events stand. Expiry prevents
> new actions (6.2 rule 2). When an action of amr-07 completes after its commitment has lapsed,
> amr-07 takes none of the action's follow-up steps: it changes no local state (the commitment
> stays `lapsed`, and the goal keeps the terminal state the expiry set), senses nothing and
> sends nothing. **This is a property of the simulation only.** It is not a stopping policy and
> not a safety guarantee for physical execution: what a robot must do with an action in
> progress when its authority lapses is RFC-001 open question 10, which stays open.

**P2: Core 8.4, a new item after "Stale":**

> - **Conflicting sequence** (decision record 0012, P2). A message whose `sequence` equals that
>   of a message currently held in the same stream (same sender, recipient and
>   `conversation_id`), but whose `message_id` differs, is rejected with `C306`. The original
>   held message is unaffected and receives exactly one terminal release receipt.

**P2: Core 8.2, the `C306` row of the code table, replacing its meaning:**

> The message would cause an impossible state transition; or it is stale: at or below the last
> processed or skipped sequence of its stream, and not a duplicate (8.4); or its sequence
> conflicts with a message currently held in its stream (8.4)

**P3: Core 8.1, replacing the sentence "Core 0.1 does not fix the order of `C204`, `C203` and
`C202` for one signed object…":**

> For each signed object, the checks apply in this order and the first failure is reported
> (decision record 0012, P3): `C204`, the algorithm is not accepted by this receiver in its mode;
> `C203`, the key is unknown, revoked, or not the signer's; `C202`, the signature does not
> verify.

## 5. Expected impact

| | Existing 24 golden records | Reference | Harness | Fixtures |
|---|---|---|---|---|
| P1 | **No change expected.** No amr-07 action completes after its commitment lapsed in any of the 24 cases (measured; the latest amr-07 move starts at 10:14:51.200, and the leases end at 10:25) | The D3 correction (INTEROP-2d) | None | A new case would be an **added** golden record (INTEROP-2f) |
| P2 | **No change expected.** No delivery in the 24 cases meets a held message of the same sequence with a different ID (measured over 406 deliveries; 3 held messages seen) | The D6 correction (INTEROP-2d) | None. The INTEROP-1 harness already treats a release receipt for a message with no open hold as inconsistent | A new sequencing case would be **added** (INTEROP-2f) |
| P3 | **No change expected.** No message in the 24 cases has more than one stage-2 fault (measured over 404 signature checks). `robolang.core` already applies this order | None: it already conforms | None | Multi-fault stage-2 fixtures (INTEROP-2f) |

The measurements ran on `main` at `c1ecdc4`, with the reference unmodified and read-only probes
around its receiver. They are expectations to be confirmed by the golden test when each
correction is implemented; they do not authorize changing any golden record.

## 6. Revision history

| Date | Change |
|---|---|
| 2026-10-28 | First draft: P1–P3 proposed (INTEROP-2c) |
| 2026-10-28 | Revision after review of PR #40 at `37595ea`; all rows still proposed. P2 now names the full stream (sender, recipient and `conversation_id`) instead of the sender alone, so the rule cannot apply across conversations; its proposed Core 8.4 wording is replaced accordingly |
| 2026-10-28 | Revision after final review of PR #40 at `0825e7b`; all rows still proposed, semantics unchanged. Two cross-reference clarifications: P1 adds wording for Sim 6.2 rule 3, so "no further motion" means starting no new motion and does not interrupt an action in progress; P2 adds wording for the Core 8.2 `C306` definition, so it covers a sequence conflicting with a held message |
| 2026-10-28 | **Approved** by the owner: P1, P2 and P3, semantics unchanged. Applied to the specifications by INTEROP-2c-SPEC with the wording of section 4: reference simulation 6.2 rule 3 and 6.4 (P1); Core 8.1 (P3), 8.2 and 8.4 (P2). Discrepancy register updated: D3 and D6 now cite the specification. RFC-001 open question 10 stays open |
