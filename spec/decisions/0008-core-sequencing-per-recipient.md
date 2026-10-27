# Decision record 0008 — Core 0.1 sequencing per sender and recipient (RL-ARCH-02-SEQ-01)

**Date:** 2026-10-27
**Status:** **PROPOSED.** Nine decisions for the owner. No implementation is authorized.
**Applies to:** [`spec/core-v0.1.md`](../core-v0.1.md) sections 3 and 8,
[`spec/reference-simulation-v0.1.md`](../reference-simulation-v0.1.md) sections 3, 5.2, 9–12,
[`tests/fixtures/arch02/`](../../tests/fixtures/arch02/README.md)
**Amends:** decision R9 of [decision record 0007](0007-rl-arch-02-core-and-reference-simulation.md)
(message-ID derivation). Decision record 0007 and its rows are not changed; this record adds
the amendment as new decisions, as 0007 section 1 requires.

## 1. The problem

Core 0.1 section 3 numbers a sender's messages once per conversation, across all recipients,
and section 8.4 says a receiver holds a message that follows a gap. Those two rules do not fit
together. If arm-02 sends sequence 1 to amr-07, sequence 2 to planner-1 and sequence 3 to
amr-07, then amr-07 sees 1 and 3 and cannot know whether 2 was lost or went to someone else.
A receiver that holds on every gap would stall; one that ignores gaps cannot detect a lost or
overtaken message.

ARCH-2 (`robolang.arch02`) therefore holds nothing: it rejects a message whose sequence is
lower than one already processed from that sender (`C306`). That avoids stalling but cannot
put an overtaken message back in order. It was flagged as a follow-up in PR #27.

## 2. Decisions

| # | Topic | Proposal | Alternative considered | Spec | Status |
|---|---|---|---|---|---|
| Q1 | Sequence numbering | `sequence` counts per **sender, recipient and conversation**: each stream starts at 1 and increases by one per message the sender sends to that recipient in that conversation | Keep one counter per sender and conversation and give up gap detection; or one counter per sender across all conversations | Core 3 | PROPOSED |
| Q2 | Check order | Sequence handling comes after form, signature, freshness and recipient (Core 8.1 stages 1–3). A message rejected at those stages is never held and does not count in its stream | Handle order first: an unauthenticated message could then hold or advance a stream | Core 8.1 | PROPOSED |
| Q3 | Gaps | A receiver processes each stream in sequence order. A message whose sequence is above the next expected one is **held**: it has no protocol or physical effect, and no reply is sent, until it becomes eligible. When the gap is filled, held messages are processed in order at that moment | Process gaps immediately (no ordering); or reject a message after a gap | Core 8.4 | PROPOSED |
| Q4 | End of a hold (liveness) | A held message is processed when its gap is filled, or **`max_hold` after its delivery**, whichever is first; in the second case the missing sequence numbers are skipped. If the held message expires first (judged in the safe direction), it is rejected with `C301` at that moment, and the stream advances past it. `max_hold` is a deployment parameter, shorter than the message lifetime | Hold until the held message expires and then reject it: one lost message would also lose the next one, for example V7's `custody_state` (section 4) | Core 8.4; sim 3 | PROPOSED |
| Q5 | Duplicates | A message ID already processed **or currently held** from the same sender is a duplicate: no second effect, and the reply to the original, if any, is sent again. IDs are kept until the message's expiry plus the clock tolerance | Treat a copy of a held message as a new arrival: it could be held twice | Core 8.4 | PROPOSED |
| Q6 | Stale messages | A message whose sequence is at or below the last processed or skipped one in its stream, and which is not a duplicate, is rejected with `C306` | Accept it out of order | Core 8.2, 8.4 | PROPOSED |
| Q7 | Message IDs (amends R9) | The reference simulation derives `message_id` from sender, recipient, conversation and sequence, with a new domain tag (sim 5.2). Core still requires only that IDs are unique per sender; this is a simulation rule, as R9 was | Keep R9's formula: two streams of one sender would produce the same ID for the same sequence number | Sim 5.2 | PROPOSED |
| Q8 | Version | An erratum to Core 0.1, not Core 0.2. Core 0.x may change incompatibly (RFC-001 6.2), and no independent implementation exists yet (gate 2) | Core 0.2: every schema version, fixture and check changes for a rule no other implementation relies on | Core header | PROPOSED |
| Q9 | Simulation and tests | Scenario parameter `max_hold` (5 s); receipt value `held`; `inject` may set an explicit `sequence` and `message_id` (simulation only, to test stale and equivocating messages); eight sequencing cases in `tests/fixtures/arch02/sequencing_cases.json`, as acceptance test **A-T20**, added to the ARCH-3 exit gate | Leave sequencing to unit tests: no shared fixtures for gate 2 implementers | Sim 3, 9–12 | PROPOSED |

## 3. Acceptance condition

From the owner's scope: **a held message produces no protocol or physical effect until it is
eligible for processing**, and signature and freshness checks precede sequence handling.
The sequencing cases check both: the participant's processing order shows the held message
taking effect only when released (`processed_by`), and an unsigned message is rejected at
once rather than held (`seq_unsigned_not_held`).

## 4. Compatibility impact on ARCH-2

When this is implemented (a separate, authorized PR):

| Area | Effect |
|---|---|
| The 16 cases in `cases.json` | Every `expect` field is unchanged. 15 cases contain no gap, so no message is held |
| V7 (`v07_release_ack_lost`) | The dropped acknowledgement leaves a real gap in amr-07's stream to arm-02, so amr-07's `custody_state` is held for `max_hold`: arm-02 resolves `IN_DOUBT` to `RELEASED` at 10:15:01.300 instead of 10:14:56.300. Its expectations still hold. Without Q4's bound, the held `custody_state` would expire and be lost |
| Message documents | `sequence` values and `message_id` values change (new numbering and formula). No case expectation depends on them, and no run ID exists yet (ARCH-3), so nothing recorded changes |
| `tests/test_arch02.py` | `test_message_overtaken_by_a_later_one_is_rejected_as_out_of_order` describes the old rule; it is replaced by the hold-then-process behavior of `seq_gap_filled` |
| `robolang.core` (ARCH-1) | Unchanged: the per-message checks are stateless and do not read `sequence` |
| Core fixtures | Unchanged: their `sequence` and `message_id` values are valid under both rules |

## 5. Revision history

| Date | Change |
|---|---|
| 2026-10-27 | First draft: Q1–Q9 proposed |
