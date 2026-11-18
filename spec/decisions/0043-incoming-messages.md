# Decision record 0043: Incoming messages (I5, G-MSG, RoboLang 2.1)

**Date:** 2026-11-18
**Status:** **APPROVED by the owner, 2026-11-18**: S1–S11 as recommended. A message counts from the start of the task
run (S2); senders are declared in the world (S4); confirmed delivery stays out of 2.1 (S7). This is step L2 of
iteration I5 in the [language roadmap](../../docs/LANGUAGE_ROADMAP.md).
**Applies to (when decided):**
- new `spec/robolang-v2.1.md` and `grammar-v2.1.ebnf`;
- Task IR 1.9 and world file 1.6;
- `simulation-v1.0.md`;
- `src/robolang/`;
- `docs/language-reference.md`;
- the ROS 2 backend's refusal list;
- `bench/` (the L3 set).

**Evidence:**
- **Atlas.** G-MSG is the first open gap in the generated
  [suggested order](../../docs/plans/humanoid-activities.md#suggested-order-of-language-work). Sixteen
  sub-activities need it. Five of them are near: sequencing, line feeding, hospital delivery, EV charging and solar
  installation.
- **R3 probes.** Three probes: `f_msg_line_docked` ("as soon as the line controller sends TUGGER DOCKED"),
  `f_msg_nurse_confirm` ("wait until the nurse confirms") and `f_msg_wind_clear` ("hold it until the site sends WIND
  CLEAR"). Each attempt fails (E211/E204), and both models declined all three.
- **Standing tasks.** `x_wait_message`, `u11_wait_go`, `u12_radio_clearance`, `x_wait_ready_message` and
  `f12_message_delivery`.

## The problem

A task can send (`communicate`) but cannot react to anything it receives. `message_received` has been a reserved
word since 1.1, waiting for a message protocol. People and systems work with robots by message: a line controller
says "docked", a nurse confirms, a site says "wind clear". Without this, those requests cannot be written.

## The idea in one paragraph

Receiving a message becomes a **condition**, `message_received "channel" with "text"`. It can stand wherever a
condition stands today:

- in a bounded `wait until` ("as soon as it says DOCKED");
- in an `if` ("only if the nurse has confirmed");
- in a mission's `until` ("until the line says SHIFT END").

Who may send on a channel is not written in the task. It is **declared in the world**, as the robot's abilities are,
so trust stays in the integrator's configuration. There is no new statement, and the reserved word is finally used.

```rob
robolang 2.1;

task feed(robot r, object tote, location handoff, location line) {
    goal { object_at tote line; }
    plan {
        move r to handoff; grasp r tote;
        wait until message_received "line" with "TUGGER DOCKED" timeout 10 min else { fail "no dock signal"; }
        move r to line; place r tote at line;
    }
}
```

## Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| S1 | Construct | A condition `message_received "CH" with "TEXT"` (an exact text), and `message_received "CH"` (any message on the channel). Used in `pre`, `if`, `wait until` and a mission's `until`; never in `goal`, because a message is not an end state of the world. One new word, `with`, already exists | A statement `receive r from "CH" as m;` that binds the message: its content is text that nothing can use without G-CALC. Kept for later |
| S2 | What counts | A message counts once it has arrived **after the task run started**, and it stays counted for the rest of the run (latched). In a mission, each task run starts fresh, and the mission's `until` sees messages since the mission started | Only messages that arrive during the wait: "as soon as it says DOCKED" sent while the robot was still driving would be missed |
| S3 | Text match | Exact and case-sensitive, after trimming spaces at both ends. No patterns | Patterns or "contains": ambiguous, invites accidental matches |
| S4 | Who may send | The world (and later the site file) declares each channel and its allowed senders: `"channels": { "line": { "from": ["rl:agent:line-ctl"] } }`. A message from anyone else is ignored, and the record notes it. A task naming an undeclared channel fails before the run (new S-code) | A `from` in the task: the task author would choose whom to trust, and that is the integrator's job (as `honours` is, 0039 P6) |
| S5 | Unknown | A channel that is declared but has no message yet is `false`, not unknown: there is no message. A channel whose state cannot be known (no message service in the world) makes the condition `unknown`, which fails closed | Treat "no message" as unknown: every wait would fail closed at once |
| S6 | Simulation | World file 1.6: `messages` as events `{ "at", "channel", "text", "from" }`, applied in time order like other events. The record lists every received message and whether it was accepted | — |
| S7 | Delivery of what the robot sends | Not in 2.1. "Make sure it was delivered" stays inexpressible: delivery is the transport's job. "Wait until the nurse confirms" is a reply message and is expressible (S1) | Track delivery inside RoboLang: a transport feature, not a language one |
| S8 | Backends and Core | The ROS 2 / Gazebo backend refuses `message_received` until a message service is defined. `export-goal` is unaffected, since goals hold no messages. `delegate` refuses a task that waits for a message (new code): the robot would need the channel's trust configuration | Pass messages through Core: Core 0.2 is frozen |
| S9 | Safety | A message can now change what a robot does, so a spoofed "WIND CLEAR" is a hazard: **H23** in the safety case. Controls: senders are declared per channel (S4), and in physical mode only authenticated senders are accepted (G4, real identity in Core). Simulation only in 2.1 | Trust any sender: one forged message could release a load |
| S10 | Not in 2.1 | The content of a message as a value (S1 alternative), "for each message" (a mission trigger, G-TRIGGER), delivery confirmation (S7), and messages between robots in the same task (a robot's own `communicate` is not a received message) | — |
| S11 | Design for the far needs | Message triggers on missions (`when message_received …`, G-TRIGGER) will reuse S1's condition. Content as a value will extend S1 with `as NAME`. Negotiation between robots (cooperation) needs both | — |

## What it makes expressible

- **The three R3 probes.**
- **Standing tasks:** `x_wait_message`, `u11_wait_go`, `u12_radio_clearance`, `x_wait_ready_message`.
- **Missions:** "until the line says SHIFT END".
- **Stays out:** `f12_message_delivery` (S7) and `x03_for_each_go_message` (a trigger).

## Pre-registration (L3), before any code

- **Set 10 "messages":** the probes and standing tasks above, relabelled for 2.1, plus new tasks of four kinds:
  - **programs:** wait for a message and then act; `if` a message has arrived; a mission `until` a message; a
    message that arrived before the wait (S2); any message on a channel; a decoy (a near-miss channel name or text);
  - **infeasible:** the channel is undeclared; the sender is not allowed and no other message comes; the timeout is
    shorter than the message's arrival;
  - **inexpressible:** confirmed delivery; for each message; a number read from a message; a pattern match;
  - **controls.**
- **Bounds:** fixed with the baseline (rule 6), including a B8-like measure: **a message wait replaced silently**, for
  example by a guessed `wait for`, a `region_clear` stand-in, or nothing.
- **Economy:** 0 new statements, 1 condition (with a reserved word that finally gets used), 1 world field (`channels`,
  plus `messages` events) and about 4 codes.

## Owner's answers (2026-11-18)

The owner accepted every recommendation: S2 from the start of the run, S4 senders in the world, and S7 delivery
out of 2.1. S1–S11 are accepted.
