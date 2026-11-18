# RoboLang 2.1: incoming messages

**Status:** front end, Task IR 1.9 (Mission IR 1.1 for a file with a mission), world file 1.6, the simulation (run record
1.9, mission record 1.1) and the IR reader complete in code. The language reference and the benchmark prompts teach 2.1 (L4 of
decision 0043); the acceptance run on benchmark set 10 is the next step (L5), and until it passes the bounds B1 to B9 2.1 is not
accepted.
Decision: [0043](decisions/0043-incoming-messages.md) (S1 to S11, and clarifications C1 to C14 approved 2026-11-18).
Grammar: [grammar-v2.1.ebnf](grammar-v2.1.ebnf). IR: [Task IR 1.9](ir-schema-v1.9.md) and [Mission IR 1.1](mission-ir-v1.0.md).
Simulation: [section 19](simulation-v1.0.md). Everything not stated here is exactly as in [RoboLang 2.0](robolang-v2.0.md): a
file may hold tasks and missions as before.

## 1. Version

`robolang 2.1;` selects 2.1. A file with this marker is read with the 2.0 rules, and may also use the condition `message_received`
(section 2). A 2.0 front end rejects the marker with `E213`. Files with no marker or with `1.1` to `2.0` are read, validated,
compiled and run exactly as before, byte for byte; **in them `message_received` is still the reserved word it has been since 1.1, and
using it is `E211`.**

**No new reserved word.** `message_received` has been reserved since 1.1, and `with` is a keyword already. One new condition
(`message_received`), no new statement, no new top-level declaration. One new world field `channels` and the `messages` events
(world file 1.6).

## 2. The condition `message_received`

```rob
robolang 2.1;

task feed(robot r, object tote, location src, location handoff, location line) {
    goal { object_at tote line; }
    plan {
        move r to src; grasp r tote; move r to handoff;
        wait until message_received "line" with "TUGGER DOCKED" timeout 10 min else { fail "no dock signal"; }
        move r to line; place r tote at line;
    }
}
```

- `message_received "CHANNEL" with "TEXT"` is true once a message with exactly that text has arrived on that channel, from a
  sender the world allows on it. `message_received "CHANNEL"` is true once **any** message has arrived on the channel (C9: a
  request that names no text means any message).
- It is a **condition** and stands wherever a condition does, except a goal (S1, C10): in a `pre`, in an `if`, in a `wait until`
  (with the `timeout` and `else` that every wait has since 1.5), and in a mission's `until`; and it combines with `and`, `or`,
  `not` and the other atoms. `not message_received "ops" with "STOP"` is allowed.
- **Never in a `goal`** (S1): a message is not an end state of the world, and a goal is what `export-goal` sends to Core. The
  `goal` of a task or of an `if` branch with a `message_received`, however nested, is `E327`. "The job is done only when the line
  has sent RECEIVED" is written as a last `wait until message_received "line" with "RECEIVED" timeout T else { fail "..."; }` in
  the plan, with the world part in the goal (C14).
- The channel and the text are string literals: an empty channel, an empty text, or a text of spaces only is `E307`. They are
  not names and not values.
- **Who may send is not in the task** (S4). It is declared by the world: each channel lists its allowed senders. A message from
  anyone else is ignored. A task cannot choose whom to trust.
- **What counts** (S2, S3): a message counts once it has arrived at or after the start of the task run, and stays counted for the
  rest of the run (latched); the text is compared exactly and case-sensitively after trimming spaces at both ends of both
  texts. A message from before the run does not count. A message from a robot of the task, sent by its own `communicate`, is never
  a received message (C6): a message is only a world event.
- **Value** (S5, C1, C10): a declared channel with no message yet is false, not unknown. A world with no `channels` has no
  message service: every `message_received` is unknown there and fails closed, so a wait times out (`S241`), an `if` fails
  (`S261`), a `pre` fails (`S102`), and `not` of it is unknown too. A task that names a channel the world does not declare, in a
  world that declares channels, fails **before the run** (`S011`).
- **A wait** ends at the arrival of the message, or at once if it has arrived, and a message at exactly the timeout counts (C5).
  The rest of section 19 of the simulation spec holds the details, including a mission (C8: each task run starts fresh; the
  mission's `until` sees the messages since the mission started).

### 2.1 What 2.1 cannot say

- **Delivery** (S7, G-DELIVERY): "make sure it was delivered", "check that it was received" about what the robot sends. A task
  cannot see whether its own `communicate` got through: delivery is the transport's job, not the language's. "Wait until the
  nurse confirms" is a reply message and is expressible.
- **"For each message"** and any trigger (S10, G-TRIGGER): "each time the line says READY", "whenever a message arrives". A mission
  starts a run when the previous one is done, never when a message comes; a loop over messages is a later version.
- **The content of a message as a value** (S10, G-CALC): "forward the order number", "read the number it sends", "do what the
  message says". A message is matched, never read: there is no binding for it.
- **Patterns** (S3): "contains READY", "starts with GO", "any message from the line that mentions the door". A text is exact. A
  program that waits for one exact text in place of a pattern is a false program.
- **A message in a goal** (S1) as an end state, and **messages between robots of one task** (S10).
- **Who to trust** (S4): the sender is not chosen in the task. A request that names a sender the world does not allow on that
  channel, or a channel the world does not declare, is infeasible, not a program.

## 3. Static rules (`validate`)

1. `message_received` takes a channel in quotes, and optionally `with` and a text in quotes: nothing else (`E201`). An empty
   channel, an empty text or a text of spaces only is `E307`.
2. A `goal` (of a task, or of an `if` branch) has no `message_received` (`E327`).
3. In a file with a version below `2.1`, `message_received` is `E211`.

Before a run, and in `validate --world`: a world that declares channels does not declare the channel the task names (`S011`).

## 4. Task IR and Mission IR

A 2.1 file compiles to **Task IR 1.9** ([ir-schema-v1.9.md](ir-schema-v1.9.md)) with `language_version` `2.1`: one new kind of
condition, `message_received`. A file with a mission compiles to **Mission IR 1.1**, which holds Task IR 1.9 tasks
([mission-ir-v1.0.md](mission-ir-v1.0.md)); a 2.0 file keeps Mission IR 1.0. The reader re-validates both in full
([ir-reader-v0.1.md](ir-reader-v0.1.md)).

## 5. Elsewhere (S8, S9)

- **Core is unchanged** and a goal holds no message: `export-goal` is unaffected.
- **`delegate` refuses a task that has a `message_received`** with **`X608`** (exit 6), and does not use such a library task: the
  robot would need the channel's trust configuration, which is the integrator's.
- **The ROS 2 backend refuses `message_received`** with **`S515`**, before any motion: it has no message service.
- **Safety** (S9): a message can now change what a robot does, so a spoofed "WIND CLEAR" is a hazard, **H23** "a spoofed message
  changes what a robot does", in the [safety case](../docs/safety-case-draft.md). Controls: senders are declared per channel
  (S4); in physical mode only authenticated senders are accepted (real identity in Core, G4); simulation only in 2.1. Nothing in
  this package authenticates a sender: the world file says who is allowed, and a simulated message is a world event.

## 6. Diagnostics

| Code | Stage | Meaning |
|---|---|---|
| `E201` | syntax | (existing) a `message_received` with no channel in quotes, or `with` with no text in quotes |
| `E211` | syntax | (existing) `message_received` in a file below 2.1: it is reserved, and is not part of that version |
| `E307` | static | (existing) an empty channel, an empty text, or a text of spaces only |
| `E327` | static | A `message_received` in a goal |
| `E519` | world file | A `channels` or `messages` entry that makes no sense (simulation section 3.9 and 19.1) |
| `S011` | run, `validate --world` | A task names a channel that a world with channels does not declare (simulation 19.3) |
| `X608` | delegate | A task with a `message_received` is not delegated (S8) |
| `S515` | ROS 2 backend | A task with a `message_received` is refused before any motion (S8) |

`E213` lists `1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0 and 2.1`. In the IR reader the new form is `I228` (`E327`), and
`I207` applies to an empty text (`E307`).

## 7. Compatibility

Nothing changes for 1.1 to 2.0 files, their IR, their run records or their world files. A world file 1.0 to 1.5 serves a 2.1 task
as before; it has no message service, so a `message_received` is unknown there and fails closed. World file 1.6 is 1.5 plus
`channels` and `messages` and nothing else.
