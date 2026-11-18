# RoboLang Task IR — schema 1.9

**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.9`
**Produced from:** RoboLang language version `2.1` ([specification](robolang-v2.1.md))
**Builds on:** [Task IR schema 1.8](ir-schema-v1.8.md) and the schemas below it, unchanged.
Implemented in `src/robolang/ir.py` and read back by `src/robolang/ir_reader.py`.

Schema 1.9 is schema 1.8 plus incoming messages (decision [0043](decisions/0043-incoming-messages.md)): one more kind of
condition, `message_received`. Everything not stated here is as in schemas 1.0 to 1.8.

## 1. Version

- A 2.1 source **with tasks only** produces `"ir_schema_version": "1.9"` and `"language_version": "2.1"`.
- A 2.1 source **with a mission** produces a [Mission IR 1.1](mission-ir-v1.0.md) document, which holds Task IR 1.9 tasks.
- A 1.1 to 2.0 source still produces schemas 1.0 to 1.8 and Mission IR 1.0, byte-identical to before.

## 2. The condition `message_received`

```json
{ "kind": "message_received", "channel": "line", "text": "TUGGER DOCKED" }
{ "kind": "message_received", "channel": "nurse" }
```

- Keys, in this order: `kind`, `channel`, then `text` **only when the source has a `with`**. With no `text` key the condition
  is true for any message on the channel.
- `channel` and `text` are strings as written in the source, **not trimmed** (the match trims, section 2 of
  [robolang-v2.1.md](robolang-v2.1.md)); neither is a parameter name. Neither is empty, and `text` is not only spaces (`I207`).
- It stands wherever a condition stands (`preconditions`, the `condition` of an `if` or a `wait_until`, a mission's `until`, and
  under `and`, `or` and `not`), **never in a goal**: `goal`, `goal_then` and `goal_else` hold no `message_received`, however
  nested (`I228`).

## 3. Consumer duties

The duties of the earlier schemas apply. The `ir_reader` also checks, for schema 1.9, the rules of
[RoboLang 2.1](robolang-v2.1.md) section 3 on the IR: no `message_received` in a goal (`I228`, the form of `E327`), and an empty
`channel` or `text` (`I207`, the form of `E307`). Form errors: `I115` for a `message_received` in a schema older than 1.9;
`I110` for a value that is not a string; `I111` / `I112` for a missing or unknown key.

## 4. Consumers

A consumer that does not know `message_received` must refuse the task, not ignore it: the ROS 2 backend refuses with `S515`,
and `delegate` refuses a task that has one with `X608` (decision 0043 S8). `export-goal` is unaffected, as a goal holds no
message. Holding the IR authorizes nothing (rule LE-13).
