# `robolang export-goal` v0.1 — a `.rob` goal as a Core `propose_goal`

**Status:** implemented in `src/robolang/export_goal.py`; decision [0019](decisions/0019-export-goal.md).
**Plan:** [`docs/plans/goals-and-core-bridge.md`](../docs/plans/goals-and-core-bridge.md), phase 3.
**Builds on:** [RoboLang 1.3](robolang-v1.3.md), [simulation spec](simulation-v1.0.md) section 3
(world file 1.1) and [Core 0.1](core-v0.1.md) sections 3, 4.1, 5 and 7.

> **Simulation only.** The command builds one message and prints it. Nothing is sent. The
> signature is `sim-hmac-sha256` under a key anyone can compute, so it shows that a check happens,
> not who signed. A message that `check_message` accepts is well-formed and attributable, never
> permission to act (Core 0.1, preface): the robot decides under its own policy.

## 1. Command

```text
robolang export-goal FILE --world WORLD [--task NAME] [--bind NAME=ID ...]
                     --from AGENT_ID [--to ROBOT_ID] [--at TIME]
```

`FILE` must be RoboLang 1.3 (earlier sources have no goal). Task selection, `--bind` and the world
file are exactly those of `simulate` (simulation 2.5, 3.5): the world file's bindings for the task,
or only the `--bind` ones. The world is validated as for `simulate`, so the same errors apply.

| Option | Meaning |
|---|---|
| `--from` | The sender and `requested_by`: a Core identifier without `#`, e.g. `rl:agent:planner-1` |
| `--to` | The recipient. Default: the Core ID of the robot bound to the task's **only** `robot` parameter. Required when the task has none or several |
| `--at` | `sent_at`, a Core time (core 2.4). Default `2000-01-01T00:00:00.000Z`, the simulation epoch (`bridge.SIM_EPOCH`) |

Output: the message as JSON on stdout (two-space indent, ASCII, fixed key order, trailing newline).
On stderr: warnings and one line `simulation only: exported <goal id> as <message id>`.

## 2. Mapping

| Core field | Source |
|---|---|
| `payload.goal.id` | `rl:goal:<task name>` |
| `goal.requested_by` | `--from` |
| `goal.desired_state` | `bridge.goal_triples`: `object_at` and `robot_at` → `located_in`, `holding` → `carried_by`, in goal order |
| `goal.success_criteria` | One `observed` criterion per triple, same order (criterion *i* is goal condition *i*) |
| criterion `min_confidence` | `@verify min_confidence`, else **`"0.9"`** |
| criterion `max_age` | `@verify max_age`, else **`{ "value": "10", "unit": "s" }`** |
| criterion `accepted_sources` | `["any_participant_in_domain"]` |
| `goal.constraints` | Each `@safety max_speed V m/s` → `{ "kind": "max_speed", "limit": { "value": V, "unit": "m/s" } }`. A `rad/s` limit has no Core form and a unitless one is deprecated: both are skipped with a warning on stderr |
| `goal.priority` | `"normal"` |
| `payload.things` | One `thing` per distinct object bound in the task, in parameter order: `id` `rl:thing:<id>`, `class` and `mass` (with `uncertainty`) from the world object |

Entity IDs become Core identifiers by prefix (`bridge.core_id`): `box1` → `rl:thing:box1`,
`table` → `rl:place:table`, `arm1` → `rl:robot:arm1`.

**Exportable goals.** A goal is exportable when every condition is `object_at`, `robot_at` or
`holding`. Any other condition (`or`, `not`, `and`, `robot_ready`, `region_clear`, `object_exists`,
`sensor_ok`) is valid locally but has no Core 0.1 form (plan decision Q3).

## 3. Envelope

Deterministic: the same inputs give byte-identical output.

| Field | Value |
|---|---|
| `message_id` | `m-` + the first 32 hex digits of *H* |
| `conversation_id` | `rl:conv:` + the next 16 hex digits of *H* |
| `sequence`, `in_reply_to`, `profile`, `type` | `1`, `null`, `"a2r"`, `"propose_goal"` |
| `sender`, `recipient` | `--from`, the resolved recipient |
| `sent_at`, `expires_at` | `--at` or the epoch default; `expires_at` is `sent_at` + 30 s |
| `signature` | `sim-hmac-sha256`, key `<from>#k1` (section 4) |

*H* is SHA-256 of the RFC 8785 canonical form of `[sender, recipient, payload]`. `sent_at` is not
in *H*, so a re-export at another time repeats the IDs (a retransmission, Core 8.4).

## 4. Simulation key

The secret of key `<participant>#k1` is `SHA-256("robolang export-goal simulation key 1:" + "<participant>#k1")`
(UTF-8). It is public by design. A receiver that wants to accept the message lists that key for the
sender in its context (`robolang.export_goal.simulation_key`). Unlike the secrets in
`tests/fixtures/core/context.json` and the `arch02` scenario, which are arbitrary values the
deployment hands out, this one needs no configuration file.

## 5. Exit codes and diagnostics

| Code | Meaning |
|---|---|
| 0 | The message is on stdout |
| 1 | `FILE` is not valid RoboLang (diagnostics on stderr) |
| 2 | Usage: bad arguments or `--bind`, unreadable file, unknown or ambiguous `--task`, invalid `--from`, `--to` or `--at`, several robots (or none) and no `--to` |
| 3 | The world file or bindings are not valid |
| 6 | The goal cannot be exported |

Exit 6 prints one diagnostic, `FILE: error[CODE]: task 'T': …`:

| Code | Meaning |
|---|---|
| `X601` | A goal condition has no Core form. Names the condition's index and kind, and its text: `goal condition 2 ('or') has no RoboLang Core 0.1 form (a or b)`. A source without a goal (1.1, 1.2) reports `no goal` |
| `X602` | A bound object has no `class` or no `mass` in the world file (needs `world_schema_version` 1.1) |

Warnings (skipped limits) never change the exit status.
