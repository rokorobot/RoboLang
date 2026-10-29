# Plan: verified outcomes in the language, and a bridge from `.rob` to Core

**Date:** 2026-10-29
**Status:** Approved by the owner 2026-10-29, with the recommendations for Q1–Q5. Phase 1 done 2026-10-29 (RoboLang 1.3, [0014](../../spec/decisions/0014-lang-2-goals-and-typed-sensors.md)).
**Fixes:** two gaps found in review.

1. **Outcome gap.** The charter makes success a verified outcome, but `.rob` could not declare
   one, and `simulate` reported `SUCCEEDED` when the steps had merely run.
2. **Two products.** `.rob` (lexer → IR → symbolic executor) and Core 0.1 / `arch02` (signed
   messages, custody, leases) share no code and no data. `.rob` does not compile to Core;
   `arch02` does not run `.rob`.

## The seam

Core 0.1 already says *where* the language plugs in, and the 1.3 goal block is shaped to fit it:

| `.rob` 1.3 goal atom | Core 0.1 triple (5.2) |
|---|---|
| `object_at box dst` | `{ subject: thing, property: located_in, value: place }` |
| `robot_at r dst` | `{ subject: robot, property: located_in, value: place }` |
| `holding r box` | `{ subject: thing, property: carried_by, value: robot }` |
| `@safety max_speed V m/s` | constraint `max_speed` (5.4). `rad/s` has no Core form |

Core decision R3 says plans never travel: robots run their own local plans. So the bridge is:

```text
AI writes .rob ──► goal + constraints ──► Core propose_goal ──► robot
                    (exported)                                   │ accepts, authorizes
                                                                 ▼
                                   robot runs the .rob plan locally (symbolic executor)
                                                                 │
AI ◄── Core outcome (verification + observation evidence) ◄──────┘
```

The goal crosses the wire, the plan stays with the robot, and the outcome comes back as
evidence. That is the same meaning on both sides, as RFC-001 intends.

## Phases

Each phase is one PR, with its own tests, and ends green. Phases 1–3 do not touch `arch02`, the
gate 1 evidence or the frozen gate 2 bundle.

### Phase 1 — Goals decide success (RoboLang 1.3) · done

- A required `goal { … }` block, with the outcome atoms `object_at`, `robot_at` and `holding`,
  and typed `sensor` parameters.
- After the plan, every goal condition is evaluated against the final world. All true →
  `SUCCEEDED`; any false → `FAILED S301`; any unknown → `FAILED S302`.
- IR 1.2 and run record 1.2 for 1.3 sources. 1.1 and 1.2 files are byte-identical to before.

**Exit:** `examples/pick_and_place_v13.rob` succeeds only when the box really ends at `dst`. A
plan that "runs" but leaves the box elsewhere fails with `S301`.

### Phase 2 — Core-shaped outcomes in run records

- `simulate` adds an `outcome` object to the run record of 1.3 sources, in the form of a Core
  0.1 `outcome` (5.9): `status` (`succeeded`, `failed`, `cancelled`, `unknown`), a
  `verification` with one `criteria_results` entry per goal condition, and the `evidence` it
  cites.
- Evidence comes from `observation` documents (5.6) that the simulator emits for the goal facts
  at the end of the run. They have `source.participant = rl:sim:world` and confidence `1`, and
  are labelled as simulator ground truth.
- The outcome object is checked with the existing `robolang.core` schema checks in tests.
- Run record 1.3. The run record for 1.1/1.2 sources is unchanged.

**Exit:** every 1.3 run record carries an outcome that passes the Core 0.1 schema, and `C402`
holds: `succeeded` is reported only when every criterion is met.

### Phase 3 — Export a `.rob` goal as a Core `propose_goal`

- New command: `robolang export-goal FILE --world W [--task T] [--bind …] --from AGENT --to ROBOT`.
  It prints a signed (simulation-only) Core 0.1 `propose_goal` message on stdout.
- Mapping:
  - goal atoms → `desired_state` and `success_criteria`;
  - `@safety max_speed … m/s` → constraint;
  - bound object IDs → `things`;
  - world IDs → Core identifiers (`box1` → `rl:thing:box1`, `table` → `rl:place:table`,
    `arm1` → `rl:robot:arm1`).
- Language addition (RoboLang 1.4): `@verify min_confidence 0.9 max_age 10s;`, which gives the
  success criteria their Core fields. Without it, export uses documented defaults.
- World file 1.1 (additive): optional `class` and `mass` on objects, needed for Core `things`.
- **Exportable subset:** a goal can be exported only if it is a plain conjunction of outcome
  atoms. `or`, `not`, `robot_ready`, `region_clear` and `sensor_ok` in a goal are valid locally
  but rejected by `export-goal` with a stable code. Validation itself is not affected.
- Every exported message passes `robolang.core.check_message`.

**Exit:** for each of the three examples, export → `check_message` accepts the message, and the
goal triples equal the `.rob` goal atoms.

### Phase 4 — Robots run `.rob` locally in the reference simulation

- A new scenario mode (`robolang arch02 --plans DIR`, or a separate `delegate` scenario). The
  existing scenario and its 13 failure variants stay byte-identical; the gate 1 evidence and the
  frozen gate 2 bundle are untouched.
- Each robot stub gets a local plan library of `.rob` tasks. When it receives an accepted and
  authorized goal, it:
  1. picks the task whose goal block matches the goal triples;
  2. binds the task to its own world view;
  3. runs it with the symbolic executor;
  4. reports the phase 2 outcome as the Core `outcome` message.
- The scripted motion in the arm and AMR stubs is replaced by plan execution for this mode only.

**Exit:** the planner proposes the goal exported from `handover.rob`, the robot runs the local
`.rob` plan, and the planner records `succeeded` with verification evidence. A world variant
where the plan leaves the box elsewhere ends `failed`, with criteria not met.

### Phase 5 — One command end to end, then resume interoperability

- `robolang delegate FILE --world W` runs phases 3 and 4 in one process and prints both the
  message log and the outcome.
- **CORE-1 (IR reader and re-validator)** follows here, so an AI can submit Task IR JSON
  instead of `.rob` source. It is not needed earlier, because plans stay local (R3).
- **INTEROP-3 is reconsidered** ([0017](../../spec/decisions/0017-reprioritise-goals-before-interop-3.md))
  with a gate 2 bundle v2 in which goals come from `.rob`. Gate 2 then tests the language end to
  end.

## Owner decisions (approved 2026-10-29: the recommendations)

| # | Question | Decision |
|---|---|---|
| Q1 | Who verifies in simulation: simulator ground truth (phase 2), or only the robot's own observations (`inspect`)? | Ground truth now, clearly labelled `rl:sim:world`. Robot-observation-only verification is Foundation 2 (Core-shaped observations) |
| Q2 | Verification parameters: language syntax (`@verify`, 1.4) or CLI defaults only? | `@verify` in the language. How success is checked is part of the goal's meaning, so it belongs in the source |
| Q3 | Goals that use `or` and `not`: keep them local-only, or extend Core? | Local-only in 0.1. Extend Core only when a real case needs it |
| Q4 | Phase 4: a new mode beside `arch02`, or replace its scripted motion? | New mode. The frozen gate 2 bundle and the gate 1 evidence stay valid |
| Q5 | Identifier mapping: derived by prefix (`rl:thing:<id>`), or declared in the world file? | Derived by prefix now. An optional explicit `core_id` per world entity comes later, if needed |

## Order and size

| Phase | Depends on | Rough size |
|---|---|---|
| 1 | — | done |
| 2 | 1 | small: record fields, observation synthesis, tests |
| 3 | 1, 2 (outcome shape); Q2, Q5 | medium: exporter, 1.4 `@verify`, world 1.1 |
| 4 | 2, 3; Q4 | large: plan-executing participants, new scenario cases |
| 5 | 4 | small CLI; CORE-1 and INTEROP-3 are separate items |

Phases 2 and 3 can run in parallel once phase 1 lands; phase 4 needs both.
