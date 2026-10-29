# Decision record 0014 — RoboLang 1.3: declared goals, outcome atoms, typed sensors

**Status:** decided and implemented together with [`spec/robolang-v1.3.md`](../robolang-v1.3.md).
**Applies to:** `robolang-v1.3.md`, `grammar-v1.3.ebnf`, `ir-schema-v1.2.md`, `simulation-v1.0.md`
section 10, `tests/fixtures/v1.3/`.

## Why

The project charter says success is a **verified outcome** and that execution alone does not establish
success. Yet a `.rob` task had only `pre` and `plan`, so `robolang simulate` reported `SUCCEEDED` when the
steps merely ran. And sensors were string literals naming global world IDs, unlike every other
entity, which is a typed, bound parameter.

## Decisions

| # | Decision |
|---|---|
| G1 | The marker `robolang 1.3;` selects 1.3, which is 1.2 plus the items below. Unmarked, 1.1 and 1.2 files keep their meaning, IR, run records and diagnostics byte for byte |
| G2 | A 1.3 task has a **required** `goal { condition; … }` block, after `pre` and before `plan`, with at least one condition. A missing goal is the static error `E309` |
| G3 | New atoms `object_at OBJ LOC`, `robot_at ROBOT LOC`, `holding ROBOT OBJ`, usable in `pre`, `goal` and `wait until`, type-checked like the existing atoms (`E303`, `E304`) and three-valued: `unknown` when the world fact is unknown. `object_at` is false while the object is held |
| G4 | New parameter type `sensor`. In 1.3, `sensor_ok S` and `inspect R O using S` take an identifier that must be a `sensor` parameter. The old string form is the static error `E310`, with a message that says to declare a `sensor` parameter. A world binds a `sensor` parameter to a declared sensor ID |
| G5 | `goal`, `object_at`, `robot_at`, `holding` and `sensor` are reserved in 1.3 files only |
| G6 | Task IR schema **1.2** and run record schema **1.2** for 1.3 sources. IR 1.0, IR 1.1 and run records 1.0, 1.1 are unchanged |
| G7 | After the last step succeeds, every goal condition is evaluated against the final state. All true → `SUCCEEDED`; any false → `FAILED` `S301` (phase `goal`); otherwise any unknown → `FAILED` `S302`. Results go in a `goal` array in the run record; a run that ended earlier records them as `not_evaluated` |

## Choices inside the decisions

- **Missing goal is static (`E309`), an empty goal block is syntax (`E208`).** The first is a fact
  about the task, the second is a malformed block, as an empty `plan` already is.
- **The quoted-sensor form is read, then rejected (`E310`)** instead of failing to parse, so the
  error can tell the author what to write.
- **IR sensor references.** The JSON shape of `sensor_ok` and `inspect` is unchanged; in schema 1.2 the
  value names a `sensor` parameter. A schema 1.2 task is recognised by its `goal`.
- **`false` beats `unknown`** when both occur in a goal: the outcome is known to be missed, so `S301`.
- **No new world-file schema.** A sensor parameter binds to a key of the existing `sensors` object.
- **Task IR 1.0 and 1.1 gain no `goal` key**, so existing snapshots do not change.

## Not decided here

Goal reachability checking, goal-driven retries or recovery, and mapping goals onto Core success criteria.
