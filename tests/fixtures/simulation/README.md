# RL-BASIC-01 simulation fixtures

Acceptance fixtures for the [simulation specification](../../../spec/simulation-v1.0.md)
(RL-BASIC-01a). They describe what a conforming `robolang simulate` must do. **No simulator
exists yet**: RL-BASIC-01b will run these cases.

Until then, `tests/test_simulation_fixtures.py` checks only that the fixtures are consistent
with the specification and with the existing front end: sources compile, bindings match task
parameters, indices are in range, codes are defined, and the run-ID test vectors follow
section 6. It executes nothing.

These fixtures are separate from the 76 RoboLang 1.1 conformance fixtures in `valid/` and
`invalid/`, which are unchanged.

## Layout

- `sources/` — `.rob` tasks written for these cases. All are valid RoboLang 1.1. The happy
  path uses [`examples/pick_and_place.rob`](../../../examples/pick_and_place.rob) itself.
- `worlds/` — world files (schema 1.0). `base.json` is a complete, consistent world with an
  entry for every task above; every other file differs from it in one respect, named by the
  file. Files whose case expects exit status 3 contain exactly one error.
- `cases.json` — the cases.

## Cases

Each case has:

| Field | Meaning |
|---|---|
| `id` | Unique name, listed in specification section 8 |
| `tests` | The acceptance tests (T1–T17) it covers |
| `source` | Path of the `.rob` file, from the repository root |
| `source_glob` | Instead of `source`: every file matching this pattern is a separate run with the same expectation (T2) |
| `task` | Value of `--task`, or `null` to omit it |
| `world` | Path of the world file, from the repository root |
| `seed` | Value of `--seed` |
| `repeat` | Optional. Run this many times; the run records must be byte-identical (T11) |
| `expect` | What must happen (below) |

`expect` fields. A field that is absent is not checked.

| Field | Checked against |
|---|---|
| `exit` | Exit status (specification 2.3) |
| `diagnostic` | Code of the world-file error reported (exit status 3) |
| `states`, `state`, `failure`, `cancelled_before_step`, `end_t_ms`, `run_id` | The run-record fields of the same name, exactly |
| `preconditions` | The `result` of each entry of `preconditions`, in order |
| `steps` | The `status` of each entry of `steps`, in order |
| `step_times` | `[t_start_ms, t_end_ms]` of each step, or `null` for a step that did not run |
| `safety_limits` | The `status` of each entry of `safety_limits`, in order |
| `messages` | The non-null `message` values of `steps`, in order |
| `observations` | The non-null `observation` values of `steps`, in order |
| `events_applied` | The run-record field, exactly |
| `final_world` | A subset of `final_world`: every entity and fact given must match; others are not checked |

For statuses 1 to 3 no run record may be written.
