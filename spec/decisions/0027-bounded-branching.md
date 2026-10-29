# Decision record 0027: RoboLang 1.4, bounded branching (N4)

**Date:** 2026-10-29
**Status:** **APPROVED by the owner, 2026-10-29**, with the recommended option for B1–B9. Design only: no grammar, parser,
IR or simulator change yet. Next is benchmark set 3, pre-registered and run against 1.3 (section 3), then code.
**Applies to (when decided):** a new `robolang-v1.4.md` and `grammar-v1.4.ebnf`, IR schema 1.3, run record 1.3,
`simulation-v1.0.md`, the language reference, `validate --world` ([0026](0026-validate-world.md))
**Evidence:** [set 2 analysis](../../docs/evidence/bench-set2-analysis.md) F3. A model silently dropped half of
"to shelf_a if the cell is clear, else to the table" and wrote a valid program that succeeds in half the worlds.
The tasks `x_route_by_cell`, `x_whichever_free` and `x_missing_box_branch` are declined today only because the
language has no run-time choice.

## 1. What changes

A plan may contain `if C { … } else { … }`. `C` is a condition (the same atoms, `and`, `or`, `not`). It is
evaluated **once, when the `if` is reached**, against the state at that moment. Each branch may add goal
conditions for its own path. No loops, no retry, no variables: those stay with the orchestrator and remain a
correct decline.

```
robolang 1.4;

task route(robot r, object box, location src, location a, location b, region cell) {
    goal { not holding r box; }                 // common to every path (optional, see B2)
    plan {
        move r to src;
        grasp r box;
        if region_clear cell {
            goal { object_at box a; }
            move r to a;
            place r box at a;
        } else {
            goal { object_at box b; }
            move r to b;
            place r box at b;
        }
    }
}
```

`x_whichever_free` becomes `if robot_ready arm1 { … with r1 … } else { … with r2 … }`, with both robots bound
as parameters and one common goal. `x_missing_box_branch` needs B4.

## 2. Decisions (the recommended column was approved)

| # | Question | Recommended | Alternatives |
|---|---|---|---|
| B1 | Syntax | `if C { [goal] statements } [else { [goal] statements }]` as a statement; `else if` is shorthand for a nested `if` | `when … otherwise`: no gain, and every model already knows `if` |
| B2 | Where goals live | The **effective goal of a path** is the task's `goal` plus the `goal` blocks of the branches it takes. The task `goal` becomes optional in 1.4, but every path must have at least one goal condition (E309 extended) | One task goal with `if` inside it: it would be evaluated at the end, when `C` may have changed, so the goal would no longer say which choice was made |
| B3 | Unknown condition | `C` unknown (three-valued) → the run fails, new code `S261`, before either branch runs | Treat unknown as false (take `else`): it silently picks a branch on missing evidence, which goes against the charter |
| B4 | A branch with no physical outcome ("if missing, move nothing and report") | New statement `fail "reason";`, allowed only as the last statement of a branch. The run ends `FAILED`, code `S262`, with the reason in the record. Messages sent before it stand. Core status `failed`, reason `declared` | (a) Force a goal such as `not object_exists box`: it is true but does not say "reported", and it blurs success with failure. (b) Leave this task inexpressible: it is a common operator request |
| B5 | Bounds | `if` nesting depth at most 4 (E216), so a task has at most 2⁴ paths. No `if` inside `wait until` or `pre` | No limit: the number of paths, and so static checks and tests, grows without bound |
| B6 | Static checks | Every path is checked as a straight-line plan: `validate --world` reports a certain failure on a path as an error naming the path (`path then/else`). A branch that can never succeed is an authoring error | Warnings only: a dead branch is the half-program F3 warned about |
| B7 | Core | `export-goal` of a task whose paths have different goals is **not exportable** in 1.4 (exit 6), until Core 0.2 has "any of" desired states (N5). A simulated run's `core_outcome` reports the **taken** path's goal, with the decision as evidence | Export only the common goal: the robot would not be told the real target |
| B8 | IR and record | IR 1.3: an `if` step `{kind, condition, then, else, goal_then, goal_else}`. Record 1.3: one `decision` per `if` reached (condition result, branch, time); steps carry their path. Scripted faults (world file) keep addressing steps by index in source order, counting through both branches. 1.1–1.3 files are unchanged byte for byte | Flatten branches into guarded steps: smaller schema, but the record could no longer show which choice was made |
| B9 | Reserved words (1.4 only) | `if`, `else`, `fail` | — |

## 3. How it is judged (strategy rule 1: evidence before syntax)

1. **Before code:** benchmark set 3 is written and pre-registered against 1.3. It has about 20 branching tasks
   from the operator requests above (route by state, pick the robot that is ready, report and stop), plus
   controls that must stay declines (retry, loops, counts, clocks, waiting for messages). The baseline run
   expects declines.
2. **After code:** the same set, with the 1.4 reference. Accept 1.4 if set 3 goal correctness is ≥ 90%
   (Sonnet) **and** false programs on the still-inexpressible controls stay ≤ 5% even with the grammar only
   (horizon H2 exit criteria). Sets 1 and 2 must not regress (strategy rule 2).
3. The language reference and `bench/prompt.md` are updated in the same PR as the grammar, as #64 requires.

## 4. Consequences

- A 1.4 file is a new language version. Nothing changes for 1.1–1.3 files, records or Core exports.
- N5 (Core 0.2, disjunctive goals) gets its first real producer: a branching task's goal is
  "any of" its paths' goals. The N5 spec is written after B7 is decided.
- The simulator stays deterministic: the taken branch depends only on the state when the `if` is reached.
- Size: grammar, parser, validator, IR, simulator, record, reference, set 3. Two to three PRs: (1) language and
  IR, (2) simulation and record, (3) `validate --world` paths and set 3.

## 5. Implemented in

Simulation, run record 1.3 and the Core and export rules (B2, B3, B4, B7, B8): N4 PR 2, see
[simulation-v1.0.md section 12](../simulation-v1.0.md). Reading of B4: Core 0.1 has no reason code `declared`, so the
Core outcome of a `fail` has `reasons: []` and the reason is in the run record. Not yet: B6 path checks and set 3
scoring (PR 3).
