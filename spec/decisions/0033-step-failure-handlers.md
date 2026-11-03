# Decision record 0033: RoboLang 1.5, step failure handlers

**Date:** 2026-11-03
**Status:** **APPROVED by the owner, 2026-11-03**: F5 and F6 as the owner's options, the recommended option for F1–F4,
F7 and F8. Implemented after [0032](0032-holder-check.md). **IMPLEMENTED** in the front end, Task IR 1.4 and
simulation (record 1.4); the language reference and benchmark prompts still teach 1.4 (section 5).
**Applies to:** [`robolang-v1.5.md`](../robolang-v1.5.md), `grammar-v1.5.ebnf`, IR schema 1.4, the run record,
`simulation-v1.0.md`, `bridge.py`, `delegate_peer.py`, the language reference.
**Evidence:** in 1.4 any failed step fails the run, and the reference says error handling and retry cannot be
represented. A task cannot say "if the zone never clears, put the box back and tell the fleet".

## 1. Proposal

A `move`, `grasp`, `place`, `inspect` or `wait until` may end with an `else` block that runs only when that step fails.

```rob
robolang 1.5;

task handover(robot arm, robot mobile, object box, location src, location handoff, location dst, region zone) {
    goal { object_at box dst; }
    plan {
        move arm to src;
        grasp arm box else {
            communicate arm to "fleet" with "BOX_MISSING";
            fail "box not found at src";
        }
        move arm to handoff;
        place arm box at handoff;
        wait until region_clear zone timeout 30 s else {
            goal { object_at box src; }
            grasp arm box;
            move arm to src;
            place arm box at src;
            communicate arm to "fleet" with "HANDOFF_ABORTED";
        }
        move mobile to handoff;
        grasp mobile box;
        move mobile to dst;
        place mobile box at dst;
    }
}
```

## 2. Decisions (recommended option first unless marked as the owner's)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| F1 | World after a failed step | Unchanged: the simulator already checks requirements and scripted faults before effects. The handler starts from that world | — |
| F2 | Handler as a path | Like a branch: it may start with its own `goal`, must have a goal or end in `fail` (`E309`), counts toward the nesting limit (`E216`). Unless it ends in `fail`, the plan continues with the next step. The failed step's code (for example `S241`) is recorded as handled | — |
| F3 | Where a handler is required | Only on `wait until`, in 1.5 files: new error `E316`. The timeout already admits the wait can fail. Optional on every other step | Required on every step: almost every step can fail (readiness), so authors would add empty `fail` handlers everywhere |
| F4 | Run record | A success on a handler path records which path succeeded, so a fallback success is never read as the main one | — |
| F5 | Retry | **Owner: allowed.** A handler may repeat the failed step once (`grasp arm box else { grasp arm box; }`). It is bounded; the reference's rule that retry cannot be represented changes accordingly | Ban it |
| F6 | `delegate` peer plans | **Owner: banned in 1.5.** No handler on a peer robot's step after the custody point ([delegate v0.2](../delegate-v0.2.md) section 4) | Allow it: its interaction with leases and custody is not designed |
| F7 | Core export | Each handler goal is one more goal alternative; a task over `MAX_ALTERNATIVES` (16, [0028](0028-core-0-2-scope.md) Q2) is refused by `export-goal` and `delegate` as today | — |
| F8 | Holder check | [0032](0032-holder-check.md) treats a handler path as "the step had no effect" | — |

## 3. Needs the owner

Nothing: F1–F8 decided.

## 4. Correction to the example in section 1

The example cannot succeed when the zone does not clear. A handler's `goal` adds to the task goal, as F2 makes it
"like a branch", so that path needs the box both at `dst` and at `src` (`S301`). The plan also goes on after the
handler, so `mobile` then looks for the box at `handoff`. The implementation follows F2 as written. A handler can
retry (F5), report and go on, or clean up and `fail`. A **fallback outcome**, a handler that replaces the goal and
ends the run, is not expressible: it would need a new rule (a handler `goal` that replaces the goal of its path) and a
way to end a plan successfully. That is an open question for the owner.

## 5. Implementation notes

- Syntax: `<step> else { ... }` in place of `;`. In IR, `else` and `goal_else` on the step, both or neither.
- F2: the two sides of a handled step are named `ok` and `failed` in path labels (`E309`, Core export). Handler steps
  are numbered after the handled step, labelled `<step>.else.<n>`.
- F3: `E316`, and `I217` in the IR reader. `else` after `wait for`, `communicate` or `fail` is `E201`.
- F4: record 1.4 adds `handled` (the code taken over) to every step; a handler's goal entries have `from`
  `"<step>.else"`.
- F6: a task with a handler is a branching task. `delegate` already refuses a branching plan with two or more robots,
  so no peer step can have a handler. This is stricter than F6, which bans handlers only after the custody point.
- F8: a grasp that always fails is still `E315` when it has a handler.
- The ROS 2 backend refuses a handler with `S508`.
- Not in this change: the shipped language reference and the benchmark prompts. Moving them to 1.5 changes at least
  one benchmark label (`x_failover_on_fault`, "switch robots after a step fails", becomes expressible), so it needs
  benchmark errata ([0025](0025-bench-2.md)) and its own approval.
