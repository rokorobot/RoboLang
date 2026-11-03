# Decision record 0033: RoboLang 1.5, step failure handlers

**Date:** 2026-11-03
**Status:** **APPROVED by the owner, 2026-11-03**: F5 and F6 as the owner's options, the recommended option for F1–F4,
F7 and F8. Implemented after [0032](0032-holder-check.md).
**Applies to:** a new `robolang-v1.5.md` and `grammar-v1.5.ebnf`, IR schema 1.4, the run record,
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
