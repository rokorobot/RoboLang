# Decision record 0041: The mission layer (G-LOOP, RoboLang 2.0)

**Date:** 2026-11-17
**Status:** **APPROVED by the owner, 2026-11-17**, design only: **G-LOOP is reopened** (LE-9), with the mission layer
as its design. Every repeat has a hard bound (M2); 2.0 has `stop`, `continue` and `retry N` (M4); M1–M10 are as
recommended. It is built at iteration I4 of the [language roadmap](../../docs/LANGUAGE_ROADMAP.md), after I2 and I3.
**Applies to (when decided, at I4):** a new layer in `spec/robolang-v2.0.md`, a mission record, `simulate`,
`docs/language-reference.md`, and benchmark set 8. Tasks, Task IR and Core 0.2 are unchanged (M8).
**Evidence:**
- **Atlas.** G-LOOP has the highest priority of all gaps (8.21). 11 of the 13 near sub-activities need it, and 21 of
  the 40 overall ([plan](../../docs/plans/humanoid-activities.md)).
- **Standing tasks.** 12 standing benchmark tasks need it, and the models decline them correctly (for example
  `x_every_hour`, `f10_retry_until_works`, `x_shuttle_until_five`).
- **Sub-activities.** Shift work, patrols, harvesting and cleaning rounds are all repetition.

## Why N4 kept loops out, and what changes

N4 kept unbounded repetition out of **tasks**. Its reason was that a task must end with a goal that can be verified;
a loop has no end state to check. That reason still holds. So repetition is not added to tasks. A **mission** is a
new layer above them: it runs **tasks**, one run after another. Every task run is a complete, bounded, verified run,
exactly as today. The mission decides only whether to start the next run. The orchestrator that N4 pointed to
becomes declarative and checkable, so it can be written in RoboLang.

```rob
robolang 2.0;

task feed_press(robot r, object blank, location rack, location press) {
    pre  { object_exists blank; }
    goal { object_at blank press; }
    plan { move r to rack; grasp r blank; move r to press; place r blank at press; }
}

mission keep_press_fed(robot r, location rack, location press) {
    repeat at most 200 runs, for at most 8 h {
        run feed_press(r, next_blank, rack, press);
    } until not object_exists next_blank;
    on failure { stop; }
}
```

The example is illustrative. How the next blank is named is G-CHOOSE (I7). Until then, a mission repeats a task
whose parameters are fixed or come from a list declared in the world.

## Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| M1 | Where repetition lives | A new top-level `mission`. It runs tasks, and tasks stay as they are | Loops inside tasks: no end state to verify, which was N4's reason |
| M2 | Bounds | **Every repeat has a hard bound**, a run count, a duration or both, so "forever" stays inexpressible (as `wait until` always has a timeout, E209). An operator stop and a lease end a mission at any time | No bound, relying on the operator: a mission that never ends cannot be approved or reviewed (safety case G6) |
| M3 | Ending | `until CONDITION`, checked after each task run, using the conditions tasks already use. It is unknown when unobserved, so the mission stops (fails closed) | Only the bound: "until the bin is empty" could not be said |
| M4 | Failure policy | `on failure { stop; }` by default. Also allowed: `continue` (log it and go on) and `retry N` (at most N more runs of the same task). A failed run is never counted as done | Always stop: shift work could not skip one bad part. Unlimited retry: the `f10` problem again |
| M5 | Outcome | The mission record lists every task run with its verified outcome. The mission ends `completed` (its `until` holds, verified), `bounded` (the bound was reached), `failed` or `stopped`. It never claims more than its task runs verified | One success flag for the mission: it would hide failed runs |
| M6 | Triggers (later) | Version 2.0 has only `repeat … until`. Time triggers (`every 1 h`, G-CLOCK, I5), message triggers (G-MSG, I6) and self-state triggers (G-SELF, I8) come later as `when …` clauses on a mission, each in its own iteration | All triggers at once: three gaps in one iteration (LE-10) |
| M7 | Static checks | The tasks a mission runs exist, and their arguments match the task parameters. The bound is present and positive. The `until` condition is well formed. Missions cannot nest, and a task cannot start a mission | Allow nesting: mission records that are hard to review |
| M8 | Core and backends | **Unchanged.** A mission runs on the planner's side and sends each task run as today, through `delegate` and Core 0.2. A robot never receives a mission, only tasks | A Core "mission" message: Core 0.2 is frozen, and robots would need to understand loops |
| M9 | Safety | Each task run keeps its own `pre` checks, limits (1.7) and verified goal. The bound and the stop are part of the signed approval (0038): an approval names the mission and its bound, so a mission cannot run past what was approved | Approve only the tasks: a mission could repeat them without limit |
| M10 | Version | 2.0, because it adds a layer. Files of 1.x tasks stay valid without change | 1.9: hides a change of layer |

## What it unlocks

On today's atlas, closing G-LOOP after G-ACT and G-CLOCK makes 2 of the 13 near sub-activities fully expressible.
With G-MSG it makes 4. The order is generated in
[suggested order](../../docs/plans/humanoid-activities.md#suggested-order-of-language-work).

## Pre-registration (at I4, L3)

- **Set 8 "missions":** the 12 standing G-LOOP tasks, relabelled, plus requests from the near sub-activities that
  need repetition. The set also gets controls and tasks that must stay inexpressible: "forever" with no bound,
  nesting, and a trigger whose iteration has not come yet.
- **Bounds:** fixed with the baseline (rule 6), including a measure of **a bound dropped or invented**: a mission
  without the bound the request implies, or with one it did not state.

## Owner's answers (2026-11-17)

1. G-LOOP is reopened, with the mission layer as its design.
2. Every repeat has a hard bound.
3. 2.0 has `continue` and `retry N`, as well as `stop`.
