# Decision record 0045: Choosing at run time (I7, G-CHOOSE, RoboLang 2.3)

**Date:** 2026-11-18
**Status:** **APPROVED by the owner, 2026-11-18**: C1–C12 as recommended (positions and straight-line distance, C5;
`delegate` accepts choices made before the first action, C8; choices inside missions, C10). This is step L2 of iteration I7 in the
[language roadmap](../../docs/LANGUAGE_ROADMAP.md). L1 is done: G-CHOOSE is first in the generated
[suggested order](../../docs/plans/humanoid-activities.md#suggested-order-of-language-work) (priority 5.23, the gap that
unlocks the most).
**Applies to (when decided):**
- new `spec/robolang-v2.3.md` and `grammar-v2.3.ebnf`;
- Task IR 1.11 and world file 1.8;
- `simulation-v1.0.md` and `src/robolang/`;
- `export-goal` and `delegate`;
- `docs/language-reference.md`;
- `bench/` (the L3 set).

**Evidence:**

- **Atlas:** 17 sub-activities need G-CHOOSE, more than any other open gap. Four of them are near: tote handling (ACT-LOG-001),
  shelf scanning (ACT-RET-001), weeding (ACT-AGR-002) and self-charging (ACT-AUTO-001). Others are within three years:
  sequencing, tugger handoff, EV charging and fleet interoperability.
- **R3 probes:**
  - `f_choose_nearest_tugger`: "send whichever of tugger_a and tugger_b is nearest to the handoff point";
  - `f_choose_nearest_dock`: "move spot1 to whichever dock is nearest";
  - `f_choose_nearer_cart`: "place it on whichever of cart_3 and conveyor_b is closer".

  Each attempt fails with `E304`, and both models declined all three.
- **Standing tasks in the gap:** `x_nearest_robot`, `x03_any_hot_pan`, `x05_every_new_part`, `x_shuttle_until_five` and
  `x03_carry_as_many_as_counted`.

## The problem

Every entity in a RoboLang task is fixed when the task starts: each robot, object and location is a parameter bound to
a world ID. Real requests often leave the choice to run time:

- "whichever tugger is nearest";
- "the next part on the conveyor";
- "a free dock".

Today these must be declined. The alternative is that a model guesses one candidate, and the choice disappears without
anyone noticing.

The roadmap notes that this changes **how goals name things**. A goal such as `robot_at t handoff` names `t`, and `t` is
not known until the choice is made. Core 0.2 sends only goals whose IDs are fixed (ground goals). So this record must
also say how a choice reaches Core.

## The idea in one paragraph

A new statement, `choose`, binds a name to **one** entity from a **finite set of candidates**:

- an optional `where` keeps only the candidates for which a condition holds;
- an optional `nearest to X` orders them;
- a mandatory `else` handles the case where no candidate is left.

After the choice, the name is used like a parameter, also in the goal. The record states which candidate was chosen
and why. The verifier checks the goal with that choice filled in.

```rob
robolang 2.3;

task send_tugger(robot a, robot b, location handoff) {
    goal { robot_at t handoff; }
    plan {
        choose robot t from (a, b) where robot_ready t nearest to handoff else { fail "no tugger is free"; }
        move t to handoff;
    }
}
```

## Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| C1 | Construct | A statement `choose TYPE NAME from CANDIDATES [where COND] [nearest to X \| farthest from X] else { … }`. `TYPE` is `robot`, `object` or `location`. The name is bound from that point on, as a `measure` binding is | Built-in functions in goals (`robot_at nearest(a, b) handoff`): the choice would be hidden inside a goal. Choosing in the task header: the choice is made before the run, so a run-time state could not decide it |
| C2 | Candidates | Three forms, all finite: **a list** of parameters or names already bound, `(a, b, c)`; **`objects at L`**, the objects at location `L` when the choice is made; **`robots`**, every robot the world declares. Nothing else: no "any object in the world" | Only lists: "the next part on the conveyor" and "whichever robot is closest" could not be said. Open sets ("any pan"): detection of objects the world does not know (G-DETECT, below) |
| C3 | `where` | A condition over the bound name, using the conditions tasks already have (`robot_ready`, `object_at`, `in_state`, `region_clear` and the others). It is evaluated from the world state at the choice. An unknown value excludes that candidate (fails closed) | `where` on a reading (`where temperature of it > 80 C`): each candidate would need its own reading, which is a sequence of robot actions, not a choice. Kept out (C9) |
| C4 | Order | `nearest to X` or `farthest from X`, where `X` is a location, robot or object. Ties, and a choice with no order, go to the **first candidate in the order written**; for `objects at L` and `robots`, the order the world lists them. So a choice is deterministic and can be reproduced | A random pick: runs could not be reproduced. "best by" a reading: C9 |
| C5 | Distance | World file 1.8 gives each location a **`position`** `[x, y]` in metres. A robot or object is at the position of its location. Distance is straight-line. A candidate or `X` with no position is unknown and excluded (fails closed). Distance is used only to order candidates; there is no number a task can read | Route lengths from a map (`routes`): more honest for a building, but one entry for every pair of locations. A later world version can add `routes`, and they would take precedence over straight lines. Travel time: needs a travel model the simulator does not have |
| C6 | Nothing to choose | `else` is **mandatory**, as the timeout of `wait until` is (E209). "No tugger is free" must be handled, never skipped | An optional `else` that fails the run by default: a missing `else` would fail silently in a request that expects a fallback |
| C7 | Goals and the record | A goal may name a chosen name. The verifier fills it in with the recorded choice: the goal is checked for the entity chosen, never "for some entity". If the run ends before the choice, the goal is not reached. The run record lists, for each choice, the candidates, those excluded by `where` (and why), the order used and the entity chosen | A goal that holds for any candidate ("some tugger is at the handoff"): verification would accept the wrong tugger |
| C8 | Core and backends (**the Core decision**) | **Core 0.2 is unchanged.** A robot only ever receives ground goals. `export-goal` and `delegate` accept a task only if every `choose` comes before its first action and uses world facts (C3, C5) only. The planner then makes the choices from the world model, records them, and sends the task with the choices filled in. Any other task with `choose` is refused (new code). The ROS 2 / Gazebo backend refuses `choose` in 2.3 | Core 0.3 with choice variables in goals: Core is frozen, and every robot would need to understand choices. Refusing every `choose` in `delegate`: even the simplest "send the nearest free tugger" could then not be delegated |
| C9 | Not in 2.3 | Choosing by a reading of each candidate ("the fullest bin", "any pan above 80 °C"); acting on **each** candidate in turn with a memory of which ones are done ("every pan", "carry as many as counted"); detecting objects the world does not list; choosing a person (G-PROX); several choices that depend on each other | — |
| C10 | Missions | A task with `choose` may run in a mission, and each run chooses afresh. "Move every new part from the conveyor for 8 hours" is a mission that repeats a task choosing `from objects at conveyor_in`. A part that has been moved is no longer a candidate | A `for each` loop in tasks: N4's reason (no end state to verify) still holds |
| C11 | Safety | A choice can send a different robot, or pick a different object, from what a reviewer expects. **Hazard H25: a stale or wrong world model makes a choice wrong.** Controls: the candidates are finite and appear in the task, so an approval (0038) names them; the choice is recorded with its reasons; unknown values exclude a candidate; in physical mode, positions and states come from the fleet manager (G4, commissioning) | — |
| C12 | Design for the far needs | G-PROX can reuse `nearest to` for a person once a person can be bound. G-CONC and ACT-AUTO-004 (a low-battery robot hands its job to a peer) choose a peer with `choose robot … where robot_ready …`. G-SELF can add a `where` on charge. Learned categories can extend C2 with `objects at L labelled "…"`. G-EACH (below) builds on C10 | — |

## What it makes expressible

- **R3 probes:** `f_choose_nearest_tugger`, `f_choose_nearest_dock` and `f_choose_nearer_cart`.
- **Standing tasks:**
  - `x_nearest_robot` ("whichever robot is closest", from `robots`);
  - `x05_every_new_part` and `x_shuttle_until_five`, as missions under C10, if the owner agrees with C10 (open point 2).

**Still inexpressible:**

- `x03_any_hot_pan`: a reading of each pan, with a memory of which were checked;
- `x03_carry_as_many_as_counted`: as many steps as the count.

These two go to a new far gap, **G-EACH** (acting on each member of a set chosen at run time, by reading or by count), so
that G-CHOOSE can close at L6. Object detection beyond what the world lists is a far gap, **G-DETECT**.

## Open points (answered 2026-11-18)

1. **Distance (C5):** straight lines from positions, or route lengths? I recommend positions: they are simple, and routes
   can be added later.
2. **Missions over `objects at L` (C10):** should `x05_every_new_part` and `x_shuttle_until_five` become programs in 2.3?
   I recommend yes. They are the shift work of the near sub-activities (tote handling, sequencing).
3. **`delegate` (C8):** accept choices that come before the first action, or refuse every `choose` in 2.3? I recommend
   accepting them, so that "send the nearest free tugger" works end to end.
4. **New gaps G-EACH and G-DETECT:** split them off now, so that G-CHOOSE has a clear end?

## Owner's answers (2026-11-18)

The owner approved C1–C12 as recommended. This answers open points 1–3: positions (C5), choices inside missions (C10),
and `delegate` accepting choices made before the first action (C8). Point 4 follows the recommendation: G-EACH and
G-DETECT are split off as far gaps, so that the L3 set can place the tasks that stay out. The owner can still change
this before L6.

## Pre-registration (L3), before any code

- **Set 12 "choose":** the three probes and the standing tasks above, relabelled for 2.3, plus new tasks of four kinds:
  - **programs:**
    - the nearest of a list;
    - a `where` filter (a free robot);
    - `objects at L`;
    - `robots`;
    - a choice in a mission;
    - a goal that names the choice;
    - a tie broken by order;
    - a decoy (a candidate that looks nearest but is not ready);
  - **infeasible:** no candidate is left and the request gives no fallback; a location with no position;
  - **inexpressible:** "the fullest bin", "every pan above 80 °C", "the nearest person";
  - **controls.**
- **Bounds:** fixed with the baseline (rule 6). They include a B8-like measure, **a choice replaced silently**: a guessed
  fixed candidate, or the first of the list with the order dropped.
- **Economy:** 1 statement, 1 world field (`position`), about 4 codes. Reserved words: `choose`, `nearest`, `farthest` (open:
  recognised by position, as with the kinds in 2.2?).

## Clarifications (APPROVED by the owner 2026-11-19, from writing the L3 set)

The 11 open points in [bench-12-choose.md](../../docs/plans/bench-12-choose.md#open-points-where-0045-does-not-decide-what-a-label-needs-approved-2026-11-19)
are approved as recommended:

- an `else` ends in `fail`;
- an indifferent request accepts any candidate;
- a finite named set compared by readings stays an `if`, and `choose` never reads a value;
- C8 is unchanged;
- `choose`, `nearest` and `farthest` are not reserved words.
