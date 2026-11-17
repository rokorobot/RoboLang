# Decision record 0039: Region limits on motion (I1, G-PATH, RoboLang 1.7)

**Date:** 2026-11-17
**Status:** **APPROVED by the owner, 2026-11-17**: P1 the reserved words, P2 the whole task, and the
recommended option for P3–P10. Step L2 of iteration I1 in the [language roadmap](../../docs/LANGUAGE_ROADMAP.md). The owner
chose the gap (L1) on 2026-11-17.
**Applies to (when decided):** a new `spec/robolang-v1.7.md` and `grammar-v1.7.ebnf`, Task IR 1.6, world file 1.3,
`simulation-v1.0.md` section 4.3, `src/robolang/`, `docs/language-reference.md`, the ROS 2 backend's refusal list,
`bench/` (L3 set).
**Evidence:**
- **Atlas gap G-PATH.** It is near, and the current horizon needs it (strategy D5). Four near sub-activities need
  it: floor cleaning, construction layout, weeding and solar installation
  ([plan](../../docs/plans/humanoid-activities.md)).
- **Probes.** The three R3 probes `f_path_avoid_wet_floor`, `f_path_stay_in_zone` and `f_path_keep_off_road`
  ([evidence](../../docs/evidence/rsi-r3-gap-verification.md)) fail in 1.6 with E201/E202. Both models declined
  all three, so the gap meets both halves of LE-7.
- **Older tasks.** The standing tasks `u09_avoid_quay` and `u10_inside_clean_room` ask for the same thing.
- **Safety case.** Gap G2 asks for "a declared workspace it cannot leave" (hazards H05 and H06).

## The idea in one paragraph

A task may declare regions its robots must stay out of, or stay inside, **while they move**. This is the same kind
of thing as `@safety max_speed`: a **request** that RoboLang checks before the run, against the world and the robot's
declared abilities. RoboLang never enforces it (rule LE-13, safety case C1). Keeping a robot out of a region is the
job of the motion planner and the safety controller. A robot that cannot honour the request makes the run fail
before anything moves.

```rob
robolang 1.7;

task to_lobby(robot r, location lobby, region wet) {
    @safety no_go_region wet;
    goal { robot_at r lobby; }
    plan { move r to lobby; }
}
```

## Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| P1 | Construct | Two task annotations: `@safety no_go_region R;` (stay out) and `@safety in_region R;` (stay inside). Both words have been reserved since 1.1, so **no new keyword** is needed | New words `keep_out` and `stay_in`: clearer English, but two more keywords (Economy). A clause on the step, `move r to x avoiding R`: see P2 |
| P2 | Scope | The **whole task**: every motion of every robot of the task, from the first step to the last | Per `move` step: needed only when a route may cross a zone at one moment and not at another. No standing task or probe asks for that. Kept for later, with evidence |
| P3 | What the request covers | **Motion of the robot's base and anything it carries**; the arm's workspace is not covered (that is P9) | Every part of the robot, arm included: needs a geometry RoboLang does not have |
| P4 | Static checks (`validate`) | R must be a `region` parameter (E5xx). The same region may not be both `no_go_region` and `in_region` (E4xx). At most 8 region limits per task | No limit on the count: invites tasks no backend can plan |
| P5 | World checks (`validate --world`, simulation) | World file 1.3: a region may list the locations it `contains`. Using a location inside a `no_go_region` as a step's or goal's target is an error before the run (S0xx), and so is a location outside an `in_region` when the region lists its locations. A region without `contains` is unknown, so nothing is concluded from it | Model routes, a graph of paths and the regions they cross: a travel model the symbolic simulation does not have (section 3.4). It belongs to the physics backends |
| P6 | Robot ability | A robot profile declares the region limits it can honour: `"honours": ["no_go_region", "in_region"]`. A task bound to a robot that does not declare one fails the run with a new S-code before `AUTHORIZED` (fail closed, as `S002` does for a missing speed unit) | Assume every robot can: a silent limit, the worst failure we have |
| P7 | Run record | Each limit is recorded with the status `checked_against_world_not_enforced`, as `max_speed` is. No record ever says "enforced" | Record nothing: the reviewer could not see the request was checked |
| P8 | Backends and Core | The ROS 2 / Gazebo backend refuses tasks with region limits, as it refuses speed limits today, until it can pass them to the planner (G2). `delegate` and Core 0.2 carry no region limits, so a task with one is refused for delegation (a new code), not silently dropped | Add region limits to Core: Core 0.2 is frozen. Revisit in Core 0.3 together with G-CHOOSE (I7) |
| P9 | Not in 1.7 | Per-step limits (P2), the arm's workspace, regions that move (a zone around a person, G-PROX), and force limits (G-FORCE) | — |
| P10 | Design for the far needs | Regions stay entities with an ID. A later version can add a region that moves (G-PROX) or a `max_force` limit (G-FORCE) **without new syntax**: one more `@safety` annotation and one more profile ability, checked the same way | A separate `zone { … }` block: more syntax now, no gain later |

## Pre-registration (L3), before any code

- **Set:** the three R3 probes, relabelled for 1.7 as programs, plus `u09_avoid_quay` and `u10_inside_clean_room`.
  New tasks will be added:
  - infeasible: the destination lies inside the no-go region; the robot cannot honour the limit;
  - inexpressible: per-step limits, an arm workspace, a zone around a person;
  - decoys: regions with near-miss names;
  - controls: tasks that need no region limit.

  The set is frozen and its baseline is run on 1.6 before L4 starts.
- **Bounds (rule 6, set with the baseline in hand):**
  - the three probes: goal correct with the limit declared, for both models;
  - false programs on the new inexpressible tasks: no worse than the baseline;
  - decision accuracy and false programs on the standing sets 1 and 2: no worse (rule 2).
- **Economy:** 0 new keywords, 2 annotations, 1 world field, 1 profile field, and about 4 new diagnostic codes.

## Open questions for the owner

1. ~~P1: reuse the reserved words or add new ones?~~ **Decided 2026-11-17: the reserved words `no_go_region`, `in_region`.**
2. ~~P2: is a whole-task limit enough for 1.7?~~ **Decided 2026-11-17: yes; per-step limits wait for evidence.**
