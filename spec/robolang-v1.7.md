# RoboLang 1.7 (delta over 1.6)

**Status:** front end, Task IR 1.6, simulation (world file 1.3, run record 1.6), `validate --world` and the IR reader
complete in code. The shipped language reference and the benchmark prompts are taught in the step after this one
(L4, second commit, decision 0039).
Decision: [0039](decisions/0039-region-limits.md) (P1 to P10 and C1 to C7, approved 2026-11-17). Grammar:
[grammar-v1.7.ebnf](grammar-v1.7.ebnf). IR: [schema 1.6](ir-schema-v1.6.md). Simulation:
[section 15](simulation-v1.0.md). Everything not stated here is exactly as in [RoboLang 1.6](robolang-v1.6.md).

## 1. Version

`robolang 1.7;` selects 1.7. A file with this marker is read with the 1.6 rules, plus this document. A 1.6 front end
rejects it with `E213`. Files with no marker or with `1.1` to `1.6` are read, validated, compiled and run exactly as
before, byte for byte (Task IR 1.0 to 1.5, run records 1.0 to 1.5). A region limit in a file of those versions is still
an error: `@safety no_go_region R;` is `E211` and `@safety in_region R;` is `E201`, as before.

There are **no new reserved words**: `no_go_region` and `in_region` have been reserved since 1.1 (decision 0039 P1).

## 2. Region limits

```
@safety no_go_region R;     // the robots must stay out of R
@safety in_region R;        // the robots must stay inside R
```

```rob
robolang 1.7;

task to_lobby(robot r, location lobby, region wet) {
    @safety no_go_region wet;
    goal { robot_at r lobby; }
    plan { move r to lobby; }
}
```

- `R` is a `region` parameter of the task. The annotation stands among the other `@safety` annotations, before `pre`,
  `goal` and `plan`, in any order with `max_speed`.
- **Scope: the whole task** (P2). The limit covers every motion of every robot of the task, from the first step to the
  last, on every path, including the recovery moves in failure handlers (C6). There is no limit on one step: `move r to
  x in_region c;` is still `E211`, with a message that says so. Limits that hold on one leg only are not in 1.7 (P9).
- **What it covers** (P3, C5): the motion of the robot's base and anything it carries. A fixed-base arm is covered
  like any robot: a limit applies to every `move` of every robot, and what that means physically is the backend's
  concern. The arm's workspace is not covered.
- **A request, not an enforcement.** As `@safety max_speed`, a region limit is something the task asks for. RoboLang
  checks it before the run against the world and the robots' declared abilities ([simulation 15](simulation-v1.0.md)),
  and **never enforces it** (rule LE-13). Keeping a robot out of a region is the job of the motion planner and the
  safety controller. A run record never says a region limit was enforced: it says
  `checked_against_world_not_enforced`.
- A robot that cannot honour the request makes the run fail before anything moves, with a code of the `S00x` family
  (section 5). There is no model of routes: a path between two allowed locations that crosses the region is not seen.
- What 1.7 does **not** have (P9, P10): limits on one step, the arm's workspace, regions that move (a zone around a
  person), force limits, and a way to name a region that is not a parameter. A future version adds such limits as
  more `@safety` annotations, with no new syntax.

## 3. Static rules (`validate`)

1. `R` is a parameter of the task (`E303` otherwise) of type `region` (`E304` otherwise).
2. A region is named by **one** region limit: two limits on the same region, whether the same kind twice or one of
   each, are `E323`.
3. A task has **at most 8** region limits (`E324` for the ninth), so that no task asks for what no backend can plan.
4. `max_speed` keeps its rules (`E305`, `E306`, `E308`); a region limit does not count towards them.

## 4. Task IR

A region limit is an entry of the task's `safety` array, in source order, with `kind` `no_go_region` or `in_region` and
a `region` (the parameter's name) instead of a `limit`: [IR schema 1.6](ir-schema-v1.6.md). A 1.7 source produces
schema 1.6.

## 5. Diagnostics

| Code | Stage | Meaning |
|---|---|---|
| `E323` | static | A region is named by more than one region limit of the task |
| `E324` | static | A task has more than 8 region limits |
| `E303`, `E304` | static | (existing) `R` is not declared / is not a region |
| `S003` | authorization | A robot that moves on some path does not declare that it honours the kind of limit (world file 1.3 `honours`); a world file 1.2 robot honours nothing (C1) |
| `S004` | authorization | A target is where the limit forbids: a `move` or `place` target, or a `robot_at` goal location, inside a `no_go_region` or outside an `in_region` that lists its locations (world file 1.3 `contains`) |
| `S005` | authorization | A robot that moves starts inside a `no_go_region` or outside an `in_region` that lists its locations (C7) |
| `E516` | world file | World file 1.3: an unknown ability in a robot's `honours` |
| `I224`, `I225` | Task IR 1.6 | `E323` and `E324` restated on IR ([ir-reader-v0.1.md](ir-reader-v0.1.md)) |
| `X605` | export | `export-goal` and `delegate`: the task has a region limit; Core has no form for it, so the task is refused (P8) |
| `S508` | ROS 2 backend | (existing) a region limit is refused before any motion, as a speed limit is (P8) |

`E213` now lists `1.1, 1.2, 1.3, 1.4, 1.5, 1.6 and 1.7`. `delegate` also refuses, among the plans of the robot's
library, one with a region limit, with the plan result `region_limits_unsupported`.

## 6. Elsewhere (P8)

- **ROS 2 / Gazebo backend:** refuses a task with a region limit with `S508` before any motion, as it refuses a speed
  limit, until it can pass them to the planner ([ros2-backend-v0.1.md](ros2-backend-v0.1.md)).
- **`export-goal`, `delegate`:** Core 0.2 carries no region limits and is frozen, so a task with one is **refused**
  with `X605`, never sent with the limit dropped. Revisit in Core 0.3.

## 7. Compatibility

Nothing changes for 1.1 to 1.6 files, their IR, their run records, or their world files 1.0 to 1.2. A world file 1.3
may serve a task of any language version; its `contains` and `honours` are read only by a task with a region limit. A
task with a region limit run against a world 1.0 to 1.2 fails closed with `S003` (C1): those robots honour nothing.
