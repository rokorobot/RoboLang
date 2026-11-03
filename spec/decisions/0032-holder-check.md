# Decision record 0032: Holder check in `validate`

**Date:** 2026-11-03
**Status:** **APPROVED by the owner, 2026-11-03** (recommended option for H1–H4), and implemented before
[0033](0033-step-failure-handlers.md). **IMPLEMENTED** with the refinement of H1 in section 5.
**Applies to:** `validator.py` and `ir_reader.py` (through `paths.double_grasps`), the language reference,
`robolang-v1.4.md` section 5, `ir-reader-v0.1.md` (`I216`).
**Evidence:** an object has one holder at a time. Only `grasp` and `place` change who holds an object
(`sim/executor.py`, `Grasp` and `Place`); world events do not. A plan that grasps an object a robot already holds fails
at run time with `S212` or `S215`, and today that is found only by `simulate`.

## 1. The problem

```rob
grasp arm box;            // arm holds box
move mobile to src;
grasp mobile box;         // certain failure: S215 (box has no location), or S212 if arm and mobile are one robot
```

`validate` accepts this. The failure does not depend on the world: if the first `grasp` had failed, the run would
already have stopped.

## 2. Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| H1 | What is checked | On every path of the plan (finite: there are no loops), track what the plan itself sets: after `grasp R O`, `R` holds `O`; after `place R O at L`, `R` holds nothing. Two errors: `grasp X O` while another robot parameter holds `O`, and `grasp R O2` while `R` holds anything. A `place` that names an object `R` is not known to hold ends tracking for `R` on that path (two object parameters may be bound to one object) | Also flag `place R O` when `R` is not known to hold `O`: the world may say `R` holds `O` at the start, so it is not certain |
| H2 | Code and severity | New source error `E315`, exit 1, reported once per statement with the position of the earlier `grasp`: `error[E315]: 'grasp mobile box': box is still held by 'arm' (grasped at 3:5); place it first` | A warning: [0026](0026-validate-world.md) W2 reports only certain failures, as errors; this one is certain |
| H3 | Versions | Applies to every language version. A program that triggers it can never succeed | Only from 1.5: no existing valid program can succeed with this pattern, so there is nothing to protect |
| H4 | Two robot parameters bound to one robot | Out of scope here. Recommended later: reject the binding only when the plan has a handoff (one robot parameter places an object another grasps); no new syntax | Reject any shared binding unless the task declares it: could break existing worlds and benchmark tasks that reuse a robot |

## 3. Not in this record

Conflicts between robots acting **at the same time**. Steps run in written order today, so they cannot occur. When
concurrency (F3-1) is designed, the same rule extends: any object, location or region changed by two parallel branches
needs a declared owner, a handoff, or read-only use, or the file does not validate.

## 4. Needs the owner

Nothing: H1–H4 approved.

## 5. Implementation notes

- **H1, refined.** Any `place` ends what is known about **every** robot on that path, not only the placing one. Two
  robot parameters and two object parameters may each be bound to one entity, so after `place R O` a fact about
  another robot may no longer hold. Keeping it could reject a program that can succeed; dropping it only misses some
  errors. The check stays certain.
- The check runs only on a task whose plan steps have no other error, so a wrong name does not add an `E315`.
- The IR reader restates it as `I216`, located by JSON Pointer, as it restates the other static rules.
- Benchmark set "ask" tasks `i15_two_bowls_one_gripper` and `i16_two_containers_one_crane` have attempts that one robot
  grasps a second object. They are now rejected with `E315` instead of failing with `S212` at run time. The frozen
  labels (`attempt_fails_with` `S212`) are unchanged; `tests/test_bench_ask.py` maps them.
