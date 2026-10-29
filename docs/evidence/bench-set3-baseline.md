# Benchmark set 3: the RoboLang 1.3 baseline (2026-10-29)

Set 3 (36 tasks, branching; decision [0027](../../spec/decisions/0027-bounded-branching.md) section 3) was
pre-registered in #68. This baseline ran on `main` before any 1.4 code ([Actions run
40944145600](https://github.com/rokorobot/RoboLang/actions/runs/40944145600)). Settings: full and grammar-only
prompts, 3 repeats, 3 models, effort medium. It cost about $3.30. Raw reports:
[full](bench-set3-baseline-full-report.md), [minimal](bench-set3-baseline-minimal-report.md).

In 1.3 the right answer to the 20 `branching` tasks is a decline. So for this run, `false_program` on them is
the F3 danger: a valid program that keeps one branch and silently drops the other.

## Headline (answers over 3 repeats)

| | Opus full | Sonnet full | Haiku full | Opus minimal | Sonnet minimal | Haiku minimal |
|---|---|---|---|---|---|---|
| Half-programs on branching requests, valid and SUCCEEDED | 6/60 | 7/60 | 17/60 | 29/60 | 30/60 | 31/60 |
| False programs on the 8 tasks that stay inexpressible in 1.4 | 0/24 | 0/24 | 0/24 | 0/24 | 0/24 | 0/24 |
| Controls (1.3 can express them) goal correct | 21/24 | 22/24 | 21/24 | 24/24 | 21/24 | 24/24 |

The gate floors fail, as they should: they are set 1+2 floors, and this run is for information only.

## Findings

**G1. Half-programs are frequent and invisible.** 30 of 180 branching answers with the reference, and 90 of 180
with the grammar only, were valid programs that ended SUCCEEDED. The checks would never have caught them.
The model reads the condition from the world snapshot ("amr2 is not ready at job start, so … amr1"; "the world
states fence_a is clear, so … the bin fallback is not needed"). It then writes only that branch, sometimes
guarded by `pre`. That is right in the world shown and wrong in the world at run time, which is what the
request was about.

**G2. Robot choice is the worst case.** With the reference: robot choice 26/45 half-programs, `robot_at` 10/36,
everything else 4/99. With the grammar only: robot choice 41/45, object choice 14/18, route 26/90. Region and
sensor routes are mostly declined correctly. A choice between two robots looks like a binding decision. The
models make it at authoring time, and nothing in 1.3 tells them otherwise.

**G3. The other limits hold.** Retry, failover, loops, counts, clocks, message waits and distances got no false
program in 144 answers. 0027 keeps these out of 1.4, and the models decline them.

**G4. One control had a bad label.** For `c_whether_or_not` ("… arm1 never enters the aisle"), the models declined
8 of 9 times with the full prompt. They read the clause as a no-go constraint, which RoboLang cannot state, and
they are right. The task is reworded (its `note` records this). The other false refusals were `c_only_if_cell`,
3 of 9 with the grammar only.

## What it means for 1.4

1. The evidence 0027 asked for holds. These requests fail without branching, and the failure is the dangerous
   kind, not a decline.
2. The language alone will not be enough for G2. The 1.4 reference must say that a condition about "when the
   job starts" or "at that moment" is decided at run time, never from the world description. The acceptance
   re-run must count half-programs on these tasks, not only goal correctness. Under 1.4 they show up as a valid
   program whose goal matches only one path.
3. For the acceptance run, the scorer needs per-path goals from `expect_1_4` and a simulation of each path
   (N4 PR 2 and PR 3).
