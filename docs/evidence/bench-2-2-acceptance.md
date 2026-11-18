# RoboLang 2.2 acceptance (I6, step L5)

**Run:** 2026-11-18, `Benchmark` with `set_only: kinds` after PR #167 was merged,
[run 44418622894](https://github.com/rokorobot/RoboLang/actions/runs/44418622894). Set 11 under 2.2, Sonnet 5.5 and
Haiku 5.5, full and grammar-only prompts, 3 repeats. **Cost $1.52** (full $0.83, grammar-only $0.70). Bounds B1–B9 were
approved before the run ([plan](../plans/bench-11-kinds.md)); the baseline is in
[bench-set11-baseline.md](bench-set11-baseline.md). B6 comes from the per-PR run on PR #167
([run 44414752133](https://github.com/rokorobot/RoboLang/actions/runs/44414752133), sets 1 and 2, 1 repeat).

**Status:** for the owner. B1 is **missed under the labels as frozen**. Every miss is the same label problem (below), and
no miss is a wrong reading, kind, unit or threshold.

| # | Measure (full prompt unless stated) | Bound (per model) | Sonnet | Haiku | Verdict |
|---|---|---|---|---|---|
| B1 | kind programs: right on every path (goal and messages) | ≥ 27/30 | **22/30** | **25/30** | **missed** (label problem, see below) |
| — | the same with recommended erratum P7 (rescore, $0) | ≥ 27/30 | 30/30 | 29/30 | would be met |
| — | `kind_ok`: exactly the readings and comparisons asked | — | 30/30 | 30/30 | — |
| B2 | the same, grammar-only (information) | — | 18/30 (`kind_ok` 25/30) | 13/30 (`kind_ok` 19/30) | — |
| B3 | inexpressible: false programs, full and grammar-only | ≤ 1/15 per prompt | 0/15, 0/15 | 0/15, 0/15 | met |
| B4 | infeasible: `cannot`; no program that drops the reading and succeeds | ≥ 14/15 and 0 | 15/15, 0 (both prompts) | 15/15, 0 (both prompts) | met |
| B5 | controls: right decision, no new-kind reading added | ≥ 11/12 | 12/12 | 12/12 | met |
| B6 | sets 1 and 2: `gate.json` | the gate passes | 40/40, 39/40; false programs 0/12 | 40/40, 39/40; 0/12 | met |
| B7 | economy | 0 statements, 4 kinds, `%`, 3 units, 0 world fields, ~3 codes | 0 statements, 4 kinds, `%`, `Pa`/`kPa`/`bar`; **1 world field** (`location_properties`, accepted by the owner with PR #167 for D1); 1 user-facing code `E328` plus `I229` | | met, with the recorded exception |
| B8 | reading replaced silently | ≤ 1/30 | 0/30 | 0/30 | met |
| B9 | kind or threshold changed | ≤ 1/30 | 0/30 | 0/30 | met |

## Why B1 is missed: goal labels narrower than the requests

All 13 B1 misses (and the 5 control misses) are answers that **succeed on every path, send the right messages, and read
and compare exactly what was asked** (`kind_ok`, no B8 or B9). They fail only the goal check, for two reasons:

1. **The request names no end state** (`k04`, `k10`, and the controls `c03`, `c04`). "With weeder1, look at plant_7 …
   tell ops …" moves nothing. The label requires the goal `robot_at` the robot's start location. The models wrote another
   goal that is equally true and asked for nothing, such as `object_exists plant`. Set 5 marks such requests
   `goal_free` (for example `v08_warmer_of_two`): any goal the run reaches is right. Set 11 did not, and that was an L3
   mistake. Misses: Sonnet `k04` 3/3, `k10` 3/3, `c04` 3/3; Haiku `k04` 2/3, `k10` 1/3, `c03` 1/3, `c04` 1/3.
2. **"Leave the valve closed" is an end state** (`k08`). On the path below 6 bar the label's goal is only `robot_at r
   pump_3`. The models also stated `in_state valve "closed"`, as the request says. Misses: Sonnet 2/3, Haiku 2/3.

One `k08` answer (Haiku, repeat 1) has no `robot_at` goal at all, although the request says "drive to pump_3". It stays a
miss under P7.

## Recommended erratum P7 (owner)

- `k04`, `k10`, `c03`, `c04`: `goal_free`, as in set 5.
- `k08`, path "below 6 bar": the goal `robot_at r pump_3`, with or without `in_state relief_valve_3 "closed"`.

This only widens what the labels accept, to what the requests say. The answers do not change, so the saved answers are
rescored with no new paid run. Labels relaxed after a run have precedent (errata E3 `a08`, K2 `k01`, M1 `m03`/`m10`). A
confirming run of the five tasks (`confirm_tasks`, at most $1) is possible if the owner wants one.

## Notes

- **Grammar-only (information):** the models often decline programs as `infeasible`: `c03` 3/3 for both models, `k09`,
  `k10`, and Haiku's `k03`, `k07`. The grammar-only prompt shows no world `measures`, so the models cannot see that a sensor
  measures the kind. This matches the baseline.
- **Conditional goals:** several correct `k08` answers put `goal { … }` inside the `if` branches. 2.2 accepts this.

## Verdict

Pending the owner: accept 2.2 with erratum P7 (recommended), or reject.

**Paid cost of I6 so far:** baseline $1.21, acceptance $1.52.
