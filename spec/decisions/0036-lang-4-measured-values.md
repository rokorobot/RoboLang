# Decision record 0036: Measured values (LANG-4 and a minimal LANG-5, RoboLang 1.6)

**Date:** 2026-11-09
**Status:** **PROPOSED.** V1–V13 need the owner, including the paid runs (V12). Decided so far by the owner on
2026-11-09: the set 5 labels, and V3 option (b). No language feature is built; set 5 is.
**Applies to (when decided):** a new `spec/robolang-v1.6.md` and `grammar-v1.6.ebnf`, Task IR 1.5, world file 1.2,
`simulation-v1.0.md`, `src/robolang/`, `docs/language-reference.md`, `bench/` (a new set 5, relabels in sets 2, 3 and
"ask").
**Evidence:** Five standing benchmark tasks are inexpressible **only** because a program cannot read a value:
`u05_below_8c`, `u06_cooled_60c` (`numeric_sensing`), `u19_send_batch`, `u20_report_temp` (`value_report`) and
`i12_scale_down` (`weighing`). Models decline them correctly: this is a **coverage** gap, not a safety failure like the
half-programs that motivated 1.4 (N4). Three labels are inconsistent with them: `w_scale_precheck` (set 1), `d_serum_tube` (set 2) and `f07` (set
"ask") accept `inspect … using <scale or thermometer>` as weighing or taking a temperature, which `i12` says it cannot do. Outside the repo,
URML ships `measure`, `detect` and `report` with values (2026-11-09 comparison). Plan and tasks:
[`docs/plans/bench-5-values.md`](../../docs/plans/bench-5-values.md).

## Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| V1 | Scope | Read a value from a sensor, compare it, send it. Nothing else: no `detect`, arithmetic, counting, clock readings, structured data (LANG-6), or values in `goal` | Add `detect` (choose an object at run time): goals and Core export name things fixed before the run, so it needs its own decision. Add arithmetic: no task needs it except controls |
| V2 | Reading | `measure r temperature of pan using probe as t;` takes one reading and binds it to `t`. A `measure` that cannot read (sensor unavailable, value not stated) **fails the step**, so a 1.5 handler can catch it | No binding, and every reading is live (`temperature of pan using probe < 60 degC`). Smaller, but a reported value can differ from the value just compared, and the record cannot say which reading decided |
| V3 | Bindings (minimal LANG-5) | **Owner, 2026-11-09, option (b):** a `measure` in the handler of a `measure` may bind the **same name**, so a retried reading is defined on both paths ("retry the reading once, then send it"). Otherwise: set once, never changed. Unique within a task. Visible after its `measure` in the same block and in nested blocks. One made inside a branch or handler is not visible after it (static error). Not visible in the handler of its own `measure`; and if that handler does not end in `fail`, not visible after it either, since on that path nothing was read (static error) | Mutable variables: needs definite-assignment analysis across paths and invites loops |
| V4 | Waiting for a value | Only `wait until` reads live: `wait until temperature of pan using probe < 60 degC timeout 10 min else { … }`, read on every world change, as other wait conditions are | Bindings only: "once it has cooled below 60 °C" (`u06`) stays inexpressible |
| V5 | Kinds and units | A closed list: `temperature` (`degC`, `K`), `mass` (`g`, `kg`), `distance` (`mm`, `cm`, `m`), and `label` (text: batch numbers, barcodes). Within a kind, units convert exactly in decimal, as durations already do (`500 ms` = `0.5 s`); across kinds is a static error. °F is not a unit: a request in °F is converted by the author, as `30 cm/s` is today | Units declared by each world: open vocabulary, nothing to check statically. No conversion at all: the author must know the sensor's unit before writing |
| V6 | Comparisons | Quantities: `<`, `<=`, `>`, `>=` only, against a literal or another binding of the same kind. Labels: `==`, `!=` against a string or label. Combine with `and`, `or`, `not` as today. No `==` on quantities: with uncertainty it is almost never decidable | Allow `==` on quantities: invites conditions that are always unknown |
| V7 | Uncertainty and unknown | A reading is `v ± u`, with `u` declared by the sensor. `t < 60 degC` is true if `v + u < 60`, false if `v − u ≥ 60`, and **unknown otherwise**. As today, an unknown condition fails the run (`S261`) or a wait keeps waiting | Compare `v` alone: a reading of 59.8 ± 0.5 would pass a 60 °C limit that it may exceed |
| V8 | Sending a value | `communicate r to "pass" with t;` sends the binding as a typed value. The run record holds subject, kind, value, unit, uncertainty, sensor and time. No string formatting | Formatting (`"Pan is {t}"`): a second small language inside strings, with its own errors |
| V9 | World file 1.2 | Sensor: `measures: { "temperature": { "unit": "degC", "uncertainty": "0.5" } }`. Object: `properties: { "temperature": { "value": "72", "unit": "degC" } }`. A new event fact `property` changes a value at a time (a pan cooling). `validate --world` reports a sensor that cannot measure the kind asked for (new code), alongside `S231`/`S232` | Properties only, no events: `u06` (waiting to cool) cannot be simulated |
| V10 | Compatibility | `robolang 1.6;` selects it. Files marked 1.1–1.5 keep their meaning. New reserved words: `measure` and `as` only; kinds and units are recognised by position, so existing names stay valid. Task IR 1.5 adds a `measure` step, a comparison condition and binding references: an IR 1.4 reader rejects it. Core export unchanged (V1: no values in goals) | Reserve every kind and unit: breaks any 1.6 file with a location named `m` or an object named `label` |
| V11 | Order | Rule 1, done properly this time ([0035](0035-bench-errata-1-5.md) E1 records that 1.5 skipped it): (1) owner approves the set 5 labels; (2) baseline under the **current** reference, where every set 5 value task must be declined; (3) spec and code; (4) reference and prompts teach 1.6; (5) acceptance run. Starts after 0035 is decided, so only one reference change is in flight at a time | Build first, measure after: no "before" numbers |
| V12 | Paid runs | Baseline: set 5, 3 models × 3 repeats × both prompts (~470 answers). Acceptance: the same under 1.6, plus sets 1, 2, 3, "ask" (and 4 if 0035 lands) once each, full prompt. At set 3's measured cost (~$4.10 per 648 answers) about $10 in all; hard cap `--max-usd 15` | One repeat: too few answers to read a 5% false-program rate |
| V13 | Acceptance | Like 0035 E7. Set 5 expressible: goal correct on every path **and** `value_ok` (the reading is of the right thing, compared the right way, sent where asked) ≥ 90% (Sonnet, full prompt). Set 5 inexpressible controls: false programs ≤ 5%, even grammar-only. Other sets: `gate.json` passes and `decision_correct` drops no more than 2 points per model. If it fails, 1.6 stays in code only and is not taught | Gate on set 5 only |

## Relabels when 1.6 is taught (old label kept as `expect_1_5`)

`u19`, `u20` → program. `u05` → program (the thermometer comes online at 5 s; wait for `sensor_ok` first). `u06` → program
with a wait (no bound is given, so any timeout is accepted, as for `a19_until_corridor`). `i12` → **infeasible**: in its
world `scale_1` is unavailable and never recovers. `w_scale_precheck`, `d_serum_tube` and `f07` stay programs, plus a check that a
mass or temperature reading was taken: an `inspect` no longer counts. `x_count_report` (counting) and `x_report_duration` (clock) stay
inexpressible.

## Consequences

- Three PRs: (1) set 5 and its baseline; (2) spec, grammar, IR 1.5, world 1.2, validator and simulator; (3) reference,
  prompts, relabels, errata and acceptance.
- The 1.6 spec states the exact error and step codes; this record only fixes the behaviour.
- The H3 Gazebo backend has no value sensors; reading values there is a separate item.

## Needs the owner

V1–V13, the set 5 labels in the plan, and authorization of the V12 spend.
