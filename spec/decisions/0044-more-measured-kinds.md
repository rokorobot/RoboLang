# Decision record 0044: More measured kinds (I6, G-PERCEIVE, RoboLang 2.2)

**Date:** 2026-11-18
**Status:** PROPOSED. This is step L2 of iteration I6 in the [language roadmap](../../docs/LANGUAGE_ROADMAP.md). The
owner chose to continue with I6 on 2026-11-18. Every point below needs the owner's review before anything is built.
**Applies to (when decided):** a new `spec/robolang-v2.2.md` and `grammar-v2.2.ebnf`, Task IR 1.10, world file 1.7,
`src/robolang/measured.py`, `tokens.py`, `docs/language-reference.md`, and `bench/` (the L3 set).

**Evidence:**

- **Atlas:** G-PERCEIVE is the first open gap in the generated
  [suggested order](../../docs/plans/humanoid-activities.md#suggested-order-of-language-work). R3 narrowed it to
  *perceived values beyond the four kinds of 1.6*.
- **R3 probes:** `f_perceive_shelf_fill` ("less than a quarter full") and `f_perceive_weed_confidence` ("less than 90
  percent confident"). Both attempts fail with E217, and both models declined both probes.
- **Sub-activities:** the requests show two more kinds:
  - a **pressure** reading: "read the gauge on pump 3 and tell me if it is above 6 bar" (inspection, ACT-INSP-001);
  - a **count**: "which products are out of stock" (retail scanning, ACT-RET-001), "an unusual pallet count"
    (ACT-INSP-002).

## The problem

RoboLang 1.6 reads four kinds: `temperature`, `mass`, `distance` and `label`. Perception on real robots reports
other values all the time: how full something is, how sure a detector is, what a gauge shows, how many items are
there. Without these kinds, those requests must be declined, or models write `inspect` and drop the condition.

## The idea in one paragraph

Add kinds to the existing `measure` machinery and change nothing else. A kind works like the four kinds of 1.6:

- a reading binds a name, carries an uncertainty, and fails closed when unknown;
- it is compared with the same operators;
- units of one kind convert exactly;
- the world declares which sensor measures which kind;
- there is no new statement.

```rob
measure r fill of shelf using cam as level;
if level < 25 % { communicate r to "lead" with "restock shelf_a"; }
```

## Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| K1 | Which kinds | Four: **`fill`** (how full: `%`), **`confidence`** (how sure a detector is of the object's label: `%`), **`pressure`** (`Pa`, `kPa`, `bar`), **`count`** (how many items a sensor sees on or in the object: a whole number, no unit) | One generic `ratio` kind for fill and confidence: shorter, but a "fill" request and a "confidence" request would read alike and be easy to mix up. Free kinds named by the world: nothing could be checked statically |
| K2 | Units | `%` is a new unit token, used for `fill` and `confidence` (0 to 100). `pressure` converts exactly between `Pa`, `kPa` and `bar` (1 bar = 100 kPa). A `count` takes no unit (`count < 1`); writing a unit on a count is an error | Plain numbers 0 to 1 for fractions: "less than a quarter full" turned into `0.25` invites mistakes |
| K3 | Comparisons | `fill`, `confidence` and `pressure` take `<`, `<=`, `>`, `>=`, as quantities do in 1.6, with no `==`. A `count` is exact: it also takes `==` and `!=`, and its uncertainty is 0 | `==` on a count fails closed like a quantity: "out of stock" (`count == 0`) could never be said |
| K4 | `confidence` belongs to a label | `measure r confidence of plant using cam as c` is the sensor's confidence in the **label it reports** for that object at that reading. To test "it is a weed and the detector is at least 90 % sure", read both, `label` and `confidence`, with the same sensor | A new syntax `measure … label … with confidence as …`: more grammar, same meaning |
| K5 | World file 1.7 | A sensor's `measures` may list the new kinds with a unit and an uncertainty, as in 1.2. An object's `properties` give the true value. As in 1.6, the world shown to the model never shows the property values | — |
| K6 | Stale labels | `u01_if_dirty` and `u02_if_damaged` (set "ask") still carry their 1.3 reason. Under 1.6 a label sensor expresses them; in their worlds no such sensor exists, so they are `infeasible` (S234). Relabel them in L3 by erratum | Leave them: they would stay in a gap that this iteration closes |
| K7 | Backends and Core | As in 1.6: Gazebo refuses `measure` (S509); bindings never appear in goals, so Core is unaffected | — |
| K8 | Not in 2.2 | Kinds the world names freely, a reading of a person (G-PROX), a reading of the robot's own battery (G-SELF, I8, which will reuse `fill`), counting objects chosen at run time and acting on each of them (G-CHOOSE), and arithmetic on readings (G-CALC) | — |
| K9 | Design for the far needs | G-SELF reads the battery as `measure r fill of r using …` or as a `charge` kind. Its decision (I8) chooses, and `%` is ready for it. G-PREDICT can return `confidence` | — |

## What it makes expressible

- **R3 probes:** `f_perceive_shelf_fill` and `f_perceive_weed_confidence`.
- **Requests from the sub-activities:** "read the gauge, tell me if it is above 6 bar", "tell the lead if the shelf
  is empty" (`count == 0`), "skip the plant unless the detector is at least 90 % sure".

## Pre-registration (L3), before any code

- **Set 11 "kinds":** the two probes, relabelled for 2.2, plus new tasks:
  - programs: one per kind, a unit conversion (kPa against bar), `count == 0`, confidence together with a label,
    a reading combined with 1.7–2.1 features, and decoys (a sensor that measures `fill` of another object);
  - infeasible: a sensor that does not measure the kind, a count with a unit;
  - inexpressible: a kind no sensor can declare (for example "smell"), the count of objects chosen at run time,
    an average of two readings;
  - controls.
- **Bounds:** fixed with the baseline (rule 6), including a B8-like measure: **a reading replaced silently**, for
  example by `inspect` (as in 1.6), by a label stand-in, or by nothing.
- **Economy:** 0 statements, 4 kinds, 1 unit token `%`, 3 pressure units, and about 3 codes.

## Open questions for the owner

1. K1: these four kinds (recommended), or fewer to start with?
2. K3: `==` on `count` (recommended)?
3. K6: relabel the stale `u01` and `u02` in this iteration?
4. Are K1–K9 accepted?
