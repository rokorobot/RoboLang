# RoboLang language roadmap

**Status:** PROVISIONAL, 2026-11-17. It is built from the atlas data of A1, A2 and R2. R3 confirms or refutes the
gaps with tests (rule LE-7). This page is revised after R3, then with each monthly proposal (R4). It sits at level 3 of
the [document map](LANGUAGE_EVOLUTION.md#document-map), next to [RSI-01](plans/rsi-01.md). It follows the
[strategy](LANGUAGE_EVOLUTION.md) and the [rules](LANGUAGE_EVOLUTION_RULES.md). The order comes from the generated
[suggested order](plans/humanoid-activities.md#suggested-order-of-language-work) and is not chosen by hand. Each
iteration still needs the owner's L1–L6 approvals.

## What the data says

- **No near sub-activity can be written in full in RoboLang 1.6.** Each of the 13 near sub-activities needs 3–4 gaps.
  A single gap rarely unlocks a sub-activity by itself, so the order is chosen by what the gaps unlock together.
- **Repetition is the backbone.** G-LOOP is needed by 11 of the 13 near sub-activities and by 21 of 38 in total.
  Shift work, patrols and harvesting are all repetition. It is out of scope since N4, so it needs a decision record
  (LE-9). The recommended design is a **mission layer** above tasks: missions repeat and react, and every task stays
  bounded and verified.
- **Perception and choice follow.** G-PERCEIVE (19 sub-activities) and G-CHOOSE (17) have the highest language need
  of any direction.

## Iterations

One gap per iteration (LE-10), each a full L1–L6 cycle ending in a release. "Unlocked" counts the near sub-activities
that are fully expressible once this gap and those above it are closed (13 near, 38 in total).

| # | Version | Gap | What it adds | Design for (far needs it must not rule out) | Unlocked near / all |
|---|---|---|---|---|---|
| I1 | 1.7 | G-PATH | Stay inside or outside a region while moving. A request checked against the controller's envelope, never enforced by RoboLang (LE-13). First because H4 needs it (D5, safety case G2) | G-PROX (a zone around a person), G-FORCE (the same "checked request" mechanism) | 0 / 0 |
| I2 | 1.8 | G-ACT | Actions beyond the seven: open, dock, charge, set a device. Declared per robot in its capability profile, typed, with a verified outcome | G-TEACH (a taught skill becomes a named action), G-SELF (dock and charge) | 0 / 0 |
| I3 | 1.9 | G-PERCEIVE | Conditions on perceived properties (ripe, dirty, damaged) with confidence. Unknown fails closed, as readings do in 1.6 | G-PREDICT (a predicted property), G-CHOOSE (choosing by property) | 1 / 1 |
| — | — | *decision record* | **Mission layer.** Written alongside I1 as design only; the owner decides whether to reopen G-LOOP | triggers for I5, I6, I8 | — |
| I4 | 2.0 | G-LOOP | Missions repeat bounded, verified tasks until a stop condition holds. A new layer, hence 2.0 | clock, message and self-state triggers | 2 / 2 |
| I5 | 2.1 | G-CLOCK | Clock readings and times of day, in conditions and as mission triggers | schedules across shifts | 3 / 4 |
| I6 | 2.2 | G-MSG | Waiting for a message (`message_received`, reserved since 1.3) and confirmed delivery | negotiation between robots (cooperation) | 5 / 8 |
| I7 | 2.3 | G-CHOOSE | Choosing objects or robots at run time (`detect`). Changes how goals name things: needs a Core decision | learned object categories | 9 / 15 |
| I8 | 2.4 | G-SELF | The robot's own state as readings (battery, health, faults); acting on it and asking for service | G-PREDICT (time to failure) | 10 / 19 |
| I9 | 2.5 | G-CALC | Minimal arithmetic, counting and text built from values | — | 12 / 22 |
| I10 | 2.6 | G-CONC | Several robots acting at the same time. The largest semantic change (Core sequencing), so last | shared workspaces with people | 13 / 26 |

The remaining 12 of the 38 sub-activities need far gaps (G-PROX, G-FORCE, G-TEACH, G-PREDICT). These get design
sketches only (LE-8). They are promoted to iterations when their sub-activities reach pilot within three years.

## How this page changes

- **After R3:** a refuted gap leaves the table and the order is regenerated. Each confirmed gap lists its probe tasks,
  which become the core of that iteration's pre-registered set (L3).
- **Weekly (R4):** the generated suggested order may move. This page is revised only by the monthly proposal, so the
  plan does not jump every week.
- **After each release (L6):** the gap closes in the atlas, the iteration is marked done here, and the next one
  starts at L1.

## Snapshots

Frozen, dated copies with a git tag: [roadmap-snapshots/](roadmap-snapshots/README.md). Latest:
[2026-11-17](roadmap-snapshots/2026-11-17.md) (tag `roadmap-2026-11-17`).

## Iteration status

| # | State |
|---|---|
| I1–I10 | Not started; waiting for R3 |
| Mission layer decision record | Not started |
