# RoboLang language roadmap

**Status:** PROVISIONAL, revised 2026-11-17 after R3. The tests of R3 confirm all nine near gaps, narrow
G-PERCEIVE, G-CHOOSE and G-SELF, and refute two sub-activity claims; a third refutation was withdrawn after the model run
([evidence](evidence/rsi-r3-gap-verification.md)). The model run meets the second half of LE-7: no model wrote a
program for any of the 25 probes. This page sits at level 3 of the [document map](LANGUAGE_EVOLUTION.md#document-map), next
to [RSI-01](plans/rsi-01.md). It follows the [strategy](LANGUAGE_EVOLUTION.md) and the
[rules](LANGUAGE_EVOLUTION_RULES.md). The order comes from the generated
[suggested order](plans/humanoid-activities.md#suggested-order-of-language-work) and is not chosen by hand. Each
iteration still needs the owner's L1–L6 approvals. The version before R3 is frozen in
[roadmap-snapshots/2026-11-17.md](roadmap-snapshots/2026-11-17.md).

## What the data says

- **No near sub-activity can be written in full in RoboLang 1.6.** Each of the 13 near sub-activities needs 3–4 gaps.
  A single gap rarely unlocks a sub-activity by itself, so the order is chosen by what the gaps unlock together.
- **Repetition is the backbone.** G-LOOP is needed by 11 of the 13 near sub-activities and by 21 of 38 in total.
  Shift work, patrols and harvesting are all repetition. It is out of scope since N4, so it needs a decision record
  (LE-9). The recommended design is a **mission layer** above tasks: missions repeat and react, and every task stays
  bounded and verified.
- **R3 moved perception down.** RoboLang 1.6 already expresses two of the perception claims with `measure … label`
  and `measure … distance`. G-PERCEIVE is now narrower: values beyond the four kinds. It moves from third to sixth
  place.
- **Each iteration starts with its probes.** The R3 probe tasks of a gap become the core of that iteration's
  pre-registered set (L3).

## Iterations

One gap per iteration (LE-10), each a full L1–L6 cycle ending in a release. "Unlocked" counts the near sub-activities
that are fully expressible once this gap and those above it are closed (13 near, 38 in total).

| # | Version | Gap | What it adds (R3 probes) | Design for (far needs it must not rule out) | Unlocked near / all |
|---|---|---|---|---|---|
| I1 | 1.7 | G-PATH | Stay inside or outside a region while moving (`avoid_wet_floor`, `stay_in_zone`, `keep_off_road`). A request checked against the controller's envelope, never enforced by RoboLang (LE-13). First because H4 needs it (D5, safety case G2) | G-PROX (a zone around a person), G-FORCE (the same "checked request" mechanism) | 0 / 0 |
| I2 | 1.8 | G-ACT | Actions beyond the seven: set a device, open, print a layout (`set_fryer`, `open_closet`, `print_layout`). Declared per robot in its capability profile, typed, with a verified outcome | G-TEACH (a taught skill becomes a named action), G-SELF (dock and charge) | 1 / 1 |
| I3 | 1.9 | G-CLOCK | Clock readings and times of day in conditions and messages (`scan_stamp`, `shift_cutoff`, `clean_at_2300`) | mission triggers by time (I4), schedules across shifts | 1 / 1 |
| — | — | *decision record* | **Mission layer.** Written alongside I1 as design only; the owner decides whether to reopen G-LOOP | triggers by time, message and self-state | — |
| I4 | 2.0 | G-LOOP | Missions repeat bounded, verified tasks until a stop condition holds. A new layer, hence 2.0 | the triggers of I3, I5, I8 | 2 / 2 |
| I5 | 2.1 | G-MSG | Waiting for a message (`message_received`, reserved since 1.3) and confirmed delivery (`line_docked`, `nurse_confirm`, `wind_clear`) | negotiation between robots (cooperation) | 4 / 5 |
| I6 | 2.2 | G-PERCEIVE | **Narrowed by R3:** perceived values beyond the four kinds of 1.6, such as a fill percentage or a detection confidence (`shelf_fill`, `weed_confidence`). Labels already cover classifications like "too dark" | G-PREDICT (a predicted property), G-CHOOSE (choosing by property) | 5 / 8 |
| I7 | 2.3 | G-CHOOSE | **Narrowed by R3:** properties of robots and locations, and choosing the nearest or best by them (`nearest_tugger`, `nearest_dock`, `nearer_cart`). Changes how goals name things: needs a Core decision | learned object categories | 9 / 15 |
| I8 | 2.4 | G-SELF | **Narrowed by R3:** the robot's battery as a reading and condition; acting on it and reporting it (`low_battery`, `enough_for_route`, `report_charge`). Own health is partly there (`sensor_ok`, `robot_ready`) | G-PREDICT (time to failure) | 10 / 19 |
| I9 | 2.5 | G-CALC | Minimal arithmetic, counting and text built from values (`total_weight`, `over_limit`, `text_line`) | — | 12 / 22 |
| I10 | 2.6 | G-CONC | Several robots acting at the same time (`carry_and_drive`, `two_hand_lift`). The largest semantic change (Core sequencing), so last | shared workspaces with people | 13 / 26 |
| I11 | 2.7 | G-PROX | **Proposed.** Limits and reactions that depend on people nearby: slow, stop or yield within a distance. A request checked against the controller's envelope, as the region limits of I1 (LE-13). Safety first, as I1 | shared workspaces with people (I10), G-PREDICT (a person's next move) | not computed |
| I12 | 2.8 | G-FORCE | **Proposed.** Grip and contact force limits, with the same "checked request" mechanism as I1 and I11 | taught skills (G-TEACH) that carry their own force limits | not computed |
| I13 | 2.9 | *G-TRIGGER* (new) | **Proposed.** A mission that starts by itself: at a time of day, on a message, or on the robot's own state. Left out of 2.0 (0041 M6); each trigger starts one bounded, approved mission | the "design for" of I3, I5 and I8 | not computed |
| I14 | 2.10 | *G-SHIFT* (new) | **Proposed.** Schedules beyond one day and across shifts. 1.9 reads one day only (`S238`, 0042 T4) | daylight-saving changes, calendars | not computed |

The remaining 12 of the 38 sub-activities need far gaps (G-PROX, G-FORCE, G-TEACH, G-PREDICT). These get design
sketches only (LE-8). They are promoted to iterations when their sub-activities reach pilot within three years.

**I11–I14 are proposed by the owner (2026-11-18), not generated.** Unlike I1–I10 they are not taken from the
[suggested order](plans/humanoid-activities.md#suggested-order-of-language-work), so they stay proposals until the
research confirms them:

- G-PROX and G-FORCE are far gaps in the atlas (R2 priority 0.84 and 0.50; G-FORCE 0.98 in the weekly R4 of
  2026-11-17). No near sub-activity needs them yet, so under LE-8 they get design sketches until one does.
- G-TRIGGER and G-SHIFT are not in the atlas. A foresight pass must add them, with probe tasks (R3), before L1.
- "Unlocked" is computed once the gaps are in the atlas.
- Candidates after I14, not yet on this page: G-TEACH, G-QUALITY (split off at I6), learned object categories,
  G-PREDICT, negotiation between robots, and shared workspaces with people.

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
| I1 G-PATH | L1–L5 done (accepted 2026-11-17, [acceptance](evidence/bench-1-7-acceptance.md): silent limit drops 3/27 → 0/27). **L6:** G-PATH closed in the atlas; release 0.4.0 with RoboLang 1.7 prepared, published by the owner (strategy D3) |
| I2 G-ACT | L1–L5 done (accepted 2026-11-17, [acceptance](evidence/bench-1-8-acceptance.md): silent action replacement 3/30 → 0/30). **L6:** G-ACT closed in the atlas; release 0.5.0 with RoboLang 1.8 prepared, published by the owner (strategy D3) |
| I3 G-CLOCK | L1–L5 done (accepted 2026-11-17, [acceptance](evidence/bench-1-9-acceptance.md)). **L6:** G-CLOCK closed in the atlas; release 0.6.0 with RoboLang 1.9 prepared, published by the owner (strategy D3) |
| I4 G-LOOP (mission layer) | L1–L5 done (accepted 2026-11-18 with one exception, [acceptance](evidence/bench-2-0-acceptance.md): repetition replaced silently 4/30 → 0/30). **L6:** G-LOOP closed in the atlas, G-UNBOUNDED split off (out, 0041 M2/M7); release 0.7.0 with RoboLang 2.0 prepared, published by the owner (strategy D3) |
| I5 G-MSG | L1–L5 done (accepted 2026-11-18, [acceptance](evidence/bench-2-1-acceptance.md): every bound met). **L6:** G-MSG closed in the atlas; release 0.8.0 with RoboLang 2.1, published after the owner approves the PyPI upload |
| I6 G-PERCEIVE | L1–L5 done (accepted 2026-11-18 with erratum P7, [acceptance](evidence/bench-2-2-acceptance.md): no reading replaced or changed, 0/30). **L6:** G-PERCEIVE closed in the atlas, G-QUALITY split off (far); release 0.9.0 with RoboLang 2.2, published after the owner approves the PyPI upload |
| I7 G-CHOOSE | L1–L2 done (0045 approved 2026-11-18: C1–C12 as recommended). **L3 done (draft for the owner):** L3: set 12 pre-registered; baseline next. 25 tasks ([plan](plans/bench-12-choose.md), 11 open points for the owner before L4, placeholder bounds B1–B9); erratum Q2 moves `x03_any_hot_pan` and `x03_carry_as_many_as_counted` to the new far gap G-EACH |
| I8–I10 | Not started |
| I11–I14 | Proposed 2026-11-18; not in the suggested order yet (see above) |
| Mission layer decision record | [0041](../spec/decisions/0041-mission-layer.md) **approved 2026-11-17**: G-LOOP reopened (LE-9), mission layer with a hard bound, `stop`/`continue`/`retry N`; built at I4 |
