# RoboLang language roadmap

**Status:** PROVISIONAL, revised 2026-11-17 after R3; I11–I20 ranked on 2026-11-24 by R3b. The tests of R3
confirm all nine near gaps, narrow
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
| I11 | 2.7 | G-TRIGGER | **Ranked by R3b, near.** A mission that starts a run by itself: at a time, every N hours, on a message or on the robot's own state (0041 M6, clarification 14). Probes `f_trigger_rounds_every_hour`, `f_trigger_scan_every_two_hours`, and 11 standing tasks. Each trigger starts one bounded, approved run (0041 M2) | G-DATE (daily and weekly schedules), G-SELF (a self-state trigger) | priority 3.11, near |
| I12 | 2.8 | G-DATE | **Ranked by R3b, near.** Dates and times on another day: tomorrow, a weekday, a night that runs past midnight. 1.9 reads the day the task starts only (`S238`, 0042 T4, K11). Probes `f_date_corridor_after_midnight`, `f_date_scan_tomorrow_at_0600` | daylight-saving changes, calendars | priority 1.76, near |
| I13 | 2.9 | G-FORCE | **Ranked by R3b, far.** Grip and contact force limits, with the same "checked request" mechanism as I1 | taught skills (G-TEACH) that carry their own force limits | priority 0.98 |
| I14 | 2.10 | G-PROX | **Ranked by R3b, far.** Limits and reactions that depend on people nearby: slow, stop or yield within a distance. A request checked against the controller's envelope, as the region limits of I1 (LE-13) | shared workspaces with people (I10), G-PREDICT (a person's next move) | priority 0.97 |
| I15 | 2.11 | G-TEACH | **Ranked by R3b, far.** A skill, object or route taught by demonstration, then used by name. A taught skill becomes a declared action (I2), so it stays typed, checked and verified | skills shared across a fleet, learned object categories | priority 0.65 |
| I16 | 2.12 | G-PREDICT | **Ranked by R3b, far.** Acting on a prediction and its confidence: a person's next move, time to failure, or a plan checked before acting. Plan section 9 keeps probabilistic prediction out of `.rob` today, so it needs a decision record first | the "design for" of I6 and I8 | priority 0.50 |
| I17 | 2.13 | G-QUALITY | **Ranked by R3b, no claimant.** Judgements of appearance or condition that no sensor kind declares ("the packaging looks worn"). Split off from G-PERCEIVE at I6 (0044 D14) | G-PREDICT (a predicted condition) | priority 0.00 |
| I18 | 2.14 | G-WORKSPACE | **Ranked by R3b, no claimant.** Limits on the arm's workspace, not only on where the robot drives. Split off from G-PATH at I1 (0039 P3) | G-PROX (a person inside the arm's reach) | priority 0.00 |
| I19 | 2.15 | G-STEPLIMIT | **Ranked by R3b, no claimant.** A region or speed limit on one step or one stretch of a route, not the whole task. Split off from G-PATH at I1 (0039 P2) | G-PROX (a slower stretch near people) | priority 0.00 |
| I20 | 2.16 | G-OBSERVE | **Ranked by R3b, no claimant.** Tasks that only observe (inspect, scan, wait) and ask for no end state, so a goal must not be invented. From the 2.0 acceptance (2026-11-18) | — | priority 0.00 |

The remaining 12 of the 38 sub-activities need far gaps (G-PROX, G-FORCE, G-TEACH, G-PREDICT). These get design
sketches only (LE-8). They are promoted to iterations when their sub-activities reach pilot within three years.

**I11–I20 are ranked by the R3b pass (2026-11-24, [evidence](evidence/rsi-r3b-new-gaps.md)), not chosen by hand.**
Near gaps come first, in the generated order; far gaps follow by their priority in the atlas. For these rows the last
column is the gap's atlas priority, not an R3 unlock count.

- **G-TRIGGER and G-DATE are near.** Three deployed sub-activities (hospital runs, shelf scanning, night cleaning) need
  a run to start at a time, and two of them on every day. Their needs were dropped instead of moved to G-TRIGGER when
  G-LOOP closed at I4; R3b claims them again with probes. The generated
  [suggested order](plans/humanoid-activities.md#suggested-order-of-language-work) now places G-TRIGGER third, after
  G-CHOOSE (I7) and G-SELF (I8) and ahead of G-CALC (I9) and G-CONC (I10), and G-DATE fifth. Moving them ahead of I9
  and I10 is for the monthly proposal; this page does not reorder I7–I10.
- **G-FORCE, G-PROX, G-TEACH and G-PREDICT are far.** No near sub-activity needs them, so under LE-8 they get design
  sketches until one does. G-FORCE and G-PROX are tied (0.98 and 0.97).
- **G-QUALITY, G-WORKSPACE, G-STEPLIMIT and G-OBSERVE have no claimant:** no sub-activity needs them. They hold I17–I20
  until a foresight pass finds one, and leave if a new gap overtakes them.
- **Refuted by R3b, not in the atlas:** G-SHIFT is G-DATE. G-CATEGORY, G-NEGOTIATE and G-SHARED, proposed on
  2026-11-18, name nothing the atlas lacks: learned categories are G-TEACH and G-CHOOSE, negotiation between robots is
  G-CHOOSE and G-CONC, and shared workspaces are G-PROX, G-FORCE and G-CONC.

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
| I7 G-CHOOSE | L1–L2 done (0045 approved 2026-11-18). **L3** done: set 12 "choose" (25 tasks, [plan](plans/bench-12-choose.md)); baseline on 2.2 ([evidence](evidence/bench-set12-baseline.md)); open points 1–11, errata Q3/Q4 and bounds B1–B9 approved 2026-11-19. **L4 done; L5 acceptance run next:** RoboLang 2.3 built and taught (the reference and both prompts teach 2.3; set 12 is labelled for 2.3, the R3 probes and three standing tasks are programs; [errata](evidence/bench-errata-2-3.md)). Closing G-CHOOSE is L6 |
| I8–I10 | Not started |
| I11–I20 | Ranked 2026-11-24 by R3b ([evidence](evidence/rsi-r3b-new-gaps.md)): I11 G-TRIGGER and I12 G-DATE near, the rest far. Not started |
| Mission layer decision record | [0041](../spec/decisions/0041-mission-layer.md) **approved 2026-11-17**: G-LOOP reopened (LE-9), mission layer with a hard bound, `stop`/`continue`/`retry N`; built at I4 |
