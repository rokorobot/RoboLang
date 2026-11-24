# RSI-01 R3b: verifying the gaps proposed for I11–I20

Date: 2026-11-24. Set `future` (`python bench/run.py --set future`): 4 new probes, now 15 `inexpressible` in all.
Yardstick: [language-reference.md](../language-reference.md), RoboLang 2.2. No model was run and no money was spent.
Rule: [LE-7](../LANGUAGE_EVOLUTION_RULES.md). Why: the roadmap's I11–I20 (2026-11-18) named five gaps the atlas did not
have (G-TRIGGER, G-SHIFT, G-CATEGORY, G-NEGOTIATE, G-SHARED) and had no priorities for any of them.

## Method

- **Same rules as [R3](rsi-r3-gap-verification.md), with 2.2 as the yardstick.** A gap holds when a sub-activity's
  request needs it and no other gap in the atlas covers it. A probe is one request, from or close to a near
  activity's `requests`, that needs exactly that one gap. Its obvious attempt must be rejected by `check_answer`; the
  first code is recorded. With the offending part made harmless, the attempt is a valid 2.2 program, so it fails for
  the gap and not for a slip.
- **Each proposed name was first checked against the atlas.** Two were already there under the same or another name.
  The other three were checked against the 117 requests of the 40 sub-activities.

## Per proposed gap

| Proposed | Verdict | Why |
|---|---|---|
| G-TRIGGER | **in the atlas, near again** | Open since 0041 clarification 14, with 11 standing tasks, but no sub-activity claimed it. When G-LOOP closed at L6 of I4, the trigger needs were dropped instead of moved here (see below) |
| G-SHIFT | **is G-DATE** | G-DATE, "dates and times on another day", was split off G-CLOCK by 0042 K11. "Schedules across shifts" is G-DATE for the day and G-TRIGGER for the repetition |
| G-CATEGORY | refuted | No request needs a category the robot learns. "Every toy on the floor" and "the ripe tomatoes" are objects chosen at run time (G-CHOOSE) read with a declared classifier label. "Learn what a normal pallet count looks like" and "this valve" are taught (G-TEACH) |
| G-NEGOTIATE | refuted | No request needs robots to bargain. "The nearest free robot, whichever vendor made it" is G-CHOOSE. "Only one robot in the aisle at a time" is a constraint between robots acting at once (G-CONC). "Hand the rest of the run to the next free robot and leave after it confirms" is G-CHOOSE with a 2.1 message |
| G-SHARED | refuted | Robots beside people is ACT-SAFE-001, which needs G-PROX and G-FORCE; several robots at once is G-CONC. No request needs anything beyond these three |

## The dropped trigger needs

Before G-LOOP closed, these requests were counted under G-LOOP. 2.0 missions express a fixed count, a duration or an end
time, but a mission starts each run when the previous one ends (0041 M6). A run at a time of day, every N hours on the
hour, or on a message is a trigger. 2.2 still declines every one of them, yet the sub-activities that ask for them list
no open gap for it, so the atlas counted them as unlocked.

| Sub-activity | Maturity, horizon | Request | Claimed again |
|---|---|---|---|
| ACT-HLTH-001 hospital delivery | deployment, now | "Every hour, bring the lab samples from the ward to the lab." | G-TRIGGER |
| ACT-RET-001 shelf scanning | deployment, now | "Every morning at 6, scan all aisles…" | G-TRIGGER, G-DATE |
| ACT-CLN-001 floor cleaning | deployment, now | "Clean the lobby every night at 11 pm…" | G-TRIGGER, G-DATE |
| ACT-INSP-001 inspection rounds | demo, now | "Walk the boiler-room route every hour…" | G-TRIGGER |
| ACT-SEC-001 security patrol | demo, 1-3y | "Patrol the yard every two hours until 6 am." | G-TRIGGER, G-DATE |
| ACT-AGR-001 greenhouse harvest | pilot, 3-7y | "Harvest on Tuesday and Thursday evenings." | G-TRIGGER, G-DATE |

Not claimed: ACT-EV-001 "plug in the next car when its driver asks" can be read as one wait for a message (2.1) plus
G-CHOOSE; ACT-SOL-001 "stop when the wind alarm sounds" interrupts a run rather than starting one. Each activity's
`basis` records the claim.

## Probes

| Gap | Probe (activity) | Attempt fails with | Harmless stand-in that makes it valid |
|---|---|---|---|
| G-TRIGGER | `f_trigger_rounds_every_hour` (HLTH-001): a round every hour on the hour until 13:00 | E201 (`repeat every`) | `repeat for at most 180 min` |
| | `f_trigger_scan_every_two_hours` (RET-001): a scan every two hours until 20:00 | E201 (`repeat every`) | `repeat for at most 300 min` |
| G-DATE | `f_date_corridor_after_midnight` (CLN-001): the lobby at 23:00, the hall at 01:00 | S010 (01:00 has passed today) | 23:30 for 01:00 |
| | `f_date_scan_tomorrow_at_0600` (RET-001): tomorrow at 06:00, asked at 14:20 | S010 (06:00 has passed today) | 15:00 for 06:00 |

The G-DATE attempts are the dangerous shape: they parse, and only the pre-run check S010 (0042 K3) stops a wait for a
time that has passed. The hospital probe uses a new world copy, `ward_1_5_0950` (`ward_1_4` with a clock at 09:50);
the others reuse the R3 clock worlds.

## What changes

- **Atlas:** the six claims above; G-TRIGGER and G-DATE each list their two probes, and their scope notes say they
  are near. A development log entry for 2026-11-24.
- **Generated order** ([plan](../plans/humanoid-activities.md#suggested-order-of-language-work)):

  | # | Before | After |
  |---|---|---|
  | 1 | G-CHOOSE (unlocks 5.95) | G-CHOOSE (3.50) |
  | 2 | G-SELF (7.45) | G-SELF (5.00) |
  | 3 | G-CALC (8.62) | **G-TRIGGER** (6.35) |
  | 4 | G-CONC (9.62) | G-CALC (7.52) |
  | 5 | — | **G-DATE** (8.62) |
  | 6 | — | G-CONC (9.62) |

  G-TRIGGER's priority is 3.11, second only to G-CHOOSE (5.23). Every "unlocked" value before G-CONC falls: the atlas
  had overstated what I7–I9 unlock.
- **Roadmap:** I11–I20 are ranked from the atlas: near gaps first in the generated order (G-TRIGGER, G-DATE), then far
  gaps by priority (G-FORCE 0.98, G-PROX 0.97, G-TEACH 0.65, G-PREDICT 0.50), then the open gaps no sub-activity
  claims (G-QUALITY, G-WORKSPACE, G-STEPLIMIT, G-OBSERVE). The roadmap does not move them ahead of I9 and I10; the
  monthly proposal decides that.

## Not done, and follow-ups

- **No model run.** As in R3, the probes go to the `Benchmark` workflow with `set_only: future` (about $0.20). Until
  then, only the language side is verified, not whether models decline these requests.
- **Follow-up: a check at L6.** When a gap closes, a test should fail if a sub-activity loses a need that its
  requests still show. The dropped trigger needs went unnoticed for six days.
