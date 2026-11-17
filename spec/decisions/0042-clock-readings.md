# Decision record 0042: Clock readings (I3, G-CLOCK, RoboLang 1.9)

**Date:** 2026-11-17
**Status:** **APPROVED by the owner, 2026-11-17**: one day only (T4), the live names `clock_time` and `clock_elapsed`
(T6), and T1–T12 as recommended. This is step L2 of iteration I3 in the
[language roadmap](../../docs/LANGUAGE_ROADMAP.md); the owner chose the gap (L1) on 2026-11-17.
**Applies to (when decided):**
- new `spec/robolang-v1.9.md` and `grammar-v1.9.ebnf`;
- Task IR 1.8 and world file 1.5;
- `simulation-v1.0.md` 4.2 and `src/robolang/`;
- `docs/language-reference.md`;
- the ROS 2 backend's refusal list;
- `bench/` (the L3 set).

**Evidence:**
- **Atlas:** G-CLOCK is the first open gap in the generated
  [suggested order](../../docs/plans/humanoid-activities.md#suggested-order-of-language-work). Eleven sub-activities
  need it, five of them near: fulfilment, sequencing, retail scanning, hospital runs and night cleaning.
- **R3 probes:** `f_clock_scan_stamp`, `f_clock_shift_cutoff` and `f_clock_clean_at_2300`. Each attempt fails, and
  both models declined all three.
- **Standing tasks:** `x_elapsed_time`, `x_report_duration`, `u17_at_1400` and `x05_cooling_time`, plus set 7's
  `x05_open_at_six`.

## The problem

A task cannot read a clock. Waits are relative (`wait for 2 s`), and the simulated clock starts at 0, so the plan
knows no time of day. Three kinds of request need a clock:

- **time of day as a value:** "tell the lead the time the scan finished";
- **a condition on the time of day:** "only if the shift has not ended at 22:00";
- **waiting for a time of day:** "at 11 pm, send the cleaner";
- **elapsed time:** "how many seconds the job took".

Repeating at a time ("every morning at 6", "every hour") is a different thing. It is a **mission trigger** (0041 M6)
and is not part of this record.

## The idea in one paragraph

The clock is read like any other value in 1.6, with `measure`, using two new kinds:

- `time`: the local time of day;
- `elapsed`: the time since the task started.

Time of day gets a literal, `22:00`. Everything else follows 1.6 unchanged:

- a reading binds a name;
- `if` compares a binding;
- `wait until` may read the clock live;
- a binding can be sent in a message.

There is no new statement and no new keyword.

```rob
robolang 1.9;

task tote_if_open(robot r, object tote, location inb, location outb) {
    goal { object_at tote outb; }
    plan {
        measure r time as now;
        if now < 22:00 {
            move r to inb; grasp r tote; move r to outb; place r tote at outb;
        } else {
            communicate r to "lead" with "the shift is over";
            fail "the shift has ended";
        }
    }
}
```

## Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| T1 | Construct | `measure r time as t;` and `measure r elapsed as d;`: the 1.6 statement, with no `of … using …` for the two clock kinds. They are read from the robot's clock, and the record names the robot | A new statement `read clock as t;` adds a statement and a keyword. Built-in names `now` and `elapsed`: values that change without a step, which 0036 V2 rules out |
| T2 | Literals | A time of day is `HH:MM` or `HH:MM:SS`, 24-hour, local time in the world's zone. An `elapsed` value compares with durations as they are written today (`90 s`, `2 min`, `1 h`) | `11 pm` (12-hour) is ambiguous and English-only, so the author converts it, as °F is converted today |
| T3 | Comparisons | `time`: `<`, `<=`, `>`, `>=` against a time literal or another `time` binding. `elapsed`: the same against a duration literal or an `elapsed` binding. `==` is not allowed for either (as 0036 V6). Kinds never mix (E319) | `==` on times: almost never true at the instant it is checked |
| T4 | Days | **One day only.** A time literal means that time on the **calendar day the task starts**, in the world's zone. A task that would need a time on the next day ("at 01:00 tonight", started at 21:00) cannot be written in 1.9. Dates come with mission schedules | "The next occurrence after the start": with that rule, "only if the shift has not ended at 22:00" read at 23:00 would mean tomorrow's 22:00, which is silently wrong |
| T5 | Waiting for a time | `wait until clock_time >= 23:00 timeout 12 h else { … }` reads the clock live, like a live reading in 1.6 (V4). The timeout stays mandatory (E209). "At 11 pm, send the cleaner" is a bounded wait, then the steps | Start the task at a time from outside: that is a mission trigger, which comes later |
| T6 | Live clock reading | In `wait until` only. The live form is `clock_time` for the time of day and `clock_elapsed` for elapsed time. Elsewhere a live reading is E322 (as in 1.6), so `if` uses a `measure`d binding. Two reserved words | Allow live readings in `if`: the record could not say which instant decided |
| T7 | World and simulation | World file 1.5: `"clock": { "start": "2026-11-17T21:30:00", "zone": "Europe/Prague" }`. Simulated time 0 is that instant; `time` is start + simulated time, and `elapsed` is the simulated time. With no `clock`, a `time` reading fails the step (fail closed) and `elapsed` still works. Readings are exact (uncertainty 0) | Uncertainty from clock sync: no task needs it yet |
| T8 | Messages | A `time` binding is sent as `HH:MM:SS`. An `elapsed` binding is sent in seconds with its unit (`37 s`), as 1.6 sends a binding with its unit | Formatting ("37 seconds"): G-CALC |
| T9 | Backends and Core | The ROS 2 / Gazebo backend supports `elapsed` from `/clock` (simulated time) and refuses `time`: it has no wall clock. Goals never hold bindings (0036), so Core and `delegate` are unaffected | Read the host's wall clock in Gazebo: it mixes simulated and real time |
| T10 | Safety | A wait for a time of day keeps the robot idle and holding its state. The timeout bounds the wait, and a pending stop (0038) still ends it. On hardware the clock source must be trustworthy (NTP or the controller's clock). Hazard H21 in the safety case: "a wrong clock makes a time condition decide wrongly"; it is controlled by fail-closed reading and commissioning (G5) | None |
| T11 | Not in 1.9 | Dates and times on another day (T4), repeating at a time (mission triggers, 0041 M6), arithmetic on times ("how long between the two scans", G-CALC), and time zones per task | — |
| T12 | Design for the far needs | Mission triggers (`when at 06:00`, `every 1 h`) will reuse the `time` literal and kind. Dates extend T4 without changing `HH:MM`. G-SELF (time to failure) and G-PREDICT can return `elapsed` values | — |

## What it makes expressible

- **The three R3 probes:** `f_clock_scan_stamp`, `f_clock_shift_cutoff`, and `f_clock_clean_at_2300` (when the start
  is earlier than 23:00 on the same day).
- **Standing tasks:**
  - `x_elapsed_time` and `x_report_duration`: elapsed time since the task started;
  - `u17_at_1400`: a bounded wait;
  - set 7's `x05_open_at_six`.

**Still inexpressible:**
- `u18_every_morning`, which is a mission trigger;
- the repeating requests of the near sub-activities ("every night at 11 pm"), which are missions (I4);
- durations between two arbitrary moments, which needs G-CALC.

## Pre-registration (L3), before any code

- **Set 8 "clock":** the probes and standing tasks above, relabelled for 1.9, plus new tasks of four kinds:
  - **programs:** a time stamp in a message, a shift cutoff, a bounded wait for a time, elapsed time of the whole
    task, a time window with `and`, two robots;
  - **infeasible:** the world has no clock for a `time` reading; the time has already passed and the wait has no
    `else`; the timeout is shorter than the wait needed;
  - **inexpressible:** every morning, tomorrow at 01:00, the time between two scans, "in 2 hours" from an arbitrary
    point;
  - **controls.**
- **Bounds:** fixed with the baseline (rule 6), including a B8-like measure: **a time silently replaced**, for
  example by a `wait for` of a guessed duration, or by dropping the time condition.
- **Economy:** 0 new statements, 2 kinds, 1 literal, 2 reserved words for the live readings (T6), 1 world field and
  about 4 codes.

## Owner's answers (2026-11-17)

1. One day only in 1.9 (T4).
2. The live names stay `clock_time` and `clock_elapsed` (T6).
3. T1–T12 are accepted.

## Clarifications (APPROVED by the owner 2026-11-17, from writing the L3 set)

| # | Question | Recommended |
|---|---|---|
| K1 | Which clock `wait until clock_time …` reads | The world clock (T7). In the simulation every robot's clock is the world clock, so `measure r time` and `clock_time` agree. The robot named in `measure` is recorded for the backends |
| K2 | A time that has already passed, with no else path (`i02`, `i05`) | `cannot / infeasible`, caught before the run by K3 |
| K3 | `wait until clock_time >= 09:00` started at 10:30 | **Fails before the run** with a new S-code ("the time has already passed"): "at 09:00" must never silently mean "now". A request "if it is already after 9, go now" is written with `measure` and `if` |
| K4 | "Midnight" and "tonight" | Midnight belongs to the next day, so it is inexpressible in 1.9 (T4). `24:00` is not a valid literal |
| K5 | When a wait for a time ends | At the exact instant the condition becomes true: the simulation knows it, because time advances continuously |
| K6 | When `elapsed` starts | Simulated time 0, when the run reaches `RUNNING`, before `pre` is checked |
| K7 | The standing task `x05_cooling_time` | **Stays inexpressible** (G-CALC). The probe is a contact sensor, so the arm moves first. "How long it took to cool" is the time between two moments, and a reading of `elapsed` since the start would include the move on a real robot. 0042's list of expressible tasks is corrected |
| K8 | Message formats (T8) | `time` as `HH:MM:SS` and `elapsed` in seconds with the unit (`37 s`). Asking for another format ("just HH:MM") is G-CALC |
| K9 | Worlds with no clock | The world shown to the model says "no clock", so `i01` and `i04` can be solved |
| K10 | `clock_elapsed` in `wait until` | Kept as approved (T6), with no task of its own. `==` stays excluded (T3) |
| K11 | Atlas gaps for `x02` (tomorrow) and `x05` (Monday) | A new far gap, **G-DATE** (dates and times on another day), so that G-CLOCK can close at L6 |

## Implementation note (L4, 2026-11-17; for the owner)

Built in PR #155. T2 and T5 write `1 h` and `12 h` "as durations are written today", but 1.1–1.8 have only `ms`, `s`
and `min`. L4 did **not** add an hour unit, so the economy of B7 is unchanged. A request in hours is written in
minutes (`720 min`), as the reference teaches; the scorer compares durations, so `1 h` in a label accepts `60 min`.
Adding `h` is one token and waits for the owner's decision.
