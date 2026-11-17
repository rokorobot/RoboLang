# RoboLang 1.9 (delta over 1.8)

**Status:** front end, Task IR 1.8, simulation (world file 1.5, run record 1.8), `validate --world` and the IR reader
complete in code. The shipped language reference and the benchmark prompts teach 1.9 (L4 of decision 0042); the
acceptance run on benchmark set 8 is the next step (L5), and until it passes the bounds B1 to B8 1.9 is not accepted.
Decision: [0042](decisions/0042-clock-readings.md) (T1 to T12 and K1 to K11, approved 2026-11-17). Grammar:
[grammar-v1.9.ebnf](grammar-v1.9.ebnf). IR: [schema 1.8](ir-schema-v1.8.md). Simulation:
[section 17](simulation-v1.0.md). Everything not stated here is exactly as in [RoboLang 1.8](robolang-v1.8.md).

## 1. Version

`robolang 1.9;` selects 1.9. A file with this marker is read with the 1.8 rules, plus this document. A 1.8 front end
rejects it with `E213`. Files with no marker or with `1.1` to `1.8` are read, validated, compiled and run exactly as
before, byte for byte (Task IR 1.0 to 1.7, run records 1.0 to 1.7). In those files `measure r time as t;` is `E217` (an
unknown measured kind) and `22:00` is `E101` (an unexpected character), as before.

**Two new reserved words** (decision 0042 T6): `clock_time` and `clock_elapsed`. In a 1.9 file a parameter or binding
named `clock_time` is `E212`. The kind names `time` and `elapsed` are *not* reserved: they are recognised after
`measure r`.

## 2. Clock readings

The clock is read like any other value in 1.6, with `measure`, and two new kinds: **`time`**, the local time of day,
and **`elapsed`**, the time since the task started (T1). There is no new statement.

```
measure r time as t;
measure r elapsed as d;
```

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

- **`measure r time as t;`** and **`measure r elapsed as d;`** (T1). `r` is a `robot` parameter. There is no `of` and
  no `using`: the clock is not an object and has no sensor. The robot is recorded for the backends; in the simulation
  every robot has the world's clock (K1). The step takes a failure handler like `measure` does (1.5), and a scripted
  fault fails it. The name is bound as in 1.6 (once, visible after the step in its block and in nested blocks, V3).
- **A time of day** (T2) is a literal `HH:MM` or `HH:MM:SS`: 24-hour, two digits each, hours `00` to `23`, minutes and
  seconds `00` to `59`, local time in the world's zone. `22:00` is `22:00:00`. `24:00` is not a time (K4); neither is
  `9:30` or `21:60`: all are `E105` (and `5pm` is `E206`, a number with an unknown unit). A request in 12-hour form ("11 pm") is converted by the author, as °F is.
- **An `elapsed` value** compares with a duration written as today (`90 s`, `2 min`, `500 ms`; there is no unit for hours,
  so 2 hours are `120 min`). A binding of kind `elapsed` is the time since the task started: the clock starts when the
  run reaches `RUNNING` (K6).
- **Comparisons** (T3): `<`, `<=`, `>`, `>=` only. A `time` compares with a time literal or another `time` binding; an
  `elapsed` with a duration or another `elapsed` binding. `==` and `!=` on either kind are `E320`, as for any quantity
  (V6). **Kinds never mix**: a time against `5 s`, a time against an elapsed binding, a time against a temperature are
  `E319`. A comparison stands in `if` and `wait until` conditions only (`E321` in `pre` and `goal`), as in 1.6.
- **The live clock** (T5, T6). `wait until clock_time >= 23:00 timeout 12 h else { ... }` reads the clock whenever the
  wait is evaluated; `clock_elapsed` does so for elapsed time (`wait until clock_elapsed > 90 s ...`, K10). They are
  allowed **in `wait until` only**: anywhere else a live reading is `E322` (`measure` it, and compare the binding). The
  timeout stays mandatory (`E209`) and the handler too (`E316`). "At 11 pm, send the cleaner" is a bounded wait, then the
  steps. The wait ends **at the exact instant** the condition becomes true (K5), and a pending stop (0038) ends it as
  any wait.
- **Sending** (T8, K8): `communicate r to "lead" with t;` sends a `time` binding as `HH:MM:SS` and an `elapsed` binding
  in seconds with the unit, `37 s`. Another format ("just HH:MM", "in minutes") is arithmetic and formatting, not in 1.9.
- **One day** (T4): a time literal means that time **on the calendar day the task starts**, in the world's zone. See
  section 5.

### 2.1 What 1.9 cannot say

The grammar file says so too (it is the grammar-only prompt). Request any of these and the answer is "cannot":

- **another day** (T4, K4): "tomorrow at 01:00", "at midnight tonight" (midnight is the next day's 00:00), "on Monday",
  a date. `>= 01:00` is true at once on the start day, so it would be silently wrong. 1.9 has no date literal;
- **repeating at a time** (T11, 0041 M6): "every morning at 6", "every hour", "every night at 23:30". That is a mission
  trigger, not a step of a task;
- **arithmetic on times and durations** (T11, G-CALC): "how much time passed between the two scans" (the difference of
  two readings), "two hours after the scan finished" (a time plus a duration), "in 2 hours" from a moment that is not
  the start. A reading of `elapsed` is the time since the *start of the task*, never since another moment;
- **other formats** (T8, K8): a time sent as `HH:MM` or as "9:30 pm", a duration sent in minutes, a count of seconds
  without its unit;
- **a time zone in the request** ("23:00 UTC"): the zone is the world's (T11).

A `time` is never compared with `==` (E320) and never with a duration (E319).

## 3. Static rules (`validate`)

1. `measure r KIND as NAME` is a `robot` parameter and a name that is not a parameter (`E303`, `E304`, `E318`, as for 1.6).
2. A time literal is well formed and in range (`E105`, lexical).
3. The kind rules of 1.6 (V5, V6) extend to the two kinds: both sides of a comparison have the same kind (`E319`), the
   operator is an ordering (`E320`), a comparison is not in `pre` or `goal` (`E321`), and `clock_time` and
   `clock_elapsed` stand only in the condition of a `wait until` (`E322`).
4. A binding of kind `time` or `elapsed` can be sent in a message and compared; it cannot be sent as a string
   constant, nor compared with one (`E319`).
5. `clock_time` and `clock_elapsed` are reserved words (`E212` as a name).

## 4. Task IR

A clock reading is a step, a time literal and the live clock are operands: [IR schema 1.8](ir-schema-v1.8.md). A 1.9
source produces schema 1.8.

## 5. Checks and meaning in the simulation

[Section 17 of the simulation spec](simulation-v1.0.md) gives the clock of a world file 1.5, the exact readings and the
failures. In short:

- **`time`** is the world clock's `start` plus the simulated time (T7). A world with no `clock` has none: a `time`
  reading fails the step (`S237`, fail closed). `elapsed` needs no clock.
- **One day** (T4): once the clock reaches the end of the day the run started on, a `time` reading fails the step
  (`S238`), as 00:10 on the next day would make `< 22:00` true. A time literal is on the start day.
- **A time that has already passed** (K3): `wait until clock_time >= 09:00` (or `>`) started after 09:00 on the start day
  fails **before the run reaches `AUTHORIZED`** with `S010`, and in `validate --world`. "At 09:00" must never silently
  mean "now". "If it is already after 9, go now" is written with `measure` and `if`: a wait inside an `if` branch or a
  failure handler is not checked, as the branch may not run.
- Readings are exact: uncertainty 0 (T7).

| Code | Meaning |
|---|---|
| `S010` | A `wait until clock_time >= T` (or `> T`) whose T has already passed at the task start (K3) |
| `S237` | A `time` reading (`measure r time`, or `clock_time` in a `wait until`) in a world with no `clock` (T7) |
| `S238` | A `time` reading once the clock has reached the end of the start day (T4) |
| `E518` | World file 1.5: a `clock` whose `start` or `zone` is malformed |

## 6. Diagnostics

| Code | Stage | Meaning |
|---|---|---|
| `E105` | lexical | A time of day that is not `HH:MM` or `HH:MM:SS` in 24-hour form (`24:00`, `9:30`, `21:60`) |
| `E201` | syntax | (existing) `measure r time of x using s as t`: a clock kind is followed by `as`, with no `of` and no `using` |
| `E217` | syntax | (existing) an unknown kind after `measure r`; in 1.9 the message lists `time` and `elapsed` too |
| `E212`, `E303`, `E304`, `E318` | static | (existing) a reserved word, an undeclared or mistyped robot, a binding that clashes |
| `E319`, `E320`, `E321`, `E322` | static | (existing, 1.6) kinds that do not match, `==` on a time or an elapsed value, a comparison in `pre` or `goal`, a live clock outside `wait until` |
| `S010`, `S237`, `S238` | authorization, step | Section 5 |
| `E518` | world file | World file 1.5: a malformed `clock` |
| `S510` | ROS 2 backend | A clock reading is refused before any motion (T9) |

`E213` now lists `1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8 and 1.9`. In the Task IR reader the new forms are `I126` (a
time literal that is not `HH:MM:SS`) and the existing `I114`, `I115`, `I125` for a kind or a step kind that the schema
does not have.

## 7. Elsewhere (T9, T10)

- **ROS 2 / Gazebo backend:** refuses `measure r time` and `measure r elapsed` with `S510` before any motion. T9 allows
  `elapsed` from `/clock` "if straightforward": it is not, because this backend has no value bindings (it already
  refuses every `measure`, `S509`) and no message sending, so `elapsed` alone would need both. It has no wall clock, so
  `time` is refused in any case. Revisit with value bindings.
- **Core, `export-goal`, `delegate`:** unaffected. A goal never holds a binding (0036), so a clock reading is never in
  one; `delegate` does not delegate a plan that has a `measure` or a clock reading (`measure_unsupported`).
- **Safety** (T10): a wait for a time of day keeps the robot idle and holding its state; the timeout bounds it, and a
  pending stop (0038) still ends it. On hardware the clock source must be trustworthy (NTP or the controller's clock):
  hazard **H21**, "a wrong clock makes a time condition decide wrongly", is controlled by fail-closed reading (`S237`,
  `S238`, `S010`) and by commissioning (G5). The simulation's clock is exact by construction, so its readings say
  nothing about a real clock.

## 8. Compatibility

Nothing changes for 1.1 to 1.8 files, their IR, their run records, or their world files 1.0 to 1.4. A world file 1.5 may
serve a task of any language version; its `clock` is read only by a task with a clock reading. A task with a `time`
reading run against a world 1.0 to 1.4 fails closed with `S237`: those worlds have no clock.
