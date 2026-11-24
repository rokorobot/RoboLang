# Decision record 0048: The robot's own state (I8, G-SELF, RoboLang 2.4)

**Date:** 2026-11-24
**Status:** **APPROVED by the owner, 2026-11-24**: S1–S11 as recommended. The kind is `charge` (S1), charge is
allowed in `where` (S7), the simulation has scheduled events only (S4), and G-HEALTH is split off (S10). This is step L2 of iteration I8 in the
[language roadmap](../../docs/LANGUAGE_ROADMAP.md). L1 is done: G-SELF is first in the generated
[suggested order](../../docs/plans/humanoid-activities.md#suggested-order-of-language-work) (it unlocks the most).
**Applies to (when decided):**
- new `spec/robolang-v2.4.md` and `grammar-v2.4.ebnf`;
- Task IR 1.12 and world file 1.9;
- the simulator (`src/robolang/sim/`);
- `docs/language-reference.md`;
- the ROS 2 backend's refusal list;
- `bench/` (the L3 set).

**Evidence:**

- **Atlas:** five sub-activities need G-SELF:
  - self-charging on long inspection missions (ACT-AUTO-001, near: deployed now);
  - battery swapping for whole-shift humanoid work (ACT-AUTO-002, 1–3 years);
  - self-diagnosis and service requests (ACT-AUTO-003);
  - mid-shift relief by a peer (ACT-AUTO-004);
  - remote takeover (ACT-HRC-003).
- **R3 probes:** `f_self_low_battery` ("if your battery is below 25 percent, go to dock_1"), `f_self_enough_for_route`
  ("check that spot1 has at least 60 percent charge") and `f_self_report_charge` ("tell the lead how much charge you have
  left"). Every attempt fails, and both models declined all three.
- **Standing tasks in the gap:** `x02_own_battery_first` (set 11) and `x02_selftest_result` (set 7).
- **R3 narrowed the gap** to the battery. Some of a robot's own health is already expressible: `robot_ready` and
  `sensor_ok`.

## The problem

A task can read an object's temperature, fill or pressure (1.6, 2.2) and the clock (1.9), but not the robot's own charge.
A request such as "if your battery is low, go and charge first" must therefore be declined, or a model drops the
condition and the robot goes on. 1.8 can already ask a robot to `do r charge charger`, but nothing can check how much
charge the robot has.

## The idea in one paragraph

Read the robot's charge with `measure`, as the clock is read in 1.9:

- `measure r charge as c;` binds the robot's own charge in `%`;
- `if` compares the binding;
- a message can send it;
- `wait until` can read it live.

There is no new statement. The charge comes from the robot, not from a sensor parameter, so there is no `of … using …`.

```rob
robolang 2.4;

task round_or_dock(robot r, location dock, location stop) {
    goal { robot_at r stop; }
    plan {
        measure r charge as c;
        if c < 25 % {
            goal { robot_at r dock; }
            move r to dock;
            communicate r to "lead" with "spot1 docked to charge";
        } else {
            move r to stop;
        }
    }
}
```

## Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| S1 | Construct | A kind **`charge`** for the robot itself: `measure r charge as c;`, with no `of … using …`, as `time` and `elapsed` in 1.9 (0042 T1). The value is in `%`, 0 to 100, the unit token of 2.2 | `measure r fill of r using …` (0044 K9): the robot is not an object (D9, `E304`), and a battery is not a sensor parameter the request names |
| S2 | Comparisons | `<`, `<=`, `>`, `>=` against a `%` literal or another `charge` binding, with no `==`, as `fill` in 2.2. An unknown or uncertain comparison fails closed (1.6 V7). Comparing it with another kind is `E319` | `==`: a charge is never exactly a value |
| S3 | Live reading | In `wait until` only, as `robot_charge r >= 80 %`. It is recognised by position and is not reserved (0044 D10, 0045 point 10). Elsewhere a live reading is `E322`, as in 1.6 and 1.9 | Live readings in `if`: the record could not say which instant decided (0042 T6) |
| S4 | World and simulation | World file 1.9 gives a robot `"charge": { "value": "64", "unit": "%", "uncertainty": "1" }`. A robot without it has an unknown charge, so a reading fails the step (fails closed). The charge changes only by **scheduled events** (`{ "at", "robot", "charge" }`, as messages in 1.6). There is no drain or charging model | A drain per metre and a charging rate: "enough for the route" would then need a prediction, which is G-PREDICT and G-CALC. The simulator would also claim a precision it does not have |
| S5 | Messages | A `charge` binding is sent with its unit (`64 %`), as 2.2 sends `fill` (0036 V13) | A formatted text ("64 percent"): G-CALC |
| S6 | Missions | `robot_charge r` may stand in a mission's `until` ("patrol until the battery is below 30 %"). It is read live after each run (0041 clarification 2), and an unknown value ends the mission `failed`. It is not a bound by itself: M2 still needs a count, a duration or a time | A self-state **trigger** (`when robot_charge r < 20 %`): G-TRIGGER, a later iteration (0041 M6) |
| S7 | Choosing (0045) | `where robot_charge t >= 50 %` is **allowed**. The charge of a candidate robot is state that the fleet manager reports, a world fact, unlike a sensor reading of an object (0045 point 5, `E330`). Mid-shift relief needs it: "send whichever ready robot has at least 50 % charge" | Keep `where` free of charge: ACT-AUTO-004 could not choose a peer |
| S8 | Backends and Core | The ROS 2 / Gazebo backend refuses `charge` in 2.4, because no battery topic is defined in its profile yet (`S509` for `measure`). Goals never hold bindings (0036), so Core and `export-goal` are unaffected. `delegate` refuses a task that reads the charge, as it refuses any `measure` | Map it to `sensor_msgs/BatteryState`: this belongs with the backend's own step (H4) |
| S9 | Safety | Hazard **H28: a wrong or stale charge reading lets a robot go on with a low battery.** Controls: an unknown charge fails closed; the uncertainty makes a comparison near the threshold unknown; and **the robot's own low-battery protection always takes precedence**. RoboLang asks; it never overrides the controller (LE-13, as with region limits in 1.7) | Treat the RoboLang check as the protection: a task could then be the only thing between a robot and a flat battery |
| S10 | Not in 2.4 | Health beyond `robot_ready` and `sensor_ok`: faults, self-test results, service requests (**G-HEALTH**, a new far gap). Time to empty and energy per route (G-PREDICT). Arithmetic on charge (G-CALC). Self-state triggers (G-TRIGGER) | — |
| S11 | Design for the far needs | G-TRIGGER reuses `robot_charge r` in `when …`. G-HEALTH reuses `measure r KIND as …` for other self-state kinds. G-PREDICT can return an `elapsed` "time to empty" | — |

## What it makes expressible

- **The three R3 probes**, and `x02_own_battery_first`.
- **Battery swapping:** a 1.8 declared action and a check of the charge afterwards (ACT-AUTO-002).
- **Mid-shift relief:** choosing a peer by charge (S7), as far as G-CONC allows (ACT-AUTO-004).

**Stays out:** `x02_selftest_result` and self-diagnosis go to G-HEALTH; "do I have enough for the route" in the sense of
a prediction goes to G-PREDICT.

## Open points (answered 2026-11-24)

1. **Kind name (S1):** `charge`, rather than reusing `fill` as 0044 K9 suggested? I recommend `charge`.
2. **Charge in `where` (S7):** allow it, as a world fact from the fleet manager? I recommend yes, for mid-shift relief.
   It narrows 0045 point 5, which keeps every reading out of `where`.
3. **Simulation (S4):** scheduled charge events only, with no drain model? I recommend events only.
4. **G-HEALTH:** split off faults and self-tests now as a far gap, so that G-SELF can close at L6? I recommend yes.

## Owner's answers (2026-11-24)

The owner approved S1–S11 as recommended. This also answers the four open points:

1. The kind is `charge` (S1).
2. Charge is allowed in a `choose … where` (S7). This narrows 0045 point 5.
3. The simulation has scheduled charge events only (S4).
4. G-HEALTH is a new far gap (S10).

## Pre-registration (L3), before any code

- **Set 13 "self":** the three probes and `x02_own_battery_first`, relabelled for 2.4, plus new tasks of four kinds:
  - **programs:**
    - charge below a threshold, then dock;
    - at least a threshold before a round;
    - report the charge;
    - wait for charging (a scheduled event);
    - a mission `until` the charge is low;
    - a choice of the peer with enough charge;
    - a reading near the threshold that fails closed;
    - a decoy (another robot's charge);
  - **infeasible:** a robot whose charge is unknown, when the request gives no fallback; a charge that never reaches the
    wait's threshold before the timeout;
  - **inexpressible:** "run your self-test and report", "how long until you are empty", "if you will not make it back";
  - **controls.**
- **Bounds:** fixed with the baseline (rule 6). They include a B8-like measure, **a charge check replaced silently**: the
  check dropped, `robot_ready` as a stand-in, or a guessed `wait for`.
- **Economy:** 0 statements, 1 kind, 1 live form, 1 robot field plus a world event, about 3 codes. No reserved words.
