# RoboLang 2.4: the robot's own state

**Status:** front end, Task IR 1.12 (Mission IR 1.4 for a file with a mission), world file 1.9, the simulation (run record 1.12, mission
record 1.4), `delegate`, the backends' refusals and the IR reader complete in code. The language reference and the benchmark prompts teach
2.4 (L4 of decision 0048); the acceptance run on benchmark set 13 is the next step (L5), and until it passes the bounds B1 to B9 2.4 is
not accepted.
Decision: [0048](decisions/0048-the-robots-own-state.md) (S1 to S11, and the clarifications approved 2026-11-25).
Grammar: [grammar-v2.4.ebnf](grammar-v2.4.ebnf). IR: [Task IR 1.12](ir-schema-v1.12.md) and [Mission IR 1.4](mission-ir-v1.0.md).
Simulation: [section 22](simulation-v1.0.md). Everything not stated here is exactly as in [RoboLang 2.3](robolang-v2.3.md).

## 1. Version

`robolang 2.4;` selects 2.4. A file with this marker is read with the 2.3 rules and may also read the robot's own charge (section 2).
A 2.3 front end rejects the marker with `E213`. Files with no marker or with `1.1` to `2.3` are read, validated, compiled and run
exactly as before, byte for byte; **in them `charge` is still an unknown measured kind (`E217`) and `robot_charge r` an unknown
condition (`E204`).**

**No new statement, no new reserved word, no new top-level declaration:** one kind, `charge`; one live form, `robot_charge r`; one
robot field and one world event in the world file. `charge` and `robot_charge` are recognised by their position and are **not
reserved** (S3): `charge` is already the name of a world action, and a program of any version that uses either word as a name
still parses.

## 2. The kind `charge` (S1, S2)

```rob
robolang 2.4;

task round_or_dock(robot r, location dock, location stop_c) {
    plan {
        measure r charge as c;
        if c < 25 % {
            goal { robot_at r dock; }
            move r to dock;
            communicate r to "lead" with "spot1 is docked";
        } else {
            goal { robot_at r stop_c; }
            move r to stop_c;
        }
    }
}
```

- **`measure r charge as c;`** binds the charge of robot `r` in `%` (0 to 100). It has **no `of ... using ...`**: the charge comes from
  the robot, not from a sensor parameter (S1). `r` is any robot parameter of the task: another robot's charge may be read as well as
  the moving robot's, because the fleet manager reports it (point 4). The statement may have a failure handler like any `measure`.
- **Comparisons** are `<`, `<=`, `>`, `>=` against a literal in `%` (`25 %`) or against another binding of kind `charge`. **There is no
  `==`** (`E320`): a charge is never exactly a value. A literal with another unit is `E319`, and so is a charge compared with another
  kind (a `fill`, a `confidence`, a temperature); a number with no unit is `E206`.
- **Unknown fails closed.** A comparison with an unknown charge, or whose uncertainty interval holds the threshold, is unknown, and
  an unknown `if` takes no branch (`S261`, as in 1.6 V7). The intervals are closed: `26 +- 1` is not below `25`; `24 +- 1` may be `25`.
- **A charge is sent with its unit** (S5, point 12): `communicate r to "lead" with c;` sends `64 %`.
- **Scope** is that of any binding (V3): visible after the statement in its block and in nested blocks (`E317` elsewhere); a binding
  has its own name, never a parameter's or a chosen name (`E318`).
- **A charge cannot be predicted.** The charge changes only by the scheduled events of the world (section 5): there is no drain, no
  energy for a route and no time until empty (S10, G-PREDICT).

## 3. The live form `robot_charge` (S3, S6)

```rob
wait until robot_charge r >= 80 % timeout 40 min else { fail "the battery is not at 80 percent in time"; }
```

`robot_charge r` is read each time the `wait until` that holds it is evaluated. It is allowed **in the condition of a `wait until` and in
a mission's `until`, and nowhere else** (`E322`, as a live reading in 1.6 and 1.9 is): in an `if`, read the charge with `measure` and
compare the binding, so that the record says which instant decided (0042 T6). It compares with a literal in `%` or a binding of kind
`charge`, with the operators of section 2, and may be one operand of an `and` or an `or` with other conditions
(`robot_charge r >= 90 % and clock_time < 23:00`). `r` is a robot parameter (`E303`, `E304`). `robot_charge` is recognised by its position
(a name followed by a name) and is not reserved: `robot_charge < 5 %` with `robot_charge` a binding is still a comparison of a binding.

## 4. Missions (S6, point 6)

```rob
mission patrol(robot r, location a, location b) {
    repeat at most 6 runs {
        run round(r, a, b);
    } until robot_charge r < 30 %;
}
```

A mission's `until` may be `robot_charge r OP T`. It is read **after each `run` line**, as the other `until` conditions are (0041
clarification 2): a mission with two `run` lines is checked twice per pass and may end after the first. **An unknown value ends the
mission `failed`** (`S402`, `until_unknown`), and the runs before it stay verified. **It is not a bound by itself**: M2 still needs a
count, a duration or a time of day (`E325`). A self-state **trigger** ("the moment it drops below 20 percent", `when robot_charge r <
20 %`) is G-TRIGGER, a later iteration: a mission starts a run only when the last is done.

## 5. The world: world file 1.9 and the simulation (S4)

A robot may have a **`charge`**: `{ "value": "64", "unit": "%", "uncertainty": "1" }`; `uncertainty` is optional and **absent means
exact**. A robot with no `charge` has an **unknown charge**: a reading of it fails the step with **`S239`**, which fails closed. A
top-level list **`charge_events`** holds `{ "at": { "value": "20", "unit": "min" }, "robot": "spot2", "charge": { ... } }`: at `at` the
robot's charge becomes `charge`. **Only scheduled events change a charge.** There is no drain and no charging model;
`do r charge charger` (a 1.8 declared action) is a request to the robot's controller and **changes no reading** (point 7), so
"charge until 80 percent" is a `do` followed by `wait until robot_charge r >= 80 %`, which the scheduled event meets. **Only `wait for`
advances time** in a world with no positions (point 2): a `move`, a `do` and a `grasp` take none.

**Before the run** (section 22.4 of the simulation spec, point 3): a step in the plan's own sequence that cannot work in this world and
has **no fallback** (no failure handler, or one that always ends in `fail`) fails before `AUTHORIZED`, and `validate --world` says so:
**`S012`** for a charge that is never known (no `charge` and no event), **`S013`** for a `wait until robot_charge r OP T` that no charge
the world gives the robot by the end of the wait can meet. A wait whose event comes too late is not `S013`: it times out (`S241`).

## 6. Choosing by charge (S7, point 5)

`choose robot t from (a, b) where robot_charge t >= 50 % else { fail "..."; }` is allowed: it is the **one reading a `where` may state**.
The charge of a candidate is a fact that the fleet manager reports, a world fact, unlike a sensor reading of an object. This narrows
`E330` of 2.3 **for this form only**: `where` still never holds a `measure` binding, the clock or a message, and a charge in it must be
the chosen robot's (`robot_charge t`, with `t` the chosen name) and compared with a literal in `%`. **A candidate with an unknown charge
(no fact, or an interval that holds the threshold) is excluded**; if none is left the `else` runs.

## 7. What 2.4 cannot say (S10)

Decline each as inexpressible:

- **health beyond `robot_ready` and `sensor_ok`**: a self-test and its result, faults, requests for service (**G-HEALTH**, a far gap);
- **time until empty, and energy for a route** ("how many minutes can I still run", "if I will not make it back"): a prediction
  (**G-PREDICT**); the simulation has no drain model;
- **arithmetic on a charge** (**G-CALC**);
- **a self-state trigger** ("the moment it drops below 20 percent, even in the middle of a round") (**G-TRIGGER**).

Do not replace a charge check: a request "if your battery is low" is never `robot_ready`, a dropped check or a guessed `wait for`; a
threshold, operator and robot the request does not state are never invented.

## 8. Static rules (`validate`)

1. In a file below 2.4, `measure r charge` is `E217` and `robot_charge r` is `E204`.
2. `measure r charge as c;` needs `as` straight after the kind (`E201`); the robot is a robot parameter (`E303`, `E304`); the binding
   is new (`E318`).
3. A charge is compared with `<`, `<=`, `>`, `>=` (`E320` otherwise), against a `%` literal (`E319` for another unit, `E206` for a
   number with no unit) or a binding of kind `charge` (`E319` for another kind).
4. `robot_charge r` is live: only in a `wait until` and a mission's `until` (`E322`).
5. A `where` may state `robot_charge t OP T` of the chosen robot `t` against a literal in `%`; any other reading, and a charge of
   another robot, stays `E330`.
6. A mission with only an `until` on the charge has no bound (`E325`).

## 9. Task IR and Mission IR

A 2.4 file compiles to **Task IR 1.12** ([ir-schema-v1.12.md](ir-schema-v1.12.md)) with `language_version` `2.4`: one step kind,
`read_charge`, and one comparison operand, `charge`. A file with a mission compiles to **Mission IR 1.4**, which holds Task IR 1.12
tasks ([mission-ir-v1.0.md](mission-ir-v1.0.md)). The reader re-validates both in full ([ir-reader-v0.1.md](ir-reader-v0.1.md)).

## 10. Backends and delegation (S8)

`export-goal` is unaffected: a goal never holds a binding (0036). **`delegate` refuses a task that reads the charge**
(`measure_unsupported`, as for any `measure`), and the **ROS 2 / Gazebo backend refuses it with `S509`** before any motion: no battery topic
is defined in its profile yet. The G1 backend refuses the step and the live form with `S508` (a step kind it does not know), before any command. Mapping the charge to `sensor_msgs/BatteryState` belongs with the
backend's own step.

## 11. Safety (S9)

**Hazard H28** ([safety case](../docs/safety-case-draft.md)): a wrong or stale charge reading lets a robot go on with a low battery.
Controls: an unknown charge fails closed (`S239`, `S261`, `S402`); the uncertainty makes a comparison near the threshold unknown; and
**the robot's own low-battery protection always takes precedence**: RoboLang asks, it never overrides the controller (LE-13, as
with region limits in 1.7). A RoboLang check is not the protection: a task must never be the only thing between a robot and a flat
battery. In physical mode the charge comes from the fleet manager, not from a file. **Simulation only:** the charge is the world file's.

## 12. Diagnostics

| Code | Stage | Meaning |
|---|---|---|
| `E201` | syntax | (existing) `measure r charge` is not followed by `as` |
| `E204` | syntax | (existing) `robot_charge` in a file below 2.4 |
| `E206` | syntax | (existing) a charge compared with a number that has no unit |
| `E217` | syntax | (existing) `charge` in a file below 2.4 |
| `E303`, `E304` | static | (existing) the robot of a charge is not declared, or not a robot |
| `E317`, `E318` | static | (existing) a charge binding that is not visible, or that has a taken name |
| `E319` | static | (existing) a charge compared with another unit or another kind |
| `E320` | static | (existing) `==` or `!=` on a charge |
| `E322` | static | (existing) `robot_charge` outside a `wait until` and a mission's `until` |
| `E325` | static | (existing) a mission with no bound but a charge `until` |
| `E330` | static | (existing, narrowed) a `where` reads anything but the chosen robot's charge against a literal in `%` |
| `S012` | authorization | A charge that is never known, with no fallback (simulation section 22.4) |
| `S013` | authorization | A `wait until robot_charge` that no charge the world gives can meet, with no fallback |
| `S239` | run | A charge reading of a robot with no charge in the world: unknown, fails closed |
| `S402` | mission | (existing) an `until` on the charge is unknown |
| `S509` | ROS 2 backend | (existing) the charge is refused before any motion |
| `S508` | G1 backend | (existing) the charge step and the live form are refused as a step kind and a condition it does not know |

In the IR reader the forms are `I114` and `I115` (a charge operand or step in an older schema), `I125`, `I203`, `I204`, `I220`, `I221` and
`I231` (the narrowed `E330`). `E213` lists `1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3 and 2.4`.

## 13. Compatibility

Nothing changes for 1.1 to 2.3 files, their IR, their run records or their world files. World files 1.0 to 1.8 serve a 2.4 task as
before; a robot in them has no charge, so a reading of it is unknown (`S239`, or `S012` before the run). World file 1.9 is 1.8 plus the
`charge` of a robot and `charge_events`, and nothing else.
