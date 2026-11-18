# RoboLang 2.2: more measured kinds

**Status:** front end, Task IR 1.10 (Mission IR 1.2 for a file with a mission), world file 1.7, the simulation (run record
1.10, mission record 1.2) and the IR reader complete in code. The language reference and the benchmark prompts teach 2.2 (L4
of decision 0044); the acceptance run on benchmark set 11 is the next step (L5), and until it passes the bounds B1 to B9 2.2 is
not accepted.
Decision: [0044](decisions/0044-more-measured-kinds.md) (K1 to K9, and clarifications D1 to D14 approved 2026-11-18).
Grammar: [grammar-v2.2.ebnf](grammar-v2.2.ebnf). IR: [Task IR 1.10](ir-schema-v1.10.md) and [Mission IR 1.2](mission-ir-v1.0.md).
Simulation: [section 20](simulation-v1.0.md). Everything not stated here is exactly as in [RoboLang 2.1](robolang-v2.1.md), and
the machinery of measured values is that of [RoboLang 1.6](robolang-v1.6.md): a reading binds a name, carries an uncertainty,
fails closed when unknown, and units of one kind convert exactly.

## 1. Version

`robolang 2.2;` selects 2.2. A file with this marker is read with the 2.1 rules, and may also read four more kinds
(section 2) and write the unit `%`. A 2.1 front end rejects the marker with `E213`. Files with no marker or with `1.1` to `2.1`
are read, validated, compiled and run exactly as before, byte for byte; **in them `fill`, `confidence`, `pressure` and `count`
are still no kind of `measure` (`E217`), and `%` is still no token (`E101`).**

**No new statement, no new reserved word, no new top-level declaration.** Four kinds, one unit token (`%`) and three units of
pressure. The kind names are recognised by position (after `measure r`, and before `of` in a live reading) and are **not
reserved** (D10): a program that uses `count` or `fill` as a name still parses. World file 1.7 adds one field, the `properties`
of a location (D1).

## 2. The four kinds

```rob
robolang 2.2;
task shelf_check(robot r, location shelf, location aisle, sensor cam, sensor tag) {
    goal { robot_at r aisle; }
    plan {
        move r to aisle;
        measure r fill of shelf using cam as level;
        measure r count of shelf using tag as n;
        if level < 25 % { communicate r to "lead" with "restock shelf_a"; }
        if n == 0 { communicate r to "lead" with n; }
    }
}
```

| Kind | What it is | Unit | Operators | Uncertainty |
|---|---|---|---|---|
| `fill` | how full something is (a shelf, a tank, a bin) | `%`, 0 to 100 | `<` `<=` `>` `>=` | the sensor's, as in 1.6 |
| `confidence` | how sure the sensor is of the **label it reports** for that object at that reading (K4) | `%`, 0 to 100 | `<` `<=` `>` `>=` | the sensor's |
| `pressure` | what a gauge shows | `Pa`, `kPa`, `bar` (1 bar = 100 kPa, exactly) | `<` `<=` `>` `>=` | the sensor's |
| `count` | how many items a sensor sees on or in the object | **none** | `<` `<=` `>` `>=` and **`==` `!=`** | **0**: a count is exact |

- **`fill` and `count` may be read of an object or a location** (D1): "bay 3 is empty", "aisle 5 is out of stock". `measure r fill
  of shelf_a using cam as f;` is valid when `shelf_a` is a `location` or an `object` parameter. `confidence` and `pressure`
  are read of an **object**. A robot is not an object or a location: `measure r fill of r ...` is `E304`, the existing type
  error (D9). The same holds for a live reading in a `wait until`.
- **Units** (K2). `%` is a unit of two kinds, `fill` and `confidence`; the kind of the **name** decides what a `%` is compared with.
  `pressure` converts exactly between its three units, in exact decimal arithmetic: `4.2 bar` is `420 kPa` is `420000 Pa`, and
  `p < 450 kPa` and `p < 4.5 bar` are the same comparison. There is no `psi`: the author converts, as with °F. A bare fraction (`f < 0.3`)
  and the word `percent` (`f < 30 percent`) are not accepted: **the unit is required** (`E206`).
- **A count takes no unit.** Its literal is a whole number: `n == 0`, `n < 3`. A unit on a count (`n < 3 kg`, `n < 3 %`) is `E319`; a
  literal that is not whole (`n < 2.5`) is **`E328`**. A count is compared with `<`, `<=`, `>`, `>=`, `==` and `!=`, and is never
  unknown on account of its uncertainty (there is none); it is unknown only when the reading cannot be taken (below).
- **Comparing two kinds** is `E319`, as in 1.6: `fill` with `confidence` (both `%`), `count` with `fill`, and so on. Only a binding
  of the same kind or a literal of its units can stand on the right.
- **`confidence` and the label** (K4, D4). `measure r confidence of plant using cam as c` is the sensor's confidence in the
  label it reports for that plant at that reading. It is allowed without a `label` reading. To test "it is a weed and the detector
  is at least 90 % sure", read **both**, `label` and `confidence`, with the **same** sensor, and test both. Pairing a label
  from one sensor with a confidence from another is a wrong program, not a static error.
- **Live readings** (D6): `wait until fill of tank_2 using cam > 80 % timeout 10 min else { ... }` and the other kinds work in a
  `wait until` as the four kinds of 1.6 do.
- **Unknown fails closed** (K3, D3). A reading near a threshold (`80.5 % ± 2` against `> 80 %`) is unknown, and an unknown `if`
  fails the run with `S261`, as in 1.6. A reading that cannot be taken (the sensor does not measure the kind, `S234`; the object or
  location has no value, `S236`) fails its step.

### 2.1 What 2.2 cannot say (K8)

- **A kind the world names freely**: "how worn the packaging looks", "does it smell burnt", "is the surface clean". The four kinds
  are the whole list, and no sensor can declare another. Decline it as inexpressible (a quality judgement, G-QUALITY, D14).
- **A reading of a person** (G-PROX, D8): "how far the nearest person is". A person is not an object of the world, so there is
  nothing to bind.
- **A reading of the robot itself**, such as its battery (G-SELF, iteration I8): `measure r fill of r` is `E304` (D9). A task reads
  an object or a location with a sensor, not the robot that carries it.
- **Counting objects chosen at run time and acting on each of them** (G-CHOOSE, D7): "carry as many cartons as counted". A plan has
  a fixed number of steps. 2.2 compares a count; it does not repeat or choose by it.
- **Arithmetic on readings** (G-CALC): an average, a sum, a difference of two readings. Reading both is possible; computing is
  not.
- **`psi`** and a bare fraction or `percent` as a unit: convert, write `%` or `bar`, or ask.

## 3. Static rules (`validate`)

1. The kind after `measure r` (and before `of` in a live reading) is one of `temperature`, `mass`, `distance`, `label`, `fill`,
   `confidence`, `pressure` and `count`; any other word is `E217`, and in a file below 2.2 the four new ones are `E217`.
2. The object of a `fill` or `count` reading is an `object` or a `location` parameter; of any other reading an `object`; a
   robot, region or sensor is `E304`.
3. `%` is a token in a 2.2 file, after a number. In a file below 2.2 it is `E101`.
4. A number in a comparison needs its unit, except for a count: a bare number against a kind with units is **`E206`**.
5. A unit of another kind, or any unit on a count, is `E319`; two kinds compared are `E319`; a count literal that is not whole is
   **`E328`**; `==` and `!=` on anything but a label or a count are `E320`.

## 4. Task IR and Mission IR

A 2.2 file compiles to **Task IR 1.10** ([ir-schema-v1.10.md](ir-schema-v1.10.md)) with `language_version` `2.2`: four more values
of a reading's `reading`, and a `quantity` operand without `unit` for a count. A file with a mission compiles to **Mission IR 1.2**,
which holds Task IR 1.10 tasks ([mission-ir-v1.0.md](mission-ir-v1.0.md)). The reader re-validates both in full
([ir-reader-v0.1.md](ir-reader-v0.1.md)).

## 5. Elsewhere (K7)

- **Core is unchanged** and a goal holds no value: a binding never appears in a goal (E321), so `export-goal` is unaffected.
- **`delegate` and the ROS 2 / Gazebo backend refuse a task with a `measure`** as before (`measure_unsupported`; `S509`), whatever
  its kind.
- **The world** (simulation section 20) declares which sensor measures which kind, with a unit and an uncertainty (none for a
  count), and what a reading would be: the `properties` of an object, and, for `fill` and `count`, of a location. The world shown
  to a model never shows a property value.

## 6. Diagnostics

| Code | Stage | Meaning |
|---|---|---|
| `E101` | lexical | (existing) `%` in a file below 2.2 |
| `E206` | syntax / static | (existing) a quantity with no unit, or an unknown unit (`percent`, `psi`): in 2.2, a bare number compared with a kind that has units |
| `E217` | syntax | (existing) an unknown measured kind; in a file below 2.2 also `fill`, `confidence`, `pressure` and `count` |
| `E304` | static | (existing) the object of a reading is a robot (or, for a kind that takes an object only, a location) |
| `E319` | static | (existing) two kinds compared (`fill` with `confidence`), a unit of another kind, a unit on a count |
| `E328` | static | A count compared with a number that is not whole (`n < 2.5`) |
| `E512`, `E513`, `E514` | world file | (existing) an unknown kind, a unit of another kind, a count or label with a `unit` or `uncertainty` |
| `S234`, `S236` | run | (existing) the sensor does not measure the kind; the object or location has no value for it |
| `S509` | ROS 2 backend | (existing) `measure` is refused, whatever the kind |

`E213` lists `1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1 and 2.2`. In the IR reader the new form is `I229` (`E328`); a
quantity with no unit where a count is not compared is `I117` (`E206`).

## 7. Compatibility

Nothing changes for 1.1 to 2.1 files, their IR, their run records or their world files. A world file 1.2 to 1.6 serves a 2.2 task as
before; it names none of the new kinds, so a reading of one fails `S234`. World file 1.7 is 1.6 plus the four kinds in `measures` and
`properties`, and the `location_properties` field, and nothing else.
