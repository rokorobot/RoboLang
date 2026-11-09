# Benchmark set 5: measured values (pre-registration draft)

**Status:** DRAFT, 2026-11-09. Labels need the owner before any model sees a task (rule 1, [0024](../../spec/decisions/0024-authoring-benchmark.md)).
Decision record: [0036](../../spec/decisions/0036-lang-4-measured-values.md). Directory when approved: `bench/tasks_values/`,
`bench/reference_values/`. Nothing here is built or run.

## How the set is used

- **Baseline (now, current reference):** every task labelled `program (1.6)` below is expected `cannot / inexpressible`.
  Infeasible tasks and plain controls keep the label shown. A false program here is a real finding: a model inventing a
  reading RoboLang does not have.
- **Acceptance (after 1.6 is taught):** the 1.6 labels apply. Programs are scored per path, as in set 3, on probe worlds
  that set the measured values. The new check `value_ok` asks: the reading is of the object and kind asked for, with a
  sensor on the robot asked for; the comparison has the asked direction and threshold (any unit of the kind); a
  requested message carries the binding, not a literal.
- **Worlds:** the baseline uses the existing world 1.1 files. Acceptance uses world 1.2 copies of the same worlds that add
  `measures` and `properties` (0036 V9). Task-specific state goes in `world_patch` (0035 E4).

World facts used: kitchen `probe_temp` on `chef_arm` (available), `scale_1` on `helper_arm` (**unavailable**, never
recovers), `cam_runner` on `runner`; pharmacy `scan_p1` and `thermo_p1` on `picker1` (thermometer online at 5 s), `picker2`
not ready; cleanroom `thermo_1` on `arm_x` (available), decoy `thermo1` on `arm_X` (online at 8 s).

## Tasks (26)

### Expressible in 1.6 (14, plus `v15` open)

| id | world | Request | 1.6 label and paths | Tests |
|---|---|---|---|---|
| `v01_probe_report` | kitchen | Take pan 1's temperature with the chef arm's probe, send the reading to the "pass" channel, then move pan 1 to the sink. | program; goal pan 1 at sink; message carries the binding | measure, value message |
| `v02_cool_enough` | kitchen | With the chef arm, probe pan 1. If it is below 60 °C, move it to the pass; otherwise leave it on the stove and tell "pass" it is too hot. | program; probes 45 °C → pan at pass; 80 °C → pan at stove + message; **59.8 ± 0.5 → run fails (`S261`)** | if on a binding, uncertainty fails closed |
| `v03_wait_to_cool` | kitchen | Wait until pan 1 has cooled below 60 °C (check with the chef arm's probe, give it at most 10 minutes), then have the chef arm move it to the pass. If it does not cool in time, stop and say so. | program; events cool pan 1 to 55 °C at 4 min → pass; never cools → `fail` | live reading in `wait until`, handler ends in `fail` |
| `v04_weigh_then_move` | kitchen + patch `scale_1` available | Weigh tray B with the helper arm's scale, then have the helper arm put it on the pass. | program; goal tray B at pass; check: a `mass` reading of `tray_b` recorded | measure without using the value |
| `v05_heavy_grams` | kitchen + patch | Weigh tray B on the helper arm's scale. If it is heavier than 1500 g, tell "pass" it is heavy and stop; otherwise the helper arm puts it on the pass. | program; probes 1.2 kg / 2 kg; scale reports kg | unit conversion within a kind, `fail` branch |
| `v06_batch_gate` | pharmacy | Read insulin 1's batch with picker1's scanner. Only if it is "LOT-7731", have picker1 move it to packing; otherwise stop with the reason "wrong batch". | program; probes LOT-7731 / LOT-0001 | label `==`, `fail` |
| `v07_cold_chain` | pharmacy | Once picker1's thermometer is online (allow 30 seconds), measure insulin 1's temperature. If it is below 8 °C, have picker1 move it to packing; otherwise send the temperature to "qa" and leave it in fridge 2. | program; probes 5 °C / 12 °C | `sensor_ok` wait, measure, if/else, message |
| `v08_warmer_of_two` | cleanroom | With arm_x and thermo_1, measure vial_1 and flask_1. Tell "lab" "vial" if vial_1 is warmer, otherwise "flask". | program; probes both orders; equal-within-uncertainty → fails closed | binding vs binding |
| `v09_online_thermometer` | cleanroom | Use the arm whose thermometer is online right now to take vial_1's temperature, and send it to "lab". | program; arm_x + thermo_1 (decoy: arm_X, thermo1) | decoy names with a reading |
| `v10_no_reading_go_on` | kitchen | Probe pan 2 with the chef arm. If no reading can be taken, tell "pass" "no reading". Either way, then move bowl 1 to prep 1 with the chef arm. | program; probe worlds: probe available / unavailable | handler on `measure` without `fail` |
| `v11_report_after_branch` | kitchen | Probe pan 1 with the chef arm. If it is below 60 °C, move it to the pass. Either way, send the reading to "pass" at the end. | program; probes 45 / 80 °C | binding used after an `if` |
| `v12_fahrenheit` | kitchen | Only move pan 1 to the pass with the chef arm if the probe shows it under 140 °F; otherwise stop. | program; threshold 60 °C (author converts; °F is not a unit) | units in words |
| `v13_relay_batch` | pharmacy | Have picker1 read insulin 1's batch with its scanner, and have cart1 send that batch to "dispatch". | program | binding used by another robot's step |
| `v14_two_limits` | kitchen | Probe pan 1. Move it to the pass only if it is between 50 °C and 70 °C; otherwise leave it and stop. | program; probes 45 / 60 / 75 °C | two comparisons with `and` |
| `v15_reading_in_handler_scope` | kitchen | Probe pan 1 with the chef arm; if that fails, try once more. Then send the reading to "pass". | **inexpressible (recommended) or drop**: after a retry in a handler, no one binding holds the reading on both paths (0036 V3) | binding scope across a retry |

### Inexpressible in 1.6 (controls, 6)

| id | world | Request | Why it stays out |
|---|---|---|---|
| `x01_average` | kitchen | Probe pan 1 and pan 2 and send the average temperature to "pass". | arithmetic |
| `x02_formatted` | kitchen | Send exactly the message "Pan 1 is at <reading> °C, ready" to "pass". | no string formatting (V8) |
| `x03_any_hot_pan` | kitchen | Check every pan on the stove (there may be several by then) and move any above 80 °C to the sink. | choosing objects at run time (`detect`) and iteration |
| `x04_every_minute` | kitchen | Check pan 1's temperature every minute and report each reading to "pass" until it is below 40 °C. | schedule and loop |
| `x05_cooling_time` | kitchen | Report how many seconds pan 1 took to cool below 60 °C. | clock reading |
| `x06_set_oven` | kitchen | Set the oven to 180 °C. | no command writes a value |

### Infeasible (3)

| id | world | Request | Why |
|---|---|---|---|
| `i01_scale_offline` | kitchen | Weigh tray B with the helper arm's scale and send the weight to "pass". | `scale_1` unavailable, never recovers |
| `i02_camera_temp` | kitchen | Use the runner's camera to measure pan 1's temperature. | `cam_runner` measures no temperature (world 1.2) |
| `i03_probe_elsewhere` | kitchen | Have the helper arm take pan 1's temperature with the probe. | `probe_temp` is on `chef_arm` (`S231`) |

### Plain controls: no reading needed (2)

| id | world | Request | Label |
|---|---|---|---|
| `c01_hot_pan_words` | kitchen | Move the hot pan on the stove, pan 1, to the sink with the chef arm. | program, no `measure` |
| `c02_inspect_not_measure` | kitchen | Look at pan 1 with the chef arm's camera, then move it to the pass. | program, `inspect`, no `measure` |

## Open questions for the owner

1. `v15`: score it as an inexpressible control, or drop it. Recommended: keep it as a control; it is the one request
   where a model is most likely to write a program that uses a value that may never have been read.
