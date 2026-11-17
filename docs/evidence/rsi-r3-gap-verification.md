# RSI-01 R3: verifying the near language gaps with benchmark tasks

Date: 2026-11-17. Set `future` (`python bench/run.py --set future`): 25 probes (`inexpressible`) and 8 controls.
Yardstick: [language-reference.md](../language-reference.md), RoboLang 1.6. No model was run and no money was spent;
the model run is the `Benchmark` workflow with `future: true`. Rule: [LE-7](../LANGUAGE_EVOLUTION_RULES.md).

## Method

- Scope: every open gap that a near activity (`pilot` or `deployment`, horizon now or 1-3y) claims in `needs`: nine
  gaps. G-LOOP is out of scope (12 tasks already) and G-FORCE, G-PROX, G-PREDICT, G-TEACH are claimed by no near activity.
- A **probe** is one operator request, from or close to a near activity's `requests`, that needs exactly one gap, in a
  world where everything else is feasible. Its obvious attempt (`bench/reference_future/<id>.attempt.rob`) must be
  rejected by `check_answer`; the first code is recorded as `attempt_fails_with`. Four new worlds (`fulfilment`,
  `site`, `ward`, `fry_station`) and `warehouse` serve them.
- Before a request was kept as a probe, I tried to express it with `if`, `measure`, failure handlers and
  `wait until`. A program that runs but drops part of the request does not count as an expression (it is the silent
  half-program). Three claims did not survive: see "Refutations".
- A **control** is a request from a near activity that 1.6 can express: a reference program that succeeds on every
  path the task lists. Controls measure false refusals in the model run.

## Per gap

All 25 attempts are rejected at validation, so no attempt has a `.attempt.json`. "Confirmed" means every probe of the
gap fails as recorded and no way to express it was found.

| Gap | Probe (activity) | Attempt fails with | Verdict |
|---|---|---|---|
| G-CLOCK | `f_clock_scan_stamp` (RET-001): report the time the scan finished | E317 (`time_now` is no binding) | confirmed |
| | `f_clock_shift_cutoff` (LOG-001): move the tote only before 22:00 | E101 (`22:00`) | confirmed |
| | `f_clock_clean_at_2300` (CLN-001): start at 11 pm | E101 (`23:00`) | confirmed |
| G-CALC | `f_calc_total_weight` (LOG-001): send the sum of two weights | E101 (`+`) | confirmed |
| | `f_calc_over_limit` (LOG-002): send how many kg over a limit | E101 (`-`) | confirmed |
| | `f_calc_text_line` (LOG-002): one message with the weight inside the text | E101 (`+`) | confirmed |
| G-MSG | `f_msg_line_docked` (MFG-002): place when the line controller sends "TUGGER DOCKED" | E211 (`message_received`) | confirmed |
| | `f_msg_nurse_confirm` (HLTH-001): wait for the nurse's confirmation | E204 (invented condition) | confirmed |
| | `f_msg_wind_clear` (SOL-001): hold until "WIND CLEAR" arrives | E204 (invented condition) | confirmed |
| G-ACT | `f_act_set_fryer` (HOS-001): set the fryer to 175 degC | E203 (unknown statement) | confirmed |
| | `f_act_open_closet` (HLTH-001): open the supply closet | E203 | confirmed |
| | `f_act_print_layout` (CON-001): print the wall layout | E203 | confirmed |
| G-PATH | `f_path_avoid_wet_floor` (CLN-001): never drive through the wet floor | E201 (`in_region` as a condition) | confirmed |
| | `f_path_stay_in_zone` (CON-001): stay inside the layout zone all the way | E201 (`in_region`) | confirmed |
| | `f_path_keep_off_road` (AGR-002): never enter the road zone | E202 (`no_go_region` on a `move`) | confirmed |
| G-CONC | `f_conc_carry_and_drive` (MFG-002): digit1 carries while tugger_a drives | E203 (`parallel`) | confirmed |
| | `f_conc_two_hand_lift` (MFG-002): two robots lift one tote at the same moment | E203 (`together`) | confirmed |
| G-CHOOSE | `f_choose_nearest_tugger` (MFG-002): the nearest of two robots | E304 (`measure` of a robot) | confirmed |
| | `f_choose_nearest_dock` (AUTO-001): the nearest of three docks | E304 (`measure` of a location) | confirmed |
| | `f_choose_nearer_cart` (LOG-001): the nearer of two carts | E304 | confirmed |
| G-PERCEIVE | `f_perceive_shelf_fill` (RET-001): shelf less than a quarter full | E217 (kind `fill`) | confirmed, narrowed |
| | `f_perceive_weed_confidence` (AGR-002): detector under 90 percent sure | E217 (kind `confidence`) | confirmed, narrowed |
| G-SELF | `f_self_low_battery` (AUTO-001): battery below 25 percent | E217 (kind `battery`) | confirmed |
| | `f_self_enough_for_route` (AUTO-001): at least 60 percent charge | E206 (unit `percent`) | confirmed |
| | `f_self_report_charge` (AUTO-001): report the charge left | E317 (`battery_level`) | confirmed |

Each gap has 2 or 3 probes from different activities where the atlas has them; G-CONC and G-SELF are each claimed by
one near activity, so their probes share it. The probe `why` fields cite the reference.

### What is confirmed, and how narrowly

- **G-CALC** is confirmed for arithmetic on readings and for text built from a value. A *fixed* count is not a gap:
  "tell the lead how many totes you moved" over two named totes is the literal "2 totes moved" (control
  `c_count_two_totes`). The activities' "how many" over an open set needs a loop and a choice first, so CALC is a
  derived need there, not shown by a task of its own.
- **G-CHOOSE** is confirmed only for choosing by a property that cannot be read: `measure` takes objects, and a robot
  or a location has no distance. Choosing among entities the world names, by their state or reading, is expressible
  (`if`, `measure`, `object_at`); see the refutation below.
- **G-PERCEIVE** is confirmed for perceived values outside the four kinds (a fill level, a confidence): no kind and no
  unit fits, and labels are exact. A perceived label or distance is expressible when the world has a sensor that
  measures it.
- **G-SELF** is confirmed for the battery. The robot's *health* is partly expressible: `sensor_ok` and `robot_ready`
  test sensors and readiness (control `c_camera_fault_stop`). The atlas still lists G-SELF for AUTO-001 because its
  requests are about charge.
- **G-PATH**: a plain `move` that happens not to name the zone satisfies "never enter it" only by construction in the
  symbolic world; it states no constraint, so it is not an expression. The probes are the same as the old
  `u09`/`u10`.
- **G-MSG** and **G-ACT**: no world fact stands for a received message or a device setting, so no workaround exists.
  The probe worlds model the line controller, the nurse and the site controller as channels only.

## Refutations (3)

Each removes the gap from that activity's `needs` and appends to its `basis`: "R3 (2026-11-17): <gap> refuted, 1.6
expresses it with ...".

1. **G-PERCEIVE for ACT-HOS-001** ("If the fries come out too dark, discard that basket"). 1.6 expresses it when the
   camera measures `label`. Control `c_fries_too_dark`:

   ```rob
   measure r label of basket using cam as shade;
   if shade == "too_dark" {
       goal { object_at basket bin; }
       grasp r basket; move r to bin; place r basket at bin;
       communicate r to "lead" with "discarded a basket of fries: too dark";
   } else {
       goal { object_at basket hot; }
       grasp r basket; move r to hot; place r basket at hot;
   }
   ```

2. **G-PERCEIVE for ACT-SOL-001** ("If a table is out of tolerance, skip it and report which one"). 1.6 expresses it
   when a sensor measures the offset as a `distance`. Control `c_table_out_of_tolerance`:

   ```rob
   measure r distance of t14 using laser as off;
   if off <= 5 mm {
       goal { object_at m14 row14; }
       move r to pallet; grasp r m14; move r to row14; place r m14 at row14;
   } else {
       communicate r to "foreman" with "table_14 skipped: out of tolerance";
   }
   move r to pallet; grasp r m15; move r to row15; place r m15 at row15;   // task goal: m15 on row15
   ```

3. **G-CHOOSE for ACT-MFG-001** ("Sort whatever parts are left in container 2 into the trolley"). 1.6 expresses it
   for parts the world names: one `if object_at part bin { goal {...} move; grasp; move; place; }` per part, then the
   message. Control `c_parts_left_in_container`. It does not cover parts the world does not name; that would need
   detection, and R3 cannot test it.

These refutations hold for a world that declares the sensor; the label vocabulary ("too_dark") comes from the
operator's request, because the world never shows what an object reads. The decision between "the camera is missing"
(infeasible) and "the language cannot" is the world's. Two old tasks are now stale for this reason: `u01_if_dirty`
(kitchen, camera measures nothing) and `u02_if_damaged` (yard) are labelled `inexpressible` for 1.3; under 1.6 they are
`infeasible` (`measure` fails with S234). They stay in G-PERCEIVE's list; relabelling them is not part of R3.

## Controls (8)

| Control (activity) | What it shows 1.6 can do |
|---|---|
| `c_retry_flag_carry_on` (LOG-001) | three tries with nested handlers, flag to the lead, carry on with the next tote (`if holding`) |
| `c_count_two_totes` (LOG-001) | a fixed count is a literal message |
| `c_wait_lane_clear` (MFG-002) | wait for the worker lane with a timeout; stop if it does not clear |
| `c_whichever_tugger_ready` (MFG-002) | "whichever is ready" is an `if robot_ready` |
| `c_parts_left_in_container` (MFG-001) | refutation 3 |
| `c_table_out_of_tolerance` (SOL-001) | refutation 2 |
| `c_fries_too_dark` (HOS-001) | refutation 1 |
| `c_camera_fault_stop` (AUTO-001) | own-sensor health with `sensor_ok`; report and stop |

Every control has `expected_goal` and, when it branches, `expect_1_6.paths` with the world patch that forces each path;
`tests/test_bench_future.py` runs the reference on every path.

## What the model run must still show (LE-7, second half)

LE-7 asks that models decline each request today. The attempts show what the *language* rejects; only a run shows what
a *model* does. Run `Benchmark` with `future: true` (Sonnet 5.5 and Haiku 5.5, full prompt, 1 repeat, at most $3), then:

1. **Decline rate on the probes**: `decision_correct` on the 25 probes, and `false_program` (a program for a probe).
   A gap enters L1 only if models decline its probes. A model that answers a probe with a valid program that runs
   (for instance two sequential moves for "at the same moment") is the silent half-program: the gap is real and the
   model is wrong, so it counts for the gap and against the model.
2. **Category**: probes expect `inexpressible`; a decline as `infeasible` or `clarify` is a finding about the world or
   the prompt, not the language. Watch G-CHOOSE (geometry may read as missing information) and G-PATH.
3. **False refusals on the 8 controls**, above all the three refutations: if models decline those, the reference
   under-sells `measure label` and `if`, and the atlas's refutation stands for the language but not for the models.
4. **The probes that depend on the world**: G-PERCEIVE and G-SELF (a model may take `robot_ready` for battery).

Not tested by R3 (no probe from that activity): G-CHOOSE for ACT-RET-001, ACT-EV-001 and ACT-AGR-002; G-PERCEIVE for
ACT-LOG-002 and ACT-CON-001; G-MSG for ACT-MFG-001 and ACT-EV-001; G-ACT for ACT-CLN-001, ACT-EV-001 and ACT-AUTO-001;
G-CLOCK for ACT-MFG-001 and ACT-HLTH-001; G-PATH for ACT-SOL-001. Those claims stay as they were.

## Observations (not bugs)

- `in_region`, `no_go_region` and the other reserved atoms report E201 in a condition (`not in_region ...`, `wait until
  in_region ...`), while `message_received` reports E211. The E211 text says "not part of v1.1" in a 1.6 file. The
  atlas's `scope_note` for G-PATH says E211. Harmless to R3: both are rejections.
- A world file cannot give a robot or a location a property, so no world can offer "distance of a tugger".
