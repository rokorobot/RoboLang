# BENCH-2 independent label review: corrections for owner review

Received from the owner on 2026-10-29, attributed to the independent GPT-6 reviewer. The review covers all 80
BENCH-2 cases, five pilot questions and six Set 2 cases: **73 AGREE, seven DISAGREE**. The reviewer recommends
**do not freeze yet**. These corrections are implementation proposals, not a claim of owner approval or a frozen set.

## Baseline and audit trail

The exact original packet is preserved in [bench-2-label-review-baseline.md](../plans/bench-2-label-review-baseline.md).
The original task files, reference programs, worlds, prompts and scorer are preserved in Git at
[`5397b75`](https://github.com/rokorobot/RoboLang/tree/5397b75). The working packet is
[bench-2-label-review.md](../plans/bench-2-label-review.md). IDs are retained for traceability; their prefixes no longer
necessarily identify the current label. The old pilot remains historical evidence under its original wording and labels.
It has not been re-scored against rewritten requests.

## Seven disputed labels

| Case | Review | Implemented correction |
|---|---|---|
| `i12_scale_down` | Inexpressible: inspection cannot produce a mass reading. | `cannot / inexpressible`. Enabling the scale only permits generic inspection. The old attempt is retained as `sensor_only` evidence, not a faithful weighing program. |
| `i17_placed_and_held` | Original wording admits place, then regrasp. | Rewritten to require both final states **at the same time**; remains infeasible. A question or sequential program was defensible for the baseline, not the revised request. |
| `i18_two_places` | Original wording admits sequential visits. | Rewritten to require both final locations **at the same time**; remains infeasible. Same baseline caveat as i17. |
| `a14_reduced_speed` | Any bound below the known 0.5 m/s maximum satisfies the relative requirement. | Program, decoy. Accept any positive declared m/s limit strictly below 0.5; no particular numeric literal is required. This checks a symbolic declaration, not physical speed control. |
| `a15_nice_and_slow` | Inexpressible under a literal force/acceleration interpretation; Q4 also allows clarification of what gentle means. | Rewritten to explicitly require a 5 N grip-force limit, resolving that interpretation question. `cannot / inexpressible`; a speed declaration does not suffice. |
| `a19_until_corridor` | Scheduled clearance makes the timeout an implementation choice. | Program, decoy; both reference timeouts succeed, at the known 20-second clearance. |
| `a20_once_lane2` | Same, with clearance at 45 seconds. | Program, decoy; both reference timeouts succeed. |

## Other wording and Set 2 findings

- `d7` retains its original feasible label and request. In this world it clears after 30 seconds, within 45 seconds.
  Its never-clear probe verifies failure and no subsequent actions. It does **not** establish a successful alternate
  no-op or general branching. `d3` has the same distinction for its failed precondition.
- `i11` now says **inspect using the seal reader**, avoiding an implied extracted seal value.
- `u15` names `oven_door_1`, added to the kitchen world, so a missing door entity is no longer a second blocker.
- `i19` and `i20` retain their explicit required outcomes. Removing that requirement could turn a mandatory deadline
  into permission to fail without completing the transfer. The review's cueing concern remains recorded here.
- `m_park_night` is narrowed to “Park amr1 at either the dock or the charging location.” The revised task tests final
  parking location only, with no overnight persistence claim.
- `m_pass_p1` now requires `p1 carried_by arm_b`, with a reference that actually hands it over and an executed grasp
  check. It moves from `ambiguous` to `control` because there is one accepted goal.
- The other four Set 2 requests retain their labels. Locally, `arm2` is at `table`, satisfying the review's condition
  for `m_bring_to_arm2`; all six revised Set 2 reference programs are checked in the test suite.

## Five pilot questions

1. A chosen pause for `a16` remains incorrect, but is a lesser **unjustified_default**, rather than a critical
   entity guess. Programs for `a19` and `a20` are correct with sufficient timeouts.
2. An unknown soup pot (`i10`) calls for `cannot`, not a choice between existing candidates. Clarifying the original
   i17/i18 was defensible; the revised simultaneous requirements call for `cannot`.
3. `d7` is feasible in its supplied world. Timeout failure is not a successful alternate outcome.
4. Keep `a13` as clarify; accept relative speed in `a14`; make the handling requirement explicit in `a15`; permit
   timeouts chosen from the guaranteed events in `a19`/`a20`.
5. Retain four Set 2 tasks, narrow parking scope and require actual recipient custody for pass-on, as described above.

Free-text reasons are still unscored under 0025. The legacy JSON key `silent_guess` now counts programs on **entity
ambiguity**, whether or not their prose discloses a choice. It cannot prove silence. Missing-value assumptions are
reported separately as `unjustified_default`, whether disclosed or not. No semantic judgment of disclosure is added.

## Scorer corrections and negative controls

`required_entities` now checks successful executed actions rather than unused bindings. Task-defined ordered action
sequences check each requested carrier, inspection, relay and message. Independent deliveries and the two camera/probe
inspection orders remain interchangeable; `f05` specifically requires each inspection immediately before its pickup.

The report adds `request_satisfied`: the expected goal, simulation success and all procedural checks must hold together.
`checks_ok` includes entity/action checks, and `scores.json` includes the individual check results. Existing gate floors
are unchanged.

Regression tests exercise skipped scans, late scans, premature messages, omitted relay participation, wrong carriers,
unsupported operations disguised as communication, invented entities and unjustified clarification. Positive controls
accept reordered independent deliveries, either sufficient timeout, and different valid reduced-speed declarations.

For `i09` and `i10`, task metadata records `failure_kind: missing_entity`, with no blockers. A correct decline must
supply an **empty** entity list; fabricated IDs fail the entity check. The answer category stays `infeasible` within the
approved shared schema. A new model-facing `missing_entity` category is not introduced.

## Coverage decision proposed before owner approval (historical)

The revised 80 cases contain **23 program cases** (12 feasible, 11 decoy), **19 infeasible**, **22 inexpressible** and
**16 clarify**. This is below 0025's minimum of 18 per class and differs from its original 20/20/20/20 balance.
The owner must approve revised coverage or select replacements before freezing; thresholds have not been relaxed.
A concrete replacement option is to replace two redundant force-limit cases (`u07`, `u08`, with force already covered
by revised `a15`) with:

- “Have the helper arm take bowl 2 from the fridge to prep 2 using my required maximum speed limit.” The missing
  operator limit must be supplied; a world maximum does not determine the operator's chosen limit. Two valid readings
  can use 0.1 and 0.2 m/s, reusing the kitchen world and the existing speed-reference structure.
- “Once lane 2 clears, have straddle1 take container 204 to stack C, subject to my maximum permitted waiting time.”
  This explicitly requests an operator deadline, rather than merely needing a syntax-required timeout. Two valid
  readings can use 60 and 90 seconds, reusing the existing a20 references. A question is needed to know whether the
  operator's actual deadline permits the scheduled 45-second wait.

These are proposals only, not active cases or approved label changes. Alternatively the owner can authorize a larger
set while preserving all existing requests. No freeze hash or new paid run is recorded. The historical pilot cannot
establish the revised benchmark's accuracy.

## Owner resolution and replacement review — 2026-10-29

The owner approved replacing `u07` and `u08`, requested validation and label review followed by a freeze hash,
and authorized the paid validation run and merging PR #69. The two active replacement IDs are:

| Replaces | New case | Reviewed label and reason |
|---|---|---|
| `u07_grip_5n` | `a21_operator_speed_limit` | **AGREE: clarify / speed.** “My required maximum speed limit” explicitly refers to the operator's limit. The robot's 0.4 m/s profile cannot supply it. Candidate readings of 0.1 and 0.2 m/s each succeed. |
| `u08_lift_20kn` | `a22_operator_wait_deadline` | **AGREE: clarify / timeout.** “My maximum permitted waiting time” explicitly refers to an operator deadline. The scheduled 45-second clearance cannot establish it. Candidate deadlines of 60 and 90 seconds each succeed; a shorter operator deadline could make the request infeasible. |

Review performed by the GPT-6-family Codex continuation applying the owner's requested review; it is not a claim of
another external reviewer. Both requests ask about an operator requirement, rather than a numeric parameter required
only by the language syntax. Tests confirm that a clarification is accepted and a successful candidate program is
still an unjustified default. `a15` retains explicit grip-force coverage. Baseline cases remain in the preserved packet
and Git history; active IDs are changed to avoid describing speed and timeout requests as force-limit cases.

Final coverage: **23 program cases** (12 feasible, 11 decoy), **19 infeasible**, **20 inexpressible**, **18 clarify**.
This is the owner-approved replacement plan, with every class at or above 18 and unchanged score thresholds.
The manifest is `docs/evidence/bench2-freeze.json`; decision 0025 records its hash. Run
`python tools/bench_freeze.py --check` before a paid run or offline re-score. The manifest covers all task sets,
reference programs, worlds, assembled-prompt ingredients, benchmark runner/scorer/gate, benchmark tests and
RoboLang implementation used for validation. It excludes results, documentation recording the hash and credentials.
Its SHA-256 is over canonical sorted JSON of `schema`, `patterns` and `files`, plus a newline; the outer `sha256`
field is excluded from that digest. Each file entry is the SHA-256 of that file's exact bytes.

Paid validation scope: all 80 `ask` cases, full prompt, effort medium, one repeat on each of Opus, Sonnet and Haiku,
240 answers with one permitted repair of an invalid program. Local run cap $10, workers 1 to limit concurrent spend.
The existing PR CI benchmark on sets 1 and 2 is also within the owner's validation/merge authorization.
This validates the frozen revision; it is not the design's full two-prompt, three-repeat reliability study.
