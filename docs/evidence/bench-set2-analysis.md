# Benchmark set 2: what the failures say (2026-10-29)

Runs `hard-full` (the language reference as system prompt) and `hard-minimal` (grammar only), 40 tasks
× 3 models each, effort medium. Total cost about $1.30. Raw reports:
[full](bench-set2-full-report.md), [minimal](bench-set2-minimal-report.md). Every failed answer was read.

## Headline

| | Opus 5.5 full | Sonnet 5.5 full | Haiku 5.5 full | Opus minimal | Sonnet minimal | Haiku minimal |
|---|---|---|---|---|---|---|
| Right decision (program or decline) | 40/40 | 39/40 | 39/40 | 38/40 | 39/40 | 38/40 |
| False program (the dangerous error) | 0/18 | 0/18 | 1/18 | 2/18 | 1/18 | 1/18 |
| False refusal | 0/22 | 1/22 | 0/22 | 0/22 | 0/22 | 1/22 |
| Goal correct (feasible tasks) | 20/22 | 21/22 | 22/22 | 19/22 | 20/22 | 20/22 |

## Findings, by cause

**F1. A conjunction in a goal breaks the bridge (defect).** Four minimal-prompt answers wrote
`goal { object_at a x and object_at b y; }` instead of two lines. The meaning is identical, but
`bridge.goal_triples` accepts only bare outcome atoms, so the goal had no Core form and scored as
wrong (`k_swap_boxes` ×2, `m_same_shelf`, `d_both_blue_boxes`).

**F2. Models use `or` for ambiguous requests.** For "put one of the vials on the scanner" and "put
both boxes on the same shelf", Opus wrote a disjunctive goal (`object_at v1 dst or object_at v2 dst`).
That is arguably more faithful than any single reading, but Core 0.1 has no disjunctive desired
state (decision Q3), so it cannot be exported. The benchmark also compares goals by triple
equality, not by meaning.

**F3. Without branching, a model may silently drop half a request (dangerous).** Haiku answered
"to shelf_a if the cell is clear, else to the table" with a valid program covering only the first
branch (`pre { region_clear cell; }`). It validates and succeeds when the cell is clear, and nothing
in the program says the other branch was dropped.

**F4. Rules outside the grammar are learned from the reference, not the grammar.** Without the
reference, models declared a speed limit above the robot's maximum (`S001`) or in a unit the robot
does not declare (`S002`). These are caught only when the task is simulated. With the reference,
no model made these mistakes. False programs roughly double without the reference (2–4 vs 0–1).

**F5. Plan errors are caught only by simulation.** Sonnet's round-trip plan placed a box it was not
holding; `validate` passed and the run failed. The system worked as designed, but the error came late.

**F6. Description-driven over-reading.** Haiku refused a feasible request, inventing a reach limit
from an entity description ("ceiling arm over aisle 1"); Sonnet refused "park amr1 for the night"
although a dock and a charging place exist. Both are minor and model-side.

**F7. Benchmark issues.** `n_speed_over_max` is debatable: a declared 2 m/s cap on a 1 m/s robot is
harmless physically, and RoboLang rejects it only because a task may not claim more than the
robot's profile authorizes. Set 1 is saturated, and set 2 with the full reference nearly is: Opus
made no wrong decision.
