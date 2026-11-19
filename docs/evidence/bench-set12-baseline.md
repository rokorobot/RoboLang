# Set 12 "choose": baseline on RoboLang 2.2 (I7, step L3)

**Run:** 2026-11-19, `Benchmark` with `set_only: choose`,
[run 44425477199](https://github.com/rokorobot/RoboLang/actions/runs/44425477199). Sonnet 5.5 and Haiku 5.5, 3 repeats.
**Cost $1.32** (full prompt $0.75, grammar-only $0.57).

**Incomplete.** The API key ran out of credit during the grammar-only step:

- Haiku: 74 of 75 grammar-only requests failed (`400: credit balance is too low`).
- Sonnet: 5 of 75 grammar-only requests failed.

The **full-prompt** half is complete for both models (75 answers each) and is the baseline below. The grammar-only half
is for information only and must be rerun once the key has credit. A second dispatch from the L3 branch
(run 44425853969) failed on its first request and cost nothing.

## Full prompt (2.2 labels: every non-control task is declined)

| Group (answers per model) | Measure | Sonnet | Haiku |
|---|---|---|---|
| 11 choose programs (33) | declined as the 2.2 label expects | 28/33 | 9/33 (18 `clarify`, 6 programs) |
| | of which: a valid, succeeding program (B8-like) | 5 (`p04` 2, `p05` 3) | 6 (`p04` 3, `p05` 3) |
| 5 infeasible (15) | `cannot` | 11/15 | 1/15 (9 `clarify`) |
| | of which: a valid, succeeding program | 4 (`i04` 3, `i05` 1) | 5 (`i03` 2, `i04` 3) |
| 5 inexpressible (15) | false programs (B3) | **0/15** | **5/15** (`x01` 3, `x02` 2) |
| 4 controls (12) | right, no `choose` (B5) | 12/12 | 12/12 |

## What the answers show

1. **`p04`: the criterion is dropped silently.** The request is "whichever of amr1 and amr2 is ready and **nearer to
   shelf_a**". Both models wrote `if robot_ready a1 … else if robot_ready a2 …`, which ignores "nearer". 2.2 has no
   distance, so this is the failure 2.3 must remove (B8/B9).
2. **`x01`, `x02` (Haiku): a run-time set written as a fixed list.** "Go by what is on the row when you start" was
   answered with a ladder over the four bins the world lists. Under point 5 (approved) this is a false program.
3. **`i03`, `i04`, `i05`: label problem.** The facts that make these infeasible are only in the 2.3 notes
   (`world_2_3`):
   - `digit2` is not ready (`i03`);
   - nothing is at `rework` (`i04`);
   - `scanbot2` is not ready (`i05`).

   The 2.2 world files that the models were shown say the opposite: `digit2` and `scanbot2` are ready, and `p5` is at
   `rework`. The models answered correctly for the world they saw ("p5 is the only part at rework"). **These 9 answers
   are not false programs.** At L4, the world 1.8 copies carry the 2.3 facts, so the acceptance run is not affected.
4. **`p05`: the 2.2 if-ladder is a right answer.** "Take whichever part is on the conveyor" was answered by both models
   with `if object_at p1 conv … else if object_at p2 conv …` over the parts the world lists. The simulator's world is
   closed: every object is listed. The ladder therefore does exactly what `choose object p from objects at conv` does,
   on every path. **Consequence for 2.3:** in a closed world, `objects at L` and `where` add brevity, a record of the
   choice, and a path to unlisted objects (G-DETECT), but **no new expressivity**. The new expressivity of 2.3 is the
   order by distance (`nearest`, `farthest`), which 2.2 cannot write at all (`p01`–`p03`, `p06`–`p08`, `p10`).
5. **Haiku asks rather than guesses.** It answered `clarify` on 18 of the 33 choice-program answers. That is a right kind of
   decline, but it is not the `cannot` the label expects.

## Errata (approved by the owner 2026-11-19)

- **Q3** (`i03`, `i04`, `i05`): in the baseline, a program that is right for the 2.2 world shown is not a false
  program. No label changes for 2.3.
- **Q4** (`p05`): accept the if-ladder over the listed parts as a right 2.2 and 2.3 answer. In B8, count only a
  ladder that drops a criterion the request states, such as `p04`'s "nearer". `p09` and `p11` (missions) get the
  same reading.

## Bounds (APPROVED by the owner 2026-11-19, rule 6)

The bounds of the [plan](../plans/bench-12-choose.md) are kept, with the baseline in hand:

| # | Measure | Baseline (2.2, full) | Proposed bound (2.3, per model) |
|---|---|---|---|
| B1 | choose programs right on every path | not expressible | ≥ 30/33 |
| B2 | the same, grammar-only | not run in full | for information |
| B3 | inexpressible: false programs, full and grammar-only | Sonnet 0/15, Haiku 5/15 | ≤ 1/15 for each model and prompt |
| B4 | infeasible: `cannot`, and 0 that fix a candidate and succeed | 11/15, 1/15 (see Q3) | ≥ 14/15, and 0 |
| B5 | controls: right, no `choose` added | 12/12, 12/12 | ≥ 11/12 |
| B6 | sets 1 and 2: `gate.json` | latest run | the gate passes |
| B7 | economy | — | 1 statement, 1 world field (`position`), about 4 codes, no new reserved words (point 10) |
| B8 | choice replaced silently (after Q4: a fixed candidate, or a dropped stated criterion) | 2/33, 3/33 (`p04`) | ≤ 1/33 |
| B9 | criterion changed | not expressible | ≤ 1/33 |

The grammar-only half of the baseline should be rerun once the key has credit (about $0.6). It is not needed to fix
these bounds, because B2 is for information only, and B3's bound of ≤ 1/15 does not depend on a baseline.
