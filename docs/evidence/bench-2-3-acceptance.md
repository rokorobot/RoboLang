# RoboLang 2.3 acceptance (I7, step L5)

**Runs:**

- **Acceptance:** 2026-11-24, `Benchmark` with `set_only: choose` on the branch of PR #172,
  [run 45499976329](https://github.com/rokorobot/RoboLang/actions/runs/45499976329). Set 12 under 2.3, Sonnet 5.5 and
  Haiku 5.5, full and grammar-only prompts, 3 repeats. **Cost $1.99** (full $1.09, grammar-only $0.91).
- **B6:** the per-PR run on PR #172, [run 45500244866](https://github.com/rokorobot/RoboLang/actions/runs/45500244866),
  sets 1 and 2, 1 repeat, $0.39. An earlier per-PR run was cancelled by a docs-only push.

Bounds B1–B9 were approved before the run ([plan](../plans/bench-12-choose.md)). The baseline is in
[bench-set12-baseline.md](bench-set12-baseline.md).

**Status:** **ACCEPTED by the owner, 2026-11-24**, after teaching fix Z1, errata Q9 to Q11 and a confirming run, with one recorded exception: Haiku's B3 on the grammar-only prompt is 2/15 (bound ≤ 1). The first acceptance run is below; the confirming run and the verdict follow.

| # | Measure (full prompt unless stated) | Bound (per model) | Sonnet | Haiku | Verdict |
|---|---|---|---|---|---|
| B1 | choose programs right on every path, `choice_ok` | ≥ 30/33 | **33/33** | **27/33** | Haiku missed |
| B2 | the same, grammar-only (information) | — | 31/33 | 17/33 | — |
| B3 | inexpressible: false programs, full / grammar-only | ≤ 1/15 each | 0/15, 1/15 | 0/15, **3/15** | Haiku grammar-only missed |
| B4 | infeasible: `cannot`; 0 that fix a candidate and succeed | ≥ 14/15 and 0 | 15/15, 0 (both prompts) | 15/15, 0 (both prompts) | met |
| B5 | controls: right, no `choose` added | ≥ 11/12 | 12/12 | **10/12** | Haiku missed (11/12 with Q9) |
| B6 | sets 1 and 2: `gate.json`; `decision_correct` drops by at most one answer | the gate passes | gate passes; set 2 39 → 38/40 (`m_pass_p1` clarify, `x_wait_message`) | 40/40, 39/40 | met |
| B7 | economy | 1 statement, 1 world field, ~4 codes | 1 statement (`choose`), 1 field (`position`), 6 codes plus 3 reader forms; no reserved words | | met with the approved exception |
| B8 | choice replaced silently | ≤ 1/33 | 0/33 | 0/33 | met |
| B9 | criterion changed | ≤ 1/33 | 0/33 | **5/33** | Haiku missed |

## The Haiku misses

1. **An invented `where robot_ready` (`p01` 2/3, `p06` 1/3: B1 and B9).** "Send whichever of tugger_a and tugger_b is
   nearer to trailer_4" was written as `choose robot t from (a, b) where robot_ready t nearest to dst`. Every path
   succeeds, because both tuggers are ready, but the program would send the farther tugger when the nearer one is not
   ready, so the `where` is a criterion change. **This is our teaching.** The only `choose` example in the language
   reference is exactly this line, with `where robot_ready`. It appears at the top of the reference and in "Choosing at
   run time" (`docs/language-reference.md` lines 17 and 307), and it comes from 0045's own example.
2. **A mission bound invented and the `until` dropped (`p11` 2/3: B1 and B9).** "Until 17:00" was written as `repeat at most
   10 runs, for at most 30 min` with no `until clock_time >= 17:00`. This is a real error: 0041 clarification 2 says a
   time of day in `until` is a hard bound by itself.
3. **An invalid program (`p07` 1/3: B1).** A `goal` written after the `choose` (`E208`, `E201`, not repaired).
4. **`c03` (B5, 2 of 3).**
   - **Repeat 3:** a real error. It ran a round trip 12 times, so 24 carries for "12 carries in all".
   - **Repeat 1:** a round trip 6 times, which is 12 carries, ending with p1 at `conveyor_in`. That is what the request
     says. The label requires the two run lines of the reference (`carry` there, `carry` back), so the answer is marked
     wrong. **This is a label problem**, the same kind as erratum P7.
5. **`x02` grammar-only (B3, 3/15; Sonnet 1/15).** "Put each pan above 75 °C in the sink … go by what is on the stove"
   was answered with `choose … from objects at stove` and one reading. In its world only `pan_1` is at the stove, so the
   program is right for every path of that world. The task is meant to test "each of several", but its world has only
   one. **This is a flaw in the task's world**, like Q3. With the full prompt, both models declined it 3/3.

## Recommended (owner)

- **Teaching fix Z1.**
  - The reference example becomes `choose robot t from (a, b) nearest to handoff else { fail … }`, with no `where`.
  - The reference gets one sentence: add a `where` only for a condition the request states, such as "whichever is
    free" or "whichever is ready".
  - The `where robot_ready` example moves to that sentence.
  - For `until`, the reference repeats 0041 clarification 2: a time of day in `until` is the bound, so no run count or
    duration is added.
- **Erratum Q9 (`c03`):** accept any mission whose runs carry p1 12 times in all and end with it at `conveyor_in`.
- **Erratum Q10 (`x02`):** its world copy gets a second pan at the stove, so that one choice is visibly not "each".
- **Confirming run** (as in I4, errata M1/M2): `confirm_tasks` with `p01`, `p06`, `p07`, `p11`, `c03` on the full
  prompt, and `x02` on the grammar-only prompt. That is 3 repeats each, both models, at most $1 per run, about $0.60 in
  all.
  - **Haiku meets B1** if it gets at least 3 more of its 6 misses right.
  - **Haiku meets B9** if it gets at most 1 criterion change.

## Confirming run (2026-11-24, after Z1, Q9 and Q10)

[Run 45502242195](https://github.com/rokorobot/RoboLang/actions/runs/45502242195) (full prompt: `p01`, `p06`, `p07`, `p11`,
`c03`; $0.25) and [run 45502453258](https://github.com/rokorobot/RoboLang/actions/runs/45502453258) (grammar-only: `x02`;
$0.07). The answers of these tasks replace those of the acceptance run, as in I4 (M1/M2). B6 is measured again on PR #172
after Z1 changed the reference ([run 45502256641](https://github.com/rokorobot/RoboLang/actions/runs/45502256641)).

| # | Bound | Sonnet | Haiku | Verdict |
|---|---|---|---|---|
| B1 | ≥ 30/33 | 33/33 | **32/33** (`p01` 3/3, `p06` 2/3, `p07` 3/3, `p11` 3/3) | met |
| B3 | ≤ 1/15, grammar-only | 1/15 (`x02` now 0/3) | **3/15** (`x02` 3/3) | **Haiku missed** |
| B5 | ≥ 11/12 | 12/12 | **11/12** (`c03` 2/3) | met |
| B6 | the gate passes, at most one answer lost | gate passes; set 2 38/40 | gate passes; set 1 40 → 39/40 (`w_notify_fleet` declined) | met |
| B9 | ≤ 1/33 | 0/33 | **1/33** (`p06` repeat 3) | met |

**`x02` with two pans.** Haiku wrote a program 3 times. Each reads the two listed pans and moves each one above 75 °C.
Sonnet declined 3/3. Only **repeat 1** pins the set in `pre` (`pre { object_at p1 stove; object_at p2 stove; }`), so that
the program fails closed when the pans on the stove are not the listed ones. Repeats 2 and 3 have no such `pre`. (The
first version of this note said that all three pinned the set; that was wrong.)

**Erratum Q11 (approved by the owner 2026-11-24):** `x02` accepts a decline, or a program that pins the listed pans in
`pre` and ends right on three paths (both hot, one hot, none hot): `expect_2_3.program_also_ok`, scored in
`bench/score.py` with `lib.pre_pins`. Rescored, Haiku's repeat 1 is right, and repeats 2 and 3 stay false programs.

| # | Bound | Sonnet | Haiku | Verdict |
|---|---|---|---|---|
| B3 grammar-only, with Q11 | ≤ 1/15 | 1/15 | **2/15** | **Haiku missed by one** |

## Verdict

**ACCEPTED by the owner, 2026-11-24, with one recorded exception.**

- Sonnet meets every bound.
- Haiku meets every bound except B3 on the grammar-only prompt, which is 2/15 against a bound of at most 1.
- Both of Haiku's grammar-only false programs are `x02`. That task cannot fully pose "each of an unknown set" in a closed
  world.
- With the full prompt, Haiku declines `x02` 3/3.

G-CHOOSE closes at L6 (erratum Q12); release 0.10.0.

**Paid cost of I7:** baseline $1.32, acceptance $1.99, confirming $0.32, per-PR runs about $0.8.
