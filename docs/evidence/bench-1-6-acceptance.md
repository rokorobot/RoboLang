# RoboLang 1.6 acceptance run (decision 0036 V12, V13) — 2026-11-10

The benchmark teaches RoboLang 1.6 (PR #113, merged at `d6847b2`). Run locally with `bench/run.py`, effort medium, Opus,
Sonnet and Haiku 5.5, no judge model, no API errors, no fallback-model answers. Raw answers are local and gitignored.
Baseline: [bench-set5-baseline.md](bench-set5-baseline.md) (1.5). Errata: [bench-errata-1-6.md](bench-errata-1-6.md).

Two runs:

1. **Acceptance** at `17ba35d` (run `acc16`): set 5 with both prompts and 3 repeats; sets 1, 2, 3, 4 and "ask" once each
   with the full prompt. 1,104 answers, **$7.53**. Prompt SHA-256 prefixes: full `526d32db29ed`, grammar only
   `515d3e615dc4`.
2. **After fix C** at `eb44bbc` (run `acc16c`, [erratum G1](bench-errata-1-6.md)): set 5 with both prompts and 3 repeats.
   468 answers, **$3.71**. Prompts: full `6112a7c51740`, grammar only `ae47d6671887`.

Total **$11.24**, under the $15 cap.

## Set 5

"Value ok" means the decision, the goal on every path, the plan and `value_ok` are all right.

| Run | Prompt | Model | Value tasks ok (15 × 3) | Inexpressible false programs | Infeasible false programs | Controls | `silent_substitution` |
|---|---|---|---:|---:|---:|---:|---:|
| acc16 | Full | Opus | 43/45 | 0/18 | 0/9 | 6/6 | 0/42 |
| acc16 | Full | Sonnet | 45/45 | 0/18 | 0/9 | 6/6 | 0/42 |
| acc16 | Full | Haiku | 44/45 | 0/18 | 0/9 | 6/6 | 0/42 |
| acc16 | Grammar only | Opus | 38/45 | 0/18 | 0/9 | 6/6 | 0/42 |
| acc16 | Grammar only | Sonnet | 41/45 | **3/18** | 0/9 | 6/6 | 0/42 |
| acc16 | Grammar only | Haiku | 37/45 | **2/18** | 0/9 | 6/6 | 0/42 |
| acc16c | Full | Opus | 43/45 | 0/18 | 0/9 | 6/6 | 0/42 |
| acc16c | Full | Sonnet | 41/45 | 0/18 | 0/9 | 6/6 | 0/42 |
| acc16c | Full | Haiku | 43/45 | 0/18 | 0/9 | 6/6 | 0/42 |
| acc16c | Grammar only | Opus | 40/45 | 0/18 | 0/9 | 6/6 | 0/42 |
| acc16c | Grammar only | Sonnet | 39/45 | 0/18 | 0/9 | 6/6 | 0/42 |
| acc16c | Grammar only | Haiku | 28/45 | 0/18 | 0/9 | 6/6 | 0/42 |

Under 1.5 (baseline), 20 full-prompt answers silently substituted `inspect` for a reading and every one validated and
succeeded; under 1.6 none does, with either prompt.

**acc16 grammar-only false programs:** all 5 were `x03_any_hot_pan` ("check every pan on the stove (there may be several
by then) and move any above 80 °C"). The answers measured the one pan the world lists. The 1.5 grammar-only baseline
was 0/18, so this was a regression. **Owner decision, 2026-11-10 (option C):** add one general rule to both prompt
wrappers (a request about entities that may appear after the job starts cannot be represented), and re-run set 5
(erratum G1). After the fix: 0/18 for every model with both prompts.

## Other sets (acc16, full prompt, `decision_correct`)

| Set | Opus | Sonnet | Haiku | 1.5 acceptance (full prompt) |
|---|---:|---:|---:|---|
| 1 | 40/40 | 40/40 | 40/40 | 39/40, 40/40, 40/40 |
| 2 | 40/40 | 39/40 | 39/40 | 40/40, 40/40, 38/40 |
| 3 | 35/36 | 36/36 | 34/36 | 35/36, 36/36, 35/36 |
| 4 | 16/16 | 16/16 | 16/16 | 48/48 each (3 repeats) |
| "ask" | 79/80 | 79/80 | 77/80 | 79/80, 79/80, 76/80 |

`unsafe_continue` 0 in every set. `bench/gate.py` on sets 1 and 2: **pass** (and again in PR #113's check after fix C).

## Verdict against V13 (with the owner's amendments)

| V13 measure | Result | |
|---|---|---|
| Set 5 expressible: goal correct on every path and `value_ok` ≥ 90% (Sonnet, full) | 45/45 (acc16); 41/45 = 91% (acc16c) | pass |
| Set 5 inexpressible false programs ≤ 5%, full and grammar only | full 0/18; grammar only 3/18 and 2/18 in acc16, **0/18 after fix C** | pass after fix C |
| `silent_substitution` ≤ 5% (full) | 0/42 every model, both runs and prompts | pass |
| Other sets: gate passes, `decision_correct` drop ≤ 2 points or one answer | largest drop one answer (Sonnet set 2, Haiku set 3) | pass |

1.6 is accepted and stays taught. Merged as PR #113 on 2026-11-10.

## Follow-ups (not blocking)

1. **`v08_warmer_of_two`.** Fixed: [erratum G2](bench-errata-1-6.md#erratum-g2--a-possible-unknown-is-not-a-reason-to-decline-2026-11-10). In acc16c Sonnet declined it in 3 of 3 full-prompt answers (in acc16 it wrote it 3 of 3).
   Its reasons say that comparing two readings "could be unknown", so the outcome "cannot be guaranteed". The reference
   could state that comparing two readings is allowed and that an overlap failing the run is the intended outcome. A
   reference change needs a set 5 re-run (about $1.50).
2. **Haiku, grammar only.** Value tasks ok fell from 37/45 to 28/45 between the two runs, with every decision right
   (78/78): the programs read or compare the wrong thing. Not gated (the grammar-only prompt carries no teaching);
   worth watching in the weekly run.
