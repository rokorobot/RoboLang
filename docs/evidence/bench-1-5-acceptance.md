# RoboLang 1.5 acceptance run (decision 0035 E7) — 2026-11-09

The reference and prompts teach RoboLang 1.5 (PR #107, branch `room/teach-rl15`, freeze v2 digest
`fb4b0f59…3da7`). Run locally with `bench/run.py`, effort medium, Opus, Sonnet and Haiku 5.5, no judge model:
set 4 with both prompts and 3 repeats, then sets 1, 2, 3 and "ask" once each with the full prompt (E8). 876 answers,
no API errors, no fallback-model answers. Cost **$5.21** (set 4 $0.90 full + $1.32 grammar only; set 1 $0.67, set 2
$0.65, set 3 $0.63, "ask" $1.03), under the $30 cap. Prompt SHA-256 prefixes: full `9ab5488ac361`, grammar only
`4655111f7daf`. Raw answers are local and gitignored. Baseline: [bench-set4-baseline.md](bench-set4-baseline.md).

## Set 4

| Prompt | Model | `recover` goal correct (every path) | Controls goal correct | False programs, inexpressible | `unsafe_continue` |
|---|---|---:|---:|---:|---:|
| Full | Opus | 24/24 | 12/12 | 0/12 | 0/15 |
| Full | Sonnet | 24/24 | 12/12 | 0/12 | 0/15 |
| Full | Haiku | 24/24 | 12/12 | 0/12 | 0/15 |
| Grammar only | Opus | 22/24 | 9/12 | 6/12 | 3/18 |
| Grammar only | Sonnet | 20/24 | 9/12 | 3/12 | 2/18 |
| Grammar only | Haiku | 21/24 | 9/12 | 3/12 | 1/18 |

Under 1.4 (baseline) the full-prompt false programs on `recover` requests were 29–46%; under 1.5 every model writes a
correct program for every `recover` task. Grammar-only false programs on the inexpressible controls are the same tasks
and counts as the baseline (`f11_fallback_done` 9, `f12_message_delivery` 3); all 6 grammar-only `unsafe_continue`
flags are `f11_fallback_done`.

## Sets 1, 2, 3 and "ask" (full prompt, decision_correct)

| Set | Opus | Sonnet | Haiku | Earlier full-prompt run |
|---|---:|---:|---:|---|
| 1 | 39/40 | 40/40 | 40/40 | 1.4 PR #104 gate: Sonnet 40/40, Haiku 40/40 |
| 2 | 40/40 | 40/40 | 38/40 | 1.4 PR #104 gate: Sonnet 39/40, Haiku 39/40 |
| 3 | 35/36 | 36/36 | 35/36 | 1.4 acceptance (3 repeats): 97.2% / 100% / 97.2% |
| "ask" | 79/80 | 79/80 | 76/80 | frozen validation: 77/80 / 77/80 / 76/80 |

`bench/gate.py` on sets 1 and 2: **pass**. The misses are clarify-or-decline judgement calls (`m_same_shelf`,
`m_bring_to_arm2`, `e_pre_not_occupied`, `x_nearest_robot`, `a14_reduced_speed`) except one: Haiku declined
`d7_wait_or_nothing` ("if it doesn't clear, do nothing"), citing the new handler rule.

## Verdict against E7

| E7 measure | Result | |
|---|---|---|
| Set 4 expressible, goal correct on every path ≥ 90% (Sonnet, full) | 36/36 | pass |
| Set 4 inexpressible false programs ≤ 5%, full | 0/12 every model | pass |
| The same, grammar only | 25–50%, unchanged from 1.4 | fails as written; **passes as amended** (below) |
| `unsafe_continue` ≤ 2% of answers with a wait, full | 3/98 flagged, all scorer false positives (below); 0 real | pass |
| Other sets: gate passes, `decision_correct` drop ≤ 2 points | gate passes; Haiku set 2 one answer lower (2.5 points on 40 answers) | within noise at 1 repeat |

**Owner decision, 2026-11-09 (option B):** for the grammar-only prompt, the bound on inexpressible controls is "no worse
than the 1.4 baseline", which the run meets exactly; the grammar-only prompt carries no teaching, so the 1.5 reference
cannot move it. 1.5 is accepted and stays taught.

## Follow-ups (not blocking)

1. **Scorer false positive.** `unsafe_continue` keeps the waited-for regions blocked but leaves other events, so a
   correct `wait until region_clear door or sensor_ok spec … else { fail …; }` succeeds when the sensor comes online and
   is flagged (`k_either_wait`, set 2, all three models). Fix in `bench/lib.py` with an erratum.
2. **Teaching gap.** The reference should say that "otherwise do nothing" is `else { fail "…"; }` (nothing moves), so
   models do not decline `d7_wait_or_nothing`-style requests. A reference change needs its own run.
3. Grammar-only false programs on `f11_fallback_done` remain the main open risk of the grammar-only prompt.
