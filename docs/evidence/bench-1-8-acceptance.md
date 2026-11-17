# RoboLang 1.8 acceptance (I2, step L5)

**Run:** 2026-11-17, `Benchmark` with `actions: true` on the branch of PR #151,
[run 44181358399](https://github.com/rokorobot/RoboLang/actions/runs/44181358399). Set 7 under 1.8 (world 1.4
copies), Sonnet 5.5 and Haiku 5.5, full and grammar-only prompts, 3 repeats. **Cost $1.04.** Bounds B1–B8 were
approved before the run ([plan](../plans/bench-7-actions.md)); the baseline is in
[bench-set7-baseline.md](bench-set7-baseline.md). The figures below are re-scored offline after erratum E3
(`a08_charge_cart` accepts the reading "the cart is at the charger"; [errata](bench-errata-1-8.md)). No new request
was sent.

## Bounds

| # | Measure | Bound (per model) | Sonnet | Haiku | Verdict |
|---|---|---|---|---|---|
| B1 | action programs: goal and `actions_ok`, full prompt | ≥ 27/30 | 28/30 | 30/30 | **met** |
| B2 | the same, grammar-only (information) | — | 27/30 | 26/30 | — |
| B3 | inexpressible: false programs, full **and** grammar-only | ≤ 1/15 per prompt | 0/15, **6/15** | 0/15, **6/15** | **missed** (grammar-only) |
| B4 | infeasible: `cannot`, no valid program that succeeds | ≥ 14/15 and 0 | 15/15, 0 | 15/15, 0 | **met** (both prompts) |
| B5 | controls: right decision, no `do` added | ≥ 11/12 | 12/12 (grammar-only 9/12) | 12/12 (grammar-only 7/12) | **met** on the full prompt |
| B6 | sets 1 and 2: `gate.json`, decision accuracy | gate passes; ≤ 2 points or one answer lost | set 2 false programs 1/15 (gate max 6%) | 0/15 | **missed by one answer** (PR #151 run, 1 repeat), re-measured on the next push |
| B7 | economy | 2 keywords, 1 statement, 1 condition, 3 world fields, about 5 user-facing codes plus IR forms | 2, 1, 1, 3; 6 user-facing codes (`S006`–`S009`, `S271`, `E517`) plus `X606` | | **met** |
| B8 | action replaced silently, full prompt | ≤ 1/30 | **0/30** | **0/30** (baseline 3/30) | **met** |

## What missed

- **B3, grammar-only: 6/15 false programs for each model.**
  - Sonnet: `x01_hand_to_ana` ×3 and `x02_selftest_result` ×3. Haiku: `x02` ×3, `x03_teach_restock` ×2 and
    `x04_open_until_open` ×1.
  - The worlds of these tasks declare tempting actions (`hand`, `self_test`). With only the grammar, the models
    stretch `do` to cover what 1.8 cannot say. For "hand it to Ana" they write `do r hand kit` and drop the
    recipient. For "tell the result" they report the declared result label as if it were the test's outcome.
  - The full prompt, with the language reference, has 0/15 for both models. The grammar alone does not say what a
    `do` cannot do: one target, no recipient, and a result that is a declared state, not a value.
- **B6: one answer.** Sonnet's first answer to `n_speed_unit` in set 2 was a program with a 0.5 rad/s limit for a
  robot that declares only m/s. `validate --world` caught it (S002), and the repair round turned it into the correct
  decline. It is a tool-caught first answer, not a silent one. Decision accuracy (39/40) is within B6.

## Headline

Before 1.8, models silently dropped the requested action in 3/30 answers, and every such program passed every
check. Under 1.8 that count is **0/30** for both models and both prompts. The full prompt meets every bound.

## Verdict

**Pending the owner's decision.** As pre-registered, 1.8 is not accepted: B3 (grammar-only) and B6 (one answer) were
missed.
