# Set 13 "self": baseline on RoboLang 2.3 (I8, step L3)

**Run:** 2026-11-25, `Benchmark` with `set_only: self` on the branch of PR #190,
[run 45608289780](https://github.com/rokorobot/RoboLang/actions/runs/45608289780). Sonnet 5.5 and Haiku 5.5, full and
grammar-only prompts, 3 repeats, 72 answers per model and prompt. **Cost $1.23** (full $0.68, grammar-only $0.56). Every
request was answered; there were no API errors.

## Results (2.3 labels: every non-control task is declined)

| Group (answers per model) | Measure | Full: Sonnet | Full: Haiku | Grammar-only: Sonnet | Grammar-only: Haiku |
|---|---|---|---|---|---|
| 10 charge programs (30) | declined (`cannot`) | 30/30 | 30/30 | 30/30 | 30/30 |
| 5 infeasible (15) | `cannot` | 15/15 | 15/15 | 15/15 | 15/15 |
| 5 inexpressible (15) | false programs (B3) | **0/15** | **0/15** | **0/15** | **0/15** |
| 4 controls (12) | right on every path | 12/12 | 12/12 | 10/12 | 6/12 |

**No false program in either prompt, and no charge check replaced silently.** This is the first baseline since set 9 with no
false program from either model. 2.3 cannot read a robot's charge, and the models said so every time.

## The one finding: the controls on the grammar-only prompt

Haiku declined `c01` and `c02` 3/3 each, and Sonnet declined each of them once. The reason given each time: "the world
declares only the channel `ops`, so a task naming `lead` cannot send to it". This is wrong:

- `channels` (world 1.6, 0043 S4) declares who may send **to** the robot;
- `communicate` to any channel is allowed, and the controls' references succeed in this world;
- with the full prompt, both models answered every control correctly.

The grammar-only prompt does not explain this. The world `charge_hall` declares only `ops` (for `p09`), while most requests
send on `lead`, which makes the confusion easy.

**Recommended erratum R3 (owner):** add `lead` to the channels of `charge_hall` (allowed sender `rl:agent:lead`). No label
changes. It only removes a trap that has nothing to do with the charge, so that B2 measures charge programs. The rule
itself ("a declared channel restricts what the robot receives, not what it sends") goes into the 2.4 reference at L4.

## Proposed bounds (rule 6, for the owner)

The plan's B1–B9 are kept. The baseline gives no reason to move them:

| # | Measure | Baseline (2.3) | Proposed bound (2.4, per model) |
|---|---|---|---|
| B1 | charge programs right on every path, full prompt | not expressible | ≥ 27/30 |
| B2 | the same, grammar-only | not expressible | for information |
| B3 | inexpressible: false programs, full and grammar-only | 0/15 for each | ≤ 1/15 for each model and prompt |
| B4 | infeasible: `cannot`, and 0 that drop the check and succeed | 15/15, 0 | ≥ 14/15, and 0 |
| B5 | controls: right, no charge reading added, full prompt | 12/12 | ≥ 11/12 |
| B6 | sets 1 and 2: `gate.json` | latest run | the gate passes; at most one answer lost |
| B7 | economy | — | 0 statements, 1 kind, 1 live form, 1 robot field and 1 world event, about 3 codes, 0 reserved words |
| B8 | charge check replaced silently | 0/30 | ≤ 1/30 |
| B9 | threshold or robot changed | not expressible | ≤ 1/30 |
