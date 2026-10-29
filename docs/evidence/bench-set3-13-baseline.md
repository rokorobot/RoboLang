# Set 3 baseline against RoboLang 1.3 — 2026-10-29

Set 3 was pre-registered in commit `01fd0772d82e21d7d0b650c90330f477fe5e4771` before any RoboLang 1.4 implementation. This baseline used its 36 unchanged tasks: 20 branching requests that 1.3 cannot express, eight still-inexpressible controls, and eight feasible controls. Each of Opus, Sonnet, and Haiku 5.5 answered every task three times with each of the full-reference and grammar-only prompts (648 answers). Effort was medium. No judge model was used.

The 113 input/reference/scorer files in `bench/tasks_branch/`, `bench/reference_branch/`, `bench/worlds/`, the two prompt wrappers, `bench/lib.py`, `bench/score.py`, `docs/language-reference.md`, and `spec/grammar-v1.3.ebnf` have a canonical path-to-SHA-256 manifest digest of `4ef9eb159ba7263c2dbd1eb8dfb8b274c4d5f9611009b2bc9a70164677a79038` (sorted compact JSON plus newline). Only `bench/run.py` was changed to add a usage-based spending guard; no case, label, prompt, reference, or scorer was changed during or after the run.

## First-decision results

The table counts answers, not distinct cases. A false program is a submitted program for a request whose expected answer is `cannot`. Every such program below was valid and reached `SUCCEEDED` in the supplied world, so the ordinary validator and simulator did not catch the omitted conditional or unsupported requirement.

| Prompt | Model | Correct branching declines | False programs on branching | False programs on still-inexpressible controls | Correct feasible controls | Cases correct on all 3 repeats |
|---|---|---:|---:|---:|---:|---:|
| Full | Opus 5.5 | 58/60 | 2/60 | 0/24 | 21/24 | 34/36 |
| Full | Sonnet 5.5 | 51/60 | 9/60 | 3/24 | 24/24 | 31/36 |
| Full | Haiku 5.5 | 46/60 | 14/60 | 0/24 | 21/24 | 29/36 |
| Grammar only | Opus 5.5 | 48/60 | 12/60 | 0/24 | 22/24 | 28/36 |
| Grammar only | Sonnet 5.5 | 20/60 | 40/60 | 3/24 | 24/24 | 21/36 |
| Grammar only | Haiku 5.5 | 46/60 | 14/60 | 1/24 | 23/24 | 27/36 |

The repeated `x_nearest_robot` control accounts for Sonnet's three false programs under each prompt. A sampled saved program picks a robot and moves the box to the destination; it cannot select the nearest robot at runtime. A sampled Opus response to `b_amr2_or_amr1` puts `not robot_ready primary` in a precondition, then uses one fixed robot; it cannot execute the alternate branch. A sampled Haiku response to `b_whichever_ready` preconditions one bound robot as ready, leaving the requested choice between robots unimplemented. These are concrete examples of the measured half-program failure.

The full prompt does not eliminate this error, and the grammar-only prompt materially worsens it for Sonnet and Opus. This is a 1.3 authoring baseline, not a 1.4 acceptance result. All scores retain the pre-registered expected answers.

## Provenance and limits

- Both runs completed all 324 requested answers: 648 usable first answers, no API errors, and no fallback-model answers.
- Recorded API cost: $1.280541 full + $1.841320 grammar only = **$3.121861**, below the $10 cap. A local usage-based guard reserved $2 per concurrent worker before sending more answers; three workers were used.
- Assembled prompt SHA-256: full `b7b19bc11b801bc2805feebc5628763772289a85f4dd9520c43829941096c178`; grammar only `4e297a05f5becdccf4d607b022553ed49503dfd4f8634064b5c9c86e34ae3f4e`.
- Canonical sorted path-to-SHA-256 manifest digest for the 648 saved raw answer records: `053395fa082b5af84b928295a100060146f6eb3bfcc3c2d36224a2ff59979f09`. Raw request/reply files are ignored by repository policy and remain local to the run environment. The [per-answer scored CSV](bench-set3-13-baseline-scores.csv) and the unmodified scorer reports for [full](bench-set3-13-baseline-full.md) and [grammar only](bench-set3-13-baseline-minimal.md) are committed evidence.
- These results measure model behavior and program execution in the supplied symbolic worlds. They do not establish physical robot safety or cross-implementation interoperability. Labels and worlds have not received an independent external audit.

The next gate under decision 0027 is RoboLang 1.4 implementation against the approved bounded-branching specification, followed by re-running this same set with its 1.4 expectations. PR #68 remains open for owner review and merge authorization.
