# Frozen BENCH-2 validation — 2026-10-29

Freeze SHA-256: `6c4d680a24ad0523029883c0f10357334676bb15ddb4073e0c4185e5903685d4`. Verified unchanged before and after the run.
Owner-authorized run: 80 ask cases × three models × one repeat, full prompt, medium effort, one worker,
$10 local cap. All 240 requested answers were saved. This is a validation run, not the full two-prompt,
three-repeat reliability study; no pass³ or minimal-prompt robustness is claimed.

The two operator-limit replacements restore 18 clarification cases. Their candidate readings and
scorer regression checks pass. Original requests and pilot scores remain preserved as historical evidence.

## Results

| Model | Complete correct | Programs fully satisfy request | Infeasible correct | Inexpressible correct | Clarify correct | Cost |
|---|---:|---:|---:|---:|---:|---:|
| claude-opus-5-5 | 77/80 | 21/23 | 19/19 | 19/20 | 18/18 | $0.6376 |
| claude-sonnet-5-5 | 77/80 | 21/23 | 19/19 | 19/20 | 18/18 | $0.3138 |
| claude-haiku-5-5 | 76/80 | 22/23 | 19/19 | 19/20 | 16/18 | $0.0289 |

Complete correct requires the expected first decision, category and entity checks where specified,
and, for programs, the final expected goal, successful simulation and all procedural checks together.
Fallback answers, if any, remain separate; no case or repair is excluded.

| Model | Programs for decline tasks | Valid, successful substitutions | Entity guesses | Unjustified value defaults | Decoy refusals |
|---|---:|---:|---:|---:|---:|
| claude-opus-5-5 | 0/39 | 0/39 | 0/18 | 0/18 | 2/11 |
| claude-sonnet-5-5 | 0/39 | 0/39 | 0/18 | 0/18 | 2/11 |
| claude-haiku-5-5 | 0/39 | 0/39 | 1/18 | 1/18 | 1/11 |

`silent_guess` is the legacy entity-choice counter, regardless of disclosure in unscored prose.
`unjustified_default` reports missing required values separately. Speed limits are symbolic declarations;
this benchmark does not verify physical control. The model-robustness targets in 0025 remain unchanged.

## Target interpretation

All models meet the zero-substitution target and the observed program/class-correctness floors. Neither Opus nor
Sonnet meets the decoy over-abstention target: 2/11 = 18.2%, above 10%. Haiku meets that target (1/11 = 9.1%) but
misses the entity-guess target: 1/18 = 5.6%, above 5%. Thus **no model meets every measured 0025 target**. The
three-repeat `pass³` target is unmeasured. Correctly running this validation does not certify general model robustness.

The shared misses are interpretation boundaries anticipated by the independent review: choosing a relative speed,
distinguishing a fixed-world wait from general conditional success, and distinguishing mass measurement from generic
inspection with an unavailable scale. Their frozen labels are retained; no case is removed or changed after seeing
model responses. All three models correctly request the missing operator values in both replacement cases.

## Every miss

| Model | Case | Observed decision | Failed requirement |
|---|---|---|---|
| claude-haiku-5-5 | `a06_which_crane` | program / none | expected clarify |
| claude-haiku-5-5 | `a14_reduced_speed` | clarify / none | expected program |
| claude-haiku-5-5 | `a16_a_moment` | program / none | expected clarify |
| claude-haiku-5-5 | `i12_scale_down` | cannot / infeasible | category must be inexpressible |
| claude-opus-5-5 | `a14_reduced_speed` | clarify / none | expected program |
| claude-opus-5-5 | `d7_wait_or_nothing` | cannot / inexpressible | expected program |
| claude-opus-5-5 | `i12_scale_down` | cannot / infeasible | category must be inexpressible |
| claude-sonnet-5-5 | `a14_reduced_speed` | clarify / none | expected program |
| claude-sonnet-5-5 | `d7_wait_or_nothing` | cannot / inexpressible | expected program |
| claude-sonnet-5-5 | `i12_scale_down` | cannot / infeasible | category must be inexpressible |

## Provenance and checks

- Run ID: `ask-frozen-validation-20261029-funded`; assembled prompt SHA-256 `744874a8696f1f90aaea986938033c2a475dac7b63b56cc1a566e80a1581c45b`.
- Total local cost from recorded usage and frozen price table: $0.9802.
- API/format/refusal outcomes: {'answer': 240}.
- Fallback answers: 0; repaired answers: 0.
- Clean-checkout offline suite: 2,539 passed, six expected failures, no skips; Ruff clean.
- All four CI test jobs passed on Ubuntu/Windows, Python 3.11/3.13.
- Raw record-manifest SHA-256: `0c5d242700c1b1cf76be5f8221a3fb79716375fb1e46740c1f6004390a03ff60` (canonical sorted JSON of path-to-SHA-256 map, plus newline).
- Per-answer scored summary: [bench2-frozen-validation.csv](bench2-frozen-validation.csv).
- Raw requests/replies remain locally in `bench/results/ask-frozen-validation-20261029-funded/`; raw results are ignored by repository policy.
- Unmodified scorer output is [bench2-frozen-validation-scorer.md](bench2-frozen-validation-scorer.md).

The earlier unfunded attempt and initial CI request returned HTTP 400 for insufficient credit, before any
model answer. Those account failures are not counted as model scores; the funded run uses a fresh ID.
The funded rerun of the existing sets 1 and 2 CI benchmark passed its unchanged gate:
[Actions run 40911271542](https://github.com/rokorobot/RoboLang/actions/runs/40911271542).
Its 240 additional answers cost $1.1269 from recorded usage ($0.5638 for set 1, $0.5631 for set 2).
A final evidence-only commit is also checked by normal PR CI; its status and costs remain in the PR checks/artifacts.
Validation environment: Python 3.14.7, Anthropic SDK 2.3.0. Frozen implementation commit: `403d2ff`.
PR benchmark CI status is recorded separately in the PR checks. No thresholds or labels were changed after this run.
