# Benchmark history

One row per benchmark run that counts (see [strategy](../STRATEGY.md), section 5). Effort medium,
1 repeat, unless noted. Decision accuracy and false programs apply to set 2 and ask; set 1 has no decline tasks.

| Date | Run | Set | Prompt | Model | Decision accuracy | False programs | Goal correct | Cost | Notes |
|---|---|---|---|---|---|---|---|---|---|
| 2026-10-29 | run3 | 1 | full | Opus 5.5 | — | — | 40/40 | $0.29 | [report](bench-run3-report.md) |
| 2026-10-29 | run3 | 1 | full | Sonnet 5.5 | — | — | 40/40 | $0.15 | |
| 2026-10-29 | run3 | 1 | full | Haiku 5.5 | — | — | 40/40 | $0.02 | Set 1 saturated |
| 2026-10-29 | hard-full | 2 | full | Opus 5.5 | 40/40 | 0/18 | 20/22 | $0.37 | [analysis](bench-set2-analysis.md) |
| 2026-10-29 | hard-full | 2 | full | Sonnet 5.5 | 39/40 | 0/18 | 21/22 | $0.19 | |
| 2026-10-29 | hard-full | 2 | full | Haiku 5.5 | 39/40 | 1/18 | 22/22 | $0.02 | Silent half-program (F3) |
| 2026-10-29 | hard-minimal | 2 | grammar | Opus 5.5 | 38/40 | 2/18 | 19/22 | $0.48 | `and` goals (F1) |
| 2026-10-29 | hard-minimal | 2 | grammar | Sonnet 5.5 | 39/40 | 1/18 | 20/22 | $0.23 | |
| 2026-10-29 | hard-minimal | 2 | grammar | Haiku 5.5 | 38/40 | 1/18 | 20/22 | $0.02 | |
| 2026-10-29 | ask-frozen-validation | ask | full | Opus 5.5 | 78/80 | 0/39 | 21/23 | $0.6376 | [frozen validation](bench2-frozen-validation.md); 77/80 complete correct |
| 2026-10-29 | ask-frozen-validation | ask | full | Sonnet 5.5 | 78/80 | 0/39 | 21/23 | $0.3138 | 77/80 complete correct |
| 2026-10-29 | ask-frozen-validation | ask | full | Haiku 5.5 | 77/80 | 0/39 | 22/23 | $0.0289 | 76/80 complete correct |
