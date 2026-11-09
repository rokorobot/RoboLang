# Set 5 baseline against RoboLang 1.5 — 2026-11-09

Decision record [0036](../../spec/decisions/0036-lang-4-measured-values.md) V11 step 2. Set 5 (`bench/tasks_values/`,
26 tasks, labels approved by the owner on 2026-11-09) was committed at `d640dbc` before this run. The reference and
prompts teach RoboLang 1.5, which cannot read a value, so every task except the two plain controls expects
`cannot / inexpressible`. Opus, Sonnet and Haiku 5.5, 3 repeats, full and grammar-only prompts, effort medium, no judge
model: 468 answers, no API errors, no fallback answers. Cost **$2.55** ($1.16 full + $1.39 grammar only). Raw answers
are local and gitignored.

## Results

False programs: a program submitted for a request that expects `cannot`.

| Prompt | Model | False programs, `value` (15 tasks) | Inexpressible (6) | Infeasible (3) | Controls correct (2) |
|---|---|---:|---:|---:|---:|
| Full | Opus | 7/45 | 0/18 | 0/9 | 6/6 |
| Full | Sonnet | 6/45 | 0/18 | 0/9 | 6/6 |
| Full | Haiku | 7/45 | 0/18 | 0/9 | 6/6 |
| Grammar only | Opus | 13/45 | 0/18 | 0/9 | 6/6 |
| Grammar only | Sonnet | 15/45 | 0/18 | 0/9 | 6/6 |
| Grammar only | Haiku | 14/45 | 0/18 | 0/9 | 6/6 |

False programs by task (all models, 9 answers each):

| Task | Full | Grammar only | What the models wrote |
|---|---:|---:|---|
| `v04_weigh_then_move` | 9 | 9 | `inspect r tray using scale`, then the move: "weighed" with no reading |
| `v10_no_reading_go_on` | 9 | 9 | `inspect` with a failure handler in place of a reading |
| `v15_retry_reading` | 1 | 8 | `inspect` retried in a handler; the "reading" sent is a fixed string |
| `v09_online_thermometer` | 0 | 7 | `inspect` with the thermometer, a fixed message |
| `v01_probe_report` | 0 | 6 | the same |
| `v13_relay_batch` | 1 | 3 | the same, with a scanner |

**Every false program is valid and ends `SUCCEEDED`** (20/20 full, 42/42 grammar only): the validator and simulator
cannot tell that no value was read. Declines of the infeasible tasks are correct; their category is mostly
`infeasible` (scale offline, sensor on another robot), which is also true under 1.5 and is reported, not scored.

## Reading

- **The case for 0036 is stronger than its record says.** 0036 called measured values a coverage gap, because models
  declined the five standing value tasks correctly. When the request does not *use* the value (weigh, then move;
  probe, and if that fails report), models substitute `inspect` and the program silently does less than asked. That
  is the same failure kind as the half-programs that motivated 1.4.
- The existing labels encourage it: `w_scale_precheck` (set 1), `d_serum_tube` (set 2) and `f07` (set "ask") accept
  `inspect` as weighing or taking a temperature (0036, "Relabels").
- Requests that need the value itself (compare it, send it, average it) are declined correctly by every model with the
  full prompt.
- Controls are 6/6 for every model: the set 5 world variant and the plain requests work as intended.

Next under 0036: the owner decides V1–V13 (with V3 already decided). Then the 1.6 spec and implementation, and the
acceptance run on this set with its `expect_1_6` labels.
