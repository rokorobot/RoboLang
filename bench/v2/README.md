# BENCH-2: knowing when not to write a program

80 cases in three worlds: 20 feasible (8 decoys), 20 infeasible, 20 unsupported in RoboLang 1.3, 20 ambiguous. The model
answers with a program, a rejection or a question. Design: [docs/plans/bench-2.md](../../docs/plans/bench-2.md);
decision record [0025](../../spec/decisions/0025-bench-2.md). BENCH-1 (`bench/`) is unchanged; this reuses its
`lib.py` and `run.py` by import.

```bash
pip install -r requirements-dev.txt -e . && pip install -r bench/requirements.txt
python bench/v2/run.py                                    # estimate only
python bench/v2/run.py --yes --max-usd 10                 # full prompt, 1 repeat (the pilot)
python bench/v2/run.py --variant minimal --repeats 3 --yes --max-usd 50
python bench/v2/score.py bench/results/<run-id>           # offline: report.md, scores.json
```

| Path | What |
|---|---|
| `cases/<id>.json` | `class`, `prompt`, `world`, `tags`, and per class: `expected_goal` (feasible); `faithful_goal`, `required_entities`, `blockers`, `fix_patch` or `no_fix` (infeasible); `construct`, `argument` (unsupported); `slot`, `candidates` (ambiguous) |
| `reference/<id>.rob`, `.json` | the reference (feasible), the faithful program (infeasible), the rejected attempt (`E211` cases) |
| `reference/<id>.<n>.rob`, `.json` | one succeeding program per reading (ambiguous) |
| `worlds/*.json` | pharmacy, yard, kitchen (world file 1.1) |
| `prompt-full.md`, `prompt-minimal.md` | the two prompt variants (hashed into each run) |
| `lib.py`, `run.py`, `score.py` | outcome classes; runner with `--max-usd`; scorer |

`tests/test_bench2.py` checks every label mechanically (design §5) and every outcome class.
