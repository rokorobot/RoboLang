# Frozen BENCH-2 validation — 2026-10-29

Freeze SHA-256: `6c4d680a24ad0523029883c0f10357334676bb15ddb4073e0c4185e5903685d4`.
Manifest: [bench2-freeze.json](bench2-freeze.json). Scope: 80 ask cases × three models × one repeat, full prompt,
medium effort, one worker, local $10 cap. The owner explicitly authorized validation and merging PR #69.

## Offline checks

Ruff and the diff whitespace check pass. The full suite before committing the freeze reported 2,538 passed,
one skipped and six expected failures. The skip is the existing commit-pinned interoperability bundle test:
it deliberately skips when building code has local modifications. It will run from the clean committed checkout/CI.
Both replacement cases have two succeeding reference readings, accepted clarifications and rejected guessed defaults.
The GPT-6-family Codex continuation reviewed both labels; reasons are in
[the review response](bench2-label-review-response.md).

## Paid attempt: blocked by account credit

Command:

```sh
python bench/run.py --set ask --workers 1 --max-usd 10 --run-id ask-frozen-validation-20261029 --yes
```

The first request (`claude-opus-5-5`, `a01_which_vial`) returned HTTP 400, `invalid_request_error`:
“Your credit balance is too low to access the Anthropic API.” The runner stopped before trying the other models.
**No model answers were produced; no generation usage or cost was reported.** This is an account failure, not a model
score. Do not interpret it as 0/80 or as validation of the revised set. The failed record is preserved locally under
`bench/results/ask-frozen-validation-20261029/`.

After adding account credit, verify the manifest and use a fresh run ID, such as `ask-frozen-validation-20261029-funded`,
so the saved rejected request is not mistaken for a completed answer by the runner's resume logic. Existing PR CI
validation uses the repository's credential and is checked separately. Merge remains pending successful checks.
