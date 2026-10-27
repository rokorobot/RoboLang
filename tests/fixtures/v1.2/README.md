# RoboLang v1.2 fixtures (LANG-1)

Conformance fixtures for the [v1.2 specification](../../../spec/robolang-v1.2.md): combined
conditions. **No v1.2 front end exists yet**; LANG-1b will run these fixtures.

Until then, `tests/test_v1_2_fixtures.py` checks only that they are well formed and consistent
with the specification, and that today's 1.1 front end:

- rejects every file in `valid/` and `invalid/` with `E213` at its version marker, so a 1.2
  file is never misread by a 1.1 front end;
- gives every file in `compat/` the result its header states, which must not change when
  LANG-1b is implemented.

These fixtures are separate from the 76 RoboLang 1.1 fixtures in `../valid/` and
`../invalid/` and from the RL-BASIC-01 simulation fixtures in `../simulation/`, which are
unchanged.

## Layout

- `valid/` — 1.2 files a conforming front end must accept with no diagnostics.
- `invalid/` — 1.2 files it must reject. The file name starts with the expected error code.
- `compat/` — 1.1 files (unmarked or `robolang 1.1;`) whose result stays exactly as in 1.1.
- `simulation/` — sources, world files and `cases.json` for specification section 8, in the
  format described in [`../simulation/README.md`](../simulation/README.md). Every world file
  is a valid world file 1.0; `base.json` is complete, and every other file differs from it in
  the respect its name gives. The `tests` field names the LANG-1 acceptance tests
  (specification section 10).

## Header

The `.rob` fixtures use the header of the 1.1 fixtures:

```rob
// RoboLang v1.2 fixture: invalid
// expect: error E214 (syntax) line 10
// why: and and or are not mixed without parentheses
```

The first line names the language version the file is read as and its set (`valid`,
`invalid`, `compat`). `expect` is `ok` or `error E… (stage) line N`, as in
[`../README.md`](../README.md). Line numbers count from the top of the file, header included.
