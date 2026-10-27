# RoboLang v1.2 fixtures (LANG-1)

Conformance fixtures for the [v1.2 specification](../../../spec/robolang-v1.2.md): combined
conditions. `tests/test_v1_2_fixtures.py` runs them (acceptance tests L-T1 to L-T6):

- every file in `valid/` passes the complete front end and matches its golden IR snapshot in
  `tests/snapshots/ir-v1.2/` (language 1.2, IR schema 1.1);
- every file in `invalid/` is rejected with the code and line its header states;
- every file in `compat/` keeps its RoboLang 1.1 result;
- every case in `simulation/cases.json` gives the stated exit status and run-record fields.

To regenerate the snapshots after an intended IR change:
`ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_v1_2_fixtures.py`.

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
