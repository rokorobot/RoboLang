# RoboLang v1.3 fixtures

Conformance fixtures for the [v1.3 specification](../../../spec/robolang-v1.3.md): declared
goals, outcome atoms and typed sensors. `tests/test_v1_3_fixtures.py` runs them (acceptance tests
G-T1 to G-T12):

- every file in `valid/` passes the complete front end and matches its golden IR snapshot in
  `tests/snapshots/ir-v1.3/` (language 1.3, IR schema 1.2);
- every file in `invalid/` is rejected with the code and line its header states;
- every file in `compat/` is a 1.1 or 1.2 file that keeps its earlier result;
- every case in `simulation/cases.json` gives the stated exit status and run-record fields.

To regenerate the snapshots after an intended IR change:
`ROBOLANG_UPDATE_SNAPSHOTS=1 python -m pytest tests/test_v1_3_fixtures.py`.

These fixtures are separate from the 1.1 fixtures in `../valid/` and `../invalid/`, the 1.2
fixtures in `../v1.2/` and the RL-BASIC-01 simulation fixtures in `../simulation/`, all unchanged.

## Layout

- `valid/` — 1.3 files a conforming front end must accept with no diagnostics.
- `invalid/` — 1.3 files it must reject. The file name starts with the expected error code.
- `compat/` — 1.1 and 1.2 files whose result stays exactly as before, including files that use the
  new reserved words as names and files that show the 1.3 constructs are not available earlier.
- `simulation/` — sources, world files and `cases.json` for specification section 8, in the format
  described in [`../simulation/README.md`](../simulation/README.md), with one addition:
  `expect.goal` lists the `result` of each entry of the run record's `goal` array. `worlds/base.json` is
  complete and has an entry for every source (and for `examples/pick_and_place_v13.rob`); every other
  world differs from it in the respect its name gives. Cases that expect exit status 3 give
  `expect.diagnostic` and write no run record.

## Header

The `.rob` fixtures use the header of the earlier fixtures:

```rob
// RoboLang v1.3 fixture: invalid
// expect: error E309 (static) line 7
// why: a 1.3 task must declare its goal
```

Line numbers count from the top of the file, header included.
