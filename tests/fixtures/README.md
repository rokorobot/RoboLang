# RoboLang v1.1 fixtures

Conformance fixtures for the draft [v1.1 specification](../../spec/robolang-v1.1.md).

`tests/test_fixtures.py` runs them against the lexer and parser in `src/robolang/`:

- every file in `valid/` must parse, with exactly the warnings its header states;
- every `invalid/` file whose stage is `lexical` or `syntax` must be rejected with the stated
  code and line;
- `invalid/` files whose stage is `static` (`E3xx`) are **not** checked for rejection yet.
  Static validation is RL-11C. Until then the suite only confirms that they are
  syntactically valid.

## Layout

- `valid/` — files a v1.1 front end must accept.
- `invalid/` — files it must reject. The file name starts with the expected error code.

## Header

Each file starts with:

```rob
// RoboLang v1.1 fixture: invalid
// expect: error E202 (syntax) line 8
// why: every statement ends with ';'
```

`expect` is one of:

| Form | Meaning |
|---|---|
| `ok` | No errors, no warnings |
| `ok, warning W401 line N` | No errors; that warning on that line |
| `error E… (stage) line N` | First error reported: its code, stage (`lexical`, `syntax`, `static`) and line |

Line numbers count from the top of the file, header included. Each invalid file targets one
error; the header names the first one a front end reports.

## Invariant

For every file in `invalid/`, a conforming implementation makes **no adapter call**. Several
fixtures put the error after otherwise valid statements to exercise this.
