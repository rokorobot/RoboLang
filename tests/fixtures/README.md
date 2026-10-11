# RoboLang v1.1 fixtures

Conformance fixtures for the draft [v1.1 specification](../../spec/robolang-v1.1.md).

**No parser exists yet.** Nothing in the test suite reads these files; they are the target
for RL-11B (parser) and RL-11C (static checks). Until then they are reviewed by hand.

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
