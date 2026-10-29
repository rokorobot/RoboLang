# Decision record 0015 — Report every diagnostic, not only the first

**Date:** 2026-10-29
**Status:** Implemented. Applies to the front end (lexer, parser, validator) and the `validate`,
`inspect` and `simulate` commands.
**Applies to:** [`spec/robolang-v1.1.md`](../robolang-v1.1.md) 5 and 6; [`spec/robolang-v1.2.md`](../robolang-v1.2.md) 5

## 1. Why

RoboLang is written mostly by AI. A model that repairs a file needs every problem at once;
one error per run costs one round trip per mistake. The README already says `validate` "checks
the whole file and prints diagnostics". The front end stopped at the first error.

## 2. Decision

1. **The validator collects every static error** (`E301`–`E308`), in source order.
2. **One error per item.** A statement, parameter, `@safety` annotation or condition atom gets at
   most one error. `grasp o r;` with the two names swapped is one `E304`, not two. An undeclared
   name (`E303`) is never also an `E304`. A duplicate parameter keeps its first declaration.
3. **The parser recovers at statement level.** After a syntax error inside `@safety`, `pre` or
   `plan` it skips past the next `;` or stops at the next `}`. A missing `;` (`E202`) skips
   nothing when the next token starts a statement or condition. Errors outside those blocks (the
   task header, block structure) skip to the next `task`, and only the first error of a task's
   structure is reported.
4. **A task with a syntax error is not statically validated.** Other tasks are. Syntax and static
   errors are merged in source order.
5. **Fatal errors stay fatal.** A lexical error (`E101`–`E104`) and a version-marker error
   (`E213` and friends) are reported alone. Recovering inside the lexer is messy and the
   later stages cannot be trusted without clean tokens.
6. **Cap of 50 errors**, then a "too many errors" note.
7. **API.** `RoboLangError.diagnostic` is still the first error; `.diagnostics` lists all (at most
   50), `.truncated` says more were found and `.warnings` carries the warnings. `check`,
   `check_file`, `parse` and `validate` still raise when there is at least one error.
8. **CLI.** Every diagnostic goes to stderr, in source order. When there is more than one
   diagnostic, `validate` ends with `FILE: not valid (3 errors, 1 warning)`. A file with exactly
   one error prints exactly what it printed before. Exit codes are unchanged.

## 3. Consequences

- The first error can now differ from before in one case: a static error in an earlier task now
  comes before a syntax error in a later task.
- Warnings are now printed for an invalid file too.
- Cascades are suppressed by rule, not by a general mechanism. A new rule must say which other
  checks it makes pointless.

## 4. Alternatives considered

- *Report every error in an item.* Simpler to explain, but one mistake gave two errors.
- *Recover in the lexer.* Rejected as above; revisit if lexical errors are common in generated code.
