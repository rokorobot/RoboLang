# Decision record 0034: A failure handler has no goal block

**Date:** 2026-11-04
**Status:** **APPROVED by the owner, 2026-11-04.** Changes [0033](0033-step-failure-handlers.md) F2 and answers the
open question of its section 4. Implemented before RoboLang 1.5 and Task IR 1.4 were merged.
**Applies to:** [`robolang-v1.5.md`](../robolang-v1.5.md), `grammar-v1.5.ebnf`, [IR schema 1.4](../ir-schema-v1.4.md),
[simulation 13](../simulation-v1.0.md).

## Decision

- A failure handler has no `goal` block. One in a handler is `E208`. Task IR 1.4 has no `goal_else` on a handled step;
  the IR reader rejects one (`I112`).
- RoboLang adds no "fallback outcome": a handler never replaces the goal of its path.
- A handler does one of two things: it **recovers** (retries, or reaches the same goal another way), or it **cleans
  up and ends in `fail`**. A run succeeds only on the goal the task states.
