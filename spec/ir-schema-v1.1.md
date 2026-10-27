# RoboLang Task IR — schema 1.1

**Status:** LANG-1 specification, **proposed for owner review** (decision record
[0006](decisions/0006-lang-1-combined-conditions.md), L6). Not implemented.
**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.1`
**Produced from:** RoboLang language version `1.2` ([specification](robolang-v1.2.md))
**Builds on:** [Task IR schema 1.0](ir-schema-v1.0.md), unchanged

Schema 1.1 is schema 1.0 plus three condition kinds. Everything not stated here, including
the trust boundary (1.0 section 2), quantities, strings and the determinism rules, is exactly
as in schema 1.0.

## 1. Version

- A 1.2 source produces `"ir_schema_version": "1.1"` and `"language_version": "1.2"`.
- A 1.1 source produces schema 1.0, byte-identical to today.
- The change is additive under the versioning rule of 1.0 section 1: no field is removed,
  renamed or redefined. A schema 1.0 consumer must reject an unknown `ir_schema_version` and
  unknown kinds (1.0 section 2, duty 1), so it cannot misread a 1.1 document.

## 2. Condition kinds

Added to the table of 1.0 section 3.3:

| `kind` | Other field |
|---|---|
| `and` | `conditions`: array of two or more condition objects, in source order |
| `or` | `conditions`: array of two or more condition objects, in source order |
| `not` | `condition`: one condition object |

A condition object is any kind of 1.0 section 3.3 or of this table, at any depth. Key order is
`kind`, then `conditions` or `condition`.

```json
{
  "kind": "and",
  "conditions": [
    {
      "kind": "or",
      "conditions": [
        { "kind": "region_clear", "region": "left" },
        { "kind": "region_clear", "region": "right" }
      ]
    },
    { "kind": "sensor_ok", "sensor": "camera_top" }
  ]
}
```

is the IR of `(region_clear left or region_clear right) and sensor_ok "camera_top"`. (Shown
compactly; the emitted text follows the determinism rules of 1.0 section 4.)

### 2.1 Canonical form

The IR depends only on the meaning of the source (1.0 section 4). For conditions:

1. **Grouping is flattened.** An `and` that is a direct operand of an `and` is replaced by its
   operands, in place; the same for `or`. `(a and b) and c`, `a and (b and c)` and
   `a and b and c` all produce one `and` with `conditions` `[a, b, c]`.
2. **Parentheses leave no trace.** `(a)` produces the IR of `a`.
3. **Nothing else is rewritten.** Operand order is kept. `not` is never removed or pushed
   inward: `not not a` stays two `not` objects. Duplicate operands are kept. No law of logic
   other than associativity of grouping is applied.

After flattening, an `and` never directly contains an `and`, an `or` never directly contains
an `or`, and every `and` and `or` has at least two operands.

## 3. Where conditions appear

As in schema 1.0: in a task's `preconditions` and in the `condition` of a `wait_until` step.

## 4. Consumer duties

The five duties of 1.0 section 2 apply. For duty 1, a schema 1.1 consumer also rejects:

- an `and` or `or` with fewer than two operands, or with a direct operand of its own kind;
- a `not` without exactly one `condition`;
- a condition nested more than 8 levels deep, counting each `and`, `or` and `not` object as
  one level. (The source bound of `robolang-v1.2.md` section 3.3 counts parentheses, so every
  IR this compiler produces is within this bound.)

Nothing in this repository reads IR yet; reading and re-validating IR is work item CORE-1.
