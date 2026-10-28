# RoboLang Core 0.1 fixtures

Conformance fixtures for [Core 0.1](../../../spec/core-v0.1.md) (RL-ARCH-02a).
`tests/test_core.py` runs them against `robolang.core` (ARCH-1): every case gets its stated
receipt (A-T16), and the schema files are checked with the `jsonschema` library (A-T17).

`tests/test_arch02_fixtures.py` additionally checks that the fixtures are consistent with the
specification: every file is used, codes and stages are defined, file names match codes, and
the `sim-hmac-sha256` signatures of the valid documents verify under the canonical form of
Core section 7.

## Layout

- `valid/` — documents a Core 0.1 receiver accepts: one per message type the scenario relies on
  most, and both capability descriptors.
- `invalid/` — documents it rejects. The file name starts with the expected code. Three
  `C101` files are deliberately not valid JSON documents.
- `context.json` — the receiver's situation: the current time, clock tolerance, maximum
  message lifetime, keys and their secrets, and each robot's authority.
- `cases.json` — each file with its expected result (`accepted`, or a code and stage), and,
  where a case needs it, another `receiver` or `receiver_mode`. A precedence fixture also lists
  `competing`: the codes of its other faults, in the order Core 8.1 applies them.

## Precedence fixtures (INTEROP-2f)

Seventeen `invalid/` files combine two or three faults, so that a receiver which checks them in
the wrong order reports a different code. The first fault's code is the receipt; each later
fault is really present: with the faults before it removed, the receiver reports that fault's
code (`tests/test_core_precedence.py`, built by `tests/core_precedence.py`).

| Fixture | Faults, in precedence order | Receipt | Rule (Core 8.1) |
|---|---|---|---|
| `c102_before_c106` | schema version `0.2`; reserved `type` | `C102` form | Stage 1, rank 2 before rank 3 |
| `c102_before_c201` | version; no `signature` | `C102` form | Rank 2 before rank 4 |
| `c102_before_c107` | version; `profile` not allowed | `C102` form | Rank 2 before rank 5 |
| `c102_before_c402` | version; `succeeded` with `verification: null` | `C102` form | Rank 2 before rank 6 |
| `c105_before_c201` | unknown `type`; no `signature` | `C105` form | Rank 3 before rank 4 |
| `c106_before_c201` | reserved `type`; no `signature` | `C106` form | Rank 3 before rank 4 |
| `c201_before_c107` | no `signature`; `profile` not allowed | `C201` signature | Rank 4 before rank 5 |
| `c201_before_c402` | no `signature`; `succeeded` with `verification: null` | `C201` signature | Rank 4 before rank 6 |
| `c103_before_c104` | unknown field; missing required field | `C103` form | Rank 7: `C103` before `C104` |
| `c204_before_c203_before_c202` | algorithm `ed25519`; another participant's key; wrong value | `C204` signature | Stage 2: `C204`, `C203`, `C202` (0012 P3) |
| `c203_before_c202` | another participant's key; wrong value | `C203` signature | Stage 2: `C203` before `C202` |
| `c202_message_before_c203_observation` | message: wrong value; observation: not its source's key | `C202` signature | Stage 2: the message before its observations |
| `c203_message_before_c204_observation` | message: another participant's key; observation: algorithm `ed25519` | `C203` signature | Stage 2: the message before its observations |
| `c302_before_c301` | lifetime 120.4 s; expired | `C302` freshness | Stage 3: `C302` before `C301` |
| `c302_before_c303` | lifetime 115 s; sent in the future | `C302` freshness | `C302` before `C303` |
| `c301_before_c307` | expired; another recipient (receiver planner-1) | `C301` freshness | `C301` before `C307` |
| `c303_before_c307` | sent in the future; another recipient (receiver planner-1) | `C303` freshness | `C303` before `C307` |

Two orderings have no fixture because the faults cannot occur together in 0.1 (Core 8.1): ranks
5 and 6 (the only type rank 6 concerns, `outcome`, allows both profiles), and `C301` and `C303`
(a message that passes `C302` cannot be both expired and sent in the future).

## Scope

These fixtures cover the checks a receiver can make on one message with the context above:
form, signature, freshness, recipient, authority (`C401`), outcomes (`C402`) and mode (`C404`).
Checks that depend on conversation state (`C306`, `C403`, duplicates and ordering) are
covered by the reference scenario's cases in [`../arch02/`](../arch02/README.md).

The secrets in `context.json` are fixture data for `sim-hmac-sha256`, which is simulation
only (Core section 7). They protect nothing.
