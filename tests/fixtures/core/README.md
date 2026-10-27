# RoboLang Core 0.1 fixtures

Conformance fixtures for [Core 0.1](../../../spec/core-v0.1.md) (RL-ARCH-02a). **No Core
implementation exists yet**: ARCH-1 will run these cases (acceptance tests A-T16 and A-T17 of
the [reference simulation](../../../spec/reference-simulation-v0.1.md)).

Until then, `tests/test_arch02_fixtures.py` checks only that the fixtures are consistent with
the specification: every file is used, codes and stages are defined, file names match codes,
and the `sim-hmac-sha256` signatures of the valid documents verify under the canonical form
of Core section 7. It executes no Core check.

## Layout

- `valid/` — documents a Core 0.1 receiver accepts: one per message type the scenario relies on
  most, and both capability descriptors.
- `invalid/` — documents it rejects. The file name starts with the expected code. Three
  `C101` files are deliberately not valid JSON documents.
- `context.json` — the receiver's situation: the current time, clock tolerance, maximum
  message lifetime, keys and their secrets, and each robot's authority.
- `cases.json` — each file with its expected result (`accepted`, or a code and stage), and,
  where a case needs it, another `receiver` or `receiver_mode`.

## Scope

These fixtures cover the checks a receiver can make on one message with the context above:
form, signature, freshness, recipient, authority (`C401`), outcomes (`C402`) and mode (`C404`).
Checks that depend on conversation state (`C306`, `C403`, duplicates and ordering) are
covered by the reference scenario's cases in [`../arch02/`](../arch02/README.md).

The secrets in `context.json` are fixture data for `sim-hmac-sha256`, which is simulation
only (Core section 7). They protect nothing.
