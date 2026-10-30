# RoboLang Core 0.2 fixtures

Conformance fixtures for the [Core 0.2 draft](../../../spec/core-v0.2.md) (decision
[0028](../../../spec/decisions/0028-core-0-2-scope.md) section 4, criterion 2). `tests/test_core_v02.py` runs them
against `robolang.core`. The 0.1 fixtures in [`../core/`](../core/README.md) are unchanged.

- `valid/` accepted by a receiver configured for 0.1 and 0.2: goals with one, two and sixteen alternatives; outcomes
  naming the pursued alternative; failed outcomes (other alternative holding, no `met_alternative`, `declared`).
- `invalid/` rejected. The file name starts with the expected code. One fixture per new or extended rule.
- `expected.json` lists each file with `accepted`, or a code and stage. `receiver_versions` (default
  `["0.1","0.2"]`) is the version configuration of the receiver. Entries whose file starts with `../core/` are 0.1
  documents accepted by a 0.2 reader. The receiver, time and keys are those of `../core/context.json`.

The `sim-hmac-sha256` signatures are fixture data (simulation only, Core section 7). Every invalid fixture is signed,
so its stated code is the first fault the receiver meets, not a signature failure.

Combined faults: `c405_before_c402` (a `declared` reason in a `succeeded` outcome with `met_alternative` null) fixes
the order inside stage 5 (`C405` before `C402`).
