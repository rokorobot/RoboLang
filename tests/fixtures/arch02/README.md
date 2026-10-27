# RL-ARCH-02 reference scenario fixtures

Acceptance fixtures for the [reference simulation](../../../spec/reference-simulation-v0.1.md)
of the RFC-001 section 18 scenario (RL-ARCH-02a). `tests/test_arch02.py` runs every case
against `robolang.arch02` (ARCH-2) and checks each `expect` field.

`tests/test_arch02_fixtures.py` also checks that the fixtures are consistent with the
specifications: the scenario's descriptors, standing authorizations, participants and
goal are well formed and correctly signed; every RFC-001 variant has a case; every case's
faults use defined kinds, its JSON Pointers resolve in the scenario, and its expected codes,
statuses, reasons and participants are defined.

## Files

- `scenario.json` — `robolang.arch02.scenario` version 0.1: the base scenario of
  specification sections 2 to 7.
- `cases.json` — one case per acceptance test (specification section 12): `id`, `tests`,
  `seed`, `faults` (section 10) and `expect` (section 11).
- `sequencing_cases.json` — the eight sequencing cases of specification section 8.1
  (acceptance test A-T20), added by
  [decision record 0008](../../../spec/decisions/0008-core-sequencing-per-recipient.md),
  approved on 2026-10-27. They describe per-recipient sequencing, which ARCH-2 does not yet
  implement, so `tests/test_arch02.py` does not run them; until then only their
  consistency is checked.

A case's `expect` lists only what that case checks. Determinism (A-T15) is checked by running
every case twice and comparing the run records byte for byte.
