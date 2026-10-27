# RL-ARCH-02 reference scenario fixtures

Acceptance fixtures for the [reference simulation](../../../spec/reference-simulation-v0.1.md)
of the RFC-001 section 18 scenario (RL-ARCH-02a). **No simulation exists yet**: ARCH-2 and
ARCH-3 will run these cases.

Until then, `tests/test_arch02_fixtures.py` checks only that the fixtures are consistent with
the specifications: the scenario's descriptors, standing authorizations, participants and
goal are well formed and correctly signed; every RFC-001 variant has a case; every case's
faults use defined kinds, its JSON Pointers resolve in the scenario, and its expected codes,
statuses, reasons and participants are defined. It runs nothing.

## Files

- `scenario.json` — `robolang.arch02.scenario` version 0.1: the base scenario of
  specification sections 2 to 7.
- `cases.json` — one case per acceptance test (specification section 12): `id`, `tests`,
  `seed`, `faults` (section 10) and `expect` (section 11).

A case's `expect` lists only what that case checks. Determinism (A-T15) is checked by running
every case twice and comparing the run records byte for byte.
