# RL-ARCH-02 / Gate 1 exit — evidence

**Date:** 2026-10-27
**Status:** **Gate 1 — PASS. Signed off by the owner on 2026-10-27**, on the evidence at `6df6fa0`
(section 10). A separate technical review also approved gate 1 as PASS with no blocking
exceptions. This document changes no implementation, dependency or approved contract.
**Gate:** Gate 1 — reproducible reference ([RFC-001 section 1.4](../../spec/architecture/rfc-001-ai-native-architecture.md#14-how-the-ambition-is-tested)),
the RL-ARCH-02 exit gate of [`reference-simulation-v0.1.md`](../../spec/reference-simulation-v0.1.md)
section 12.
**Specifications:** [`core-v0.1.md`](../../spec/core-v0.1.md), [`reference-simulation-v0.1.md`](../../spec/reference-simulation-v0.1.md),
decision records [0007](../../spec/decisions/0007-rl-arch-02-core-and-reference-simulation.md) (R1–R14) and
[0008](../../spec/decisions/0008-core-sequencing-per-recipient.md) (Q1–Q9)
**Evidence commit:** `main` at `6df6fa0` (ARCH-3 merge, PR #30)

## 1. Gate criteria

| Criterion | Source | Evidence | Met |
|---|---|---|---|
| A reproducible simulation of the RFC-001 section 18 scenario shows correct messaging, local authorization, failure handling and evidence-based completion | RFC-001 1.4, gate 1 | Sections 3 to 5 | Yes |
| The happy path ends in a verified `succeeded` outcome; every failure variant ends in its stated result with no simulated motion where none is allowed; every run produces a reconstructable message and event log | RFC-001 19 (acceptance) | A-T1, A-T2–A-T14, A-T15; section 5 | Yes |
| A-T1 to A-T20 pass in CI on every supported platform | Reference simulation 12 (as amended by 0008, Q9) | Section 3; CI in section 2 (Ubuntu and Windows, Python 3.11 and 3.13) | Yes |
| The README states that the reference simulation is simulation only | Reference simulation 12 | `README.md`: the `arch02` Quickstart bullet ("**Simulation only**: one process, a symbolic world, no network, no ROS 2, no hardware, and simulation-only signatures") and the Implemented-today row | Yes |
| Owner approval | Reference simulation 12 | Section 10 | Yes — 2026-10-27 |

## 2. Implementation traceability

Every pull request was squash-merged at a head whose CI passed 4/4 (Ubuntu and Windows,
Python 3.11 and 3.13: ruff, the full suite, the v1 demo, and `tools/smoke_install.py`, which
builds and installs the wheel and source distribution and runs the installed command), and
CI on `main` passed after each merge.

| Slice | PR | Approved head | PR CI run | Merge commit | `main` CI run |
|---|---|---|---|---|---|
| RL-ARCH-02a — specification, schemas, fixtures | #25 | `19dc5cc` | 40466620356 | `a80ffe0` | 40466817796 |
| ARCH-1 — Core 0.1 checks (`robolang.core`) | #26 | `e45a934` | 40468141885 | `d88d104` | 40469270558 |
| ARCH-2 — reference simulation (`robolang.arch02`) | #27 | `1833978` | 40471053313 | `8580075` | 40471388127 |
| RL-ARCH-02-SEQ-01 — decision record 0008 | #28 | `da8737d` | 40476627246 | `79ea239` | 40476868713 |
| ARCH-SEQ — per-recipient sequencing | #29 | `2d892f0` | 40508578675 | `dfc95d4` | 40508969251 |
| ARCH-3 — run record, run ID, `robolang arch02` | #30 | `5c9f771` | 40510278236 | `6df6fa0` | 40510696688 |

**Evidence commit `6df6fa0`:** `main` CI run 40510696688 — 4/4 jobs passed, **1311 passed and
6 expected failures** in every job, all packaging checks passed (including "arch02 runs the
reference scenario's happy path in simulation", from the installed wheel and source
distribution); Pages deployment 40510696663 succeeded. Re-run locally on Windows with
Python 3.13 at the same commit: 1311 passed, 6 xfailed; `ruff check .` clean.

## 3. Acceptance matrix

| # | Test | Where | Result at `6df6fa0` |
|---|---|---|---|
| A-T1 | Happy path: the 20 messages of spec 6.1 in order, verified `succeeded`, designated custody amr-07, C42 at S3 | `tests/test_arch02.py`: `test_case_meets_its_expectation[happy_path:*]`, `test_happy_path_follows_the_specification_timeline` (all 20 send times equal the table), `test_happy_path_outcome_is_verified_by_evidence` | Pass |
| A-T2 … A-T14 | Failure variants V1 … V13 (14 cases; V12 has two) | `test_case_meets_its_expectation[v01_…–v13_…:*]`: every `expect` field of `tests/fixtures/arch02/cases.json` | Pass (section 5) |
| A-T15 | Determinism: byte-identical records and run ID on rerun; seed changes only the run ID | `tests/test_arch02_record.py`: `test_record_is_byte_identical_when_run_twice` (24 cases), `test_seed_changes_only_the_run_id_and_the_seed`; `tests/test_arch02.py`: `test_run_is_deterministic` | Pass |
| A-T16 | Core fixtures: every valid document accepted, every invalid one rejected with its code and stage | `tests/test_core.py`: `test_core_fixture_gets_its_stated_receipt` (30 cases) | Pass |
| A-T17 | Schemas valid JSON Schema 2020-12; fixtures validate exactly as their stage says | `tests/test_core.py`: `test_schema_is_valid_json_schema_2020_12`, `test_fixture_passes_the_schema_exactly_when_its_stage_says` (the package's validator agrees with `jsonschema` on every document) | Pass |
| A-T18 | Isolation and mode: no `rclpy`, adapter, `socket`, `ssl`, `http` or `urllib.request` module loaded; every record says `"mode": "simulation"`; `physical` mode rejected | `test_importing_robolang_core_loads_no_network_or_adapter_module`, `test_importing_robolang_arch02_loads_no_network_or_adapter_module`, `test_arch02_imports_only_robolang_core_from_the_package`, `test_record_has_the_specified_shape` (24 cases), Core fixture `c404_physical_mode` | Pass |
| A-T19 | Regression: RoboLang 1.1 and 1.2, IR snapshots, RL-BASIC-01 and LANG-1 cases and run-ID vectors unchanged | The full suite: 1311 passed, 6 expected failures (the `xfail` known-limitation tests of `tests/test_known_limitations.py`, unchanged by RL-ARCH-02) | Pass |
| A-T20 | Sequencing (decision record 0008): 8 cases | `test_case_meets_its_expectation[seq_…:*]` including `held` and `processed_by`; `test_held_message_has_no_effect_until_released`; `test_sequence_numbers_count_per_stream` | Pass |

Test counts by file at `6df6fa0`: `test_core.py` 95, `test_arch02.py` 142, `test_arch02_record.py` 58,
`test_arch02_fixtures.py` 61.

## 4. Reproducibility

**Inputs.** `tests/fixtures/arch02/scenario.json`, normalized SHA-256
`58a70241c009a918068c1c0a3f6b101dc60df141071c282a050f38604d249cab`; cases in
`tests/fixtures/arch02/cases.json` (16) and `sequencing_cases.json` (8).

**Reproduce.** From a clone at `6df6fa0`, after `python -m pip install .`:

```bash
robolang arch02 --scenario tests/fixtures/arch02/scenario.json --cases tests/fixtures/arch02/cases.json --case happy_path
```

The run ID of `happy_path` (seed `0`) is
`arch02-b15d8830279932950822877f5de7252f6f3aef2b2623f385369f736a9207cb82`.
`tests/test_arch02_record.py` recomputes every run ID from spec section 9.1 without the
package; the scenario hash ignores a BOM and CR LF, so the ID is the same on every platform.

**All 24 cases**, each run twice through `robolang arch02` (2026-10-27, Windows, Python 3.13,
`6df6fa0`). Every pair was byte-identical; exit status 0 for all. Run ID and record SHA-256
are shown by their first 12 hex digits.

| Case | Run ID | Record SHA-256 | planner-1 | Motion arm-02 / amr-07 | C42 responsible | Rejections / held / duplicates |
|---|---|---|---|---|---|---|
| `happy_path` | `b15d88302799` | `b368933e321e` | succeeded | 1 / 2 | amr-07 | — |
| `happy_path_seed_1` | `b324b0fe8021` | `dad8f12a9349` | succeeded | 1 / 2 | amr-07 | — |
| `v01_declines` | `e166ece00931` | `b5085f83d313` | rejected | 0 / 0 | arm-02 | — |
| `v02_duplicate_commit` | `ed82670d905f` | `2d10a72b871d` | succeeded | 1 / 2 | amr-07 | 1 duplicate |
| `v03_replayed_commit` | `caf4e8d84bf6` | `1c7397503c0e` | succeeded | 1 / 2 | amr-07 | `commit` C301 |
| `v04_assistant_silent` | `57ee49a8972e` | `bc3f78c88a2a` | failed | 0 / 0 | arm-02 | — |
| `v05_stale_confirmation` | `79fa4cfcdd4f` | `d24e050aedb2` | failed | 1 / 1 | arm-02 | — |
| `v06_partition_before_release` | `0be8e15746df` | `7dd26e370f0a` | unknown | 1 / 1 | arm-02 | — |
| `v07_release_ack_lost` | `65ad935dd8d1` | `a56d8f0e3f4a` | succeeded | 1 / 2 | amr-07 | 1 held |
| `v08_arrived_without_evidence` | `5867134fe347` | `3b4e3f26e788` | unknown | 1 / 2 | amr-07 | — |
| `v09_stale_reverification` | `4ecfb92fe5cb` | `5e1d22c9f8f8` | unknown | 1 / 2 | amr-07 | — |
| `v10_requester_authorizes` | `52046f143faf` | `8a9d3df887aa` | succeeded | 1 / 2 | amr-07 | `authorization_decision` C401 |
| `v11_payload_beyond_standing_authorization` | `3efb508107f5` | `016714fe1f0c` | rejected | 0 / 0 | arm-02 | — |
| `v12a_unsigned_commit` | `7808906d916b` | `347f96e29fa8` | rejected | 0 / 0 | arm-02 | `commit` C201 |
| `v12b_wrongly_signed_commit` | `59af7997a729` | `57a83ade5e79` | rejected | 0 / 0 | arm-02 | `commit` C202 |
| `v13_no_grasp` | `ea8b32e4e4df` | `ddae698ce7cf` | rejected | 0 / 0 | arm-02 | — |
| `seq_gap_filled` | `776066983ce8` | `9786d05b57f7` | succeeded | 1 / 2 | amr-07 | 1 held |
| `seq_gap_abandoned` | `0a9ea5166905` | `4b7be79fbf14` | succeeded | 1 / 2 | amr-07 | `assessment` C301; 1 held |
| `seq_held_message_expires` | `c2e90fff2ed5` | `2d108070590d` | succeeded | 1 / 2 | amr-07 | `assessment` C301, `accept` C301; 1 held |
| `seq_duplicate_while_held` | `c7fe20f6d26f` | `7af80821880f` | succeeded | 1 / 2 | amr-07 | 1 held, 1 duplicate |
| `seq_replay_while_fresh` | `25f98f570127` | `d42d46f130ec` | succeeded | 1 / 2 | amr-07 | 1 duplicate |
| `seq_stale_nonduplicate` | `f9a335f49182` | `e60b43745aea` | succeeded | 1 / 2 | amr-07 | `assessment` C306 |
| `seq_unsigned_not_held` | `8c74489e919c` | `22473677b6f8` | succeeded | 1 / 2 | amr-07 | `accept` C201; 1 held |
| `seq_lost_ack_holds_next` | `c0e5bce63ad4` | `69759484322d` | succeeded | 1 / 2 | amr-07 | 1 held |

**What this shows, and what it does not.** Determinism is shown per platform: every CI job
(Ubuntu and Windows) runs every case twice and compares the records byte for byte, and
verifies every run ID against the formula. The record SHA-256 values above were taken on
Windows only; no test yet compares a record across platforms against a pinned golden record
(follow-up F1, section 9).

## 5. Safety and authority evidence

| Property | Evidence |
|---|---|
| **No motion before authorization** | `test_no_motion_happens_without_an_authorization_first`: in all 16 cases, every motion event of arm-02 and amr-07 comes after that robot's authorization event |
| **Accepting is not authorizing** | Happy path: arm-02 `accept`s at 10:14:00.500, requests an explicit grant (outside its `r2r` standing authorization) and moves only after the authority's decision at 10:14:20.700 |
| **V11: denial stops a capable robot** | `test_v11_denied_authorization_means_no_motion_despite_capacity`: amr-07 (40 kg capacity) is denied for 35 kg against its 30 kg standing authorization; zero amr-07 motion; planner-1 records `rejected` |
| **Requester is never its own authority** | V10: planner-1's self-issued `authorization_decision` is rejected with `C401`; arm-02's authorization record names the authority's decision |
| **Signatures before content** | V12a/V12b: unsigned and wrongly signed `commit` rejected with `C201`/`C202`, no reply, no commitment, no motion; `seq_unsigned_not_held`: a signature failure is never held and does not count in its stream |
| **Replay and duplicates** | V2: duplicate `commit` recognized, one commitment; V3: replay after expiry `C301`; `seq_replay_while_fresh`: replay before expiry is a duplicate; `seq_duplicate_while_held`: a copy of a held message has no second effect |
| **Custody commit rule** | V5: stale confirmation (40 s) → no release, custody stays with arm-02, amr-07 does not move after the placement; V6: release lost in a partition → amr-07 never reaches the commit point, arm-02 `IN_DOUBT`, both escalate at lease expiry, outcome `unknown`; V7: acknowledgement lost → amr-07 is responsible and continues, arm-02 reconciles with one `custody_query` |
| **No success without evidence** | V8: arrival without an observation → `unknown`, although the symbolic world has C42 at S3; V9: planner-1's re-verification finds 15.2 s old evidence → `unknown` |
| **Sequencing (0008)** | `test_held_message_has_no_effect_until_released`: a held message produces no state change until released; stale non-duplicate `C306`; expiry while held `C301`; `max_hold` bounds the delay (`seq_gap_abandoned`, `seq_lost_ack_holds_next`) |
| **Fail closed on unknown** | V4 (silent assistant), V5, V6, V8, V9 all end `failed` or `unknown`, never `succeeded` |

## 6. Deviations and corrections on record

| Item | Where recorded | Status |
|---|---|---|
| V11 raises arm-02's grasp limit to 40 kg so that amr-07's standing authorization is what the variant tests | Decision R11 (0007) | Approved 2026-10-27 |
| Core 8.4 sequencing ambiguity (sequence numbers across recipients) resolved by per-recipient streams and a bounded hold | Decision record 0008, Q1–Q9 | Approved 2026-10-27; implemented by ARCH-SEQ |
| Five 0008 example times were 100 ms early (a `delay` counts from normal delivery) | 0008 revision history | Corrected in PR #29; decision text unchanged |
| amr-07 reported an internal commitment state `reported`; now a Core 5.8 state | PR #30 | Corrected; no expectation depended on it |
| ARCH-1's isolation test was too strict on Python 3.11 (`pathlib` imports `urllib.parse`) | PR #26 | Corrected before merge |

## 7. Limitations (what gate 1 does not show)

- **Symbolic simulation only.** One process, one shared perfect clock, a symbolic world of
  facts; no geometry, kinematics, collisions, real time or clock skew; no crash and restart.
- **No real robot execution.** No adapter, ROS 2 or hardware; physical mode is rejected
  (`C404`). Nothing in a run record is evidence about a real robot.
- **No network interoperability.** No transport or binding (A2A, local peer-to-peer, ROS 2,
  MCP); messages are passed in memory with scripted latency and faults.
- **No production authentication.** `sim-hmac-sha256` uses secrets shared with the verifier;
  it shows that checks happen, not who signed. `ed25519`, key distribution and revocation are
  not implemented (RFC-001 open question 5).
- **One reference scenario.** One conversation, four participants, fixed policies as data
  (open question 11), bilateral commitments only (D5).
- **Not gate 2.** Interoperability with an independently written participant, using only the
  published schemas and specification, is a separate milestone, not scoped or authorized.

## 8. Stale milestone wording

Identified at `6df6fa0`. Corrections are status text only and change no decision.

| Location | Stale text | Handling |
|---|---|---|
| `README.md`, top summary | "Specified and approved, not implemented: versioned Core semantic contracts, A2R and R2R …" | Corrected in this PR: implemented in simulation only (Core 0.1 and the reference scenario); the rest remains specified, not implemented |
| `README.md`, Project status table | "Next milestone: RL-ARCH-02 … implementation needs separate owner authorization" | Corrected in this PR: implemented in simulation; gate 1 awaiting sign-off |
| `README.md`, Architecture | "Approved target, not implemented" | Corrected in this PR: exercised only by the Core 0.1 reference simulation |
| `README.md`, Current limitations | "No Core schemas yet … drafted in RL-ARCH-02" | Corrected in this PR: draft Core 0.1 schemas exist; simulation only, no transport, no independent implementation |
| `docs/DEVELOPMENT_PLAN.md`, Immediate next action | "The next planned milestone is RL-ARCH-02, which waits for the owner's separate implementation authorization" | Corrected in this PR |
| RFC-001 header note | "It defines no implemented behavior … to be validated by RL-ARCH-02 … nothing here describes a capability RoboLang has today" | **Not changed** (approved document). Proposed non-normative annotation, for the owner: "*Status note (2026-10-27): RL-ARCH-02 has implemented a draft Core 0.1 subset in a symbolic reference simulation; see docs/evidence/rl-arch-02-exit-gate.md. Nothing here describes a capability for real robots or networks.*" |
| RFC-001 section 2, line "The next planned milestone is one small executable proof (section 19)" | — | **Not changed.** Same annotation would cover it |
| RFC-001 section 19, RL-ARCH-02 row: "**Not started.**" | — | **Not changed.** Proposed status-only replacement: "**Implemented in simulation (2026-10-27); gate 1 exit evidence in docs/evidence/rl-arch-02-exit-gate.md.**" The row's scope, boundaries and the D8 text stay as approved |

## 9. Gate conclusion

**Recommendation: PASS.** Every gate 1 criterion of RFC-001 1.4 and of the reference
simulation's exit gate (A-T1 to A-T20 in CI on every supported platform; README states
simulation only) is met on `main` at `6df6fa0`, with owner approval the only open item.

**Exceptions:** none blocking. The deviations in section 6 are approved or corrected on record.

**Follow-ups (not gate conditions):**

| # | Follow-up | Why |
|---|---|---|
| F1 | A golden record test: pin the SHA-256 of the 24 records and check it on Ubuntu and Windows | Shows cross-platform byte identity directly (charter MC8), beyond per-platform determinism |
| F2 | Apply, or decline, the RFC-001 status annotations of section 8 | Keeps the approved RFC's status lines accurate without changing its decisions |
| F3 | Plan gate 2 (an independent participant from a separate codebase) | RFC-001 1.4; separate milestone, not authorized |

## 10. Owner sign-off

| Item | Decision | Date |
|---|---|---|
| RL-ARCH-02 / gate 1 exit met (sections 1–5) | **PASS — signed off by the owner**, on the evidence at `6df6fa0` | 2026-10-27 |
| Technical review (architecture and technical reviewer) | Gate 1 APPROVED, PASS; no blocking exceptions; F1–F3 non-blocking | 2026-10-27 |

Signing off gate 1 authorizes nothing further: not gate 2, not CORE-1, not any Foundation 2 or 3
item, no real network interoperability, and not any physical execution. Follow-ups F1–F3
(section 9) remain open and non-blocking; each needs its own scope and authorization.

## 11. Addendum: follow-ups after sign-off

Sections 1–10 are kept as signed. This section records follow-up work done afterwards.

| # | Follow-up | Status |
|---|---|---|
| F1 | Golden records: `tests/fixtures/arch02/golden_records.json` pins the run ID and the record SHA-256 of all 24 cases; `tests/test_arch02_golden.py` recomputes them on every CI job (Ubuntu and Windows, Python 3.11 and 3.13). The pinned values equal those of section 4, taken at `6df6fa0` | Authorized and delivered 2026-10-27 |
| F2 | RFC-001 status annotations (section 8) | Open |
| F3 | Gate 2 proposal | Open; not authorized |
