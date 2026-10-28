# INTEROP-2g — Final bundle audit and freeze preparation

**Date:** 2026-10-28. **Audited source:** `main` at `707e992` (INTEROP-2f merged), plus the
editorial status corrections of this PR. **Authorization:** INTEROP-2g, 2026-10-28: audit and
bundle preparation. The freeze itself needs a separate owner approval (section 8).

**Question:** can a completely independent developer implement amr-07 from the gate 2 bundle
alone (`spec/interop-gate-2-v0.1.md` 4.1), without the Python reference, the harness-private
scenario or earlier RoboLang discussions?

**Answer:** yes, as far as this audit can establish. Every rule the reference amr-07 and its
receiver apply is stated in a bundled document (A1). Every operative rule of decision records
0010–0013 is incorporated in the bundled specifications, so none of those records is needed
(A2). The candidate bundle holds exactly the allowlist and no private material (A3). The 47
fixtures give their stated receipts from the bundle's own files (A4). The example view is
exactly what 6.4 allows (A5). The bundle is reproducible and verifiable with a standard tool
(A6). **No specification gap was found.** Four non-blocking risks are recorded (section 7).

## 1. Baseline preserved

| Component | Count | Checked by |
|---|---|---|
| Core 0.1 conformance fixtures | 47 (17 multi-fault) | `tests/test_core.py`, `tests/test_core_precedence.py`, `tests/test_interop_bundle.py` |
| Reference simulation cases (with sequencing) | 25 (9 sequencing) | `tests/test_arch02.py` |
| Golden records | 25, unchanged by this PR | `tests/test_arch02_golden.py` |
| INTEROP-1 self-test (external amr-07) | 25 cases, byte-identical records | `tests/test_interop.py` |

This PR changes no reference behavior, harness logic, schema, fixture or golden record.

## 2. A1 — Traceability of the reference amr-07 and its receiver

Every behavior of `Amr` (`src/robolang/arch02/participants.py`) and of the receiver it uses
(`Inbox`, `src/robolang/arch02/engine.py`; `check_message`, `src/robolang/core/receiver.py`),
mapped to the bundled section that requires it. "Sim" is `spec/reference-simulation-v0.1.md`,
"Core" `spec/core-v0.1.md`, "G2" `spec/interop-gate-2-v0.1.md`.

### 2.1 Receiver

| Reference behavior | Bundled rule |
|---|---|
| Strict JSON reading: UTF-8, no BOM, no duplicate key, integers only (`C101`) | Core 2.3, 8.1 rank 1 |
| Stage 1 checks and their precedence (`C102`–`C107`, `C201` without signature, form-stage `C402`) | Core 8.1 stage 1 table and fixtures; Core 4.1–4.2, 9 |
| Stage 2: message signature, then each observation in `observations`, then `evidence`; `C204`, `C203`, `C202` per object | Core 7, 8.1 stage 2 (0012 P3 inline) |
| JCS canonical form and `sim-hmac-sha256` | Core 7 |
| Trust context: amr-07 verifies with its own key and its view's verification keys only | Sim 5.3 (0013 H3 inline); G2 6.4 |
| Stage 3: `C302`, `C301`, `C303`, `C307`, exact comparisons with the clock tolerance | Core 8.1 stage 3 |
| Stage 4: duplicates by message ID per sender, kept until `expires_at` + tolerance; reply resent | Core 8.4; G2 6.3 `receipt` row (N2: the harness resends) |
| Streams keyed by sender, recipient and `conversation_id`; processing in `sequence` order from 1 | Core 3, 8.4 |
| Holds after a gap; release on fill, at `max_hold` (gap skipped) or `C301` at expiry; expiry wins a tie | Core 8.4; G2 6.2, 6.3 `set_timer` (N3) |
| Stale message `C306`; conflicting sequence while held `C306`, the held message untouched | Core 8.2, 8.4 (0012 P2 inline) |
| Stage 5 after stage 4 (`C401`, `C402` unmet criterion, `C404`, then `C306`/`C403` state rules) | Core 8.1 stage 5 |
| A stage-5 rejection is not remembered: a later copy is judged again (stale if passed) | Core 8.4 erratum (N13) |
| One receipt per delivery, first in its cycle; release receipts after it | G2 6.3 `receipt` row (N14) |
| A receiver does not compare `conversation_id` with its own | Core 8.4 |

### 2.2 amr-07

| Reference behavior (`Amr`) | Bundled rule |
|---|---|
| Sends: envelope fields, `expires_at = sent_at + message_ttl`, sequence per stream, signed with its own key | Core 3, 7; Sim 3 (`message_ttl`), 5.1; G2 6.4 (key) |
| Message IDs by the formula of Sim 5.2 | Reference serialization, not required (G2 3, 9.2; Sim 6.4 values) |
| `in_reply_to` per type | Sim 6.4 replies (N4) |
| Normative values: profile `r2r` for `outcome`, lease `granted_at` = send time, `evaluated_at`/`reported_at` = send time, `succeeded` with empty `reasons`, other outcomes with `verification: null` and `evidence: []` | Sim 6.4 values (N5) |
| Derived values: sub-goal, requester, hand-off place, mass, places, `any_participant_in_domain` | Sim 6.4 derived values (N10) |
| `request_assistance`: policy, then capability test; `capability_unsupported` (steps 1–2) or `capability_insufficient` (3–4); `offer_assistance` with `resource_held` and lease to `offer_lease_end`, else `decline` | Sim 6.4 inputs table and capability test (N8, N10); Sim 3 (`offer_lease_end`); Core 5.11 |
| `commit`: commitment `active`, goal `ACCEPTED` → `AUTHORIZING`, lease timer, then standing authorization or `authorization_request` | Sim 6.4 inputs table; Sim 7.2; Core 5.8, 5.10 |
| Standing authorization coverage, `granted_at ≤ now < expires_at − tolerance` | Sim 7.2; Sim 6.4 |
| `authorization_decision`: `granted` → `AUTHORIZED`, `reserve`; `denied` → `DENIED`, commitment `failed`, outcome `rejected` | Sim 6.4 inputs table; Core 8.2 (`C401`) |
| `reserve` the hand-off place until `reservation_end`; `reservation_decision` `granted` → `RUNNING` and move; `denied` → diagnostic only | Sim 3 (`reservation_end`), 6.4 inputs table, 7.5 |
| Before every move: authorization still established, goal `RUNNING`, commitment `active` and unexpired, granted reservation unexpired, safe direction; else `motion_refused` | Sim 6.2 rule 2, 6.4 |
| `coordinate` `ready_for_handover` → `peer_ready`; `placed` → sense `carried_by`, `custody_confirmed` or `no_confirmation_evidence` | Sim 6.4 inputs table; Sim 4.3 |
| `transfer_custody` → `ACKNOWLEDGED`, `acknowledge` (`accepted: true`), `ACCEPTED` | Sim 6.4 inputs table; RFC-001 10.4 |
| Forwarding its reading in `custody_confirmed` without a local freshness check | Sim 6.4 forwarding (N9) |
| `custody_released`: `C403` first, then `C306`; `HOLDING`, commit point, `acknowledge`; sense (rule 9), then move or `physical_custody_unknown` | Sim 6.4 inputs table; RFC-001 10.4 rules 3, 9 |
| `custody_query`: `C306` without a sub-goal; `custody_state` with the accepted release if `HOLDING` | Sim 6.4 inputs table |
| Observations: IDs `rl:obs:<robot>:<n>`, numbered per robot, a null reading uses no number; when amr-07 senses | Sim 4.3 (N6) |
| Arrival: evaluate the criteria (relevant, fresh enough, met); `succeeded` / `unknown` (`evidence_missing`, `evidence_stale`) / `failed` with empty `reasons` | Sim 6.3 (N11) |
| Lease expiry: one timer at exactly `lease.expires_at`; `lapsed`, `lease_expired`, and the result by custody state | Sim 6.4 lease expiry (N12); Sim 6.2 rule 3 |
| A move in progress at expiry completes; afterwards no state change, sensing or message; not a physical guarantee | Sim 6.2 rule 3, 6.4 (0012 P1 inline); RFC-001 open question 10 stays open |
| Any other message type, or a missing state: `C306`; no reply to a rejected message | Sim 6.4 inputs table |
| Claims: goal and custody transitions, commitment state, authorization, commit point, escalation, `lease_expired`; five optional diagnostics | G2 6.3 claim variants (N7; 0010 B1, B2) |
| Binding: lock-step frames, sensing, timers, actions, unique names | G2 6.2–6.5 (N14) |

No behavior of the reference amr-07 or its receiver is untraced.

## 3. A2 — Decision records 0010–0013

None of these records is in the bundle (G2 4.1 names 0007 and 0008). Each operative rule is
stated in a bundled specification, marked with its row ID:

| Record | Rows | Where the rule is stated |
|---|---|---|
| 0010 | B1, B2 | G2 6.3 (event kinds, commitment state claim), 9.1.1 |
| 0011 | N1, N7, N14 | G2 6.3–6.5 |
| 0011 | N2 | G2 6.3 `receipt` row; Core 8.4 |
| 0011 | N3 | G2 6.2, 6.3 `set_timer` (including the ownership rules of 0011 section 3.1); Core 8.4 |
| 0011 | N4–N6, N8–N12 | Sim 4.3, 6.2, 6.3, 6.4 |
| 0011 | N13 | Core 8.1, 8.4 (including section 3.5(b): ties, "processed", `conversation_id`); G2 9.1.1 |
| 0012 | P1 | Sim 6.2 rule 3, 6.4 |
| 0012 | P2 | Core 8.2 (`C306`), 8.4 |
| 0012 | P3 | Core 8.1 stage 2 |
| 0013 | H3 | Sim 5.3 |

**Acceptance criterion met:** the independent author does not need an excluded decision record to
understand a required behavior. The bundled specifications cite these records by name (for
example "*Erratum (decision record 0011, N3)*"); those citations are informative.

**Links to files outside the bundle.** All are informative citations, classified and guarded by
`tests/test_interop_bundle.py` (a new unclassified link fails the test):

| Target | Cited from | Why informative |
|---|---|---|
| Decision records 0002, 0009–0013 | RFC-001, G2, Core, Sim | History and reasons; every operative rule is inline (above) |
| `docs/interop/spec-implementation-discrepancies.md` | G2, Sim | Reference-status register; D1–D7, N3, H1, H2, N13(c), H3 are all corrected |
| `docs/evidence/rl-arch-02-exit-gate.md`, `docs/DEVELOPMENT_PLAN.md` | G2, RFC-001 | Evidence and planning |
| `spec/robolang-v1.1.md`, `spec/ir-schema-v1.0.md`, `spec/simulation-v1.0.md` | RFC-001, Core, Sim | The language front end and the earlier simulator; Core 8.1 states its codes inline, and Sim 9 states its record format |
| `tests/fixtures/arch02/` | Sim, 0007, 0008 | The harness-private scenario and cases; Sim sections 2–8, 8.1 and 10 describe them in prose (G2 4.1) |

## 4. A3 — Allowlist and information barrier

The candidate bundle (`tools/interop_bundle.py build`) holds exactly:

| Path | Files |
|---|---|
| `spec/architecture/rfc-001-ai-native-architecture.md` | 1 |
| `spec/core-v0.1.md`, `spec/core/*.schema.json` | 1 + 2 |
| `spec/reference-simulation-v0.1.md` | 1 |
| `spec/decisions/0007-…`, `0008-…` | 2 |
| `spec/interop-gate-2-v0.1.md` | 1 |
| `tests/fixtures/core/` (README, `context.json`, `cases.json`, 47 fixtures) | 50 |
| `view/amr-07-base-scenario.json` | 1 |
| `BUNDLE-README.md` (delivery metadata, not normative), `MANIFEST.sha256` | 2 |

Checked by `tests/test_interop_bundle.py` from the built directory:

- the file set equals the allowlist;
- no source, tool, Python or test file, nothing from `tests/fixtures/arch02/`, no golden record;
- no golden `run_id` or record hash appears in any file;
- no harness-private data appears: the authority's decision table, durations, arm-02's policy and
  standing authorization, fault scripts or case expectations;
- no code.

`tests/fixtures/core/cases.json` is the public conformance manifest and is included; the
scenario's `cases.json`, `sequencing_cases.json`, `scenario.json` and `golden_records.json`
are excluded.

## 5. A4 — Conformance kit

- **Fixtures:** 47, each with exactly one entry in `cases.json`: `accepted`, or a code and stage
  defined in the bundle's Core 8.2. A case may name another `receiver` or `receiver_mode`.
  The 17 precedence fixtures also list `competing` codes, documented in the kit's README with
  the precedence matrix.
- **Standalone:** `tests/test_interop_bundle.py` reads the kit from the built bundle and gets
  every stated receipt, using only the bundle's `context.json` and `cases.json`.
- **Schemas:** both are valid JSON Schema 2020-12. Every `$ref` is internal, so the schemas
  resolve without any other file, and the valid message fixtures validate against them.
- **Check-mode contract:** G2 7 (read one fixture and `context.json`, print the code and stage
  or `accepted`), the kit README (layout, fields, the receiver default) and `context.json` (its
  `receiver` and `receiver_mode` notes) together define it. The output format is not fixed
  byte for byte (risk R-2).

## 6. A5 and A6 — Example view, reproducibility

**A5.** `view/amr-07-base-scenario.json` is `robolang.interop.build_view` applied to the base
scenario (no faults, no overrides): the approved builder of INTEROP-1, unchanged. Its keys are
exactly those of 6.4. Its `verification_keys` are arm-02's and the authority's (H3: no more), its
only sensor is `deck_camera` without lag or availability, and it has no goal, decision table,
durations or other participant's private configuration. The test compares the file to the
builder's output.

**A6.** `python tools/interop_bundle.py build OUT --rev COMMIT` reads every file from that git
revision (all are stored with LF), so the bundle is byte-identical on every platform. Two builds
are compared in the tests. `MANIFEST.sha256` is in `sha256sum` format, with sorted paths and LF
endings. `sha256sum -c MANIFEST.sha256` verifies it; `tools/interop_bundle.py verify DIR` also
rejects extra or missing files. The bundle ID is the SHA-256 of the manifest. `BUNDLE-README.md`
names the source commit, so the ID belongs to that commit.

## 7. Findings and risks

| # | Kind | Finding | Disposition |
|---|---|---|---|
| E-1 | Editorial, fixed here | G2 header ("no work authorized"), §11 (INTEROP-1 "24 cases") and §13 (status stopped at INTEROP-2a); Sim header ("Implementation: none"), §13 ("A-T2 to A-T14 in progress") and §15 (no INTEROP-2d/H3 row) were stale | Corrected; no rule changed |
| R-1 | Risk, accepted design | The kit's `context.json` holds all four `sim-hmac-sha256` secrets, identical to the scenario's, including planner-1's, which the view withholds (H3) | No gate 2 result depends on it: G-T0 uses `context.json` by design (G2 7), and no case sends from planner-1 to amr-07. G2 6.4 already says shared secrets prove nothing here. Changing it would change fixtures (not authorized) |
| R-2 | Risk, low | G2 7 does not fix the check-mode output byte for byte | G-T0 is "the stated receipt for every fixture"; the code and stage are unambiguous. A strict format, if wanted for automated G-T0, is an erratum |
| R-3 | Risk, clarity | G-T0's receiver context is `context.json` (all keys), while the live amr-07 verifies with its view's keys (Sim 5.3) | Both are stated. No fixture's receipt differs between the two contexts (checked: no fixture addressed to amr-07 is signed with a key outside its view) |
| R-4 | Risk, low | Bundled documents name reference files (`src/…`, test files) in status sections and link to excluded documents | Informative; names, not code or behavior. Every link is classified (section 3) and guarded by a test |

**Specification gaps:** none found. Nothing was stopped for a normative decision.

## 8. Freeze readiness and protocol

**Freezing can proceed** once this PR is approved and merged. Then:

1. Take the squash-merge commit of this PR on `main` as the definitive source commit.
2. `python tools/interop_bundle.py build gate2-bundle --rev <merge commit>`, in an empty directory.
3. `python tools/interop_bundle.py verify gate2-bundle`, and `sha256sum -c MANIFEST.sha256`.
4. Record the freeze: the source commit, the bundle ID (the SHA-256 of `MANIFEST.sha256`) and the
   manifest, in G2 13 and the development plan, in a separately approved change.
5. From then on, a change to any bundled file is an erratum with a new bundle (G2 4.2).

The candidate built from this PR's head is **not** the frozen bundle; its ID is given in the PR
for review only.
