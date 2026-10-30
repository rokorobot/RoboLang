# Decision record 0028: Core 0.2, scope and compatibility (N5)

**Date:** 2026-10-30
**Status:** **PROPOSED.** Owner approval pending. Spec only: no code, no schema file and nothing in `src/` changes.
**Applies to (when decided):** [`core-v0.2-draft.md`](../core-v0.2-draft.md) (this record's delta over
[`core-v0.1.md`](../core-v0.1.md)), later `spec/core/message-0.2.schema.json`, `export-goal`, `delegate`, the simulation
outcome ([`simulation-v1.0.md`](../simulation-v1.0.md) 10.5, 12.4)
**Evidence:** [set 2 analysis](../../docs/evidence/bench-set2-analysis.md) F2 (models write `or` for "one of the
vials" and "the same shelf", and the goal cannot be exported); [0027](0027-bounded-branching.md) B4 and B7 (a
branching task has several goals, and `fail` has no Core reason); [0018](0018-core-shaped-outcomes.md) and
[0023](0023-keep-interop-3-deferred.md) (the known 0.1 gaps). The set 3 acceptance run is not on file yet
([0027](0027-bounded-branching.md) section 5).

## 1. What Core 0.1 does today (the facts this record rests on)

- A goal has one `desired_state` (a list of triples) and one `success_criteria` list, at least one of each
  ([`core-v0.1.md`](../core-v0.1.md) 5.1, line 162). Criterion `i` in a result is the integer index into that list
  (5.9, line 244).
- `succeeded` needs a verification in which every criterion is met (`C402`; 5.9, 8.2 line 442).
- The reason codes are a closed list of twelve; a reason is `{ "code" }` only (2.4, 5.11 line 272). None says "the
  robot declared this failure".
- Every field is required, unknown fields are rejected (`C103`), and there are no extension points (2.3, lines
  83–84). A receiver rejects an unsupported version with `C102` (2.2, line 69).
- Versions and descriptors are configured per peer; discovery and negotiation are left out (10, line 516).
- `export-goal` exports only a conjunction of `object_at`, `robot_at` and `holding` atoms
  ([`export-goal-v0.1.md`](../export-goal-v0.1.md) section 2 "Exportable goals"). `or` gives `X601`; a branching task
  whose paths differ, or one path ending in `fail`, gives `X603` ([0027](0027-bounded-branching.md) B7).
- Core has no "nowhere": an absent object gets no observation, and its criterion is just not met
  (`sim/outcome.py` `object_triple`, line 25; [0018](0018-core-shaped-outcomes.md) Consequences).
- The robot matches a proposed goal as a set and re-indexes its outcome to the proposal
  ([`delegate-v0.1.md`](../delegate-v0.1.md) section 3 item 3, amended by [0020](0020-delegate.md) D2). Core itself says
  nothing about re-indexing.

## 2. Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| C1 | "Any of" desired states | A goal carries `alternatives`: a list of 1 to 16 alternatives. Each is `{desired_state, success_criteria}` with the 0.1 meaning (a conjunction). The goal is met when **any one** alternative is met. A plain goal is one alternative. The two 0.1 fields leave the goal and live in the alternatives, so there is still one document shape | (b) Keep the 0.1 fields and add an optional `alternatives`: two shapes for one meaning, against 0.1 2.3. (c) A boolean expression tree: nobody needs nesting, and a bounded list is easy to check and test. (d) One `propose_goal` per alternative: the robot would be asked for all of them, not any |
| C2 | Which alternative was met, and indexing | `criteria_results` entries gain `alternative` (integer from 0), beside `criterion`. `verification` gains `met_alternative`: the index the verifier relies on, or `null`. Indices are positions in the goal **as sent** and never change. `succeeded` needs a non-null `met_alternative` whose criteria are all met (`C402`). The verifier may report `failed` even if another alternative holds (see Q3 below) | (b) Name alternatives with ids: one more field to keep unique, and positions are already how 0.1 indexes criteria. (c) Infer the met alternative from the results: a receiver would have to guess when two hold |
| C3 | Declared failure reason | New reason code `declared` with a `text` field: `{ "code": "declared", "text": "box1 missing" }`. `text` is 1 to 200 characters, no control characters. Allowed only in an `outcome` with status `failed` (`C405`). It is the reporter's own words: never evidence, never used to decide a status, an authorization or a verification, and shown to people and AI planners as untrusted text | (b) A code per cause: the cause is operator text, the list would never close. (c) Keep it out of Core: the planner of a delegated task sees `failed` with no cause, and the sim spec already promised 0.2 (12.4). (d) Allow it in every `reasons` list: no producer needs it in `reject` or `decline` |
| C4 | Absent thing ("nowhere") | **Out of scope.** No producer: no benchmark task or recorded request wants a goal "box1 is not present" (`not object_exists` is local-only, and [0027](0027-bounded-branching.md) B4 rejected forcing it as a goal). The draft only states 0.1 behaviour: an unobservable criterion is not met, with empty evidence. Design kept for later: a property `absent_from` whose value is the place or domain that was searched, because absence is always relative to what was observed | (b) A reserved place `rl:place:nowhere`: identifiers are opaque (0.1 2.4) and "nowhere" is not a place. (c) Add `absent_from` now: untested syntax, against strategy rule 1 (evidence before syntax) |
| C5 | Re-indexing | Keep positional indices (C2) and make the rule normative: an outcome indexes alternatives and criteria as the **proposal** did. A robot that matches a goal in another order, or as a set, must re-index. How a robot matches stays local policy, not Core. This writes down what `delegate` already does (0020 D2 amendment) | (b) Criterion ids: robust to reordering but adds a field to every criterion for a problem that positional indices with a rule already solve. Revisit if a second implementer trips on it. (c) Leave it implementation-defined: the planner cannot tell a reordered outcome from a wrong one |
| C6 | Export of branching tasks (0027 B7) | The alternatives of a branching task are the **distinct effective goals of its paths**, in path order (source order, `then` before `else`). A path ending in `fail` adds no alternative: it is a declared failure (C3). A top-level `or` of conjunctions of the three outcome atoms is exported as alternatives too (F2), up to 16. `not` and non-outcome atoms stay unexportable (`X601`). `X603` is retired for 0.2 | (b) Keep `X603` and only export equal paths: leaves the producer 0027 named unserved. (c) Export `or` only, not branching: the bridge would then have two rules for one idea |
| C7 | Versioning | Messages become `schema_version` `0.2`, schema file `message-0.2.schema.json`. The 0.1 message schema file is not touched. The capability descriptor does not change, so it stays `0.1` and keeps its signatures. A 0.2 implementation accepts 0.1 and 0.2 messages. A 0.1 document is read as a 0.2 goal with one alternative. 0.1 documents stay valid, byte for byte | (b) Bump everything to 0.2: re-signs every descriptor for no change. (c) Make the 0.1 schema accept 0.2 fields: breaks the frozen gate 2 bundle (`e6c9bb75…d104`) |
| C8 | A 0.1-only peer meets a 0.2 peer | **No negotiation in 0.2.** Versions per peer stay configuration, as in 0.1 (10, line 516). A 0.2 message to a 0.1-only peer is rejected with `C102` at stage 1 and gets no reply (8.2). So a sender sends 0.2 only to a peer configured for it. A goal with one alternative may be sent as 0.1; one with two or more alternatives cannot, and the exporter says so (exit 6) instead of dropping alternatives | (b) Version negotiation through `discover`/`advertise` (reserved, 4.2): a new protocol with no evidence that anyone needs it. (c) Silent downgrade to the first alternative: the 0027 F3 failure again, at the protocol level |
| C9 | Other deferred items | See section 3. In scope: C1, C2, C3, C5, C6, C7. Out of scope, with a reason each | Fold every collected item into 0.2: nothing asks for them, and each would need fixtures and a bundle |
| C10 | Freeze criteria | Section 4. Core 0.2 is frozen only when all hold | Freeze on owner approval of the text: it is how 0.1 got `C306` erratum churn (0008, 0011, 0012) |

## 3. Every deferred item or known 0.1 gap found

Found by `grep -rn` over `spec/`, `docs/`, `src/`, `tests/`, `bench/` (the search words were "Core 0.2", "nowhere",
"re-index", "any of", "disjunct", "deferred", "later version", "no Core form").

| Item | Where it is said | In 0.2? |
|---|---|---|
| "Any of" desired states | [0027](0027-bounded-branching.md) B7, lines 54 and 72–73; [`simulation-v1.0.md`](../simulation-v1.0.md) 12.4 line 838; `src/robolang/bridge.py` lines 121–128; `tests/test_sim14.py:416`; [set 2 analysis](../../docs/evidence/bench-set2-analysis.md) F2; [plan](../../docs/DEVELOPMENT_PLAN.md) N5 line 32 | **In (C1, C2, C6)** |
| `or` in a goal has no Core form (plan Q3: "extend Core only when a real case needs it") | [`goals-and-core-bridge.md`](../../docs/plans/goals-and-core-bridge.md) line 151; [`export-goal-v0.1.md`](../export-goal-v0.1.md) "Exportable goals" and `X601`; `bridge.py:12`. F2 and B7 are the real case | **In for `or` of conjunctions (C6).** `not`, `robot_ready`, `region_clear`, `object_exists`, `sensor_ok`: out, they describe the world before or during a run and not an outcome |
| A declared failure has no Core reason | [`simulation-v1.0.md`](../simulation-v1.0.md) 12.4 lines 833–834; [0027](0027-bounded-branching.md) section 5 "Reading of B4" | **In (C3)** |
| Absent thing, "no 'nowhere' value" | [0018](0018-core-shaped-outcomes.md) Consequences line 25; [0023](0023-keep-interop-3-deferred.md) reason 1 line 23; `src/robolang/sim/outcome.py:25` | **Out (C4)** |
| Outcomes renumbered when goal order differs (re-indexing) | [0023](0023-keep-interop-3-deferred.md) line 23; [0020](0020-delegate.md) D2 and its amendment; [`delegate-v0.1.md`](../delegate-v0.1.md) line 70; `tests/test_delegate.py:400` | **In (C5)**, as a Core rule |
| Reservation, `coordinate` and custody recovery "still unused in practice" | [0023](0023-keep-interop-3-deferred.md) line 23 | Out: a usage observation, not a spec gap. No change without a failing case |
| Discovery and version negotiation (`discover`, `advertise`) | [`core-v0.1.md`](../core-v0.1.md) 10 line 516, 4.2 | Out (C8). Reserved names stay reserved |
| Plans in messages (`plan_kind`, Task IR) | `core-v0.1.md` 10 line 517 (R3: plans stay local) | Out: the seam is "the goal crosses, the plan stays" ([0020](0020-delegate.md) section 1) |
| `negotiate`, `progress`, `cancel`, `renew`, `release`, `share_observation`, `observe`, `subscribe` | `core-v0.1.md` 10 and 4.2 (reserved) | Out: no benchmark or recorded request needs them |
| Places with geometry and frames; multi-party commitments; `ed25519` in the reference; key distribution and revocation; bindings and transports | `core-v0.1.md` 10 | Out. Keys and transport belong to H3 with INTEROP-3 |
| A `rad/s` limit has no Core form; a unitless limit is deprecated | [`export-goal-v0.1.md`](../export-goal-v0.1.md) section 2 "constraints" and X6; [`goals-and-core-bridge.md`](../../docs/plans/goals-and-core-bridge.md) line 22 | Out: a skipped limit is a warning today and nobody has asked for it. Add `max_angular_speed` if a task needs it |
| Optional explicit `core_id` per world entity (IDs are derived by prefix) | [`goals-and-core-bridge.md`](../../docs/plans/goals-and-core-bridge.md) Q5, line 153 | Out: bridge and world file, not Core |
| A branching plan that binds two or more robots (`robots_unsupported`) | [`simulation-v1.0.md`](../simulation-v1.0.md) 12.4 | Out: a `delegate` limit, not Core |
| INTEROP-3 and INTEROP-4 | [0023](0023-keep-interop-3-deferred.md) K1, K2; [STRATEGY](../../docs/STRATEGY.md) H3 row | Referenced only. Relaunch is H3, on the frozen Core 0.2 (section 4) |

## 4. Freeze criteria

Core 0.2 is frozen only when **all** of these exist:

1. **Reference implementation in this repo:** the 0.2 message schema file; `robolang.core` reading and checking 0.2
   messages (and still 0.1); `bridge` and `export-goal` exporting alternatives; the simulated outcome with
   `met_alternative` and `declared`; `delegate` matching alternatives and re-indexing.
2. **Conformance fixtures** in `tests/fixtures/core/`: valid 0.2 documents (a goal with one alternative, with several,
   an outcome with a met alternative, a `declared` failure); one invalid document for every new or changed code;
   a 0.1 document accepted by a 0.2 receiver; a 0.2 document answered `C102` by a receiver configured for 0.1 only;
   a reordered match that is re-indexed. The 0.1 fixtures, gate 1 and the frozen gate 2 bundle still pass unchanged.
3. **One benchmark task family** exports "any of" goals end to end: the set 3 branching tasks whose paths differ
   (such as `x_route_by_cell`) and the F2 tasks (such as `m_same_shelf`), through `export-goal` and `delegate`, with a
   verified outcome that names the met alternative.
4. **The 1.4 acceptance run** of set 3 is on file ([0027](0027-bounded-branching.md) section 3 item 2), because it
   decides whether branching, the producer of C1, stays as designed.
5. **Owner approval** of the text and of the fixtures.

INTEROP-3 relaunch on frozen Core 0.2 is H3 ([STRATEGY](../../docs/STRATEGY.md)) and is not planned here.

## 5. Needs the owner

C1 (the goal shape), C3 (`declared` and its 200-character limit), C4 (leave absence out), C6 (retire `X603`,
export `or`), C7 and C8 (version rules, no negotiation), and the freeze criteria in section 4.

## 6. Open questions

- **Q1.** Is 200 characters right for `text`? It fits a short operator message; `fail "reason"` has no limit in 1.4
  (`robolang-v1.4.md` 3.4, a non-empty string). 1.4 would need the same limit.
- **Q2.** Is 16 alternatives right? It is 2⁴, the largest number of paths 1.4 allows (0027 B5).
- **Q3.** When a 1.4 run takes path A and ends with only path B's goal true, the simulator says `failed`
  (0027 B7: the taken path counts), while "any of" would say met. The draft lets the verifier be the stricter one.
  The alternative is to relax the simulator to any-of. Recommended: keep the stricter reading.
- **Q4.** `declared` text can carry anything an AI wrote. A planner that feeds it to a model must treat it as
  untrusted. Is a Core rule enough, or should `delegate` also strip it from planner prompts?
