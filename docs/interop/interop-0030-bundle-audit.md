# Gate 2 bundle audit for the relaunch on Core 0.2 (0030 J4)

**Date:** 2026-11-02. **Decision:** [0030](../../spec/decisions/0030-relaunch-interop-3-on-core-0-2.md) J4.
**Audited:** the candidate bundle built by `tools/interop_bundle.py` after [#94](https://github.com/rokorobot/RoboLang/pull/94),
for what the files added by J2 change for amr-07. The rest was audited for the first bundle
([INTEROP-2g audit](interop-2g-bundle-audit.md)) and is unchanged.

## Question

Can an independent author build a Core 0.2 receiver that plays amr-07 from the bundle alone? Specifically, does any
rule amr-07 needs live only in a file the new documents link to but the bundle does not contain?

## Method

1. Read `spec/core-v0.2.md` in full and decision 0028's decisions, against what amr-07 sends and receives in the
   reference simulation (section 6.1, happy-path message table): it receives `request_assistance` (with a sub-goal), `commit`,
   `reservation_decision`, `coordinate`, `transfer_custody` and `custody_released`. It sends, but never receives,
   an `outcome`.
2. Followed every link from the two new documents that leaves the bundle (12 targets).
3. Checked the version configuration of the reference participants (`robolang.core` `Context.versions`, default
   `{"0.1"}`; neither `arch02` nor `interop` passes another).
4. Ran both conformance kits and the schemas from the built bundle (`tests/test_interop_bundle.py`).

## Findings

| # | Finding | Kind | Fix |
|---|---|---|---|
| A1 | The bundle did not say that the reference participants accept **Core 0.1 only**. An amr-07 that sent `0.2` would be rejected with `C102` and get no reply, and nothing in the bundle would tell its author why | Specification gap | Gate 2 6.4 erratum: reference participants are 0.1-only; amr-07 sends every message as 0.1; its own receive configuration may be 0.1 or 0.1 and 0.2, with the same result on 0.1 messages; a sub-goal reads as one alternative |
| A2 | Gate 2 section 7 gave the 0.2 kit its version configuration but not the 0.1 kit. Five 0.1 fixtures use `0.2` as the unsupported version and expect `C102`, which holds only for a 0.1-only receiver (Core 0.2 section 1 says so, but the gate did not) | Specification gap | Gate 2 7 erratum: the 0.1 kit runs with a 0.1-only receiver |
| A3 | Gate 2's "Builds on" did not name Core 0.2 | Editorial | Added |
| A4 | The bundle README's list of informative citations did not cover strategy, plans, export-goal and delegate | Editorial | README text widened |
| A5 | Only the 0.1 schema was checked against valid fixtures from the bundle | Test gap | The bundle test validates the 0.2 kit's valid messages against the bundled 0.2 schema too |
| A6 | Core 0.2 section 3 points to `delegate-core-0.2.md` (not bundled) for how the **holder of a goal** checks a `succeeded` claim | Informative, no fix | amr-07 never receives an `outcome` in the scenario, so it never holds a goal it re-evaluates. Stateless checks (`C402`) are fully stated in Core 0.2 |
| A7 | The other 11 outside links (0018, 0020, 0023, 0027, 0030, STRATEGY, the set 2 and 3 evidence, the bridge plan, `delegate-v0.1`, `export-goal-v0.1`) give rationale or describe the `.rob` export and `delegate` tools | Informative, no fix | No receiver or amr-07 rule is stated only there. Core 0.2 section 7 (the bridge) is marked non-normative |

## Result

After the A1 to A5 fixes, every rule amr-07 needs is in the bundle. Both kits give their stated receipts from bundled
files alone (47 Core 0.1 fixtures with a 0.1-only receiver; 62 Core 0.2 entries with each entry's configuration),
and all three schemas resolve without external references.

The bundle is ready for J6 step 3: build twice from one clean commit, then the owner approves the freeze.
