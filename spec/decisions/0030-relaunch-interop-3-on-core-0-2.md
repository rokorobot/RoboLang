# Decision record 0030: Relaunch INTEROP-3 on Core 0.2 (H3 gate 2)

**Date:** 2026-11-01
**Status:** **PROPOSED**, for owner review. Nothing is built, frozen or launched by this record.
**Amends:** [0023](0023-keep-interop-3-deferred.md) K1 (INTEROP-3 and INTEROP-4 deferred) and the gate 2 bundle
contents of [0009](0009-gate-2-independent-interoperability.md) I3 and I12. All other 0009 rows stand.
**Evidence and direction:** [STRATEGY](../../docs/STRATEGY.md) H3, second exit criterion: "Independent Core 0.2
implementation interoperates (gate 2)". The first criterion was met on 2026-11-01. 0023 K2 now holds, since Core 0.2
was frozen on 2026-10-30 ([0028](0028-core-0-2-scope.md)).

## 1. Facts this record rests on

- The frozen gate 2 bundle `e6c9bb75…d104` ([freeze record](../../docs/interop/gate2-bundle-freeze.md)) is Core 0.1
  only. `tools/interop_bundle.py` excludes `message-0.2.schema.json` on purpose (line 43).
- The reference simulation sends only Core 0.1 messages (`arch02/engine.py` line 96). Its scenario has no goal with
  more than one alternative and no `declared` failure.
- Core 0.2 has its own conformance kit: 62 fixtures in `tests/fixtures/core_v02/`. A 0.2 receiver must accept
  0.1 messages and read them as one-alternative goals ([0028](0028-core-0-2-scope.md) C7).
- The [INTEROP-3 protocol](../../docs/interop/interop-3-protocol.md) is approved for preparation. Its launch needs a
  verified model ID, a signed operator attestation, and owner authorization. None exists yet.

## 2. Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| J1 | What "on Core 0.2" means | **The independent amr-07 implements a Core 0.2 receiver.** It accepts 0.1 and 0.2 messages and passes both conformance kits (G-T0: 47 + 62 fixtures). The gate 2 scenario stays unchanged: Core 0.1 messages on the wire, which a 0.2 receiver must accept (C7). Reference simulation, golden records and the binding do not change | (b) Move the reference simulation to Core 0.2, with goals that have alternatives: this needs a new reference-simulation version, new golden records and a repeat of INTEROP-1/2, and nothing in the scenario asks for "any of" (evidence before syntax, STRATEGY 4.1). (c) Run on the frozen 0.1 bundle unchanged: cheapest, but it would show an independent reading of a Core that is no longer current |
| J2 | Bundle | A **new bundle**: the current file list, plus `spec/core-v0.2.md`, [0028](0028-core-0-2-scope.md), `spec/core/message-0.2.schema.json` and `tests/fixtures/core_v02/`, plus the gate 2 errata of J3. It is built by `tools/interop_bundle.py` at one new source commit, with a new freeze record. The old bundle is marked **superseded**, never delivered, and kept for history | Ship two bundles: two IDs for one run, against gate 2 4.2 (one bundle per run) |
| J3 | Gate 2 text | Errata to `interop-gate-2-v0.1.md`, in the J2 bundle: section 1 reads Core 0.2 where it names the Core implemented; section 7 G-T0 covers both kits; the bundle list of section 4 adds the J2 files. No change to the binding, the scenario cases or the success criteria (I6, I7) | A new gate 2 version: no rule changes beyond those three, so a version adds churn |
| J4 | Pre-freeze review | One completeness audit of the added files against what amr-07 must do, as INTEROP-2g did ([audit](../../docs/interop/interop-2g-bundle-audit.md)). Its findings are fixed before the freeze, not by Q&A during the run | Freeze on approval of this record: every finding then becomes an erratum during the run |
| J5 | Protocol | The INTEROP-3 protocol stays, with the frozen inputs replaced by the J2 bundle. In the verbatim prompt, "Core 0.1" becomes "Core 0.2 (which also accepts Core 0.1)" and "all 47 supplied fixtures" becomes "all supplied fixtures in both conformance kits". **Changing the prompt needs owner approval of the new text** | Leave the prompt as approved: it would tell the author to implement 0.1 |
| J6 | Order | (1) bundle tool and gate 2 errata (PR) → (2) J4 audit and fixes (PR) → (3) build twice, freeze, freeze record (owner approves) → (4) protocol update (owner approves the prompt) → (5) INTEROP-3 launch (operator attestation, owner authorizes) → (6) INTEROP-4 gate 2 run and evidence (owner sign-off). Each step is separately authorized, as in 0009 I10 | Launch first and fix the bundle by errata: a defect found mid-run costs a re-freeze and both sides retested |

## 3. What only the owner can do

- Approve J1–J6, then each step of J6 when it is reached.
- Approve the J3 errata text, the freeze, and the new prompt wording.
- Name the independent author: the protocol prefers an OpenAI GPT-6-family API session. The exact model ID must be
  verified outside the RoboLang sessions.
- Have an environment operator, who is not this session, provision the isolated environment and sign the
  attestation (protocol section 5). Nobody who worked on the reference may act as the author or pass it knowledge.

## 4. Not changed

Core 0.2 itself, the reference simulation, `robolang-sim-stdio` 0.1, the gate 2 success criteria, and the run-record
schema. Physical execution stays unauthorized.
