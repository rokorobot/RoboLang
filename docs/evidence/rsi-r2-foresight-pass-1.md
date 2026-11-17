# RSI-01 R2: foresight pass 1

Date: 2026-11-16. Branch `room/rsi-01-r2-foresight-pass-g46j`. Output: `bench/atlas.json` (25 activities, 3 new gaps).

## Method

- About 26 web searches and 36 page fetches (10 fetches failed with 403, 429 or timeout, so those pages are not cited). This is above the 30-50 budget: the owner added the eight directions mid-pass, and prediction, learning, safety and reasoning needed their own sources.
- Searches covered: humanoids in car plants and warehouses, home laundry, hospital and elder care, restaurants, inspection, retail, agriculture, construction, machine tending, multi-robot standards, humanoid safety standards, EV charging, cleaning, and arXiv cs.RO 2026 work on handover, assembly, human-motion prediction and world models.
- Every URL in `sources` was fetched and read. `basis` states only what the fetched page said. The fetch tool summarises pages with a small model, so numbers are as that summary reported them; a reviewer should spot-check the ones that matter.
- `needs` were judged against `docs/language-reference.md` (1.6: no loops, clocks, message waits, arithmetic, new actions, region or people conditions, run-time choice; fixed retries, `if`, handlers, `measure` are available). No model was run.
- Maturity is strict: a shipped product feature with no named customer site is `demo` (ACT-INSP-001, ACT-INSP-002, ACT-SEC-001); this probably understates them.
- "Main direction" of an activity is the first entry of its `directions`.

## Activities

| ID | Name | Dir. (main first) | Maturity | Horizon | Needs | p |
|---|---|---|---|---|---|---|
| ACT-LOG-001 | Humanoid tote handling in fulfilment centres | action | deployment | now | G-LOOP, G-CLOCK, G-CALC, G-CHOOSE | 0.60 |
| ACT-LOG-002 | Autonomous trailer and container unloading | action, perception | deployment | now | G-LOOP, G-PERCEIVE, G-CALC | 0.45 |
| ACT-MFG-001 | Parts sequencing for the assembly line | action, reasoning | pilot | 1-3y | G-LOOP, G-CHOOSE, G-MSG, G-CLOCK | 0.60 |
| ACT-MFG-002 | Humanoid-to-tugger handoff of totes in a car plant | cooperation, action | pilot | 1-3y | G-MSG, G-CONC, G-CHOOSE | 0.65 |
| ACT-RET-001 | Store shelf scanning and out-of-stock reporting | perception, communication | deployment | now | G-CLOCK, G-LOOP, G-PERCEIVE, G-CHOOSE | 0.55 |
| ACT-HOS-001 | Fry-station cooking in restaurant kitchens | action, perception | deployment | now | G-ACT, G-PERCEIVE | 0.45 |
| ACT-HLTH-001 | Hospital delivery and supply runs for nurses and pharmacy | action, communication | deployment | now | G-ACT, G-MSG, G-LOOP, G-CLOCK | 0.55 |
| ACT-ELD-001 | Companion robot in dementia and memory care | communication, perception | pilot | 3-7y | G-CLOCK, G-ACT, G-LOOP, G-PERCEIVE, G-MSG | 0.30 |
| ACT-HOME-001 | Laundry folding in the home | learning, action | research | 3-7y | G-ACT, G-LOOP, G-CHOOSE, G-PERCEIVE, G-CALC | 0.30 |
| ACT-HOME-002 | General household chores by a home humanoid | action, reasoning | demo | 3-7y | G-CLOCK, G-ACT, G-LOOP, G-CHOOSE, G-PATH, G-PERCEIVE | 0.40 |
| ACT-INSP-001 | Autonomous gauge and thermal inspection rounds | perception | demo | now | G-PERCEIVE, G-LOOP, G-CLOCK | 0.80 |
| ACT-INSP-002 | Inspection that learns the facility | learning, perception | demo | 1-3y | G-TEACH, G-PERCEIVE, G-CALC | 0.55 |
| ACT-SEC-001 | Security patrol that reacts to people | safety, perception | demo | 1-3y | G-LOOP, G-CLOCK, G-PERCEIVE, G-PROX, G-PATH | 0.60 |
| ACT-SAFE-001 | Fence-free humanoid work beside people | safety, cooperation | demo | 3-7y | G-PROX, G-PATH, G-FORCE | 0.65 |
| ACT-CON-001 | Construction-site layout printing | action, perception | deployment | now | G-PATH, G-LOOP, G-PERCEIVE, G-ACT | 0.45 |
| ACT-AGR-001 | Greenhouse fruit harvesting | perception, action | pilot | 3-7y | G-CHOOSE, G-PERCEIVE, G-LOOP, G-CLOCK, G-FORCE | 0.60 |
| ACT-EV-001 | Robotic EV charging in garages and depots | action | pilot | 1-3y | G-ACT, G-MSG, G-CHOOSE, G-LOOP | 0.55 |
| ACT-CLN-001 | Scheduled autonomous floor cleaning | action, learning | deployment | now | G-CLOCK, G-LOOP, G-ACT, G-PATH | 0.55 |
| ACT-MRC-001 | Mixed robot teams on one workflow | cooperation, reasoning | demo | 3-7y | G-CONC, G-CHOOSE, G-MSG | 0.35 |
| ACT-MRC-002 | Multi-vendor fleet interoperability | communication, cooperation | demo | 1-3y | G-MSG, G-CHOOSE, G-CONC | 0.55 |
| ACT-HRC-001 | Task-oriented object handover to a person | cooperation, communication | research | 3-7y | G-ACT, G-PERCEIVE, G-MSG, G-CHOOSE | 0.35 |
| ACT-HRC-002 | Following ambiguous and corrective instructions during shared work | reasoning, cooperation | research | 3-7y | G-MSG, G-CHOOSE, G-PERCEIVE | 0.40 |
| ACT-ASM-001 | Robot-team structural assembly with symbolic replanning | reasoning, cooperation | research | 7y+ | G-LOOP, G-PERCEIVE, G-CONC | 0.25 |
| ACT-PRED-001 | Anticipating a coworker's next movement | prediction, cooperation | research | 3-7y | G-PREDICT, G-PROX | 0.40 |
| ACT-PRED-002 | Checking a plan with a world model before acting | prediction, learning | research | 3-7y | G-PREDICT, G-CHOOSE | 0.45 |

Maturity mix: 7 deployment, 5 pilot, 7 demo, 6 research. Each of the eight directions is the main one of at least two activities (checked by a test).

## Gap ranking (plan section 5)

Score = sum over activities needing the gap of p x nearness (now 1, 1-3y 0.7, 3-7y 0.4, 7y+ 0.1). Safety relevance and grammar cost are not applied yet, so this is a first cut. "Near" = needed by a `pilot` or `deployment` activity with horizon now or 1-3y (count in brackets).

| Rank | Gap | Direction | Scope | Score | Activities | Near |
|---|---|---|---|---|---|---|
| 1 | G-LOOP | reasoning | out | 5.84 | 15 | **yes** (8) |
| 2 | G-PERCEIVE | perception | open | 4.47 | 14 | **yes** (4) |
| 3 | G-CLOCK | reasoning | open | 4.41 | 10 | **yes** (5) |
| 4 | G-CHOOSE | perception | open | 3.94 | 13 | **yes** (5) |
| 5 | G-ACT | action | open | 2.93 | 9 | **yes** (5) |
| 6 | G-MSG | communication | open | 2.76 | 9 | **yes** (4) |
| 7 | G-PATH | safety | open | 1.84 | 5 | **yes** (2) |
| 8 | G-CALC | reasoning | open | 1.55 | 4 | **yes** (2) |
| 9 | G-CONC | cooperation | open | 1.00 | 4 | **yes** (1) |
| 10 | G-PROX | safety | open | 0.84 | 3 | no |
| 11 | G-FORCE | safety | open | 0.50 | 2 | no |
| 12 | G-TEACH | learning | open | 0.39 | 1 | no |
| 13 | G-PREDICT | prediction | open | 0.34 | 2 | no |
| 14 | G-FALLBACK | reasoning | out | 0.00 | 0 | no |

Reading it:

- **Near gaps (9):** G-LOOP, G-PERCEIVE, G-CLOCK, G-CHOOSE, G-ACT, G-MSG, G-PATH, G-CALC, G-CONC. Together they need ten deployed or piloted activities, so they are eligible for a language proposal under plan section 5.
- **G-LOOP ranks first and is `out` of scope** (N4, 2026-10-29). Eight near activities need it (tote handling, unloading, shelf scanning, floor cleaning, hospital rounds, and others). This is the case the plan names: the loop may propose revisiting it, never decide it.
- **Far (no near activity):** G-FORCE, G-PROX, G-TEACH, G-PREDICT, and G-FALLBACK. G-FALLBACK is needed by no activity here; its only evidence is the one benchmark task. No activity request needed a different end state to count as done.
- G-CHOOSE (D4 deferred) is needed by 13 activities and is near through five of them.

## New gaps (scope `open`, no tasks yet)

- **G-PROX** (safety): limits and reactions that depend on people nearby (slow, stop or yield within a distance). Needed by ACT-SAFE-001, ACT-SEC-001, ACT-PRED-001. Not covered by G-PATH (regions) or `@safety max_speed` (fixed).
- **G-PREDICT** (prediction): acting on a predicted future (a person's next move, the outcome of a plan). Needed by ACT-PRED-001, ACT-PRED-002. Plan section 9 keeps probabilistic prediction out of `.rob` for now; this gap is recorded so it can be ranked, and it is far.
- **G-TEACH** (learning): teaching the robot a new skill, object or route by demonstration, then using it by name. Needed by ACT-INSP-002.

No existing gap was changed except for adding `direction`, with the owner's suggested values unchanged.

## By direction

| Direction | Main for (activities) | Also exercised by | Gaps | Near gaps |
|---|---|---|---|---|
| perception | ACT-RET-001, ACT-INSP-001, ACT-AGR-001 (3) | ACT-LOG-002, ACT-HOS-001, ACT-ELD-001, ACT-INSP-002, ACT-SEC-001, ACT-CON-001 | G-CHOOSE, G-PERCEIVE | G-CHOOSE, G-PERCEIVE |
| reasoning | ACT-HRC-002, ACT-ASM-001 (2) | ACT-MFG-001, ACT-HOME-002, ACT-MRC-001 | G-LOOP, G-CLOCK, G-CALC, G-FALLBACK | G-LOOP, G-CLOCK, G-CALC |
| prediction | ACT-PRED-001, ACT-PRED-002 (2) | - | G-PREDICT | none |
| action | ACT-LOG-001, ACT-LOG-002, ACT-MFG-001, ACT-HOS-001, ACT-HLTH-001, ACT-HOME-002, ACT-CON-001, ACT-EV-001, ACT-CLN-001 (9) | ACT-MFG-002, ACT-HOME-001, ACT-AGR-001 | G-ACT | G-ACT |
| learning | ACT-HOME-001, ACT-INSP-002 (2) | ACT-CLN-001, ACT-PRED-002 | G-TEACH | none |
| cooperation | ACT-MFG-002, ACT-MRC-001, ACT-HRC-001 (3) | ACT-SAFE-001, ACT-MRC-002, ACT-HRC-002, ACT-ASM-001, ACT-PRED-001 | G-CONC | G-CONC |
| safety | ACT-SEC-001, ACT-SAFE-001 (2) | - | G-PATH, G-FORCE, G-PROX | G-PATH |
| communication | ACT-ELD-001, ACT-MRC-002 (2) | ACT-RET-001, ACT-HLTH-001, ACT-HRC-001 | G-MSG | G-MSG |

Prediction and learning are thin as expected: every prediction activity is `research`, and the learning ones are one `research` and one `demo`. Their gaps (G-PREDICT, G-TEACH) have no near activity.

## Weakest parts of this pass

1. **Domains with no activity.** Machine tending: the one search found only vendor "intended use" lists and the Apptronik article I opened did not mention it, so I dropped it. Weeding (Carbon Robotics, 403) and robotic solar installation (Maximo, timeout) were found but not opened, so they are not in the atlas.
2. **Thin maturity evidence.** ACT-INSP-001/002 and ACT-SEC-001 rest on one vendor release and one vendor blog, with no customer named. ACT-EV-001 rests on one Electrek article ("deployed in multiple Chinese cities", no site named). ACT-MRC-002 is a published standard; I opened no source on live multi-vendor sites.
3. **Vendor numbers.** Dusty (300M sq ft), Brain Corp (40,000+ robots), Simbe (3,000 units under contract, not only Tally) and Stretch hours are company claims. The BMW Figure 02 figures are Figure's own.
4. **ACT-HOME-001/002.** Physical Intelligence's own blog (Weave customer-site folding) returned 429, so the laundry activity rests on the arXiv paper alone (`research`). For 1X NEO, sources disagree on whether any customer delivery has happened; I only opened the May 2026 heise article, so `demo`.
5. **ACT-ELD-001** is a companion robot (conversation, music), not physical care. I found no opened source for robots doing physical elder care.
6. **Standards status.** ISO 25785-1 status comes from one explainer whose status section says "as of early 2026"; the ISO page itself returned 403. The forecast p of 0.65 reflects that.
7. **Prediction and learning** are research-only. The prediction paper is a preliminary simulation evaluation, and the ambiguity paper does not say the robot asks questions, so ACT-HRC-002's third request is a guess at the direction of travel.
8. **`needs` and `p` are my judgement.** Neither has been checked by a second reader. Forecast p values are not calibrated; most sit between 0.3 and 0.65 on purpose.
