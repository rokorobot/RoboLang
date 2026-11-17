# RSI-01 A2: horizon scan 1

Date: 2026-11-17. Branch `room/rsi-01-a2-horizon-scan-upk7`. Output: `bench/atlas.json` (38 activities, was 28; 15 gaps, unchanged), `docs/plans/humanoid-activities.md` (generated).

## Method

- 25 web searches and about 36 page fetches, about 60 calls in all; 11 fetches failed (403, 404, 429, timeout) or returned the wrong page, and those pages are not cited.
- Searches: ISO 25785-1 status; robot foundation models (pi0.7, Gemini Robotics 2); labour data (Japan elder care, US construction, specialty agriculture); analyst forecasts (Goldman, Morgan Stanley); funding (Physical Intelligence, 1X); R2's holes (machine tending, weeding, solar); and arXiv cs.RO 2026 on VLA failure detection, VLA safety filters, learning from human video, proactive assistance, patient transfer, fleet battery handover, and fall recovery.
- Every URL in a new `sources` or `leading` entry was fetched this pass, except six pages that pass 1 opened (listed under "Weakest parts", item 3). Claims state only what the fetched page said.
- Methods named in each `basis`: *combination*, *extrapolation*, *demand pull*. `needs` were judged against `docs/language-reference.md` (1.6). No model was run.
- The search tool's summaries are unreliable: one returned the title "ReconVLA" for arXiv 2604.16677, which is SentryVLA. I used only what the fetched page said.

## New concept activities (7)

| ID | Method | Directions | Horizon | p | Leading signals (kind) |
|---|---|---|---|---|---|
| ACT-PRED-003 Forecasting its own failure and asking before it happens | combination | prediction, reasoning | 3-7y | 0.35 | research (SentryVLA arXiv 2604.16677), roadmap (Gemini Robotics ER 2) |
| ACT-PRED-004 Helping before being asked | combination + demand pull | prediction, cooperation | 7y+ | 0.15 | research (arXiv 2603.23950), demand (Japan caregiver gap, e27) |
| ACT-LRN-001 Learning a task from half an hour of a worker's video | combination | learning, action | 3-7y | 0.35 | research (HumanEgo 2605.24934), funding (Physical Intelligence talks, Dealroom) |
| ACT-LRN-002 One taught skill, any robot on the site | extrapolation | learning, cooperation | 3-7y | 0.30 | roadmap (Gemini On-Device 2), research (HumanEgo) |
| ACT-AUTO-004 Mid-shift relief: low-battery robot hands its job to a peer | combination | autonomy, cooperation | 3-7y | 0.30 | research (arXiv 2603.22731), roadmap (UBTech Walker S2 swap), standard (ISO 21423:2026) |
| ACT-SAFE-002 Safety layer under a learned policy that vetoes unsafe moves | combination | safety, action | 3-7y | 0.30 | research (arXiv 2606.09749), standard (DIN EN ISO 25785-1 project), roadmap (Agility Digit v5) |
| ACT-CARE-001 Physical help with standing and transfers in care homes | demand pull + combination | safety, action, cooperation | 7y+ | 0.12 | demand (e27), roadmap (Enactic Ena, e27), research (Moby arXiv 2508.19816) |

Directions: prediction 2 (main), learning 2, autonomy 1, safety 2 (main 2). Every one has two or three signals of different kinds and a forecast due 2029-12-31 or 2030-12-31.

## R2's holes (3, not concept)

- **ACT-MFG-003 machine tending** (pilot, 1-3y, p 0.45): Hexagon AEON at a Fill client site in Gurten (Robotics and Automation News, 2026-05-15). No metrics, no named customer.
- **ACT-AGR-002 laser weeding** (deployment, now, p 0.40): Carbon Robotics, over 100 M USD revenue to March 2026, about 20 countries (AgFunderNews, 2026-08-31). No machine count on the page.
- **ACT-SOL-001 solar installation** (deployment, now, p 0.50): AES Maximo, 100 MW at Bellefield, up to one module a minute (eWeek, 2026-03-31).

## Leading signals added to far activities (12 far activities in the atlas at the start; all now have at least one)

| Activity | Signals |
|---|---|
| ACT-ELD-001 | demand (Japan 690,000 caregiver gap), roadmap (ugo, Enactic) |
| ACT-HOME-001 | research (pi0.7 zero-shot laundry on another robot), funding (Physical Intelligence talks) |
| ACT-HOME-002 | research (pi0.7 multi-stage kitchen tasks), forecast (Goldman Sept 2026: names no household use case, so a weak signal) |
| ACT-SAFE-001 | standard (DIN EN ISO 25785-1 project), roadmap (Digit v5) |
| ACT-AGR-001 | roadmap (Tokuiten 1 ha March 2027), funding (Carbon Robotics) |
| ACT-MRC-001 | roadmap (Gemini Robotics ER 2), standard (ISO 21423:2026) |
| ACT-HRC-001, ACT-HRC-002, ACT-PRED-001, ACT-PRED-002 | research only (their own arXiv source) |
| ACT-ASM-001 | research (own source), demand (ABC 349,000 workers 2026, 456,000 in 2027) |
| ACT-AUTO-003 | already had two (unchanged) |

18 signals on 11 existing activities, 17 on the new ones: 35 in all.

## Changes to existing activities

None. No source I opened changed a maturity or horizon. (pi0.7 is a stronger research result for ACT-HOME-001, but it stays `research`: no customer use.)

## New gaps

None. Each new concept maps onto an existing gap: G-PREDICT (failure forecast, proactive help), G-TEACH (video and fleet skills), G-SELF and G-MSG (relief), G-PROX, G-FORCE, G-PATH (safety layer, care), G-CHOOSE (which robot can do X). Judged against the 1.6 reference; no capability was left over.

## Direction table (before -> after)

| Direction | Priority | Language need | Future need |
|---|---|---|---|
| action | 3.43 -> 4.17 | 3.96 -> 4.30 | 0.65 -> 1.01 |
| perception | 2.33 -> 2.78 | 9.06 -> 10.76 | 0.45 -> 0.45 |
| cooperation | 0.86 -> 0.99 | 1.00 -> 1.00 | 1.20 -> 1.61 |
| communication | 0.87 -> 0.87 | 2.76 -> 3.86 | 0.32 -> 0.32 |
| learning | 0.62 -> 0.75 | 0.39 -> 0.65 | 0.38 -> 0.70 |
| reasoning | 0.45 -> 0.68 | 5.96 -> 6.28 | 0.70 -> 0.88 |
| autonomy | 0.61 -> 0.67 | 1.22 -> 1.33 | 0.23 -> 0.38 |
| safety | 0.43 -> 0.49 | 3.18 -> 4.46 | 0.55 -> 0.74 |
| prediction | 0.17 -> 0.25 | 0.34 -> 0.50 | 0.43 -> 0.68 |

Ranking moves: cooperation +1, reasoning +1, communication -1, autonomy -1. Prediction stays last on priority but its future need rose by 58%, learning's by 84%. Most of the *priority* rise comes from the three deployed hole-fillers (action, perception), not from the concepts: far activities add little to priority because nearness is 0.1-0.4. Future need is the column that carries this scan. Language need rose where the new activities need open gaps (communication +1.10 through G-MSG, safety +1.28 through G-PROX, G-FORCE, G-PATH).

## Weakest parts

1. **Every `p` and horizon is my judgement**, uncalibrated; concepts sit at 0.12-0.35 on purpose. Strictest calls: ACT-PRED-003 may already be `research` (several 2026 papers; none ships the "ask before it happens" behaviour), and ACT-AUTO-004's `standard` signal is a stretch (ISO 21423 shares tasking and availability; the article mentions neither handover nor battery).
2. **Preprints, mostly simulation.** ACT-SAFE-002's filter paper is simulation only; HumanEgo is four lab tasks; SentryVLA's page gives no numbers; ACT-PRED-004's paper is one tabletop task.
3. **Four far activities carry one signal, their own pass-1 source, and six URLs were not re-opened this pass:** arXiv 2602.05760, 2602.14551, 2601.00978, 2604.03065, 2606.13672, and the Tokuiten FreshPlaza page. Their claims repeat pass 1's `basis`.
4. **Demand signals are thin:** one e27 article carries three activities' demand or roadmap claim. The agriculture labour survey (Fruit Growers News) returned 403, so ACT-AGR-001 has no labour-data signal. No labour data on the care step beyond Japan.
5. **New deployment entries rest on one article each**, vendor figures (Maximo "100 MW", Carbon "$100 M"), with no count of robots or machines; `deployment` may overstate ACT-SOL-001 and ACT-AGR-002.
6. **ISO 25785-1 status** is still unconfirmed: the DIN page lists a start date and no stage; i-scoop failed to load. Reports disagree on committee-draft vs working-draft; publication by 2028 stays as in pass 1.
7. **Missing sources:** pi.website/pi07 (429), Hexagon's own release (403), Maximo's release (403), Meta's data-centre robot pilot (403, so data-centre technician work is not in the atlas); lights-out factories were only vendor marketing and are not in the atlas.
8. **Goldman signal for ACT-HOME-002** is a counter-signal (no household use named); kept because it is the only forecast I opened.
