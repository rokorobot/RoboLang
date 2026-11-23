# RSI-01 weekly atlas update, 2026-11-23

Branch `atlas-weekly-2026-11-23`. Loops A1 (present scan) and A2 (horizon scan). Output: `bench/atlas.json` (41 activities, was 40; gaps unchanged), `docs/plans/humanoid-activities.md` (generated).

## Method

About 14 searches and 12 page fetches. e27, eWeek and ARC Advisory refused fetching (403), so they are not cited (LE-2). Search summaries were treated as leads only.

## A1: present scan

- **No change in maturity or horizon.** Searches found nothing dated after 2026-11-17 on Agility (GXO, AGCO, Toyota, Schaeffler: all already in the atlas), Boston Dynamics, Hexagon, UBTech or Apptronik.
- Boston Dynamics' Metaplant Application Center (Robot Report, 2026-09-21) says Atlas is in training there, not production, with component assembly planned by 2030. That is below `pilot` (LE-3), so no new activity.
- Figure and Catalyst Brands (figure.ai, 2026-05-26): an agreement for the Reno distribution centre; the page does not say robots are working yet. Nothing new for ACT-LOG-001.
- Hexagon and Schaeffler (robotics.hexagon.com, 2026-04-22): the fleet plan of 1,000 AEON by 2032 rests on a 2025 pilot; unchanged from the last run's review.
- ISO 25785-1: search shows a committee draft only. The ISO page was not opened, so ACT-SAFE-001 is unchanged.

## A2: horizon scan

- **New concept: ACT-CARE-002, answering a nurse call** (1-3y, p 0.4). donut robotics' humanoid has patrolled a Charm Care nursing home since July 2026; the companies plan to extend it to nurse calls, and Charm Care (over 100 homes) invests 300 million yen. Signals: roadmap, funding, demand (Japan's ministry: about 570,000 care workers short in fiscal 2040). Kept at `concept`: only patrol is shown, and no trial results are published. Needs G-TRIGGER.
- **Not added:** any research signal for nurse-call response. A search for an arXiv paper found none.

## Effect on ranking

- Directions: order unchanged. cooperation 1.07 -> 1.21, communication 0.95 -> 1.09; future need: cooperation 1.81 -> 2.01, communication 0.52 -> 0.72.
- Suggested order of language work: unchanged (G-CHOOSE, G-SELF, G-CALC, G-CONC). G-TRIGGER priority 0 -> 0.28, still not near.
- No gap or scope changed (LE-9).

## Weakest parts

1. ACT-CARE-002's roadmap and funding signals come from one trade article, so two of the three signals share a source.
2. Its `requests` (arrival, escalation, two calls at once) are my reading; the source says only "assist in responding".
3. Horizon 1-3y for a `concept` is unusual in the atlas; it follows from the trial already running. Probabilities are my estimates.

## Sources opened

- https://roboticsandautomationnews.com/2026/10/05/japanese-nursing-home-operator-charm-care-invests-in-humanoid-company-donut-robotics/105545/
- https://www.thestar.com.my/aseanplus/aseanplus-news/2024/12/17/japan-to-actively-recruit-nurses-from-south-east-asia
- https://www.therobotreport.com/boston-dynamics-opens-metaplant-application-center-train-atlas-humanoid-robots/
- https://figure.ai/news/figure-signs-agreement-with-catalyst-brands
- https://robotics.hexagon.com/hexagon-robotics-and-schaeffler-deploy-a-fleet-of-aeon-humanoids/
- https://techcrunch.com/2026/02/19/toyota-hires-seven-agility-humanoid-robots-for-canadian-factory/
