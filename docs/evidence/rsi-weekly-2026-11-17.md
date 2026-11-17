# RSI-01 weekly atlas update, 2026-11-17

Branch `atlas-weekly-2026-11-17`. Loops A1 (present scan) and A2 (horizon scan). Output: `bench/atlas.json` (40 activities, was 38; gaps unchanged), `docs/plans/humanoid-activities.md` (generated).

## Method

About 20 searches and 15 page fetches. Several sites refused fetching (iso.org, robotics247, interestingengineering, gcn.com, grabarobot; arXiv only through curl and alphaxiv). Pages that did not open are not cited (LE-2). Search summaries were treated as leads, never as facts.

## A1: present scan

- **New: ACT-MFG-004, humanoid assembly and component work in a car plant** (`demo`, 1-3y, p 0.5). Hexagon says AEON has begun production tasks at BMW Plant Leipzig (June 2026); BMW announced a two-robot pilot for battery assembly and exterior parts in March 2026. Kept at `demo`: no named task beyond a laser scan, no metrics, assembly not yet on the plant floor (LE-3). Needs G-FORCE, G-PERCEIVE.
- **No change in maturity or horizon.** Checked and unchanged: Agility (GXO adds two sites in early 2027; three unnamed paid manufacturing programmes; AGCO pilot from 2027, all dated 2026-11-02), Apptronik (Mercedes work is still "exploring", no sites or counts), Figure (no new primary source opened), UBTech (no named factory shift with battery swap found), Eskaton memory-care pilot (already in ACT-ELD-001). The ACT-MFG-002 forecast (a third automaker, named) is not yet met: Agility's automaker is unnamed.
- **ISO 25785-1**: the CEN listing (genorma) shows prEN ISO 25785-1 at stage 10.99, new project approved 2026-01-09. The ISO page itself did not open, so ACT-SAFE-001 is unchanged.

## A2: horizon scan

- **New concept: ACT-HRC-003, handing a stuck job to a remote operator and taking it back** (3-7y, p 0.4). Signals: roadmap (1X NEO Expert Mode, Foundation's remote-human model), research (arXiv 2605.00416, fleet learning from interventions). No source shows the robot asking and resuming itself. Needs G-SELF, G-MSG.
- **Not added:** an "auditable decision record" concept for EU Machinery Regulation 2023/1230 (applies 2027-01-20). The page I opened (Nemko) names the AI high-risk categories but says nothing on logging, so only one signal kind held up.

## Effect on ranking

- Directions: order unchanged. action 4.17 -> 4.52, cooperation 0.99 -> 1.07, communication 0.87 -> 0.95; future need: cooperation 1.61 -> 1.81, communication 0.32 -> 0.52.
- Suggested order of language work: unchanged (same ten gaps, same order). Priorities moved: G-FORCE 0.63 -> 0.98, G-PERCEIVE 4.59 -> 4.94, G-MSG 3.86 -> 4.02, G-SELF 1.33 -> 1.50.
- No gap or scope changed (LE-9).

## Weakest parts

1. ACT-MFG-004 rests on a vendor page and one trade article; BMW's own statement was seen only through that article.
2. ACT-HRC-003's roadmap signals are a consumer-robot article (2025) and a secondary summary of a 2025 interview; both are old. Its `needs` are my judgement and no benchmark probe tests them.
3. Forecast probabilities are my estimates, uncalibrated.

## Sources opened

- https://robotics.hexagon.com/?p=720
- https://www.artificialintelligence-news.com/news/bmw-humanoid-robots-manufacturing-europe-leipzig/
- https://roadtovr.com/helper-robot-neo-vr-telepresence/
- https://sacra.com/chat/h/93dc70de-7775-4bc2-b477-ce4cd741da03/
- https://arxiv.org/abs/2605.00416 (and https://www.alphaxiv.org/abs/2605.00416)
- https://roboticsandautomationnews.com/2026/11/02/agility-robotics-says-more-than-30-companies-considering-digit-humanoid-robot/106595/
- https://startupfortune.com/apptroniks-apollo-robot-has-left-the-lab-and-is-now-working-factory-shifts-at-mercedes-benz/
- https://genorma.com/en/standards/pren-iso-25785-1
- https://digital.nemko.com/regulations/eu-machinery-regulation
- https://discover.eskaton.org/blog/eskaton-introduces-companion-robot-pilot-at-eskaton-village-carmichael-in-partnership-with-uc-davis-school-of-nursing/
