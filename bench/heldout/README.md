# Held-out set

A set of plain-English requests written by people outside the project, kept private, and run only at milestones. It
measures how well models write RoboLang for requests that no prompt, reference or model was tuned on. It is the
"preregistered held-out task set" of RL-20D and MC7 ([development plan](../../docs/DEVELOPMENT_PLAN.md),
[charter](../../spec/architecture/ai-first-language-charter.md)): at least 50 tasks, with the target approved before the
first run.

> **AI agents working in this repo:** do not open, search, summarize or copy the private set, wherever it is. Do not
> ask for it. If you find it, stop and tell the owner.

## What is here and what is not

| Path | In git | What |
|---|---|---|
| `README.md` | yes | these rules |
| `BRIEF.md` | yes | the brief for the people who write the requests |
| `template/` | yes | the starting copy of the private repository: its README (every file format), worked examples, `PREREGISTRATION.md` |
| `manifest.sha256` | yes, once frozen | SHA-256 of the frozen private files: proves what was frozen, and when, without showing it |
| `PREREGISTRATION.md` | yes, once frozen | the targets, the models, the prompt and the date, fixed before the first run |
| `private/` | **never** (gitignored) | a safety net only. Keep the real copy **outside the repo** |

The repo is public. Anything committed, pasted into an issue or PR, or shown to an AI coding session is no longer held
out. Keep the set in its own private repository (for example `RoboLang-heldout`), cloned only on the owner's computer,
with one backup. Never add it to an AI coding session, a cloud agent workspace or a public CI job: their logs and
artifacts are visible, and agents can read everything in their workspace. Give access only to the owner and the
person who checks the labels. Turn off AI features (code review, assistants) on that repository.

Start it from [`template/`](template/README.md): `cp -r bench/heldout/template RoboLang-heldout`. Layout:

```
RoboLang-heldout/
├── README.md                 who has access; a link to these rules
├── scenes/scene-a.md         the plain-English scene sent to the writers
├── worlds/scene-a.json       world files, same format as bench/worlds/; new worlds, not copies
├── raw/w1.xlsx               the writers' files exactly as received, never edited
├── raw/dropped.md            requests dropped while labelling, and why
├── tasks/h001.json           as in bench/tasks_ask/, plus "source": "human" and "writer": "w1"
├── reference/h001.rob        + h001.json: a hand-written solution and its bindings (feasible tasks)
├── reference/h002.attempt.rob   the obvious attempt that fails (infeasible or inexpressible tasks)
├── synthetic/tasks/          optional model-written requests, scored separately (see below)
├── synthetic/reference/
├── results/2027-01-v2.0/     raw answers; they contain the requests, so they never leave this repository
├── reports/2027-01-v2.0.md   totals only: the one thing copied to the public repo
├── manifest.sha256           the freeze: fingerprints of tasks/, reference/ and worlds/
└── PREREGISTRATION.md        the draft; the approved copy is committed in RoboLang's bench/heldout/
```

- Use neutral task IDs (`h001`, `h002`, ...): file names appear in the public manifest.
- Writers are `w1`, `w2`, ... in every file. Keep names and contact details out of the repository.
- Publish only totals and failure categories, never request text or raw answers.

## Steps

1. **Worlds.** Build 3 or 4 new worlds that are not in `bench/worlds/`. For each, write a plain-English scene
   description in the format of `BRIEF.md`.
2. **Writers.** Recruit 3 to 5 people who have not worked on RoboLang. Operators, technicians and people who have
   worked with robots or warehouses are best. Send each person `BRIEF.md` with the scenes filled in, privately.
3. **Collect.** Aim for 80 to 120 raw requests, so that 50 or more survive labelling. Save the files in `raw/`
   exactly as received.
4. **Label.** For each request, decide what the writer meant, using their "what done looks like" note, and record
   the expected decision (`program`, `cannot`, `clarify`) and goal, as in set "ask". Fix nothing in the wording,
   including typos. Drop a request only if its meaning cannot be recovered, and record why in `raw/dropped.md`. A
   second person checks every label. Label by hand with `robolang validate` and `robolang simulate`, not with a
   model under test.
5. **Freeze.** In the private repository, `sha256sum tasks/* reference/* worlds/* > manifest.sha256`. Only these
   three folders are frozen: `raw/`, `results/` and `reports/` keep changing. Copy the manifest here. Write `PREREGISTRATION.md`: the targets (at least the plan's
   "≥ 90% valid within two attempts" and 0 false programs), the models, the prompt hash, the number of tasks per
   group, and the date. The owner approves it. Commit both.
6. **Run.** Only at milestones: the v2.0 sign-off, and after a major language or prompt change. At most once per
   release. Record every run in `PREREGISTRATION.md`, including ones you would rather forget. Report every failure.
   Do not exclude any afterwards.

## Synthetic requests are a different set

Requests written by a model (any provider) are not held out: they are tidy, they share one style, and they favour
models like the one that wrote them. They are still useful for volume, for more infeasible and unclear cases, and
for rehearsing the labelling. Keep them apart:

- Store them in `synthetic/`, never in the held-out `tasks/`, and give each task `"source": "synthetic-<provider>"`
  (for example `synthetic-openai`). Human tasks have `"source": "human"`.
- Score and report them separately. They never count toward the 50 held-out tasks or toward the MC7 target.
- Never show a model the human requests as examples when generating. If a model writes or rewrites a request, it is
  synthetic, even if a person edited it afterwards.
- A model may help build the worlds and scene descriptions. That does not make the requests written for them synthetic.
- Synthetic sets need not be private. They can become a public bench set.

## Using the results without spoiling the set

- Look at the totals and the failure categories (diagnostic codes, groups), not at individual requests while you
  change the prompt or the language.
- If a specific held-out request drives a change, it is no longer held out. Move it to a public set, record that in
  `PREREGISTRATION.md`, and write a new one to replace it before the next run.
- When more than a quarter of the set has been moved out, or after about a year, retire the set. Publish it as a
  public bench set and write a new held-out set.
