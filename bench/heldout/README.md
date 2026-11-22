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
| `manifest.sha256` | yes, once frozen | SHA-256 of every private file: proves what was frozen, and when, without showing it |
| `PREREGISTRATION.md` | yes, once frozen | the targets, the models, the prompt and the date, fixed before the first run |
| `private/` | **never** (gitignored) | a safety net only. Keep the real copy **outside the repo** |

The repo is public. Anything committed, pasted into an issue or PR, or shown to an AI coding session is no longer held
out. Keep the set in a private place (a private repository or the owner's own storage) with one backup.

Private layout, the same as the other sets:

```
<private>/tasks/h001.json        id, prompt, world, group, expect, expected_goal, tags (as in bench/tasks_ask/)
<private>/reference/h001.rob     + h001.json: a hand-written solution and its bindings (feasible tasks)
<private>/reference/h001.attempt.rob   the obvious attempt that fails (infeasible or inexpressible tasks)
<private>/worlds/*.json          new worlds, not copies of bench/worlds/
<private>/raw/                   the writers' original files, unedited
```

Use neutral IDs (`h001`, `h002`, ...). File names appear in the manifest.

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
5. **Freeze.** From the private folder, `find . -type f -not -path './raw/*' | sort | xargs sha256sum >
   manifest.sha256`. Copy that file here. Write `PREREGISTRATION.md`: the targets (at least the plan's
   "≥ 90% valid within two attempts" and 0 false programs), the models, the prompt hash, the number of tasks per
   group, and the date. The owner approves it. Commit both.
6. **Run.** Only at milestones: the v2.0 sign-off, and after a major language or prompt change. At most once per
   release. Record every run in `PREREGISTRATION.md`, including ones you would rather forget. Report every failure.
   Do not exclude any afterwards.

## Using the results without spoiling the set

- Look at the totals and the failure categories (diagnostic codes, groups), not at individual requests while you
  change the prompt or the language.
- If a specific held-out request drives a change, it is no longer held out. Move it to a public set, record that in
  `PREREGISTRATION.md`, and write a new one to replace it before the next run.
- When more than a quarter of the set has been moved out, or after about a year, retire the set. Publish it as a
  public bench set and write a new held-out set.
