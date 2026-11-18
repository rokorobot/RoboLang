# Benchmark set 10 "messages": baseline on RoboLang 2.0

**Run:** 2026-11-18, `Benchmark` with `set_only: messages`,
[run 44321697493](https://github.com/rokorobot/RoboLang/actions/runs/44321697493). Sonnet 5.5 and Haiku 5.5, full and
grammar-only prompts, 3 repeats, 24 tasks ([plan](../plans/bench-10-messages.md), decision
[0043](../../spec/decisions/0043-incoming-messages.md)). **Cost $0.99.** Not gated.

| Prompt | Model | Message tasks declined | False programs | Infeasible | Inexpressible | Controls |
|---|---|---|---|---|---|---|
| full | Sonnet 5.5 | 30/30 | 0/30 | 15/15 | 15/15 | 12/12 |
| full | Haiku 5.5 | 30/30 | 0/30 | 15/15 | 15/15 | 12/12 |
| grammar | Sonnet 5.5 | 30/30 | 0/30 | 15/15 | 15/15 | 12/12 |
| grammar | Haiku 5.5 | 30/30 | 0/30 | 15/15 | 15/15 | 12/12 |

## Reading

Every decision is right and there are no silent failures, as in set 8 (clock). RoboLang 2.1 adds coverage: it makes
expressible the requests that models today decline correctly. The bounds guard against the reverse risk, that the new
condition invites false programs. The likely ones are a wrong channel or text (B9), a guessed wait, and a message
"trusted" from a sender the world does not allow.
