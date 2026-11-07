# RoboLang development steps

A one-page history of RoboLang, from the legacy v1 code to RoboLang 1.5 and the ROS 2 physics
simulation backend. Sources: [development plan](DEVELOPMENT_PLAN.md), [strategy](STRATEGY.md),
[decision records](../spec/decisions/) 0001–0035 and git history up to 2026-11-04.

> ⚠️ Everything below is simulation only. Physical execution is not authorized.

The Mermaid diagram below is the editable source. Update it when a step lands.
[`development-steps.png`](development-steps.png) is a static snapshot as of 2026-11-07.

```mermaid
timeline
    title RoboLang development steps
    2025-11-08 : 0 · Legacy v1 baseline (9e9f92f) : text-scanner runtime and ROS 2 adapter, no grammar or tests
    2026-10-11 : 1 · Development plan v1.1 → v2.0 : 2 · RoboLang 1.1 language correctness (RL-11A–E) : lexer, parser, validator, Task IR 1.0, validate and inspect
    2026-10-19 : 3 · RFC-001 AI-native architecture (0002, D1–D8)
    2026-10-27 : 4 · Foundation 1, RL-BASIC-01 simulate (signed off) : 5 · RL-ARCH-02 Core 0.1 and reference simulation (gate 1 PASS) : 6 · RoboLang 1.2 combined conditions (LANG-1)
    2026-10-28 : 7 · INTEROP-1/2, gate 2 bundle frozen (28c69fe) : INTEROP-3/4 deferred (0017, 0023)
    2026-10-29 : 8 · Authoring benchmark drives the roadmap (0024) : 9 · RoboLang 1.3 goals and typed sensors, CORE-1 IR reader, export-goal, delegate, validate --world, shipped reference
    2026-10-30 : 10 · RoboLang 1.4 bounded branching (N4) : 11 · Core 0.2 frozen (0028) : 12 · 0.2.0 on PyPI
    2026-10-30 to 11-01 : 13 · H3-1 ROS 2 backend in Gazebo, phases 1–4 : 11 tasks × 3 runs succeed, 51/51 oracle checks agree
    2026-11-02 : 14 · INTEROP-3 relaunched on Core 0.2 (0030), bundle frozen
    2026-11-02 to 11-04 : 15 · RoboLang 1.5 (0031–0034) : region_clear and robot_ready, holder check, step failure handlers
    Next : 16 · Teach 1.5 and benchmark set 4 (0035, proposed) : 17 · Independent Core 0.2 implementation (open) : 18 · H4 hardware (not authorized)
```

## Horizons ([strategy](STRATEGY.md) section 3)

| Horizon | Target | State on 2026-11-07 |
|---|---|---|
| H1 Trust the basics | ~Nov 2026 | Likely met: reference shipped, PyPI live (≥97% accuracy target not re-checked) |
| H2 Express real requests | ~Q1 2027 | Work done early: 1.4 branching, `validate --world`, Core 0.2 frozen |
| H3 Leave the symbolic world | ~Q2 2027 | Half met: Gazebo task family passes; independent Core 0.2 implementation still open |
| H4 Earn hardware | ~H2 2027 | Not started; needs owner sign-off on a written safety case |
