"""
Reference simulation of the RFC-001 section 18 scenario (ARCH-2; spec/reference-simulation-v0.1.md).

One Python process, one symbolic world, four participants exchanging Core 0.1 messages that
`robolang.core` checks. SIMULATION ONLY: no transport, no network, no adapter, no ROS 2, no
hardware, no AI model. A run that ends `succeeded` says the participants followed Core 0.1 and
RFC-001 sections 9-13 consistently with a declared world; it is not evidence about real robots.

The JSON run record, the run ID and an entry point are ARCH-3.
"""

from __future__ import annotations

from .engine import Delivery, Event, Reject, RunResult, SentMessage, Simulation, run_case
from .scenario import ScenarioError, apply_overrides, load_scenario, ms, time_text
from .world import Fact, World

__all__ = [
    "Delivery",
    "Event",
    "Fact",
    "Reject",
    "RunResult",
    "ScenarioError",
    "SentMessage",
    "Simulation",
    "World",
    "apply_overrides",
    "load_scenario",
    "ms",
    "run_case",
    "time_text",
]
