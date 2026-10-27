"""
Reference simulation of the RFC-001 section 18 scenario (ARCH-2; spec/reference-simulation-v0.1.md).

One Python process, one symbolic world, four participants exchanging Core 0.1 messages that
`robolang.core` checks. SIMULATION ONLY: no transport, no network, no adapter, no ROS 2, no
hardware, no AI model. A run that ends `succeeded` says the participants followed Core 0.1 and
RFC-001 sections 9-13 consistently with a declared world; it is not evidence about real robots.

`simulate_case` runs one case and returns its run record (ARCH-3); the `robolang arch02`
command prints it.
"""

from __future__ import annotations

from .engine import Delivery, Event, Reject, RunResult, SentMessage, Simulation, run_case
from .record import RUN_RECORD_SCHEMA, RUN_RECORD_SCHEMA_VERSION, run_id, simulate_case, to_json, to_record
from .scenario import ScenarioError, apply_overrides, load_scenario, ms, time_text
from .world import Fact, World

__all__ = [
    "RUN_RECORD_SCHEMA",
    "RUN_RECORD_SCHEMA_VERSION",
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
    "run_id",
    "simulate_case",
    "time_text",
    "to_json",
    "to_record",
]
