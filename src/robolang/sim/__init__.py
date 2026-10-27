"""
Simulation-only package (RL-BASIC-01b). Nothing here drives a robot.

It never imports an adapter, the v1 interpreter or ROS 2. It provides the value types, the
world-file loader, the executor, the run record and the run ID of spec/simulation-v1.0.md;
the `simulate` command is in robolang.cli.
"""

from __future__ import annotations

from .executor import (
    AppliedEvent,
    Failure,
    FinalWorld,
    LimitResult,
    Message,
    ObjectState,
    Observation,
    PreconditionResult,
    RegionState,
    RobotState,
    RunResult,
    SensorState,
    SimulationInputError,
    StepResult,
    run,
)
from .record import DEFAULT_SEED, SEED, run_id, sha256_normalized, to_json, to_record
from .values import DURATION_UNITS, SPEED_UNITS, Speed, Truth, duration_ms, duration_ms_of, ms_text, parse_decimal
from .world import (
    WORLD_SCHEMA,
    WORLD_SCHEMA_VERSION,
    Event,
    Region,
    Robot,
    Sensor,
    TaskScript,
    World,
    WorldError,
    WorldObject,
    load_world,
)

__all__ = [
    "DEFAULT_SEED",
    "SEED",
    "run_id",
    "sha256_normalized",
    "to_json",
    "to_record",
    "AppliedEvent",
    "Failure",
    "FinalWorld",
    "LimitResult",
    "Message",
    "ObjectState",
    "Observation",
    "PreconditionResult",
    "RegionState",
    "RobotState",
    "RunResult",
    "SensorState",
    "SimulationInputError",
    "StepResult",
    "run",
    "DURATION_UNITS",
    "SPEED_UNITS",
    "WORLD_SCHEMA",
    "WORLD_SCHEMA_VERSION",
    "Event",
    "Region",
    "Robot",
    "Sensor",
    "Speed",
    "TaskScript",
    "Truth",
    "World",
    "WorldError",
    "WorldObject",
    "duration_ms",
    "duration_ms_of",
    "load_world",
    "ms_text",
    "parse_decimal",
]
