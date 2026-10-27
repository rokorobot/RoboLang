"""
Gate 2 interoperability (spec/interop-gate-2-v0.1.md, decision record 0009).

INTEROP-1: the harness side of the test binding `robolang-sim-stdio` 0.1 and the reference
amr-07 run through it as a separate process. SIMULATION ONLY.
"""

from __future__ import annotations

from .harness import (
    AMR,
    EVENT_KINDS,
    EVENT_KINDS_0009,
    BindingFailure,
    ExternalParticipant,
    InconsistentClaim,
    run_case_external,
    simulate_case_external,
)
from .view import build_view

__all__ = [
    "AMR",
    "EVENT_KINDS",
    "EVENT_KINDS_0009",
    "BindingFailure",
    "ExternalParticipant",
    "InconsistentClaim",
    "build_view",
    "run_case_external",
    "simulate_case_external",
]
