"""
The Core-shaped outcome of a simulated run (spec/simulation-v1.0.md section 10.5, decision 0018).

For a RoboLang 1.3 source, the run record carries `core_outcome`: the payload of a RoboLang Core 0.1
`outcome` message (core-v0.1.md 5.9), `{ "outcome": ..., "evidence": [...] }`. The simulator is the
verifier and says so: the evidence is simulator ground truth, signed as the participant `SIM`.

SIM_KEY is a FIXED, PUBLISHED, SIMULATION-ONLY key. Anyone can compute the signatures, so they show
that the record was produced by the documented procedure and nothing about who produced it.
"""

from __future__ import annotations
from .. import ir
from ..bridge import NotExportable, Triple, core_id, core_time, goal_triples
from ..core.signatures import SIM_HMAC, Key, sign_sim
from .executor import FinalWorld, RunResult
from .values import Truth

SIM = "rl:sim:world"
SIM_KEY_REF = SIM + "#sim-1"
SIM_KEY = Key(SIM, SIM_HMAC, b"robolang-simulation-only-key-not-a-secret")
OBSERVATION_VALID_FOR = {"value": "10", "unit": "s"}


def _actual(condition: ir.Condition, bound: dict[str, str], world: FinalWorld) -> Triple | None:
    """What is actually true of the subject of an outcome atom in the final world; None if unknown or no Core form."""
    if isinstance(condition, ir.RobotAt):
        robot = bound[condition.robot]
        return Triple(core_id("robot", robot), "located_in", core_id("location", dict(world.robots)[robot].location))
    obj = bound[condition.obj]
    state = dict(world.objects)[obj]
    if state.present is not True:  # unknown, or absent: Core 0.1 has no property for "nowhere"
        return None
    if state.held_by is not None:
        return Triple(core_id("object", obj), "carried_by", core_id("robot", state.held_by))
    if state.location is not None:
        return Triple(core_id("object", obj), "located_in", core_id("location", state.location))
    return None


def _observation(obs_id: str, triple: Triple, observed_at: str) -> dict:
    return sign_sim(
        {
            "kind": "observation",
            "id": obs_id,
            **triple.as_dict(),
            "observed_at": observed_at,
            "valid_for": dict(OBSERVATION_VALID_FOR),
            "confidence": "1",
            "source": {"participant": SIM, "sensor": "world", "method": "simulation_ground_truth"},
        },
        SIM_KEY_REF,
        SIM_KEY,
    )


def _status(result: RunResult) -> str:
    if result.state == "SUCCEEDED":
        return "succeeded"
    if result.state == "CANCELLED":
        return "cancelled"
    return "unknown" if result.failure.code == "S302" else "failed"


def core_outcome(result: RunResult, task: ir.Task, run_id: str) -> dict | None:
    """The `outcome` payload for a run of a task with a goal; None when the goal has no Core 0.1 form."""
    bound = dict(result.bindings)
    try:
        criteria = goal_triples(task, bound)
    except NotExportable:
        return None
    end = core_time(result.end_t_ms)
    evaluated = all(g.result is not None for g in result.goal)
    evidence, results = [], []
    for i, (condition, triple, goal) in enumerate(zip(task.goal, criteria, result.goal)):
        actual = _actual(condition, bound, result.final_world) if evaluated else None
        cited = []
        if actual is not None:
            evidence.append(_observation(f"rl:obs:{run_id[4:20]}-{i}", actual, end))
            cited = [evidence[-1]["id"]]
        results.append({"criterion": i, "met": goal.result is Truth.TRUE, "evidence": cited})
    verification = {
        "verified_by": SIM, "method": "criteria_evaluated", "evaluated_at": end, "criteria_results": results,
    } if evaluated else None
    outcome = {
        "kind": "outcome",
        "goal": f"rl:goal:{result.task_name}",
        "status": _status(result),
        "verification": verification,
        "reasons": [{"code": "evidence_missing"}] if result.failure and result.failure.code == "S302" else [],
        "reported_by": SIM,
        "reported_at": end,
    }
    return {"outcome": outcome, "evidence": evidence}
