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
from ..bridge import (
    NotExportable, Triple, alternatives_of, core_id, core_time, flatten_atoms, goal_alternatives, triple_of,
)
from ..core.signatures import SIM_HMAC, Key, sign_sim
from .executor import FinalWorld, RunResult

SIM = "rl:sim:world"
SIM_KEY_REF = SIM + "#sim-1"
SIM_KEY = Key(SIM, SIM_HMAC, b"robolang-simulation-only-key-not-a-secret")
OBSERVATION_VALID_FOR = {"value": "10", "unit": "s"}


def object_triple(obj: str, world: FinalWorld) -> Triple | None:
    """What is actually true of an object in the final world; None if unknown or absent (Core has no "nowhere")."""
    state = dict(world.objects)[obj]
    if state.present is not True:
        return None
    if state.held_by is not None:
        return Triple(core_id("object", obj), "carried_by", core_id("robot", state.held_by))
    if state.location is not None:
        return Triple(core_id("object", obj), "located_in", core_id("location", state.location))
    return None


def _actual(condition: ir.Condition, bound: dict[str, str], world: FinalWorld) -> Triple | None:
    """What is actually true of the subject of an outcome atom in the final world; None if unknown or no Core form."""
    if isinstance(condition, ir.RobotAt):
        robot = bound[condition.robot]
        return Triple(core_id("robot", robot), "located_in", core_id("location", dict(world.robots)[robot].location))
    return object_triple(bound[condition.obj], world)


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


def core_outcome(result: RunResult, task: ir.Task, run_id: str, *, goal_id: str | None = None,
                 reported_by: str = SIM, t0_ms: int = 0, criterion_of: tuple[int, ...] | None = None,
                 core: str = "0.1", index_of: tuple[tuple[int, tuple[int, ...]], ...] | None = None) -> dict | None:
    """
    The `outcome` payload for a run of a task with a goal; None when the goal has no Core 0.1 form.

    By default it describes the run on its own: the goal is the task's (`rl:goal:<task name>`), the
    reporter is SIM and times count from the start of the run. `delegate` passes the proposed goal's
    ID, the robot as the reporter, the Core time `t0_ms` at which the run began, and `criterion_of`: for each
local goal condition, the index of its criterion in the proposed goal, whose order may differ. Results and
evidence are listed in criterion order. The verifier stays SIM.

    `core="0.2"` gives the Core 0.2 payload (spec/core-v0.2-draft.md section 3): see `_core_outcome_02`;
    `index_of` then replaces `criterion_of`.
    """
    if core == "0.2":
        return _core_outcome_02(result, task, run_id, goal_id, reported_by, t0_ms, index_of)
    bound = dict(result.bindings)
    try:
        # The goal of the path actually taken (decision 0027 B7); for a task without `if`, the task's goal.
        atoms = flatten_atoms(tuple(g.condition for g in result.goal))
        criteria = tuple(triple_of(atom, bound) for atom in atoms)
    except NotExportable:
        return None
    end = core_time(t0_ms + result.end_t_ms)
    evaluated = bool(result.goal) and all(g.result is not None for g in result.goal)
    order = criterion_of or tuple(range(len(criteria)))
    evidence, results = [], []
    for condition, criterion, i in sorted(zip(atoms, criteria, order), key=lambda entry: entry[2]):
        actual = _actual(condition, bound, result.final_world) if evaluated else None
        cited = []
        if actual is not None:
            evidence.append(_observation(f"rl:obs:{run_id[4:20]}-{i}", actual, end))
            cited = [evidence[-1]["id"]]
        results.append({"criterion": i, "met": actual == criterion, "evidence": cited})
    verification = {
        "verified_by": SIM, "method": "criteria_evaluated", "evaluated_at": end, "criteria_results": results,
    } if evaluated else None
    outcome = {
        "kind": "outcome",
        "goal": goal_id or f"rl:goal:{result.task_name}",
        "status": _status(result),
        "verification": verification,
        "reasons": [{"code": "evidence_missing"}] if result.failure and result.failure.code == "S302" else [],
        "reported_by": reported_by,
        "reported_at": end,
    }
    return {"outcome": outcome, "evidence": evidence}


def _core_outcome_02(result: RunResult, task: ir.Task, run_id: str, goal_id: str | None, reported_by: str,
                     t0_ms: int, index_of: tuple[tuple[int, tuple[int, ...]], ...] | None) -> dict | None:
    """
    The Core 0.2 `outcome` payload (decision 0028 C2, C3, Q3); None when the goal has no Core form.

    `index_of[i]` is, for local alternative i (`bridge.goal_alternatives` order), the pair (its index in the
    proposed goal, the proposed index of each of its criteria); by default the identity (the goal as exported).
    `alternative` is on every result. `met_alternative` is the alternative the run pursued: for a plain or
    branching goal, the one of the path taken (its criteria are all listed, met or not); for a goal that is an
    `or`, the first alternative whose criteria all hold in the final world, or null (every alternative is listed).
    A run ended by `fail` (S262) is `failed` with a `declared` reason and no verification.
    """
    bound = dict(result.bindings)
    try:
        local = goal_alternatives(task, bound)
        pursued_goal = alternatives_of(tuple(g.condition for g in result.goal)) if result.goal else [()]
    except NotExportable:
        return None
    triples = [tuple(triple_of(atom, bound) for atom in atoms) for atoms in local]
    index_of = index_of or tuple((i, tuple(range(len(t)))) for i, t in enumerate(triples))
    end = core_time(t0_ms + result.end_t_ms)
    verification, evidence, reasons = None, [], []
    if result.failure is not None and result.failure.code == "S262":
        reasons = [{"code": "declared", "text": result.failure.reason}]
    elif result.failure is not None and result.failure.code == "S302":
        reasons = [{"code": "evidence_missing"}]
    elif result.goal and all(g.result is not None for g in result.goal):
        by_set = {frozenset(t): i for i, t in enumerate(triples)}
        shown = sorted({by_set[frozenset(triple_of(a, bound) for a in atoms)] for atoms in pursued_goal},
                       key=lambda i: index_of[i][0])
        is_or = len(result.goal) == 1 and isinstance(result.goal[0].condition, ir.Or)
        results, holds = [], {}
        for i in shown:
            alternative, criteria = index_of[i]
            holds[i] = True
            for atom, triple, criterion in zip(local[i], triples[i], criteria):
                actual = _actual(atom, bound, result.final_world)
                cited = []
                if actual is not None:
                    evidence.append(_observation(f"rl:obs:{run_id[4:20]}-{alternative}-{criterion}", actual, end))
                    cited = [evidence[-1]["id"]]
                holds[i] &= actual == triple
                results.append({"alternative": alternative, "criterion": criterion, "met": actual == triple,
                                "evidence": cited})
        pursued = next((i for i in shown if holds[i]), None) if is_or else shown[0]
        results.sort(key=lambda r: (r["alternative"], r["criterion"]))
        verification = {
            "verified_by": SIM, "method": "criteria_evaluated", "evaluated_at": end, "criteria_results": results,
            "met_alternative": None if pursued is None else index_of[pursued][0],
        }
    outcome = {
        "kind": "outcome",
        "goal": goal_id or f"rl:goal:{result.task_name}",
        "status": _status(result),
        "verification": verification,
        "reasons": reasons,
        "reported_by": reported_by,
        "reported_at": end,
    }
    return {"outcome": outcome, "evidence": evidence}
