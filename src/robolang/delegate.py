"""
`delegate`: an AI's goal crosses the wire, a robot runs its own `.rob` plan, a verified outcome comes
back (spec/delegate-v0.1.md, decision record 0020; plan phases 4 and 5).

One deterministic, in-process exchange of full Core 0.1 messages between a planner and a robot. No
threads, no network, simulation only. Every message is signed `sim-hmac-sha256` under a published key
and checked by its receiver with `robolang.core.check_message`. The plan never travels (Core R3): the
robot matches the goal to a task in its own library and runs it with the symbolic executor in its own
world. Verification is simulator ground truth, labelled `rl:sim:world` (plan decision Q1).
"""

from __future__ import annotations
import hashlib
import json
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from decimal import Decimal
from math import floor

from . import bridge, ir
from .core import Context, Key, Receipt, canonical_bytes, check_message, sign_sim, time_ms, verify_sim
from .export_goal import MESSAGE_LIFETIME_MS, export_goal, simulation_key
from .sim import DEFAULT_SEED, World, WorldError, load_world, run, run_id, sha256_normalized, to_record
from .sim.outcome import SIM_KEY, SIM_KEY_REF, core_outcome

RECORD_SCHEMA = "robolang.delegate"
RECORD_VERSION = "0.1"
LATENCY_MS = 50  # delivery of every message, and the pause between two actions of one participant
CLOCK_TOLERANCE_MS = 500
MAX_MESSAGE_LIFETIME_MS = 60_000
MESSAGE_ID_TAG = "robolang-delegate-message/0.1"
STANDING_AUTHORIZATION = "standing_authorization"
_DURATION_MS = {"ms": Decimal(1), "s": Decimal(1000), "min": Decimal(60_000)}


class DelegateError(Exception):
    """The exchange cannot be started. `code` is D701 (a mode other than simulation)."""

    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class Plan:
    """One `.rob` file of the robot's local plan library."""

    path: str
    source: bytes
    tasks: tuple[ir.Task, ...]
    language_version: str


@dataclass(frozen=True)
class Delegation:
    record: dict
    status: str  # the planner's verdict: succeeded, failed, unknown, cancelled or rejected
    rejected: bool  # the robot rejected the goal, or the planner rejected a message
    verified: bool  # the planner accepted the outcome and its own re-evaluation agrees with the robot's claim
    goal: str
    robot: str


@dataclass(frozen=True)
class _Match:
    plan: Plan
    task: ir.Task
    world: World
    order: tuple[int, ...]  # order[i]: the index in the proposed desired_state of local goal condition i


class _Rejected(Exception):
    def __init__(self, receipt: Receipt, message: dict, receiver: str):
        super().__init__(receipt.code)
        self.receipt = receipt
        self.message = message
        self.receiver = receiver


def _digest(*parts: object) -> str:
    return hashlib.sha256(canonical_bytes(list(parts))).hexdigest()


def _plain(receipt: Receipt) -> dict:
    return {"status": receipt.status, "code": receipt.code}


# -------- the robot's side: matching a goal to a local plan --------

def _consider(plan: Plan, task: ir.Task, robot_world: bytes, world_name: str, robot_entity: str | None,
              proposed: tuple[bridge.Triple, ...]) -> tuple[str, _Match | None]:
    """How one library task relates to the proposed goal, and the match if it is one."""
    if task.goal is None:
        return "no_goal", None
    try:
        world = load_world(robot_world, world_name, task)
    except WorldError as error:
        return f"world_{error.code}", None
    bindings = dict(world.task.bindings)
    try:
        triples = bridge.goal_triples(task, bindings)
    except bridge.NotExportable:
        return "no_core_form", None
    if len(set(triples)) != len(triples) or set(triples) != set(proposed) or len(triples) != len(proposed):
        return "goal_differs", None  # the same triples, in any order
    robots = {bindings[p.name] for p in task.parameters if p.type == "robot"}
    if robot_entity not in robots:
        return "robot_not_bound", None
    return "matched", _Match(plan, task, world, tuple(proposed.index(t) for t in triples))


def _robot_entity(recipient: str) -> str | None:
    prefix = bridge.core_id("robot", "")
    return recipient[len(prefix):] if recipient.startswith(prefix) else None


# -------- the planner's side: independent verification --------

def _duration_ms(duration: Mapping) -> Decimal:
    return Decimal(duration["value"]) * _DURATION_MS[duration["unit"]]


def _criterion_reason(criterion: Mapping, observation: Mapping, evaluated_ms: int) -> str | None:
    """Why an observation does not meet a criterion, or None when it does."""
    if (observation["subject"], observation["property"], observation["value"]) != (
            criterion["subject"], criterion["property"], criterion["value"]):
        return "criterion_not_met"
    if Decimal(observation["confidence"]) < Decimal(criterion["min_confidence"]):
        return "confidence_too_low"
    age = evaluated_ms - time_ms(observation["observed_at"])
    if not 0 <= age <= min(_duration_ms(criterion["max_age"]), _duration_ms(observation["valid_for"])):
        return "evidence_stale"
    return None


def _evaluate(index: int, criterion: Mapping, result: Mapping | None, evidence: Mapping[str, dict],
              keys: Mapping[str, Key], evaluated_ms: int | None) -> dict:
    """The planner's own verdict on one success criterion, from the observations the outcome cites."""
    cited = [] if result is None else [evidence[i] for i in result["evidence"] if i in evidence]
    if not cited or evaluated_ms is None:
        return {"criterion": index, "met": False, "evidence": [], "reason": "evidence_missing"}
    reason, met = "signature_invalid", []
    for observation in cited:
        key = keys.get(observation["signature"]["key"])
        if key is None or key.participant != observation["source"]["participant"] \
                or not verify_sim(observation, key):
            continue
        why = _criterion_reason(criterion, observation, evaluated_ms)
        if why is None:
            met.append(observation["id"])
        elif reason == "signature_invalid" or why == "criterion_not_met":
            reason = why
    return {"criterion": index, "met": bool(met), "evidence": met, "reason": None if met else reason}


def verify_outcome(goal: Mapping, message: Mapping, keys: Mapping[str, Key]) -> dict:
    """
    The planner's verdict on an `outcome` message that `check_message` accepted, judged against its own
    proposed `goal`: every evidence signature is verified and every success criterion is re-evaluated
    (observed triple equals the criterion triple, confidence at least `min_confidence`, age at
    `evaluated_at` within `max_age` and the observation's own `valid_for`). `succeeded` is the verdict
    only if the robot claimed it and all of that holds. Returns `{status, criteria, reasons}`.
    """
    payload = message["payload"]
    outcome = payload["outcome"]
    verification = outcome["verification"]
    evidence = {o["id"]: o for o in payload["evidence"]}
    results = {r["criterion"]: r for r in verification["criteria_results"]} if verification else {}
    evaluated_ms = time_ms(verification["evaluated_at"]) if verification else None
    criteria = [_evaluate(i, c, results.get(i), evidence, keys, evaluated_ms)
                for i, c in enumerate(goal["success_criteria"])]
    reasons = []
    if outcome["goal"] != goal["id"]:
        reasons.append({"code": "goal_mismatch"})
    if outcome["reported_by"] != message["sender"]:
        reasons.append({"code": "reporter_mismatch"})
    reasons += [{"code": "claim_not_supported", "criterion": c["criterion"]}
                for c in criteria if c["met"] != bool(results.get(c["criterion"], {}).get("met"))]
    claimed = outcome["status"]
    holds = not reasons and all(c["met"] for c in criteria)
    status = ("succeeded" if holds else "unknown") if claimed == "succeeded" else claimed
    return {"status": status, "criteria": criteria, "reasons": reasons}


# -------- the exchange --------

class _Exchange:
    def __init__(self, proposal: dict, library: Sequence[Plan], robot_world: bytes, robot_world_name: str,
                 intercept: Callable[[dict], dict] | None):
        self.proposal = proposal
        self.planner, self.robot = proposal["sender"], proposal["recipient"]
        self.goal = proposal["payload"]["goal"]
        self.library, self.robot_world, self.robot_world_name = library, robot_world, robot_world_name
        self.intercept = intercept
        planner_key, robot_key = f"{self.planner}#k1", f"{self.robot}#k1"
        self.keys = {ref: simulation_key(ref) for ref in (planner_key, robot_key)} | {SIM_KEY_REF: SIM_KEY}
        self.sent: dict[tuple[str, str], int] = {(self.planner, self.robot): 1}  # propose_goal is sequence 1
        self.expected: dict[tuple[str, str], int] = {}
        self.messages: list[dict] = []
        self.events: list[dict] = []
        self.robot_run: dict | None = None
        self.verdict: dict | None = None

    def event(self, t_ms: int, actor: str, kind: str, **detail) -> None:
        self.events.append({"t_ms": t_ms, "actor": actor, "event": kind, **detail})

    # -- wire --

    def _build(self, sender: str, recipient: str, mtype: str, payload: dict, sent_ms: int, reply_to: str) -> dict:
        stream = (sender, recipient)
        self.sent[stream] = sequence = self.sent.get(stream, 0) + 1
        conversation = self.proposal["conversation_id"]
        sent_at = bridge.core_time(sent_ms)
        message = {
            "schema": "robolang.core.message",
            "schema_version": "0.1",
            "message_id": "m-" + _digest(MESSAGE_ID_TAG, sender, recipient, conversation, sequence)[:32],
            "conversation_id": conversation,
            "sequence": sequence,
            "in_reply_to": reply_to,
            "profile": "a2r",
            "type": mtype,
            "sender": sender,
            "recipient": recipient,
            "sent_at": sent_at,
            "expires_at": bridge.core_time_after(sent_at, MESSAGE_LIFETIME_MS),
            "payload": payload,
        }
        key_ref = f"{sender}#k1"
        return sign_sim(message, key_ref, simulation_key(key_ref))

    def deliver(self, message: dict, sent_ms: int) -> dict:
        """
        Carry `message` to its recipient, who checks it: Core section 8 (`check_message`, with the
        meaning stage) and the order of its stream (0008). A rejected message raises `_Rejected`.
        """
        if self.intercept is not None:
            message = self.intercept(message)
        at_ms = sent_ms + LATENCY_MS
        context = Context(receiver=message["recipient"], now_ms=time_ms(bridge.core_time(at_ms)),
                          clock_tolerance_ms=CLOCK_TOLERANCE_MS, max_message_lifetime_ms=MAX_MESSAGE_LIFETIME_MS,
                          keys=self.keys)
        receipt = check_message(json.dumps(message, ensure_ascii=True).encode("ascii"), context)
        stream = (message["sender"], message["recipient"])
        if receipt.accepted and message["sequence"] != self.expected.get(stream, 1):
            receipt = Receipt("rejected", "C306", "order", "/sequence")
        self.messages.append({"t_ms": at_ms, "document": message, "receipt": _plain(receipt)})
        if not receipt.accepted:
            raise _Rejected(receipt, message, message["recipient"])
        self.expected[stream] = message["sequence"] + 1
        return message

    def transmit(self, sender: str, recipient: str, mtype: str, payload: dict, sent_ms: int, reply_to: str) -> dict:
        return self.deliver(self._build(sender, recipient, mtype, payload, sent_ms, reply_to), sent_ms)

    # -- robot --

    def find_plan(self, t_ms: int) -> _Match | None:
        proposed = tuple(bridge.Triple(d["subject"], d["property"], d["value"]) for d in self.goal["desired_state"])
        entity = _robot_entity(self.robot)
        considered, found = [], None
        for plan in self.library:
            for task in plan.tasks:
                result, found = _consider(plan, task, self.robot_world, self.robot_world_name, entity, proposed)
                considered.append({"file": plan.path, "task": task.name, "result": result})
                if found is not None:
                    break
            if found is not None:
                break
        self.event(t_ms, self.robot, "plans_considered", plans=considered)
        if found is None:
            self.event(t_ms, self.robot, "no_plan_matched", goal=self.goal["id"])
        else:
            self.event(t_ms, self.robot, "plan_matched", file=found.plan.path, task=found.task.name,
                       bindings=dict(found.world.task.bindings))
        return found

    def run_plan(self, match: _Match, start_ms: int) -> tuple[dict, int]:
        """Run the matched task in the robot's world. Returns the outcome payload and the end time."""
        result = run(match.task, match.world)
        identifier = run_id(match.plan.source, self.robot_world, match.task.name, DEFAULT_SEED)
        self.event(start_ms, self.robot, "run_started", file=match.plan.path, task=match.task.name, run_id=identifier)
        self.robot_run = to_record(
            result, task=match.task, run_id=identifier, source_sha256=sha256_normalized(match.plan.source),
            world_sha256=sha256_normalized(self.robot_world), seed=DEFAULT_SEED,
            language_version=match.plan.language_version, world_schema_version=match.world.schema_version)
        end_ms = start_ms + floor(result.end_t_ms)
        self.event(end_ms, self.robot, "run_ended", state=result.state,
                   failure=result.failure.code if result.failure else None)
        payload = core_outcome(result, match.task, identifier, goal_id=self.goal["id"], reported_by=self.robot,
                               t0_ms=start_ms, criterion_of=match.order)
        return payload, end_ms

    # -- the conversation --

    def converse(self) -> None:
        goal_id, reply = self.goal["id"], self.proposal["message_id"]
        self.deliver(self.proposal, 0)
        t = LATENCY_MS  # the robot has the goal
        match = self.find_plan(t)
        if match is None:
            reasons = [{"code": "capability_unsupported"}]
            self.transmit(self.robot, self.planner, "reject", {"goal": goal_id, "reasons": reasons}, t, reply)
            self.verdict = {"status": "rejected", "criteria": [], "reasons": reasons}
            return
        self.transmit(self.robot, self.planner, "assessment",
                      {"goal": goal_id, "result": "feasible", "reasons": []}, t, reply)
        self.transmit(self.robot, self.planner, "accept", {"goal": goal_id}, t + LATENCY_MS, reply)
        start = t + 2 * LATENCY_MS
        self.event(start, self.robot, "authorization", basis=STANDING_AUTHORIZATION, mode="simulation",
                   goal=goal_id)
        payload, end = self.run_plan(match, start)
        outcome = self.transmit(self.robot, self.planner, "outcome", payload, end, reply)
        self.verdict = verify_outcome(self.goal, outcome, self.keys)

    def finish(self) -> Delegation:
        rejected = False
        try:
            self.converse()
        except _Rejected as stop:
            code = {"code": stop.receipt.code, "pointer": stop.receipt.pointer, "stage": stop.receipt.stage,
                    "message": stop.message.get("type"), "receiver": stop.receiver}
            self.verdict = {"status": "unknown", "criteria": [], "reasons": [code]}
            rejected = True
        status = self.verdict["status"]
        record = {
            "delegate_record_schema": RECORD_SCHEMA,
            "version": RECORD_VERSION,
            "mode": "simulation",
            "goal": self.goal["id"],
            "planner": self.planner,
            "robot": self.robot,
            "messages": self.messages,
            "events": sorted(self.events, key=lambda e: e["t_ms"]),
            "robot_run": self.robot_run,
            "planner_verdict": self.verdict,
        }
        rejected = rejected or status == "rejected"
        verified = not rejected and not self.verdict["reasons"]
        return Delegation(record, status, rejected, verified, self.goal["id"], self.robot)


def delegate(task: ir.Task, world: World, *, sender: str, recipient: str, library: Sequence[Plan],
             robot_world: bytes, robot_world_name: str = "<robot world>",
             intercept: Callable[[dict], dict] | None = None, mode: str = "simulation") -> Delegation:
    """
    Export `task`'s goal as a signed `propose_goal` from `sender` to `recipient`, let the robot match it
    to a task of its `library` (bound by `robot_world`), run that task, report the outcome, and let the
    planner verify it. `intercept` sees every message in flight and may change it: a test seam for
    tampering. Raises DelegateError (D701) for any `mode` but `simulation`: Core 0.1 never accepts physical,
    and `export_goal.ExportError` or `bridge.NotExportable` for a goal without a Core form.
    """
    if mode != "simulation":
        raise DelegateError("D701", f"mode '{mode}' is refused: delegate runs in simulation only")
    proposal = export_goal(task, world, sender=sender, recipient=recipient).message
    return _Exchange(proposal, library, robot_world, robot_world_name, intercept).finish()
