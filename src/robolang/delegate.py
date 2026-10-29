"""
`delegate`: an AI's goal crosses the wire, a robot runs its own `.rob` plan, a verified outcome comes
back (spec/delegate-v0.1.md, decision record 0020; plan phases 4 and 5). When the plan binds a second
robot, that peer takes part: commitment, lease and custody transfer (spec/delegate-v0.2.md, 0022).

One deterministic, in-process exchange of full Core 0.1 messages between a planner and a robot (and its
peer). No threads, no network, simulation only. Every message is signed `sim-hmac-sha256` under a published key
and checked by its receiver with `robolang.core.check_message`. The plan never travels (Core R3): the
robot matches the goal to a task in its own library and runs it with the symbolic executor in its own
world. Verification is simulator ground truth, labelled `rl:sim:world` (plan decision Q1).
"""

from __future__ import annotations
import hashlib
import json
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
from decimal import Decimal
from math import floor

from . import bridge, ir
from .core import Context, Key, Receipt, canonical_bytes, check_message, sign_sim, time_ms, verify_sim
from .delegate_peer import (
    DEFAULT_LEASE_MS, CustodyPoint, custody_point, handoff_criteria, peer_observation, trust_keys,
)
from .export_goal import MESSAGE_LIFETIME_MS, export_goal, simulation_key
from .sim import DEFAULT_SEED, RunResult, World, WorldError, load_world, run, run_id, sha256_normalized, to_record
from .sim.outcome import SIM_KEY, SIM_KEY_REF, core_outcome, object_triple

RECORD_SCHEMA = "robolang.delegate"
RECORD_VERSION = "0.1"
RECORD_VERSION_PEER = "0.2"  # a record of a delegation in which a peer robot took part
LATENCY_MS = 50  # delivery of every message, and the pause between two actions of one participant
CLOCK_TOLERANCE_MS = 500
MAX_MESSAGE_LIFETIME_MS = 60_000
MESSAGE_ID_TAG = "robolang-delegate-message/0.1"
STANDING_AUTHORIZATION = "standing_authorization"
EXCHANGE_MS = 4 * LATENCY_MS  # custody exchange: transfer_custody, custody_confirmed, custody_released, acknowledge
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
    peer: str | None  # the world ID of the one other robot the task binds, if any


@dataclass(frozen=True)
class _Stop:
    """The custody exchange stops the run: the commitment's end state, the Core reasons, when the robot reports."""

    commitment: str
    reasons: tuple[str, ...]
    at_ms: int


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
    if len(robots) > 2:
        return "robots_unsupported", None  # at most one peer takes part (spec/delegate-v0.2.md)
    peer = next(iter(robots - {robot_entity}), None)
    return "matched", _Match(plan, task, world, tuple(proposed.index(t) for t in triples), peer)


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
                 intercept: Callable[[dict], dict] | None, lease_ms: int):
        self.proposal = proposal
        self.planner, self.robot = proposal["sender"], proposal["recipient"]
        self.goal = proposal["payload"]["goal"]
        self.library, self.robot_world, self.robot_world_name = library, robot_world, robot_world_name
        self.intercept, self.lease_ms = intercept, lease_ms
        self.peer: str | None = None
        self.commitment: dict | None = None
        self.confirmations: dict[str, str] = {}  # thing -> the custody_confirmed message ID the peer sent
        self.sent: dict[tuple[str, str], int] = {(self.planner, self.robot): 1}  # propose_goal is sequence 1
        self.expected: dict[tuple[str, str], int] = {}
        self.messages: list[dict] = []
        self.events: list[dict] = []
        self.robot_run: dict | None = None
        self.verdict: dict | None = None

    def event(self, t_ms: int, actor: str, kind: str, **detail) -> None:
        self.events.append({"t_ms": t_ms, "actor": actor, "event": kind, **detail})

    # -- wire --

    def _build(self, sender: str, recipient: str, mtype: str, payload: dict, sent_ms: int,
               reply_to: str | None) -> dict:
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
            "profile": "r2r" if self.peer in (sender, recipient) else "a2r",  # robot to robot
            "type": mtype,
            "sender": sender,
            "recipient": recipient,
            "sent_at": sent_at,
            "expires_at": bridge.core_time_after(sent_at, MESSAGE_LIFETIME_MS),
            "payload": payload,
        }
        key_ref = f"{sender}#k1"
        return sign_sim(message, key_ref, simulation_key(key_ref))

    def trust(self, receiver: str) -> dict[str, Key]:
        """The keys `receiver` verifies with: its counterparts' only (0013), and for the planner the simulator's."""
        return trust_keys(self.planner, self.robot, self.peer, SIM_KEY_REF, SIM_KEY).get(receiver, {})

    def deliver(self, message: dict, sent_ms: int) -> dict:
        """
        Carry `message` to its recipient, who checks it: Core section 8 (`check_message`, with the
        meaning stage), the order of its stream (0008) and, for a release, the confirmation it names
        (`C403`). A rejected message raises `_Rejected`.
        """
        if self.intercept is not None:
            message = self.intercept(message)
        at_ms = sent_ms + LATENCY_MS
        context = Context(receiver=message["recipient"], now_ms=time_ms(bridge.core_time(at_ms)),
                          clock_tolerance_ms=CLOCK_TOLERANCE_MS, max_message_lifetime_ms=MAX_MESSAGE_LIFETIME_MS,
                          keys=self.trust(message["recipient"]))
        receipt = check_message(json.dumps(message, ensure_ascii=True).encode("ascii"), context)
        stream = (message["sender"], message["recipient"])
        if receipt.accepted and message["sequence"] != self.expected.get(stream, 1):
            receipt = Receipt("rejected", "C306", "order", "/sequence")
        if receipt.accepted and message["type"] == "custody_released" \
                and message["payload"]["confirmation"] != self.confirmations.get(message["payload"]["thing"]):
            receipt = Receipt("rejected", "C403", "meaning", "/payload/confirmation")
        self.messages.append({"t_ms": at_ms, "document": message, "receipt": _plain(receipt)})
        if not receipt.accepted:
            raise _Rejected(receipt, message, message["recipient"])
        self.expected[stream] = message["sequence"] + 1
        return message

    def transmit(self, sender: str, recipient: str, mtype: str, payload: dict, sent_ms: int,
                 reply_to: str | None = None) -> dict:
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

    def record_run(self, match: _Match, result: RunResult, start_ms: int, pause_ms: int = 0) -> tuple[dict, int]:
        """
        Keep `result`, the robot's run of the matched task that began at `start_ms`, as the run record.
        `pause_ms` is the time the custody exchange took inside the run. Returns the outcome payload and the end time.
        """
        identifier = run_id(match.plan.source, self.robot_world, match.task.name, DEFAULT_SEED)
        self.event(start_ms, self.robot, "run_started", file=match.plan.path, task=match.task.name, run_id=identifier)
        self.robot_run = to_record(
            result, task=match.task, run_id=identifier, source_sha256=sha256_normalized(match.plan.source),
            world_sha256=sha256_normalized(self.robot_world), seed=DEFAULT_SEED,
            language_version=match.plan.language_version, world_schema_version=match.world.schema_version)
        end_ms = start_ms + pause_ms + floor(result.end_t_ms)
        self.event(end_ms, self.robot, "run_ended", state=result.state,
                   failure=result.failure.code if result.failure else None)
        payload = core_outcome(result, match.task, identifier, goal_id=self.goal["id"], reported_by=self.robot,
                               t0_ms=start_ms + pause_ms, criterion_of=match.order)
        return payload, end_ms

    def run_plan(self, match: _Match, start_ms: int) -> tuple[dict, int]:
        """Run the matched task in the robot's world. Returns the outcome payload and the end time."""
        return self.record_run(match, run(match.task, match.world), start_ms)

    # -- the peer: commitment, lease, custody --

    def sub_goal(self) -> dict:
        return dict(self.goal, id=self.goal["id"] + "-sub-1", requested_by=self.robot)

    def request_commitment(self, match: _Match, t_ms: int, reply: str) -> bool:
        """
        request_assistance, then the peer's offer (with a lease) and the robot's commit; the robot
        rejects the goal if the peer declines. Returns True when the commitment is active.
        """
        sub = self.sub_goal()
        request = self.transmit(self.robot, self.peer, "request_assistance", {
            "goal": sub, "things": self.proposal["payload"]["things"], "parent_goal": self.goal["id"]}, t_ms)
        t_peer = t_ms + LATENCY_MS  # the peer has the request
        if match.world.robots[match.peer].ready is not True:
            reasons = [{"code": "participant_busy"}]
            self.transmit(self.peer, self.robot, "decline", {"goal": sub["id"], "reasons": reasons}, t_peer,
                          request["message_id"])
            self.reject_goal(t_peer + LATENCY_MS, reply)
            return False
        granted = bridge.core_time(t_peer)
        self.lease_end_ms = t_peer + self.lease_ms
        lease = {"granted_at": granted, "expires_at": bridge.core_time(self.lease_end_ms), "renewal": 0}
        offer = self.transmit(self.peer, self.robot, "offer_assistance",
                              {"goal": sub["id"], "conditions": [], "lease": lease}, t_peer, request["message_id"])
        payload = offer["payload"]
        t_robot = t_peer + LATENCY_MS
        self.commitment = {
            "kind": "commitment", "id": self.goal["id"].replace("rl:goal:", "rl:commit:", 1) + "-assist",
            "debtor": self.peer, "creditor": self.robot, "obligation": {"goal": sub["id"]},
            "conditions": payload["conditions"], "lease": payload["lease"], "state": "active",
        }
        self.transmit(self.robot, self.peer, "commit", {"commitment": self.commitment}, t_robot)
        self.event(t_robot, self.robot, "commitment", commitment=self.commitment["id"], state="active")
        return True

    def reject_goal(self, t_ms: int, reply: str) -> None:
        """The robot withdraws its assessment and rejects the goal: no peer can help (`assistance_unavailable`)."""
        reasons = [{"code": "assistance_unavailable"}]
        goal_id = self.goal["id"]
        self.transmit(self.robot, self.planner, "assessment",
                      {"goal": goal_id, "result": "infeasible", "reasons": reasons}, t_ms, reply)
        self.transmit(self.robot, self.planner, "reject", {"goal": goal_id, "reasons": reasons},
                      t_ms + LATENCY_MS, reply)
        self.verdict = {"status": "rejected", "criteria": [], "reasons": reasons}

    def settle(self, state: str, t_ms: int) -> None:
        """The commitment ends in `state` (fulfilled, failed, or lapsed when its lease ends) at `t_ms`."""
        self.commitment = dict(self.commitment, state=state)
        if state == "lapsed":
            t_ms = self.lease_end_ms
            self.event(t_ms, self.robot, "lease_expired", lease="commitment", commitment=self.commitment["id"])
        self.event(t_ms, self.robot, "commitment", commitment=self.commitment["id"], state=state)

    def custody_state(self, actor: str, t_ms: int, thing: str, state: str) -> None:
        self.event(t_ms, actor, "custody", thing=thing, state=state)

    def custody_exchange(self, point: CustodyPoint, final_world, t_ms: int) -> _Stop | None:
        """
        The custody exchange at the custody point, `t_ms`: None when the peer holds the thing, else why the
        run must stop. The lease must still hold when the exchange ends, judged in the safe direction.
        """
        thing = bridge.core_id("object", point.thing)
        self.event(t_ms, self.robot, "custody_point", step=point.step, thing=thing,
                   place=bridge.core_id("location", point.place))
        if t_ms + EXCHANGE_MS >= self.lease_end_ms - CLOCK_TOLERANCE_MS:
            return _Stop("lapsed", ("lease_expired",), max(t_ms, self.lease_end_ms))
        criteria = handoff_criteria(point, self.peer)
        self.transmit(self.robot, self.peer, "transfer_custody", {
            "thing": thing, "commitment": self.commitment["id"], "criteria": criteria}, t_ms)
        self.custody_state(self.robot, t_ms, thing, "OFFERED")
        t_peer = t_ms + LATENCY_MS
        triple = object_triple(point.thing, final_world)
        if triple is None:  # the peer cannot see the thing: it confirms nothing
            self.event(t_peer, self.peer, "no_confirmation_evidence", thing=thing)
            self.custody_state(self.robot, t_peer + LATENCY_MS, thing, "HOLDING")
            return _Stop("failed", ("custody_unknown",), t_peer + LATENCY_MS)
        observation = peer_observation("rl:obs:" + _digest(self.commitment["id"], "custody")[:16], triple,
                                       bridge.core_time(t_peer), self.peer)
        confirmed = self.transmit(self.peer, self.robot, "custody_confirmed",
                                  {"thing": thing, "observations": [observation]}, t_peer)
        self.confirmations[thing] = confirmed["message_id"]
        self.custody_state(self.peer, t_peer, thing, "CONFIRMATION_SENT")
        t_robot = t_peer + LATENCY_MS
        reason = self.confirmation_reason(criteria, confirmed["payload"]["observations"],
                                          time_ms(bridge.core_time(t_robot)))
        if reason is not None:  # no release: the thing stays with the robot
            self.event(t_robot, self.robot, "confirmation_not_accepted", thing=thing, reason=reason)
            self.custody_state(self.robot, t_robot, thing, "HOLDING")
            return _Stop("failed", ("custody_unknown",), t_robot)
        release = self.transmit(self.robot, self.peer, "custody_released",
                                {"thing": thing, "confirmation": confirmed["message_id"]}, t_robot,
                                confirmed["message_id"])
        self.custody_state(self.robot, t_robot, thing, "RELEASE_SENT")
        t_ack = t_robot + LATENCY_MS  # the peer has the release: accepted durably, then acknowledged (rule 3)
        self.custody_state(self.peer, t_ack, thing, "HOLDING")
        self.event(t_ack, self.peer, "custody_committed", thing=thing, release=release["message_id"])
        self.transmit(self.peer, self.robot, "acknowledge",
                      {"acknowledged": release["message_id"], "accepted": True}, t_ack, release["message_id"])
        self.custody_state(self.robot, t_ack + LATENCY_MS, thing, "RELEASED")
        return None

    def confirmation_reason(self, criteria: list[dict], observations: list[dict], evaluated_ms: int) -> str | None:
        """Why the peer's own observations do not meet the criteria (the planner's rules), or None when they do."""
        own = {o["id"]: o for o in observations if o["source"]["participant"] == self.peer}
        for index, criterion in enumerate(criteria):
            verdict = _evaluate(index, criterion, {"evidence": list(own)}, own, self.trust(self.robot), evaluated_ms)
            if not verdict["met"]:
                return verdict["reason"]
        return None

    def run_with_peer(self, match: _Match, start_ms: int) -> tuple[dict, int]:
        """
        Run the plan up to the custody point (the world script cancels before that step: `robolang.sim`
        stays pure), do the exchange there, and if it holds run the whole plan again as the final run.
        When the exchange fails, the cancelled run is the final run. Returns the outcome payload and its time.
        """
        bindings = dict(match.world.task.bindings)
        point = custody_point(match.task, bindings, _robot_entity(self.robot), match.peer)
        scripted = match.world.task.cancel_before_step
        result, stop, pause = None, None, 0
        if point is not None and (scripted is None or scripted >= point.step):
            probe = run(match.task, replace(match.world, task=replace(match.world.task, cancel_before_step=point.step)))
            result = probe
            if probe.cancelled_before_step == point.step:
                stop = self.custody_exchange(point, probe.final_world, start_ms + floor(probe.end_t_ms))
                if stop is None:
                    result, pause = None, EXCHANGE_MS
        result = result or run(match.task, match.world)
        if stop is None:
            late = self.first_step_past_lease(match, result, start_ms, point, pause)
            if late is not None:  # the peer would act without a valid commitment: stop before that step
                result = run(match.task, replace(match.world, task=replace(match.world.task, cancel_before_step=late)))
                stop = _Stop("lapsed", ("lease_expired",), self.lease_end_ms)
        payload, end = self.record_run(match, result, start_ms, pause)
        if stop is None:
            self.settle("fulfilled" if result.state == "SUCCEEDED" else "failed", end)
            return payload, end
        self.settle(stop.commitment, stop.at_ms)
        payload["outcome"].update(status="failed", reasons=[{"code": c} for c in stop.reasons],
                                  reported_at=bridge.core_time(stop.at_ms))
        return payload, stop.at_ms

    def first_step_past_lease(self, match: _Match, result: RunResult, start_ms: int,
                              point: CustodyPoint | None, pause: int) -> int | None:
        """
        The first plan step of the peer that would start when the lease no longer holds (judged in the safe
        direction, as at the custody point), or None. Every act of the peer is checked, not only the custody
        exchange (decision record 0022, amendment).
        """
        bindings = dict(match.world.task.bindings)
        peer_params = {p.name for p in match.task.parameters if p.type == "robot" and bindings[p.name] == match.peer}
        for step in result.steps:
            if step.status == "not_run" or getattr(step.step, "robot", None) not in peer_params:
                continue
            shift = pause if point is not None and step.index >= point.step else 0
            if start_ms + shift + floor(step.t_start_ms) >= self.lease_end_ms - CLOCK_TOLERANCE_MS:
                return step.index
        return None

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
        if match.peer is None:
            self.transmit(self.robot, self.planner, "assessment",
                          {"goal": goal_id, "result": "feasible", "reasons": []}, t, reply)
            t_accept = t + LATENCY_MS
            self.transmit(self.robot, self.planner, "accept", {"goal": goal_id}, t_accept, reply)
        else:
            self.peer = bridge.core_id("robot", match.peer)
            self.event(t, self.robot, "peer_selected", peer=self.peer)
            self.transmit(self.robot, self.planner, "assessment", {
                "goal": goal_id, "result": "feasible_with_assistance", "reasons": [{"code": "assistance_required"}]},
                t, reply)
            if not self.request_commitment(match, t + LATENCY_MS, reply):
                return
            t_accept = t + 4 * LATENCY_MS
            self.transmit(self.robot, self.planner, "accept", {"goal": goal_id}, t_accept, reply)
            self.event(t_accept, self.peer, "authorization", basis=STANDING_AUTHORIZATION, mode="simulation",
                       goal=self.sub_goal()["id"])
        start = t_accept + LATENCY_MS
        self.event(start, self.robot, "authorization", basis=STANDING_AUTHORIZATION, mode="simulation",
                   goal=goal_id)
        payload, end = self.run_plan(match, start) if match.peer is None else self.run_with_peer(match, start)
        outcome = self.transmit(self.robot, self.planner, "outcome", payload, end, reply)
        self.verdict = verify_outcome(self.goal, outcome, self.trust(self.planner))

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
        peered = self.peer is not None  # a peer took part: version 0.2, with `peer` and `commitment`
        record = {
            "delegate_record_schema": RECORD_SCHEMA,
            "version": RECORD_VERSION_PEER if peered else RECORD_VERSION,
            "mode": "simulation",
            "goal": self.goal["id"],
            "planner": self.planner,
            "robot": self.robot,
            **({"peer": self.peer} if peered else {}),
            "messages": self.messages,
            "events": sorted(self.events, key=lambda e: e["t_ms"]),
            "robot_run": self.robot_run,
            **({"commitment": self.commitment} if peered else {}),
            "planner_verdict": self.verdict,
        }
        rejected = rejected or status == "rejected"
        verified = not rejected and not self.verdict["reasons"]
        return Delegation(record, status, rejected, verified, self.goal["id"], self.robot)


def delegate(task: ir.Task, world: World, *, sender: str, recipient: str, library: Sequence[Plan],
             robot_world: bytes, robot_world_name: str = "<robot world>",
             intercept: Callable[[dict], dict] | None = None, mode: str = "simulation",
             lease_ms: int = DEFAULT_LEASE_MS) -> Delegation:
    """
    Export `task`'s goal as a signed `propose_goal` from `sender` to `recipient`, let the robot match it
    to a task of its `library` (bound by `robot_world`), run that task, report the outcome, and let the
    planner verify it. `intercept` sees every message in flight and may change it: a test seam for
    tampering. Raises DelegateError (D701) for any `mode` but `simulation`: Core 0.1 never accepts physical,
    and `export_goal.ExportError` or `bridge.NotExportable` for a goal without a Core form. When the
    matched task binds a second robot, that robot is the peer: the robot asks it for help, and `lease_ms`
    is the lease the peer offers (spec/delegate-v0.2.md).
    """
    if mode != "simulation":
        raise DelegateError("D701", f"mode '{mode}' is refused: delegate runs in simulation only")
    proposal = export_goal(task, world, sender=sender, recipient=recipient).message
    return _Exchange(proposal, library, robot_world, robot_world_name, intercept, lease_ms).finish()
