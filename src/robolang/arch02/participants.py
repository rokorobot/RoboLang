"""
The four participants of the reference scenario (spec/reference-simulation-v0.1.md sections 2,
6 and 7): planner-1, the authority stub, arm-02 (receives the goal, gives custody) and amr-07
(takes on the sub-goal, receives custody).

Each is a deterministic state machine. Goal states follow RFC-001 9.2, custody states 10.4,
commitment states Core 5.8. A participant never acts on a message alone: every action checks
local authorization, the commitment and the reservation first (spec 6.2 rule 2).
"""

from __future__ import annotations
from decimal import Decimal

from ..core import descriptor_is_current, time_ms, verify_sim
from .engine import Reject, Simulation
from .evidence import CONTRADICTED, MET, evaluate
from .scenario import ms, time_text

COMMITMENT_ID = "rl:commit:c42-transport"  # spec 5.2
TERMINAL = {"REJECTED", "DENIED", "SUCCEEDED", "FAILED", "CANCELLED", "TIMED_OUT", "UNKNOWN"}
_STATUS = {"succeeded": "SUCCEEDED", "failed": "FAILED", "unknown": "UNKNOWN", "rejected": "REJECTED"}


def _reasons(*codes: str) -> list[dict]:
    return [{"code": code} for code in codes]


def _outcome(goal: str, status: str, reporter: str, now: str, reasons=(), verification=None) -> dict:
    return {"kind": "outcome", "goal": goal, "status": status, "verification": verification,
            "reasons": _reasons(*reasons), "reported_by": reporter, "reported_at": now}


class Participant:
    def __init__(self, sim: Simulation, pid: str):
        self.sim = sim
        self.id = pid
        self.config = sim.participants_config[pid]

    def start(self) -> None:
        pass

    def handle(self, document: dict) -> None:
        handler = getattr(self, f"on_{document['type']}", None)
        if handler is None:
            raise Reject("C306")  # this participant has no transition for that message
        handler(document)

    def send(self, mtype: str, profile: str, recipient: str, payload: dict, reply: dict | None = None):
        return self.sim.send(self.id, mtype, profile, recipient, payload,
                             reply["message_id"] if reply else None)

    def after(self, delay_ms: int, callback, *args) -> None:
        self.sim.after(delay_ms, self.id, callback, *args)

    def at(self, t_ms: int, callback, *args) -> None:
        self.sim.at(t_ms, self.id, callback, *args)

    def event(self, kind: str, **detail) -> None:
        self.sim.event(kind, self.id, **detail)

    def domain_members(self) -> frozenset[str]:
        domain = self.config["authority_domain"]
        return frozenset(p for p, c in self.sim.participants_config.items() if c["authority_domain"] == domain)

    def evaluate_all(self, criteria: list[dict], observations: list[dict]):
        return [evaluate(c, observations, self.sim.now, self.sim.tolerance_ms, self.domain_members()) for c in criteria]


# -------- configuration lookups --------

def capability(sim: Simulation, robot: str, kind: str) -> dict | None:
    """The robot's declared capability of `kind`, if its descriptor is current, signed and lists it."""
    descriptor = sim.scenario["descriptors"].get(robot)
    if descriptor is None or not descriptor_is_current(descriptor, sim.now, sim.tolerance_ms):
        return None
    key = sim.keys.get(descriptor["signature"]["key"])
    if key is None or key.participant != robot or not verify_sim(descriptor, key) or kind in descriptor["unsupported"]:
        return None
    return next((c for c in descriptor["capabilities"] if c["kind"] == kind), None)


def standing_authorization(sim: Simulation, robot: str, profile: str, classes: set[str], places: set[str],
                           counterpart: str, mass: str) -> str | None:
    """The ID of the robot's standing authorization if it covers the goal (spec 7.2), else None."""
    sa = sim.scenario["standing_authorizations"].get(robot)
    if sa is None:
        return None
    key = sim.keys.get(sa["signature"]["key"])
    if key is None or key.participant != sa["issued_by"] or sa["issued_by"] != sim.scenario["authorities"].get(robot):
        return None
    if not verify_sim(sa, key) or sa["grantee"] != robot or sa["mode"] != "simulation":
        return None
    lease = sa["lease"]
    if not time_ms(lease["granted_at"]) <= sim.now < time_ms(lease["expires_at"]) - sim.tolerance_ms:
        return None
    scope = sa["scope"]
    within = {p: sim.scenario["places"].get(p, {}).get("within") for p in places}
    counterpart_domain = sim.participants_config[counterpart]["authority_domain"]
    covered = (
        profile in scope["profiles"]
        and classes <= set(scope["goal_classes"])
        and all(p in scope["places"] or within[p] in scope["places"] for p in places)
        and counterpart_domain in scope["counterparts"]
        and Decimal(mass) <= Decimal(scope["max_mass"]["value"])
    )
    return sa["id"] if covered else None


def duration(sim: Simulation, action: str, start: str | None = None, end: str | None = None) -> int:
    for entry in sim.scenario["durations"]:
        if entry["action"] == action and entry.get("from") == start and entry.get("to") == end:
            return ms(entry["duration"])
    raise KeyError(f"no duration for {action} {start} -> {end}")


def thing_mass(things: list[dict], thing: str) -> str:
    return next(t["mass"]["value"] for t in things if t["id"] == thing)


# -------- planner-1 --------

class Planner(Participant):
    status: str | None = None

    def start(self) -> None:
        self.at(self.sim.epoch_ms, self._propose)

    def _propose(self) -> None:
        self.goal = self.sim.scenario["goal"]
        self.send("propose_goal", "a2r", self.sim.scenario["goal_recipient"],
                  {"goal": self.goal, "things": self.sim.scenario["things"]})

    def _record(self, status: str, reason: str | None = None) -> None:
        if self.status is None:
            self.status = status
            self.event("state", record="planner_goal", status=status, reason=reason)

    def on_assessment(self, document: dict) -> None:
        self.event("state", record="assessment", result=document["payload"]["result"])

    def on_accept(self, document: dict) -> None:
        self.event("state", record="accepted")

    def on_reject(self, document: dict) -> None:
        self._record("rejected")

    def on_outcome(self, document: dict) -> None:
        outcome = document["payload"]["outcome"]
        if outcome["goal"] != self.goal["id"]:
            raise Reject("C306")
        if outcome["status"] != "succeeded" or not self.sim.scenario["policies"][self.id]["reverify_outcomes"]:
            return self._record(outcome["status"])
        results = self.evaluate_all(self.goal["success_criteria"], document["payload"]["evidence"])
        failed = next((r for r in results if r.result != MET), None)
        if failed is None:
            return self._record("succeeded")
        self._record("failed" if failed.result == CONTRADICTED else "unknown", failed.reason)


# -------- the authority stub --------

class Authority(Participant):
    def on_authorization_request(self, document: dict) -> None:
        self.after(self.sim.authority_delay_ms, self._decide, document)

    def _decide(self, request: dict) -> None:
        goal = request["payload"]["goal"]["id"]
        decision = next(
            (row["decision"] for row in self.sim.scenario["authority_table"]
             if row["grantee"] == request["sender"] and row["goal"] == goal),
            "denied",  # a request not in the table is denied (spec 7.3)
        )
        lease_end = time_text(self.sim.epoch_ms)[:11] + "11:00:00.000Z"
        self.send("authorization_decision", "a2r", request["sender"], {
            "goal": goal, "decision": decision, "mode": "simulation", "constraints": [],
            "lease": {"granted_at": self.sim.time(), "expires_at": lease_end, "renewal": 0},
        }, reply=request)


# -------- arm-02: receives the goal, gives custody --------

class Arm(Participant):
    def __init__(self, sim: Simulation, pid: str):
        super().__init__(sim, pid)
        self.goal_state: str | None = None
        holder = sim.scenario["initial_custody"]
        self.custody: str | None = "HOLDING" if self.id in holder.values() else None
        self.goal = self.proposal = self.peer = None
        self.commitment_state: str | None = None
        self.reservation_until: int | None = None
        self.peer_ready = False
        self.transfer_id = self.release_id = None
        self.confirmation_reason: str | None = None
        self.handoff = next(place for place, arbiter in sim.scenario["arbiters"].items() if arbiter == pid)

    def _state(self, state: str) -> None:
        self.goal_state = state
        self.event("state", record="goal", state=state)

    def _custody(self, state: str) -> None:
        self.custody = state
        self.event("state", record="custody", state=state)

    @property
    def terminal(self) -> bool:
        return self.goal_state in TERMINAL

    # -- assessment --

    def on_propose_goal(self, document: dict) -> None:
        if self.goal is not None:
            raise Reject("C306")
        self.proposal, self.goal = document, document["payload"]["goal"]
        self.things = document["payload"]["things"]
        self.planner = document["sender"]
        self.thing = self.goal["desired_state"][0]["subject"]
        self.target = self.goal["desired_state"][0]["value"]
        self.mass = thing_mass(self.things, self.thing)
        self._state("ASSESSING")
        grasp = capability(self.sim, self.id, "grasp")
        if grasp is None:
            return self._decline_goal("capability_unsupported")
        if Decimal(self.mass) > Decimal(grasp["max_mass"]["value"]) or \
                self.sim.world.get(self.thing, "located_in") not in grasp["reach"]:
            return self._decline_goal("capability_insufficient")
        self.peer = self._find_peer()
        if self.peer is None:
            return self._decline_goal("assistance_unavailable")
        self.send("assessment", "a2r", self.planner, {
            "goal": self.goal["id"], "result": "feasible_with_assistance", "reasons": _reasons("assistance_required"),
        }, reply=document)
        self.sub_goal = dict(self.goal, id=self.goal["id"] + "-sub-1", requested_by=self.id)
        self.send("request_assistance", "r2r", self.peer,
                  {"goal": self.sub_goal, "things": self.things, "parent_goal": self.goal["id"]})

    def _find_peer(self) -> str | None:
        for peer in self.sim.scenario["peers"].get(self.id, []):
            transport = capability(self.sim, peer, "transport_payload")
            dock = capability(self.sim, peer, "dock_at")
            if transport and dock and Decimal(self.mass) <= Decimal(transport["max_mass"]["value"]) \
                    and {self.handoff, self.target} <= set(dock["places"]):
                return peer
        return None

    def _decline_goal(self, *reasons: str) -> None:
        self.send("assessment", "a2r", self.planner,
                  {"goal": self.goal["id"], "result": "infeasible", "reasons": _reasons(*reasons)}, reply=self.proposal)
        self.send("reject", "a2r", self.planner, {"goal": self.goal["id"], "reasons": _reasons(*reasons)},
                  reply=self.proposal)
        self._state("REJECTED")

    # -- assistance and commitment --

    def on_offer_assistance(self, document: dict) -> None:
        payload = document["payload"]
        if self.goal_state != "ASSESSING" or document["sender"] != self.peer or payload["goal"] != self.sub_goal["id"]:
            raise Reject("C306")
        self.commitment = {
            "kind": "commitment", "id": COMMITMENT_ID, "debtor": self.peer, "creditor": self.id,
            "obligation": {"goal": self.sub_goal["id"]}, "conditions": payload["conditions"],
            "lease": payload["lease"], "state": "active",
        }
        self.send("commit", "r2r", self.peer, {"commitment": self.commitment})
        self.commitment_state = "active"
        self.at(time_ms(payload["lease"]["expires_at"]), self._lease_expired)

    def on_decline(self, document: dict) -> None:
        if self.goal_state != "ASSESSING":
            raise Reject("C306")
        self._decline_goal("assistance_unavailable")

    def on_reserve(self, document: dict) -> None:
        payload = document["payload"]
        if self.commitment_state != "active" or payload["resource"] != self.handoff \
                or payload["commitment"] != COMMITMENT_ID or self.reservation_until is not None:
            raise Reject("C306")
        until = time_ms(payload["lease"]["expires_at"])
        granted = until <= time_ms(self.commitment["lease"]["expires_at"])
        self.send("reservation_decision", "r2r", document["sender"], {
            "resource": self.handoff, "decision": "granted" if granted else "denied",
            "lease": {"granted_at": self.sim.time(), "expires_at": payload["lease"]["expires_at"], "renewal": 0},
        }, reply=document)
        if not granted:
            return
        self.reservation_until = until
        self.at(until, self._reservation_expired)
        self.send("accept", "a2r", self.planner, {"goal": self.goal["id"]})
        self._state("ACCEPTED")
        self._authorize()

    def _reservation_expired(self) -> None:
        self.event("lease_expired", lease="reservation", resource=self.handoff)

    # -- authorization --

    def _authorize(self) -> None:
        self._state("AUTHORIZING")
        basis = standing_authorization(self.sim, self.id, "a2r", {"custody_transfer"},
                                       {self.sim.world.get(self.thing, "located_in"), self.handoff},
                                       self.planner, self.mass)
        if basis is not None:
            self.event("authorization", decision="granted", basis=basis)
            self._state("AUTHORIZED")
            return self._try_handover()
        self.send("authorization_request", "a2r", self.sim.scenario["authorities"][self.id], {
            "goal": self.goal, "things": self.things, "mode": "simulation",
            "reasons": _reasons("outside_standing_authorization"),
        })

    def on_authorization_decision(self, document: dict) -> None:
        payload = document["payload"]
        if self.goal_state != "AUTHORIZING" or payload["goal"] != self.goal["id"]:
            raise Reject("C306")
        self.event("authorization", decision=payload["decision"], basis=document["message_id"])
        if payload["decision"] == "granted" and payload["mode"] == "simulation":
            self._state("AUTHORIZED")
            return self._try_handover()
        self._state("DENIED")
        self._report("rejected", "authorization_denied")

    # -- handover --

    def on_coordinate(self, document: dict) -> None:
        if document["sender"] != self.peer or document["payload"]["phase"] != "ready_for_handover":
            raise Reject("C306")
        self.peer_ready = True
        self._try_handover()

    def _try_handover(self) -> None:
        ready = (
            self.goal_state == "AUTHORIZED" and self.peer_ready and self.custody == "HOLDING"
            and self.commitment_state == "active"
            and self.reservation_until is not None and self.sim.now < self.reservation_until
        )
        if not ready:
            return
        self._state("RUNNING")
        self.send("coordinate", "r2r", self.peer, {"commitment": COMMITMENT_ID, "phase": "ready_for_handover"})
        transfer = self.send("transfer_custody", "r2r", self.peer, {
            "thing": self.thing, "commitment": COMMITMENT_ID, "criteria": self.sim.scenario["handover_criteria"],
        })
        self.transfer_id = transfer["message_id"]
        self._custody("OFFERED")

    def on_acknowledge(self, document: dict) -> None:
        payload = document["payload"]
        if payload["acknowledged"] == self.transfer_id and self.custody == "OFFERED":
            if not payload["accepted"]:
                return self._custody("HOLDING")
            self._custody("AWAITING_CONFIRMATION")
            return self._place()
        if payload["acknowledged"] == self.release_id and self.custody in ("RELEASE_SENT", "IN_DOUBT"):
            return self._custody("RELEASED")
        raise Reject("C306")

    def _place(self) -> None:
        if self.sim.world.get(self.peer, "located_in") != self.handoff or self.commitment_state != "active":
            return self.event("state", record="placement_refused")
        self.sim.start_action(
            self.id, "handover_place", duration(self.sim, "handover_place"),
            lambda: self.sim.world.set(self.thing, "carried_by", self.peer),
            lambda: self.send("coordinate", "r2r", self.peer, {"commitment": COMMITMENT_ID, "phase": "placed"}),
            thing=self.thing, onto=self.peer,
        )

    def on_custody_confirmed(self, document: dict) -> None:
        if self.custody != "AWAITING_CONFIRMATION" or document["payload"]["thing"] != self.thing:
            raise Reject("C306")
        results = self.evaluate_all(self.sim.scenario["handover_criteria"], document["payload"]["observations"])
        own = self.sim.observe(self.id, "cell_camera", self.thing, "carried_by")
        unmet = next((r for r in results if r.result != MET), None)
        if unmet is not None or own is None or own["value"] != self.peer:
            self.confirmation_reason = unmet.reason if unmet else "custody_unknown"
            return self.event("state", record="confirmation_not_accepted", reason=self.confirmation_reason)
        release = self.send("custody_released", "r2r", self.peer,
                            {"thing": self.thing, "confirmation": document["message_id"]}, reply=document)
        self.release_id = release["message_id"]
        self._custody("RELEASE_SENT")
        self.after(self.sim.ack_timeout_ms, self._ack_timeout)

    def _ack_timeout(self) -> None:
        if self.custody == "RELEASE_SENT":
            self._custody("IN_DOUBT")
            self._query()

    def _query(self) -> None:
        if self.custody != "IN_DOUBT" or self.commitment_state != "active":
            return
        self.send("custody_query", "r2r", self.peer, {"thing": self.thing})
        self.after(self.sim.query_interval_ms, self._query)

    def on_custody_state(self, document: dict) -> None:
        if self.custody != "IN_DOUBT":
            raise Reject("C306")
        accepted = document["payload"]["accepted_release"]
        self._custody("RELEASED" if accepted == self.release_id else "HOLDING")

    # -- outcomes --

    def on_outcome(self, document: dict) -> None:
        outcome = document["payload"]["outcome"]
        if document["sender"] != self.peer or outcome["goal"] != self.sub_goal["id"]:
            raise Reject("C306")
        if self.commitment_state == "active":
            self.commitment_state = "fulfilled" if outcome["status"] == "succeeded" else "failed"
            self.event("state", record="commitment", state=self.commitment_state)
        if self.terminal:
            return
        if self.goal_state == "ASSESSING":
            return self._decline_goal("assistance_unavailable")
        if outcome["status"] in ("rejected", "failed", "cancelled", "timed_out"):
            return self._report("failed", "assistance_unavailable")
        evidence = document["payload"]["evidence"]
        results = self.evaluate_all(self.goal["success_criteria"], evidence)
        if all(r.result == MET for r in results):
            cited = {i for r in results for i in r.evidence}
            verification = {
                "verified_by": self.id,
                "method": "criteria_evaluated" if any(o["source"]["participant"] != self.id for o in evidence
                                                      if o["id"] in cited) else "self_reported",
                "evaluated_at": self.sim.time(),
                "criteria_results": [{"criterion": i, "met": True, "evidence": list(r.evidence)}
                                     for i, r in enumerate(results)],
            }
            return self._report("succeeded", verification=verification,
                                evidence=[o for o in evidence if o["id"] in cited])
        failed = next(r for r in results if r.result != MET)
        self._report("failed" if failed.result == CONTRADICTED else "unknown", failed.reason)

    def _lease_expired(self) -> None:
        if self.commitment_state != "active":
            return
        self.commitment_state = "lapsed"
        self.event("lease_expired", lease="commitment", commitment=COMMITMENT_ID)
        if self.terminal:
            return
        if self.goal_state == "ASSESSING":
            return self._decline_goal("assistance_unavailable", "lease_expired")
        if self.custody == "HOLDING":
            return self._report("failed", "lease_expired")  # C42 never left arm-02
        if self.custody in ("OFFERED", "AWAITING_CONFIRMATION"):
            self._custody("TIMED_OUT")
            reasons = [r for r in (self.confirmation_reason, "lease_expired") if r]
            return self._report("failed", *reasons)  # arm-02 sees C42 at H1, not at S3
        if self.custody in ("RELEASE_SENT", "IN_DOUBT"):
            self.event("escalated", reason="custody_unknown")
            return self._report("unknown", "custody_unknown", "lease_expired")
        self._report("unknown", "lease_expired")

    def _report(self, status: str, *reasons: str, verification=None, evidence=()) -> None:
        self.send("outcome", "a2r", self.planner, {
            "outcome": _outcome(self.goal["id"], status, self.id, self.sim.time(), reasons, verification),
            "evidence": list(evidence),
        })
        self._state(_STATUS[status] if status != "rejected" else "DENIED")


# -------- amr-07: takes on the sub-goal, receives custody --------

class Amr(Participant):
    def __init__(self, sim: Simulation, pid: str):
        super().__init__(sim, pid)
        self.goal_state: str | None = None
        self.custody: str | None = None
        self.goal = self.request = self.commitment = None
        self.commitment_state: str | None = None
        self.confirmation_id = self.release_id = None

    def _state(self, state: str) -> None:
        self.goal_state = state
        self.event("state", record="goal", state=state)

    def _custody(self, state: str) -> None:
        self.custody = state
        self.event("state", record="custody", state=state)

    def on_request_assistance(self, document: dict) -> None:
        if self.goal is not None:
            raise Reject("C306")
        self.request, self.goal = document, document["payload"]["goal"]
        self.requester = document["sender"]
        self.things = document["payload"]["things"]
        self.thing = self.goal["desired_state"][0]["subject"]
        self.target = self.goal["desired_state"][0]["value"]
        self.mass = thing_mass(self.things, self.thing)
        self.handoff = next(p for p, arbiter in self.sim.scenario["arbiters"].items() if arbiter == self.requester)
        self._state("ASSESSING")
        if not self.sim.scenario["policies"][self.id]["accept_assistance"]:
            return self._decline("participant_busy")
        transport = capability(self.sim, self.id, "transport_payload")
        dock = capability(self.sim, self.id, "dock_at")
        if not (transport and dock and Decimal(self.mass) <= Decimal(transport["max_mass"]["value"])
                and {self.handoff, self.target} <= set(dock["places"])):
            return self._decline("capability_insufficient")
        self.send("offer_assistance", "r2r", self.requester, {
            "goal": self.goal["id"], "conditions": [{"kind": "resource_held", "resource": self.handoff}],
            "lease": {"granted_at": self.sim.time(), "expires_at": self.sim.scenario["parameters"]["offer_lease_end"],
                      "renewal": 0},
        }, reply=document)

    def _decline(self, *reasons: str) -> None:
        self.send("decline", "r2r", self.requester, {"goal": self.goal["id"], "reasons": _reasons(*reasons)},
                  reply=self.request)
        self._state("REJECTED")

    def on_commit(self, document: dict) -> None:
        commitment = document["payload"]["commitment"]
        if self.goal_state != "ASSESSING" or commitment["debtor"] != self.id \
                or commitment["obligation"]["goal"] != self.goal["id"]:
            raise Reject("C306")
        self.commitment, self.commitment_state = commitment, "active"
        self._state("ACCEPTED")
        self.at(time_ms(commitment["lease"]["expires_at"]), self._lease_expired)
        self._authorize()

    def _authorize(self) -> None:
        self._state("AUTHORIZING")
        basis = standing_authorization(self.sim, self.id, "r2r", {"transport_payload", "custody_transfer"},
                                       {self.handoff, self.target}, self.requester, self.mass)
        if basis is not None:
            self.event("authorization", decision="granted", basis=basis)
            self._state("AUTHORIZED")
            return self._reserve()
        self.send("authorization_request", "a2r", self.sim.scenario["authorities"][self.id], {
            "goal": self.goal, "things": self.things, "mode": "simulation",
            "reasons": _reasons("outside_standing_authorization"),
        })

    def on_authorization_decision(self, document: dict) -> None:
        payload = document["payload"]
        if self.goal_state != "AUTHORIZING" or payload["goal"] != self.goal["id"]:
            raise Reject("C306")
        self.event("authorization", decision=payload["decision"], basis=document["message_id"])
        if payload["decision"] == "granted" and payload["mode"] == "simulation":
            self._state("AUTHORIZED")
            return self._reserve()
        self._state("DENIED")
        self.commitment_state = "failed"
        self._outcome("rejected", "authorization_denied")

    def _reserve(self) -> None:
        self.send("reserve", "r2r", self.sim.scenario["arbiters"][self.handoff], {
            "resource": self.handoff, "commitment": self.commitment["id"],
            "lease": {"granted_at": self.sim.time(), "expires_at": self.sim.scenario["parameters"]["reservation_end"],
                      "renewal": 0},
        })

    def on_reservation_decision(self, document: dict) -> None:
        payload = document["payload"]
        if self.goal_state != "AUTHORIZED" or payload["resource"] != self.handoff:
            raise Reject("C306")
        if payload["decision"] != "granted":
            return self.event("state", record="reservation_denied")
        self._state("RUNNING")
        self._move(self.handoff, lambda: self.send(
            "coordinate", "r2r", self.requester, {"commitment": self.commitment["id"], "phase": "ready_for_handover"}))

    def _move(self, to: str, done) -> None:
        if self.goal_state != "RUNNING" or self.commitment_state != "active":
            return self.event("state", record="motion_refused", to=to)
        self.sim.move(self.id, to, done)  # the world and the durations are the simulation's

    def on_coordinate(self, document: dict) -> None:
        phase = document["payload"]["phase"]
        if phase == "ready_for_handover":
            return self.event("state", record="peer_ready")
        if self.custody != "ACCEPTED":
            raise Reject("C306")
        observation = self.sim.observe(self.id, "deck_camera", self.thing, "carried_by")
        if observation is None:
            return self.event("state", record="no_confirmation_evidence")
        confirmation = self.send("custody_confirmed", "r2r", self.requester,
                                 {"thing": self.thing, "observations": [observation]})
        self.confirmation_id = confirmation["message_id"]
        self._custody("CONFIRMATION_SENT")

    def on_transfer_custody(self, document: dict) -> None:
        payload = document["payload"]
        if self.goal_state != "RUNNING" or payload["commitment"] != self.commitment["id"] \
                or payload["thing"] != self.thing or self.custody is not None:
            raise Reject("C306")
        self._custody("ACKNOWLEDGED")
        self.send("acknowledge", "r2r", self.requester,
                  {"acknowledged": document["message_id"], "accepted": True}, reply=document)
        self._custody("ACCEPTED")

    def on_custody_released(self, document: dict) -> None:
        payload = document["payload"]
        if payload["confirmation"] != self.confirmation_id or payload["thing"] != self.thing:
            raise Reject("C403")
        if self.custody != "CONFIRMATION_SENT":
            raise Reject("C306")
        # Commit point (RFC-001 10.4 rule 3): accepted durably before acknowledging.
        self.release_id = document["message_id"]
        self._custody("HOLDING")
        self.event("custody_committed", thing=self.thing, release=self.release_id)
        self.send("acknowledge", "r2r", self.requester,
                  {"acknowledged": document["message_id"], "accepted": True}, reply=document)
        if self.sim.observe(self.id, "deck_camera", self.thing, "carried_by") is None:  # rule 9
            return self.event("state", record="physical_custody_unknown")
        self._move(self.target, self._arrived)

    def _arrived(self) -> None:
        observation = self.sim.observe(self.id, "deck_camera", self.thing, "located_in")
        evidence = [observation] if observation else []
        results = self.evaluate_all(self.goal["success_criteria"], evidence)
        met = all(r.result == MET for r in results)
        self.commitment_state = "fulfilled" if met else "failed"  # Core 5.8 local states
        if met:
            verification = {
                "verified_by": self.id, "method": "self_reported", "evaluated_at": self.sim.time(),
                "criteria_results": [{"criterion": i, "met": True, "evidence": list(r.evidence)}
                                     for i, r in enumerate(results)],
            }
            self._state("SUCCEEDED")
            return self._outcome("succeeded", verification=verification, evidence=evidence)
        failed = next(r for r in results if r.result != MET)
        self._state("UNKNOWN" if failed.result != CONTRADICTED else "FAILED")
        self._outcome("unknown" if failed.result != CONTRADICTED else "failed", failed.reason)

    def on_custody_query(self, document: dict) -> None:
        if document["payload"]["thing"] != self.thing:
            raise Reject("C306")
        observation = self.sim.observe(self.id, "deck_camera", self.thing, "carried_by")
        self.send("custody_state", "r2r", self.requester, {
            "thing": self.thing, "accepted_release": self.release_id if self.custody == "HOLDING" else None,
            "observations": [observation] if observation else [],
        }, reply=document)

    def _lease_expired(self) -> None:
        if self.commitment_state != "active":
            return
        self.commitment_state = "lapsed"
        self.event("lease_expired", lease="commitment", commitment=self.commitment["id"])
        if self.custody in ("ACKNOWLEDGED", "ACCEPTED", "CONFIRMATION_SENT"):
            self._custody("TIMED_OUT")  # never accepted a release: rule 11
            self.event("escalated", reason="custody_unknown")
            self._state("FAILED")
            return self._outcome("failed", "lease_expired", "custody_unknown")
        if self.custody == "HOLDING":
            self._state("UNKNOWN")
            return self._outcome("unknown", "lease_expired")
        self._state("FAILED")
        self._outcome("failed", "lease_expired")

    def _outcome(self, status: str, *reasons: str, verification=None, evidence=()) -> None:
        self.send("outcome", "r2r", self.requester, {
            "outcome": _outcome(self.goal["id"], status, self.id, self.sim.time(), reasons, verification),
            "evidence": list(evidence),
        })


def build(sim: Simulation) -> dict[str, Participant]:
    """One participant object per scenario participant, by role."""
    goal = sim.scenario["goal"]
    recipient = sim.scenario["goal_recipient"]
    authorities = set(sim.scenario["authorities"].values())
    out: dict[str, Participant] = {}
    for pid, config in sim.participants_config.items():
        if pid == goal["requested_by"]:
            out[pid] = Planner(sim, pid)
        elif pid in authorities:
            out[pid] = Authority(sim, pid)
        elif pid == recipient:
            out[pid] = Arm(sim, pid)
        elif config["kind"] == "robot":
            out[pid] = Amr(sim, pid)
        else:
            raise ValueError(f"no role for participant {pid}")
    return out
