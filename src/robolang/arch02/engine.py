"""
The reference simulation engine (spec/reference-simulation-v0.1.md sections 3, 5, 8 and 10).

One shared simulated clock, an event queue ordered by time and then by insertion, message
delivery with a fixed latency, and the scripted faults. Every delivery gets a receipt: the
per-message Core 0.1 checks of `robolang.core`, then duplicate and order handling, then the
participant's own state rules (`C306`, `C403`).

Order follows decision record 0008: sequence numbers count per sender, recipient and
conversation; a message after a gap is held, with no effect, until the gap is filled or
`max_hold` has passed; a message that expires while held is rejected (`C301`).

SIMULATION ONLY. Nothing here opens a connection, loads an adapter or moves a robot: the
world is symbolic and every "motion" is a recorded event (decision R8).
"""

from __future__ import annotations
import hashlib
import heapq
import json
from dataclasses import dataclass
from typing import Any, Callable

from ..core import Context, Key, check_message, sign_sim, time_ms
from .scenario import apply_overrides, keys_of, ms, time_text
from .world import Fact, Sensor, World


class Reject(Exception):
    """A participant refuses a message that passed the per-message checks (`C306`, `C403`)."""

    def __init__(self, code: str):
        super().__init__(code)
        self.code = code


@dataclass(frozen=True)
class SentMessage:
    index: int
    t_sent_ms: int
    document: dict
    fate: str  # delivered, dropped, injected


@dataclass(frozen=True)
class Delivery:
    t_ms: int
    message: int  # index into the sent messages
    receiver: str
    kind: str  # original, duplicate, replay, resend, released (a held message leaving the hold)
    receipt: str  # accepted, duplicate, held, rejected, not_handled
    code: str | None


@dataclass(frozen=True)
class Event:
    t_ms: int
    kind: str  # motion, effect, observation, state, authorization, lease_expired, escalated, custody_committed
    participant: str
    detail: tuple[tuple[str, Any], ...]

    def get(self, key: str, default=None):
        return dict(self.detail).get(key, default)


@dataclass(frozen=True)
class RunResult:
    """Everything the case expectations check; ARCH-3 serializes it as the run record."""

    case: str
    seed: str
    sent: tuple[SentMessage, ...]
    deliveries: tuple[Delivery, ...]
    events: tuple[Event, ...]
    planner_status: str | None
    goal_states: tuple[tuple[str, str], ...]  # (participant, final state of its goal)
    custody: tuple[tuple[str, str], ...]  # (participant, final local custody state)
    commitments: tuple[tuple[str, str], ...]  # (participant, final local commitment state)
    designated_custody: tuple[tuple[str, str], ...]  # (thing, responsible participant)
    final_world: tuple[Fact, ...]
    end_t_ms: int


def _message_id(sender: str, recipient: str, conversation: str, sequence: int) -> str:
    """Spec 5.2 as amended by decision record 0008, Q7: the recipient is part of the ID."""
    text = f"robolang-arch02-msg/0.1-seq\n{sender}\n{recipient}\n{conversation}\n{sequence}"
    return "m-" + hashlib.sha256(text.encode("utf-8")).hexdigest()[:32]


def build_message(sender: str, recipient: str, conversation: str, sequence: int, mtype: str, profile: str,
                  payload: dict, in_reply_to: str | None, now_ms: int, ttl_ms: int, key: Key,
                  message_id: str | None = None) -> dict:
    """One signed Core 0.1 message as the reference participants build it (spec 5.2)."""
    return sign_sim({
        "schema": "robolang.core.message", "schema_version": "0.1",
        "message_id": message_id or _message_id(sender, recipient, conversation, sequence),
        "conversation_id": conversation, "sequence": sequence, "in_reply_to": in_reply_to,
        "profile": profile, "type": mtype, "sender": sender, "recipient": recipient,
        "sent_at": time_text(now_ms), "expires_at": time_text(now_ms + ttl_ms), "payload": payload,
    }, f"{sender}#k1", key)


def build_observation(participant: str, number: int, thing: str, prop: str, value: Any, observed_at_ms: int,
                      sensor_name: str, valid_for: dict, confidence: str, method: str, key: Key) -> dict:
    """One signed observation (Core 5.6) made by `participant`, its `number`-th."""
    return sign_sim({
        "kind": "observation", "id": f"rl:obs:{participant.split(':')[-1]}:{number}", "subject": thing,
        "property": prop, "value": value, "observed_at": time_text(observed_at_ms),
        "valid_for": valid_for, "confidence": confidence,
        "source": {"participant": participant, "sensor": sensor_name, "method": method},
    }, f"{participant}#k1", key)


@dataclass
class _Held:
    """A message waiting for a gap in its stream (Core 8.4); it has had no effect yet."""

    document: dict
    ref: Any  # how the host identifies the delivered message (the engine: its index)
    held_at_ms: int


class Inbox:
    """
    One receiver's handling of a delivered message: the Core 0.1 checks of `robolang.core`, then
    duplicates, order and holds (decision record 0008), then the participant's own rules.

    The host supplies time, timers and the participant: the engine for its in-process
    participants, and the participant process of the `robolang-sim-stdio` binding for itself.
    It provides `now`, `tolerance_ms`, `max_hold_ms`, `at`, `context`, `receipt`, `resend`,
    `is_silent` and `hand_over`.
    """

    def __init__(self, host, receiver: str):
        self.host = host
        self.receiver = receiver
        self._processed: dict[tuple[str, str], int] = {}  # (sender, message_id) -> remembered until
        self._next: dict[str, int] = {}  # sender -> next expected sequence
        self._held: dict[str, dict[int, _Held]] = {}  # sender -> sequence -> held

    def _record(self, ref, kind: str, receipt: str, code: str | None = None) -> None:
        self.host.receipt(ref, self.receiver, kind, receipt, code)

    def deliver(self, data: bytes, ref, kind: str) -> None:
        host = self.host
        receipt = check_message(data, host.context(self.receiver))
        if not receipt.accepted:
            return self._record(ref, kind, "rejected", receipt.code)
        # Only a message that passed form, signature, freshness and recipient reaches this point:
        # a rejected one is never held and does not count in its stream (0008, Q2).
        document = json.loads(data)
        sender, message_id = document["sender"], document["message_id"]
        held = self._held.setdefault(sender, {})
        if ((sender, message_id) in self._processed and host.now <= self._processed[(sender, message_id)]) or any(
            h.document["message_id"] == message_id for h in held.values()
        ):
            self._record(ref, kind, "duplicate")  # Q5: processed or held already; no second effect
            host.resend(message_id)  # the original reply is sent again
            return
        sequence = document["sequence"]
        expected = self._next.get(sender, 1)
        if sequence < expected:
            return self._record(ref, kind, "rejected", "C306")  # Q6: stale and not a duplicate
        if sequence > expected:
            # Q3: held, with no effect and no reply, until the gap is filled, `max_hold` has
            # passed (Q4) or the message expires (judged in the safe direction).
            held[sequence] = _Held(document, ref, host.now)
            self._record(ref, kind, "held")
            expires_safe_ms = time_ms(document["expires_at"]) - host.tolerance_ms
            host.at(expires_safe_ms, None, self._check_held, sender, sequence)
            host.at(host.now + host.max_hold_ms, None, self._check_held, sender, sequence)
            return
        self._process(document, ref, kind)
        self._drain(sender)

    def _process(self, document: dict, ref, kind: str) -> None:
        """Hand a message that is next in its stream to the participant; the stream advances."""
        sender = document["sender"]
        self._next[sender] = document["sequence"] + 1
        if self.host.is_silent(self.receiver):
            return self._record(ref, kind, "not_handled")
        try:
            self.host.hand_over(self.receiver, document)
        except Reject as rejection:
            return self._record(ref, kind, "rejected", rejection.code)
        expiry = time_ms(document["expires_at"]) + self.host.tolerance_ms
        self._processed[(sender, document["message_id"])] = expiry
        self._record(ref, kind, "accepted")

    def _drain(self, sender: str) -> None:
        """Process the held messages that the stream has now reached, in sequence order."""
        held = self._held.get(sender, {})
        while self._next.get(sender, 1) in held:
            entry = held.pop(self._next[sender])
            self._process(entry.document, entry.ref, "released")

    def _check_held(self, sender: str, sequence: int) -> None:
        """End a hold that expired (`C301`) or reached `max_hold` (the gap is skipped)."""
        entry = self._held.get(sender, {}).get(sequence)
        if entry is None:
            return  # already released
        now = self.host.now
        if now >= time_ms(entry.document["expires_at"]) - self.host.tolerance_ms:
            del self._held[sender][sequence]
            self._record(entry.ref, "released", "rejected", "C301")
            self._next[sender] = max(self._next.get(sender, 1), sequence + 1)
        elif now >= entry.held_at_ms + self.host.max_hold_ms:
            del self._held[sender][sequence]
            self._process(entry.document, entry.ref, "released")
        else:
            return
        self._drain(sender)


def _matches(rule: dict, document: dict, count: int) -> bool:
    return rule["type"] == document["type"] and rule["sender"] == document["sender"] and rule["n"] == count


@dataclass
class _Partition:
    between: frozenset[str]
    start_ms: int
    until_ms: int


class Simulation:
    """
    `external` maps a participant ID to a factory `(simulation, participant) -> proxy` that
    replaces the in-process participant: the proxy is that participant (`start`, final states)
    and its inbox (`deliver`). INTEROP-1 uses it for the `robolang-sim-stdio` binding.
    """

    def __init__(self, scenario: dict, case: dict, external: dict[str, Callable] | None = None):
        self.case = case
        self.scenario = apply_overrides(scenario, case["faults"])
        p = self.scenario["parameters"]
        self.epoch_ms = time_ms(p["epoch"])
        self.now = self.epoch_ms
        self.latency_ms = ms(p["latency"])
        self.ttl_ms = ms(p["message_ttl"])
        self.max_lifetime_ms = ms(p["max_message_lifetime"])
        self.tolerance_ms = ms(p["clock_tolerance"])
        self.ack_timeout_ms = ms(p["ack_timeout"])
        self.query_interval_ms = ms(p["query_interval"])
        self.authority_delay_ms = ms(p["authority_delay"])
        self.max_hold_ms = ms(p["max_hold"])
        self.conversation = self.scenario["conversation_id"]
        self.keys: dict[str, Key] = keys_of(self.scenario)
        self.world = World(self.scenario["initial_facts"])
        self.participants_config = {entry["id"]: entry for entry in self.scenario["participants"]}
        self.sensors: dict[tuple[str, str], Sensor] = {}
        for s in self.scenario["sensors"]:
            self.sensors[(s["participant"], s["sensor"])] = Sensor(
                s["participant"], s["sensor"], s["scope"], tuple(s["places"] or ()), s["valid_for"],
                s["confidence"], s["method"],
            )
        self._queue: list = []
        self._order = 0
        self._sequence: dict[tuple[str, str], int] = {}  # (sender, recipient) -> last sequence sent
        self._type_counts: dict[tuple[str, str], int] = {}
        self._observation_counts: dict[str, int] = {}
        self._sent: list[SentMessage] = []
        self._deliveries: list[Delivery] = []
        self._events: list[Event] = []
        self._replies: dict[str, list[int]] = {}
        self._silent: set[str] = set()
        self._partitions: list[_Partition] = []
        self._apply_sensor_faults()
        from .participants import build  # participants need the engine; built last
        self.participants = build(self)
        self.inboxes = {pid: Inbox(self, pid) for pid in self.participants}
        for pid, factory in (external or {}).items():
            self.participants[pid] = self.inboxes[pid] = factory(self, pid)

    # -------- faults that act on sensors --------

    def _apply_sensor_faults(self) -> None:
        for fault in self.case["faults"]:
            if fault["kind"] == "sensor_lag":
                self.sensors[(fault["participant"], fault["sensor"])].lag_ms = ms(fault["lag"])
            elif fault["kind"] == "sensor_unavailable":
                self.sensors[(fault["participant"], fault["sensor"])].unavailable_from_ms = time_ms(fault["from"])

    # -------- scheduling --------

    def at(self, t_ms: int, owner: str | None, callback: Callable, *args) -> None:
        """Run `callback` at `t_ms`, unless `owner` has gone silent by then."""
        self._order += 1
        heapq.heappush(self._queue, (t_ms, self._order, owner, callback, args))

    def after(self, delay_ms: int, owner: str | None, callback: Callable, *args) -> None:
        self.at(self.now + delay_ms, owner, callback, *args)

    def time(self) -> str:
        return time_text(self.now)

    def event(self, kind: str, participant: str, **detail) -> None:
        self._events.append(Event(self.now, kind, participant, tuple(sorted(detail.items()))))

    def is_silent(self, participant: str) -> bool:
        return participant in self._silent

    # -------- messages --------

    def send(self, sender: str, mtype: str, profile: str, recipient: str, payload: dict,
             in_reply_to: str | None = None, injected: bool = False,
             sequence: int | None = None, message_id: str | None = None) -> dict | None:
        """Send one message. An explicit `sequence` or `message_id` is only for injected test messages."""
        if sender in self._silent:
            return None
        if sequence is None:
            sequence = self._sequence.get((sender, recipient), 0) + 1
            self._sequence[(sender, recipient)] = sequence
        document = build_message(sender, recipient, self.conversation, sequence, mtype, profile, payload,
                                 in_reply_to, self.now, self.ttl_ms, self.keys[f"{sender}#k1"], message_id)
        self._dispatch(document, injected)
        return document

    def send_document(self, document: dict) -> None:
        """Send a message an external participant built and signed itself (binding frame `send`)."""
        sender, recipient = document["sender"], document["recipient"]
        if sender in self._silent:
            return
        self._sequence[(sender, recipient)] = max(self._sequence.get((sender, recipient), 0), document["sequence"])
        self._dispatch(document, False)

    def _dispatch(self, document: dict, injected: bool) -> None:
        """Record one sent message, apply the faults that match it and schedule its deliveries."""
        sender, recipient, mtype = document["sender"], document["recipient"], document["type"]
        in_reply_to = document["in_reply_to"]
        count = self._type_counts.get((sender, mtype), 0) + 1
        self._type_counts[(sender, mtype)] = count
        index = len(self._sent)
        data = json.dumps(document).encode("utf-8")
        delivery_ms = self.now + self.latency_ms
        dropped = self._partitioned(sender, recipient)
        for fault in self.case["faults"]:
            rule = fault.get("match")
            if rule is None or not _matches(rule, document, count):
                continue
            kind = fault["kind"]
            if kind == "drop":
                dropped = True
            elif kind == "delay":
                delivery_ms += ms(fault["by"])
            elif kind == "strip_signature":
                data = json.dumps({k: v for k, v in document.items() if k != "signature"}).encode("utf-8")
            elif kind == "corrupt_signature":
                altered = dict(document, signature=dict(document["signature"], value="0" * 64))
                data = json.dumps(altered).encode("utf-8")
        fate = "injected" if injected else ("dropped" if dropped else "delivered")
        self._sent.append(SentMessage(index, self.now, document, fate))
        if in_reply_to is not None:
            self._replies.setdefault(in_reply_to, []).append(index)
        if not dropped:
            self.at(delivery_ms, None, self._deliver, data, index, "original")
            for fault in self.case["faults"]:
                rule = fault.get("match")
                if rule is None or not _matches(rule, document, count):
                    continue
                if fault["kind"] == "duplicate":
                    self.at(delivery_ms + ms(fault["after"]), None, self._deliver, data, index, "duplicate")
                elif fault["kind"] == "replay":
                    self.at(time_ms(fault["at"]), None, self._deliver, data, index, "replay")

    def _partitioned(self, a: str, b: str) -> bool:
        return any(
            p.between == frozenset((a, b)) and p.start_ms <= self.now < p.until_ms for p in self._partitions
        )

    def _start_rules(self, document: dict) -> None:
        """Silence and partitions start on the delivery of the message they name (spec section 10)."""
        count = sum(1 for s in self._sent[: self._index_of(document) + 1]
                    if s.document["type"] == document["type"] and s.document["sender"] == document["sender"])
        for fault in self.case["faults"]:
            rule = fault.get("from")
            if not isinstance(rule, dict) or not _matches(rule, document, count):
                continue
            if fault["kind"] == "silence":
                self._silent.add(fault["participant"])
            elif fault["kind"] == "partition":
                self._partitions.append(_Partition(frozenset(fault["between"]), self.now, time_ms(fault["until"])))

    def _index_of(self, document: dict) -> int:
        return next(s.index for s in self._sent if s.document["message_id"] == document["message_id"]
                    and s.document["sender"] == document["sender"])

    def _deliver(self, data: bytes, index: int, kind: str) -> None:
        sent = self._sent[index].document
        receiver = sent["recipient"]
        if kind == "original":
            self._start_rules(sent)
        if receiver in self._silent:
            return self.receipt(index, receiver, kind, "not_handled", None)
        self.inboxes[receiver].deliver(data, index, kind)

    # -------- the inbox host (Inbox) --------

    def context(self, receiver: str) -> Context:
        return Context(receiver, self.now, self.tolerance_ms, self.max_lifetime_ms, self.keys,
                       self.scenario["authorities"], "simulation")

    def receipt(self, index: int, receiver: str, kind: str, receipt: str, code: str | None) -> None:
        self._deliveries.append(Delivery(self.now, index, receiver, kind, receipt, code))

    def resend(self, message_id: str) -> None:
        """A duplicate was received: the replies already sent to it are sent again (0008, Q5)."""
        for reply in self._replies.get(message_id, []):
            reply_doc = self._sent[reply].document
            self.after(self.latency_ms, None, self._deliver, json.dumps(reply_doc).encode("utf-8"), reply, "resend")

    def hand_over(self, receiver: str, document: dict) -> None:
        self.participants[receiver].handle(document)

    # -------- world --------

    def start_action(self, participant: str, action: str, duration_ms: int, effect: Callable[[], None],
                     done: Callable[[], None], **args) -> bool:
        """Start an action: one motion event now, its effect and `done` after `duration_ms`."""
        if participant in self._silent:
            return False
        self.event("motion", participant, action=action, **args)

        def finish() -> None:
            effect()
            self.event("effect", participant, action=action, **args)
            done()

        self.after(duration_ms, participant, finish)
        return True

    def move(self, participant: str, to: str, done: Callable[[], None]) -> bool:
        """Move `participant` to `to`, taking the scenario's duration (spec section 3)."""
        from .participants import duration
        here = self.world.get(participant, "located_in")
        return self.start_action(participant, "move", duration(self, "move", here, to),
                                 lambda: self.world.set(participant, "located_in", to), done, to=to)

    def sense(self, participant: str, sensor_name: str, thing: str, prop: str) -> tuple[Any, int, int] | None:
        """
        What one sensor reads now, as (value, observed_at in ms, the participant's observation
        number), recorded as an observation event; None if the sensor cannot make one.
        """
        sensor = self.sensors[(participant, sensor_name)]
        if not sensor.available(self.now) or not sensor.sees(self.world, thing):
            return None
        value = self.world.get(thing, prop)
        if value is None:
            return None
        number = self._observation_counts.get(participant, 0) + 1
        self._observation_counts[participant] = number
        self.event("observation", participant, id=f"rl:obs:{participant.split(':')[-1]}:{number}",
                   subject=thing, property=prop, value=value)
        return value, self.now - sensor.lag_ms, number

    def observe(self, participant: str, sensor_name: str, thing: str, prop: str) -> dict | None:
        """A signed observation of `thing` by one sensor, or None if the sensor cannot make one."""
        reading = self.sense(participant, sensor_name, thing, prop)
        if reading is None:
            return None
        value, observed_at_ms, number = reading
        sensor = self.sensors[(participant, sensor_name)]
        return build_observation(participant, number, thing, prop, value, observed_at_ms, sensor_name,
                                 sensor.valid_for, sensor.confidence, sensor.method,
                                 self.keys[f"{participant}#k1"])

    # -------- run --------

    def run(self) -> RunResult:
        for fault in self.case["faults"]:
            if fault["kind"] == "inject":
                self.at(time_ms(fault["at"]), None, self._inject, fault["message"])
        for participant in self.participants.values():
            participant.start()
        while self._queue:
            t_ms, _, owner, callback, args = heapq.heappop(self._queue)
            self.now = t_ms
            if owner is not None and owner in self._silent:
                continue
            callback(*args)
        return self._result()

    def _inject(self, message: dict) -> None:
        self.send(message["sender"], message["type"], message["profile"], message["recipient"],
                  message["payload"], injected=True,
                  sequence=message.get("sequence"), message_id=message.get("message_id"))

    def _result(self) -> RunResult:
        designated = dict(self.scenario["initial_custody"])
        for event in self._events:
            if event.kind == "custody_committed":
                designated[event.get("thing")] = event.participant
        planner = self.participants[self.scenario["goal"]["requested_by"]]
        return RunResult(
            case=self.case["id"],
            seed=self.case.get("seed", "0"),
            sent=tuple(self._sent),
            deliveries=tuple(self._deliveries),
            events=tuple(self._events),
            planner_status=planner.status,
            goal_states=tuple(sorted((pid, p.goal_state) for pid, p in self.participants.items()
                                     if getattr(p, "goal_state", None))),
            custody=tuple(sorted((pid, p.custody) for pid, p in self.participants.items()
                                 if getattr(p, "custody", None))),
            commitments=tuple(sorted((pid, p.commitment_state) for pid, p in self.participants.items()
                                     if getattr(p, "commitment_state", None))),
            designated_custody=tuple(sorted(designated.items())),
            final_world=self.world.facts(),
            end_t_ms=max([s.t_sent_ms for s in self._sent] + [d.t_ms for d in self._deliveries]
                         + [e.t_ms for e in self._events]),
        )


def run_case(scenario: dict, case: dict) -> RunResult:
    """Run one case of tests/fixtures/arch02/cases.json against a scenario."""
    return Simulation(scenario, case).run()
