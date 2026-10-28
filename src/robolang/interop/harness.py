"""
The harness side of `robolang-sim-stdio` 0.1 (spec/interop-gate-2-v0.1.md section 6).

One participant of the reference simulation runs as a child process; the harness owns the
clock, the world, the sensors, the durations and the faults, and talks to it in lock-step
frames. Everything the participant says is a claim, corroborated against what the harness
observed itself (9.1.1): a broken frame is a `BindingFailure`, a claim the harness's own
observations contradict is an `InconsistentClaim`. Either fails the run; neither is retried.

SIMULATION ONLY: the only I/O is the child's standard streams.
"""

from __future__ import annotations
import json
import os
import queue
import subprocess
import sys
import threading
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Any

from ..arch02.engine import RunResult, Simulation, build_observation
from ..arch02.participants import standing_authorization, thing_mass
from ..arch02.record import to_record
from ..arch02.scenario import load_scenario
from ..core import SIM_HMAC, check_message, time_ms, verify_sim
from .view import build_view

AMR = "rl:robot:amr-07"
FRAME_TIMEOUT_S = 10.0  # 6.5 rule 4, per expected frame
EVENT_KINDS_0009 = ("state", "authorization", "escalated", "custody_committed")  # 6.3 as approved in 0009
EVENT_KINDS = EVENT_KINDS_0009 + ("lease_expired",)  # with erratum 0010, B1 (finding E1)
COMMITMENT_STATES = {None: ("active",), "active": ("fulfilled", "failed", "lapsed")}  # Core 5.8; 0010, B2
RECEIPTS = ("accepted", "duplicate", "held", "rejected")
# Stage 5 (Core 8.1): after duplicate and order handling, so not recomputed by the harness but judged
# against the reference run (9.1.1, erratum 0011 N13(c)).
PARTICIPANT_CODES = ("C306", "C401", "C402", "C403", "C404")
STATE_RECORDS = ("goal", "custody", "commitment")  # 6.3, N7: the required state claims
DIAGNOSTICS = ("peer_ready", "reservation_denied", "motion_refused", "no_confirmation_evidence",
               "physical_custody_unknown")  # 6.3, N7: optional, corroborated when written
_DURATION_MS = {"ms": 1, "s": 1000, "min": 60000}  # Core 2: duration units


class BindingFailure(Exception):
    """Framing, lock-step or a time limit broken (6.5 rule 4)."""


class InconsistentClaim(Exception):
    """A claim the harness's own observations do not support (9.1.1)."""


@dataclass(frozen=True)
class _Hold:
    """An open entry of the hold ledger (6.2, erratum 0011 N3): a message the participant holds."""

    index: int  # the delivered message
    stream: tuple[str, str]  # (sender, conversation_id); the recipient is the participant
    sequence: int
    expires_safe_ms: int  # expires_at - clock_tolerance
    held_at_ms: int  # its delivery


def _duration_ms(quantity: Any) -> Decimal | None:
    try:
        return Decimal(quantity["value"]) * _DURATION_MS[quantity["unit"]]
    except (TypeError, KeyError, InvalidOperation):
        return None


def _decimal(text: Any) -> Decimal | None:
    try:
        return Decimal(text) if isinstance(text, str) else None
    except InvalidOperation:
        return None


def _instant(text: Any) -> int | None:
    try:
        return time_ms(text) if isinstance(text, str) else None
    except ValueError:
        return None


def participant_command() -> list[str]:
    """The reference participant process (INTEROP-1)."""
    return [sys.executable, "-m", "robolang.interop.participant"]


class ExternalParticipant:
    """The engine's view of a participant that runs as a child process: participant and inbox."""

    def __init__(self, sim: Simulation, pid: str, command: list[str] | None = None,
                 frame_timeout_s: float = FRAME_TIMEOUT_S, event_kinds=EVENT_KINDS):
        self.sim, self.id = sim, pid
        self.command = command or participant_command()
        self.frame_timeout_s = frame_timeout_s
        self.event_kinds = tuple(event_kinds)
        self.goal_state: str | None = None
        self.custody: str | None = None
        self.commitment_state: str | None = None
        self._process: subprocess.Popen | None = None
        self._lines: queue.Queue = queue.Queue()
        self._pending: tuple[int, str, Any] | None = None  # the delivery awaiting its receipt
        self._held: dict[tuple[str, str], _Hold] = {}  # the hold ledger: (sender, message_id) -> open entry
        self._names: dict[str, set[str]] = {}  # per kind (timer, sense request, action): no reuse in a run
        self._delivered: set[str] = set()  # message IDs delivered to the participant
        self._accepted: list[dict] = []  # documents the participant accepted
        self._sent: list[dict] = []  # documents the participant sent
        self._observations: dict[str, dict] = {}  # observation ID -> the observation as the harness made it
        self._claims: list[tuple[str, dict]] = []
        self._acts: list[str] = []
        self._cycle_accepted: list[dict] = []  # documents accepted in this cycle (diagnostics, 6.3)
        self._cycle_unseen: list[int] = []  # `sensed` with `value: null` in this cycle, by the sends before it

    # -------- process and framing --------

    def _spawn(self) -> None:
        import robolang
        env = dict(os.environ)
        src = str(Path(robolang.__file__).resolve().parent.parent)
        env["PYTHONPATH"] = os.pathsep.join(p for p in (src, env.get("PYTHONPATH")) if p)
        self._process = subprocess.Popen(self.command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, env=env)
        threading.Thread(target=self._pump, args=(self._process.stdout,), daemon=True).start()

    def _pump(self, stream) -> None:
        for line in iter(stream.readline, b""):
            self._lines.put(line)
        self._lines.put(None)

    def _write(self, frame: dict) -> None:
        try:
            self._process.stdin.write((json.dumps(frame) + "\n").encode("utf-8"))
            self._process.stdin.flush()
        except OSError as error:
            raise BindingFailure(f"cannot write {frame.get('frame')}: {error}") from error

    def _read(self) -> dict:
        try:
            line = self._lines.get(timeout=self.frame_timeout_s)
        except queue.Empty:
            raise BindingFailure(f"no frame within {self.frame_timeout_s} s") from None
        if line is None:
            raise BindingFailure("the participant closed its output")
        try:
            frame = json.loads(line.decode("utf-8"))
        except ValueError:
            raise BindingFailure(f"invalid JSON: {line[:80]!r}") from None
        if not isinstance(frame, dict):
            raise BindingFailure(f"not a frame: {line[:80]!r}")
        return frame

    def _now_ms(self) -> int:
        return self.sim.now - self.sim.epoch_ms

    def _name(self, name: Any, what: str) -> str:
        names = self._names.setdefault(what, set())
        if not isinstance(name, str) or name in names:
            raise BindingFailure(f"{what} name {name!r} missing or reused")
        names.add(name)
        return name

    def _cycle(self, frame: dict) -> None:
        """Write one harness frame and take the participant's frames, in order, until `idle`."""
        self._write(frame)
        while True:
            reply = self._read()
            kind = reply.get("frame")
            if kind == "idle":
                break
            handler = getattr(self, f"_on_{kind}", None) if kind in (
                "send", "receipt", "set_timer", "sense", "act", "event") else None
            if handler is None:
                raise BindingFailure(f"unknown frame {kind!r}")
            handler(reply)
        if self._pending is not None:
            raise BindingFailure("no receipt for the delivered message")
        self._corroborate()

    # -------- the engine's side: participant and inbox --------

    def start(self) -> None:
        self._spawn()
        self._cycle({"frame": "start", "now_ms": self._now_ms(), "participant": self.id,
                     "view": build_view(self.sim.scenario, self.id)})

    def deliver(self, data: bytes, index: int, kind: str) -> None:
        self._delivered.add(self.sim._sent[index].document["message_id"])
        # Stages 1-3 only (9.1.1, erratum 0011 N13(c)): stage 5 comes after duplicates and order.
        self._pending = (index, kind, check_message(data, self.sim.context(self.id), meaning=False))
        self._cycle({"frame": "deliver", "now_ms": self._now_ms(), "data": data.decode("utf-8")})

    def _timer(self, timer: str) -> None:
        self._cycle({"frame": "timer", "now_ms": self._now_ms(), "timer": timer})

    def _action_done(self, action: str) -> None:
        self._cycle({"frame": "action_done", "now_ms": self._now_ms(), "action": action})

    def end(self) -> None:
        if self._process is None:
            return
        self._write({"frame": "end"})
        self._process.stdin.close()
        try:
            line = self._lines.get(timeout=self.frame_timeout_s)
        except queue.Empty:
            raise BindingFailure("the participant did not exit after end") from None
        if line is not None:
            raise BindingFailure(f"output after end: {line[:80]!r}")
        self._process.wait(timeout=self.frame_timeout_s)

    def kill(self) -> None:
        if self._process is not None and self._process.poll() is None:
            self._process.kill()
            self._process.wait()

    # -------- participant frames --------

    def _on_send(self, frame: dict) -> None:
        document = frame.get("message")
        if not isinstance(document, dict) or document.get("sender") != self.id:
            raise InconsistentClaim(f"{self.id} sent a message as someone else: {document!r:.120}")
        reply_to = document.get("in_reply_to")
        if reply_to is not None and reply_to not in self._delivered:
            raise InconsistentClaim(f"reply to {reply_to}, which was never delivered to {self.id}")
        payload = document.get("payload") if isinstance(document.get("payload"), dict) else {}
        for observation in [*payload.get("observations", []), *payload.get("evidence", [])]:
            if not self._observed(observation):
                raise InconsistentClaim(f"observation not as the harness made it: {observation!r:.160}")
        self._sent.append(document)
        self.sim.send_document(document)

    def _on_receipt(self, frame: dict) -> None:
        message_id, sender, receipt, code = (frame.get(k) for k in ("message_id", "sender", "receipt", "code"))
        if receipt not in RECEIPTS:
            raise BindingFailure(f"unknown receipt {receipt!r}")
        if self._pending is not None:  # the receipt for the message just delivered
            index, kind, core = self._pending
            self._pending = None
            document = self.sim._sent[index].document
            if not core.accepted:  # stages 1-3, recomputed on the delivered bytes
                if (receipt, code) != ("rejected", core.code):
                    raise InconsistentClaim(f"receipt {receipt} {code} where Core 8 gives {core.code}")
            else:
                if (message_id, sender) != (document["message_id"], document["sender"]):
                    raise InconsistentClaim(f"receipt for {message_id}, but {document['message_id']} was delivered")
                if receipt == "rejected" and code not in PARTICIPANT_CODES:
                    raise InconsistentClaim(f"rejected with {code}, but the message passes Core 8 stages 1-3")
        else:  # a message the participant held leaves the hold
            entry = self._held.pop((sender, message_id), None)
            if entry is None or receipt in ("held", "duplicate"):
                raise InconsistentClaim(f"receipt {receipt} for {message_id}, which is not held")
            index, kind = entry.index, "released"
        if receipt == "held":
            self._open_hold(index)
        self.sim.receipt(index, self.id, kind, receipt, code)
        if receipt == "accepted":
            self._accepted.append(self.sim._sent[index].document)
            self._cycle_accepted.append(self.sim._sent[index].document)
        elif receipt == "duplicate":
            self.sim.resend(message_id)  # finding E3: the binding does not say who sends it again

    def _on_set_timer(self, frame: dict) -> None:
        timer = self._name(frame.get("timer"), "timer")
        at_ms = frame.get("at_ms")
        if not isinstance(at_ms, int):
            raise BindingFailure(f"set_timer without at_ms: {frame!r}")
        self.sim.at(self.sim.epoch_ms + at_ms, self.id, self._timer, timer)

    def _on_sense(self, frame: dict) -> None:
        request = self._name(frame.get("request"), "sense request")
        sensor = frame.get("sensor")
        if (self.id, sensor) not in self.sim.sensors:
            raise BindingFailure(f"{self.id} has no sensor {sensor!r}")
        thing, prop = frame.get("thing"), frame.get("property")
        reading = self.sim.sense(self.id, sensor, thing, prop)
        value, observed_at_ms = (None, None) if reading is None else reading[:2]
        if reading is None:
            self._cycle_unseen.append(len(self._sent))
        if reading is not None:
            # The complete observation the participant can build from this reading, signature included:
            # any observation it later presents must equal it (provenance, 9.1.1).
            s = self.sim.sensors[(self.id, sensor)]
            expected = build_observation(self.id, reading[2], thing, prop, value, observed_at_ms, sensor, s.valid_for,
                                         s.confidence, s.method, self.sim.keys[f"{self.id}#k1"])
            self._observations[expected["id"]] = expected
        self._write({"frame": "sensed", "now_ms": self._now_ms(), "request": request, "value": value,
                     "observed_at_ms": None if observed_at_ms is None else observed_at_ms - self.sim.epoch_ms})

    def _on_act(self, frame: dict) -> None:
        action = self._name(frame.get("action"), "action")
        if frame.get("kind") != "move" or not isinstance(frame.get("to"), str):
            raise BindingFailure(f"unknown action: {frame!r}")
        self._acts.append(action)
        self.sim.move(self.id, frame["to"], lambda: self._action_done(action))

    def _on_event(self, frame: dict) -> None:
        kind, detail = frame.get("kind"), frame.get("detail")
        if kind not in self.event_kinds or not isinstance(detail, dict):
            raise BindingFailure(f"unknown event: {frame!r:.120}")
        if kind == "state" and detail.get("record") not in STATE_RECORDS + DIAGNOSTICS:
            raise BindingFailure(f"unknown state record: {frame!r:.120}")  # 6.3, N7
        self._claims.append((kind, detail))
        if kind == "state" and detail.get("record") == "commitment":
            return  # 0010, B2: the final state only, set once corroborated; the reference records no event
        self.sim.event(kind, self.id, **detail)
        if kind == "state" and detail.get("record") == "goal":
            self.goal_state = detail.get("state")
        elif kind == "state" and detail.get("record") == "custody":
            self.custody = detail.get("state")

    # -------- the hold ledger (6.2, erratum 0011 N3) --------

    def _open_hold(self, index: int) -> None:
        """A `held` receipt opens an entry; the harness checks it at both Core 8.4 instants."""
        document = self.sim._sent[index].document
        entry = _Hold(index, (document["sender"], document["conversation_id"]), document["sequence"],
                      time_ms(document["expires_at"]) - self.sim.tolerance_ms, self.sim.now)
        key = (document["sender"], document["message_id"])
        self._held[key] = entry
        for t_ms in (entry.expires_safe_ms, entry.held_at_ms + self.sim.max_hold_ms):
            self.sim.at(t_ms, None, self._take_over, key)  # no owner: it fires while the participant is silent

    def _take_over(self, key: tuple[str, str]) -> None:
        """While the participant is silent, close a due entry as Core 8.4 would; expiry wins a tie."""
        entry = self._held.get(key)
        if entry is None or not self.sim.is_silent(self.id):
            return  # closed already, or the participant still owns its holds
        if self.sim.now >= entry.expires_safe_ms:
            self._close_hold(key, "rejected", "C301")
        elif self.sim.now >= entry.held_at_ms + self.sim.max_hold_ms:
            self._close_hold(key, "not_handled", None)  # the missing numbers are skipped
        else:
            return
        sequence = entry.sequence + 1  # the stream is past this message: release what is now next
        while (nxt := next((k for k, e in self._held.items()
                            if e.stream == entry.stream and e.sequence == sequence), None)) is not None:
            self._close_hold(nxt, "not_handled", None)
            sequence += 1

    def _close_hold(self, key: tuple[str, str], receipt: str, code: str | None) -> None:
        """The one terminal release receipt of a held message; the entry is removed."""
        entry = self._held.pop(key)
        self.sim.receipt(entry.index, self.id, "released", receipt, code)

    # -------- corroboration (9.1.1) --------

    def _accepted_of(self, mtype: str) -> list[dict]:
        return [d for d in self._accepted if d["type"] == mtype]

    def _standing(self) -> str | None:
        """The standing authorization covering the sub-goal on the wire, evaluated from the scenario (7.2)."""
        request = next(iter(self._accepted_of("request_assistance")), None)
        if request is None:
            return None
        goal, requester = request["payload"]["goal"], request["sender"]
        thing, target = goal["desired_state"][0]["subject"], goal["desired_state"][0]["value"]
        handoff = next((p for p, a in self.sim.scenario["arbiters"].items() if a == requester), None)
        return standing_authorization(self.sim, self.id, "r2r", {"transport_payload", "custody_transfer"},
                                      {handoff, target}, requester, thing_mass(request["payload"]["things"], thing))

    def _committed(self) -> set[str]:
        """Releases the participant accepted and acknowledged (RFC-001 10.4 rules 3-4)."""
        released = {d["message_id"] for d in self._accepted_of("custody_released")}
        return {d["payload"]["acknowledged"] for d in self._sent if d["type"] == "acknowledge"
                and d["payload"].get("accepted") is True and d["payload"]["acknowledged"] in released}

    def _commitments(self) -> dict[str, dict]:
        """Commitments the participant accepted in a `commit`, by ID, where it is the debtor."""
        return {c["payload"]["commitment"]["id"]: c["payload"]["commitment"] for c in self._accepted_of("commit")
                if c["payload"]["commitment"]["debtor"] == self.id}

    def _lease_expired(self, commitment: dict) -> bool:
        return self.sim.now >= time_ms(commitment["lease"]["expires_at"])

    def _observed(self, observation: Any) -> bool:
        """
        An observation equal in meaning to one built from a reading the harness gave (`sensed`), with a
        valid signature under the participant's key (9.1.1, erratum 0011 N5). Same fields, `id`, `kind`,
        `subject`, `property`, `value` and `source`; the same instant for `observed_at`, duration for
        `valid_for` and decimal for `confidence`. Formatting alone is not a failure.
        """
        if not isinstance(observation, dict):
            return False
        expected = self._observations.get(observation.get("id"))
        if expected is None or set(observation) != set(expected):
            return False
        same = all(observation[f] == expected[f] for f in ("kind", "subject", "property", "value", "source"))
        observed_at = _instant(observation["observed_at"])
        valid_for, confidence = _duration_ms(observation["valid_for"]), _decimal(observation["confidence"])
        if not (same and observed_at is not None and observed_at == _instant(expected["observed_at"])
                and valid_for is not None and valid_for == _duration_ms(expected["valid_for"])
                and confidence is not None and confidence == _decimal(expected["confidence"])):
            return False
        key_ref = f"{self.id}#k1"
        signature = observation["signature"]
        return (isinstance(signature, dict) and signature.get("alg") == SIM_HMAC and signature.get("key") == key_ref
                and isinstance(signature.get("value"), str) and verify_sim(observation, self.sim.keys[key_ref]))

    def _commitment_claim(self, state: Any) -> None:
        """0010, B2: a supported transition, backed by an accepted commitment and the wire."""
        commitments = self._commitments()
        if state not in COMMITMENT_STATES.get(self.commitment_state, ()) or not commitments:
            raise InconsistentClaim(f"commitment state {self.commitment_state} -> {state} not supported")
        outcomes = [d["payload"] for d in self._sent if d["type"] == "outcome"]
        if state == "fulfilled":
            ok = any(o["outcome"]["status"] == "succeeded" and o["evidence"]
                     and all(self._observed(e) for e in o["evidence"]) for o in outcomes)
        elif state == "failed":
            ok = any(o["outcome"]["status"] != "succeeded" for o in outcomes)
        elif state == "lapsed":
            ok = any(self._lease_expired(c) for c in commitments.values())
        else:
            ok = True
        if not ok:
            raise InconsistentClaim(f"commitment state {state} not supported by the wire")
        self.commitment_state = state

    def _diagnostic(self, detail: dict, acts: list[str], accepted: list[dict], unseen: list[int]) -> bool:
        """An optional diagnostic, consistent with what the harness saw in the same cycle (6.3, N7)."""
        record = detail["record"]
        if set(detail) != ({"record", "to"} if record == "motion_refused" else {"record"}):
            return False
        if record == "peer_ready":
            return any(d["type"] == "coordinate" and d["payload"].get("phase") == "ready_for_handover"
                       for d in accepted)
        if record == "reservation_denied":
            return any(d["type"] == "reservation_decision" and d["payload"].get("decision") == "denied"
                       for d in accepted)
        if record == "motion_refused":
            return not acts and isinstance(detail["to"], str)
        if record == "no_confirmation_evidence":
            return bool(unseen)
        # physical_custody_unknown: a null reading after the commit point (the acknowledge of a release)
        committed = self._committed()
        acknowledged = [i for i, d in enumerate(self._sent) if d["type"] == "acknowledge"
                        and d["payload"].get("acknowledged") in committed]
        return any(i < sends for i in acknowledged for sends in unseen)

    def _corroborate(self) -> None:
        """Check this cycle's claims and actions against the wire and the world, at the end of the cycle."""
        claims, acts, self._claims, self._acts = self._claims, self._acts, [], []
        accepted, unseen, self._cycle_accepted, self._cycle_unseen = self._cycle_accepted, self._cycle_unseen, [], []
        decisions = {d["message_id"]: d["payload"] for d in self._accepted_of("authorization_decision")}
        # A granted decision covers an `act` only while its lease is unexpired at that moment, in the
        # safe direction (reference simulation 6.4, clarification R-5; finding H4). The cycle is one instant.
        granted = any(p["decision"] == "granted" and p["mode"] == "simulation"
                      and self.sim.now < time_ms(p["lease"]["expires_at"]) - self.sim.tolerance_ms
                      for p in decisions.values())
        for action in acts:
            if not (granted or self._standing() is not None):
                raise InconsistentClaim(f"action {action} without an authorization the harness can establish")
        for kind, detail in claims:
            if kind == "authorization":
                basis = detail.get("basis")
                ok = (decisions[basis]["decision"] == detail.get("decision") if basis in decisions
                      else detail.get("decision") == "granted" and basis is not None and basis == self._standing())
                if not ok:
                    raise InconsistentClaim(f"authorization claim not established: {detail}")
            elif kind == "custody_committed":
                if detail.get("release") not in self._committed():
                    raise InconsistentClaim(f"commit point without an acknowledged release: {detail}")
            elif kind == "state" and detail.get("record") == "custody" and detail.get("state") == "HOLDING":
                if not self._committed():
                    raise InconsistentClaim("HOLDING custody without an acknowledged release")
            elif kind == "state" and detail.get("record") == "goal" and detail.get("state") == "SUCCEEDED":
                if not any(d["type"] == "outcome" and d["payload"]["outcome"]["status"] == "succeeded"
                           for d in self._sent):
                    raise InconsistentClaim("SUCCEEDED goal without a sent outcome of succeeded")
            elif kind == "lease_expired":
                commitment = self._commitments().get(detail.get("commitment"))
                if detail.get("lease") != "commitment" or commitment is None or not self._lease_expired(commitment):
                    raise InconsistentClaim(f"lease_expired claim not established: {detail}")
            elif kind == "state" and detail.get("record") == "commitment":
                self._commitment_claim(detail.get("state"))
            elif kind == "state" and detail.get("record") in DIAGNOSTICS:
                if not self._diagnostic(detail, acts, accepted, unseen):
                    raise InconsistentClaim(f"diagnostic not consistent with this cycle: {detail}")
            elif kind == "escalated":
                expired = any(self._lease_expired(c) for c in self._commitments().values())
                if not expired or self._committed():
                    raise InconsistentClaim("escalation without an expired lease and unresolved custody")


def run_case_external(scenario: dict, case: dict, participant: str = AMR, **options) -> RunResult:
    """Run one case with `participant` as an external process through the binding."""
    proxies: list[ExternalParticipant] = []

    def factory(sim: Simulation, pid: str) -> ExternalParticipant:
        proxy = ExternalParticipant(sim, pid, **options)
        proxies.append(proxy)
        return proxy

    sim = Simulation(scenario, case, external={participant: factory})
    try:
        result = sim.run()
        for proxy in proxies:
            proxy.end()
    finally:
        for proxy in proxies:
            proxy.kill()
    return result


def simulate_case_external(scenario: bytes, case: dict, participant: str = AMR, **options) -> dict:
    """The run record of one case with `participant` external; same inputs, same run ID (10)."""
    case = dict(case, seed=case.get("seed", "0"))
    parsed = load_scenario(scenario)
    result = run_case_external(parsed, case, participant, **options)
    return to_record(result, scenario=scenario, epoch=parsed["parameters"]["epoch"])
