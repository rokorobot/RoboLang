"""
The reference amr-07 as an external process of `robolang-sim-stdio` 0.1 (INTEROP-1).

    python -m robolang.interop.participant

Reads harness frames on standard input and writes its own on standard output
(spec/interop-gate-2-v0.1.md 6.2-6.5). It knows only its view (6.4) and what arrives through the
binding: no scenario file, no world, no durations, no faults, no case.

The participant logic is the reference `Amr` unchanged, with the reference receiver (`Inbox`).
Only the host differs: time, timers, sensors and actions are binding frames here.
"""

from __future__ import annotations
import json
import sys
from typing import Any, Callable

from ..arch02.engine import Inbox, build_message, build_observation
from ..arch02.participants import Amr
from ..arch02.scenario import ms, time_text
from ..core import Context, Key, time_ms


class BindingError(Exception):
    """The harness broke the binding; the process stops."""


class Runtime:
    """The host of the in-process `Amr`, built from the view and talking through the binding."""

    def __init__(self, view: dict, read: Callable[[], dict], write: Callable[[dict], None]):
        self._read, self._write = read, write
        self.id = pid = view["participant"]
        p = view["parameters"]
        self.epoch_ms = self.now = time_ms(p["epoch"])
        self.ttl_ms = ms(p["message_ttl"])
        self.max_lifetime_ms = ms(p["max_message_lifetime"])
        self.tolerance_ms = ms(p["clock_tolerance"])
        self.max_hold_ms = ms(p["max_hold"])
        self.conversation = view["conversation_id"]
        entries = {view["key"]["ref"]: view["key"], **view["verification_keys"]}
        self.keys = {ref: Key(e["participant"], e["alg"], bytes.fromhex(e["secret"])) for ref, e in entries.items()}
        self.participants_config = {record["id"]: record for record in view["participants"]}
        self.scenario = {
            "descriptors": {pid: view["descriptor"]} if view["descriptor"] else {},
            "standing_authorizations": {pid: view["standing_authorization"]} if view["standing_authorization"] else {},
            "policies": {pid: view["policy"]},
            "parameters": p,
            "authorities": {pid: view["authority"]},
            "arbiters": view["arbiters"],
            "places": view["places"],
        }
        self.sensors = {s["sensor"]: s for s in view["sensors"]}
        self._sequence: dict[str, int] = {}
        self._observations = 0
        self._ids = 0
        self._timers: dict[str, tuple[Callable, tuple]] = {}
        self._actions: dict[str, Callable[[], None]] = {}
        self.participant = Amr(self, pid)
        self.inbox = Inbox(self, pid)
        self._claimed_commitment: str | None = None

    def _next_id(self, prefix: str) -> str:
        self._ids += 1
        return f"{prefix}{self._ids}"

    # -------- what Amr uses of its host --------

    def time(self) -> str:
        return time_text(self.now)

    def at(self, t_ms: int, owner: str | None, callback: Callable, *args) -> None:
        timer = self._next_id("t")
        self._timers[timer] = (callback, args)
        self._write({"frame": "set_timer", "timer": timer, "at_ms": t_ms - self.epoch_ms})

    def after(self, delay_ms: int, owner: str | None, callback: Callable, *args) -> None:
        self.at(self.now + delay_ms, owner, callback, *args)

    def event(self, kind: str, participant: str, **detail) -> None:
        self._write({"frame": "event", "kind": kind, "detail": detail})

    def send(self, sender: str, mtype: str, profile: str, recipient: str, payload: dict,
             in_reply_to: str | None = None) -> dict:
        sequence = self._sequence.get(recipient, 0) + 1
        self._sequence[recipient] = sequence
        document = build_message(sender, recipient, self.conversation, sequence, mtype, profile, payload,
                                 in_reply_to, self.now, self.ttl_ms, self.keys[f"{sender}#k1"])
        self._write({"frame": "send", "message": document})
        return document

    def observe(self, participant: str, sensor_name: str, thing: str, prop: str) -> dict | None:
        request = self._next_id("s")
        self._write({"frame": "sense", "request": request, "sensor": sensor_name, "thing": thing, "property": prop})
        answer = self._read()
        if answer.get("frame") != "sensed" or answer.get("request") != request:
            raise BindingError(f"expected sensed for {request}, got {answer!r}")
        if answer["value"] is None:
            return None
        self._observations += 1
        sensor = self.sensors[sensor_name]
        return build_observation(participant, self._observations, thing, prop, answer["value"],
                                 self.epoch_ms + answer["observed_at_ms"], sensor_name, sensor["valid_for"],
                                 sensor["confidence"], sensor["method"], self.keys[f"{participant}#k1"])

    def move(self, participant: str, to: str, done: Callable[[], None]) -> bool:
        action = self._next_id("a")
        self._actions[action] = done
        self._write({"frame": "act", "action": action, "kind": "move", "to": to})
        return True

    # -------- the inbox host --------

    def context(self, receiver: str) -> Context:
        return Context(receiver, self.now, self.tolerance_ms, self.max_lifetime_ms, self.keys,
                       self.scenario["authorities"], "simulation")

    def receipt(self, ref: tuple[Any, Any], receiver: str, kind: str, receipt: str, code: str | None) -> None:
        message_id, sender = ref
        self._write({"frame": "receipt", "message_id": message_id, "sender": sender, "receipt": receipt, "code": code})

    def resend(self, message_id: str) -> None:
        pass  # the harness sends the original reply again from its message log (finding E3)

    def is_silent(self, receiver: str) -> bool:
        return False  # silence is a harness-side fault (6.2): a silent participant gets no frames

    def hand_over(self, receiver: str, document: dict) -> None:
        self.participant.handle(document)

    # -------- frames --------

    def handle(self, frame: dict) -> None:
        kind = frame.get("frame")
        self.now = self.epoch_ms + frame["now_ms"]
        if kind == "deliver":
            data = frame["data"].encode("utf-8")
            try:
                document = json.loads(data)
            except ValueError:
                document = None
            ref = (document.get("message_id"), document.get("sender")) if isinstance(document, dict) else (None, None)
            self.inbox.deliver(data, ref, "delivered")
        elif kind == "timer":
            callback, args = self._timers.pop(frame["timer"])
            callback(*args)
        elif kind == "action_done":
            self._actions.pop(frame["action"])()
        else:
            raise BindingError(f"unexpected frame {kind!r}")
        self._claim_commitment()

    def _claim_commitment(self) -> None:
        """Commitment state changes are claims too (6.3); the reference Amr keeps them as a field."""
        state = self.participant.commitment_state
        if state != self._claimed_commitment:
            self._claimed_commitment = state
            self.event("state", self.id, record="commitment", state=state)


def serve(stdin, stdout) -> None:
    """Run one participant over binary line streams until the `end` frame."""

    def write(frame: dict) -> None:
        stdout.write((json.dumps(frame) + "\n").encode("utf-8"))
        stdout.flush()

    def read() -> dict:
        line = stdin.readline()
        if not line:
            raise BindingError("the harness closed the input")
        return json.loads(line.decode("utf-8"))

    start = read()
    if start.get("frame") != "start":
        raise BindingError(f"expected start, got {start!r}")
    runtime = Runtime(start["view"], read, write)
    runtime.now = runtime.epoch_ms + start["now_ms"]
    write({"frame": "idle"})
    while True:
        frame = read()
        if frame.get("frame") == "end":
            return
        runtime.handle(frame)
        write({"frame": "idle"})


if __name__ == "__main__":
    serve(sys.stdin.buffer, sys.stdout.buffer)
