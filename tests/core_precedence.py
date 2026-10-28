"""
The multi-fault precedence fixtures of tests/fixtures/core (INTEROP-2f, F2-F5), built from the valid
fixtures by named faults.

Each fixture lists its faults in the order Core 8.1 requires: the first one's code is the receipt.
`build(fixture, faults)` applies any subset, so a test can show that every later fault is really
present: applying `faults[i:]` alone gives the code of `faults[i]`. Faults are applied from last to
first, so an earlier (winning) fault overrides a field a later one also sets.

`python tests/core_precedence.py` writes the fixture files; tests/test_core_precedence.py checks that
the files on disk are exactly what this module builds.
"""

from __future__ import annotations
import copy
import json
from pathlib import Path
from typing import Callable

from robolang.core import Key, sign_sim

CORE = Path(__file__).parent / "fixtures" / "core"
CONTEXT = json.loads((CORE / "context.json").read_text(encoding="utf-8"))
KEYS = {ref: Key(k["participant"], k["alg"], bytes.fromhex(k["secret"])) for ref, k in CONTEXT["keys"].items()}
DAY = "2026-10-19T"
OTHER_KEY = {"rl:robot:arm-02": "rl:agent:planner-1#k1", "rl:robot:amr-07": "rl:robot:arm-02#k1"}


def base(name: str) -> dict:
    document = json.loads((CORE / "valid" / f"{name}.json").read_text(encoding="utf-8"))
    return {k: v for k, v in document.items() if k != "signature"}


class Draft:
    """A message being built: its body, how it is signed, and its first observation's signing."""

    def __init__(self, body: dict):
        self.body = body
        self.key_ref = f"{body['sender']}#k1"
        self.alg: str | None = None
        self.corrupt = self.unsigned = False

    def document(self) -> dict:
        if self.unsigned:
            return self.body
        signed = sign_sim(self.body, self.key_ref, KEYS[self.key_ref])
        if self.alg:
            signed["signature"]["alg"] = self.alg
        if self.corrupt:
            signed["signature"]["value"] = "0" * 64
        return signed


def _times(sent: str | None = None, expires: str | None = None) -> Callable[[Draft], None]:
    def apply(d: Draft) -> None:
        if sent:
            d.body["sent_at"] = DAY + sent
        if expires:
            d.body["expires_at"] = DAY + expires
    return apply


def _set(field: str, value) -> Callable[[Draft], None]:
    return lambda d: d.body.__setitem__(field, value)


def _observation(change: Callable[[dict], dict]) -> Callable[[Draft], None]:
    def apply(d: Draft) -> None:
        d.body["payload"]["observations"][0] = change(d.body["payload"]["observations"][0])
    return apply


def _observation_other_key(o: dict) -> dict:
    """Signed correctly, but with a key that is not its source's (C203)."""
    return sign_sim(o, "rl:robot:arm-02#k1", KEYS["rl:robot:arm-02#k1"])


def _observation_alg(o: dict) -> dict:
    return dict(o, signature=dict(o["signature"], alg="ed25519"))  # not accepted in simulation (C204)


# name -> (code, change). Context: now 10:14:00.500, tolerance 500 ms, maximum lifetime 60 s.
FAULTS: dict[str, tuple[str, Callable[[Draft], None]]] = {
    "version": ("C102", _set("schema_version", "0.2")),
    "reserved_type": ("C106", _set("type", "negotiate")),
    "unknown_type": ("C105", _set("type", "teleport")),
    "unsigned": ("C201", lambda d: setattr(d, "unsigned", True)),
    "profile": ("C107", _set("profile", "r2r")),  # assessment allows a2r only
    "no_verification": ("C402", lambda d: d.body["payload"]["outcome"].__setitem__("verification", None)),
    "unknown_field": ("C103", _set("note", "not a Core 0.1 field")),
    "missing_field": ("C104", lambda d: d.body.pop("in_reply_to")),
    "algorithm": ("C204", lambda d: setattr(d, "alg", "ed25519")),
    "other_key": ("C203", lambda d: setattr(d, "key_ref", OTHER_KEY[d.body["sender"]])),
    "bad_value": ("C202", lambda d: setattr(d, "corrupt", True)),
    "observation_other_key": ("C203", _observation(_observation_other_key)),
    "observation_algorithm": ("C204", _observation(_observation_alg)),
    "lifetime_sent_early": ("C302", _times(sent="10:12:00.000Z")),  # with the expiry below: 120.4 s
    "lifetime_expires_late": ("C302", _times(expires="10:16:00.000Z")),  # with the future send below: 115 s
    "expired": ("C301", _times(sent="10:13:30.000Z", expires="10:14:00.400Z")),
    "future": ("C303", _times(sent="10:14:05.000Z", expires="10:14:35.000Z")),
    "wrong_recipient": ("C307", _set("recipient", "rl:robot:amr-07")),  # the receiver is planner-1
}

# file stem -> (valid fixture it starts from, stage, faults in precedence order, receiver if not the recipient)
FIXTURES: dict[str, tuple[str, str, list[str], str | None]] = {
    # F2, stage 1 (Core 8.1, ranks 2-7)
    "c102_before_c106": ("assessment", "form", ["version", "reserved_type"], None),
    "c102_before_c201": ("assessment", "form", ["version", "unsigned"], None),
    "c102_before_c107": ("assessment", "form", ["version", "profile"], None),
    "c102_before_c402": ("outcome_succeeded", "form", ["version", "no_verification"], None),
    "c105_before_c201": ("assessment", "form", ["unknown_type", "unsigned"], None),
    "c106_before_c201": ("assessment", "form", ["reserved_type", "unsigned"], None),
    "c201_before_c107": ("assessment", "signature", ["unsigned", "profile"], None),
    "c201_before_c402": ("outcome_succeeded", "signature", ["unsigned", "no_verification"], None),
    "c103_before_c104": ("assessment", "form", ["unknown_field", "missing_field"], None),
    # F3, stage 2 (Core 8.1; decision record 0012, P3)
    "c204_before_c203_before_c202": ("assessment", "signature", ["algorithm", "other_key", "bad_value"], None),
    "c203_before_c202": ("assessment", "signature", ["other_key", "bad_value"], None),
    "c202_message_before_c203_observation": ("custody_confirmed", "signature",
                                             ["bad_value", "observation_other_key"], None),
    "c203_message_before_c204_observation": ("custody_confirmed", "signature",
                                             ["other_key", "observation_algorithm"], None),
    # F4, stage 3 (Core 8.1)
    "c302_before_c301": ("assessment", "freshness", ["lifetime_sent_early", "expired"], None),
    "c302_before_c303": ("assessment", "freshness", ["lifetime_expires_late", "future"], None),
    "c301_before_c307": ("assessment", "freshness", ["expired", "wrong_recipient"], "rl:agent:planner-1"),
    "c303_before_c307": ("assessment", "freshness", ["future", "wrong_recipient"], "rl:agent:planner-1"),
}


def build(stem: str, faults: list[str] | None = None) -> dict:
    """The fixture `stem` with `faults` (default: all of its faults) applied."""
    start, _, all_faults, _ = FIXTURES[stem]
    draft = Draft(copy.deepcopy(base(start)))
    for name in reversed(all_faults if faults is None else faults):
        FAULTS[name][1](draft)
    return draft.document()


def case(stem: str) -> dict:
    """The fixture's entry in cases.json."""
    _, stage, faults, receiver = FIXTURES[stem]
    codes = [FAULTS[name][0] for name in faults]
    entry = {"file": f"invalid/{stem}.json", "expect": codes[0], "stage": stage}
    if receiver:
        entry["receiver"] = receiver
    entry["competing"] = codes[1:]
    return entry


def write() -> None:
    for stem in FIXTURES:
        (CORE / "invalid" / f"{stem}.json").write_text(json.dumps(build(stem), indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    write()
