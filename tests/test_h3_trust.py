"""
Regression tests for H3: receiver-scoped verification keys (decision record 0013; reference
simulation 5.3). amr-07 verifies with its own key and the verification keys of its gate 2 6.4 view
only; the reference engine, the harness and an amr-07 built from the view agree. The other
participants keep every scenario key.
"""

from __future__ import annotations
import json
from pathlib import Path

import pytest

from robolang.arch02 import RunResult, Simulation, load_scenario, run_case
from robolang.arch02.engine import build_message
from robolang.core import check_message, sign_sim
from robolang.interop import build_view, run_case_external

ROOT = Path(__file__).resolve().parent.parent
PARSED = load_scenario((ROOT / "tests" / "fixtures" / "arch02" / "scenario.json").read_bytes())
P, A, M, H = "rl:agent:planner-1", "rl:robot:arm-02", "rl:robot:amr-07", "rl:human:shift-lead-3"
C42 = "rl:thing:container-c42"
DAY = "2026-10-19T"
CASE = {"id": "custom", "seed": "0", "faults": []}


def simulation() -> Simulation:
    return Simulation(PARSED, CASE)


def query(sim: Simulation, sender: str, recipient: str = M) -> dict:
    """A correctly signed custody_query from `sender`, signed with its own key."""
    return build_message(sender, recipient, sim.conversation, 1, "custody_query", "r2r", {"thing": C42}, None,
                         sim.now, sim.ttl_ms, sim.keys[f"{sender}#k1"])


def resigned(document: dict, key_ref: str, secret_of: str, sim: Simulation, alg: str | None = None) -> dict:
    unsigned = {k: v for k, v in document.items() if k != "signature"}
    signed = sign_sim(unsigned, key_ref, sim.keys[f"{secret_of}#k1"])
    if alg is not None:
        signed["signature"] = dict(signed["signature"], alg=alg)
    return signed


def code(sim: Simulation, document: dict, receiver: str = M) -> str | None:
    """The stage 1-3 receipt code at `receiver` (None if it passes)."""
    return check_message(json.dumps(document).encode("utf-8"), sim.context(receiver), meaning=False).code


# -------- stage 2 with amr-07's trust context --------

def test_message_from_arm02_passes_signature_validation():
    sim = simulation()
    assert code(sim, query(sim, A)) is None


def test_message_from_the_configured_authority_passes_signature_validation():
    sim = simulation()
    assert code(sim, query(sim, H)) is None


def test_message_from_planner1_outside_the_view_is_c203():
    sim = simulation()
    assert code(sim, query(sim, P)) == "C203"


def test_unknown_key_reference_is_c203():
    sim = simulation()
    assert code(sim, resigned(query(sim, A), f"{A}#k9", A, sim)) == "C203"


def test_configured_key_with_an_invalid_signature_is_c202():
    sim = simulation()
    document = query(sim, A)
    assert code(sim, dict(document, signature=dict(document["signature"], value="0" * 64))) == "C202"


def test_unaccepted_algorithm_with_an_unknown_key_is_c204_first():
    sim = simulation()
    assert code(sim, resigned(query(sim, P), f"{P}#k1", P, sim, alg="ed25519")) == "C204"


def test_trust_context_is_exactly_the_view():
    sim = simulation()
    view = build_view(sim.scenario, M)
    assert set(sim.context(M).keys) == {view["key"]["ref"], *view["verification_keys"]}
    assert set(view["verification_keys"]) == {f"{A}#k1", f"{H}#k1"}  # no key added to the view


@pytest.mark.parametrize("receiver", [A, P])
def test_other_participants_keep_every_scenario_key(receiver):
    sim = simulation()
    assert sim.context(receiver).keys == sim.keys
    sender = M if receiver == A else A
    assert code(sim, query(sim, sender, receiver), receiver) is None


# -------- before duplicate and order handling; reference and external agree --------

def inject_from(sender: str, sequence: int | None = None, message_id: str = "m-h3") -> dict:
    message = {"type": "custody_query", "profile": "r2r", "sender": sender, "recipient": M,
               "message_id": message_id, "payload": {"thing": C42}}
    if sequence is not None:
        message["sequence"] = sequence
    return {"kind": "inject", "at": DAY + "10:14:10.000Z", "message": message}


def receipts(r: RunResult) -> list[tuple]:
    return [(d.t_ms, r.sent[d.message].document["message_id"], d.receiver, d.kind, d.receipt, d.code)
            for d in r.deliveries]


def of(r: RunResult, message_id: str) -> list[tuple]:
    return [(d.kind, d.receipt, d.code) for d in r.deliveries if r.sent[d.message].document["message_id"] == message_id]


def test_c203_comes_before_order_handling_no_hold_and_no_stream_advance():
    # Sequence 2 from planner-1 would be held if it passed stage 2; it is rejected at once.
    sim = Simulation(PARSED, dict(CASE, faults=[inject_from(P, sequence=2)]))
    r = sim.run()
    assert of(r, "m-h3") == [("original", "rejected", "C203")]
    inbox = sim.inboxes[M]
    assert not any(stream[0] == P for stream in inbox._next) and not any(stream[0] == P for stream in inbox._held)


@pytest.mark.parametrize("sender, expected", [
    (P, [("original", "rejected", "C203")]),  # outside the view: stage 2
    (H, [("original", "accepted", None)]),  # in the view: past stage 2; amr-07 answers the query
    (A, [("original", "accepted", None)]),
], ids=["planner1", "authority", "arm02"])
def test_reference_and_external_amr07_give_the_same_receipts(sender, expected):
    case = dict(CASE, faults=[inject_from(sender)])
    reference = run_case(PARSED, case)
    external = run_case_external(PARSED, case)
    assert receipts(external) == receipts(reference)
    assert of(reference, "m-h3") == expected
