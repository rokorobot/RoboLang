"""
Regression tests for INTEROP-2e: the harness corrections N3, H1, H2 and N13(c) (decision record
0011; spec/interop-gate-2-v0.1.md 6.2, 6.3 and 9.1.1; docs/interop/spec-implementation-discrepancies.md
section 2).

The reference amr-07 runs as an external process. Where the specification makes the reference run
the source of truth (stages 4-5, and the holds of a silent participant, which the reference engine
closes as Core 8.4 says), the external run's receipts must equal the reference run's. Patched
participants show that unsupported claims still fail. No golden record is added (N3's
`seq_silent_while_held` is INTEROP-2f).
"""

from __future__ import annotations
import sys
from pathlib import Path

import pytest

from robolang.arch02 import RunResult, load_scenario, run_case, time_text
from robolang.interop import BindingFailure, InconsistentClaim, run_case_external

ROOT = Path(__file__).resolve().parent.parent
PARSED = load_scenario((ROOT / "tests" / "fixtures" / "arch02" / "scenario.json").read_bytes())
A, M = "rl:robot:arm-02", "rl:robot:amr-07"
DAY = "2026-10-19T"
SUB_GOAL = PARSED["goal"]["id"] + "-sub-1"

PATCHED = """
import sys
from robolang.interop import participant as p
from robolang.arch02.participants import Amr
from robolang.core import Key, sign_sim
{patch}
p.serve(sys.stdin.buffer, sys.stdout.buffer)
"""


def patched(patch: str) -> list[str]:
    return [sys.executable, "-c", PATCHED.format(patch=patch)]


def custom(*faults: dict) -> dict:
    return {"id": "custom", "seed": "0", "faults": list(faults)}


def receipts(r: RunResult) -> list[tuple]:
    return [(d.t_ms, r.sent[d.message].document["message_id"], d.receiver, d.kind, d.receipt, d.code)
            for d in r.deliveries]


def external_matches_reference(case: dict, **options) -> RunResult:
    external = run_case_external(PARSED, case, **options)
    assert receipts(external) == receipts(run_case(PARSED, case))
    return external


def of(r: RunResult, message_id: str) -> list[tuple]:
    return [(time_text(d.t_ms)[11:23], d.kind, d.receipt, d.code)
            for d in r.deliveries if r.sent[d.message].document["message_id"] == message_id]


def override(path: str, value) -> dict:
    return {"kind": "override", "path": path, "value": value}


# -------- N13(c): stages 1-3 recomputed and compared; stage 5 after stage 4 --------

def foreign_decision(sequence: int = 1) -> dict:
    """
    An authorization_decision to amr-07 from arm-02, which is not its authority: C401, a stage-5 code.
    It is sent in a conversation of its own, so its stream starts at 1 and the scenario is undisturbed.
    The sender is arm-02 because amr-07's view holds only arm-02's and its authority's keys: from
    planner-1 it would be C203 (finding H3 of the discrepancy register).
    """
    message = {"type": "authorization_decision", "profile": "a2r", "sender": A, "recipient": M,
               "conversation_id": "rl:conv:other", "sequence": sequence,
               "message_id": "m-foreign-decision", "payload": {
                   "goal": SUB_GOAL, "decision": "granted", "mode": "simulation", "constraints": [],
                   "lease": {"granted_at": DAY + "10:14:10.000Z", "expires_at": DAY + "11:00:00.000Z", "renewal": 0}}}
    return {"kind": "inject", "at": DAY + "10:14:10.000Z", "message": message}


def test_n13c_stage_5_fault_behind_a_sequence_gap_is_held_first():
    r = external_matches_reference(custom(foreign_decision(sequence=2)))
    assert of(r, "m-foreign-decision") == [("10:14:10.100", "original", "held", None),
                                           ("10:14:15.100", "released", "rejected", "C401")]


def test_n13c_duplicate_handling_comes_before_stage_5():
    duplicate = {"kind": "duplicate", "match": {"type": "authorization_decision", "sender": A, "n": 1},
                 "after": {"value": "1", "unit": "s"}}
    r = external_matches_reference(custom(foreign_decision(sequence=2), duplicate))
    assert of(r, "m-foreign-decision") == [("10:14:10.100", "original", "held", None),
                                           ("10:14:11.100", "duplicate", "duplicate", None),
                                           ("10:14:15.100", "released", "rejected", "C401")]


def test_n13c_in_order_stage_5_fault_is_still_rejected_with_its_code():
    r = external_matches_reference(custom(foreign_decision()))
    assert of(r, "m-foreign-decision") == [("10:14:10.100", "original", "rejected", "C401")]


V12A = {"id": "v12a", "seed": "0", "faults": [
    {"kind": "strip_signature", "match": {"type": "commit", "sender": A, "n": 1}}]}  # C201 at stage 1


@pytest.mark.parametrize("receipt, code", [("rejected", "C401"), ("rejected", "C306"), ("held", None)],
                         ids=["stage_5_code", "participant_code", "held"])
def test_n13c_genuine_stage_1_to_3_failure_needs_its_exact_code(receipt, code):
    lie = (f"real = p.Runtime.receipt\n"
           f"p.Runtime.receipt = lambda self, ref, r, k, receipt, code: "
           f"real(self, ref, r, k, *(({receipt!r}, {code!r}) if code == 'C201' else (receipt, code)))")
    with pytest.raises(InconsistentClaim, match="where Core 8 gives C201"):
        run_case_external(PARSED, V12A, command=patched(lie))


def test_n13c_genuine_stage_1_to_3_failure_with_the_right_code_passes():
    external_matches_reference(V12A)


# -------- H1: observations compared in meaning, with a valid signature --------

RESIGN = """
real = p.build_observation
def changed(*a):
    o = {{k: v for k, v in real(*a).items() if k != 'signature'}}
{change}
    return sign_sim(o, o['source']['participant'] + '#k1', a[-1])
p.build_observation = changed
"""


def resigned(change: str) -> list[str]:
    """The participant presents its observation changed by `change`, signed again with its own key."""
    return patched(RESIGN.format(change="\n".join("    " + line for line in change.splitlines())))


@pytest.mark.parametrize("change", [
    # 10:14:51.000Z -> 10:14:51Z; 10:14:51.200Z -> 10:14:51.2Z
    "t = o['observed_at']\n"
    "o['observed_at'] = t[:-5] + 'Z' if t.endswith('.000Z') else t[:-1].rstrip('0') + 'Z'",
    "o['confidence'] = o['confidence'] + '0'",
    "o['valid_for'] = {'value': str(int(o['valid_for']['value']) * 1000), 'unit': 'ms'}",
], ids=["timestamp_format", "decimal_format", "duration_unit"])
def test_h1_equivalent_representations_are_accepted(change):
    r = run_case_external(PARSED, custom(), command=resigned(change))
    assert r.planner_status == "succeeded"


@pytest.mark.parametrize("change", [
    "o['value'] = 'rl:robot:arm-02' if o['property'] == 'carried_by' else 'rl:place:dock-d1'",
    "o['subject'] = 'rl:thing:other'",
    "o['id'] = o['id'] + '0'",
    "o['observed_at'] = o['observed_at'][:-5] + '001Z'",
    "o['valid_for'] = {'value': '6', 'unit': 's'}",
    "o['confidence'] = '0.951'",
    "o['source'] = dict(o['source'], sensor='lidar')",
    "o['note'] = 'extra'",
], ids=["value", "subject", "id", "instant", "duration", "confidence", "source", "extra_field"])
def test_h1_altered_meaning_is_rejected(change):
    with pytest.raises(InconsistentClaim, match="observation not as the harness made it"):
        run_case_external(PARSED, custom(), command=resigned(change))


@pytest.mark.parametrize("patch", [
    # Another secret under the participant's own key reference: the signature does not verify.
    "real = p.build_observation\n"
    "p.build_observation = lambda *a: real(*a[:-1], Key(a[0], 'sim-hmac-sha256', b'x' * 32))",
    # The participant's own secret, but naming arm-02's key: not the participant's key.
    "real = p.build_observation\n"
    "def other(*a):\n"
    "    o = {k: v for k, v in real(*a).items() if k != 'signature'}\n"
    "    return sign_sim(o, 'rl:robot:arm-02#k1', a[-1])\n"
    "p.build_observation = other",
    # No signature at all.
    "real = p.build_observation\n"
    "p.build_observation = lambda *a: {k: v for k, v in real(*a).items() if k != 'signature'}",
], ids=["wrong_secret", "wrong_key_reference", "unsigned"])
def test_h1_invalid_or_wrongly_signed_observation_is_rejected(patch):
    with pytest.raises(InconsistentClaim, match="observation not as the harness made it"):
        run_case_external(PARSED, custom(), command=patched(patch))


# -------- H2: the five diagnostics, corroborated in the same cycle --------

DIAGNOSTIC_CASES = {
    "peer_ready": custom(),
    # arm-02 denies a reservation beyond the commitment's lease.
    "reservation_denied": custom(override("/parameters/reservation_end", DAY + "10:30:00.000Z")),
    # The reservation has expired (safe direction) when the final move is due (D5).
    "motion_refused": custom(override("/parameters/reservation_end", DAY + "10:14:51.000Z")),
    # No reading for `custody_confirmed` after the placement.
    "no_confirmation_evidence": custom({"kind": "sensor_unavailable", "participant": M, "sensor": "deck_camera",
                                        "from": DAY + "10:14:50.000Z"}),
    # A reading for the confirmation (10:14:51.000), none after the commit point (10:14:51.200).
    "physical_custody_unknown": custom({"kind": "sensor_unavailable", "participant": M, "sensor": "deck_camera",
                                        "from": DAY + "10:14:51.100Z"}),
}


@pytest.mark.parametrize("record", sorted(DIAGNOSTIC_CASES))
def test_h2_corroborated_diagnostic_is_accepted(record):
    r = external_matches_reference(DIAGNOSTIC_CASES[record])
    assert [e.get("record") for e in r.events if e.participant == M and e.get("record") == record] == [record]


EMIT_ON_COMMIT = """
real = Amr.on_commit
def claim(self, d):
    real(self, d); self.event('state', **{detail!r})
Amr.on_commit = claim
"""


@pytest.mark.parametrize("detail", [
    {"record": "peer_ready"},
    {"record": "reservation_denied"},
    {"record": "no_confirmation_evidence"},
    {"record": "physical_custody_unknown"},
    {"record": "peer_ready", "extra": 1},
], ids=["peer_ready", "reservation_denied", "no_confirmation_evidence", "physical_custody_unknown", "extra_field"])
def test_h2_unsupported_diagnostic_is_inconsistent(detail):
    with pytest.raises(InconsistentClaim, match="diagnostic not consistent"):
        run_case_external(PARSED, custom(), command=patched(EMIT_ON_COMMIT.format(detail=detail)))


def test_h2_motion_refused_with_an_act_in_the_same_cycle_is_inconsistent():
    patch = ("real = Amr._move\n"
             "def both(self, to, done):\n"
             "    real(self, to, done); self.event('state', record='motion_refused', to=to)\n"
             "Amr._move = both")
    with pytest.raises(InconsistentClaim, match="diagnostic not consistent"):
        run_case_external(PARSED, custom(), command=patched(patch))


def test_h2_physical_custody_unknown_before_the_commit_point_is_inconsistent():
    # A null reading in the cycle, but before any acknowledged release.
    patch = ("real = Amr.event\n"
             "def also(self, kind, **detail):\n"
             "    real(self, kind, **detail)\n"
             "    if detail.get('record') == 'no_confirmation_evidence':\n"
             "        real(self, 'state', record='physical_custody_unknown')\n"
             "Amr.event = also")
    with pytest.raises(InconsistentClaim, match="diagnostic not consistent"):
        run_case_external(PARSED, DIAGNOSTIC_CASES["no_confirmation_evidence"], command=patched(patch))


def test_h2_unknown_state_record_is_a_binding_failure():
    with pytest.raises(BindingFailure, match="unknown state record"):
        run_case_external(PARSED, custom(), command=patched(EMIT_ON_COMMIT.format(detail={"record": "teleported"})))


def test_h2_omitted_diagnostics_are_accepted():
    patch = ("real = Amr.event\n"
             "Amr.event = lambda self, kind, **d: None if d.get('record') in ("
             "'peer_ready', 'reservation_denied', 'motion_refused', 'no_confirmation_evidence', "
             "'physical_custody_unknown') else real(self, kind, **d)")
    for case in DIAGNOSTIC_CASES.values():
        run_case_external(PARSED, case, command=patched(patch))


# -------- N3: the hold ledger; the harness closes holds while the participant is silent --------

def held_by_amr(sequence: int, message_id: str, at: str = "10:14:00.150Z") -> dict:
    """A message from arm-02 to amr-07 ahead of its stream (sequence 2 is next): amr-07 holds it."""
    return {"kind": "inject", "at": DAY + at, "message": {
        "type": "custody_query", "profile": "r2r", "sender": A, "recipient": M, "sequence": sequence,
        "message_id": message_id, "payload": {"thing": "rl:thing:container-c42"}}}


SILENT_FROM_COMMIT = {"kind": "silence", "participant": M, "from": {"type": "commit", "sender": A, "n": 1}}


def max_hold(value: str) -> dict:
    return override("/parameters/max_hold", {"value": value, "unit": "ms"})


def released(r: RunResult) -> dict[str, list[tuple]]:
    """Every release receipt, by message: each held message must get exactly one."""
    out: dict[str, list[tuple]] = {}
    for d in r.deliveries:
        if d.kind == "released":
            out.setdefault(r.sent[d.message].document["message_id"], []).append((d.receipt, d.code))
    return out


def test_n3_silent_with_a_hold_max_hold_first():
    # Delivered 10:14:00.250; max_hold 5 s; expires 10:14:30.150 - 0.5 s.
    r = external_matches_reference(custom(held_by_amr(9, "m-held"), SILENT_FROM_COMMIT))
    assert of(r, "m-held") == [("10:14:00.250", "original", "held", None),
                               ("10:14:05.250", "released", "not_handled", None)]


def test_n3_silent_with_a_hold_expiry_first():
    # max_hold 29.5 s (shorter than the 30 s lifetime, Core 8.4): 10:14:29.750 is after the expiry instant.
    r = external_matches_reference(custom(held_by_amr(9, "m-held"), SILENT_FROM_COMMIT, max_hold("29500")))
    assert of(r, "m-held")[-1] == ("10:14:29.650", "released", "rejected", "C301")


def test_n3_expiry_wins_a_tie():
    # delivery + max_hold = 10:14:00.250 + 29.4 s = expires_at - tolerance = 10:14:29.650.
    r = external_matches_reference(custom(held_by_amr(9, "m-held"), SILENT_FROM_COMMIT, max_hold("29400")))
    assert of(r, "m-held")[-1] == ("10:14:29.650", "released", "rejected", "C301")


def test_n3_consecutive_holds_are_released_together_in_sequence_order():
    r = external_matches_reference(custom(held_by_amr(9, "m-held-9"), held_by_amr(10, "m-held-10", "10:14:00.160Z"),
                                          SILENT_FROM_COMMIT))
    assert of(r, "m-held-9")[-1] == ("10:14:05.250", "released", "not_handled", None)
    assert of(r, "m-held-10")[-1] == ("10:14:05.250", "released", "not_handled", None)  # now next
    order = [r.sent[d.message].document["message_id"] for d in r.deliveries if d.kind == "released"]
    assert order == ["m-held-9", "m-held-10"]


def test_n3_a_hold_that_is_not_next_waits_for_its_own_instant():
    r = external_matches_reference(custom(held_by_amr(9, "m-held-9"), held_by_amr(11, "m-held-11", "10:14:00.160Z"),
                                          SILENT_FROM_COMMIT))
    assert of(r, "m-held-11")[-1] == ("10:14:05.260", "released", "not_handled", None)


def test_n3_no_duplicate_terminal_receipt():
    # Released by the live participant first, then silence: the harness adds nothing.
    silent_later = {"kind": "silence", "participant": M, "from": {"type": "coordinate", "sender": A, "n": 1}}
    r = external_matches_reference(custom(held_by_amr(9, "m-held"), silent_later))
    assert released(r) == {"m-held": [("accepted", None)]}  # amr-07's own release, at its max_hold
    for case in (custom(held_by_amr(9, "m-held"), SILENT_FROM_COMMIT),
                 custom(held_by_amr(9, "m-held-9"), held_by_amr(10, "m-held-10", "10:14:00.160Z"), SILENT_FROM_COMMIT)):
        assert all(len(v) == 1 for v in released(run_case_external(PARSED, case)).values())
