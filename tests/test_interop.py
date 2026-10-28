"""
INTEROP-1: the binding self-test (spec/interop-gate-2-v0.1.md section 11, decision record 0009).

The reference amr-07 runs as a separate process through `robolang-sim-stdio` 0.1, knowing only
its view (6.4), and every case's run record must equal the F1 golden record byte for byte. The
harness checks are shown to work by participants that break the binding or make claims the
harness cannot corroborate (9.1.1).
"""

from __future__ import annotations
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from robolang.arch02 import Simulation, load_scenario, to_json
from robolang.interop import (
    AMR,
    EVENT_KINDS_0009,
    BindingFailure,
    InconsistentClaim,
    build_view,
    run_case_external,
    simulate_case_external,
)

ROOT = Path(__file__).resolve().parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "arch02"
SCENARIO = (FIXTURES / "scenario.json").read_bytes()
PARSED = load_scenario(SCENARIO)
GOLDEN = json.loads((FIXTURES / "golden_records.json").read_text(encoding="utf-8"))["records"]
CASES = {
    f"{name}#{case['id']}": case
    for name in ("cases.json", "sequencing_cases.json")
    for case in json.loads((FIXTURES / name).read_text(encoding="utf-8"))["cases"]
}
BY_ID = {case["id"]: case for case in CASES.values()}


def case(case_id: str) -> dict:
    return dict(BY_ID[case_id], seed=BY_ID[case_id].get("seed", "0"))


# -------- the self-test: all 24 cases, external amr-07, identical records --------

@pytest.mark.parametrize("key", sorted(CASES))
def test_external_amr07_reproduces_the_golden_record(key):
    record = simulate_case_external(SCENARIO, CASES[key])
    assert record["run_id"] == GOLDEN[key]["run_id"]
    assert hashlib.sha256(to_json(record).encode("ascii")).hexdigest() == GOLDEN[key]["record_sha256"], key


def test_finding_e1_event_kinds_of_0009_alone_lose_lease_expired():
    """With only the event kinds 0009 approved, amr-07's `lease_expired` claim has no frame (E1, erratum 0010)."""
    failing = []
    for case_id in ("v05_stale_confirmation", "v06_partition_before_release", "happy_path"):
        try:
            run_case_external(PARSED, case(case_id), event_kinds=EVENT_KINDS_0009)
        except BindingFailure as failure:
            assert "lease_expired" in str(failure)
            failing.append(case_id)
    assert failing == ["v05_stale_confirmation", "v06_partition_before_release"]


# -------- the view (6.4) --------

def test_view_holds_only_what_6_4_allows():
    sim = Simulation(PARSED, case("happy_path"))
    view = build_view(sim.scenario, AMR)
    assert set(view) == {
        "participant", "key", "descriptor", "standing_authorization", "policy", "sensors", "location",
        "parameters", "conversation_id", "participants", "places", "arbiters", "authority", "verification_keys",
    }
    assert [p["id"] for p in view["participants"]] == [AMR, "rl:robot:arm-02", "rl:human:shift-lead-3"]
    assert sorted(view["verification_keys"]) == ["rl:human:shift-lead-3#k1", "rl:robot:arm-02#k1"]
    assert set(view["parameters"]) == {"epoch", "message_ttl", "max_message_lifetime", "clock_tolerance",
                                       "max_hold", "offer_lease_end", "reservation_end"}
    assert [s["sensor"] for s in view["sensors"]] == ["deck_camera"]
    text = json.dumps(view)
    private = [sim.scenario["keys"]["rl:agent:planner-1#k1"]["secret"], sim.scenario["goal"]["id"],
               "authority_table", "durations", "cell_camera", "happy_path", "expect", "faults"]
    private += [json.dumps(sim.scenario["standing_authorizations"]["rl:robot:arm-02"]["id"])]
    assert [item for item in private if item in text] == []


def test_view_follows_the_case_overrides():
    declined = build_view(Simulation(PARSED, case("v01_declines")).scenario, AMR)
    assert declined["policy"] == {"accept_assistance": False}


# -------- harness checks: framing and lock-step (6.5) --------

def scripted(code: str) -> list[str]:
    return [sys.executable, "-c", code]


PATCHED = """
import sys
from robolang.interop import participant as p
from robolang.arch02.participants import Amr
{patch}
p.serve(sys.stdin.buffer, sys.stdout.buffer)
"""


def patched(patch: str) -> list[str]:
    return scripted(PATCHED.format(patch=patch))


@pytest.mark.parametrize("command, message", [
    (scripted("import time; time.sleep(30)"), "no frame within"),
    (scripted("import sys, time; sys.stdin.readline(); print('not json', flush=True); time.sleep(30)"),
     "invalid JSON"),
    (scripted("import sys, time; sys.stdin.readline(); print('{\"frame\": \"teleport\"}', flush=True); "
              "time.sleep(30)"), "unknown frame"),
    (scripted("import sys; sys.stdin.readline()"), "closed its output"),
    (patched("p.serve(sys.stdin.buffer, sys.stdout.buffer); print('{\"frame\": \"idle\"}', flush=True); "
             "raise SystemExit"), "output after end"),
], ids=["timeout", "invalid_json", "unknown_frame", "exit", "output_after_end"])
def test_binding_failures(command, message):
    with pytest.raises(BindingFailure, match=message):
        run_case_external(PARSED, case("happy_path"), command=command, frame_timeout_s=2.0)


# -------- harness checks: corroboration of claims (9.1.1) --------

@pytest.mark.parametrize("case_id, patch, message", [
    # A receipt that contradicts Core 8 stages 1-3, recomputed by the harness.
    ("v12a_unsigned_commit",
     "real = p.Runtime.receipt\n"
     "p.Runtime.receipt = lambda self, ref, r, k, receipt, code: real(self, ref, r, k, 'accepted', None)",
     "where Core 8 gives"),
    # Motion with no authorization on the wire and none standing (the payload is beyond it). The
    # patch bypasses amr-07's own pre-move checks (D5) to act directly.
    ("v11_payload_beyond_standing_authorization",
     "def go(self):\n    self._state('RUNNING'); self.sim.move(self.id, self.handoff, lambda: None)\n"
     "Amr._authorize = go",
     "without an authorization"),
    # An authorization claim the harness cannot establish.
    ("happy_path",
     "real = Amr._authorize\n"
     "def claim(self):\n    self.event('authorization', decision='granted', basis='rl:sa:made-up'); real(self)\n"
     "Amr._authorize = claim",
     "authorization claim not established"),
    # A commit point claimed before any release was received and acknowledged.
    ("happy_path",
     "real = Amr.on_transfer_custody\n"
     "def early(self, d):\n"
     "    real(self, d); self.event('custody_committed', thing=self.thing, release=d['message_id'])\n"
     "Amr.on_transfer_custody = early",
     "commit point without"),
    # A SUCCEEDED goal with no outcome of succeeded on the wire.
    ("happy_path", "Amr._outcome = lambda self, *a, **k: None", "SUCCEEDED goal without"),
    # An escalation with no expired lease.
    ("happy_path",
     "real = Amr.on_commit\n"
     "def esc(self, d):\n    real(self, d); self.event('escalated', reason='custody_unknown')\n"
     "Amr.on_commit = esc",
     "escalation without"),
    # A message sent in someone else's name.
    ("happy_path",
     "real = p.Runtime.send\n"
     "p.Runtime.send = lambda self, s, *a, **k: real(self, 'rl:robot:arm-02', *a, **k)",
     "as someone else"),
    # Erratum 0010, B1: a lease_expired claim before the lease has expired.
    ("happy_path",
     "real = Amr.on_commit\n"
     "def early(self, d):\n"
     "    real(self, d); self.event('lease_expired', lease='commitment', commitment=self.commitment['id'])\n"
     "Amr.on_commit = early",
     "lease_expired claim not established"),
    # B1: a lease_expired claim for a commitment that was never accepted.
    ("v12a_unsigned_commit",
     "real = Amr.on_request_assistance\n"
     "def lie(self, d):\n"
     "    real(self, d); self.event('lease_expired', lease='commitment', commitment='rl:commit:c42-transport')\n"
     "Amr.on_request_assistance = lie",
     "lease_expired claim not established"),
    # B2: a commitment state with no accepted commitment.
    ("happy_path",
     "real = Amr.on_request_assistance\n"
     "def lie(self, d):\n    real(self, d); self.commitment_state = 'active'\n"
     "Amr.on_request_assistance = lie",
     "commitment state None -> active not supported"),
    # B2: a transition Core 5.8 does not allow (straight to failed, never active).
    ("happy_path",
     "real = Amr.on_commit\n"
     "def skip(self, d):\n    real(self, d); self.commitment_state = 'failed'\n"
     "Amr.on_commit = skip",
     "commitment state None -> failed not supported"),
    # B2, after a corroborated `active`: lapsed before the lease has expired.
    ("happy_path",
     "real = Amr.on_reservation_decision\n"
     "def lapse(self, d):\n    real(self, d); self.commitment_state = 'lapsed'\n"
     "Amr.on_reservation_decision = lapse",
     "commitment state lapsed not supported by the wire"),
    # B2: fulfilled with no successful outcome on the wire.
    ("happy_path",
     "real = Amr.on_reservation_decision\n"
     "def done(self, d):\n    real(self, d); self.commitment_state = 'fulfilled'\n"
     "Amr.on_reservation_decision = done",
     "commitment state fulfilled not supported by the wire"),
    # B2: fulfilled with a succeeded outcome but no evidence the harness observed.
    ("happy_path",
     "def arrived(self):\n"
     "    self.commitment_state = 'fulfilled'; self._state('SUCCEEDED')\n"
     "    self._outcome('succeeded', verification=None, evidence=())\n"
     "Amr._arrived = arrived",
     "commitment state fulfilled not supported by the wire"),
    # B2: failed without any outcome sent.
    ("happy_path",
     "real = Amr.on_reservation_decision\n"
     "def fail(self, d):\n    real(self, d); self.commitment_state = 'failed'\n"
     "Amr.on_reservation_decision = fail",
     "commitment state failed not supported by the wire"),
], ids=["receipt_stage_2", "motion_unauthorized", "authorization_claim", "commit_point", "succeeded",
        "escalation", "impersonation", "lease_expired_early", "lease_expired_no_commitment",
        "commitment_without_commit", "commitment_transition", "lapsed_early", "fulfilled_without_outcome",
        "fulfilled_without_evidence", "failed_without_outcome"])
def test_inconsistent_claims_fail_the_run(case_id, patch, message):
    with pytest.raises(InconsistentClaim, match=message):
        run_case_external(PARSED, case(case_id), command=patched(patch))


# Observation provenance: an observation the participant presents must equal, field for field and
# signature included, the one built from the harness's reading. Each patch re-signs or alters one
# protected field of a genuinely sensed observation (same ID and value).
@pytest.mark.parametrize("patch", [
    "real = p.build_observation\np.build_observation = lambda *a: real(*a[:5], a[5] - 1000, *a[6:])",
    "real = p.build_observation\np.build_observation = lambda *a: real(*a[:8], '0.99', *a[9:])",
    "real = p.build_observation\np.build_observation = lambda *a: real(*a[:9], 'lidar', *a[10:])",
    "real = p.build_observation\n"
    "def forged(*a):\n    o = real(*a); return dict(o, signature=dict(o['signature'], value='0' * 64))\n"
    "p.build_observation = forged",
], ids=["observed_at", "confidence", "method", "signature"])
def test_altered_observations_fail_the_run(patch):
    with pytest.raises(InconsistentClaim, match="observation not as the harness made it"):
        run_case_external(PARSED, case("happy_path"), command=patched(patch))


# -------- isolation --------

FORBIDDEN_LOADED = ("socket", "ssl", "http", "urllib.request", "rclpy", "robolang_adapter", "robolang_runtime")


def test_the_participant_process_loads_no_network_or_adapter_module():
    probe = (
        "import sys, robolang.interop.participant; "
        f"print([m for m in sys.modules if any(m == f or m.startswith(f + '.') for f in {FORBIDDEN_LOADED!r})])"
    )
    result = subprocess.run([sys.executable, "-c", probe], capture_output=True, text=True, check=True,
                            env={**os.environ, "PYTHONPATH": str(ROOT / "src")})
    assert result.stdout.strip() == "[]"
