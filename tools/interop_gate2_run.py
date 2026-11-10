"""INTEROP-4: run gate 2 (spec/interop-gate-2-v0.1.md sections 8 and 9) with an independent amr-07.

Usage: python tools/interop_gate2_run.py --cmd "node /path/to/impl/dist/src/main.js run" --out results.json

For every case of G-T1 to G-T4 the independent amr-07 runs as an external process through
`robolang-sim-stdio` 0.1, and the run is checked against section 9.1:
  1. every `expect` field holds (the reference's own evaluator, tests/test_arch02.py `observed`);
  2. every receipt equals the reference run's (receipt and code, per delivery);
  3. the sent messages (type, sender, recipient, send time) equal the reference run's;
  4./5. no inconsistent claim and no binding failure (the harness raises them, 9.1.1 and 6.5).
G-T5 runs every case a second time and compares the two gate 2 records byte for byte.
JCS identity of each message with the reference (9.2) is measured and reported, not required.
SIMULATION ONLY. The independent program is untrusted; it only gets the binding's standard streams.
"""
from __future__ import annotations
import argparse
import json
import shlex
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "src"), str(ROOT)]

from robolang.arch02 import load_scenario, to_json  # noqa: E402
from robolang.arch02.engine import run_case  # noqa: E402
from robolang.core.canonical import canonical_bytes  # noqa: E402
from robolang.interop import BindingFailure, InconsistentClaim, run_case_external, simulate_case_external  # noqa: E402
from tests.test_arch02 import observed  # noqa: E402  (the reference simulation 11 evaluator)

FIXTURES = ROOT / "tests" / "fixtures" / "arch02"
SCENARIO = (FIXTURES / "scenario.json").read_bytes()
PARSED = load_scenario(SCENARIO)
CASES = {c["id"]: c for name in ("cases.json", "sequencing_cases.json")
         for c in json.loads((FIXTURES / name).read_text(encoding="utf-8"))["cases"]}
GROUPS = {
    "G-T1": ["happy_path", "happy_path_seed_1"],
    "G-T2": ["v01_declines", "v02_duplicate_commit", "v03_replayed_commit", "v05_stale_confirmation",
             "v06_partition_before_release", "v07_release_ack_lost", "v08_arrived_without_evidence",
             "v11_payload_beyond_standing_authorization", "v12a_unsigned_commit", "v12b_wrongly_signed_commit"],
    "G-T3": sorted(k for k in CASES if k.startswith("seq_")),
    "G-T4": ["v04_assistant_silent", "v09_stale_reverification", "v10_requester_authorizes", "v13_no_grasp"],
}


def receipts(r):
    return [(d.t_ms, r.sent[d.message].document["message_id"], d.receiver, d.kind, d.receipt, d.code)
            for d in r.deliveries]


def sends(r):
    return [(s.document["type"], s.document["sender"], s.document["recipient"], s.t_sent_ms) for s in r.sent]


def first_difference(a, b):
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return {"index": i, "independent": list(x), "reference": list(y)}
    return {"index": min(len(a), len(b)), "lengths": [len(a), len(b)]} if len(a) != len(b) else None


def check(case_id: str, cmd: list[str]) -> dict:
    case = dict(CASES[case_id], seed=CASES[case_id].get("seed", "0"))
    out = {"case": case_id}
    ref = run_case(PARSED, case)
    try:
        ext = run_case_external(PARSED, case, command=cmd)
    except (BindingFailure, InconsistentClaim) as exc:
        out.update(passed=False, failure=f"{type(exc).__name__}: {exc}")
        return out
    except Exception as exc:  # anything else is reported, never hidden
        out.update(passed=False, failure=f"{type(exc).__name__}: {exc}", trace=traceback.format_exc()[-1500:])
        return out
    expect_fail = {}
    for field, expected in case["expect"].items():
        want = sorted(expected) if field == "escalated" else expected  # as tests/test_arch02.py
        got = observed(ext, field, case["expect"])
        if got != want:
            expect_fail[field] = {"independent": got, "expected": want}
    receipt_diff = first_difference(receipts(ext), receipts(ref))
    send_diff = first_difference(sends(ext), sends(ref))
    jcs_same = sum(canonical_bytes(a.document) == canonical_bytes(b.document) for a, b in zip(ext.sent, ref.sent))
    rec1 = to_json(simulate_case_external(SCENARIO, CASES[case_id], command=cmd))
    rec2 = to_json(simulate_case_external(SCENARIO, CASES[case_id], command=cmd))
    out.update(
        expectations_ok=not expect_fail, expectation_failures=expect_fail,
        receipts_ok=receipt_diff is None, receipt_difference=receipt_diff,
        sends_ok=send_diff is None, send_difference=send_diff,
        deterministic=rec1 == rec2,
        jcs_identical=f"{jcs_same}/{len(ref.sent)}",
    )
    out["passed"] = all((out["expectations_ok"], out["receipts_ok"], out["sends_ok"], out["deterministic"]))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cmd", required=True, help="the independent amr-07 process, e.g. 'node dist/src/main.js run'")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    cmd = shlex.split(args.cmd)
    results = {g: [check(c, cmd) for c in ids] for g, ids in GROUPS.items()}
    flat = [r for rs in results.values() for r in rs]
    summary = {g: f"{sum(r['passed'] for r in rs)}/{len(rs)}" for g, rs in results.items()}
    summary["G-T5 deterministic"] = f"{sum(r.get('deterministic', False) for r in flat)}/{len(flat)}"
    Path(args.out).write_text(json.dumps({"command": cmd, "summary": summary, "results": results}, indent=2) + "\n")
    print(json.dumps(summary))
    for r in flat:
        if not r["passed"]:
            print("FAIL", r["case"], {k: v for k, v in r.items() if k not in ("case", "passed", "trace")})
    return 0 if all(r["passed"] for r in flat) else 1


if __name__ == "__main__":
    sys.exit(main())
