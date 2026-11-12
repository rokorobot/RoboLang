"""Physical-run authorization chain (decision 0038 step 2): cell configuration, task approval, run grant."""

from __future__ import annotations
import base64
import json
from pathlib import Path

import pytest

pytest.importorskip("cryptography")

from robolang import authorization as auth  # noqa: E402
from robolang.core import ED25519, Key, generate_ed25519, sign_ed25519, time_ms  # noqa: E402
from robolang.ir import compile_source  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
IR = compile_source((ROOT / "src" / "robolang" / "reference" / "fetch.rob").read_text(), "fetch.rob").ir
NOW = time_ms("2026-11-12T10:00:00Z")

OWNER, REVIEWER, OPERATOR = "rl:human:owner", "rl:human:reviewer-1", "rl:human:operator-1"
PAIRS = {who: generate_ed25519() for who in (OWNER, REVIEWER, OPERATOR, "rl:human:stranger")}


def b64(raw: bytes) -> str:
    return base64.urlsafe_b64encode(raw).rstrip(b"=").decode()


def owner_keys():
    return {f"{OWNER}#k1": Key(OWNER, ED25519, public_key=PAIRS[OWNER][1])}


def signed(doc, who):
    return sign_ed25519(doc, f"{who}#k1", PAIRS[who][0])


def cell(**change):
    doc = {
        "kind": auth.CELL, "version": "0.1", "id": "rl:cell_config:lab-1-v1", "robot": "rl:robot:arm-1",
        "cell": "rl:cell:lab-1",
        "safety_case": {"version": "0.1", "sha256": "a" * 64},
        "commissioning": {"record": "commissioning-2026-11-12", "sha256": "b" * 64},
        "keys": {f"{w}#k1": {"alg": "ed25519", "public_key": b64(PAIRS[w][1])} for w in (REVIEWER, OPERATOR)},
        "authorities": [OPERATOR], "reviewers": [REVIEWER], "max_grant_lease_s": 900,
        "safe_state": {"position": "hold", "gripper": "unchanged"},
        "valid_from": "2026-11-01T00:00:00Z", "valid_until": "2026-12-01T00:00:00Z",
    }
    return signed({**doc, **change}, OWNER)


def approval(**change):
    doc = {
        "kind": auth.APPROVAL, "version": "0.1", "id": "rl:approval:fetch-1", "ir_sha256": auth.ir_sha256(IR),
        "task": "fetch", "language_version": IR.language_version, "robot": "rl:robot:arm-1", "cell": "rl:cell:lab-1",
        "author": "model:claude-opus-5-5", "reviewer": REVIEWER, "approved_at": "2026-11-12T09:50:00Z",
    }
    return signed({**doc, **change}, change.get("reviewer", REVIEWER))


def grant(approval_doc=None, **change):
    doc = {
        "kind": auth.GRANT, "version": "0.1", "id": "rl:grant:fetch-1-run-1",
        "approval_sha256": auth.document_sha256(approval_doc or approval()), "robot": "rl:robot:arm-1",
        "cell": "rl:cell:lab-1", "operator": OPERATOR, "mode": "physical",
        "granted_at": "2026-11-12T09:55:00Z", "expires_at": "2026-11-12T10:05:00Z",
    }
    return signed({**doc, **change}, change.get("operator", OPERATOR))


def code(c=None, a=None, g=None, ir=IR, now=NOW, keys=None):
    a = a or approval()
    try:
        auth.check_chain(ir, c or cell(), a, g or grant(a), keys or owner_keys(), now)
    except auth.AuthorizationError as exc:
        return exc.code, exc.document, exc.pointer
    return "ok"


def test_a_valid_chain_is_still_refused():
    chain = auth.check_chain(IR, cell(), approval(), grant(), owner_keys(), NOW)
    assert chain.approval_sha256 == auth.document_sha256(approval())
    assert auth.refuse_physical(chain).code == "A731"


def test_documents_also_read_from_bytes():
    a = approval()
    as_bytes = [json.dumps(d).encode() for d in (cell(), a, grant(a))]
    assert auth.check_chain(IR, *as_bytes, owner_keys(), NOW)


@pytest.mark.parametrize("data, expected", [
    (b"{", "A701"), (b"[]", "A701"), (b'{"kind": "robolang.cell_configuration", "version": "0.2"}', "A701"),
    (b'{"kind": "robolang.cell_configuration", "version": "0.1", "version": "0.1"}', "A701"),
    (b'{"kind": "robolang.cell_configuration", "version": "0.1", "max_grant_lease_s": 1.5}', "A701"),
])
def test_unreadable_cell(data, expected):
    assert code(c=data)[0] == expected


@pytest.mark.parametrize("change, pointer", [
    ({"extra": 1}, "/extra"),
    ({"safe_state": {"position": "home", "gripper": "unchanged"}}, "/safe_state"),
    ({"max_grant_lease_s": 0}, "/max_grant_lease_s"),
    ({"authorities": []}, "/authorities"),
    ({"keys": {f"{REVIEWER}#k1": {"alg": "ed25519", "public_key": "short"}}}, "/keys"),
    ({"safety_case": {"version": "0.1"}}, "/safety_case"),
])
def test_cell_form(change, pointer):
    assert code(c=cell(**change)) == ("A702", "cell", pointer)


def test_cell_must_be_signed_by_an_installed_owner_key():
    other_seed, _ = generate_ed25519()
    forged = sign_ed25519({k: v for k, v in cell().items() if k != "signature"}, f"{OWNER}#k1", other_seed)
    assert code(c=forged)[:2] == ("A713", "cell")
    by_reviewer = signed({k: v for k, v in cell().items() if k != "signature"}, REVIEWER)
    assert code(c=by_reviewer)[:2] == ("A712", "cell")
    assert code(keys={"x": Key(OWNER, "sim-hmac-sha256", secret=b"s")})[:2] == ("A712", "cell")
    tampered = {**cell(), "max_grant_lease_s": 86400}
    assert code(c=tampered)[:2] == ("A713", "cell")


def test_cell_must_be_current():
    assert code(now=time_ms("2026-12-01T00:00:00Z"))[:2] == ("A721", "cell")
    assert code(now=time_ms("2026-10-31T23:59:59Z"))[:2] == ("A721", "cell")


def test_approval_must_name_this_task_robot_and_cell():
    other = compile_source((ROOT / "src" / "robolang" / "reference" / "fetch.rob").read_text()
                           .replace("place r box at dst;", ""), "x.rob").ir
    assert code(ir=other) == ("A722", "approval", "/ir_sha256")
    assert code(a=approval(task="other")) == ("A722", "approval", "/task")
    assert code(a=approval(robot="rl:robot:arm-2")) == ("A722", "approval", "/robot")
    assert code(a=approval(cell="rl:cell:lab-2")) == ("A722", "approval", "/cell")


def test_reviewer_rules():
    assert code(a=approval(reviewer=OPERATOR)) == ("A723", "approval", "/reviewer")
    assert code(a=approval(author=REVIEWER)) == ("A723", "approval", "/reviewer")
    assert code(a=approval(reviewer="rl:human:stranger"))[:2] == ("A712", "approval")


def test_grant_must_name_this_approval_and_come_from_an_authority():
    a = approval()
    assert code(a=a, g=grant(approval(id="rl:approval:other"))) == ("A724", "grant", "/approval_sha256")
    assert code(a=a, g=grant(a, robot="rl:robot:arm-2")) == ("A724", "grant", "/robot")
    assert code(a=a, g=grant(a, operator=REVIEWER)) == ("A725", "grant", "/operator")
    assert code(a=a, g=grant(a, mode="simulation")) == ("A702", "grant", "/mode")


def test_grant_lease_and_time():
    a = approval()
    assert code(a=a, g=grant(a, expires_at="2026-11-12T10:10:01Z"))[0] == "A726"  # 901 s > 900
    assert code(a=a, g=grant(a, expires_at="2026-11-12T09:55:00Z"))[0] == "A726"  # empty
    assert code(a=a, now=time_ms("2026-11-12T10:05:00Z"))[0] == "A726"  # expired


def test_clock_tolerance_is_applied_on_the_safe_side():
    a = approval()
    args = (IR, cell(), a, grant(a), owner_keys())
    auth.check_chain(*args, time_ms("2026-11-12T10:04:59Z"))
    with pytest.raises(auth.AuthorizationError, match="A726"):
        auth.check_chain(*args, time_ms("2026-11-12T10:04:59Z"), clock_tolerance_ms=2000)


def test_ir_hash_is_the_same_from_rob_and_from_ir_json():
    from robolang.ir import to_json
    from robolang.ir_reader import read_ir
    assert auth.ir_sha256(read_ir(to_json(IR).encode())) == auth.ir_sha256(IR)
