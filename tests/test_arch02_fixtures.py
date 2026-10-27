"""
Consistency checks for the RL-ARCH-02a fixtures (tests/fixtures/core, tests/fixtures/arch02).

No Core implementation or reference simulation exists yet (ARCH-1 to ARCH-3). These tests
execute no Core check and run nothing: they check that the fixtures, the two schema files and
the specifications (spec/core-v0.1.md, spec/reference-simulation-v0.1.md) agree, so that a
later implementation is tested against fixtures that are themselves sound. Signatures are
recomputed with the standard library, as Core section 7 defines them.
"""

from __future__ import annotations
import hashlib
import hmac
import json
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).parent.parent
CORE = ROOT / "tests" / "fixtures" / "core"
ARCH = ROOT / "tests" / "fixtures" / "arch02"
CORE_SPEC = (ROOT / "spec" / "core-v0.1.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "reference-simulation-v0.1.md").read_text(encoding="utf-8")
MESSAGE_SCHEMA = ROOT / "spec" / "core" / "message-0.1.schema.json"
DESCRIPTOR_SCHEMA = ROOT / "spec" / "core" / "capability-descriptor-0.1.schema.json"


def strict(pairs):
    keys = [key for key, _ in pairs]
    assert len(keys) == len(set(keys)), f"duplicate key in {keys}"
    return dict(pairs)


def load(path: Path):
    data = path.read_bytes()
    assert not data.startswith(b"\xef\xbb\xbf"), path.name
    return json.loads(data.decode("utf-8"), object_pairs_hook=strict)


CONTEXT = load(CORE / "context.json")
CORE_CASES = load(CORE / "cases.json")["cases"]
SCENARIO = load(ARCH / "scenario.json")
ARCH_CASES = load(ARCH / "cases.json")["cases"]


def table_column(text: str, heading: str) -> list[list[str]]:
    """Rows of the first table after `heading`, split into cells."""
    section = text[text.index(heading):]
    rows = []
    for line in section.splitlines()[1:]:
        if rows and not line.startswith("|"):
            break
        if line.startswith("|") and not line.startswith("|---"):
            rows.append([cell.strip() for cell in line.strip("|").split("|")])
    return rows[1:]


CODES = {row[0].strip("`"): row[1] for row in table_column(CORE_SPEC, "### 8.2 Rejection codes")}
REASONS = {row[0].strip("`") for row in table_column(CORE_SPEC, "### 5.11 Reason codes")}
DEFINED_TYPES = [row[0].strip("`") for row in table_column(CORE_SPEC, "### 4.1 Defined in 0.1")]
RESERVED_TYPES = re.search(r"### 4\.2 Reserved in 0\.1.*?```text\n(.*?)\n```", CORE_SPEC, re.S).group(1).split()
FAULT_KINDS = {row[0].strip("`") for row in table_column(SIM_SPEC, "## 10. Faults and overrides")}
EXPECT_FIELDS = {name.strip("` ") for row in table_column(SIM_SPEC, "`expect` fields") for name in row[0].split(",")}
STATUSES = {"succeeded", "failed", "cancelled", "timed_out", "rejected", "unknown"}


def jcs(doc) -> bytes:
    """Core section 7: JCS, which for integer-only documents is sorted keys and no whitespace."""
    return json.dumps(doc, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def signature_verifies(doc, keys) -> bool:
    sig = doc["signature"]
    body = {k: v for k, v in doc.items() if k != "signature"}
    expected = hmac.new(bytes.fromhex(keys[sig["key"]]["secret"]), jcs(body), hashlib.sha256).hexdigest()
    return sig["alg"] == "sim-hmac-sha256" and hmac.compare_digest(sig["value"], expected)


def signer(doc) -> str:
    return doc.get("sender") or doc.get("robot") or doc.get("issued_by") or doc["source"]["participant"]


# -------- schemas and specification --------

def test_schema_files_are_draft_2020_12_and_list_the_defined_types():
    message = load(MESSAGE_SCHEMA)
    descriptor = load(DESCRIPTOR_SCHEMA)
    for schema in (message, descriptor):
        assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert message["properties"]["type"]["enum"] == DEFINED_TYPES
    assert len(DEFINED_TYPES) == 20 and len(RESERVED_TYPES) == 10
    assert not set(RESERVED_TYPES) & set(DEFINED_TYPES)
    assert message["$defs"]["reason"]["properties"]["code"]["enum"] == sorted(REASONS, key=list(
        message["$defs"]["reason"]["properties"]["code"]["enum"]).index)
    assert set(message["$defs"]["reason"]["properties"]["code"]["enum"]) == REASONS


# -------- Core fixtures --------

def test_every_core_fixture_file_has_exactly_one_case():
    files = sorted(p.relative_to(CORE).as_posix() for p in CORE.glob("*/*.json"))
    assert sorted(case["file"] for case in CORE_CASES) == files


def test_every_stateless_code_has_a_fixture():
    covered = {case["expect"] for case in CORE_CASES} - {"accepted"}
    assert covered == set(CODES) - {"C306", "C403"}  # state-dependent: covered by the scenario


@pytest.mark.parametrize("case", CORE_CASES, ids=lambda case: case["file"])
def test_core_case_is_consistent(case):
    path = CORE / case["file"]
    if case["expect"] == "accepted":
        doc = load(path)
        assert doc["schema_version"] == "0.1"
        assert signature_verifies(doc, CONTEXT["keys"]), "valid fixture must be correctly signed"
        assert CONTEXT["keys"][doc["signature"]["key"]]["participant"] == signer(doc)
        for observation in doc.get("payload", {}).get("observations", []) + doc.get("payload", {}).get("evidence", []):
            assert signature_verifies(observation, CONTEXT["keys"])
            assert CONTEXT["keys"][observation["signature"]["key"]]["participant"] == signer(observation)
        return

    code = case["expect"]
    assert path.name.startswith(code.lower() + "_")
    assert CODES[code] == case["stage"], f"{code} is a {CODES[code]} code"
    if code == "C101":
        with pytest.raises((ValueError, AssertionError)):
            data = path.read_bytes()
            assert not data.startswith(b"\xef\xbb\xbf")
            json.loads(data.decode("utf-8"), object_pairs_hook=strict, parse_float=lambda s: (_ for _ in ()).throw(
                ValueError("non-integer number")))
        return
    doc = load(path)
    if code == "C202":
        assert not signature_verifies(doc, CONTEXT["keys"])
    if code == "C203":
        assert signature_verifies(doc, CONTEXT["keys"])
        assert CONTEXT["keys"][doc["signature"]["key"]]["participant"] != signer(doc)
    if CODES[code] in ("freshness", "meaning") or code == "C204":
        assert signature_verifies(doc, CONTEXT["keys"]), "later-stage fixtures must pass the signature stage"


# -------- reference scenario --------

def test_scenario_documents_are_signed_by_their_signers():
    keys = SCENARIO["keys"]
    documents = [*SCENARIO["descriptors"].values(), *SCENARIO["standing_authorizations"].values()]
    for doc in documents:
        assert signature_verifies(doc, keys)
        assert keys[doc["signature"]["key"]]["participant"] == signer(doc)
    for robot, descriptor in SCENARIO["descriptors"].items():
        assert descriptor["robot"] == robot
        kinds = [c["kind"] for c in descriptor["capabilities"]] + descriptor["unsupported"]
        assert len(kinds) == len(set(kinds)), "a capability kind appears at most once"
    for grantee, authorization in SCENARIO["standing_authorizations"].items():
        assert authorization["grantee"] == grantee and authorization["mode"] == "simulation"
    assert SCENARIO["goal"]["success_criteria"]


def test_every_rfc_variant_has_a_case_and_every_test_is_covered():
    variants = [row[0] for row in table_column(SIM_SPEC, "## 8. Failure variants")]
    assert variants == ["V1", "V2", "V3", "V4", "V5", "V6", "V7", "V8", "V9", "V10", "V11", "V12a", "V12b", "V13"]
    ids = [case["id"] for case in ARCH_CASES]
    assert len(ids) == len(set(ids))
    for variant in variants:
        number = int(re.match(r"V(\d+)", variant).group(1))
        suffix = variant[-1] if variant[-1] in "ab" else ""
        prefix = f"v{number:02d}{suffix}_"
        assert sum(i.startswith(prefix) for i in ids) == 1, variant
    covered = {test for case in ARCH_CASES for test in case["tests"]}
    assert covered == {f"A-T{n}" for n in range(1, 16)}


def resolve(document, pointer: str):
    node = document
    for token in pointer.split("/")[1:]:
        token = token.replace("~1", "/").replace("~0", "~")
        node = node[int(token)] if isinstance(node, list) else node[token]
    return node


@pytest.mark.parametrize("case", ARCH_CASES, ids=lambda case: case["id"])
def test_arch02_case_is_consistent(case):
    participants = {p["id"] for p in SCENARIO["participants"]}
    for fault in case["faults"]:
        assert fault["kind"] in FAULT_KINDS
        if fault["kind"] == "override":
            resolve(SCENARIO, fault["path"])
        for key in ("match", "from"):
            if isinstance(fault.get(key), dict):
                assert fault[key]["type"] in DEFINED_TYPES and fault[key]["sender"] in participants
        if fault["kind"] == "inject":
            assert fault["message"]["type"] in DEFINED_TYPES

    expect = case["expect"]
    assert set(expect) <= EXPECT_FIELDS
    assert expect["planner_status"] in STATUSES
    assert set(expect.get("reasons", [])) <= REASONS
    for mtype, receiver, code in expect.get("rejections", []):
        assert mtype in DEFINED_TYPES and receiver in participants and code in CODES
    for entry in expect.get("message_types", []):
        assert entry[0] in DEFINED_TYPES and set(entry[1:]) <= participants
    for entry in expect.get("messages_present", []) + expect.get("messages_absent", []):
        assert entry[0] in DEFINED_TYPES and entry[1] in participants
    assert set(expect.get("motion", {})) <= participants
    assert set(expect.get("escalated", [])) <= participants


def test_happy_path_messages_match_the_specification_table():
    rows = table_column(SIM_SPEC, "### 6.1 Happy path")
    expected = [row[4].strip("`") for row in rows]
    happy = next(case for case in ARCH_CASES if case["id"] == "happy_path")
    assert [entry[0] for entry in happy["expect"]["message_types"]] == expected
    assert len(expected) == 20


# -------- sequencing cases (decision record 0008, proposed; A-T20) --------

SEQ_CASES = load(ARCH / "sequencing_cases.json")["cases"]
CLOCK = re.compile(r"[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{3}")


def test_sequencing_cases_match_the_specification_table():
    listed = [row[0].strip("`") for row in table_column(SIM_SPEC, "### 8.1 Sequencing cases")]
    assert [case["id"] for case in SEQ_CASES] == listed
    assert len(listed) == 8 and all(case["tests"] == ["A-T20"] for case in SEQ_CASES)
    assert SCENARIO["parameters"]["max_hold"] == {"value": "5", "unit": "s"}


@pytest.mark.parametrize("case", SEQ_CASES, ids=lambda case: case["id"])
def test_sequencing_case_is_consistent(case):
    participants = {p["id"] for p in SCENARIO["participants"]}
    for fault in case["faults"]:
        assert fault["kind"] in FAULT_KINDS
        if "match" in fault:
            assert fault["match"]["type"] in DEFINED_TYPES and fault["match"]["sender"] in participants
        if fault["kind"] == "inject":
            assert fault["message"]["type"] in DEFINED_TYPES
    expect = case["expect"]
    assert set(expect) <= EXPECT_FIELDS
    assert expect["planner_status"] in STATUSES
    for mtype, receiver, held_at, released_at, result in expect.get("held", []):
        assert mtype in DEFINED_TYPES and receiver in participants
        assert CLOCK.fullmatch(held_at) and CLOCK.fullmatch(released_at) and held_at < released_at
        assert result == "accepted" or result in CODES
    for mtype, receiver, code in expect.get("rejections", []):
        assert mtype in DEFINED_TYPES and receiver in participants and code in CODES
    for receiver, types in expect.get("processed_by", {}).items():
        assert receiver in participants and set(types) <= set(DEFINED_TYPES)
