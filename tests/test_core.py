"""
Tests for robolang.core, Core 0.1 document checks (ARCH-1; spec/core-v0.1.md).

A-T16: every Core fixture gets its stated receipt. A-T17: both schema files are valid JSON
Schema 2020-12 under the `jsonschema` library, the fixtures validate (or fail) as their stage
says, and the package's own validator agrees with `jsonschema` on every document.
Also: RFC 8785 canonical form against vectors from the RFC itself, the strict reader, and
isolation.
"""

from __future__ import annotations
import json
import subprocess
import sys
from pathlib import Path

import jsonschema
import pytest

from robolang import core
from robolang.core import Context, Key, canonical_bytes, check_descriptor, check_message, read_document

ROOT = Path(__file__).parent.parent
CORE_FIXTURES = ROOT / "tests" / "fixtures" / "core"
SCENARIO = json.loads((ROOT / "tests" / "fixtures" / "arch02" / "scenario.json").read_text(encoding="utf-8"))
CONTEXT = json.loads((CORE_FIXTURES / "context.json").read_text(encoding="utf-8"))
CASES = json.loads((CORE_FIXTURES / "cases.json").read_text(encoding="utf-8"))["cases"]
KEYS = {ref: Key(k["participant"], k["alg"], bytes.fromhex(k["secret"])) for ref, k in CONTEXT["keys"].items()}
SPEC_SCHEMAS = {name: ROOT / "spec" / "core" / name for name in (core.MESSAGE_SCHEMA, core.DESCRIPTOR_SCHEMA)}


def ms(quantity: dict) -> int:
    return int(quantity["value"]) * {"ms": 1, "s": 1000, "min": 60000}[quantity["unit"]]


def context(receiver: str, mode: str = "simulation") -> Context:
    return Context(
        receiver=receiver,
        now_ms=core.time_ms(CONTEXT["now"]),
        clock_tolerance_ms=ms(CONTEXT["clock_tolerance"]),
        max_message_lifetime_ms=ms(CONTEXT["max_message_lifetime"]),
        keys=KEYS,
        authorities=CONTEXT["authorities"],
        mode=mode,
    )


def receipt_for(case) -> core.Receipt:
    data = (CORE_FIXTURES / case["file"]).read_bytes()
    try:
        document = json.loads(data)
    except ValueError:
        document = {}
    if document.get("schema") == "robolang.core.capability-descriptor":
        return check_descriptor(data, context(document["robot"]))
    receiver = case.get("receiver") or document.get("recipient") or "rl:robot:arm-02"
    return check_message(data, context(receiver, case.get("receiver_mode", "simulation")))


# -------- A-T16: Core fixtures --------

@pytest.mark.parametrize("case", CASES, ids=lambda case: case["file"])
def test_core_fixture_gets_its_stated_receipt(case):
    receipt = receipt_for(case)
    if case["expect"] == "accepted":
        assert receipt.accepted, receipt
    else:
        assert (receipt.status, receipt.code, receipt.stage) == ("rejected", case["expect"], case["stage"]), receipt


def test_a_receipt_is_deterministic():
    for case in CASES:
        assert receipt_for(case) == receipt_for(case)


# -------- A-T17: schemas --------

def test_packaged_schemas_are_identical_to_the_specification():
    for name, path in SPEC_SCHEMAS.items():
        assert core.load_schema(name) == json.loads(path.read_text(encoding="utf-8"))


@pytest.mark.parametrize("name", list(SPEC_SCHEMAS))
def test_schema_is_valid_json_schema_2020_12(name):
    jsonschema.Draft202012Validator.check_schema(json.loads(SPEC_SCHEMAS[name].read_text(encoding="utf-8")))


def documents():
    """Every fixture document that parses as JSON, with the schema it is checked against."""
    for case in CASES:
        if case["expect"] == "C101":
            continue  # rejected by the strict reader before any schema; jsonschema accepts 1.0 as an integer
        document = json.loads((CORE_FIXTURES / case["file"]).read_bytes())
        schema = core.DESCRIPTOR_SCHEMA if document.get("schema") == "robolang.core.capability-descriptor" \
            else core.MESSAGE_SCHEMA
        yield pytest.param(document, schema, case, id=case["file"])


@pytest.mark.parametrize(("document", "schema", "case"), list(documents()))
def test_fixture_passes_the_schema_exactly_when_its_stage_says(document, schema, case):
    validator = jsonschema.Draft202012Validator(json.loads(SPEC_SCHEMAS[schema].read_text(encoding="utf-8")))
    valid = validator.is_valid(document)
    fails_on_form = case["expect"] in ("C102", "C103", "C104", "C105", "C106", "C107", "C201") or (
        case["file"] == "invalid/c402_succeeded_without_verification.json"
    )
    assert valid is not fails_on_form
    assert (not core.validate(document, schema)) is valid, "robolang.core disagrees with jsonschema"


def test_scenario_documents_validate_under_both_validators():
    message = json.loads(SPEC_SCHEMAS[core.MESSAGE_SCHEMA].read_text(encoding="utf-8"))
    descriptor = jsonschema.Draft202012Validator(
        json.loads(SPEC_SCHEMAS[core.DESCRIPTOR_SCHEMA].read_text(encoding="utf-8"))
    )
    for document in SCENARIO["descriptors"].values():
        assert descriptor.is_valid(document) and not core.validate(document, core.DESCRIPTOR_SCHEMA)
    authorization = jsonschema.Draft202012Validator(
        {"$ref": "#/$defs/standing_authorization", "$defs": message["$defs"]}
    )
    for document in SCENARIO["standing_authorizations"].values():
        assert authorization.is_valid(document)


def test_unsupported_schema_keyword_is_refused():
    with pytest.raises(ValueError):
        core.schema._check_keywords({"type": "string", "format": "date-time"})


# -------- RFC 8785 --------

def test_rfc_8785_section_3_2_2_example():
    value = json.loads(
        '{"numbers": [333333333.33333329, 1E30, 4.50, 2e-3, 0.000000000000000000000000001],'
        ' "string": "\\u20ac$\\u000F\\u000aA\'\\u0042\\u0022\\u005c\\\\\\"\\/",'
        ' "literals": [null, true, false]}'
    )
    expected = (
        '{"literals":[null,true,false],"numbers":[333333333.3333333,1e+30,4.5,0.002,1e-27],'
        '"string":"€$\\u000f\\nA\'B\\"\\\\\\\\\\"/"}'
    )
    assert canonical_bytes(value) == expected.encode("utf-8")


def test_rfc_8785_sorts_keys_by_utf16_code_units():
    value = {
        "\u20ac": "Euro Sign",
        "\r": "Carriage Return",
        "\ufb33": "Hebrew Letter Dalet With Dagesh",
        "1": "One",
        "\U0001f600": "Emoji: Grinning Face",
        "\u0080": "Control",
        "\u00f6": "Latin Small Letter O With Diaeresis",
    }
    keys = list(json.loads(canonical_bytes(value)))
    # U+1F600 is encoded as the surrogates D83D DE00, which sort before U+FB33.
    assert keys == ["\r", "1", "\u0080", "\u00f6", "\u20ac", "\U0001f600", "\ufb33"]


@pytest.mark.parametrize(
    ("number", "text"),
    [
        (0.0, "0"), (-0.0, "0"), (1.0, "1"), (-1.5, "-1.5"), (1e21, "1e+21"), (1e20, "100000000000000000000"),
        (1e-6, "0.000001"), (1e-7, "1e-7"), (5e-324, "5e-324"),
        (1.7976931348623157e308, "1.7976931348623157e+308"), (9007199254740992.0, "9007199254740992"),
        (295147905179352830000.0, "295147905179352830000"), (0.1 + 0.2, "0.30000000000000004"),
    ],
)
def test_rfc_8785_number_serialization(number, text):
    assert canonical_bytes(number) == text.encode()


def test_integers_and_non_ascii_text():
    expected = '{"a":"café\\u001f","b":[1,-2,100000000000000000000]}'
    assert canonical_bytes({"b": [1, -2, 10**20], "a": "café\u001f"}) == expected.encode()


def test_nan_has_no_canonical_form():
    with pytest.raises(ValueError):
        canonical_bytes(float("nan"))


def test_signature_round_trip_and_tampering():
    key = KEYS["rl:robot:arm-02#k1"]
    signed = core.sign_sim({"a": 1, "b": "x"}, "rl:robot:arm-02#k1", key)
    assert core.verify_sim(signed, key)
    assert not core.verify_sim({**signed, "b": "y"}, key)


# -------- strict reader --------

@pytest.mark.parametrize(
    "data",
    [b'{"a": 1, "a": 2}', b'{"a": 1.0}', b'{"a": 1e3}', b'{"a": NaN}', b"\xef\xbb\xbf{}", b'{"a": "\xff"}', b"{"],
)
def test_reader_rejects(data):
    with pytest.raises(core.ReadError):
        read_document(data)


def test_reader_accepts_integers_and_unicode():
    assert read_document('{"a": [1, -2], "b": "é"}'.encode()) == {"a": [1, -2], "b": "é"}


# -------- times, descriptors and algorithms --------

def test_impossible_dates_are_not_times():
    assert core.time_ms("2026-10-19T10:14:00.5Z") == core.time_ms("2026-10-19T10:14:00.500Z")
    for text in ("2026-02-30T00:00:00Z", "2026-10-19T24:00:00Z", "2026-10-19T10:14:00+01:00"):
        with pytest.raises(ValueError):
            core.time_ms(text)


def test_descriptor_currency_applies_the_tolerance_in_the_safe_direction():
    descriptor = SCENARIO["descriptors"]["rl:robot:arm-02"]
    end = core.time_ms(descriptor["valid_until"])
    assert core.descriptor_is_current(descriptor, end - 1000, 500)
    assert not core.descriptor_is_current(descriptor, end - 500, 500)


def test_ed25519_is_not_accepted_by_this_implementation():
    document = json.loads((CORE_FIXTURES / "valid" / "assessment.json").read_bytes())
    document["signature"]["alg"] = "ed25519"
    receipt = check_message(json.dumps(document).encode(), context(document["recipient"]))
    assert (receipt.code, receipt.stage) == ("C204", "signature")


def test_a_lease_must_end_after_it_starts():
    document = json.loads((CORE_FIXTURES / "valid" / "commit.json").read_bytes())
    lease = document["payload"]["commitment"]["lease"]
    lease["expires_at"] = lease["granted_at"]
    signed = core.sign_sim(document, document["signature"]["key"], KEYS[document["signature"]["key"]])
    receipt = check_message(json.dumps(signed).encode(), context(document["recipient"]))
    assert (receipt.code, receipt.pointer) == ("C104", "/payload/commitment/lease/expires_at")


# -------- isolation (part of A-T18) --------

FORBIDDEN_IMPORTS = ("socket", "http", "urllib", "rclpy", "robolang_adapter", "robolang_runtime")
# Modules that can open connections or drive a robot. `urllib.parse` is not among them: it only
# parses strings, and the standard library's own `pathlib` imports it on Python 3.11.
FORBIDDEN_LOADED = ("socket", "ssl", "http", "urllib.request", "rclpy", "robolang_adapter", "robolang_runtime")


def test_importing_robolang_core_loads_no_network_or_adapter_module():
    probe = (
        "import sys, robolang.core; "
        f"print([m for m in sys.modules if any(m == f or m.startswith(f + '.') for f in {FORBIDDEN_LOADED!r})])"
    )
    result = subprocess.run([sys.executable, "-c", probe], capture_output=True, text=True, check=True,
                            env={**__import__("os").environ, "PYTHONPATH": str(ROOT / "src")})
    assert result.stdout.strip() == "[]"


def test_core_modules_import_nothing_from_the_language_or_simulation():
    for path in (ROOT / "src" / "robolang" / "core").glob("*.py"):
        text = path.read_text(encoding="utf-8")
        for module in ("parser", "lexer", "ir", "validator", "nodes", "sim", "cli", *FORBIDDEN_IMPORTS):
            assert f"from ..{module}" not in text and f"from robolang.{module}" not in text, (path.name, module)
            assert f"import {module}" not in text, (path.name, module)
