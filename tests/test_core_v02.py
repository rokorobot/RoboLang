"""
Tests for Core 0.2 documents (spec/core-v0.2-draft.md, decision 0028): the 0.2 message schema, the
checks in robolang.core, and the conformance fixtures in tests/fixtures/core_v02/.

Every rule and code of the draft has a fixture; 0.1 documents keep their 0.1 receipts.
"""

from __future__ import annotations
import copy
import json
from pathlib import Path

import jsonschema
import pytest

from robolang import core
from robolang.core import Context, Key, check_message

ROOT = Path(__file__).parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "core_v02"
CORE01 = ROOT / "tests" / "fixtures" / "core"
CONTEXT = json.loads((CORE01 / "context.json").read_text(encoding="utf-8"))
EXPECTED = json.loads((FIXTURES / "expected.json").read_text(encoding="utf-8"))
CASES = EXPECTED["cases"]
CASES_01 = json.loads((CORE01 / "cases.json").read_text(encoding="utf-8"))["cases"]
KEYS = {ref: Key(k["participant"], k["alg"], bytes.fromhex(k["secret"])) for ref, k in CONTEXT["keys"].items()}
SPEC_SCHEMA = ROOT / "spec" / "core" / core.MESSAGE_SCHEMA_02
BOTH = frozenset({"0.1", "0.2"})
NEW_CODES = {"C102", "C103", "C104", "C402", "C405"}


def ms(quantity: dict) -> int:
    return int(quantity["value"]) * {"ms": 1, "s": 1000, "min": 60000}[quantity["unit"]]


def context(receiver: str, versions=BOTH, mode: str = "simulation") -> Context:
    return Context(
        receiver=receiver,
        now_ms=core.time_ms(CONTEXT["now"]),
        clock_tolerance_ms=ms(CONTEXT["clock_tolerance"]),
        max_message_lifetime_ms=ms(CONTEXT["max_message_lifetime"]),
        keys=KEYS,
        authorities=CONTEXT["authorities"],
        mode=mode,
        versions=frozenset(versions),
    )


def load(case) -> tuple[bytes, dict]:
    data = (FIXTURES / case["file"]).read_bytes()
    return data, json.loads(data)


def receipt(case) -> core.Receipt:
    data, document = load(case)
    versions = case.get("receiver_versions", BOTH)
    return check_message(data, context(document["recipient"], versions))


def resign(document: dict) -> bytes:
    ref = document["signature"]["key"]
    return json.dumps(core.sign_sim(document, ref, KEYS[ref])).encode()


def check(document: dict, versions=BOTH) -> core.Receipt:
    return check_message(resign(document), context(document["recipient"], versions))


def fixture(name: str) -> dict:
    return json.loads((FIXTURES / name).read_text(encoding="utf-8"))


# -------- schema --------

def test_packaged_schema_is_identical_to_the_specification():
    assert core.load_schema(core.MESSAGE_SCHEMA_02) == json.loads(SPEC_SCHEMA.read_text(encoding="utf-8"))
    packaged = ROOT / "src" / "robolang" / "core" / "schemas" / core.MESSAGE_SCHEMA_02
    assert packaged.read_bytes() == SPEC_SCHEMA.read_bytes()


def test_schema_is_valid_json_schema_2020_12():
    jsonschema.Draft202012Validator.check_schema(json.loads(SPEC_SCHEMA.read_text(encoding="utf-8")))


def test_the_0_1_schema_and_descriptor_are_untouched_and_descriptor_stays_0_1():
    assert core.load_schema(core.MESSAGE_SCHEMA)["properties"]["schema_version"] == {"const": "0.1"}
    assert core.load_schema(core.MESSAGE_SCHEMA_02)["properties"]["schema_version"] == {"const": "0.2"}
    assert core.DESCRIPTOR == ("robolang.core.capability-descriptor", "0.1")
    assert core.load_schema(core.DESCRIPTOR_SCHEMA)["$id"].endswith(":0.1")


# -------- fixtures --------

@pytest.mark.parametrize("case", CASES, ids=lambda case: case["file"])
def test_fixture_gets_its_stated_receipt(case):
    result = receipt(case)
    if case["expect"] == "accepted":
        assert result.accepted, result
    else:
        assert (result.status, result.code, result.stage) == ("rejected", case["expect"], case["stage"]), result


def schema_cases():
    for case in CASES:
        if case["file"].startswith("../"):
            continue
        yield pytest.param(case, id=case["file"])


# faults the schema cannot state: the receiver checks them (C104 duplicates)
DUPLICATE_RULES = {
    "invalid/c104_duplicate_alternative_triples.json",
    "invalid/c104_duplicate_alternative_triples_reordered.json",
    "invalid/c104_duplicate_result_pair.json",
}


@pytest.mark.parametrize("case", list(schema_cases()))
def test_package_validator_agrees_with_jsonschema_and_the_stage_says(case):
    _, document = load(case)
    if document["schema_version"] != "0.2":
        return  # judged by the 0.1 schema, covered by tests/test_core.py
    validator = jsonschema.Draft202012Validator(json.loads(SPEC_SCHEMA.read_text(encoding="utf-8")))
    valid = validator.is_valid(document)
    assert (not core.validate(document, core.MESSAGE_SCHEMA_02)) is valid, "robolang.core disagrees with jsonschema"
    meaning_or_state = case["expect"] in ("accepted", "C402", "C405") or case["file"] in DUPLICATE_RULES
    if case["file"].endswith("c402_succeeded_verification_null.json"):
        meaning_or_state = False  # rank 6 of Core 8.1: also fails the schema
    if case["expect"] != "C102":
        assert valid is meaning_or_state


def test_every_fixture_file_is_listed_and_every_listed_file_exists():
    listed = {case["file"] for case in CASES if not case["file"].startswith("../")}
    on_disk = {p.relative_to(FIXTURES).as_posix() for p in FIXTURES.glob("*/*.json")}
    assert listed == on_disk
    for case in CASES:
        assert (FIXTURES / case["file"]).is_file()


def test_there_is_a_fixture_for_every_new_rule_and_code():
    codes = {case["expect"] for case in CASES}
    assert NEW_CODES <= codes
    names = {Path(case["file"]).name for case in CASES}
    for required in (
        "goal_one_alternative.json", "goal_two_alternatives.json", "goal_sixteen_alternatives.json",
        "outcome_met_alternative_1.json", "outcome_failed_declared.json", "outcome_failed_declared_200_chars.json",
        "outcome_failed_other_alternative_holds.json", "c102_v02_message_to_v01_only_receiver.json",
        "c103_v02_goal_with_desired_state.json", "c103_v01_goal_with_alternatives.json",
        "c104_alternatives_empty.json", "c104_alternatives_too_many.json", "c104_duplicate_alternative_triples.json",
        "c104_duplicate_result_pair.json", "c104_met_alternative_negative.json", "c104_declared_without_text.json",
        "c104_declared_text_too_long.json", "c104_declared_text_with_newline.json", "c104_text_on_other_code.json",
        "c402_succeeded_met_alternative_null.json", "c402_succeeded_pursued_alternative_unmet_other_holds.json",
        "c405_declared_in_succeeded_outcome.json", "c405_declared_in_reject.json",
    ):
        assert required in names, required


def test_fixtures_are_signed_so_the_receipt_is_about_the_rule_under_test():
    # Every 0.2 fixture passes stages 1-3 apart from the form or meaning fault the case names.
    for case in CASES:
        if case["file"].startswith("../") or case["expect"] in ("C102", "C103", "C104", "C402", "C405", "accepted"):
            continue
        pytest.fail(f"unexpected code in 0.2 fixtures: {case}")
    for case in CASES:
        result = receipt(case)
        assert result.stage in (None, "form", "meaning"), (case["file"], result)


# -------- 0.1 compatibility (C7, C8) --------

@pytest.mark.parametrize("case", CASES_01, ids=lambda case: case["file"])
def test_every_0_1_fixture_gets_the_same_receipt_from_a_reader_of_both_versions(case):
    data = (CORE01 / case["file"]).read_bytes()
    try:
        document = json.loads(data)
    except ValueError:
        document = {}
    if document.get("schema") == "robolang.core.capability-descriptor":
        pytest.skip("descriptors do not change")
    receiver = case.get("receiver") or document.get("recipient") or "rl:robot:arm-02"
    mode = case.get("receiver_mode", "simulation")
    only_01 = check_message(data, context(receiver, {"0.1"}, mode))
    both = check_message(data, context(receiver, BOTH, mode))
    expected = (case["expect"], case.get("stage"))
    assert (only_01.code or "accepted", only_01.stage) == expected
    # the 0.2 version fixtures are 0.1-only faults: a reader of both accepts the version and goes on
    if case["file"].endswith(("c102_unsupported_version.json",)):
        assert both.code != "C102" or both.stage != "form"
    elif "c102_before_" not in case["file"]:
        assert both == only_01


def test_the_default_context_is_configured_for_0_1_only():
    assert Context("r", 0, 0, 0, {}).versions == frozenset({"0.1"})
    data = (FIXTURES / "valid/goal_two_alternatives.json").read_bytes()
    assert check_message(data, Context("rl:robot:arm-02", 0, 0, 0, {})).code == "C102"


def test_a_0_1_document_is_not_accepted_by_a_0_2_only_receiver():
    data = (CORE01 / "valid" / "propose_goal.json").read_bytes()
    assert check_message(data, context("rl:robot:arm-02", {"0.2"})).code == "C102"


def test_a_0_1_document_is_judged_by_0_1_rules_only():
    goal = fixture("../core/valid/propose_goal.json")
    goal["payload"]["goal"]["alternatives"] = []
    assert check(goal).code == "C103"
    outcome = fixture("../core/valid/outcome_succeeded.json")
    outcome["payload"]["outcome"]["verification"]["met_alternative"] = 0
    assert check(outcome).code == "C103"
    # 0.1 succeeded does not need met_alternative
    assert check(fixture("../core/valid/outcome_succeeded.json")).accepted


def test_an_unknown_version_is_c102_even_for_a_reader_of_both():
    document = fixture("valid/goal_two_alternatives.json")
    document["schema_version"] = "0.3"
    assert check(document).code == "C102"


# -------- the rules, one by one --------

def with_alternatives(count: int) -> dict:
    document = fixture("valid/goal_two_alternatives.json")
    alternative = document["payload"]["goal"]["alternatives"][0]
    alternatives = []
    for i in range(count):
        item = copy.deepcopy(alternative)
        item["desired_state"][0]["value"] = f"rl:place:p{i}"
        alternatives.append(item)
    document["payload"]["goal"]["alternatives"] = alternatives
    return document


@pytest.mark.parametrize(("count", "accepted"), [(0, False), (1, True), (16, True), (17, False)])
def test_alternatives_are_one_to_sixteen(count, accepted):
    result = check(with_alternatives(count))
    assert result.accepted is accepted
    if not accepted:
        assert (result.code, result.stage) == ("C104", "form")


def test_alternatives_with_the_same_triples_are_c104_in_any_order_but_overlap_is_fine():
    document = with_alternatives(2)
    document["payload"]["goal"]["alternatives"][1]["desired_state"][0]["value"] = "rl:place:p0"
    result = check(document)
    assert (result.code, result.pointer) == ("C104", "/payload/goal/alternatives/1")
    document = with_alternatives(2)
    document["payload"]["goal"]["alternatives"][1]["desired_state"].append(
        document["payload"]["goal"]["alternatives"][0]["desired_state"][0]
    )
    assert check(document).accepted


def test_the_order_of_alternatives_is_not_a_priority_and_indices_are_positional():
    document = with_alternatives(2)
    document["payload"]["goal"]["alternatives"].reverse()
    assert check(document).accepted
    outcome = fixture("valid/outcome_met_alternative_1.json")
    verification = outcome["payload"]["outcome"]["verification"]
    assert (verification["met_alternative"], verification["criteria_results"][0]["alternative"]) == (1, 1)


def outcome_with(status="succeeded", results=None, met=0, reasons=None, verification=True) -> dict:
    document = fixture("valid/outcome_met_alternative_0.json")
    outcome = document["payload"]["outcome"]
    outcome["status"] = status
    outcome["reasons"] = reasons or []
    if not verification:
        outcome["verification"] = None
    else:
        outcome["verification"]["criteria_results"] = results or outcome["verification"]["criteria_results"]
        outcome["verification"]["met_alternative"] = met
    return document


def result(alternative, criterion=0, met=True):
    return {"alternative": alternative, "criterion": criterion, "met": met, "evidence": []}


def test_succeeded_needs_the_pursued_alternative_met():
    assert check(outcome_with()).accepted
    assert check(outcome_with(results=[result(1)], met=1)).accepted
    assert check(outcome_with(results=[result(0, 0), result(0, 1)], met=0)).accepted
    for results, met in (
        ([result(0)], None),
        ([result(0, met=False), result(1)], 0),
        ([result(0, 0), result(0, 1, met=False)], 0),
        ([result(1)], 0),
    ):
        got = check(outcome_with(results=results, met=met))
        assert (got.code, got.stage) == ("C402", "meaning"), (results, met)
    assert check(outcome_with(verification=False)).code == "C402"


def test_two_alternatives_holding_is_not_an_error_the_claim_names_one():
    both = [result(0), result(1)]
    assert check(outcome_with(results=both, met=0)).accepted
    assert check(outcome_with(results=both, met=1)).accepted


def test_met_alternative_beyond_the_goal_is_not_checked_by_a_stateless_receiver():
    assert check(outcome_with(results=[result(7)], met=7)).accepted


def test_other_statuses_may_carry_any_met_alternative():
    for status in ("failed", "cancelled", "timed_out", "rejected", "unknown"):
        assert check(outcome_with(status=status, results=[result(0, met=False)], met=None)).accepted
        assert check(outcome_with(status=status, results=[result(0, met=False)], met=0)).accepted


def declared(text="box1 missing") -> list:
    return [{"code": "declared", "text": text}]


def test_declared_is_allowed_only_in_a_failed_outcome():
    assert check(outcome_with(status="failed", verification=False, reasons=declared())).accepted
    for status in ("succeeded", "cancelled", "timed_out", "rejected", "unknown"):
        got = check(outcome_with(status=status, reasons=declared()))
        assert (got.code, got.stage) == ("C405", "meaning"), status


def test_declared_text_limits():
    def failed(text):
        return check(outcome_with(status="failed", verification=False, reasons=declared(text)))

    assert failed("x").accepted
    assert failed("x" * 200).accepted
    assert failed("日本" * 100).accepted
    assert failed("tab\tinside").code == "C104"
    for text in ("", "x" * 201, "a\x00b", "a\x1fb", "a\x7fb", "a\nb", "a\rb"):
        assert failed(text).code == "C104", repr(text)
    # U+0080 to U+009F are not in the draft's list of control characters
    assert failed("a\x85b").accepted


def test_text_is_only_for_declared_and_declared_always_has_text():
    got = check(outcome_with(status="failed", verification=False, reasons=[{"code": "evidence_missing", "text": "x"}]))
    assert (got.code, got.pointer) == ("C104", "/payload/outcome/reasons/0/text")
    assert check(outcome_with(status="failed", verification=False, reasons=[{"code": "declared"}])).code == "C104"


def test_c405_is_reported_before_c402_when_both_apply():
    got = check(outcome_with(results=[result(0)], met=None, reasons=declared()))
    assert got.code == "C405"


def test_c405_pointer_names_the_reason():
    got = check(outcome_with(reasons=declared()))
    assert got.pointer == "/payload/outcome/reasons/0"


def test_meaning_is_stage_5_so_check_meaning_applies_c405_and_c402():
    document = outcome_with(reasons=declared())
    assert core.check_meaning(document, context("rl:agent:planner-1")).code == "C405"
    assert core.check_meaning(outcome_with(met=None), context("rl:agent:planner-1")).code == "C402"
    assert core.check_meaning(outcome_with(), context("rl:agent:planner-1")).accepted


def test_physical_mode_is_still_rejected():
    document = fixture("valid/goal_two_alternatives.json")
    assert core.check_meaning({**document, "payload": {**document["payload"], "mode": "physical"}},
                              context("rl:robot:arm-02")).code == "C404"
