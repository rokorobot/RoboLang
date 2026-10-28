"""
INTEROP-2g: the gate 2 specification bundle (spec/interop-gate-2-v0.1.md 4.1), built by
tools/interop_bundle.py into an isolated directory and checked from that directory alone.

A3: exactly the allowlist, and nothing private (source, golden records, scenario, fault scripts,
expected run results). A4: the 47 fixtures give their stated receipts from the bundle's own files,
and the schemas resolve. A5: the example view is exactly what 6.4 allows. A6: the bundle is
reproducible and its manifest detects any change. Links: every relative link either resolves in
the bundle or is one of the classified informative citations.
"""

from __future__ import annotations
import importlib.util
import json
import posixpath
import re
from pathlib import Path

import jsonschema
import pytest

from robolang import core
from robolang.arch02 import load_scenario
from robolang.core import Context, Key, check_descriptor, check_message
from robolang.interop import AMR, build_view

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("interop_bundle", ROOT / "tools" / "interop_bundle.py")
bundle_tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bundle_tool)

# Links from bundled documents to files outside the bundle: informative citations (A2). The rules
# they record are stated in the bundle (the audit report traces them); a new link must be classified here.
INFORMATIVE = {
    "docs/DEVELOPMENT_PLAN.md", "docs/evidence/rl-arch-02-exit-gate.md",
    "docs/interop/spec-implementation-discrepancies.md",
    "spec/decisions/0002-ai-native-architecture.md", "spec/decisions/0009-gate-2-independent-interoperability.md",
    "spec/decisions/0010-binding-errata-lease-expired-commitment-state.md",
    "spec/decisions/0011-interop-2a-bundle-errata.md",
    "spec/decisions/0012-interop-2c-remaining-normative-decisions.md",
    "spec/decisions/0013-h3-receiver-scoped-verification-keys.md",
    "spec/ir-schema-v1.0.md", "spec/robolang-v1.1.md", "spec/simulation-v1.0.md",
    "tests/fixtures/arch02/README.md", "tests/fixtures/arch02",
}


@pytest.fixture(scope="module")
def bundle(tmp_path_factory) -> Path:
    out = tmp_path_factory.mktemp("bundle") / "gate2"
    bundle_tool.build(out)
    return out


def files(directory: Path) -> set[str]:
    return {p.relative_to(directory).as_posix() for p in directory.rglob("*") if p.is_file()}


# -------- A3: allowlist and information barrier --------

def test_bundle_holds_exactly_the_allowlist(bundle):
    kit = {p.relative_to(ROOT).as_posix() for p in (ROOT / "tests" / "fixtures" / "core").rglob("*") if p.is_file()}
    schemas = {p.relative_to(ROOT).as_posix() for p in (ROOT / "spec" / "core").glob("*.schema.json")}
    expected = set(bundle_tool.DOCUMENTS) | schemas | kit | {bundle_tool.VIEW, bundle_tool.README, bundle_tool.MANIFEST}
    assert files(bundle) == expected
    assert len([f for f in kit if "/valid/" in f or "/invalid/" in f]) == 47
    assert len(schemas) == 2


def test_bundle_holds_no_private_or_implementation_material(bundle):
    names = files(bundle)
    assert not [n for n in names if n.startswith(("src/", "tools/")) or n.endswith((".py", ".pyc"))]
    assert not [n for n in names if "arch02" in n or "golden" in n or "snapshot" in n]
    text = "\n".join((bundle / n).read_text(encoding="utf-8", errors="replace") for n in names)
    golden = json.loads((ROOT / "tests" / "fixtures" / "arch02" / "golden_records.json").read_text(encoding="utf-8"))
    for record in golden["records"].values():  # no expected run result
        assert record["run_id"] not in text and record["record_sha256"] not in text
    scenario = json.loads((ROOT / "tests" / "fixtures" / "arch02" / "scenario.json").read_text(encoding="utf-8"))
    private = [json.dumps(scenario["authority_table"]), json.dumps(scenario["durations"]),
               json.dumps(scenario["policies"]["rl:robot:arm-02"]), '"faults"', '"planner_status"']
    private += [s["id"] for s in scenario["standing_authorizations"].values() if s["grantee"] != AMR]
    assert [item for item in private if item in text] == []
    assert not re.search(r"^\s*(def|class|import) \w", text, re.M)  # no code


# -------- A5: the example view --------

def test_example_view_is_exactly_what_6_4_allows(bundle):
    view = json.loads((bundle / bundle_tool.VIEW).read_text(encoding="utf-8"))
    scenario = load_scenario((ROOT / "tests" / "fixtures" / "arch02" / "scenario.json").read_bytes())
    assert view == build_view(scenario, AMR)
    assert set(view) == {"participant", "key", "descriptor", "standing_authorization", "policy", "sensors", "location",
                         "parameters", "conversation_id", "participants", "places", "arbiters", "authority",
                         "verification_keys"}
    assert sorted(view["verification_keys"]) == ["rl:human:shift-lead-3#k1", "rl:robot:arm-02#k1"]  # H3: no more
    assert [s["sensor"] for s in view["sensors"]] == ["deck_camera"] and "lag" not in json.dumps(view["sensors"])
    assert "goal" not in view and scenario["goal"]["id"] not in json.dumps(view)


# -------- A4: the conformance kit, from the bundle alone --------

def ms(quantity: dict) -> int:
    return int(quantity["value"]) * {"ms": 1, "s": 1000, "min": 60000}[quantity["unit"]]


def test_all_47_fixtures_give_their_stated_receipt_from_the_bundle(bundle):
    kit = bundle / "tests" / "fixtures" / "core"
    context = json.loads((kit / "context.json").read_text(encoding="utf-8"))
    cases = json.loads((kit / "cases.json").read_text(encoding="utf-8"))["cases"]
    keys = {ref: Key(k["participant"], k["alg"], bytes.fromhex(k["secret"])) for ref, k in context["keys"].items()}
    codes = {row.split("|")[1].strip().strip("`"): row.split("|")[2].strip()
             for row in (bundle / "spec" / "core-v0.1.md").read_text(encoding="utf-8").split("### 8.2")[1]
             .split("###")[0].splitlines() if re.match(r"\| `C\d{3}`", row)}
    assert len(cases) == 47 and sorted(c["file"] for c in cases) == sorted(
        p.relative_to(kit).as_posix() for p in kit.glob("*/*.json"))
    for case in cases:
        data = (kit / case["file"]).read_bytes()
        try:
            document = json.loads(data)
        except ValueError:
            document = {}
        receiver = case.get("receiver") or document.get("robot") or document.get("recipient") or "rl:robot:arm-02"
        ctx = Context(receiver, core.time_ms(context["now"]), ms(context["clock_tolerance"]),
                      ms(context["max_message_lifetime"]), keys, context["authorities"],
                      case.get("receiver_mode", "simulation"))
        receipt = (check_descriptor if document.get("schema") == "robolang.core.capability-descriptor"
                   else check_message)(data, ctx)
        if case["expect"] == "accepted":
            assert receipt.accepted, case
        else:
            assert (receipt.code, receipt.stage) == (case["expect"], case["stage"]), case
            assert codes[case["expect"]] == case["stage"]  # defined in the bundle's Core 8.2
            assert all(c in codes for c in case.get("competing", []))


def test_schemas_resolve_within_the_bundle(bundle):
    for path in (bundle / "spec" / "core").glob("*.schema.json"):
        schema = json.loads(path.read_text(encoding="utf-8"))
        jsonschema.Draft202012Validator.check_schema(schema)
        refs = re.findall(r'"\$ref":\s*"([^"]*)"', path.read_text(encoding="utf-8"))
        assert refs and all(ref.startswith("#/") for ref in refs), path.name  # no external schema
    validator = jsonschema.Draft202012Validator(
        json.loads((bundle / "spec" / "core" / "message-0.1.schema.json").read_text(encoding="utf-8")))
    for path in (bundle / "tests" / "fixtures" / "core" / "valid").glob("*.json"):
        document = json.loads(path.read_text(encoding="utf-8"))
        if document["schema"] == "robolang.core.message":
            assert validator.is_valid(document), path.name


# -------- links (A2, C) --------

def test_every_link_resolves_in_the_bundle_or_is_a_classified_citation(bundle):
    unclassified = set()
    for name in files(bundle):
        if not name.endswith(".md"):
            continue
        for target in re.findall(r"\]\(([^)#\s]+)(?:#[^)]*)?\)", (bundle / name).read_text(encoding="utf-8")):
            if target.startswith(("http://", "https://")):
                continue
            path = posixpath.normpath(posixpath.join(posixpath.dirname(name), target))
            if not (bundle / path).exists() and path not in INFORMATIVE:
                unclassified.add(f"{name} -> {path}")
    assert unclassified == set()


# -------- A6: reproducibility and integrity --------

def test_bundle_is_reproducible_and_verifiable(bundle, tmp_path):
    again = tmp_path / "again"
    assert bundle_tool.build(again) == bundle_tool.verify(bundle)
    assert {n: (again / n).read_bytes() for n in files(again)} == {n: (bundle / n).read_bytes() for n in files(bundle)}
    lines = (bundle / bundle_tool.MANIFEST).read_text(encoding="ascii").splitlines()
    assert len(lines) == len(files(bundle)) - 1 and all(re.fullmatch(r"[0-9a-f]{64}  \S+", line) for line in lines)


@pytest.mark.parametrize("change", ["edit", "extra", "missing"])
def test_verify_detects_any_change(bundle, tmp_path, change):
    copy = tmp_path / "copy"
    bundle_tool.build(copy)
    target = copy / "spec" / "core-v0.1.md"
    if change == "edit":
        target.write_bytes(target.read_bytes() + b" ")
    elif change == "extra":
        (copy / "notes.txt").write_text("x", encoding="utf-8")
    else:
        target.unlink()
    with pytest.raises(SystemExit):
        bundle_tool.verify(copy)


# -------- A6: commit-pinned builds fail closed (review finding F-1) --------

def test_pinned_build_rejects_a_checkout_of_another_commit(tmp_path, monkeypatch):
    monkeypatch.setattr(bundle_tool.Source, "_head", classmethod(lambda cls: "0" * 40))
    with pytest.raises(SystemExit, match="check out the pinned commit"):
        bundle_tool.build(tmp_path / "b", rev="HEAD")
    assert not (tmp_path / "b").exists()  # nothing written


def test_pinned_build_rejects_local_changes_to_the_building_code(tmp_path, monkeypatch):
    monkeypatch.setattr(bundle_tool.Source, "_changed_code", classmethod(lambda cls: ["src/robolang/interop/view.py"]))
    with pytest.raises(SystemExit, match="local changes to the building code"):
        bundle_tool.build(tmp_path / "b", rev="HEAD")


def test_pinned_build_from_a_clean_checkout_names_its_commit(tmp_path):
    if bundle_tool.Source._changed_code():
        pytest.skip("the building code has local changes; a pinned build would rightly refuse")
    head = bundle_tool.Source._head()
    out = tmp_path / "pinned"
    bundle_id = bundle_tool.build(out, rev="HEAD")
    assert bundle_tool.verify(out) == bundle_id
    assert f"commit `{head}`" in (out / bundle_tool.README).read_text(encoding="utf-8")
    for name in files(out) - {bundle_tool.README, bundle_tool.MANIFEST, bundle_tool.VIEW}:
        assert (out / name).read_bytes() == bundle_tool.Source._git("show", f"{head}:{name}"), name
