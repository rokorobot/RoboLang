"""
Consistency checks for the gate 2 specification (spec/interop-gate-2-v0.1.md, proposed).

Nothing here implements gate 2. These checks keep the specification consistent with the
repository: every bundle file it names exists, and its test groups G-T1 to G-T4 cover every
reference-simulation case exactly once.
"""

from __future__ import annotations
import json
import re
from pathlib import Path

ROOT = Path(__file__).parent.parent
SPEC = (ROOT / "spec" / "interop-gate-2-v0.1.md").read_text(encoding="utf-8")
FIXTURES = ROOT / "tests" / "fixtures" / "arch02"


def section(heading: str) -> str:
    start = SPEC.index(heading)
    following = SPEC.find("\n## ", start + len(heading))
    return SPEC[start:following if following != -1 else None]


def test_every_bundle_file_exists():
    bundle = section("### 4.1 Contents")
    paths = re.findall(r"`((?:spec|tests)/[^`]+)`", bundle)
    assert paths
    for path in paths:
        if path.endswith("…"):  # decision records are named by their number
            assert list(ROOT.glob(path.replace("…", "*"))), path
        elif "*" in path:
            assert list(ROOT.glob(path)), path
        else:
            assert (ROOT / path).exists(), path
    for name in re.findall(r"`(\w+\.json)`", bundle):
        assert (FIXTURES / name).exists(), name


def test_gate_2_groups_cover_every_case_exactly_once():
    cases = [
        case["id"]
        for name in ("cases.json", "sequencing_cases.json")
        for case in json.loads((FIXTURES / name).read_text(encoding="utf-8"))["cases"]
    ]
    rows = {row.split("|")[1].strip(): row.split("|")[3] for row in section("## 8. Interoperability tests").splitlines()
            if re.match(r"\| G-T[1-4] \|", row)}
    named = [c for group in ("G-T1", "G-T2", "G-T4") for c in re.findall(r"`([a-z0-9_]+)`", rows[group])]
    assert "All 8 `seq_*` cases" in rows["G-T3"]
    named += [c for c in cases if c.startswith("seq_")]
    assert sorted(named) == sorted(cases)
    assert len(named) == len(set(named)) == 24
