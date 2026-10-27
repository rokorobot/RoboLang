"""
Consistency checks for the LANG-1 fixtures (tests/fixtures/v1.2).

No v1.2 front end exists yet (LANG-1b). These tests execute nothing new: they check that
the fixtures agree with spec/robolang-v1.2.md, that today's 1.1 front end rejects every
1.2 file at its version marker (E213), and that the 1.1 compat files give their stated
result, which LANG-1b must not change.
"""

from __future__ import annotations
import json
import re
from pathlib import Path

import pytest

from robolang import RoboLangError, check_file

ROOT = Path(__file__).parent.parent
FIXTURES = ROOT / "tests" / "fixtures" / "v1.2"
SPEC = (ROOT / "spec" / "robolang-v1.2.md").read_text(encoding="utf-8")
SPEC_V1_1 = (ROOT / "spec" / "robolang-v1.1.md").read_text(encoding="utf-8")
SIM_SPEC = (ROOT / "spec" / "simulation-v1.0.md").read_text(encoding="utf-8")

HEADER = re.compile(
    r"\A// RoboLang (?P<version>v1\.[12]) fixture: (?P<set>valid|invalid|compat)\n"
    r"// expect: (?:(?P<ok>ok)|error (?P<code>E\d{3}) \((?P<stage>lexical|syntax|static)\) line (?P<line>\d+))\n"
    r"// why: .+\n"
)
STAGE_DIGIT = {"lexical": "1", "syntax": "2", "static": "3"}


def header(path: Path) -> dict:
    match = HEADER.match(path.read_text(encoding="utf-8"))
    assert match, f"{path.name}: missing or malformed header"
    return match.groupdict()


def files(folder: str) -> list:
    return [pytest.param(p, id=p.stem) for p in sorted((FIXTURES / folder).glob("*.rob"))]


def defined_codes(text: str) -> set[str]:
    return set(re.findall(r"^\| `([EWS]\d{3})` \|", text, re.MULTILINE))


def test_fixture_counts_match_the_specification():
    counts = {folder: len(list((FIXTURES / folder).glob("*.rob"))) for folder in ("valid", "invalid", "compat")}
    cases = json.loads((FIXTURES / "simulation" / "cases.json").read_text(encoding="utf-8"))["cases"]
    assert counts == {"valid": 9, "invalid": 14, "compat": 3}
    assert len(cases) == 12
    for folder, count in counts.items():
        assert f"| `{folder}/` |" in SPEC and f"| {count} |" in SPEC


@pytest.mark.parametrize("path", files("valid") + files("invalid"))
def test_v1_2_fixture_is_well_formed(path):
    expected = header(path)
    folder = path.parent.name
    assert expected["version"] == "v1.2" and expected["set"] == folder
    if folder == "valid":
        assert expected["ok"]
        return
    code = expected["code"]
    assert path.name.startswith(code.lower() + "_")
    assert code[1] == STAGE_DIGIT[expected["stage"]]
    assert code in defined_codes(SPEC) | defined_codes(SPEC_V1_1), f"{code} is not defined"


@pytest.mark.parametrize("path", files("valid") + files("invalid"))
def test_v1_1_front_end_rejects_every_v1_2_file_at_the_marker(path):
    lines = path.read_text(encoding="utf-8").splitlines()
    marker = next(i for i, line in enumerate(lines, 1) if line.startswith("robolang "))
    with pytest.raises(RoboLangError) as raised:
        check_file(path)
    assert (raised.value.code, raised.value.line) == ("E213", marker), raised.value


@pytest.mark.parametrize("path", files("compat"))
def test_compat_fixture_keeps_its_v1_1_result(path):
    expected = header(path)
    assert expected["version"] == "v1.1" and expected["set"] == "compat"
    if expected["ok"]:
        result = check_file(path)
        assert not result.warnings
        return
    with pytest.raises(RoboLangError) as raised:
        check_file(path)
    assert (raised.value.code, raised.value.line) == (expected["code"], int(expected["line"])), raised.value


def test_simulation_cases_are_consistent():
    sim = FIXTURES / "simulation"
    cases = json.loads((sim / "cases.json").read_text(encoding="utf-8"))["cases"]
    ids = [case["id"] for case in cases]
    assert len(ids) == len(set(ids))

    used = {case["source"] for case in cases} | {case["world"] for case in cases}
    for path in sorted((sim / "sources").glob("*.rob")) + sorted((sim / "worlds").glob("*.json")):
        assert path.relative_to(ROOT).as_posix() in used, f"{path.name} is not used by any case"

    run_codes = defined_codes(SIM_SPEC)
    for case in cases:
        source = (ROOT / case["source"]).read_text(encoding="utf-8")
        assert "\nrobolang 1.2;\n" in source
        name, params = re.search(r"^task (\w+)\(([^)]*)\)", source, re.MULTILINE).groups()
        world = json.loads((ROOT / case["world"]).read_text(encoding="utf-8"))
        assert world["world_schema_version"] == "1.0"
        bindings = world["tasks"][name]["bindings"]
        assert list(bindings) == [p.split()[1] for p in params.split(", ")], case["id"]

        expect = case["expect"]
        assert set(case["tests"]) <= {"L-T5", "L-T6"}
        assert expect["state"] == expect["states"][-1]
        assert expect["exit"] == {"SUCCEEDED": 0, "FAILED": 4}[expect["state"]]
        if expect["failure"]:
            assert expect["failure"]["code"] in run_codes
        if expect["state"] == "SUCCEEDED":
            assert all(r == "true" for r in expect["preconditions"])
