"""
Runs all v1.1 conformance fixtures (tests/fixtures) against the front end.

- valid fixtures must pass the complete front end (`check_file`) with exactly
  the warnings their header states;
- lexical (E1xx) and syntax (E2xx) fixtures must be rejected by the parser;
- static (E3xx) fixtures must get past the parser and be rejected by static
  validation.

Each rejection is checked for the expected code and line.
"""

from __future__ import annotations
import re
from pathlib import Path

import pytest

from robolang import RoboLangError, check_file, parse_file

FIXTURES = Path(__file__).parent / "fixtures"
EXPECT = re.compile(
    r"^// expect: (?:(?P<ok>ok)(?:, warning (?P<wcode>W\d+) line (?P<wline>\d+))?"
    r"|error (?P<code>E\d+) \((?P<stage>\w+)\)(?: line (?P<line>\d+))?)$",
    re.MULTILINE,
)


def expectation(path: Path) -> dict:
    match = EXPECT.search(path.read_text(encoding="utf-8"))
    assert match, f"{path.name}: missing or malformed '// expect:' header"
    return match.groupdict()


def fixtures(folder: str, stages: tuple[str, ...] = ()) -> list:
    paths = sorted((FIXTURES / folder).glob("*.rob"))
    if stages:
        paths = [p for p in paths if expectation(p)["stage"] in stages]
    return [pytest.param(p, id=p.stem) for p in paths]


VALID = fixtures("valid")
INVALID_SYNTAX = fixtures("invalid", ("lexical", "syntax"))
INVALID_STATIC = fixtures("invalid", ("static",))


def test_fixture_inventory():
    """Every fixture is assigned to exactly one group, and the groups are not empty."""
    total = len(list(FIXTURES.glob("*/*.rob")))
    assert len(VALID) + len(INVALID_SYNTAX) + len(INVALID_STATIC) == total
    assert VALID and INVALID_SYNTAX and INVALID_STATIC


@pytest.mark.parametrize("path", VALID)
def test_valid_fixture_passes_the_complete_front_end(path):
    expected = expectation(path)
    result = check_file(path)

    warnings = [(w.code, w.line) for w in result.warnings]
    if expected["wcode"]:
        assert warnings == [(expected["wcode"], int(expected["wline"]))]
    else:
        assert warnings == []


@pytest.mark.parametrize("path", INVALID_SYNTAX)
def test_lexical_or_syntax_fixture_is_rejected_by_the_parser(path):
    expected = expectation(path)

    for front_end in (parse_file, check_file):
        with pytest.raises(RoboLangError) as raised:
            front_end(path)

        assert raised.value.code == expected["code"], raised.value
        if expected["line"]:
            assert raised.value.line == int(expected["line"]), raised.value
        assert raised.value.code[1] == {"lexical": "1", "syntax": "2"}[expected["stage"]]


@pytest.mark.parametrize("path", INVALID_STATIC)
def test_static_fixture_is_rejected_by_static_validation(path):
    expected = expectation(path)

    parse_file(path)  # syntactically valid: the error is a static one
    with pytest.raises(RoboLangError) as raised:
        check_file(path)

    assert raised.value.code == expected["code"], raised.value
    assert raised.value.line == int(expected["line"]), raised.value
    assert raised.value.code[1] == "3"
