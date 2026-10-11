"""
Runs the v1.1 conformance fixtures (tests/fixtures) against the lexer and parser.

Covered here: every valid fixture and every invalid fixture whose expected
error is lexical (E1xx) or syntactic (E2xx).

Not covered: the static fixtures (E3xx). Static validation is RL-11C. For those
this module only checks that the file is syntactically valid, so the error they
describe really is a static one.
"""

from __future__ import annotations
import re
from pathlib import Path

import pytest

from robolang import RoboLangError, parse_file

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
def test_valid_fixture_parses(path):
    expected = expectation(path)
    result = parse_file(path)

    warnings = [(w.code, w.line) for w in result.warnings]
    if expected["wcode"]:
        assert warnings == [(expected["wcode"], int(expected["wline"]))]
    else:
        assert warnings == []


@pytest.mark.parametrize("path", INVALID_SYNTAX)
def test_invalid_fixture_is_rejected(path):
    expected = expectation(path)

    with pytest.raises(RoboLangError) as raised:
        parse_file(path)

    assert raised.value.code == expected["code"], raised.value
    if expected["line"]:
        assert raised.value.line == int(expected["line"]), raised.value
    assert raised.value.code[1] == {"lexical": "1", "syntax": "2"}[expected["stage"]]


@pytest.mark.parametrize("path", INVALID_STATIC)
def test_static_fixture_is_syntactically_valid(path):
    """E3xx fixtures must get past the parser; rejecting them is RL-11C."""
    parse_file(path)
