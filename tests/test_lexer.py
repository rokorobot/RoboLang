"""Tests for the v1.1 lexer (src/robolang/lexer.py)."""

from __future__ import annotations

import pytest

from robolang import Lexer, RoboLangError, TokenKind, tokenize
from robolang.nodes import Span
from robolang.tokens import FUTURE_RESERVED, KEYWORDS

K = TokenKind


def kinds(source: str) -> list[TokenKind]:
    return [t.kind for t in tokenize(source)]


def texts(source: str) -> list[str]:
    return [t.text for t in tokenize(source)][:-1]


def lex_error(source: str) -> RoboLangError:
    with pytest.raises(RoboLangError) as raised:
        tokenize(source)
    return raised.value


# -------- tokens --------

def test_statement_tokens():
    tokens = tokenize('move r to "dock";')

    assert [(t.kind, t.text) for t in tokens] == [
        (K.KEYWORD, "move"),
        (K.IDENT, "r"),
        (K.KEYWORD, "to"),
        (K.STRING, '"dock"'),
        (K.SEMICOLON, ";"),
        (K.EOF, ""),
    ]


def test_punctuation_and_annotation():
    assert kinds("@safety ( ) { } , ;") == [
        K.AT_SAFETY, K.LPAREN, K.RPAREN, K.LBRACE, K.RBRACE, K.COMMA, K.SEMICOLON, K.EOF,
    ]


@pytest.mark.parametrize("word", sorted(KEYWORDS | FUTURE_RESERVED))
def test_reserved_words_are_keywords(word):
    assert kinds(word) == [K.KEYWORD, K.EOF]


@pytest.mark.parametrize("word", ["max_speed", "ms", "s", "min", "m", "rad", "Move", "TASK", "mover", "r2_d2"])
def test_other_words_are_identifiers(word):
    assert kinds(word) == [K.IDENT, K.EOF]


@pytest.mark.parametrize("number", ["0", "5", "30", "0.5", "2.50", "007"])
def test_numbers(number):
    (token, _eof) = tokenize(number)
    assert (token.kind, token.text) == (K.NUMBER, number)


@pytest.mark.parametrize("source", ["5s", "5 s", "500ms", "1 min"])
def test_duration_is_number_then_word(source):
    assert kinds(source) == [K.NUMBER, K.IDENT, K.EOF]


@pytest.mark.parametrize("source, unit", [("0.5 m/s", "m/s"), ("0.5m/s", "m/s"), ("1.5rad/s", "rad/s")])
def test_speed_units(source, unit):
    tokens = tokenize(source)
    assert [t.kind for t in tokens] == [K.NUMBER, K.SPEED_UNIT, K.EOF]
    assert tokens[1].text == unit


def test_speed_unit_must_end_at_a_word_boundary():
    assert lex_error("m/sec").code == "E101"


# -------- strings --------

def test_string_value_excludes_quotes():
    (token, _eof) = tokenize('"camera_top"')
    assert (token.text, token.value) == ('"camera_top"', "camera_top")


def test_string_escapes_are_decoded():
    (token, _eof) = tokenize(r'"a \"b\" c:\\d\ne\tf"')
    assert token.value == 'a "b" c:\\d\ne\tf'


def test_empty_string_is_a_token():
    (token, _eof) = tokenize('""')
    assert (token.kind, token.value) == (K.STRING, "")


def test_string_may_contain_comment_markers_and_other_characters():
    (token, _eof) = tokenize('"// not a comment #-@ é"')
    assert token.value == "// not a comment #-@ é"


@pytest.mark.parametrize("escape", ["\\c", "\\'", "\\0", "\\x", "\\ "])
def test_unknown_escape(escape):
    error = lex_error(f'"ab{escape}"')
    assert (error.code, error.line, error.column) == ("E104", 1, 4)


@pytest.mark.parametrize("source", ['"abc', '"abc\n"', '"abc\\', '"abc\\"', '"abc\r\n"'])
def test_unterminated_string(source):
    error = lex_error(source)
    assert (error.code, error.line, error.column) == ("E102", 1, 1)


# -------- whitespace, comments, positions --------

def test_comments_and_whitespace_are_skipped():
    assert texts("// first\n  move r // trailing\n\tto dst ; // end") == ["move", "r", "to", "dst", ";"]


def test_spans_are_one_based_with_exclusive_end():
    tokens = tokenize('move r to "x";\n  grasp r box;')

    assert tokens[0].span == Span(1, 1, 1, 5)
    assert tokens[3].span == Span(1, 11, 1, 14)
    assert tokens[5].span == Span(2, 3, 2, 8)
    assert tokens[-1].span == Span(2, 15, 2, 15)


def test_crlf_line_endings_count_as_one_line_break():
    tokens = tokenize("move\r\nr\r\n")
    assert [(t.span.line, t.span.column) for t in tokens] == [(1, 1), (2, 1), (3, 1)]


def test_tab_counts_as_one_column():
    (token, _eof) = tokenize("\t\tmove")
    assert token.span.column == 3


def test_leading_byte_order_mark_is_ignored():
    (token, _eof) = tokenize("\ufeffmove")
    assert token.span == Span(1, 1, 1, 5)


def test_empty_source_is_just_eof():
    (eof,) = tokenize("")
    assert (eof.kind, eof.span) == (K.EOF, Span(1, 1, 1, 1))


# -------- errors --------

@pytest.mark.parametrize("source, column", [
    ("move #", 6),
    ("x - 1", 3),
    ("_r", 1),
    ("/* c */", 1),
    ("a / b", 3),
    ("@unsafe", 1),
    ("@safetynet", 1),
    ("é", 1),
    (".", 1),
])
def test_unexpected_character(source, column):
    error = lex_error(source)
    assert (error.code, error.line, error.column) == ("E101", 1, column)


@pytest.mark.parametrize("source", ["5.", "5.s", ".5", "1.2.3", "1.1.0"])
def test_malformed_number(source):
    assert lex_error(source).code == "E103"


def test_error_reports_filename_and_position():
    with pytest.raises(RoboLangError) as raised:
        tokenize("move r\n  to #", filename="demo.rob")

    assert str(raised.value) == "demo.rob:2:6: error[E101]: unexpected character '#'"


def test_lexer_is_lazy():
    """Text after the tokens already requested is not examined."""
    lexer = Lexer("robolang 2.0; # ~ not v1.1 syntax")

    assert [lexer.next_token().text for _ in range(3)] == ["robolang", "2.0", ";"]
    with pytest.raises(RoboLangError):
        lexer.next_token()
