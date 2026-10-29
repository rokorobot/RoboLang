"""
Lexer for RoboLang v1.1 to v1.3 (spec/robolang-v1.1.md, sections 2 and 3).

Tokens are produced on demand. The parser uses this to reject an unsupported
language version (E213) without interpreting the rest of the file; after the
version marker it lexes the remainder in one go. A `robolang 1.2;` or `robolang 1.3;` marker
switches the reserved words to that version's set before any later word is lexed.
"""

from __future__ import annotations

from .diagnostics import Diagnostic, RoboLangError
from .nodes import Span
from .tokens import ESCAPES, RESERVED, RESERVED_BY_VERSION, SPEED_UNITS, Token, TokenKind

_LETTERS = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ"
_DIGITS = "0123456789"
_WORD_CHARS = _LETTERS + _DIGITS + "_"
_PUNCTUATION = {
    "(": TokenKind.LPAREN,
    ")": TokenKind.RPAREN,
    "{": TokenKind.LBRACE,
    "}": TokenKind.RBRACE,
    ",": TokenKind.COMMA,
    ";": TokenKind.SEMICOLON,
}
_BOM = "﻿"
_AT_SAFETY = "@safety"
_AT_VERIFY = "@verify"


class Lexer:
    def __init__(self, source: str, filename: str = "<string>"):
        self._src = source[1:] if source.startswith(_BOM) else source
        self._filename = filename
        self._pos = 0
        self._line = 1
        self._col = 1
        self.reserved = RESERVED  # RESERVED_BY_VERSION[...] after a `robolang 1.2;` or `1.3;` marker
        self.verify = False  # `@verify` is an annotation after a `robolang 1.3;` marker only

    def set_language(self, version: str):
        """Switch to the words and annotations of language `version` (a `robolang` marker was read)."""
        if version in RESERVED_BY_VERSION:
            self.reserved = RESERVED_BY_VERSION[version]
        self.verify = version == "1.3"

    # -------- helpers --------

    def _peek(self, offset: int = 0) -> str:
        i = self._pos + offset
        return self._src[i] if i < len(self._src) else ""

    def _span(self, length: int) -> Span:
        """Span of `length` characters starting at the current position (tokens never span lines)."""
        return Span(self._line, self._col, self._line, self._col + length)

    def _error(self, code: str, message: str, length: int = 1):
        raise RoboLangError(Diagnostic(code, message, self._span(length), filename=self._filename))

    def _take(self, kind: TokenKind, length: int, value: str | None = None) -> Token:
        token = Token(kind, self._src[self._pos:self._pos + length], self._span(length), value)
        self._pos += length
        self._col += length
        return token

    def _skip_trivia(self):
        while True:
            ch = self._peek()
            if ch == "\n":
                self._pos += 1
                self._line += 1
                self._col = 1
            elif ch and ch in " \t\r":
                self._pos += 1
                self._col += 1
            elif ch == "/" and self._peek(1) == "/":
                while self._peek() not in ("", "\n"):
                    self._pos += 1
                    self._col += 1
            else:
                return

    # -------- token rules --------

    def next_token(self) -> Token:
        self._skip_trivia()
        ch = self._peek()

        if ch == "":
            return self._take(TokenKind.EOF, 0)
        if ch == '"':
            return self._string()
        if ch in _DIGITS:
            return self._number()
        if ch == "." and self._peek(1) and self._peek(1) in _DIGITS:
            self._error("E103", "malformed number: a decimal point needs digits on both sides")
        if ch == "@":
            return self._at_safety()
        if ch in _LETTERS:
            return self._unit_or_word()
        if ch in _PUNCTUATION:
            return self._take(_PUNCTUATION[ch], 1)
        if ch == "/" and self._peek(1) == "*":
            self._error("E101", "unexpected character '/': block comments are not supported, use '//'")
        self._error("E101", f"unexpected character {ch!r}")

    def _string(self) -> Token:
        chars: list[str] = []
        i = 1
        while True:
            ch = self._peek(i)
            if ch in ("", "\n", "\r"):
                self._error("E102", "unterminated string: a string must close on the line it starts on")
            if ch == '"':
                return self._take(TokenKind.STRING, i + 1, "".join(chars))
            if ch == "\\":
                escaped = self._peek(i + 1)
                if escaped in ("", "\n", "\r"):
                    self._error("E102", "unterminated string: a string must close on the line it starts on")
                if escaped not in ESCAPES:
                    self._col += i
                    self._error(
                        "E104",
                        f"unknown escape sequence '\\{escaped}'; the escapes are \\\" \\\\ \\n \\t",
                        length=2,
                    )
                chars.append(ESCAPES[escaped])
                i += 2
            else:
                chars.append(ch)
                i += 1

    def _number(self) -> Token:
        i = 0
        while self._peek(i) and self._peek(i) in _DIGITS:
            i += 1
        if self._peek(i) == ".":
            i += 1
            if not (self._peek(i) and self._peek(i) in _DIGITS):
                self._error("E103", "malformed number: a decimal point needs digits on both sides", length=i)
            while self._peek(i) and self._peek(i) in _DIGITS:
                i += 1
            if self._peek(i) == ".":
                self._error("E103", "malformed number: more than one decimal point", length=i + 1)
        return self._take(TokenKind.NUMBER, i)

    def _at_safety(self) -> Token:
        for text, kind in ((_AT_SAFETY, TokenKind.AT_SAFETY), (_AT_VERIFY, TokenKind.AT_VERIFY)):
            after = self._peek(len(text))
            if self._src.startswith(text, self._pos) and not (after and after in _WORD_CHARS):
                if kind is TokenKind.AT_SAFETY or self.verify:
                    return self._take(kind, len(text))
        only = "annotations are '@safety' and '@verify'" if self.verify else "annotation is '@safety'"
        self._error("E101", f"unexpected character '@': the only {only}")

    def _unit_or_word(self) -> Token:
        for unit in SPEED_UNITS:
            after = self._peek(len(unit))
            if self._src.startswith(unit, self._pos) and not (after and after in _WORD_CHARS):
                return self._take(TokenKind.SPEED_UNIT, len(unit))
        i = 1
        while self._peek(i) and self._peek(i) in _WORD_CHARS:
            i += 1
        word = self._src[self._pos:self._pos + i]
        return self._take(TokenKind.KEYWORD if word in self.reserved else TokenKind.IDENT, i)


def tokenize(source: str, filename: str = "<string>") -> list[Token]:
    """
    All tokens of `source`, ending with an EOF token.

    Reserved words follow the version marker, as in the parser: the 1.2 or 1.3 set after a leading
    `robolang 1.2` or `robolang 1.3`, otherwise the 1.1 set, where `and`, `or` and `not` are names.
    """
    lexer = Lexer(source, filename)
    tokens = []
    while True:
        token = lexer.next_token()
        tokens.append(token)
        if token.kind is TokenKind.EOF:
            return tokens
        if len(tokens) == 2 and tokens[0].text == "robolang" and token.text in RESERVED_BY_VERSION:
            lexer.set_language(token.text)
