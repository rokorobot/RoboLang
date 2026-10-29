"""The shipped RoboLang 1.3 reference, CLI command, examples, vocabulary and benchmark integration."""

from __future__ import annotations
import io
import re
from pathlib import Path

from bench import lib as bench
from robolang import tokens
from robolang.cli import main
from robolang.ir import compile_source

ROOT = Path(__file__).resolve().parent.parent
DOC = ROOT / "docs" / "language-reference.md"
PACKAGED = ROOT / "src" / "robolang" / "reference" / "language-reference.md"


def reference() -> str:
    return DOC.read_text(encoding="utf-8")


def test_docs_package_cli_and_benchmark_use_the_same_reference():
    text = reference()
    assert text.isascii()  # the CLI promises deterministic bytes even on narrow Windows consoles
    assert PACKAGED.read_text(encoding="utf-8") == text
    out, err = io.StringIO(), io.StringIO()
    assert main(["reference"], out=out, err=err) == 0
    assert (out.getvalue(), err.getvalue()) == (text, "")
    prompt = bench.load_prompt("full")
    assert bench.REFERENCE_MARK not in prompt and prompt.count(text.rstrip("\n")) == 1


def test_every_complete_rob_example_compiles():
    examples = re.findall(r"```rob\n(.*?)```", reference(), flags=re.DOTALL)
    assert examples
    for index, source in enumerate(examples, start=1):
        result = compile_source(source, f"language-reference-example-{index}.rob")
        assert result.ir.language_version == "1.3"


def test_reference_names_every_reserved_word_and_annotation_key():
    words = set(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", reference()))
    assert set(tokens.RESERVED_V1_3) <= words
    assert set(tokens.SAFETY_KEYS + tokens.VERIFY_KEYS) <= words


def test_every_diagnostic_code_in_the_reference_exists_in_the_implementation():
    cited = set(re.findall(r"\b[A-Z]\d{3}\b", reference()))
    implemented = set()
    for path in (ROOT / "src" / "robolang").rglob("*.py"):
        implemented.update(re.findall(r'["\']([A-Z]\d{3})["\']', path.read_text(encoding="utf-8")))
    assert cited and cited <= implemented
