"""The shipped RoboLang 1.4 reference, CLI command, examples, vocabulary and benchmark integration."""

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
        assert result.ir.language_version == "1.4"


def test_reference_names_every_reserved_word_and_annotation_key():
    words = set(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", reference()))
    assert set(tokens.RESERVED_V1_4) <= words
    assert set(tokens.SAFETY_KEYS + tokens.VERIFY_KEYS) <= words


def test_every_diagnostic_code_in_the_reference_exists_in_the_implementation():
    cited = set(re.findall(r"\b[A-Z]\d{3}\b", reference()))
    implemented = set()
    for path in (ROOT / "src" / "robolang").rglob("*.py"):
        implemented.update(re.findall(r'["\']([A-Z]\d{3})["\']', path.read_text(encoding="utf-8")))
    assert cited and cited <= implemented


def test_reference_is_1_4_and_teaches_branching_and_what_stays_out():
    text = reference()
    assert text.startswith("# RoboLang 1.4 language reference") and "robolang 1.3;" not in text
    for new in ("`if", "`else if", "fail \"", "E216", "E307", "E309", "E313", "S261", "S262"):
        assert new in text, new
    # nesting depth, the goal of a path, and what stays out of the language
    assert "at most 4 deep" in text and "Every path\n  must have at least one goal condition" in text
    for out in ("loops", "retry", "error handling", "counts", "clocks", "message waits"):
        assert out in " ".join(text.split()), out


def test_reference_teaches_that_a_run_time_condition_is_decided_by_an_if_never_by_the_description():
    """The baseline lesson (docs/evidence/bench-set3-baseline.md G1, G2), in plain words."""
    text = " ".join(reference().split())
    for phrase in ("when the job starts", "at that moment", "decided by the program, with `if`",
                   "Never settle it by reading the description", "never write only one branch"):
        assert phrase in text, phrase


def test_the_prompts_are_for_1_4():
    full, minimal = bench.load_prompt("full"), bench.load_prompt("minimal")
    for prompt in (full, minimal):
        assert prompt.startswith("You write RoboLang 1.4 programs.") and "RoboLang 1.3 programs" not in prompt
    assert "robolang 1.4;" in full and "robolang 1.3;" not in full
    # 1.4 is a delta over 1.3, so the grammar-only prompt pastes both files, the base first
    assert "spec/grammar-v1.4.ebnf" in minimal and '"if"' in minimal and "robolang 1.3 grammar" not in minimal.lower()
    assert bench.GRAMMAR_FILE.name == "grammar-v1.4.ebnf" and bench.BASE_GRAMMAR_FILE.name == "grammar-v1.3.ebnf"
    base, delta = (f.read_text(encoding="utf-8").strip() for f in (bench.BASE_GRAMMAR_FILE, bench.GRAMMAR_FILE))
    assert base in minimal and delta in minimal and minimal.index(base) < minimal.index(delta)
