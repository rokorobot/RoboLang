"""The shipped RoboLang 1.6 reference, CLI command, examples, vocabulary and benchmark integration."""

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
        assert result.ir.language_version == "1.6"


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


def test_reference_is_1_6_and_teaches_branching_handlers_and_what_stays_out():
    text = reference()
    assert text.startswith("# RoboLang 1.6 language reference")
    assert "robolang 1.3;" not in text and "robolang 1.4;" not in text and "robolang 1.5;" not in text
    for new in ("`if", "`else if", "fail \"", "E216", "E307", "E309", "E313", "S261", "S262",
                "else {", "E201", "E316"):
        assert new in text, new
    # nesting depth, the goal of a path, and what stays out of the language
    assert "at most 4 deep" in text and "Every path\n  must have at least one goal condition" in text
    for out in ("loops", "counts", "clocks", "message waits", "arithmetic", "string formatting"):
        assert out in " ".join(text.split()), out


def test_reference_teaches_the_1_5_handler_rules():
    """Decision 0035 E6: a wait's handler fails unless told otherwise; a handler never succeeds on another goal."""
    text = " ".join(reference().split())
    for phrase in ("Every `wait until` has a handler", 'else { fail "<what did not happen>"; }',
                   "never changes what success means", "call it done", "cannot be represented",
                   "A fixed number of retries is allowed", "as many times as it takes",
                   "If it does not clear, do nothing", "It is representable: write the program",
                   "give each step that can fail its own handler", "only when the request says to carry on"):
        assert phrase in text, phrase


def test_reference_teaches_measured_values():
    """Decision 0036 PR 2 (V13 amendment 1): `inspect` never reads a value; the rest of "Measured values"."""
    text = " ".join(reference().split())
    assert "## Measured values" in text
    for phrase in ("**`inspect` never reads a value.**",
                   "Weighing, taking a temperature, reading a batch number or a barcode is `measure`",
                   "Do not substitute `inspect`",
                   "`temperature`: `degC`, `K`", "`mass`: `g`, `kg`", "`distance`: `mm`, `cm`, `m`", "`label`: text",
                   "Units of one kind convert exactly", "**Degrees Fahrenheit is not a unit:**",
                   "only in `if` and `wait until`", "not in `pre` or `goal` (`E321`)",
                   "**Live readings, in `wait until` only.**", "**Uncertainty.**", "otherwise unknown",
                   "An unknown condition fails the run in an `if` (`S261`)",
                   "a reading of `59.8 degC +/- 0.5` against `< 60 degC` is unknown",
                   "**Bindings.**", "set once and never changed",
                   "**One exception, to retry a reading:**", "`E317`", "`E318`", "`E319`", "`E320`", "`E322`",
                   '`communicate r to "pass" with t;`', "There is no formatting",
                   "**Contact.**", "`move r` to the object first", "`S235`", "`S236`",
                   "**The value is read when the program runs.**", "Never settle a comparison from a description",
                   "arithmetic (an average", "choosing an object at run time"):
        assert phrase in text, phrase
    assert "robolang 1.6;" in text and "measure r" in text


def test_reference_teaches_that_a_run_time_condition_is_decided_by_an_if_never_by_the_description():
    """The baseline lesson (docs/evidence/bench-set3-baseline.md G1, G2), in plain words."""
    text = " ".join(reference().split())
    for phrase in ("when the job starts", "at that moment", "decided by the program, with `if`",
                   "Never settle it by reading the description", "never write only one branch"):
        assert phrase in text, phrase


def test_the_prompts_are_for_1_6():
    full, minimal = bench.load_prompt("full"), bench.load_prompt("minimal")
    for prompt in (full, minimal):
        assert prompt.startswith("You write RoboLang 1.6 programs.") and "RoboLang 1.5 programs" not in prompt
        assert "RoboLang 1.5 cannot say it" not in prompt and "RoboLang 1.6 cannot say it" in prompt
    assert "measure" in full and "robolang 1.6;" in full and "robolang 1.5;" not in full and "robolang 1.4;" not in full
    # 1.6 is a delta over 1.5, over 1.4, over 1.3: the grammar-only prompt pastes all four, the base first
    assert "spec/grammar-v1.6.ebnf" in minimal and '"measure"' in minimal and '"if"' in minimal and "handler" in minimal
    assert [f.name for f in bench.GRAMMAR_FILES] == ["grammar-v1.3.ebnf", "grammar-v1.4.ebnf", "grammar-v1.5.ebnf",
                                                     "grammar-v1.6.ebnf"]
    texts = [f.read_text(encoding="utf-8").strip() for f in bench.GRAMMAR_FILES]
    assert all(t in minimal for t in texts)
    assert [minimal.index(t) for t in texts] == sorted(minimal.index(t) for t in texts)


def test_both_prompts_say_a_request_about_entities_not_listed_now_cannot_be_represented():
    """Decision 0036, acceptance option C (2026-11-10): the grammar-only answers to x03_any_hot_pan measured the one pan
    the world listed, although the request was about pans that may arrive. Both wrappers carry the general rule."""
    for name in ("full", "minimal"):
        text = " ".join(bench.load_prompt(name).split())
        assert "may appear or change after the job starts" in text and "cannot be represented" in text, name
