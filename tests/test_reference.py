"""The shipped RoboLang 2.3 reference, CLI command, examples, vocabulary and benchmark integration."""

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
        assert result.ir.language_version == "2.3"


def test_reference_names_every_reserved_word_and_annotation_key():
    words = set(re.findall(r"[A-Za-z_][A-Za-z0-9_]*", reference()))
    assert set(tokens.RESERVED_V1_4) <= words
    assert set(tokens.MISSION_WORDS) <= words  # RoboLang 2.0
    assert set(tokens.SAFETY_KEYS + tokens.VERIFY_KEYS) <= words


def test_every_diagnostic_code_in_the_reference_exists_in_the_implementation():
    cited = set(re.findall(r"\b[A-Z]\d{3}\b", reference()))
    implemented = set()
    for path in (ROOT / "src" / "robolang").rglob("*.py"):
        implemented.update(re.findall(r'["\']([A-Z]\d{3})["\']', path.read_text(encoding="utf-8")))
    assert cited and cited <= implemented


def test_reference_is_2_3_and_teaches_branching_handlers_missions_and_what_stays_out():
    text = reference()
    assert text.startswith("# RoboLang 2.3 language reference")
    for old in ("robolang 1.3;", "robolang 1.4;", "robolang 1.5;", "robolang 1.6;", "robolang 1.7;", "robolang 1.9;"):
        assert old not in text, old
    for new in ("`if", "`else if", "fail \"", "E216", "E307", "E309", "E313", "S261", "S262",
                "else {", "E201", "E316"):
        assert new in text, new
    # nesting depth, the goal of a path, and what stays out of the language
    assert "at most 4 deep" in text and "Every path\n  must have at least one goal condition" in text
    for out in ("loops", "dates", "confirming that a message the robot sent was delivered", "arithmetic",
                "string formatting", "triggers", "choosing by a reading of each candidate"):
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
                   "arithmetic (an average", "choosing an object by a reading of each candidate"):
        assert phrase in text, phrase
    assert "robolang 2.3;" in text and "measure r" in text


def test_reference_teaches_that_a_run_time_condition_is_decided_by_an_if_never_by_the_description():
    """The baseline lesson (docs/evidence/bench-set3-baseline.md G1, G2), in plain words."""
    text = " ".join(reference().split())
    for phrase in ("when the job starts", "at that moment", "decided by the program, with `if`",
                   "Never settle it by reading the description", "never write only one branch"):
        assert phrase in text, phrase


def test_the_prompts_are_for_2_3():
    full, minimal = bench.load_prompt("full"), bench.load_prompt("minimal")
    for prompt in (full, minimal):
        assert prompt.startswith("You write RoboLang 2.3 programs.") and "RoboLang 2.2 programs" not in prompt
        assert "RoboLang 2.2 cannot say it" not in prompt and "RoboLang 2.3 cannot say it" in prompt
        assert '"mission": "<name of the mission>"' in prompt and "needs no mission, and no decline" in prompt
    assert "measure" in full and "robolang 2.3;" in full and "robolang 2.2;" not in full and "robolang 1.4;" not in full
    # 2.3 is a delta over 2.2, 2.1, 2.0, 1.9, 1.8, 1.7, 1.6, 1.5, 1.4 and 1.3: the grammar-only prompt pastes all eleven
    assert "spec/grammar-v2.3.ebnf" in minimal and "spec/grammar-v2.2.ebnf" in minimal and "The eleven files" in minimal
    assert '"choose"' in minimal and "E329" in minimal and "E330" in minimal and "MUST END IN fail" in minimal
    assert "choosing by a reading of each candidate" in minimal and "choosing a person" in minimal
    assert "NEVER READS A VALUE" in minimal and "Do not replace a choice" in minimal and "may come first" in minimal
    assert '"measure"' in minimal and '"if"' in minimal and "handler" in minimal
    assert '"no_go_region"' in minimal and "S003" in minimal and "X605" in minimal
    assert '"do"' in minimal and '"in_state"' in minimal and "S006" in minimal
    assert '"clock_time"' in minimal and '"time"' in minimal and "S010" in minimal and "another day" in minimal
    assert '"mission"' in minimal and '"repeat"' in minimal and "E325" in minimal and "S401" in minimal
    assert "triggers" in minimal and "nesting" in minimal and "480 min" in minimal
    assert '"message_received"' in minimal and "E327" in minimal and "S011" in minimal and "delivery" in minimal
    assert "patterns" in minimal and "the sender is the world's" in minimal
    assert '"fill"' in minimal and '"confidence"' in minimal and '"pressure"' in minimal and "E328" in minimal
    assert "a reading of a person" in minimal and "Do not replace a reading" in minimal and "OR a location" in minimal
    assert [f.name for f in bench.GRAMMAR_FILES] == ["grammar-v1.3.ebnf", "grammar-v1.4.ebnf", "grammar-v1.5.ebnf",
                                                     "grammar-v1.6.ebnf", "grammar-v1.7.ebnf", "grammar-v1.8.ebnf",
                                                     "grammar-v1.9.ebnf", "grammar-v2.0.ebnf", "grammar-v2.1.ebnf",
                                                     "grammar-v2.2.ebnf", "grammar-v2.3.ebnf"]
    texts = [f.read_text(encoding="utf-8").strip() for f in bench.GRAMMAR_FILES]
    assert all(t in minimal for t in texts)
    assert [minimal.index(t) for t in texts] == sorted(minimal.index(t) for t in texts)


def test_reference_teaches_missions_and_the_fixed_count_fix():
    """0041 L4 and the owner's teaching fix (2026-11-18): a fixed count is one task with the steps repeated (or a
    mission with that run count), a bounded number of retries of one step is a 1.5 handler: models over-declined
    both."""
    text = " ".join(reference().split())
    assert "## Missions" in text
    for phrase in ("**A fixed count is not a mission:**", '"three round trips" is one task with the steps written out',
                   "a mission with `at most 3 runs` is also accepted", '"retry the step up to 3 times"',
                   "is a failure handler, a program, not a decline", "**A fixed count needs no mission.**",
                   "Every repeat has a hard bound", "no hours, 3 hours are `180 min`", "`E325`", "`E326`", "`S401`",
                   "`S402`", "retry N;", "on failure",
                   "**Not in the language** (decline as `inexpressible`): **no bound**",
                   "**triggers**", "**nesting**", "**Choosing the next object**",
                   "Retries count against the bound", "A robot never receives a mission",
                   "never a count that was not asked for"):
        assert phrase in text, phrase


def test_both_prompts_say_a_request_about_entities_not_listed_now_cannot_be_represented():
    """Decision 0036, acceptance option C (2026-11-10): the grammar-only answers to x03_any_hot_pan measured the one pan
    the world listed, although the request was about pans that may arrive. Both wrappers carry the general rule."""
    for name in ("full", "minimal"):
        text = " ".join(bench.load_prompt(name).split())
        assert "may appear or change after the job starts" in text and "cannot be represented" in text, name


def test_reference_says_a_possible_unknown_is_not_a_reason_to_decline():
    """0036 follow-up (2026-11-10): Sonnet declined v08_warmer_of_two 3/3 because the comparison "could be unknown"."""
    text = " ".join(reference().split())
    assert "The chance of an unknown is never a reason to decline." in text and "which is the intended outcome" in text



def test_reference_teaches_region_limits():
    """Decision 0039 (L4): when to write a limit, never drop one, add none unasked, the whole task, infeasible."""
    text = " ".join(reference().split())
    assert "## Region limits" in text
    for phrase in ("@safety no_go_region wet;", "@safety in_region lane;",
                   "Write one limit for each region the request names",
                   "**Never drop a requested limit.**", "A `move` is a route, not a jump",
                   "**Do not add one the request did not ask for.**", "never for \"busy areas\"",
                   "**The whole task.**", "There is no limit on one step", "cannot be represented",
                   "**At most 8 limits per task, and a region is named once.**", "`E323`", "`E324`",
                   "**A request, not an enforcement.**", "never as enforced", "**What the world says.**",
                   "`contains`", "`honours`", "(`S003`, `S004`, `S005`)", "infeasible",
                   "Core carries no region limits"):
        assert phrase in text, phrase
    assert "E323" in text and "S005" in text and tokens.REGION_LIMITS == ("no_go_region", "in_region")
    assert set(tokens.REGION_LIMITS) <= set(tokens.RESERVED_V1_6)  # no new word: both have been reserved since 1.1


def test_reference_teaches_declared_actions():
    """Decision 0040 (L4): when to write `do`, always check the result, no replacing, no extra, at the target,
    one target, infeasible, never delegated."""
    text = " ".join(reference().split())
    assert "## Declared actions" in text
    for phrase in ("do r open door;", "do r set fryer to 175 degC;", 'in_state door "open"',
                   "exactly the **actions its profile declares**", "An action is a **name**, not a keyword",
                   "`open_door` is not `open`", "**When to write `do`.**", "**one** target", "**A value.**",
                   "never assumed reached", "**Always check the result.**", "`requested`",
                   "Never replace an action with a `move`, an `inspect` or a `communicate`",
                   "**Add no action the request did not ask for.**", "**The robot has to be at the target.**",
                   "(`S271`)", "**One target only.**", "cannot be represented", "`do`, then `wait for 3 min`",
                   "**Infeasible, not inexpressible.**", "(`S006`)", "(`S007`)", "(`S008`)", "(`S009`)",
                   "unknown fails closed", "`X606`", "**Charge and dock.**", "`in_state charger \"charging\"`"):
        assert phrase in text, phrase
    assert {"do", "in_state"} <= set(tokens.RESERVED_V1_8) - set(tokens.RESERVED_V1_6)
    assert "do in_state" in text and "| `in_state x \"open\"` |" in text


def test_reference_teaches_incoming_messages():
    """0043 (L4): the condition, copying the channel and text, the world's senders, never in a goal, no replacing, and
    what stays out."""
    text = " ".join(reference().split())
    assert "## Incoming messages" in text
    for phrase in ('message_received "line" with "TUGGER DOCKED"', 'message_received "nurse"', "any message at all",
                   "never a `wait for` of a guessed time", "a `region_clear` or a `sensor_ok` in its place",
                   "Copy the channel and the text exactly as the request gives them", "case-sensitive",
                   "Never change a case, never make up a text", "**no `with`**: any message",
                   "uses the channel the world lists for that party", "ask (`clarify`)",
                   "Who may send is the world's, never the program's", "accepts messages from", "ignored",
                   "There is no `from` in the program", "at or after the start of the run", "stays counted",
                   "a message at exactly the timeout counts", "**Never in a goal** (`E327`)",
                   "as the **last step of the plan**", "**Not asked, not added.**", "never a message it receives",
                   "this world has no message service", "`S011`", "`infeasible`", "if it does not come, stop the job",
                   "**delivery**", "**for each message**", "**the content of a message as a value**", "**patterns**",
                   "`X608`"):
        assert phrase in text, phrase
    assert "message_received" in tokens.FUTURE_RESERVED  # no new word: reserved since 1.1


def test_reference_teaches_the_four_new_kinds_2_2():
    """0044 (L4): the kinds, the unit `%`, pressure units, an exact count, a location, confidence with its label, no
    replacing, and what stays out."""
    text = " ".join(reference().split())
    assert "### More kinds: fill, confidence, pressure, count" in text
    for phrase in ("**`fill`** is how full something is", "`level < 25 %`", "**`confidence`** how sure the sensor is",
                   "`c >= 90 %`", "**`pressure`**", "`p < 450 kPa` and `p < 4.5 bar` are the same test",
                   "**`count`** how many items", "`n == 0`", "`measure r count of aisle_5 using tag as n;`",
                   "`%` is the unit of `fill` and `confidence`", "A bare fraction (`0.3`) or the word `percent`",
                   "(`E206`)", "comparing one with the other is `E319`", "There is no `psi`", "A `count` is exact",
                   "`n < 2.5` is `E328`", "it also takes `==` and `!=`",
                   "**`fill` and `count` may be read of an object or of a location**",
                   "A robot is not an object or a location", "`E304`", "**`confidence` belongs to a label.**",
                   "read with the **same** sensor", "**Do not replace a reading.**", "never an `inspect`",
                   "never a `label` reading in its place", "never dropped", "`infeasible`", "**Not in 2.2**",
                   "a kind no sensor can declare", "a reading of a person", "a reading of the robot itself",
                   "counting objects chosen at run time", "arithmetic on readings", "measures fill in % +/- 2",
                   "never what the value is"):
        assert phrase in text, phrase
    assert "fill" not in tokens.RESERVED_V2_0 and "count" not in tokens.RESERVED_V2_0  # recognised by position


def test_reference_teaches_choosing_at_run_time_2_3():
    """0045 (L4): the statement, the three candidate forms, `where`, the order and ties, `else` ends in `fail`, the goal
    names the choice, `choose` never reads a value (point 5), a choice may come first so the task can be delegated
    (point
    8), missions choose afresh, and what stays out (C9)."""
    text = " ".join(reference().split())
    assert "## Choosing at run time" in text
    for phrase in ("choose TYPE NAME from CANDIDATES [where CONDITION] [nearest to X | farthest from X]",
                   "binds a new name", "**Candidates**", "**`objects at L`**", "**`robots`**",
                   'There is no "any object in the world"', "**`where`**", "excludes", "world facts only", "`E330`",
                   "**`nearest to X`** or **`farthest from X`**", "straight-line distance", "(no position)",
                   "**A tie, and a choice with no order, go to the first candidate",
                   "**`else` is required, and it ends in `fail`**", "`E316`", "`E329`",
                   "A message may come before the `fail`", "`infeasible`",
                   "**The new name is used like a parameter** afterwards", "checked for the entity chosen",
                   'never "for some entity"', "`E331`", "`E302`", "**`choose` never reads a value.**",
                   "compare the names in an `if`", "**A choice that uses only world facts may come first**",
                   "can be sent to a robot", "**In a mission, each run chooses afresh.**", "no longer there",
                   "**`choose` is not a loop:**", "**Not in 2.3**", "by a **reading** of each candidate",
                   "acting on **each** candidate", "detecting objects the world does not list",
                   "**choosing a person**", "several choices that depend on each other",
                   "Do not replace a choice"):
        assert phrase in text, phrase
    for word in ("choose", "from", "where", "nearest", "farthest", "objects", "robots"):
        assert word not in tokens.RESERVED_V2_0  # recognised by position, so a program may still use them as names


def test_the_world_shown_to_the_model_lists_positions_in_a_world_file_1_8():
    world = {"world_schema_version": "1.8", "locations": ["a", {"id": "b", "position": [3, 4]},
                                                         {"id": "c", "position": ["1.5", 2]}]}
    assert bench._locations(world) == ["locations (position x, y in metres): a (no position), b (3, 4), c (1.5, 2)"]
    assert bench._locations({"world_schema_version": "1.7", "locations": ["a", "b"]}) == ["locations: a, b"]
