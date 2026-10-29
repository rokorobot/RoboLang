"""Tests for LANG-1, combined conditions in RoboLang 1.2 (spec/robolang-v1.2.md, spec/ir-schema-v1.1.md)."""

from __future__ import annotations
import json

import pytest

from robolang import RoboLangError, TokenKind, check, parse, tokenize
from robolang import nodes as ast
from robolang.ir import compile_source, to_dict, to_json
from robolang.nodes import Span
from robolang.sim.values import Truth

PARAMS = "robot r, object box, location dst, region a, region b, region c"


def task(pre: str = "", plan: str = "move r to dst;", version: str | None = "1.2") -> str:
    marker = f"robolang {version};\n" if version else ""
    return f"{marker}task t({PARAMS}) {{\n  pre {{ {pre} }}\n  plan {{ {plan} }}\n}}\n"


def error(source: str) -> RoboLangError:
    with pytest.raises(RoboLangError) as raised:
        compile_source(source)
    return raised.value


def pre_ir(condition: str) -> dict:
    return to_dict(compile_source(task(condition + ";")).ir.tasks[0].preconditions[0])


# -------- version and reserved words (sections 2.1, 2.2) --------

def test_operator_words_are_names_in_v1_1_and_reserved_in_v1_2():
    source = "task t(robot and, location or) {\n  plan { move and to or; }\n}\n"
    assert check(source).program.version is None
    assert check("robolang 1.1;\n" + source).program.version.version == "1.1"
    raised = error("robolang 1.2;\n" + source)
    assert (raised.code, raised.line, raised.column) == ("E212", 2, 14)


@pytest.mark.parametrize(
    ("marker", "kind"),
    [("", TokenKind.IDENT), ("robolang 1.1; ", TokenKind.IDENT), ("robolang 1.2; ", TokenKind.KEYWORD)],
)
def test_tokenize_follows_the_version_marker_like_the_parser(marker, kind):
    words = [t for t in tokenize(marker + "and or not and_x") if t.text in ("and", "or", "not", "and_x")]
    assert [t.kind for t in words] == [kind, kind, kind, TokenKind.IDENT]


def test_combined_conditions_need_the_v1_2_marker():
    for version in (None, "1.1"):
        assert error(task("region_clear a and region_clear b;", version=version)).code == "E202"


def test_unsupported_versions_name_every_supported_one():
    raised = error("robolang 1.4;\n" + task(version=None))
    assert (raised.code, raised.line, raised.column) == ("E213", 1, 10)
    assert "1.1, 1.2 and 1.3" in str(raised)


def test_ir_and_language_versions_follow_the_marker():
    for version, expected in ((None, ("1.1", "1.0")), ("1.1", ("1.1", "1.0")), ("1.2", ("1.2", "1.1"))):
        document = json.loads(to_json(compile_source(task("robot_ready r;", version=version)).ir))
        assert (document["language_version"], document["ir_schema_version"]) == expected


# -------- syntax and AST (sections 3, 6) --------

def conditions(pre: str) -> tuple:
    return parse(task(pre)).program.tasks[0].pre.conditions


def test_not_binds_to_the_single_operand_after_it():
    (condition,) = conditions("not region_clear a and region_clear b;")
    assert isinstance(condition, ast.And)
    assert isinstance(condition.operands[0], ast.Not) and isinstance(condition.operands[1], ast.RegionClear)


def test_ast_keeps_grouping_as_written():
    (condition,) = conditions("(region_clear a and region_clear b) and region_clear c;")
    assert isinstance(condition, ast.And) and isinstance(condition.operands[0], ast.And)
    assert len(condition.operands) == 2


def test_spans_cover_operands_and_exclude_parentheses():
    (condition,) = conditions("not (region_clear a);")
    # line 3 is "  pre { not (region_clear a); }"
    assert condition.span == Span(3, 9, 3, 28)
    assert condition.operand.span == Span(3, 14, 3, 28)


@pytest.mark.parametrize(
    ("pre", "column"),
    [
        ("region_clear a and region_clear b or region_clear c;", 43),
        ("region_clear a or region_clear b and region_clear c;", 42),
        ("(region_clear a and region_clear b or region_clear c);", 44),
    ],
)
def test_mixing_and_with_or_is_reported_at_the_differing_word(pre, column):
    raised = error(task(pre))
    assert (raised.code, raised.line, raised.column) == ("E214", 3, column)


@pytest.mark.parametrize(
    ("pre", "valid"),
    [
        ("((((not not not not robot_ready r))));", True),
        ("not not not not not not not robot_ready r and region_clear a;", True),
        ("not not not not not not not not robot_ready r and region_clear a;", False),
        ("not not not not not not not not not robot_ready r;", False),
        ("(((((((((robot_ready r)))))))));", False),
        ("((((((((region_clear a and region_clear b))))))));", False),
    ],
)
def test_depth_bound_counts_not_parentheses_and_chains(pre, valid):
    if valid:
        compile_source(task(pre))
        return
    raised = error(task(pre))
    assert (raised.code, raised.line, raised.column) == ("E215", 3, 9)


@pytest.mark.parametrize(
    ("pre", "code"),
    [
        ("not ;", "E201"),
        ("robot_ready r and ;", "E201"),
        ("();", "E201"),
        ("(robot_ready r;", "E201"),
        ("robot_ready r and flying r;", "E204"),
        ("robot_ready r or message_received \"x\";", "E211"),
        ("robot_ready r and region_clear middle;", "E303"),
        ("not (robot_ready r and region_clear dst);", "E304"),
        ("robot_ready r or sensor_ok \"\";", "E307"),
    ],
)
def test_errors_inside_combined_conditions(pre, code):
    assert error(task(pre)).code == code


def test_wait_until_takes_a_combined_condition():
    step = compile_source(task(plan="wait until not region_clear a or region_clear b timeout 5s;")).ir.tasks[0].steps[0]
    assert to_dict(step)["condition"] == {
        "kind": "or",
        "conditions": [
            {"kind": "not", "condition": {"kind": "region_clear", "region": "a"}},
            {"kind": "region_clear", "region": "b"},
        ],
    }


# -------- IR canonical form (IR schema 1.1 section 2.1) --------

def test_grouping_flattens_into_one_operator():
    flat = pre_ir("region_clear a and region_clear b and region_clear c")
    assert pre_ir("(region_clear a and region_clear b) and region_clear c") == flat
    assert pre_ir("region_clear a and (region_clear b and region_clear c)") == flat
    assert [c["region"] for c in flat["conditions"]] == ["a", "b", "c"]


def test_only_grouping_is_rewritten():
    assert pre_ir("(region_clear a)") == {"kind": "region_clear", "region": "a"}
    assert pre_ir("not not region_clear a") == {
        "kind": "not", "condition": {"kind": "not", "condition": {"kind": "region_clear", "region": "a"}},
    }
    mixed = pre_ir("(region_clear a or region_clear b) and region_clear c")
    assert [c["kind"] for c in mixed["conditions"]] == ["or", "region_clear"]
    assert pre_ir("region_clear b and region_clear a")["conditions"][0]["region"] == "b"
    assert len(pre_ir("region_clear a and region_clear a")["conditions"]) == 2


def test_key_order_is_kind_first():
    assert list(pre_ir("region_clear a or region_clear b")) == ["kind", "conditions"]
    assert list(pre_ir("not region_clear a")) == ["kind", "condition"]


# -------- three-valued logic (section 8.1) --------

T, F, U = Truth.TRUE, Truth.FALSE, Truth.UNKNOWN


def test_not_table():
    assert [v.negate() for v in (T, F, U)] == [F, T, U]


@pytest.mark.parametrize(
    ("a", "b", "conjunction", "disjunction"),
    [
        (T, T, T, T), (T, F, F, T), (T, U, U, T),
        (F, T, F, T), (F, F, F, F), (F, U, F, U),
        (U, T, U, T), (U, F, F, U), (U, U, U, U),
    ],
)
def test_and_or_tables(a, b, conjunction, disjunction):
    assert Truth.all_of((a, b)) is conjunction
    assert Truth.any_of((a, b)) is disjunction
