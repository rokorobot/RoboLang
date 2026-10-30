"""Tests for RoboLang 1.4, the front end: if/else, fail, IR 1.3 (spec/robolang-v1.4.md, decision 0027 B1-B9)."""

from __future__ import annotations
import io
import json
import os
from pathlib import Path

import pytest

from robolang import RoboLangError, check, parse, tokenize
from robolang import nodes as ast
from robolang.cli import main
from robolang.ir import compile_source, to_json
from robolang.ir_reader import IRError, read_ir

ROOT = Path(__file__).resolve().parent.parent
REFERENCE = ROOT / "bench" / "reference_branch"
ATTEMPTS = sorted(REFERENCE.glob("b_*.rob"))
PARAMS = "robot r, object box, location src, location dst, region cell, sensor cam"


def task(plan: str = "move r to dst;", goal: str | None = "goal { robot_at r dst; }",
         version: str | None = "1.4") -> str:
    marker = f"robolang {version};\n" if version else ""
    return f"{marker}task t({PARAMS}) {{\n{goal or ''}\nplan {{\n{plan}\n}}\n}}\n"


def errors(source: str) -> list[tuple[str, int]]:
    with pytest.raises(RoboLangError) as caught:
        check(source)
    return [(d.code, d.line) for d in caught.value.diagnostics]


def codes(source: str) -> list[str]:
    return [code for code, _ in errors(source)]


def message(source: str) -> str:
    with pytest.raises(RoboLangError) as caught:
        check(source)
    return caught.value.diagnostic.message


def cli(*args: str) -> tuple[int, str, str]:
    out, err = io.StringIO(), io.StringIO()
    previous = Path.cwd()
    os.chdir(ROOT)
    try:
        code = main(list(args), out=out, err=err)
    finally:
        os.chdir(previous)
    return code, out.getvalue(), err.getvalue()


def nested(depth: int) -> str:
    text = "move r to dst;"
    for _ in range(depth):
        text = f"if region_clear cell {{ {text} }}"
    return text


# -------- valid programs --------

def test_the_marker_selects_1_4():
    result = compile_source(task("if region_clear cell { move r to dst; }"))
    assert (result.ir.language_version, result.ir.ir_schema_version) == ("1.4", "1.3")


@pytest.mark.parametrize("path", ATTEMPTS, ids=lambda p: p.name)
def test_every_bench_reference_program_validates_under_the_1_4_marker(path):
    source = path.read_text(encoding="utf-8").replace("robolang 1.3;", "robolang 1.4;", 1)
    assert check(source).program.version.version == "1.4"


def test_there_are_twenty_bench_programs():
    assert len(ATTEMPTS) == 20


def test_if_with_else_and_branch_goals_parses_to_the_syntax_tree():
    plan = parse(task(
        "if region_clear cell { goal { object_at box src; } move r to src; } "
        "else { goal { object_at box dst; } move r to dst; }", goal=None)).program.tasks[0].plan
    (statement,) = plan.statements
    assert isinstance(statement, ast.If)
    assert statement.then.goal.conditions[0].location.text == "src"
    assert statement.orelse.goal.conditions[0].location.text == "dst"


def test_else_if_is_a_nested_if_in_the_else_branch():
    source = task("if region_clear cell { move r to src; } else if robot_ready r { move r to dst; }")
    (statement,) = parse(source).program.tasks[0].plan.statements
    (inner,) = statement.orelse.statements
    assert isinstance(inner, ast.If) and inner.orelse is None and statement.orelse.goal is None


def test_conditions_may_be_combined_and_statements_may_follow_an_if():
    check(task("if not region_clear cell and robot_ready r { wait for 1 s; } move r to dst;"))


def test_a_task_goal_is_optional_when_every_path_has_a_branch_goal():
    check(task("if region_clear cell { goal { robot_at r dst; } move r to dst; } "
               "else { goal { robot_at r src; } move r to src; }", goal=None))


def test_a_path_that_ends_in_fail_needs_no_goal():
    check(task('if region_clear cell { goal { robot_at r dst; } move r to dst; } else { fail "blocked"; }', goal=None))


def test_a_branch_may_hold_only_a_fail():
    check(task('move r to dst; if region_clear cell { fail "blocked"; }'))


def test_nesting_four_deep_is_allowed():
    check(task(nested(4)))


# -------- E216, E309, E313 and the other new errors --------

def test_nesting_five_deep_is_e216_at_the_fifth_if():
    assert errors(task(nested(5))) == [("E216", 5)]
    assert "deeper than 4" in message(task(nested(5)))


def test_an_else_if_counts_as_a_nested_if():
    chain = "if robot_ready r { move r to dst; }"
    for _ in range(4):
        chain = f"if robot_ready r {{ move r to dst; }} else {chain}"
    assert codes(task(chain)) == ["E216"]


def test_e309_names_the_path_without_a_goal():
    source = task("if region_clear cell { goal { robot_at r dst; } move r to dst; } else { move r to src; }", goal=None)
    assert codes(source) == ["E309"]
    assert "path else" in message(source)


def test_e309_names_a_nested_path():
    source = task("if region_clear cell { goal { robot_at r dst; } move r to dst; } "
                  "else { if robot_ready r { goal { robot_at r src; } move r to src; } else { move r to src; } }",
                  goal=None)
    assert "path else/else" in message(source)


def test_an_if_without_else_leaves_an_else_path_with_no_goal():
    source = task("if region_clear cell { goal { robot_at r dst; } move r to dst; }", goal=None)
    assert "path else" in message(source)


def test_a_plan_with_no_if_and_no_goal_is_e309():
    assert codes(task(goal=None)) == ["E309"]


def test_e309_is_unchanged_in_1_3():
    assert codes(task(goal=None, version="1.3")) == ["E309"]
    assert "RoboLang 1.3 requires" in message(task(goal=None, version="1.3"))


def test_fail_must_be_the_last_statement_of_a_branch():
    source = task('if region_clear cell { fail "x"; move r to dst; }')
    assert codes(source) == ["E313"]


def test_fail_outside_a_branch_is_e313():
    assert codes(task('move r to dst; fail "x";')) == ["E313"]


def test_fail_needs_a_reason_that_is_not_empty():
    assert codes(task('if region_clear cell { fail ""; }')) == ["E307"]
    assert codes(task("if region_clear cell { fail; }"))[0] == "E201"


def test_a_branch_goal_must_be_the_first_item_of_a_branch():
    assert codes(task("if region_clear cell { move r to dst; goal { robot_at r dst; } }")) == ["E208"]
    assert codes(task("goal { robot_at r dst; } move r to dst;", goal=None)) == ["E208"]


def test_an_empty_branch_or_empty_branch_goal_is_e208():
    assert codes(task("if region_clear cell { }")) == ["E208"]
    assert codes(task("if region_clear cell { goal { } move r to dst; }"))[0] == "E208"


def test_names_in_conditions_and_branches_are_checked():
    assert codes(task("if region_clear box { move r to dst; }")) == ["E304"]
    assert codes(task("if region_clear cell { move r to nowhere; } else { move r to dst; }")) == ["E303"]
    assert codes(task("if region_clear cell { goal { robot_at r nowhere; } move r to dst; }")) == ["E303"]


def test_else_without_if_and_a_missing_condition_are_syntax_errors():
    assert codes(task("else { move r to dst; }"))[0] == "E201"  # the rest of the statement is skipped
    assert codes(task("if { move r to dst; }"))[0] == "E201"


# -------- reserved words (B9) --------

@pytest.mark.parametrize("word", ["if", "else", "fail"])
def test_the_new_words_are_reserved_in_1_4_only(word):
    body = f"{{ goal {{ robot_ready {word}; }} plan {{ wait for 1 s; }} }}"
    assert codes(f"robolang 1.4;\ntask t(robot {word}) {body}") == ["E212"]
    for version in ("1.3", "1.2", "1.1"):
        goal = "goal { robot_ready %s; }" % word if version == "1.3" else ""
        pre = "" if version == "1.3" else "pre { robot_ready %s; }" % word
        check(f"robolang {version};\ntask t(robot {word}) {{ {pre} {goal} plan {{ wait for 1 s; }} }}")
    assert [t.kind.name for t in tokenize("robolang 1.4; if")][-2] == "KEYWORD"
    assert [t.kind.name for t in tokenize("robolang 1.3; if")][-2] == "IDENT"


def test_if_is_an_unknown_statement_in_1_3():
    assert codes(task("if region_clear cell { move r to dst; }", version="1.3"))[0] == "E203"


# -------- older versions are unchanged --------

def test_1_3_source_still_compiles_to_schema_1_2():
    ir = compile_source(task(version="1.3")).ir
    assert (ir.language_version, ir.ir_schema_version) == ("1.3", "1.2")
    assert json.loads(to_json(ir))["tasks"][0]["goal"] == [{"kind": "robot_at", "robot": "r", "location": "dst"}]


def test_a_1_3_ir_has_no_branching_steps():
    assert not compile_source(task(version="1.3")).ir.tasks[0].branching


# -------- IR 1.3 --------

SOURCE = task(
    'if region_clear cell { goal { robot_at r dst; } move r to dst; } else if robot_ready r { fail "later"; }',
    goal="goal { not holding r box; }")


def test_the_if_and_fail_steps_have_the_documented_shape():
    steps = json.loads(to_json(compile_source(SOURCE).ir))["tasks"][0]["steps"]
    assert list(steps[0]) == ["kind", "condition", "then", "else", "goal_then", "goal_else"]
    assert steps[0]["goal_then"] == [{"kind": "robot_at", "robot": "r", "location": "dst"}]
    assert steps[0]["goal_else"] is None
    inner = steps[0]["else"][0]
    assert inner["kind"] == "if" and inner["else"] == [] and inner["goal_then"] is None
    assert inner["then"] == [{"kind": "fail", "reason": "later"}]


def test_a_task_without_a_task_goal_has_an_empty_goal_array():
    source = task("if region_clear cell { goal { robot_at r dst; } move r to dst; } else { fail \"x\"; }", goal=None)
    assert json.loads(to_json(compile_source(source).ir))["tasks"][0]["goal"] == []


@pytest.mark.parametrize("source", [SOURCE, task("if region_clear cell { goal { robot_at r dst; } move r to dst; } "
                                                  "else { fail \"x\"; }", goal=None)])
def test_ir_round_trips_through_the_reader(source):
    compiled = compile_source(source).ir
    read = read_ir(to_json(compiled).encode("ascii"))
    assert read == compiled
    assert to_json(read) == to_json(compiled)


def edited(change):
    document = json.loads(to_json(compile_source(
        task("if region_clear cell { goal { robot_at r dst; } move r to dst; } else { fail \"x\"; }", goal=None)).ir))
    change(document["tasks"][0])
    return json.dumps(document).encode()


def problems(data: bytes) -> list[tuple[str, str]]:
    with pytest.raises(IRError) as caught:
        read_ir(data)
    return [(d.code, d.pointer) for d in caught.value.diagnostics]


def test_the_reader_names_a_path_without_a_goal():
    def drop(t):
        t["steps"][0]["goal_then"] = None
    ((code, pointer),) = problems(edited(drop))
    assert (code, pointer) == ("I209", "/tasks/0/goal")


def test_the_reader_rejects_a_fail_that_is_not_last_or_not_in_a_branch():
    def move_fail(t):
        t["steps"].append(t["steps"][0]["else"][0])
    assert problems(edited(move_fail)) == [("I214", "/tasks/0/steps/1")]


def test_the_reader_checks_the_form_of_if_steps():
    def break_if(t):
        del t["steps"][0]["goal_else"]
        t["steps"][0]["then"] = []
    assert problems(edited(break_if)) == [("I111", "/tasks/0/steps/0"), ("I120", "/tasks/0/steps/0/then")]


def test_the_reader_rejects_if_steps_in_older_schemas():
    def old(document):
        document["ir_schema_version"], document["language_version"] = "1.2", "1.3"
    document = json.loads(edited(lambda t: None))
    old(document)
    assert ("I115", "/tasks/0/steps/0/kind") in problems(json.dumps(document).encode())


def test_the_reader_applies_the_static_rules_inside_branches():
    def undeclared(t):
        t["steps"][0]["then"][0]["target"] = "nowhere"
    assert problems(edited(undeclared)) == [("I203", "/tasks/0/steps/0/then/0/target")]


# -------- inspect --------

def test_inspect_shows_branches_and_fail(tmp_path):
    path = tmp_path / "a.rob"
    path.write_text(SOURCE, encoding="utf-8")
    code, out, _ = cli("inspect", str(path))
    assert code == 0
    assert "RoboLang 1.4 | IR schema 1.3" in out
    assert "1. if region_clear cell" in out
    assert "goal: robot_at r dst" in out
    assert '1.1.1. fail "later"' in out
    code, out, _ = cli("inspect", "--json", str(path))
    assert json.loads(out)["tasks"][0]["steps"][0]["kind"] == "if"


# -------- validate --world on a branching task (path checks: see test_validate_world.py) --------

WORLD = "bench/worlds/warehouse.json"
BRANCHING = (REFERENCE / "b_aisle_or_report.rob").read_text(encoding="utf-8")
BIND = ["--bind", "r=amr1", "c=crate1", "src=dock", "dst=handoff", "aisle=aisle"]


def test_validate_world_checks_every_path_of_a_branching_task_and_says_so(tmp_path):
    path = tmp_path / "b.rob"
    path.write_text(BRANCHING, encoding="utf-8")
    code, out, err = cli("validate", str(path), "--world", WORLD, *BIND)
    assert code == 0 and err == ""
    assert "every path of its branches checked" in out and "no certain failure found" in out


def test_validate_world_still_checks_a_straight_1_4_task(tmp_path):
    path = tmp_path / "s.rob"
    path.write_text(task("move r to dst;").replace("task t(" + PARAMS + ")", "task t(robot r, location dst)"),
                    encoding="utf-8")
    code, out, _ = cli("validate", str(path), "--world", WORLD, "--bind", "r=amr1", "dst=handoff")
    assert code == 0 and "no certain failure found" in out


def test_a_branching_answer_is_valid_for_the_benchmark_and_judged_per_path():
    """The benchmark accepts a 1.4 `if` answer and scores it on every path (N4 PR 3; B005 is gone)."""
    from bench import lib

    ref = lib.BRANCH_REFERENCE_DIR
    answer = {"decision": "program", "category": "none", "reason": "",
              "rob": (ref / "b_route_by_cell.rob").read_text(encoding="utf-8"),
              **json.loads((ref / "b_route_by_cell.json").read_text(encoding="utf-8"))}
    task_def = lib.load_set("branch")["b_route_by_cell"]
    checked = lib.check_answer(task_def, answer)
    assert checked.valid and checked.language_version == "1.4"
    judgement = lib.judge(task_def, checked, answer["bindings"])
    assert judgement.goal_correct and judgement.plan_succeeds and len(judgement.paths) == 2


@pytest.mark.parametrize("reason, ok", [("x" * 200, True), ("x" * 201, False), ("two\\nlines", False),
                                        ("tab\\there", False), ("box1 missing", True)])
def test_a_fail_reason_fits_core_declared_text(reason, ok):
    """At most 200 characters and no control characters (E314), as Core 0.2 `declared` text (decision 0028 Q1)."""
    source = ("robolang 1.4;\n\ntask t(robot r, location a) {\n    plan {\n        if robot_ready r {\n"
              "            goal { robot_at r a; }\n            move r to a;\n        } else {\n"
              f'            fail "{reason}";\n        }}\n    }}\n}}\n').replace("}}", "}")
    if ok:
        check(source)
    else:
        assert codes(source) == ["E314"]


def test_the_ir_reader_restates_the_fail_reason_rule():
    from robolang.ir_reader import IRError, read_ir

    source = ("robolang 1.4;\n\ntask t(robot r, location a) {\n    plan {\n        if robot_ready r {\n"
              "            goal { robot_at r a; }\n            move r to a;\n        } else {\n"
              '            fail "ok";\n        }\n    }\n}\n')
    document = json.loads(to_json(compile_source(source, "t.rob").ir))
    document["tasks"][0]["steps"][0]["else"][0]["reason"] = "x" * 201
    with pytest.raises(IRError) as error:
        read_ir(json.dumps(document).encode("utf-8"), "t.json")
    assert [d.code for d in error.value.diagnostics] == ["I215"]
