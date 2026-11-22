"""The executor never reads MuJoCo state, the oracle or SportModeState, and never takes the robot's balance away.

Hard rule of the phase 2 brief: only the *checks* (phase2.py, harness.py) may read the oracle. This test fails if an
executor module refers to it (comments and docstrings, which say that it is not used, do not count), or to the stand-in,
or to what is forbidden on a biped (decision 0046 U9: Damp, zero torque, SwitchToUserCtrl, rt/lowcmd).
"""

import ast
import io
import re
import tokenize
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1] / "robolang_g1"
EXECUTOR_MODULES = [
    "cli.py",
    "executor.py",
    "ops.py",
    "perception.py",
    "kinematics.py",
    "robot.py",
    "site.py",
    "guard.py",
    "observe.py",
    "response.py",
    "heading.py",
]
FORBIDDEN = re.compile(
    r"oracle|SportModeState|sportmodestate|\bstandin\b|\.sim\.standin|rt/lowcmd|LowCmd_\b.*lowcmd|"
    r"\bDamp\s*\(|SwitchToUserCtrl\s*\(|ZeroTorque\s*\(|\bSetFsmId\s*\(",
    re.I,
)


def code_only(path: Path) -> str:
    """The module's source without comments and docstrings (strings that are the first statement of a body)."""
    source = path.read_text()
    tree = ast.parse(source)
    doc_lines: set[int] = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.ClassDef, ast.AsyncFunctionDef)):
            first = node.body[0] if node.body else None
            if (
                isinstance(first, ast.Expr)
                and isinstance(first.value, ast.Constant)
                and isinstance(first.value.value, str)
            ):
                doc_lines.update(range(first.lineno, first.end_lineno + 1))
    kept = []
    for tok in tokenize.generate_tokens(io.StringIO(source).readline):
        if tok.type == tokenize.COMMENT or tok.start[0] in doc_lines:
            continue
        kept.append(tok.string)
    return " ".join(kept)


@pytest.mark.parametrize("module", EXECUTOR_MODULES)
def test_executor_module_never_refers_to_the_oracle_or_what_a_biped_must_not_do(module):
    found = FORBIDDEN.findall(code_only(PKG / module))
    assert not found, f"{module} refers to {found}"


def test_executor_modules_do_not_import_the_simulator_internals():
    for module in EXECUTOR_MODULES:
        tree = ast.parse((PKG / module).read_text())
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.ImportFrom):
                names = [(node.module or "")] + [a.name for a in node.names]
            elif isinstance(node, ast.Import):
                names = [a.name for a in node.names]
            for name in names:
                assert "standin" not in name and "stance" not in name, f"{module} imports {name}"
                if "sim" in name.split(".") and node.level == 1 or name == "sim":
                    # `from .sim import scene` is allowed in kinematics (the robot's description); the stand-in is not
                    assert module in ("kinematics.py", "perception.py"), f"{module} imports {name}"


def test_the_check_catches_a_reference(tmp_path):
    bad = tmp_path / "x.py"
    bad.write_text(
        '"""docstring about the oracle is fine."""\n# the oracle in a comment is fine\n'
        'x = json.load(open("oracle.json"))\n'
    )
    assert FORBIDDEN.findall(code_only(bad)) == ["oracle"]
