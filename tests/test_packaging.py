"""
Guards on the package metadata (pyproject.toml).

These read the file; they do not build or install anything. Building and
installing is covered by tools/smoke_install.py, which CI runs separately.
"""

from __future__ import annotations
import tomllib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
PYPROJECT = tomllib.loads((REPO_ROOT / "pyproject.toml").read_text(encoding="utf-8"))
PROJECT = PYPROJECT["project"]


def test_distribution_version_is_the_approved_pre_release():
    assert PROJECT["version"] == "0.1.0a1"


def test_distribution_version_is_not_the_language_or_schema_version():
    from robolang import ir

    assert (ir.LANGUAGE_VERSION, ir.IR_SCHEMA_VERSION) == ("1.1", "1.0")
    assert PROJECT["version"] not in (ir.LANGUAGE_VERSION, ir.IR_SCHEMA_VERSION)


def test_default_installation_has_no_dependencies():
    assert PROJECT["dependencies"] == []
    for extra, requirements in PROJECT.get("optional-dependencies", {}).items():
        for requirement in requirements:
            assert not requirement.startswith(("rclpy", "control_msgs", "trajectory_msgs", "std_msgs")), extra


def test_the_only_installed_command_is_the_read_only_cli():
    """No console script may lead to the v1 interpreter or the ROS 2 adapter."""
    assert PROJECT["scripts"] == {"robolang": "robolang.cli:run"}
    assert "gui-scripts" not in PROJECT
    assert "entry-points" not in PROJECT


def test_setup_py_adds_no_metadata_of_its_own():
    source = (REPO_ROOT / "setup.py").read_text(encoding="utf-8")

    assert "entry_points" not in source
    assert "install_requires" not in source
    assert "console_scripts" not in source


LEGACY_MODULES = ("robolang_runtime", "robolang_adapter", "robolang_adapter_ros2", "robolang_ros2_main")


def test_only_the_read_only_package_is_installed():
    """The v1 prototype modules must not be part of the installed distribution."""
    setuptools = PYPROJECT["tool"]["setuptools"]

    assert setuptools["packages"] == ["robolang", "robolang.sim", "robolang.core"]
    assert "py-modules" not in setuptools
    assert not ({"find", "packages.find"} & set(setuptools))


def test_legacy_sources_are_still_in_the_repository():
    for module in LEGACY_MODULES:
        assert (REPO_ROOT / "src" / f"{module}.py").is_file(), module


def test_read_only_package_does_not_depend_on_the_legacy_modules():
    package = REPO_ROOT / "src" / "robolang"
    for path in package.rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        for module in LEGACY_MODULES:
            assert f"import {module}" not in text and f"from {module}" not in text, (path.name, module)
