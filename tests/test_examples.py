"""Every program in examples/ validates and runs to SUCCEEDED against examples/world.json."""

from __future__ import annotations
import io
import json
from pathlib import Path

import pytest

from robolang.cli import main

REPO_ROOT = Path(__file__).resolve().parent.parent
EXAMPLES = sorted((REPO_ROOT / "examples").glob("*.rob"))
WORLD = REPO_ROOT / "examples" / "world.json"


def test_examples_exist():
    assert {p.name for p in EXAMPLES} >= {"pick_and_place.rob", "inspect_object.rob", "handover.rob"}


@pytest.mark.parametrize("path", EXAMPLES, ids=lambda p: p.name)
def test_example_simulates_to_succeeded(path):
    out, err = io.StringIO(), io.StringIO()
    status = main(["simulate", str(path), "--world", str(WORLD)], out=out, err=err)
    assert status == 0, err.getvalue()
    record = json.loads(out.getvalue())
    assert record["state"] == "SUCCEEDED"
    assert all(step["status"] == "succeeded" for step in record["steps"])
