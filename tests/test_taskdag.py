"""tools/taskdag.py: reading Claude Code task lists into a DAG and noticing changes."""

from __future__ import annotations
import importlib.util
import json
from pathlib import Path

TOOL = Path(__file__).resolve().parent.parent / "tools" / "taskdag.py"
spec = importlib.util.spec_from_file_location("taskdag", TOOL)
taskdag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(taskdag)


def write(d: Path, **task) -> None:
    d.mkdir(parents=True, exist_ok=True)
    (d / f"{task['id']}.json").write_text(json.dumps(task))


def test_scan_reads_lists_merges_edges_and_drops_deleted(tmp_path):
    lst = tmp_path / "session-a"
    write(lst, id="1", subject="Plan", status="completed", blocks=["2"], blockedBy=[])
    write(lst, id="2", subject="Build", status="in_progress", blockedBy=[], owner="w1")
    write(lst, id="3", subject="Ship", status="pending", blockedBy=["2", "9"])
    write(lst, id="4", subject="Gone", status="deleted", blockedBy=[])
    (lst / "partial.json").write_text("{not json")  # a file mid-write is skipped, not fatal
    tasks = {t["id"]: t for t in taskdag.scan(tmp_path)["session-a"]["tasks"]}
    assert set(tasks) == {"1", "2", "3"}
    assert tasks["2"]["blockedBy"] == ["1"]  # edge recorded only as 1.blocks
    assert tasks["3"]["blockedBy"] == ["2"]  # unknown blocker 9 dropped
    assert tasks["2"]["owner"] == "w1"


def test_scan_accepts_a_single_plan_file(tmp_path):
    plan = tmp_path / "plan.json"
    plan.write_text(json.dumps({"tasks": [{"id": 1, "subject": "A"}, {"id": 2, "subject": "B", "blockedBy": [1]}]}))
    tasks = taskdag.scan(plan)["plan"]["tasks"]
    assert [t["id"] for t in tasks] == ["1", "2"] and tasks[1]["blockedBy"] == ["1"]
    assert tasks[0]["status"] == "pending"


def test_watcher_bumps_version_only_on_change(tmp_path):
    lst = tmp_path / "s"
    write(lst, id="1", subject="A", status="pending", blockedBy=[])
    w = taskdag.Watcher.__new__(taskdag.Watcher)  # no polling thread
    w.root, w.cond, w.version, w.payload, w._last = tmp_path, taskdag.threading.Condition(), 0, b"", None
    w.refresh()
    w.refresh()
    assert w.version == 1
    write(lst, id="1", subject="A", status="completed", blockedBy=[])
    w.refresh()
    assert w.version == 2 and b'"completed"' in w.payload
