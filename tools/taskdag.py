"""
Live DAG view of the tasks your coding agents are working on.

Claude Code keeps each task list as one JSON file per task under
~/.claude/tasks/<list-id>/<task-id>.json (written by TaskCreate / TaskUpdate:
subject, status, owner, blocks, blockedBy). This serves a page that draws that
graph and redraws it the moment any task changes status.

    python tools/taskdag.py                     # watch ~/.claude/tasks, open http://127.0.0.1:8765
    python tools/taskdag.py --root DIR          # another tasks folder (or a folder of task JSON files)
    python tools/taskdag.py --root plan.json    # one JSON file: {"tasks": [...]} or [...]

Any agent can feed it: write files with the same fields (id, subject, status,
blockedBy, owner, activeForm, description). Status is pending, in_progress,
completed or deleted. Python standard library only; reads, never writes.
The page itself is tools/taskdag.html.
"""

from __future__ import annotations
import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

POLL_SECONDS = 0.5
PAGE = (Path(__file__).with_name("taskdag.html")).read_bytes()


def _load_task(path: Path) -> dict | None:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None  # mid-write or not a task; the next poll picks it up
    if not isinstance(data, dict) or "id" not in data:
        return None
    return data


def _normalize(raw: list[dict]) -> list[dict]:
    tasks = {}
    for t in raw:
        if t.get("status") == "deleted":
            continue
        tid = str(t["id"])
        tasks[tid] = {
            "id": tid,
            "subject": str(t.get("subject") or t.get("content") or tid),
            "description": str(t.get("description") or ""),
            "activeForm": str(t.get("activeForm") or ""),
            "status": str(t.get("status") or "pending"),
            "owner": str(t.get("owner") or ""),
            "blockedBy": {str(b) for b in t.get("blockedBy") or []},
        }
    # Edges may be recorded on either end (blocks / blockedBy); merge both.
    for t in raw:
        for b in t.get("blocks") or []:
            if str(b) in tasks and str(t["id"]) in tasks:
                tasks[str(b)]["blockedBy"].add(str(t["id"]))
    for t in tasks.values():
        t["blockedBy"] = sorted(b for b in t["blockedBy"] if b in tasks)
    return sorted(tasks.values(), key=lambda t: (len(t["id"]), t["id"]))


def _project_label(list_id: str) -> str:
    """Claude Code names a session's list after the session; find its project folder."""
    projects = Path.home() / ".claude" / "projects"
    for hit in projects.glob(f"*/{list_id}.jsonl"):
        return hit.parent.name.replace("-", "/").lstrip("/")
    return ""


def scan(root: Path) -> dict:
    """Return {list_id: {"tasks": [...], "mtime": float, "label": str}}."""
    lists: dict[str, dict] = {}

    def add(list_id: str, files: list[Path], label: str = "") -> None:
        raw, mtime = [], 0.0
        for f in files:
            t = _load_task(f)
            if t is not None:
                raw.append(t)
                mtime = max(mtime, f.stat().st_mtime)
        if raw:
            lists[list_id] = {"tasks": _normalize(raw), "mtime": mtime, "label": label}

    if root.is_file():
        try:
            data = json.loads(root.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return lists
        tasks = data.get("tasks", []) if isinstance(data, dict) else data
        tasks = [t for t in tasks if isinstance(t, dict) and "id" in t]
        if tasks:
            lists[root.stem] = {"tasks": _normalize(tasks), "mtime": root.stat().st_mtime, "label": str(root)}
        return lists
    if not root.is_dir():
        return lists
    add(root.name, sorted(root.glob("*.json")), str(root))
    for d in sorted(p for p in root.iterdir() if p.is_dir()):
        add(d.name, sorted(d.glob("*.json")), _project_label(d.name))
    return lists


class Watcher:
    """Polls the tasks folder; bumps a version whenever anything changes."""

    def __init__(self, root: Path):
        self.root = root
        self.cond = threading.Condition()
        self.version = 0
        self.payload = b"{}"
        self._last = None
        self.refresh()
        threading.Thread(target=self._loop, daemon=True).start()

    def refresh(self) -> None:
        lists = scan(self.root)
        snapshot = json.dumps(lists, sort_keys=True)
        if snapshot != self._last:
            self._last = snapshot
            body = json.dumps({"root": str(self.root), "lists": lists, "at": time.time()})
            with self.cond:
                self.payload = body.encode()
                self.version += 1
                self.cond.notify_all()

    def _loop(self) -> None:
        while True:
            time.sleep(POLL_SECONDS)
            try:
                self.refresh()
            except OSError:
                pass  # a file vanished between listing and reading

    def wait(self, seen: int, timeout: float) -> tuple[int, bytes]:
        with self.cond:
            self.cond.wait_for(lambda: self.version != seen, timeout=timeout)
            return self.version, self.payload


def make_handler(watcher: Watcher):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):  # keep the terminal quiet
            pass

        def _send(self, body: bytes, ctype: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                self._send(PAGE, "text/html; charset=utf-8")
            elif self.path == "/state":
                self._send(watcher.payload, "application/json")
            elif self.path == "/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                seen = -1
                try:
                    while True:
                        version, payload = watcher.wait(seen, timeout=15)
                        if version == seen:
                            self.wfile.write(b": keepalive\n\n")
                        else:
                            seen = version
                            self.wfile.write(b"data: " + payload + b"\n\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return
            else:
                self.send_error(404)

    return Handler


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    default_root = Path(os.environ.get("CLAUDE_CONFIG_DIR", Path.home() / ".claude")) / "tasks"
    ap.add_argument("--root", type=Path, default=default_root,
                    help=f"tasks folder or JSON file (default {default_root})")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    args = ap.parse_args()
    watcher = Watcher(args.root.expanduser())
    server = ThreadingHTTPServer((args.host, args.port), make_handler(watcher))
    server.daemon_threads = True
    print(f"taskdag: watching {watcher.root} -> http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
