"""Entry point for the optional simulation-only backend; ROS imports happen after source validation."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from robolang import ir

from .executor import BackendFailure, execute
from .site import read_site


def main() -> None:
    ap = argparse.ArgumentParser(description="Run a RoboLang task against the declared Gazebo model only")
    ap.add_argument("source", help="validated .rob source")
    ap.add_argument("--task", required=True)
    ap.add_argument("--site", required=True)
    ap.add_argument("--bind", action="append", default=[], metavar="NAME=ID")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    try:
        source = Path(args.source).read_text(encoding="utf-8")
        compiled = ir.compile_source(source, args.source).ir
        task = compiled.task(args.task)
        if task is None:
            raise BackendFailure("S507", f"unknown task {args.task!r}")
        bindings = {}
        for item in args.bind:
            if "=" not in item:
                raise BackendFailure("S507", "--bind must be NAME=ID")
            name, value = item.split("=", 1)
            if not name or not value or name in bindings:
                raise BackendFailure("S507", "binding names and IDs must be nonempty and unique")
            bindings[name] = value
        site = read_site(args.site)
        from .ros import ROSOperations  # never import ROS for an invalid task or site
        with ROSOperations() as ops:
            record = execute(task, bindings, site, ops)
        Path(args.out).write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        if record["state"] == "FAILED":
            print(f"{record['failure']['code']}: {record['failure']['message']}", file=sys.stderr)
            sys.exit(1)
    except BackendFailure as exc:
        print(f"{exc.code}: {exc.message}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
