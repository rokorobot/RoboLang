"""Entry point for the optional simulation-only backend; ROS imports happen after source validation."""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from robolang import ir
from robolang.diagnostics import RoboLangError

from .executor import BackendFailure, Stop, execute
from .site import read_site


def _finish(code: int) -> None:
    # MoveItPy 2.15.2 can segfault while its native executor is destroyed at Python
    # shutdown. The CLI is a dedicated process: flush its record and exit directly.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


def main() -> None:
    ap = argparse.ArgumentParser(description="Run a RoboLang task against the declared Gazebo model only")
    ap.add_argument("source", help="validated .rob source")
    ap.add_argument("--task", required=True)
    ap.add_argument("--site", required=True)
    ap.add_argument("--bind", action="append", default=[], metavar="NAME=ID")
    ap.add_argument("--out", required=True)
    ap.add_argument("--deadline-s", type=float, metavar="S",
                    help="stop the run S seconds of simulation time after the guard passes (S512, decision 0038 P7)")
    ap.add_argument("--stop-file", metavar="PATH",
                    help="stop the run when PATH holds 'cancel' (S511) or 'revoke' (S513), even during a motion")
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
        after = None if args.deadline_s is None else int(args.deadline_s * 1000)
        if after is not None and after <= 0:
            raise BackendFailure("S507", "--deadline-s must be greater than zero")
        stop = Stop(stop_file=args.stop_file, after_ms=after) if after is not None or args.stop_file else None
        ops = ROSOperations(site, stop)
        record = execute(task, bindings, site, ops, stop)
        Path(args.out).write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        if record["state"] == "SUCCEEDED":
            print("goal verified from observations", file=sys.stderr)
            _finish(0)
        print(f"{record['failure']['code']}: {record['failure']['message']}", file=sys.stderr)
        # S302: the observations could not decide the goal. Never reported as completion.
        _finish(3 if record["failure"]["code"] == "S302" else 1)
    except RoboLangError as exc:
        for diagnostic in exc.diagnostics:
            print(diagnostic.format(), file=sys.stderr)
        _finish(1)
    except OSError as exc:
        print(f"S507: file error: {exc}", file=sys.stderr)
        _finish(1)
    except BackendFailure as exc:
        print(f"{exc.code}: {exc.message}", file=sys.stderr)
        _finish(1)


if __name__ == "__main__":
    main()
