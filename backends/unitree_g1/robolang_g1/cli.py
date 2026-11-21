"""Run one validated task on the simulated G1, against a stand-in that is already running (the guard decides).

    python -m robolang_g1.cli FILE.rob --task NAME --site SITE.json --bind P=ID ... --out record.json
        [--stop-file F] [--deadline-s S] [--head-dir DIR]

`--bind P=ID` binds the task's parameter P to the entity ID `ID`; the site's `bindings` say which robot, object and
location of the site each ID is. `--stop-file F` stops the run when F holds `cancel` (S511) or `revoke` (S513), even
during a motion; `--deadline-s S` stops it S simulation seconds after the guard passes (S512). The head camera's
frames are
found through the stand-in's identity API; `--head-dir` names them when it does not say.

Exit status: 4 when every step ran (the goal is **not evaluated** in phase 2: this is not a success), 1 when the run
failed or was refused (the record has the code), 2 for a usage or source error. The record is written to `--out` in
every
case where the task was understood.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from robolang import ir
from robolang.diagnostics import RoboLangError

from . import site as site_mod
from .executor import BackendFailure, Stop, execute

EXIT_STEPS_COMPLETED, EXIT_FAILED, EXIT_USAGE = 4, 1, 2


def _json_default(obj):
    if hasattr(obj, "tolist"):
        return obj.tolist()
    return str(obj)


def refusal_record(task_name: str, bindings: dict, failure: BackendFailure) -> dict:
    """The record of a run refused before the guard (S514: a mission is not a task)."""
    return {
        "run_record_schema": "robolang.g1.run",
        "run_record_schema_version": "0.1",
        "mode": "simulation",
        "backend": {
            "kind": "unitree-g1-mujoco",
            "versions": None,
            "grasp_model": "weld_after_hand_close",
            "stance_model": None,
        },
        "task": {"name": task_name},
        "bindings": bindings,
        "state": "FAILED",
        "steps": [],
        "stop": None,
        "failure": {"code": failure.code, "message": failure.message, "step": None},
        "goal_evaluated": False,
        "core_outcome": None,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("source", help="validated .rob source")
    ap.add_argument("--task", required=True)
    ap.add_argument("--site", required=True)
    ap.add_argument("--bind", action="append", default=[], metavar="NAME=ID")
    ap.add_argument("--out", required=True)
    ap.add_argument(
        "--deadline-s",
        type=float,
        metavar="S",
        help="stop the run S seconds of simulation time after the guard passes (S512, decision 0038 P7)",
    )
    ap.add_argument(
        "--stop-file",
        metavar="PATH",
        help="stop the run when PATH holds 'cancel' (S511) or 'revoke' (S513), even during a motion",
    )
    ap.add_argument("--head-dir", help="the head camera's frame directory, if the stand-in does not report it")
    args = ap.parse_args(argv)
    try:
        compiled = ir.compile_source(Path(args.source).read_text(encoding="utf-8"), args.source).ir
        bindings: dict[str, str] = {}
        for item in args.bind:
            name, _, value = item.partition("=")
            if not name or not value or name in bindings:
                raise BackendFailure("S507", "--bind must be NAME=ID, with unique names")
            bindings[name] = value
        record = None
        if compiled.mission(args.task) is not None:
            record = refusal_record(
                args.task,
                bindings,
                BackendFailure(
                    "S514",
                    f"{args.task!r} is a mission: a mission runs on the planner's side and this backend is sent "
                    "tasks only, one task run at a time (RoboLang 2.0, decision 0041 M8)",
                ),
            )
        else:
            task = compiled.task(args.task)
            if task is None:
                raise BackendFailure("S507", f"unknown task {args.task!r}")
            if args.deadline_s is not None and args.deadline_s <= 0:
                raise BackendFailure("S507", "--deadline-s must be greater than zero")
            try:
                site = site_mod.load_site(args.site)
            except (OSError, ValueError) as exc:
                raise BackendFailure("S507", f"cannot read the site: {exc}") from exc
            after = None if args.deadline_s is None else int(args.deadline_s * 1000)
            stop = Stop(stop_file=args.stop_file, after_ms=after) if after is not None or args.stop_file else None
            from .ops import G1Operations  # DDS and the SDK are imported only for a valid task and site

            ops = G1Operations(site, stop, args.head_dir)
            record = execute(task, bindings, site, ops, stop)
    except RoboLangError as exc:
        for diagnostic in exc.diagnostics:
            print(diagnostic.format(), file=sys.stderr)
        return EXIT_USAGE
    except OSError as exc:
        print(f"S507: file error: {exc}", file=sys.stderr)
        return EXIT_USAGE
    except BackendFailure as exc:
        print(f"{exc.code}: {exc.message}", file=sys.stderr)
        return EXIT_USAGE
    Path(args.out).write_text(json.dumps(record, indent=2, default=_json_default) + "\n", encoding="utf-8")
    if record["state"] == "STEPS_COMPLETED":
        print("every step ran; the goal was not evaluated (phase 3 of this backend): not a success", file=sys.stderr)
        return EXIT_STEPS_COMPLETED
    failure = record["failure"]
    print(f"{failure['code']}: {failure['message']}", file=sys.stderr)
    confirmation = (record.get("stop") or {}).get("confirmation")
    if confirmation is not None and not confirmation.get("confirmed") and failure["code"] != "S516":
        print("warning: the robot was not confirmed standing and still after the stop; check the cell", file=sys.stderr)
    return EXIT_FAILED


if __name__ == "__main__":
    sys.exit(main())
