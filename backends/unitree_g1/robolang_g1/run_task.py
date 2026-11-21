"""Start a stand-in for a site and run one task on it (the helper the phase 2 checks use).

    python -m robolang_g1.run_task FILE.rob --task NAME --site SITE.json --bind P=ID ... --out record.json
        [--stance free|supported] [--deadline-s S] [--stop-file F] [--scene SCENE.json] [--run-dir DIR]

It starts the stand-in on `--scene` (default: the site itself; a scene may differ from the site the executor is told,
which is how an absent object or a covered marker is made), runs `robolang_g1.cli` against it as a separate process,
stops the stand-in and prints where the record is. The stand-in writes the oracle file (`DIR/oracle.json`) for the
checks; the executor never reads it. Exit status is the CLI's (4: every step ran, goal not evaluated; 1: failed).
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from . import guard
from .harness import OracleTail, StandIn


class Session:
    """One stand-in with its oracle tail, for one scenario. `with Session(...) as s:` starts and always stops it."""

    def __init__(self, scene: str | Path, run_dir: Path, stance: str = "free", test_hooks: bool = True):
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.standin = StandIn(str(scene), self.run_dir, stance, ["--test-hooks"] if test_hooks else [])
        self.tail: OracleTail | None = None
        self.sim = None  # the harness's own robolang_sim client (status, and the test push)
        self.final: dict | None = None
        self.started: dict | None = None
        self.ready: dict | None = None

    def __enter__(self) -> "Session":
        try:
            self.ready = self.standin.wait_ready()
            self.sim, _ = guard.require_simulation(timeout=30.0)
            self.tail = OracleTail(self.standin.oracle)
            self.tail.start()
            self.tail.wait(lambda s: s["sim_time"] > 0, 30, "the oracle")
            self.started = self.sim.Status()[1]
        except BaseException:  # never leave a stand-in running: a second one on the domain spoils every later run
            self.stop()
            raise
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    def stop(self) -> dict | None:
        if self.tail is not None:
            self.tail.stop()
        if self.final is None:
            self.final = self.standin.stop()
        return self.final

    def commands(self) -> dict:
        """The per-channel command counts the stand-in has received so far (from its oracle)."""
        return self.tail.latest()["commands"]

    def run(self, rob: str | Path, task: str, site: str | Path, bind: dict[str, str], out: Path, *,
            stop_file: Path | None = None, deadline_s: float | None = None, timeout: float = 900.0,
            watch=None) -> tuple[int, dict | None, str]:
        """Run `robolang_g1.cli` against this stand-in. `watch(proc)`, if given, is called every 50 ms while it runs (a
        check uses it to cancel, revoke or push at the moment it wants). Returns (exit status, record or None, stderr)."""
        cmd = [sys.executable, "-m", "robolang_g1.cli", str(rob), "--task", task, "--site", str(site), "--out", str(out)]
        for name, value in bind.items():
            cmd += ["--bind", f"{name}={value}"]
        if stop_file is not None:
            cmd += ["--stop-file", str(stop_file)]
        if deadline_s is not None:
            cmd += ["--deadline-s", str(deadline_s)]
        log = Path(out).with_suffix(".log")
        with log.open("w") as fh:
            proc = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, env=dict(os.environ))
            t_end = time.monotonic() + timeout
            while proc.poll() is None:
                if time.monotonic() > t_end:
                    proc.kill()
                    break
                if watch is not None:
                    watch(proc)
                time.sleep(0.05)
        record = json.loads(Path(out).read_text()) if Path(out).exists() else None
        return proc.wait(), record, log.read_text()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("source")
    ap.add_argument("--task", required=True)
    ap.add_argument("--site", required=True)
    ap.add_argument("--scene", help="the scene the stand-in builds (default: --site)")
    ap.add_argument("--bind", action="append", default=[], metavar="NAME=ID")
    ap.add_argument("--out", required=True)
    ap.add_argument("--stance", default="free", choices=["free", "supported"])
    ap.add_argument("--deadline-s", type=float)
    ap.add_argument("--stop-file")
    ap.add_argument("--run-dir")
    args = ap.parse_args(argv)
    run_dir = Path(args.run_dir) if args.run_dir else Path(tempfile.mkdtemp(prefix="g1_run_"))
    bind = dict(item.split("=", 1) for item in args.bind)
    with Session(args.scene or args.site, run_dir, args.stance) as session:
        status, record, log = session.run(args.source, args.task, args.site, bind, Path(args.out),
                                          stop_file=Path(args.stop_file) if args.stop_file else None,
                                          deadline_s=args.deadline_s)
    print(log.strip(), file=sys.stderr)
    final = session.final or {}
    print(json.dumps({"exit": status, "state": record and record["state"], "failure": record and record["failure"],
                      "stand_in": {k: final.get(k) for k in ("realtime_ratio", "worst_second_ratio", "stalls",
                                                              "step_compute_ms")},
                      "record": args.out, "run_dir": str(run_dir)}))
    return status


if __name__ == "__main__":
    sys.exit(main())
