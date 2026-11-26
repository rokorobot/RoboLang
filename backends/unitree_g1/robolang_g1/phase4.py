"""Phase 4 of the G1 backend: the acceptance run (decision 0046 section 3).

    python -m robolang_g1.phase4 --out phase4.json [--run-dir DIR] [--repeats 3] [--only NAME,NAME] [--stance free]

The 14 reference tasks of the family run **unchanged**, each `--repeats` times on its scene (`scenes.py`), through
`robolang_g1.cli` as a separate process against a fresh stand-in, exactly as a user would run them. Then the scripted
failure cases run once each. The **judge** reads the oracle (MuJoCo ground truth that the stand-in writes); the executor
never does.

The bar (0046 section 3, item 3), over the runs that count:

* at least 90% of the task runs end `SUCCEEDED`, decided from observations;
* the observed verdict of every goal and precondition atom agrees with the oracle on at least 98% of the atoms the
  oracle decides;
* **no false success**: no run ends `SUCCEEDED` while the oracle says its goal is not met (this alone blocks it);
* every scripted failure case ends `FAILED` with its code, and none ends `SUCCEEDED`.

**Runs that do not count.** A run in which the simulation fell more than 5% behind real time in any second is a
harness failure (0046 U13): it is recorded, listed under `not_counted`, and the run is repeated (at most
`--retries` times). A false success in such a run still blocks acceptance.

**The run can be resumed.** Every run writes `DIR/<name>/result.json` when it ends; a run that has one is not run again.
Delete the directory of a run to repeat it. Exit status 0 only if the bar is met by the whole family.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

from . import assets, scenes
from .phase2 import PUSH_N, PUSH_S, _Push, realtime_of
from .phase3 import COMMAND_KEYS, Ctx as Ctx3, collect
from .run_task import Session

SUCCESS_MIN = 0.90
AGREEMENT_MIN = 0.98
TASK_TIMEOUT_S = 1800.0  # wall clock; the longest task (a_sort_three) walks six times between the tables


class Ctx(Ctx3):
    def __init__(self, run_dir: Path, stance: str):
        super().__init__(run_dir, stance)
        self.built: dict[str, dict] = {}

    def scene(self, name: str) -> dict:
        if name not in self.built:
            self.built[name] = scenes.write(name, self.dir / "scenes" / name)
        return self.built[name]

    def paths(self, name: str) -> tuple[Path, Path]:
        d = self.dir / "scenes" / name
        return d / "site.json", d / "scene.json"


def _run(ctx: Ctx, name: str, scene: str, *, before=None, watch=None) -> dict:
    """One run of a scene's task on a fresh stand-in. Returns what the judge needs; nothing is judged here."""
    built = ctx.scene(scene)
    site_path, world_path = ctx.paths(scene)
    out = ctx.dir / name / "record.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    started = time.time()
    box: list = [None]
    with Session(world_path, ctx.dir / name, ctx.stance) as s:
        box[0] = s
        if before is not None:
            before(s)
        status, record, log = s.run(
            built["source"], built["task"], site_path, built["bind"], out,
            watch=watch(s) if watch else None, timeout=TASK_TIMEOUT_S,
        )
        time.sleep(0.5)
        samples, commands = list(s.tail.samples), s.commands()
    return {
        "name": name,
        "scene": scene,
        "exit": status,
        "record": record,
        "log_tail": log[-600:],
        "samples": samples,
        "commands": commands,
        "realtime": realtime_of(s),
        "wall_s": round(time.time() - started, 1),
    }


def run_task(ctx: Ctx, scene: str, attempt: str) -> dict:
    """One counted-or-not run of a family task, judged against the oracle. Cached in `result.json`."""
    name = f"{scene}_{attempt}"
    cache = ctx.dir / name / "result.json"
    if cache.exists():
        return json.loads(cache.read_text())
    built = ctx.scene(scene)
    site_path, _ = ctx.paths(scene)
    r = _run(ctx, name, scene)
    rec = r["record"]
    rows0, false0 = len(ctx.rows), len(ctx.false_successes)
    got = (
        collect(ctx, name, rec, Path(built["source"]), built["task"], built["bind"], site_path, r["samples"])
        if rec
        else {"atoms": [], "goal_true_in_truth": None}
    )
    final = r["samples"][-1] if r["samples"] else {}
    result = {
        "name": name,
        "task": scene,
        "exit": r["exit"],
        "state": rec and rec["state"],
        "failure": rec and rec["failure"],
        "succeeded": bool(rec and rec["state"] == "SUCCEEDED" and r["exit"] == 0),
        "goal_true_per_oracle": got["goal_true_in_truth"],
        "false_success": len(ctx.false_successes) > false0,
        "atoms": ctx.rows[rows0:],
        "fallen_per_oracle": final.get("fallen"),
        "stance_model": ctx.stance,
        "arm_stance": built["site"]["robot"]["arm_stance"],
        "observation_motion": [(m["phase"], m["performed"]) for m in (rec or {}).get("observation_motion", [])],
        "steps": [
            {"kind": s["kind"], "status": s["status"], "sim_s": round((s["t_end_ms"] - s["t_start_ms"]) / 1000, 1)}
            for s in (rec or {}).get("steps", [])
            if "t_end_ms" in s
        ],
        "realtime": r["realtime"],
        "counts": bool(r["realtime"]["realtime_valid"]),
        "wall_s": r["wall_s"],
        "log_tail": None if rec else r["log_tail"],
    }
    cache.write_text(json.dumps(result, indent=1, default=str))
    return result


# ------------------------------------------------------------------------------------------ scripted failure cases


def _cover(s: Session) -> None:
    s.sim.CoverCamera(True)
    time.sleep(1.0)


def _push_when_walking(state: dict):
    """Push the torso sideways 1.5 s after the robot starts to walk (as phase 2's E5 does)."""

    def make(s: Session):
        pusher: list = []

        def watch(proc):
            t = s.tail.latest()
            if "t" not in state and t["velocity_command"][0] > 0.3 and t["sim_time"] > 6:
                state["t"] = t["sim_time"]
            if "t" in state and t["sim_time"] > state["t"] + 1.5 and "pushed" not in state:
                if not pusher:
                    pusher.append(_Push())
                state["pushed"] = t["sim_time"]
                state["push_reply"] = pusher[0].push(PUSH_N, PUSH_S)[0]

        return watch

    return make


def run_case(ctx: Ctx, scene: str) -> dict:
    """One scripted failure case: `FAILED` with its code, never `SUCCEEDED`, and what else the case must show."""
    name = scene
    cache = ctx.dir / name / "result.json"
    if cache.exists():
        return json.loads(cache.read_text())
    case = scenes.CASES[scene]
    state: dict = {}
    hook = case.get("hook")
    r = _run(
        ctx, name, scene,
        before=_cover if hook == "cover" else None,
        watch=_push_when_walking(state) if hook == "push" else None,
    )
    rec, samples, commands = r["record"], r["samples"], r["commands"]
    first, final = (samples[0], samples[-1]) if samples else ({}, {})
    code = (rec or {}).get("failure") and rec["failure"]["code"]
    result = {
        "name": name,
        "what": case["what"],
        "expected": case["code"],
        "exit": r["exit"],
        "state": rec and rec["state"],
        "failure": rec and rec["failure"],
        "commands": {k: commands[k]["count"] for k in COMMAND_KEYS} if commands else None,
        "attached": final.get("attached"),
        "fallen_per_oracle": final.get("fallen"),
        "realtime": r["realtime"],
        "wall_s": r["wall_s"],
        "log_tail": None if rec else r["log_tail"],
    }
    ok = bool(rec and rec["state"] == "FAILED" and code == case["code"] and r["exit"] != 0)
    box = "box"
    if scene in ("fail_object_absent", "fail_object_moved", "fail_camera_covered"):
        # nothing reached for, nothing closed, nothing welded
        quiet = all(commands[k]["count"] == 0 for k in ("arm_sdk", "hand_left", "hand_right", "attach"))
        p0, p1 = first["objects"][box]["pos"], final["objects"][box]["pos"]
        still = float(np.linalg.norm(np.array(p1) - np.array(p0))) < 0.01
        result.update(no_arm_or_hand_command=quiet, box_not_moved=still)
        ok = ok and quiet and still
    if scene == "fail_out_of_reach":
        # refused at the place, before the arm moves for it: the box is still in the hand, and nothing was detached
        held = box in (final.get("attached") or {})
        result.update(box_still_held=held, detach_commands=commands["detach"]["count"],
                      failed_step=rec and rec["failure"].get("step"))
        ok = ok and held and commands["detach"]["count"] == 0
    if scene == "fail_push_fall":
        fell = bool(final.get("fallen"))
        stop = (rec or {}).get("stop") or {}
        last = {k: v["last_sim_time"] for k, v in commands.items()}
        after = {k: bool(v is not None and fell and final.get("fell_at") is not None and v > final["fell_at"])
                 for k, v in last.items()}
        result.update(trigger=state, fell_at=final.get("fell_at"), command_after_fall=after,
                      stop_kind=stop.get("kind"))
        ok = ok and fell and not any(after.values()) and stop.get("kind") == "no_further_command"
    result["pass"] = bool(ok)
    result["succeeded"] = bool(rec and rec["state"] == "SUCCEEDED")
    cache.write_text(json.dumps(result, indent=1, default=str))
    return result


# ------------------------------------------------------------------------------------------ the judge


def judge(tasks: dict[str, list[dict]], cases: dict[str, dict], family: list[str], repeats: int) -> dict:
    """The bar of 0046 section 3, item 3, over the runs that count."""
    counted = [r for t in family for r in tasks.get(t, []) if r["counts"]]
    not_counted = [r["name"] for t in family for r in tasks.get(t, []) if not r["counts"]]
    succeeded = [r for r in counted if r["succeeded"]]
    atoms = [a for r in counted for a in r["atoms"]]
    agree = [a for a in atoms if a["verdict"] == str(a["truth"]).lower()]
    wrong = [a for a in atoms if a["verdict"] in ("true", "false") and a["verdict"] != str(a["truth"]).lower()]
    unknown = [a for a in atoms if a["verdict"] == "unknown"]
    false_successes = [r["name"] for t in family for r in tasks.get(t, []) if r["false_success"]]
    complete = all(sum(1 for r in tasks.get(t, []) if r["counts"]) >= repeats for t in family)
    rate = len(succeeded) / len(counted) if counted else 0.0
    agreement = len(agree) / len(atoms) if atoms else 0.0
    cases_ok = bool(cases) and all(c["pass"] for c in cases.values()) and len(cases) == len(scenes.CASES)
    case_succeeded = [n for n, c in cases.items() if c["succeeded"]]
    return {
        "family": family,
        "repeats": repeats,
        "runs_counted": len(counted),
        "runs_expected": len(family) * repeats,
        "complete": complete,
        "succeeded": len(succeeded),
        "success_rate": round(rate, 4),
        "success_required": SUCCESS_MIN,
        "atoms_judged": len(atoms),
        "atoms_agree": len(agree),
        "agreement": round(agreement, 4),
        "agreement_required": AGREEMENT_MIN,
        "wrong_true_or_false": wrong,
        "unknown_where_the_oracle_decides": len(unknown),
        "false_successes": false_successes,
        "failure_cases_pass": cases_ok,
        "failure_cases_succeeded": case_succeeded,
        "not_counted": not_counted,
        "pass": bool(
            complete
            and rate >= SUCCESS_MIN
            and agreement >= AGREEMENT_MIN
            and not false_successes
            and cases_ok
            and not case_succeeded
        ),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="RoboLang G1 phase 4: the acceptance run")
    ap.add_argument("--out", default="phase4.json")
    ap.add_argument("--run-dir")
    ap.add_argument("--stance", default="free", choices=["free", "supported"])
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--retries", type=int, default=2, help="extra runs of a task for runs that fell behind real time")
    ap.add_argument("--only", default="", help="development: comma list of scenes (tasks or cases) to run")
    ap.add_argument("--leave-out", default="", help="comma list of tasks that leave the family (the evidence says why)")
    args = ap.parse_args(argv)
    out = Path(args.out)
    ctx = Ctx(Path(args.run_dir) if args.run_dir else out.with_suffix(".run"), args.stance)
    ctx.dir.mkdir(parents=True, exist_ok=True)
    only = set(filter(None, args.only.split(",")))
    left_out = set(filter(None, args.leave_out.split(",")))
    family = [t for t in scenes.TASKS if t not in left_out]
    result: dict = {
        "stance_model": args.stance,
        "versions": assets.versions(),
        "arch": os.uname().machine,
        "left_out": sorted(left_out),
        "visibility": {},
        "tasks": {},
        "cases": {},
    }
    t0 = time.perf_counter()

    def write():
        result["bar"] = judge(result["tasks"], result["cases"], family, args.repeats)
        result["elapsed_s"] = round(time.perf_counter() - t0, 1)
        out.write_text(json.dumps(result, indent=1, default=str))

    for task in family:
        if only and task not in only:
            continue
        problems = scenes.check_visibility(ctx.scene(task))
        result["visibility"][task] = problems or "holds"
        if problems:  # the rule is a property of the scene: a scene that breaks it is not run
            print(f"{task}: the visibility rule does not hold: {problems}", flush=True)
            continue
        runs = result["tasks"].setdefault(task, [])
        attempt = 0
        while sum(1 for r in runs if r["counts"]) < args.repeats and attempt < args.repeats + args.retries:
            attempt += 1
            try:
                r = run_task(ctx, task, str(attempt))
            except Exception as exc:  # a harness error must not stop the report
                import traceback

                r = {"name": f"{task}_{attempt}", "task": task, "succeeded": False, "counts": False, "atoms": [],
                     "false_success": False, "state": None, "failure": None,
                     "harness_error": f"{type(exc).__name__}: {exc}", "trace": traceback.format_exc()}
            runs.append(r)
            mark = "SUCCEEDED" if r["succeeded"] else f"{r.get('state')} {(r.get('failure') or {}).get('code')}"
            print(f"{r['name']}: {mark}{'' if r['counts'] else '  (not counted)'}"
                  f"{'  FALSE SUCCESS' if r['false_success'] else ''}  ({r.get('wall_s')} s)", flush=True)
            write()
    for case in scenes.CASES:
        if only and case not in only:
            continue
        try:
            c = run_case(ctx, case)
        except Exception as exc:
            import traceback

            c = {"name": case, "pass": False, "succeeded": False,
                 "harness_error": f"{type(exc).__name__}: {exc}", "trace": traceback.format_exc()}
        result["cases"][case] = c
        print(f"{case}: {'pass' if c['pass'] else 'FAIL'}  ({c.get('state')} "
              f"{(c.get('failure') or {}).get('code')}, expected {c.get('expected')})", flush=True)
        write()
    write()
    bar = result["bar"]
    result["go"] = bool(bar["pass"] and not only)
    out.write_text(json.dumps(result, indent=1, default=str))
    print(f"go: {result['go']}  ({bar['succeeded']}/{bar['runs_counted']} succeeded, agreement {bar['agreement']}, "
          f"false successes {len(bar['false_successes'])}, failure cases {bar['failure_cases_pass']}); wrote {out}")
    return 0 if result["go"] else 1


if __name__ == "__main__":
    sys.exit(main())
