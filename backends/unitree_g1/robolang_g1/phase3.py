"""Phase 3 checks O1-O7 of the G1 backend: goal and precondition verdicts from observations (decision 0046 U8).

    python -m robolang_g1.phase3 --out phase3.json [--run-dir DIR] [--only O1,O2] [--stance free|supported]

Each scenario starts its own stand-in (oracle on, test hooks on). The tasks run through `robolang_g1.cli` as separate
processes, exactly as a user would; the scripted scenes of O1 drive the executor's own operations in this process
(`G1Operations.snapshot`) so that every atom can be asked of a snapshot. The **checks** read the oracle (MuJoCo ground
truth that the stand-in writes) to judge the verdicts; the executor never does. Exit status 0 only if every check
passes.

O1 atom verdicts: scripted scenes, each atom with the verdict expected and the oracle's truth. O2 preconditions:
`w_part_precheck.rob` unchanged, item at the source / elsewhere in view (S101) / camera covered (S102), the last two
with no command sent. O3 successes: three reference tasks, unchanged, twice each. O4 goal failures: no `place` step
(S301), camera covered before the goal snapshot (S302), an obstacle in the way of the observation motion (S302, the
arm does not move), a cancel during the observation motion (S511, a biped stop, the hand unchanged). O5 `@verify`.
O6 agreement with the oracle over every atom of O1-O5, and no false success. O7 Core outcomes of every `SUCCEEDED`
record.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np

from robolang import ir
from robolang.core import Context, check_meaning, verify_sim
from robolang.core.schema import _Validator, load_schema

from . import assets, executor
from . import observe as O
from . import site as site_mod
from .ops import HAND_CLOSE, G1Operations
from .phase2 import (
    BENCH,
    BIND_CH,
    BIND_PP,
    GRAB,
    SITES,
    Ctx as Ctx2,
    _Tail,
    in_region,
    station_error,
)
from .run_task import Session

BIND_PC = {"r": "robot_1", "item": "box_1", "src": "loc_a", "dst": "loc_b"}  # w_part_precheck.rob
GRAB_CH = {"r": "robot_1", "o": "crate_1", "l": "loc_dock"}  # GRAB on the carry_hold site
ROBOT_AT_M, ROBOT_AT_RAD = O.ROBOT_AT_M, O.ROBOT_AT_RAD
AGREEMENT_MIN = 0.98  # O6
NAMES = {"r": "g1", "o": "box", "a": "bench", "b": "shelf", "c": "pad"}  # the scripted scenes' parameters (O1)
ELSEWHERE = (2.90, -0.10, 0.79)  # the box at the far edge of the bench table, 6 cm outside the bench region (world)
IN_PLACE = (2.73, -0.10, 0.79)  # where the box starts, inside the bench region
COMMAND_KEYS = ("sport", "arm_sdk", "hand_left", "hand_right", "attach", "detach")

PICK_NO_PLACE = """robolang 1.3;
task pick_no_place(robot r, object box, location src, location dst) {
    goal { object_at box dst; }
    plan { move r to src; grasp r box; move r to dst; }
}
"""
GO = """robolang 1.3;
task go(robot r, location a) {
%s    goal { robot_at r a; }
    plan { move r to a; }
}
"""


# ------------------------------------------------------------------------------------------------ oracle truth


def truth_of(condition, names: dict[str, str], site: dict, sample: dict) -> bool:
    """What is actually so at the oracle's `sample` (MuJoCo ground truth), for any condition (Kleene is not needed: the
    oracle always decides). An object in a hand is not at a place; `holding` is the stand-in's weld."""
    if isinstance(condition, ir.And):
        return all(truth_of(c, names, site, sample) for c in condition.conditions)
    if isinstance(condition, ir.Or):
        return any(truth_of(c, names, site, sample) for c in condition.conditions)
    if isinstance(condition, ir.Not):
        return not truth_of(condition.condition, names, site, sample)
    if isinstance(condition, ir.ObjectAt):
        obj = names[condition.obj]
        if obj in sample["attached"]:
            return False
        return in_region(sample["markers"][f"marker_{obj}"]["pos"], site, names[condition.location])
    if isinstance(condition, ir.Holding):
        return names[condition.obj] in sample["attached"]
    if isinstance(condition, ir.RobotAt):
        station = site_mod.station(site, site_mod.location(site, names[condition.location])["station"])
        err_m, err_rad = station_error(sample, station)
        return err_m <= ROBOT_AT_M and err_rad <= ROBOT_AT_RAD
    if isinstance(condition, ir.RobotReady):
        return bool(not sample["fallen"] and sample["stance"]["standing"])
    raise ValueError(condition.KIND)


class Ctx(Ctx2):
    """Phase 2's context (run directory, variants, sessions) plus the part_precheck site and what O6 and O7 collect."""

    def __init__(self, run_dir: Path, stance: str):
        super().__init__(run_dir, stance)
        self.sites["part_precheck"] = site_mod.load_site(SITES / "part_precheck.json")
        self.rows: list[dict] = []  # every atom's verdict next to the oracle's truth (O6)
        self.false_successes: list[dict] = []
        self.successes: list[dict] = []  # {"run", "record"}: every SUCCEEDED record (O7)
        self.records: dict[str, dict] = {}
        self.robot_at: list[dict] = []  # every robot_at verdict with its measured offset (the margins)

    def part_precheck_scene(self, item_at=None, start=None) -> Path:
        """The part_precheck site as a scene, with the item and the robot's start moved if asked. The executor is given
        the site itself."""
        raw = json.loads((SITES / "part_precheck.json").read_text())
        if item_at:
            raw["objects"][0]["pos"] = list(item_at)
        if start:
            raw["robot"]["start"] = list(start)
        p = self.dir / f"scene_{self.n}.json"
        self.n += 1
        p.write_text(json.dumps(raw))
        return p

    def site_file(self, name: str, edit) -> Path:
        """A site (what the executor is given) made from a shipped one by `edit(dict)`."""
        raw = json.loads((SITES / f"{name}.json").read_text())
        edit(raw)
        p = self.dir / f"site_{self.n}.json"
        self.n += 1
        p.write_text(json.dumps(raw))
        return p


def run3(ctx: Ctx, name: str, scene, site_path, source: Path, task: str, bind: dict, *, before=None, watch=None, **kw):
    """One scenario: a fresh stand-in on `scene`, the CLI on the executor's site. `before(session)` runs after the
    stand-in is up and before the CLI starts; `watch(session)` returns the callback `Session.run` polls while the CLI
    runs.
    Returns (exit status, record, log, oracle samples, command counts)."""
    out = ctx.dir / name / "record.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    with ctx.session(name, scene) as s:
        if before is not None:
            before(s)
        status, record, log = s.run(source, task, site_path, bind, out, watch=watch(s) if watch else None, **kw)
        samples, commands = list(s.tail.samples), s.commands()
    ctx.done(name, s, record, {"exit": status})
    if record is not None:
        ctx.records[name] = record
    return status, record, log, samples, commands


def collect(
    ctx: Ctx,
    name: str,
    rec: dict,
    source: Path,
    task: str,
    bind: dict,
    site_path,
    samples: list,
    expect_unknown: bool = False,
) -> dict:
    """Compare every precondition and goal atom of a record with the oracle at the time of its snapshot (O6); note the
    robot_at offsets (the margins); list a `SUCCEEDED` record for O7, and a false success if the goal is not true."""
    site = site_mod.load_site(site_path)
    compiled = ir.compile_source(Path(source).read_text(), str(source)).ir
    t = compiled.task(task)
    names = executor.resolve(t, bind, site)
    tail = _Tail(samples)
    out = {"atoms": [], "goal_true_in_truth": None}
    for phase, conditions in (("preconditions", t.preconditions), ("goal", t.goal or ())):
        entries = rec.get(phase) or []
        snap = next((s for s in rec.get("snapshots", []) if s["purpose"] == phase), None)
        if snap is None or all(e["result"] == "not_evaluated" for e in entries):
            continue
        sample = tail.nearest(snap["t_ms"] / 1000)
        atoms = [a for c in conditions for a in O.atoms_of(c)]
        reports = [r for e in entries for r in e["atoms"]]
        for atom, report in zip(atoms, reports):
            truth = truth_of(atom, names, site, sample)
            row = {
                "run": name,
                "phase": phase,
                "atom": report["atom"],
                "verdict": report["result"],
                "truth": truth,
                "expected_unknown": expect_unknown,
                "t_ms": snap["t_ms"],
            }
            ctx.rows.append(row)
            out["atoms"].append(row)
            if atom.KIND == "robot_at":
                ctx.robot_at.append(
                    {
                        k: report.get(k)
                        for k in (
                            "station",
                            "offset_m",
                            "offset_rad",
                            "tolerance_m",
                            "tolerance_rad",
                            "confirmed_at_station",
                            "why",
                        )
                    }
                    | {"run": name, "phase": phase, "verdict": report["result"], "truth": truth}
                )
        if phase == "goal":
            out["goal_true_in_truth"] = all(truth_of(c, names, site, sample) for c in conditions)
    if rec.get("state") == "SUCCEEDED":
        ctx.successes.append({"run": name, "record": rec})
        if not out["goal_true_in_truth"]:
            ctx.false_successes.append({"run": name, "atoms": out["atoms"]})
    return out


def no_commands(commands: dict) -> bool:
    return all(commands[k]["count"] == 0 for k in COMMAND_KEYS)


# --------------------------------------------------------------------------------------------------- O1: atoms


class Probe:
    """The executor's own operations against a running stand-in, so that atoms can be asked of snapshots (O1)."""

    def __init__(self, ctx: Ctx, session: Session, site_path):
        self.ctx, self.session = ctx, session
        self.site = site_mod.load_site(site_path)
        self.ops = G1Operations(self.site)
        self.ops.guard(self.site["name"])
        self.ops.station = "bench"
        self.policy = O.Policy.of(ir.compile_source(GO % "", "go.rob").ir.task("go"))
        self.tail = lambda: _Tail(list(session.tail.samples))

    def ask(self, scene: str, label: str, cases: list, show=(), ready: bool = False) -> dict:
        """One snapshot, and for each case (name, condition, expected verdict) the verdict, the oracle's truth at the
        snapshot's time, and the numbers behind the verdict."""
        snap, note = self.ops.snapshot(show=frozenset(show), ready=ready)
        sample = self.tail().nearest(snap.t_ms / 1000)
        rows = []
        for name, condition, expected in cases:
            v = (
                O.atom(condition, NAMES, self.site, snap, self.policy)
                if not isinstance(condition, (ir.And, ir.Or, ir.Not))
                else None
            )
            result = O.evaluate(condition, NAMES, self.site, snap, self.policy)
            truth = truth_of(condition, NAMES, self.site, sample)
            row = {
                "scene": scene,
                "step": label,
                "atom": name,
                "expected": expected,
                "verdict": result,
                "truth": truth,
                "detail": v.detail if v else None,
            }
            rows.append(row)
            self.ctx.rows.append(
                {
                    "run": f"{scene}/{label}",
                    "phase": "scripted",
                    "atom": name,
                    "verdict": result,
                    "truth": truth,
                    "expected_unknown": expected == "unknown",
                    "t_ms": snap.t_ms,
                }
            )
            if isinstance(condition, ir.RobotAt):
                self.ctx.robot_at.append(
                    {
                        "run": f"{scene}/{label}",
                        "phase": "scripted",
                        "verdict": result,
                        "truth": truth,
                        **{
                            k: (v.detail or {}).get(k)
                            for k in (
                                "station",
                                "offset_m",
                                "offset_rad",
                                "tolerance_m",
                                "tolerance_rad",
                                "confirmed_at_station",
                                "why",
                            )
                        },
                    }
                )
        return {
            "label": label,
            "t_ms": snap.t_ms,
            "rows": rows,
            "motion": note.get("observation_motion"),
            "markers": {
                k: {"confidence": v.confidence, "age_ms": snap.t_ms - v.stamp_ms, "range_m": round(v.range_m, 2)}
                for k, v in sorted(snap.markers.items())
            },
        }

    def close(self) -> None:
        self.ops.robot.stop_arm_sdk()


def _case(kind, *args):
    return {"object_at": ir.ObjectAt, "robot_at": ir.RobotAt, "holding": ir.Holding, "robot_ready": ir.RobotReady}[
        kind
    ](*args)


A_O_AT_A = ("object_at box at bench", _case("object_at", "o", "a"))
A_O_AT_B = ("object_at box at shelf", _case("object_at", "o", "b"))
A_HOLD = ("holding g1 box", _case("holding", "r", "o"))
A_R_AT_A = ("robot_at g1 bench", _case("robot_at", "r", "a"))
A_R_AT_B = ("robot_at g1 shelf", _case("robot_at", "r", "b"))
A_R_AT_C = ("robot_at g1 pad", _case("robot_at", "r", "c"))
A_READY = ("robot_ready g1", _case("robot_ready", "r"))


def check_o1(ctx: Ctx) -> dict:
    steps: list[dict] = []
    bench = site_mod.station(ctx.sites["part_precheck"], "bench")["pose"]

    def scene_run(name, start, script, item_at=None):
        scene = ctx.part_precheck_scene(item_at=item_at, start=start)
        site = ctx.site_file("part_precheck", lambda d: d["robot"].update(start=list(start)))
        with ctx.session(name, scene) as s:
            probe = Probe(ctx, s, site)
            try:
                script(s, probe)
            finally:
                probe.close()
        ctx.done(name, s, None)

    def A(s, p):
        # on the bench station, hands open, the item on the bench: what is seen, what is out of view
        steps.append(
            p.ask(
                "A",
                "at the bench",
                [
                    (*A_O_AT_A, "true"),
                    (*A_O_AT_B, "unknown"),
                    (*A_HOLD, "false"),
                    (*A_R_AT_A, "true"),
                    (*A_R_AT_B, "false"),
                    (*A_R_AT_C, "false"),
                    (*A_READY, "true"),
                ],
                ready=True,
            )
        )
        s.sim.PlaceObject("box", ELSEWHERE)
        time.sleep(1.5)
        steps.append(p.ask("A", "item elsewhere on the bench", [(*A_O_AT_A, "false")]))
        s.sim.CoverCamera(True)
        time.sleep(1.5)
        steps.append(
            p.ask(
                "A",
                "camera covered",
                [(*A_O_AT_A, "unknown"), (*A_R_AT_A, "unknown"), (*A_HOLD, "false"), (*A_READY, "true")],
                ready=True,
            )
        )
        s.sim.CoverCamera(False)
        s.sim.PlaceObject("box", IN_PLACE)
        p.ops.robot.hand_command("right", HAND_CLOSE["right"], seconds=2.0, check=p.ops.check)
        time.sleep(1.0)
        steps.append(p.ask("A", "right hand closed on nothing", [(*A_HOLD, "false"), (*A_O_AT_A, "true")]))
        p.ops._hand_to("right", [0.0] * 7, "open")
        p.ops.grasp("box")
        steps.append(
            p.ask(
                "A",
                "carrying the box, shown to the camera",
                [(*A_HOLD, "true"), (*A_O_AT_A, "false")],
                show={"box"},
            )
        )
        steps.append(
            p.ask(
                "A", "carrying the box, in the carry pose (out of view)", [(*A_HOLD, "unknown"), (*A_O_AT_A, "unknown")]
            )
        )
        s.sim.CoverCamera(True)
        steps.append(
            p.ask("A", "carrying the box, camera covered", [(*A_HOLD, "unknown"), (*A_O_AT_A, "unknown")], show={"box"})
        )

    def B(s, p):
        steps.append(
            p.ask(
                "B",
                "0.30 m short of the bench",
                [(*A_R_AT_A, "false"), (*A_O_AT_A, "true"), (*A_R_AT_B, "unknown"), (*A_READY, "true")],
                ready=True,
            )
        )

    def Bh(s, p):
        steps.append(p.ask("B2", "heading 0.2 rad off", [(*A_R_AT_A, "false"), (*A_O_AT_A, "true")]))

    def C(s, p):
        steps.append(
            p.ask(
                "C",
                "facing away: nothing in view",
                [(*A_O_AT_A, "unknown"), (*A_R_AT_A, "unknown"), (*A_HOLD, "false"), (*A_READY, "true")],
                ready=True,
            )
        )

    scene_run("o1_a_bench", bench, A)
    scene_run("o1_b_short", [bench[0] - 0.30, bench[1], bench[2]], B)
    scene_run("o1_b2_heading", [bench[0], bench[1], bench[2] + 0.2], Bh)
    scene_run("o1_c_away", [bench[0], bench[1], bench[2] + math.pi], C)
    rows = [r for st in steps for r in st["rows"]]
    wrong = [r for r in rows if r["verdict"] != r["expected"]]
    disagree = [r for r in rows if r["expected"] != "unknown" and r["verdict"] != str(r["truth"]).lower()]
    seen = {
        k: {v for r in rows if r["atom"].startswith(k) for v in [r["verdict"]]}
        for k in ("object_at", "holding", "robot_at", "robot_ready")
    }
    covered = {k: seen[k] >= {"true", "false", "unknown"} for k in ("object_at", "holding", "robot_at")}
    ready_true = any(r["atom"].startswith("robot_ready") and r["verdict"] == "true" for r in rows)
    return {
        "pass": bool(not wrong and not disagree and all(covered.values()) and ready_true),
        "details": {
            "atoms": len(rows),
            "not_the_expected_verdict": wrong,
            "disagree_with_oracle_where_it_decides": disagree,
            "verdicts_seen": {k: sorted(v) for k, v in seen.items()},
            "robot_ready_true": ready_true,
            "steps": steps,
        },
    }


# ------------------------------------------------------------------------------------------- O2, O3, O4, O5


def part_precheck(ctx: Ctx, name: str, scene=None, **kw):
    scene = scene or ctx.part_precheck_scene()
    site_path = SITES / "part_precheck.json"
    status, rec, log, samples, commands = run3(
        ctx, name, scene, site_path, BENCH / "w_part_precheck.rob", "move_if_present", BIND_PC, **kw
    )
    got = (
        collect(ctx, name, rec, BENCH / "w_part_precheck.rob", "move_if_present", BIND_PC, site_path, samples)
        if rec
        else {"atoms": []}
    )
    return status, rec, samples, commands, got


def check_o2(ctx: Ctx) -> dict:
    out: dict = {}
    status, rec, samples, commands, got = part_precheck(ctx, "o2_item_at_src")
    ok_a = bool(
        rec
        and rec["state"] == "SUCCEEDED"
        and status == 0
        and rec["preconditions"][0]["result"] == "true"
        and [s["status"] for s in rec["steps"]] == ["succeeded"] * 4
    )
    out["item_at_src"] = {
        "pass": ok_a,
        "exit": status,
        "state": rec and rec["state"],
        "failure": rec and rec["failure"],
        "precondition": rec and rec["preconditions"][0]["result"],
        "atoms": got["atoms"],
    }
    # the item elsewhere on the (wide) bench table, in view of the start pose
    scene = ctx.part_precheck_scene(item_at=ELSEWHERE)
    status, rec, samples, commands, got = part_precheck(ctx, "o2_item_elsewhere", scene)
    atom = (rec or {}).get("preconditions", [{}])[0].get("atoms", [{}])[0]
    ok_b = bool(
        rec
        and rec["failure"]
        and rec["failure"]["code"] == "S101"
        and status == 1
        and no_commands(commands)
        and rec["steps"] == []
        and rec["stop"] is None
        and atom.get("region_clearance_m", 0) < 0
    )
    out["item_elsewhere"] = {
        "pass": ok_b,
        "exit": status,
        "failure": rec and rec["failure"],
        "commands_sent": {k: commands[k]["count"] for k in COMMAND_KEYS},
        "precondition_atom": atom,
        "atoms": got["atoms"],
    }

    def cover(s):
        s.sim.CoverCamera(True)
        time.sleep(1.0)

    status, rec, samples, commands, got = part_precheck(ctx, "o2_camera_covered", before=cover)
    for row in got["atoms"]:
        row["expected_unknown"] = True  # the scene is made so that nothing can be seen
    for row in ctx.rows[-len(got["atoms"]) :]:
        row["expected_unknown"] = True
    ok_c = bool(
        rec
        and rec["failure"]
        and rec["failure"]["code"] == "S102"
        and status == 1
        and no_commands(commands)
        and rec["steps"] == []
        and rec["stop"] is None
    )
    out["camera_covered"] = {
        "pass": ok_c,
        "exit": status,
        "failure": rec and rec["failure"],
        "commands_sent": {k: commands[k]["count"] for k in COMMAND_KEYS},
        "atoms": got["atoms"],
    }
    return {"pass": ok_a and ok_b and ok_c, "details": out}


def goal_agrees(got: dict) -> bool:
    return bool(got["atoms"]) and all(
        str(r["truth"]).lower() == r["verdict"] for r in got["atoms"] if r["phase"] == "goal"
    )


def check_o3(ctx: Ctx) -> dict:
    runs = []
    cases = [
        ("e_pick_place", BENCH / "e_pick_place.rob", "pick_place", BIND_PP, SITES / "pick_place.json", "pick_place"),
        ("w_carry_hold", BENCH / "w_carry_hold.rob", "carry_crate", BIND_CH, SITES / "carry_hold.json", "carry_hold"),
        (
            "w_part_precheck",
            BENCH / "w_part_precheck.rob",
            "move_if_present",
            BIND_PC,
            SITES / "part_precheck.json",
            "part_precheck",
        ),
    ]
    for label, source, task, bind, site_path, scene_name in cases:
        for k in (1, 2):
            name = f"o3_{label}_{k}"
            if label == "w_part_precheck" and k == 1 and "o2_item_at_src" in ctx.records:
                rec = ctx.records["o2_item_at_src"]  # O2's first scenario is this run: the same task, the same site
                got = {"atoms": [r for r in ctx.rows if r["run"] == "o2_item_at_src"]}
                status = 0 if rec["state"] == "SUCCEEDED" else 1
                name = "o2_item_at_src"
            else:
                scene = ctx.part_precheck_scene() if label == "w_part_precheck" else site_path
                status, rec, log, samples, commands = run3(ctx, name, scene, site_path, source, task, bind)
                got = collect(ctx, name, rec, source, task, bind, site_path, samples) if rec else {"atoms": []}
            ok = bool(rec and rec["state"] == "SUCCEEDED" and status == 0 and goal_agrees(got))
            runs.append(
                {
                    "task": label,
                    "run": k,
                    "scenario": name,
                    "pass": ok,
                    "state": rec and rec["state"],
                    "failure": rec and rec["failure"],
                    "exit": status,
                    "goal": [
                        (r["atom"]["kind"], r["verdict"], r["truth"]) for r in got["atoms"] if r["phase"] == "goal"
                    ],
                    "observation_motion": [
                        (m["phase"], m["performed"]) for m in (rec or {}).get("observation_motion", [])
                    ],
                    "steps_s": [
                        round((s["t_end_ms"] - s["t_start_ms"]) / 1000, 1) for s in (rec or {}).get("steps", [])
                    ],
                }
            )
    return {"pass": all(r["pass"] for r in runs) and len(runs) == 6, "details": {"runs": runs}}


def check_o4(ctx: Ctx) -> dict:
    out: dict = {}
    pp_site = SITES / "pick_place.json"
    src = ctx.source("pick_no_place", PICK_NO_PLACE)
    # (a) no place step: the box is in the hand at the shelf, seen there: false, S301
    status, rec, log, samples, commands = run3(ctx, "o4_no_place", pp_site, pp_site, src, "pick_no_place", BIND_PP)
    got = collect(ctx, "o4_no_place", rec, src, "pick_no_place", BIND_PP, pp_site, samples) if rec else {"atoms": []}
    motion = (rec or {}).get("observation_motion") or [{}]
    ok = bool(
        rec
        and rec["failure"]
        and rec["failure"]["code"] == "S301"
        and status == 1
        and rec["state"] != "SUCCEEDED"
        and got["atoms"]
        and got["atoms"][-1]["truth"] is False
        and motion[0].get("performed") is True
    )
    out["no_place_step"] = {
        "pass": ok,
        "exit": status,
        "failure": rec and rec["failure"],
        "atoms": got["atoms"],
        "observation_motion": [
            (m["phase"], m["performed"], m["why"]) for m in (rec or {}).get("observation_motion", [])
        ],
    }

    # (b) the camera is covered after the box was placed, before the goal snapshot: unknown, S302
    def cover_after_detach(s):
        done = []

        def watch(proc):
            if not done and s.tail.latest()["commands"]["detach"]["count"] >= 1:
                done.append(s.sim.CoverCamera(True))

        return watch

    status, rec, log, samples, commands = run3(
        ctx, "o4_covered", pp_site, pp_site, BENCH / "e_pick_place.rob", "pick_place", BIND_PP, watch=cover_after_detach
    )
    got = (
        collect(
            ctx,
            "o4_covered",
            rec,
            BENCH / "e_pick_place.rob",
            "pick_place",
            BIND_PP,
            pp_site,
            samples,
            expect_unknown=True,
        )
        if rec
        else {"atoms": []}
    )
    ok = bool(
        rec
        and rec["failure"]
        and rec["failure"]["code"] == "S302"
        and status == 3
        and rec["state"] != "SUCCEEDED"
        and [s["status"] for s in rec["steps"]] == ["succeeded"] * 4
    )
    out["camera_covered_before_goal"] = {
        "pass": ok,
        "exit": status,
        "failure": rec and rec["failure"],
        "atoms": got["atoms"],
    }
    # (c) the view pose is not shown clear: no arm motion, the atom stays unknown (S302)
    post = {"name": "post", "pos": [2.73, 2.30, 0.93], "half_size": [0.05, 0.05, 0.10]}  # at the shelf's view pose
    blocked_site = ctx.site_file("pick_place", lambda d: d.update(obstacles=[post]))
    status, rec, log, samples, commands = run3(ctx, "o4_blocked", pp_site, blocked_site, src, "pick_no_place", BIND_PP)
    got = (
        collect(ctx, "o4_blocked", rec, src, "pick_no_place", blocked_site, samples, expect_unknown=True)
        if rec
        else {"atoms": []}
    )
    motion = ((rec or {}).get("observation_motion") or [{}])[0]
    last_end = rec["steps"][-1]["t_end_ms"] / 1000 if rec and rec["steps"] else None
    tail = _Tail(samples)
    after = [x for x in samples if last_end is not None and x["sim_time"] >= last_end]
    wrist_m = (
        float(np.linalg.norm(np.array(after[-1]["wrists"]["right"]) - np.array(after[0]["wrists"]["right"])))
        if len(after) > 1
        else None
    )
    stance_after = {x["stance"]["model"] for x in after}
    ok = bool(
        rec
        and rec["failure"]
        and rec["failure"]["code"] == "S302"
        and status == 3
        and motion.get("performed") is False
        and (motion.get("clearance") or {}).get("closest", {}).get("box") == "post"
        and wrist_m is not None
        and wrist_m <= 0.01
        and stance_after == {"free"}
        and tail.latest()["attached"] == {"box": "right"}
    )
    out["view_pose_blocked"] = {
        "pass": ok,
        "exit": status,
        "failure": rec and rec["failure"],
        "observation_motion": motion,
        "wrist_moved_after_last_step_m": wrist_m,
        "stance_after_last_step": sorted(stance_after),
        "atoms": got["atoms"],
    }

    # (d) a cancel during the observation motion: a biped stop, the hand unchanged, never a success
    def cancel_in_excursion(s, stop_file):
        state = {"supported_edges": 0, "was": None, "at": None, "written": False}

        def watch(proc):
            now = s.tail.latest()
            model = now["stance"]["model"]
            if model == "supported" and state["was"] != "supported":
                state["supported_edges"] += 1
                if state["supported_edges"] == 2:  # the second arm-work stance is the observation motion's
                    state["at"] = now["sim_time"]
            state["was"] = model
            if state["at"] is not None and not state["written"] and now["sim_time"] >= state["at"] + 3.0:
                stop_file.write_text("cancel")
                state["written"] = True

        return watch, state

    stop_file = ctx.dir / "o4_cancel.stop"
    stop_file.write_text("")
    grab = ctx.source("grab", GRAB)
    ch_site = SITES / "carry_hold.json"
    holder = {}

    def make_watch(s):
        w, state = cancel_in_excursion(s, stop_file)
        holder["state"] = state
        return w

    status, rec, log, samples, commands = run3(
        ctx, "o4_cancel_in_excursion", ch_site, ch_site, grab, "grab", GRAB_CH, watch=make_watch, stop_file=stop_file
    )
    last = samples[-1] if samples else {}
    stop = (rec or {}).get("stop") or {}
    conf = stop.get("confirmation") or {}
    motion = (rec or {}).get("observation_motion") or [None]
    held_after = last.get("attached", {})
    hands_ok = (
        last.get("hands", {}).get("right") is not None
        and float(max(abs(np.array(last["hands"]["right"]) - np.array(HAND_CLOSE["right"])))) < 0.3
    )
    ok = bool(
        rec
        and rec["failure"]
        and rec["failure"]["code"] == "S511"
        and rec["state"] != "SUCCEEDED"
        and status == 1
        and stop.get("kind") == "controlled_stand"
        and stop.get("safe_state", {}).get("hands") == "unchanged"
        and conf.get("confirmed")
        and held_after == {"crate": "right"}
        and hands_ok
        and holder["state"]["written"]
    )
    out["cancel_during_observation_motion"] = {
        "pass": ok,
        "exit": status,
        "failure": rec and rec["failure"],
        "stop": {k: stop.get(k) for k in ("reason", "code", "kind", "safe_state")},
        "confirmation": conf,
        "still_holding_at_end": held_after,
        "right_hand_joint_error_from_closed_rad": None
        if not hands_ok and not last
        else float(max(abs(np.array(last["hands"]["right"]) - np.array(HAND_CLOSE["right"])))),
        "cancel_written": holder["state"]["written"],
        "observation_motion_recorded": motion,
    }
    return {"pass": all(v["pass"] for v in out.values()), "details": out}


def check_o5(ctx: Ctx) -> dict:
    out: dict = {}
    pp = SITES / "pick_place.json"
    for label, header, expect_state, expect_code, status_want, verify in (
        ("max_age_1ms", "    @verify max_age 1ms;\n", "FAILED", "S302", 3, {"min_confidence": "0.9", "max_age_ms": 1}),
        (
            "relaxed",
            "    @verify min_confidence 0.5;\n    @verify max_age 30s;\n",
            "SUCCEEDED",
            None,
            0,
            {"min_confidence": "0.5", "max_age_ms": 30000},
        ),
        ("defaults", "", "SUCCEEDED", None, 0, {"min_confidence": "0.9", "max_age_ms": 10000}),
    ):
        src = ctx.source(f"go_{label}", GO % header)
        name = f"o5_{label}"
        status, rec, log, samples, commands = run3(ctx, name, pp, pp, src, "go", {"r": "robot_1", "a": "loc_a"})
        got = (
            collect(
                ctx,
                name,
                rec,
                src,
                "go",
                {"r": "robot_1", "a": "loc_a"},
                pp,
                samples,
                expect_unknown=expect_state == "FAILED",
            )
            if rec
            else {"atoms": []}
        )
        ok = bool(
            rec
            and rec["state"] == expect_state
            and ((rec["failure"] or {}).get("code") == expect_code)
            and status == status_want
            and rec["verify"] == verify
        )
        out[label] = {
            "pass": ok,
            "exit": status,
            "state": rec and rec["state"],
            "failure": rec and rec["failure"],
            "verify": rec and rec["verify"],
            "atoms": got["atoms"],
            "marker_age_ms": [m["age_ms"] for sn in (rec or {}).get("snapshots", []) for m in sn["markers"].values()],
        }
    return {"pass": all(v["pass"] for v in out.values()), "details": out}


# ------------------------------------------------------------------------------------------------ O6 and O7


def check_o6(ctx: Ctx) -> dict:
    decided = [r for r in ctx.rows if not r["expected_unknown"]]
    agree = [r for r in decided if r["verdict"] == str(r["truth"]).lower()]
    unknown = [r for r in decided if r["verdict"] == "unknown"]
    wrong = [r for r in decided if r["verdict"] in ("true", "false") and r["verdict"] != str(r["truth"]).lower()]
    scripted_unknown = [r for r in ctx.rows if r["expected_unknown"]]
    wrongly_decided = [r for r in scripted_unknown if r["verdict"] != "unknown"]
    rate = len(agree) / len(decided) if decided else 0.0
    ok = bool(decided) and rate >= AGREEMENT_MIN and not ctx.false_successes and not wrongly_decided
    return {
        "pass": ok,
        "details": {
            "atoms_the_oracle_decides": len(decided),
            "agree": len(agree),
            "agreement": round(rate, 4),
            "required": AGREEMENT_MIN,
            "unknown_where_the_oracle_decides": unknown,
            "wrong_true_or_false": wrong,
            "scripted_unknown_atoms": len(scripted_unknown),
            "scripted_unknown_decided_anyway": wrongly_decided,
            "false_successes": ctx.false_successes,
            "runs_succeeded": len(ctx.successes),
        },
    }


def core_accepts(core: dict) -> list[str]:
    """What the robolang.core checks demand of an outcome: the payload fits the Core 0.1 schema, every observation's
    signature verifies and names the reporter, and the meaning check (C402: a `succeeded` outcome has every criterion
    met) accepts it. Returns the problems."""
    problems = []
    validator = _Validator(load_schema("message-0.1.schema.json"))
    for e in validator.errors(core, {"$ref": "#/$defs/payload_outcome"}):
        problems.append(f"schema: {e}")
    for observation in core["evidence"]:
        if not verify_sim(observation, O.SIM_KEY):
            problems.append(f"signature of {observation['id']}")
        if observation["source"]["participant"] != O.SIM_PARTICIPANT:
            problems.append(f"participant of {observation['id']}")
    receipt = check_meaning(
        {"type": "outcome", "schema_version": "0.1", "payload": core},
        Context("rl:sim:check", 0, 0, 0, {O.SIM_KEY_REF: O.SIM_KEY}),
    )
    if not receipt.accepted:
        problems.append(f"meaning: {receipt}")
    return problems


def check_o7(ctx: Ctx) -> dict:
    rows = []
    for item in ctx.successes:
        core = item["record"].get("core_outcome")
        problems = ["no core_outcome"] if core is None else core_accepts(core)
        if core is not None:
            met = [r["met"] for r in core["outcome"]["verification"]["criteria_results"]]
            if not (core["outcome"]["status"] == "succeeded" and all(met) and len(core["evidence"]) == len(met)):
                problems.append("status, criteria or evidence do not match")
        rows.append(
            {
                "run": item["run"],
                "problems": problems,
                "evidence": [
                    (e["subject"], e["property"], e["value"], e["confidence"]) for e in (core or {}).get("evidence", [])
                ],
            }
        )
    return {"pass": bool(rows) and all(not r["problems"] for r in rows), "details": {"succeeded_records": rows}}


# ------------------------------------------------------------------------------------------------------ main


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="RoboLang G1 phase 3 checks O1-O7")
    ap.add_argument("--out", default="phase3.json")
    ap.add_argument("--run-dir")
    ap.add_argument("--stance", default="free", choices=["free", "supported"])
    ap.add_argument("--only", default="", help="development: comma list of O1,O2,O3,O4,O5 to run (O6, O7 follow)")
    args = ap.parse_args(argv)
    out = Path(args.out)
    ctx = Ctx(Path(args.run_dir) if args.run_dir else out.with_suffix(".run"), args.stance)
    ctx.dir.mkdir(parents=True, exist_ok=True)
    only = set(filter(None, args.only.split(",")))
    result: dict = {
        "checks": {},
        "stance_model": args.stance,
        "versions": assets.versions(),
        "arch": os.uname().machine,
    }
    t0 = time.perf_counter()

    def write():
        out.write_text(
            json.dumps({**result, "runs": ctx.runs, "robot_at_verdicts": ctx.robot_at}, indent=2, default=str)
        )

    def record(name, fn):
        started = time.perf_counter()
        try:
            r = fn()
        except Exception as exc:  # a failed check must not stop the report
            import traceback

            r = {"pass": False, "details": {"error": f"{type(exc).__name__}: {exc}", "trace": traceback.format_exc()}}
        r["elapsed_s"] = round(time.perf_counter() - started, 1)
        result["checks"][name] = r
        print(f"{name}: {'pass' if r['pass'] else 'FAIL'}  ({r['elapsed_s']} s)", flush=True)
        write()

    for name, fn in (("O1", check_o1), ("O2", check_o2), ("O3", check_o3), ("O4", check_o4), ("O5", check_o5)):
        if not only or name in only:
            record(name, lambda fn=fn: fn(ctx))
    record("O6", lambda: check_o6(ctx))
    record("O7", lambda: check_o7(ctx))
    result["go"] = all(c["pass"] for c in result["checks"].values()) and not only
    result["realtime_valid_all_runs"] = all(r["realtime_valid"] for r in ctx.runs)
    result["robot_at_margins"] = {
        "tolerance_m": ROBOT_AT_M,
        "tolerance_rad": ROBOT_AT_RAD,
        "largest_offset_m_when_true": max(
            (r["offset_m"] for r in ctx.robot_at if r["verdict"] == "true"), default=None
        ),
        "largest_offset_rad_when_true": max(
            (r["offset_rad"] for r in ctx.robot_at if r["verdict"] == "true"), default=None
        ),
    }
    result["elapsed_s"] = round(time.perf_counter() - t0, 1)
    write()
    print(f"go: {result['go']}  (realtime_valid in every run: {result['realtime_valid_all_runs']}); wrote {out}")
    return 0 if result["go"] else 1


if __name__ == "__main__":
    sys.exit(main())
