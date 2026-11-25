"""Phase 4 scenes (decision 0046 section 3): one site per benchmark task of the acceptance family, same entity IDs.

    python -m robolang_g1.scenes --list                 the scene names (14 tasks, then the scripted failure cases)
    python -m robolang_g1.scenes SCENE [--out DIR]      writes site.json (what the executor is told) and scene.json
                                                        (what the stand-in builds), prints the task and its bindings

The reference `.rob` files run unchanged, with the bindings of `bench/reference/<task>.json`. Only the world is
rewritten, as in the ROS 2 backend's `scenes.py`: every object is a box with an ArUco marker on its top, every location
a region on a table in front of a **station** (a marker plate on the wall behind the table and a standing pose), or a
station alone for a place only the robot goes to.

**The layout follows the visibility rule (0046 section 3, item 2).** The robot has one camera, in its head, and an atom
out of view is `unknown` (S302). So every location a goal names is in view from the station where the plan ends:
locations that a goal names together are slots of one table, seen from one station (`a_sort_three`, `w_split_boxes`,
`l_vial_hold`), and the dock of `e_box_then_dock` is a standing place one step back from the destination's.
`check_visibility` checks the rule from the geometry before any run; a scene that fails it is not run.

A box's marker is 5.7 cm wide. The camera reads it surely up to about half a metre ahead of the pelvis; at 0.61 m its
confidence was 0.4 in one run (phase 2, x86), which is refused as not seen. `SEEN_AHEAD_M` is the limit used here.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from pathlib import Path

from . import site as site_mod

REPO = Path(__file__).resolve().parents[3]
REFERENCE = REPO / "bench" / "reference"

WALL_X = 3.75  # the station plates stand on this line, facing the robot (-x)
PLATE = {"size": 0.24, "z": 0.19, "facing": 3.14159265}
TABLE_X, TABLE_HALF_X, TABLE_HALF_Z = 2.81, 0.13, 0.375  # the table top is at 0.75 m
BOX_Z = 0.79  # centre of an 8 cm box standing on a table
STAND_GRASP, STAND_PLACE = 1.3, 1.35  # the pelvis this far in front of the plate: where grasps, and places, were verified
STEP_BACK = 0.12  # the dock of e_box_then_dock: this far behind the destination's standing place
SEEN_AHEAD_M = 0.52  # a box's marker further ahead of the pelvis than this is not counted as in view
REGION_Y, REGION_Z = (0.56, 0.76), (0.91, 1.09)  # a region in the plate's frame: height above its centre, distance out
# Slots across a table, as the pelvis's sideways offset to the slot's centre (right is negative). "M" is where phases 2
# and 3 grasp and place, with the right hand. The arm planner takes a box between 5 and 20 cm to the side of the pelvis
# with the hand of that side (measured on the robot's own model: quick from 0.05 to 0.20 m, a minute of search with the
# waist leaning at 0.22 m, none at 0.01 or 0.28 m), so a table holds two boxes for one hand and three for both:
# "R" and "L" for two (one hand each), "R1" and "R2" for two of the right hand, "L1" and "L2" for two of the left.
SLOT = {"M": -0.10, "R": -0.12, "L": 0.12, "R1": -0.20, "R2": -0.06, "L1": 0.06, "L2": 0.20}
SLOT_HALF = 0.065
PAD = {"pos": [0.55, 2.5, PLATE["z"]], "facing": -1.570796325}  # a plate with no table: a place only the robot goes to

FLOOR = {"center": [2.0, 1.2], "half_size": [5.5, 4.0], "friction": 1.0}
OBJECT = {"half_size": [0.04, 0.04, 0.04], "mass": 0.2}


def _region(slot: str | None, z=REGION_Z) -> dict:
    """A slot's region in the plate's frame (x to the plate's right as the robot sees it, y up, z out of the plate).
    The plate is 0.45 m to the left of the pelvis, so a sideways offset `s` of the pelvis frame is x = 0.45 - s."""
    if slot is None:  # the whole table, as in phases 2 and 3
        x = (0.35, 0.75)
    else:
        x = (0.45 - SLOT[slot] - SLOT_HALF, 0.45 - SLOT[slot] + SLOT_HALF)
    return {"min": [round(x[0], 3), REGION_Y[0], z[0]], "max": [round(x[1], 3), REGION_Y[1], z[1]]}


def _unit(name: str, y0: float, marker: int, stand: float, wide: bool) -> tuple[dict, dict]:
    """A table and its station, for a robot that stands at y = `y0` facing the wall."""
    table = {
        "name": f"{name}_table",
        "pos": [TABLE_X, round(y0 - (0.02 if wide else 0.065), 4), TABLE_HALF_Z],
        "half_size": [TABLE_HALF_X, 0.28 if wide else 0.185, TABLE_HALF_Z],
    }
    station = {
        "name": name,
        "marker": {"id": marker, "size": PLATE["size"], "pos": [WALL_X, round(y0 + 0.45, 4), PLATE["z"]],
                   "facing": PLATE["facing"]},
        "stand": [stand, 0.45, 0.0],
    }
    return table, station


# task: units, locations and objects.
#   units:     {unit: (y0, stand, wide)}; "pad" is the plate with no table
#   locations: {parameter: (unit, slot, or None for the table as in phases 2 and 3, "*" for a wide table from side to
#              side, or "-" for a station with no region)}
#   objects:   {parameter: (unit, slot)} where it starts
#   start:     the robot's start pose (default the origin, 2.45 m from the first table)
#   extra:     stations made from another's standing place: {parameter: (unit, metres back)}
A, B = 0.0, 2.4  # the two tables' y0
TASKS: dict[str, dict] = {
    "a_scrap_part": {
        "units": {"a": (A, STAND_GRASP, False), "b": (B, STAND_PLACE, False)},
        "locations": {"here": ("a", None), "bin": ("b", None)},
        "objects": {"part": ("a", "M")},
    },
    "a_sort_three": {  # three parts on the feed table; the three goal slots on one table, seen from the last one.
        # The right hand takes x and y (the outer one first), the left hand z; each is placed in a slot of its own hand.
        "units": {"a": (A, STAND_GRASP, True), "b": (B, STAND_PLACE, True)},
        "locations": {"feed": ("a", "*"), "sx": ("b", "R2"), "sy": ("b", "R1"), "sz": ("b", "L")},
        "objects": {"x": ("a", "R1"), "y": ("a", "R2"), "z": ("a", "L")},
    },
    "e_box_then_dock": {  # the dock: one step back from the destination's standing place, the destination in view
        "units": {"a": (A, STAND_GRASP, False), "b": (B, STAND_PLACE, False)},
        "locations": {"src": ("a", None), "dst": ("b", None)},
        "objects": {"box": ("a", "M")},
        "extra": {"dock": ("b", STEP_BACK)},
    },
    "e_hold_box": {
        "units": {"a": (A, STAND_GRASP, False)},
        "locations": {"src": ("a", None)},
        "objects": {"box": ("a", "M")},
    },
    "e_pause_home": {
        "units": {"a": (A, STAND_GRASP, False), "pad": None},
        "locations": {"spot": ("a", "-"), "home": ("pad", "-")},
        "objects": {},
    },
    "e_pick_place": {
        "units": {"a": (A, STAND_GRASP, False), "b": (B, STAND_PLACE, False)},
        "locations": {"src": ("a", None), "dst": ("b", None)},
        "objects": {"box": ("a", "M")},
    },
    "l_vial_hold": {  # the bench and the spinner are two slots of one table: the spinner is seen from the bench.
        # The left hand moves vial a, so that the right hand, which shows vial b to the camera at the end, is on the
        # other side of the table from a. The robot grasps and places at this one station, so its regions lie 5 cm
        # nearer the plate: the place is then 0.35 m ahead, as from the place stations.
        "units": {"a": (A, STAND_GRASP, True), "b": (B, STAND_GRASP, False)},
        "locations": {"bench": ("a", "L2"), "spin": ("a", "L1"), "fridge": ("b", None)},
        "objects": {"a": ("a", "L2"), "b": ("b", "M")},
        "region_z": {"a": (0.86, 1.04)},
    },
    "l_vial_to_fridge": {
        "units": {"a": (A, STAND_GRASP, False), "b": (B, STAND_PLACE, False)},
        "locations": {"src": ("a", None), "dst": ("b", None)},
        "objects": {"vial": ("a", "M")},
    },
    "w_part_precheck": {  # the robot starts in view of the source: the precondition is observed from the start pose
        "units": {"a": (A, STAND_GRASP, False), "b": (B, STAND_PLACE, False)},
        "locations": {"src": ("a", None), "dst": ("b", None)},
        "objects": {"item": ("a", "M")},
        "start": [2.25, 0.0, 0.0],
    },
    "w_pick_from_table": {
        "units": {"a": (A, STAND_GRASP, False), "b": (B, STAND_PLACE, False)},
        "locations": {"src": ("a", None), "dst": ("b", None)},
        "objects": {"item": ("a", "M")},
    },
    "w_split_boxes": {  # two sources on one table, two destinations on the other, both seen from the last; a hand each
        "units": {"a": (A, STAND_GRASP, True), "b": (B, STAND_PLACE, True)},
        "locations": {"sa": ("a", "R"), "sb": ("a", "L"), "ta": ("b", "R"), "tb": ("b", "L")},
        "objects": {"a": ("a", "R"), "b": ("a", "L")},
    },
    "e_park_amr": {
        "units": {"pad": None},
        "locations": {"home": ("pad", "-")},
        "objects": {},
    },
    "w_carry_hold": {
        "units": {"a": (A, STAND_GRASP, False), "pad": None},
        "locations": {"dock": ("a", None), "dst": ("pad", "-")},
        "objects": {"crate": ("a", "M")},
    },
    "w_clear_dock": {
        "units": {"a": (A, STAND_GRASP, False), "b": (B, STAND_PLACE, False)},
        "locations": {"dock": ("a", None), "dst": ("b", None)},
        "objects": {"crate": ("a", "M")},
    },
}
# The two tasks that walk while holding (0046 U6).
WALK_HOLDING = ("w_carry_hold", "w_clear_dock")

# Scripted failure cases (0046 section 3, item 4), on e_pick_place. Each must end FAILED with its code, never SUCCEEDED.
#   scene:  an edit of what the stand-in builds (the executor is told the unedited site)
#   site:   an edit of what the executor is told
#   hook:   something the acceptance run does while the task runs (see phase4.py)
CASES: dict[str, dict] = {
    "fail_object_absent": {"what": "the box is not in the scene", "code": "S506", "scene": "absent"},
    "fail_object_moved": {
        "what": "the box is on the destination's table, not at the source the task names",
        "code": "S506",
        "scene": "moved",
    },
    "fail_out_of_reach": {
        "what": "the destination's region is 0.8 m ahead of where the robot stands, behind the reach of the arm",
        "code": "S502",
        "site": "far_region",
        "scene": "far_region",
    },
    "fail_camera_covered": {"what": "the head camera is covered before the task starts", "code": "S517",
                            "hook": "cover"},
    "fail_push_fall": {"what": "a push on the torso while the robot walks makes it fall", "code": "S516",
                       "hook": "push"},
}


def names() -> list[str]:
    return [*TASKS, *CASES]


def reference(task_id: str) -> dict:
    return json.loads((REFERENCE / f"{task_id}.json").read_text(encoding="utf-8"))


def build(scene: str) -> dict:
    """A scene: the reference task and its bindings, the site the executor is told and the scene the stand-in builds."""
    task_id = scene if scene in TASKS else "e_pick_place"
    spec, ref = TASKS[task_id], reference(task_id)
    bind = ref["bindings"]
    tables, stations, by_unit = [], [], {}
    marker = 20
    for unit, params in spec["units"].items():
        if params is None:  # the pad
            st = {"name": "pad", "marker": {"id": marker, "size": PLATE["size"], **PAD}, "stand": [1.3, 0.45, 0.0]}
        else:
            y0, stand, wide = params
            table, st = _unit(unit, y0, marker, stand, wide)
            tables.append(table)
        stations.append(st)
        by_unit[unit] = st
        marker += 1
    locations, loc_bind = [], {}
    for param, (unit, slot) in spec["locations"].items():
        loc = {"name": param, "station": by_unit[unit]["name"]}
        if slot == "*":
            loc["region"] = {"min": [0.20, REGION_Y[0], REGION_Z[0]], "max": [0.72, REGION_Y[1], REGION_Z[1]]}
        elif slot != "-":
            loc["region"] = _region(slot, spec.get("region_z", {}).get(unit, REGION_Z))
        locations.append(loc)
        loc_bind[bind[param]] = param
    for param, (unit, back) in spec.get("extra", {}).items():
        y0, stand, _ = spec["units"][unit]
        # its own plate, on the wall to the right of the table as the unit's is to the left
        st = {
            "name": param,
            "marker": {"id": marker, "size": PLATE["size"], "pos": [WALL_X, round(y0 - 0.60, 4), PLATE["z"]],
                       "facing": PLATE["facing"]},
            "stand": [round(stand + back, 4), -0.60, 0.0],
        }
        marker += 1
        stations.append(st)
        locations.append({"name": param, "station": param})
        loc_bind[bind[param]] = param
    objects, obj_bind = [], {}
    for k, (param, (unit, slot)) in enumerate(spec["objects"].items()):
        y0 = spec["units"][unit][0]
        objects.append({
            "name": param,
            "pos": [2.73, round(y0 + SLOT[slot], 4), BOX_Z],
            **copy.deepcopy(OBJECT),
            "marker": {"id": 7 + k, "face": "top"},
            "grasp": {"hand": "left" if SLOT[slot] > 0 else "right"},
        })
        obj_bind[bind[param]] = param
    robot_param = next(p for p in bind if p not in spec["locations"] and p not in spec["objects"]
                       and p not in spec.get("extra", {}))
    site = {
        "schema": site_mod.SITE_SCHEMA,
        "name": scene,
        "floor": copy.deepcopy(FLOOR),
        "robot": {"name": "g1", "start": list(spec.get("start", [0.0, 0.0, 0.0])), "arm_stance": "supported"},
        "tables": tables,
        "objects": objects,
        "stations": stations,
        "locations": locations,
        "bindings": {"robots": {bind[robot_param]: "g1"}, "objects": obj_bind, "locations": loc_bind},
    }
    world = copy.deepcopy(site)  # what the stand-in builds
    case = CASES.get(scene)
    if case:
        if case.get("scene") == "absent":
            world["objects"][0]["pos"] = [-3.0, -2.0, 0.04]  # on the floor behind the start, seen from nowhere on the way
        elif case.get("scene") == "moved":
            world["objects"][0]["pos"][1] = round(B + SLOT["M"], 4)
        if case.get("site") == "far_region":
            for d in (site, world):  # a deep table, and the region at its far side: 0.8 m ahead of the pelvis
                t = next(t for t in d["tables"] if t["name"] == "b_table")
                t["pos"][0], t["half_size"][0] = 3.03, 0.35
                loc = next(x for x in d["locations"] if x["name"] == "dst")
                loc["region"]["min"][2], loc["region"]["max"][2] = 0.46, 0.64
    return {
        "scene": scene,
        "task_id": task_id,
        "task": ref["task"],
        "source": str(REFERENCE / f"{task_id}.rob"),
        "bind": bind,
        "site": site,
        "world": world,
        "case": case,
    }


# ------------------------------------------------------------------------------------------ the visibility rule


def check_visibility(built: dict) -> list[str]:
    """The visibility rule for a scene, from the site geometry alone. Returns what breaks it (empty: it holds).

    `perception.visibility_violations` checks the rule itself: every location a goal atom names is in the head
    camera's field of view, over the tables, from the station where the plan ends, and every location a precondition
    names from the start pose. This adds the range at which a box's marker is read: the centre of an `object_at`
    region is no further ahead of the pelvis than `SEEN_AHEAD_M`."""
    from robolang import ir

    from . import executor, kinematics, perception
    from . import observe as O

    site = site_mod.load_site(copy.deepcopy(built["site"]))
    task = ir.compile_source(Path(built["source"]).read_text(encoding="utf-8"), built["source"]).ir.task(built["task"])
    names = executor.refuse(task, built["bind"], site)
    problems = perception.visibility_violations(site, task, names, kinematics.Kinematics(site["camera"]))
    moves = [s for _, s in ir.walk(task.steps) if isinstance(s, ir.Move)]
    end = (
        site_mod.station(site, site_mod.location(site, names[moves[-1].target])["station"])["pose"]
        if moves
        else site["robot"]["start"]
    )
    for label, conditions, pose in (("goal", task.goal or (), end), ("precondition", task.preconditions,
                                                                     site["robot"]["start"])):
        for atom in (a for c in conditions for a in O.atoms_of(c)):
            if isinstance(atom, ir.ObjectAt):
                loc = site_mod.location(site, names[atom.location])
                R, p = site_mod.marker_frame(site_mod.station(site, loc["station"])["marker"])
                centre = R @ site_mod.region_centre_in_marker(loc) + p
                ahead = math.cos(pose[2]) * (centre[0] - pose[0]) + math.sin(pose[2]) * (centre[1] - pose[1])
                if ahead > SEEN_AHEAD_M:
                    problems.append(f"{label} object_at {names[atom.location]}: its region is {ahead:.2f} m ahead of "
                                    f"the pelvis, over the {SEEN_AHEAD_M} m at which a box's marker is read")
    return problems


def write(scene: str, out: Path) -> dict:
    built = build(scene)
    site_mod.load_site(copy.deepcopy(built["site"]))  # a scene that is not a valid site is an error here, not in a run
    site_mod.load_site(copy.deepcopy(built["world"]))
    out.mkdir(parents=True, exist_ok=True)
    (out / "site.json").write_text(json.dumps(built["site"], indent=1), encoding="utf-8")
    (out / "scene.json").write_text(json.dumps(built["world"], indent=1), encoding="utf-8")
    return built


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("scene", nargs="?")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--out", type=Path, default=Path("."))
    args = ap.parse_args(argv)
    if args.list or not args.scene:
        print("\n".join(names()))
        return 0
    if args.scene not in names():
        print(f"unknown scene {args.scene!r}", file=sys.stderr)
        return 2
    built = write(args.scene, args.out)
    problems = check_visibility(built)
    print(json.dumps({"task": built["task"], "source": built["source"], "bind": built["bind"],
                      "site": str(args.out / "site.json"), "scene": str(args.out / "scene.json"),
                      "visibility": problems or "holds"}, indent=1))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
