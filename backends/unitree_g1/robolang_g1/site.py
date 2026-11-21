"""The site file (`robolang-g1-site/2`; `/1` still loads): the world, as the scene generator and executor read it.

A site is a small JSON file. The scene generator (`sim/scene.py`) builds a MuJoCo scene from it; the executor reads
the same file for what an installation tells the robot: where the station markers are, where to stand in front of
them, which locations are where, which object carries which marker, and which `.rob` entity IDs mean which of those.
The executor does not use the objects' positions (an object is found by its marker), and a scene may differ from the
site the executor is given (an object removed, a marker covered): that is how the failure cases are made.

    {
      "schema": "robolang-g1-site/2", "name": "...",
      "floor":  {"center": [x, y], "half_size": [hx, hy], "friction": 1.0},
      "robot":  {"name": "g1", "start": [x, y, yaw], "arm_stance": "free" | "supported"},   # start: pelvis pose
                                                                       # in the map at start (what the robot is told)
      "camera": {"width": 640, "height": 480, "fovy": 58.0, "pos": [x, y, z], "pitch": 0.83},  # head calibration
      "tables": [{"name", "pos", "half_size", "rgba"?}],
      "objects": [{"name", "pos", "half_size", "mass"?, "rgba"?, "marker": {"id", "face", "size"?},
                   "grasp"?: {"hand": "nearest" | "left" | "right"}}],
      "stations": [{"name", "marker": {"id", "size", "pos": [x, y, z], "facing": yaw, "tilt"?: 0.0, "covered"?: false},
                    "stand": [d, l, a]}],
      "locations": [{"name", "station", "region"?: {"min": [x, y, z], "max": [x, y, z]}}],
      "bindings": {"robots": {ID: robot}, "objects": {ID: object}, "locations": {ID: location}}
    }

Frames. The map is the world frame: x forward at yaw 0, z up, metres, radians. **The marker frame** of a station
marker has its origin at the marker's centre, x to the right as seen from the front, y up, z out of the marker
(toward the viewer); for a vertical panel facing `facing` (the yaw of its outward normal) the axes are (up x out, up,
out). `stand` is where the pelvis stands relative to the marker, in plan view: `d` along the outward normal, `l` to
the left of it (seen from the marker), and the pelvis yaw is `facing + pi + a`, so `a = 0` looks straight at the
marker. A location's `region` is a box in its station's marker frame: an object whose marker is inside it is "at" the
location. Positions in the executor's own frame (the pelvis) never appear in a site.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

SCHEMA_1 = "robolang-g1-site/1"
SITE_SCHEMA = SCHEMA_2 = "robolang-g1-site/2"
MARKER_BORDER = 0.2  # white border around an ArUco marker, as a fraction of its side
FACE_NORMALS = {"top": (0, 0, 1), "-x": (-1, 0, 0), "+x": (1, 0, 0), "-y": (0, -1, 0), "+y": (0, 1, 0)}
# The G1's head camera (an Intel RealSense D435) is a fixed joint on the torso link in Unitree's with-hand description:
# unitree_rl_gym g1_29dof_with_hand.urdf:623-624: origin xyz 0.0576235 0.01753 0.41987, rpy 0 0.8307767 0 (pitched
# down). The field of view (58 degrees vertical, 640x480) is the spike's choice and is not checked against a D435.
DEFAULT_CAMERA = {"width": 640, "height": 480, "fovy": 58.0, "pos": [0.0576235, 0.01753, 0.41987], "pitch": 0.8307767}
HANDS = ("nearest", "left", "right")
_TOP = {"schema", "name", "floor", "robot", "camera", "tables", "objects", "stations", "locations", "bindings"}


class SiteError(ValueError):
    """The site file is not valid; the message names the entry and the field."""


def load_site(site) -> dict:
    """Read and check a site (a path or a parsed dict); returns it with defaults and derived values filled in."""
    data = json.loads(Path(site).read_text()) if not isinstance(site, dict) else dict(site)
    schema = data.get("schema")
    if schema not in (SCHEMA_1, SCHEMA_2):
        raise SiteError(f"schema must be {SCHEMA_2!r} (or {SCHEMA_1!r})")
    if schema == SCHEMA_2 and set(data) - _TOP:
        raise SiteError(f"unknown site keys {sorted(set(data) - _TOP)}")
    names: set[tuple[str, str]] = set()

    def need(entry, field, size=None, where=""):
        value = entry.get(field)
        if value is None:
            raise SiteError(f"{where}: missing {field!r}")
        if size is not None and (
            not isinstance(value, list)
            or len(value) != size
            or not all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in value)
        ):
            raise SiteError(f"{where}: {field!r} must be a list of {size} numbers")
        return value

    def name_of(entry, kind):
        name = entry.get("name")
        if not isinstance(name, str) or not name or (kind, name) in names:
            raise SiteError(f"{kind}: missing or duplicate name {name!r}")
        names.add((kind, name))
        return f"{kind} {name}"

    marker_ids: dict[int, str] = {}

    def check_marker(marker, where, size_required):
        mid = need(marker, "id", where=where)
        if not isinstance(mid, int) or isinstance(mid, bool) or not 0 <= mid < 50:
            raise SiteError(f"{where}: marker id must be an integer 0-49 (ArUco 4x4_50)")
        if mid in marker_ids:
            raise SiteError(f"{where}: marker id {mid} is already used by {marker_ids[mid]}")
        marker_ids[mid] = where
        if size_required and not (isinstance(marker.get("size"), (int, float)) and marker["size"] > 0):
            raise SiteError(f"{where}: marker needs a positive 'size' (side of the black square, m)")

    data.setdefault("name", "site")
    data.setdefault("floor", {})
    data.setdefault("robot", {})
    for kind in ("tables", "objects", "stations", "locations"):
        data.setdefault(kind, [])
    data.setdefault("bindings", {})
    data["robot"] = dict(data["robot"])
    data["robot"].setdefault("name", "g1")
    if data["robot"].setdefault("arm_stance", "free") not in ("free", "supported"):
        raise SiteError("robot.arm_stance must be 'free' or 'supported'")
    start = data["robot"].setdefault("start", [0.0, 0.0, 0.0])
    if not isinstance(start, list) or len(start) != 3:
        raise SiteError("robot.start must be [x, y, yaw]")
    camera = {**DEFAULT_CAMERA, **dict(data.get("camera") or {})}
    need(camera, "pos", 3, "camera")
    if not (
        isinstance(camera["pitch"], (int, float))
        and 0 <= camera["pitch"] < math.pi / 2
        and camera["fovy"] > 0
        and isinstance(camera["width"], int)
        and isinstance(camera["height"], int)
    ):
        raise SiteError("camera: width and height are integers, fovy is positive, pitch is in [0, pi/2)")
    data["camera"] = camera
    for t in data["tables"]:
        where = name_of(t, "table")
        need(t, "pos", 3, where)
        need(t, "half_size", 3, where)
    for o in data["objects"]:
        where = name_of(o, "object")
        need(o, "pos", 3, where)
        need(o, "half_size", 3, where)
        if not isinstance(o.get("mass", 0.2), (int, float)) or o.get("mass", 0.2) <= 0:
            raise SiteError(f"{where}: mass must be positive")
        marker = o.get("marker")
        if marker is not None:
            check_marker(marker, where, False)
            if marker.setdefault("face", "-x") not in FACE_NORMALS:
                raise SiteError(f"{where}: marker face must be one of {sorted(FACE_NORMALS)}")
            # the plate (marker plus its white border) fills the smaller side of the face
            plate = float(min(np.delete(o["half_size"], int(np.argmax(np.abs(FACE_NORMALS[marker["face"]]))))))
            marker.setdefault("size", 2 * plate / (1 + 2 * MARKER_BORDER))
        grasp = o.setdefault("grasp", {})
        if grasp.setdefault("hand", "nearest") not in HANDS:
            raise SiteError(f"{where}: grasp.hand must be one of {list(HANDS)}")
    for s in data["stations"]:
        where = name_of(s, "station")
        marker = need(s, "marker", where=where)
        check_marker(marker, where, True)
        need(marker, "pos", 3, where)
        need(marker, "facing", where=where)
        if schema == SCHEMA_2:
            d, lat, a = need(s, "stand", 3, where)
            f = float(marker["facing"])
            x = marker["pos"][0] + d * math.cos(f) - lat * math.sin(f)
            y = marker["pos"][1] + d * math.sin(f) + lat * math.cos(f)
            s["pose"] = [round(x, 6), round(y, 6), round(wrap(f + math.pi + a), 6)]
        else:
            need(s, "pose", 3, where)
    stations = {s["name"] for s in data["stations"]}
    for loc in data["locations"]:
        where = name_of(loc, "location")
        if loc.get("station") not in stations:
            raise SiteError(f"{where}: unknown station {loc.get('station')!r}")
        region = loc.get("region")
        if region is not None:
            lo, hi = need(region, "min", 3, where), need(region, "max", 3, where)
            if not all(a < b for a, b in zip(lo, hi)):
                raise SiteError(f"{where}: region min must be below max")
    kinds = {
        "robots": {data["robot"]["name"]},
        "objects": {o["name"] for o in data["objects"]},
        "locations": {loc["name"] for loc in data["locations"]},
    }
    for kind, table in data["bindings"].items():
        if kind not in kinds:
            raise SiteError(f"bindings: unknown kind {kind!r}")
        for entity, target in table.items():
            if target not in kinds[kind]:
                raise SiteError(f"bindings.{kind}: {entity!r} names {target!r}, which the site does not have")
    return data


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rot_z(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def yaw_of(R: np.ndarray) -> float:
    """The heading of a rotation matrix: the direction of its x axis seen from above."""
    return math.atan2(R[1, 0], R[0, 0])


def marker_frame(marker: dict) -> tuple[np.ndarray, np.ndarray]:
    """(R, p) of a station marker in the map: the columns of R are the marker's x (right), y (up), z (out).

    `facing` is the yaw of the plate's outward normal; `tilt` (default 0, a vertical panel) leans the plate back so its
    face looks up by that angle: a head camera that looks down sees a tilted panel more squarely."""
    f, tilt = float(marker["facing"]), float(marker.get("tilt", 0.0))
    out = np.array([math.cos(f), math.sin(f), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    z = out * math.cos(tilt) + up * math.sin(tilt)
    y = up * math.cos(tilt) - out * math.sin(tilt)
    return np.column_stack([np.cross(y, z), y, z]), np.array(marker["pos"], float)


def station(site: dict, name: str) -> dict:
    return next(s for s in site["stations"] if s["name"] == name)


def location(site: dict, name: str) -> dict:
    return next(loc for loc in site["locations"] if loc["name"] == name)


def obj(site: dict, name: str) -> dict:
    return next(o for o in site["objects"] if o["name"] == name)


def intrinsics(camera: dict) -> tuple[float, float, float, float]:
    """fx, fy, cx, cy in pixels (square pixels, principal point at the image centre) from the calibration."""
    w, h = camera["width"], camera["height"]
    f = (h / 2) / math.tan(math.radians(camera["fovy"]) / 2)
    return f, f, w / 2, h / 2


def region_centre_in_marker(loc: dict) -> np.ndarray:
    return (np.array(loc["region"]["min"], float) + np.array(loc["region"]["max"], float)) / 2
