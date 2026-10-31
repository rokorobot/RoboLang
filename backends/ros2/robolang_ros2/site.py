"""Small, closed site file: mappings from validated task IDs to Gazebo and MoveIt targets."""
from __future__ import annotations

import json
import math
import re
from pathlib import Path

from .executor import BackendFailure, Site

_KEYS = {"gazebo_model", "robot", "objects", "poses", "grasps", "markers", "regions", "camera"}
_NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*\Z")


def read_site(path: str | Path) -> Site:
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"), object_pairs_hook=_unique_pairs)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise BackendFailure("S507", f"cannot read site: {exc}") from exc
    if not isinstance(data, dict) or set(data) != _KEYS:
        raise BackendFailure("S507", "site must have exactly " + ", ".join(sorted(_KEYS)))
    if not all(isinstance(data[k], str) and _NAME.fullmatch(data[k]) for k in ("gazebo_model", "robot")):
        raise BackendFailure("S507", "site robot and model must be identifiers")
    objects, poses, grasps = data["objects"], data["poses"], data["grasps"]
    if not isinstance(objects, dict) or not isinstance(poses, dict) or not isinstance(grasps, dict):
        raise BackendFailure("S507", "site objects, poses and grasps must be mappings")
    for key, value in objects.items():
        if (not isinstance(key, str) or not _NAME.fullmatch(key) or
                not isinstance(value, str) or not _NAME.fullmatch(value)):
            raise BackendFailure("S507", "object IDs and model names must be identifiers")
    for key, pose in {**poses, **grasps}.items():
        if not isinstance(key, str) or not _NAME.fullmatch(key) or not isinstance(pose, dict):
            raise BackendFailure("S507", "pose entries must map location IDs to poses")
        if set(pose) != {"frame", "position", "orientation"}:
            raise BackendFailure("S507", f"pose {key} needs frame, position and orientation")
        if pose["frame"] != "base_link":
            raise BackendFailure("S507", f"pose {key} must use base_link frame")
        if not _vector(pose["position"], 3) or not _vector(pose["orientation"], 4):
            raise BackendFailure("S507", f"pose {key} has invalid coordinates")
        if abs(sum(x * x for x in pose["orientation"]) - 1) > 0.01:
            raise BackendFailure("S507", f"pose {key} orientation is not a unit quaternion")
    markers, regions, camera = data["markers"], data["regions"], data["camera"]
    if (not isinstance(markers, dict) or set(markers) - set(objects) or
            not all(isinstance(v, int) and not isinstance(v, bool) and 0 <= v < 50 for v in markers.values()) or
            len(set(markers.values())) != len(markers)):
        raise BackendFailure("S507", "markers must map site objects to distinct ArUco 4x4_50 IDs")
    if not isinstance(regions, dict):
        raise BackendFailure("S507", "regions must be a mapping")
    for key, region in regions.items():
        if (not isinstance(key, str) or not _NAME.fullmatch(key) or not isinstance(region, dict) or
                set(region) != {"frame", "min", "max"} or region["frame"] != "base_link" or
                not _vector(region["min"], 3) or not _vector(region["max"], 3) or
                not all(lo < hi for lo, hi in zip(region["min"], region["max"]))):
            raise BackendFailure("S507", f"region {key} needs a base_link frame and min below max")
    if (not isinstance(camera, dict) or
            set(camera) != {"frame", "position", "orientation", "horizontal_fov", "width", "height",
                            "marker_length"} or
            camera["frame"] != "base_link" or not _vector(camera["position"], 3) or
            not _vector(camera["orientation"], 4) or abs(sum(x * x for x in camera["orientation"]) - 1) > 0.01 or
            not _vector([camera["horizontal_fov"], camera["marker_length"]], 2) or
            not 0 < camera["horizontal_fov"] < math.pi or not 0 < camera["marker_length"] < 1 or
            not all(isinstance(camera[k], int) and not isinstance(camera[k], bool) and camera[k] > 0
                    for k in ("width", "height"))):
        raise BackendFailure("S507", "camera needs a base_link pose, field of view, image size and marker length")
    return Site(data["gazebo_model"], data["robot"], objects, poses, grasps, markers, regions, camera)


def _unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise BackendFailure("S507", f"duplicate site key {key!r}")
        result[key] = value
    return result


def _vector(value, n: int) -> bool:
    return (isinstance(value, list) and len(value) == n and
            all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in value))
