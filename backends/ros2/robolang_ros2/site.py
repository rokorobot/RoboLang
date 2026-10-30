"""Small, closed site file: mappings from validated task IDs to Gazebo and MoveIt targets."""
from __future__ import annotations

import json
import math
import re
from pathlib import Path

from .executor import BackendFailure, Site

_NAME = re.compile(r"[A-Za-z][A-Za-z0-9_]*\Z")


def read_site(path: str | Path) -> Site:
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"), object_pairs_hook=_unique_pairs)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise BackendFailure("S507", f"cannot read site: {exc}") from exc
    if not isinstance(data, dict) or set(data) != {"gazebo_model", "robot", "objects", "poses", "grasps"}:
        raise BackendFailure("S507", "site must have gazebo_model, robot, objects, poses and grasps")
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
    return Site(data["gazebo_model"], data["robot"], objects, poses, grasps)


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
