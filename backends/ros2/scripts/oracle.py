"""Test-only ground-truth oracle (decision 0029 X6): compare observed goal verdicts with Gazebo's object poses.

Never imported by the backend. Usage: oracle.py RECORD SITE OUT. Exit 1 on a false success or any disagreement.
"""
import json
import math
import re
import subprocess
import sys

MARKER_HEIGHT_M = 0.0505  # the marker face above the box centre (h3_world.sdf)


def model_pose(model: str):
    out = subprocess.run(["gz", "model", "-m", model, "-p"], check=True, capture_output=True, text=True,
                         timeout=20).stdout
    match = re.search(r"(?m)^\s*- Pose \[ XYZ[^\n]*\n\s*\[([^\]]+)\]\s*\n\s*\[([^\]]+)\]", out)
    if not match:
        raise SystemExit(f"cannot parse pose of {model}: {out!r}")
    return [float(v) for v in match.group(1).split()], [float(v) for v in match.group(2).split()]


def marker_point(xyz, rpy):
    """The marker face centre: the box centre plus its rotated local +z offset (base_link is the world origin)."""
    roll, pitch, yaw = rpy
    x, y, z = math.cos(roll) * math.sin(pitch), -math.sin(roll), math.cos(roll) * math.cos(pitch)
    offset = (x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw), z)
    return [p + MARKER_HEIGHT_M * o for p, o in zip(xyz, offset)]


def main():
    record, site = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))
    bound = record["bindings"]
    result = {"goal": [], "false_success": False, "agree": True}
    for entry in record["goal"]:
        condition = entry["condition"]
        if condition["kind"] != "object_at":
            raise SystemExit("this oracle covers object_at goals only")
        model = site["objects"][bound[condition["object"]]]
        xyz, rpy = model_pose(model)
        point = marker_point(xyz, rpy)
        region = site["regions"][bound[condition["location"]]]
        truth = all(lo <= x <= hi for x, lo, hi in zip(point, region["min"], region["max"]))
        observed = entry["result"]
        agree = observed == ("true" if truth else "false")
        result["goal"].append({"index": entry["index"], "model_xyz": xyz, "model_rpy": rpy, "marker_point": point,
                               "oracle": truth, "observed": observed, "agree": agree})
        result["agree"] &= agree
        result["false_success"] |= observed == "true" and not truth
    result["false_success"] |= record["state"] == "SUCCEEDED" and not all(g["oracle"] for g in result["goal"])
    json.dump(result, open(sys.argv[3], "w"), indent=2)
    print(json.dumps(result, indent=2))
    sys.exit(0 if result["agree"] and not result["false_success"] else 1)


if __name__ == "__main__":
    main()
