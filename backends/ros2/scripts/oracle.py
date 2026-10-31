"""Test-only ground-truth oracle (decision 0029 X6): compare observed goal verdicts with Gazebo's true state.

Never imported by the backend. Usage: oracle.py RECORD SITE OUT. Reads box and wrist poses from Gazebo and the
gripper joint from /joint_states (the simulator's own joint state). Exit 1 on a false success or any disagreement.
"""
import json
import math
import re
import subprocess
import sys

import yaml

POSITION_TOLERANCE_M = 0.02  # as robot_at in robolang_ros2.observe
GRIPPER_CLOSED_RAD = 0.75
UNDER_XY_M, UNDER_Z_M = 0.1, (0.05, 0.4)  # the box centre under tool0 (tool0 sits on wrist_3_link's origin)


def _pose(args):
    out = subprocess.run(["gz", "model", *args], check=True, capture_output=True, text=True, timeout=20).stdout
    match = re.search(r"(?m)^\s*- Pose \[ XYZ[^\n]*\n\s*\[([^\]]+)\]", out)
    if not match:
        raise SystemExit(f"cannot parse pose from gz model {args}: {out!r}")
    return [float(v) for v in match.group(1).split()]


def _gripper() -> float:
    out = subprocess.run(["ros2", "topic", "echo", "--once", "/joint_states"], check=True, capture_output=True,
                         text=True, timeout=30).stdout
    message = next(doc for doc in yaml.safe_load_all(out) if doc)
    return dict(zip(message["name"], message["position"]))["robotiq_85_left_knuckle_joint"]


def truth(condition, bound, site, cache):
    if "box" not in cache:
        cache["box"] = {m: _pose(["-m", m, "-p"]) for m in site["objects"].values()}
        cache["wrist"] = _pose(["-m", site["gazebo_model"], "--link", "wrist_3_link"])
    kind = condition["kind"]
    if kind == "object_at":
        box = cache["box"][site["objects"][bound[condition["object"]]]]
        region = site["regions"][bound[condition["location"]]]
        return all(lo <= x <= hi for x, lo, hi in zip(box, region["min"], region["max"]))
    if kind == "robot_at":
        target = site["poses"][bound[condition["location"]]]["position"]
        return math.dist(cache["wrist"], target) <= POSITION_TOLERANCE_M
    if kind == "holding":
        if "gripper" not in cache:
            cache["gripper"] = _gripper()
        box, wrist = cache["box"][site["objects"][bound[condition["object"]]]], cache["wrist"]
        return (cache["gripper"] >= GRIPPER_CLOSED_RAD and math.dist(box[:2], wrist[:2]) <= UNDER_XY_M and
                UNDER_Z_M[0] <= wrist[2] - box[2] <= UNDER_Z_M[1])
    raise SystemExit(f"the oracle has no ground truth for {kind}")


def main():
    record, site = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))
    bound, cache = record["bindings"], {}
    result = {"state": record["state"], "failure": record["failure"], "goal": [], "false_success": False,
              "agree": True}
    for entry in record["goal"]:
        if entry["result"] == "not_evaluated":
            continue
        actual = truth(entry["condition"], bound, site, cache)
        agree = entry["result"] == ("true" if actual else "false")
        result["goal"].append({"index": entry["index"], "condition": entry["condition"], "observed": entry["result"],
                               "oracle": actual, "agree": agree})
        result["agree"] &= agree
        result["false_success"] |= entry["result"] == "true" and not actual
    result["false_success"] |= record["state"] == "SUCCEEDED" and not all(g["oracle"] for g in result["goal"])
    result["ground_truth"] = cache
    json.dump(result, open(sys.argv[3], "w"), indent=2)
    print(json.dumps(result, indent=2))
    sys.exit(0 if result["agree"] and not result["false_success"] else 1)


if __name__ == "__main__":
    main()
