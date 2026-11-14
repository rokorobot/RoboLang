"""Test-only ground-truth oracle (decision 0029 X6): compare observed goal verdicts with Gazebo's true state.

Never imported by the backend. Usage: oracle.py RECORD SITE OUT [SCENE]. Reads box and wrist poses from Gazebo and
the gripper joint from /joint_states (the simulator's own joint state). Exit 1 on a false success or any disagreement.
For a stop case (SCENE has `stop`, decision 0038 P8) it also checks the safe state: the arm holds still, and when
the box was held, the gripper is still closed with the box under it.
"""
import json
import math
import re
import subprocess
import sys
import time

POSITION_TOLERANCE_M = 0.02  # as robot_at in robolang_ros2.observe
HOLD_DRIFT_M, HOLD_WATCH_S = 0.01, 3.0  # a held arm moves less than 1 cm in 3 s
GRIPPER_CLOSED_RAD = 0.75
UNDER_XY_M, UNDER_Z_M = 0.1, (0.05, 0.4)  # the box centre under tool0 (tool0 sits on wrist_3_link's origin)


def _pose(args):
    out = subprocess.run(["gz", "model", *args], check=True, capture_output=True, text=True, timeout=20).stdout
    match = re.search(r"(?m)^\s*- Pose \[ XYZ[^\n]*\n\s*\[([^\]]+)\]", out)
    if not match:
        raise SystemExit(f"cannot parse pose from gz model {args}: {out!r}")
    return [float(v) for v in match.group(1).split()]


GRIPPER_READER = """
import os, sys, time
import rclpy
from sensor_msgs.msg import JointState
rclpy.init()
node = rclpy.create_node("h3_oracle")
joints = {}
node.create_subscription(JointState, "/joint_states", lambda m: joints.update(zip(m.name, m.position)), 10)
deadline = time.monotonic() + 30
while "robotiq_85_left_knuckle_joint" not in joints and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
print(joints.get("robotiq_85_left_knuckle_joint", "missing"))
sys.stdout.flush()
os._exit(0)  # rclpy can crash while shutting down; the value is already printed
"""


def _gripper() -> float:
    """The simulated knuckle joint from one /joint_states message, read in a child process (its teardown can crash)."""
    out = subprocess.run([sys.executable, "-c", GRIPPER_READER], capture_output=True, text=True, timeout=60).stdout
    try:
        return float(out.strip().splitlines()[-1])
    except (IndexError, ValueError):
        raise SystemExit(f"no gripper joint state: {out!r}") from None


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


def safe_state(record, site, stop) -> dict:
    """0038 P8 after a stop: the hold was sent, the arm stays where it stopped, and a held box was not dropped."""
    wrist = _pose(["-m", site["gazebo_model"], "--link", "wrist_3_link"])
    time.sleep(HOLD_WATCH_S)
    later = _pose(["-m", site["gazebo_model"], "--link", "wrist_3_link"])
    stop_record = record.get("stop") or {}
    out = {"hold_sent": stop_record.get("hold") == "sent",
           "stop_confirmed": bool((stop_record.get("confirmation") or {}).get("confirmed")),
           "arm_drift_m": round(math.dist(wrist, later), 4)}
    out["arm_holds"] = out["arm_drift_m"] < HOLD_DRIFT_M
    if stop["holding"]:
        box = _pose(["-m", next(iter(site["objects"].values())), "-p"])
        out["gripper"] = _gripper()
        out["box_held"] = (out["gripper"] >= GRIPPER_CLOSED_RAD and math.dist(box[:2], later[:2]) <= UNDER_XY_M and
                           UNDER_Z_M[0] <= later[2] - box[2] <= UNDER_Z_M[1])
    # H18: the backend's own confirmation must agree with the arm the oracle sees holding still.
    out["ok"] = out["hold_sent"] and out["stop_confirmed"] and out["arm_holds"] and out.get("box_held", True)
    return out


def main():
    record, site = json.load(open(sys.argv[1])), json.load(open(sys.argv[2]))
    scene = json.load(open(sys.argv[4])) if len(sys.argv) > 4 else {}
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
    if scene.get("stop"):
        result["safe_state"] = safe_state(record, site, scene["stop"])
        result["agree"] &= result["safe_state"]["ok"]
    result["ground_truth"] = cache
    json.dump(result, open(sys.argv[3], "w"), indent=2)
    print(json.dumps(result, indent=2))
    sys.exit(0 if result["agree"] and not result["false_success"] else 1)


if __name__ == "__main__":
    main()
