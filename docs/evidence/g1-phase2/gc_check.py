"""Does Python's garbage collector explain the stand-in's occasional long step (`step_compute_ms.max`)?

    python docs/evidence/g1-phase2/gc_check.py --out gc.json [--no-gc-freeze] [--seconds 90]

Starts the stand-in (camera streaming, as in every run), drives it through the SDK for `--seconds` with the work a
task does (walking, arm_sdk at 50 Hz, a hand), then prints the stand-in's `step_compute_ms` and `gc` status: how many
collections of each generation ran and the longest one. With --no-gc-freeze the stand-in leaves the collector as it
was before phase 2 (but still times it).
"""
import argparse
import json
import tempfile
import time
from pathlib import Path

import numpy as np

from robolang_g1 import assets, guard
from robolang_g1.harness import StandIn
from robolang_g1.robot import Robot, arm_gains
from robolang_g1.sim.policy import load_deploy

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--seconds", type=float, default=90)
ap.add_argument("--no-gc-freeze", action="store_true")
ap.add_argument("--site", default=str(Path(__file__).resolve().parents[3] / "backends/unitree_g1/robolang_g1/sites"
                                       / "phase1.json"))
args = ap.parse_args()
run_dir = Path(tempfile.mkdtemp(prefix="gc_check_"))
standin = StandIn(args.site, run_dir, extra_args=["--no-gc-freeze"] if args.no_gc_freeze else [])
standin.wait_ready()
client, _ = guard.require_simulation(timeout=30)
robot = Robot()
robot.wait_state()
cfg = load_deploy(assets.asset(*assets.POLICY_DIR, "params", "deploy.yaml"))
default = np.zeros(29)
default[cfg["joint_ids_map"]] = cfg["default_joint_pos"]
kp, kd = arm_gains(cfg["stiffness"], cfg["damping"])
robot.start_arm_sdk()
t_end = time.monotonic() + args.seconds
k = 0
while time.monotonic() < t_end:
    reach = default.copy()
    reach[22], reach[25] = -0.9 * (k % 2), 0.6 * (k % 2)
    robot.ramp_arm(reach, kp, kd, 1.0, 1.0, q_from=default if k == 0 else None)
    robot.set_velocity(0.3, 0, 0.1, 3.0)
    time.sleep(4)
    k += 1
robot.stop_arm_sdk()
status = client.Status()[1]
final = standin.stop()
Path(args.out).write_text(json.dumps({"no_gc_freeze": args.no_gc_freeze, "seconds": args.seconds,
                                      "step_compute_ms": final["step_compute_ms"], "gc": final["gc"],
                                      "realtime_ratio": final["realtime_ratio"], "stalls": final["stalls"],
                                      "worst_second_ratio": final["worst_second_ratio"]}, indent=2))
print(json.dumps({k: final[k] for k in ("step_compute_ms", "gc", "stalls", "worst_second_ratio")}))
