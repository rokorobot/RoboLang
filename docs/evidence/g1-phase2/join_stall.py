# ruff: noqa: E501
"""Does a DDS client joining the domain stall the stand-in's loop? Start a stand-in, let it run 8 s alone, then join with
a client (the guard creates a participant and calls the identity API), and read the stand-in's slow steps (> 20 ms) with
their simulation times. Usage: python docs/evidence/g1-phase2/join_stall.py [--tail] [--early]"""

import json
import tempfile
import time
from pathlib import Path

from robolang_g1 import guard
import sys

from robolang_g1.harness import OracleTail, StandIn
from robolang_g1.robot import Robot

WITH_ORACLE_TAIL = "--tail" in sys.argv  # also read the oracle file every 8 ms, as the checks do

site = Path(__file__).resolve().parents[3] / "backends/unitree_g1/robolang_g1/sites/pick_place.json"
run = Path(tempfile.mkdtemp(prefix="join_"))
si = StandIn(str(site), run, "free")
si.wait_ready()
tail = OracleTail(si.oracle) if WITH_ORACLE_TAIL else None
if tail:
    tail.start()
time.sleep(0.5 if "--early" in sys.argv else 8)  # alone: no client has joined (--early: join at once)
t_join = time.monotonic()
client, identity = guard.require_simulation(timeout=30)  # the first participant joins here
joined_at = identity["sim_time"]
robot = Robot()  # more readers and writers: subscriptions to lowstate, hand states; publishers
robot.wait_state()
time.sleep(4)
status = client.Status()[1]
if tail:
    tail.stop()
final = si.stop()
print(json.dumps({"sim_time_when_the_client_joined": round(joined_at, 2), "join_took_s": round(time.monotonic() - t_join, 2),
                  "slow_steps_sim_time_ms_gc_collections": final["gc"]["slow_steps"], "step_compute_ms": final["step_compute_ms"]}))
