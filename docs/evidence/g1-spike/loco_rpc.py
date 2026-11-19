"""Spike: the official G1 LocoClient talks to a sim-side 'sport' service over CycloneDDS on loopback.

Usage: loco_rpc.py [both|serve|call]. `both` (default) runs the service and the client in one process; `serve` runs
only the service for 20 s; `call` runs only the client, so the two can be separate processes.
"""

import json
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco import g1_loco_api as api
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.rpc.server import Server

mode = sys.argv[1] if len(sys.argv) > 1 else "both"
ChannelFactoryInitialize(1, "lo")  # domain 1, loopback: never the real robot's domain 0
got = []


class SimLoco(Server):
    def __init__(self):
        super().__init__(api.LOCO_SERVICE_NAME)

    def Init(self):
        self._SetApiVersion(api.LOCO_API_VERSION)
        self._RegistHandler(api.ROBOT_API_ID_LOCO_SET_VELOCITY, self.set_velocity, False)
        self._RegistHandler(api.ROBOT_API_ID_LOCO_GET_FSM_ID, lambda p: (0, json.dumps({"data": 801})), False)

    def set_velocity(self, parameter):
        got.append(json.loads(parameter))
        print("service received", parameter, flush=True)
        return 0, ""


if mode in ("both", "serve"):
    srv = SimLoco()
    srv.Init()
    srv.Start()
    if mode == "serve":
        time.sleep(20)
        sys.exit(0)
time.sleep(0.5)
cli = LocoClient()
cli.SetTimeout(3.0)
cli.Init()
time.sleep(0.5)
t = time.perf_counter()
code = cli.SetVelocity(0.4, 0.0, 0.0, 2.0)
dt = time.perf_counter() - t
code2, fsm = cli.GetFsmId()
print(
    json.dumps(
        {
            "mode": mode,
            "set_velocity_code": code,
            "round_trip_ms": round(dt * 1000, 1),
            "received_in_this_process": got,
            "get_fsm_id": [code2, fsm],
        }
    )
)
