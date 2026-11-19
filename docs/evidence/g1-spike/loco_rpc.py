"""Spike: the official G1 LocoClient talks to a sim-side 'sport' service over CycloneDDS on loopback."""

import json
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.rpc.server import Server
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.loco import g1_loco_api as api

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
        return 0, ""


srv = SimLoco()
srv.Init()
srv.Start()
time.sleep(0.5)
cli = LocoClient()
cli.SetTimeout(3.0)
cli.Init()
t = time.perf_counter()
code = cli.SetVelocity(0.4, 0.0, 0.0, 2.0)
dt = time.perf_counter() - t
code2, fsm = cli.GetFsmId() if hasattr(cli, "GetFsmId") else (None, None)
print(
    json.dumps(
        {
            "set_velocity_code": code,
            "round_trip_ms": round(dt * 1000, 1),
            "server_received": got,
            "get_fsm_id": [code2, fsm],
        }
    )
)
