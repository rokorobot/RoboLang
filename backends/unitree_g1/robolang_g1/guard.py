"""The simulation guard (decision 0046, U9). Failure code S501.

Before the backend sends any command to a robot it requires, in this order:

1. DDS domain other than 0, and the loopback interface `lo`. This is checked **before any DDS participant exists**:
   a refusal here sends nothing anywhere, because nothing was ever created. (The real G1 is on domain 0.)
2. An answer from the simulation-identity API of the stand-in (service `robolang_sim`) within a timeout. Only the
   stand-in implements it; a real robot does not, so its absence is the refusal.
3. `simulation: true`, the same domain, and a simulation clock that advances.

There is no flag, environment variable or argument that skips a step.
"""
from __future__ import annotations

import json
import time

from . import API_ATTACH, API_DETACH, API_IDENTITY, API_STATUS, DOMAIN, INTERFACE, SIM_API_VERSION, SIM_SERVICE

CODE = "S501"


class GuardError(RuntimeError):
    """The backend refuses to run: the other end is not a simulation, or the network is not the simulation's."""

    code = CODE

    def __init__(self, reason: str):
        super().__init__(f"{CODE}: {reason}")
        self.reason = reason


def check_network(domain: int, interface: str) -> None:
    """Step 1. Pure: creates no DDS participant. Raises GuardError for domain 0 or any interface but `lo`."""
    if not isinstance(domain, int) or isinstance(domain, bool) or domain < 0:
        raise GuardError(f"invalid DDS domain {domain!r}")
    if domain == 0:
        raise GuardError("DDS domain 0 is the real robot's domain; the simulation backend never uses it")
    if interface != INTERFACE:
        raise GuardError(f"network interface {interface!r} is not the loopback interface {INTERFACE!r}")


def sim_client():
    """A client of the stand-in's `robolang_sim` service (needs ChannelFactoryInitialize to have run)."""
    from unitree_sdk2py.rpc.client import Client

    class SimClient(Client):
        def __init__(self):
            super().__init__(SIM_SERVICE, False)

        def Init(self):
            self._SetApiVerson(SIM_API_VERSION)
            for api in (API_IDENTITY, API_STATUS, API_ATTACH, API_DETACH):
                self._RegistApi(api, 0)

        def _json(self, api, parameter=None):
            code, data = self._Call(api, json.dumps(parameter or {}))
            return code, (json.loads(data) if code == 0 and data else None)

        def Identity(self):
            """(code, {"simulation": true, "domain": 1, "sim_time": t, "stance_model": ...})"""
            return self._json(API_IDENTITY)

        def Status(self):
            """(code, real-time ratio, camera frames, stance phase ...)"""
            return self._json(API_STATUS)

        def Attach(self, obj: str, hand: str, teleport: bool = False):
            """Weld an object to a hand ('left' or 'right'); `teleport` first moves it to the palm."""
            return self._json(API_ATTACH, {"object": obj, "hand": hand, "teleport": teleport})

        def Detach(self, obj: str):
            return self._json(API_DETACH, {"object": obj})

    client = SimClient()
    client.SetTimeout(1.0)
    client.Init()
    return client


def require_simulation(domain: int = DOMAIN, interface: str = INTERFACE, timeout: float = 8.0,
                       advance_after: float = 0.4):
    """Run the guard. Returns `(client, identity)` where `client` is the stand-in's `SimClient`.

    Initialises the DDS channel factory (once per process) only after the network checks passed.
    """
    check_network(domain, interface)
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(domain, interface)
    client = sim_client()
    deadline = time.monotonic() + timeout
    identity = None
    while time.monotonic() < deadline and identity is None:
        code, identity = client.Identity()
        if code != 0:
            identity = None
            time.sleep(0.2)
    if identity is None:
        raise GuardError(f"no answer from the simulation identity API ({SIM_SERVICE}) within {timeout} s: "
                         "this is not the simulator")
    if identity.get("simulation") is not True:
        raise GuardError(f"the identity API says it is not a simulation: {identity}")
    if identity.get("domain") != domain:
        raise GuardError(f"the identity API is on domain {identity.get('domain')}, expected {domain}")
    t0, wall0 = identity["sim_time"], time.monotonic()
    time.sleep(advance_after)
    code, later = client.Identity()
    if code != 0 or later is None:
        raise GuardError("the simulation identity API stopped answering")
    if not later["sim_time"] > t0 + 0.25 * (time.monotonic() - wall0):
        raise GuardError(f"simulation time does not advance ({t0} then {later['sim_time']})")
    return client, later
