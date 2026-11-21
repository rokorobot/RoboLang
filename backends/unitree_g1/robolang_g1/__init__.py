"""RoboLang G1 backend (simulation only): the Unitree G1 humanoid in MuJoCo behind Unitree's own SDK (decision 0046)."""

__version__ = "0.1.0"

# The only network the backend may use: DDS domain 1 on the loopback interface. The real robot is on domain 0.
DOMAIN = 1
INTERFACE = "lo"
# The stand-in's identity and control service. It exists only in the simulator: its absence is how the guard tells
# that the other end is not a simulation. API ids below 100 belong to the SDK's RPC layer.
SIM_SERVICE = "robolang_sim"
SIM_API_VERSION = "1.0.0.0"
API_IDENTITY, API_STATUS, API_ATTACH, API_DETACH = 1001, 1002, 1003, 1004
API_SET_STANCE = 1006  # the stand-in's balance support: free or supported (the approved fallback, 0046 U4)
# Test hook, answered only by a stand-in started with --test-hooks (the executor never calls it): push the torso.
API_PUSH = 1005
# Error code the stand-in's `sport` service returns for a request it refuses (not a code the robot uses).
ERR_REFUSED = 7400
