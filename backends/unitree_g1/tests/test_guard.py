"""The guard (decision 0046 U9, S501): the network refusals need no DDS participant and no stand-in."""
import json
import subprocess
import sys
import time
from pathlib import Path

import pytest

from robolang_g1 import guard

pytest.importorskip("unitree_sdk2py")


def run_guard(code: str) -> dict:
    """Run `code` in a fresh process (the DDS channel factory is a per-process singleton)."""
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout.strip().splitlines()[-1])


CHECK = """
import json
from robolang_g1 import guard
from unitree_sdk2py.core.channel import ChannelFactory
try:
    guard.require_simulation({args})
    result = {{"refused": False}}
except guard.GuardError as exc:
    result = {{"refused": True, "code": exc.code, "reason": exc.reason}}
result["participant_created"] = bool(ChannelFactory._ChannelFactory__initialized)
print(json.dumps(result))
"""


@pytest.mark.parametrize("args", ["domain=0", "interface='eth0'", "interface='127.0.0.1'", "domain=0, interface='lo'",
                                  "domain=-1"])
def test_refuses_before_any_dds_participant(args):
    result = run_guard(CHECK.format(args=args))
    assert result["refused"] is True
    assert result["code"] == "S501"
    assert result["reason"]
    assert result["participant_created"] is False


def test_network_check_is_pure():
    guard.check_network(1, "lo")  # accepted: nothing is created
    guard.check_network(7, "lo")
    for domain, interface in ((0, "lo"), (1, "eth0"), (1, ""), (True, "lo"), ("1", "lo")):
        with pytest.raises(guard.GuardError) as caught:
            guard.check_network(domain, interface)
        assert caught.value.code == "S501"


def test_no_way_to_switch_it_off():
    import inspect

    assert set(inspect.signature(guard.require_simulation).parameters) == {"domain", "interface", "timeout",
                                                                           "min_advance"}


class FakeStandIn:
    """Answers the identity API with a clock that moves `step` simulated seconds per call."""

    def __init__(self, step):
        self.t, self.step = 5.0, step

    def Identity(self):
        self.t += self.step
        return 0, {"simulation": True, "domain": 1, "sim_time": self.t, "stance_model": "free"}


@pytest.fixture
def fake_dds(monkeypatch):
    import unitree_sdk2py.core.channel as channel

    monkeypatch.setattr(channel, "ChannelFactoryInitialize", lambda *a, **k: None)

    def use(step):
        fake = FakeStandIn(step)
        monkeypatch.setattr(guard, "sim_client", lambda: fake)
        return fake

    return use


def test_a_slow_but_moving_clock_is_accepted(fake_dds):
    """A stand-in still warming up (here 0.02 simulated s per 0.1 s poll, a fifth of real time) is a simulation."""
    fake_dds(0.02)
    client, identity = guard.require_simulation(timeout=5.0)
    assert identity["simulation"] is True


def test_a_frozen_clock_is_refused(fake_dds):
    fake_dds(0.0)
    with pytest.raises(guard.GuardError) as caught:
        guard.require_simulation(timeout=1.0)
    assert caught.value.code == "S501" and "does not advance" in caught.value.reason


def test_missing_identity_api_is_refused_and_no_command_is_sent(tmp_path):
    """A robot that answers `sport` but has no `robolang_sim` service: refused, and it never saw a request."""
    count_file = tmp_path / "requests.json"
    robot = subprocess.Popen([sys.executable, "-m", "robolang_g1.harness", "--plain-robot", str(count_file)],
                             stdout=subprocess.PIPE, text=True)
    try:
        for line in robot.stdout:
            if line.strip() == "ready":
                break
        result = run_guard(CHECK.format(args="timeout=3.0"))
        time.sleep(0.3)
        assert result["refused"] is True and result["code"] == "S501"
        assert "identity" in result["reason"]
        assert json.loads(Path(count_file).read_text()) == []
    finally:
        robot.terminate()
        robot.wait(timeout=10)
