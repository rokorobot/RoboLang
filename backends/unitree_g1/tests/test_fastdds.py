"""The stand-in publishes `rt/lowstate` and the hand states from pre-serialized buffers (sim/fastdds.py): the bytes must
be the ones the SDK's own serializer writes, whatever the values."""

import numpy as np
import pytest

pytest.importorskip("unitree_sdk2py")

from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandState_, unitree_hg_msg_dds__LowState_  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_, LowState_  # noqa: E402

from robolang_g1.sim.fastdds import PatchedSample  # noqa: E402

N = 29


def fast_low():
    low = unitree_hg_msg_dds__LowState_()
    low.mode_machine = 5
    for m in low.motor_state[:N]:
        m.mode = 1
    ms, imu = low.motor_state, low.imu_state
    fast = PatchedSample(
        low,
        {
            "q": [lambda v, i=i: setattr(ms[i], "q", v) for i in range(N)],
            "dq": [lambda v, i=i: setattr(ms[i], "dq", v) for i in range(N)],
            "tau": [lambda v, i=i: setattr(ms[i], "tau_est", v) for i in range(N)],
            "quat": [lambda v, i=i: imu.quaternion.__setitem__(i, v) for i in range(4)],
            "gyro": [lambda v, i=i: imu.gyroscope.__setitem__(i, v) for i in range(3)],
            "acc": [lambda v, i=i: imu.accelerometer.__setitem__(i, v) for i in range(3)],
            "rpy": [lambda v, i=i: imu.rpy.__setitem__(i, v) for i in range(3)],
        },
        {"tick": [lambda v: setattr(low, "tick", v)]},
    )
    return low, fast


def reference_low(q, dq, tau, quat, gyro, acc, rpy, tick):
    """The sample as the stand-in filled it before: every field set in Python, then the SDK's serializer."""
    low = unitree_hg_msg_dds__LowState_()
    low.mode_machine = 5
    for i in range(N):
        m = low.motor_state[i]
        m.mode, m.q, m.dq, m.tau_est = 1, float(q[i]), float(dq[i]), float(tau[i])
    low.imu_state.quaternion[:] = quat
    low.imu_state.gyroscope[:] = gyro
    low.imu_state.accelerometer[:] = acc
    low.imu_state.rpy[:] = rpy
    low.tick = tick
    return low


def test_a_patched_lowstate_is_the_same_bytes_as_the_sdk_serializes():
    rng = np.random.default_rng(0)
    low, fast = fast_low()
    for _ in range(20):
        q, dq, tau = rng.normal(0, 1, N), rng.normal(0, 3, N), rng.normal(0, 20, N)
        quat, gyro, acc, rpy = rng.normal(0, 1, 4), rng.normal(0, 1, 3), rng.normal(0, 5, 3), rng.normal(0, 1, 3)
        tick = int(rng.integers(0, 2**32))
        fast.set("q", q)
        fast.set("dq", dq)
        fast.set("tau", tau)
        fast.set("quat", quat)
        fast.set("gyro", gyro)
        fast.set("acc", acc)
        fast.set("rpy", rpy)
        fast.set_int("tick", tick)
        assert low.serialize() == reference_low(q, dq, tau, quat, gyro, acc, rpy, tick).serialize()


def test_a_patched_lowstate_reads_back_through_the_sdks_own_deserializer():
    low, fast = fast_low()
    q = np.linspace(-1, 1, N)
    fast.set("q", q)
    fast.set("dq", -q)
    fast.set("tau", 2 * q)
    fast.set("quat", [0.5, 0.5, 0.5, 0.5])
    fast.set("gyro", [0.1, 0.2, 0.3])
    fast.set("acc", [0.0, 0.0, 9.81])
    fast.set("rpy", [0.0, 0.0, 1.0])
    fast.set_int("tick", 123456)
    back = LowState_.deserialize(low.serialize())
    assert back.tick == 123456 and back.mode_machine == 5
    assert [m.q for m in back.motor_state[:N]] == pytest.approx(list(q), abs=1e-6)
    assert [m.tau_est for m in back.motor_state[:N]] == pytest.approx(list(2 * q), abs=1e-6)
    assert list(back.imu_state.gyroscope) == pytest.approx([0.1, 0.2, 0.3], abs=1e-7)
    assert all(m.mode == 1 for m in back.motor_state[:N])
    assert isinstance(low, LowState_)  # the writer's isinstance check


def test_a_patched_hand_state_is_the_same_bytes_as_the_sdk_serializes():
    rng = np.random.default_rng(1)
    state = unitree_hg_msg_dds__HandState_()
    for m in state.motor_state[:7]:
        m.mode = 1
    hm = state.motor_state
    fast = PatchedSample(
        state,
        {
            "q": [lambda v, i=i: setattr(hm[i], "q", v) for i in range(7)],
            "dq": [lambda v, i=i: setattr(hm[i], "dq", v) for i in range(7)],
            "tau": [lambda v, i=i: setattr(hm[i], "tau_est", v) for i in range(7)],
        },
    )
    for _ in range(10):
        q, dq, tau = rng.normal(0, 1, 7), rng.normal(0, 3, 7), rng.normal(0, 1, 7)
        fast.set("q", q)
        fast.set("dq", dq)
        fast.set("tau", tau)
        ref = unitree_hg_msg_dds__HandState_()
        for i in range(7):
            m = ref.motor_state[i]
            m.mode, m.q, m.dq, m.tau_est = 1, float(q[i]), float(dq[i]), float(tau[i])
        assert state.serialize() == ref.serialize()
        assert isinstance(state, HandState_)


def test_a_field_that_cannot_be_found_in_the_bytes_is_refused():
    low = unitree_hg_msg_dds__LowState_()
    with pytest.raises(ValueError):
        PatchedSample(low, {"x": [lambda v: None]})  # nothing was written anywhere: the value is not in the bytes
