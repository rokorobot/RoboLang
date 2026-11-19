"""The supported stance and the simulator around it, without DDS: the robot never falls, stops, and walks again."""
import pytest
from conftest import SITE, needs_assets

from robolang_g1.sim import stance
from robolang_g1.sim.standin import ArmCommand, Simulator

pytestmark = needs_assets


@pytest.fixture(scope="module")
def sim_factory():
    return lambda: Simulator(SITE, camera=False)


def run(sim, seconds):
    for _ in range(round(seconds / sim.model.opt.timestep)):
        sim.step()


def test_stance_interface_and_registry():
    assert set(stance.STANCES) == {"supported"}
    assert stance.STANCES["supported"].name == "supported"
    with pytest.raises(ValueError, match="unknown stance"):
        stance.make_stance("free", None)


def test_starts_held_walks_stops_settles_and_walks_again(sim_factory):
    sim = sim_factory()
    run(sim, 1.0)
    assert sim.stance.phase == "hold" and sim.stance.standing and sim.fell_at is None
    sim.set_velocity(0.5, 0, 0, 2.0)
    run(sim, 0.5)
    assert sim.stance.phase == "walking" and not sim.stance.standing
    run(sim, 2.0)  # the velocity ran out after 2 s of simulation time
    assert sim._command() == (0.0, 0.0, 0.0)
    seen = []
    for _ in range(round(8.0 / sim.model.opt.timestep)):
        sim.step()
        if not seen or seen[-1] != sim.stance.phase:
            seen.append(sim.stance.phase)
    assert seen[-4:] == ["stopping", "raising", "easing", "hold"]
    assert sim.fell_at is None
    sim.set_velocity(0.5, 0, 0, 1.0)  # the next non-zero velocity releases the hold
    run(sim, 1.0)
    assert sim.stance.phase in ("walking", "stopping") and sim.fell_at is None


def test_velocity_is_clipped_to_the_policy_range_and_expires(sim_factory):
    sim = sim_factory()
    sim.set_velocity(5.0, -5.0, 5.0, 1.0)
    assert sim.last_command == (1.0, -0.3, 0.2)
    run(sim, 1.2)
    assert sim._command() == (0.0, 0.0, 0.0)


def test_arm_sdk_blend_moves_the_arm_and_hold_keeps_the_torso(sim_factory):
    import numpy as np

    sim = sim_factory()
    run(sim, 0.5)
    target = sim.policy.default.copy()
    target[22] = -0.6  # right shoulder pitch
    kp = np.full(29, 100.0)
    kd = np.full(29, 3.0)
    p0 = sim.data.xpos[sim.robot.pelvis].copy()
    for w in np.linspace(0, 1, 100):
        sim.set_arm(ArmCommand(float(w), target, np.zeros(29), kp, kd, np.zeros(29)))
        run(sim, 0.01)
    run(sim, 1.5)
    q = sim.data.qpos[sim.robot.qadr]
    assert abs(q[22] - (-0.6)) < 0.1
    assert np.linalg.norm(sim.data.xpos[sim.robot.pelvis] - p0) < 0.02 and sim.fell_at is None


def test_attach_and_detach_an_object(sim_factory):
    sim = sim_factory()
    run(sim, 0.5)
    info = sim.attach("payload", "right", teleport=True)
    assert info["hand"] == "right" and "payload" in sim.attached
    with pytest.raises(ValueError):
        sim.attach("payload", "left")
    run(sim, 1.0)
    palm = sim.model.body("right_wrist_yaw_link").id
    body = sim.model.body("payload").id
    assert abs(sim.data.xpos[body] - sim.data.xpos[palm]).max() < 0.2  # it stays with the hand
    sim.detach("payload")
    assert not sim.attached
    with pytest.raises(ValueError):
        sim.detach("payload")
