"""The whole-body policy port: joint order, shapes, history."""
import numpy as np
import pytest
from conftest import needs_assets

from robolang_g1 import assets
from robolang_g1.sim import policy as P

pytestmark = needs_assets


@pytest.fixture(scope="module")
def deploy():
    return P.load_deploy(assets.asset(*assets.POLICY_DIR, "params", "deploy.yaml"))


@pytest.fixture(scope="module")
def pol():
    d = assets.asset(*assets.POLICY_DIR)
    return P.WholeBodyPolicy(d / "exported" / "policy.onnx", d / "params" / "deploy.yaml")


def test_deploy_shapes(deploy):
    for key in ("joint_ids_map", "stiffness", "damping", "default_joint_pos", "offset", "scale"):
        assert deploy[key].shape == (29,)
    assert sorted(deploy["joint_ids_map"]) == list(range(29))


def test_gains_are_in_motor_order_and_defaults_in_policy_order(deploy, pol):
    # motor order (SDK): 3 is the left knee (kp 150), 12-14 the waist (kp 200). Policy order has a hip roll at 3.
    assert pol.kp[3] == 150 and pol.kp[9] == 150
    assert list(pol.kp[12:15]) == [200, 200, 200]
    assert list(pol.kp[4:6]) == [40, 40]  # left ankle pitch and roll
    # the standing pose in motor order: left knee 0.3, left elbow 0.97, right shoulder roll -0.25
    assert pol.default[3] == pytest.approx(0.3)
    assert pol.default[18] == pytest.approx(0.97)
    assert pol.default[23] == pytest.approx(-0.25)
    # and it is the policy-order default scattered by joint_ids_map
    assert np.allclose(pol.default[deploy["joint_ids_map"]], deploy["default_joint_pos"])


def test_projected_gravity_upright_points_down():
    assert np.allclose(P.projected_gravity([1, 0, 0, 0]), [0, 0, -1])


def test_observation_shape_and_history(pol):
    pol.reset()
    q = pol.default.copy()
    obs = pol.observation(np.zeros(3), [1, 0, 0, 0], [0.5, 0, 0], q, np.zeros(29))
    assert obs.shape == (1, P.OBS_DIM) == (1, 480) and obs.dtype == np.float32
    # the first observation fills all five history slots; terms are laid out [w, g, c, q, dq, a], each 5 deep
    command_block = obs[0, 5 * 3 + 5 * 3: 5 * 3 + 5 * 3 + 5 * 3].reshape(5, 3)
    assert np.allclose(command_block, [0.5, 0, 0])
    obs2 = pol.observation(np.zeros(3), [1, 0, 0, 0], [0.0, 0, 0], q, np.zeros(29))
    command_block = obs2[0, 30:45].reshape(5, 3)
    assert np.allclose(command_block[:4], [0.5, 0, 0]) and np.allclose(command_block[4], 0)  # oldest first


def test_act_returns_motor_order_targets_near_the_standing_pose(pol):
    pol.reset()
    target = pol.act(np.zeros(3), [1, 0, 0, 0], [0, 0, 0], pol.default, np.zeros(29))
    assert target.shape == (29,)
    assert np.abs(target - pol.default).max() < 0.5  # a standing robot asks for a standing pose
    assert pol.last_action.shape == (29,)
