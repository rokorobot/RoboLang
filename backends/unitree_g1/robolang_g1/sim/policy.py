"""The G1 whole-body walking policy (unitree_rl_lab, `velocity/v0`, ONNX, 50 Hz), ported from the 0046 spike.

Joint order is the trap here. In `deploy.yaml`:
  * `stiffness` and `damping` are in MOTOR (SDK) order: Unitree's `State_RLBase.h` writes `motor_cmd[i].kp` from
    `joint_stiffness[i]`. The first port applied them in policy order and the robot fell within 1.5 s.
  * `default_joint_pos`, the actions and the observations are in POLICY order. `joint_ids_map[i]` is the motor index
    of policy joint i.
This class takes and returns motor-order vectors and does the mapping inside.

Observation (480 inputs): 5-step histories, oldest first, of six terms concatenated in this order: base angular
velocity x0.2 (3), projected gravity (3), velocity command (3), joint position minus default (29), joint velocity x0.05
(29), last action (29). The action is a joint position offset: target = action x 0.25 + offset, in policy order.

`deploy.yaml` is read with a few regular expressions, not a YAML library: the backend's dependencies stay those of
decision 0046 (U10), and the file is a fixed, flat export.
"""
from __future__ import annotations

import collections
import re
from pathlib import Path

import numpy as np

HISTORY = 5
TERMS = ("w", "g", "c", "q", "dq", "a")
OBS_DIM = HISTORY * (3 + 3 + 3 + 29 + 29 + 29)
N = 29
ANG_VEL_SCALE, JOINT_VEL_SCALE, ACTION_SCALE = 0.2, 0.05, 0.25
STEP_DT = 0.02  # the policy runs at 50 Hz


def _numbers(text: str, key: str, anchor: str | None = None) -> np.ndarray:
    """The flow list `key: [a, b, ...]` after `anchor` (a section name), as floats."""
    start = text.index(anchor) if anchor else 0
    found = re.compile(rf"^\s*{re.escape(key)}:\s*\[([^\]]*)\]", re.M | re.S).search(text, start)
    if not found:
        raise ValueError(f"deploy.yaml: no list {key!r}" + (f" after {anchor!r}" if anchor else ""))
    return np.array([float(x) for x in found.group(1).replace("\n", " ").split(",") if x.strip()])


def load_deploy(path: str | Path) -> dict:
    """Read the policy's `deploy.yaml`; checks the shapes and the scales this port hardcodes."""
    text = Path(path).read_text()
    cfg = {
        "joint_ids_map": _numbers(text, "joint_ids_map").astype(int),
        "stiffness": _numbers(text, "stiffness"),  # motor order
        "damping": _numbers(text, "damping"),  # motor order
        "default_joint_pos": _numbers(text, "default_joint_pos"),  # policy order
        "offset": _numbers(text, "offset", "JointPositionAction:"),  # policy order
        "scale": _numbers(text, "scale", "JointPositionAction:"),
    }
    step_dt = re.search(r"^step_dt:\s*([0-9.]+)", text, re.M)
    for k, v in cfg.items():
        if v.shape != (N,):
            raise ValueError(f"deploy.yaml: {k} has {v.shape} entries, expected {N}")
    if sorted(cfg["joint_ids_map"]) != list(range(N)):
        raise ValueError("deploy.yaml: joint_ids_map is not a permutation of 0..28")
    if not step_dt or abs(float(step_dt.group(1)) - STEP_DT) > 1e-9:
        raise ValueError("deploy.yaml: step_dt is not 0.02")
    if not np.allclose(cfg["scale"], ACTION_SCALE):
        raise ValueError("deploy.yaml: action scale is not 0.25")
    if not np.allclose(_numbers(text, "scale", "base_ang_vel:"), ANG_VEL_SCALE) or not np.allclose(
        _numbers(text, "scale", "joint_vel_rel:"), JOINT_VEL_SCALE
    ):
        raise ValueError("deploy.yaml: observation scales differ from the ones this port hardcodes")
    if int(re.search(r"history_length:\s*(\d+)", text).group(1)) != HISTORY:
        raise ValueError(f"deploy.yaml: history_length is not {HISTORY}")
    return cfg


def projected_gravity(quat_wxyz) -> np.ndarray:
    """The gravity direction in the body frame, from the IMU quaternion (w, x, y, z)."""
    w, x, y, z = quat_wxyz
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])


class WholeBodyPolicy:
    """ONNX policy plus its observation history. Vectors in and out are in motor (SDK) order, 29 entries."""

    def __init__(self, onnx_path: str | Path, deploy_path: str | Path, providers: list[str] | None = None):
        import onnxruntime as ort

        cfg = load_deploy(deploy_path)
        self.jmap = cfg["joint_ids_map"]
        self.kp = cfg["stiffness"]  # motor order
        self.kd = cfg["damping"]  # motor order
        self.default_policy = cfg["default_joint_pos"]  # policy order
        self.offset = cfg["offset"]
        self.default = np.zeros(N)  # the standing pose, in motor order
        self.default[self.jmap] = self.default_policy
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1  # the policy is small; one thread keeps it off the camera's cores
        self.session = ort.InferenceSession(str(onnx_path), options, providers=providers or ["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        if tuple(self.session.get_inputs()[0].shape[1:]) != (OBS_DIM,):
            raise ValueError(f"policy input is {self.session.get_inputs()[0].shape}, expected (1, {OBS_DIM})")
        self.history = {k: collections.deque(maxlen=HISTORY) for k in TERMS}
        self.last_action = np.zeros(N)
        self.reset()

    def reset(self) -> None:
        """Forget the history and the last action (the policy then starts from the next observation, as from rest)."""
        for h in self.history.values():
            h.clear()
        self.last_action = np.zeros(N)

    def observation(self, gyro, quat, command, q, dq) -> np.ndarray:
        """Push one observation (motor-order q and dq) and return the (1, 480) float32 policy input."""
        terms = {
            "w": np.asarray(gyro, float) * ANG_VEL_SCALE,
            "g": projected_gravity(quat),
            "c": np.asarray(command, float),
            "q": np.asarray(q, float)[self.jmap] - self.default_policy,
            "dq": np.asarray(dq, float)[self.jmap] * JOINT_VEL_SCALE,
            "a": self.last_action.copy(),
        }
        for key, v in terms.items():
            if not self.history[key]:
                self.history[key].extend([v] * HISTORY)  # the first observation fills the history
            else:
                self.history[key].append(v)
        return np.concatenate([np.concatenate(list(self.history[k])) for k in TERMS]).astype(np.float32)[None]

    def act(self, gyro, quat, command, q, dq) -> np.ndarray:
        """One policy step: the joint position targets, in motor order."""
        obs = self.observation(gyro, quat, command, q, dq)
        self.last_action = self.session.run(None, {self.input_name: obs})[0][0].astype(float)
        target = np.zeros(N)
        target[self.jmap] = self.last_action * ACTION_SCALE + self.offset
        return target
