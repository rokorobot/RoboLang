"""The robot's own kinematic model: forward kinematics from measured joints, and inverse kinematics for the arms.

A **separate MjModel** of the robot (no floor, no objects, its own MjData) used as a kinematics library. It is fed
with joints measured on `rt/lowstate`; it never sees, and is never given, the stand-in's MuJoCo state. The model is the
robot's description (Unitree's MJCF, as the stand-in's own robot) plus the head camera at the calibration the site
gives. The pelvis is the root of every pose here: the pelvis frame has x forward, y left, z up, origin at the pelvis.

Inverse kinematics is damped least squares on MuJoCo's own Jacobians (decision 0046 U10: no IK library). The waist is
held at its measured joints (the arms' base does not move); each arm is 7 joints. The target is the hand's **grasp
frame**: its origin is the pinch point of the Dex3-1 hand (between the thumb and the curling fingers, in the wrist link
frame, thumb side y: +0.04 for the right hand, -0.04 for the left), its x axis points along the fingers.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np

from .sim import scene

PINCH = {"right": np.array([0.10, 0.04, 0.0]), "left": np.array([0.10, -0.04, 0.0])}  # wrist-link frame, m
# The grasp frame that points the fingers down with the thumb toward the body's midline and the finger pair
# (fore and aft) along the pelvis x axis. Columns: the hand's x (fingers), y (thumb side, for the right hand), z.
TOP_DOWN = np.array([[0, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0]])  # rows are pelvis x, y, z; columns the hand's axes
ARM_JOINTS = {"left": list(range(15, 22)), "right": list(range(22, 29))}  # SDK joint indices of each arm
WAIST = [12, 13, 14]
IK_TOL_M, IK_TOL_RAD = 0.01, 0.35  # accepted: position within 1 cm, orientation within 0.35 rad (20 degrees)
WAIST_PITCH = 14  # the one waist joint IK may use, leaning forward only and only as far as `waist_max`
WAIST_COST = 0.3  # its Jacobian column is scaled by this, so the solver reaches with the arm first


class Kinematics:
    """FK and IK on a private copy of the robot. Not thread-safe: one user at a time."""

    def __init__(self, camera: dict | None = None):
        spec = scene.robot_spec(camera)
        for side in ("left", "right"):
            body = spec.body(scene.HAND_BODY[side])
            body.add_site(name=f"grasp_{side}", pos=list(PINCH[side]), size=[0.005, 0, 0])
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.idx = scene.Robot.of(self.model)
        m = self.model
        self.limits = np.array([m.jnt_range[m.joint(j).id] for j in scene.BODY_JOINTS])  # (29, 2)
        self.site = {s: m.site(f"grasp_{s}").id for s in ("left", "right")}
        self.camera_id = m.camera("head").id
        self.palm = {s: m.body(scene.HAND_BODY[s]).id for s in ("left", "right")}
        self._jacp = np.zeros((3, m.nv))
        self._jacr = np.zeros((3, m.nv))
        self.set_pose(np.zeros(29))

    # ------------------------------------------------------------------------------------------ forward

    def set_pose(self, q29, hand_q: dict | None = None) -> None:
        """Pelvis at the origin, upright; the 29 body joints (SDK order) from `q29` (and the hands if given)."""
        d, r = self.data, self.idx
        d.qpos[0:3] = 0
        d.qpos[3:7] = (1, 0, 0, 0)
        d.qpos[r.qadr] = q29
        for side, q in (hand_q or {}).items():
            d.qpos[r.hand_qadr[side]] = q
        mujoco.mj_kinematics(self.model, d)
        mujoco.mj_comPos(self.model, d)

    def camera_pose(self, q29) -> tuple[np.ndarray, np.ndarray]:
        """The head camera in the pelvis frame as (R, p), MuJoCo convention (looks along -z, +y up)."""
        self.set_pose(q29)
        mujoco.mj_camlight(self.model, self.data)
        return self.data.cam_xmat[self.camera_id].reshape(3, 3).copy(), self.data.cam_xpos[self.camera_id].copy()

    def grasp_frame(self, side: str, q29) -> tuple[np.ndarray, np.ndarray]:
        """The hand's grasp frame in the pelvis frame as (R, p), from the 29 body joints (measured)."""
        self.set_pose(q29)
        return self.data.site_xmat[self.site[side]].reshape(3, 3).copy(), self.data.site_xpos[self.site[side]].copy()

    # ------------------------------------------------------------------------------------------ inverse

    def solve(
        self,
        side: str,
        p_target,
        R_target,
        q_seed,
        rot_weight: float = 0.25,
        iters: int = 300,
        posture: np.ndarray | None = None,
        waist_max: float = 0.0,
    ) -> tuple[np.ndarray | None, dict]:
        """Damped least squares for the arm `side`: the grasp frame at (R_target, p_target) in the pelvis frame.

        `q_seed` (29 joints) starts the search and gives every other joint its value. The waist is held, except that
        with `waist_max` > 0 the waist pitch may lean forward by up to that much (at a higher cost than the arm's
        joints). `posture` (29) is a pose the redundant joints are gently pulled towards (the elbow-down arm). Returns
        `(q29, info)` with the 29 joints, or `(None, info)` when it did not converge within the tolerances.
        """
        d, m = self.data, self.model
        q = np.array(q_seed, float)
        joints = ARM_JOINTS[side] + ([WAIST_PITCH] if waist_max > 0 else [])
        dofs = self.idx.dadr[joints]
        lo, hi = self.limits[joints, 0].copy(), self.limits[joints, 1].copy()
        scale = np.ones(len(joints))
        if waist_max > 0:
            lo[-1], hi[-1], scale[-1] = 0.0, waist_max, WAIST_COST
        home = np.array(posture if posture is not None else q_seed, float)
        p_target, R_target = np.asarray(p_target, float), np.asarray(R_target, float)
        lam = 0.05
        err_p = err_r = float("inf")
        for it in range(iters):
            self.set_pose(q)
            p = d.site_xpos[self.site[side]]
            R = d.site_xmat[self.site[side]].reshape(3, 3)
            err_p_vec = p_target - p
            rot_err = _rotation_error(R_target, R)
            err_p, err_r = float(np.linalg.norm(err_p_vec)), float(np.linalg.norm(rot_err))
            if err_p < IK_TOL_M * 0.5 and err_r < 0.05:
                break
            mujoco.mj_jacSite(m, d, self._jacp, self._jacr, self.site[side])
            J = np.vstack([self._jacp[:, dofs], rot_weight * self._jacr[:, dofs]]) * scale
            e = np.concatenate([err_p_vec, rot_weight * rot_err])
            free = np.ones(len(joints), bool)  # joints held at a limit that the step would push through are dropped
            for _ in range(len(joints)):
                Jf = J[:, free]
                JJt = Jf @ Jf.T + lam**2 * np.eye(6)
                step_f = Jf.T @ np.linalg.solve(JJt, e)
                null = np.eye(int(free.sum())) - Jf.T @ np.linalg.solve(
                    JJt, Jf
                )  # pull to the posture in the null space
                step_f += null @ (0.15 * (home[joints] - q[joints])[free] / scale[free])
                dq = np.zeros(len(joints))
                dq[free] = step_f * scale[free]
                new = q[joints] + dq
                over = ((new < lo) & (dq < 0)) | ((new > hi) & (dq > 0))
                if not over.any():
                    break
                free &= ~over
            top = np.max(np.abs(dq))
            if top > 0.15:
                dq *= 0.15 / top
            q[joints] = np.clip(q[joints] + dq, lo, hi)
        info = {"position_error_m": err_p, "orientation_error_rad": err_r, "iterations": it + 1}
        ok = err_p <= IK_TOL_M and err_r <= IK_TOL_RAD
        return (q if ok else None), info


def _rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rotation_error(R_target: np.ndarray, R: np.ndarray) -> np.ndarray:
    """The rotation vector (axis x angle) that takes R to R_target, in the world frame."""
    E = R_target @ R.T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(E) - 1) / 2)))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    norm = np.linalg.norm(axis)
    if norm < 1e-9:  # a half turn: any axis perpendicular to the error will do
        return np.array([math.pi, 0, 0])
    return axis / norm * angle
