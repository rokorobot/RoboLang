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

STAND_HEIGHT = scene.STAND_HEIGHT  # the pelvis's height above the floor when the robot stands (its own standing pose)
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

    def arm_spheres(self, side: str, q29) -> list[tuple[np.ndarray, float, str]]:
        """Spheres that cover the arm `side` in the pelvis frame, from the 29 body joints: (centre, radius, link). The
        upper arm (shoulder to elbow), the forearm (elbow to wrist) and the hand (wrist to the pinch point and a little
        past it, where the fingers curl) are sampled every 4 cm along their lengths; the radii are generous for the
        links (5, 4.5 and 5.5 cm)."""
        self.set_pose(q29)
        m, d = self.model, self.data
        p = {n: d.xpos[m.body(f"{side}_{n}_link").id].copy() for n in ("shoulder_pitch", "elbow", "wrist_yaw")}
        tip = d.site_xpos[self.site[side]].copy()
        wrist_to_tip = tip - p["wrist_yaw"]
        ends = tip + wrist_to_tip / max(float(np.linalg.norm(wrist_to_tip)), 1e-6) * 0.03  # the finger tips
        out = []
        for a, b, radius, link in (
            (p["shoulder_pitch"], p["elbow"], 0.05, "upper arm"),
            (p["elbow"], p["wrist_yaw"], 0.045, "forearm"),
            (p["wrist_yaw"], ends, 0.055, "hand"),
        ):
            n = max(2, int(np.ceil(np.linalg.norm(b - a) / 0.04)) + 1)
            out += [(a + (b - a) * t, radius, link) for t in np.linspace(0.0, 1.0, n)]
        return out

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


class Clearance:
    """Distances from the robot's own geometry to boxes, with MuJoCo's own distance queries on the robot's collision
    shapes (the meshes' convex hulls, the palm, the finger capsules): to the site's declared tables and obstacles, and
    between a held object and the robot's links.

    A private model of the robot, the declared boxes (put in the pelvis frame for the pose at hand, each grown by a
    sideways and a vertical margin) and one box per hand for the held object, fixed to its wrist. Like `Kinematics` it
    is fed joints only; it never sees the stand-in. The arm is described in three groups: the **arm** (shoulder to wrist
    pitch), the **hand** (the wrist's last link, the palm and the fingers) and the **held** object.
    """

    ARM = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch")
    HAND = ("wrist_yaw",)  # and the finger links

    def __init__(self, camera: dict | None = None, n_boxes: int = 0):
        spec = scene.robot_spec(camera)
        for side in ("left", "right"):
            body = spec.body(scene.HAND_BODY[side])
            body.add_site(name=f"grasp_{side}", pos=list(PINCH[side]), size=[0.005, 0, 0])
            body.add_geom(
                name=f"held_{side}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[1e-3] * 3,
                pos=[0, 0, 0],
                contype=0,
                conaffinity=0,
                group=5,
            )
        for i in range(n_boxes):
            spec.worldbody.add_geom(
                name=f"declared_{i}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[1e-3] * 3,
                pos=[0, 0, -9.0 - i],
                contype=0,
                conaffinity=0,
                group=5,
            )
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.idx = scene.Robot.of(self.model)
        m = self.model
        self.n_boxes = n_boxes
        self.boxes = [m.geom(f"declared_{i}").id for i in range(n_boxes)]
        self.names: list[str] = []
        self.held = {s: m.geom(f"held_{s}").id for s in ("left", "right")}
        self.hand_geoms = {}
        self.arm_geoms = {}
        for side in ("left", "right"):
            self.arm_geoms[side] = self._geoms([f"{side}_{n}_link" for n in self.ARM])
            self.hand_geoms[side] = self._geoms(
                [f"{side}_wrist_yaw_link"]
                + [m.body(b).name for b in range(m.nbody) if m.body(b).name.startswith(f"{side}_hand_")],
                fingers=True,
            )
        # every collision shape of the robot's body and the other links, for the held object's clearance
        self.links = {m.body(m.geom_bodyid[g]).name: [] for g in range(m.ngeom)}
        for g in range(m.ngeom):
            body = m.body(m.geom_bodyid[g]).name
            if (
                body != "world"
                and m.geom_group[g] == 0
                and m.geom_contype[g] in (1, 4)
                or (m.geom_group[g] == 1 and m.geom_contype[g] == 4)
            ):
                self.links[body].append(g)
        self._fromto = np.zeros(6)

    def _geoms(self, bodies: list[str], fingers: bool = False) -> list[int]:
        m = self.model
        out = []
        for g in range(m.ngeom):
            body = m.body(m.geom_bodyid[g]).name
            if body not in bodies or g in self.held.values():
                continue
            if (m.geom_group[g] == 0 and m.geom_contype[g] in (1, 4)) or (
                fingers and m.geom_group[g] == 1 and m.geom_contype[g] == 4
            ):
                out.append(g)
        return out

    # ------------------------------------------------------------------------------------------ what is where

    def set_boxes(self, boxes: list[dict], pelvis_map: tuple[float, float, float], xy_margin: float, z_margin: float):
        """Put the declared boxes (map frame: `pos`, `half_size`) in the pelvis frame of a pelvis standing at
        `pelvis_map` (x, y, yaw), each grown by the margins. At most `n_boxes`."""
        assert len(boxes) <= self.n_boxes
        m = self.model
        x, y, yaw = pelvis_map
        c, s = math.cos(yaw), math.sin(yaw)
        R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1.0]])  # map -> pelvis
        base = np.array([x, y, scene.STAND_HEIGHT])
        self.names = [b["name"] for b in boxes]
        for i, g in enumerate(self.boxes):
            if i < len(boxes):
                half = np.array(boxes[i]["half_size"], float) + np.array([xy_margin, xy_margin, z_margin])
                m.geom_pos[g] = R @ (np.array(boxes[i]["pos"], float) - base)
                m.geom_quat[g] = (math.cos(-yaw / 2), 0, 0, math.sin(-yaw / 2))
                m.geom_size[g] = half
            else:
                m.geom_pos[g], m.geom_size[g] = (0, 0, -9.0 - i), (1e-3, 1e-3, 1e-3)
            m.geom_rbound[g] = float(np.linalg.norm(m.geom_size[g]))

    def set_held(self, side: str, half_size, offset_in_hand, rotation_in_hand) -> None:
        """The held object as a box fixed to the hand: `offset_in_hand` is its centre from the pinch point and
        `rotation_in_hand` its axes, both in the hand's frame (`Operations.held`)."""
        m, g = self.model, self.held[side]
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, np.asarray(rotation_in_hand, float).flatten())
        m.geom_pos[g] = PINCH[side] + np.asarray(offset_in_hand, float)
        m.geom_sameframe[g] = 0  # the placeholder sat on its body's frame, which MuJoCo then skips transforming
        m.geom_quat[g] = quat
        m.geom_size[g] = np.asarray(half_size, float)
        m.geom_rbound[g] = float(np.linalg.norm(half_size))

    def clear_held(self, side: str) -> None:
        self.model.geom_size[self.held[side]] = (1e-3, 1e-3, 1e-3)

    def _pose(self, q29, hand_q: dict | None = None) -> None:
        d, r = self.data, self.idx
        d.qpos[0:3], d.qpos[3:7] = 0, (1, 0, 0, 0)
        d.qpos[r.qadr] = q29
        for side in ("left", "right"):  # the fingers: open unless a pose is given
            d.qpos[r.hand_qadr[side]] = (hand_q or {}).get(side, np.zeros(7))
        mujoco.mj_kinematics(self.model, d)

    def _min(self, geoms_a: list[int], geoms_b: list[int], reach: float = 0.5) -> tuple[float, tuple[int, int] | None]:
        best, pair = reach, None
        for a in geoms_a:
            for b in geoms_b:
                dist = mujoco.mj_geomDistance(self.model, self.data, a, b, reach, self._fromto)
                if dist < best:
                    best, pair = dist, (a, b)
        return best, pair

    # ------------------------------------------------------------------------------------------ queries

    def to_boxes(self, side: str, q29, held: bool = True, hand_q=None) -> dict[str, tuple[float, str, str]]:
        """For the arm, the hand and (if one is set and `held`) the held object: the smallest distance to the declared
        boxes grown by the margins (negative: inside the grown box), with the link and the box it is to. `hand_q` is
        the working hand's seven finger joints (open if None)."""
        self._pose(q29, {side: hand_q} if hand_q is not None else None)
        m = self.model
        out = {}
        groups = {"arm": self.arm_geoms[side], "hand": self.hand_geoms[side]}
        if held and m.geom_size[self.held[side]][0] > 0.002:
            groups["held"] = [self.held[side]]
        for name, geoms in groups.items():
            dist, pair = self._min(geoms, self.boxes)
            link = "" if pair is None else m.body(m.geom_bodyid[pair[0]]).name
            box = "" if pair is None else self.names[self.boxes.index(pair[1])]
            out[name] = (dist, link, box)
        return out

    def held_to_links(self, side: str, q29) -> tuple[float, str]:
        """The smallest distance from the held object to any link of the robot except the hand that holds it (its
        wrist's last link, palm and fingers touch it by design): (distance, link)."""
        self._pose(q29)
        mine = {f"{side}_wrist_yaw_link"} | {n for n in self.links if n.startswith(f"{side}_hand_")}
        best, who = 1.0, ""
        for body, geoms in self.links.items():
            if body in mine or body == "world":
                continue
            dist, _ = self._min([self.held[side]], geoms, 1.0)
            if dist < best:
                best, who = dist, body
        return best, who

    def held_to_links_with(self, side: str, q29, half_size, offset_in_hand, rotation_in_hand) -> tuple[float, str]:
        """`held_to_links` for an object given here (and not kept afterwards)."""
        self.set_held(side, half_size, offset_in_hand, rotation_in_hand)
        out = self.held_to_links(side, q29)
        self.clear_held(side)
        return out
