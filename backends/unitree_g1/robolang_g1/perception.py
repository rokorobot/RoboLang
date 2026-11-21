"""What the head camera tells the robot: ArUco 4x4_50 markers, as poses in the pelvis frame, with confidence and age.

Frames come from the head camera (`robot.read_frame`; a file source in simulation). Each frame is stamped with the
simulation time it was rendered at. For each frame:

* the markers are detected (sub-pixel corners) and each one's pose in the camera comes from `solvePnP` with ideal
  pinhole intrinsics from the site's camera calibration and the marker's printed size from the site;
* the camera's pose in the pelvis frame comes from the **robot's own kinematic model fed with the joints measured at
  the frame's time** (`kinematics.Kinematics`, never the simulator's state): the head camera is on the torso, which
  moves with the waist;
* the marker pose in the pelvis frame follows.

Over several frames the position of each marker is the median, its **confidence** the fraction of the frames whose
detection lies within 1 cm of that median (the ROS 2 backend's rule, `observe.fuse`), and its **age** at any later time
is that time minus the stamp of the newest agreeing frame (`Reading.age_ms`). Phase 3 builds goal verdicts on these
readings, so nothing here is specific to a task.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import site as site_mod
from .kinematics import Kinematics
from .robot import Robot, read_frame

SPREAD_M = 0.01  # a frame agrees with the median when its detection is within this
CV_TO_MJ = np.diag([1.0, -1.0, -1.0])  # OpenCV camera frame (z forward, y down) -> MuJoCo's (-z forward, y up)


@dataclass(frozen=True)
class Reading:
    """One marker as seen over a few frames, in the pelvis frame (x forward, y left, z up)."""

    marker_id: int
    position: np.ndarray  # (3,) median position of the marker's centre
    rotation: np.ndarray  # (3, 3) the marker's axes (x right, y up, z out of the plate) in the pelvis frame
    confidence: float  # fraction of the frames agreeing with the median within SPREAD_M
    stamp_ms: int  # simulation time of the newest agreeing frame
    frames: int  # how many frames were taken
    detections: int  # in how many of them the marker was found
    range_m: float  # distance from the camera to the marker

    def age_ms(self, now_ms: int) -> int:
        return now_ms - self.stamp_ms

    def as_dict(self, now_ms: int | None = None) -> dict:
        out = {
            "id": self.marker_id,
            "position": [round(float(x), 4) for x in self.position],
            "normal": [round(float(x), 4) for x in self.rotation[:, 2]],
            "confidence": round(self.confidence, 3),
            "stamp_ms": self.stamp_ms,
            "frames": self.frames,
            "detections": self.detections,
            "range_m": round(self.range_m, 3),
        }
        if now_ms is not None:
            out["age_ms"] = self.age_ms(now_ms)
        return out


class Perception:
    """Detects the site's markers in head-camera frames and places them in the pelvis frame."""

    def __init__(self, robot: Robot, kin: Kinematics, site: dict, head_dir: str | Path):
        import cv2

        self.robot, self.kin, self.head = robot, kin, Path(head_dir)
        self.camera = site["camera"]
        fx, fy, cx, cy = site_mod.intrinsics(self.camera)
        self.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
        self.sizes: dict[int, float] = {s["marker"]["id"]: float(s["marker"]["size"]) for s in site["stations"]}
        self.sizes |= {o["marker"]["id"]: float(o["marker"]["size"]) for o in site["objects"] if o.get("marker")}
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)
        self.last_frame = 0

    # ------------------------------------------------------------------------------------------ one frame

    def _detect(self, image) -> dict[int, tuple[np.ndarray, np.ndarray]]:
        """{id: (rvec, tvec) in the OpenCV camera frame} for the known markers in the image."""
        import cv2

        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        found = {}
        for c, mid in zip(corners or [], ids.flatten() if ids is not None else []):
            side = self.sizes.get(int(mid))
            if side is None:
                continue
            obj = np.array(
                [
                    [-side / 2, side / 2, 0],
                    [side / 2, side / 2, 0],
                    [side / 2, -side / 2, 0],
                    [-side / 2, -side / 2, 0],
                ],
                np.float32,
            )
            ok, rvec, tvec = cv2.solvePnP(obj, c.reshape(4, 2), self.K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                found[int(mid)] = (rvec, tvec.flatten())
        return found

    def frame_poses(self, image, stamp_ms: int) -> dict[int, tuple[np.ndarray, np.ndarray, float]]:
        """{id: (R, p, range)} of every known marker in a frame, in the pelvis frame, with the camera placed from the
        joints measured nearest the frame's time. Empty if there is no lowstate for that time."""
        import cv2

        found = self._detect(image)
        if not found:
            return {}
        sample = self.robot.sample_at(stamp_ms)
        if sample is None:
            return {}
        Rc, pc = self.kin.camera_pose(sample.q)
        out = {}
        for mid, (rvec, tvec) in found.items():
            Rm_cv, _ = cv2.Rodrigues(rvec)
            out[mid] = (Rc @ CV_TO_MJ @ Rm_cv, pc + Rc @ CV_TO_MJ @ tvec, float(np.linalg.norm(tvec)))
        return out

    # ------------------------------------------------------------------------------------------ several frames

    def observe(self, ids=None, frames: int = 10, timeout: float = 12.0, check=None) -> dict[int, Reading]:
        """Take `frames` new frames and fuse them: {id: Reading} for the markers seen in at least one.

        `ids` limits which markers are reported. `check()` (may raise) is called while waiting for frames. Only frames
        rendered after the call started are used (the robot or an arm may have moved before it).
        """
        per_id: dict[int, list[tuple[np.ndarray, np.ndarray, float, int]]] = {}
        deadline = time.monotonic() + timeout
        t_call, got = self.robot.now_ms, 0
        while got < frames and time.monotonic() < deadline:
            if check is not None:
                check()
            try:
                image, meta = read_frame(self.head, self.last_frame, timeout=0.5)
            except TimeoutError:
                continue
            self.last_frame = meta["frame"]
            stamp_ms = int(round(meta["sim_time"] * 1000))
            if stamp_ms < t_call:
                continue
            got += 1
            for mid, (R, p, rng) in self.frame_poses(image, stamp_ms).items():
                if ids is None or mid in ids:
                    per_id.setdefault(mid, []).append((R, p, rng, stamp_ms))
        return {mid: _fuse(mid, obs, got) for mid, obs in per_id.items()}


def _fuse(mid: int, obs: list, frames: int) -> Reading:
    P = np.array([o[1] for o in obs])
    median = np.median(P, axis=0)
    dist = np.linalg.norm(P - median, axis=1)
    agree = [i for i, d in enumerate(dist) if d <= SPREAD_M]
    best = int(np.argmin(dist))  # the rotation of the frame closest to the median (rotations do not average)
    stamp = max(obs[i][3] for i in agree) if agree else obs[best][3]
    return Reading(
        mid,
        median,
        obs[best][0],
        len(agree) / max(frames, 1),
        stamp,
        frames,
        len(obs),
        float(np.median([o[2] for o in obs])),
    )


# ---------------------------------------------------------------------------------------- geometry from readings


def object_centre(reading: Reading, half_extent_along_normal: float) -> np.ndarray:
    """The centre of a box from its marker reading: the marker lies on a face, the box is behind it."""
    return reading.position - reading.rotation[:, 2] * half_extent_along_normal


def pelvis_pose_in_map(station_marker: dict, reading: Reading) -> tuple[float, float, float, np.ndarray]:
    """The pelvis's (x, y, yaw, up) in the map from a station marker seen in the pelvis frame: marker (map) o inverse of
    marker (pelvis). `up` is the vertical as this solution puts it in the pelvis frame: a standing robot's IMU gives
    the same vector, so a solution whose `up` disagrees with the IMU's is the wrong one of the two poses a planar
    target allows (the solvePnP ambiguity) and should be ignored. The height is dropped."""
    R_mm, p_mm = site_mod.marker_frame(station_marker)  # marker in the map
    R_pm, p_pm = reading.rotation, reading.position  # marker in the pelvis
    R = R_mm @ R_pm.T  # pelvis in the map
    p = p_mm - R @ p_pm
    return float(p[0]), float(p[1]), site_mod.yaw_of(R), R[2, :].copy()


def in_view(camera_R: np.ndarray, camera_p: np.ndarray, point, camera: dict, margin: float = 0.9) -> bool:
    """Whether a point (same frame as the camera pose, MuJoCo camera convention) is inside the field of view, within
    `margin` of its half-angles."""
    v = camera_R.T @ (np.asarray(point, float) - camera_p)  # in the camera frame: looks along -z, +y up
    if v[2] >= -1e-6:
        return False
    fx, fy, cx, cy = site_mod.intrinsics(camera)
    u, w = abs(v[0] / -v[2]) * fx, abs(v[1] / -v[2]) * fy
    return bool(u <= cx * margin and w <= cy * margin)


# ------------------------------------------------------------ the visibility rule (0046 section 3)


def visibility_violations(site: dict, task, names: dict[str, str], kin: Kinematics) -> list[str]:
    """The scene rule of decision 0046 section 3, checked from the site's geometry before a run: every location a goal
    atom names is in the head camera's view from the station where the plan ends, and every location a precondition
    names is in view from the start pose. An `object_at` location is its region's centre; a `robot_at` location is its
    station's marker. Returns the violations (empty when the rule holds). The camera is placed by the robot's own
    kinematics at the standing posture (waist at zero) on a pelvis 0.788 m up."""
    from robolang import ir

    from .sim.scene import STAND_HEIGHT

    Rc, pc = kin.camera_pose(np.zeros(29))
    moves = [s for _, s in ir.walk(task.steps) if isinstance(s, ir.Move)]
    end = (
        site_mod.station(site, site_mod.location(site, names[moves[-1].target])["station"])["pose"]
        if moves
        else site["robot"]["start"]
    )
    start = site["robot"]["start"]

    def seen(pose, point) -> bool:
        R = site_mod.rot_z(pose[2])
        return in_view(R @ Rc, np.array([pose[0], pose[1], STAND_HEIGHT]) + R @ pc, point, site["camera"])

    def atoms(condition):
        if isinstance(condition, (ir.And, ir.Or)):
            for child in condition.conditions:
                yield from atoms(child)
        elif isinstance(condition, ir.Not):
            yield from atoms(condition.condition)
        else:
            yield condition

    bad = []
    for label, conditions, pose in (("goal", task.goal or (), end), ("precondition", task.preconditions, start)):
        for atom in (a for c in conditions for a in atoms(c)):
            if isinstance(atom, ir.ObjectAt):
                loc = site_mod.location(site, names[atom.location])
                R, p = site_mod.marker_frame(site_mod.station(site, loc["station"])["marker"])
                point = R @ site_mod.region_centre_in_marker(loc) + p
            elif isinstance(atom, ir.RobotAt):
                point = site_mod.marker_frame(
                    site_mod.station(site, site_mod.location(site, names[atom.location])["station"])["marker"]
                )[1]
            else:
                continue
            if not seen(pose, point):
                bad.append(
                    f"{label} {atom.KIND} {names[atom.location]}: out of the camera's view from "
                    f"{[round(float(v), 2) for v in pose]}"
                )
    return bad
