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

SPREAD_M = 0.01  # a frame agrees with the median when its detection is within this (`observe`, the executor's looks)
# Snapshots (phase 3, `Perception.snapshot`): a frame agrees with the others when its marker is within
# SNAPSHOT_SPREAD_M of the median position and within SNAPSHOT_SPREAD_RAD of the median rotation; a station marker's
# pose must also put the vertical where the IMU puts it (MAX_UP_DISAGREEMENT_RAD).
# docs/evidence/g1-phase3-observations.md says how these were chosen.
SNAPSHOT_SPREAD_M = 0.02
SNAPSHOT_SPREAD_RAD = 0.05
MAX_UP_DISAGREEMENT_RAD = 0.03  # a station marker's implied vertical may differ from the IMU's by this much
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
    agreeing: int = 0  # (snapshots) how many frames agreed with the others: confidence = agreeing / frames
    spread_m: float = 0.0  # (snapshots) the largest distance of an agreeing frame from the median position
    spread_rad: float = 0.0  # (snapshots) the largest angle of an agreeing frame from the fused rotation

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
        if self.agreeing:
            out.update(agreeing=self.agreeing, spread_m=round(self.spread_m, 4), spread_rad=round(self.spread_rad, 4))
        if now_ms is not None:
            out["age_ms"] = self.age_ms(now_ms)
        return out


@dataclass(frozen=True)
class Detection:
    """One marker in one frame, in the pelvis frame, as `Perception.snapshot` collects them."""

    position: np.ndarray
    rotation: np.ndarray
    range_m: float
    stamp_ms: int
    up_error_rad: float | None  # a station marker: how far its implied vertical is from the IMU's (None for others)
    oriented: bool = False  # whether the marker's rotation is used (a station marker's is: it gives the robot's pose)


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
        self.station_markers = {s["marker"]["id"]: s["marker"] for s in site["stations"]}
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)
        self.last_frame = 0

    # ------------------------------------------------------------------------------------------ one frame

    def _detect(self, image) -> dict[int, list[tuple[np.ndarray, np.ndarray]]]:
        """{id: [(rvec, tvec), ...] in the OpenCV camera frame} for the known markers in the image: the two poses a
        planar target allows (IPPE), the better-fitting one first."""
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
            n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, c.reshape(4, 2), self.K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if n:
                found[int(mid)] = [(r, t.flatten()) for r, t in zip(rvecs, tvecs)]
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
        for mid, solutions in found.items():
            poses = []
            for rvec, tvec in solutions:
                Rm_cv, _ = cv2.Rodrigues(rvec)
                poses.append((Rc @ CV_TO_MJ @ Rm_cv, pc + Rc @ CV_TO_MJ @ tvec, float(np.linalg.norm(tvec))))
            if (
                mid in self.station_markers
            ):  # its orientation is known: the pose that puts the vertical where the IMU does
                poses = pick_by_gravity(poses, self.station_markers[mid], imu_up(sample.quat))
            out[mid] = poses[0]
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

    def snapshot(
        self,
        ids=None,
        frames: int = 10,
        timeout: float = 15.0,
        check=None,
        spread_m: float = SNAPSHOT_SPREAD_M,
        spread_rad: float = SNAPSHOT_SPREAD_RAD,
    ) -> tuple[dict[int, Reading], int]:
        """About `frames` new frames, fused per marker (`fuse_frames`): ({id: Reading}, frames taken). Only frames
        rendered after the call started are used. A marker seen in no frame is absent."""
        per_id: dict[int, list[Detection]] = {}
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
            sample = self.robot.sample_at(stamp_ms)
            for mid, (R, p, rng) in self.frame_poses(image, stamp_ms).items():
                if ids is not None and mid not in ids:
                    continue
                up_error = None
                if mid in self.station_markers and sample is not None:
                    *_, up = pelvis_pose_in_map(self.station_markers[mid], Reading(mid, p, R, 1.0, stamp_ms, 1, 1, rng))
                    up_error = float(np.arccos(np.clip(up @ imu_up(sample.quat), -1, 1)))
                per_id.setdefault(mid, []).append(Detection(p, R, rng, stamp_ms, up_error, mid in self.station_markers))
        return {mid: fuse_frames(mid, det, got, spread_m, spread_rad) for mid, det in per_id.items()}, got


def pick_by_gravity(poses: list, station_marker: dict, imu_vertical: np.ndarray) -> list:
    """The poses (R, p, range) of a station marker in the pelvis frame, the one whose implied vertical is nearest the
    IMU's first. A planar target has two poses and at a long range the wrong one often fits the image better; the wrong
    one has a pelvis several degrees off upright, which the IMU shows."""
    R_marker = site_mod.marker_frame(station_marker)[0]  # the marker in the map

    def error(pose) -> float:
        up = (R_marker @ pose[0].T)[2, :]  # the map's vertical in the pelvis frame
        return float(np.arccos(np.clip(up @ imu_vertical, -1, 1)))

    return sorted(poses, key=error)


def imu_up(quat) -> np.ndarray:
    """The vertical (world z) in the pelvis frame, from the IMU's quaternion (w, x, y, z)."""
    w, x, y, z = quat
    return np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])


def _angle(Ra: np.ndarray, Rb: np.ndarray) -> float:
    return float(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1.0, 1.0)))


def fuse_frames(
    mid: int,
    dets: list[Detection],
    frames: int,
    spread_m: float = SNAPSHOT_SPREAD_M,
    spread_rad: float = SNAPSHOT_SPREAD_RAD,
) -> Reading:
    """One reading from the per-frame detections of a snapshot. The position is the median over the frames. A frame
    **agrees** when its position is within `spread_m` of that median and, for a station marker (the only kind whose
    rotation is used), its rotation is within `spread_rad` of the median rotation (the detection closest in angle to
    all the others) and its implied vertical within MAX_UP_DISAGREEMENT_RAD of the IMU's (the wrong one of the two
    poses a planar target allows has a tilted vertical). **Confidence** = agreeing frames / frames taken, so a frame
    that did not see the marker counts against it. The rotation is the mean of the agreeing frames' (projected to a
    rotation); the age of the reading is measured from the newest agreeing frame."""
    P = np.array([d.position for d in dets])
    median = np.median(P, axis=0)
    dist = np.linalg.norm(P - median, axis=1)
    rots = [d.rotation for d in dets]
    total = [sum(_angle(a, b) for b in rots) for a in rots]
    medoid = rots[int(np.argmin(total))]
    agree = [
        i
        for i, d in enumerate(dets)
        if dist[i] <= spread_m
        and (not d.oriented or _angle(d.rotation, medoid) <= spread_rad)
        and (d.up_error_rad is None or d.up_error_rad <= MAX_UP_DISAGREEMENT_RAD)
    ]
    if agree:
        U, _, Vt = np.linalg.svd(sum(dets[i].rotation for i in agree))
        rotation = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
        spread, angle = max(dist[i] for i in agree), max(_angle(dets[i].rotation, rotation) for i in agree)
        stamp = max(dets[i].stamp_ms for i in agree)
    else:  # nothing agrees: a reading with confidence 0 (never usable), kept so the record shows why
        rotation, spread, angle, stamp = medoid, float(dist.max()), 0.0, max(d.stamp_ms for d in dets)
    return Reading(
        mid,
        median,
        rotation,
        len(agree) / max(frames, 1),
        stamp,
        frames,
        len(dets),
        float(np.median([d.range_m for d in dets])),
        len(agree),
        float(spread),
        float(angle),
    )


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


def line_of_sight_blocked(start, end, boxes: list[dict]) -> str | None:
    """The name of the first declared box (a table: `pos`, `half_size`, in the map) that the straight line from `start`
    to `end` passes through, else None (the slab method)."""
    start, d = np.asarray(start, float), np.asarray(end, float) - np.asarray(start, float)
    for box in boxes:
        lo, hi = np.array(box["pos"], float) - box["half_size"], np.array(box["pos"], float) + box["half_size"]
        t0, t1 = 0.0, 1.0
        for k in range(3):
            if abs(d[k]) < 1e-12:
                if not lo[k] <= start[k] <= hi[k]:
                    t0, t1 = 1.0, 0.0
                    break
                continue
            a, b = sorted(((lo[k] - start[k]) / d[k], (hi[k] - start[k]) / d[k]))
            t0, t1 = max(t0, a), min(t1, b)
        if t0 < t1:
            return box["name"]
    return None


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
        eye = np.array([pose[0], pose[1], STAND_HEIGHT]) + R @ pc
        # in the field of view, and not behind a declared table (the head camera looks over them)
        return in_view(R @ Rc, eye, point, site["camera"]) and line_of_sight_blocked(eye, point, site["tables"]) is None

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
