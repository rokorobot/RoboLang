"""Phase 1 harness: checks P1-P6 of decision 0046 section 7 against the stand-in, through the real SDK clients.

    python -m robolang_g1.harness --out harness.json

It starts the stand-in as a subprocess (with the oracle on) and drives it only through Unitree's interfaces:
`LocoClient` for walking, `rt/arm_sdk` and `rt/dex3/*` for the arms and hands, `rt/lowstate` for joint state, and the
camera frames in `DIR/head/`. The oracle file (ground truth written by the stand-in) is used to judge P2-P5, and to
steer the approach to the table in P5: in phase 1 only the harness reads it; the backend never will.

Go (exit 0): P1, P2, P5 and P6 pass, and P3 passes (free or supported). P4 is reported, not fatal.

Hidden mode `--plain-robot FILE` is the fixture for P1: a process that answers `sport` like a robot with no identity
API and counts every request it gets.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

from . import DOMAIN, ERR_REFUSED, INTERFACE, assets, guard
from .sim import scene
from .sim.policy import load_deploy

WALK_SPEED, WALK_SECONDS = 0.5, 4.0
FORWARD_RANGE, SIDEWAYS_MAX = (1.6, 2.4), 0.15  # P2
SWAY_MAX, SWAY_SECONDS = 0.02, 10.0  # P3
ARM_TOLERANCE = 0.05  # P4, rad
MARKER_ERROR_MAX, CAMERA_RANGE = 0.02, (0.5, 1.0)  # P5, m
REALTIME_MIN = 0.95  # P6
# arm_sdk gains the harness uses (the executor will choose its own): waist, shoulders and elbows, wrists
ARM_GAINS = {"waist": (300.0, 8.0), "arm": (200.0, 5.0), "wrist": (60.0, 2.0)}


# ------------------------------------------------------------------------------------------------ fixtures


def plain_robot(count_file: str) -> int:
    """A process that looks like a robot to the guard but has no identity API. Counts the requests it receives."""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco import g1_loco_api as api
    from unitree_sdk2py.rpc.server import Server

    ChannelFactoryInitialize(DOMAIN, INTERFACE)
    received = []

    class Sport(Server):
        def __init__(self):
            super().__init__(api.LOCO_SERVICE_NAME)

        def Init(self):
            self._SetApiVersion(api.LOCO_API_VERSION)
            for api_id in (api.ROBOT_API_ID_LOCO_SET_VELOCITY, api.ROBOT_API_ID_LOCO_SET_FSM_ID,
                           api.ROBOT_API_ID_LOCO_GET_FSM_ID):
                self._RegistHandler(api_id, lambda p, api_id=api_id: self.note(api_id, p), False)

        def note(self, api_id, parameter):
            received.append([api_id, parameter])
            Path(count_file).write_text(json.dumps(received))
            return 0, "{}"

    server = Sport()
    server.Init()
    server.Start()
    Path(count_file).write_text("[]")
    print("ready", flush=True)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    stop.wait()
    return 0


class StandIn:
    """The stand-in as a subprocess."""

    def __init__(self, site: str, run_dir: Path):
        self.run_dir, self.oracle = run_dir, run_dir / "oracle.json"
        env = dict(os.environ)
        env.setdefault("MUJOCO_GL", "egl")
        self.log = open(run_dir / "standin.log", "w")
        self.proc = subprocess.Popen(
            [sys.executable, "-m", "robolang_g1.sim.standin", "--site", site, "--run-dir", str(run_dir),
             "--oracle", str(self.oracle)], stdout=subprocess.PIPE, stderr=self.log, text=True, env=env)
        self.ready = None
        self.last_line = None

    def wait_ready(self, timeout: float = 90.0) -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError(f"the stand-in exited ({self.proc.poll()}); see {self.run_dir / 'standin.log'}")
            if line.startswith("{"):
                self.ready = json.loads(line)
                return self.ready
        raise TimeoutError("the stand-in did not become ready")

    def stop(self) -> dict | None:
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
        try:
            out, _ = self.proc.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            out, _ = self.proc.communicate()
        for line in reversed((out or "").splitlines()):
            if line.startswith("{"):
                return json.loads(line)
        return None


class OracleTail(threading.Thread):
    """Reads the oracle file as fast as it changes and keeps every distinct sample."""

    def __init__(self, path: Path):
        super().__init__(daemon=True)
        self.path, self.samples, self._stop_event = path, [], threading.Event()

    def run(self):
        last = None
        while not self._stop_event.is_set():
            try:
                sample = json.loads(self.path.read_text())
            except (OSError, ValueError):
                time.sleep(0.005)
                continue
            if sample["sim_time"] != last:
                last = sample["sim_time"]
                self.samples.append(sample)
            time.sleep(0.008)

    def stop(self):
        self._stop_event.set()

    def save_trace(self, path: Path, every: int = 10) -> None:
        """A 5 Hz trace of the run (pelvis pose, stance phase, command, fallen) for the evidence."""
        with path.open("w") as f:
            for s in self.samples[::every]:
                f.write(json.dumps({"t": round(s["sim_time"], 2), "pelvis": s["pelvis"]["pos"],
                                    "yaw_deg": round(float(np.degrees(yaw_of(s["pelvis"]["quat"]))), 1),
                                    "phase": s["stance"]["phase"], "cmd": s["velocity_command"],
                                    "fallen": s["fallen"]}) + "\n")

    def latest(self) -> dict:
        return self.samples[-1]

    def wait(self, predicate, timeout: float, what: str = "condition") -> dict:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.samples and predicate(self.samples[-1]):
                return self.samples[-1]
            time.sleep(0.01)
        raise TimeoutError(f"timed out after {timeout} s waiting for {what}")

    def between(self, t0: float, t1: float) -> list[dict]:
        return [s for s in list(self.samples) if t0 <= s["sim_time"] <= t1]

    def nearest(self, sim_time: float) -> dict:
        return min(list(self.samples), key=lambda s: abs(s["sim_time"] - sim_time))


def yaw_of(quat) -> float:
    w, x, y, z = quat
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def in_frame(p0, p1, yaw0) -> tuple[float, float]:
    """The displacement p0 -> p1 as (forward, sideways-left) in the heading `yaw0` (robot frame at the start)."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    return float(dx * np.cos(yaw0) + dy * np.sin(yaw0)), float(-dx * np.sin(yaw0) + dy * np.cos(yaw0))


def factory_initialized() -> bool:
    from unitree_sdk2py.core.channel import ChannelFactory

    return bool(ChannelFactory._ChannelFactory__initialized)


# ------------------------------------------------------------------------------------------------ SDK clients


class Robot:
    """What the harness uses to talk to the robot: the SDK's own clients and topics."""

    def __init__(self):
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_, unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, LowCmd_, LowState_

        self.loco = LocoClient()
        self.loco.SetTimeout(3.0)
        self.loco.Init()
        self.low_q, self.low_tick, self.low_count = np.zeros(29), 0, 0
        self.hand_q = {"left": np.zeros(7), "right": np.zeros(7)}
        sub = ChannelSubscriber("rt/lowstate", LowState_)
        sub.Init(self._on_low, 10)
        self._subs = [sub]
        for side in ("left", "right"):
            hs = ChannelSubscriber(f"rt/dex3/{side}/state", HandState_)
            hs.Init(lambda m, side=side: self.hand_q.__setitem__(side, np.array([x.q for x in m.motor_state[:7]])), 10)
            self._subs.append(hs)
        self.arm_pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_pub.Init()
        self.hand_pub = {s: ChannelPublisher(f"rt/dex3/{s}/cmd", HandCmd_) for s in ("left", "right")}
        for pub in self.hand_pub.values():
            pub.Init()
        self._arm_msg, self._hand_msg = unitree_hg_msg_dds__LowCmd_(), unitree_hg_msg_dds__HandCmd_()
        self.arm_target = None  # (q, kp, kd) the arm thread publishes; None until set
        self.arm_weight = 0.0
        self._arm_thread = None
        self._arm_stop = threading.Event()
        self._lock = threading.Lock()

    def _on_low(self, msg):
        self.low_q = np.array([m.q for m in msg.motor_state[:29]])
        self.low_tick, self.low_count = msg.tick, self.low_count + 1

    # arm_sdk: published at 50 Hz while the thread runs; q, gains and weight can change at any time
    def start_arm_sdk(self):
        self._arm_stop.clear()
        self._arm_thread = threading.Thread(target=self._arm_loop, daemon=True)
        self._arm_thread.start()

    def stop_arm_sdk(self):
        self._arm_stop.set()
        if self._arm_thread:
            self._arm_thread.join(timeout=2)

    def _arm_loop(self):
        msg = self._arm_msg
        while not self._arm_stop.is_set():
            with self._lock:
                target, weight = self.arm_target, self.arm_weight
            if target is not None:
                q, kp, kd = target
                for i in range(29):
                    c = msg.motor_cmd[i]
                    c.q, c.dq, c.tau, c.kp, c.kd = float(q[i]), 0.0, 0.0, float(kp[i]), float(kd[i])
                msg.motor_cmd[29].q = float(weight)  # kNotUsedJoint: the arm_sdk weight
                self.arm_pub.Write(msg)
            time.sleep(0.02)

    def set_arm(self, q, kp, kd, weight):
        with self._lock:
            self.arm_target, self.arm_weight = (np.array(q, float), np.array(kp, float), np.array(kd, float)), weight

    def ramp_arm(self, q_to, kp, kd, w_to, seconds, q_from=None, w_from=None):
        """Move the arm_sdk targets and weight linearly over `seconds`."""
        q0 = self.low_q.copy() if q_from is None else np.array(q_from, float)
        w0 = self.arm_weight if w_from is None else w_from
        steps = max(1, int(seconds / 0.02))
        for i in range(1, steps + 1):
            a = i / steps
            self.set_arm((1 - a) * q0 + a * np.array(q_to), kp, kd, (1 - a) * w0 + a * w_to)
            time.sleep(0.02)

    def hand_command(self, side, q, kp=1.5, kd=0.2, seconds=2.0):
        msg = self._hand_msg
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            for i in range(7):
                c = msg.motor_cmd[i]
                c.q, c.dq, c.tau, c.kp, c.kd = float(q[i]), 0.0, 0.0, kp, kd
            self.hand_pub[side].Write(msg)
            time.sleep(0.02)


def arm_gains(policy_kp, policy_kd) -> tuple[np.ndarray, np.ndarray]:
    """The gains the harness gives arm_sdk for joints 12-28 (the rest keep the policy's)."""
    kp, kd = np.array(policy_kp, float), np.array(policy_kd, float)
    for j in range(12, 15):
        kp[j], kd[j] = ARM_GAINS["waist"]
    for j in range(15, 29):
        kp[j], kd[j] = ARM_GAINS["wrist" if (j - 15) % 7 >= 4 else "arm"]
    return kp, kd


# ------------------------------------------------------------------------------------------------ the checks


def check_p1_guard(run_dir: Path) -> dict:
    """P1, part 1: the refusals, before the stand-in exists. Returns the details so far."""
    d: dict = {}
    for name, kwargs in (("domain_0", {"domain": 0}), ("interface_eth0", {"interface": "eth0"})):
        try:
            guard.require_simulation(**kwargs)
            d[name] = {"refused": False}
        except guard.GuardError as exc:
            d[name] = {"refused": True, "code": exc.code, "reason": exc.reason,
                       "dds_participant_created": factory_initialized()}
    count_file = run_dir / "plain_robot_requests.json"
    plain = subprocess.Popen([sys.executable, "-m", "robolang_g1.harness", "--plain-robot", str(count_file)],
                             stdout=subprocess.PIPE, text=True)
    try:
        for line in plain.stdout:  # the SDK prints its own lines first
            if line.strip() == "ready":
                break
        else:
            raise RuntimeError("the plain-robot fixture did not start")
        try:
            guard.require_simulation(timeout=3.0)
            d["identity_missing"] = {"refused": False}
        except guard.GuardError as exc:
            time.sleep(0.3)
            received = json.loads(count_file.read_text())
            d["identity_missing"] = {"refused": True, "code": exc.code, "reason": exc.reason,
                                     "requests_received_by_the_robot": len(received)}
    finally:
        plain.send_signal(signal.SIGTERM)
        plain.wait(timeout=10)
    return d


def p1_ok(d: dict) -> bool:
    refused = all(d[k].get("refused") and d[k].get("code") == "S501" for k in ("domain_0", "interface_eth0",
                                                                                   "identity_missing"))
    clean = (not any(d[k].get("dds_participant_created") for k in ("domain_0", "interface_eth0"))
             and d["identity_missing"].get("requests_received_by_the_robot") == 0)
    accepted = bool(d.get("stand_in_accepted"))
    damp = d.get("damp_refused", {})
    return bool(refused and clean and accepted and damp.get("code") == ERR_REFUSED and damp.get("still_standing"))


def check_p2_p3(robot: Robot, tail: OracleTail) -> tuple[dict, dict]:
    runs, p3_runs = [], []
    for k in range(3):
        tail.wait(lambda s: s["stance"]["standing"] and not s["fallen"], 40, "standing")
        start = tail.latest()
        p0, yaw0 = start["pelvis"]["pos"], yaw_of(start["pelvis"]["quat"])
        code = robot.loco.SetVelocity(WALK_SPEED, 0.0, 0.0, WALK_SECONDS)
        issued = tail.wait(lambda s: s["velocity_command"][0] != 0, 5, "the velocity command")
        until = issued["command_until"]
        end = tail.wait(lambda s: s["sim_time"] >= until, WALK_SECONDS + 5, "the end of the walk")
        end = tail.nearest(until)
        forward, sideways = in_frame(p0, end["pelvis"]["pos"], yaw0)
        run = {"set_velocity_code": code, "forward_m": round(forward, 3), "sideways_m": round(sideways, 3),
               "heading_change_deg": round(float(np.degrees(yaw_of(end["pelvis"]["quat"]) - yaw0)), 1),
               "fallen": bool(end["fallen"]), "walked_s": round(end["sim_time"] - issued["sim_time"], 2)}
        run["pass"] = (code == 0 and FORWARD_RANGE[0] <= forward <= FORWARD_RANGE[1]
                       and abs(sideways) <= SIDEWAYS_MAX and not end["fallen"])
        runs.append(run)
        # P3: after the stand-in's stop, wait until it stands, then watch the torso
        stood = tail.wait(lambda s: s["stance"]["standing"], 40, "the stop to settle")
        window = tail.between(stood["sim_time"], stood["sim_time"] + SWAY_SECONDS)
        tail.wait(lambda s: s["sim_time"] >= stood["sim_time"] + SWAY_SECONDS, SWAY_SECONDS + 5, "the sway window")
        window = tail.between(stood["sim_time"], stood["sim_time"] + SWAY_SECONDS)
        pos = np.array([s["torso"]["pos"] for s in window])
        sway = (pos.max(0) - pos.min(0))
        p3_runs.append({"torso_sway_cm_xyz": np.round(sway * 100, 2).tolist(), "samples": len(window),
                        "settled_after_s": round(stood["sim_time"] - end["sim_time"], 2),
                        "stance_model": stood["stance"]["model"], "pass": bool(sway.max() <= SWAY_MAX
                                                                                and not tail.latest()["fallen"])})
    p2 = {"pass": all(r["pass"] for r in runs), "details": {"runs": runs, "command": f"SetVelocity({WALK_SPEED}, 0, 0, "
                                                                                      f"{WALK_SECONDS})",
                                                            "frame": "robot heading at the start of each walk"}}
    p3 = {"pass": all(r["pass"] for r in p3_runs), "details": {"runs": p3_runs,
                                                               "stance_model": tail.latest()["stance"]["model"]}}
    return p2, p3


def check_p4_arms(robot: Robot, tail: OracleTail, sim_client, policy_cfg: dict) -> dict:
    d: dict = {}
    tail.wait(lambda s: s["stance"]["standing"], 40, "standing")
    default = np.zeros(29)
    default[policy_cfg["joint_ids_map"]] = policy_cfg["default_joint_pos"]
    kp, kd = arm_gains(policy_cfg["stiffness"], policy_cfg["damping"])
    reach = default.copy()
    reach[22], reach[25] = -0.9, 0.6  # right shoulder pitch and elbow: reach forward
    robot.start_arm_sdk()
    try:
        robot.ramp_arm(reach, kp, kd, 1.0, 1.0, q_from=default, w_from=0.0)
        time.sleep(2.5)
        err = np.abs(robot.low_q[12:29] - reach[12:29])
        d["tracking_max_error_rad"] = round(float(err.max()), 4)
        d["tracking_worst_joint"] = 12 + int(err.argmax())
        d["tracking_tolerance_rad"] = ARM_TOLERANCE
        # the right hand through rt/dex3/right/cmd: close, then read it back from rt/dex3/right/state
        close = np.array([0.0, -0.6, -1.0, 1.0, 1.2, 1.0, 1.2])
        robot.hand_command("right", close, seconds=2.0)
        d["hand_tracking_max_error_rad"] = round(float(np.abs(robot.hand_q["right"] - close).max()), 4)
        robot.hand_command("right", np.zeros(7), seconds=1.0)
        # carry pose (the policy's arm pose) with a 0.3 kg payload welded to the right hand, then walk about 2 m
        robot.ramp_arm(default, kp, kd, 1.0, 1.5)
        code, attached = sim_client.Attach("payload", "right", True)
        d["attach"] = {"code": code, **(attached or {})}
        start = tail.latest()
        p0, yaw0 = start["pelvis"]["pos"], yaw_of(start["pelvis"]["quat"])
        t_end, walked = time.monotonic() + 15, 0.0
        while time.monotonic() < t_end and walked < 2.0 and not tail.latest()["fallen"]:
            robot.loco.SetVelocity(WALK_SPEED, 0.0, 0.0, 1.0)
            time.sleep(0.4)
            walked, _ = in_frame(p0, tail.latest()["pelvis"]["pos"], yaw0)
        robot.loco.SetVelocity(0.0, 0.0, 0.0, 1.0)
        stood = tail.wait(lambda s: s["stance"]["standing"] or s["fallen"], 40, "standing after the carry walk")
        end = tail.latest()
        d["carry_walk"] = {"walked_m": round(float(walked), 2), "payload_kg": 0.3, "fallen": bool(end["fallen"]),
                           "payload_attached": "payload" in end["attached"],
                           "stood_again": bool(stood["stance"]["standing"])}
        carry_error = np.abs(robot.low_q[12:29] - default[12:29])
        d["carry_walk"]["arm_error_after_rad"] = round(float(carry_error.max()), 4)
        sim_client.Detach("payload")
        robot.ramp_arm(default, kp, kd, 0.0, 1.0)
    finally:
        robot.stop_arm_sdk()
    ok = (d["tracking_max_error_rad"] <= ARM_TOLERANCE and not d["carry_walk"]["fallen"]
          and d["carry_walk"]["walked_m"] >= 1.5 and d["carry_walk"]["payload_attached"])
    return {"pass": bool(ok), "details": d}


def read_frame(head: Path, after_frame: int = 0, timeout: float = 5.0):
    """A consistent (png path, meta): the JSON, the PNG, and the JSON again must name the same frame."""
    import cv2

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            a = json.loads((head / "latest.json").read_text())
            image = cv2.imread(str(head / "latest.png"))
            b = json.loads((head / "latest.json").read_text())
        except (OSError, ValueError):
            time.sleep(0.02)
            continue
        if a["frame"] == b["frame"] and a["frame"] > after_frame and image is not None:
            return image, a
        time.sleep(0.02)
    raise TimeoutError("no new camera frame")


def detect_markers(image, sizes: dict[int, float]):
    """{id: (center in the OpenCV camera frame (m), reprojection px)} for the markers found in `image`."""
    import cv2

    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), params)
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    fx, fy, cx, cy = scene.camera_intrinsics()
    camera = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    found = {}
    for c, mid in zip(corners or [], (ids.flatten() if ids is not None else [])):
        if int(mid) not in sizes:
            continue
        s = sizes[int(mid)]
        obj = np.array([[-s / 2, s / 2, 0], [s / 2, s / 2, 0], [s / 2, -s / 2, 0], [-s / 2, -s / 2, 0]], np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj, c.reshape(4, 2), camera, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if ok:
            found[int(mid)] = tvec.flatten()
    return found, corners, ids


def check_p5_camera(robot: Robot, tail: OracleTail, run_dir: Path, site: dict) -> dict:
    import cv2

    target = "marker_box_far"
    marker = next(o["marker"] for o in site["objects"] if o["name"] == "box_far")
    sizes = {marker["id"]: marker["size"]} | {s["marker"]["id"]: s["marker"]["size"] for s in site["stations"]}

    def rng(s):
        return float(np.linalg.norm(np.array(s["markers"][target]["pos"]) - np.array(s["head_camera"]["pos"])))

    d: dict = {"approach": []}
    # Stand 0.8 m in front of the marker (its face looks along -x), facing it. The policy's walk drifts to the right and
    # turns, so go there in legs, correcting forward, sideways and turn from the oracle each time the robot stands.
    stand_point = np.array(tail.latest()["markers"][target]["pos"][:2]) + [-0.8, 0.0]
    for _ in range(16):
        s = tail.wait(lambda s: s["stance"]["standing"], 60, "standing")
        if s["fallen"]:
            d["error"] = f"the robot fell at sim time {s['fell_at']}"
            return {"pass": False, "details": d}
        r = rng(s)
        pelvis, yaw = np.array(s["pelvis"]["pos"]), yaw_of(s["pelvis"]["quat"])
        ahead, left = in_frame(pelvis, np.append(stand_point, 0.0), yaw)
        turn = float((-yaw + np.pi) % (2 * np.pi) - np.pi)  # to face +x, the marker
        d["approach"].append({"range_m": round(r, 2), "ahead_m": round(ahead, 2), "left_m": round(left, 2),
                              "turn_deg": round(float(np.degrees(turn)), 1)})
        if CAMERA_RANGE[0] <= r <= CAMERA_RANGE[1] and abs(left) < 0.25 and abs(turn) < np.radians(15):
            break
        seconds = float(np.clip(max(np.hypot(ahead, left) / 0.4, abs(turn) / 0.15), 0.6, WALK_SECONDS))
        robot.loco.SetVelocity(float(np.clip(ahead / seconds, -0.2, 0.5)), float(np.clip(left / seconds, -0.3, 0.3)),
                               float(np.clip(turn / seconds, -0.2, 0.2)), seconds)
        tail.wait(lambda s: not s["stance"]["standing"], 5, "the move to start")
    frames, errors = [], []
    head = run_dir / "head"
    last = 0
    for i in range(6):
        image, meta = read_frame(head, last)
        last = meta["frame"]
        found, corners, ids = detect_markers(image, sizes)
        s = tail.nearest(meta["sim_time"])
        cam_pos, cam_mat = np.array(s["head_camera"]["pos"]), np.array(s["head_camera"]["mat"]).reshape(3, 3)
        entry = {"frame": meta["frame"], "sim_time": round(meta["sim_time"], 3), "ids": sorted(found)}
        if marker["id"] in found:
            t = found[marker["id"]]
            world = cam_pos + cam_mat @ np.array([t[0], -t[1], -t[2]])  # OpenCV camera frame -> MuJoCo's -> world
            truth = np.array(s["markers"][target]["pos"])
            err = world - truth
            entry["error_m"] = round(float(np.linalg.norm(err)), 4)
            entry["range_m"] = round(float(np.linalg.norm(truth - cam_pos)), 3)
            errors.append(float(np.linalg.norm(err)))
        frames.append(entry)
        if i < 3:
            cv2.imwrite(str(run_dir / f"p5_head_{meta['frame']:04d}.png"), image)
    d["frames"] = frames
    final = tail.latest()
    d["range_m"] = round(rng(final), 3)
    d["detected_frames"] = len(errors)
    d["median_error_m"] = round(float(np.median(errors)), 4) if errors else None
    d["max_error_m"] = round(float(np.max(errors)), 4) if errors else None
    ok = (len(errors) >= 4 and max(errors) <= MARKER_ERROR_MAX and CAMERA_RANGE[0] <= rng(final) <= CAMERA_RANGE[1])
    return {"pass": bool(ok), "details": d}


def check_p6_time(sim_client, started: dict, tail: OracleTail) -> dict:
    code, status = sim_client.Status()
    now = tail.latest()
    wall = time.perf_counter() - started["wall"]
    ratio = (now["sim_time"] - started["sim"]) / wall
    d = {"harness_ratio": round(ratio, 4), "harness_wall_s": round(wall, 1), "stand_in": status}
    cam = (status or {}).get("camera", {})
    ok = (code == 0 and ratio >= REALTIME_MIN and status["realtime_ratio"] >= REALTIME_MIN
          and (status["worst_second_ratio"] or 0) >= REALTIME_MIN and cam.get("hz", 0) >= 4.5)
    return {"pass": bool(ok), "details": d}


# ------------------------------------------------------------------------------------------------ main


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="RoboLang G1 phase 1 harness (checks P1-P6 of decision 0046)")
    parser.add_argument("--out", default="harness.json")
    parser.add_argument("--run-dir", help="where frames, the oracle and logs go (default: <out>.run next to --out)")
    parser.add_argument("--site", default=str(Path(__file__).parent / "sites" / "phase1.json"))
    parser.add_argument("--skip", default="", help="development only: comma list of checks to skip (P2 also skips P3, "
                                                   "P4); the go rule then fails")
    parser.add_argument("--plain-robot", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.plain_robot:
        return plain_robot(args.plain_robot)

    out = Path(args.out)
    run_dir = Path(args.run_dir) if args.run_dir else out.with_suffix(".run")
    run_dir.mkdir(parents=True, exist_ok=True)
    site = scene.load_site(args.site)
    result: dict = {"checks": {}, "stance_model": None, "versions": assets.versions(), "arch": os.uname().machine}
    standin = tail = None
    t_start = time.perf_counter()

    def record(name, fn):
        try:
            result["checks"][name] = fn()
        except Exception as exc:  # a failed check must not stop the report
            result["checks"][name] = {"pass": False, "details": {"error": f"{type(exc).__name__}: {exc}"}}
        print(f"{name}: {'pass' if result['checks'][name]['pass'] else 'FAIL'}", flush=True)

    p1: dict = {}
    try:
        p1 = check_p1_guard(run_dir)
        standin = StandIn(args.site, run_dir)
        result["stand_in_ready"] = standin.wait_ready()
        client, identity = guard.require_simulation(timeout=30.0)
        p1["stand_in_accepted"] = identity
        tail = OracleTail(standin.oracle)
        tail.start()
        tail.wait(lambda s: s["sim_time"] > 0, 30, "the oracle")
        robot = Robot()
        time.sleep(1.0)
        code = robot.loco.SetFsmId(1)  # Damp: must be refused
        time.sleep(1.0)
        p1["damp_refused"] = {"code": code, "still_standing": bool(tail.latest()["stance"]["standing"]
                                                                  and not tail.latest()["fallen"]),
                              "fsm_id_after": robot.loco.GetFsmId()}
        result["checks"]["P1"] = {"pass": p1_ok(p1), "details": p1}
        print(f"P1: {'pass' if result['checks']['P1']['pass'] else 'FAIL'}", flush=True)
        started = {"wall": time.perf_counter(), "sim": tail.latest()["sim_time"]}

        p23 = {}

        def run_p2_p3():
            p2, p3 = check_p2_p3(robot, tail)
            p23["P3"] = p3
            return p2

        skip = set(args.skip.split(","))
        if "P2" not in skip:
            record("P2", run_p2_p3)
            result["checks"]["P3"] = p23.get("P3", {"pass": False, "details": {"error": "P2 did not complete"}})
            print(f"P3: {'pass' if result['checks']['P3']['pass'] else 'FAIL'}", flush=True)
        policy_cfg = load_deploy(assets.asset(*assets.POLICY_DIR, "params", "deploy.yaml"))
        if "P4" not in skip and "P2" not in skip:
            record("P4", lambda: check_p4_arms(robot, tail, client, policy_cfg))
        if "P5" not in skip:
            record("P5", lambda: check_p5_camera(robot, tail, run_dir, site))
        record("P6", lambda: check_p6_time(client, started, tail))
        result["stance_model"] = tail.latest()["stance"]["model"]
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        if "P1" not in result["checks"]:
            result["checks"]["P1"] = {"pass": False, "details": {**p1, "error": result["error"]}}
    finally:
        if tail:
            tail.stop()
            tail.save_trace(run_dir / "oracle_trace.jsonl")
        if standin:
            result["stand_in_final"] = standin.stop()
    for name in ("P1", "P2", "P3", "P4", "P5", "P6"):
        result["checks"].setdefault(name, {"pass": False, "details": {"error": "not run"}})
    checks = result["checks"]
    result["go"] = all(checks[c]["pass"] for c in ("P1", "P2", "P3", "P5", "P6"))
    result["elapsed_s"] = round(time.perf_counter() - t_start, 1)
    out.write_text(json.dumps(result, indent=2))
    print(f"go: {result['go']}  (P4 {'pass' if checks['P4']['pass'] else 'reported: not passing'}); wrote {out}")
    return 0 if result["go"] else 1


if __name__ == "__main__":
    sys.exit(main())
