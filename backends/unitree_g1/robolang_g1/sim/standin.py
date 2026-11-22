"""The G1 stand-in process: MuJoCo paced to real time behind the interfaces the real robot answers on (decision 0046).

    python -m robolang_g1.sim.standin --site SITE.json --run-dir DIR [--oracle PATH] [--stance free|supported]

What it serves, on DDS domain 1 and the loopback interface only (both fixed):

* `rt/lowstate` (unitree_hg `LowState_`) at 250 Hz: joint q, dq and torque for the 29 body motors,
  the pelvis IMU, and `tick` = SIMULATION time in milliseconds (the robot's `tick` is its own millisecond counter).
  The IMU is not MuJoCo's exact sensor: a gyro bias, noise and a drifting yaw (`imu.py`). The message is serialized
  once and its changing fields patched in a buffer (`fastdds.py`): the SDK's own bytes, for a twentieth of the CPU.
* `rt/arm_sdk` (`LowCmd_`) in: the weight is `motor_cmd[29].q`; joints 12-28 (waist and arms) are blended over the
  stance's targets as on the robot: torque = (1-w) x its own PD + w x (kp (q* - q) + kd (dq* - dq) + tau). The last
  command stays in force if the publisher stops (the arms hold where they are).
* `rt/dex3/left|right/cmd` (`HandCmd_`) in, `rt/dex3/left|right/state` (`HandState_`) out at 100 Hz: 7 motors per hand,
  torque = kp (q* - q) + kd (dq* - dq) + tau. With no command a hand holds open with kp 1.5, kd 0.2.
* RPC service `sport` (the SDK's `Server` class): what `LocoClient` needs. `SetVelocity` (vx, vy, yaw rate clipped to
  the policy's trained range; the velocity returns to 0 when `duration` has passed, in simulation time), `GetFsmId`
  (always 500: standing or walking), `SetFsmId` (500 is accepted; **every other id is refused with code 7400,
  `Damp` included**). The stand-in never collapses the robot: there is no damping, zero torque or user-control
  mode. Anything else answers "not implemented" (3203). Note that the SDK's `LocoClient.Damp()` swallows the code
  and returns nothing; the refusal shows in `SetFsmId(1)`.
* RPC service `robolang_sim`, which exists only here: identity (`{"simulation": true, "domain": 1, "sim_time": t,
  "stance_model": ...}`), status (real-time ratio, camera frames), and attach / detach of an object to a hand (a MuJoCo
  weld, decision 0046 U7). Simulation time is also in `LowState_.tick` (ms) and in every camera frame's JSON.
* The head camera: 640x480 at 5 Hz, rendered on its own thread from a copy of the state, so physics never waits for
  rendering. `DIR/head/latest.png` and `latest.json` (frame, sim_time) are replaced atomically; read the JSON, then the
  PNG, then the JSON again and keep the frame if both JSONs agree. On the robot the camera is read directly, not
  through the SDK, so a file frame source is a fair stand-in.
* The stance (`stance.py`) decides what the robot does when it is asked to stand: `free` (default: the policy alone,
  which comes to rest by itself at dt 0.002) or `supported` (the torso welded to a mocap body: the approved fallback).
  Its name is `stance_model` in the identity API, the status and the oracle.
* With `--oracle PATH` only: ground truth for the test harness (poses, markers, a fallen flag), written 50 times a
  second. Nothing else reads MuJoCo state.

Real time (U13): physics runs at dt 0.002 against the wall clock and the PD law is evaluated on every physics step
(500 Hz, the robot's own low-level rate); the policy runs at 50 Hz. 0046's dt of 0.005 does not work: MuJoCo
integrates a torque explicitly, which is stable only for dt x kd / I < 2, and with the policy's arm gain kd = 10 on
arm joints of inertia I down to 0.011 kg m^2 that is 4.5 at dt 0.005 (1.8 at 0.002). At 0.005 the arm torques chatter
between their limits (64% of steps saturated at rest) and nothing downstream is reproducible; see
docs/evidence/g1-phase1-harness.md. The loop measures its own lag (`status`): the overall ratio and the worst
one-second ratio, and for a second below 0.95 whether the loop thread was computing or waiting for a CPU. The camera
thread runs at nice 19 and the executor at nice 10 (harness.YIELD_NICE), so that the loop, at nice 0, is never queued
behind them (docs/evidence/g1-phase4-prereqs.md).
"""
from __future__ import annotations

import argparse
import collections
import gc
import json
import os
import queue
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from .. import (API_ATTACH, API_COVER_CAMERA, API_DETACH, API_IDENTITY, API_PLACE_OBJECT, API_PUSH, API_SET_STANCE,
                API_STATUS, DOMAIN, ERR_REFUSED)
from .. import INTERFACE
from .. import SIM_API_VERSION
from .. import SIM_SERVICE, assets
from . import scene
from .fastdds import PatchedSample
from .imu import Imu
from .policy import STEP_DT, WholeBodyPolicy
from .scene import HAND_ACT, HAND_BODY, N_BODY
from .stance import STANCES, Body, make_stance

CAMERA_HZ = 5.0
ORACLE_HZ = 50.0
# The camera thread (rendering, PNG encoding) and the renderer's worker threads it starts are the stand-in's largest
# consumer of CPU (about 0.7 of a core at 5 Hz), and none of it is time-critical. They yield to the physics loop: at the
# lowest priority, so that a wake-up of the loop (500 a second) is never queued behind a render. Measured with the
# executor starting up on two CPUs: the loop waited 0.4-0.6 s in the run queue in one second at nice 5 (docs/evidence/
# g1-phase4-prereqs.md).
CAMERA_NICE = 19
POLICY_EVERY = round(STEP_DT / scene.TIMESTEP)  # physics steps per policy step (50 Hz)
LOWSTATE_EVERY = round(1 / (250 * scene.TIMESTEP))  # lowstate at 250 Hz
HAND_EVERY = round(1 / (100 * scene.TIMESTEP))  # hand states at 100 Hz
LIMITS = ((-0.5, 1.0), (-0.3, 0.3), (-0.2, 0.2))  # vx, vy, yaw rate: the range the policy was trained on
GRASP_POS = np.array([0.09, 0.0, 0.0])  # where `attach` puts an object, in the wrist link frame (palm centre)
HAND_DEFAULT = (1.5, 0.2)  # kp, kd of a hand nobody has commanded
FALLEN_HEIGHT, FALLEN_UP = 0.45, 0.5  # pelvis height (m) and up-vector z below which the robot has fallen


@dataclass
class ArmCommand:
    weight: float
    q: np.ndarray
    dq: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    tau: np.ndarray


@dataclass
class HandCommand:
    q: np.ndarray
    dq: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    tau: np.ndarray


@dataclass
class Timing:
    """Real-time bookkeeping (U13): simulated seconds against wall seconds, overall and per window of one second."""

    wall0: float = 0.0
    window_wall: float = 0.0
    window_ratio_min: float = float("inf")
    steps: int = 0
    stalls: int = 0
    camera_frames: int = 0
    camera_dropped: int = 0
    camera_ms: float = 0.0
    camera_stamps: list = field(default_factory=list)
    # (sim time, ratio, loop thread CPU s, loop thread run-queue wait s) of the seconds below 0.95: a second that was
    # slow because the loop waited for a CPU shows a wait of tenths of a second (Linux; both None elsewhere)
    slow_windows: list = field(default_factory=list)
    sched_prev: tuple | None = None  # the loop thread's (CPU s, run-queue wait s) at the last window boundary
    step_ms: collections.deque = field(default_factory=lambda: collections.deque(maxlen=20000))  # compute per step
    gc_count: list = field(default_factory=lambda: [0, 0, 0])  # collections by generation since start-up
    gc_max_ms: list = field(default_factory=lambda: [0.0, 0.0, 0.0])  # longest collection by generation
    gc_started: float = 0.0
    slow_steps: list = field(default_factory=list)  # (sim time, ms, collections during it) of passes > 20 ms


def thread_sched() -> tuple[float, float] | None:
    """This thread's (CPU seconds, seconds waiting in the run queue for a CPU) since it started, from the kernel's
    per-thread scheduler statistics; None where there are none."""
    try:
        with open("/proc/thread-self/schedstat") as f:
            fields = f.read().split()
        return int(fields[0]) / 1e9, int(fields[1]) / 1e9
    except (OSError, ValueError, IndexError):
        return None


class Simulator:
    """MuJoCo, the policy, a stance, the arm blend, the hands, the camera thread and the oracle. No DDS in here."""

    def __init__(self, site, stance: str = "free", run_dir=None, oracle=None, camera: bool = True,
                 imu_seed: int | None = None, imu_bias: float | None = None, imu_ideal: bool = False):
        self.model, self.robot, self.site = scene.build(site)
        self.imu = Imu(imu_seed, imu_bias, imu_ideal)  # what `rt/lowstate` carries as the IMU (sim/imu.py)
        self.data = mujoco.MjData(self.model)
        m, d, r = self.model, self.data, self.robot
        pol_dir = assets.asset(*assets.POLICY_DIR)
        self.policy = WholeBodyPolicy(pol_dir / "exported" / "policy.onnx", pol_dir / "params" / "deploy.yaml")
        d.qpos[r.qadr] = self.policy.default
        mujoco.mj_forward(m, d)
        self.stance = make_stance(stance, Body(m, d, r, self.policy))
        lo, hi = m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1]
        self._lo, self._hi = lo, hi
        self.n = 0
        self.sim_time = 0.0
        self._velocity = (0.0, 0.0, 0.0, 0.0)  # vx, vy, wz, until (simulation time)
        self.last_command = (0.0, 0.0, 0.0)
        self._arm: ArmCommand | None = None
        self._hand = {s: None for s in ("left", "right")}
        self._requests: queue.SimpleQueue = queue.SimpleQueue()
        self.fell_at: float | None = None
        self.attached: dict[str, str] = {}  # object -> hand side
        self._saved_collision: dict[str, list] = {}
        self.timing = Timing()
        self.run_dir = Path(run_dir) if run_dir else None
        self.oracle_path = Path(oracle) if oracle else None
        self.camera = camera and self.run_dir is not None
        self._markers = self._marker_table()
        self._cam_buffers = [mujoco.MjData(m) for _ in range(2)]
        self._cam_next = 0
        self._cam_queue: queue.Queue = queue.Queue(maxsize=1)
        self._cam_busy = False
        self._camera_ready = threading.Event()
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self.on_step = None  # called after every physics step on the simulation thread (the DDS bridge)
        # What the robot has been sent: per channel, how many commands and the simulation time of the last one. The
        # harness reads it from the oracle and `status` ("no further command after a fall"); nothing else does.
        self.commands = {k: {"count": 0, "last_sim_time": None} for k in ("sport", "arm_sdk", "hand_left", "hand_right",
                                                                           "attach", "detach")}
        self._push: tuple[np.ndarray, np.ndarray, float] | None = None  # test hook: (force, torque on the torso, until)
        self.camera_covered = False  # test hook: the head camera shows black frames
        self._torso_dof_body = self.robot.torso

    # ------------------------------------------------------------------------------------------ inputs

    def note_command(self, channel: str) -> None:
        c = self.commands[channel]
        c["count"] += 1
        c["last_sim_time"] = self.sim_time

    def push(self, force, duration: float, torque=(0.0, 0.0, 0.0)) -> dict:
        """Test hook (simulation thread): a force in N and a torque in N m on the torso, world frame, for `duration`
        simulated seconds. A steady torque about the vertical is a heading creep the stress check can dial in."""
        self._push = (np.asarray(force, float), np.asarray(torque, float), self.sim_time + float(duration))
        return {"force": [float(x) for x in force], "torque": [float(x) for x in torque], "until": self._push[2]}

    def cover_camera(self, covered: bool) -> dict:
        """Test hook: a covered head camera renders black frames (the camera thread reads the flag)."""
        self.camera_covered = bool(covered)
        return {"covered": self.camera_covered}

    def place_object(self, obj: str, position, yaw: float = 0.0) -> dict:
        """Test hook (simulation thread): put a free object upright at `position` (world, its centre), at rest."""
        m, d = self.model, self.data
        if obj in self.attached:
            raise ValueError(f"{obj} is attached to a hand")
        body = m.body(obj).id  # KeyError for an unknown object
        jadr, dadr = int(m.jnt_qposadr[m.body_jntadr[body]]), int(m.jnt_dofadr[m.body_jntadr[body]])
        d.qpos[jadr:jadr + 3] = position
        d.qpos[jadr + 3:jadr + 7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        d.qvel[dadr:dadr + 6] = 0
        mujoco.mj_forward(m, d)
        return {"object": obj, "position": [float(x) for x in position]}

    def watch_gc(self) -> None:
        """Time every garbage collection (a full one on a large heap can stall the 2 ms loop for hundreds of ms)."""
        t = self.timing

        def callback(phase, info):
            if phase == "start":
                t.gc_started = time.perf_counter()
            elif t.gc_started:
                gen = info["generation"]
                t.gc_count[gen] += 1
                t.gc_max_ms[gen] = max(t.gc_max_ms[gen], (time.perf_counter() - t.gc_started) * 1000)

        gc.callbacks.append(callback)

    def freeze_gc(self) -> None:
        """After start-up: everything allocated so far (the SDK, onnxruntime, MuJoCo's Python objects, DDS types) is
        moved out of the collector's reach, and a full collection is made unlikely during the run. A full collection
        of a heap this size is a pause in a loop that has 2 ms per step; the objects the loop allocates afterwards are
        short-lived and are freed by reference counting, so the young generations still run."""
        gc.collect()
        gc.freeze()
        gc.set_threshold(700, 10, 1_000_000)  # generation 2 only after a million generation-1 collections

    def set_velocity(self, vx: float, vy: float, wz: float, duration: float) -> None:
        """Command a velocity for `duration` simulated seconds; it returns to zero after that."""
        clipped = tuple(float(np.clip(v, lo, hi)) for v, (lo, hi) in zip((vx, vy, wz), LIMITS))
        self.last_command = clipped
        self._velocity = (*clipped, self.sim_time + float(duration))
        self.note_command("sport")

    def set_arm(self, command: ArmCommand | None) -> None:
        self._arm = command
        self.note_command("arm_sdk")

    def set_hand(self, side: str, command: HandCommand) -> None:
        self._hand[side] = command
        self.note_command(f"hand_{side}")

    def request(self, fn, *args, timeout: float = 5.0):
        """Run `fn(*args)` on the simulation thread between two steps and return its result (or raise its error)."""
        done = threading.Event()
        box: list = []
        self._requests.put((fn, args, box, done))
        if not done.wait(timeout):
            raise TimeoutError("the simulation did not answer in time")
        if isinstance(box[0], Exception):
            raise box[0]
        return box[0]

    # ------------------------------------------------------------------------------------------ physics

    def _command(self) -> tuple[float, float, float]:
        vx, vy, wz, until = self._velocity
        return (vx, vy, wz) if self.sim_time < until else (0.0, 0.0, 0.0)

    def step(self) -> None:
        m, d, r = self.model, self.data, self.robot
        target = self.stance.targets(self._command(), self.n % POLICY_EVERY == 0)
        q, dq = d.qpos[r.qadr], d.qvel[r.dadr]
        tau = self.policy.kp * (target - q) - self.policy.kd * dq  # PD in SDK joint order, the policy's gains
        arm = self._arm
        if arm is not None and arm.weight > 0:
            w = min(arm.weight, 1.0)
            user = arm.kp * (arm.q - q) + arm.kd * (arm.dq - dq) + arm.tau
            tau[12:N_BODY] = (1 - w) * tau[12:N_BODY] + w * user[12:N_BODY]
        ctrl = np.zeros(m.nu)
        ctrl[:N_BODY] = tau
        for side, first in HAND_ACT.items():
            hq, hdq = d.qpos[r.hand_qadr[side]], d.qvel[r.hand_dadr[side]]
            c = self._hand[side]
            if c is None:
                ctrl[first:first + 7] = HAND_DEFAULT[0] * (0 - hq) - HAND_DEFAULT[1] * hdq
            else:
                ctrl[first:first + 7] = c.kp * (c.q - hq) + c.kd * (c.dq - hdq) + c.tau
        d.ctrl[:] = np.clip(ctrl, self._lo, self._hi)
        if self._push is not None:
            force, torque, until = self._push
            on = self.sim_time < until
            d.xfrc_applied[self._torso_dof_body, :3] = force if on else 0.0
            d.xfrc_applied[self._torso_dof_body, 3:] = torque if on else 0.0
            if not on:
                self._push = None
        mujoco.mj_step(m, d)
        self.n += 1
        self.sim_time = float(d.time)
        if self.fell_at is None and self.fallen_now():
            self.fell_at = self.sim_time
        if self.on_step is not None:
            self.on_step(self)

    def fallen_now(self) -> bool:
        d, r = self.data, self.robot
        w, x, y, z = d.xquat[r.pelvis]
        return bool(d.xpos[r.pelvis][2] < FALLEN_HEIGHT or 1 - 2 * (x * x + y * y) < FALLEN_UP)

    # ------------------------------------------------------------------------------------------ attach / detach

    def _eq(self, obj: str, side: str) -> int:
        try:
            return self.model.equality(f"attach_{obj}_{side}").id
        except KeyError:
            raise ValueError(f"unknown object {obj!r} or hand {side!r}") from None

    def attach(self, obj: str, side: str, teleport: bool = False) -> dict:
        """Weld an object to a hand (simulation thread only). With `teleport` the object first moves to the palm."""
        m, d = self.model, self.data
        eq = self._eq(obj, side)
        if obj in self.attached:
            raise ValueError(f"{obj} is already attached to the {self.attached[obj]} hand")
        palm, body = m.body(HAND_BODY[side]).id, m.body(obj).id
        if teleport:
            jadr = int(m.jnt_qposadr[m.body_jntadr[body]])
            dadr = int(m.jnt_dofadr[m.body_jntadr[body]])
            d.qpos[jadr:jadr + 3] = d.xpos[palm] + d.xmat[palm].reshape(3, 3) @ GRASP_POS
            d.qpos[jadr + 3:jadr + 7] = d.xquat[palm]
            d.qvel[dadr:dadr + 6] = 0
            mujoco.mj_forward(m, d)
        rot = d.xmat[palm].reshape(3, 3)
        rel_pos = rot.T @ (d.xpos[body] - d.xpos[palm])
        rel_quat, inv = np.zeros(4), np.zeros(4)
        mujoco.mju_negQuat(inv, d.xquat[palm])
        mujoco.mju_mulQuat(rel_quat, inv, d.xquat[body])
        m.eq_data[eq, :11] = [0, 0, 0, *rel_pos, *rel_quat, 1]
        geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == body]
        self._saved_collision[obj] = [(g, int(m.geom_contype[g]), int(m.geom_conaffinity[g])) for g in geoms]
        for g in geoms:  # a held object does not push on the hand it is welded to
            m.geom_contype[g] = m.geom_conaffinity[g] = 0
        d.eq_active[eq] = 1
        self.attached[obj] = side
        self.note_command("attach")
        return {"object": obj, "hand": side, "relative_position": [round(float(x), 4) for x in rel_pos]}

    def set_stance(self, model: str) -> dict:
        """Switch the balance support (simulation thread). Only while the robot stands still: `supported` welds the
        pelvis
        where it is, `free` lets go and hands the legs to the policy from a clean history."""
        if model not in STANCES:
            raise ValueError(f"unknown stance {model!r}")
        if model != self.stance.name:
            if not self.stance.standing:
                raise ValueError("the robot is not standing still")
            if hasattr(self.stance, "_release"):
                self.stance._release()
            self.stance = make_stance(model, Body(self.model, self.data, self.robot, self.policy))
        return {"model": self.stance.name, "phase": self.stance.phase}

    def detach(self, obj: str) -> dict:
        m, d = self.model, self.data
        if obj not in self.attached:
            raise ValueError(f"{obj} is not attached")
        side = self.attached.pop(obj)
        self.note_command("detach")
        d.eq_active[self._eq(obj, side)] = 0
        for g, contype, conaffinity in self._saved_collision.pop(obj):
            m.geom_contype[g], m.geom_conaffinity[g] = contype, conaffinity
        return {"object": obj, "hand": side}

    # ------------------------------------------------------------------------------------------ oracle

    def _marker_table(self) -> dict:
        """name -> (geom id, marker id, side length): every marker plate in the scene."""
        table = {}
        for obj in self.site["objects"]:
            if obj.get("marker"):
                table[f"marker_{obj['name']}"] = (obj["marker"]["id"], obj["marker"]["size"])
        for station in self.site["stations"]:
            table[f"marker_station_{station['name']}"] = (station["marker"]["id"], station["marker"]["size"])
        return {name: (self.model.geom(f"{name}_plate").id, mid, size) for name, (mid, size) in table.items()}

    def oracle(self) -> dict:
        """Ground truth for the harness. Poses are world frame; quaternions are (w, x, y, z)."""
        d, r = self.data, self.robot

        def pose(body):
            return {"pos": np.round(d.xpos[body], 5).tolist(), "quat": np.round(d.xquat[body], 5).tolist()}

        return {
            "sim_time": self.sim_time,
            "wall_time": time.time(),
            "pelvis": pose(r.pelvis),
            "torso": pose(r.torso),
            "head_camera": {"pos": np.round(d.cam_xpos[r.camera], 5).tolist(),
                            "mat": np.round(d.cam_xmat[r.camera], 6).tolist()},
            "objects": {o["name"]: pose(self.model.body(o["name"]).id) for o in self.site["objects"]},
            "markers": {name: {"id": mid, "size": size, "pos": np.round(d.geom_xpos[g], 5).tolist()}
                        for name, (g, mid, size) in self._markers.items()},
            "attached": dict(self.attached),
            "imu": self.imu.truth(),
            "hands": {side: np.round(d.qpos[r.hand_qadr[side]], 4).tolist() for side in ("left", "right")},
            "wrists": {side: np.round(d.xpos[self.model.body(HAND_BODY[side]).id], 4).tolist()
                       for side in ("left", "right")},
            "commands": {k: dict(v) for k, v in self.commands.items()},
            "stance": {"model": self.stance.name, "phase": self.stance.phase, "standing": self.stance.standing},
            "velocity_command": list(self._command()),
            "command_until": self._velocity[3],
            "fallen": self.fell_at is not None,
            "fell_at": self.fell_at,
        }

    @staticmethod
    def _write_atomic(path: Path, data: bytes, tmp_suffix: str) -> None:
        tmp = path.with_name(path.stem + tmp_suffix + path.suffix)
        tmp.write_bytes(data)
        os.replace(tmp, path)

    # ------------------------------------------------------------------------------------------ camera

    def _camera_loop(self) -> None:
        import cv2

        width, height = self.site["camera"]["width"], self.site["camera"]["height"]
        try:  # rendering yields to physics when the CPU is short; the renderer's worker threads inherit this
            os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), CAMERA_NICE)
        except OSError:
            pass
        renderer = mujoco.Renderer(self.model, height=height, width=width)  # the GL context belongs to this thread
        options = mujoco.MjvOption()
        options.geomgroup[0] = options.geomgroup[scene.HIDDEN] = 0  # collision meshes, parts the camera never sees
        renderer.update_scene(self._cam_buffers[0], camera="head", scene_option=options)
        renderer.render()  # the first frame pays for shader and buffer set-up: do it before the clock starts
        head = self.run_dir / "head"
        head.mkdir(parents=True, exist_ok=True)
        self._camera_ready.set()
        frame = 0
        while not self._stop.is_set():
            try:
                snapshot, sim_time = self._cam_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            started = time.perf_counter()
            if self.camera_covered:
                rgb = np.zeros((height, width, 3), np.uint8)  # a covered lens: black
            else:
                renderer.update_scene(snapshot, camera="head", scene_option=options)
                rgb = renderer.render()
            ok, png = cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_PNG_COMPRESSION, 1])
            if ok:
                frame += 1
                self._write_atomic(head / "latest.png", png.tobytes(), ".tmp")
                meta = {"frame": frame, "sim_time": sim_time, "wall_time": time.time(), "width": width,
                        "height": height}
                self._write_atomic(head / "latest.json", json.dumps(meta).encode(), ".tmp")
                self.timing.camera_frames = frame
                self.timing.camera_stamps.append(time.perf_counter())
            self.timing.camera_ms = (time.perf_counter() - started) * 1000
            self._cam_busy = False
        del renderer

    def _hand_frame_to_camera(self) -> None:
        """Called from the simulation thread: copy the state for the camera thread, unless it is still busy."""
        if self._cam_busy:
            self.timing.camera_dropped += 1
            return
        snapshot = self._cam_buffers[self._cam_next]
        self._cam_next ^= 1
        mujoco.mj_copyData(snapshot, self.model, self.data)
        self._cam_busy = True
        self._cam_queue.put_nowait((snapshot, self.sim_time))

    # ------------------------------------------------------------------------------------------ the loop

    def _step_stats(self) -> dict | None:
        """Compute time of one loop pass (service, physics, PD, policy, camera hand-off, oracle) vs the step budget."""
        ms = sorted(self.timing.step_ms)
        if not ms:
            return None
        budget = float(self.model.opt.timestep) * 1000
        return {"budget": budget, "mean": round(sum(ms) / len(ms), 3), "p99": round(ms[int(0.99 * (len(ms) - 1))], 3),
                "max": round(ms[-1], 1), "over_budget": round(sum(m > budget for m in ms) / len(ms), 4)}

    def _close_window(self, now: float) -> None:
        """One simulated second has passed, at wall time `now`: its real-time ratio, and for a slow one (below 0.95) how
        much of the second the loop thread spent computing and how much waiting in the run queue for a CPU."""
        t = self.timing
        ratio = 1.0 / max(now - t.window_wall, 1e-9)
        t.window_ratio_min = min(t.window_ratio_min, ratio)
        sched = thread_sched()
        if ratio < 0.95:
            used = (None, None) if sched is None or t.sched_prev is None else (
                round(sched[0] - t.sched_prev[0], 3), round(sched[1] - t.sched_prev[1], 3))
            t.slow_windows.append((round(self.sim_time, 2), round(ratio, 3), *used))
        t.sched_prev = sched
        t.window_wall = now

    def status(self) -> dict:
        t = self.timing
        wall = time.perf_counter() - t.wall0 if t.wall0 else 0.0
        stamps = t.camera_stamps[-26:]
        hz = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0
        return {
            "sim_time": self.sim_time,
            "wall_elapsed": round(wall, 3),
            "realtime_ratio": round(self.sim_time / wall, 4) if wall > 0 else None,
            "worst_second_ratio": None if t.window_ratio_min == float("inf") else round(t.window_ratio_min, 4),
            "stalls": t.stalls,
            "slow_windows": t.slow_windows[-10:],
            "step_compute_ms": self._step_stats(),
            "gc": {"collections": list(t.gc_count), "max_ms": [round(x, 2) for x in t.gc_max_ms],
                   "slow_steps": t.slow_steps[-10:], "frozen": gc.get_freeze_count(), "enabled": gc.isenabled(),
                   "threshold": list(gc.get_threshold())},
            "commands": {k: dict(v) for k, v in self.commands.items()},
            "camera": {"frames": t.camera_frames, "dropped": t.camera_dropped, "render_ms": round(t.camera_ms, 1),
                       "hz": round(hz, 2)},
            "stance": {"model": self.stance.name, "phase": self.stance.phase},
            "velocity_command": list(self._command()),
            "fell": self.fell_at is not None,
        }

    def _service(self) -> None:
        while True:
            try:
                fn, args, box, done = self._requests.get_nowait()
            except queue.Empty:
                return
            try:
                box.append(fn(*args))
            except Exception as exc:  # reported to the caller, never fatal to the simulation
                box.append(exc)
            done.set()

    def start(self) -> None:
        if self.camera:
            thread = threading.Thread(target=self._camera_loop, name="head-camera", daemon=True)
            thread.start()
            self._threads.append(thread)
            if not self._camera_ready.wait(60):
                raise RuntimeError("the camera thread did not start (is MUJOCO_GL=egl or osmesa set?)")

    def run(self, stop: threading.Event | None = None, realtime: bool = True) -> None:
        """Step the simulation against the wall clock until `stop` is set."""
        stop = stop or self._stop
        dt = float(self.model.opt.timestep)
        t = self.timing
        t.wall0 = t.window_wall = time.perf_counter()
        anchor, n0 = t.wall0, self.n
        cam_every = max(1, round(1 / (CAMERA_HZ * dt)))
        oracle_every = max(1, round(1 / (ORACLE_HZ * dt)))
        steps_per_second = round(1 / dt)
        while not stop.is_set():
            loop_start = time.perf_counter()
            gc_before = sum(t.gc_count)
            self._service()
            self.step()
            if self.camera and self.n % cam_every == 0:
                self._hand_frame_to_camera()
            if self.oracle_path is not None and self.n % oracle_every == 0:
                self._write_atomic(self.oracle_path, json.dumps(self.oracle()).encode(), ".tmp")
            if self.n % steps_per_second == 0:
                self._close_window(time.perf_counter())
            took = (time.perf_counter() - loop_start) * 1000
            t.step_ms.append(took)
            if took > 20 and len(t.slow_steps) < 50:
                t.slow_steps.append((round(self.sim_time, 3), round(took, 1), sum(t.gc_count) - gc_before))
            if not realtime:
                continue
            slack = anchor + (self.n - n0) * dt - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            elif slack < -0.25:  # a long stall: count it, and do not race to catch up
                t.stalls += 1
                anchor -= slack

    def stop(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=3)


# ---------------------------------------------------------------------------------------------- DDS


class Bridge:
    """The DDS side: publishers, subscribers and the two RPC services. Created after ChannelFactoryInitialize."""

    def __init__(self, sim: Simulator):
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import (unitree_hg_msg_dds__HandState_, unitree_hg_msg_dds__LowState_)
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_, LowCmd_, LowState_

        self.sim = sim
        self.low = unitree_hg_msg_dds__LowState_()
        self.low.mode_machine = 5  # the 29-DoF G1
        for i in range(N_BODY):
            self.low.motor_state[i].mode = 1
        self.hand_state = {s: unitree_hg_msg_dds__HandState_() for s in ("left", "right")}
        for state in self.hand_state.values():
            for motor in state.motor_state[:7]:
                motor.mode = 1
        # The messages are serialized once and their changing fields patched in place (fastdds.py): the SDK's Python CDR
        # serializer cost 250 us per lowstate, more than half of a physics step. The bytes on the wire are the same.
        ms, imu = self.low.motor_state, self.low.imu_state
        self._low_fast = PatchedSample(
            self.low,
            {
                "q": [lambda v, i=i: setattr(ms[i], "q", v) for i in range(N_BODY)],
                "dq": [lambda v, i=i: setattr(ms[i], "dq", v) for i in range(N_BODY)],
                "tau": [lambda v, i=i: setattr(ms[i], "tau_est", v) for i in range(N_BODY)],
                "quat": [lambda v, i=i: imu.quaternion.__setitem__(i, v) for i in range(4)],
                "gyro": [lambda v, i=i: imu.gyroscope.__setitem__(i, v) for i in range(3)],
                "acc": [lambda v, i=i: imu.accelerometer.__setitem__(i, v) for i in range(3)],
                "rpy": [lambda v, i=i: imu.rpy.__setitem__(i, v) for i in range(3)],
            },
            {"tick": [lambda v: setattr(self.low, "tick", v)]},
        )
        self._hand_fast = {}
        for side, state in self.hand_state.items():
            hm = state.motor_state
            self._hand_fast[side] = PatchedSample(
                state,
                {
                    "q": [lambda v, i=i, hm=hm: setattr(hm[i], "q", v) for i in range(7)],
                    "dq": [lambda v, i=i, hm=hm: setattr(hm[i], "dq", v) for i in range(7)],
                    "tau": [lambda v, i=i, hm=hm: setattr(hm[i], "tau_est", v) for i in range(7)],
                },
            )
        self.low_pub = ChannelPublisher("rt/lowstate", LowState_)
        self.low_pub.Init()
        self.hand_pub = {}
        for side in ("left", "right"):
            self.hand_pub[side] = ChannelPublisher(f"rt/dex3/{side}/state", HandState_)
            self.hand_pub[side].Init()
        self.subs = []
        arm = ChannelSubscriber("rt/arm_sdk", LowCmd_)
        arm.Init(self._on_arm, 10)
        self.subs.append(arm)
        for side in ("left", "right"):
            sub = ChannelSubscriber(f"rt/dex3/{side}/cmd", HandCmd_)
            sub.Init(lambda msg, side=side: self._on_hand(side, msg), 10)
            self.subs.append(sub)
        sim.on_step = self.publish

    def _on_arm(self, msg) -> None:
        cmds = msg.motor_cmd
        f = np.array([[c.q, c.dq, c.kp, c.kd, c.tau] for c in cmds[:N_BODY]], float)
        self.sim.set_arm(ArmCommand(float(cmds[N_BODY].q), f[:, 0], f[:, 1], f[:, 2], f[:, 3], f[:, 4]))

    def _on_hand(self, side: str, msg) -> None:
        cmds = msg.motor_cmd[:7]
        if len(cmds) < 7:
            return
        f = np.array([[c.q, c.dq, c.kp, c.kd, c.tau] for c in cmds], float)
        self.sim.set_hand(side, HandCommand(f[:, 0], f[:, 1], f[:, 2], f[:, 3], f[:, 4]))

    def publish(self, sim: Simulator) -> None:
        if sim.n % LOWSTATE_EVERY == 0:
            self._publish_low(sim)
        if sim.n % HAND_EVERY == 0:
            self._publish_hands(sim)

    def _publish_low(self, sim: Simulator) -> None:
        d, r = sim.data, sim.robot
        fast = self._low_fast
        fast.set("q", d.qpos[r.qadr])
        fast.set("dq", d.qvel[r.dadr])
        fast.set("tau", d.actuator_force[:N_BODY])
        quat, gyro, acc = sim.imu.read(sim.sim_time, d.sensordata[r.imu_quat:r.imu_quat + 4],
                                       d.sensordata[r.imu_gyro:r.imu_gyro + 3], d.sensordata[r.imu_acc:r.imu_acc + 3])
        fast.set("quat", quat)
        fast.set("gyro", gyro)
        fast.set("acc", acc)
        w, x, y, z = quat
        fast.set("rpy", [np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                         np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
                         np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))])
        fast.set_int("tick", int(sim.sim_time * 1000) & 0xFFFFFFFF)  # simulation time, ms
        self.low_pub.Write(self.low)

    def _publish_hands(self, sim: Simulator) -> None:
        d, r = sim.data, sim.robot
        for side in ("left", "right"):
            fast = self._hand_fast[side]
            fast.set("q", d.qpos[r.hand_qadr[side]])
            fast.set("dq", d.qvel[r.hand_dadr[side]])
            fast.set("tau", d.actuator_force[HAND_ACT[side]:HAND_ACT[side] + 7])
            self.hand_pub[side].Write(self.hand_state[side])


def make_services(sim: Simulator, test_hooks: bool = False):
    """The `sport` and `robolang_sim` RPC services (SDK `Server` subclasses); call Init() and Start() on both."""
    from unitree_sdk2py.g1.loco import g1_loco_api as api
    from unitree_sdk2py.rpc.internal import RPC_ERR_SERVER_API_PARAMETER
    from unitree_sdk2py.rpc.server import Server

    class Sport(Server):
        def __init__(self):
            super().__init__(api.LOCO_SERVICE_NAME)

        def Init(self):
            self._SetApiVersion(api.LOCO_API_VERSION)
            self._RegistHandler(api.ROBOT_API_ID_LOCO_GET_FSM_ID, self.get_fsm_id, False)
            self._RegistHandler(api.ROBOT_API_ID_LOCO_SET_FSM_ID, self.set_fsm_id, False)
            self._RegistHandler(api.ROBOT_API_ID_LOCO_SET_VELOCITY, self.set_velocity, False)

        def get_fsm_id(self, parameter):
            return 0, json.dumps({"data": 500})

        def set_fsm_id(self, parameter):
            fsm = json.loads(parameter).get("data")
            if fsm == 500:  # Start: the stand-in is always running
                return 0, ""
            return ERR_REFUSED, ""  # Damp (1), zero torque (0), sit, squat...: the robot is never let fall

        def set_velocity(self, parameter):
            p = json.loads(parameter)
            velocity, duration = p.get("velocity"), p.get("duration", 1.0)
            if (not isinstance(velocity, list) or len(velocity) != 3 or not isinstance(duration, (int, float))
                    or duration <= 0 or not all(isinstance(v, (int, float)) for v in velocity)):
                return RPC_ERR_SERVER_API_PARAMETER, ""
            sim.set_velocity(*velocity, duration)
            return 0, ""

    class SimService(Server):
        def __init__(self):
            super().__init__(SIM_SERVICE)

        def Init(self):
            self._SetApiVersion(SIM_API_VERSION)
            self._RegistHandler(API_IDENTITY, self.identity, False)
            self._RegistHandler(API_STATUS, lambda p: (0, json.dumps(sim.status())), False)
            self._RegistHandler(API_ATTACH, self.attach, False)
            self._RegistHandler(API_DETACH, self.detach, False)
            self._RegistHandler(API_SET_STANCE, self.set_stance, False)
            if test_hooks:  # only with --test-hooks: a push for the fall test, a covered camera, a moved object
                self._RegistHandler(API_PUSH, self.push, False)
                self._RegistHandler(API_COVER_CAMERA, self.cover_camera, False)
                self._RegistHandler(API_PLACE_OBJECT, self.place_object, False)

        def set_stance(self, parameter):
            try:
                return 0, json.dumps(sim.request(sim.set_stance, json.loads(parameter)["model"]))
            except (ValueError, KeyError):
                return RPC_ERR_SERVER_API_PARAMETER, ""

        def push(self, parameter):
            p = json.loads(parameter)
            return 0, json.dumps(sim.request(sim.push, p["force"], p["duration"], p.get("torque", (0.0, 0.0, 0.0))))

        def cover_camera(self, parameter):
            return 0, json.dumps(sim.request(sim.cover_camera, bool(json.loads(parameter)["covered"])))

        def place_object(self, parameter):
            p = json.loads(parameter)
            try:
                return 0, json.dumps(sim.request(sim.place_object, p["object"], p["position"], p.get("yaw", 0.0)))
            except (ValueError, KeyError):
                return RPC_ERR_SERVER_API_PARAMETER, ""

        def identity(self, parameter):
            return 0, json.dumps({"simulation": True, "domain": DOMAIN, "interface": INTERFACE,
                                  "sim_time": sim.sim_time, "stance_model": sim.stance.name,
                                  "grasp_model": "weld_after_hand_close",
                                  # where the head camera's frames are (on the robot the camera is read directly)
                                  "head_camera_dir": str(sim.run_dir / "head") if sim.run_dir else None})

        def attach(self, parameter):
            p = json.loads(parameter)
            try:
                return 0, json.dumps(sim.request(sim.attach, p["object"], p["hand"], bool(p.get("teleport", False))))
            except (ValueError, KeyError):
                return RPC_ERR_SERVER_API_PARAMETER, ""

        def detach(self, parameter):
            try:
                return 0, json.dumps(sim.request(sim.detach, json.loads(parameter)["object"]))
            except (ValueError, KeyError):
                return RPC_ERR_SERVER_API_PARAMETER, ""

    return Sport(), SimService()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--site", required=True, help="site file (JSON, schema robolang-g1-site/1)")
    parser.add_argument("--run-dir", required=True, help="where the head camera frames go (DIR/head/)")
    parser.add_argument("--oracle", help="write ground truth for the test harness to this file (the only reader of "
                                         "MuJoCo state besides the simulator itself)")
    parser.add_argument("--stance", default="free", choices=sorted(STANCES),
                        help="what the robot does when it stands still: free (the policy alone) or "
                             "supported (the torso welded to a mocap body; the approved fallback)")
    parser.add_argument("--imu-seed", type=int, help="the seed of the IMU's bias and noise (default: random, printed)")
    parser.add_argument("--imu-bias", type=float,
                        help="the gyro's z bias in rad/s, instead of the drawn one (a worst case for the stress check)")
    parser.add_argument("--ideal-imu", action="store_true",
                        help="measurement only: publish MuJoCo's exact IMU (no bias, no noise, no yaw drift)")
    parser.add_argument("--no-gc-freeze", action="store_true",
                        help="measurement only: leave the garbage collector as it is (to compare step_compute_ms)")
    parser.add_argument("--test-hooks", action="store_true",
                        help="harness only: also answer robolang_sim's push (a force on the torso, for the fall test)")
    args = parser.parse_args(argv)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    sim = Simulator(args.site, stance=args.stance, run_dir=args.run_dir, oracle=args.oracle, imu_seed=args.imu_seed,
                    imu_bias=args.imu_bias, imu_ideal=args.ideal_imu)
    sim.watch_gc()
    ChannelFactoryInitialize(DOMAIN, INTERFACE)  # domain 1 and loopback are not options
    bridge = Bridge(sim)
    services = make_services(sim, args.test_hooks)
    for service in services:
        service.Init()
        service.Start()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    sim.start()
    if not args.no_gc_freeze:
        sim.freeze_gc()
    print(json.dumps({"ready": True, "pid": os.getpid(), "domain": DOMAIN, "interface": INTERFACE,
                      "stance_model": sim.stance.name, "run_dir": str(args.run_dir), "imu": sim.imu.truth(),
                      "versions": assets.versions()}),
          flush=True)
    try:
        sim.run(stop)
    finally:
        sim.stop()
        del bridge
    print(json.dumps({"stopped": True, **sim.status()}), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
