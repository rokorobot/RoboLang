# ruff: noqa: E501, E741, E702
"""Experiment (reads the oracle; not the backend): Lowstate signals (IMU rate, joint velocities) against the oracle's pelvis speed, standing, walking, turning, coasting.

Run from the repository root with a stand-in-capable environment (see docs/evidence/g1-phase2-executor.md).
"""

import time
import tempfile
import math
import numpy as np
from pathlib import Path
from robolang_g1 import guard
from robolang_g1.harness import StandIn, OracleTail, yaw_of
from robolang_g1.robot import Robot

np.set_printoptions(precision=3, suppress=True)
run = Path(tempfile.mkdtemp(prefix="exp_"))
si = StandIn("backends/unitree_g1/robolang_g1/sites/pick_place.json", run, "free", ["--test-hooks"])
si.wait_ready()
client, ident = guard.require_simulation(timeout=30)
tail = OracleTail(si.oracle)
tail.start()
tail.wait(lambda s: s["sim_time"] > 0, 30)
robot = Robot()
robot.wait_state()


def stats(t0, t1):
    s = [x for x in list(robot.history) if t0 <= x.tick <= t1]
    if not s:
        return None
    gyro = max(np.linalg.norm(x.gyro) for x in s)
    dql = max(np.max(np.abs(x.dq[:12])) for x in s)
    dqa = max(np.max(np.abs(x.dq[12:])) for x in s)
    up = [1 - 2 * (x.quat[1] ** 2 + x.quat[2] ** 2) for x in s]
    tilt = math.degrees(math.acos(min(up)))
    return gyro, dql, dqa, tilt


def pelvis_speed(t0, t1):
    o = tail.between(t0 / 1000, t1 / 1000)
    if len(o) < 2:
        return None
    p = np.array([x["pelvis"]["pos"][:2] for x in o])
    dt = o[-1]["sim_time"] - o[0]["sim_time"]
    yaw = [yaw_of(x["pelvis"]["quat"]) for x in o]
    return float(np.linalg.norm(p[-1] - p[0]) / dt), abs(yaw[-1] - yaw[0]) / dt


def timeline(label, seconds, step=500):
    t = robot.now_ms
    for k in range(int(seconds * 1000 / step)):
        time.sleep(step / 1000)
        st = stats(t + k * step, t + (k + 1) * step)
        sp = pelvis_speed(t + k * step, t + (k + 1) * step)
        print(
            f"{label} t+{(k + 1) * step / 1000:4.1f}s gyro {st[0]:.3f} dq_leg {st[1]:.3f} dq_arm {st[2]:.3f} tilt {st[3]:4.1f} | pelvis v {sp[0]:.3f} m/s w {sp[1]:.3f} rad/s"
        )


time.sleep(5)
timeline("stand", 6)
t_before = tail.latest()
print("walk 0.5 x 3s")
robot.set_velocity(0.5, 0, 0, 3.0)
timeline("walk", 3)
timeline("coast", 5)
print("turn 0.2 x 4s")
y0 = yaw_of(tail.latest()["pelvis"]["quat"])
p0 = tail.latest()["pelvis"]["pos"]
robot.set_velocity(0, 0, 0.2, 4.0)
timeline("turn", 4)
timeline("coast", 4)
y1 = yaw_of(tail.latest()["pelvis"]["quat"])
p1 = tail.latest()["pelvis"]["pos"]
print("turn commanded 0.8 rad, yaw change", y1 - y0, "drift", np.linalg.norm(np.array(p1[:2]) - np.array(p0[:2])))
print("walk 0.15 x 2s")
p0 = tail.latest()["pelvis"]["pos"]
robot.set_velocity(0.15, 0, 0, 2.0)
timeline("slow", 2)
timeline("coast", 4)
p1 = tail.latest()["pelvis"]["pos"]
print("slow walk commanded 0.30 m, moved", np.linalg.norm(np.array(p1[:2]) - np.array(p0[:2])))
print("side 0.15 x 2s")
p0 = tail.latest()["pelvis"]["pos"]
robot.set_velocity(0, 0.15, 0, 2.0)
time.sleep(6)
p1 = tail.latest()["pelvis"]["pos"]
print("side commanded 0.30 m, moved", np.array(p1[:2]) - np.array(p0[:2]))
print("tiny 0.05 x 2s")
p0 = tail.latest()["pelvis"]["pos"]
robot.set_velocity(0.05, 0, 0, 2.0)
time.sleep(6)
p1 = tail.latest()["pelvis"]["pos"]
print("tiny commanded 0.10 m, moved", np.linalg.norm(np.array(p1[:2]) - np.array(p0[:2])))
tail.stop()
print(si.stop() and "stopped")
