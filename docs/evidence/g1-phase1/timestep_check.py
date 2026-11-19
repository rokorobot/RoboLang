"""G1 phase 1: why the physics step is 0.002 s and not 0.005 s (decision 0046 U13 said 0.005).

Usage: timestep_check.py   (needs the assets and robolang-g1; takes about a minute)

The stand-in applies the policy's PD law as a torque on every physics step, like the robot's low-level loop does. MuJoCo
integrates a torque explicitly, so a velocity feedback gain kd on a joint with effective inertia I is stable only if
dt x kd / I < 2. For the G1's arm joints (kd = 10 in the policy's deploy.yaml, I about 0.02 kg m^2 for the shoulder yaw
and the wrists) that is 2.5 or more at dt 0.005. The arm torque then chatters between its limits, the arms sit far
from their targets, and the whole robot drifts. This script prints the inertia table and, for both steps, what the
robot does: arm torque saturation, drift at rest, a 4 s walk, and standing free after the walk.

It reads MuJoCo state: it is an experiment, not the backend.
"""
import numpy as np
import mujoco

from robolang_g1 import assets
from robolang_g1.sim import scene
from robolang_g1.sim.policy import WholeBodyPolicy

SITE = scene.load_site({"schema": scene.SITE_SCHEMA, "robot": {"start": [0, 0, 0]}})


def arm_inertia(m, d, robot):
    """Effective inertia of each arm joint (the mass matrix diagonal, armature included), joints 15-28."""
    full = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, d, full)
    return full[robot.dadr[15:29], robot.dadr[15:29]]


def experiment(dt):
    scene.TIMESTEP = dt
    m, robot, _ = scene.build(SITE)
    d = mujoco.MjData(m)
    pol_dir = assets.asset(*assets.POLICY_DIR)
    pol = WholeBodyPolicy(pol_dir / "exported" / "policy.onnx", pol_dir / "params" / "deploy.yaml")
    d.qpos[robot.qadr] = pol.default
    mujoco.mj_forward(m, d)
    inertia = arm_inertia(m, d, robot)
    every = round(0.02 / dt)
    lim = m.actuator_ctrlrange[:, 1]
    state = {"n": 0, "target": pol.default.copy(), "sat": 0, "steps": 0, "command": [0, 0, 0]}
    trail = []  # pelvis position every 0.02 s

    def step():
        if state["n"] % every == 0:
            state["target"] = pol.act(d.sensordata[robot.imu_gyro:robot.imu_gyro + 3],
                                      d.sensordata[robot.imu_quat:robot.imu_quat + 4], state["command"],
                                      d.qpos[robot.qadr], d.qvel[robot.dadr])
            trail.append((d.time, d.xpos[robot.pelvis].copy(), d.xpos[robot.torso].copy()))
        tau = pol.kp * (state["target"] - d.qpos[robot.qadr]) - pol.kd * d.qvel[robot.dadr]
        d.ctrl[:29] = np.clip(tau, -lim[:29], lim[:29])
        state["sat"] += int(np.sum(np.abs(tau[15:29]) >= lim[15:29]))
        state["steps"] += 1
        mujoco.mj_step(m, d)
        state["n"] += 1

    def run(seconds):
        for _ in range(round(seconds / dt)):
            step()

    result = {"dt": dt, "dt_kd_over_I_arm_joints_max": round(float(np.max(dt * pol.kd[15:29] / inertia)), 2),
              "arm_inertia_min_max": [round(float(inertia.min()), 4), round(float(inertia.max()), 4)]}
    run(3)
    result["arm_torque_saturated_steps_3s_at_rest"] = f"{state['sat'] / (state['steps'] * 14):.0%}"
    result["rest_drift_cm_3s"] = round(float(np.linalg.norm(d.xpos[robot.pelvis][:2]) * 100), 1)
    p0 = d.xpos[robot.pelvis].copy()
    state["command"] = [0.5, 0, 0]
    run(4)
    result["walk_4s_forward_m"] = round(float(d.xpos[robot.pelvis][0] - p0[0]), 2)
    result["walk_4s_sideways_m"] = round(float(d.xpos[robot.pelvis][1] - p0[1]), 2)
    state["command"], t_stop = [0, 0, 0], d.time
    run(20)
    # at rest: at most 1 cm of pelvis path in 1 s
    pts = [(t, p) for t, p, _ in trail if t >= t_stop]
    rest = None
    for i in range(len(pts)):
        j = min(k for k in range(i + 1) if pts[i][0] - pts[k][0] <= 1.0 + 1e-9)
        if pts[i][0] - pts[0][0] >= 1.0 and sum(np.linalg.norm(pts[k + 1][1] - pts[k][1]) for k in range(j, i)) <= 0.01:
            rest = pts[i][0]
            break
    if rest is None:
        result["free_stop"] = "never came to rest within 20 s"
        result["pelvis_moved_in_last_10s_m"] = round(float(np.linalg.norm(pts[-1][1][:2] - pts[-500][1][:2])), 2)
    else:
        coast = np.linalg.norm([p for t, p in pts if t >= rest][0][:2] - pts[0][1][:2])
        result["free_stop"] = {"rest_after_s": round(rest - t_stop, 2), "coast_m": round(float(coast), 3)}
        run(10)
        tor = np.array([q for t, _, q in trail if t >= rest + 0.0][:500])
        result["torso_sway_cm_10s_at_rest"] = np.round((tor.max(0) - tor.min(0)) * 100, 1).tolist()
    result["fell"] = bool(d.xpos[robot.pelvis][2] < 0.45)
    return result


if __name__ == "__main__":
    import json

    for step_size in (0.005, 0.002):
        print(json.dumps(experiment(step_size)), flush=True)
