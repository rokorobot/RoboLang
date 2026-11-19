"""G1 spike: torso sway with the walking policy at zero command vs a static PD stand.

Spike only: reads MuJoCo state as an oracle.
"""

import sys
import json
import numpy as np
import mujoco
import torch

scene, policy_path = sys.argv[1], sys.argv[2]
m = mujoco.MjModel.from_xml_path(scene)
d = mujoco.MjData(m)
dt = 0.002
m.opt.timestep = dt
kp = np.array([100, 100, 100, 150, 40, 40] * 2)
kd = np.array([2, 2, 2, 4, 2, 2] * 2)
dflt = np.array([-0.1, 0, 0, 0.3, -0.2, 0] * 2)
ukp = np.array([200] * 3 + [60] * 14)
ukd = np.array([5] * 3 + [2] * 14)
pol = torch.jit.load(policy_path)


def g(q):
    w, x, y, z = q
    return np.array([2 * (-z * x + w * y), -2 * (z * y + w * x), 1 - 2 * (w * w + z * z)])


def run(mode_seq):
    mujoco.mj_resetData(m, d)
    d.qpos[7:19] = dflt
    mujoco.mj_forward(m, d)
    obs = np.zeros(47, np.float32)
    act = np.zeros(12, np.float32)
    tgt = dflt.copy()
    n = 0
    out = {}
    for name, dur, cmd in mode_seq:
        tr = []
        for _ in range(int(dur / dt)):
            if name == "static":
                tgt = dflt
                kk, dd = kp * 2, kd * 2
            else:
                kk, dd = kp, kd
            tau = np.r_[(tgt - d.qpos[7:19]) * kk - d.qvel[6:18] * dd, (0 - d.qpos[19:36]) * ukp - d.qvel[18:35] * ukd]
            d.ctrl[:] = np.clip(tau, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
            mujoco.mj_step(m, d)
            n += 1
            if name != "static" and n % 10 == 0:
                ph = (n * dt) % 0.8 / 0.8
                obs[:] = np.r_[
                    d.qvel[3:6] * 0.25,
                    g(d.qpos[3:7]),
                    np.array(cmd) * [2, 2, 0.25],
                    d.qpos[7:19] - dflt,
                    d.qvel[6:18] * 0.05,
                    act,
                    np.sin(2 * np.pi * ph),
                    np.cos(2 * np.pi * ph),
                ]
                with torch.no_grad():
                    act = pol(torch.from_numpy(obs)[None]).numpy()[0]
                tgt = act * 0.25 + dflt
            tr.append(d.xpos[m.body("torso_link").id].copy())
        tr = np.array(tr[len(tr) // 3 :])
        out[name] = {
            "torso_sway_cm": np.round((tr.max(0) - tr.min(0)) * 100, 1).tolist(),
            "upright": bool(d.xpos[1][2] > 0.6),
        }
    return out


def blend_run():
    out = run([("walk", 3, [0.5, 0, 0]), ("policy0", 6, [0, 0, 0])])["policy0"]
    return out


print(
    json.dumps(
        {
            "policy_zero_cmd_6s_last4s": blend_run(),
            "static_pd_from_rest": run([("static", 6, None)])["static"],
            "walk_then_long_settle_then_static": run(
                [("walk", 3, [0.5, 0, 0]), ("policy0", 4, [0, 0, 0]), ("static", 6, None)]
            )["static"],
        }
    )
)
