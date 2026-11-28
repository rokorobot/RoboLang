# Measurement only (g1-phase4-acceptance.md, F2). Needs the backend installed; writes under /tmp/f2tools.
"""Measurement (F2): the pelvis pose each station marker implies, against the oracle's, over many viewpoints.
measure.py SCENE OUT.json"""

import json
import math
import sys
import time
from pathlib import Path
import numpy as np
from robolang_g1 import scenes, site as S
from robolang_g1.ops import G1Operations, MARKER_MIN_CONF, MARKER_MAX_RANGE_M
from robolang_g1.perception import MAX_UP_DISAGREEMENT_RAD, pelvis_pose_in_map
from robolang_g1.run_task import Session

scene, out = sys.argv[1], Path(sys.argv[2])
d = Path("/tmp/f2tools/run_" + scene)
built = scenes.write(scene, d / "scene")
site = S.load_site(d / "scene" / "site.json")
plates = {s["marker"]["id"]: s["marker"] for s in site["stations"]}
targets = []
for x in (1.1, 1.6, 2.0, 2.3):
    for y in (0.0, 0.7, 1.4, 2.1, 2.6):
        for yaw in (0.0, 0.45, -0.45):
            targets.append((x, y, yaw))
rows = []
with Session(d / "scene" / "scene.json", d / "run", "free") as s:
    ops = G1Operations(site)
    ops.guard(site["name"])
    t_end = time.monotonic() + float(sys.argv[3] if len(sys.argv) > 3 else 1500)
    for k, (x, y, yaw) in enumerate(targets):
        if time.monotonic() > t_end:
            break
        try:
            ops.walk_to(x, y, yaw, stop_short=0.15, deadline_ms=ops.now_ms() + 60000)
            ops.turn_to(yaw, tolerance=0.15)
        except Exception as exc:  # a walk that fails still leaves a viewpoint
            print("walk", type(exc).__name__, str(exc)[:160], flush=True)
        ops.settle()
        readings = ops.perc.observe(ids=set(plates), frames=5)
        truth = s.tail.latest()
        tp = truth["pelvis"]["pos"]
        w, qx, qy, qz = truth["pelvis"]["quat"]
        tyaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        w, a, b, c = ops.robot.last.quat
        imu_up = np.array([2 * (a * c - w * b), 2 * (b * c + w * a), 1 - 2 * (a * a + b * b)])
        for mid, r in readings.items():
            px, py, pyaw, up = pelvis_pose_in_map(plates[mid], r)
            m = plates[mid]
            vx, vy = tp[0] - m["pos"][0], tp[1] - m["pos"][1]
            ang = math.degrees(
                math.acos(
                    max(-1, min(1, (vx * math.cos(m["facing"]) + vy * math.sin(m["facing"])) / math.hypot(vx, vy)))
                )
            )
            dis = float(np.degrees(np.arccos(np.clip(up @ imu_up, -1, 1))))
            accepted = (
                r.confidence >= MARKER_MIN_CONF
                and r.range_m <= MARKER_MAX_RANGE_M
                and math.radians(dis) <= MAX_UP_DISAGREEMENT_RAD
            )
            rows.append(
                {
                    "k": k,
                    "marker": mid,
                    "angle_deg": round(ang, 1),
                    "range_m": round(r.range_m, 3),
                    "confidence": r.confidence,
                    "up_vs_imu_deg": round(dis, 2),
                    "accepted": accepted,
                    "error_m": round(math.hypot(px - tp[0], py - tp[1]), 3),
                    "error_yaw_rad": round(abs(S.wrap(pyaw - tyaw)), 3),
                    "true": [round(tp[0], 3), round(tp[1], 3), round(tyaw, 3)],
                    "implied": [round(px, 3), round(py, 3), round(pyaw, 3)],
                }
            )
        out.write_text(json.dumps(rows))
        tp0 = s.tail.latest()["pelvis"]["pos"]
        print(k, len(rows), [round(v, 2) for v in tp0[:2]], sorted(readings), flush=True)
print("done", len(rows))
