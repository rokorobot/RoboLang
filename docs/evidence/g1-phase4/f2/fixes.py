# Measurement only (g1-phase4-acceptance.md, F2). Needs the backend installed; writes under /tmp/f2tools.
"""Measurement (F2): run real tasks through the CLI, keep the oracle's samples, and compare every marker fix the
executor made (its 'localized' events) with where the oracle had the pelvis at that time.
    fixes.py OUT.json SCENE:N [SCENE:N ...]"""

import json
import math
import sys
from pathlib import Path
from robolang_g1 import scenes, site as S
from robolang_g1.run_task import Session

out = Path(sys.argv[1])
rows = []
for spec in sys.argv[2:]:
    scene, n = spec.split(":")
    for k in range(int(n)):
        d = Path(f"/tmp/f2tools/fx_{scene}_{k}")
        built = scenes.write(scene, d / "scene")
        site = S.load_site(d / "scene" / "site.json")
        plates = {s["marker"]["id"]: s["marker"] for s in site["stations"]}
        with Session(d / "scene" / "scene.json", d / "run", "free") as s:
            status, rec, log = s.run(
                built["source"],
                built["task"],
                d / "scene" / "site.json",
                built["bind"],
                d / "record.json",
                timeout=1800,
            )
            samples = list(s.tail.samples)
        times = [x["sim_time"] for x in samples]
        import bisect

        for st in (rec or {}).get("steps", []):
            moving = st["kind"] == "move"
            for e in (st.get("details") or {}).get("events") or []:
                if e.get("event") != "localized":
                    continue
                t = e["t_ms"] / 1000
                i = min(max(bisect.bisect_left(times, t), 0), len(samples) - 1)
                tp = samples[i]["pelvis"]["pos"]
                w, qx, qy, qz = samples[i]["pelvis"]["quat"]
                tyaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                m = plates[e["marker"]]
                vx, vy = tp[0] - m["pos"][0], tp[1] - m["pos"][1]
                ang = math.degrees(
                    math.acos(
                        max(-1, min(1, (vx * math.cos(m["facing"]) + vy * math.sin(m["facing"])) / math.hypot(vx, vy)))
                    )
                )
                true_range = math.sqrt(vx * vx + vy * vy + (1.26 - m["pos"][2]) ** 2)
                rows.append(
                    {
                        "run": f"{scene}_{k}",
                        "step": st["index"],
                        "kind": st["kind"],
                        "marker": e["marker"],
                        "true_angle_deg": round(ang, 1),
                        "true_range_m": round(true_range, 2),
                        "error_m": round(math.hypot(e["to"][0] - tp[0], e["to"][1] - tp[1]), 3),
                        "error_yaw": round(abs(S.wrap(e["to"][2] - tyaw)), 3),
                        "jump_m": round(math.hypot(e["to"][0] - e["from"][0], e["to"][1] - e["from"][1]), 3),
                        "state": rec and rec["state"],
                    }
                )
        out.write_text(json.dumps(rows))
        print(scene, k, rec and rec["state"], rec and (rec.get("failure") or {}).get("code"), len(rows), flush=True)
print("done", len(rows))
