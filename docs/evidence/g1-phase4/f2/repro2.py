# Measurement only (g1-phase4-acceptance.md, F2). Needs the backend installed; writes under /tmp/f2tools.
import json
import subprocess
import sys
import time
from pathlib import Path
from robolang_g1 import scenes
from robolang_g1.run_task import Session
from robolang_g1.phase2 import GO

tag, n, start = sys.argv[1], int(sys.argv[2]), json.loads(sys.argv[3])
for k in range(n):
    d = Path(f"/tmp/f2tools/r2_{tag}_{k}")
    built = scenes.build("w_split_boxes")
    for key in ("site", "world"):
        built[key]["robot"]["start"] = start
    d.mkdir(parents=True, exist_ok=True)
    (d / "site.json").write_text(json.dumps(built["site"]))
    (d / "scene.json").write_text(json.dumps(built["world"]))
    (d / "go.rob").write_text(GO)
    with Session(d / "scene.json", d / "run", "free") as s:
        s.sim.Attach("a", "right", True)
        s.sim.Attach("b", "left", True)
        time.sleep(2.0)
        cmd = [
            sys.executable,
            str(Path(__file__).with_name("cli_patched.py")),
            str(d / "go.rob"),
            "--task",
            "go",
            "--site",
            str(d / "site.json"),
            "--out",
            str(d / "record.json"),
            "--bind",
            "r=arm1",
            "--bind",
            "a=conveyor",
        ]
        subprocess.run(cmd, stdout=open(d / "cli.log", "w"), stderr=subprocess.STDOUT, timeout=600)
        final = s.tail.latest()
    rec = json.loads((d / "record.json").read_text()) if (d / "record.json").exists() else None
    ev = [e["event"] for st in (rec or {}).get("steps", []) for e in (st.get("details") or {}).get("events") or []]
    p = final["pelvis"]["pos"]
    print(
        tag,
        k,
        rec and rec["state"],
        rec and (rec.get("failure") or {}).get("code"),
        {e: ev.count(e) for e in ("sidestep", "scan", "approach_again")},
        "final",
        [round(p[0], 2), round(p[1], 2)],
        flush=True,
    )
