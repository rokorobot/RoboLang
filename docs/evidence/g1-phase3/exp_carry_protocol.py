# ruff: noqa
"""Experiment for phase 3 (reads the oracle; not the backend). Run from the repository root with a stand-in-capable environment (see docs/evidence/g1-phase3-observations.md)."""
"""Walk-with-object protocol: 2 m at 0.5 m/s, stand 10 s, 90 degree shuffle turn; x3. argv: baseline | view [x y z]"""
import json, math, sys, threading, time
from pathlib import Path
import numpy as np
from robolang_g1.run_task import Session
from robolang_g1.phase2 import SITES, _Tail
from robolang_g1 import site as site_mod, ops as OPS
from robolang_g1.ops import G1Operations
from robolang_g1.executor import BackendFailure

mode = sys.argv[1]
target = np.array([float(v) for v in sys.argv[2:5]]) if len(sys.argv) >= 5 else np.array([0.22, -0.15, 0.04])
out = Path(f"/tmp/exp/carry_{mode}"); out.mkdir(exist_ok=True)
raw = json.loads((SITES / "carry_hold.json").read_text())
raw["robot"]["start"] = site_mod.load_site(SITES / "carry_hold.json")["stations"][0]["pose"]
SCENE = out / "scene.json"; SCENE.write_text(json.dumps(raw))
site = site_mod.load_site(SCENE)
MID = 8                                                    # the crate's marker
ARM = list(range(12, 29))

def yaw_of(q): return math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi
def tilt(x):
    q = x["pelvis"]["quat"]; return math.degrees(math.acos(max(-1, min(1, 1 - 2 * (q[1] ** 2 + q[2] ** 2)))))

class Tracker(threading.Thread):
    """arm_sdk target against measured joints, from lowstate, every 20 ms of wall time."""
    def __init__(self, robot):
        super().__init__(daemon=True); self.robot, self.rows, self.stop_ev = robot, [], threading.Event()
    def run(self):
        while not self.stop_ev.is_set():
            tgt = self.robot.arm_target
            if tgt is not None:
                self.rows.append((self.robot.now_ms, np.abs(self.robot.low_q - tgt[0])[ARM].copy()))
            time.sleep(0.02)

def detect_in_background(ops, frames, store):
    def run():
        try:
            readings, got = ops.perc.snapshot(ids={MID}, frames=frames, timeout=40.0)
            r = readings.get(MID)
            store.update(frames=got, detections=0 if r is None else r.detections,
                         confidence=0.0 if r is None else r.confidence, spread_cm=None if r is None else round(r.spread_m * 100, 2),
                         range_m=None if r is None else round(r.range_m, 3))
        except Exception as e:
            store["error"] = repr(e)
    t = threading.Thread(target=run, daemon=True); t.start(); return t

with Session(SCENE, out / "run", "free") as s:
    ops = G1Operations(site); ops.guard("g1"); ops.station = "dock"
    d = ops.grasp("crate")
    print("grasped; attached:", s.tail.latest()["attached"], flush=True)
    print("carry pose:", d.get("carry_pose"), "closest to a declared table:", d["clearance"]["closest"], flush=True)
    result = {"mode": mode, "target_pinch_pelvis": target.tolist(), "runs": []}
    if mode == "view":
        deg = ops.held["fingers_down_deg"]; R = OPS._frame(math.radians(deg))
        q_view, info = ops._solve("right", target, R, [ops._ready_pose("right"), ops.q_carry], 0.0)
        assert q_view is not None, "no IK"
        ops._arm_stance("supported"); ops._arm_begin()
        ops._arm_move(ops._clear_pose("right"), 1.6, label="clear")
        ops._arm_move(q_view, 2.4, label="view")
        ops.sleep(1.0)
        ops._arm_stance("free"); ops.settle(8.0)
        result["fingers_down_deg"] = deg
        result["ik"] = {k: round(float(v), 4) for k, v in info.items() if isinstance(v, (int, float))}
    # a marker reading at rest, and where it is against the field of view
    r0 = ops.perc.snapshot(ids={MID}, frames=10)[0].get(MID)
    Rc, pc = ops.kin.camera_pose(ops.robot.low_q)
    if r0 is not None:
        v = r0.position - pc; depth = v @ (-Rc[:, 2])
        result["marker_at_rest"] = {"confidence": r0.confidence, "range_m": round(r0.range_m, 3),
                                    "vertical_deg": round(math.degrees(math.atan2(v @ Rc[:, 1], depth)), 1),
                                    "horizontal_deg": round(math.degrees(math.atan2(v @ Rc[:, 0], depth)), 1),
                                    "position_pelvis": np.round(r0.position, 3).tolist()}
    else:
        v = None
        result["marker_at_rest"] = None
    print("marker at rest:", result["marker_at_rest"], flush=True)
    # turn round (the table is ahead): 180 degrees by the shuffle
    ops.turn_by(math.pi); ops.settle(8.0)
    trk = Tracker(ops.robot); trk.start()
    for run in range(1, 4):
        tail = _Tail(list(s.tail.samples))
        t_a = ops.now_ms()
        o0 = s.tail.latest(); p0 = np.array(o0["pelvis"]["pos"][:2]); yaw0 = yaw_of(o0["pelvis"]["quat"])
        walk_cam, rest_cam = {}, {}
        th = detect_in_background(ops, 28, walk_cam)
        res = {"run": run, "fell": False}
        try:
            ops.command(0.5, 0.0, 0.0, 4.0)
            t_walk_end = ops.now_ms()
            ops.sleep(2.0)                       # the coast
            th.join(timeout=30)
            cam = detect_in_background(ops, 10, rest_cam)
            ops.sleep(8.0)                       # stand: 10 s in all after the command ended
            cam.join(timeout=30)
            t_rest_end = ops.now_ms()
            o1 = s.tail.latest()
            p1 = np.array(o1["pelvis"]["pos"][:2]); R0 = np.array([[math.cos(yaw0), math.sin(yaw0)], [-math.sin(yaw0), math.cos(yaw0)]])
            fwd, side = R0 @ (p1 - p0)
            samples = _Tail(list(s.tail.samples))
            walk = samples.between(t_a / 1000, t_walk_end / 1000 + 2)
            rest = samples.between(t_walk_end / 1000 + 2, t_rest_end / 1000)
            torso = np.array([x["torso"]["pos"][:2] for x in rest])
            dev = np.linalg.norm(torso - torso.mean(axis=0), axis=1)
            res.update(forward_m=round(float(fwd), 3), forward_error_m=round(float(fwd) - 2.0, 3), sideways_m=round(float(side), 3),
                       heading_change_during_walk_rad=round(wrap(yaw_of(o1["pelvis"]["quat"]) - yaw0), 3),
                       max_tilt_deg=round(max(tilt(x) for x in walk + rest), 2),
                       rest_torso_sway_max_dev_cm=round(float(dev.max()) * 100, 2), rest_torso_range_cm=round(float(np.ptp(torso, axis=0).max()) * 100, 2),
                       fell=any(x["fallen"] for x in walk + rest), attached=o1["attached"].get("crate"))
            res["camera_walk"], res["camera_rest"] = walk_cam, rest_cam
            tw = [e for t, e in trk.rows if t_a <= t <= t_walk_end + 2000]; tr = [e for t, e in trk.rows if t_walk_end + 2000 <= t <= t_rest_end]
            f = lambda rows: {"max_rad": round(float(np.max(rows)), 4), "rms_rad": round(float(np.sqrt(np.mean(np.square(rows)))), 4), "worst_joint": 12 + int(np.argmax(np.max(rows, axis=0)))} if rows else None
            res["arm_tracking_walk"], res["arm_tracking_rest"] = f(tw), f(tr)
            # the 90 degree heading change
            yaw_b = yaw_of(s.tail.latest()["pelvis"]["quat"])
            tt = ops.now_ms()
            ops.turn_by(math.pi / 2); ops.settle(8.0)
            yaw_c = yaw_of(s.tail.latest()["pelvis"]["quat"])
            res["turn_90"] = {"achieved_rad": round(wrap(yaw_c - yaw_b), 3), "error_rad": round(wrap(yaw_c - yaw_b) - math.pi / 2, 3),
                              "took_s": round((ops.now_ms() - tt) / 1000, 1),
                              "position_drift_cm": round(float(np.linalg.norm(np.array(s.tail.latest()["pelvis"]["pos"][:2]) - p1)) * 100, 1)}
        except BackendFailure as e:
            res.update(fell=e.code == "S516", failure=f"{e.code} {e.message}")
        print(json.dumps(res), flush=True)
        result["runs"].append(res)
        if res.get("failure"): break
    trk.stop_ev.set()
    final = s.sim.Status()[1]
    result["standin"] = {k: final.get(k) for k in ("realtime_ratio", "worst_second_ratio", "stalls")}
Path(out / "result.json").write_text(json.dumps(result, indent=1))
print("done", flush=True)
