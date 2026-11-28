# ruff: noqa
"""Measurement only (g1-phase4-prereqs.md): the stand-in with its physics loop timed in sections, run under a CPU limit.

    taskset -c 0-3 python exp_loop_profile.py [--task pick_place|idle] [--out DIR] [--seconds N]

Starts the stand-in (this file patches `Simulator.run` and calls the stand-in's own `main`), reads its oracle as the
checks do, and runs `robolang_g1.cli` on `e_pick_place.rob` against it (or nothing: `idle`). When the stand-in stops it
writes `loop_profile.json`: per pass type the compute-time quantiles, the time the loop spent in each section, the
sleep overshoot (wake-up latency), the thread's CPU time against its wall time, and for every slow second (ratio
below 0.97) what the process's threads were doing. Not part of the backend.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

SECTIONS = ("service", "targets", "mj_step", "publish", "camera", "oracle")


def install_profile(out_path: str) -> None:
    from robolang_g1.sim import standin

    if os.environ.get("G1_NO_CAMERA"):  # no camera thread at all: the GIL and CPU the render and encode cost, taken out
        real_init = standin.Simulator.__init__
        standin.Simulator.__init__ = lambda self, *a, **kw: real_init(self, *a, **{**kw, "camera": False})
    camera_tid: dict = {}
    real_camera_loop = standin.Simulator._camera_loop

    def camera_loop(self):  # note which thread renders, to tell its CPU from the loop's and the DDS threads'
        camera_tid["tid"] = threading.get_native_id()
        return real_camera_loop(self)

    standin.Simulator._camera_loop = camera_loop
    real_setpriority = os.setpriority
    os.setpriority = lambda which, who, prio: real_setpriority(which, who, int(os.environ.get("G1_CAMERA_NICE", prio)))
    rec = {"t0": [], "total": [], "cpu": [], "over": [], **{k: [] for k in SECTIONS}}
    acc = dict.fromkeys(SECTIONS, 0)
    secs: list[dict] = []
    clk = time.perf_counter_ns

    def timed(name, fn):
        def wrapper(*a, **kw):
            t = clk()
            try:
                return fn(*a, **kw)
            finally:
                acc[name] += clk() - t
        return wrapper

    def threads_cpu() -> dict:
        out = {}
        hz = os.sysconf("SC_CLK_TCK")
        for tid in os.listdir("/proc/self/task"):
            try:
                stat = Path(f"/proc/self/task/{tid}/stat").read_text()
                comm = stat[stat.index("(") + 1:stat.rindex(")")]
                f = stat[stat.rindex(")") + 2:].split()
                sched = Path(f"/proc/self/task/{tid}/schedstat").read_text().split()
                st = Path(f"/proc/self/task/{tid}/status").read_text()
                vol = int(st.split("voluntary_ctxt_switches:")[1].split()[0])
                inv = int(st.split("nonvoluntary_ctxt_switches:")[1].split()[0])
                out[f"{comm}:{tid}"] = {"cpu_s": (int(f[11]) + int(f[12])) / hz, "run_s": int(sched[0]) / 1e9,
                                        "delay_s": int(sched[1]) / 1e9, "vol": vol, "inv": inv}
            except (OSError, ValueError, IndexError):
                pass
        return out

    def procs_cpu() -> dict:
        """(utime + stime) in seconds of every process that can be read, by 'pid:comm:cmdline-tail'."""
        hz = os.sysconf("SC_CLK_TCK")
        out = {}
        for pid in os.listdir("/proc"):
            if not pid.isdigit():
                continue
            try:
                stat = Path(f"/proc/{pid}/stat").read_text()
                comm = stat[stat.index("(") + 1:stat.rindex(")")]
                f = stat[stat.rindex(")") + 2:].split()
                cpu = (int(f[11]) + int(f[12])) / hz
                if cpu > 0:
                    cmd = Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode()[-60:]
                    out[f"{pid}:{comm}:{cmd}"] = cpu
            except (OSError, ValueError, IndexError):
                pass
        return out

    def system_cpu() -> list:
        rows = []
        for line in Path("/proc/stat").read_text().splitlines():
            if line.startswith("cpu") and line[3].isdigit():
                v = [int(x) for x in line.split()[1:]]
                rows.append((int(line.split()[0][3:]), sum(v), v[3] + v[4]))  # cpu, total, idle+iowait
        return rows

    def run(self, stop=None, realtime=True):
        stop = stop or self._stop
        t = self.timing
        sim_tid = threading.get_native_id()
        # time the sections (the loop below is the stand-in's own, with timers around its calls)
        self.stance.targets = timed("targets", self.stance.targets)
        standin.mujoco.mj_step = timed("mj_step", standin.mujoco.mj_step)
        self._service = timed("service", self._service)
        self._hand_frame_to_camera = timed("camera", self._hand_frame_to_camera)
        self._write_atomic = timed("oracle", self._write_atomic)
        orig_oracle = self.oracle
        self.oracle = timed("oracle", orig_oracle)
        on_step = self.on_step
        self.on_step = timed("publish", on_step)
        dt = float(self.model.opt.timestep)
        t.wall0 = t.window_wall = time.perf_counter()
        anchor, n0 = t.wall0, self.n
        cam_every = max(1, round(1 / (standin.CAMERA_HZ * dt)))
        oracle_every = max(1, round(1 / (standin.ORACLE_HZ * dt)))
        steps_per_second = round(1 / dt)
        prev_threads, prev_cpu, prev_procs = threads_cpu(), system_cpu(), procs_cpu()
        window_first = 0
        while not stop.is_set():
            for k in acc:
                acc[k] = 0
            loop_start = time.perf_counter()
            cpu0 = time.thread_time_ns()
            self._service()
            self.step()
            if self.camera and self.n % cam_every == 0:
                self._hand_frame_to_camera()
            if self.oracle_path is not None and self.n % oracle_every == 0:
                standin.Simulator._write_atomic(self.oracle_path, json.dumps(self.oracle()).encode(), ".tmp")
            window_end = False
            if self.n % steps_per_second == 0:
                now = time.perf_counter()
                ratio = 1.0 / max(now - t.window_wall, 1e-9)
                t.window_ratio_min = min(t.window_ratio_min, ratio)
                if ratio < 0.95:
                    t.slow_windows.append((round(self.sim_time, 2), round(ratio, 3)))
                t.window_wall = now
                window_end = True
            cpu1 = time.thread_time_ns()
            took = (time.perf_counter() - loop_start) * 1000
            t.step_ms.append(took)
            rec["t0"].append(loop_start)
            rec["total"].append(took)
            rec["cpu"].append((cpu1 - cpu0) / 1e6)
            for k in SECTIONS:
                rec[k].append(acc[k] / 1e6)
            over = 0.0
            if realtime:
                slack = anchor + (self.n - n0) * dt - time.perf_counter()
                if slack > 0:
                    time.sleep(slack)
                    over = (time.perf_counter() - (anchor + (self.n - n0) * dt)) * 1000  # wake-up latency, ms
                elif slack < -0.25:
                    t.stalls += 1
                    anchor -= slack
            rec["over"].append(over)
            if window_end:
                threads, cpu, procs = threads_cpu(), system_cpu(), procs_cpu()
                top = sorted(((round(v - prev_procs.get(k, v), 2), k) for k, v in procs.items()), reverse=True)[:6]
                d = {k: {f: round(v[f] - prev_threads.get(k, v)[f], 3) for f in v} for k, v in threads.items()}
                busy = {c: round(1 - (i - pi) / max(tot - pt, 1), 3) for (c, tot, i), (_, pt, pi) in zip(cpu, prev_cpu)}
                n = len(rec["total"])
                me = d.get(next((k for k in d if k.endswith(f":{sim_tid}")), ""), {})
                seg = slice(window_first, n)
                secs.append({"sim_time": round(self.sim_time, 2), "ratio": round(ratio, 4), "first": window_first,
                             "last": n, "threads": d, "cpu_busy": busy, "top_processes": top,
                             # the loop thread over this second: passes' wall time, its CPU, run-queue delay (all of it,
                             # wake-ups included), the sleep overshoot (the wake-up part), the rest is blocked time
                             "loop": {"passes_wall_ms": round(float(np.sum(rec["total"][seg])), 1),
                                      "passes_cpu_ms": round(float(np.sum(rec["cpu"][seg])), 1),
                                      "overshoot_ms": round(float(np.sum(rec["over"][seg])), 1),
                                      "runq_delay_ms": round(me.get("delay_s", 0) * 1000, 1),
                                      "involuntary_switches": me.get("inv"), "voluntary_switches": me.get("vol")}})
                window_first, prev_threads, prev_cpu, prev_procs = n, threads, cpu, procs
        totals = {k: v["cpu_s"] for k, v in threads_cpu().items()}  # CPU seconds by thread over the whole run
        cam_tid = camera_tid.get("tid")
        # summaries
        tot = np.array(rec["total"])
        kind = np.array(range(len(tot)))
        n_off = self.n - len(tot)  # step numbers: pass i has n = n0 + i + 1
        steps = np.arange(len(tot)) + 1 + n0
        masks = {"all": np.ones(len(tot), bool), "policy (n%10==0)": steps % 10 == 0, "oracle (n%10==0, 50 Hz)": steps % 10 == 0,
                 "lowstate (n%2==0)": steps % 2 == 0, "camera (n%100==0)": steps % 100 == 0,
                 "plain (n%2==1)": steps % 2 == 1}

        def q(x):
            x = np.asarray(x)
            return {"mean": round(float(x.mean()), 3), "p50": round(float(np.quantile(x, .5)), 3),
                    "p99": round(float(np.quantile(x, .99)), 3), "p999": round(float(np.quantile(x, .999)), 3),
                    "max": round(float(x.max()), 2), "over_2ms": round(float((x > 2).mean()), 4)}

        summary = {"passes": len(tot), "total_ms": {k: q(tot[m]) for k, m in masks.items() if m.any()},
                   "cpu_ms": q(rec["cpu"]), "sleep_overshoot_ms": q(rec["over"]),
                   "section_ms": {k: q(rec[k]) for k in SECTIONS},
                   "wall_minus_cpu_ms": q(np.array(rec["total"]) - np.array(rec["cpu"]))}
        slow = [s for s in secs if s["ratio"] < 0.97]
        for s in slow:
            seg = slice(s["first"], s["last"])
            tt = np.array(rec["total"][seg])
            s["pass_max_ms"] = round(float(tt.max()), 1) if len(tt) else None
            s["passes_over_5ms"] = int((tt > 5).sum())
            s["sum_over_budget_ms"] = round(float(np.clip(tt - 2, 0, None).sum()), 1)
            s["overshoot_sum_ms"] = round(float(np.sum(rec["over"][seg])), 1)
            s["cpu_mean_ms"] = round(float(np.mean(rec["cpu"][seg])), 3)
            s["section_sum_ms"] = {k: round(float(np.sum(rec[k][seg])), 1) for k in SECTIONS}
        Path(out_path).write_text(json.dumps({"summary": summary, "seconds": len(secs),
                                              "thread_cpu_s": totals, "sim_tid": sim_tid, "camera_tid": cam_tid,
                                              "wall_s": round(time.perf_counter() - t.wall0, 1),
                                              "slow_seconds": slow, "stalls": t.stalls, "per_second": [{k: v for k, v in x.items() if k not in
                                                                          ("threads", "cpu_busy", "top_processes")} for x in secs],
                                              "worst_ratio": t.window_ratio_min,
                                              "all_ratios": [s["ratio"] for s in secs]}, indent=1))
        # per-pass trace of the slow passes for the report
        idx = np.nonzero(tot > 10)[0]
        Path(out_path + ".passes.json").write_text(json.dumps(
            [{"sim_n": int(steps[i]), "t0": rec["t0"][i], "total_ms": round(tot[i], 2), "cpu_ms": round(rec["cpu"][i], 2),
              **{k: round(rec[k][i], 2) for k in SECTIONS}, "over_ms": round(rec["over"][i], 2)} for i in idx]))

    standin.Simulator.run = run


def main() -> int:
    if os.environ.get("G1_PROFILE_OUT"):  # inside the stand-in process
        install_profile(os.environ["G1_PROFILE_OUT"])
        from robolang_g1.sim import standin
        return standin.main()
    ap = argparse.ArgumentParser()
    ap.add_argument("--task", default="pick_place", choices=["pick_place", "idle"])
    ap.add_argument("--out", default="/tmp/g1_prof")
    ap.add_argument("--seconds", type=float, default=60.0, help="idle: how long")
    ap.add_argument("--no-tail", action="store_true", help="do not read the oracle (the checks' own process)")
    ap.add_argument("--no-camera", action="store_true", help="the stand-in without its camera thread")
    ap.add_argument("--exec-nice", type=int, default=0, help="the executor process's niceness")
    ap.add_argument("--camera-nice", type=int, default=5, help="the camera thread's niceness (the stand-in's own: 5)")
    ap.add_argument("--parent-nice", type=int, default=0, help="this process's (the checks') niceness")
    args = ap.parse_args()
    if args.parent_nice:
        os.nice(args.parent_nice)
    from robolang_g1 import guard
    from robolang_g1.harness import OracleTail

    run_dir = Path(args.out)
    run_dir.mkdir(parents=True, exist_ok=True)
    sites = Path(__import__("robolang_g1").__file__).parent / "sites"
    env = dict(os.environ, MUJOCO_GL=os.environ.get("MUJOCO_GL", "egl"), G1_PROFILE_OUT=str(run_dir / "loop_profile.json"),
               G1_CAMERA_NICE=str(args.camera_nice), **({"G1_NO_CAMERA": "1"} if args.no_camera else {}))
    proc = subprocess.Popen([sys.executable, __file__, "--site", str(sites / "pick_place.json"), "--run-dir", str(run_dir),
                             "--oracle", str(run_dir / "oracle.json"), "--test-hooks"], stdout=subprocess.PIPE,
                            stderr=open(run_dir / "standin.log", "w"), text=True, env=env)
    while True:
        line = proc.stdout.readline()
        if line.startswith("{"):
            break
        if not line:
            raise SystemExit("the stand-in died")
    sim, _ = guard.require_simulation(timeout=30.0)
    tail = None
    if not args.no_tail:
        tail = OracleTail(run_dir / "oracle.json")
        tail.start()
    status = 0
    if args.task == "pick_place":
        rob = Path(__file__).resolve().parents[3] / "bench/reference/e_pick_place.rob"
        cmd = [sys.executable, "-m", "robolang_g1.cli", str(rob), "--task", "pick_place", "--site", str(sites / "pick_place.json"),
               "--out", str(run_dir / "record.json"), "--bind", "r=robot_1", "--bind", "box=box_1", "--bind", "src=loc_a",
               "--bind", "dst=loc_b"]
        with (run_dir / "cli.log").open("w") as fh:
            cli = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, env=os.environ,
                                   preexec_fn=(lambda: os.nice(args.exec_nice)) if args.exec_nice else None)
            _, wait_status, usage = os.wait4(cli.pid, 0)
            status = os.waitstatus_to_exitcode(wait_status)
            cli.returncode = status
            (run_dir / "executor_cpu.json").write_text(json.dumps(
                {"executor_cpu_s": round(usage.ru_utime + usage.ru_stime, 2)}))
    else:
        time.sleep(args.seconds)
    final = sim.Status()[1]
    if tail:
        tail.stop()
    proc.send_signal(15)
    proc.communicate(timeout=30)
    ts = os.times()  # this process: the checks' side (its main thread and the oracle reader)
    (run_dir / "checks_cpu.json").write_text(json.dumps({"checks_process_cpu_s": round(ts.user + ts.system, 2)}))
    print(json.dumps({"exit": status, "status": {k: final.get(k) for k in ("realtime_ratio", "worst_second_ratio", "stalls",
                                                                          "step_compute_ms", "slow_windows")}}))
    return status


if __name__ == "__main__":
    sys.exit(main())
