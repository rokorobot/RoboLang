"""Keeping pace with real time on a loaded host (docs/evidence/g1-phase4-prereqs.md): the priorities the processes run
at, and what the stand-in reports about a second that fell behind."""
import os
import shutil
import sys
import threading

import pytest
from conftest import SITE, needs_assets

from robolang_g1 import harness
from robolang_g1.sim import standin
from robolang_g1.sim.standin import Simulator


def test_the_executor_is_started_at_a_lower_priority_than_the_stand_in():
    cmd = ["python", "-m", "robolang_g1.cli", "x.rob"]
    if shutil.which("nice"):
        assert harness.yielding(cmd) == ["nice", "-n", str(harness.YIELD_NICE), *cmd]
    else:
        assert harness.yielding(cmd) == cmd
    assert harness.YIELD_NICE > 0  # lowering a priority needs no privilege; the stand-in's loop keeps its own (0)
    assert standin.CAMERA_NICE > harness.YIELD_NICE  # and the renderer yields to the executor too


@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="the kernel's per-thread scheduler statistics")
def test_the_loop_thread_can_read_its_cpu_time_and_its_wait_for_a_cpu():
    cpu0, wait0 = standin.thread_sched()
    x = 0
    for i in range(300_000):
        x += i
    cpu1, wait1 = standin.thread_sched()
    assert cpu1 > cpu0 and wait1 >= wait0 >= 0.0


@needs_assets
def test_a_slow_second_says_whether_the_loop_computed_or_waited_for_a_cpu():
    sim = Simulator(SITE, camera=False)
    t = sim.timing
    t.window_wall = 100.0
    sim._close_window(101.0)  # a second on the dot: nothing slow, the baseline for the next window is taken
    assert t.slow_windows == [] and t.window_ratio_min == pytest.approx(1.0)
    spin = threading.Event()
    x = 0
    while not spin.is_set():  # some CPU time for the thread to account for
        x += 1
        if x > 200_000:
            spin.set()
    sim._close_window(102.25)  # 1.25 s of wall time for the simulated second: 0.8 of real time
    (sim_time, ratio, cpu, wait), = t.slow_windows
    assert ratio == pytest.approx(0.8, abs=0.001)
    if os.path.exists("/proc/thread-self/schedstat"):
        assert cpu > 0.0 and wait >= 0.0
    assert t.window_ratio_min == pytest.approx(0.8, abs=0.001)
    assert tuple(sim.status()["slow_windows"][-1][:2]) == (sim_time, ratio)
