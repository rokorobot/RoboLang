"""The simulation-only contract is checked without importing ROS or contacting a controller."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backends" / "ros2"))

from robolang import ir  # noqa: E402
from robolang_ros2.executor import BackendFailure, Site, execute  # noqa: E402


SOURCE = """robolang 1.3;
task transfer(robot r, object box, location src, location dst) {
  goal { object_at box dst; }
  plan { move r to src; grasp r box; move r to dst; place r box at dst; }
}
"""


class FakeOps:
    def __init__(self, fail=None):
        self.calls = []
        self.fail = fail
        self.time = 0

    def _call(self, name, *args):
        self.calls.append((name, *args))
        if name == self.fail:
            raise BackendFailure("S501" if name == "guard" else "S502", name)

    def guard(self, model): self._call("guard", model)
    def now_ms(self):
        self.time += 1
        return self.time
    def move(self, pose): self._call("move", pose)
    def grip(self, closed): self._call("grip", closed)
    def attach(self, model): self._call("attach", model)
    def detach(self, model): self._call("detach", model)
    def wait_ms(self, duration): self._call("wait", duration)
    def versions(self): return {"ros": "lyrical"}


def fixture():
    task = ir.compile_source(SOURCE).ir.tasks[0]
    site = Site("ur", "arm1", {"box1": "box"}, {"table": {"x": 1}, "shelf": {"x": 2}})
    return task, {"r": "arm1", "box": "box1", "src": "table", "dst": "shelf"}, site


def test_execution_sequence_never_claims_unobserved_goal():
    task, bindings, site = fixture()
    ops = FakeOps()
    record = execute(task, bindings, site, ops)
    assert [call[0] for call in ops.calls] == ["guard", "move", "grip", "attach", "move", "move", "grip", "detach"]
    assert record["state"] == "UNKNOWN"
    assert record["goal"] == "not_evaluated"
    assert record["core_outcome"] is None
    assert record["backend"]["kind"] == "ros2-gazebo"


def test_guard_blocks_every_control_call():
    task, bindings, site = fixture()
    ops = FakeOps(fail="guard")
    record = execute(task, bindings, site, ops)
    assert record["failure"]["code"] == "S501"
    assert ops.calls == [("guard", "ur")]


def test_failed_move_stops_before_gripper():
    task, bindings, site = fixture()
    ops = FakeOps(fail="move")
    record = execute(task, bindings, site, ops)
    assert record["state"] == "FAILED"
    assert record["failure"]["code"] == "S502"
    assert record["failure"]["step"] == 1
    assert [call[0] for call in ops.calls] == ["guard", "move"]


def test_missing_pose_fails_closed():
    task, bindings, site = fixture()
    del site.poses["table"]
    ops = FakeOps()
    record = execute(task, bindings, site, ops)
    assert record["failure"]["code"] == "S507"
    assert ops.calls == [("guard", "ur")]
