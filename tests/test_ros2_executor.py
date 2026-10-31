"""The simulation-only contract is checked without importing ROS or contacting a controller."""
import json
import sys
import subprocess
from pathlib import Path

import jsonschema
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backends" / "ros2"))

from robolang import ir  # noqa: E402
from robolang.core import verify_sim  # noqa: E402
from robolang_ros2 import observe  # noqa: E402
from robolang_ros2.executor import BackendFailure, Site, execute  # noqa: E402
from robolang_ros2.ros import _gazebo_hardware_matches, gazebo_joint_command  # noqa: E402


SOURCE = """robolang 1.3;
task transfer(robot r, object box, location src, location dst) {
  goal { object_at box dst; }
  plan { move r to src; grasp r box; move r to dst; place r box at dst; }
}
"""


ROOT = Path(__file__).resolve().parents[1]
SCHEMA = json.loads((ROOT / "spec" / "core" / "message-0.1.schema.json").read_text(encoding="utf-8"))
DOWN = (1.0, 0.0, 0.0, 0.0)
IN_DST = (0.0, 0.4, 0.3)
IN_SRC = (0.0, -0.4, 0.3)


def reading(position, confidence=1.0, stamp=1000, orientation=None):
    return observe.Reading("overhead_camera", "aruco_fiducial", position, orientation, confidence, stamp)


def snapshot(marker=IN_DST, confidence=1.0, stamp=1000, t=1500, tool=(0.0, 0.4, 0.5), grip=0.0):
    markers = {} if marker is None else {7: reading(marker, confidence, stamp)}
    return observe.Snapshot(t, markers, observe.Reading("tf", "tf_tool0", tool, DOWN, 1.0, t),
                            observe.Reading("joint_states", "gripper", (grip,), None, 1.0, t))


class FakeOps:
    def __init__(self, fail=None, snap=None, grasp_snap=None):
        """`snap` answers the goal and precondition observations, `grasp_snap` the check right after a move."""
        self.calls = []
        self.fail = fail
        self.time = 0
        self.snap = snap or snapshot()
        self.grasp_snap = grasp_snap or snapshot()

    def _call(self, name, *args):
        self.calls.append((name, *args))
        if name == self.fail:
            codes = {"guard": "S501", "move": "S502", "grip": "S505", "attach": "S506"}
            raise BackendFailure(codes[name], name)

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
    def observe(self, marker_ids):
        after_move = self.calls[-1][0] == "move"
        self.calls.append(("observe", tuple(marker_ids)))
        return self.grasp_snap if after_move else self.snap


def pose(position):
    return {"frame": "base_link", "position": list(position), "orientation": list(DOWN)}


def region(lo, hi):
    return {"frame": "base_link", "min": list(lo), "max": list(hi)}


def fixture():
    task = ir.compile_source(SOURCE).ir.tasks[0]
    site = Site("ur", "arm1", {"box1": "box"}, {"table": pose((0, -0.4, 0.5)), "shelf": pose((0, 0.4, 0.5))},
                {"box1": pose((0, -0.4, 0.4))}, markers={"box1": 7},
                regions={"table": region((-0.1, -0.5, 0.2), (0.1, -0.3, 0.4)),
                         "shelf": region((-0.1, 0.3, 0.2), (0.1, 0.5, 0.4))})
    return task, {"r": "arm1", "box": "box1", "src": "table", "dst": "shelf"}, site


def validator(name):
    return jsonschema.Draft202012Validator({"$ref": f"#/$defs/{name}", "$defs": SCHEMA["$defs"]})


def check_core(core):
    validator("payload_outcome").validate(core)
    for observation in core["evidence"]:
        assert verify_sim(observation, observe.GAZEBO_KEY)
        assert observation["source"]["participant"] == observe.GAZEBO
    return core["outcome"]


def compile_task(source):
    return ir.compile_source(source).ir.tasks[0]


def test_goal_verified_from_observations():
    task, bindings, site = fixture()
    ops = FakeOps()
    record = execute(task, bindings, site, ops)
    assert [call[0] for call in ops.calls] == [
        "guard", "move", "move", "observe", "grip", "attach", "move", "move", "grip", "detach", "observe",
    ]
    assert record["state"] == "SUCCEEDED" and record["failure"] is None
    assert [g["result"] for g in record["goal"]] == ["true"]
    assert record["observations"][0]["markers"]["7"]["position"] == list(IN_DST)
    outcome = check_core(record["core_outcome"])
    assert outcome["status"] == "succeeded"
    assert outcome["verification"]["criteria_results"][0]["met"] is True
    (evidence,) = record["core_outcome"]["evidence"]
    assert (evidence["subject"], evidence["value"], evidence["confidence"]) == (
        "rl:thing:box1", "rl:place:shelf", "1")
    assert record["backend"]["kind"] == "ros2-gazebo"


def test_object_seen_elsewhere_is_false():
    task, bindings, site = fixture()
    record = execute(task, bindings, site, FakeOps(snap=snapshot(marker=IN_SRC)))
    assert record["failure"] == {"code": "S301", "message": "a goal condition is false from the observations",
                                 "step": None}
    outcome = check_core(record["core_outcome"])
    assert outcome["status"] == "failed"
    assert record["core_outcome"]["evidence"][0]["value"] == "rl:place:table"


@pytest.mark.parametrize("snap", [
    snapshot(marker=None),  # camera covered or object absent
    snapshot(confidence=0.8),  # below the default min_confidence 0.9
    snapshot(stamp=0, t=10_001),  # older than the default max_age 10 s
], ids=["unseen", "low_confidence", "stale"])
def test_unusable_observation_is_never_success(snap):
    task, bindings, site = fixture()
    record = execute(task, bindings, site, FakeOps(snap=snap))
    assert record["state"] == "FAILED" and record["failure"]["code"] == "S302"
    outcome = check_core(record["core_outcome"])
    assert outcome["status"] == "unknown" and record["core_outcome"]["evidence"] == []
    assert outcome["verification"]["criteria_results"][0]["met"] is False


def test_verify_annotation_sets_thresholds():
    task = compile_task(SOURCE.replace("  goal {", "  @verify min_confidence 0.7;\n  @verify max_age 2s;\n  goal {"))
    _, bindings, site = fixture()
    assert execute(task, bindings, site, FakeOps(snap=snapshot(confidence=0.8)))["state"] == "SUCCEEDED"
    record = execute(task, bindings, site, FakeOps(snap=snapshot(stamp=0, t=2001)))
    assert record["failure"]["code"] == "S302"
    assert record["verify"] == {"min_confidence": "0.7", "max_age_ms": 2000}


def test_robot_at_and_holding_from_tool_and_gripper():
    _, bindings, site = fixture()
    policy = observe.Policy(observe.DEFAULT_MIN_CONFIDENCE, observe.DEFAULT_MAX_AGE_MS)
    at = compile_task(SOURCE.replace("object_at box dst;", "robot_at r dst;")).goal[0]
    held = compile_task(SOURCE.replace("object_at box dst;", "holding r box;")).goal[0]
    assert observe.evaluate(at, bindings, site, snapshot(), policy) == "true"
    assert observe.evaluate(at, bindings, site, snapshot(tool=(0.0, 0.43, 0.5)), policy) == "false"
    near = snapshot(marker=(0.0, 0.4, 0.4), grip=0.8)
    assert observe.evaluate(held, bindings, site, near, policy) == "true"
    assert observe.evaluate(held, bindings, site, snapshot(marker=(0.0, 0.4, 0.4)), policy) == "false"
    assert observe.evaluate(held, bindings, site, snapshot(marker=None, grip=0.8), policy) == "unknown"
    assert observe.evaluate(held, bindings, site, snapshot(marker=IN_SRC, grip=0.8), policy) == "false"


def test_precondition_false_blocks_before_motion():
    task = compile_task(SOURCE.replace("  goal {", "  pre { object_at box src; }\n  goal {"))
    _, bindings, site = fixture()
    ops = FakeOps()  # the box is seen at dst, not src
    record = execute(task, bindings, site, ops)
    assert record["failure"]["code"] == "S101"
    assert [call[0] for call in ops.calls] == ["guard", "observe"]
    assert record["preconditions"][0]["result"] == "false"
    assert record["core_outcome"]["outcome"]["verification"] is None


def test_precondition_unknown_blocks_before_motion():
    task = compile_task(SOURCE.replace("  goal {", "  pre { object_at box src; }\n  goal {"))
    _, bindings, site = fixture()
    record = execute(task, bindings, site, FakeOps(snap=snapshot(marker=None)))
    assert record["failure"]["code"] == "S102"


@pytest.mark.parametrize("snap", [snapshot(marker=None), snapshot(marker=IN_SRC)], ids=["unseen", "elsewhere"])
def test_grasp_needs_the_object_seen_under_the_gripper(snap):
    task, bindings, site = fixture()
    ops = FakeOps(grasp_snap=snap)
    record = execute(task, bindings, site, ops)
    assert record["failure"]["code"] == "S506" and record["failure"]["step"] == 2
    assert [call[0] for call in ops.calls][-2:] == ["move", "observe"]  # never closed or attached


def test_approach_height_adds_straight_moves():
    task, bindings, site = fixture()
    site = Site(**{**site.__dict__, "approach_height": 0.1})
    ops = FakeOps()
    assert execute(task, bindings, site, ops)["state"] == "SUCCEEDED"
    heights = [round(call[1]["position"][2], 3) for call in ops.calls if call[0] == "move"]
    # move to src; grasp: above, down, back up; move to dst; place: to dst, down, (release), back up
    assert heights == [0.5, 0.5, 0.4, 0.5, 0.5, 0.5, 0.4, 0.5]


def test_object_without_marker_fails_before_motion():
    task, bindings, site = fixture()
    site.markers.clear()
    ops = FakeOps()
    record = execute(task, bindings, site, ops)
    assert record["failure"]["code"] == "S507"
    assert ops.calls == [("guard", "ur")]


def test_goal_without_core_form_still_verified():
    task = compile_task(SOURCE.replace("object_at box dst;", "not object_at box src;"))
    _, bindings, site = fixture()
    record = execute(task, bindings, site, FakeOps())
    assert record["state"] == "SUCCEEDED" and record["core_outcome"] is None


def test_camera_geometry_maps_optical_frame_to_base():
    camera = {"position": [0.0, 0.65, 1.4], "orientation": [0.0, 0.70711, 0.0, 0.70711],
              "width": 640, "height": 480, "horizontal_fov": 1.0}
    below = observe.camera_to_base((0.0, 0.0, 1.0), camera)
    assert below == pytest.approx((0.0, 0.65, 0.4), abs=1e-4)
    # image right is -y and image down is -x for this downward camera (h3_world.sdf: image up = +x)
    assert observe.camera_to_base((0.1, 0.2, 1.0), camera) == pytest.approx((-0.2, 0.55, 0.4), abs=1e-4)
    assert observe.intrinsics(camera)[1:] == (320, 240)


def test_fusion_confidence_counts_agreeing_frames():
    positions = [(0.0, 0.4, 0.3)] * 8 + [(0.2, 0.4, 0.3)]
    fused = observe.fuse(positions, list(range(9)), 10, "camera", "aruco")
    assert fused.position == (0.0, 0.4, 0.3) and fused.confidence == 0.8 and fused.stamp_ms == 7
    assert observe.fuse([], [], 10, "camera", "aruco") is None


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


def test_gripper_failure_never_attaches():
    task, bindings, site = fixture()
    ops = FakeOps(fail="grip")
    record = execute(task, bindings, site, ops)
    assert record["failure"]["code"] == "S505"
    assert "attach" not in [call[0] for call in ops.calls]


def test_condition_without_observation_blocks_before_motion():
    source = SOURCE.replace("  goal {", "  pre { object_exists box; }\n  goal {")
    task = ir.compile_source(source).ir.tasks[0]
    _, bindings, site = fixture()
    ops = FakeOps()
    record = execute(task, bindings, site, ops)
    assert record["failure"]["code"] == "S508"
    assert ops.calls == [("guard", "ur")]


def test_unsupported_later_step_blocks_before_first_motion():
    source = SOURCE.replace("place r box at dst;", 'place r box at dst; communicate r to "fleet" with "done";')
    task = ir.compile_source(source).ir.tasks[0]
    _, bindings, site = fixture()
    ops = FakeOps()
    record = execute(task, bindings, site, ops)
    assert record["failure"]["code"] == "S508"
    assert ops.calls == [("guard", "ur")]


def test_declared_speed_limit_blocks_until_enforced():
    source = SOURCE.replace("  goal {", "  @safety max_speed 0.5 m/s;\n  goal {")
    task = ir.compile_source(source).ir.tasks[0]
    _, bindings, site = fixture()
    ops = FakeOps()
    record = execute(task, bindings, site, ops)
    assert record["failure"]["code"] == "S508"
    assert ops.calls == [("guard", "ur")]


def test_live_hardware_guard_matches_active_gazebo_components():
    description = '<robot name="ur"><ros2_control name="ur"><hardware><plugin>' \
                  'gz_ros2_control/GazeboSimSystem</plugin></hardware></ros2_control></robot>'
    hardware = 'Hardware Component 1\n  name: ur\n  plugin name: \n  state: id=3 label=active\n'
    assert _gazebo_hardware_matches(hardware, description, "ur")
    assert not _gazebo_hardware_matches(hardware, description, "other")
    assert not _gazebo_hardware_matches(hardware.replace("label=active", "label=inactive"), description, "ur")
    assert not _gazebo_hardware_matches(hardware + hardware.replace("Component 1", "Component 2"), description, "ur")
    assert not _gazebo_hardware_matches(hardware, description.replace("GazeboSimSystem", "RealSystem"), "ur")


def test_gazebo_joint_retries_a_lost_transport_publication(monkeypatch):
    sends = []

    class Subscriber:
        attempts = 0

        def communicate(self, timeout):
            self.attempts += 1
            if self.attempts == 1:
                raise subprocess.TimeoutExpired("gz topic", timeout)
            return 'data: "attached"\n', ""

        def poll(self):
            return 0 if self.attempts > 1 else None

    subscriber = Subscriber()
    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: subscriber)
    monkeypatch.setattr(subprocess, "run", lambda args, **kwargs: sends.append(args))
    monkeypatch.setattr("robolang_ros2.ros.time.sleep", lambda _: None)
    gazebo_joint_command("ur", "box", "attach")
    assert len(sends) == 2
    assert sends[0][3] == "/model/ur/detachable_joint/box/attach"


def test_gazebo_joint_fails_after_bounded_retries(monkeypatch):
    class Subscriber:
        def communicate(self, timeout):
            raise subprocess.TimeoutExpired("gz topic", timeout)

        def poll(self):
            return 0

    monkeypatch.setattr(subprocess, "Popen", lambda *args, **kwargs: Subscriber())
    monkeypatch.setattr(subprocess, "run", lambda *args, **kwargs: None)
    monkeypatch.setattr("robolang_ros2.ros.time.sleep", lambda _: None)
    with pytest.raises(BackendFailure, match="five requests") as exc:
        gazebo_joint_command("ur", "box", "attach")
    assert exc.value.code == "S506"
