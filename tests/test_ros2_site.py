import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "backends" / "ros2"))

from robolang_ros2.executor import BackendFailure  # noqa: E402
from robolang_ros2.site import read_site  # noqa: E402


def site():
    return {"gazebo_model": "ur", "robot": "arm1", "objects": {"box1": "box"},
            "poses": {"table": {"frame": "base_link", "position": [0, 0.4, 0.5],
                                "orientation": [0, 0, 0, 1]}}}


def test_site_accepts_finite_pose(tmp_path):
    path = tmp_path / "site.json"
    path.write_text(json.dumps(site()))
    assert read_site(path).poses["table"]["position"] == [0, 0.4, 0.5]


@pytest.mark.parametrize("edit", [
    lambda s: s["poses"]["table"]["position"].__setitem__(0, float("nan")),
    lambda s: s["poses"]["table"]["orientation"].__setitem__(3, 0.1),
    lambda s: s["objects"].__setitem__("box1", "box;rm -rf /"),
    lambda s: s.__setitem__("physical_mode", True),
])
def test_site_rejects_invalid_or_extra_fields(tmp_path, edit):
    value = site()
    edit(value)
    path = tmp_path / "site.json"
    path.write_text(json.dumps(value))
    with pytest.raises(BackendFailure) as exc:
        read_site(path)
    assert exc.value.code == "S507"
