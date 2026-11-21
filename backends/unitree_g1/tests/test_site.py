"""The /2 site: stations, locations, bindings, the visibility rule (decision 0046 section 3), and /1 still loading."""

import copy
import json
import math
from pathlib import Path

import pytest
from conftest import SITE, needs_assets

from robolang_g1 import site as S

SITES = Path(__file__).resolve().parents[1] / "robolang_g1" / "sites"


def raw(name="pick_place"):
    return json.loads((SITES / f"{name}.json").read_text())


@pytest.mark.parametrize("name", ["pick_place", "carry_hold", "part_precheck"])
def test_shipped_sites_load_and_derive_each_station_pose(name):
    site = S.load_site(SITES / f"{name}.json")
    assert site["schema"] == S.SCHEMA_2
    for station in site["stations"]:
        d, lat, a = station["stand"]
        f = station["marker"]["facing"]
        mx, my = station["marker"]["pos"][:2]
        x, y, yaw = station["pose"]
        # the pelvis stands `d` out from the marker along its normal and `lat` to the left of it, facing the marker
        assert math.hypot(x - mx, y - my) == pytest.approx(math.hypot(d, lat), abs=1e-5)
        assert S.wrap(yaw - (f + math.pi + a)) == pytest.approx(0, abs=1e-5)
    assert site["robot"]["arm_stance"] in ("free", "supported")


def test_phase1_site_still_loads_and_keeps_its_stand_pose():
    site = S.load_site(SITE)
    assert site["schema"] == S.SCHEMA_1 and site["stations"][0]["pose"] == [-0.55, 0.0, 0.0]
    assert site["camera"]["fovy"] == 58.0 and site["bindings"] == {}


def test_marker_frame_axes_are_right_up_out_and_a_tilt_leans_the_face_up():
    R, p = S.marker_frame({"pos": [1, 2, 0.3], "facing": math.pi})
    assert R[:, 2] == pytest.approx([-1, 0, 0], abs=1e-9)  # out of the plate: -x
    assert R[:, 1] == pytest.approx([0, 0, 1], abs=1e-9)  # up
    assert R[:, 0] == pytest.approx([0, -1, 0], abs=1e-9)  # right as seen from the front (looking along +x)
    assert S.marker_frame({"pos": [0, 0, 0], "facing": 0.0, "tilt": 0.5})[0][:, 2][2] == pytest.approx(math.sin(0.5))


@pytest.mark.parametrize(
    "edit, message",
    [
        (lambda s: s["bindings"]["locations"].update(loc_a="nowhere"), "does not have"),
        (lambda s: s["locations"][0].update(station="nowhere"), "unknown station"),
        (lambda s: s["stations"][1]["marker"].update(id=20), "already used"),
        (lambda s: s["objects"][0]["marker"].update(id=21), "already used"),
        (lambda s: s["locations"][0]["region"].update(min=[1, 1, 1], max=[0, 0, 0]), "min must be below max"),
        (lambda s: s["bindings"].update(rooms={}), "unknown kind"),
        (lambda s: s.update(extras=1), "unknown site keys"),
        (lambda s: s["robot"].update(arm_stance="floating"), "arm_stance"),
        (lambda s: s["objects"][0]["grasp"].update(hand="both"), "grasp.hand"),
        (lambda s: s["stations"][0].pop("stand"), "stand"),
    ],
)
def test_site_errors_name_the_problem(edit, message):
    s = copy.deepcopy(raw())
    edit(s)
    with pytest.raises(S.SiteError, match=message):
        S.load_site(s)


@needs_assets
@pytest.mark.parametrize(
    "site_name, rob, task, bind",
    [
        (
            "pick_place",
            "e_pick_place.rob",
            "pick_place",
            {"r": "robot_1", "box": "box_1", "src": "loc_a", "dst": "loc_b"},
        ),
        (
            "carry_hold",
            "w_carry_hold.rob",
            "carry_crate",
            {"r": "robot_1", "crate": "crate_1", "dock": "loc_dock", "dst": "loc_dst"},
        ),
        (  # the precondition's location is in view from the start pose, the goal's from the end station
            "part_precheck",
            "w_part_precheck.rob",
            "move_if_present",
            {"r": "robot_1", "item": "box_1", "src": "loc_a", "dst": "loc_b"},
        ),
    ],
)
def test_the_shipped_sites_meet_the_visibility_rule_for_their_tasks(site_name, rob, task, bind):
    from robolang import ir

    from robolang_g1 import executor, kinematics, perception

    site = S.load_site(SITES / f"{site_name}.json")
    repo = Path(__file__).resolve().parents[3]
    t = ir.compile_source((repo / "bench" / "reference" / rob).read_text(), rob).ir.task(task)
    names = executor.refuse(t, bind, site)
    assert perception.visibility_violations(site, t, names, kinematics.Kinematics(site["camera"])) == []


@needs_assets
def test_a_goal_location_out_of_view_from_the_final_station_is_reported():
    from robolang import ir

    from robolang_g1 import executor, kinematics, perception

    site = S.load_site(SITES / "pick_place.json")
    source = (
        "robolang 1.3;\ntask t(robot r, object o, location a, location b) {\n  goal { object_at o a; }\n"
        "  plan { move r to a; move r to b; }\n}\n"
    )
    t = ir.compile_source(source, "t.rob").ir.task("t")
    names = executor.refuse(t, {"r": "robot_1", "o": "box_1", "a": "loc_a", "b": "loc_b"}, site)
    bad = perception.visibility_violations(site, t, names, kinematics.Kinematics(site["camera"]))
    assert len(bad) == 1 and "object_at bench" in bad[0]  # the plan ends at the shelf; the goal is at the bench


def test_a_declared_table_between_the_camera_and_a_marker_is_an_occlusion():
    from robolang_g1.perception import line_of_sight_blocked

    site = S.load_site(SITES / "pick_place.json")
    boxes = site["tables"]
    marker = [3.75, 0.45, 0.19]  # the bench station's marker, low behind the bench table
    assert line_of_sight_blocked([2.41, 0.02, 1.208], marker, boxes) is None  # from the station: over the table
    wide = [
        dict(b, half_size=[b["half_size"][0], 0.30, b["half_size"][2]]) if b["name"] == "bench_table" else b
        for b in boxes
    ]
    assert line_of_sight_blocked([2.21, 0.02, 1.208], marker, wide) == "bench_table"  # from 0.2 m short, a wider table
    assert line_of_sight_blocked([2.21, 0.02, 1.208], marker, boxes) is None  # the shipped table is narrower
