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


@pytest.mark.parametrize("name", ["pick_place", "carry_hold"])
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


def test_declared_boxes_and_the_clearance_of_a_sphere_from_them():
    site = S.load_site(SITES / "pick_place.json")
    assert [b["name"] for b in S.declared_boxes(site)] == ["bench_table", "shelf_table"]
    box = {"pos": [0.0, 0.0, 0.5], "half_size": [0.2, 0.2, 0.5]}  # a table: top at z = 1
    assert S.box_clearance([0.0, 0.0, 1.3], 0.1, box) == pytest.approx(0.2)  # 0.3 above the top, minus the radius
    assert S.box_clearance([0.5, 0.0, 0.5], 0.05, box) == pytest.approx(0.25)  # beside it
    assert S.box_clearance([0.0, 0.0, 1.02], 0.05, box) == pytest.approx(-0.03)  # 2 cm above the top, 5 cm wide
    assert S.box_clearance([0.0, 0.0, 0.9], 0.0, box) == pytest.approx(-0.1)  # inside: the depth to the nearest face
    assert S.box_clearance([0.0, 0.0, 1.3], 0.1, box, z_margin=0.1) == pytest.approx(0.1)  # grown by a margin
    assert S.box_clearance([0.35, 0.0, 0.5], 0.1, box, xy_margin=0.05) == pytest.approx(0.0, abs=1e-9)


def test_a_site_may_declare_obstacles_and_the_scene_builds_them():
    from robolang_g1.sim import scene

    raw_site = raw()
    raw_site["obstacles"] = [{"name": "post", "pos": [1.0, 1.0, 0.5], "half_size": [0.05, 0.05, 0.5]}]
    site = S.load_site(raw_site)
    assert [b["name"] for b in S.declared_boxes(site)][-1] == "post"
    model, _, _ = scene.build(raw_site)
    assert model.geom("post").size[2] == pytest.approx(0.5)
    bad = raw()
    bad["obstacles"] = [{"name": "bench_table", "pos": [0, 0, 0], "half_size": [1, 1, 1]}]  # the name is taken
    with pytest.raises(S.SiteError):
        S.load_site(bad)
