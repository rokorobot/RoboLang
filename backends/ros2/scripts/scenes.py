"""H3 phase 4 scenes (decision 0029 section 3): one Gazebo world and site file per benchmark task, same entity IDs.

The reference `.rob` files are used unchanged. Only the world is rewritten: every object is a 0.1 m box with an
ArUco marker on its top and +y faces, every location a slot on one table, seen by an overhead and a side camera.

    python3 scenes.py SCENE OUT_DIR    writes world.sdf, robot.urdf.xacro, site.json, scene.json and marker PNGs
    python3 scenes.py --list           prints the scene names
"""
from __future__ import annotations

import json
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent  # backends/ros2 (the image keeps the same layout under /opt/h3)
IMAGE_REFERENCE = Path("/opt/h3/bench/reference")
REFERENCE = IMAGE_REFERENCE if IMAGE_REFERENCE.is_dir() else ROOT.parent.parent / "bench" / "reference"

TABLE_TOP = 0.25
BOX = 0.1
GRASP_Z = 0.55  # tool0 above a box's slot: the fingertips (about 0.16 m below tool0) clear the box top
POSE_Z = 0.68  # tool0 at a location: a held box hangs clear of the boxes on the table
APPROACH = POSE_Z - GRASP_Z  # grasp and place come straight down to the grasp height
DOWN = [0.8601, -0.5101, 0.0, 0.0]  # tool0 pointing down, as verified in phases 2 and 3
MARKER_LENGTH = BOX * 400 / 600  # the black square of a 400 px marker with a 100 px white border, on a 0.1 m face
TABLE_Y = (0.3, 0.85)  # the arm's start pose lies low near the base, up to y = 0.2
SLOT = {  # an arc 0.55 to 0.6 m from the arm base: distinct x, so the side camera sees every box
    "A": (-0.42, 0.38), "B": (-0.22, 0.54), "C": (0.0, 0.6), "D": (0.22, 0.54), "E": (0.42, 0.38),
}
AIR = {"home": (0.0, 0.35, 0.75), "dock": (-0.3, 0.3, 0.75)}  # robot-only locations
CAMERAS = [  # name, position, roll-pitch-yaw of the Gazebo camera link, horizontal field of view
    ("overhead", (0.0, 0.55, 1.5), (0.0, math.pi / 2, math.pi / 2), 1.1),  # image width along x: the table's length
    ("side", (0.0, 1.6, 0.42), (0.0, 0.1, -math.pi / 2), 0.9),
]

# task: ({location parameter: slot or air pose}, {object parameter: start slot}); "feed" holds three boxes in a row.
TASKS = {
    "a_scrap_part": ({"here": "B", "bin": "D"}, {"part": "B"}),
    "a_sort_three": ({"feed": "feed", "sx": "A", "sy": "E", "sz": "C"},
                     {"x": "feed0", "y": "feed1", "z": "feed2"}),
    "e_box_then_dock": ({"src": "B", "dst": "D", "dock": "dock"}, {"box": "B"}),
    "e_hold_box": ({"src": "C"}, {"box": "C"}),
    "e_pause_home": ({"spot": "C", "home": "home"}, {}),
    "e_pick_place": ({"src": "B", "dst": "D"}, {"box": "B"}),
    "l_vial_hold": ({"bench": "B", "spin": "D", "fridge": "E"}, {"a": "B", "b": "E"}),
    "l_vial_to_fridge": ({"src": "B", "dst": "D"}, {"vial": "B"}),
    "w_part_precheck": ({"src": "C", "dst": "E"}, {"item": "C"}),
    "w_pick_from_table": ({"src": "C", "dst": "A"}, {"item": "C"}),
    "w_split_boxes": ({"sa": "A", "sb": "B", "ta": "D", "tb": "E"}, {"a": "A", "b": "B"}),
}
# Scripted failure cases (0029 section 3, item 4) on e_pick_place. Each must end FAILED with a code.
CASES = {  # scene: (what is wrong, the failure code the run must end with)
    "fail_object_absent": ("the box is outside the workcell, seen by no camera", "S506"),
    "fail_object_moved": ("the box is in slot E, not at the source the site declares", "S506"),
    "fail_out_of_reach": ("the destination pose is 1.3 m from the arm base", "S502"),
    "fail_cameras_covered": ("an opaque plate covers each camera lens", "S506"),
}
FEED = [(-0.12, 0.38), (0.0, 0.38), (0.12, 0.38)]  # three boxes in a row, between slots A and E


def names() -> list[str]:
    return [*TASKS, *CASES]


def build(scene: str) -> dict:
    """Everything of a scene except files: the reference task, bindings, site and world contents."""
    task_id = scene if scene in TASKS else "e_pick_place"
    places, starts = TASKS[task_id]
    reference = json.loads((REFERENCE / f"{task_id}.json").read_text(encoding="utf-8"))
    bindings = reference["bindings"]
    robot = next(v for k, v in bindings.items() if k in _robot_params(task_id))
    poses, regions, grasps, objects, markers, boxes = {}, {}, {}, {}, {}, {}
    for param, slot in places.items():
        location = bindings[param]
        if slot in AIR:
            poses[location] = _pose(*AIR[slot])
            continue
        x, y = (0.0, FEED[0][1]) if slot == "feed" else SLOT[slot]
        if scene == "fail_out_of_reach" and param == "dst":
            x, y = 1.3, 0.0
        poses[location] = _pose(x, y, POSE_Z)
        half = (0.2, 0.07) if slot == "feed" else (0.08, 0.08)  # neighbouring ring slots are 0.18 m apart in x and y
        regions[location] = {"frame": "base_link", "min": [round(x - half[0], 3), round(y - half[1], 3), 0.2],
                             "max": [round(x + half[0], 3), round(y + half[1], 3), 0.4]}
    for index, (param, slot) in enumerate(sorted(starts.items())):
        obj = bindings[param]
        model = f"box{index + 1}"
        objects[obj], markers[obj] = model, 7 + index
        x, y = FEED[int(slot[4:])] if slot.startswith("feed") else SLOT[slot]
        grasps[obj] = _pose(x, y, GRASP_Z)
        if scene == "fail_object_absent":
            x, y = 2.0, 2.0
        elif scene == "fail_object_moved":
            x, y = SLOT["E"]
        boxes[model] = (x, y, 7 + index)
    cameras = []
    for name, position, rpy, fov in CAMERAS:
        cameras.append({"name": name, "topic": f"/{name}_camera/image", "frame": "base_link",
                        "position": list(position), "orientation": [round(q, 6) for q in _quaternion(*rpy)],
                        "horizontal_fov": fov, "width": 640, "height": 480, "marker_length": round(MARKER_LENGTH, 4)})
    site = {
        "gazebo_model": "ur", "robot": robot, "objects": objects, "poses": poses, "grasps": grasps,
        "markers": markers, "regions": regions, "cameras": cameras,
        # The table and a 0.1 m layer above it for the boxes, so MoveIt keeps the gripper clear of both.
        "obstacles": [{"frame": "base_link", "min": [-0.6, TABLE_Y[0], 0.0],
                       "max": [0.6, TABLE_Y[1], TABLE_TOP + BOX]}],
        "approach_height": round(APPROACH, 3),
    }
    return {"scene": scene, "task_id": task_id, "task": reference["task"], "bindings": bindings,
            "source": str(REFERENCE / f"{task_id}.rob"), "expect": "FAILED" if scene in CASES else "SUCCEEDED",
            "case": CASES[scene][0] if scene in CASES else None,
            "expect_code": CASES[scene][1] if scene in CASES else None, "site": site, "boxes": boxes}


def _robot_params(task_id: str) -> set[str]:
    source = (REFERENCE / f"{task_id}.rob").read_text(encoding="utf-8")
    return set(re.findall(r"\brobot\s+([A-Za-z_][A-Za-z0-9_]*)", source))


def _pose(x: float, y: float, z: float) -> dict:
    return {"frame": "base_link", "position": [x, y, z], "orientation": list(DOWN)}


def _quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """x, y, z, w of the SDF rotation Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def world_sdf(spec: dict, out: Path) -> str:
    models = []
    for model, (x, y, marker) in sorted(spec["boxes"].items()):
        texture = out / f"marker_{marker}.png"
        face = (f'<geometry><box><size>{BOX} {BOX} 0.001</size></box></geometry><material><ambient>1 1 1 1</ambient>'
                f'<diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>file://{texture}</albedo_map></metal></pbr>'
                '</material>')
        models.append(f"""
    <model name="{model}">
      <pose>{x} {y} {TABLE_TOP + BOX / 2 + 0.005} 0 0 0</pose>
      <link name="link">
        <inertial><mass>0.2</mass><inertia><ixx>0.0005</ixx><iyy>0.0005</iyy><izz>0.0005</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="c"><geometry><box><size>{BOX} {BOX} {BOX}</size></box></geometry></collision>
        <visual name="body"><geometry><box><size>{BOX} {BOX} {BOX}</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
        <visual name="top"><pose>0 0 {BOX / 2 + 0.0005} 0 0 0</pose>{face}</visual>
        <visual name="front"><pose>0 {BOX / 2 + 0.0005} 0 {-math.pi / 2} 0 0</pose>{face}</visual>
      </link>
    </model>""")
    for name, (px, py, pz), (roll, pitch, yaw), fov in CAMERAS:
        models.append(f"""
    <model name="{name}_camera">
      <static>true</static>
      <pose>{px} {py} {pz} {roll} {pitch} {yaw}</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <always_on>true</always_on><update_rate>10</update_rate><topic>{name}_camera/image</topic>
          <camera><horizontal_fov>{fov}</horizontal_fov>
            <image><width>640</width><height>480</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>5</far></clip></camera>
        </sensor>
      </link>
    </model>""")
        if spec["scene"] == "fail_cameras_covered":  # a plate 0.15 m in front of the lens, along the link's +x
            r = _rotation(roll, pitch, yaw)
            cx, cy, cz = (p + 0.15 * r[i][0] for i, p in enumerate((px, py, pz)))
            models.append(f"""
    <model name="{name}_cover"><static>true</static><pose>{cx:.4f} {cy:.4f} {cz:.4f} {roll} {pitch} {yaw}</pose>
      <link name="link"><visual name="v"><geometry><box><size>0.01 0.5 0.5</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material></visual></link>
    </model>""")
    table = f"1.2 {TABLE_Y[1] - TABLE_Y[0]:.2f} {TABLE_TOP}"
    return f"""<?xml version="1.0"?>
<!-- Generated by backends/ros2/scripts/scenes.py for scene {spec["scene"]}. Arm base at the origin. -->
<sdf version="1.9">
  <world name="h3_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 0.2 -1</direction>
    </light>
    <model name="ground"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
        <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material></visual></link>
    </model>
    <model name="table"><static>true</static><pose>0 {sum(TABLE_Y) / 2} {TABLE_TOP / 2} 0 0 0</pose><link name="link">
      <collision name="c"><geometry><box><size>{table}</size></box></geometry></collision>
      <visual name="v"><geometry><box><size>{table}</size></box></geometry>
        <material><ambient>0.35 0.25 0.15 1</ambient><diffuse>0.35 0.25 0.15 1</diffuse></material></visual></link>
    </model>{"".join(models)}
  </world>
</sdf>
"""


def _rotation(roll: float, pitch: float, yaw: float):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw),
                              math.sin(yaw))
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def robot_xacro(spec: dict) -> str:
    """The arm and gripper description with one detachable joint per box (initially detached by the executor)."""
    template = (ROOT / "urdf" / "ur5e_robotiq_gz.urdf.xacro").read_text(encoding="utf-8")
    joints = "".join(f"""
  <gazebo>
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>wrist_3_link</parent_link>
      <child_model>{model}</child_model>
      <child_link>link</child_link>
      <initial_attach>true</initial_attach>
      <attach_topic>/model/ur/detachable_joint/{model}/attach</attach_topic>
      <detach_topic>/model/ur/detachable_joint/{model}/detach</detach_topic>
      <output_topic>/model/ur/detachable_joint/{model}/state</output_topic>
    </plugin>
  </gazebo>""" for model in sorted(spec["boxes"]))
    assert "<!-- DETACHABLE_JOINTS -->" in template
    return template.replace("<!-- DETACHABLE_JOINTS -->", joints.strip())


def write(scene: str, out: Path) -> dict:
    import cv2

    out.mkdir(parents=True, exist_ok=True)
    spec = build(scene)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for _, _, marker in spec["boxes"].values():
        image = cv2.aruco.generateImageMarker(dictionary, marker, 400)
        cv2.imwrite(str(out / f"marker_{marker}.png"),
                    cv2.copyMakeBorder(image, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255))
    (out / "world.sdf").write_text(world_sdf(spec, out), encoding="utf-8")
    (out / "robot.urdf.xacro").write_text(robot_xacro(spec), encoding="utf-8")
    (out / "site.json").write_text(json.dumps(spec["site"], indent=2) + "\n", encoding="utf-8")
    scene_info = {k: v for k, v in spec.items() if k not in ("site", "boxes")}
    scene_info["box_models"] = sorted(spec["boxes"])
    (out / "scene.json").write_text(json.dumps(scene_info, indent=2) + "\n", encoding="utf-8")
    return spec


if __name__ == "__main__":
    if sys.argv[1:] == ["--list"]:
        print("\n".join(names()))
    else:
        write(sys.argv[1], Path(sys.argv[2]))
