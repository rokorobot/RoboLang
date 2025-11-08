"""
robolang_runtime.py

Very simple line-based interpreter for RoboLang v1.

It ignores most of the language structure and looks only for core
action lines inside a task's `plan { ... }` block:

    move ...
    grasp ...
    place ...
    inspect ...
    wait ...
    communicate ...

This is enough to demo RoboLang with the simulated adapter.
"""

from __future__ import annotations
import re
import sys
from pathlib import Path

from robolang_adapter import RobotAdapter


class RoboLangRuntime:
    def __init__(self, adapter: RobotAdapter):
        self.adapter = adapter

    def execute_file(self, path: str | Path):
        code = Path(path).read_text(encoding="utf-8")
        self.execute(code)

    def execute(self, rob_code: str):
        """
        Very naive executor:
        - Ignores everything until it sees 'plan {'
        - Executes lines starting with known primitives
        - Stops when '}' at same indentation as plan
        """
        in_plan = False
        for raw_line in rob_code.splitlines():
            line = raw_line.strip()
            if not line or line.startswith("//"):
                continue

            if line.startswith("plan"):
                in_plan = True
                continue
            if in_plan and line.startswith("}"):
                # end of plan block
                in_plan = False
                continue
            if not in_plan:
                continue

            print(f"[RUNTIME] executing: {line}")
            self._dispatch(line)

    # -------- primitive dispatch --------

    def _dispatch(self, line: str):
        if line.startswith("move"):
            # For now we ignore actual target and use simple demo joint positions.
            joints = [0.0, -1.0, 1.0, 0.0, 1.0, 0.0]
            self.adapter.move_to_pose(joint_names=[], joint_positions=joints, duration=2.0)

        elif line.startswith("grasp"):
            # Close gripper
            self.adapter.set_gripper(position=1.0)

        elif line.startswith("place"):
            # Open gripper
            self.adapter.set_gripper(position=0.0)

        elif line.startswith("inspect"):
            # Try to extract object name and sensor
            sensor_matches = re.findall(r'"(.*?)"', line)
            sensor = sensor_matches[0] if sensor_matches else "camera_top"
            self.adapter.inspect(target_id="object", sensor_name=sensor)

        elif line.startswith("wait for"):
            # Format: wait for 5s;
            num = re.findall(r"([0-9]+(?:\\.[0-9]+)?)", line)
            seconds = float(num[0]) if num else 1.0
            self.adapter.wait_for(seconds)

        elif line.startswith("communicate"):
            parts = re.findall(r'"(.*?)"', line)
            if len(parts) == 2:
                channel, message = parts
                self.adapter.communicate(channel, message)
            else:
                self.adapter.communicate("default", line)

        else:
            print(f"[RUNTIME] (ignored line): {line}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python -m robolang_runtime <file.rob>")
        sys.exit(1)

    file_path = sys.argv[1]
    adapter = RobotAdapter("robot_arm_1")
    runtime = RoboLangRuntime(adapter)
    runtime.execute_file(file_path)


if __name__ == "__main__":
    main()
