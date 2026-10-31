"""Save one frame from each H3 camera as PNG (evidence only; never used for a verdict). Usage: snapshot.py OUT."""
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

rclpy.init()
node = rclpy.create_node("h3_snapshot")
frames = {}
for name in ("overhead", "side"):
    node.create_subscription(Image, f"/{name}_camera/image", lambda m, n=name: frames.setdefault(n, m),
                             qos_profile_sensor_data)
deadline = time.monotonic() + 20
while len(frames) < 2 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
for name, m in frames.items():
    image = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.width, 3)
    cv2.imwrite(f"{sys.argv[1]}/{name}.png", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
print(sorted(frames))
