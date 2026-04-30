import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import time
from frame_store import frames


class MultiCameraNode(Node):

    def __init__(self):
        super().__init__("multi_camera_node")

        self.last_times = {"drone": 0, "ika": 0, "arm": 0}
        self.FPS_LIMIT = 10

        self.create_subscription(Image, "/drone/camera", self.drone_cb, 10)
        self.create_subscription(Image, "/ika/camera", self.ika_cb, 10)
        self.create_subscription(Image, "/arm/camera", self.arm_cb, 10)

    def process(self, msg, name):
        now = time.time()
        if now - self.last_times[name] < 1.0 / self.FPS_LIMIT:
            return

        self.last_times[name] = now

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )

        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # 🔥 RAM’e yaz (disk yok)
        frames[name] = frame

    def drone_cb(self, msg):
        self.process(msg, "drone")

    def ika_cb(self, msg):
        self.process(msg, "ika")

    def arm_cb(self, msg):
        self.process(msg, "arm")


def main():
    rclpy.init()
    node = MultiCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()