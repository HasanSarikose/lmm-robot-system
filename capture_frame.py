import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
import os

class DroneCamera(Node):
    def __init__(self):
        super().__init__("drone_camera_stream")
        self.sub = self.create_subscription(
            Image, "/drone/camera", self.cb, 10
        )

    def cb(self, msg):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3
            )

            frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

            temp = "/tmp/drone_frame_tmp.png"
            final = "/tmp/drone_frame.png"

            cv2.imwrite(temp, frame)
            os.replace(temp, final)

        except Exception as e:
            print("DRONE ERROR:", e)


rclpy.init()
node = DroneCamera()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()