import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

import numpy as np
import cv2
import time
import subprocess
import math

from frame_buffer import frames
from vision.red_ball_detector import detect_red_ball


class CameraNode(Node):

    def __init__(self):
        super().__init__("camera_node")

        self.last_times = {"drone": 0, "ika": 0, "arm": 0}
        self.FPS = 15

        # 🔥 IKA POSE
        self.ika_pose = None

        # 🔥 SUBSCRIBERS
        self.create_subscription(Image, "/drone/camera", self.drone_cb, 10)
        self.create_subscription(Image, "/ika/camera", self.ika_cb, 10)
        self.create_subscription(Image, "/arm/camera", self.arm_cb, 10)

        self.create_subscription(
            Odometry,
            "/model/ika_with_arm/odometry_with_covariance",
            self.ika_pose_cb,
            10
        )

        self.get_logger().info("🚀 ROS Camera Node Started")

    # ================= POSE =================

    def ika_pose_cb(self, msg):
        self.ika_pose = msg.pose.pose

    # ================= COMMON =================

    def process(self, msg, name):
        now = time.time()
        if now - self.last_times[name] < 1.0 / self.FPS:
            return

        self.last_times[name] = now

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )

        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        frames[name] = frame

    # ================= 🔥 PIXEL → ANGLE =================

    def pixel_to_angle(self, cx, width):
        fov = 90  # approx
        center = width / 2

        angle = (cx - center) / center * (fov / 2)
        return math.radians(angle)

    # ================= 🔥 TARGET ESTIMATE =================

    def estimate_target(self, angle, distance=2.0):
        if self.ika_pose is None:
            return None

        x = self.ika_pose.position.x
        y = self.ika_pose.position.y

        target_x = x + distance * math.cos(angle)
        target_y = y + distance * math.sin(angle)

        return target_x, target_y

    # ================= 🔥 MOVE TO TARGET =================

    def move_to_target(self, tx, ty):
        if self.ika_pose is None:
            return

        x = self.ika_pose.position.x
        y = self.ika_pose.position.y

        dx = tx - x
        dy = ty - y

        distance = math.sqrt(dx*dx + dy*dy)

        # 🎯 hedefe ulaştı
        if distance < 0.5:
            print("🛑 TARGET REACHED (WORLD)")

            subprocess.Popen([
                "ign", "topic",
                "-t", "/ugv_cmd_vel",
                "-m", "ignition.msgs.Twist",
                "-p", "linear: {x: 0.0}"
            ])

            self.run_arm()
            return

        # yön hesapla
        angle = math.atan2(dy, dx)

        cmd = f"linear: {{x: 0.5}}, angular: {{z: {angle}}}"

        subprocess.Popen([
            "ign", "topic",
            "-t", "/ugv_cmd_vel",
            "-m", "ignition.msgs.Twist",
            "-p", cmd
        ])

    # ================= 🔥 ARM =================

    def run_arm(self):
        print("🤖 ARM STARTED (simulated)")

    # ================= DRONE =================

    def drone_cb(self, msg):
        now = time.time()
        if now - self.last_times["drone"] < 1.0 / self.FPS:
            return

        self.last_times["drone"] = now

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )

        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # 🔥 DETECTION
        frame, found, cx, cy, area = detect_red_ball(frame)

        if found:
            print(f"🎯 RED BALL FOUND at {cx}, {cy}")

            # 🔥 ANGLE
            angle = self.pixel_to_angle(cx, msg.width)

            # 🔥 TARGET
            target = self.estimate_target(angle)

            if target:
                tx, ty = target
                print(f"🌍 TARGET: {tx:.2f}, {ty:.2f}")

                # 🔥 MOVE
                self.move_to_target(tx, ty)

        frames["drone"] = frame

    # ================= OTHER CAMERAS =================

    def ika_cb(self, msg):
        now = time.time()
        if now - self.last_times["ika"] < 1.0 / self.FPS:
            return
        self.last_times["ika"] = now

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # Kirmizi top tespiti
        frame, found, cx, cy, area = detect_red_ball(frame)

        if found:
            cv2.putText(frame, f"IKA: RED BALL ({cx},{cy})",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)

        frames["ika"] = frame
        
    def arm_cb(self, msg):
        self.process(msg, "arm")


# ================= START =================

def start_ros():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)