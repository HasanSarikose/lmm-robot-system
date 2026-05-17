import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry

import numpy as np
import cv2
import time
import subprocess
import math

from frame_buffer import frames
from state_buffer import robot_state, state_lock
from vision.red_ball_detector import detect_red_ball


class CameraNode(Node):

    detected_targets = []

    def __init__(self):
        super().__init__("camera_node")

        self.last_times = {"drone": 0, "ika": 0, "arm": 0}
        self.FPS = 15

        # Eski özellik: IKA pose node içinde de tutulmaya devam ediyor
        self.ika_pose = None

        # ================= CAMERA SUBSCRIBERS =================
        self.create_subscription(Image, "/drone/camera", self.drone_cb, 10)
        self.create_subscription(Image, "/ika/camera", self.ika_cb, 10)
        self.create_subscription(Image, "/arm/camera", self.arm_cb, 10)

        # ================= ODOM SUBSCRIBERS =================
        # Eski world topic adı
        self.create_subscription(
            Odometry,
            "/model/ika_with_arm/odometry_with_covariance",
            self.ika_pose_cb,
            10
        )

        # Bridge üzerinden kullanmak istediğimiz temiz topic adı
        self.create_subscription(
            Odometry,
            "/ika/odom",
            self.ika_pose_cb,
            10
        )

        # ================= LIDAR SUBSCRIBER =================
        self.create_subscription(
            LaserScan,
            "/ika/lidar",
            self.lidar_cb,
            10
        )

        self.get_logger().info("ROS Camera/Odom/Lidar Node Started")

    # ================= POSE / ODOM =================

    def ika_pose_cb(self, msg):
        """
        Eski özellik korunuyor:
        - self.ika_pose güncelleniyor.

        Yeni özellik:
        - robot_state içine IKA x, y, yaw yazılıyor.
        - IKACtrl artık subprocess ile ign topic okumadan buradan veri alacak.
        """

        self.ika_pose = msg.pose.pose

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Quaternion -> yaw
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        with state_lock:
            robot_state["ika_x"] = float(x)
            robot_state["ika_y"] = float(y)
            robot_state["ika_yaw"] = float(yaw)
            robot_state["ika_odom_ready"] = True

    # ================= LIDAR =================

    def lidar_cb(self, msg):
        """
        LaserScan verisini state_buffer'a yazar.
        Sonsuz/NaN değerleri inf olarak saklarız.
        Böylece eski obstacle avoidance mantığı bozulmadan çalışabilir.
        """

        ranges = []

        for r in msg.ranges:
            # NaN kontrolü: r == r false ise NaN'dır
            if r == r and 0.05 < r < 30.0:
                ranges.append(float(r))
            else:
                ranges.append(float("inf"))

        with state_lock:
            robot_state["lidar_ranges"] = ranges
            robot_state["lidar_ready"] = True

    # ================= COMMON IMAGE PROCESS =================

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

    # ================= PIXEL → ANGLE =================

    def pixel_to_angle(self, cx, width):
        """
        Eski özellik korunuyor.
        Piksel merkez offsetini yaklaşık açıya çevirir.
        """

        fov = 90  # approx degree
        center = width / 2

        angle = (cx - center) / center * (fov / 2)
        return math.radians(angle)

    # ================= TARGET ESTIMATE =================

    def estimate_target(self, angle, distance=2.0):
        """
        Eski özellik korunuyor.
        IKA pose biliniyorsa kamera açısına göre hedef tahmini üretir.
        """

        if self.ika_pose is None:
            return None

        x = self.ika_pose.position.x
        y = self.ika_pose.position.y

        target_x = x + distance * math.cos(angle)
        target_y = y + distance * math.sin(angle)

        return target_x, target_y

    # ================= MOVE TO TARGET =================

    def move_to_target(self, tx, ty):
        """
        Eski özellik korunuyor.
        Bu fonksiyon hâlâ ign topic üzerinden komut gönderebilir.
        Ana mission akışı IKACtrl kullansa da bunu silmiyoruz.
        """

        if self.ika_pose is None:
            return

        x = self.ika_pose.position.x
        y = self.ika_pose.position.y

        dx = tx - x
        dy = ty - y

        distance = math.sqrt(dx * dx + dy * dy)

        # Hedefe ulaştı
        if distance < 0.5:
            print("TARGET REACHED (WORLD)")

            subprocess.Popen([
                "ign", "topic",
                "-t", "/ugv_cmd_vel",
                "-m", "ignition.msgs.Twist",
                "-p", "linear: {x: 0.0}, angular: {z: 0.0}"
            ])

            self.run_arm()
            return

        # Yön hesapla
        angle = math.atan2(dy, dx)

        cmd = f"linear: {{x: 0.5}}, angular: {{z: {angle}}}"

        subprocess.Popen([
            "ign", "topic",
            "-t", "/ugv_cmd_vel",
            "-m", "ignition.msgs.Twist",
            "-p", cmd
        ])

    # ================= ARM =================

    def run_arm(self):
        """
        Eski özellik korunuyor.
        """
        print("ARM STARTED (simulated)")

    # ================= DRONE CAMERA =================

    def drone_cb(self, msg):
        now = time.time()
        if now - self.last_times["drone"] < 1.0 / self.FPS:
            return

        self.last_times["drone"] = now

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )

        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # Eski red ball detector özelliği korunuyor
        frame, found, cx, cy, area = detect_red_ball(frame)

        if found:
            # Eski yaklaşık world coordinate hesabı korunuyor
            drone_x = getattr(self, "drone_x", 0.0)
            drone_y = getattr(self, "drone_y", 0.0)
            drone_z = getattr(self, "drone_z", 3.0)

            img_w, img_h = msg.width, msg.height
            fov = 1.8

            ground_w = 2 * drone_z * math.tan(fov / 2)
            ground_h = ground_w * img_h / img_w

            wx = drone_x + (cx - img_w / 2) / img_w * ground_w
            wy = drone_y - (cy - img_h / 2) / img_h * ground_h

            print(
                f"RED TARGET at pixel({cx},{cy}) "
                f"-> world approx ({wx:.1f},{wy:.1f})"
            )

        frames["drone"] = frame

    # ================= IKA CAMERA =================

    def ika_cb(self, msg):
        now = time.time()
        if now - self.last_times["ika"] < 1.0 / self.FPS:
            return

        self.last_times["ika"] = now

        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )

        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # Eski kırmızı hedef tespiti korunuyor
        frame, found, cx, cy, area = detect_red_ball(frame)

        if found:
            cv2.putText(
                frame,
                f"IKA: RED TARGET ({cx},{cy})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

        frames["ika"] = frame

    # ================= ARM CAMERA =================

    def arm_cb(self, msg):
        self.process(msg, "arm")


# ================= START =================

def start_ros():
    """
    Backend main.py içinde background thread olarak çağrılıyor.
    """
    rclpy.init()
    node = CameraNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()