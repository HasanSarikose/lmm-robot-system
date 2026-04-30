#!/usr/bin/env python3
"""
OTONOM COKLU ROBOT GOREV SENARYOSU
- Drone alani tarar, kirmizi toplari kamera ile tespit eder
- Koordinatlari IKA'ya aktarir
- IKA hedefe LiDAR ile engelden kacinarak gider
- Robot kol topu toplar
- IKA baslangica doner
"""
import subprocess
import time
import math
import threading
import numpy as np

# OpenCV ve ROS 2 lazy import (main'de kontrol edilecek)
USE_ROS_CAMERA = False
detected_balls_from_camera = []

# ========== DRONE ==========
class DroneCtrl:
    def __init__(self):
        self.x, self.y, self.z = 0.0, 0.0, 0.15

    def set_pose(self):
        try:
            subprocess.run(["ign", "service", "-s", "/world/lmm_world/set_pose",
                "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
                "--timeout", "300",
                "--req", f"name: 'drone', position: {{x: {self.x}, y: {self.y}, z: {self.z}}}"],
                capture_output=True, timeout=10)
        except:
            pass

    def takeoff(self, alt=8.0):
        print(f"[DRONE] Kalkis: {alt}m")
        while self.z < alt:
            self.z += 0.5
            self.set_pose()
            time.sleep(0.05)
        print(f"[DRONE] Yukseklik: {self.z:.1f}m")

    def goto(self, tx, ty, tz):
        print(f"[DRONE] Hedef: ({tx:.1f}, {ty:.1f}, {tz:.1f})")
        dist = ((tx-self.x)**2 + (ty-self.y)**2 + (tz-self.z)**2)**0.5
        if dist < 0.01:
            return
        steps = max(int(dist / 0.5), 1)
        dx = (tx-self.x)/steps
        dy = (ty-self.y)/steps
        dz = (tz-self.z)/steps
        for _ in range(steps):
            self.x += dx; self.y += dy; self.z += dz
            self.set_pose()
            time.sleep(0.05)
        self.x, self.y, self.z = tx, ty, tz
        self.set_pose()

    def land(self):
        print("[DRONE] Inis")
        while self.z > 0.15:
            self.z -= 0.5
            self.set_pose()
            time.sleep(0.05)
        self.z = 0.15
        self.set_pose()
        print("[DRONE] Inis tamamlandi")


def detect_red_balls_from_image(drone_x, drone_y, drone_z):
    import cv2
    import os

    # Ayrı process ile frame kaydet
    try:
        subprocess.run(
            ["bash", "-c", "source /opt/ros/humble/setup.bash && python3 /home/hasan/lmm_robot_ws/capture_frame.py"],
            capture_output=True, timeout=30
        )
    except Exception as e:
        print(f"[KAMERA] Capture hata: {e}")
        return []

    img = cv2.imread("/tmp/drone_frame.png")
    if img is None:
        print("[KAMERA] Goruntu okunamadi")
        return []

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
    mask = mask1 | mask2

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    img_h, img_w = img.shape[:2]
    fov = 1.8
    alt = drone_z
    found = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 15 < area < 5000:
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * math.pi * area / (perimeter * perimeter)
            if circularity < 0.4:
                continue
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                px = int(M["m10"] / M["m00"])
                py = int(M["m01"] / M["m00"])
                ground_width = 2 * alt * math.tan(fov / 2)
                ground_height = ground_width * img_h / img_w
                wx = drone_x + (px - img_w / 2) / img_w * ground_width
                wy = drone_y - (py - img_h / 2) / img_h * ground_height
                found.append({"x": round(wx, 2), "y": round(wy, 2),
                             "px": px, "py": py, "area": area})
                print(f"[KAMERA] Top: piksel({px},{py}) -> dunya({wx:.1f},{wy:.1f}) alan={area:.0f}")

    return found
# ========== IKA ==========
class IKACtrl:
    def __init__(self):
        self.start_x = -2.0
        self.start_y = 0.0

    def read_lidar(self):
        try:
            result = subprocess.run(
                ["ign", "topic", "-e", "-t", "/ika/lidar", "--num", "1"],
                capture_output=True, text=True, timeout=5)
            ranges = []
            for line in result.stdout.split("\n"):
                line = line.strip()
                if line.startswith("ranges:"):
                    try:
                        val = float(line.split(":")[1].strip())
                        if 0.01 < val < 100:
                            ranges.append(val)
                    except:
                        pass
            return ranges
        except:
            return []

    def check_front(self, ranges):
        if not ranges:
            return False, 999
        n = len(ranges)
        front_start = n // 2 - n // 12
        front_end = n // 2 + n // 12
        front = ranges[front_start:front_end]
        if not front:
            return False, 999
        min_d = min(front)
        return min_d < 0.6, min_d

    def send_cmd(self, lx, az, duration=0.3):
        steps = int(duration / 0.1)
        for _ in range(steps):
            subprocess.run(["ign", "topic", "-t", "/ugv_cmd_vel",
                "-m", "ignition.msgs.Twist",
                "-p", f"linear: {{x: {lx}}}, angular: {{z: {az}}}"],
                capture_output=True)
            time.sleep(0.1)

    def stop(self):
        subprocess.run(["ign", "topic", "-t", "/ugv_cmd_vel",
            "-m", "ignition.msgs.Twist",
            "-p", "linear: {x: 0}, angular: {z: 0}"],
            capture_output=True)

    def read_odom(self):
        try:
            result = subprocess.run(
                ["ign", "topic", "-e", "-t", "/ika/odom", "--num", "1"],
                capture_output=True, text=True, timeout=5)
            x = y = 0
            qz = qw = 0
            in_pos = False
            in_orient = False
            for line in result.stdout.split("\n"):
                line = line.strip()
                if "position" in line and "orientation" not in line:
                    in_pos = True
                    in_orient = False
                elif "orientation" in line:
                    in_orient = True
                    in_pos = False
                elif in_pos and line.startswith("x:"):
                    try: x = float(line.split(":")[1].strip())
                    except: pass
                elif in_pos and line.startswith("y:"):
                    try: y = float(line.split(":")[1].strip())
                    except: pass
                elif in_orient and line.startswith("z:"):
                    try: qz = float(line.split(":")[1].strip())
                    except: pass
                elif in_orient and line.startswith("w:"):
                    try: qw = float(line.split(":")[1].strip())
                    except: pass
                    in_orient = False
            yaw = 2 * math.atan2(qz, qw)
            return x, y, yaw
        except:
            return 0, 0, 0

    def navigate_to(self, target_x, target_y):
        print(f"[IKA] Navigasyon: ({target_x:.1f}, {target_y:.1f})")
        for step in range(200):
            cx, cy, yaw = self.read_odom()
            dx = target_x - cx
            dy = target_y - cy
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 0.4:
                self.stop()
                print(f"[IKA] Hedefe ulasildi! ({cx:.1f}, {cy:.1f})")
                return True

            # Hedef aci ve aci farki
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - yaw
            # -pi ile pi arasina normalize et
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # LiDAR kontrolu
            ranges = self.read_lidar()
            obstacle, min_d = self.check_front(ranges)

            if obstacle:
                print(f"[IKA] ENGEL! {min_d:.2f}m - Kaciniliyor...")
                self.send_cmd(0.0, -0.5, 1.0)
                self.send_cmd(0.2, 0.0, 0.8)
            else:
                # Onc hedefe don, sonra ilerle
                if abs(angle_diff) > 0.3:
                    # Cok sapma var, yerinde don
                    turn = 0.5 if angle_diff > 0 else -0.5
                    self.send_cmd(0.0, turn, 0.3)
                else:
                    # Az sapma, ilerle ve hafif duzelt
                    self.send_cmd(0.2, angle_diff * 0.8, 0.3)

            if step % 10 == 0:
                print(f"[IKA] Poz: ({cx:.1f},{cy:.1f}) Yaw: {math.degrees(yaw):.0f}° Hedefe: {dist:.1f}m")

        self.stop()
        print("[IKA] Maks adim - durduruluyor")
        return False
    
    def return_home(self):
        print(f"[IKA] Eve donus: ({self.start_x}, {self.start_y})")
        return self.navigate_to(self.start_x, self.start_y)


# ========== ROBOT KOL ==========
class ArmCtrl:
    def joint(self, num, angle):
        subprocess.run(["ign", "topic", "-t", f"/arm_j{num}",
            "-m", "ignition.msgs.Double", "-p", f"data: {angle}"],
            capture_output=True)
        time.sleep(0.5)

    def gripper(self, action):
        vl = -0.02 if action == "ac" else 0.0
        vr = 0.02 if action == "ac" else 0.0
        subprocess.run(["ign", "topic", "-t", "/gripper_left",
            "-m", "ignition.msgs.Double", "-p", f"data: {vl}"], capture_output=True)
        subprocess.run(["ign", "topic", "-t", "/gripper_right",
            "-m", "ignition.msgs.Double", "-p", f"data: {vr}"], capture_output=True)
        time.sleep(0.3)

    def home(self):
        print("[KOL] Home")
        for j in range(1, 7):
            self.joint(j, 0.0)
        time.sleep(1)

    def pick(self):
        print("[KOL] Toplama...")
        self.gripper("ac")
        time.sleep(0.5)
        self.joint(2, 1.5)
        time.sleep(1.5)
        self.joint(3, -2.0)
        time.sleep(1.5)
        self.joint(5, 1.3)
        time.sleep(1)
        self.gripper("kapa")
        time.sleep(0.5)
        self.joint(3, -0.5)
        time.sleep(1)
        self.joint(2, 0.3)
        time.sleep(1)
        print("[KOL] Toplandi")

    def place(self):
        print("[KOL] Birakma...")
        self.joint(1, 1.5)
        time.sleep(1)
        self.joint(2, 0.8)
        time.sleep(1)
        self.joint(3, -1.0)
        time.sleep(1)
        self.gripper("ac")
        time.sleep(0.5)
        self.home()
        print("[KOL] Birakildi")


# ========== ANA GOREV ==========
def run_mission():
    drone = DroneCtrl()
    ika = IKACtrl()
    arm = ArmCtrl()

    print("=" * 60)
    print("  OTONOM COKLU ROBOT GOREV SENARYOSU")
    print("  Drone Kesfeder -> IKA Gider -> Kol Toplar")
    print("=" * 60)

    # ADIM 1: Drone kalkar
    print("\n" + "="*50)
    print("ADIM 1: DRONE KALKIS VE ALAN TARAMASI")
    print("="*50)
    drone.takeoff(8)
    time.sleep(1)

    # ADIM 2: Drone alani tarar ve her noktada kirmizi top arar
    scan_points = [
        (0, 0), (3, 0), (3, -3), (0, -3),
        (-3, -3), (-3, 0), (-3, 3), (0, 3), (3, 3)
    ]

    all_balls = []
    for i, (sx, sy) in enumerate(scan_points):
        print(f"\n[DRONE] Tarama {i+1}/{len(scan_points)}: ({sx}, {sy})")
        drone.goto(sx, sy, 8)
        time.sleep(1)

        # Kirmizi top tespiti
        balls = detect_red_balls_from_image(drone.x, drone.y, drone.z)
        if balls:
            for b in balls:
                # Ayni topu tekrar ekleme
                duplicate = False
                for existing in all_balls:
                    if abs(existing["x"] - b["x"]) < 1.0 and abs(existing["y"] - b["y"]) < 1.0:
                        duplicate = True
                        break
                if not duplicate:
                    all_balls.append(b)
                    print(f"[DRONE] KIRMIZI TOP BULUNDU! Dunya: ({b['x']}, {b['y']})")

    print(f"\n[DRONE] Tarama tamamlandi. Toplam {len(all_balls)} top tespit edildi:")
    for i, b in enumerate(all_balls):
        print(f"  Top {i+1}: ({b['x']}, {b['y']})")

    if not all_balls:
        print("[GOREV] Hic top bulunamadi. Gorev sona eriyor.")
        drone.land()
        return

    # Drone beklemede
    drone.goto(0, 0, 8)
    print("[DRONE] Koordinatlar IKA'ya aktarildi. Beklemede...")

    # ADIM 3: Her top icin IKA gider + Kol toplar
    for i, ball in enumerate(all_balls):
        print(f"\n{'='*50}")
        print(f"ADIM 3.{i+1}: TOP {i+1} TOPLAMA ({ball['x']}, {ball['y']})")
        print("="*50)

        arm.home()
        time.sleep(0.5)

        success = ika.navigate_to(ball["x"], ball["y"])
        if success:
            print(f"[GOREV] Top {i+1} konumunda. Kol toplama basliyor...")
            time.sleep(1)
            arm.pick()
            time.sleep(1)

            print(f"[GOREV] Top {i+1} toplandi. Eve donus...")
            arm.home()
            time.sleep(0.5)
            ika.return_home()

            print(f"[GOREV] Evde. Top {i+1} birakiliyor...")
            arm.place()
            time.sleep(1)
        else:
            print(f"[GOREV] Top {i+1} ulasilamadi, sonrakine geciliyor...")

    # ADIM 4: Bitis
    print(f"\n{'='*50}")
    print("ADIM 4: GOREV TAMAMLANDI")
    print("="*50)
    drone.land()
    arm.home()
    ika.stop()
    print("[GOREV] Tum islemler tamamlandi!")
    print(f"[GOREV] {len(all_balls)} top tespit edildi ve toplandi.")


if __name__ == "__main__":
    print("Otonom gorev baslatilsin mi? (e/h): ", end="")
    if input().strip().lower() == "e":
        run_mission()
    else:
        print("Iptal.")