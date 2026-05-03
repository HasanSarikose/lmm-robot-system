import json
import subprocess
import re
import time
import math
import threading
import numpy as np

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

    def takeoff(self, alt=3.0):
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
                        if 0.5 < val < 100:
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
        s = n // 2 - n // 12
        e = n // 2 + n // 12
        front = ranges[s:e]
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
            x = y = qz = qw = 0
            in_pos = False
            in_orient = False
            for line in result.stdout.split("\n"):
                line = line.strip()
                if "position" in line and "orientation" not in line:
                    in_pos = True; in_orient = False
                elif "orientation" in line:
                    in_orient = True; in_pos = False
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

        for step in range(300):
            cx, cy, yaw = self.read_odom()
            dx = target_x - cx
            dy = target_y - cy
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 0.5:
                self.stop()
                print(f"[IKA] Hedefe ulasildi! ({cx:.1f}, {cy:.1f})")
                return True

            # Hedef acisi
            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - yaw
            while angle_diff > math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi

            # LiDAR
            ranges = self.read_lidar()
            obstacle, min_d = self.check_front(ranges)

            if obstacle:
                print(f"[IKA] ENGEL! {min_d:.2f}m")
                n = len(ranges)
                if n > 0:
                    left = ranges[n//2:n//2+n//4] if n > 0 else []
                    right = ranges[n//4:n//2] if n > 0 else []
                    left_avg = sum(left)/len(left) if left else 0
                    right_avg = sum(right)/len(right) if right else 0
                    
                    # Once geri git
                    subprocess.Popen(['ign', 'topic', '-t', '/ugv_cmd_vel', '-m', 'ignition.msgs.Twist', '-p', 'linear: {x: -0.15}, angular: {z: 0.0}'])
                    time.sleep(0.5)
                    
                    # Bos tarafa don
                    if left_avg > right_avg:
                        subprocess.Popen(['ign', 'topic', '-t', '/ugv_cmd_vel', '-m', 'ignition.msgs.Twist', '-p', 'linear: {x: 0.0}, angular: {z: 0.5}'])
                    else:
                        subprocess.Popen(['ign', 'topic', '-t', '/ugv_cmd_vel', '-m', 'ignition.msgs.Twist', '-p', 'linear: {x: 0.0}, angular: {z: -0.5}'])
                    time.sleep(0.8)
                    
                    # Ileri git
                    subprocess.Popen(['ign', 'topic', '-t', '/ugv_cmd_vel', '-m', 'ignition.msgs.Twist', '-p', 'linear: {x: 0.2}, angular: {z: 0.0}'])
                    time.sleep(0.6)
            else:
                # ADIM 1: Hedefe don
                if abs(angle_diff) > 0.2:
                    az = 0.3 if angle_diff > 0 else -0.3
                    subprocess.Popen(['ign', 'topic', '-t', '/ugv_cmd_vel', '-m', 'ignition.msgs.Twist', '-p', f'linear: {{x: 0.05}}, angular: {{z: {az}}}'])
                    time.sleep(0.2)
                else:
                    # ADIM 2: Duz git
                    subprocess.Popen(['ign', 'topic', '-t', '/ugv_cmd_vel', '-m', 'ignition.msgs.Twist', '-p', f'linear: {{x: 0.2}}, angular: {{z: {angle_diff * 0.3}}}'])
                    time.sleep(0.3)

            if step % 15 == 0:
                print(f"[IKA] ({cx:.1f},{cy:.1f}) Yaw:{math.degrees(yaw):.0f} Hedefe:{dist:.1f}m")

        self.stop()
        print("[IKA] Maks adim")
        return False
    
    def return_home(self):
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
        for j in range(1, 7): self.joint(j, 0.0)
        time.sleep(1)

    def pick(self):
        print("[KOL] Toplama...")
        self.gripper("ac"); time.sleep(0.5)
        self.joint(2, 1.5); time.sleep(1.5)
        self.joint(3, -2.0); time.sleep(1.5)
        self.joint(5, 1.3); time.sleep(1)
        self.gripper("kapa"); time.sleep(0.5)
        self.joint(3, -0.5); time.sleep(1)
        self.joint(2, 0.3); time.sleep(1)
        print("[KOL] Toplandi")

    def place(self):
        print("[KOL] Birakma...")
        self.joint(1, 1.5); time.sleep(1)
        self.joint(2, 0.8); time.sleep(1)
        self.joint(3, -1.0); time.sleep(1)
        self.gripper("ac"); time.sleep(0.5)
        self.home()

# ========== IKA KAMERA DOGRULAMA ==========
def verify_with_ika_camera(frames_dict):
    """IKA kamerasinda kirmizi top gorunuyor mu kontrol et"""
    import cv2

    frame = frames_dict.get("ika")
    if frame is None:
        return False, 0, 0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([15, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([165, 100, 50]), np.array([180, 255, 255]))
    mask = mask1 | mask2

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(cnt)
        if area > 50:
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                img_w = frame.shape[1]
                offset = (cx - img_w / 2) / (img_w / 2)
                print(f"[IKA-CAM] Top goruldu! Alan={area:.0f} Offset={offset:.2f}")
                return True, area, offset

    return False, 0, 0 

# ========== KIRMIZI TOP TESPITI ==========
def detect_from_frame(frames_dict, drone_x, drone_y, drone_z):
    import cv2

    frame = frames_dict.get("drone")
    if frame is None:
        print("[DETECT] Drone frame yok")
        return []

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([15, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([165, 100, 50]), np.array([180, 255, 255]))
    mask = mask1 | mask2

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    img_h, img_w = frame.shape[:2]
    fov = 1.8
    found = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 80 < area < 2000:
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * math.pi * area / (perimeter * perimeter)
            if circularity < 0.6:
                continue
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                px = int(M["m10"] / M["m00"])
                py = int(M["m01"] / M["m00"])
                ground_w = 2 * drone_z * math.tan(fov / 2)
                ground_h = ground_w * img_h / img_w
                wx = drone_x + (px - img_w/2) / img_w * ground_w
                wy = drone_y - (py - img_h/2) / img_h * ground_h
                found.append({"x": round(wx, 2), "y": round(wy, 2), "area": area})
                print(f"[DETECT] Top: piksel({px},{py}) -> dunya({wx:.1f},{wy:.1f})")

    return found


# ========== MISSION LOG ==========
mission_log = []

def get_log():
    return mission_log


def execute_mission(llm_output, frames_dict=None):
    global mission_log
    mission_log = []

    def log(msg):
        print(msg)
        mission_log.append(msg)

    def run():
        drone = DroneCtrl()
        ika = IKACtrl()
        arm = ArmCtrl()

        log("=" * 50)
        log("GOREV BASLADI")
        log("=" * 50)

        # ADIM 1: Drone kalkar ve tarar
        log("[ADIM 1] Drone kalkiyor...")
        drone.takeoff(3)
        time.sleep(1)

        scan_points = [
            (0, 0), (2, 0), (4, 0),
            (4, -2), (2, -2), (0, -2),
            (-2, -2), (-4, -2),
            (-4, 0), (-2, 0),
            (-2, 2), (0, 2), (2, 2), (4, 2),
            (4, 4), (2, 4), (0, 4),
            (-2, 4), (-4, 4),
            (-4, 2), (-3, 0), (3, -1),
        ]

        all_balls = []
        for i, (sx, sy) in enumerate(scan_points):
            log(f"[DRONE] Tarama {i+1}/{len(scan_points)}: ({sx}, {sy})")
            drone.goto(sx, sy, 3)
            time.sleep(3)

            if frames_dict:
                balls = detect_from_frame(frames_dict, drone.x, drone.y, drone.z)
                for b in balls:
                    dup = False
                    for ex in all_balls:
                        if abs(ex["x"]-b["x"]) < 2.0 and abs(ex["y"]-b["y"]) < 2.0:
                            dup = True
                            break
                    if not dup:
                        all_balls.append(b)
                        log(f"[DRONE] TOP BULUNDU! ({b['x']}, {b['y']})")

        log(f"[DRONE] Tarama bitti. {len(all_balls)} top tespit edildi.")

        if not all_balls:
            log("[GOREV] Top bulunamadi. Bitis.")
            drone.land()
            return

        drone.goto(0, 0, 3)
        log("[DRONE] Koordinatlar IKA'ya aktarildi.")

        # ADIM 2: Her top icin IKA git + kol topla
        for i, ball in enumerate(all_balls):
            log(f"[ADIM 2.{i+1}] Top {i+1}: ({ball['x']}, {ball['y']})")
            arm.home()
            time.sleep(0.5)

            success = ika.navigate_to(ball["x"], ball["y"])
            if success:
                # IKA kamerasiyla topu bul ve tam ustune git
                log(f"[IKA-CAM] Top araniyor...")
                aligned = False
                for attempt in range(20):
                    if frames_dict:
                        found, area, offset = verify_with_ika_camera(frames_dict)
                        if found:
                            if area > 500:
                                log(f"[IKA-CAM] Top cok yakin! Alan={area:.0f}")
                                ika.stop()
                                aligned = True
                                break
                            elif abs(offset) > 0.15:
                                az = -0.2 if offset > 0 else 0.2
                                subprocess.Popen(['ign', 'topic', '-t', '/ugv_cmd_vel', '-m', 'ignition.msgs.Twist', '-p', f'linear: {{x: 0.1}}, angular: {{z: {az}}}'])
                                time.sleep(0.3)
                            else:
                                subprocess.Popen(['ign', 'topic', '-t', '/ugv_cmd_vel', '-m', 'ignition.msgs.Twist', '-p', 'linear: {x: 0.15}, angular: {z: 0.0}'])
                                time.sleep(0.3)
                        else:
                            subprocess.Popen(['ign', 'topic', '-t', '/ugv_cmd_vel', '-m', 'ignition.msgs.Twist', '-p', 'linear: {x: 0.1}, angular: {z: 0.1}'])
                            time.sleep(0.3)
                    time.sleep(0.2)

                ika.stop()
                if not aligned:
                    log(f"[IKA-CAM] Top hizalanamadi ama yine de toplaniyor...")

                log(f"[GOREV] Top {i+1} konumunda. Kol topluyor...")
                arm.pick()
                time.sleep(1)
                log(f"[GOREV] Top {i+1} toplandi. Eve donus...")
                arm.home()
                ika.return_home()
                arm.place()
                log(f"[GOREV] Top {i+1} birakildi.")
            else:
                log(f"[GOREV] Top {i+1} ulasilamadi.")

        # ADIM 3: Bitis
        drone.land()
        arm.home()
        ika.stop()
        log("=" * 50)
        log("GOREV TAMAMLANDI!")
        log("=" * 50)

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return "Gorev baslatildi"


def clean_json(text):
    match = re.search(r'\{.*\}', text, re.DOTALL)
    return match.group(0) if match else text