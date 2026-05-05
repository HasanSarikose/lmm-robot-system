import subprocess
import time
import math

def ign_cmd(lx, az):
    """Tek bir hareket komutu gonder ve process'i kapat"""
    try:
        p = subprocess.Popen(['ign', 'topic', '-t', '/ugv_cmd_vel',
            '-m', 'ignition.msgs.Twist',
            '-p', f'linear: {{x: {lx}}}, angular: {{z: {az}}}'])
        p.wait(timeout=2)
    except:
        try: p.kill()
        except: pass


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

    def stop(self):
        ign_cmd(0.0, 0.0)

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

            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - yaw
            while angle_diff > math.pi: angle_diff -= 2 * math.pi
            while angle_diff < -math.pi: angle_diff += 2 * math.pi

            ranges = self.read_lidar()
            obstacle, min_d = self.check_front(ranges)

            if obstacle:
                print(f"[IKA] ENGEL! {min_d:.2f}m")
                n = len(ranges)
                left = ranges[n//2:n//2+n//4] if n > 0 else []
                right = ranges[n//4:n//2] if n > 0 else []
                left_avg = sum(left)/len(left) if left else 0
                right_avg = sum(right)/len(right) if right else 0
                ign_cmd(-0.15, 0.0)
                time.sleep(0.5)
                if left_avg > right_avg:
                    ign_cmd(0.0, 0.5)
                else:
                    ign_cmd(0.0, -0.5)
                time.sleep(0.8)
                ign_cmd(0.2, 0.0)
                time.sleep(0.6)
            else:
                if abs(angle_diff) > 0.2:
                    az = 0.3 if angle_diff > 0 else -0.3
                    ign_cmd(0.05, az)
                    time.sleep(0.2)
                else:
                    ign_cmd(0.2, angle_diff * 0.3)
                    time.sleep(0.3)

            if step % 15 == 0:
                print(f"[IKA] ({cx:.1f},{cy:.1f}) Yaw:{math.degrees(yaw):.0f} Hedefe:{dist:.1f}m")

        self.stop()
        return False

    def return_home(self):
        return self.navigate_to(self.start_x, self.start_y)


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