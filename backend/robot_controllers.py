import subprocess
import time
import math
import threading
from state_buffer import robot_state, state_lock

def ign_cmd(lx, az):
    """Tek bir hareket komutu gonder ve process'i kapat."""
    try:
        p = subprocess.Popen([
            "ign", "topic",
            "-t", "/ugv_cmd_vel",
            "-m", "ignition.msgs.Twist",
            "-p", f"linear: {{x: {lx}}}, angular: {{z: {az}}}"
        ])
        p.wait(timeout=2)
    except Exception:
        try:
            p.kill()
        except Exception:
            pass


class DroneCtrl:
    def __init__(self):
        self.x, self.y, self.z = 0.0, 0.0, 0.15

    def set_pose(self):
        try:
            subprocess.run([
                "ign", "service",
                "-s", "/world/lmm_world/set_pose",
                "--reqtype", "ignition.msgs.Pose",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "300",
                "--req",
                f"name: 'drone', position: {{x: {self.x}, y: {self.y}, z: {self.z}}}"
            ], capture_output=True, timeout=10)
        except Exception:
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

        dist = ((tx - self.x) ** 2 + (ty - self.y) ** 2 + (tz - self.z) ** 2) ** 0.5

        if dist < 0.01:
            return

        steps = max(int(dist / 0.5), 1)

        dx = (tx - self.x) / steps
        dy = (ty - self.y) / steps
        dz = (tz - self.z) / steps

        for _ in range(steps):
            self.x += dx
            self.y += dy
            self.z += dz
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
        """
        Eski ign topic subprocess okuması yerine,
        ros_node.py içindeki LaserScan subscriber'dan gelen veriyi okur.
        """
        with state_lock:
            if not robot_state.get("lidar_ready", False):
                return []

            return list(robot_state.get("lidar_ranges", []))

    def check_front(self, ranges):
        if not ranges:
            return False, 999

        n = len(ranges)

        # 360 derece LiDAR varsayimi:
        # array'in orta bolgesi robotun onu kabul ediliyor.
        s = n // 2 - n // 12
        e = n // 2 + n // 12

        front = ranges[s:e]

        if not front:
            return False, 999

        min_d = min(front)

        # 1.0 cok agresif durdurabiliyordu. 0.75 daha stabil.
        return min_d < 0.75, min_d

    def sector_average(self, ranges, start_ratio, end_ratio):
        if not ranges:
            return 0.0

        n = len(ranges)

        s = max(0, min(n - 1, int(n * start_ratio)))
        e = max(0, min(n, int(n * end_ratio)))

        sector = [r for r in ranges[s:e] if 0.2 < r < 12.0]

        if not sector:
            return 0.0

        return sum(sector) / len(sector)

    def avoid_obstacle(self, ranges):
        """
        Basit lokal engel kacis manevrasi.
        A* yerine daha stabil demo davranisi verir.
        """
        print("[IKA] Lokal engel kacis manevrasi")

        # Kisa geri
        ign_cmd(-0.12, 0.0)
        time.sleep(0.4)

        # 360 derece LiDAR varsayimi:
        # sol sektor: 0.50 - 0.75
        # sag sektor: 0.25 - 0.50
        left_avg = self.sector_average(ranges, 0.50, 0.75)
        right_avg = self.sector_average(ranges, 0.25, 0.50)

        if left_avg >= right_avg:
            ign_cmd(0.0, 0.45)
        else:
            ign_cmd(0.0, -0.45)

        time.sleep(0.7)

        # Kisa ileri
        ign_cmd(0.12, 0.0)
        time.sleep(0.5)

        self.stop()

    def stop(self):
        ign_cmd(0.0, 0.0)

    def read_odom(self):
        """
        Eski ign topic subprocess + string parse yerine,
        ros_node.py içindeki Odometry subscriber'dan gelen state'i okur.
        """
        with state_lock:
            if not robot_state.get("ika_odom_ready", False):
                return self.start_x, self.start_y, 0.0

            return (
                float(robot_state.get("ika_x", self.start_x)),
                float(robot_state.get("ika_y", self.start_y)),
                float(robot_state.get("ika_yaw", 0.0)),
            )

    def navigate_to(self, target_x, target_y, stop_distance=0.9, max_steps=220):
        """
        IKA'yi hedef bolgeye goturur.

        Topun tam merkezine gitmeye calismaz.
        stop_distance kadar yaklasinca basarili sayar.
        """
        print(
            f"[IKA] Navigasyon: hedef=({target_x:.1f}, {target_y:.1f}), "
            f"stop_distance={stop_distance:.1f}m"
        )

        for step in range(max_steps):
            cx, cy, yaw = self.read_odom()

            dx = target_x - cx
            dy = target_y - cy
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < stop_distance:
                self.stop()
                print(
                    f"[IKA] Hedef bolgeye ulasildi. "
                    f"Poz=({cx:.2f},{cy:.2f}), hedefe mesafe={dist:.2f}m"
                )
                return True

            target_angle = math.atan2(dy, dx)
            angle_diff = target_angle - yaw

            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi

            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            ranges = self.read_lidar()
            obstacle, min_d = self.check_front(ranges)

            if obstacle:
                print(f"[IKA] ENGEL algilandi: {min_d:.2f}m")
                self.avoid_obstacle(ranges)
                continue

            linear_speed = 0.18

            if dist < 1.8:
                linear_speed = 0.10

            if abs(angle_diff) > 0.45:
                az = 0.35 if angle_diff > 0 else -0.35
                ign_cmd(0.0, az)
                time.sleep(0.25)

            elif abs(angle_diff) > 0.18:
                az = 0.28 if angle_diff > 0 else -0.28
                ign_cmd(0.05, az)
                time.sleep(0.25)

            else:
                ign_cmd(linear_speed, angle_diff * 0.45)
                time.sleep(0.25)

            if step % 10 == 0:
                print(
                    f"[IKA] Poz=({cx:.2f},{cy:.2f}) "
                    f"Yaw={math.degrees(yaw):.0f} "
                    f"Hedefe={dist:.2f}m "
                    f"AciFarki={math.degrees(angle_diff):.1f}"
                )

        self.stop()

        cx, cy, _ = self.read_odom()
        final_dist = math.sqrt((target_x - cx) ** 2 + (target_y - cy) ** 2)

        print(f"[IKA] Hedef bolgeye ulasilamadi. Son mesafe={final_dist:.2f}m")

        return final_dist < stop_distance + 0.4

    def return_home(self):
        return self.navigate_to(self.start_x, self.start_y, stop_distance=0.45)


class ArmCtrl:
    def joint(self, num, angle):
        subprocess.run([
            "ign", "topic",
            "-t", f"/arm_j{num}",
            "-m", "ignition.msgs.Double",
            "-p", f"data: {angle}"
        ], capture_output=True)

        time.sleep(0.5)

    def gripper(self, action):
        vl = -0.02 if action == "ac" else 0.0
        vr = 0.02 if action == "ac" else 0.0

        subprocess.run([
            "ign", "topic",
            "-t", "/gripper_left",
            "-m", "ignition.msgs.Double",
            "-p", f"data: {vl}"
        ], capture_output=True)

        subprocess.run([
            "ign", "topic",
            "-t", "/gripper_right",
            "-m", "ignition.msgs.Double",
            "-p", f"data: {vr}"
        ], capture_output=True)

        time.sleep(0.3)

    def home(self):
        print("[KOL] Home")

        for j in range(1, 7):
            self.joint(j, 0.0)

        time.sleep(1)

    def pick(self):
        print("[KOL] Kalibre edilmis toplama hareketi...")

        # Gripper ac
        self.gripper("ac")
        time.sleep(0.5)

        # Kalibre edilen pick pozisyonu
        self.joint(1, -0.15)
        time.sleep(0.4)

        self.joint(2, 1.62)
        time.sleep(0.8)

        self.joint(3, 0.62)
        time.sleep(0.8)

        self.joint(4, -0.35)
        time.sleep(0.4)

        self.joint(5, 0.08)
        time.sleep(0.4)

        self.joint(6, 0.35)
        time.sleep(0.4)

        # Gripper kapa
        self.gripper("kapa")
        time.sleep(0.8)

        print("[KOL] Gripper Kapandı, attac için hazır")

    def lift_after_attach(self):
        """
        Attach komutundan sonra hedefi yerden kaldırmak için kullanılır.
        Attach'tan önce çağrılmamalı.
        """
        print("[KOL] Attach sonrasi hedef kaldiriliyor...")

        self.joint(2, 0.8)
        time.sleep(0.8)

        self.joint(3, -0.2)
        time.sleep(0.8)

        self.joint(4, 0.0)
        time.sleep(0.4)

        print("[KOL] Hedef tasima pozisyonuna alindi.")
    
    def place(self):
        print("[KOL] Birakma...")

        self.joint(1, 0.0)
        time.sleep(0.5)

        self.joint(2, 1.15)
        time.sleep(0.7)

        self.joint(3, 0.25)
        time.sleep(0.7)
        
        self.joint(4, -0.20)
        time.sleep(0.5)
        
        self.joint(5, 0.05)
        time.sleep(0.5)
        
        self.joint(6, 0.0)

        self.gripper("ac")
        time.sleep(0.5)

        print(" [KOL] Gripper acildi, detach için hazir.")

    def _publish_empty(self, topic):
        """
        Ignition/Gazebo topic'e Empty mesajı gönderir.
        DetachableJoint attach/detach için kullanılır.
        """
        try:
            subprocess.run([
                "ign", "topic",
                "-t", topic,
                "-m", "ignition.msgs.Empty",
                "-p", "unused: true"
            ], capture_output=True, timeout=2)
            return True
        except Exception as e:
            print(f"[KOL] Empty publish hata: {topic} -> {e}")
            return False

    def attach_target(self, target_id):
        """
        Hedefi gripper_base linkine DetachableJoint ile bağlar.
        target_id: red_circle, red_square, red_triangle
        """
        topic = f"/{target_id}/attach"
        print(f"[KOL] Attach istegi: {target_id} -> {topic}")

        ok = self._publish_empty(topic)
        time.sleep(0.4)

        if ok:
            print(f"[KOL] {target_id} attach komutu gonderildi.")
        else:
            print(f"[KOL] {target_id} attach komutu basarisiz.")

        return ok

    def detach_target(self, target_id):
        """
        Hedefi gripper_base linkinden ayırır.
        """
        topic = f"/{target_id}/detach"
        print(f"[KOL] Detach istegi: {target_id} -> {topic}")

        ok = self._publish_empty(topic)
        time.sleep(0.4)

        if ok:
            print(f"[KOL] {target_id} detach komutu gonderildi.")
        else:
            print(f"[KOL] {target_id} detach komutu basarisiz.")

        return ok

    def set_model_pose(self, model_name, x, y, z, yaw=0.0):
        """
        Eski yardımcı fonksiyon.
        Artık taşıma için kullanılmamalı.
        Sadece bırakma sonrası hedefi drop alanına düzgün yerleştirmek için kullanılabilir.
        """
        try:
            subprocess.run([
                "ign", "service",
                "-s", "/world/lmm_world/set_pose",
                "--reqtype", "ignition.msgs.Pose",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "300",
                "--req",
                (
                    f"name: '{model_name}', "
                    f"position: {{x: {x}, y: {y}, z: {z}}}, "
                    f"orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}"
                )
            ], capture_output=True, timeout=5)
        except Exception as e:
            print(f"[KOL] set_model_pose hata: {e}")

    def start_carrying_target(self, target_id, ika):
        """
        LEGACY:
        Eski set_model_pose tabanli tasima sistemi.
        Mission akışında artık kullanılmamalı.
        """
        stop_event = threading.Event()

        def carry_loop():
            print(f"[KOL] LEGACY {target_id} tasima modu basladi")

            while not stop_event.is_set():
                x, y, yaw = ika.read_odom()

                carry_x = x - 0.15 * math.cos(yaw)
                carry_y = y - 0.15 * math.sin(yaw)
                carry_z = 0.45

                self.set_model_pose(target_id, carry_x, carry_y, carry_z, yaw)
                time.sleep(0.2)

            print(f"[KOL] LEGACY {target_id} tasima modu bitti")

        t = threading.Thread(target=carry_loop, daemon=True)
        t.start()

        return stop_event, t

    def place_target(self, target_id, drop_x, drop_y, drop_z=0.03):
        """
        Hedefi home/drop alanina birakir.
        Not: Bu fonksiyon taşıma için değil, detach sonrası son yerleşim düzeltmesi için kullanılır.
        """
        print(f"[KOL] {target_id} drop noktasina birakiliyor: ({drop_x}, {drop_y})")
        self.set_model_pose(target_id, drop_x, drop_y, drop_z, 0.0)