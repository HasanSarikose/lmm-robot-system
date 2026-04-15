#!/usr/bin/env python3
import subprocess
import time

class DroneController:
    def __init__(self):
        self.x, self.y, self.z = 0.0, 0.0, 0.2

    def set_pose(self):
        subprocess.run(["ign", "service", "-s", "/world/lmm_world/set_pose",
            "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
            "--timeout", "100",
            "--req", f"name: 'drone', position: {{x: {self.x}, y: {self.y}, z: {self.z}}}"],
            capture_output=True)

    def takeoff(self, alt=5.0):
        print(f"Drone kalkiyor: {alt}m")
        step = 0.05
        while self.z < alt:
            self.z += step
            self.set_pose()
            time.sleep(0.05)
        print(f"Yukseklik: {self.z:.1f}m")

    def land(self):
        print("Drone iniyor...")
        while self.z > 0.2:
            self.z -= 0.05
            self.set_pose()
            time.sleep(0.05)
        self.z = 0.2
        self.set_pose()
        print("Inis tamam")

    def goto(self, tx, ty, tz):
        print(f"Drone hedefe gidiyor: ({tx}, {ty}, {tz})")
        dist = ((tx-self.x)**2 + (ty-self.y)**2 + (tz-self.z)**2)**0.5
        steps = max(int(dist / 0.05), 1)
        dx, dy, dz = (tx-self.x)/steps, (ty-self.y)/steps, (tz-self.z)/steps
        for _ in range(steps):
            self.x += dx; self.y += dy; self.z += dz
            self.set_pose()
            time.sleep(0.05)
        self.x, self.y, self.z = tx, ty, tz
        self.set_pose()
        print(f"Drone ulasildi: ({self.x:.1f}, {self.y:.1f}, {self.z:.1f})")

class UGVController:
    def cmd(self, lx, az, duration=2.0):
        print(f"TurtleBot: linear={lx}, angular={az}, sure={duration}s")
        steps = int(duration / 0.1)
        for _ in range(steps):
            subprocess.run(["ign", "topic", "-t", "/ugv_cmd_vel",
                "-m", "ignition.msgs.Twist",
                "-p", f"linear: {{x: {lx}}}, angular: {{z: {az}}}"],
                capture_output=True)
            time.sleep(0.1)
        # Dur
        subprocess.run(["ign", "topic", "-t", "/ugv_cmd_vel",
            "-m", "ignition.msgs.Twist",
            "-p", "linear: {x: 0}, angular: {z: 0}"],
            capture_output=True)
        print("TurtleBot durdu")

    def ileri(self, sure=2.0):
        self.cmd(0.2, 0.0, sure)

    def geri(self, sure=2.0):
        self.cmd(-0.2, 0.0, sure)

    def sola_don(self, sure=1.5):
        self.cmd(0.0, 0.5, sure)

    def saga_don(self, sure=1.5):
        self.cmd(0.0, -0.5, sure)

    def daire(self, sure=5.0):
        self.cmd(0.15, 0.3, sure)

class ArmController:
    def set_joint(self, joint, value):
        print(f"Robot kol joint{joint}: {value} rad")
        subprocess.run(["ign", "topic", "-t", f"/arm_j{joint}",
            "-m", "ignition.msgs.Double",
            "-p", f"data: {value}"],
            capture_output=True)

    def home(self):
        print("Robot kol home pozisyonu")
        for j in range(1, 5):
            self.set_joint(j, 0.0)
            time.sleep(0.3)

    def pick_pose(self):
        print("Robot kol pick pozisyonu")
        self.set_joint(2, 0.8)
        time.sleep(1)
        self.set_joint(3, -1.0)
        time.sleep(1)
        self.set_joint(1, 0.5)

    def place_pose(self):
        print("Robot kol place pozisyonu")
        self.set_joint(1, -0.5)
        time.sleep(1)
        self.set_joint(2, 0.3)
        time.sleep(1)
        self.set_joint(3, -0.5)

drone = DroneController()
ugv = UGVController()
arm = ArmController()

print("="*50)
print("  LMM ROBOT KONTROL PANELI")
print("="*50)
print()
print("DRONE komutlari:")
print("  drone takeoff [alt]    - Kalkis")
print("  drone goto x y z       - Hedefe git")
print("  drone land             - Inis")
print()
print("TURTLEBOT komutlari:")
print("  ugv ileri [sure]       - Ileri git")
print("  ugv geri [sure]        - Geri git")
print("  ugv sola [sure]        - Sola don")
print("  ugv saga [sure]        - Saga don")
print("  ugv daire [sure]       - Daire ciz")
print()
print("ROBOT KOL komutlari:")
print("  arm j1/j2/j3/j4 deger - Eklem kontrol")
print("  arm home               - Home pozisyon")
print("  arm pick               - Alma pozisyonu")
print("  arm place              - Birakma pozisyonu")
print()
print("  quit                   - Cikis")
print("="*50)

while True:
    try:
        parts = input("robot> ").strip().split()
        if not parts:
            continue

        if parts[0] == "drone":
            if parts[1] == "takeoff":
                alt = float(parts[2]) if len(parts) > 2 else 5.0
                drone.takeoff(alt)
            elif parts[1] == "goto":
                drone.goto(float(parts[2]), float(parts[3]), float(parts[4]))
            elif parts[1] == "land":
                drone.land()

        elif parts[0] == "ugv":
            if parts[1] == "ileri":
                ugv.ileri(float(parts[2]) if len(parts) > 2 else 2.0)
            elif parts[1] == "geri":
                ugv.geri(float(parts[2]) if len(parts) > 2 else 2.0)
            elif parts[1] == "sola":
                ugv.sola_don(float(parts[2]) if len(parts) > 2 else 1.5)
            elif parts[1] == "saga":
                ugv.saga_don(float(parts[2]) if len(parts) > 2 else 1.5)
            elif parts[1] == "daire":
                ugv.daire(float(parts[2]) if len(parts) > 2 else 5.0)

        elif parts[0] == "arm":
            if parts[1] in ["j1","j2","j3","j4"]:
                arm.set_joint(int(parts[1][1]), float(parts[2]))
            elif parts[1] == "home":
                arm.home()
            elif parts[1] == "pick":
                arm.pick_pose()
            elif parts[1] == "place":
                arm.place_pose()

        elif parts[0] == "quit":
            break
        else:
            print("Bilinmeyen komut")

    except (IndexError, ValueError) as e:
        print(f"Hata: {e}")
    except KeyboardInterrupt:
        break

print("Kapatiliyor...")
