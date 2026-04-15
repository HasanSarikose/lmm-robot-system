#!/usr/bin/env python3
import subprocess
import time

class DroneController:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.2

    def set_pose(self):
        subprocess.run([
            "ign", "service", "-s", "/world/lmm_world/set_pose",
            "--reqtype", "ignition.msgs.Pose",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "100",
            "--req", f"name: 'drone', position: {{x: {self.x}, y: {self.y}, z: {self.z}}}"
        ], capture_output=True)

    def takeoff(self, target_alt=5.0, speed=1.0):
        print(f"Kalkis: hedef {target_alt}m")
        step = speed * 0.05
        while self.z < target_alt:
            self.z += step
            self.set_pose()
            time.sleep(0.05)
        print(f"Yukseklik: {self.z:.1f}m")

    def land(self, speed=1.0):
        print("Inis yapiliyor...")
        step = speed * 0.05
        while self.z > 0.2:
            self.z -= step
            self.set_pose()
            time.sleep(0.05)
        self.z = 0.2
        self.set_pose()
        print("Inis tamamlandi")

    def goto(self, tx, ty, tz, speed=1.0):
        print(f"Hedef: ({tx}, {ty}, {tz})")
        dist = ((tx-self.x)**2 + (ty-self.y)**2 + (tz-self.z)**2)**0.5
        steps = max(int(dist / (speed * 0.05)), 1)
        dx = (tx - self.x) / steps
        dy = (ty - self.y) / steps
        dz = (tz - self.z) / steps
        for i in range(steps):
            self.x += dx
            self.y += dy
            self.z += dz
            self.set_pose()
            time.sleep(0.05)
        self.x, self.y, self.z = tx, ty, tz
        self.set_pose()
        print(f"Ulasildi: ({self.x:.1f}, {self.y:.1f}, {self.z:.1f})")

if __name__ == "__main__":
    drone = DroneController()
    print("Drone Controller")
    print("Komutlar: takeoff [alt], goto x y z, land, quit")
    while True:
        try:
            cmd = input("drone> ").strip().split()
            if not cmd:
                continue
            if cmd[0] == "takeoff":
                alt = float(cmd[1]) if len(cmd) > 1 else 5.0
                drone.takeoff(alt)
            elif cmd[0] == "goto":
                drone.goto(float(cmd[1]), float(cmd[2]), float(cmd[3]))
            elif cmd[0] == "land":
                drone.land()
            elif cmd[0] == "quit":
                break
            else:
                print("Kullanim: takeoff [alt], goto x y z, land, quit")
        except (IndexError, ValueError):
            print("Hatali parametre")
        except KeyboardInterrupt:
            break
