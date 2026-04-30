#!/usr/bin/env python3
import subprocess
import json
import time
import requests
import re

# ========== SENSOR OKUMA ==========

def read_sensor(topic, num=1, timeout=3):
    try:
        result = subprocess.run(
            ["ign", "topic", "-e", "-t", topic, "--num", str(num)],
            capture_output=True, text=True, timeout=timeout
        )
        return result.stdout
    except:
        return ""

def get_drone_imu():
    data = read_sensor("/drone/imu")
    if not data:
        return {"status": "veri yok"}
    info = {}
    for line in data.split("\n"):
        if "linear_acceleration" in data:
            info["sensor"] = "aktif"
    return {"sensor": "aktif" if data else "pasif"}

def get_ika_odom():
    data = read_sensor("/ika/odom")
    if not data:
        return {"x": 0, "y": 0, "yaw": 0}
    x = y = 0
    for line in data.split("\n"):
        line = line.strip()
        if line.startswith("x:") and "position" not in line:
            try:
                val = float(line.split(":")[1].strip())
                if x == 0:
                    x = val
            except:
                pass
        elif line.startswith("y:") and x != 0:
            try:
                y = float(line.split(":")[1].strip())
            except:
                pass
    return {"x": round(x, 2), "y": round(y, 2)}

def get_ika_lidar_summary():
    data = read_sensor("/ika/lidar")
    if not data:
        return {"engel": "bilinmiyor"}
    ranges = []
    for line in data.split("\n"):
        line = line.strip()
        if line.startswith("ranges:"):
            try:
                val = float(line.split(":")[1].strip())
                if val > 0.01 and val < 100:
                    ranges.append(val)
            except:
                pass
    if not ranges:
        return {"engel": "veri yok", "min_mesafe": 0}
    min_range = min(ranges)
    engel = "var" if min_range < 0.5 else "yok"
    return {
        "engel": engel,
        "min_mesafe": round(min_range, 2),
        "ortalama_mesafe": round(sum(ranges)/len(ranges), 2),
        "olcum_sayisi": len(ranges)
    }

def get_arm_joints():
    data = read_sensor("/arm/joint_states")
    if not data:
        return {}
    joints = {}
    current_joint = None
    for line in data.split("\n"):
        line = line.strip()
        if line.startswith("name:"):
            name = line.split(":")[1].strip().strip('"')
            current_joint = name
        elif "position" in line and current_joint:
            pass
    return {"status": "aktif"}

def get_all_status():
    status = {
        "drone": {
            "pozisyon": {"x": drone.x, "y": drone.y, "z": drone.z},
            "imu": get_drone_imu()
        },
        "ika": {
            "odom": get_ika_odom(),
            "lidar": get_ika_lidar_summary()
        },
        "robot_kol": {
            "joints": get_arm_joints()
        }
    }
    return status


# ========== ROBOT KONTROL ==========

class DroneCtrl:
    def __init__(self):
        self.x, self.y, self.z = 0.0, 0.0, 0.15

    def set_pose(self):
        subprocess.run(["ign", "service", "-s", "/world/lmm_world/set_pose",
            "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
            "--timeout", "100",
            "--req", f"name: 'drone', position: {{x: {self.x}, y: {self.y}, z: {self.z}}}"],
            capture_output=True)

    def takeoff(self, alt=5.0):
        print(f"[DRONE] Kalkis: {alt}m")
        while self.z < alt:
            self.z += 0.03
            self.set_pose()
            time.sleep(0.03)
        return f"Drone {alt}m yukseklige cikti"

    def goto(self, tx, ty, tz):
        print(f"[DRONE] Hedef: ({tx}, {ty}, {tz})")
        dist = ((tx-self.x)**2 + (ty-self.y)**2 + (tz-self.z)**2)**0.5
        if dist < 0.01:
            return f"Drone zaten ({tx}, {ty}, {tz}) konumunda"
        steps = max(int(dist / 0.03), 1)
        dx = (tx-self.x)/steps
        dy = (ty-self.y)/steps
        dz = (tz-self.z)/steps
        for _ in range(steps):
            self.x += dx; self.y += dy; self.z += dz
            self.set_pose()
            time.sleep(0.03)
        self.x, self.y, self.z = tx, ty, tz
        self.set_pose()
        return f"Drone ({tx}, {ty}, {tz}) konumuna ulasti"

    def land(self):
        print("[DRONE] Inis")
        while self.z > 0.15:
            self.z -= 0.03
            self.set_pose()
            time.sleep(0.03)
        self.z = 0.15
        self.set_pose()
        return "Drone inis yapti"


def ugv_move(direction, duration=2.0):
    print(f"[IKA] Hareket: {direction}, sure: {duration}s")
    speeds = {
        "ileri": (0.2, 0.0), "geri": (-0.2, 0.0),
        "sola": (0.0, 0.5), "saga": (0.0, -0.5),
        "daire": (0.15, 0.3), "dur": (0.0, 0.0)
    }
    lx, az = speeds.get(direction, (0.0, 0.0))
    steps = int(duration / 0.1)
    for _ in range(steps):
        subprocess.run(["ign", "topic", "-t", "/ugv_cmd_vel",
            "-m", "ignition.msgs.Twist",
            "-p", f"linear: {{x: {lx}}}, angular: {{z: {az}}}"],
            capture_output=True)
        time.sleep(0.1)
    subprocess.run(["ign", "topic", "-t", "/ugv_cmd_vel",
        "-m", "ignition.msgs.Twist", "-p", "linear: {x: 0}, angular: {z: 0}"],
        capture_output=True)
    return f"IKA {direction} yonunde {duration}s hareket etti"


def arm_joint(joint_num, angle):
    print(f"[KOL] Joint {joint_num}: {angle} rad")
    subprocess.run(["ign", "topic", "-t", f"/arm_j{joint_num}",
        "-m", "ignition.msgs.Double", "-p", f"data: {angle}"],
        capture_output=True)
    time.sleep(0.5)
    return f"Robot kol joint{joint_num} {angle} rad ayarlandi"


def arm_gripper(action):
    print(f"[KOL] Gripper: {action}")
    val_l = -0.02 if action == "ac" else 0.0
    val_r = 0.02 if action == "ac" else 0.0
    subprocess.run(["ign", "topic", "-t", "/gripper_left", "-m", "ignition.msgs.Double", "-p", f"data: {val_l}"], capture_output=True)
    subprocess.run(["ign", "topic", "-t", "/gripper_right", "-m", "ignition.msgs.Double", "-p", f"data: {val_r}"], capture_output=True)
    return f"Gripper {action} yapildi"


def arm_home():
    print("[KOL] Home pozisyon")
    for j in range(1, 7):
        arm_joint(j, 0.0)
    return "Robot kol home pozisyonuna geldi"


def arm_pick():
    print("[KOL] Pick hareketi")
    arm_joint(2, -.5)
    time.sleep(1.5)
    arm_joint(3, 1.5)
    time.sleep(1.5)
    arm_joint(5, 1.3)
    time.sleep(1)
    arm_gripper("ac")
    time.sleep(0.5)
    arm_gripper("kapa")
    return "Robot kol nesneyi aldi"


def arm_place():
    print("[KOL] Place hareketi")
    arm_joint(5, 0.0)
    time.sleep(0.5)
    arm_joint(3, -0.5)
    time.sleep(0.5)
    arm_joint(2, 0.3)
    time.sleep(0.5)
    arm_joint(1, 1.5)
    time.sleep(1)
    arm_gripper("ac")
    return "Robot kol nesneyi birakti"


# ========== LLM ==========

SYSTEM_PROMPT = """Sen bir coklu robot kontrol sistemisin. Kullanicinin dogal dil komutlarini analiz edip uygun robot komutlarina donusturuyorsun.

Kontrol edebildigin robotlar:
1. DRONE (IHA): takeoff(altitude), goto(x,y,z), land
2. UGV/IKA (4 tekerlekli arac): move(direction: ileri/geri/sola/saga/daire/dur, duration)
3. ARM/KOL (6-DOF Robot Kol): joint(num:1-6, angle), gripper(ac/kapa), home, pick, place

Ayrica sensor verilerini sorgulayabilirsin:
4. STATUS: Tum robotlarin sensor durumunu sorgula

Kullanicinin komutunu analiz et ve asagidaki JSON formatinda yanit ver:

{
  "commands": [
    {"robot": "drone", "action": "takeoff", "params": {"altitude": 5}},
    {"robot": "ugv", "action": "move", "params": {"direction": "ileri", "duration": 3}},
    {"robot": "arm", "action": "pick", "params": {}},
    {"robot": "system", "action": "status", "params": {}}
  ],
  "explanation": "Yapilan islemin kisa aciklamasi"
}

Onemli kurallar:
- robot degeri kucuk harf olmali: drone, ugv, arm, system
- IKA icin robot="ugv", action="move" kullan
- Eger kullanici durum/bilgi/sensor soruyorsa robot="system", action="status" kullan
- SADECE JSON formatinda yanit ver"""


def ask_llm(user_input):
    try:
        response = requests.post("http://localhost:11434/api/generate",
            json={
                "model": "llama3.1",
                "prompt": f"SYSTEM: {SYSTEM_PROMPT}\n\nKULLANICI: {user_input}\n\nJSON:",
                "stream": False,
                "options": {"temperature": 0.1}
            }, timeout=60)
        result = response.json()["response"].strip()
        if "{" in result:
            json_str = result[result.index("{"):result.rindex("}")+1]
            return json.loads(json_str)
    except Exception as e:
        print(f"LLM Hata: {e}")
    return None


def execute_commands(commands):
    results = []
    for cmd in commands:
        robot = cmd.get("robot", "").lower()
        action = cmd.get("action", "").lower()
        params = cmd.get("params", {})
        print(f"[DEBUG] robot='{robot}' action='{action}' params={params}")

        if robot == "drone":
            if action == "takeoff":
                results.append(drone.takeoff(params.get("altitude", 5)))
            elif action == "goto":
                results.append(drone.goto(params.get("x", 0), params.get("y", 0), params.get("z", 5)))
            elif action == "land":
                results.append(drone.land())
        elif robot in ["ugv", "ika", "ikav"]:
            if action == "move":
                results.append(ugv_move(params.get("direction", "ileri"), params.get("duration", 2)))
            elif action in ["ileri", "geri", "sola", "saga", "daire", "dur"]:
                results.append(ugv_move(action, params.get("duration", 2)))
        elif robot in ["arm", "kol", "robot_arm", "robot kol"]:
            if action == "joint":
                results.append(arm_joint(params.get("joint_num", params.get("num", 1)), params.get("angle", 0)))
            elif action == "gripper":
                results.append(arm_gripper(params.get("action", "ac")))
            elif action == "home":
                results.append(arm_home())
            elif action == "pick":
                results.append(arm_pick())
            elif action == "place":
                results.append(arm_place())
        elif robot == "system":
            if action == "status":
                status = get_all_status()
                print("\n[SISTEM DURUMU]")
                print(f"  DRONE  - Pozisyon: x={status['drone']['pozisyon']['x']:.2f} y={status['drone']['pozisyon']['y']:.2f} z={status['drone']['pozisyon']['z']:.2f}")
                print(f"           IMU: {status['drone']['imu']}")
                print(f"  IKA    - Odom: {status['ika']['odom']}")
                print(f"           LiDAR: {status['ika']['lidar']}")
                print(f"  KOL    - Joints: {status['robot_kol']['joints']}")
                results.append("Sistem durumu raporlandi")
    return results


# ========== ANA DONGU ==========

drone = DroneCtrl()

print("="*60)
print("  LMM TABANLI COKLU ROBOT KONTROL SISTEMI")
print("  Ollama + Llama 3.1 | Gazebo Fortress")
print("  Sensor Entegrasyonu Aktif")
print("="*60)
print()
print("Dogal dil ile komut verin:")
print("  Ornek: Drone'u 10 metreye kaldir")
print("  Ornek: IKA'nin onunde engel var mi?")
print("  Ornek: Tum robotlarin durumunu goster")
print("  Ornek: Drone'u kaldir ve IKA'yi ileri gonder")
print("  quit - Cikis")
print("="*60)

while True:
    try:
        user_input = input("\n> ").strip()
        if not user_input:
            continue
        if user_input.lower() == "quit":
            break

        print("\n[LLM] Komut analiz ediliyor...")
        result = ask_llm(user_input)

        if result and "commands" in result:
            print(f"[LLM] {result.get('explanation', '')}")
            print(f"[LLM] {len(result['commands'])} komut tespit edildi\n")
            results = execute_commands(result["commands"])
            print("\n[SONUC]")
            for r in results:
                print(f"  - {r}")
        else:
            print("[HATA] Komut anlasilamadi. Tekrar deneyin.")

    except KeyboardInterrupt:
        break

print("\nSistem kapatiliyor...")