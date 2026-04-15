#!/usr/bin/env python3
import subprocess
import json
import time
import requests

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
    arm_joint(2, 0.7)
    time.sleep(0.8)
    arm_joint(3, -0.9)
    time.sleep(0.8)
    arm_joint(5, 0.5)
    time.sleep(0.5)
    arm_gripper("ac")
    time.sleep(0.5)
    arm_gripper("kapa")
    return "Robot kol nesneyi aldi"


def arm_place():
    print("[KOL] Place hareketi")
    arm_joint(1, 1.5)
    time.sleep(1)
    arm_joint(2, 0.5)
    time.sleep(0.8)
    arm_joint(3, -0.5)
    time.sleep(0.5)
    arm_gripper("ac")
    return "Robot kol nesneyi birakti"


# ========== LLM ENTEGRASYONU ==========

SYSTEM_PROMPT = """Sen bir coklu robot kontrol sistemisin. Kullanicinin dogal dil komutlarini analiz edip uygun robot komutlarina donusturuyorsun.

Kontrol edebildigin robotlar:
1. DRONE (IHA): takeoff, goto(x,y,z), land
2. IKA (4 tekerlekli arac): ileri, geri, sola, saga, daire, dur
3. ROBOT KOL (6-DOF): joint(1-6, aci), gripper(ac/kapa), home, pick, place

Kullanicinin komutunu analiz et ve asagidaki JSON formatinda yanit ver. Birden fazla robot komutu olabilir:

{
  "commands": [
    {"robot": "drone", "action": "takeoff", "params": {"altitude": 5}},
    {"robot": "ugv", "action": "move", "params": {"direction": "ileri", "duration": 3}},
    {"robot": "arm", "action": "pick", "params": {}}
  ],
  "explanation": "Yapilan islemin kisa aciklamasi"
}

SADECE JSON formatinda yanit ver, baska bir sey yazma."""


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
    return results


# ========== ANA DONGU ==========

drone = DroneCtrl()

print("="*60)
print("  LMM TABANLI COKLU ROBOT KONTROL SISTEMI")
print("  Ollama + Llama 3.1 | Gazebo Fortress")
print("="*60)
print()
print("Dogal dil ile komut verin:")
print("  Ornek: Drone'u 10 metreye kaldir")
print("  Ornek: IKA'yi ileri gonder ve kolu pick pozisyonuna getir")
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