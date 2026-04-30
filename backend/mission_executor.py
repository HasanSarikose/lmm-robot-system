import json
import subprocess
import re 

def execute_mission(llm_output):
    try:
        clean = clean_json(llm_output)
        data = json.loads(clean)

        print("PARSED:", data)

        for robot in data["robots"]:
            name = robot["name"]
            command = robot["command"]

            print(f"➡ {name} → {command}")

            if name == "drone":
                run_drone(command)

            elif name == "ika":
                run_ika(command)

            elif name == "arm":
                run_arm(command)

    except Exception as e:
        print("Mission error:", e)


def run_drone(command):
    if command == "search":
        print("🚁 Drone searching")
        subprocess.Popen(["python3", "autonomous_mission.py"])


def run_ika(command):
    if command == "go":
        print("🚗 IKA moving")

        subprocess.Popen([
            "ign", "topic",
            "-t", "/ugv_cmd_vel",
            "-m", "ignition.msgs.Twist",
            "-p", "linear: {x: 0.5}"
        ])


def run_arm(command):
    if command == "pick":
        print("🤖 Arm picking")


def clean_json(text):
    match = re.search(r'\{.*\}', text, re.DOTALL)
    return match.group(0) if match else text