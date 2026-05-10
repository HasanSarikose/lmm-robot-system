from fastapi import FastAPI, WebSocket
import threading
import cv2
import base64
import asyncio
import subprocess
import requests
from fastapi.middleware.cors import CORSMiddleware

from frame_buffer import frames
from ros_node import start_ros
from mission_executor import execute_mission, get_log

from robot_controllers import ign_cmd

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ROS'u background thread'te baslat
threading.Thread(target=start_ros, daemon=True).start()
# ================= MANUAL ARM STATE =================

arm_state = {
    1: 0.0,
    2: 0.0,
    3: 0.0,
    4: 0.0,
    5: 0.0,
    6: 0.0,
}

arm_limits = {
    1: (-3.14, 3.14),
    2: (-1.57, 1.57),
    3: (-2.35, 2.35),
    4: (-3.14, 3.14),
    5: (-2.0, 2.0),
    6: (-3.14, 3.14),
}


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def send_arm_joint(joint_num, angle):
    subprocess.Popen([
        "ign", "topic",
        "-t", f"/arm_j{joint_num}",
        "-m", "ignition.msgs.Double",
        "-p", f"data: {angle}"
    ])


def send_gripper(action):
    if action == "open":
        left = -0.02
        right = 0.02
    else:
        left = 0.0
        right = 0.0

    subprocess.Popen([
        "ign", "topic",
        "-t", "/gripper_left",
        "-m", "ignition.msgs.Double",
        "-p", f"data: {left}"
    ])

    subprocess.Popen([
        "ign", "topic",
        "-t", "/gripper_right",
        "-m", "ignition.msgs.Double",
        "-p", f"data: {right}"
    ])

# ================= WEBSOCKET =================

@app.websocket("/ws/{camera}")
async def websocket_stream(websocket: WebSocket, camera: str):
    await websocket.accept()
    while True:
        frame = frames.get(camera)
        if frame is None:
            await asyncio.sleep(0.1)
            continue
        _, buffer = cv2.imencode(".jpg", frame)
        jpg_as_text = base64.b64encode(buffer).decode("utf-8")
        await websocket.send_text(jpg_as_text)
        await asyncio.sleep(0.03)


# ================= LIDAR =================

@app.get("/lidar")
def get_lidar():
    result = subprocess.run(
        ["ign", "topic", "-e", "-t", "/ika/lidar", "--num", "1"],
        capture_output=True,
        text=True,
        timeout=5
    )
    lines = result.stdout.split("\n")
    ranges = []
    for line in lines:
        if "ranges:" in line:
            try:
                val = float(line.split(":")[1])
                if val != float("inf") and val > 0:
                    ranges.append(val)
            except:
                continue
    return {"ranges": ranges[:360]}

# ================= MANUAL IKA CONTROL =================

@app.post("/manual/ika")
def manual_ika_control(data: dict):
    direction = data.get("direction", "stop")
    speed = float(data.get("speed", 0.18))
    turn = float(data.get("turn", 0.45))

    if direction == "forward":
        lx, az = speed, 0.0
    elif direction == "backward":
        lx, az = -speed, 0.0
    elif direction == "left":
        lx, az = 0.0, turn
    elif direction == "right":
        lx, az = 0.0, -turn
    elif direction == "forward_left":
        lx, az = speed * 0.7, turn * 0.6
    elif direction == "forward_right":
        lx, az = speed * 0.7, -turn * 0.6
    elif direction == "backward_left":
        lx, az = -speed * 0.7, turn * 0.6
    elif direction == "backward_right":
        lx, az = -speed * 0.7, -turn * 0.6
    else:
        lx, az = 0.0, 0.0

    ign_cmd(lx, az)

    return {
        "direction": direction,
        "linear_x": lx,
        "angular_z": az
    }

# ================= MANUAL ARM CONTROL =================

@app.post("/manual/arm")
def manual_arm_control(data: dict):
    action = data.get("action", "")
    joint = int(data.get("joint", 1))
    delta = float(data.get("delta", 0.08))

    if action == "joint_plus":
        if joint not in arm_state:
            return {"error": "invalid joint"}

        low, high = arm_limits[joint]
        arm_state[joint] = clamp(arm_state[joint] + delta, low, high)
        send_arm_joint(joint, arm_state[joint])

        return {
            "action": action,
            "joint": joint,
            "angle": arm_state[joint]
        }

    elif action == "joint_minus":
        if joint not in arm_state:
            return {"error": "invalid joint"}

        low, high = arm_limits[joint]
        arm_state[joint] = clamp(arm_state[joint] - delta, low, high)
        send_arm_joint(joint, arm_state[joint])

        return {
            "action": action,
            "joint": joint,
            "angle": arm_state[joint]
        }

    elif action == "home":
        for j in arm_state:
            arm_state[j] = 0.0
            send_arm_joint(j, 0.0)

        return {
            "action": "home",
            "state": arm_state
        }

    elif action == "open_gripper":
        send_gripper("open")
        return {"action": "open_gripper"}

    elif action == "close_gripper":
        send_gripper("close")
        return {"action": "close_gripper"}

    elif action == "pick_pose":
        # Basit toplama pozisyonu
        target = {
            1: -0.15,
            2: 1.57,
            3: 0.52,
            4: -0.28,
            5: 0.11,
            6: 0.35,
        }

        for j, angle in target.items():
            low, high = arm_limits[j]
            arm_state[j] = clamp(angle, low, high)
            send_arm_joint(j, arm_state[j])

        return {
            "action": "pick_pose",
            "state": arm_state
        }

    elif action == "place_pose":
        # Basit birakma pozisyonu
        target = {
            1: 1.5,
            2: 0.8,
            3: -1.0,
            4: 0.0,
            5: 0.0,
            6: 0.0,
        }

        for j, angle in target.items():
            low, high = arm_limits[j]
            arm_state[j] = clamp(angle, low, high)
            send_arm_joint(j, arm_state[j])

        return {
            "action": "place_pose",
            "state": arm_state
        }

    elif action == "status":
        return {
            "state": arm_state
        }

    return {"error": "unknown action"}

# ================= LLM + GOREV =================

@app.post("/llm")
def run_llm(data: dict):
    prompt = data.get("prompt", "")

    system_prompt = f"""Sen bir robot gorev planlayicisisin.
Kullanicinin komutunu analiz et.

Eger kullanici kirmizi top bulmak, toplamak, taramak gibi bir gorev istiyorsa:
{{"mission": "find_and_collect"}}

Eger sadece drone komutuysa:
{{"mission": "drone_only", "action": "takeoff", "params": {{"altitude": 5}}}}

Eger sadece IKA komutuysa:
{{"mission": "ika_only", "action": "move", "params": {{"direction": "ileri", "duration": 3}}}}

Eger sadece kol komutuysa:
{{"mission": "arm_only", "action": "pick"}}

SADECE JSON yanit ver.

Komut: {prompt}"""

    try:
        response = requests.post(
            "http://localhost:11434/api/generate",
            json={
                "model": "llama3.1",
                "prompt": system_prompt,
                "stream": False,
                "options": {"temperature": 0.1}
            },
            timeout=60
        )
        result = response.json()["response"]
        print("LLM:", result)

        # Gorevi calistir
        import json, re
        match = re.search(r'\{.*\}', result, re.DOTALL)
        if match:
            parsed = json.loads(match.group(0))
            mission_type = parsed.get("mission", "")

            if mission_type == "find_and_collect":
                execute_mission(result, frames_dict=frames)
                return {"response": result, "status": "Gorev baslatildi"}
            else:
                execute_mission(result, frames_dict=frames)
                return {"response": result, "status": "Komut islendi"}
        else:
            return {"response": result, "status": "JSON parse edilemedi"}

    except Exception as e:
        print(f"LLM Hata: {e}")
        return {"response": str(e), "status": "Hata"}


# ================= GOREV LOG =================

@app.get("/mission-log")
def mission_log():
    return {"log": get_log()}

@app.get("/detected-balls")
def get_detected_balls():
    from mission_executor import get_log
    logs = get_log()
    balls = []
    for l in logs:
        if "TOP BULUNDU" in l:
            balls.append(l)
    return {"balls": balls}


# ================= STATUS =================

@app.get("/status")
def get_status():
    has_drone = frames.get("drone") is not None
    has_ika = frames.get("ika") is not None
    has_arm = frames.get("arm") is not None
    return {
        "drone_camera": has_drone,
        "ika_camera": has_ika,
        "arm_camera": has_arm,
    }