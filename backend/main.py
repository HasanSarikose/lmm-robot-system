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