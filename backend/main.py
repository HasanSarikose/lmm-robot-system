from fastapi import FastAPI, WebSocket
import threading
import cv2
import base64
import asyncio
import subprocess

from frame_buffer import frames
from ros_node import start_ros

import requests
from mission_executor import execute_mission
from fastapi.middleware.cors import CORSMiddleware
import mission_executor

print("EXECUTOR PATH:", mission_executor.__file__)

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
# 🔥 ROS'u background thread'te başlat
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

        # 🔥 base64 encode
        jpg_as_text = base64.b64encode(buffer).decode("utf-8")

        await websocket.send_text(jpg_as_text)

        await asyncio.sleep(0.03)  # ~30 FPS

@app.get("/lidar")
def get_lidar():
    import subprocess

    result = subprocess.run(
        ["ign", "topic", "-e", "-t", "/ika/lidar", "--num", "1"],
        capture_output=True,
        text=True
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

    return {
        "ranges": ranges[:360]  # max 360 açı
    }

@app.post("/llm")
def run_llm(data: dict):
    prompt = data.get("prompt", "")

    system_prompt = f"""
You are a robot mission planner.

Convert command to JSON.

Robots:
- drone: search
- ika: go
- arm: pick

ONLY JSON.

Command:
{prompt}
"""

    response = requests.post(
        "http://localhost:11434/api/generate",
        json={
            "model": "llama3.1",
            "prompt": system_prompt,
            "stream": False
        }
    )

    result = response.json()["response"]

    print("LLM:", result)

    # 🔥 GÖREVİ ÇALIŞTIR
    execute_mission(result)

    return {"response": result}
