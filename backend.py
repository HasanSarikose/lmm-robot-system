from fastapi import FastAPI
from fastapi.responses import StreamingResponse, JSONResponse
import cv2
import subprocess
import requests
import time

from mission_executor import execute_mission
from frame_store import frames

app = FastAPI()

# ================= GENERIC CAMERA STREAM =================

def generate_stream(name):
    while True:
        frame = frames.get(name)

        if frame is None:
            time.sleep(0.1)
            continue

        _, buffer = cv2.imencode('.jpg', frame)

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

        time.sleep(0.03)
# ================= CAMERA ENDPOINTS =================

@app.get("/camera/drone")
def drone():
    return StreamingResponse(generate_stream("drone"),
        media_type='multipart/x-mixed-replace; boundary=frame')

@app.get("/camera/ika")
def ika():
    return StreamingResponse(generate_stream("ika"),
        media_type='multipart/x-mixed-replace; boundary=frame')

@app.get("/camera/arm")
def arm():
    return StreamingResponse(generate_stream("arm"),
        media_type='multipart/x-mixed-replace; boundary=frame')
# ================= LIDAR =================

@app.get("/lidar")
def get_lidar():
    result = subprocess.run(
        ["ign", "topic", "-e", "-t", "/ika/lidar", "--num", "1"],
        capture_output=True,
        text=True
    )

    data = result.stdout

    if "range" in data:
        status = "Obstacle detected"
    else:
        status = "Clear"

    return {
        "status": status,
        "raw": data[:300]
    }

# ================= LLM =================

@app.post("/llm")
def run_llm(data: dict):
    try:
        prompt = data.get("prompt", "")

        system_prompt = f"""
You are a robot mission planner.

Convert the user command into JSON.

Available robots:
- drone → search
- ika → navigate
- arm → pick

Example:
Command: find red ball and bring it

Output:
{{
  "steps": [
    {{"robot": "drone", "action": "search_red_ball"}},
    {{"robot": "ika", "action": "go_to_target"}},
    {{"robot": "arm", "action": "pick_object"}}
  ]
}}

IMPORTANT:
Return ONLY valid JSON. No explanation.

Now convert this:
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

        # 🔥 1. LLM çıktısını al
        result = response.json()["response"]

        print("LLM OUTPUT:", result)  # debug için çok önemli

        # 🔥 2. MISSION ÇALIŞTIR
        execute_mission(result)

        # 🔥 3. UI’a geri gönder
        return {"response": result}

    except Exception as e:
        return {"error": str(e)}

# ================= HEALTH CHECK =================

@app.get("/")
def root():
    return {"status": "Backend running 🚀"}