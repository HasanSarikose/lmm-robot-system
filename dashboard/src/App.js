import React, { useEffect, useRef, useState } from "react";
import axios from "axios";


// ================= CAMERA COMPONENT =================
function Camera({ name }) {
  const imgRef = useRef();

  useEffect(() => {
    const ws = new WebSocket(`ws://localhost:8000/ws/${name}`);

    ws.onmessage = (event) => {
      if (imgRef.current) {
        imgRef.current.src = `data:image/jpeg;base64,${event.data}`;
      }
    };

    return () => ws.close();
  }, [name]);

  return (
    <div style={{ flex: 1 }}>
      <h3>📷 {name.toUpperCase()}</h3>
      <img ref={imgRef} width="100%" alt="camera stream" style={{ borderRadius: "10px" }} />
    </div>
  );
}


// ================= LIDAR CANVAS =================
function LidarCanvas({ data }) {
  const canvasRef = useRef(null);

  useEffect(() => {
    if (!data || data.length === 0) return;

    const canvas = canvasRef.current;
    const ctx = canvas.getContext("2d");

    const W = 400;
    const H = 400;

    ctx.clearRect(0, 0, W, H);

    const cx = W / 2;
    const cy = H / 2;

    const maxRange = Math.max(...data.filter(v => isFinite(v)));

    data.forEach((r, i) => {
      if (!isFinite(r)) return;

      const angle = (i / data.length) * 2 * Math.PI;
      const norm = r / maxRange;
      const radius = norm * 150;

      const x = cx + radius * Math.cos(angle);
      const y = cy + radius * Math.sin(angle);

      ctx.beginPath();
      ctx.arc(x, y, 2, 0, 2 * Math.PI);
      ctx.fillStyle = "lime";
      ctx.fill();
    });

    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, 2 * Math.PI);
    ctx.fillStyle = "red";
    ctx.fill();

  }, [data]);

  return (
    <canvas
      ref={canvasRef}
      width={400}
      height={400}
      style={{ background: "black" }}
    />
  );
}

// ================= MAIN APP =================
function App() {
  const [lidarData, setLidarData] = useState([]);
  const [prompt, setPrompt] = useState("");
  const [response, setResponse] = useState("");
  const [missionLog, setMissionLog] = useState([]);

  // REAL-TIME LIDAR
  useEffect(() => {
    const interval = setInterval(async () => {
      try {
        const res = await axios.get("http://localhost:8000/lidar");
        setLidarData(res.data.ranges || []);
      } catch (err) {
        console.error(err);
      }
    }, 500);

    return () => clearInterval(interval);
  }, []);

  // MISSION LOG POLLING
  useEffect(() => {
    const interval = setInterval(async () => {
      try {
        const res = await axios.get("http://localhost:8000/mission-log");
        setMissionLog(res.data.log || []);
      } catch (err) {}
    }, 2000);

    return () => clearInterval(interval);
  }, []);

  // LLM
  const sendPrompt = async () => {
    try {
      const res = await axios.post("http://localhost:8000/llm", {
        prompt: prompt,
      });
      setResponse(res.data.response);
    } catch (err) {
      console.error(err);
    }
  };


  return (
    <div style={{
      background: "#0f172a",
      color: "white",
      minHeight: "100vh",
      padding: "20px"
    }}>

      <h1>🚀 Multi-Robot AI Dashboard</h1>

      {/* ================= CAMERAS ================= */}
      <div style={{
        display: "flex",
        gap: "20px",
        marginTop: "20px"
      }}>
        <Camera name="drone" />
        <Camera name="ika" />
        <Camera name="arm" />
      </div>

      {/* ================= BOTTOM ================= */}
      <div style={{
        display: "flex",
        gap: "20px",
        marginTop: "30px"
      }}>

        {/* LIDAR */}
        <div style={{
          flex: 1,
          background: "#1e293b",
          padding: "15px",
          borderRadius: "10px"
        }}>
          <h2>📡 LiDAR (Real-Time)</h2>
          <LidarCanvas data={lidarData} />
        </div>

        {/* LLM + LOG */}
        <div style={{
          flex: 1,
          background: "#1e293b",
          padding: "15px",
          borderRadius: "10px"
        }}>
          <h2>🤖 LLM Komut</h2>

          <input
            value={prompt}
            onChange={(e) => setPrompt(e.target.value)}
            placeholder="örn: kırmızı topu bul ve getir"
            style={{
              width: "100%",
              padding: "10px",
              marginBottom: "10px",
              borderRadius: "5px",
              color: "black"
            }}
          />

          <button onClick={sendPrompt} style={{
            padding: "8px 20px",
            background: "#3b82f6",
            color: "white",
            border: "none",
            borderRadius: "5px",
            cursor: "pointer"
          }}>
            Gönder
          </button>

          <h3 style={{ marginTop: "15px" }}>🧠 LLM Çıktı:</h3>
          <pre style={{
            maxHeight: "100px",
            overflow: "auto",
            background: "#020617",
            padding: "10px",
            borderRadius: "5px",
            fontSize: "12px"
          }}>
            {response}
          </pre>

          <h3 style={{ marginTop: "15px" }}>📋 Görev Log:</h3>
          <pre style={{
            maxHeight: "250px",
            overflow: "auto",
            background: "#020617",
            padding: "10px",
            borderRadius: "5px",
            fontSize: "11px",
            whiteSpace: "pre-wrap",
            color: "#4ade80"
          }}>
            {missionLog.length > 0 ? missionLog.join("\n") : "Görev bekleniyor..."}
          </pre>
        </div>

      </div>

    </div>
  );
}

export default App;