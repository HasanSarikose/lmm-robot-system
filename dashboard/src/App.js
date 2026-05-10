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

// ================= MANUAL IKA CONTROL =================
function ManualIKAControl() {
  const intervalRef = useRef(null);

  const sendCommand = async (direction) => {
    try {
      await axios.post("http://localhost:8000/manual/ika", {
        direction: direction,
        speed: 0.16,
        turn: 0.45,
      });
    } catch (err) {
      console.error(err);
    }
  };

  const startMove = (direction) => {
    sendCommand(direction);

    if (intervalRef.current) {
      clearInterval(intervalRef.current);
    }

    intervalRef.current = setInterval(() => {
      sendCommand(direction);
    }, 150);
  };

  const stopMove = () => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }

    sendCommand("stop");
  };

  const btnStyle = {
    width: "75px",
    height: "45px",
    margin: "4px",
    borderRadius: "8px",
    border: "none",
    background: "#2563eb",
    color: "white",
    fontWeight: "bold",
    cursor: "pointer",
    userSelect: "none",
  };

  const stopStyle = {
    ...btnStyle,
    background: "#dc2626",
  };

  const controlButton = (label, direction, style = btnStyle) => (
    <button
      style={style}
      onMouseDown={() => startMove(direction)}
      onMouseUp={stopMove}
      onMouseLeave={stopMove}
      onTouchStart={(e) => {
        e.preventDefault();
        startMove(direction);
      }}
      onTouchEnd={(e) => {
        e.preventDefault();
        stopMove();
      }}
    >
      {label}
    </button>
  );

  return (
    <div
      style={{
        background: "#1e293b",
        padding: "15px",
        borderRadius: "10px",
        marginTop: "20px",
      }}
    >
      <h2>🎮 Manuel İKA Kontrol</h2>

      <div style={{ textAlign: "center" }}>
        <div>
          {controlButton("↖", "forward_left")}
          {controlButton("↑", "forward")}
          {controlButton("↗", "forward_right")}
        </div>

        <div>
          {controlButton("←", "left")}
          {controlButton("■", "stop", stopStyle)}
          {controlButton("→", "right")}
        </div>

        <div>
          {controlButton("↙", "backward_left")}
          {controlButton("↓", "backward")}
          {controlButton("↘", "backward_right")}
        </div>
      </div>

      <p style={{ fontSize: "12px", color: "#94a3b8", marginTop: "10px" }}>
        Butona basılı tuttuğun sürece İKA hareket eder, bıraktığında durur.
      </p>
    </div>
  );
}
// ================= MANUAL ARM CONTROL =================
function ManualArmControl() {
  const intervalRef = useRef(null);
  const [armState, setArmState] = useState({});

  const sendArmCommand = async (payload) => {
    try {
      const res = await axios.post("http://localhost:8000/manual/arm", payload);
      if (res.data.state) {
        setArmState(res.data.state);
      } else if (res.data.joint) {
        setArmState((prev) => ({
          ...prev,
          [res.data.joint]: res.data.angle,
        }));
      }
    } catch (err) {
      console.error(err);
    }
  };

  const startJointMove = (joint, direction) => {
    const action = direction === "plus" ? "joint_plus" : "joint_minus";

    sendArmCommand({
      action,
      joint,
      delta: 0.07,
    });

    if (intervalRef.current) {
      clearInterval(intervalRef.current);
    }

    intervalRef.current = setInterval(() => {
      sendArmCommand({
        action,
        joint,
        delta: 0.07,
      });
    }, 180);
  };

  const stopJointMove = () => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
  };

  const singleCommand = (action) => {
    stopJointMove();
    sendArmCommand({ action });
  };

  const btnStyle = {
    padding: "8px 12px",
    margin: "4px",
    borderRadius: "8px",
    border: "none",
    background: "#7c3aed",
    color: "white",
    fontWeight: "bold",
    cursor: "pointer",
    userSelect: "none",
  };

  const smallBtnStyle = {
    ...btnStyle,
    width: "45px",
  };

  const redBtnStyle = {
    ...btnStyle,
    background: "#dc2626",
  };

  const greenBtnStyle = {
    ...btnStyle,
    background: "#16a34a",
  };

  const jointRow = (joint) => (
    <div
      key={joint}
      style={{
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
        gap: "6px",
        marginBottom: "6px",
      }}
    >
      <span style={{ width: "45px" }}>J{joint}</span>

      <button
        style={smallBtnStyle}
        onMouseDown={() => startJointMove(joint, "minus")}
        onMouseUp={stopJointMove}
        onMouseLeave={stopJointMove}
        onTouchStart={(e) => {
          e.preventDefault();
          startJointMove(joint, "minus");
        }}
        onTouchEnd={(e) => {
          e.preventDefault();
          stopJointMove();
        }}
      >
        -
      </button>

      <span style={{ width: "75px", textAlign: "center", fontSize: "12px" }}>
        {armState[joint] !== undefined
          ? Number(armState[joint]).toFixed(2)
          : "0.00"}
      </span>

      <button
        style={smallBtnStyle}
        onMouseDown={() => startJointMove(joint, "plus")}
        onMouseUp={stopJointMove}
        onMouseLeave={stopJointMove}
        onTouchStart={(e) => {
          e.preventDefault();
          startJointMove(joint, "plus");
        }}
        onTouchEnd={(e) => {
          e.preventDefault();
          stopJointMove();
        }}
      >
        +
      </button>
    </div>
  );

  return (
    <div
      style={{
        background: "#1e293b",
        padding: "15px",
        borderRadius: "10px",
        marginTop: "20px",
      }}
    >
      <h2>🦾 Manuel Robot Kol Kontrol</h2>

      <div>
        {[1, 2, 3, 4, 5, 6].map((j) => jointRow(j))}
      </div>

      <div style={{ marginTop: "12px" }}>
        <button style={greenBtnStyle} onClick={() => singleCommand("open_gripper")}>
          Gripper Aç
        </button>

        <button style={redBtnStyle} onClick={() => singleCommand("close_gripper")}>
          Gripper Kapat
        </button>
      </div>

      <div style={{ marginTop: "8px" }}>
        <button style={btnStyle} onClick={() => singleCommand("home")}>
          Home
        </button>

        <button style={btnStyle} onClick={() => singleCommand("pick_pose")}>
          Pick Pose
        </button>

        <button style={btnStyle} onClick={() => singleCommand("place_pose")}>
          Place Pose
        </button>
      </div>

      <p style={{ fontSize: "12px", color: "#94a3b8", marginTop: "10px" }}>
        Joint butonlarına basılı tuttukça eklem açısı küçük adımlarla değişir.
      </p>
    </div>
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
          <ManualIKAControl />
          <ManualArmControl />
        </div>

      </div>

    </div>
  );
}

export default App;