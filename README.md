# 🔥 LifeLine AI — Intelligent Fire Detection & Smart Evacuation System

> *Because when a building is on fire, "find your own way out" is not good enough.*

![ESP32](https://img.shields.io/badge/Hardware-ESP32-blue)
![Unity](https://img.shields.io/badge/App-Unity%20AR-black)
![Node.js](https://img.shields.io/badge/Server-Node.js-green)
![Status](https://img.shields.io/badge/Status-Final%20Prototype-orange)

---

## 🧠 What is LifeLine AI?

LifeLine AI is a fully **offline** intelligent fire detection and evacuation guidance system built as a final year BSc Computer Science project at the University of Hertfordshire.

It doesn't just detect fire — it **thinks**, and then it **guides you out**.

The system fuses real-time data from temperature, smoke, and CO sensors across an 11-room physical building model, computes personalised escape routes using a hybrid **A* and Q-Learning** pathfinding engine, and delivers turn-by-turn guidance through an **Augmented Reality** app on your phone.

No internet. No cloud. No failure point. Just get out safe.

---

## 🏗️ System Architecture

```
[ Physical Building Model ]
         │
    [ ESP32 Firmware ]        ← sensors + fusion algorithm + WiFi SoftAP
         │  HTTP
    [ Node.js Server ]        ← A* pathfinding + Q-Learning + REST API
         │  HTTP
  [ Unity iOS App ]           ← 2D map + AR navigation arrows
  [ Admin Dashboard ]         ← live risk heatmap + occupant tracking
```
## 📦 What's in This Repository

- `firmware/` — ESP32 Arduino sketch (.ino)
- `server/server.js` — A*, Q-Learning, all REST endpoints
- `server/qtable.json` — trained Q-table (auto-generated on first run)
- `server/package.json` — Node.js dependencies
  
> 🎮 The Unity project (4 GB) is hosted separately on Google Drive:
> **[Download Unity Project](https://drive.google.com/file/d/1x9IwmpWfwi-GgqRgaILUa2VmPCz-plc_/view?usp=sharing)**

---

## ⚡ How to Run It

### 1. Flash the ESP32 Firmware
- Open `firmware/LifeLine_AI.ino` in Arduino IDE
- Select board: **ESP32 Dev Module**
- Select the correct COM/USB port
- Click **Upload**
- Open Serial Monitor at **115200 baud** to confirm sensors are live

### 2. Start the Node.js Server
```bash
cd server
npm install
node server.js
```
Server runs on `http://localhost:3000`

### 3. Connect to the ESP32 WiFi
- On your laptop/phone, connect to WiFi: **SmartEvacSystem**
- Password: `evac2025`

### 4. Launch the Unity App
- Open the Unity project in Unity Editor
- Build and run on an iOS device with ARKit support
- Or run in the editor for 2D map mode

---

## 🔬 How the Magic Works

### 🌡️ Sensor Fusion
Each room has an **adaptive baseline** — the system learns what "normal" looks like for that specific room. Risk is scored from *deviation* from normal, not from fixed thresholds. A kitchen that's always warm doesn't trigger false alarms.

Four scoring components feed into every risk calculation:
- Temperature deviation from learned baseline
- Rate of temperature rise
- Gas (CO/smoke) deviation
- Cross-sensor confirmation bonus (when multiple sensors agree, risk multiplies)

### 🗺️ Pathfinding
A hybrid of two algorithms:
- **A*** finds the shortest safe path in real time, avoiding dangerous or blocked rooms
- **Q-Learning** scores every candidate path using learned traversal preferences, subtracting a reward bonus for historically good routes

The result: routes that are fast, safe, and get smarter over time.

### 📱 Augmented Reality
AR arrows appear at each doorway along your escape route. Point your camera, follow the arrows, reach the exit. Even in smoke, even in panic — one thing to focus on.

---

## 🧪 Testing Summary

| Test | Stimulus | Result |
|------|----------|--------|
| A | No stimulus (baseline) | ✅ Zero false positives |
| B | Incense at Kitchen MQ7 | ✅ Elevated risk, propagation correct |
| C | Lighter at Kitchen MQ7 | ✅ Risk capped at 39, alerts silent |
| D | Lighter at DS18B20 sensor | ✅ 94.8°C → LED + BUZZER activated |
| F | Stimulus removed | ✅ Decay confirmed, system cleared |

---

## 🛠️ Built With

| Layer | Technology |
|-------|-----------|
| Microcontroller | ESP32 DevKit |
| Temperature sensors | DS18B20 (1-Wire) |
| Gas sensors | MQ2 (smoke) + MQ7 (CO) |
| Occupancy sensors | PIR x9 |
| Firmware | Arduino IDE (C++) |
| Server | Node.js + Express |
| Pathfinding | A* + Q-Learning |
| App | Unity + ARKit (iOS) |
| Physical model | Laser-cut MDF + acrylic |

---

## 👩‍💻 Author

**Nancy Mohamed Mostafa Kamal**
BSc Computer Science — Artificial Intelligence
University of Hertfordshire, Cairo
Final Year Project — 2025/2026
Supervised by Dr. Mahmoud El Ghorab

---

## 📄 Licence

This project was developed for academic purposes.
All rights reserved © 2026 Nancy Mohamed Mostafa Kamal.
