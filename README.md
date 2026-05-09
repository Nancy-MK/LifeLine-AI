# LifeLine AI — Source Code

ESP32 firmware and Node.js server for the LifeLine AI fire detection 
and evacuation guidance system.

## Contents
- /firmware — Arduino IDE sketch for ESP32
- /server — Node.js Express server with A* and Q-Learning pathfinding

## Running the server
1. Navigate to /server
2. Run: npm install
3. Run: node server.js

## Flashing the firmware
Open the .ino file in Arduino IDE, select ESP32 Dev Module, and upload.
