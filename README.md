# 🐭 Micromouse Maze Solver 🤖

![Arduino](https://img.shields.io/badge/Microcontroller-Arduino%20Nano-blue?logo=arduino)
![C++](https://img.shields.io/badge/Language-C%2B%2B-brightgreen?logo=c%2B%2B)
![Status](https://img.shields.io/badge/Status-Working%20Prototype-success)

An **autonomous micromouse robot** designed for **maze navigation on a 10x10 cm grid** using a **left-hand wall following algorithm** and **Time-of-Flight (ToF) sensors**.  
The robot uses encoder feedback and PID-based correction for stable and accurate motion.

---

## 🚀 Features
- **Autonomous maze navigation** using Left-Hand Rule  
- **5× VL53L0X ToF sensors** for precise distance sensing  
- **Encoder-based motion tracking**  
- **PID-assisted wall following** for stability  
- **Accurate 90° and 180° turns** using kinematics  
- **Compact circular micromouse design**  

---

## 🧠 Working Logic

1. **Sensor Scan:** Reads distances from 5 ToF sensors  
2. **Wall Detection:** Determines presence of walls (front, left, right)  
3. **Decision Making (Left-Hand Rule):**
   - No wall on left → Turn Left  
   - Left blocked, front open → Move Forward  
   - Left & front blocked → Turn Right  
   - Dead end → Turn Back  

4. **Motion Execution:**
   - Forward movement with **PID correction**
   - Turns using encoder-based rotation  

---

## 🛠 Hardware Components

| Component | Qty | Description |
|----------|-----|-------------|
| **Arduino Nano** | 1 | Main controller |
| **VL53L0X ToF Sensors** | 5 | Distance sensing |
| **TB6612FNG** | 1 | Motor driver |
| **N20 Motors with Encoders** | 2 | Drive motors |
| **Li-ion Batteries (3.7V)** | 2 | Power supply (~7.4V) |
| **Perfboard Chassis** | 1 | Custom base |
| **Wheels + Caster** | — | Locomotion |

---

## ⚙️ Code Highlights
- **State-based decision system** using `enum Action`  
- **Left-hand rule implementation** for maze solving  
- **PID wall-following logic**
  - Uses **Kp = 0.8**, **Kd = 2.5**  
- **Encoder-based distance tracking**
- **Precise turning using arc length calculation**  
- **Multi-sensor I2C handling with unique addresses**  

---

## 📚 Libraries Required
- **Wire**
- **Encoder**
- **VL53L0X (Pololu library)**

---

## 📊 Key Parameters

- **Cell Size:** 100 mm  
- **Ticks per mm:** 4.244  
- **Track Width:** 75 mm  
- **Forward Speed:** 70  
- **Turn Speed:** 60  

---

## 📊 Current Status
- ✅ Fully working prototype  
- ⚙️ Reliable wall-following navigation  
- ⚙️ Under tuning and optimization  

---

## 📈 Future Improvements
- **Flood Fill algorithm** for optimal path solving  
- **Full PID (add Ki)** for better control  
- **Speed optimization for competition runs**  
- **Maze mapping + memory storage**  
- **Upgrade to ESP32 for higher performance**  

---

## 📌 Notes
- PID constants may require tuning based on maze and surface  
- Sensor thresholds (~80mm) are experimentally tuned  
- Accurate **TRACK_WIDTH_MM** is critical for precise turns  
- Ensure proper I2C wiring and unique sensor addressing via XSHUT  

---
