# Development of Safety Systems for Marine Vehicles in Bangladesh Perspective

This repository contains the firmware for the prototype developed for the undergraduate thesis: **"Development of Safety Systems for Marine Vehicles in Bangladesh Perspective"**.

The project implements an Arduino-based control system for inland launches that combines autonomous collision avoidance with active stability monitoring.

**Contributors:**
* Istiak Ahmed Fahim (Roll: 1702132)
* Mohammad Mehedi Hasan (Roll: 1702128)

**Supervisor:**
Dr. Md. Rokunuzzaman, Professor, Dept. of Mechanical Engineering, RUET.

## System Overview

The system operates on two main logic loops to ensure vessel safety:

1. **Safety & Monitoring:**
   - **Tilt:** Uses an MPU6050 gyroscope. If tilt exceeds 20°, the system engages a safety lockout.
   - **Load:** Uses a resistive water sensor to monitor draft. Feedback is provided via LEDs (Safe/Warn/Critical) and a buzzer for the overweight condition (>5cm rise).
   - **Action:** If either condition is critical, the motor is cut off and alarms are triggered.

2. **Navigation:**
   - **Autonomous Mode:** Uses an HC-SR04 ultrasonic sensor. If an obstacle is detected within 100cm, the boat stops, steers to 135°, and navigates around it.
   - **Manual Mode:** Bluetooth control (HM-10) allows the operator to drive the vessel when safety conditions are met.

## Hardware Setup

**Platform:** Arduino UNO

| Component | Function | Connections |
| :--- | :--- | :--- |
| **L298N** | Motor Driver | D10 (PWM), D12, D11 |
| **MPU6050** | Gyroscope | A4 (SDA), A5 (SCL) |
| **HC-SR04** | Sonar | D8 (Trig), D9 (Echo) |
| **SG90** | Rudder Servo | D13 |
| **Water Sensor** | Load Monitor | A0 |
| **HM-10** | Bluetooth | D2 (RX), D3 (TX) |
| **LCD 16x2** | Display | I2C (A4, A5) |
| **Buzzer** | Alarm | D7 |
| **LEDs** | Status | D4 (Safe), D5 (Warn), D6 (Critical) |

*Note: LED status indications are calibrated as White (OK), Yellow (Warning), and Critical (Overweight).*

## Installation

1. Wiring must match the table above (refer to Figure 3.2 in the thesis report for the complete circuit diagram).
2. Required Libraries:
   - `LiquidCrystal_I2C`
   - `SoftwareSerial` (Built-in)
   - `Wire` (Built-in)
   - `Servo` (Built-in)
3. Upload `MarineSafetySystem.ino`.

## Bluetooth Commands

The system accepts the following char inputs via the HM-10 module:
* `F` : Forward
* `B` : Reverse
* `S` : Stop
* `L` : Rudder Left (45°)
* `R` : Rudder Right (135°)
* `C` : Rudder Center (90°)

## References

For full methodology, algorithm flowcharts (Fig 3.37), and experimental results, please refer to the attached project report.

**License**
MIT License
