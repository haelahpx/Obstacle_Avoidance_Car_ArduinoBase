# 🚗 Arduino Autonomous Car with Obstacle Avoidance

This project is an Arduino-based autonomous car that uses **ultrasonic sensors**, a **servo motor**, and an **AI-like decision system** to detect and avoid obstacles. The car can move forward, stop, turn left/right, or make a U-turn depending on the distance readings from the sensor.

---

## 📌 Features
- Motor control using **L298N / H-Bridge driver**.
- **Ultrasonic sensor (HC-SR04)** for distance measurement.
- **Servo motor** to rotate the ultrasonic sensor and scan surroundings.
- Basic AI decision-making:
  - Move forward when the path is clear.
  - Stop when an obstacle is near.
  - Decide whether to turn left, right, or turn around.
- Serial output for debugging and monitoring distances.

---

## 🛠️ Hardware Requirements
- Arduino Mega (recommended for more pins)
- Motor Driver (L298N or similar)
- 2 DC Motors (for left & right wheels)
- Ultrasonic Sensor (HC-SR04)
- Servo Motor (SG90 or MG90S)
- Power supply (battery pack)
- Jumper wires & breadboard

---

## ⚙️ Pin Configuration
| Component             | Pin(s)       |
|-----------------------|--------------|
| Motor 1 Enable        | 9            |
| Motor 1 Direction     | 22, 23       |
| Motor 2 Enable        | 10           |
| Motor 2 Direction     | 24, 25       |
| Ultrasonic Trigger    | 26           |
| Ultrasonic Echo       | 27           |
| Servo Motor           | 11           |

---

## 💻 Software Requirements
- [Arduino IDE](https://www.arduino.cc/en/software) or [VS Code with Arduino extension](https://marketplace.visualstudio.com/items?itemName=vsciot-vscode.vscode-arduino)
- Required Libraries:
  - **Servo.h** (comes with Arduino IDE)
  - **NewPing.h** ([Download here](https://playground.arduino.cc/Code/NewPing/))

---

