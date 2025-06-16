# ESP32 Drone Flight Controller  
[![DOI](https://zenodo.org/badge/807372763.svg)](https://zenodo.org/doi/10.5281/zenodo.11381128)

A WiFi-enabled drone flight controller built with an ESP32, MPU6050 IMU, and brushless motor control using PID stabilization.

## 🚁 Overview

This project implements a real-time quadcopter flight controller that:
- Uses an **MPU6050** IMU for angle measurements.
- Stabilizes the drone using a **PID controller**.
- Controls 4 brushless motors via **ESCs** and **ESP32Servo**.
- Accepts user inputs over **WiFi** via an HTTP server.
- Supports **live tuning** of PID values and arming/disarming.

## 📦 Features
- Complementary filter for sensor fusion
- ESC calibration routine
- PID control for roll, pitch, and yaw
- Web control interface
- Real-time telemetry via serial
- Modular architecture: `main.cpp`, `controller.cpp`, `wifiServer.cpp`

## 🛠️ Hardware Requirements
- ESP32 Dev Board  
- MPU6050 sensor  
- 4x Brushless motors & ESCs  
- Power supply (LiPo)  
- WiFi-enabled device for control  

## ⚙️ Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/esp32-drone-controller.git
   cd esp32-drone-controller
   ```

2. Open in [PlatformIO](https://platformio.org/) or VS Code.

3. Connect your ESP32 and upload the code:
   ```bash
   pio run --target upload
   ```

4. Monitor serial output (for debugging and telemetry):
   ```bash
   pio device monitor
   ```

## 📡 Controls

Control inputs are sent via HTTP GET requests to the ESP32 over WiFi. Supported parameters include:

| Parameter | Function         |
|----------|------------------|
| `t`      | Throttle         |
| `r`      | Roll             |
| `p`      | Pitch            |
| `y`      | Yaw              |
| `aux1`   | Arm/Disarm       |
| `aux2`   | Acro mode toggle |
| `aux3`   | PID P gain       |
| `aux4`   | PID I gain       |
| `aux5`   | PID D gain       |

Example URL:
```
http://192.168.0.1/control?t=1200&r=1500&p=1500&y=1500&aux1=1600
```

## 📁 File Structure
- `main.cpp` – Main control loop and PID logic
- `wifiServer.cpp` – WiFi configuration and web server
- `controller.cpp` – Signal parsing and state management

## 📌 Notes
- For safety, the drone is automatically **disarmed** when WiFi is disconnected or more than one client is connected.
- You can use any HTML or app frontend to send control signals to the server at `http://192.168.0.1/control`.

## 📜 License
MIT License
