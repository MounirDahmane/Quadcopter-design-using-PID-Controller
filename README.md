# ESP32 Drone Flight Controller  

[![DOI](https://zenodo.org/badge/807372763.svg)](https://zenodo.org/doi/10.5281/zenodo.11381128)

A WiFi-enabled drone flight controller built with an ESP32, MPU6050 IMU, and brushless motor control using PID controller for stabilization.

## 🚁 Overview

This project implements a real-time quadcopter flight controller that:

- Uses an **MPU6050** IMU for angle measurements via complementary filter sensor fusion
- Stabilizes the drone using a **PID controller** with tunable gains
- Controls 4 brushless motors via **ESCs** and **ESP32Servo** library
- Accepts user inputs over **WiFi** via an HTTP REST API from mobile application
- Implements safety features (auto-disarm on WiFi loss, single-client enforcement)

## 📦 Features

- **Sensor Fusion**: Complementary filter combining gyroscope and accelerometer data
- **PID Stabilization**: Separate PID loops for roll, pitch, and yaw control
- **Motor Mixing**: X-configuration motor layout with proper stabilization mix
- **ESC Calibration**: Automatic calibration routine on startup
- **WiFi Control**: Real-time control via HTTP requests
- **Safety Systems**: 
  - Automatic disarm on WiFi disconnect
  - Single-client enforcement (prevents multi-user interference)
  - Motor speed clamping and anti-windup protection
- **Debug Telemetry**: Serial output for tuning and diagnostics
- **Status LED**: Visual feedback (flashing = disarmed, solid = armed)

## 🛠️ Hardware Requirements

- **Microcontroller**: ESP32 (any variant, tested on ESP32 DevKit v1)
- **IMU**: MPU6050 (6-axis gyro + accelerometer)
- **Motors**: 4x Brushless DC motors (typical quadcopter specs)
- **ESCs**: 4x Electronic Speed Controllers (ESCs) rated for motor
- **Power**: Suitable LiPo battery (see utils/Report.pdf for full BOM)
- **Communication**: I2C bus for MPU6050 (GPIO 21 = SDA, GPIO 22 = SCL)
- **Motor Pins**: GPIO 15, 23, 4, 16

For detailed hardware specifications, see `utils/Report.pdf`

## ⚙️ Software Architecture

### File Structure

```
├── main.cpp              # Flight control loop and stabilization algorithm
├── controller.h/cpp      # WiFi input controller interface
├── wifiserver.h/cpp      # HTTP server and WiFi soft AP setup
├── secret.h              # WiFi credentials (excluded from git)
├── README.md             # This file
└── platformio.ini        # PlatformIO build configuration
```

### Key Components

**Main Loop** (`main.cpp`):
- Reads IMU orientation at high frequency (>100Hz)
- Processes control inputs from WiFi
- Calculates PID corrections
- Sends corrected signals to motors
- Updates status LED

**PID Controller**:
- **Roll/Pitch Control**: Angle-based stabilization
  - Kp = 0.93 (proportional gain)
  - Ki = 0.03-0.035 (integral gain for drift elimination)
  - Kd = 3.0/0 (derivative for damping)
- **Yaw Control**: Rate-based (not used in current config)

**WiFi Interface**:
- Creates soft access point (192.168.0.1)
- Accepts HTTP requests at `/control` endpoint
- Parses stick inputs (microsecond range: 1000-2000)
- Maps to physical angles (-10° to +10°)

### Control Input Protocol

Send HTTP requests to: `http://192.168.0.1/control?r=1500&p=1500&t=1500&aux1=2000`

**Query Parameters**:

| Parameter | Name | Range | Mapping |
|-----------|------|-------|---------|
| `r` | Roll | 1000-2000 µs | -10° to +10° |
| `p` | Pitch | 1000-2000 µs | -10° to +10° |
| `t` | Throttle | 1000-2000 µs | Motor speed |
| `y` | Yaw | 1000-2000 µs | *Reserved* |
| `aux1` | Armed | ≥1500 = armed | Arm/Disarm |
| `aux2` | Acro Mode | 0-2000 | Mode select |
| `aux3` | P Gain | 0-200 | PID tuning |
| `aux4` | I Gain | 0-100 | PID tuning |
| `aux5` | D Gain | 0-100 | PID tuning |

**Example HTTP Request** (cURL):
```bash
curl "http://192.168.0.1/control?r=1500&p=1500&t=1200&aux1=1500&aux3=93&aux4=3&aux5=30"
```

## 🚀 Getting Started

### Prerequisites
- PlatformIO or Arduino IDE with ESP32 board support
- Required libraries:
  - `rfetick/MPU6050_light@^1.1.0`
  - `madhephaestus/ESP32Servo@^3.0.8`

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/MounirDahmane/Quadcopter-design-using-PID-Controller.git
   cd Quadcopter-design-using-PID-Controller
   ```

2. **Update WiFi credentials** (edit `secret.h`)
   ```cpp
   #define WIFI_SSID "YourNetworkName"
   #define WIFI_PASSWORD "YourPassword"
   ```

3. **Build and Upload with PlatformIO**
   ```bash
   pio run -t upload
   ```
   Or with Arduino IDE: File → Upload

4. **Open Serial Monitor** (115200 baud)
   ```bash
   pio device monitor
   ```

### Initial Setup Checklist

⚠️ **PROPELLERS OFF** during entire initial setup

1. **Verify Serial Output**
   - Should see "MPU6050 status: 0" (0 = success)
   - ESC calibration will complete (~2 seconds)
   - LED will turn on = ready

2. **Check WiFi**
   - Connect phone/computer to "ESP32" WiFi network
   - Password: "12345678" (from secret.h)
   - Try accessing: `http://192.168.0.1/` (should show "Hello world")

3. **Test Motor Controls** (propellers STILL OFF)
   - Access: `http://192.168.0.1/control?r=1500&p=1500&t=1200`
   - Watch Serial Monitor output and verify values update
   - Check LED status (should blink when disarmed)

4. **Calibrate IMU** (optional, if drifts)
   - Uncomment `calibration();` in `setup()`
   - Ensure drone is level and stationary
   - Takes ~30 seconds

5. **Tune PID Gains** (advanced)
   - Adjust `Kp`, `Ki`, `Kd` arrays in `main.cpp`
   - Or use `aux3`, `aux4`, `aux5` parameters via HTTP
   - Monitor Serial telemetry (INFO function output)

### Flight Operations

⚠️ **All propellers must be installed and tightened**

1. **Pre-Flight Check**
   - Battery fully charged
   - All motor bearings spin freely
   - No loose wires or connections
   - IMU is level and stationary

2. **Arming**
   - Send: `http://192.168.0.1/control?aux1=2000` (armed signal)
   - LED should turn solid on (indicates armed)
   - Motors will respond to throttle input

3. **Flight**
   - Gradually increase throttle (`t` parameter) from 1000
   - Use roll (`r`) and pitch (`p`) to stabilize
   - Monitor Serial output for diagnostics

4. **Emergency Disarm**
   - Send: `http://192.168.0.1/control?aux1=1000` (disarmed signal)
   - Or: Disconnect WiFi client
   - LED will blink (indicates safe/disarmed)

## 📊 Telemetry Output

Serial Monitor outputs tab-separated values:

```
Throttle | Angle_X | Angle_Y | Angle_Z | Roll_PID | Pitch_PID | Yaw_PID | ESC1 | ESC2 | ESC3 | ESC4 | X_err | Y_err | Z_err
1200     | 2.3     | -1.1    | 0.5     | 15.2     | -8.3      | 0.2     | 1215 | 1195 | 1210 | 1190 | 0.02  | -0.01 | 0.00
```

Use this for tuning and diagnostics. Update interval: 100ms

## 📋 PID Tuning Guide

### Proportional Gain (Kp)
- Controls response strength to angle error
- **Too Low**: Slow, sluggish response, drifts
- **Too High**: Oscillations, jittery control

### Integral Gain (Ki)
- Eliminates steady-state drift over time
- **Too Low**: Drone doesn't stay level
- **Too High**: Windup effect, overshoot
- **Windup Prevention**: Integral resets when stick is centered

### Derivative Gain (Kd)
- Dampens oscillations and overshoot
- **Too Low**: Bouncy, oscillatory response
- **Too High**: Jerky movements, amplifies noise

### Tuning Process
1. Set `Kp` high enough for response (aim for 0.8-1.2)
2. Add small `Ki` (0.02-0.05) to eliminate drift
3. Add `Kd` (2-5) if oscillating
4. Monitor Serial telemetry while adjusting
5. Test in flight and adjust gradually

## 📌 Important Notes & Safety

### Auto-Disarm Features
For safety, the drone is automatically **disarmed** when:
- ❌ WiFi is disconnected (no active client)
- ❌ More than one client is connected (prevents interference)
- ⏱️ No input for > 2 seconds
- 🔧 Motors will stop; LED blinks to indicate disarmed state

### Troubleshooting

**Problem**: Motor doesn't respond to throttle
- Check ESC calibration completed
- Verify GPIO pin assignments match your wiring
- Check ESC power and signal connections

**Problem**: Drone drifts or won't stabilize
- Run IMU calibration routine
- Check for bent props or unbalanced motors
- Adjust PID gains (start with higher Kp)
- Verify IMU is mounted level

**Problem**: WiFi disconnects frequently
- Change WiFi password (weak interference)
- Move controller closer to drone
- Check antenna connections on ESP32

**Problem**: Motors spin on initialization
- Ensure drone is stationary during setup
- ESC may not have received minimum throttle signal
- Try re-uploading firmware and resetting power

## 🎮 Mobile App Integration

You can use any HTML/app frontend to send control signals:

**Example JavaScript (Web App)**:
```javascript
async function sendControl(roll, pitch, throttle, armed) {
  const params = new URLSearchParams({
    r: roll,
    p: pitch,
    t: throttle,
    aux1: armed ? 2000 : 1000,  // Armed = 1500+, Disarmed = <1500
    aux3: 93,    // P gain
    aux4: 3,     // I gain
    aux5: 30     // D gain
  });
  
  const response = await fetch(`http://192.168.0.1/control?${params}`);
  console.log('Control sent:', response.status);
}

// Send control: roll=1500, pitch=1500, throttle=1200, armed
sendControl(1500, 1500, 1200, true);
```

## 📚 References

### External Libraries
- **MPU6050_light**: [GitHub](https://github.com/rfetick/MPU6050_light)
- **ESP32Servo**: [GitHub](https://github.com/madhephaestus/ESP32Servo)

### PID Theory
- [PID Controller Wikipedia](https://en.wikipedia.org/wiki/PID_controller)
- [Drone Stability Control Fundamentals](https://oscarliang.com/quadcopter-imu-explained/)

### Related Projects
- Crazyflie Quadcopter
- ArduPilot Flight Controller
- Betaflight Flight Controller

## 📜 License

MIT License - See LICENSE file for details

This project is published on Zenodo with DOI: [10.5281/zenodo.11381128](https://doi.org/10.5281/zenodo.11381128)

## 🤝 Contributing

Found a bug or want to improve this project?

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/your-feature`)
3. Commit changes (`git commit -am 'Add feature'`)
4. Push to branch (`git push origin feature/your-feature`)
5. Open a Pull Request

## 📝 Citation

If you use this project in your research or work, please cite:

```bibtex
@software{mounirdahmane_2024_11381129,
  author       = {MounirDahmane},
  title        = {{MounirDahmane/Quadcopter-design-using-PID-Controller: Init}},
  month        = may,
  year         = 2024,
  publisher    = {Zenodo},
  version      = {0.1.0},
  doi          = {10.5281/zenodo.11381129},
  url          = {https://doi.org/10.5281/zenodo.11381129}
}
```

## 👥 Author

**Mounir Dahmane** - [GitHub Profile](https://github.com/MounirDahmane)

---
