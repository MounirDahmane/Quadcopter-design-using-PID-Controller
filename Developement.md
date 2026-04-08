# Development & Advanced Tuning Guide

This document provides detailed information for developers and advanced users who want to understand, modify, or extend the flight controller code.

## 📐 Control System Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                         Mobile App/Browser                       │
└────────────────────────────┬────────────────────────────────────┘
                             │
                    HTTP GET Requests
                    (1000-2000 microseconds)
                             │
         ┌───────────────────▼───────────────────┐
         │   WiFi Server (192.168.0.1)          │
         │   - Parse HTTP /control requests     │
         │   - Update Controller object         │
         └───────────────────┬───────────────────┘
                             │
         ┌───────────────────▼───────────────────┐
         │   Controller Object                   │
         │   - throttle, roll, pitch, armed      │
         │   - PID gains (P, I, D)               │
         └───────────────────┬───────────────────┘
                             │
         ┌───────────────────▼───────────────────┐
         │   Main Flight Control Loop            │
         │   1. Read IMU sensors                 │
         │   2. Calculate PID corrections        │
         │   3. Mix to motor speeds              │
         │   4. Send PWM to ESCs                 │
         └───────────────────┬───────────────────┘
                             │
         ┌───────────────────▼───────────────────┐
         │   Motor/ESC Control (GPIO)            │
         │   - PWM frequency: ~50Hz              │
         │   - Pulse range: 1000-2000 µs         │
         └───────────────────┬───────────────────┘
                             │
         ┌───────────────────▼───────────────────┐
         │   4x Brushless Motors                 │
         │   - Spins proportional to pulse       │
         │   - Gyroscopic stabilization          │
         └───────────────────────────────────────┘
```

### Sensor Fusion Pipeline

```
MPU6050 (100Hz raw data)
    │
    ├──► Accelerometer (gravity vector)
    │        │
    │        ▼
    │    Low-pass filter
    │        │
    │        ▼
    │    Compute angles (asin, atan2)
    │
    ├──► Gyroscope (rotation rates)
    │        │
    │        ▼
    │    Integrate over dt
    │        │
    │        ▼
    │    Angle rate of change
    │
    └──► Complementary Filter
             │
             ▼
         Fused Angles (98% gyro, 2% accel)
             │
             ▼
         Angle error calculation
```

## 🎛️ PID Control Deep Dive

### Mathematical Model

The quadcopter stabilization uses 3 independent PID loops:

```
For each axis (Roll, Pitch, Yaw):

error = actual_angle - desired_angle

P_term = Kp * error

I_term = Ki * ∫(error * dt)  [from error_sum]

D_term = Kd * (d(error)/dt)  [from error_dt]

correction = P_term + I_term + D_term
```

### Motor Mixing Equations

The drone uses an X-frame configuration:

```
         Front
        1 ─── 2
        │     │
       3 ─── 4
         Back
```

Motor assignments:
- Motor 1: Front-right (GPIO 15)
- Motor 2: Back-right (GPIO 23)
- Motor 3: Front-left (GPIO 4)
- Motor 4: Back-left (GPIO 16)

**Motor Speed Formulas**:

```
esc1 = throttle - roll_correction + pitch_correction + yaw_correction
esc2 = throttle + roll_correction + pitch_correction - yaw_correction
esc3 = throttle - roll_correction - pitch_correction - yaw_correction
esc4 = throttle + roll_correction - pitch_correction + yaw_correction
```

**Why this works**:
- To roll right: Increase motors 2,4 (right side), decrease 1,3 (left side)
- To pitch forward: Increase motors 1,3 (front), decrease 2,4 (back)
- To yaw: Cross-couple to counter gyroscopic effects

### Tuning Parameters

#### Proportional Gain (Kp)

Controls **response speed** to angle errors.

```
Current Settings:
Kp[ROLL] = 0.93
Kp[PITCH] = 0.93
Kp[YAW] = 55.0
```

**Effect Analysis**:

| Kp Value | Behavior | Symptoms |
|----------|----------|----------|
| Too Low (<0.5) | Slow response | Sluggish, drifts with wind |
| Optimal (0.8-1.2) | Quick response | Stable hover, quick corrections |
| Too High (>1.5) | Aggressive | Oscillations, jitter |

**Tuning Steps**:
1. Set Ki, Kd to 0 initially
2. Slowly increase Kp until oscillations appear
3. Back off by ~20% from oscillation point
4. This is your optimal Kp

#### Integral Gain (Ki)

Eliminates **steady-state drift** over time.

```
Current Settings:
Ki[ROLL] = 0.03
Ki[PITCH] = 0.035
Ki[YAW] = 0.0
```

**Windup Prevention**:

The code prevents integral windup with three mechanisms:

1. **Error Clamping** (±400):
   ```cpp
   minmax(error_sum[ROLL], 400, -400);
   ```

2. **Stick Centering Reset**:
   ```cpp
   if(controller.roll < 1200)  // Center stick detected
       error_sum[ROLL] = 0;    // Reset integral
   ```

3. **Disarm Reset**:
   ```cpp
   if (!armed)
       for (int i = 0; i < 3; i++) error_sum[i] = 0;
   ```

**Tuning Steps**:
1. Fly with only Kp and Kd (Ki = 0)
2. Observe if drone drifts in any direction
3. Add small Ki (0.01) if drift observed
4. Gradually increase until drift stops
5. Don't exceed 0.05 (windup risk)

#### Derivative Gain (Kd)

Dampens **oscillations** and overshoot.

```
Current Settings:
Kd[ROLL] = 3.0
Kd[PITCH] = 0.0
Kd[YAW] = 0.0
```

**Function**:
- Reacts to *rate* of angle change
- Opposes rapid movements
- Prevents overshoot from P term

**Tuning Steps**:
1. If seeing oscillations, add small Kd (0.5)
2. Increase slowly until oscillations dampen
3. Too much causes jerky movements
4. Typical range: 0-5

### Dynamic Tuning (Runtime PID Adjustment)

The HTTP API supports dynamic PID tuning without re-uploading code:

```
GET /control?aux3=X&aux4=Y&aux5=Z

Where:
aux3 = Proportional gain (mapped to Kp)
aux4 = Integral gain (mapped to Ki)
aux5 = Derivative gain (mapped to Kd)
```

**Current Implementation Note**: 
The code receives these values but doesn't map them to the PID gains yet. To enable runtime tuning, modify the control loop:

```cpp
// Add to main loop after controller.loop():
if(controller.p > 0) {
    Kp[ROLL] = Kp[PITCH] = controller.p / 100.0;  // Scale to 0.0-2.0 range
}
if(controller.i > 0) {
    Ki[ROLL] = Ki[PITCH] = controller.i / 1000.0; // Scale to 0.0-0.1 range
}
if(controller.d > 0) {
    Kd[ROLL] = controller.d / 100.0;  // Scale to 0.0-2.0 range
}
```

## 🔧 Hardware Integration

### I2C Bus Configuration

MPU6050 communicates via I2C:
```
ESP32 Pin 21 (GPIO21) ──► MPU6050 SDA (serial data)
ESP32 Pin 22 (GPIO22) ──► MPU6050 SCL (serial clock)
                      
I2C Address: 0x68 (7-bit)
Frequency: 100-400 kHz (default 100 kHz)
```

### Motor Pin Assignments

```
ESP32 GPIO   ──► Motor Control Pin ──► Motor Type
GPIO 15      ──► MOTOR_PIN1         ──► Front-right
GPIO 23      ──► MOTOR_PIN2         ──► Back-right
GPIO 4       ──► MOTOR_PIN3         ──► Front-left
GPIO 16      ──► MOTOR_PIN4         ──► Back-left

ESP32 Servo Library PWM Frequency: ~50Hz (standard for ESCs)
Pulse Width Range: 1000-2000 microseconds (typical)
```

### External Hardware Interface

```
3.3V (regulated) ──┐
                   │
              [MPU6050]  ← Gyro + Accelerometer
                   │
GND ──────────────┴

3.3V ──┐
       │
   [Level Shifter] ← For 5V ESC signal compatibility
       │
GPIO ──┴──► ESC Signal Input (1-4)

Battery+ ──► ESC Power
Battery- ──► GND (common ground)
```

## 📊 Performance Metrics

### Loop Frequency

The main loop runs as fast as the ESP32 can execute. Typical performance:

```
Target: ≥100 Hz (10ms or less per iteration)
Typical: 200-300 Hz
Absolute Maximum: ~1000 Hz (hardware limit)

Breakdown of loop execution time:
- get_angle():        ~1-2ms (I2C read + calculations)
- calculate_errors(): ~0.5ms
- PID():              ~0.5ms
- brushless_speed():  ~2-3ms (4x PWM writes)
- WiFi processing:    ~1-5ms (varies with requests)
───────────────────
Total per loop:       ~5-13ms (≈75-200Hz effective)
```

### Memory Usage

```
Program Space (Flash):
- Compiled binary: ~150-200 KB
- Available: 1 MB (ESP32)
- Usage: ~15-20%

RAM Usage:
- Global variables: ~2-3 KB
- Stack (main loop): ~4-5 KB
- WiFi buffer: ~10-15 KB
- Total: ~20 KB used of 160 KB available
```

### Power Consumption

```
Idle (disarmed, WiFi active):
- ESP32: ~80 mA
- IMU: ~5 mA
- WiFi: ~100-150 mA
- Total: ~185-235 mA

Flying (all motors at 50% throttle):
- Total system: ~5-10 A (depends on motors)
- ESP32 alone: ~100 mA
```

## 🐛 Debugging & Diagnostics

### Serial Telemetry

Enable serial output at 115200 baud to see real-time flight data:

```
Format: throttle | angle_x | angle_y | angle_z | roll_pid | pitch_pid | yaw_pid | esc1 | esc2 | esc3 | esc4 | x_err | y_err | z_err
```

**Analysis Tips**:

1. **Check IMU Calibration**:
   - Disarmed, on level surface
   - `angle_x` and `angle_y` should be ≈0°
   - `angle_z` will drift slowly (expected)

2. **Monitor PID Outputs**:
   - `roll_pid`, `pitch_pid` should be ≈0 when hovering
   - Large PID values = large corrections = instability

3. **Check Motor Balancing**:
   - All `esc` values should be similar at same throttle
   - Large differences = unbalanced propellers or misaligned motors

4. **Verify Error Accumulation**:
   - `error_sum` grows if integral term is working
   - Should decrease when stick is centered (resets)

### Common Issues & Solutions

**Problem: High frequency oscillation (buzz)**
- Cause: Kp too high or Kd too low
- Solution: Reduce Kp by 20%, increase Kd
- Check: `roll_pid` alternating +/- rapidly in telemetry

**Problem: Slow roll/pitch response**
- Cause: Kp too low
- Solution: Increase Kp gradually (0.1 increments)
- Check: `errors[ROLL/PITCH]` large while `roll_pid` small

**Problem: Drifts in one direction**
- Cause: Unbalanced props, bent frame, or Ki too low
- Solution: 
  1. Check prop alignment and balance
  2. Verify frame is straight
  3. Increase Ki by 0.01
- Check: `error_sum[axis]` growing continuously

**Problem: WiFi drops during flight**
- Cause: Signal interference or distance
- Solution:
  1. Change WIFI_SSID/PASSWORD to reduce interference
  2. Move transmitter closer
  3. Check antenna connections
- Note: System auto-disarms on WiFi loss (safety)

## 🔐 Security Considerations

### Credentials Management

**Current Approach** (NOT RECOMMENDED for production):
- Hardcoded WiFi SSID/password in `secret.h`
- Stored in plain text

**Recommended Improvements**:

1. **Use ESP32 NVS (Non-Volatile Storage)**:
   ```cpp
   #include "nvs_flash.h"
   #include "nvs.h"
   
   // Initialize NVS
   nvs_flash_init();
   
   // Read credentials
   nvs_handle_t my_handle;
   nvs_open("storage", NVS_READWRITE, &my_handle);
   char ssid[32];
   nvs_get_str(my_handle, "ssid", ssid, sizeof(ssid));
   ```

2. **Web-Based Configuration**:
   - Serve HTML form to set credentials
   - Store in NVS after verification
   - Reboot to apply changes

3. **Environment Variables**:
   - Store in `.env` file (excluded from git)
   - Load at compile time with PlatformIO

### WiFi Security Best Practices

- ✅ Use strong password (12+ characters)
- ✅ Change default SSID to something unique
- ✅ Don't broadcast credentials in code comments
- ✅ Consider WPA2 encryption (current code uses WPA2)
- ⚠️ Soft AP mode (current) is vulnerable if password is weak

**Future Enhancement**: WPA3 encryption or certificate-based auth

## 📈 Performance Optimization

### Reducing Loop Time

Current bottlenecks:
1. **I2C Communication** (1-2ms)
   - Optimize: Increase I2C speed to 400 kHz
   ```cpp
   Wire.setClock(400000); // 400 kHz instead of 100 kHz
   ```

2. **WiFi Server** (1-5ms variable)
   - Optimize: Process WiFi in separate task/RTOS
   - Use lightweight HTTP library instead of WebServer

3. **Serial Output** (variable)
   - Optimize: Reduce telemetry frequency or move to separate task

### Memory Optimization

Current usage is acceptable, but for future expansion:
- Use `PROGMEM` for large constants
- Reduce buffer sizes if possible
- Use DMA for I2C if supported

## 🧪 Testing & Validation

### Unit Testing Approach

For PID calculations:
```cpp
void test_pid_math() {
    // Set known errors
    errors[ROLL] = 5.0;  // 5 degree error
    
    // Calculate
    float output = Kp[ROLL] * errors[ROLL];
    
    // Verify
    assert(output == 4.65);  // 0.93 * 5.0
}
```

### Integration Testing

1. **Hardware in the Loop (HIL)**:
   - Use flight simulator to provide IMU data
   - Verify motor commands respond correctly

2. **Bench Testing**:
   - Connect motors on bench (without propellers)
   - Verify motor speeds change with control inputs
   - Test PID responsiveness

3. **Flight Testing**:
   - Start in open area with calm weather
   - Gradually increase throttle
   - Test roll/pitch stability
   - Verify auto-disarm on WiFi loss

## 📚 Further Reading

- **PID Control Theory**: [Control Systems Academy](https://www.controlsystemsacademy.com/)
- **Drone Physics**: [DroneForums PID Tutorial](https://oscarliang.com/quadcopter-imu-explained/)
- **ESP32 Documentation**: [Espressif IoT Development Framework](https://docs.espressif.com/projects/esp-idf/en/latest/)
- **MPU6050 Sensor Fusion**: [Jeff Rowberg's MPU6050 Guide](https://www.i2cdevlib.com/devices/mpu6050)

---

**Questions?** Open an issue or discussion on the GitHub repository.