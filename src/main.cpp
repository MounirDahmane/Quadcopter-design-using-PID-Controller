/**
 * @file main.cpp
 * @brief Flight controller main loop - orchestrates all subsystems
 * 
 * This refactored version separates concerns into dedicated modules:
 * - imu.h/cpp     : Sensor reading and calibration
 * - pid.h/cpp     : Stabilization algorithm
 * - motor.h/cpp   : ESC and motor control
 * - controller.h/cpp : WiFi communication (unchanged)
 * 
 * Main responsibility: coordinate subsystems and execute flight loop
 * 
 * Flight Control Flow:
 * 1. Read IMU sensors
 * 2. Check WiFi connection and process inputs
 * 3. Update desired angles from stick input
 * 4. Calculate PID errors
 * 5. Compute PID corrections
 * 6. Mix corrections into motor speeds
 * 7. Write speeds to ESCs
 * 8. Update status LED
 * 
 * @author [Your Name/Team]
 * @date 2024
 */

#include "controller.h"
#include "imu.h"
#include "pid.h"
#include "motor.h"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

#define LED_BUILTIN 2 ///< LED pin for status indication

// ============================================================================
// FLIGHT STATE
// ============================================================================

/// Overall armed state (true = motors running, false = motors idle)
static bool armed = false;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Update desired angles from stick input and apply smoothing
 * 
 * Maps RC stick values (1000-2000 microseconds) to angle commands (-10 to +10°)
 * and applies exponential smoothing to prevent jerky movements.
 * 
 * @param stick_roll Current roll stick position
 * @param stick_pitch Current pitch stick position
 * 
 * @note Applies deadband filter to reject stick noise
 */
static void update_desired_angles(int stick_roll, int stick_pitch)
{
  // Map stick inputs (1000-2000 µs) to angle commands (-10 to +10°)
  float roll_target  = mapFloat(stick_roll,  MIN_SIGNAL, MAX_SIGNAL, -10.0, 10.0);
  float pitch_target = mapFloat(stick_pitch, MIN_SIGNAL, MAX_SIGNAL, -10.0, 10.0);

  // Apply exponential smoothing to desired angles
  // Prevents jerky, rapid angle changes
  // alpha = 0.15 means new_desired = 0.85*old + 0.15*target
  roll_desired  = roll_desired  * (1.0 - alpha) + roll_target  * alpha;
  pitch_desired = pitch_desired * (1.0 - alpha) + pitch_target * alpha;

  // Apply deadband filter: small stick inputs are ignored
  // Prevents drift from stick noise/centering errors
  if (abs(roll_desired) < 1.0)  roll_desired = 0;
  if (abs(pitch_desired) < 1.0) pitch_desired = 0;
}

/**
 * @brief Update armed state from control input
 * 
 * Convention: aux1 >= 1500 = armed, < 1500 = disarmed
 * 
 * @param armed_input Input value from controller
 */
static void update_armed_state(int armed_input)
{
  if (armed_input >= 1500) {
    armed = true;
    digitalWrite(LED_BUILTIN, HIGH); // Solid LED when armed
  } 
  else {
    armed = false;
    // LED blink will be handled in main loop
  }
}

/**
 * @brief Update LED status indicator
 * 
 * Flashing LED = disarmed (safe)
 * Solid LED = armed (motors running)
 */
static void update_led_status()
{
  static unsigned long lastBlink = 0;
  
  if (!armed) {
    // Blink LED every 500ms when disarmed
    if (millis() - lastBlink > 500) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      lastBlink = millis();
    }
  } 
  else {
    // Solid LED when armed (set above in update_armed_state)
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

/**
 * @brief Full system reset/disarm
 * 
 * Called when disarming or on error condition.
 * Resets all subsystems to safe state.
 */
static void system_reset()
{
  armed = false;
  motor_reset();
  pid_reset();
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================

/**
 * @brief Print flight telemetry over serial for debugging
 * 
 * Outputs tab-separated telemetry data at 115200 baud
 * Update interval: ~100ms
 */
static void print_telemetry()
{
  Serial.print(throttle);          Serial.print("\t");
  Serial.print(angle_x);           Serial.print("\t");
  Serial.print(angle_y);           Serial.print("\t");
  Serial.print(angle_z);           Serial.print("\t");
  Serial.print("ROLL:");           Serial.print(roll_pid);  Serial.print("\t");
  Serial.print("PITCH:");          Serial.print(pitch_pid); Serial.print("\t");
  Serial.print("YAW:");            Serial.print(yaw_pid);   Serial.print("\t");
  Serial.print(esc1_speed);        Serial.print("\t");
  Serial.print(esc2_speed);        Serial.print("\t");
  Serial.print(esc3_speed);        Serial.print("\t");
  Serial.print(esc4_speed);        Serial.print("\t");
  Serial.print(X_error);           Serial.print("\t");
  Serial.print(Y_error);           Serial.print("\t");
  Serial.println(Z_error);
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================

/**
 * @brief Arduino setup function - called once at power-on
 * 
 * Initialization sequence:
 * 1. Start serial communication for debugging
 * 2. Initialize IMU sensor (MPU6050)
 * 3. Initialize motors/ESCs
 * 4. Set up LED status indicator
 * 5. Perform ESC calibration (requires propellers OFF)
 * 6. Reset flight state
 * 7. Initialize WiFi controller
 * 8. Signal ready status with LED
 * 
 * @warning Blocks during ESC calibration (~2.1 seconds with propellers OFF)
 * @warning Do NOT start propellers during setup
 */
void setup() 
{
  // Initialize serial communication for debugging output
  Serial.begin(115200);
  Serial.println("\n=== Flight Controller Startup ===");
  
  // Initialize IMU (MPU6050) sensor
  Serial.println("Initializing IMU...");
  IMU_init();
  Serial.println("✓ IMU initialized");
  
  // Optionally perform manual IMU calibration (uncomment if needed)
  // Takes ~30 seconds; only necessary if auto-calibration is insufficient
  // Serial.println("Calibrating IMU...");
  // calibration();
  // Serial.println("✓ IMU calibration complete");
  
  // Initialize motor control pins and set to minimum speed
  Serial.println("Initializing motors...");
  motor_init();
  Serial.println("✓ Motors initialized");

  // Set up LED indicator pin
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Perform ESC calibration
  Serial.println("Calibrating ESCs (propellers OFF!)...");
  digitalWrite(LED_BUILTIN, HIGH);  // LED on = calibration in progress
  esc_calibration();                // Performs 2-second calibration sequence
  digitalWrite(LED_BUILTIN, LOW);   // LED off = calibration done
  Serial.println("✓ ESC calibration complete");
  delay(100);

  // Reset all flight state variables
  system_reset();
  Serial.println("✓ System reset");
  
  // Initialize WiFi controller and HTTP server
  Serial.println("Starting WiFi...");
  controller.begin();
  Serial.println("✓ WiFi server started");
  
  // Signal ready status
  Serial.println("\n=== System Ready ===");
  Serial.println("SSID: ESP32");
  Serial.println("IP: 192.168.0.1");
  Serial.println("Endpoint: /control");
  digitalWrite(LED_BUILTIN, HIGH);  // LED on = ready, waiting for commands
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);   // LED off = ready to fly
  Serial.println("Awaiting control input...\n");
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================

/**
 * @brief Arduino main loop - called repeatedly at maximum speed
 * 
 * Flight Control Loop (target frequency: >100Hz):
 * 
 * 1. Read IMU orientation
 * 2. Check WiFi connection status (must have exactly 1 client)
 * 3. Process WiFi control inputs
 * 4. Update armed state from control input
 * 5. Update desired angles from stick input
 * 6. Calculate PID errors
 * 7. Compute PID corrections
 * 8. Mix corrections into motor speeds
 * 9. Write corrected motor speeds to ESCs
 * 10. Update status LED
 * 
 * Safety Features:
 * - Auto-disarms if WiFi disconnected
 * - Auto-disarms if more than one client connected
 * - Clamps all values to safe ranges
 * - Resets integral term when stick is centered
 * 
 * @note Runs as fast as the ESP32 can execute (~1kHz typical)
 * @note Actual PID loop frequency depends on execution time (~100-200Hz typical)
 */
void loop() 
{
  // ========== SENSOR INPUT ==========
  
  // Step 1: Read current orientation from IMU
  get_angle();

  // ========== SAFETY CHECKS ==========
  
  // Step 2: Check WiFi connection status
  // For safety: only allow operation with exactly 1 connected client
  if (WiFi.softAPgetStationNum() != 1) {
    // Multiple or zero clients connected: auto-disarm
    if (armed) {
      Serial.println("⚠ Disarming: WiFi client count != 1");
    }
    system_reset();
    return; // Exit early, don't process flight control
  }

  // ========== CONTROL INPUT ==========
  
  // Step 3: Update controller (process WiFi input and HTTP requests)
  controller.loop();
  
  // Clamp throttle to valid range
  minmax(throttle, (int)MAX_SIGNAL, (int)MIN_SIGNAL);

  // Step 4: Update armed state from control input
  update_armed_state(controller.armed);
  
  // Step 5: Reset integral errors when disarmed (prevents windup)
  if (!armed) {
    for (int i = 0; i < 3; i++) {
      error_sum[i] = 0;
    }
  }

  // ========== CONTROL SETPOINT ==========
  
  // Step 6: Update desired angles from stick input with smoothing
  update_desired_angles(controller.roll, controller.pitch);

  // ========== PID CONTROL ==========
  
  // Step 7: Calculate PID error terms
  calculate_errors(angle_x, angle_y, angle_z, armed, 
                   controller.roll, controller.pitch);
  
  // Step 8: Compute PID corrections
  if (armed) {
    compute_pid();
  } else {
    // Clear PID outputs when disarmed
    roll_pid = 0;
    pitch_pid = 0;
    yaw_pid = 0;
  }
  
  // ========== MOTOR CONTROL ==========
  
  // Step 9: Mix PID corrections into motor speeds
  motor_mix(controller.throtle, roll_pid, pitch_pid, yaw_pid);
  
  // Step 10: Write corrected motor speeds to ESCs
  write_motors();

  // ========== STATUS INDICATION ==========
  
  // Step 11: Update LED status indicator
  update_led_status();
  
  // ========== DEBUG OUTPUT ==========
  
  // Optional: Print telemetry every ~100ms
  // Uncomment to enable serial output for debugging
  // static unsigned long lastPrint = 0;
  // if (millis() - lastPrint > 100) {
  //   print_telemetry();
  //   lastPrint = millis();
  // }
}
