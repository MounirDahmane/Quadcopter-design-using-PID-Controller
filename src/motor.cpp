/**
 * @file motor.cpp
 * @brief Motor and ESC control implementation
 * 
 * Implements all motor control operations and safety features
 */

#include "motor.h"
#include "pid.h"

// ============================================================================
// MOTOR PIN CONFIGURATION
// ============================================================================

#define MOTOR_PIN1 15 ///< Front right motor
#define MOTOR_PIN2 23 ///< Back right motor
#define MOTOR_PIN3 4  ///< Front left motor
#define MOTOR_PIN4 16 ///< Back left motor

#define LED_BUILTIN 2 ///< ESP32 built-in LED for status indication

// ============================================================================
// MOTOR STATE
// ============================================================================

/// Servo objects controlling the ESCs for each motor
static Servo motor1; ///< Front right motor
static Servo motor2; ///< Back right motor
static Servo motor3; ///< Front left motor
static Servo motor4; ///< Back left motor

/// Current motor speeds (1000-2000 microseconds)
int esc1_speed = 1000; ///< Front right motor speed
int esc2_speed = 1000; ///< Back right motor speed
int esc3_speed = 1000; ///< Front left motor speed
int esc4_speed = 1000; ///< Back left motor speed

/// Throttle input from user
int throttle = 1000;

// ============================================================================
// UTILITY FUNCTIONS (from pid.h)
// ============================================================================

extern void minmax(int& var, int maxVal, int minVal);
extern void minmax(float& var, float maxVal, float minVal);

// ============================================================================
// ESC CALIBRATION
// ============================================================================

void esc_calibration()
{
  // Send maximum throttle to all ESCs for calibration
  motor1.writeMicroseconds((int)MAX_SIGNAL);
  motor2.writeMicroseconds((int)MAX_SIGNAL);
  motor3.writeMicroseconds((int)MAX_SIGNAL);
  motor4.writeMicroseconds((int)MAX_SIGNAL);
  
  delay(2000); // Hold max signal for 2 seconds

  // Send minimum throttle to complete calibration
  motor1.writeMicroseconds((int)MIN_SIGNAL);
  motor2.writeMicroseconds((int)MIN_SIGNAL);
  motor3.writeMicroseconds((int)MIN_SIGNAL);
  motor4.writeMicroseconds((int)MIN_SIGNAL);

  delay(20);
}

// ============================================================================
// MOTOR INITIALIZATION
// ============================================================================

void motor_init()
{
  // Attach each servo/motor to its control pin
  // Parameters: (GPIO pin, min microseconds, max microseconds)
  motor1.attach(MOTOR_PIN1, (int)MIN_SIGNAL, (int)MAX_SIGNAL);
  motor2.attach(MOTOR_PIN2, (int)MIN_SIGNAL, (int)MAX_SIGNAL);
  motor3.attach(MOTOR_PIN3, (int)MIN_SIGNAL, (int)MAX_SIGNAL);
  motor4.attach(MOTOR_PIN4, (int)MIN_SIGNAL, (int)MAX_SIGNAL);

  delay(20);

  // Initialize all motors to minimum speed (stopped)
  motor1.writeMicroseconds((int)MIN_SIGNAL);
  motor2.writeMicroseconds((int)MIN_SIGNAL);
  motor3.writeMicroseconds((int)MIN_SIGNAL);
  motor4.writeMicroseconds((int)MIN_SIGNAL);
  
  delay(20);
}

// ============================================================================
// MOTOR SPEED CONTROL
// ============================================================================

void write_motors()
{
  // Transmit current motor speeds to ESCs
  motor1.writeMicroseconds(esc1_speed);
  motor2.writeMicroseconds(esc2_speed);
  motor3.writeMicroseconds(esc3_speed);
  motor4.writeMicroseconds(esc4_speed);
}

// ============================================================================
// MOTOR MIXING
// ============================================================================

void motor_mix(int base_throttle, float roll_correction, 
               float pitch_correction, float yaw_correction)
{
  // Store throttle for reference
  throttle = base_throttle;
  
  // ==== MOTOR MIXING ====
  // Combine throttle + PID corrections to get individual motor speeds
  
  // Front right motor
  esc1_speed = base_throttle - roll_correction + pitch_correction + yaw_correction;
  
  // Back right motor
  esc2_speed = base_throttle + roll_correction + pitch_correction - yaw_correction;
  
  // Front left motor
  esc3_speed = base_throttle - roll_correction - pitch_correction - yaw_correction;
  
  // Back left motor
  esc4_speed = base_throttle + roll_correction - pitch_correction + yaw_correction;

  // Clamp motor speeds to safe operating range (1050-1800 microseconds)
  // 1050 = just above stall, 1800 = max power to prevent ESC overload
  minmax(esc1_speed, MAX_MOTOR_SPEED, MIN_MOTOR_SPEED);
  minmax(esc2_speed, MAX_MOTOR_SPEED, MIN_MOTOR_SPEED);
  minmax(esc3_speed, MAX_MOTOR_SPEED, MIN_MOTOR_SPEED);
  minmax(esc4_speed, MAX_MOTOR_SPEED, MIN_MOTOR_SPEED);
}

// ============================================================================
// MOTOR RESET
// ============================================================================

void motor_reset()
{
  // Set all ESCs to minimum speed (motors stop)
  esc1_speed = (int)MIN_SIGNAL;
  esc2_speed = (int)MIN_SIGNAL;
  esc3_speed = (int)MIN_SIGNAL;
  esc4_speed = (int)MIN_SIGNAL;

  // Send minimum speed to all motors
  write_motors();

  // Turn off LED indicator
  digitalWrite(LED_BUILTIN, LOW);
}
