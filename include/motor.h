#pragma once

/**
 * @file motor.h
 * @brief Motor and ESC (Electronic Speed Controller) management
 * 
 * Handles all motor control operations:
 * - ESC calibration
 * - Motor initialization
 * - Motor speed commands
 * - Motor mixing (converting PID outputs to motor speeds)
 * - Safety limits
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// ============================================================================
// MOTOR SPEED STATE
// ============================================================================

/// @brief Current motor speeds (1000-2000 microseconds)
/// These are computed by the PID control and sent to the ESCs
extern int esc1_speed; ///< Front right motor speed
extern int esc2_speed; ///< Back right motor speed
extern int esc3_speed; ///< Front left motor speed
extern int esc4_speed; ///< Back left motor speed

/// @brief Throttle input from user (1000-2000 microseconds)
extern int throttle;

// ============================================================================
// HARDWARE CONSTANTS
// ============================================================================

/// @brief ESC signal range in microseconds
static const float MAX_SIGNAL = 2000.0; ///< Maximum throttle
static const float MIN_SIGNAL = 1000.0; ///< Minimum throttle (motor stop)

/// @brief Safe operating range for motor speeds
static const int MAX_MOTOR_SPEED = 1800; ///< Max power to prevent ESC overload
static const int MIN_MOTOR_SPEED = 1050; ///< Just above stall speed

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

/**
 * @brief Initialize and calibrate ESCs (Electronic Speed Controllers)
 * 
 * ESC calibration sequence:
 * 1. Send maximum throttle signal for 2 seconds
 * 2. Send minimum throttle signal
 * 3. ESCs learn the min/max range and are now ready to use
 * 
 * @warning Must be performed with propellers OFF for safety
 * @warning Only needed when first programming or replacing ESCs
 * 
 * @note Uses LED to signal calibration progress:
 * - Before: LED ON (all motors receive max signal)
 * - After: LED OFF (calibration complete)
 */
void esc_calibration();

/**
 * @brief Attach servos to motor control pins and set to minimum speed
 * 
 * Initializes the ESP32Servo library and maps each motor to its GPIO pin.
 * Sets PWM range to standard ESC values (1000-2000 microseconds).
 * 
 * Motor Pin Assignments:
 * - Motor 1 (front-right):  GPIO 15
 * - Motor 2 (back-right):   GPIO 23
 * - Motor 3 (front-left):   GPIO 4
 * - Motor 4 (back-left):    GPIO 16
 * 
 * @note Each motor starts at minimum speed (1000 us) for safety
 * @note Must be called before any motor speed changes
 * @note Called once during setup()
 */
void motor_init();

/**
 * @brief Write current motor speeds to ESCs
 * 
 * Transmits PWM signals to each ESC based on esc1_speed through esc4_speed.
 * Should be called frequently (≥100Hz) to ensure responsive motor control.
 * 
 * @note Motor speeds should already be clamped to safe range
 * @note Call at end of each control loop iteration
 */
void write_motors();

/**
 * @brief Motor mixing: convert PID outputs to motor speeds
 * 
 * The quadcopter frame uses the following motor configuration:
 * ```
 *     Front
 *   1 ---- 2
 *   |      |
 *   3 ---- 4
 *     Back
 * ```
 * 
 * Motor Mixing Matrix (how control inputs affect motors):
 * - Roll correction: Increase right motors (1,2), decrease left motors (3,4)
 * - Pitch correction: Increase front motors (1,3), decrease back motors (2,4)
 * - Yaw correction: Cross-couple to prevent rotation
 * 
 * Motor Equations:
 * - esc1 = throttle - roll_pid + pitch_pid + yaw_pid   (front-right)
 * - esc2 = throttle + roll_pid + pitch_pid - yaw_pid   (back-right)
 * - esc3 = throttle - roll_pid - pitch_pid - yaw_pid   (front-left)
 * - esc4 = throttle + roll_pid - pitch_pid + yaw_pid   (back-left)
 * 
 * @param base_throttle Base throttle value (1000-2000)
 * @param roll_correction PID roll correction output
 * @param pitch_correction PID pitch correction output
 * @param yaw_correction PID yaw correction output
 * 
 * @note Clamps all outputs to safe range (1050-1800)
 * @note Call after PID computation, before write_motors()
 */
void motor_mix(int base_throttle, float roll_correction, 
               float pitch_correction, float yaw_correction);

/**
 * @brief Reset motor speeds and clear all motor state
 * 
 * Called when disarming or encountering error conditions.
 * Ensures a clean state and safe default (all motors stopped).
 * 
 * Sets:
 * - All motor speeds to minimum (1000 microseconds = stopped)
 * - Writes minimum to all ESCs
 * 
 * @note Prevents "kick" when re-arming after disarm
 * @note Ensures safe state if WiFi is lost
 */
void motor_reset();
