#pragma once

/**
 * @file pid.h
 * @brief PID stabilization controller for quadcopter
 * 
 * Implements the core stabilization algorithm:
 * - Error calculation (desired vs actual angles)
 * - PID term computation (proportional, integral, derivative)
 * - Motor mixing (converting corrections to motor speeds)
 */

#include <Arduino.h>

// ============================================================================
// PID AXIS DEFINITIONS
// ============================================================================

/// PID axis indices
enum PID_Axis {
    ROLL = 0,   ///< Roll axis (rotation around front-back)
    PITCH = 1,  ///< Pitch axis (rotation around left-right)
    YAW = 2     ///< Yaw axis (rotation around vertical)
};

// ============================================================================
// PID GAINS (TUNABLE PARAMETERS)
// ============================================================================

/// @brief Proportional gain (Kp) for each axis
/// Higher values = stronger correction for angle errors
extern float Kp[3];

/// @brief Integral gain (Ki) for each axis
/// Accumulates error over time to eliminate steady-state drift
extern float Ki[3];

/// @brief Derivative gain (Kd) for each axis
/// Dampens oscillations by reacting to rate of change
extern float Kd[3];

// ============================================================================
// PID STATE VARIABLES
// ============================================================================

/// @brief Current angle error for each axis: (actual - desired)
extern float errors[3];

/// @brief Previous error from last iteration (for derivative calculation)
extern float prev_error[3];

/// @brief Accumulated error sum over time (for integral calculation)
extern float error_sum[3];

/// @brief Rate of error change (derivative term)
extern float error_dt[3];

/// @brief Desired angles set by user input (after mapping and smoothing)
extern float roll_desired;   ///< User-commanded roll angle
extern float pitch_desired;  ///< User-commanded pitch angle
extern float yaw_desired;    ///< User-commanded yaw angle

/// @brief PID output values (used to correct towards desired angles)
extern float roll_pid;   ///< PID correction for roll
extern float pitch_pid;  ///< PID correction for pitch
extern float yaw_pid;    ///< PID correction for yaw

/// @brief Time tracking for PID control loop
extern unsigned long prev_time;
extern float dt;

/// @brief Exponential smoothing factor for desired angle (0.15 = smooth, less jerky)
extern const float alpha;

// ============================================================================
// PID ALGORITHM FUNCTIONS
// ============================================================================

/**
 * @brief Calculate angle errors and PID terms for all three axes
 * 
 * This function implements the core stabilization algorithm:
 * 1. Compute error between desired and actual angles
 * 2. Calculate integral (error accumulation)
 * 3. Calculate derivative (error rate of change)
 * 4. Store for next iteration's derivative calculation
 * 
 * Error Integration (Windup Prevention):
 * - Only accumulates integral if armed and stick is not centered
 * - Integral is clamped to ±400 to prevent windup
 * 
 * @param angle_x Current roll angle (from IMU)
 * @param angle_y Current pitch angle (from IMU)
 * @param angle_z Current yaw angle (from IMU)
 * @param armed Whether the drone is currently armed
 * @param stick_roll Current roll stick position (for windup prevention)
 * @param stick_pitch Current pitch stick position (for windup prevention)
 * 
 * @note Should be called at constant frequency (preferably ≥100Hz)
 * @note Uses millis() for timing; ensure reliable timekeeping
 */
void calculate_errors(float angle_x, float angle_y, float angle_z, 
                     bool armed, int stick_roll, int stick_pitch);

/**
 * @brief Compute PID control outputs
 * 
 * Calculates the PID correction for each axis using:
 * correction = Kp*error + Ki*∫error + Kd*de/dt
 * 
 * Updates global variables:
 * - roll_pid
 * - pitch_pid
 * - yaw_pid
 * 
 * @note Call after calculate_errors()
 * @note Values are clamped to ±400 to prevent saturation
 */
void compute_pid();

/**
 * @brief Reset all PID state variables
 * 
 * Called when disarming or encountering error conditions.
 * Clears all accumulated errors and previous values to ensure
 * clean state when re-arming.
 * 
 * Resets:
 * - errors[3]
 * - prev_error[3]
 * - error_sum[3]
 * - error_dt[3]
 * - roll_pid, pitch_pid, yaw_pid
 * - roll_desired, pitch_desired, yaw_desired
 */
void pid_reset();

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Linear mapping from input range to output range
 * 
 * Used to convert RC receiver values (1000-2000us) to physical angles (-10 to +10°)
 * Formula: out = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
 * 
 * @param x Input value to map
 * @param in_min Input range minimum
 * @param in_max Input range maximum
 * @param out_min Output range minimum
 * @param out_max Output range maximum
 * @return Mapped value in the output range
 */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

/**
 * @brief Clamp a float value between min and max bounds
 * 
 * Prevents overflow and out-of-range values
 * 
 * @param var Reference to variable to clamp
 * @param maxVal Upper bound (inclusive)
 * @param minVal Lower bound (inclusive)
 */
void minmax(float& var, float maxVal, float minVal);

/**
 * @brief Clamp an integer value between min and max bounds
 * 
 * Integer version of minmax for motor speed control
 * 
 * @param var Reference to variable to clamp
 * @param maxVal Upper bound (inclusive)
 * @param minVal Lower bound (inclusive)
 */
void minmax(int& var, int maxVal, int minVal);
