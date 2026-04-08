/**
 * @file pid.cpp
 * @brief PID stabilization controller implementation
 * 
 * Implements the core flight stabilization algorithm
 */

#include "pid.h"

// ============================================================================
// PID GAIN CONSTANTS (TUNABLE)
// ============================================================================

/**
 * @brief Proportional gain (Kp) for each axis
 * 
 * Higher values = stronger correction for angle errors
 * - Roll & Pitch: ~0.93 (tuned for stable hover)
 * - Yaw: ~55.0 (much higher because yaw inertia is lower)
 * 
 * Tuning Guide:
 * - Too low: Sluggish response, drone drifts
 * - Too high: Oscillations, instability
 */
float Kp[3] = {0.93, 0.93, 55.0};

/**
 * @brief Integral gain (Ki) for each axis
 * 
 * Accumulates error over time to eliminate steady-state drift
 * - Roll & Pitch: ~0.03-0.035 (small value, prevents windup)
 * - Yaw: 0 (yaw typically doesn't need integral term)
 * 
 * Tuning Guide:
 * - Too low: Cannot eliminate drift
 * - Too high: Windup effect, oscillations after disturbance
 */
float Ki[3] = {0.03, 0.035, 0.0};

/**
 * @brief Derivative gain (Kd) for each axis
 * 
 * Dampens oscillations by reacting to rate of change
 * - Roll: 3.0 (provides damping)
 * - Pitch: 0.0 (not needed for pitch stability)
 * - Yaw: 0.0 (yaw control doesn't require derivative)
 * 
 * Tuning Guide:
 * - Too low: Oscillations, bouncy response
 * - Too high: Jerky movements, sensor noise amplification
 */
float Kd[3] = {3.0, 0.0, 0.0};

// ============================================================================
// PID STATE VARIABLES
// ============================================================================

/// Current angle error for each axis: (actual - desired)
float errors[3] = {0, 0, 0};

/// Previous error from last iteration (for derivative calculation)
float prev_error[3] = {0, 0, 0};

/// Accumulated error sum over time (for integral calculation)
float error_sum[3] = {0, 0, 0};

/// Rate of error change (derivative term)
float error_dt[3] = {0, 0, 0};

// ============================================================================
// DESIRED vs ACTUAL STATE
// ============================================================================

/// Desired angles set by user input (after mapping and smoothing)
float roll_desired = 0;   ///< User-commanded roll angle
float pitch_desired = 0;  ///< User-commanded pitch angle
float yaw_desired = 0;    ///< User-commanded yaw angle

/// PID output values (used to correct towards desired angles)
float roll_pid = 0;   ///< PID correction for roll
float pitch_pid = 0;  ///< PID correction for pitch
float yaw_pid = 0;    ///< PID correction for yaw

/// Time tracking for PID control loop
unsigned long prev_time = 0;
float dt = 0.004; ///< Loop time in seconds (assumed 4ms = 250Hz)

/// Exponential smoothing factor for desired angle (0.15 = smooth, less jerky)
const float alpha = 0.15;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void minmax(float& var, float maxVal, float minVal)
{
  if (var > maxVal)  var = maxVal;
  if (var < minVal)  var = minVal;
}

void minmax(int& var, int maxVal, int minVal)
{
  if (var > maxVal)  var = maxVal;
  if (var < minVal)  var = minVal;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ============================================================================
// PID ALGORITHM IMPLEMENTATION
// ============================================================================

void calculate_errors(float angle_x, float angle_y, float angle_z,
                     bool armed, int stick_roll, int stick_pitch)
{
  // Calculate current angle errors (actual - desired)
  // Positive error means drone is pitched/rolled more than desired
  errors[ROLL]  = angle_x - roll_desired;
  errors[PITCH] = angle_y - pitch_desired;
  errors[YAW]   = angle_z - yaw_desired;

  // Calculate loop time since last iteration
  unsigned long now = millis();
  dt = (now - prev_time) / 1000.0; // Convert ms to seconds
  prev_time = now;

  // ==== INTEGRAL TERM (I) ====
  // Accumulate error over time to eliminate steady-state bias
  // If error persists for 1 second, integral grows to remove it
  if (armed) {
    error_sum[ROLL]  += errors[ROLL]  * dt;
    error_sum[PITCH] += errors[PITCH] * dt;
    error_sum[YAW]   += errors[YAW]   * dt;
  }

  // Prevent integral windup: clamp accumulated error
  // Windup occurs when error integrates to extreme values
  // Solution: limit integral to reasonable bounds (±400)
  minmax(error_sum[ROLL], 400, -400);
  minmax(error_sum[PITCH], 400, -400);
  minmax(error_sum[YAW], 400, -400);

  // Additional windup prevention: reset integral if stick is centered
  // Assuming 1200us is the "neutral" position for the stick
  if(stick_roll < 1200)
    error_sum[ROLL] = 0;
  if(stick_pitch < 1200)
    error_sum[PITCH] = 0;

  // ==== DERIVATIVE TERM (D) ====
  // Calculate rate of error change for damping
  // Derivative opposes rapid angle changes (reduces overshoot)
  error_dt[ROLL]  = (errors[ROLL]  - prev_error[ROLL])  / dt;
  error_dt[PITCH] = (errors[PITCH] - prev_error[PITCH]) / dt;
  error_dt[YAW]   = (errors[YAW]   - prev_error[YAW])   / dt;

  // Save current errors for next iteration's derivative calculation
  prev_error[ROLL]  = errors[ROLL];
  prev_error[PITCH] = errors[PITCH];
  prev_error[YAW]   = errors[YAW];
}

void compute_pid()
{
  // ==== COMPUTE PID OUTPUTS ====
  // Each axis gets its correction: Kp*e + Ki*∫e + Kd*de/dt
  
  roll_pid  = (Kp[ROLL]  * errors[ROLL])  + (Ki[ROLL]  * error_sum[ROLL])  + (Kd[ROLL]  * error_dt[ROLL]);
  pitch_pid = (Kp[PITCH] * errors[PITCH]) + (Ki[PITCH] * error_sum[PITCH]) + (Kd[PITCH] * error_dt[PITCH]);
  yaw_pid   = (Kp[YAW]   * errors[YAW])   + (Ki[YAW]   * error_sum[YAW])   + (Kd[YAW]   * error_dt[YAW]);

  // Clamp PID outputs to prevent extreme corrections
  minmax(roll_pid,  400, -400);
  minmax(pitch_pid, 400, -400);
  minmax(yaw_pid,   400, -400);
}

void pid_reset()
{
  // Clear all PID error history to prevent jerky re-arming
  for(int i=0; i<3; i++)
  {
    prev_error[i] = 0;   // No previous error
    error_dt[i] = 0;     // No error rate
    errors[i] = 0;       // No current error
    error_sum[i] = 0;    // No accumulated error (prevents windup)
  }
  
  // Clear computed PID outputs
  roll_pid = 0;
  pitch_pid = 0;
  yaw_pid = 0;
  
  // Clear desired angles to prevent drift on re-arm
  roll_desired = 0;
  pitch_desired = 0;
  yaw_desired = 0;
}
