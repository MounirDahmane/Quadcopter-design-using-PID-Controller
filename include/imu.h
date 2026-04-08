#pragma once

/**
 * @file imu.h
 * @brief IMU (Inertial Measurement Unit) sensor interface
 * 
 * Handles all MPU6050 gyroscope and accelerometer operations:
 * - I2C communication setup
 * - Sensor initialization and calibration
 * - Angle reading (roll, pitch, yaw)
 * - Sensor fusion via complementary filter
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

// ============================================================================
// ANGLE SENSOR STATE
// ============================================================================

/// @brief Current pitch angle (X-axis rotation) in degrees
/// Roll = rotation around front-back axis
extern float angle_x;

/// @brief Current pitch angle (Y-axis rotation) in degrees
/// Pitch = rotation around left-right axis
extern float angle_y;

/// @brief Current yaw angle (Z-axis rotation) in degrees
/// Yaw = rotation around vertical axis
extern float angle_z;

/// @brief Calibration offset for roll axis (to remove sensor bias)
extern float X_error;

/// @brief Calibration offset for pitch axis
extern float Y_error;

/// @brief Calibration offset for yaw axis
extern float Z_error;

// ============================================================================
// IMU FUNCTIONS
// ============================================================================

/**
 * @brief Initialize the MPU6050 IMU sensor
 * 
 * Setup sequence:
 * 1. Initialize I2C communication (Wire) on pins 21(SDA), 22(SCL)
 * 2. Connect to MPU6050 and verify communication
 * 3. Enable digital low-pass filter (DLPF) to smooth sensor readings
 * 4. Calibrate gyro and accelerometer offsets
 * 5. Set gyro filter coefficient to reduce noise
 * 
 * @note The function halts if MPU6050 cannot be detected (status != 0)
 * @note Calibration takes ~3 seconds; ensure IMU is stationary
 * @note Executed once during setup()
 */
void IMU_init();

/**
 * @brief Read current orientation angles from MPU6050
 * 
 * Fetches the latest gyro/accelerometer data and computes Euler angles
 * using the MPU6050 library's complementary filter.
 * 
 * Updates global variables:
 * - angle_x (roll)
 * - angle_y (pitch)
 * - angle_z (yaw)
 * 
 * @note Must be called frequently (>100Hz) for accurate angle tracking
 * @note Call at start of main loop
 */
void get_angle();

/**
 * @brief Perform manual IMU calibration (advanced)
 * 
 * Takes 3000 samples and averages them to determine sensor offsets.
 * Use if calcOffsets() in IMU_init() is insufficient.
 * 
 * Computes and stores:
 * - X_error (roll bias)
 * - Y_error (pitch bias)
 * - Z_error (yaw bias)
 * 
 * @warning Takes ~30 seconds; drone must be stationary and level
 * @warning Recalibrate after:
 * - Replacing IMU
 * - Significant temperature changes
 * - Physical shocks/impacts
 * @note Uncomment call in setup() if needed
 */
void calibration();

/**
 * @brief Get access to the MPU6050 sensor instance
 * 
 * @return Reference to the global MPU6050 object
 * 
 * Used internally for raw sensor access if needed
 */
extern MPU6050 mpu;
