/**
 * @file imu.cpp
 * @brief IMU (Inertial Measurement Unit) sensor implementation
 * 
 * Implements all MPU6050 gyroscope and accelerometer operations
 */

#include "imu.h"

// ============================================================================
// GLOBAL IMU STATE
// ============================================================================

/// MPU6050 IMU sensor instance using I2C communication (Wire)
MPU6050 mpu(Wire);

/// Euler angles in degrees from MPU6050
/// These represent the drone's orientation relative to horizontal
float angle_x = 0; ///< Roll angle (rotation around front-back axis)
float angle_y = 0; ///< Pitch angle (rotation around left-right axis)
float angle_z = 0; ///< Yaw angle (rotation around vertical axis)

/// Calibration offsets to remove gyro/accel bias
float X_error = 0; ///< Roll axis offset
float Y_error = 0; ///< Pitch axis offset
float Z_error = 0; ///< Yaw axis offset

// ============================================================================
// IMU INITIALIZATION
// ============================================================================

void IMU_init()
{
  // Initialize I2C bus for MPU6050 communication
  Wire.begin();
  
  // Connect to MPU6050 and check status
  byte status = mpu.begin(1, 2); // Arguments: SDA pin 1, SCL pin 2 (default)
  // mpu.upsideDownMounting = true; // Uncomment if MPU6050 is inverted
  
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  
  // Halt if MPU6050 cannot be detected
  while (status != 0) {} 
  delay(100);

  // Configure digital low-pass filter (DLPF) to reduce noise
  // DLPF reduces gyro/accel noise at the cost of slight latency
  Wire.beginTransmission(0x68); // MPU6050 I2C address
  Wire.write(0x1A);             // Register 26: DLPF config
  Wire.write(0x01);             // DLPF bandwidth ~184Hz
  Wire.endTransmission();

  // Auto-calibrate gyro and accelerometer offsets
  // Must be stationary for accurate calibration
  mpu.calcOffsets();
  
  // Set gyro filter coefficient for complementary filter fusion
  // 0.98 = trust gyro 98%, accel 2% (good for short-term stability)
  mpu.setFilterGyroCoef(0.98);
}

// ============================================================================
// SENSOR READING
// ============================================================================

void get_angle()
{
  // Update sensor readings and compute angles
  mpu.update();
  
  // Read current roll (X), pitch (Y), yaw (Z) angles
  // Optional: subtract calibration offsets (X_error, Y_error, Z_error)
  angle_x = mpu.getAngleX(); ///< Roll angle in degrees
  angle_y = mpu.getAngleY(); ///< Pitch angle in degrees
  angle_z = mpu.getAngleZ(); ///< Yaw angle in degrees
}

// ============================================================================
// ADVANCED CALIBRATION
// ============================================================================

void calibration()
{ 
  X_error = 0;  Y_error = 0;  Z_error = 0;
  
  // Collect 3000 samples
  for(int i = 0; i< 3000; i++)
  {
    float X = mpu.getAngleX();
    float Y = mpu.getAngleY();
    float Z = mpu.getAngleZ();

    X_error += X;
    Y_error += Y;
    Z_error += Z;
  }
  
  // Compute average offset for each axis
  X_error /= 3000;
  Y_error /= 3000;
  Z_error /= 3000;
  
  Serial.print("Calibration complete. Offsets: ");
  Serial.print(X_error); Serial.print(" ");
  Serial.print(Y_error); Serial.print(" ");
  Serial.println(Z_error);
}
