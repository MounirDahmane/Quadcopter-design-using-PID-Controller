#include <ESP32Servo.h>
#include <MPU6050_light.h>
#include "controller.h"

#include <Arduino.h>
#include "Wire.h"

double output_pitch, output_roll, output_yaw;
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
MPU6050 mpu(Wire);
float newthrotle;
double prev_error, prev_error_roll, prev_error_yaw;
double error_roll, error_pitch, error_yaw;
double propotional_pitch, propotional_roll, propotional_yaw;
double integral_pitch, integral_roll, integral_yaw;
double derivative_pitch, derivative_roll, derivative_yaw;
int motor1speed, motor2speed, motor3speed, motor4speed;
double Setpoints_pitch, Setpoints_roll, Setpoints_yaw;
float NewSetPointRoll, NewSetPointPitch;
// Specify the links and initial tuning parameters
double Kp_pitch = 0, Ki_pitch = 0, Kd_pitch = 0;
double Kp_roll = 0, Ki_roll = 0, Kd_roll = 0;
double kp_yaw = 3, ki_yaw = 0.005, kd_yaw = 0;
void setup()
{
  Serial.begin(115200);
  motor1.attach(23);
  motor2.attach(16);
  motor3.attach(4);
  motor4.attach(15);
  delay(1000);
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(2000);
  motor1speed = motor2speed = motor3speed = motor4speed = 1000;
  propotional_pitch = propotional_roll = propotional_yaw = 0;
  integral_pitch = integral_roll = integral_yaw = 0;
  derivative_pitch = derivative_roll = derivative_yaw = 0;
  error_roll = error_pitch = error_yaw = 0;
  prev_error = prev_error_roll = prev_error_yaw = 0;
  Wire.begin();
  byte status = mpu.begin(1, 0); // gyro range = +- 500 deg/s ==> 65.5 / ACC range = +- 2 g acc_lsb_to_g = 16384.0;

  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0)
  {
  } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);

  mpu.calcOffsets(); // calculate gyro and accelerometer offset 2000 time then divide it by 2000 ,you change it from the library
  Serial.println("Done!\n");
  mpu.setFilterGyroCoef(0.99);
  Wire.beginTransmission(0x68); // now we want to setup our gyro coefficient
  Wire.write(0x1A);             // the config address
  Wire.write(0x05);             // the config value
  Wire.endTransmission();       // set the kalman filer coeffecient
  controller.begin();           // start the communication with the phone
  Setpoints_pitch = 0;
  Setpoints_roll = 0;
  Setpoints_yaw = 0;
  delay(2000); // wait the 7s
}
void loop()
{
  mpu.update(); // update the angles
  int numClients = WiFi.softAPgetStationNum();

  // Check if any clients are connected

  /*
    Serial.print("X : ");
    Serial.print(mpu.getAngleX());
    Serial.print("\tY : ");
    Serial.print(mpu.getAngleY());
    Serial.print("\tZ : ");-
    Serial.println(mpu.getAngleZ());
    */

  float angleZ1 = mpu.getAngleZ();
  int total_angle_Z = angleZ1;
  while (total_angle_Z > +180)
    total_angle_Z -= 360;
  while (total_angle_Z < -180)
    total_angle_Z += 360;

  controller.loop(); // start reading data sent from phone
  if (controller.throtle < 1000)
  {
    controller.throtle = 1000;
  }
  if (controller.throtle > 1800)
  {
    controller.throtle = 1800;
  }

  // PID implementation
  error_pitch = 0 - mpu.getAngleX();
  propotional_pitch = error_pitch * 1.2;
  integral_pitch = error_pitch * 0.00 + integral_pitch;
  derivative_pitch = (error_pitch - prev_error) * 60;
  prev_error = error_pitch;
  // now we need to limitate integral value
  if (integral_pitch > 400)
  {
    integral_pitch = 400;
  }
  if (integral_pitch < -400)
  {
    integral_pitch = -400;
  }
  output_pitch = propotional_pitch + integral_pitch + derivative_pitch;

  if (output_pitch > 400)
  {
    output_pitch = 400;
  }
  if (output_pitch < -400)
  {
    output_pitch = -400;
  }

  error_roll = 0 - mpu.getAngleY();
  propotional_roll = error_roll * 1.2;
  integral_roll = error_roll * 0.00 + integral_roll;
  derivative_roll = (error_roll - prev_error_roll) * 60;
  prev_error_roll = error_roll;
  if (integral_roll > 400)
  {
    integral_roll = 400;
  }
  if (integral_roll < -400)
  {
    integral_roll = -400;
  }
  output_roll = propotional_roll + integral_roll + derivative_roll;

  if (output_roll > 400)
  {
    output_roll = 400;
  }
  if (output_roll < -400)
  {
    output_roll = -400;
  }
  error_yaw = 0 - total_angle_Z;
  propotional_yaw = error_yaw * 1.4;
  integral_yaw = error_yaw * 0 + integral_yaw;
  derivative_yaw = (error_yaw - prev_error_yaw) * kd_yaw;
  prev_error_yaw = error_yaw;
  output_yaw = propotional_yaw + integral_yaw;
  if (output_yaw > 400)
  {
    output_yaw = 400;
  }
  if (output_yaw < -400)
  {
    output_yaw = -400;
  }
  // Altitude pid
  motor1speed = controller.throtle - output_pitch - output_roll + output_yaw;//motor1.attach(23);
  motor2speed = controller.throtle - output_pitch + output_roll - output_yaw;//motor2.attach(16);
  motor3speed = controller.throtle + output_pitch + output_roll + output_yaw;//motor3.attach(4);
  motor4speed = controller.throtle + output_pitch - output_roll - output_yaw;//motor4.attach(15);
 // Serial.print("\t roll :"); Serial.print(controller.roll);Serial.print("\t pitch:"); Serial.println(controller.pitch);
  if (motor1speed < 1050)
  {
    motor1speed = 1010;
  }
  if (motor1speed > 1950)
  {
    motor1speed = 1950;
  }
  if (motor2speed < 1050)
  {
    motor2speed = 1010;
  }
  if (motor2speed > 1950)
  {
    motor2speed = 1950;
  }
  if (motor3speed < 1050)
  {
    motor3speed = 1010;
  }
  if (motor3speed > 1950)
  {
    motor3speed = 1950;
  }
  if (motor4speed < 1050)
  {
    motor4speed = 1010;
  }
  if (motor4speed > 1950)
  {
    motor4speed = 1950;
  }
  if (controller.armed == 1250 || controller.armed == 0 || numClients == 0)
  {
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);

    output_roll = 0;
    output_pitch = 0;
    output_yaw = 0;
    
  }
  else if (controller.armed == 1750 && numClients > 0)
  {
    motor1.writeMicroseconds(motor1speed);
    motor2.writeMicroseconds(motor2speed);
    motor3.writeMicroseconds(motor3speed);
    motor4.writeMicroseconds(motor4speed);
  }
}
