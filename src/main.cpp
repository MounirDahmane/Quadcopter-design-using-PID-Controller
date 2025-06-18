#include "controller.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include <ESP32Servo.h>

#define LED_BUILTIN 2 // multiple LEDs can be used for different info, i just used the built-in one with different ways
MPU6050 mpu(Wire);

float angle_x = 0, angle_y = 0, angle_z = 0;

#define MAX_SIGNAL 2000.0
#define MIN_SIGNAL 1000.0
#define Throttle   1000.0

#define MOTOR_PIN1 15 //front right
#define MOTOR_PIN2 23 //back right
#define MOTOR_PIN3 4  //front left
#define MOTOR_PIN4 16 //back left

Servo motor1; 
Servo motor2; 
Servo motor3; 
Servo motor4; 
                      // PID = e.Kp + ∫e.Ki + Δe.Kd

enum {ROLL, PITCH, YAW};
// PID coefficients: roll, pitch, yaw
float Kp[3] = {0.93, 0.93, 55.0};     
float Ki[3] = {0.03, 0.035, 0.0};    
float Kd[3] = {3.0, 0.0, 0.0};        

int   esc1_speed = 1000, esc2_speed = 1000, esc3_speed = 1000, esc4_speed = 1000;
float roll_desired = 0, pitch_desired = 0, yaw_desired = 0;
float errors[3] = {0, 0, 0}, prev_error[3] = {0, 0, 0}, error_sum[3] = {0, 0, 0}, error_dt[3] = {0, 0, 0};

float roll_pid = 0, pitch_pid = 0, yaw_pid = 0;
float X_error=0, Y_error=0, Z_error=0;

bool armed = false;

unsigned long prev_time = 0;
float dt = 0.004;

const float alpha = 0.15;

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
void RST()
{
  esc1_speed = 1000;
  esc2_speed = 1000;
  esc3_speed = 1000;
  esc4_speed = 1000;

  motor1.writeMicroseconds(esc1_speed);
  motor2.writeMicroseconds(esc2_speed);
  motor3.writeMicroseconds(esc3_speed);
  motor4.writeMicroseconds(esc4_speed);

  for(int i=0; i<3; i++)
  {
    prev_error[i] = 0;
    error_dt  [i] = 0;
    errors    [i] = 0;
    error_sum [i] = 0;
  }
  roll_pid = 0;
  pitch_pid = 0;
  yaw_pid = 0;
  angle_x = 0;
  angle_y = 0;
  angle_z = 0;
  
  //  to prevent "carryover" drift after disarming.
  roll_desired = 0;
  pitch_desired = 0;
  yaw_desired = 0;

  digitalWrite(LED_BUILTIN, LOW);
  mpu.update();
}
 

void IMU_init()
{
  Wire.begin();
  byte status = mpu.begin(1, 2);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {} // stop everything if could not connect to MPU6050 
  delay(100);

  // Enables DLPF
  Wire.beginTransmission(0x68); 
  Wire.write(0x1A);             
  Wire.write(0x01);             
  Wire.endTransmission();  

  mpu.calcOffsets(); // gyro and accelero offset calculation for calibration
  mpu.setFilterGyroCoef(0.98);
}
void get_angle()
{
  mpu.update();
  
  angle_x = mpu.getAngleX() ;//- X_error; // for calibration() if used
  angle_y = mpu.getAngleY() ;//- Y_error; // for calibration() if used
  angle_z = mpu.getAngleZ() ;//- Z_error; // for calibration() if used
}
void calibration()
{ 
  X_error = 0;  Y_error = 0;  Z_error = 0;
  for(int i = 0; i< 3000; i++)
  {
    float X = mpu.getAngleX(); // reading the high and low gyro data, by or op and shift op 
    float Y = mpu.getAngleY();
    float Z = mpu.getAngleZ();

    X_error += X ;
    Y_error += Y ;
    Z_error += Z ;
  }
  
  X_error /= 3000;
  Y_error /= 3000;
  Z_error /= 3000;
}

void calculate_errors()
{
    // Calculate current errors
  errors[ROLL]  = angle_x - roll_desired;
  errors[PITCH] = angle_y - pitch_desired;
  errors[YAW]   = angle_z - yaw_desired;

  unsigned long now = millis();
  dt = (now - prev_time) / 1000.0;
  prev_time = now;

    // Calculate sum of errors : Integral coefficients
  error_sum[ROLL]  += errors[ROLL]  * dt;
  error_sum[PITCH] += errors[PITCH] * dt;
  error_sum[YAW]   += errors[YAW]   * dt;

    // Keep values in acceptable range to prevent windup effect
  minmax(error_sum[ROLL], 400, -400);
  minmax(error_sum[PITCH], 400, -400);
  minmax(error_sum[YAW], 400, -400);

  if(controller.roll < 1200)
    error_sum[ROLL] = 0;
  if(controller.pitch < 1200)
    error_sum[PITCH] = 0; 
    // Calculate error delta : Derivative coefficients
  error_dt[ROLL]  = (errors[ROLL]  - prev_error[ROLL])  / dt;
  error_dt[PITCH] = (errors[PITCH] - prev_error[PITCH]) / dt;
  error_dt[YAW]   = (errors[YAW]   - prev_error[YAW])   / dt;
    // Save current error as previous_error for next time
  
  prev_error[PITCH] = errors[PITCH];
  prev_error[ROLL] = errors[ROLL];
  prev_error[YAW] = errors[YAW];

}
void PID() {
  if(armed)
  {
    
    roll_pid  = (Kp[ROLL]  * errors[ROLL])  + (Ki[ROLL]  * error_sum[ROLL])  + (Kd[ROLL]  * error_dt[ROLL]) ;
    pitch_pid = (Kp[PITCH] * errors[PITCH]) + (Ki[PITCH] * error_sum[PITCH]) + (Kd[PITCH] * error_dt[PITCH]);
    yaw_pid   = (Kp[YAW]   * errors[YAW])   + (Ki[YAW]   * error_sum[YAW])   + (Kd[YAW]   * error_dt[YAW])  ;

    minmax(roll_pid,  400, -400);
    minmax(pitch_pid, 400, -400);
    minmax(yaw_pid,   400, -400);

    esc1_speed = controller.throtle - roll_pid + pitch_pid +  yaw_pid;  //15 front right
    esc2_speed = controller.throtle + roll_pid + pitch_pid -  yaw_pid;  //23 back right
    esc3_speed = controller.throtle - roll_pid - pitch_pid -  yaw_pid;  //4  front left
    esc4_speed = controller.throtle + roll_pid - pitch_pid +  yaw_pid;  //16 back left

    // Prevent out-of-range-values
    minmax(esc1_speed, 1800, 1050);
    minmax(esc2_speed, 1800, 1050);
    minmax(esc3_speed, 1800, 1050);
    minmax(esc4_speed, 1800, 1050);
  }
  else
  {
   RST();
  }
}

void motor_init()
{
  motor1.attach(MOTOR_PIN1, MIN_SIGNAL, MAX_SIGNAL);
  motor2.attach(MOTOR_PIN2, MIN_SIGNAL, MAX_SIGNAL);
  motor3.attach(MOTOR_PIN3, MIN_SIGNAL, MAX_SIGNAL);
  motor4.attach(MOTOR_PIN4, MIN_SIGNAL, MAX_SIGNAL);

  delay(20);

  motor1.writeMicroseconds(MIN_SIGNAL); 
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL); 
  motor4.writeMicroseconds(MIN_SIGNAL);
  
  delay(20);
}
void brushless_speed()
{
  motor1.writeMicroseconds(esc1_speed);
  motor2.writeMicroseconds(esc2_speed);
  motor3.writeMicroseconds(esc3_speed);
  motor4.writeMicroseconds(esc4_speed);
}
void esc_calibration()
{
  motor1.writeMicroseconds(MAX_SIGNAL);
  motor2.writeMicroseconds(MAX_SIGNAL);
  motor3.writeMicroseconds(MAX_SIGNAL);
  motor4.writeMicroseconds(MAX_SIGNAL);
  
  delay(2000);

  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL);

  delay(20);
}
void INFO()
{
  Serial.print(controller.throtle);Serial.print("\t");
  Serial.print(angle_x);Serial.print("\t");Serial.print(angle_y);Serial.print("\t");Serial.print(angle_z);
  Serial.print("\t ROLL : ");Serial.print(roll_pid);Serial.print("\t PITCH : ");Serial.print(pitch_pid);Serial.print("\t YAW : ");Serial.print(yaw_pid);Serial.print("\t");
  Serial.print(esc1_speed);Serial.print("\t");Serial.print(esc2_speed);Serial.print("\t");Serial.print(esc3_speed);Serial.print("\t");Serial.print(esc4_speed);Serial.print("\t");
  Serial.print(X_error);Serial.print("\t");Serial.print(Y_error);Serial.print("\t");Serial.print(Z_error);Serial.print("\n");
 // Serial.print("armed");Serial.print(calibration.armed);Serial.print("\n");
  delay(100);
}
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() {

  Serial.begin(115200);
  IMU_init();
  //calibration(); // if calcOffsets() is not enough, uncomment this line to calibrate the IMU
  motor_init();

  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);  // Starting ESC calibration
  esc_calibration();

  digitalWrite(LED_BUILTIN, LOW);   // ESC calibration done
  delay(100);

  RST();
  
  controller.begin();
  digitalWrite(LED_BUILTIN, HIGH);  // Finished setup, waiting
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);   // Ready for connectio
}

void loop() {
  get_angle(); 

  // Check for single client connection
  if (WiFi.softAPgetStationNum() != 1) {
    if (armed) {
      armed = false;
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("Disarming: multiple or zero clients connected.");
    }
    RST(); 
    return;  
  }

  controller.loop();
  minmax(controller.throtle, MAX_SIGNAL, MIN_SIGNAL);

  if(controller.armed >= 1500) 
    armed = true; 
  else if(controller.armed < 1500) 
    armed = false;
  
  if (!armed)
    for (int i = 0; i < 3; i++) error_sum[i] = 0;

  float roll_target  = mapFloat(controller.roll, MIN_SIGNAL, MAX_SIGNAL, -10.0, 10.0);
  float pitch_target = mapFloat(controller.pitch, MIN_SIGNAL, MAX_SIGNAL, -10.0, 10.0);

  // Exponential smoothing
  roll_desired  = roll_desired  * (1.0 - alpha) + roll_target  * alpha;
  pitch_desired = pitch_desired * (1.0 - alpha) + pitch_target * alpha;


  // Prevent small values from causing drift: A deadband filter
  if (abs(roll_desired) < 1.0) roll_desired = 0;
  if (abs(pitch_desired) < 1.0) pitch_desired = 0;

  calculate_errors();
  PID();
  brushless_speed();

static unsigned long lastBlink = 0;
if (!armed) {// Blinking LED when disarmed
  if (millis() - lastBlink > 500) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    lastBlink = millis();
  }
} 
else
  digitalWrite(LED_BUILTIN, HIGH); // solid when armed
}
