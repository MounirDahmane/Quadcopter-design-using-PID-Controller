# ESP32 Drone Flight Controller  
[![DOI](https://zenodo.org/badge/807372763.svg)](https://zenodo.org/doi/10.5281/zenodo.11381128)

A WiFi-enabled drone flight controller built with an ESP32, MPU6050 IMU, and brushless motor control using PID controller for stabilization.

## 🚁 Overview

This project implements a real-time quadcopter flight controller that:
- Uses an **MPU6050** IMU for angle measurements.
- Stabilizes the drone using a **PID controller**.
- Controls 4 brushless motors via **ESCs** and **ESP32Servo** library.
- Accepts user inputs over **WiFi** via an HTTP server from mobile application.

## 📦 Features
- Complementary filter for sensor fusion
- ESC calibration routine
- PID control for roll, pitch, and yaw
- Web control interface

## 🛠️ Hardware Requirements
- Check utils/Report.pdf

## 📌 Notes
- For safety, the drone is automatically **disarmed** when WiFi is disconnected or more than one client is connected.
- You can use any HTML or app frontend to send control signals to the server at `http://192.168.0.1/control`.

## 📜 License
MIT License
