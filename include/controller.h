#pragma once

#include "wifiServer.h"

/**
 * @class Controller
 * @brief Main control system for the quadcopter flight controller
 * 
 * This class handles:
 * - WiFi connectivity management
 * - Reception of control inputs from the mobile application
 * - Storage of PID parameters (P, I, D gains)
 * - Storage of flight control inputs (throttle, roll, pitch, yaw, armed state)
 * 
 * The Controller acts as the bridge between WiFi input and the main flight control loop.
 */
class Controller {

  public:
    /// WiFi connectivity check interval in milliseconds
    static const unsigned int WIFI_CHECK_INTERVAL_MS = 500;
    
    /// Timeout duration before disarming if WiFi signal is lost
    static const unsigned int WIFI_FAIL_TIMEOUT_MS = 2000;

    // Control Input Variables
    /// Throttle input from controller (1000-2000 microseconds)
    int throtle;
    
    /// Proportional (P) gain for PID controller
    int p;
    
    /// Integral (I) gain for PID controller
    int i;
    
    /// Derivative (D) gain for PID controller
    int d;
    
    /// Acro/stabilization mode toggle (0 = stabilize, 1 = acro)
    int acro;
    
    /// Roll input from controller (1000-2000 microseconds, -10 to +10 degrees after mapping)
    int roll;
    
    /// Armed state flag (1500+ = armed, < 1500 = disarmed)
    int armed;
    
    /// Pitch input from controller (1000-2000 microseconds, -10 to +10 degrees after mapping)
    int pitch;

    /**
     * @brief Initialize the controller and WiFi system
     * 
     * Initializes all control input variables to zero and starts the WiFi server.
     * This function should be called once during the ESP32 setup phase.
     */
    static void begin(void);

    /**
     * @brief Main control loop - should be called repeatedly in the main loop
     * 
     * Performs the following tasks:
     * - Updates the WiFi timestamp
     * - Checks if WiFi connection is still active
     * - Processes incoming HTTP control requests
     * - Handles WiFi disconnection scenarios
     */
    static void loop(void);

  private:
    /// Current system time in milliseconds (from millis())
    unsigned long currentMillis = 0;
    
    /// Timestamp of last successful WiFi reception (used to detect disconnection)
    unsigned long wifiRecievedMillis = 0;
};

/// Global controller instance accessible throughout the application
extern Controller controller;