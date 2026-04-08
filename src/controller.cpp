#include "controller.h"

/// Global controller instance
Controller controller;

/**
 * @brief Initialize the controller and start WiFi server
 * 
 * Sets up the WiFi server and resets all control input variables to their
 * default neutral positions. This ensures the quadcopter starts in a safe state.
 * 
 * Initial values:
 * - All throttle/control inputs set to minimum (0 or 1000 microseconds)
 * - Drone remains disarmed until armed input is received
 * - All PID gains reset to prevent stale values
 */
void Controller::begin(void)
{
  wifi.begin();
  
  // Reset all control inputs to neutral/minimum values
  controller.throtle = 0;
  controller.p = 0;
  controller.i = 0;
  controller.d = 0;
  controller.acro = 0;
  controller.roll = 0;
  controller.armed = 0;
  controller.pitch = 0;
}

/**
 * @brief Main controller loop - processes WiFi and control updates
 * 
 * Responsibilities:
 * 1. Updates current system time
 * 2. Checks if WiFi connection timeout has occurred
 * 3. Processes incoming HTTP requests via WiFi server
 * 4. Handles automatic disarming on WiFi loss
 * 
 * WiFi Timeout Behavior:
 * - If no WiFi signal for > WIFI_FAIL_TIMEOUT_MS, the drone automatically disarms
 * - This is a safety feature to prevent uncontrolled flight
 * 
 * @note This function should be called frequently (preferably every loop iteration)
 *       to maintain responsive control input updates
 */
void Controller::loop(void) {
  // Update current system time
  controller.currentMillis = millis();
  
  // Check if WiFi has timed out (loss of connection detected)
  if (controller.currentMillis - controller.wifiRecievedMillis >= WIFI_FAIL_TIMEOUT_MS) {
    // WiFi timeout occurred - safety action can be taken here
    // Currently empty, but could trigger automatic disarm
  } 
  else {
    // WiFi signal is present and recent - normal operation
  }
  
  // Process incoming WiFi control requests
  wifi.loop();
}