#pragma once

#include <Arduino.h>
#include "secret.h"
#include <WiFi.h>
#include <WebServer.h>

/**
 * @class WifiServer
 * @brief HTTP web server for receiving quadcopter control commands over WiFi
 * 
 * This class manages:
 * - WiFi soft access point (AP) creation
 * - HTTP server setup and client handling
 * - Control input parsing from HTTP requests
 * - Transmission of control data to the flight controller
 * 
 * Communication Protocol:
 * The ESP32 creates a WiFi network that a mobile app can connect to.
 * Control commands are sent via HTTP GET requests with parameters:
 * - /control?r=1500&p=1500&t=1500&y=1500&aux1=1500&aux2=1500...
 * 
 * Where:
 * - r = Roll (1000-2000 microseconds)
 * - p = Pitch (1000-2000 microseconds)
 * - t = Throttle (1000-2000 microseconds)
 * - y = Yaw (1000-2000 microseconds)
 * - aux1-10 = Auxiliary channels for PID gains and modes
 * 
 * Thread Safety: 
 * Not thread-safe; designed for single-threaded Arduino environment
 */
class WifiServer {

  public:
    /**
     * @brief Initialize WiFi access point and start HTTP server
     * 
     * Sets up:
     * - Soft AP with configured SSID and password
     * - IP address: 192.168.0.1 (gateway and AP IP)
     * - Subnet mask: 255.255.255.0
     * - HTTP server on port 80
     * - HTTP request handlers for "/" and "/control" paths
     * 
     * After this function, the ESP32 will be visible as a WiFi network
     * and ready to accept control commands.
     */
    void begin();

    /**
     * @brief Process incoming HTTP requests
     * 
     * Must be called repeatedly in the main loop to handle client connections
     * and process incoming control requests. Each call handles one pending
     * client connection.
     * 
     * This function:
     * - Checks for new client connections
     * - Parses incoming HTTP requests
     * - Extracts control parameters
     * - Updates the Controller object with new values
     * - Sends HTTP response to client
     */
    void loop();

  private:
    /// HTTP server port number (standard web server port)
    static const unsigned int WEBSERVER_PORT = 80;
    
    /// HTTP endpoint for receiving control inputs
    const char * CONTROL_PATH = "/control";
    
    /// mDNS domain name (quadcopter.local for easier connection)
    const char * MDNS_DOMAIN_NAME = "quadcopter";
    
    /// Pointer to the WebServer instance (allocated in begin())
    WebServer * server;

    /**
     * @brief Initialize HTTP request handlers and routes
     * 
     * Sets up the routing table to map HTTP paths to callback functions:
     * - "/" (root) -> onRoot() - returns welcome message
     * - "/control" -> onControl() - processes flight control inputs
     * 
     * @param server Pointer to the WebServer instance to configure
     */
    void beginWebServer(WebServer * server);
    
    /**
     * @brief HTTP handler for root path "/"
     * 
     * Returns a simple HTML page when accessing the ESP32 IP in a browser.
     * Used primarily for testing and diagnostics.
     * 
     * Response: "Hello world" (plain text)
     * HTTP Status: 200 OK
     */
    static void onRoot();
    
    /**
     * @brief HTTP handler for control commands at "/control"
     * 
     * Parses the following query parameters and updates the Controller:
     * 
     * Roll/Pitch/Throttle/Yaw:
     * - r (roll):     maps to controller.roll
     * - p (pitch):    maps to controller.pitch
     * - t (throttle): maps to controller.throtle
     * - y (yaw):      not currently used in flight control
     * 
     * Auxiliary Channels:
     * - aux1: armed state flag
     * - aux2: acro mode toggle
     * - aux3: proportional (P) gain
     * - aux4: integral (I) gain
     * - aux5: derivative (D) gain
     * - aux6-10: reserved for future use
     * 
     * Validation:
     * - Throttle values are bounded between 1000-2000 microseconds
     * - Invalid values are clamped to minimum (1000)
     * 
     * Response: Empty 200 OK response
     */
    static void onControl();
    
    /// Callback function pointer for control updates (currently unused)
    void (*controlCallback)(uint8_t list[14*2]);
};

/// Global WiFi server instance accessible throughout the application
extern WifiServer wifi;