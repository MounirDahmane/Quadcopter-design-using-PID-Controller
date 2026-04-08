#ifndef Secret_h
#define Secret_h

/**
 * @file secret.h
 * @brief WiFi credentials and sensitive configuration
 * 
 * SECURITY WARNING: 
 * This file contains WiFi credentials in plaintext. In production:
 * - Store credentials in secure storage or NVS (Non-Volatile Storage)
 * - Consider using strong passwords
 * - Change default credentials from factory settings
 * - This file should be added to .gitignore to prevent accidental commits
 * 
 * Current Configuration:
 * - AP Name (SSID): ESP32 (creates a WiFi network named "ESP32")
 * - Password: 12345678 (default, should be changed)
 * - Network Type: Soft AP (the ESP32 acts as a WiFi access point)
 */

/// @brief WiFi SSID (network name) that the ESP32 will broadcast
/// @details The mobile app will connect to this network to control the drone
#define WIFI_SSID "ESP32"

/// @brief WiFi password for connection security
/// @warning Change this default password for any real deployment
#define WIFI_PASSWORD "12345678"

#endif