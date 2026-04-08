#include "controller.h"
#include "esp_http_server.h"

/// Global WiFi server instance
WifiServer wifi;

/**
 * @brief Initialize the WiFi soft access point and HTTP server
 * 
 * Setup sequence:
 * 1. Creates a static WebServer instance on port 80
 * 2. Configures WiFi soft AP (Access Point) with:
 *    - IP: 192.168.0.1 (access point IP)
 *    - Gateway: 192.168.0.1
 *    - Subnet: 255.255.255.0
 * 3. Starts WiFi with configured SSID and password from secret.h
 * 4. Registers HTTP request handlers
 * 5. Starts the HTTP server
 * 
 * After this call, the ESP32 will appear as a WiFi network that can be
 * connected to by mobile devices or computers.
 * 
 * Connection info for clients:
 * - Network name (SSID): "ESP32" (from WIFI_SSID in secret.h)
 * - Password: "12345678" (from WIFI_PASSWORD in secret.h)
 * - IP address: http://192.168.0.1
 */
void WifiServer::begin()
{
  // Create a static WebServer instance
  // Static ensures it persists for the lifetime of the program
  static WebServer server(WEBSERVER_PORT);
  this->server = &server;
  
  // Configure soft AP network settings
  // IP Address: 192.168.0.1
  // Gateway: 192.168.0.1
  // Subnet Mask: 255.255.255.0
  WiFi.softAPConfig(IPAddress(192, 168, 0, 1), IPAddress(192, 168, 0, 1), IPAddress(255, 255, 255, 0));
  
  // Start the soft access point with credentials from secret.h
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  
  // Register HTTP request handlers for available endpoints
  this->beginWebServer(this->server);
  
  // Start the HTTP server and begin listening for client connections
  this->server->begin();
}

/**
 * @brief Process pending HTTP client requests
 * 
 * This function must be called regularly (typically every loop iteration)
 * to handle incoming HTTP requests. It performs one pass of client handling:
 * - Checks for new client connections
 * - Reads and parses HTTP requests
 * - Routes to appropriate handler based on request path
 * - Sends HTTP responses
 * 
 * Non-blocking: This function returns quickly even if no clients are connected
 */
void WifiServer::loop()
{
  // Process one pending HTTP client request if available
  this->server->handleClient();
}

/**
 * @brief Register HTTP request handlers and setup routing
 * 
 * Maps HTTP request paths to their corresponding handler functions:
 * - GET / -> onRoot() - returns status/welcome page
 * - GET /control -> onControl() - receives flight control commands
 * 
 * @param server Pointer to the WebServer instance to configure
 */
void WifiServer::beginWebServer(WebServer * server)
{
  // Register handler for root path
  server->on("/", onRoot);
  
  // Register handler for control commands
  server->on(CONTROL_PATH, onControl);
}

/**
 * @brief HTTP handler for root path "/"
 * 
 * Called when a client accesses http://192.168.0.1/
 * Returns a simple HTML response for diagnostics and testing.
 * 
 * Response:
 * - HTTP Status: 200 OK
 * - Content-Type: text/html
 * - Body: "HEllo world" (note: typo in original)
 * 
 * Use case:
 * Can be accessed from browser to verify WiFi server is running
 */
void WifiServer::onRoot()
{
  wifi.server->send(200, "text/html", "HEllo world");
}

/**
 * @brief HTTP handler for flight control commands at "/control"
 * 
 * Processes incoming control inputs from the mobile application and
 * updates the global Controller object with new values.
 * 
 * Supported Query Parameters:
 * ===========================
 * 
 * Basic Flight Controls:
 * - r (roll):     Roll angle input (1000-2000 us) -> maps to controller.roll
 * - p (pitch):    Pitch angle input (1000-2000 us) -> maps to controller.pitch
 * - t (throttle): Motor speed (1000-2000 us) -> maps to controller.throtle
 * - y (yaw):      Yaw rotation input (1000-2000 us) -> maps to controller.yaw
 * 
 * Auxiliary/Configuration Channels:
 * - aux1: Armed state (1500+ arms) -> maps to controller.armed
 * - aux2: Acro mode toggle -> maps to controller.acro
 * - aux3: PID P gain -> maps to controller.p
 * - aux4: PID I gain -> maps to controller.i
 * - aux5: PID D gain -> maps to controller.d
 * - aux6-10: Reserved for future expansion
 * 
 * Example Request:
 * GET /control?r=1500&p=1500&t=1500&aux1=2000&aux3=93
 * 
 * Validation:
 * - Throttle value checking: 
 *   - If incommingValue < 1000 OR > 2000, set to minimum (1000)
 *   - This prevents ESC stalls and out-of-range motor commands
 * 
 * Response:
 * - HTTP Status: 200 OK
 * - Content-Type: text/plain
 * - Body: empty string
 * 
 * @note Each parameter is optional; missing parameters don't affect others
 * @note All values are integers representing microseconds (1000-2000 range)
 */
void WifiServer::onControl()
{
  // Buffer for parsing control values (14 channels * 2 bytes each)
  static uint8_t list[14*2];
  
  // Define all available control channel names
  // Maps to: roll, pitch, throttle, yaw, aux1-10
  const char* args[] = { "r", "p", "t", "y", "aux1", "aux2", "aux3", "aux4", "aux5", "aux6", "aux7", "aux8", "aux9", "aux10"};
  
  // Process throttle input
  if(wifi.server->hasArg("t")) {
    int incommingValue = wifi.server->arg("t").toInt();
    
    // Validate throttle is in acceptable range (1000-2000 microseconds)
    if(incommingValue < 1000 && incommingValue > 2000) {
      incommingValue = 1000; // Clamp to minimum if out of range
    }
    
    controller.throtle = incommingValue;
  }
  
  // Process PID P gain (aux3)
  if(wifi.server->hasArg("aux3")) {
    int incommingValue = wifi.server->arg("aux3").toInt();
    controller.p = incommingValue;
  }
  
  // Process PID I gain (aux4)
  if(wifi.server->hasArg("aux4")) {
    int incommingValue = wifi.server->arg("aux4").toInt();
    controller.i = incommingValue;
  }
  
  // Process PID D gain (aux5)
  if(wifi.server->hasArg("aux5")) {
    int incommingValue = wifi.server->arg("aux5").toInt();
    controller.d = incommingValue;
  }
  
  // Process acro mode toggle (aux2)
  if(wifi.server->hasArg("aux2")) {
    int incommingValue = wifi.server->arg("aux2").toInt();
    controller.acro = incommingValue;
  }
  
  // Process roll input
  if(wifi.server->hasArg("r")) {
    int incommingValue = wifi.server->arg("r").toInt();
    controller.roll = incommingValue;
  }
  
  // Process armed state (aux1)
  // Convention: 1500+ = armed, < 1500 = disarmed
  if(wifi.server->hasArg("aux1")) {
    int incommingValue = wifi.server->arg("aux1").toInt();
    controller.armed = incommingValue;
  }
  
  // Process pitch input
  if(wifi.server->hasArg("p")) {
    int incommingValue = wifi.server->arg("p").toInt();
    controller.pitch = incommingValue;
  }
  
  // Send empty response to indicate successful reception
  wifi.server->send(200, "text/plain", "");
}