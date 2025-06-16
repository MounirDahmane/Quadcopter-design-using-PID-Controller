#include "controller.h"
#include "esp_http_server.h"

WifiServer wifi;

void WifiServer::begin()
{
  static WebServer server(WEBSERVER_PORT);
  this->server = &server;
  WiFi.softAPConfig(IPAddress(192, 168, 0, 1), IPAddress(192, 168, 0, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  this->beginWebServer(this->server);
  this->server->begin();
}

void WifiServer::loop()
{
  this->server->handleClient();
}

void WifiServer::beginWebServer(WebServer * server)
{
  server->on("/", onRoot);
  server->on(CONTROL_PATH, onControl);
}

void WifiServer::onRoot()
{
  wifi.server->send(200, "text/html", "HEllo world");
}

void WifiServer::onControl()
{
  static uint8_t list[14*2];
  // Corresponding to AERT1234 (roll, pitch, throttle, yaw, aux1,acro, p value, i value, d value, ...)
  const char* args[] = { "r", "p", "t", "y", "aux1", "aux2", "aux3", "aux4", "aux5", "aux6", "aux7", "aux8", "aux9", "aux10"};
  if(wifi.server->hasArg("t")) {
    int incommingValue = wifi.server->arg("t").toInt();
    if(incommingValue < 1000 && incommingValue > 2000) {
      incommingValue = 1000;
    }
    controller.throtle = incommingValue;
  }
  if(wifi.server->hasArg("aux3")) {
    int incommingValue = wifi.server->arg("aux3").toInt();
    controller.p = incommingValue;
  }
  if(wifi.server->hasArg("aux4")) {
    int incommingValue = wifi.server->arg("aux4").toInt();
    controller.i = incommingValue;
  }
  if(wifi.server->hasArg("aux5")) {
    int incommingValue = wifi.server->arg("aux5").toInt();
    controller.d = incommingValue;
  }
  if(wifi.server->hasArg("aux2")) {
    int incommingValue = wifi.server->arg("aux2").toInt();
    controller.acro = incommingValue;
  }
  if(wifi.server->hasArg("r")) {
    int incommingValue = wifi.server->arg("r").toInt();
    controller.roll = incommingValue;
  }
  if(wifi.server->hasArg("aux1")) {
    int incommingValue = wifi.server->arg("aux1").toInt();
  controller.armed = incommingValue;
  }
  if(wifi.server->hasArg("p")) {
    int incommingValue = wifi.server->arg("p").toInt();
    controller.pitch = incommingValue;
  }
  wifi.server->send(200, "text/plain", "");
}

