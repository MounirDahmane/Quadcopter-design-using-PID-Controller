#pragma once

#include <Arduino.h>
#include "secret.h"
#include <WiFi.h>
#include <WebServer.h>

class WifiServer {
  public:
    void begin();
    void loop();

  private:
    static const unsigned int WEBSERVER_PORT = 80;
    const char * CONTROL_PATH = "/control";
    const char * MDNS_DOMAIN_NAME = "quadcopter";
    WebServer * server;

    void beginWebServer(WebServer * server);
    static void onRoot();
    static void onControl();
    void (*controlCallback)(uint8_t list[14*2]);
};

extern WifiServer wifi;

