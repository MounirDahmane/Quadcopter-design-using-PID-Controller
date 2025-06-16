#pragma once

#include "wifiServer.h"

class Controller {
  public:
    static const unsigned int WIFI_CHECK_INTERVAL_MS = 500;
    static const unsigned int WIFI_FAIL_TIMEOUT_MS = 2000;
    int throtle;
    int p;
    int i;
    int d;
    int acro;
    int roll;
    int armed;
    int pitch;
    static void begin(void);
    static void loop(void);

  private:
    unsigned long currentMillis = 0;
    unsigned long wifiRecievedMillis = 0;
    
};

extern Controller controller;