#include "controller.h"

Controller controller;

void Controller::begin(void)
{
  wifi.begin();
  controller.throtle = 0;
  controller.p = 0;
  controller.i = 0;
  controller.d = 0;
  controller.acro=0;
  controller.roll=0;
  controller.armed=0;
  controller.pitch=0;
}

void Controller::loop(void) {
  controller.currentMillis = millis();
  if (controller.currentMillis - controller.wifiRecievedMillis >= WIFI_FAIL_TIMEOUT_MS) {} 
  else {}
  wifi.loop();
}
