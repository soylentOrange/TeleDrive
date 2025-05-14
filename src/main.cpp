// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <Preferences.h>
#include <TaskScheduler.h>
#include <thingy.h>

#define TAG "Main"

// Create the WebServer, ESPConnect, Task-Scheduler,... here
AsyncWebServer webServer(HTTP_PORT);
Scheduler scheduler;
ESPNetwork espNetwork(webServer);
EventHandler eventHandler(espNetwork);
WebServerAPI webServerAPI(webServer);
WebSite webSite(webServer);
LED led;
Stepper stepper;
FastAccelStepperEngine engine = FastAccelStepperEngine();

// Allow logging for app via serial
#if defined(MYCILA_LOGGER_SUPPORT_APP)
Mycila::Logger* serialLogger = nullptr;
#endif

// Allow logging for app via webserial
#if defined(MYCILA_WEBSERIAL_SUPPORT_APP)
WebSerial webSerial;
Mycila::Logger* webLogger = nullptr;
#endif

void setup() {
#ifdef MYCILA_LOGGER_SUPPORT_APP
  // Start Serial or USB-CDC
  #if !ARDUINO_USB_CDC_ON_BOOT
  Serial.begin(MONITOR_SPEED);
  // Only wait for serial interface to be set up when not using USB-CDC
  while (!Serial)
    continue;
  #else
  // USB-CDC doesn't need a baud rate
  Serial.begin();

  // Note: Enabling Debug via USB-CDC is handled via framework
  #endif

  serialLogger = new Mycila::Logger();
  serialLogger->forwardTo(&Serial);
  serialLogger->setLevel(ARDUHAL_LOG_LEVEL_DEBUG);
#endif

  // Add LED-Task to Scheduler
  led.begin(&scheduler);

  // Add ESPConnect-Task to Scheduler
  espNetwork.begin(&scheduler);

  // Add EventHandler to Scheduler
  // Will also spawn the WebServerAPI and WebSite (when ESPConnect says so...)
  eventHandler.begin(&scheduler);
}

void loop() {
  scheduler.execute();
}
