// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <Preferences.h>
#include <TaskScheduler.h>
#include <thingy.h>

#define TAG "MAIN"

// Create the WebServer, ESPConnect, Task-Scheduler,... here
AsyncWebServer webServer(HTTP_PORT);
Scheduler scheduler;
ESPNetwork espNetwork(webServer);
EventHandler eventHandler(espNetwork);
WebServerAPI webServerAPI(webServer);
WebSite webSite(webServer);
LED led;
Stepper stepper;

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

  // stepper_driver.setup(Serial1, 115200, TMC2209::SerialAddress::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);
  // commTest = new Task(3000, TASK_FOREVER, [&] {
  //   if (stepper_driver.isSetupAndCommunicating()) {
  //     LOGD(TAG, "Stepper driver is setup and communicating!");
  //     LOGD(TAG, "Stepper driver is%shardware disabled", stepper_driver.hardwareDisabled() ? " " : " not ");
  //     TMC2209::Settings settings = stepper_driver.getSettings();
  //     TMC2209::Status status = stepper_driver.getStatus();
  //     LOGD(TAG, "Stepper driver is%ssoftware disabled", settings.software_enabled ? " not " : " ");
  //     LOGD(TAG, "settings.microsteps_per_step: %d", settings.microsteps_per_step);
  //     LOGD(TAG, "settings.irun_percent: %d", settings.irun_percent);
  //     LOGD(TAG, "settings.ihold_percent: %d", settings.ihold_percent);
  //     LOGD(TAG, "settings.inverse_motor_direction_enabled: %s", settings.inverse_motor_direction_enabled ? "true" : "false");
  //     LOGD(TAG, "settings.stealth_chop_enabled: %s", settings.stealth_chop_enabled ? "true" : "false");
  //     switch (settings.standstill_mode) {
  //       case TMC2209::NORMAL:
  //         LOGD(TAG, "settings.standstill_mode: normal");
  //         break;
  //       case TMC2209::FREEWHEELING:
  //         LOGD(TAG, "settings.standstill_mode: freewheeling");
  //         break;
  //       case TMC2209::STRONG_BRAKING:
  //         LOGD(TAG, "settings.standstill_mode: strong_braking");
  //         break;
  //       case TMC2209::BRAKING:
  //         LOGD(TAG, "settings.standstill_mode: braking");
  //         break;
  //     }
  //     LOGD(TAG, "status.open_load_a: %s", status.open_load_a ? "true" : "false");
  //     LOGD(TAG, "status.open_load_b: %s", status.open_load_b ? "true" : "false");
  //     LOGD(TAG, "status.standstill: %s", status.standstill ? "true" : "false");
  //   } else if (stepper_driver.isCommunicatingButNotSetup())    {
  //     LOGD(TAG, "Stepper driver is communicating but not setup!");
  //     LOGD(TAG, "Running setup again...");
  //     stepper_driver.setup(Serial1, 115200, TMC2209::SerialAddress::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);
  //   } else {
  //     LOGD(TAG, "Stepper driver is not communicating!");
  //     LOGD(TAG, "Try turning driver power on to see what happens.");
  //   } }, &scheduler, false, NULL, NULL, true);
  // commTest->enable();
}

void loop() {
  scheduler.execute();
}
