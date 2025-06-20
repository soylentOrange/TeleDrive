// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <Preferences.h>
#include <thingy.h>

#include <string>

#define TAG "ESPNetwork"

void ESPNetwork::begin(Scheduler* scheduler) {
  LOGD(TAG, "Schedule ESPConnect...");
  // stop possibly running espConnect first
  if (_espConnect.getState() != Mycila::ESPConnect::State::NETWORK_DISABLED)
    _espConnect.end();

  // load ESPConnect configuration
  Mycila::ESPConnect::Config espConnectConfig;
  _espConnect.loadConfiguration(espConnectConfig);

  // reuse a potentially set hostname from main app, or set a default one
  if (!espConnectConfig.hostname.length()) {
    espConnectConfig.hostname = APP_NAME;
  }

  // If the passed config has a SSID that's fine.
  // If the passed config is empty or is to be in AP mode, we'll stop rigth here and restart in Safeboot-Mode
  if (espConnectConfig.apMode || !espConnectConfig.wifiSSID.length()) {
    while (true) {
      LOGW(TAG, "No valid WiFi-configuration found! Restarting in SafeBoot-mode...");
      if (Mycila::System::restartFactory("safeboot", 1000)) {
        LOGW(TAG, "Restarting in SafeBoot-mode...");
        led.setMode(LED::LEDMode::WAITING_CAPTIVE);
      } else {
        LOGE(TAG, "SafeBoot-partition not found");
        Mycila::System::restart(1000);
        led.setMode(LED::LEDMode::ERROR);
      }
      delay(1500);
    }
  } else {
    LOGI(TAG, "Trying to connect to saved WiFi (%s) in the background...", espConnectConfig.wifiSSID.c_str());
  }

  // configure and begin espConnect
  _espConnect.setAutoRestart(true);
  _espConnect.setBlocking(false);
  _espConnect.setConnectTimeout(ESPCONNECT_TIMEOUT_CONNECT);
  _espConnect.begin(espConnectConfig.hostname.c_str(), "", espConnectConfig);

  // Task handling
  _scheduler = scheduler;
  _espConnectTask = new Task(TASK_IMMEDIATE, TASK_FOREVER, [&] { _espConnectCallback(); }, _scheduler, false, NULL, NULL, true);
  _espConnectTask->enable();

  LOGD(TAG, "ESPConnect is scheduled for start...");
}

void ESPNetwork::end() {
  LOGD(TAG, "Stopping ESPConnect...");
  _espConnectTask->disable();
  _espConnect.end();
  LOGD(TAG, "...done!");
}

Mycila::ESPConnect* ESPNetwork::getESPConnect() {
  return &_espConnect;
}

void ESPNetwork::clearConfiguration() {
  _espConnect.clearConfiguration();
}

// Loop espConnect
void ESPNetwork::_espConnectCallback() {
  _espConnect.loop();
}
