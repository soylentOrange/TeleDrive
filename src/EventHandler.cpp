// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <thingy.h>
#define TAG "EventHandler"

void EventHandler::begin(Scheduler* scheduler) {
  _networkState = _espNetwork->getESPConnect()->getState();

  // Task handling
  _scheduler = scheduler;
  _srConnected.setWaiting();

  // Register Callback to espConnect
  _espNetwork->getESPConnect()->listen([&](__unused Mycila::ESPConnect::State previous, Mycila::ESPConnect::State state) {
    _networkStateCallback(state);
  });

  LOGD(TAG, "Registered EventHandler to ESPConnect...");
}

void EventHandler::end() {
  LOGW(TAG, "Disabling EventHandler...");
  _espNetwork->getESPConnect()->listen(nullptr);
  _networkState = Mycila::ESPConnect::State::NETWORK_DISABLED;
  _srConnected.setWaiting();
}

Mycila::ESPConnect::State EventHandler::getNetworkState() {
  return _networkState;
}

// Handle events from ESPConnect
void EventHandler::_networkStateCallback(Mycila::ESPConnect::State state) {
  _networkState = state;

  switch (state) {
    case Mycila::ESPConnect::State::NETWORK_CONNECTED:
      LOGI(TAG, "--> Connected to network...");
      LOGI(TAG, "IPAddress: %s", _espNetwork->getESPConnect()->getIPAddress().toString().c_str());
      led.setMode(stepper.getMotorState_as_LEDMode());
      _srConnected.signalComplete();
      break;

    case Mycila::ESPConnect::State::NETWORK_TIMEOUT:
      LOGW(TAG, "--> Timeout connecting to network...");
      if (Mycila::System::restartFactory("safeboot", 1000)) {
        LOGW(TAG, "Restarting in SafeBoot mode...");
        led.setMode(LED::LEDMode::WAITING_CAPTIVE);
      } else {
        LOGE(TAG, "SafeBoot partition not found");
        Mycila::System::restart(1000);
        led.setMode(LED::LEDMode::ERROR);
      }
      _srConnected.setWaiting();
      break;

    case Mycila::ESPConnect::State::NETWORK_DISCONNECTED:
      LOGI(TAG, "--> Disconnected from network...");
      led.setMode(LED::LEDMode::WAITING_WIFI);
      _srConnected.setWaiting();
      break;

    // This must not happen
    case Mycila::ESPConnect::State::AP_STARTED:
      LOGE(TAG, "--> Created AP...");
      Mycila::System::restart(1000);
      led.setMode(LED::LEDMode::ERROR);
      break;

    // This must not happen
    case Mycila::ESPConnect::State::PORTAL_STARTED:
      LOGE(TAG, "--> Started Captive Portal...");
      Mycila::System::restart(1000);
      led.setMode(LED::LEDMode::ERROR);
      break;

    // This must not happen
    case Mycila::ESPConnect::State::PORTAL_COMPLETE: {
      LOGE(TAG, "--> Captive Portal has ended, auto-save the configuration...");
      Mycila::System::restart(1000);
      led.setMode(LED::LEDMode::ERROR);
      break;
    }

    default:
      break;
  } /* switch (state) */
}
