// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <thingy.h>

#include <string>

#define TAG "WebSite"

#ifndef WSL_MAX_WS_CLIENTS
  #define WSL_MAX_WS_CLIENTS DEFAULT_MAX_WS_CLIENTS
#endif

// gzipped website
extern const uint8_t thingy_html_start[] asm("_binary__pio_embed_website_html_gz_start");
extern const uint8_t thingy_html_end[] asm("_binary__pio_embed_website_html_gz_end");

// constants from build process
extern const char* __COMPILED_BUILD_BOARD__;

void WebSite::begin(Scheduler* scheduler) {
  // Task handling
  _scheduler = scheduler;
  _sr.setWaiting();

  // create and run a task for setting up the website
  Task* webSiteTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _webSiteCallback(); }, _scheduler, false, NULL, NULL, true);
  webSiteTask->enable();
  webSiteTask->waitFor(webServerAPI.getStatusRequest());
}

void WebSite::end() {
  _webEventCallback = nullptr;
  _sr.setWaiting();

  // end the cleanup task
  if (_wsCleanupTask != nullptr) {
    _wsCleanupTask->disable();
    _wsCleanupTask = nullptr;
  }

  // delete websock handler
  if (_ws != nullptr) {
    _webServer->removeHandler(_ws);
    _ws = nullptr;
  }

#ifdef MYCILA_WEBSERIAL_SUPPORT_APP
  webSerial.end();
  delete webLogger;
#endif
}

// Add Handlers to the webserver
void WebSite::_webSiteCallback() {
  LOGD(TAG, "Starting WebSite...");

  // Allow web-logging for app via WebSerial
#ifdef MYCILA_WEBSERIAL_SUPPORT_APP
  webSerial.begin(_webServer, "/weblog", _scheduler);
  webSerial.setBuffer(100);
  webLogger = new Mycila::Logger();
  webLogger->setLevel(ARDUHAL_LOG_LEVEL_INFO);
  webLogger->forwardTo(&webSerial);
#endif

  // create websock handler
  _ws = new AsyncWebSocket("/ws");

  _ws->onEvent([&](__unused AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, __unused size_t len) -> void {
    if (type == WS_EVT_CONNECT) {
      client->keepAlivePeriod(10);
      client->setCloseClientOnQueueFull(true);

      // send ID, motor_state, position, speed,...
      JsonDocument jsonMsg;
      jsonMsg["type"] = "initial_config";
      jsonMsg["id"] = client->id();
      jsonMsg["config"]["autoHome"] = stepper.getAutoHome();
      jsonMsg["homing_state"] = stepper.getHomingState().c_str();
      jsonMsg["motor_state"]["move_state"]["position"] = stepper.getCurrentPosition();
      jsonMsg["motor_state"]["move_state"]["speed"] = stepper.getCurrentSpeed();
      jsonMsg["motor_state"]["state"] = stepper.getMotorState_as_string().c_str();
      jsonMsg["motor_state"]["destination"]["position"] = stepper.getDestinationPosition();
      jsonMsg["motor_state"]["destination"]["speed"] = stepper.getDestinationSpeed();
      jsonMsg["motor_state"]["destination"]["acceleration"] = stepper.getDestinationAcceleration();
      AsyncWebSocketMessageBuffer* buffer = new AsyncWebSocketMessageBuffer(measureJson(jsonMsg));
      serializeJson(jsonMsg, buffer->get(), buffer->length());
      client->text(buffer);
    } else if (type == WS_EVT_DATA) {
      AwsFrameInfo* info = reinterpret_cast<AwsFrameInfo*>(arg);
      if (info->final && info->index == 0 && info->len == len) {
        if (info->opcode == WS_TEXT) {
          data[len] = 0;
        }
        // pong on client keep-alive message
        if (strcmp(reinterpret_cast<char*>(data), "ping") == 0) {
          // LOGD(TAG, "Client %d pinged us", client->id());
          client->text("pong");
        } else { // some message is received
          JsonDocument jsonRXMsg;
          DeserializationError error = deserializeJson(jsonRXMsg, reinterpret_cast<char*>(data));
          if (error == DeserializationError::Ok) {
            // ...pass command to stepper and let it decide...
            if (_webEventCallback != nullptr) {
              _webEventCallback(jsonRXMsg);
            } else {
              LOGE(TAG, "No event listener (_webEventCallback) available!");
            }
          }
        }
      }
    }
  });

  _webServer->addHandler(_ws);

  // serve driver info
  _webServer->on("/driver", HTTP_GET, [](AsyncWebServerRequest* request) {
              // LOGD(TAG, "Serve driver info");
              auto* response = request->beginResponse(200, "text/plain", "TMC2209");
              request->send(response); })
    .setFilter([](__unused AsyncWebServerRequest* request) { return eventHandler.getNetworkState() != Mycila::ESPConnect::State::PORTAL_STARTED; });

  // serve boardname info
  _webServer->on("/boardname", HTTP_GET, [](AsyncWebServerRequest* request) {
              // LOGD(TAG, "Serve boardname");
              auto* response = request->beginResponse(200, "text/plain", __COMPILED_BUILD_BOARD__);
              request->send(response); })
    .setFilter([](__unused AsyncWebServerRequest* request) { return eventHandler.getNetworkState() != Mycila::ESPConnect::State::PORTAL_STARTED; });

  // serve app-version info
  _webServer->on("/appversion", HTTP_GET, [](AsyncWebServerRequest* request) {
              // LOGD(TAG, "Serve appversion");
              auto* response = request->beginResponse(200, "text/plain", APP_VERSION);
              request->send(response); })
    .setFilter([](__unused AsyncWebServerRequest* request) { return eventHandler.getNetworkState() != Mycila::ESPConnect::State::PORTAL_STARTED; });

  // serve our home page here, yet only when the ESPConnect portal is not shown
  _webServer->on("/", HTTP_GET, [&](AsyncWebServerRequest* request) {
              // LOGD(TAG, "Serve...");
              auto* response = request->beginResponse(200,
                                                      "text/html",
                                                      thingy_html_start,
                                                      thingy_html_end - thingy_html_start);
              response->addHeader("Content-Encoding", "gzip");
              request->send(response); })
    .setFilter([](__unused AsyncWebServerRequest* request) { return eventHandler.getNetworkState() != Mycila::ESPConnect::State::PORTAL_STARTED; });

  // register event handlers to stepper
  LOGD(TAG, "register event handlers to stepper");
  stepper.listenMotorEvent([&](JsonDocument doc) { _motorEventCallback(doc); });

  // set up a task to cleanup orphan websock-clients
  _disconnectTime = millis();
  Task* _wsCleanupTask = new Task(1000, TASK_FOREVER, [&] { _wsCleanupCallback(); }, _scheduler, false, NULL, NULL, true);
  _wsCleanupTask->enable();

  _sr.signalComplete();
  LOGD(TAG, "...done!");
}

// get StatusRequest object for initializing task
StatusRequest* WebSite::getStatusRequest() {
  return &_sr;
}

// Handle events from motor
// just forward the event to the website client(s)
void WebSite::_motorEventCallback(JsonDocument doc) {
  _ws->cleanupClients(WSL_MAX_WS_CLIENTS);
  if (_ws->count()) {
    AsyncWebSocketMessageBuffer* buffer = new AsyncWebSocketMessageBuffer(measureJson(doc));
    serializeJson(doc, buffer->get(), buffer->length());
    _ws->textAll(buffer);
  }
}

void WebSite::_wsCleanupCallback() {
  _ws->cleanupClients(WSL_MAX_WS_CLIENTS);
}
