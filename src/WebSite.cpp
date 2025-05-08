// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <Preferences.h>
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
  // _spooldataCallback = nullptr;
  _sr.setWaiting();

  // end the cleanup task
  if (_wsCleanupTask != nullptr) {
    _wsCleanupTask->disable();
    _wsCleanupTask = nullptr;
  }

  // delete websock handler
  if (_ws != nullptr) {
    _webServer->removeHandler(_ws);
    delete _ws;
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

  LOGD(TAG, "Get persistent options from preferences...");
  // Preferences preferences;
  // preferences.begin("tdrive", true);
  // _beepOnRW = preferences.getBool("beep", false);
  // _cloneSerial = preferences.getBool("clone", false);
  // preferences.end();

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
      client->setCloseClientOnQueueFull(false);

      // send ID, config, spooldata, arming,...
      JsonDocument jsonMsg;
      jsonMsg["type"] = "initial_config";
      jsonMsg["id"] = client->id();
//       jsonMsg["host"] = espNetwork.getESPConnect()->getIPAddress().toString().c_str();
//       jsonMsg["cloneSerial"] = _cloneSerial;
// #ifdef USE_BEEPER
//       jsonMsg["beepAvailable"] = true;
//       jsonMsg["beepOnRW"] = _beepOnRW;
// #else
//       jsonMsg["beepAvailable"] = false;
//       jsonMsg["beepOnRW"] = false;
// #endif
//       jsonMsg["writeTags"] = rfid.getWriteEnabled();
//       jsonMsg["writeEmptyTags"] = !rfid.getOverwriteEnabled();
//       jsonMsg["PN532"] = rfid.getStatus();

//       // append spooldata - from RFID - only when length is available
//       JsonDocument jsonSpool = static_cast<JsonDocument>(rfid.getSpooldata());
//       if (jsonSpool["length"].as<const uint32_t>() > 0) {
//         jsonMsg["spooldata"] = jsonSpool;
      // }

      // send welcome message
      AsyncWebSocketMessageBuffer* buffer = new AsyncWebSocketMessageBuffer(measureJson(jsonMsg));
      serializeJson(jsonMsg, buffer->get(), buffer->length());
      client->text(buffer);
      LOGD(TAG, "Client %d connected", client->id());
    } else if (type == WS_EVT_DISCONNECT) {
      LOGD(TAG, "Client %d disconnected", client->id());
    } else if (type == WS_EVT_ERROR) {
      LOGD(TAG, "Client %d error", client->id());
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
        }
      }
    } });

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

  // // register event handlers to reader
  // LOGD(TAG, "register event handlers to reader");
  // rfid.listenTagRead([&](CFSTag tag)
  //                    { _tagReadCallback(tag); });
  // rfid.listenTagWrite([&](bool success)
  //                     { _tagWriteCallback(success); });

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

// // Handle spooldata from reader received event
// void WebSite::_tagReadCallback(CFSTag tag)
// {
//   if (tag.isEmpty())
//   {
//     JsonDocument jsonMsg;
//     jsonMsg["type"] = "read_tag";
//     jsonMsg["uid"] = static_cast<std::string>(tag.getUid()).c_str();
//     AsyncWebSocketMessageBuffer *buffer = new AsyncWebSocketMessageBuffer(measureJson(jsonMsg));
//     serializeJson(jsonMsg, buffer->get(), buffer->length());
//     if (_ws->count())
//     {
//       _ws->textAll(buffer);
//     }
//   }
//   else
//   {
//     SpoolData spooldata = tag.getSpooldata();
//     JsonDocument jsonMsg;
//     jsonMsg["type"] = "read_spool";
//     jsonMsg["uid"] = static_cast<std::string>(tag.getUid()).c_str();
//     jsonMsg["spooldata"] = static_cast<JsonDocument>(tag.getSpooldata());
//     AsyncWebSocketMessageBuffer *buffer = new AsyncWebSocketMessageBuffer(measureJson(jsonMsg));
//     serializeJson(jsonMsg, buffer->get(), buffer->length());
//     if (_ws->count())
//     {
//       _ws->textAll(buffer);
//     }
//   }
// }

// // Handle spooldata written by reader event
// void WebSite::_tagWriteCallback(bool success)
// {
//   LOGD(TAG, "Spooldata written %s", success ? "sucessfully" : "unsucessfully");
//   JsonDocument jsonMsg;
//   jsonMsg["type"] = "write_spool";
//   jsonMsg["result"] = success;
//   AsyncWebSocketMessageBuffer *buffer = new AsyncWebSocketMessageBuffer(measureJson(jsonMsg));
//   serializeJson(jsonMsg, buffer->get(), buffer->length());
//   if (_ws->count())
//   {
//     _ws->textAll(buffer);
//   }
// }

void WebSite::_wsCleanupCallback() {
  _ws->cleanupClients(WSL_MAX_WS_CLIENTS);
}
