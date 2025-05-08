// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2023-2025 Mathieu Carbou
 */

#pragma once

#if defined(ESP8266)
  #include "ESP8266WiFi.h"
#elif defined(ESP32)
  #include "WiFi.h"
#endif

#include <ESPAsyncWebServer.h>
#include <TaskSchedulerDeclarations.h>
#include <functional>
#include <string>

#define WSL_VERSION          "8.1.1"
#define WSL_VERSION_MAJOR    8
#define WSL_VERSION_MINOR    1
#define WSL_VERSION_REVISION 1

#ifndef WSL_MAX_WS_CLIENTS
  #define WSL_MAX_WS_CLIENTS DEFAULT_MAX_WS_CLIENTS
#endif

// High performance mode:
// - Low memory footprint (no stack allocation, no global buffer by default)
// - Low latency (messages sent immediately to the WebSocket queue)
// - High throughput (up to 20 messages per second, no locking mechanism)
// Also recommended to tweak AsyncTCP and ESPAsyncWebServer settings, for example:
//  -D CONFIG_ASYNC_TCP_QUEUE_SIZE=64  // AsyncTCP queue size
//  -D CONFIG_ASYNC_TCP_RUNNING_CORE=1  // core for the async_task
//  -D WS_MAX_QUEUED_MESSAGES=128       // WS message queue size

class WebSerial : public Print {
  public:
    void begin(AsyncWebServer* server, const char* url = "/webserial", Scheduler* scheduler = nullptr);
    void end();
    size_t write(uint8_t) override;
    size_t write(const uint8_t* buffer, size_t size) override;

    // A buffer (shared across cores) can be initialised with an initial capacity to be able to use any Print functions event those that are not buffered and would
    // create a performance impact for WS calls. The goal of this buffer is to be used with lines ending with '\n', like log messages.
    // The buffer size will eventually grow until a '\n' is found, then the message will be sent to the WS clients and a new buffer will be created.
    // Set initialCapacity to 0 to disable buffering.
    // Must be called before begin(): calling it after will erase the buffer and its content will be lost.
    // The buffer is not enabled by default.
    void setBuffer(size_t initialCapacity);

    // Expose the internal WebSocket makeBuffer to even improve memory consumption on client-side
    // 1. make a AsyncWebSocketMessageBuffer
    // 2. put the data inside
    // 3. send the buffer
    // This method avoids a buffer copy when creating the WebSocket message
    AsyncWebSocketMessageBuffer* makeBuffer(size_t size = 0) {
      if (!_ws)
        return nullptr;
      return _ws->makeBuffer(size);
    }

    void send(AsyncWebSocketMessageBuffer* buffer) {
      if (!_ws || !buffer)
        return;
      if (_ws->count())
        _ws->textAll(buffer);
    }

  private:
    void _wsCleanupCallback();
    Task* _wsCleanupTask = nullptr;
    Scheduler* _scheduler = nullptr;
    // Server
    AsyncWebServer* _server;
    AsyncWebSocket* _ws;
    size_t _initialBufferCapacity = 0;
    std::string _buffer;
    void _send(const uint8_t* buffer, size_t size);
};
