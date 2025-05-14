// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */
#pragma once

#include <ESPAsyncWebServer.h>
#include <TaskSchedulerDeclarations.h>

#include <string>

class WebSite {
  public:
    explicit WebSite(AsyncWebServer& webServer) : _webServer(&webServer) { _sr.setWaiting(); }
    void begin(Scheduler* scheduler);
    void end();
    typedef std::function<void(JsonDocument doc)> WebEventCallback;
    void listenWebEvent(WebEventCallback callback) { _webEventCallback = callback; }
    StatusRequest* getStatusRequest();

  private:
    void _webSiteCallback();
    void _wsCleanupCallback();
    Task* _wsCleanupTask = nullptr;
    Scheduler* _scheduler = nullptr;
    StatusRequest _sr;
    AsyncWebServer* _webServer;
    AsyncWebSocket* _ws = nullptr;
    uint32_t _disconnectTime;
    // to be called by website for motor specific events
    WebEventCallback _webEventCallback = nullptr;
    // to be called by stepper for motor specific events
    void _motorEventCallback(JsonDocument doc);
};
