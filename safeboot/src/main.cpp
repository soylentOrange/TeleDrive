// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2025 Mathieu Carbou, 2025 Robert Wendlandt
 */

#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <HardwareSerial.h>
#include <MycilaESPConnect.h>
#include <StreamString.h>
#include <WebServer.h>

#include <esp_ota_ops.h>
#include <esp_partition.h>

#ifndef MYCILA_SAFEBOOT_NO_MDNS
  #include <ESPmDNS.h>
#endif

#ifdef MYCILA_SAFEBOOT_LOGGING
  #define LOG(format, ...) Serial.printf(format, ##__VA_ARGS__)
#else
  #define LOG(format, ...)
#endif

extern const char* __COMPILED_APP_VERSION__;
extern const uint8_t update_html_start[] asm("_binary__pio_embed_website_html_gz_start");
extern const uint8_t update_html_end[] asm("_binary__pio_embed_website_html_gz_end");
extern const uint8_t logo_start[] asm("_binary__pio_embed_logo_safeboot_svg_gz_start");
extern const uint8_t logo_end[] asm("_binary__pio_embed_logo_safeboot_svg_gz_end");
static const char* successResponse = "Update Success! Rebooting...";
static const char* cancelResponse = "Rebooting...";

static WebServer webServer(80);
static Mycila::ESPConnect espConnect;
static Mycila::ESPConnect::Config espConnectConfig;
static StreamString updaterError;

#ifdef MYCILA_SAFEBOOT_USE_LED
  #define LEDC_DUTY_RES          (8)
  #define LED_BRIGHT_OFF         (0)
  #define LED_BRIGHT_DIM         (50)
  #define LEDC_FREQ              (4000)
  #define LED_MILLI_COMPARE_FAST (100)
  #define LED_MILLI_COMPARE_SLOW (400)
  #define LED_MILLI_COMPARE_OFF  (-100)
uint32_t lastMillis;
uint32_t led_milliCompare = LED_MILLI_COMPARE_SLOW;
  #ifdef RGB_BUILTIN
bool ledState = false;
  #endif
#endif

static void scanWiFi() {
  WiFi.scanDelete();
#ifndef ESP8266
  WiFi.scanNetworks(true, false, false, 500, 0, nullptr, nullptr);
#else
  WiFi.scanNetworks(true);
#endif
}

static void start_web_server() {
  webServer.onNotFound([]() {
    webServer.sendHeader("Location", "/");
    webServer.send(302, "text/plain", "");
  });

  webServer.on("/chipspecs", HTTP_GET, [&]() {
    String chipSpecs = ESP.getChipModel();
    chipSpecs += " (" + String(ESP.getFlashChipSize() >> 20) + " MB)";
    webServer.send(200, "text/plain", chipSpecs.c_str());
  });

  webServer.on("/sbversion", HTTP_GET, [&]() {
    webServer.send(200, "text/plain", APP_VERSION);
  });

  // serve the logo
  webServer.on("/safeboot_logo", HTTP_GET, [&]() {
    webServer.sendHeader("Content-Encoding", "gzip");
    webServer.send_P(200, "image/svg+xml", reinterpret_cast<const char*>(logo_start), logo_end - logo_start);
  });

  webServer.on(
    "/cancel",
    HTTP_POST,
    [&]() {
      webServer.send(200, "text/plain", cancelResponse);
      webServer.client().stop();
      delay(1000);
#ifndef RGB_BUILTIN
      ledcWrite(LED_BUILTIN, LED_BRIGHT_OFF);
#else
      rgbLedWrite(LED_BUILTIN, 0, 0, 0);
#endif
      ESP.restart();
    },
    [&]() {
    });

  webServer.on("/", HTTP_GET, [&]() {
    webServer.sendHeader("Content-Encoding", "gzip");
    webServer.send_P(200, "text/html", reinterpret_cast<const char*>(update_html_start), update_html_end - update_html_start);
  });

  webServer.on("/scan", HTTP_GET, [&]() {
    int n = WiFi.scanComplete();

    if (n == WIFI_SCAN_RUNNING) {
      LOG("WIFI_SCAN_RUNNING\n");
      // scan still running ? wait...
      webServer.send(202);

    } else if (n == WIFI_SCAN_FAILED) {
      LOG("WIFI_SCAN_FAILED\n");
      // scan error or finished with no result ?
      // re-scan
      scanWiFi();
      webServer.send(202);

    } else {
      // scan results ?
      JsonDocument json;
      JsonArray array = json.to<JsonArray>();

      // we have some results
      for (int i = 0; i < n; ++i) {
        JsonObject entry = array.add<JsonObject>();
        entry["bssid"] = WiFi.BSSIDstr(i);
        entry["name"] = WiFi.SSID(i);
        entry["rssi"] = WiFi.RSSI(i);
        entry["open"] = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
        entry["current"] = (WiFi.SSID(i) == espConnectConfig.wifiSSID) && !espConnectConfig.apMode;
      }

      // Add a possible access point (when not connected yet)
      if (espConnectConfig.apMode) {
        JsonObject entry = array.add<JsonObject>();
        entry["bssid"] = "AP";
        entry["name"] = espConnectConfig.hostname;
        entry["rssi"] = 0;
        entry["open"] = true;
        entry["current"] = true;
      }

      WiFi.scanDelete();
      json.shrinkToFit();
      size_t jsonSize = measureJson(json);
      void* buffer = malloc(jsonSize);
      serializeJson(json, buffer, jsonSize);
      webServer.send_P(200, "application/json", reinterpret_cast<const char*>(buffer), measureJson(json));
      free(buffer);

      // start next scan for wifi networks
      scanWiFi();
    }
  });

  webServer.on(
    "/",
    HTTP_POST,
    [&]() {
      if (Update.hasError()) {
        webServer.send(500, "text/plain", "Update error: " + updaterError);
      } else {
        webServer.client().setNoDelay(true);
        webServer.send(200, "text/plain", successResponse);
        webServer.client().stop();
        delay(1000);
        ESP.restart();
      } },
    [&]() {
      // handler for the file upload, gets the sketch bytes, and writes
      // them through the Update object
      HTTPUpload& upload = webServer.upload();

      if (upload.status == UPLOAD_FILE_START) {
        updaterError.clear();
        int otaMode = webServer.hasArg("mode") && webServer.arg("mode") == "1" ? U_SPIFFS : U_FLASH;
        LOG("Mode: %d\n", otaMode);
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, otaMode)) { // start with max available size
          Update.printError(updaterError);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE && !updaterError.length()) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(updaterError);
        }
      } else if (upload.status == UPLOAD_FILE_END && !updaterError.length()) {
        if (!Update.end(true)) { // true to set the size to the current progress
          Update.printError(updaterError);
        }
      } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.end();
      }
    });

  // receive credentials for connecting to a network
  webServer.on("/connect", HTTP_POST, [&]() {
    if (webServer.hasArg("plain") == false) {
      webServer.send(400);
    }
    String body = webServer.arg("plain");
    JsonDocument jsonRXMsg;
    DeserializationError error = deserializeJson(jsonRXMsg, body);

    if (error == DeserializationError::Ok) {
      webServer.client().setNoDelay(true);
      webServer.send(200, "text/plain", "OK");
      webServer.client().stop();
      espConnectConfig.apMode = false;
      espConnectConfig.wifiBSSID = jsonRXMsg["bssid"].as<const char*>();
      espConnectConfig.wifiSSID = jsonRXMsg["ssid"].as<const char*>();
      espConnectConfig.wifiPassword = jsonRXMsg["pwd"].as<const char*>();
      espConnect.saveConfiguration(espConnectConfig);
      delay(1000);
#ifndef RGB_BUILTIN
      ledcWrite(LED_BUILTIN, LED_BRIGHT_OFF);
#else
      rgbLedWrite(LED_BUILTIN, 0, 0, 0);
#endif
      ESP.restart();
    } else {
      webServer.send(400);
    }
  });

  webServer.begin();

#ifndef MYCILA_SAFEBOOT_NO_MDNS
  MDNS.addService("http", "tcp", 80);
#endif

  LOG("Web Server started\n");
}

static void set_next_partition_to_boot() {
  const esp_partition_t* partition = esp_partition_find_first(esp_partition_type_t::ESP_PARTITION_TYPE_APP, esp_partition_subtype_t::ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr);
  if (partition) {
    esp_ota_set_boot_partition(partition);
  }
}

static void start_network_manager() {
  // load ESPConnect configuration
  espConnect.loadConfiguration(espConnectConfig);
  espConnect.setBlocking(true);
  espConnect.setAutoRestart(false);

  // reuse a potentially set hostname from main app, or set a default one
  if (!espConnectConfig.hostname.length()) {
    espConnectConfig.hostname = DEFAULT_HOSTNAME;
  }

  // If the passed config is to be in AP mode, or has a SSID that's fine.
  // If the passed config is empty, we need to check if the board supports ETH.
  // - For boards relying only on Wifi, if a SSID is not set and AP is not set (config empty), then we need to go to AP mode.
  // - For boards supporting Ethernet, we do not know if an Ethernet cable is plugged, so we cannot directly start AP mode, because the config might be no AP mode and no SSID.
  //   So we will start, wait for connect timeout (20 seconds), to get DHCP address from ETH, and if failed, we start in AP mode
  if (!espConnectConfig.apMode && !espConnectConfig.wifiSSID.length()) {
#ifdef ESPCONNECT_ETH_SUPPORT
    espConnect.setCaptivePortalTimeout(20);
#else
    espConnectConfig.apMode = true;
    // Show quickly blinking LED
    // apMode == true --> happens automatically
#endif
  }

  espConnect.listen([](Mycila::ESPConnect::State previous, Mycila::ESPConnect::State state) {
    if (state == Mycila::ESPConnect::State::NETWORK_TIMEOUT) {
      LOG("Connect timeout! Starting AP mode...\n");
      // if ETH DHCP times out, we start AP mode
      espConnectConfig.apMode = true;
      espConnect.setConfig(espConnectConfig);
      // Show quickly flashing white LED
      // apMode == true --> happens automatically
    } else if (state == Mycila::ESPConnect::State::NETWORK_CONNECTED) {
      LOG("Connected to WiFi...\n");
      // Show flashing white LED
      led_milliCompare = LED_MILLI_COMPARE_SLOW;
    }
  });

  // show config
  LOG("Hostname: %s\n", espConnectConfig.hostname.c_str());
  if (espConnectConfig.apMode) {
    LOG("AP: %s\n", espConnectConfig.hostname.c_str());
  } else if (espConnectConfig.wifiSSID.length()) {
    LOG("SSID: %s\n", espConnectConfig.wifiSSID.c_str());
    LOG("BSSID: %s\n", espConnectConfig.wifiBSSID.c_str());
  }

  // Show solid white LED when waiting for connection
  if (!espConnectConfig.apMode) {
    led_milliCompare = LED_MILLI_COMPARE_OFF;
#ifndef RGB_BUILTIN
    ledcWrite(LED_BUILTIN, LED_BRIGHT_DIM);
#else
    ledState = 1;
    rgbLedWrite(LED_BUILTIN, COLOR_CORR_R >> 2, COLOR_CORR_G >> 2, COLOR_CORR_B >> 2);
#endif
  }

  // connect...
  espConnect.begin(espConnectConfig.hostname.c_str(), "", espConnectConfig);
  LOG("IP: %s\n", espConnect.getIPAddress().toString().c_str());
}

static void start_mdns() {
#ifndef MYCILA_SAFEBOOT_NO_MDNS
  MDNS.begin(espConnectConfig.hostname.c_str());
  LOG("mDNS started\n");
#endif
}

static void start_arduino_ota() {
  ArduinoOTA.setHostname(espConnectConfig.hostname.c_str());
  ArduinoOTA.begin();
  LOG("OTA Server started on port 3232\n");
}

void setup() {
#ifdef MYCILA_SAFEBOOT_LOGGING
  Serial.begin(115200);
  #if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
  delay(100);
  #else
  while (!Serial)
    yield();
  #endif
#endif

  LOG("Version: %s\n", APP_VERSION);
  set_next_partition_to_boot();
  start_network_manager();

  // scan for wifi networks
  scanWiFi();

  start_mdns();
  start_web_server();
  start_arduino_ota();

  // setup LED
#ifdef MYCILA_SAFEBOOT_USE_LED
  // use LEDC PWM timer for plain LEDs
  if (!RGB_BUILTIN) {
    ledcAttach(LED_BUILTIN, LEDC_FREQ, LEDC_DUTY_RES);
    ledcWrite(LED_BUILTIN, 0);
  }
  // remember timestamp of start
  lastMillis = millis();
#endif
}

void loop() {
  webServer.handleClient();
  ArduinoOTA.handle();

  // toggle the LED
#ifdef MYCILA_SAFEBOOT_USE_LED
  uint32_t currentMillis = millis();
  if (currentMillis - lastMillis > (espConnectConfig.apMode ? LED_MILLI_COMPARE_FAST : led_milliCompare)) {
    lastMillis = currentMillis;

  #ifndef RGB_BUILTIN
    if (ledcRead(LED_BUILTIN) == LED_BRIGHT_DIM) {
      ledcWrite(LED_BUILTIN, LED_BRIGHT_OFF);
    } else {
      ledcWrite(LED_BUILTIN, LED_BRIGHT_DIM);
    }
  #else
    if (!ledState) {
      ledState = 1;
      rgbLedWrite(LED_BUILTIN, COLOR_CORR_R >> 2, COLOR_CORR_G >> 2, COLOR_CORR_B >> 2);
    } else {
      ledState = 0;
      rgbLedWrite(LED_BUILTIN, 0, 0, 0);
    }
  #endif
  }
#endif
}
