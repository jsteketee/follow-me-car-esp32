// Soft-AP WiFi, ArduinoOTA, and a single-client telnet server for remote serial/control access.
#include "wifi_config.h"
#include "config.h"
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include "esp_log.h"

static const char *TAG = "wifi";
static WiFiServer telnetServer(TELNET_PORT);
static WiFiClient telnetClient;

void wifi_init() {
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    MDNS.begin(WIFI_HOSTNAME);

    ArduinoOTA.setHostname(WIFI_HOSTNAME);
    ArduinoOTA.onStart([]()  { ESP_LOGI("ota", "OTA starting..."); });
    ArduinoOTA.onEnd([]()    { ESP_LOGI("ota", "OTA done"); });
    ArduinoOTA.onError([](ota_error_t e) { ESP_LOGE("ota", "OTA error %u", e); });
    ArduinoOTA.begin();

    telnetServer.begin();
    Serial.printf("[%s] ✅ AP ready — SSID: %s  IP: %s  -> nc %s.local %d\n",
                  TAG, WIFI_AP_SSID, WiFi.softAPIP().toString().c_str(),
                  WIFI_HOSTNAME, TELNET_PORT);
}

void wifi_update() {
    ArduinoOTA.handle();

    if (telnetServer.hasClient()) {
        WiFiClient incoming = telnetServer.accept();
        if (!telnetClient || !telnetClient.connected()) {
            telnetClient = incoming;
            telnetClient.println("=== followme-car connected ===");
        } else {
            incoming.println("busy");
            incoming.stop();
        }
    }
}

void wifi_printf(const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial.print(buf);
    if (telnetClient && telnetClient.connected()) {
        telnetClient.print(buf);
    }
}

bool wifi_available() {
    return telnetClient && telnetClient.connected() && telnetClient.available() > 0;
}

char wifi_read() {
    return (char)telnetClient.read();
}
