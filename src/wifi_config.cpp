// STA-first WiFi (apartment → phone hotspot → soft-AP fallback), ArduinoOTA, and a
// single-client telnet server for remote serial/control access.
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

// STA networks to try at boot, in priority order; soft-AP is the final fallback.
struct StaNetwork { const char *ssid; const char *pass; };
static const StaNetwork _staNetworks[] = {
    { WIFI_SSID,         WIFI_PASS },          // apartment
    { WIFI_HOTSPOT_SSID, WIFI_HOTSPOT_PASS },  // phone hotspot
};
static const size_t _staNetworkCount = sizeof(_staNetworks) / sizeof(_staNetworks[0]);

static size_t   _staIndex       = 0;     // which STA network is currently being attempted
static uint32_t _attemptStartMs = 0;     // when the current attempt began
static bool     _online         = false; // true once services are up (STA joined or AP fallback)
static WifiInfo _wifiInfo       = { false, false, "", "", 0 };

// Returns the latest WiFi connection state.
const WifiInfo& wifi_get() { return _wifiInfo; }

// Kicks off the STA connection attempt at _staIndex (non-blocking; wifi_update polls the result).
static void start_sta_attempt() {
    Serial.printf("[%s] trying \"%s\"...\n", TAG, _staNetworks[_staIndex].ssid);
    snprintf(_wifiInfo.ssid, sizeof(_wifiInfo.ssid), "%s", _staNetworks[_staIndex].ssid);
    WiFi.begin(_staNetworks[_staIndex].ssid, _staNetworks[_staIndex].pass);
    _attemptStartMs = millis();
}

// Starts mDNS, OTA, and telnet once a network is up, and prints how to reach the car.
static void start_services(bool sta) {
    _wifiInfo.online = true;
    _wifiInfo.sta    = sta;
    snprintf(_wifiInfo.ssid, sizeof(_wifiInfo.ssid), "%s",
             sta ? WiFi.SSID().c_str() : WIFI_AP_SSID);
    snprintf(_wifiInfo.ip, sizeof(_wifiInfo.ip), "%s",
             sta ? WiFi.localIP().toString().c_str() : WiFi.softAPIP().toString().c_str());
    _wifiInfo.onlineSinceMs = millis();

    MDNS.begin(WIFI_HOSTNAME);

    ArduinoOTA.setHostname(WIFI_HOSTNAME);
    ArduinoOTA.onStart([]()  { ESP_LOGI("ota", "OTA starting..."); });
    ArduinoOTA.onEnd([]()    { ESP_LOGI("ota", "OTA done"); });
    ArduinoOTA.onError([](ota_error_t e) { ESP_LOGE("ota", "OTA error %u", e); });
    ArduinoOTA.begin();

    telnetServer.begin();
    if (sta) {
        Serial.printf("[%s] ✅ joined \"%s\"  IP: %s  -> dashboard http://%s/  telnet nc %s %d\n",
                      TAG, WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(),
                      WiFi.localIP().toString().c_str(),
                      WiFi.localIP().toString().c_str(), TELNET_PORT);
    } else {
        Serial.printf("[%s] ✅ AP ready — SSID: %s  IP: %s  -> nc %s.local %d\n",
                      TAG, WIFI_AP_SSID, WiFi.softAPIP().toString().c_str(),
                      WIFI_HOSTNAME, TELNET_PORT);
    }
}

// Begins the first STA attempt and returns immediately; wifi_update() drives the rest.
void wifi_init() {
    WiFi.mode(WIFI_STA);
    // Disable modem power save (on by default): DTIM sleep batches inbound packets
    // into 100-300ms bursts, which starves the 300ms DIRECT cmd-timeout failsafe
    // fed by the dashboard slider heartbeat. Costs ~80mA — nothing next to the drive motor.
    WiFi.setSleep(false);
    WiFi.setHostname(WIFI_HOSTNAME);
    start_sta_attempt();
}

// Advances the boot connection state machine, then services OTA and telnet once online.
void wifi_update() {
    // Boot connection state machine: poll the current STA attempt; on timeout advance to
    // the next network, then to the soft-AP fallback. Costs one status check per loop.
    if (!_online) {
        if (WiFi.status() == WL_CONNECTED) {
            WiFi.setAutoReconnect(true);  // rejoin this network on a drop (no runtime AP fallback)
            start_services(true);
            _online = true;
        } else if (millis() - _attemptStartMs >= WIFI_STA_TIMEOUT_MS) {
            WiFi.disconnect();  // clear the failed attempt, keep the radio on
            if (++_staIndex < _staNetworkCount) {
                start_sta_attempt();
            } else {
                WiFi.mode(WIFI_AP);
                WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
                start_services(false);
                _online = true;
            }
        }
        return;  // services not started yet — no OTA/telnet to handle
    }

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
