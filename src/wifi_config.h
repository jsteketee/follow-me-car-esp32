#pragma once

#include <stdarg.h>
#include <stdint.h>

// WiFi connection state. While connecting, ssid holds the network currently being
// attempted; once online it holds the joined network (or the soft-AP name).
struct WifiInfo {
    bool     online;         // a network is up (STA joined or AP fallback started)
    bool     sta;            // true = joined an existing network; false = own soft-AP
    char     ssid[33];       // network name (32-char SSID max + NUL)
    char     ip[16];         // dotted-quad address serving the dashboard/telnet
    uint32_t onlineSinceMs;  // millis() when the network came up (0 until then)
};

void wifi_init();
void wifi_update();         // call each loop — drives connection state machine, accepts new telnet clients
const WifiInfo& wifi_get(); // latest connection state
void wifi_printf(const char *fmt, ...);  // print to USB Serial + telnet client
bool wifi_available();      // bytes waiting from telnet client
char wifi_read();           // read one byte from telnet client
